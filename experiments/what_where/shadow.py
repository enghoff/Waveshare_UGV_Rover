"""Step 3 on the rover, in shadow: what it would report, logged and never acted
on (docs/plans/what-where-streams.md, the order of work's step 5).

    python3 shadow.py [--every 60] [--window 900] [--log ~/.ugv/whatwhere]

Run on the rover inside the ROS environment (ros_nav/env.sh, dds.sh), from a
folder holding this experiment's modules beside the M5 scripts it imports
(change_detect.py, arm_replay.py): nothing in the deploy tree, nothing deployed,
nothing restarted. It reads the live world store read-only, asks the daemon for
the change watch's report, and keeps its own rolling buffer of lidar scans and
transforms so that regions are placed exactly as the offline version places
them. Every EVERY seconds it builds step 3's events (events.build) over the
last WINDOW seconds and appends the new ones to a JSON-lines log, one file a
day. It drives nothing and holds nothing the rover needs.

Memory is every region stored before the earliest change the watch is
reporting began (less a minute), or before the window when it reports none.
An event is new when no event of the same kind lies within 0.3 m of it in the
last hour of the log.
"""
import argparse
import json
import math
import os
import socket
import sqlite3
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import events  # noqa: E402
import load  # noqa: E402
import recorded  # noqa: E402

DB = os.path.expanduser("~/.ugv/world/world.db")
#: The what threshold of step 1, from the controls of 2026-10-11.
T_WHAT = 0.3216
#: How long the scan buffer reaches back.
BUFFER_S = 1800.0
SAME_EVENT_M = 0.3
SAME_EVENT_S = 3600.0


def call(name, arguments=None, timeout=30):
    s = socket.create_connection(("127.0.0.1", 8769), timeout)
    f = s.makefile("rwb")
    f.write(json.dumps({"call": name, "arguments": arguments or {}}).encode() + b"\n")
    f.flush()
    line = f.readline()
    s.close()
    return json.loads(line) if line.strip() else {}


class Buffer:
    """The lidar's scans and the two transforms, as bag_extract.py keeps them."""

    def __init__(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage
        self.lock = threading.Lock()
        self.scans, self.meta, self.odom, self.map = [], [], [], []
        rclpy.init()
        self.node = rclpy.create_node("whatwhere_shadow")
        self.node.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.node.create_subscription(TFMessage, "/tf", self._tf, 100)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    @staticmethod
    def _stamp(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _scan(self, msg):
        with self.lock:
            self.scans.append(np.asarray(msg.ranges, dtype=np.float32))
            self.meta.append((self._stamp(msg.header), time.time(), msg.angle_min,
                              msg.angle_increment, msg.time_increment, msg.scan_time))

    def _tf(self, msg):
        with self.lock:
            for t in msg.transforms:
                row = (self._stamp(t.header), time.time(), t.transform.translation.x,
                       t.transform.translation.y, self._yaw(t.transform.rotation))
                if t.header.frame_id == "odom" and t.child_frame_id == "base_link":
                    self.odom.append(row)
                elif t.header.frame_id == "map" and t.child_frame_id == "odom":
                    self.map.append(row)

    def close(self):
        import rclpy
        self.node.destroy_node()
        rclpy.shutdown()
        self.thread.join(timeout=5)

    def recording(self):
        """What the buffer holds now, as a recorded.Recording, older than
        BUFFER_S dropped; None until it holds scans and both transforms."""
        with self.lock:
            now = time.time()
            keep = lambda rows, at: [r for r in rows if now - r[at] <= BUFFER_S]
            k = [i for i, m in enumerate(self.meta) if now - m[1] <= BUFFER_S]
            self.scans = [self.scans[i] for i in k]
            self.meta = [self.meta[i] for i in k]
            self.odom, self.map = keep(self.odom, 1), keep(self.map, 1)
            if len(self.scans) < 10 or len(self.odom) < 10 or len(self.map) < 1:
                return None
            width = max(len(s) for s in self.scans)
            ranges = np.full((len(self.scans), width), np.nan, dtype=np.float32)
            for i, s in enumerate(self.scans):
                ranges[i, :len(s)] = s
            arrays = {"ranges": ranges, "scan_meta": np.array(self.meta),
                      "odom_tf": np.array(self.odom), "map_tf": np.array(self.map)}
        return recorded.Recording(arrays=arrays)


def changes_now():
    reply = call("nav_changes")
    rows = reply.get("changes") if isinstance(reply, dict) else None
    return [dict(c, logged_at=time.time()) for c in rows or []]


class Store:
    """Every region of the live store's map session, loaded once and then only
    what was added since; a region keeps the pose it was first loaded with."""

    def __init__(self, map_session):
        self.map_session = map_session
        self.regions = None

    def now(self, rec):
        after = 0 if self.regions is None or self.regions.n == 0 else int(self.regions.id.max())
        new = load.load(DB, 1.7e9, time.time() + 1.0, map_session=self.map_session,
                        recordings=[rec], after_id=after)
        self.regions = new if self.regions is None else load.concat(self.regions, new)
        return self.regions


def map_session():
    con = sqlite3.connect(f"file:{DB}?mode=ro", uri=True)
    value = con.execute("select value from meta where key='map_session'").fetchone()
    con.close()
    return int(value[0])


def cycle(buffer, store, window, log_dir, seen):
    rec = buffer.recording()
    if rec is None:
        return 0
    now = time.time()
    changes = changes_now()
    starts = [c["seen_from"] for c in changes if c.get("seen_from")]
    cutoff = (min(starts) - events.ATTACH_S) if starts else now - window
    cutoff = max(cutoff, now - window)
    regions = store.now(rec)
    memory = regions.subset(regions.t < cutoff)
    query = regions.subset(regions.t >= cutoff)
    if query.n == 0:
        return 0
    evs, _ = events.build(changes, query, memory, [rec], T_WHAT, person=events.people(DB, query))
    fresh = []
    for e in evs:
        key = (e["kind"], e["x_m"], e["y_m"])
        if any(k == key[0] and math.hypot(x - key[1], y - key[2]) <= SAME_EVENT_M
               and now - t <= SAME_EVENT_S for k, x, y, t in seen):
            continue
        seen.append((key[0], key[1], key[2], now))
        fresh.append(dict(e, at=now, memory_cutoff=cutoff, version="step 3 version 3"))
    if fresh:
        os.makedirs(log_dir, exist_ok=True)
        path = os.path.join(log_dir, time.strftime("shadow-%Y%m%d.jsonl"))
        with open(path, "a") as f:
            for e in fresh:
                f.write(json.dumps(e, default=float) + "\n")
    return len(fresh)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--every", type=float, default=60.0)
    p.add_argument("--window", type=float, default=900.0)
    p.add_argument("--log", default=os.path.expanduser("~/.ugv/whatwhere"))
    p.add_argument("--once", action="store_true")
    p.add_argument("--warmup", type=float, default=30.0,
                   help="seconds of scans and transforms to gather before the first cycle")
    a = p.parse_args()
    buffer = Buffer()
    store = Store(map_session())
    seen = []
    time.sleep(a.warmup)
    while True:
        began = time.time()
        try:
            n = cycle(buffer, store, a.window, a.log, seen)
            print(time.strftime("%H:%M:%S"), "new events", n, "took %.1f s" % (time.time() - began),
                  flush=True)
        except Exception as exc:  # a cycle that fails is logged and the next one tried
            print(time.strftime("%H:%M:%S"), "cycle failed:", repr(exc), flush=True)
        if a.once:
            break
        time.sleep(max(1.0, a.every - (time.time() - began)))
    buffer.close()


if __name__ == "__main__":
    main()
