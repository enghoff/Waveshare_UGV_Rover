#!/usr/bin/env python3
"""The change watch, fed from the live scan: what has changed since each place
was last seen.

    python3 change_node.py [--nav-port 8773]

**A passenger.** It subscribes to `/scan`, reads where the rover was at each
scan off the transform tree, and adds every third scan taken while not turning
fast to `change_watch.ChangeWatch`. It publishes nothing and serves nothing over
ROS: navigation does not know it is there, and a crash here costs nothing but
the report. Its own process, so the second of ray-casting a minute it adds is
not the bridge's or the mapper's.

**Only while the map is believed.** A scan laid on the map at the wrong pose is
the whole flat changed at once, so scans count only while navigation says the
map is settled and the position trusted, asked every few seconds over the
bridge's own `status`. A restore nobody has confirmed, or a refit pending, is a
pause, and says so in the report. So is navigation's drift check finding the
scan fitting better somewhere else. Neither is enough alone: on 2026-10-10 a
restore 170 degrees out stayed settled for a quarter of an hour, so each scan
must also fit what the watch already knows (`change_watch.FIT_MIN`), and one
that does not is refused and counted.

**What it writes.** `~/.ugv/changes/<map_id>.npz`, the tallies, every two
minutes and at exit, so that the last visit survives a restart and a reboot;
and `<map_id>.json`, the changes found, every thirty seconds; and each change
the first time it is found, appended to `<map_id>.log.jsonl`, which is what a
run's changes are counted from afterwards. A different
`map_id` is a different map, whose cells are other places: each map keeps its
own pair, and a cleared map starts with nothing to compare against.
"""
import argparse
import json
import math
import os
import signal
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

import change_watch as cw

STATE_DIR = os.path.expanduser("~/.ugv/changes")
EVERY = 3
MAX_TURN_DPS = 40.0
STATUS_EVERY_S = 5.0
REPORT_EVERY_S = 30.0
SAVE_EVERY_S = 120.0
MARGIN_M = 3.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ChangeNode(Node):
    def __init__(self, args):
        super().__init__("change_node")
        self.args = args
        self.watch = None
        self.map_id = None
        self.map_box = None
        self.believed = False
        self.why_paused = "navigation has not answered yet"
        self.seen = 0
        self.fed = 0
        self.refused = 0
        self.last_yaw = None
        self.previous = []
        self.saved_at = self.reported_at = time.monotonic()
        os.makedirs(STATE_DIR, exist_ok=True)

        sensor = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                            durability=QoSDurabilityPolicy.VOLATILE,
                            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        latched = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, "scan", self.on_scan, sensor)
        self.create_subscription(OccupancyGrid, "map", self.on_map, latched)
        self.create_timer(STATUS_EVERY_S, self.ask_status)
        self.create_timer(REPORT_EVERY_S, self.tick)

    # --- what navigation says -------------------------------------------------
    def ask_status(self):
        try:
            with socket.create_connection(("127.0.0.1", self.args.nav_port), 2) as s:
                s.settimeout(3)
                f = s.makefile("rwb")
                f.write(b'{"op": "status"}\n')
                f.flush()
                status = json.loads(f.readline())
        except (OSError, ValueError) as error:
            self.believed, self.why_paused = False, "navigation did not answer: %s" % error
            return
        map_id = status.get("map_id")
        if map_id != self.map_id:
            self.switch_map(map_id)
        drift = status.get("map_drift") or {}
        if not status.get("map_settled"):
            self.believed, self.why_paused = False, "the map is not settled"
        elif not status.get("position_trusted"):
            self.believed, self.why_paused = False, "the position is not trusted"
        elif drift.get("trusted") and drift.get("agrees") is False:
            # Navigation's own look at the lidar says the rover is somewhere
            # else, and goes on calling the map settled -- which is what it
            # did at 18:06 on 2026-10-10. Paused until a later look agrees.
            self.believed = False
            self.why_paused = "the scan fits the map better elsewhere: %s" % drift.get("why")
        else:
            self.believed, self.why_paused = True, ""

    def switch_map(self, map_id):
        if self.watch is not None and self.map_id:
            self.save()
        self.map_id, self.watch, self.last_yaw = map_id, None, None
        self.previous = []
        if map_id:
            # What the last report said, so that a restart does not log again
            # every change it had already logged.
            try:
                with open(self.path("json")) as f:
                    self.previous = json.load(f).get("changes") or []
            except (OSError, ValueError):
                pass
            watch, meta = cw.ChangeWatch.load(self.path("npz"))
            if watch is not None and meta.get("map_id") == map_id:
                self.watch = watch
                self.get_logger().info("change watch: map %s, %d scans remembered"
                                       % (map_id, watch.scans))
        self.fit_to_map()

    def on_map(self, msg):
        info = msg.info
        x0, y0 = info.origin.position.x, info.origin.position.y
        self.map_box = (x0, y0, x0 + info.width * info.resolution,
                        y0 + info.height * info.resolution)
        self.fit_to_map()

    def fit_to_map(self):
        if self.map_box is None or not self.map_id:
            return
        if self.watch is None:
            self.watch = cw.ChangeWatch.around(*self.map_box, margin_m=MARGIN_M)
        elif not self.watch.covers(*self.map_box):
            self.watch = self.watch.grown_to(*self.map_box, margin_m=MARGIN_M)

    # --- the scans -----------------------------------------------------------
    def on_scan(self, msg):
        self.seen += 1
        if self.seen % EVERY or not self.believed or self.watch is None:
            return
        # At the stamp itself: lidar_node puts a turning scan back together to the
        # moment its sweep began and stamps it with that moment
        # (scan_deskew.py), so that is the pose its points are seen from.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            at = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time.from_msg(msg.header.stamp),
                rclpy.duration.Duration(seconds=0.2))
        except Exception:
            return
        t = at.transform.translation
        yaw = yaw_of(at.transform.rotation)
        if self.last_yaw is not None:
            then, was = self.last_yaw
            gap = stamp - then
            if gap > 0 and abs(math.degrees(math.atan2(
                    math.sin(yaw - was), math.cos(yaw - was)))) / gap > MAX_TURN_DPS:
                self.last_yaw = (stamp, yaw)
                return
        self.last_yaw = (stamp, yaw)
        n = len(msg.ranges)
        angles = [msg.angle_min + msg.angle_increment * i for i in range(n)]
        if self.watch.add_scan(t.x, t.y, yaw, msg.ranges, angles, stamp) is False:
            self.refused += 1
        else:
            self.fed += 1

    # --- what it writes ------------------------------------------------------
    def path(self, ext):
        return os.path.join(STATE_DIR, "%s.%s" % (self.map_id, ext))

    def save(self):
        if self.watch is not None and self.map_id:
            self.watch.save(self.path("npz"), map_id=self.map_id)
        self.saved_at = time.monotonic()

    def tick(self):
        if self.watch is None or not self.map_id:
            return
        started = time.monotonic()
        found = self.watch.changes()
        report = {"map_id": self.map_id, "at": time.time(),
                  "counting": self.believed, "paused": self.why_paused,
                  "scans": self.watch.scans, "refused": self.watch.refused,
                  "changes": found}
        tmp = self.path("json") + ".tmp"
        with open(tmp, "w") as out:
            json.dump(report, out)
        os.replace(tmp, self.path("json"))
        fresh = cw.ChangeWatch.first_seen(found, self.previous)
        if fresh:
            with open(self.path("log.jsonl"), "a") as log:
                for g in fresh:
                    log.write(json.dumps({"logged_at": report["at"],
                                          "map_id": self.map_id, **g}) + "\n")
        self.previous = found
        large = sum(1 for c in found if c["large"])
        self.get_logger().info(
            "change watch: %d scans added since the last report and %d refused "
            "as not fitting, %d in all; %d changes, %d furniture-sized%s (%.0f ms)"
            % (self.fed, self.refused, self.watch.scans, len(found), large,
               "" if self.believed else "; paused: " + self.why_paused,
               1000.0 * (time.monotonic() - started)))
        self.fed = self.refused = 0
        if time.monotonic() - self.saved_at >= SAVE_EVERY_S:
            self.save()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--nav-port", type=int, default=8773)
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = ChangeNode(args)

    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
