"""Version 4 of the world model kept up to date on the rover, beside the store.

    python3 job.py [--state DIR] [--every S] [--once]

Run by run_job.sh inside the navigation environment (it subscribes to the
lidar). Every `--every` seconds it reads the regions the store has added
(read-only), lets the engine file every visit that has closed, and writes:

- `things.json`: the model's records -- place, state, when seen, and the
  store's observation ids of every region filed to each -- for the daemon to
  read in place of the store's entities (`find_thing` first);
- `engine.pkl`: the engine itself, so a restart carries on where it stopped;
- `decisions.jsonl`: what each visit decided, appended;
- `scans/`: the lidar and navigation's pose over each batch of visits it
  filed, in the drive recordings' format (bag_extract.py), so ordinary drives
  leave recordings behind as staged sessions do.

With no `engine.pkl` it first files the whole history, with the drive
recordings in `recordings/` (the desk's, copied over, read in name order as
the desk's run_versions.sh lists them), then carries on with the live lidar.

The store keeps filing as it always has; nothing here writes to it. A region
is read only once it is READ_DELAY_S old, so that what the store settles
about it just after the look is in. Regions of another map session than the
model's are not read (data.MAP_SESSION).
"""
import argparse
import json
import math
import os
import sqlite3
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import engine  # noqa: E402
import data  # noqa: E402
import recorded  # noqa: E402

WORLD = os.path.expanduser(os.environ.get("UGV_WORLD_DIR", "~/.ugv/world"))
DB = os.path.join(WORLD, "world.db")
#: Where world_state/visit_things.py reads the records from.
STATE = os.path.join(WORLD, "visit_model")
READ_DELAY_S = 60.0
#: How much lidar the job holds: a visit filed later than this after it closed
#: is filed without the lidar's absence evidence, as the desk does for looks no
#: recording covers.
BUFFER_S = 1800.0
#: Lidar kept on disk around each batch of visits filed.
SCAN_MARGIN_S = 10.0
SCANS_KEPT_S = 14 * 86400.0


class Buffer:
    """The lidar's scans and the two transforms, kept as bag_extract.py keeps
    them (experiments/what_where/shadow.py's, on branch what-where)."""

    def __init__(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage
        self.lock = threading.Lock()
        self.scans, self.meta, self.odom, self.map = [], [], [], []
        rclpy.init()
        self.node = rclpy.create_node("visit_model_job")
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

    def arrays(self):
        """What the buffer holds, older than BUFFER_S dropped; None until it
        holds scans and both transforms."""
        with self.lock:
            now = time.time()
            k = [i for i, m in enumerate(self.meta) if now - m[1] <= BUFFER_S]
            self.scans = [self.scans[i] for i in k]
            self.meta = [self.meta[i] for i in k]
            self.odom = [r for r in self.odom if now - r[1] <= BUFFER_S]
            self.map = [r for r in self.map if now - r[1] <= BUFFER_S]
            if len(self.scans) < 10 or len(self.odom) < 10 or len(self.map) < 1:
                return None
            width = max(len(s) for s in self.scans)
            ranges = np.full((len(self.scans), width), np.nan, dtype=np.float32)
            for i, s in enumerate(self.scans):
                ranges[i, :len(s)] = s
            return {"ranges": ranges, "scan_meta": np.array(self.meta),
                    "odom_tf": np.array(self.odom), "map_tf": np.array(self.map)}


def _slice(arrays, t0, t1):
    """The part of a recording's arrays between t0 and t1 (by stamp), with the
    last map correction before t0 kept so that its poses still compose."""
    s = (arrays["scan_meta"][:, 0] >= t0) & (arrays["scan_meta"][:, 0] <= t1)
    o = (arrays["odom_tf"][:, 0] >= t0) & (arrays["odom_tf"][:, 0] <= t1)
    m = arrays["map_tf"][:, 0] <= t1
    before = np.flatnonzero(arrays["map_tf"][:, 0] < t0)
    if len(before):
        m &= arrays["map_tf"][:, 0] >= arrays["map_tf"][before[-1], 0]
    return {"ranges": arrays["ranges"][s], "scan_meta": arrays["scan_meta"][s],
            "odom_tf": arrays["odom_tf"][o], "map_tf": arrays["map_tf"][m]}


def map_session(db_path):
    con = sqlite3.connect("file:%s?mode=ro" % db_path, uri=True)
    try:
        row = con.execute("select value from meta where key = 'map_session'").fetchone()
    finally:
        con.close()
    return int(row[0]) if row and str(row[0]).isdigit() else None


def things(e):
    """The records as the daemon reads them."""
    members = {}
    for rid, oid in e.resolved().items():
        members.setdefault(oid, []).append(int(rid))
    out = []
    for o in e.mem.objects:
        ids = sorted(members.get(o["id"], []))
        if not ids:
            continue
        out.append({"id": o["id"], "x_m": round(float(o["x"]), 3), "y_m": round(float(o["y"]), 3),
                    "spread_m": round(math.sqrt(float(o["var"])), 3),
                    "extent_m": round(float(o["extent"]), 3),
                    "height_m": None if not math.isfinite(float(o["z"])) else round(float(o["z"]), 3),
                    "state": o["state"], "seen_at": o["seen"], "since": o["since"],
                    "gone_at": o["gone_at"], "visits": len(o["visits"]),
                    "path": o["path"], "observations": ids})
    return out


def write_json(path, payload):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(payload, f)
    os.replace(tmp, path)


def once(e, state, buf, log):
    now = time.time()
    session = map_session(DB)
    if session != data.MAP_SESSION:
        log("the store is on map session %s, the model on %s: nothing read" % (session, data.MAP_SESSION))
        return 0
    arrays = buf.arrays() if buf is not None else None
    recs = [recorded.Recording(arrays=arrays)] if arrays is not None else []
    a = engine.prepare(DB, recs, after_id=e.last_id)
    if len(a["id"]):
        # Only regions READ_DELAY_S old, and only an unbroken run of ids, so
        # that a region the store adds late is not skipped.
        young = a["t"] > now - READ_DELAY_S
        if young.any():
            first_young = int(a["id"][young].min())
            keep = a["id"] < first_young
            a = {k: v[keep] for k, v in a.items()}
        e.add(a)
    decisions = []
    filed = e.step(now - READ_DELAY_S, recs, decisions)
    if decisions:
        with open(os.path.join(state, "decisions.jsonl"), "a") as f:
            for d in decisions:
                f.write(json.dumps(d) + "\n")
    if filed:
        if arrays is not None and decisions:
            ts = [d["t"] for d in decisions]
            part = _slice(arrays, min(ts) - SCAN_MARGIN_S - 150.0, e.taken_until + SCAN_MARGIN_S)
            if len(part["scan_meta"]):
                os.makedirs(os.path.join(state, "scans"), exist_ok=True)
                np.savez_compressed(os.path.join(state, "scans", "%d.npz" % int(part["scan_meta"][0, 0])),
                                    **part)
        e.save(os.path.join(state, "engine.pkl"))
    records = things(e)
    write_json(os.path.join(state, "things.json"), {
        "model": "visit_model version 4 (cabe003)", "written_at": time.time(),
        "filed_until": e.taken_until, "last_observation": e.last_id,
        "waiting": 0 if e.pending is None else int(len(e.pending["id"])),
        "lidar": arrays is not None, "things": records})
    if filed:
        log("filed %d regions; %d records, %d waiting; lidar %s" % (
            filed, len(records), 0 if e.pending is None else len(e.pending["id"]),
            "yes" if arrays is not None else "no"))
    return filed


def bootstrap(state, log):
    """An engine that has filed the whole history, with the drive recordings
    kept in `recordings/`."""
    import glob
    paths = sorted(glob.glob(os.path.join(state, "recordings", "*.npz")))
    recs = [recorded.Recording(p) for p in paths]
    log("filing the history with %d recordings" % len(recs))
    t0 = time.time()
    e = engine.Engine()
    a = engine.prepare(DB, recs, after_id=0)
    young = a["t"] > time.time() - READ_DELAY_S
    if young.any():
        keep = a["id"] < int(a["id"][young].min())
        a = {k: v[keep] for k, v in a.items()}
    e.add(a)
    decisions = []
    n = e.step(time.time() - READ_DELAY_S, recs, decisions)
    with open(os.path.join(state, "decisions.jsonl"), "a") as f:
        for d in decisions:
            f.write(json.dumps(d) + "\n")
    log("history filed: %d regions in %.0f s, %d records" % (n, time.time() - t0, len(e.mem.objects)))
    return e


def prune(state):
    d = os.path.join(state, "scans")
    if not os.path.isdir(d):
        return
    for name in os.listdir(d):
        p = os.path.join(d, name)
        if time.time() - os.path.getmtime(p) > SCANS_KEPT_S:
            os.remove(p)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", default=STATE)
    ap.add_argument("--every", type=float, default=60.0)
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--no-lidar", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.state, exist_ok=True)

    def log(msg):
        print("%s %s" % (time.strftime("%Y-%m-%d %H:%M:%S"), msg), flush=True)

    path = os.path.join(args.state, "engine.pkl")
    if os.path.exists(path):
        e = engine.Engine.load(path)
    else:
        e = bootstrap(args.state, log)
        e.save(path)
    log("started: %d regions filed so far, last observation %d" % (len(e.filing), e.last_id))
    buf = None if args.no_lidar else Buffer()
    if buf is not None:
        time.sleep(min(args.every, 30.0))  # some lidar before the first batch
    while True:
        try:
            once(e, args.state, buf, log)
            prune(args.state)
        except Exception as exc:  # noqa: BLE001 - a bad batch must not stop the job
            log("batch failed: %r" % (exc,))
            if args.once:
                raise
        if args.once:
            return
        time.sleep(args.every)


if __name__ == "__main__":
    main()
