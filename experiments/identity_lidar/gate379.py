"""Would a lidar range have kept the armchair pictures out of object:379?

For every labelled picture filed to 379 in session 1, the lidar's range across
it from the session's own recording, and the resolver's range gate
(locate.stands_at_range) asked against 379 placed where the note says it was.
"""
import json
import math
import re
import sqlite3
import sys

sys.path.insert(0, "D:/UGV Rover")
sys.path.insert(0, "D:/UGV Rover worktrees/what-where/experiments/what_where")
from world_state import locate  # noqa: E402
import lidar  # noqa: E402
import recorded  # noqa: E402

DB = "D:/UGV Rover/tmp/session1-1014/session1-world.db"
recs = [recorded.Recording("D:/UGV Rover/tmp/session1-1014/session1-1014.npz")]
labels = json.load(open("D:/UGV Rover/experiments/m5_changes/labels-session1-1014.json"))
things = labels.get("things") or labels.get("subjects")
thing_of = {i: t for t, v in things.items() for i in v}
con = sqlite3.connect(f"file:{DB}?mode=ro", uri=True)
con.row_factory = sqlite3.Row
ent = con.execute("select placement_json from entities where id='object:379'").fetchone()
final = json.loads(ent["placement_json"])
print("379 now:", {k: final.get(k) for k in ("x_m", "y_m", "uncertainty_m", "extent_m",
                                               "error_major_m", "error_minor_m")})
taken = dict(con.execute("select id, taken_at from frames"))
for r in con.execute("select * from observations where entity_id='object:379' order by id"):
    if r["id"] not in thing_of:
        continue
    pose = json.loads(r["observer_pose_json"])
    bbox = json.loads(r["bbox_json"])
    t = taken.get(r["frame_id"]) or r["observed_at"]
    p = lidar.place(recs, t, float(r["bearing_deg"]), float(r["span_deg"] or 0.0))
    lr = None if p is None else math.hypot(p[0] - pose["x_m"], p[1] - pose["y_m"])
    away = float(re.search(r"object:379 ([\d.]+) m away", r["note"]).group(1))
    b = math.radians(float(r["bearing_deg"]))
    # 379 where the note puts it: `away` along this region's own bearing.
    point = dict(final, x_m=pose["x_m"] + away * math.cos(b), y_m=pose["y_m"] + away * math.sin(b))
    ray = {"x_m": pose["x_m"], "y_m": pose["y_m"], "span_deg": r["span_deg"],
           "range_m": lr, "range_sigma_m": 0.4}
    gate = None if lr is None else locate.stands_at_range(point, ray)
    tol = None if lr is None else locate.range_tolerance_m(point, ray)
    print(r["id"], thing_of[r["id"]][:36].ljust(36), "depth", r["range_m"],
          "lidar", None if lr is None else round(lr, 2), "379 at", away,
          "tol", None if tol is None else round(tol, 2),
          "floor_sized", lidar_ok if (lidar_ok := (bbox[3] >= 0.57 and (bbox[2]-bbox[0])*(bbox[3]-bbox[1]) >= 0.012 and bbox[3]-bbox[1] >= 0.1)) else False,
          "-> kept out" if gate is False else ("-> still in" if gate else ""))
