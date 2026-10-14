"""Where does the lidar's range agree with the depth camera's, by how low a
region reaches in the picture and in elevation?

For every region with a depth range and a pose in sessions 1 and 2 of
2026-10-14, the lidar's range across it at its shutter; binned by the region's
lower edge (elevation, tilt bias out) and by the box rule `floor_sized`.
"""
import json
import math
import sqlite3
import sys

import numpy as np

sys.path.insert(0, "D:/UGV Rover")
sys.path.insert(0, "D:/UGV Rover worktrees/what-where/experiments/what_where")
from world_state import locate  # noqa: E402
import lidar  # noqa: E402
import recorded  # noqa: E402

SETS = [("D:/UGV Rover/tmp/session1-1014/session1-world.db",
         ["D:/UGV Rover/tmp/session1-1014/session1-1014.npz"]),
        ("D:/UGV Rover/tmp/session2-1014/session2-world.db",
         ["D:/UGV Rover/tmp/session2-1014/session2-1014.npz"])]


def floor_sized(b):
    return b[3] >= 0.57 and (b[2] - b[0]) * (b[3] - b[1]) >= 0.012 and (b[3] - b[1]) >= 0.1


rows = []
for db, npz in SETS:
    recs = [recorded.Recording(p) for p in npz]
    lo, hi = min(r.scan_t[0] for r in recs), max(r.scan_t[-1] for r in recs)
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    con.row_factory = sqlite3.Row
    taken = dict(con.execute("select id, taken_at from frames"))
    for r in con.execute("select * from observations where observed_at between ? and ?"
                         " and bearing_deg is not null and observer_pose_json is not null",
                         (lo, hi)):
        pose = json.loads(r["observer_pose_json"])
        b = json.loads(r["bbox_json"])
        t = taken.get(r["frame_id"]) or r["observed_at"]
        p = lidar.place(recs, t, float(r["bearing_deg"]), float(r["span_deg"] or 0.0))
        if p is None:
            continue
        lr = math.hypot(p[0] - pose["x_m"], p[1] - pose["y_m"])
        e = locate.elevation_of(dict(r))
        low = None if e is None else e - float(r["elevation_span_deg"] or 0.0) / 2.0
        rows.append((r["range_m"], lr, low, floor_sized(b), b[3], r["id"], db))

print("regions with a lidar range:", len(rows), " with depth too:",
      sum(1 for x in rows if x[0] is not None))


def report(name, sel):
    d = [(x[0], x[1]) for x in sel if x[0] is not None]
    if not d:
        print(f"{name:42s} n=0  (no depth; {len(sel)} lidar-only)")
        return
    diff = np.array([lr - dr for dr, lr in d])
    print(f"{name:42s} n={len(d):4d}  within 0.3 m {np.mean(np.abs(diff) <= 0.3):.0%}"
          f"  within 1 m {np.mean(np.abs(diff) <= 1.0):.0%}"
          f"  >0.8 short {np.mean(diff < -0.8):.0%}  >0.8 long {np.mean(diff > 0.8):.0%}"
          f"   (+{sum(1 for x in sel if x[0] is None)} unranged)")


report("all", rows)
report("floor_sized", [x for x in rows if x[3]])
report("not floor_sized", [x for x in rows if not x[3]])
for a, b in ((-90, -10), (-10, -5), (-5, -2), (-2, 0), (0, 2), (2, 5), (5, 10), (10, 90)):
    report(f"lower edge elevation {a}..{b} deg",
           [x for x in rows if x[2] is not None and a <= x[2] < b])
for cut in (0.0, 2.0, 4.0):
    report(f"lower edge <= {cut} deg", [x for x in rows if x[2] is not None and x[2] <= cut])
    report(f"lower edge <= {cut} deg, not floor_sized",
           [x for x in rows if x[2] is not None and x[2] <= cut and not x[3]])
for i in (102584, 102601, 102685, 102694, 102701):
    x = [x for x in rows if x[5] == i]
    print(i, "lower edge", None if not x or x[0][2] is None else round(x[0][2], 1))
