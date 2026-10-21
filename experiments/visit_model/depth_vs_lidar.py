"""For every labelled region of a thing standing on the floor that the depth
camera ranged, where the lidar put it at the same moment: how far the depth
camera's range runs past the lidar's return, per object. On the multi-day
benchmark's days, also how far each place is from the spot the thing was set
out at.

    python depth_vs_lidar.py LOOKS.npz RECORDING ...

A thing that reaches the lidar's plane, about 20 cm up, is hit by it, so the
lidar's return along the bearing is the thing's near face; a depth range well
past it says the depth camera read long (the black bean bag's record of
2026-10-17 lay 0.6 to 0.9 m beyond every return).
"""
import json
import math
import os
import sys

import numpy as np

import data
import lidar
import recorded
import truth

#: Things standing on the floor and reaching the lidar's plane.
FLOOR = ("red bean bag", "black bean bag", "stool", "box", "step ladder", "cabinet",
         "plain armchair", "throw armchair", "bed")
#: Where the benchmark's things were set out (experiments/multiday/benchmark.json).
SPOTS = {"P1": (-20.2, -12.55), "P2": (-17.45, -13.4), "P3": (-16.45, -11.76),
         "P4": (-17.9, -16.55), "P5": (-19.5, -16.3)}
DAY_PUT = {"md day 1": {"red bean bag": "P1", "black bean bag": "P4", "stool": "P2"},
           "md day 2": {"red bean bag": "P1", "black bean bag": "P5", "stool": "P3"}}


def main():
    a = data.load(sys.argv[1])
    recs = [recorded.Recording(p) for p in sys.argv[2:]]
    t_of = dict(zip(a["id"].tolist(), a["t"].tolist()))
    row = {int(i): k for k, i in enumerate(a["id"])}
    objs = truth.regions_by_object(t_of)
    src = np.array([str(s) for s in a["place_from"]])
    out = {}
    for obj in FLOOR:
        rows = []
        for rid, sess in objs.get(obj, {}).items():
            k = row.get(rid)
            if k is None or src[k] not in ("outline", "box") or not np.isfinite(a["range"][k]):
                continue
            p = lidar.place(recs, float(a["shutter"][k]), float(a["bearing"][k]), float(a["span"][k]))
            if p is None:
                continue
            lr = math.hypot(p[0] - a["cx"][k], p[1] - a["cy"][k])
            entry = {"id": rid, "session": sess, "src": src[k], "depth_m": float(a["range"][k]), "lidar_m": lr,
                     "over_m": float(a["range"][k]) - lr}
            spot = DAY_PUT.get(sess, {}).get(obj)
            if spot:
                sx, sy = SPOTS[spot]
                entry["depth_off_m"] = math.hypot(a["px"][k] - sx, a["py"][k] - sy)
                entry["lidar_off_m"] = math.hypot(p[0] - sx, p[1] - sy)
            rows.append(entry)
        out[obj] = rows
        if not rows:
            print("%-16s no region both ranged and recorded" % obj)
            continue
        over = np.array([r["over_m"] for r in rows])
        line = "%-16s %4d regions: depth past the lidar by median %+.2f m; past by over 0.3 m %d%%, over 0.6 m %d%%" % (
            obj, len(rows), np.median(over), round(100 * np.mean(over > 0.3)), round(100 * np.mean(over > 0.6)))
        days = [r for r in rows if "depth_off_m" in r]
        if days:
            line += "; benchmark days (%d): off the spot, depth median %.2f m, lidar %.2f m" % (
                len(days), np.median([r["depth_off_m"] for r in days]), np.median([r["lidar_off_m"] for r in days]))
        print(line)
    json.dump(out, open(os.path.join(truth.TMP, "visit-model", "depth-vs-lidar.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
