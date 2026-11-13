"""Counts that say whether the monocular ranges folded duplicates, for two runs.

    python mono_compare.py LOOKS.npz NAME=RUN.pkl ...

Per run: records in all, standing records, regions filed (all, and of those
that had neither a depth nor a wall place before), and for the 11 objects
that never moved, the records within 0.8 m (and 0.5 m of height) of each
object's main record, or holding a labelled picture of it.
"""
import json
import math
import os
import pickle
import sys
from collections import Counter

import numpy as np

VM = os.environ["VISIT_MODEL"]
sys.path.insert(0, VM)
import data  # noqa: E402

NEAR_M, HEIGHT_M = 0.8, 0.5
a = data.load(sys.argv[1])
row = {int(i): k for k, i in enumerate(a["id"])}
src = np.array([str(x) for x in a["place_from"]])
posed = np.array([p is not None for p in a["pose_from"]])
oak = np.isin(src, ("outline", "box")) & np.isfinite(a["range"])
on_wall = np.isfinite(a["wx"]) & (np.nan_to_num(a["wr"]) >= 2.0)
was_unplaced = posed & ~oak & ~on_wall
stay = json.load(open(os.path.join(VM, "labels", "stayed-put.json")))["objects"]

for arg in sys.argv[2:]:
    name, path = arg.split("=", 1)
    run = pickle.load(open(path, "rb"))
    f = run["filing"]
    objs = {o["id"]: o for o in run["objects"]}
    filed = np.array([int(i) in f for i in a["id"]])
    near_total, lab_total, per = 0, 0, []
    for obj, by in stay.items():
        ids = [int(i) for v in by.values() if isinstance(v, list) for i in v if int(i) in row]
        recs = Counter(f.get(i) for i in ids)
        recs.pop(None, None)
        main = recs.most_common(1)[0][0]
        m = objs[main]
        near = set(r for r in recs if r != main)
        lab_total += len(near)
        for r, o in objs.items():
            if r == main or o["state"] != "present":
                continue
            dz = abs(o["z"] - m["z"]) if math.isfinite(o["z"]) and math.isfinite(m["z"]) else 0.0
            if math.hypot(o["x"] - m["x"], o["y"] - m["y"]) <= NEAR_M and dz <= HEIGHT_M:
                near.add(r)
        near_total += len(near)
        per.append("%s %d" % (obj.split()[0], len(near)))
    print("%-10s records %4d (standing %4d)  filed %5.1f%%  filed of formerly unplaced %5.1f%%  "
          "never-moved objects' duplicates %3d (of them holding a labelled picture %2d)" % (
              name, len(objs), sum(o["state"] == "present" for o in objs.values()), 100 * filed.mean(),
              100 * filed[was_unplaced].mean(), near_total, lab_total))
    print("           " + ", ".join(per))
