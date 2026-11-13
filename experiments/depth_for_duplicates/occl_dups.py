"""The duplicate records of objects that never moved, under a filing.

    python occl_dups.py LOOKS.npz RUN.pkl OUT.json

For each object of labels/stayed-put.json: its main record (most labelled
regions), every other record holding a labelled region of it, and every other
standing record placed within NEAR_M of the main record at a height within
HEIGHT_M -- a record of the same thing made of pictures the labels left out.
"""
import json
import math
import os
import pickle
import sys
from collections import Counter

import numpy as np

HERE = os.environ["VISIT_MODEL"]
sys.path.insert(0, HERE)
import data  # noqa: E402

NEAR_M = 0.8
HEIGHT_M = 0.5

a = data.load(sys.argv[1])
run = pickle.load(open(sys.argv[2], "rb"))
filing = run["filing"]
objs = {o["id"]: o for o in run["objects"]}
stay = json.load(open(os.path.join(HERE, "labels", "stayed-put.json")))["objects"]
row = {int(i): k for k, i in enumerate(a["id"])}

members = {}
for oid, rec in filing.items():
    members.setdefault(rec, []).append(oid)

out = {}
for name, by_session in stay.items():
    ids = [int(i) for s, v in by_session.items() if isinstance(v, list) for i in v]
    ids = [i for i in ids if i in row]
    recs = Counter(filing.get(i) for i in ids)
    unfiled = recs.pop(None, 0)
    main, n_main = recs.most_common(1)[0]
    m = objs[main]
    labelled_others = {r: n for r, n in recs.items() if r != main}
    near = []
    for r, o in objs.items():
        if r == main or o["state"] != "present" and r not in labelled_others:
            continue
        d = math.hypot(o["x"] - m["x"], o["y"] - m["y"])
        dz = abs(o["z"] - m["z"]) if math.isfinite(o["z"]) and math.isfinite(m["z"]) else 0.0
        if r in labelled_others or (d <= NEAR_M and dz <= HEIGHT_M):
            mem = sorted(members.get(r, []), key=lambda i: a["t"][row[i]])
            near.append({"record": r, "d": round(d, 2), "dz": round(dz, 2), "state": o["state"],
                         "regions": len(mem), "labelled": labelled_others.get(r, 0),
                         "first": mem[:6], "frames": len(o["frames"])})
    near.sort(key=lambda x: (-x["labelled"], x["d"]))
    out[name] = {"labelled": len(ids), "unfiled": unfiled, "main": main, "main_n": n_main,
                 "main_regions": len(members.get(main, [])),
                 "main_xyz": [round(m["x"], 2), round(m["y"], 2), round(m["z"], 2)], "others": near}
    print("%-28s labelled %4d main %-6s %4d (of %4d in it)  unfiled %3d  other records holding labels %d (%d regions)  "
          "unlabelled nearby %d" % (name, len(ids), main, n_main, len(members.get(main, [])), unfiled,
                                    len(labelled_others), sum(labelled_others.values()),
                                    sum(1 for x in near if not x["labelled"])))
json.dump(out, open(sys.argv[3], "w"), indent=1)
