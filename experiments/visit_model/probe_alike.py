"""Version 4's memory just before a visit, and how alike each labelled thing's
regions in that visit look to every record: what a move rule could go on.

    python probe_alike.py LOOKS.npz FILING.json STOP_T [RECORDING ...]

Version 4 unless a switch is set. STOP_T: the first visit ending at or after it is probed, not filed."""
import json
import math
import os
import sys
import time
from collections import Counter, defaultdict

import numpy as np

import data
import model
import recorded
import truth

a = data.load(sys.argv[1])
final = {int(k): v for k, v in json.load(open(sys.argv[2])).items()}
stop = float(sys.argv[3])
recs = [recorded.Recording(p) for p in sys.argv[4:]]
t_of = dict(zip(a["id"].tolist(), a["t"].tolist()))
objs = truth.regions_by_object(t_of)
label = {r: obj for obj, rs in objs.items() for r in rs}
held = defaultdict(Counter)
for r, obj in label.items():
    if final.get(r):
        held[final[r]][obj] += 1

L = model.Looks(a)
mem = model.Memory()
decisions = []
WATCH = os.environ.get("WATCH", "").split(",") if os.environ.get("WATCH") else []
for idx in model.windows(a["t"]):
    t = float(a["t"][idx].max())
    if t >= stop:
        break
    if WATCH and t >= float(os.environ.get("WATCH_FROM", 0)):
        t0 = float(a["t"][idx].min())
        cx, cy = a["cx"][idx][L.posed[idx]], a["cy"][idx][L.posed[idx]]
        for o in mem.objects:
            if o["id"] in WATCH and len(cx):
                near = float(np.min(np.hypot(cx - o["x"], cy - o["y"])))
                cov = recorded.for_time(recs, t0) is not None or recorded.for_time(recs, t) is not None
                if near <= model.GONE_REACH_M and cov:
                    p_, h_ = model.lidar.outcomes(recs, np.array([o["x"]]), np.array([o["y"]]), t0, t,
                                                  after=np.array([o["seen"]]))
                    print("  %s %s visit %s: camera within %.1fm, solid %d/%d, lidar passes %d hits %d" % (
                        o["id"], o["state"], time.strftime("%m-%d %H:%M", time.localtime(t)), near,
                        o["solid_hits"], o["solid_judged"], int(p_[0]), int(h_[0])))
    mem.visit += 1
    filed = model.reconcile(L, mem, model.cluster(L, idx), t, decisions)
    if recs:
        model.solid_votes(L, mem, filed, recs)
        model.absent(L, mem, idx, filed, recs, decisions)
    model.consolidate(mem, sorted({mem.resolve(v) for v in filed.values()}), t, decisions)
    model.settle_pending(mem, t, decisions)

print("probed visit %s-%s, %d regions" % (time.strftime("%m-%d %H:%M:%S", time.localtime(a["t"][idx].min())),
                                         time.strftime("%H:%M:%S", time.localtime(t)), len(idx)))
row = {int(i): k for k, i in enumerate(a["id"])}
groups = defaultdict(list)
for i in idx:
    obj = label.get(int(a["id"][i]))
    if obj:
        groups[obj].append(i)
for obj, m in groups.items():
    vecs = L.a["vec"][m]
    cols = model.cols_of(L, m)
    p = model.place_of(L, m)
    print("\n%s: %d regions, place %s" % (obj, len(m), None if not p else "(%.2f, %.2f)" % (p["x"], p["y"])))
    rows = []
    for o in mem.objects:
        rows.append((mem.likeness(vecs, o), mem.colour(cols, o), o))
    rows.sort(key=lambda r: -r[0])
    for cos, col, o in rows[:10]:
        d = math.hypot(o["x"] - p["x"], o["y"] - p["y"]) if p else float("nan")
        print("  %-6s like %.3f colour %.2f %-7s seen %s d %.1fm holds %s" % (
            o["id"], cos, col, o["state"], time.strftime("%m-%d %H:%M", time.localtime(o["seen"])), d,
            dict(held.get(o["id"], {}))))
    # Every record of the thing, wherever it ranks.
    mine = [r for r in rows if obj in held.get(r[2]["id"], {})]
    print("  records of it: " + ", ".join("%s %.3f/%.2f %s" % (o["id"], c, cl, o["state"]) for c, cl, o in mine))
