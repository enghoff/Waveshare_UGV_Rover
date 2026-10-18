"""For each trial look, the regions of its picture that point at the target
record, and where the world model filed each: the target, another record (how
far from the target, how alike), or nowhere. Read-only.

    python3 pointing.py LANDED.json OUT.json     (on the rover; landed.py's output)

A region points at the target when its sight line passes within 0.3 m, half
the target's width and half the region's span of the target's place, 0.3 to
6 m ahead, and its measured place, if it has one, is within 0.6 m and half the
width of it. A look is "landed", "pointing regions filed elsewhere",
"pointing regions unfiled", "nothing points at it", or "target not standing"
(merged away since)."""
import json
import math
import os
import pickle
import sqlite3
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser("~/ugv/world_state/visit_model"))
import engine  # noqa: E402
import model  # noqa: E402

S = os.path.expanduser("~/.ugv/world/visit_model")
DB = os.path.expanduser("~/.ugv/world/world.db")
e = pickle.load(open(os.path.join(S, "engine.pkl"), "rb"))
objs = {o["id"]: o for o in e.mem.objects}
things = json.load(open(os.path.join(S, "things.json")))
owner = {o: t["id"] for t in things["things"] for o in t["observations"]}
looks = json.load(open(sys.argv[1]))["looks"]

con = sqlite3.connect("file:%s?mode=ro" % DB, uri=True)
frames = [l["frame"] for l in looks]
q = "select min(id) from observations where frame_id in (%s)" % ",".join("?" * len(frames))
first = con.execute(q, frames).fetchone()[0]
a = engine.prepare(DB, (), after_id=first - 1, session=things["map_session"])
a["vec"] = a["lvec"]
by_frame = {}
for i, f in enumerate(a["frame"]):
    by_frame.setdefault(f, []).append(i)

out = []
for l in looks:
    t = objs.get(l["record"])
    if t is None:
        out.append({"frame": l["frame"], "record": l["record"], "landed": l["landed"], "class": "target not standing"})
        continue
    rows = []
    for i in by_frame.get(l["frame"], []):
        if not math.isfinite(a["cx"][i]):
            continue
        b = math.radians(a["bearing"][i])
        ux, uy = math.cos(b), math.sin(b)
        dx, dy = t["x"] - a["cx"][i], t["y"] - a["cy"][i]
        along = dx * ux + dy * uy
        across = abs(-dx * uy + dy * ux)
        half = math.radians(min(float(a["span"][i]) if math.isfinite(a["span"][i]) else 10.0, 60.0) / 2.0)
        slack = 0.3 + t["extent"] / 2.0 + along * math.tan(half)
        if not (0.3 <= along <= 6.0 and across <= slack):
            continue
        placed = math.isfinite(a["px"][i])
        off = math.hypot(a["px"][i] - t["x"], a["py"][i] - t["y"]) if placed else None
        if placed and off > 0.6 + t["extent"] / 2.0:
            continue  # its measured place is elsewhere: something in front or behind
        oid = int(a["id"][i])
        rid = owner.get(oid)
        o = objs.get(rid)
        cos = float(model.sim(a["vec"][i] @ t["ex"].T).max())
        rows.append({"region": oid, "to": rid, "off_m": None if off is None else round(off, 2),
                     "cos_target": round(cos, 3),
                     "to_dist": None if o is None else round(math.hypot(o["x"] - t["x"], o["y"] - t["y"]), 2),
                     "to_visits": None if o is None else len(o["visits"]),
                     "to_like_target": None if o is None or o is t else round(model._object_likeness(o, t), 3)})
    if l["landed"]:
        c = "landed"
    elif not rows:
        c = "nothing points at it"
    elif all(r["to"] is None for r in rows):
        c = "pointing regions unfiled"
    else:
        c = "pointing regions filed elsewhere"
    out.append({"frame": l["frame"], "stop": l["step"] + "/" + l["stop"], "record": l["record"],
                "landed": l["landed"], "since_h": l["since_h"], "target_visits": len(t["visits"]),
                "target_z": round(t["z"], 2) if math.isfinite(t["z"]) else None,
                "class": c, "pointing": rows, "regions": len(by_frame.get(l["frame"], []))})
json.dump(out, open(sys.argv[2], "w"), indent=1)
from collections import Counter
print(Counter(o["class"] for o in out))
