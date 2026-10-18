"""Per record: the mean of its regions' DINOv2 and SigLIP vectors, and what
its regions measured, for a thing / not-a-thing classifier.
python3 record_features.py IDS.json OUT.npz   (IDS "all" for every record)"""
import json
import os
import sqlite3
import sys

import numpy as np

DB = os.path.expanduser("~/.ugv/world/world.db")
S = os.path.expanduser("~/.ugv/world/visit_model")
things = json.load(open(os.path.join(S, "things.json")))
recs = {t["id"]: t for t in things["things"]}
ids = list(recs) if sys.argv[1] == "all" else json.load(open(sys.argv[1]))
db = sqlite3.connect("file:%s?mode=ro" % DB, uri=True)


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


out = {k: [] for k in ("id", "dino", "siglip", "z", "extent", "visits", "spread", "n",
                       "rscore", "mask", "range", "area", "elev", "bottom", "top")}
for rid in ids:
    r = recs[rid]
    obs = r["observations"]
    q = ("select dino_alone_blob, siglip_blob, region_score, mask_share, range_m, bbox_json, elevation_deg "
         "from observations where id in (%s)") % ",".join("?" * len(obs))
    rows = db.execute(q, obs).fetchall()
    d = [unit(np.frombuffer(x[0], np.float32)) for x in rows if x[0]]
    s = [unit(np.frombuffer(x[1], np.float32)) for x in rows if x[1]]
    if not d or not s:
        continue
    med = lambda k: float(np.median([x[k] for x in rows if x[k] is not None])) if any(x[k] is not None for x in rows) else np.nan
    boxes = np.array([json.loads(x[5]) for x in rows])
    out["id"].append(rid)
    out["dino"].append(unit(np.mean(d, 0)))
    out["siglip"].append(unit(np.mean(s, 0)))
    out["z"].append(r.get("height_m") if r.get("height_m") is not None else np.nan)
    out["extent"].append(r.get("extent_m") or np.nan)
    out["visits"].append(r["visits"])
    out["spread"].append(r.get("spread_m") or np.nan)
    out["n"].append(len(rows))
    out["rscore"].append(med(2))
    out["mask"].append(med(3))
    out["range"].append(med(4))
    out["area"].append(float(np.median((boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1]))))
    out["elev"].append(med(6))
    out["bottom"].append(float(np.median(boxes[:, 3])))
    out["top"].append(float(np.median(boxes[:, 1])))
np.savez_compressed(sys.argv[2], **{k: np.array(v) for k, v in out.items()})
print(len(out["id"]), "records", np.array(out["dino"]).shape, np.array(out["siglip"]).shape)
