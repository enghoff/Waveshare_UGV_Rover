"""Each recorded look as M10's predictor sees it: what was known when it was
chosen, and whether it landed.

    python features.py OUTCOMES.jsonl HISTORY.npz OUT.jsonl

A look *lands* when a region of its picture was filed to the record it was
aimed at (outcomes.py's `fed`). Everything else is read from the replayed
world model strictly before the look's picture, or from what the goal planned:

- `visits`: how many visits the target had been seen on;
- `spread_m`: the target's placement spread;
- `rival`: another record at least as established standing within RIVAL_M;
- `crowd`: how many other records stood within RIVAL_M;
- `nearest_m`: the nearest other record, capped at 2 m;
- `since_h`: hours since the target was last filed to, which the model weighs: a
  record unseen for three hours takes a look only on a move's likeness;
- `range_m`, `in_depth_view`, `needs_movement`, `elevation_deg`: the look as
  planned.

Looks whose target was found only by the aim point are left out: which record
they meant is a guess.
"""
import bisect
import collections
import json
import math
import sys

import numpy as np

RIVAL_M = 0.8


def main():
    outcomes_path, hist_path, out_path = sys.argv[1:4]
    h = np.load(hist_path, allow_pickle=True)
    notes = collections.defaultdict(list)
    for t, oid, x, y, s, v in zip(h["t"], h["oid"], h["x"], h["y"], h["spread"], h["visits"]):
        notes[str(oid)].append((float(t), float(x), float(y), float(s), int(v)))
    for k in notes:
        notes[k].sort()
    times = {k: [n[0] for n in v] for k, v in notes.items()}

    def before(oid, t):
        k = bisect.bisect_left(times[oid], t)
        return notes[oid][k - 1] if k else None

    n = 0
    with open(out_path, "w") as out:
        for line in open(outcomes_path):
            g = json.loads(line)
            if g.get("target_by") == "nearest the aim":
                continue
            o = g["outcome"]
            t = o["t_look"]
            tgt = before(o["target"], t)
            if tgt is None:
                continue
            crowd, rival, nearest = 0, False, 2.0
            for rec in notes:
                if rec == o["target"] or notes[rec][0][0] >= t:
                    continue
                b = before(rec, t)
                d = math.hypot(b[1] - tgt[1], b[2] - tgt[2])
                nearest = min(nearest, d)
                if d <= RIVAL_M:
                    crowd += 1
                    rival = rival or b[4] >= tgt[4]
            c = g.get("constraints") or {}
            row = {"episode": g["episode"], "at": g["at"], "t_look": t, "target": o["target"],
                   "lands": bool(o["fed"]), "visits": tgt[4], "spread_m": round(tgt[3], 4),
                   "rival": rival, "crowd": crowd, "nearest_m": round(nearest, 3),
                   "since_h": round((t - tgt[0]) / 3600.0, 3),
                   "range_m": c.get("range_m"), "in_depth_view": c.get("in_depth_view"),
                   "needs_movement": c.get("needs_movement"), "elevation_deg": c.get("elevation_deg"),
                   "predicted_gain_m": (g.get("gain_detail") or {}).get("gain_m")}
            out.write(json.dumps(row) + "\n")
            n += 1
    print(n, "looks")


if __name__ == "__main__":
    main()
