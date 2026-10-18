"""Why a look did not reach its target's record.

    python diagnose.py OUTCOMES.jsonl HISTORY.npz LOOKS.npz OUT.jsonl

For each look outcomes.py scored, the regions of its picture that point at the
target -- a sight line passing within NEAR_M (+ NEAR_PER_M per metre out) of
the target's place as the model held it before the look, between 0.3 and 6 m
ahead, and, where the region has a measured place, that place within PLACED_M
of it -- and what became of them:

- `fed`: one was filed to the target;
- `elsewhere`: they were filed to other records only, each reported with how
  many visits it had been seen on before the look (0: started by this look)
  and how far its place is from the target's;
- `unfiled`: they were left unfiled;
- `missed`: the picture has regions, none pointing at the target;
- `empty`: the picture has no posed region at all.
"""
import bisect
import collections
import json
import math
import sys

import numpy as np

NEAR_M, NEAR_PER_M, PLACED_M = 0.5, 0.05, 0.8


def main():
    outcomes_path, hist_path, looks_path, out_path = sys.argv[1:5]
    h = np.load(hist_path, allow_pickle=True)
    notes = collections.defaultdict(list)
    for t, oid, x, y, s, v in zip(h["t"], h["oid"], h["x"], h["y"], h["spread"], h["visits"]):
        notes[str(oid)].append((float(t), float(x), float(y), float(s), int(v)))
    for k in notes:
        notes[k].sort()
    times = {k: [n[0] for n in v] for k, v in notes.items()}
    filing = dict(zip(h["region"].tolist(), [str(r) for r in h["record"].tolist()]))
    lk = np.load(looks_path, allow_pickle=True)
    rows_of = collections.defaultdict(list)
    for k, fr in enumerate(lk["frame"].tolist()):
        rows_of[str(fr)].append(k)

    def before(oid, t):
        ns = notes.get(oid)
        if not ns:
            return None
        k = bisect.bisect_left(times[oid], t)
        return ns[k - 1] if k else None

    counts = collections.Counter()
    with open(out_path, "w") as out:
        for line in open(outcomes_path):
            g = json.loads(line)
            o = g["outcome"]
            tgt = before(o["target"], o["t_look"])
            if tgt is None:
                continue
            tx, ty = tgt[1], tgt[2]
            rows = [k for f in g["frames"] for k in rows_of.get(f, [])]
            posed = [k for k in rows if math.isfinite(lk["cx"][k]) and math.isfinite(lk["bearing"][k])]
            pointing = []
            for k in posed:
                b = math.radians(lk["bearing"][k])
                dx, dy = tx - lk["cx"][k], ty - lk["cy"][k]
                along = dx * math.cos(b) + dy * math.sin(b)
                across = abs(-dx * math.sin(b) + dy * math.cos(b))
                if not (0.3 <= along <= 6.0 and across <= NEAR_M + NEAR_PER_M * along):
                    continue
                if math.isfinite(lk["px"][k]) and math.hypot(lk["px"][k] - tx, lk["py"][k] - ty) > PLACED_M:
                    continue
                pointing.append(k)
            filed_to = [filing.get(int(lk["id"][k])) for k in pointing]
            if not posed:
                why = "empty"
            elif not pointing:
                why = "missed"
            elif o["target"] in filed_to:
                why = "fed"
            elif any(filed_to):
                why = "elsewhere"
            else:
                why = "unfiled"
            others = []
            for rec in sorted({r for r in filed_to if r and r != o["target"]}):
                b4 = before(rec, o["t_look"])
                first = notes[rec][0] if rec in notes else None
                place = b4 or first
                others.append({"record": rec, "visits_before": b4[4] if b4 else 0,
                               "from_target_m": round(math.hypot(place[1] - tx, place[2] - ty), 2) if place else None})
            g["why"] = why
            g["pointing"] = len(pointing)
            g["regions"] = len(posed)
            g["others"] = others
            counts[why] += 1
            out.write(json.dumps(g) + "\n")
    print(dict(counts))


if __name__ == "__main__":
    main()
