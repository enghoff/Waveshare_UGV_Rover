"""Score M10's look trial: did each look land, and how good were the predictions.

    python3 landed.py LOOKS.jsonl [OUT.json]

Run on the rover once the world model has filed the trial's pictures (its
`filed_until` past the last look). A look *landed* when a region of its
picture is filed to the record it was aimed at, following any merge of that
record since (`decisions.jsonl`). Then, by the rules declared before the trial
(docs/decisions/m10-predicts-whether-a-look-lands.md):

- the Brier score of the frozen predictor's chances against two baselines --
  the share of the history's looks that landed (115 of 430) as a constant,
  and the planner's old rule (a half for a record seen on six visits or more,
  none otherwise) -- with a bootstrap interval on the difference from the
  constant (criterion 3);
- calibration in bins of the prediction (criterion 5);
- at each stop, the look the new ranking would have chosen (chance times what
  it gains if it lands) against the old one's (what it gains if it lands), on
  the looks actually taken there, all of whose outcomes are known (criterion
  4's offline comparison).
"""
import collections
import json
import os
import random
import sqlite3
import sys

STATE = os.path.expanduser("~/.ugv/world/visit_model")
WORLD = os.path.expanduser("~/.ugv/world/world.db")
HISTORY_SHARE = 115 / 430
BINS = (0.0, 0.05, 0.1, 0.2, 0.3, 1.0001)


def brier(p, y):
    return sum((a - b) ** 2 for a, b in zip(p, y)) / len(y)


def main():
    looks = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
    looks = [l for l in looks if l.get("frame")]
    things = json.load(open(os.path.join(STATE, "things.json")))
    last = max(l["t"] for l in looks)
    if (things.get("filed_until") or 0) < last + 60:
        raise SystemExit("the world model has filed to %s, not yet past the last look; wait" % things.get("filed_until"))
    owner = {o: t["id"] for t in things["things"] for o in t["observations"]}
    spread = {t["id"]: t.get("spread_m") for t in things["things"]}
    merged = {}
    for line in open(os.path.join(STATE, "decisions.jsonl")):
        d = json.loads(line)
        if "merged" in d:
            merged[d["merged"]] = d["into"]

    def now_named(rid):
        seen = set()
        while rid in merged and rid not in seen:
            seen.add(rid)
            rid = merged[rid]
        return rid

    db = sqlite3.connect("file:%s?mode=ro" % WORLD, uri=True)
    for l in looks:
        ids = [r[0] for r in db.execute("select id from observations where frame_id = ?", (l["frame"],))]
        target = now_named(l["record"])
        filed_to = [owner.get(i) for i in ids]
        l["landed"] = target in filed_to
        l["filed_elsewhere"] = sorted({f for f in filed_to if f and f != target})
        l["spread_after_m"] = spread.get(target)

    y = [int(l["landed"]) for l in looks]
    p = [l["lands"] for l in looks]
    rule = [0.5 if (l.get("visits") or 0) >= 6 else 0.0 for l in looks]
    const = [HISTORY_SHARE] * len(looks)
    out = {"looks": len(looks), "landed": sum(y),
           "brier": {"learned": brier(p, y), "constant": brier(const, y), "old rule": brier(rule, y)}}
    rng = random.Random(0)
    diffs = []
    for _ in range(2000):
        idx = [rng.randrange(len(y)) for _ in y]
        diffs.append(brier([p[i] for i in idx], [y[i] for i in idx]) - brier([const[i] for i in idx], [y[i] for i in idx]))
    diffs.sort()
    out["learned_minus_constant_95"] = [diffs[50], diffs[1949]]
    cal = []
    for lo, hi in zip(BINS, BINS[1:]):
        sel = [(a, b) for a, b in zip(p, y) if lo <= a < hi]
        if sel:
            cal.append({"bin": [lo, min(hi, 1.0)], "looks": len(sel),
                        "predicted": round(sum(a for a, _ in sel) / len(sel), 3),
                        "landed": round(sum(b for _, b in sel) / len(sel), 3)})
    out["calibration"] = cal
    stops = collections.defaultdict(list)
    for l in looks:
        stops[(l["step"], l["stop"])].append(l)
    choice = collections.Counter()
    for key, ls in stops.items():
        old = max(ls, key=lambda l: (l.get("gain_if_filed_m") or 0.0, l["record"]))
        new = max(ls, key=lambda l: ((l.get("gain_if_filed_m") or 0.0) * l["lands"], l["record"]))
        choice["stops"] += 1
        choice["same choice"] += old is new
        choice["old choice landed"] += old["landed"]
        choice["new choice landed"] += new["landed"]
    out["at_each_stop"] = dict(choice)
    print(json.dumps(out, indent=1))
    if len(sys.argv) > 2:
        json.dump({"summary": out, "looks": looks}, open(sys.argv[2], "w"), indent=1)


if __name__ == "__main__":
    main()
