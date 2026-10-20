"""Every pair of records holding labelled pictures, after a whole run: how alike
records of one thing and of two things are, by the model's own object
likeness and colour. A record is a thing's when 80% or more of its labelled
regions (and at least two) show it; the two armchairs are one.

    python pair_likeness.py LOOKS.npz RECORDING ..."""
import sys
from collections import Counter, defaultdict

import data
import model
import recorded
import truth

ALIKE = {"plain armchair": "armchair", "throw armchair": "armchair"}
a = data.load(sys.argv[1])
recs = [recorded.Recording(p) for p in sys.argv[2:]]
filing, mem, _ = model.run(a, recs)
t_of = dict(zip(a["id"].tolist(), a["t"].tolist()))
held = defaultdict(Counter)
for obj, rs in truth.regions_by_object(t_of).items():
    for r in rs:
        if filing.get(r):
            held[filing[r]][ALIKE.get(obj, obj)] += 1
by = {o["id"]: o for o in mem.objects}
owner = {}
for k, c in held.items():
    obj, n = c.most_common(1)[0]
    if k in by and n >= 2 and n >= 0.8 * sum(c.values()):
        owner[k] = obj
keys = sorted(owner)
same, diff = [], []
for i, k0 in enumerate(keys):
    for k1 in keys[i + 1:]:
        o0, o1 = by[k0], by[k1]
        if o0["frames"] & o1["frames"]:
            continue
        row = (model._object_likeness(o0, o1), model._object_colour(o0, o1), k0, k1, owner[k0], owner[k1],
               o0["state"], o1["state"])
        (same if owner[k0] == owner[k1] else diff).append(row)
print("%d records owned, %d same-thing pairs, %d two-thing pairs" % (len(keys), len(same), len(diff)))
ok = [r for r in diff if r[1] >= model.COLOUR_MIN and model._heights_agree(by[r[2]]["z"], by[r[3]]["z"])]
ok.sort(key=lambda r: -r[0])
print("two-thing pairs passing colour and height, most alike:")
for r in ok[:12]:
    print("  %.3f colour %.2f %s (%s, %s) ~ %s (%s, %s)" % (r[0], r[1], r[2], r[4], r[6], r[3], r[5], r[7]))
same.sort(key=lambda r: -r[0])
print("same-thing pairs:")
for bar in (0.70, 0.72, 0.74, 0.76, 0.78, 0.81):
    s = sum(1 for r in same if r[0] >= bar and r[1] >= model.COLOUR_MIN)
    d = sum(1 for r in ok if r[0] >= bar)
    print("  bar %.2f: same %d of %d, two-thing %d" % (bar, s, len(same), d))
for obj in sorted({r[4] for r in same}):
    rows = [r for r in same if r[4] == obj]
    print("  %-22s %s" % (obj, " ".join("%.2f/%.2f" % (r[0], r[1]) for r in rows[:10])))
