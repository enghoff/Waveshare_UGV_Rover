"""Gone events of records that hold labelled pictures of things never moved
(stayed-put.json): each one called a thing gone that was there.

    python false_gone.py LOOKS.npz NAME=FILING.json ..."""
import json
import sys
import time
from collections import Counter, defaultdict

import data
import truth

a = data.load(sys.argv[1])
t_of = dict(zip(a["id"].tolist(), a["t"].tolist()))
objs = truth.regions_by_object(t_of)
still = set(truth.stayed_put())
for arg in sys.argv[2:]:
    name, path = arg.split("=", 1)
    filing = {int(k): v for k, v in json.load(open(path)).items()}
    dec = json.load(open(path + ".decisions.json"))
    held = defaultdict(Counter)
    for obj, rs in objs.items():
        for r in rs:
            if filing.get(r):
                held[filing[r]][obj] += 1
    gone = [d for d in dec if "gone" in d]
    bad = []
    for d in gone:
        c = held.get(d["gone"], Counter())
        s = {o: n for o, n in c.items() if o in still and n >= 2}
        if s:
            bad.append((d, s))
    print("%s: %d gone events, %d of records holding a thing never moved" % (name, len(gone), len(bad)))
    for d, s in bad:
        print("   %s %s at %s %s" % (time.strftime("%m-%d %H:%M", time.localtime(d["t"])), d["gone"], d["at"], s))
