"""For each labelled object, the keys holding two or more of its development
regions, with where each key's first decision placed it and how."""
import json
import sys
import time
from collections import Counter

import data
import truth

a = data.load(sys.argv[1])
filing = {int(k): v for k, v in json.load(open(sys.argv[2])).items()}
dec = json.load(open(sys.argv[2] + ".decisions.json"))
first = {}
for d in dec:
    for k in ("new",):
        if k in d and d[k] not in first:
            first[d[k]] = d
t_of = {int(i): float(t) for i, t in zip(a["id"], a["t"])}
ss = truth.sessions()
roles = {s.name: s.role for s in ss}
objs = truth.regions_by_object(t_of, ss)
for obj, mine in objs.items():
    c = Counter(filing.get(r) for r, s in mine.items() if roles[s] == "development" and filing.get(r))
    keys = [(k, n) for k, n in c.most_common() if n >= 2]
    if len(keys) < 2:
        continue
    print(obj)
    for k, n in keys:
        f = first.get(k, {})
        print("   %-6s %3d  made %s at %s from %s regions" % (
            k, n, time.strftime("%m-%d %H:%M", time.localtime(f["t"])) if f else "?", f.get("at"), f.get("n")))
