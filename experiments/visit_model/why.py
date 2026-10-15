"""Where one labelled object's regions went, for development.

    python why.py LOOKS.npz FILING.json OBJECT [--test]

Per session, each labelled region: its time, the object it was filed to, where
that region stood (measured place, or none), and the objects' current places.
Development only unless --test.
"""
import json
import sys
import time
from collections import Counter

import numpy as np

import data
import truth


def main():
    args = [x for x in sys.argv[1:] if x != "--test"]
    test = "--test" in sys.argv
    a = data.load(args[0])
    filing = {int(k): v for k, v in json.load(open(args[1])).items()}
    decisions = json.load(open(args[1] + ".decisions.json"))
    obj = args[2]
    ss = truth.sessions()
    t_of = dict(zip(a["id"].tolist(), range(len(a["id"]))))
    objs = truth.regions_by_object({int(i): float(t) for i, t in zip(a["id"], a["t"])}, ss)
    roles = {s.name: s.role for s in ss}
    rows = sorted(objs[obj].items(), key=lambda kv: a["t"][t_of[kv[0]]])
    keys = Counter()
    for rid, sess in rows:
        if roles[sess] == "test" and not test:
            continue
        i = t_of[rid]
        key = filing.get(rid)
        keys[key] += 1
        place = "(%.2f, %.2f) %s" % (a["px"][i], a["py"][i], a["place_from"][i]) \
            if np.isfinite(a["px"][i]) else "ray %.0f deg from (%.2f, %.2f)" % (
                a["bearing"][i], a["cx"][i], a["cy"][i]) if np.isfinite(a["bearing"][i]) else "unposed"
        print("%-10s %d %s %-6s %s" % (sess, rid, time.strftime("%H:%M:%S", time.localtime(a["t"][i])),
                                       key, place))
    print(keys.most_common())
    for key, _n in keys.most_common():
        events = [d for d in decisions if key in (d.get("moved"), d.get("gone"), d.get("new"),
                                                  d.get("here"), d.get("sighted"))]
        print(key, len(events))
        for d in events[:40]:
            print("    ", time.strftime("%m-%d %H:%M:%S", time.localtime(d["t"])),
                  {k: v for k, v in d.items() if k != "t"})


if __name__ == "__main__":
    main()
