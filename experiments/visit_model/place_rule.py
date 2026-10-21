"""Where LIDAR_LOW_M's rule fires, and whether the place it gives is nearer
the thing than the depth camera's, judged against the staged places.

    python place_rule.py LOOKS.npz OVER_M RECORDING ...

A labelled region of a staged thing is judged against where that thing was
set out (staged/*.json "at" or "to"), from its staging until its next one in
the session, outside bad-pose windows. Things that stayed put have no staged
place; for them only how often the rule fires is reported."""
import json
import math
import sys
from collections import Counter, defaultdict

import numpy as np

import data
import model
import recorded
import truth


def staged_places(s):
    """[(object, from time, until time or None, (x, y), approximate)]."""
    items = [i for key in ("added", "moves", "also_moved") for i in s.staged.get(key, [])
             if i["what"] in truth.OBJECTS]
    rows = []
    for i in items:
        at = i.get("at") or i.get("to")
        if at:
            rows.append((truth.OBJECTS[i["what"]], s._at_step(i), tuple(at), bool(i.get("approximate"))))
    out = []
    for obj in {r[0] for r in rows}:
        mine = sorted(r for r in rows if r[0] == obj)
        for k, (o, t, at, approx) in enumerate(mine):
            out.append((o, t, mine[k + 1][1] if k + 1 < len(mine) else None, at, approx))
    return out


def main():
    a = data.load(sys.argv[1])
    over_m = float(sys.argv[2])
    recs = [recorded.Recording(p) for p in sys.argv[3:]]
    b, moved = model.lidar_low(a, recs, over_m)
    moved = set(moved)
    ss = truth.sessions()
    t_of = {int(i): float(t) for i, t in zip(a["id"], a["t"])}
    row = {int(i): k for k, i in enumerate(a["id"])}
    by_obj = truth.regions_by_object(t_of, ss)
    labelled = {}
    for obj, regions in by_obj.items():
        for rid, sess in regions.items():
            labelled[rid] = (obj, sess)
    print("rule at %.2f m: %d regions moved of %d depth-placed" % (
        over_m, len(moved), int(np.sum(np.isin(np.array([str(x) for x in a["place_from"]]), ("outline", "box"))))))
    fired = Counter()
    for k in moved:
        rid = int(a["id"][k])
        fired[labelled[rid][0] if rid in labelled else "(unlabelled)"] += 1
    print("moved, by labelled object:", dict(fired.most_common()))

    judged = defaultdict(list)
    for s in ss:
        for obj, t0, t1, at, approx in staged_places(s):
            for rid, sess in by_obj.get(obj, {}).items():
                if sess != s.name or rid not in row:
                    continue
                k = row[rid]
                t = t_of[rid]
                if t < t0 or (t1 is not None and t >= t1) or s.is_bad(t) or k not in moved:
                    continue
                e_depth = math.hypot(a["px"][k] - at[0], a["py"][k] - at[1])
                e_rule = math.hypot(b["px"][k] - at[0], b["py"][k] - at[1])
                judged[obj].append((s.name, rid, e_depth, e_rule, approx, float(a["range"][k]), float(b["range"][k])))
    allrows = [r for v in judged.values() for r in v]
    print("judged against staged places: %d moved regions" % len(allrows))
    for obj, rs in sorted(judged.items()):
        ed = np.array([r[2] for r in rs])
        er = np.array([r[3] for r in rs])
        print("  %-16s %3d  depth median %.2f m, rule %.2f m; rule nearer in %d, farther in %d; "
              "within 0.5 m: depth %d, rule %d" % (
                  obj, len(rs), np.median(ed), np.median(er), int(np.sum(er < ed)), int(np.sum(er > ed)),
                  int(np.sum(ed <= 0.5)), int(np.sum(er <= 0.5))))
    if allrows:
        ed = np.array([r[2] for r in allrows])
        er = np.array([r[3] for r in allrows])
        print("  all              %3d  depth median %.2f m, rule %.2f m; rule nearer in %d, farther in %d" % (
            len(allrows), np.median(ed), np.median(er), int(np.sum(er < ed)), int(np.sum(er > ed))))
        print("  worst made by the rule:")
        for r in sorted(allrows, key=lambda r: r[2] - r[3])[:8]:
            print("    %s %s %d: depth %.2f m off (range %.2f), rule %.2f m off (range %.2f)%s" % (
                r[0], next(o for o, v in judged.items() if r in v), r[1], r[2], r[5], r[3], r[6],
                " approx" if r[4] else ""))
    json.dump({"over_m": over_m, "moved": sorted(int(a["id"][k]) for k in moved),
               "judged": {o: v for o, v in judged.items()}},
              open("%s/visit-model/place-rule-%.2f.json" % (truth.TMP, over_m), "w"), indent=0)


if __name__ == "__main__":
    main()
