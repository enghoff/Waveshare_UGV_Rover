"""What the camera's rule set aside as not found again, judged.

    python unfound.py LOOKS.npz FILING.json

From FILING.json.decisions.json: every record set aside ("unfound") and every
one found again after it. A record holding an object never staged
(labels/stayed-put.json) set aside is a claim about a thing that was there --
counted with whether a later look took it off again, and whether it is still
set aside at the end; a staged thing's record is listed with when; the rest
are counted. And the records standing at the end that are not set aside, by
how many visits each was seen on.
"""
import collections
import json
import sys
import time

import numpy as np

import model
import truth


def main():
    looks, filing_path = sys.argv[1:3]
    a = np.load(looks, allow_pickle=True)
    t_of = {int(r): float(t) for r, t in zip(a["id"], a["t"])}
    filing = {int(k): v for k, v in json.load(open(filing_path)).items()}
    decisions = json.load(open(filing_path + ".decisions.json"))
    by_obj = truth.regions_by_object(t_of)
    stayed = set(truth.stayed_put())
    holds = collections.defaultdict(collections.Counter)
    for obj, regions in by_obj.items():
        for r in regions:
            if filing.get(r):
                holds[filing[r]][obj] += 1
    aside, again, state, merged = {}, collections.Counter(), {}, {}
    for d in decisions:
        if "unfound" in d:
            aside.setdefault(d["unfound"], []).append(d)
        if "found again" in d:
            again[d["found again"]] += 1
        if "gone" in d:
            state[d["gone"]] = "gone"
        for key in ("new", "here", "moved"):
            if isinstance(d.get(key), str):
                state[d[key]] = "present"
        if "merged" in d:
            merged[d["merged"]] = d["into"]
    still = collections.Counter()
    for rid, ds in aside.items():
        still[rid] = len(ds) - again[rid]
    print("set aside: %d records, %d times; found again after: %d times" % (
        len(aside), sum(len(v) for v in aside.values()), sum(again.values())))
    false = [r for r in aside if holds.get(r) and holds[r].most_common(1)[0][0] in stayed]
    staged = [r for r in aside if holds.get(r) and holds[r].most_common(1)[0][0] not in stayed]
    print("records of objects never staged, set aside: %d" % len(false))
    for r in false:
        obj, n = holds[r].most_common(1)[0]
        print("  %-6s %-28s %3d labelled pictures; set aside %s; %s at the end" % (
            r, obj, n, ", ".join(time.strftime("%m-%d %H:%M", time.localtime(d["t"])) for d in aside[r]),
            "merged away" if r in merged else "still aside" if still[r] > 0 else "found again"))
    print("records of staged things, set aside: %d" % len(staged))
    for r in staged:
        obj, n = holds[r].most_common(1)[0]
        print("  %-6s %-28s %3d labelled pictures; set aside %s; %s at the end" % (
            r, obj, n, ", ".join(time.strftime("%m-%d %H:%M", time.localtime(d["t"])) for d in aside[r]),
            "merged away" if r in merged else "still aside" if still[r] > 0 else "found again"))
    print("records holding no labelled object, set aside: %d" % (len(aside) - len(false) - len(staged)))
    members = collections.defaultdict(list)
    for r, o in filing.items():
        if o is not None and r in t_of:
            members[o].append(t_of[r])
    counts = collections.Counter()
    for o, ts in members.items():
        if state.get(o) == "gone":
            continue
        ts.sort()
        v = sum(1 for k, x in enumerate(ts) if k == 0 or x - ts[k - 1] > model.GAP_S)
        b = "1" if v == 1 else "2" if v == 2 else "3-5" if v <= 5 else "6+"
        counts[(b, "aside" if still[o] > 0 else "standing")] += 1
    for b in ("1", "2", "3-5", "6+"):
        print("  seen on %-3s visits: standing %3d, set aside %3d" % (
            b, counts[(b, "standing")], counts[(b, "aside")]))


if __name__ == "__main__":
    main()
