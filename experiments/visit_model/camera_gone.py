"""What the camera's absence rule called gone (version 5), judged.

    python camera_gone.py LOOKS.npz FILING.json [BASE_FILING.json]

Every "gone" decision taken "by": "camera" in FILING.json.decisions.json, with
the labelled object its record holds, if any:

- a record holding an object never staged (labels/stayed-put.json) called gone
  is a false claim: they were there throughout;
- one holding a staged thing is listed with when the thing was staged, to be
  read against the session's record;
- one holding no labelled object is counted, by how many visits it was seen on.

And the records left standing at the end, against BASE_FILING (version 4) when
given, by how many visits each was seen on.
"""
import collections
import json
import sys
import time

import numpy as np

import model
import truth


def standing(filing_path, t_of):
    """{record: visits seen} for the records not gone at the end."""
    decisions = json.load(open(filing_path + ".decisions.json"))
    state = {}
    for d in decisions:
        if "gone" in d:
            state[d["gone"]] = "gone"
        for key in ("new", "here", "moved", "into"):
            if key in d and isinstance(d[key], str):
                state[d[key]] = "present"
    filing = json.load(open(filing_path))
    members = collections.defaultdict(list)
    for r, o in filing.items():
        if o is not None and int(r) in t_of:
            members[o].append(t_of[int(r)])
    out = {}
    for o, ts in members.items():
        if state.get(o) == "gone":
            continue
        ts.sort()
        out[o] = sum(1 for k, x in enumerate(ts) if k == 0 or x - ts[k - 1] > model.GAP_S)
    return out


def band(v):
    return "1" if v == 1 else "2" if v == 2 else "3-5" if v <= 5 else "6+"


def main():
    looks, filing_path = sys.argv[1:3]
    base = sys.argv[3] if len(sys.argv) > 3 else None
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
    cam = [d for d in decisions if d.get("by") == "camera" and "gone" in d]
    print("gone by the camera: %d" % len(cam))
    false, staged, unlabelled = [], [], collections.Counter()
    for d in cam:
        objs = holds.get(d["gone"])
        when = time.strftime("%m-%d %H:%M", time.localtime(d["t"]))
        if not objs:
            unlabelled["records"] += 1
            continue
        obj, n = objs.most_common(1)[0]
        line = "  %s %-6s %s (%d of its labelled regions), %d empty visits, %d pictures" % (
            when, d["gone"], obj, n, d["empty_visits"], d["pictures"])
        (false if obj in stayed else staged).append(line)
    print("records of objects never staged, called gone (false): %d" % len(false))
    for line in false:
        print(line)
    print("records of staged things, called gone (to be read against the sessions): %d" % len(staged))
    for line in staged:
        print(line)
    print("records holding no labelled object, called gone: %d" % unlabelled["records"])
    now = standing(filing_path, t_of)
    print("standing at the end, by visits seen: %s, total %d" % (
        dict(sorted(collections.Counter(band(v) for v in now.values()).items())), len(now)))
    if base:
        then = standing(base, t_of)
        print("version 4 standing at the end:      %s, total %d" % (
            dict(sorted(collections.Counter(band(v) for v in then.values()).items())), len(then)))


if __name__ == "__main__":
    main()
