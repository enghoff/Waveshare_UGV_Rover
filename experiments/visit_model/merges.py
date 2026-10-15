"""The labelled regions behind each wrong merge of a filing, for development.

    python merges.py LOOKS.npz FILING.json

For each key holding two labelled objects in a session: every labelled region
it holds, with time, place and the decisions that touched the key.
"""
import json
import sys
import time

import numpy as np

import data
import score
import truth


def main():
    a = data.load(sys.argv[1])
    filing = {int(k): v for k, v in json.load(open(sys.argv[2])).items()}
    decisions = json.load(open(sys.argv[2] + ".decisions.json"))
    out = score.score(filing, a, roles=("development",))
    ss = truth.sessions()
    t_of = {int(i): float(t) for i, t in zip(a["id"], a["t"])}
    idx = {int(i): k for k, i in enumerate(a["id"])}
    objs = truth.regions_by_object(t_of, ss)
    for m in out["wrong_merges"]:
        key = m["key"]
        print("==", m["session"], key, m["objects"])
        for obj in m["objects"]:
            for rid, sess in sorted(objs[obj].items()):
                if filing.get(rid) != key:
                    continue
                i = idx[rid]
                where = "(%.2f, %.2f) %s" % (a["px"][i], a["py"][i], a["place_from"][i]) \
                    if np.isfinite(a["px"][i]) else "ray %.0f from (%.2f, %.2f)" % (
                        a["bearing"][i], a["cx"][i], a["cy"][i])
                print("   %-16s %-10s %d %s %s" % (obj, sess, rid,
                                                   time.strftime("%m-%d %H:%M:%S", time.localtime(t_of[rid])), where))
        merged = [d for d in decisions if d.get("into") == key or d.get("merged") == key]
        for d in merged[:12]:
            print("    ", time.strftime("%m-%d %H:%M:%S", time.localtime(d["t"])), d)


if __name__ == "__main__":
    main()
