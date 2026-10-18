"""What each recorded look did to its target, on the world model's records.

    python outcomes.py GOALS.jsonl HISTORY.npz LOOKS.npz OUT.jsonl

GOALS from inventory.py, HISTORY from replay_history.py, LOOKS the looks file
the history was replayed from. For every `improve_geometry` goal that took a
picture, the target record is found -- by name since 2026-10-17, when goals
name the world model's records; before that, the replayed record standing
nearest the place the look was aimed at, within NEAR_M, among those noted
before the look -- and its spread is read at its last note before the look's
picture and at its first note from the look's visit on. The look improved its
target when a region of its picture was filed to it and the spread fell.
Writes the goal rows with `outcome` added: target, spread before and after,
whether the look's picture fed it, and the gain realised (before minus after,
zero when the picture fed nothing to it).
"""
import bisect
import collections
import json
import math
import os
import sys
import time

import numpy as np

NEAR_M = 0.6
MODEL_SINCE = time.mktime((2026, 10, 17, 0, 0, 0, 0, 0, -1))


def main():
    goals_path, hist_path, looks_path, out_path = sys.argv[1:5]
    h = np.load(hist_path, allow_pickle=True)
    notes = collections.defaultdict(list)          # record -> [(t, spread, x, y, visits)]
    for t, oid, x, y, s, v in zip(h["t"], h["oid"], h["x"], h["y"], h["spread"], h["visits"]):
        notes[str(oid)].append((float(t), float(s), float(x), float(y), int(v)))
    for k in notes:
        notes[k].sort()
    filing = dict(zip(h["region"].tolist(), [str(r) for r in h["record"].tolist()]))
    lk = np.load(looks_path, allow_pickle=True)
    by_frame = collections.defaultdict(list)
    t_of_frame = {}
    for rid, fr, t in zip(lk["id"].tolist(), lk["frame"].tolist(), lk["t"].tolist()):
        by_frame[str(fr)].append(int(rid))
        t_of_frame[str(fr)] = min(t_of_frame.get(str(fr), t), t)

    def state_at(oid, t):
        """The record's last note before t, and its first note at or after t."""
        ns = notes.get(oid)
        if not ns:
            return None, None
        k = bisect.bisect_left([n[0] for n in ns], t)
        return (ns[k - 1] if k else None), (ns[k] if k < len(ns) else None)

    # The store's thing each region was filed under, for goals that named a
    # store thing: the record intended is the one holding most of that thing's
    # earlier pictures as the model filed them (the method of
    # docs/progress/2026-10-18-aimed-looks-feed-only-established-records.md).
    by_entity = collections.defaultdict(list)
    for rid, ent, t in zip(lk["id"].tolist(), lk["entity"].tolist(), lk["t"].tolist()):
        if ent:
            by_entity[str(ent)].append((float(t), int(rid)))

    def record_of_thing(entity, t):
        held = collections.Counter(filing.get(r) for tt, r in by_entity.get(entity, ()) if tt < t)
        held.pop(None, None)
        return held.most_common(1)[0][0] if held else None

    def nearest_record(x, y, t):
        best = None
        for oid, ns in notes.items():
            if ns[0][0] >= t:
                continue
            before, _ = state_at(oid, t)
            d = math.hypot(before[2] - x, before[3] - y)
            if d <= NEAR_M and (best is None or d < best[0]):
                best = (d, oid)
        return best[1] if best else None

    counts = collections.Counter()
    with open(out_path, "w") as out:
        for line in open(goals_path):
            g = json.loads(line)
            if g["type"] != "improve_geometry":
                continue
            frames = [f for f in g.get("frames") or [] if f in t_of_frame]
            if not frames:
                counts["no picture in the looks"] += 1
                continue
            t_look = min(t_of_frame[f] for f in frames)
            aim = (g.get("constraints") or {}).get("look_at") or {}
            if g["at"] >= MODEL_SINCE and g.get("target", "").startswith("v"):
                target = g["target"] if g["target"] in notes else None
            else:
                target = record_of_thing("ws/%s" % g["target"], t_look) or \
                    record_of_thing(g["target"], t_look)
                g["target_by"] = "the thing's earlier pictures" if target else None
                if target is None and aim.get("x_m") is not None:
                    target = nearest_record(aim["x_m"], aim["y_m"], t_look)
                    g["target_by"] = "nearest the aim" if target else None
            if target is None:
                counts["no target record found"] += 1
                continue
            before, after = state_at(target, t_look)
            if before is None:
                counts["target had no place before the look"] += 1
                continue
            regions = [r for f in frames for r in by_frame[f]]
            fed = any(filing.get(r) == target for r in regions)
            spread_after = after[1] if (fed and after is not None) else before[1]
            g["outcome"] = {"target": target, "t_look": t_look, "spread_before": round(before[1], 4),
                            "spread_after": round(spread_after, 4), "fed": fed,
                            "visits_before": before[4],
                            "gain": round(before[1] - spread_after, 4)}
            counts["scored"] += 1
            counts["fed" if fed else "not fed"] += 1
            if before[1] - spread_after > 1e-4:
                counts["improved"] += 1
            out.write(json.dumps(g) + "\n")
    print(dict(counts))


if __name__ == "__main__":
    main()
