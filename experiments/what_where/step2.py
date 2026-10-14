"""Step 2 (step2.md): the step 1 streams with navigation's pose from the drive
recordings, and room-controls judged by where a flagged thing stands.

    python step2.py RECORDINGS_DIR

Run from the repository root; writes step2_development.json and
step2_results.json beside this file.
"""
import glob
import json
import os
import sys

import numpy as np

import load
import recorded
import run
import streams

HERE = os.path.dirname(os.path.abspath(__file__))
#: After this the rover stood 20 cm out short of the charger (the shakedown's
#: entry); finds without it are reported beside the rest.
LATE = "08:17:15"


def recordings(folder):
    return [recorded.Recording(p) for p in sorted(glob.glob(os.path.join(folder, "*.npz")))]


def development(recs):
    day = "2026-10-11"
    allr = load.load(run.DB, 1.7e9, run.ts(day, "13:08"), recordings=recs)
    what, where, gone, per = [], [], [], []
    for a, b in run.CONTROLS:
        q, m, out = run.score_visit(allr, run.ts(day, a), run.ts(day, b), run.ts(day, a))
        fl = q.floor_sized()
        what.append(out["what"][fl])
        where.append(out["where"][fl & np.isfinite(out["where"])])
        g = [s for s, n in out["visit_gone"].values() if n >= 2]
        gone += g
        per.append({"control": f"{a}-{b}", "regions": int(fl.sum()),
                    "posed": int((fl & q.posed).sum()),
                    "recorded_pose": int((fl & (q.source == "recorded")).sum()),
                    "with_references": int((fl & np.isfinite(out["where"])).sum()),
                    "expected_twice": len(g)})
    what, where, gone = np.concatenate(what), np.concatenate(where), np.array(gone)
    dev = {"T_what": float(np.percentile(what, run.PERCENTILE)),
           "T_where": float(np.percentile(where, run.PERCENTILE)),
           "T_gone": float(np.percentile(gone, run.PERCENTILE)),
           "n_what": len(what), "n_where": len(where), "n_gone": len(gone),
           "controls": per}
    json.dump(dev, open(os.path.join(HERE, "step2_development.json"), "w"), indent=1)
    return dev


def place_of(q, i, recs):
    """Where a flagged region's thing stands: its depth point, else the lidar's."""
    if q.ranged[i]:
        return float(q.px[i]), float(q.py[i]), "depth"
    if not q.posed[i]:
        return None
    rec = recorded.for_time(recs, q.shutter[i])
    p = None if rec is None else rec.lidar_point(q.shutter[i], q.bearing[i])
    return None if p is None else (p[0], p[1], "lidar")


def test(recs, dev):
    tw, th, tg = dev["T_what"], dev["T_where"], dev["T_gone"]
    day = "2026-10-12"
    labels = json.load(open(run.LABELS))["subjects"]
    staged = json.load(open(run.STAGED))
    allr = load.load(run.DB, 1.7e9, run.ts(day, "08:21:00"), recordings=recs)
    cutoff, late = run.ts(day, run.CUTOFF), run.ts(day, LATE)
    label_of = {oid: k for k, ids in labels.items() for oid in ids}
    olds = {m["what"]: m["from"] for m in staged["moves"]}
    regions, controls, gone_found, gone_false, unplaced = [], [], {}, [], []
    for name, a, b in run.VISITS:
        q, m, out = run.score_visit(allr, run.ts(day, a), run.ts(day, b), cutoff)
        fl = q.floor_sized()
        new = fl & (out["what"] > tw)
        moved = fl & np.isfinite(out["where"]) & (out["where"] > th)
        flagged = np.flatnonzero(new | moved)
        places = {int(i): place_of(q, i, recs) for i in flagged}
        for i in range(q.n):
            pl = places.get(i)
            regions.append({"visit": name, "id": int(q.id[i]), "frame": q.frame[i],
                            "t": float(q.t[i]), "label": label_of.get(int(q.id[i])),
                            "floor": bool(fl[i]), "pose": q.source[i],
                            "what": round(float(out["what"][i]), 4),
                            "where": (None if not np.isfinite(out["where"][i])
                                      else round(float(out["where"][i]), 4)),
                            "new": bool(new[i]), "out_of_place": bool(moved[i]),
                            "place": None if pl is None else [round(pl[0], 2), round(pl[1], 2), pl[2]],
                            "place_room": None if pl is None else load.room_of(pl[0], pl[1])})
        for i in flagged:
            if places[int(i)] is None:
                unplaced.append(int(q.id[i]))
        for room in ("corridor", "bedroom"):
            rows = [i for i in flagged if places[int(i)] is not None
                    and load.room_of(places[int(i)][0], places[int(i)][1]) == room]
            groups = run.persistent_groups(q, rows)
            controls.append({"visit": name, "room": room, "flagged_regions": len(rows),
                             "persistent_flags": [[int(q.id[i]) for i in g] for g in groups]})
        for row, (score, n) in out["visit_gone"].items():
            if n < 2 or score <= tg:
                continue
            hit = [w for w, place in olds.items() if run.near_place(m, row, place)]
            entry = {"visit": name, "memory_id": int(m.id[row]), "score": round(score, 4),
                     "looks": n}
            for w in hit:
                gone_found.setdefault(w, []).append(entry)
            point_room = (load.room_of(m.px[row], m.py[row]) if m.ranged[row] else m.room[row])
            if not hit and point_room in ("corridor", "bedroom"):
                gone_false.append(entry)
    found = {}
    for k in labels:
        rows = [r for r in regions if r["label"] == k]
        fl = [r for r in rows if r["floor"] and (r["new"] or r["out_of_place"])]
        early = [r for r in fl if r["t"] < late]
        found[k] = {"labelled_in_windows": len(rows),
                    "floor_sized": sum(r["floor"] for r in rows),
                    "with_where": sum(r["floor"] and r["where"] is not None for r in rows),
                    "with_where_recorded": sum(r["floor"] and r["where"] is not None
                                               and r["pose"] == "recorded" for r in rows),
                    "flagged_looks": len({r["frame"] for r in fl}),
                    "by_what": len({r["frame"] for r in fl if r["new"]}),
                    "by_where": len({r["frame"] for r in fl if r["out_of_place"]}),
                    "found": len({r["frame"] for r in fl}) >= 2,
                    "found_before_late": len({r["frame"] for r in early}) >= 2,
                    "found_by_where_alone": len({r["frame"] for r in fl if r["out_of_place"]}) >= 2}
    for w in olds:
        found[f"old place: {w}"] = {"gone_flags": gone_found.get(w, []),
                                    "found": bool(gone_found.get(w))}
    arrivals = sum(found[k]["found"] for k in labels)
    persistent = sum(len(c["persistent_flags"]) for c in controls) + len(gone_false)
    res = {"thresholds": {"what": tw, "where": th, "gone": tg}, "found": found,
           "arrivals_found": arrivals,
           "arrivals_found_before_late": sum(found[k]["found_before_late"] for k in labels),
           "old_places_found": sum(found[f"old place: {w}"]["found"] for w in olds),
           "room_controls": controls, "gone_false_alarms": gone_false,
           "unplaced_flags": unplaced, "persistent_false_alarms": persistent,
           "shows_promise": arrivals >= 3 and persistent <= 1, "regions": regions}
    json.dump(res, open(os.path.join(HERE, "step2_results.json"), "w"), indent=1)
    return res


if __name__ == "__main__":
    recs = recordings(sys.argv[1])
    dev = development(recs)
    print(json.dumps(dev, indent=1))
    res = test(recs, dev)
    print(json.dumps({k: v for k, v in res.items() if k != "regions"}, indent=1))
