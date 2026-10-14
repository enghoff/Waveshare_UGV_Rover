"""The what/where streams on the October 11 controls (development) and the
shakedown of October 12 (test), by protocol.md.

    python run.py dev     thresholds from the controls -> development.json
    python run.py test    the shakedown, scored -> results.json

Run from the repository root of a checkout that has tmp/codex-m5-audit/world.db.
"""
import datetime
import json
import os
import sys

import numpy as np

import load
import streams

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
DB = os.environ.get("WORLD_DB", os.path.join("tmp", "codex-m5-audit", "world.db"))
LABELS = os.path.join(ROOT, "experiments", "perception_bench", "shakedown_labels.json")
STAGED = os.path.join(ROOT, "experiments", "m5_changes", "staged", "shakedown-1012.json")

#: The unchanged controls of 2026-10-11, start and end in IST, each a visit
#: whose memory is everything before its start.
CONTROLS = [("08:04", "08:20"), ("08:50", "09:00"), ("09:14", "09:25"),
            ("09:33", "09:49"), ("12:47", "12:55"), ("12:55", "12:57"),
            ("13:01", "13:07")]
#: The shakedown: memory frozen at the end of loop A, when the staging began;
#: the run until navigation's pose went wrong, and everything after the refit.
CUTOFF = "07:48:05"
VISITS = [("run", "07:52:15", "08:00:58"), ("after refit", "08:11:00", "08:21:00")]
#: The share of an unchanged room's scores a flag must exceed.
PERCENTILE = 99.0
#: Two flags are one thing when they look alike at or above this and sit in
#: one place: map points within FLAG_M, or, unranged, cameras within FLAG_M and
#: bearings within streams.BEARING_DEG.
FLAG_ALIKE = 0.5
FLAG_M = 1.0
#: A staged old place is found gone when a gone region's map point lies within
#: this of it, or its unranged sight line passes within ROW_CROSS_M of it.
PLACE_M = 1.0
ROW_CROSS_M = 0.5


def ts(day, hm):
    fmt = "%Y-%m-%d %H:%M:%S" if hm.count(":") == 2 else "%Y-%m-%d %H:%M"
    return datetime.datetime.strptime(f"{day} {hm}", fmt).timestamp()


def score_visit(allr, start, end, cutoff):
    memory = allr.subset(allr.t < cutoff)
    query = allr.subset((allr.t >= start) & (allr.t < end))
    out = streams.run(query, memory)
    looks = streams.Looks(query)
    out["visit_gone"] = streams.gone_over_visit(out["gone"], looks.frames)
    return query, memory, out


def development():
    day = "2026-10-11"
    allr = load.load(DB, 1.7e9, ts(day, "13:08"))
    what, where, gone, per = [], [], [], []
    for a, b in CONTROLS:
        q, m, out = score_visit(allr, ts(day, a), ts(day, b), ts(day, a))
        fl = q.floor_sized()
        what.append(out["what"][fl])
        where.append(out["where"][fl & np.isfinite(out["where"])])
        g = [s for s, n in out["visit_gone"].values() if n >= 2]
        gone += g
        per.append({"control": f"{a}-{b}", "regions": int(fl.sum()),
                    "with_references": int((fl & np.isfinite(out["where"])).sum()),
                    "expected_twice": len(g)})
    what, where, gone = np.concatenate(what), np.concatenate(where), np.array(gone)
    dev = {"percentile": PERCENTILE,
           "T_what": float(np.percentile(what, PERCENTILE)),
           "T_where": float(np.percentile(where, PERCENTILE)),
           "T_gone": float(np.percentile(gone, PERCENTILE)),
           "n_what": len(what), "n_where": len(where), "n_gone": len(gone),
           "controls": per,
           "parameters": {k: getattr(streams, k) for k in
                          ("REF_M", "REF_DEG", "REF_LOOKS", "BEARING_DEG", "RANGE_M",
                           "VIEW_HALF_DEG", "VIEW_MAX_M", "SAME_THING")}}
    for name, v in (("what", what), ("where", where), ("gone", gone)):
        dev[f"{name}_percentiles_50_90_95_99"] = [round(float(x), 4) for x in
                                                  np.percentile(v, [50, 90, 95, 99])]
    json.dump(dev, open(os.path.join(HERE, "development.json"), "w"), indent=1)
    print(json.dumps(dev, indent=1))


def _same_place(q, i, j):
    if q.ranged[i] and q.ranged[j]:
        return np.hypot(q.px[i] - q.px[j], q.py[i] - q.py[j]) <= FLAG_M
    return (np.hypot(q.cx[i] - q.cx[j], q.cy[i] - q.cy[j]) <= FLAG_M and
            abs(streams._wrap(q.bearing[i] - q.bearing[j])) <= streams.BEARING_DEG)


def persistent_groups(q, rows):
    """Flagged rows grouped into things; a group seen in two or more looks."""
    groups = []
    for i in rows:
        for g in groups:
            if any(q.vec[i] @ q.vec[j] >= FLAG_ALIKE and _same_place(q, i, j) for j in g):
                g.append(i)
                break
        else:
            groups.append([i])
    return [g for g in groups if len(set(q.frame[g])) >= 2]


def near_place(memory, row, place):
    """Is a remembered region at a staged place?"""
    if memory.ranged[row]:
        return np.hypot(memory.px[row] - place[0], memory.py[row] - place[1]) <= PLACE_M
    b = np.radians(memory.bearing[row])
    dx, dy = place[0] - memory.cx[row], place[1] - memory.cy[row]
    along = dx * np.cos(b) + dy * np.sin(b)
    cross = abs(-dx * np.sin(b) + dy * np.cos(b))
    return 0.3 <= along <= 6.0 and cross <= ROW_CROSS_M


def test():
    dev = json.load(open(os.path.join(HERE, "development.json")))
    tw, th, tg = dev["T_what"], dev["T_where"], dev["T_gone"]
    day = "2026-10-12"
    labels = json.load(open(LABELS))["subjects"]
    staged = json.load(open(STAGED))
    allr = load.load(DB, 1.7e9, ts(day, "08:21:00"))
    cutoff = ts(day, CUTOFF)
    label_of = {oid: k for k, ids in labels.items() for oid in ids}
    things = {k: [] for k in labels}
    controls, gone_found, gone_false, coverage = [], {}, [], {}
    olds = {m["what"]: m["from"] for m in staged["moves"]}
    region_rows = []
    for name, a, b in VISITS:
        q, m, out = score_visit(allr, ts(day, a), ts(day, b), cutoff)
        fl = q.floor_sized()
        new = fl & (out["what"] > tw)
        moved = fl & np.isfinite(out["where"]) & (out["where"] > th)
        for i in range(q.n):
            k = label_of.get(int(q.id[i]))
            room = q.room[i]
            region_rows.append({"visit": name, "id": int(q.id[i]), "frame": q.frame[i],
                                "label": k, "room": room, "floor": bool(fl[i]),
                                "posed": bool(q.posed[i]),
                                "what": round(float(out["what"][i]), 4),
                                "where": (None if not np.isfinite(out["where"][i])
                                          else round(float(out["where"][i]), 4)),
                                "new": bool(new[i]), "out_of_place": bool(moved[i])})
            if k is not None:
                things[k].append(region_rows[-1])
        # Room-controls: corridor and bedroom looks.
        for room in ("corridor", "bedroom"):
            in_room = q.room == room
            flagged = np.flatnonzero(in_room & (new | moved))
            groups = persistent_groups(q, flagged)
            controls.append({"visit": name, "room": room,
                             "looks": int(len(set(q.frame[in_room]))),
                             "regions": int((in_room & fl).sum()),
                             "flagged_regions": int(len(flagged)),
                             "persistent_flags": [[int(q.id[i]) for i in g] for g in groups]})
        # Gone: per visit, expected in two or more looks and matched in none.
        for row, (score, n) in out["visit_gone"].items():
            if n < 2 or score <= tg:
                continue
            hit = [w for w, place in olds.items() if near_place(m, row, place)]
            entry = {"visit": name, "memory_id": int(m.id[row]), "score": round(score, 4),
                     "looks": n, "camera_room": m.room[row],
                     "point": (None if not m.ranged[row] else
                               [round(float(m.px[row]), 2), round(float(m.py[row]), 2)])}
            for w in hit:
                gone_found.setdefault(w, []).append(entry)
            point_room = (load.room_of(m.px[row], m.py[row]) if m.ranged[row] else m.room[row])
            if not hit and point_room in ("corridor", "bedroom"):
                gone_false.append(entry)
    found = {}
    for k, rows in things.items():
        flagged = [r for r in rows if r["floor"] and (r["new"] or r["out_of_place"])]
        found[k] = {"labelled_in_windows": len(rows),
                    "floor_sized": sum(r["floor"] for r in rows),
                    "with_where": sum(r["where"] is not None and r["floor"] for r in rows),
                    "flagged_looks": len({r["frame"] for r in flagged}),
                    "by_what": len({r["frame"] for r in flagged if r["new"]}),
                    "by_where": len({r["frame"] for r in flagged if r["out_of_place"]}),
                    "found": len({r["frame"] for r in flagged}) >= 2}
    for w in olds:
        found[f"old place: {w}"] = {"gone_flags": gone_found.get(w, []),
                                    "found": bool(gone_found.get(w))}
    arrivals = sum(found[k]["found"] for k in labels)
    persistent = sum(len(c["persistent_flags"]) for c in controls) + len(gone_false)
    res = {"thresholds": {"what": tw, "where": th, "gone": tg},
           "found": found, "arrivals_found": arrivals,
           "old_places_found": sum(found[f"old place: {w}"]["found"] for w in olds),
           "room_controls": controls, "gone_false_alarms": gone_false,
           "persistent_false_alarms": persistent,
           "shows_promise": arrivals >= 3 and persistent <= 1,
           "regions": region_rows}
    json.dump(res, open(os.path.join(HERE, "results.json"), "w"), indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != "regions"}, indent=1))


if __name__ == "__main__":
    {"dev": development, "test": test}[sys.argv[1]]()
