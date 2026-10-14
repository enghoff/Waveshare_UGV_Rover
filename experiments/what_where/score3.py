"""Step 3's test, scored: the watch alone and step 3 on the same scans of one
staged M5 session, by M5's counting rules (experiments/m5_changes/score_session.py)
and step3.md.

    python score3.py SESSION.json STAGED.json RECORDING.npz WORLD.db GRID.json
                     (--watch WATCH_AFTER_WAIT.npz | --history REC.npz ...) [--out OUT.json]

SESSION.json is session.py's record, RECORDING.npz the session's recording as
bag_extract.py writes it, WORLD.db a SQLite copy of the world store taken after
the session, GRID.json the session's nav_grid reply. The watch's state when the
staging began is the one session.py saved after the wait (--watch); --history
rebuilds one from recordings instead, for a dry run on a session that did not
keep it. Each arm after the staging is replayed on its own from that state, as
arm_replay.py does, with a last report at the arm's end.
"""
import argparse
import copy
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "m5_changes"))
import events  # noqa: E402
import load  # noqa: E402
import recorded  # noqa: E402
import score_session as m5  # noqa: E402
from arm_replay import grid_of  # noqa: E402
from change_detect import compose, interp_pose, last_pose  # noqa: E402

cw = events.cw
#: The what threshold of step 1, from the controls of 2026-10-11.
T_WHAT = 0.3216


def feed(watch, rec, t_from, t_to, log=None):
    """Scans of rec in [t_from, t_to] into watch, as change_node feeds it; with
    log (a list), report every REPORT_EVERY_S and once at t_to, logging news."""
    previous = watch.changes() if log is not None else None
    last_report, received, last_yaw = t_from, 0, None
    for k in range(len(rec.ranges)):
        t = float(rec.meta[k][0])
        if not t_from <= t <= t_to:
            continue
        received += 1
        if log is not None and t - last_report >= events.REPORT_EVERY_S:
            last_report = t
            now = watch.changes()
            for g in cw.ChangeWatch.news(now, previous, log, t):
                log.append(dict(g, logged_at=t))
            previous = now
        if received % events.EVERY:
            continue
        ob, mo = interp_pose(rec.odom, t), last_pose(rec.map, t)
        if ob is None or mo is None:
            continue
        x, y, yaw = compose(mo, ob)
        if last_yaw is not None:
            then, was = last_yaw
            if t > then and abs(math.degrees(math.atan2(math.sin(yaw - was), math.cos(yaw - was)))) \
                    / (t - then) > events.MAX_TURN_DPS:
                last_yaw = (t, yaw)
                continue
        last_yaw = (t, yaw)
        r = rec.ranges[k]
        watch.add_scan(x, y, yaw, r, rec.meta[k][2] + rec.meta[k][3] * np.arange(len(r)), t)
    if log is not None:
        now = watch.changes()
        for g in cw.ChangeWatch.news(now, previous, log, t_to):
            log.append(dict(g, logged_at=t_to))


def as_rows(evs, t_from, t_to, query):
    """Step 3's events as change-log rows: every event reported counts as
    furniture-sized (step 3 reports only what it stands behind); a move is a
    GONE where it left and a NEW where it went."""
    shutter = {int(i): float(t) for i, t in zip(query.id, query.shutter)}
    rows = []
    for e in evs:
        times = [shutter[i] for i in e.get("regions", []) if i in shutter]
        at = min(max(max(times) if times else t_to - 1.0, t_from), t_to)
        base = {"large": True, "extent_m": e.get("extent_m", 0.6), "logged_at": at, "event": e["kind"]}
        if e["kind"] == "moved":
            rows.append(dict(base, kind="GONE", x_m=e["from"][0], y_m=e["from"][1]))
            rows.append(dict(base, kind="NEW", x_m=e["to"][0], y_m=e["to"][1]))
        elif e["kind"] == "gone":
            rows.append(dict(base, kind="GONE", x_m=e["x_m"], y_m=e["y_m"]))
        else:
            rows.append(dict(base, kind="NEW", x_m=e["x_m"], y_m=e["y_m"]))
    return rows


def first_steps(steps, staged):
    """For each staged place, the first loop or run after the staging whose
    stops came within the lidar's reach of it, as M5 counts finds."""
    names = [w["step"] for w in steps]
    after = names.index(staged.get("staged_in", "wait")) + 1
    places = [m[k] for m in staged.get("moves", []) for k in ("from", "to")] +         [a["at"] for a in staged.get("added", [])] +         [m[k] for m in staged.get("also_moved", []) for k in ("from", "to")]
    out = []
    for p in places:
        for w in steps[after:]:
            if w["step"] in m5.STEPS and any(math.hypot(sx - p[0], sy - p[1]) <= m5.SEEN_M
                                             for sx, sy in w["stops"]):
                out.append((p, w["step"]))
                break
    return out


def named(evs, staged, firsts):
    """(right, counted): step 3's events at staged places, in the step that
    counts for that place (first_steps), and how many are named right. A move is right when it left from within FOUND_M of a staged move's
    old place and arrived within FOUND_M of a staged move's new place (which of
    two identical things is not asked); a new thing when within FOUND_M of an
    addition. Arrived and gone name nothing and are never right."""
    near = lambda p, q: math.hypot(p[0] - q[0], p[1] - q[1]) <= m5.FOUND_M
    froms = [m["from"] for m in staged.get("moves", [])]
    tos = [m["to"] for m in staged.get("moves", [])]
    adds = [a["at"] for a in staged.get("added", [])] + \
        [m["to"] for m in staged.get("also_moved", [])]
    places = froms + tos + adds + [m["from"] for m in staged.get("also_moved", [])]
    right = counted = 0
    for e in evs:
        ends = [e["from"], e["to"]] if e["kind"] == "moved" else [[e["x_m"], e["y_m"]]]
        if not any(near(p, q) and e["step"] == step for p in ends for q, step in firsts):
            continue
        counted += 1
        if e["kind"] == "moved":
            right += any(near(e["from"], f) for f in froms) and any(near(e["to"], t) for t in tos)
        elif e["kind"] == "new thing":
            right += any(near([e["x_m"], e["y_m"]], a) for a in adds)
    return right, counted


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("session")
    p.add_argument("staged")
    p.add_argument("recording")
    p.add_argument("world")
    p.add_argument("grid")
    p.add_argument("--watch")
    p.add_argument("--history", nargs="*", default=[])
    p.add_argument("--out")
    a = p.parse_args()
    record = json.load(open(a.session))
    staged = json.load(open(a.staged))
    steps = m5.windows(record["entries"])
    names = [w["step"] for w in steps]
    staged_in = staged.get("staged_in", "wait")
    cutoff = steps[names.index(staged_in)]["from"]
    rec = recorded.Recording(a.recording)
    grid = grid_of(json.load(open(a.grid)))
    if a.watch:
        watch, _ = cw.ChangeWatch.load(a.watch)
    else:
        watch = cw.ChangeWatch(-24.0, -22.0, 400, 360)
        watch.set_map(*grid)
        for h in sorted((recorded.Recording(x) for x in a.history), key=lambda r: r.start):
            feed(watch, h, -np.inf, min(cutoff, h.end))
        feed(watch, rec, -np.inf, cutoff)
    watch.set_map(*grid)
    regions = load.load(a.world, 1.7e9, max(w["to"] for w in steps) + 1.0, recordings=[rec])
    memory = regions.subset(regions.t < cutoff)
    watch_rows, step3_rows, all_events = [], [], []
    for w in steps[names.index(staged_in) + 1:]:
        if w["step"] not in m5.STEPS:
            continue
        arm = copy.deepcopy(watch)
        log = []
        feed(arm, rec, w["from"], w["to"], log)
        watch_rows += log
        query = regions.subset((regions.t >= w["from"]) & (regions.t <= w["to"]))
        evs, _ = events.build([dict(c) for c in log], query, memory, [rec], T_WHAT)
        step3_rows += as_rows(evs, w["from"], w["to"], query)
        all_events += [dict(e, step=w["step"]) for e in evs]
    by_watch = m5.score(record, staged, watch_rows)
    by_step3 = m5.score(record, staged, step3_rows)
    found = lambda s: {(e["what"], e["kind"]) for e in s["events"] if e["outcome"] == "found"}
    alarms = lambda s: sum(len(r["false_alarms"]) for r in s["room_controls"] if r["control"])
    right, counted = named(all_events, staged, first_steps(steps, staged))
    fw, f3 = found(by_watch), found(by_step3)
    verdict = {
        "watch_found": len(fw), "step3_found": len(f3), "events_staged": len(by_watch["events"]),
        "step3_finds_all_the_watch_finds": fw <= f3,
        "step3_finds_more": len(f3 - fw), "named_right": right, "named_counted": counted,
        "watch_false_alarms": alarms(by_watch), "step3_false_alarms": alarms(by_step3)}
    verdict["passes"] = bool(verdict["step3_finds_all_the_watch_finds"]
                             and (verdict["step3_finds_more"] >= 1
                                  or (counted > 0 and right >= 0.8 * counted))
                             and verdict["step3_false_alarms"] <= verdict["watch_false_alarms"] + 1)
    out = {"verdict": verdict, "watch": by_watch, "step3": by_step3, "events": all_events}
    print(json.dumps(verdict, indent=1))
    for e in by_step3["events"]:
        print("  step3 %-28s %-4s %s" % (e["what"][:28], e["kind"], e["outcome"]))
    for e in by_watch["events"]:
        print("  watch %-28s %-4s %s" % (e["what"][:28], e["kind"], e["outcome"]))
    for r in by_step3["room_controls"]:
        if r["control"] and r["false_alarms"]:
            print("  step3 false alarm", r["step"], r["room"], r["false_alarms"])
    for r in by_watch["room_controls"]:
        if r["control"] and r["false_alarms"]:
            print("  watch false alarm", r["step"], r["room"], r["false_alarms"])
    if a.out:
        json.dump(out, open(a.out, "w"), indent=1, default=str)


if __name__ == "__main__":
    main()
