"""When each move would have been known on the rover: step 3's events, built
the way the shadow runner builds them (experiments/what_where/shadow.py), once a
minute over a session's arms, each dated by the minute it first appeared.

    python timeline.py SESSION.json STAGED.json RECORDING.npz WORLD.db GRID.json
        (--watch WATCH_AFTER_WAIT.npz | --history REC.npz ...) OUT.json

Arguments as experiments/what_where/score3.py takes them. Every EVERY seconds
from the first arm's start to the last arm's end, the change watch's latest
report is the changes, memory is every region before the earliest of them began
(less events.ATTACH_S) or before WINDOW_S, and the query every region since. An
event is new when none of the same kind lies within SAME_EVENT_M of it already.
Writes the new events with `known_at`, the minute they were first built.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WW = os.path.join(HERE, "..", "what_where")
sys.path.insert(0, WW)
sys.path.insert(0, os.path.join(HERE, "..", "m5_changes"))
import events  # noqa: E402
import load  # noqa: E402
import recorded  # noqa: E402
import score3  # noqa: E402
import score_session as m5  # noqa: E402
from arm_replay import grid_of  # noqa: E402

cw = events.cw
EVERY = 60.0
WINDOW_S = 900.0
SAME_EVENT_M = 0.3


def build(session, staged, recording, world, grid_json, watch_npz=None, history=()):
    record = json.load(open(session))
    staged = json.load(open(staged))
    steps = m5.windows(record["entries"])
    names = [w["step"] for w in steps]
    staged_in = staged.get("staged_in", "wait")
    cutoff = steps[names.index(staged_in)]["from"]
    rec = recorded.Recording(recording)
    grid = grid_of(json.load(open(grid_json)))
    if watch_npz:
        watch, _ = cw.ChangeWatch.load(watch_npz)
    else:
        watch = cw.ChangeWatch(-24.0, -22.0, 400, 360)
        watch.set_map(*grid)
        for h in sorted((recorded.Recording(x) for x in history), key=lambda r: r.start):
            score3.feed(watch, h, -np.inf, min(cutoff, h.end))
        score3.feed(watch, rec, -np.inf, cutoff)
    watch.set_map(*grid)
    arms = [w for w in steps[names.index(staged_in) + 1:] if w["step"] in m5.STEPS]
    if not arms:
        return []
    log, reports = [], []
    score3.feed(watch, rec, arms[0]["from"], arms[-1]["to"], log, reports)
    regions = load.load(world, 1.7e9, arms[-1]["to"] + 1.0, recordings=[rec])
    out, seen = [], []
    t = arms[0]["from"] + EVERY
    while t <= arms[-1]["to"] + EVERY:
        latest = [now for when, now in reports if when <= t]
        changes = [dict(c, logged_at=t) for c in (latest[-1] if latest else [])]
        starts = [c["seen_from"] for c in changes if c.get("seen_from")]
        since = max((min(starts) - events.ATTACH_S) if starts else t - WINDOW_S, t - WINDOW_S)
        query = regions.subset((regions.t >= since) & (regions.t <= t))
        if query.n:
            memory = regions.subset(regions.t < since)
            evs, _ = events.build(changes, query, memory, [rec], score3.T_WHAT,
                                  person=events.people(world, query))
            for e in evs:
                if any(k == e["kind"] and math.hypot(x - e["x_m"], y - e["y_m"]) <= SAME_EVENT_M
                       for k, x, y in seen):
                    continue
                seen.append((e["kind"], e["x_m"], e["y_m"]))
                out.append(dict(e, known_at=t, session=os.path.basename(session)))
        t += EVERY
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    for name in ("session", "staged", "recording", "world", "grid"):
        p.add_argument(name)
    p.add_argument("out")
    p.add_argument("--watch")
    p.add_argument("--history", nargs="*", default=[])
    a = p.parse_args()
    evs = build(a.session, a.staged, a.recording, a.world, a.grid, a.watch, a.history)
    for e in evs:
        if e["kind"] == "moved":
            print("moved %s -> %s known %.0f cos %.2f, %d regions, remembered %s" % (
                e["from"], e["to"], e["known_at"], e["cos"], len(e["regions"]), e["remembered"]))
    print(len(evs), "events,", sum(e["kind"] == "moved" for e in evs), "moved")
    json.dump(evs, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
