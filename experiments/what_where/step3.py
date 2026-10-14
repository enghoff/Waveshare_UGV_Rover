"""Step 3's events (events.py) over the controls and scene 1 of 2026-10-11 and
the shakedown of 2026-10-12.

    python step3.py RECORDINGS_DIR [OUT.json]

Run from the repository root. RECORDINGS_DIR holds bag_extract.py's arrays of
those recordings and the shakedown's nav_grid reply (shakedown-1012-grid.json),
which gates scans against the map as the deployed watch does.
"""
import glob
import json
import os
import sys

import numpy as np

import events
import load
import recorded
import run

#: The windows judged, each with the memory cutoff it is compared against.
WINDOWS = [("control " + str(i + 1), "2026-10-11", a, b, a)
           for i, (a, b) in enumerate(run.CONTROLS)] + [
    ("scene 1", "2026-10-11", "14:12", "14:42", "13:08"),
    ("shakedown run", "2026-10-12", "07:52:15", "08:00:58", run.CUTOFF),
    ("shakedown after refit", "2026-10-12", "08:11:00", "08:21:00", run.CUTOFF)]
#: Navigation's pose was wrong here (the shakedown's entry); its scans are not fed.
BAD_POSE = ("2026-10-12", "08:00:58", "08:11:00")
#: The what threshold of step 1, from the controls.
T_WHAT = 0.3216


def main(folder, out=None):
    recs = [recorded.Recording(p) for p in sorted(glob.glob(os.path.join(folder, "*.npz")))
            if "watch" not in os.path.basename(p)]
    skip = [(run.ts(BAD_POSE[0], BAD_POSE[1]), run.ts(BAD_POSE[0], BAD_POSE[2]))]
    logged = events.replay(recs, os.path.join(folder, "shakedown-1012-grid.json"), skip)
    report = []
    regions = {}
    for name, day, a, b, cut in WINDOWS:
        if day not in regions:
            regions[day] = load.load(run.DB, 1.7e9, run.ts(day, "23:59"), recordings=recs)
        allr = regions[day]
        A, B, C = run.ts(day, a), run.ts(day, b), run.ts(day, cut)
        memory = allr.subset(allr.t < C)
        query = allr.subset((allr.t >= A) & (allr.t < B))
        changes = [dict(c) for c in logged if A <= c["logged_at"] <= B]
        evs, changes = events.build(changes, query, memory, recs, T_WHAT,
                                    person=events.people(run.DB, query))
        report.append({"window": name, "changes": [{k: c[k] for k in
                       ("kind", "x_m", "y_m", "extent_m", "large", "logged_at")} for c in changes],
                       "events": evs})
        print(f"== {name}: watch logged {sum(c['large'] for c in changes)} large,"
              f" {sum(not c['large'] for c in changes)} small")
        for e in evs:
            print("   ", e["kind"], e.get("from", ""), (e["x_m"], e["y_m"]),
                  e.get("extent_m", ""), e.get("by", ""), e.get("cos", ""),
                  "regions", len(e.get("regions", [])))
    if out:
        json.dump(report, open(out, "w"), indent=1)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
