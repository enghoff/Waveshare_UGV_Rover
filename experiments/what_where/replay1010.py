"""Step 3 version 2 on the recordings of 2026-10-10, which it was not developed
on (replay1010.md).

    python replay1010.py RECORDINGS_1010_DIR GRID.json OUT_DIR

Run from the repository root with tmp/codex-m5-audit/world.db present. Builds,
for two replays, a recording joined from the day's extracts, a session record
in session.py's shape and a staging file, and scores each with score3.py:

- controls: sessions 15-16 as the visit before, sessions 17 and 18-19 after,
  nothing staged; only false alarms are asked;
- return: sessions 15-16 and 17 as history, 18-19 as the visit before,
  session 20 after; the armchair's return from (-18.4, -15.4) to
  (-17.18, -16.73), as docs/progress/2026-10-10-m5-what-reports-a-change.md
  measured it.
"""
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import recorded  # noqa: E402

DB = os.path.join("tmp", "codex-m5-audit", "world.db")
PATH_STEP_M = 0.5


def joined(paths, out):
    """One recording from several extracts, oldest first."""
    parts = [np.load(p) for p in paths]
    width = max(p["ranges"].shape[1] for p in parts)
    ranges = np.concatenate([np.pad(p["ranges"], ((0, 0), (0, width - p["ranges"].shape[1])),
                                    constant_values=np.nan) for p in parts])
    out_d = {"ranges": ranges}
    for key in ("scan_meta", "odom_tf", "map_tf", "odom"):
        out_d[key] = np.concatenate([p[key] for p in parts])
    np.savez_compressed(out, **out_d)


def path_of(rec_path):
    """Where a recording's rover went, a point every PATH_STEP_M."""
    rec = recorded.Recording(rec_path)
    path, last = [], None
    for t in rec.scan_t[::10]:
        p = rec.pose(t)
        if p is None:
            continue
        if last is None or np.hypot(p[0] - last[0], p[1] - last[1]) >= PATH_STEP_M:
            path.append([round(p[0], 2), round(p[1], 2)])
            last = p
    return rec.start, rec.end, path


def record(steps):
    """session.py's record from (name, start, end, path) in order."""
    entries = [{"step": "start", "at": steps[0][1] - 1.0}]
    for name, a, b, path in steps:
        entries.append({"step": name, "at": a, "begin": True})
        entries.append({"step": name, "at": b, "end": True, "path": path, "report": {"changes": []}})
    return {"name": "replay-1010", "entries": entries}


def main(folder, grid, out):
    os.makedirs(out, exist_ok=True)
    rec = {n: os.path.join(folder, n + ".npz") for n in ("s15-16", "s17", "s18-19", "s20")}
    span = {n: path_of(p) for n, p in rec.items()}
    nothing = {"staged_in": "wait", "moves": [], "added": [], "removed": [], "doors": [],
               "rooms": [], "reset_rooms": [], "also_moved": []}
    back = dict(nothing, moves=[{"what": "grey armchair, back to its place",
                                 "from": [-18.4, -15.4], "to": [-17.18, -16.73]}],
                rooms=["living"])
    plans = {
        "controls": {"history": [], "join": ["s15-16", "s17", "s18-19"], "staged": nothing,
                     "steps": [("loop A", "s15-16"), ("run", "s17"), ("loop B", "s18-19")]},
        "return": {"history": ["s15-16", "s17"], "join": ["s18-19", "s20"], "staged": back,
                   "steps": [("loop A", "s18-19"), ("run", "s20")]},
    }
    for name, plan in plans.items():
        joined_path = os.path.join(out, name + "-recording.npz")
        joined([rec[n] for n in plan["join"]], joined_path)
        # The visit before, the wait the staging falls in, then the visits after.
        steps = [(step,) + span[n] for step, n in plan["steps"]]
        steps = [steps[0], ("wait", steps[0][2], steps[1][1], [])] + steps[1:]
        rec_path = os.path.join(out, name + "-record.json")
        json.dump(record(steps), open(rec_path, "w"), indent=1)
        staged_path = os.path.join(out, name + "-staged.json")
        json.dump(plan["staged"], open(staged_path, "w"), indent=1)
        cmd = [sys.executable, os.path.join(HERE, "score3.py"), rec_path, staged_path, joined_path,
               DB, grid, "--history"] + [rec[n] for n in plan["history"]] + \
              ["--out", os.path.join(out, name + "-score3.json")]
        print("=== " + name, flush=True)
        subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main(*sys.argv[1:4])
