"""The first-principles plan's first test on recordings: localising on stable
cells, judged as declared on 2026-10-12 (docs/plans/perception-from-first-
principles.md, "What could be tried on the recordings now", 1).

    python shakedown_test.py REC.npz GRID.json BASE.json STABLE.json LIVE_TIMELINE.txt

BASE is the deployed setting's replay of the shakedown from its start graph,
STABLE the stable-cell replay, both as graph_replay_tool.py writes them. Scored:

1. over the stationary stretch 08:02:30-08:11, where the rover stood stuck and
   the owner's refit is the reference, within 0.3 m and 3 degrees of the
   refit's answer (-19.42, -13.71, 140);
2. within 0.15 m and 3 degrees of the reference poses (the best fit within
   0.5 m and 25 degrees, pose_timeline.py) at least as often as the deployed
   setting, outside the stationary stretch;
3. on the drive home (08:17:44-08:18:00), no further from the best fit than
   the deployed setting;
4. no correction over 0.15 m in one step that neither the live mapper nor the
   deployed setting made.
The replay of 2026-10-12 started from a graph saved at 08:00:11, which is gone;
both replays here start from the session's start graph.
"""
import json
import math
import subprocess
import sys
import time

import numpy as np

REFIT = (-19.42, -13.71, 140.0)
DAY = (2026, 10, 12)


def clock(hms):
    h, m, s = (int(v) for v in hms.split(":"))
    return time.mktime(DAY + (h, m, s, 0, 0, -1))


def timeline(rec, grid, tf):
    out = subprocess.run([sys.executable, "pose_timeline.py", rec, grid, "--every", "10", "--map-tf", tf],
                         capture_output=True, text=True).stdout
    rows = []
    for line in out.splitlines():
        parts = line.replace("(", " ").replace(")", " ").replace(",", " ").split()
        if len(parts) >= 9 and ":" in parts[0]:
            try:
                rows.append((parts[0], float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]),
                             float(parts[5]), float(parts[6]), float(parts[8])))
            except ValueError:
                pass
    return rows


def load_tf(path):
    return np.array(json.load(open(path)))


def jumps(tf, threshold=0.15):
    """Times at which map -> odom moved more than threshold between steps."""
    d = np.hypot(np.diff(tf[:, 1]), np.diff(tf[:, 2]))
    return [(float(tf[i + 1, 0]), float(d[i])) for i in np.flatnonzero(d > threshold)]


def main():
    rec, grid, base, stable, live = sys.argv[1:6]
    tl = {"deployed": timeline(rec, grid, base), "stable": timeline(rec, grid, stable)}
    lo, hi = "08:02:30", "08:11:00"
    for name, rows in tl.items():
        st = [r for r in rows if lo <= r[0] <= hi]
        d = [math.hypot(r[1] - REFIT[0], r[2] - REFIT[1]) for r in st]
        h = [abs((r[3] - REFIT[2] + 180) % 360 - 180) for r in st]
        out = [r for r in rows if not (lo <= r[0] <= hi) and r[4] > 0]
        good = np.mean([(r[6] <= 0.15 and abs(r[7]) <= 3.0) for r in out]) if out else 0
        home = [r for r in rows if "08:17:44" <= r[0] <= "08:18:00"]
        print("%-9s stationary: %.2f-%.2f m, %.1f-%.1f deg from the refit | reference within 0.15 m: %.0f%% of %d"
              " | home: %s" % (name, min(d), max(d), min(h), max(h), 100 * good, len(out),
                               ", ".join("%.2f" % r[6] for r in home)))
    jb, js = jumps(load_tf(base)), jumps(load_tf(stable))
    m = np.load(rec)["map_tf"]
    m = m[np.argsort(m[:, 0])]
    jl = jumps(np.column_stack([m[:, 0], m[:, 2], m[:, 3]]))
    print("steps over 0.15 m: live %d, deployed %d, stable %d" % (len(jl), len(jb), len(js)))
    near = lambda t, lst: any(abs(t - u) < 2.0 for u, _ in lst)
    only = [(time.strftime("%H:%M:%S", time.localtime(t)), round(d, 2)) for t, d in js
            if not near(t, jb) and not near(t, jl)]
    print("stable's steps the deployed setting did not make:", only[:20])


if __name__ == "__main__":
    main()
