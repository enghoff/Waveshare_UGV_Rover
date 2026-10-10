"""Today's recordings, in order, through ros_nav/change_watch.py as the rover would.

Each recording's scans are fed at their own times, so the gaps between runs are
the real ones and the visits fall where they would have. After each recording,
the changes the watch would report.

    python replay_watch.py A.npz B.npz ... [--every 3]
"""
import math, os, sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "ros_nav"))
sys.path.insert(0, HERE)
import change_watch  # noqa: E402
from change_detect import compose, interp_pose, last_pose  # noqa: E402

EVERY = int(sys.argv[sys.argv.index("--every") + 1]) if "--every" in sys.argv else 3
MAX_TURN_DPS = 40.0
paths = [p for p in sys.argv[1:] if p.endswith(".npz")]
watch = change_watch.ChangeWatch(-24.0, -22.0, 400, 360)
for path in paths:
    d = np.load(path)
    ranges, meta = d["ranges"], d["scan_meta"]
    odom = d["odom_tf"][np.argsort(d["odom_tf"][:, 0])]
    mapt = d["map_tf"][np.argsort(d["map_tf"][:, 0])]
    prev = None
    fed = 0
    for k in range(0, len(ranges), EVERY):
        stamp, _recv, amin, ainc, _tinc, stime = meta[k]
        t = stamp  # the sweep's start, which the deskewed scan is seen from
        ob = interp_pose(odom, t)
        mo = last_pose(mapt, t)
        if ob is None or mo is None:
            continue
        if prev is not None and t > prev[0]:
            rate = abs(math.degrees(math.atan2(math.sin(ob[2] - prev[1]),
                                               math.cos(ob[2] - prev[1])))) / (t - prev[0])
            prev = (t, ob[2])
            if rate > MAX_TURN_DPS:
                continue
        prev = (t, ob[2])
        x, y, th = compose(mo, ob)
        r = ranges[k]
        watch.add_scan(x, y, th, r, amin + ainc * np.arange(len(r)), t)
        fed += 1
    print(f"=== after {os.path.basename(path)}: {fed} scans")
    for c in watch.changes():
        print("   ", {k: c[k] for k in ("kind", "x_m", "y_m", "extent_m", "large")},
              f"seen {c['seen_until'] - c['seen_from']:.0f} s,"
              f" otherwise {c['seen_from'] - c['was_otherwise_at']:.0f} s before")
