"""What one arm of a session found, judged on its own scans: the change watch as
it stood after the wait, fed only that arm's part of the recording, the way
`ros_nav/change_node.py` feeds it. The benchmark's way of keeping the second arm
from inheriting what the first saw
(docs/decisions/m5-sessions-are-counted-before-they-start.md).

    python arm_replay.py WATCH.npz RECORDING.npz GRID.json FROM TO

WATCH.npz is the watch's tallies (`session.py` keeps them after the wait as
NAME-watch-after-wait.npz); RECORDING.npz is the session's recording as
`bag_extract.py` writes it; GRID.json is the daemon's `nav_grid` reply the map
gate judges scans against (NAME-grid.json); FROM and TO are seconds since the
epoch, the arm's step as the session record has it.

What it found is what the watch would have logged in the window and was not in
its report when the window opened. Navigation's own trust in the pose, which the
node also waits on, is not in a recording and is taken as given.
"""
import base64
import json
import math
import os
import sys
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
# The watch from the repository beside this, or the one deployed on the rover
# when this has been copied there on its own with change_detect.py.
for _where in (os.path.join(HERE, "..", "..", "ros_nav"),
               os.path.expanduser("~/ugv/ros_nav")):
    if os.path.exists(os.path.join(_where, "change_watch.py")):
        sys.path.insert(0, _where)
        break
import change_watch as cw  # noqa: E402
from change_detect import compose, interp_pose, last_pose  # noqa: E402

#: As change_node feeds the watch: every third scan, none turning faster.
EVERY = 3
MAX_TURN_DPS = 40.0
REPORT_EVERY_S = 30.0
OCCUPIED = 65


def grid_of(reply):
    """The occupied cells and their corner, from a `nav_grid` reply."""
    cells = np.frombuffer(zlib.decompress(base64.b64decode(reply["data"])), dtype=np.uint8)
    cells = cells.reshape(reply["height"], reply["width"]).astype(np.int16)
    cells[cells == 255] = -1
    return (cells >= OCCUPIED), reply["origin_x_m"], reply["origin_y_m"], reply["resolution_m"]


def replay(watch, recording, t_from, t_to):
    """(found, report at the end): what the watch logs over the window and was
    not already reporting when it opened."""
    meta, ranges = recording["scan_meta"], recording["ranges"]
    odom = recording["odom_tf"][np.argsort(recording["odom_tf"][:, 0])]
    mapt = recording["map_tf"][np.argsort(recording["map_tf"][:, 0])]
    previous = watch.changes()
    logged, found = [], []
    last_report, last_yaw, seen = t_from, None, 0
    for k in range(len(ranges)):
        t = float(meta[k][0])
        if not t_from <= t <= t_to:
            continue
        seen += 1
        if t - last_report >= REPORT_EVERY_S:
            last_report = t
            now = watch.changes()
            for group in cw.ChangeWatch.news(now, previous, logged, t):
                logged.append(dict(group, logged_at=t))
                found.append(dict(group, logged_at=t))
            previous = now
        if seen % EVERY:
            continue
        ob = interp_pose(odom, t)
        mo = last_pose(mapt, t)
        if ob is None or mo is None:
            continue
        x, y, yaw = compose(mo, ob)
        if last_yaw is not None:
            then, was = last_yaw
            gap = t - then
            if gap > 0 and abs(math.degrees(math.atan2(
                    math.sin(yaw - was), math.cos(yaw - was)))) / gap > MAX_TURN_DPS:
                last_yaw = (t, yaw)
                continue
        last_yaw = (t, yaw)
        r = ranges[k]
        watch.add_scan(x, y, yaw, r, meta[k][2] + meta[k][3] * np.arange(len(r)), t)
    return found, watch.changes()


def main():
    if len(sys.argv) != 6:
        raise SystemExit(__doc__)
    watch, _ = cw.ChangeWatch.load(sys.argv[1])
    if watch is None:
        raise SystemExit("no watch tallies at %s" % sys.argv[1])
    with open(sys.argv[3]) as f:
        watch.set_map(*grid_of(json.load(f)))
    found, _ = replay(watch, np.load(sys.argv[2]), float(sys.argv[4]), float(sys.argv[5]))
    for one in found:
        print("%-4s (%6.2f, %6.2f) %.2f m%s" % (one["kind"], one["x_m"], one["y_m"],
                                                one["extent_m"],
                                                "  furniture" if one["large"] else ""))
    print(json.dumps([one for one in found if one["large"]]))


if __name__ == "__main__":
    main()
