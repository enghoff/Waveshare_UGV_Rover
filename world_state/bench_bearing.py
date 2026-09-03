#!/usr/bin/env python3
"""What the world state's pixels-times-a-gain bearing cost, on real boxes.

    python world_state/bench_bearing.py /tmp/run.db
    ssh orin 'cd ~/ugv/world_state && python3 bench_bearing.py'

**Past tense since 2026-09-03: `view` works bearings out through the fitted lens
now, and this is the measurement that made the case.** `view._from_box` used to
map a box's horizontal centre to an angle with one multiplication and to throw
the gimbal's tilt away. `face_tracking/lens.py` holds a lens that was swept and
fitted on this rover, and `aiming.py` had been moved off exactly that
multiplication a fortnight earlier, on 2026-08-19, because it is only right along
the two centre lines.

So the retired formula is the one line kept here, and what it is compared against
is imported rather than copied -- there is one description of this camera's
optics and it is not in this file. Run it on a recording taken before the change
to see what those bearings were worth; run it on one taken after and the answer
is what the store already holds.
"""
from __future__ import annotations

import argparse
import json
import os
import sqlite3
import sys

#: `lens.py` sits under `face_tracking/` in the checkout and is flattened
#: straight into `~/ugv/` once deployed, so the directory above this one is where
#: it lives on the rover and `face_tracking/` beside it is where it lives here.
#: Both go on the path; whichever exists wins.
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "face_tracking"))

from world_state.view import azimuth_deg     # noqa: E402

parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
parser.add_argument("database", nargs="?",
                    default=os.path.expanduser("~/.ugv/world/world.db"),
                    help="a world.db the rover wrote")
parser.add_argument("--fov", type=float, default=130.0,
                    help="what the daemon tells the store the camera sees")
parser.add_argument("--size", default="640x480", help="the frames' own size")
args = parser.parse_args()
W, H = (int(part) for part in args.size.lower().split("x"))
FOV = args.fov
SIZE = (W, H)

def retired(box):
    """The angle the old model gave a box, and the cone it called its width.

    One multiplication: this far across the picture is that many degrees round.
    Kept because it is what is being measured, and kept *here* because nothing
    should be able to reach it from the rover.
    """
    left, _top, right, _bottom = (float(value) for value in box)
    centre = (left + right) / 2.0
    return (0.5 - centre) * FOV, abs(right - left) * FOV

db = sqlite3.connect(f"file:{args.database}?mode=ro", uri=True)
rows = db.execute("SELECT bbox_json, observer_tilt_deg FROM observations").fetchall()
worst, errs = None, []
for bbox_json, tilt in rows:
    box = json.loads(bbox_json)
    linear, _span = retired(box)
    cx, cy = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
    fitted = azimuth_deg(cx, cy, tilt or 0.0, SIZE)
    err = abs(linear - fitted)
    errs.append(err)
    if worst is None or err > worst[0]:
        worst = (err, cx, cy, tilt, linear, fitted)
errs.sort()
n = len(errs)
print(f"{n} stored boxes, frames {W}x{H}, fov {FOV} deg, gimbal tilt as recorded")
print(f"  median error {errs[n//2]:.2f} deg")
print(f"  90th pct     {errs[int(n*0.9)]:.2f} deg")
print(f"  worst        {errs[-1]:.2f} deg")
print(f"  over the 1.5 deg the geometry expects: {sum(1 for e in errs if e > 1.5)} of {n}")
e, cx, cy, tilt, lin, fit = worst
print(f"\nworst box: centre ({cx:.2f}, {cy:.2f}) of frame, gimbal tilt {tilt} deg")
print(f"  pixels-times-a-gain said {lin:+.1f} deg, the fitted lens says {fit:+.1f} deg")

print("\nwhere the error lives, at the recorded tilt of 10 deg:")
print(f"{'':>8}" + "".join(f"{c:>9.2f}" for c in (0.1, 0.3, 0.5, 0.7, 0.9)))
for cy in (0.1, 0.3, 0.5, 0.7, 0.9):
    cells = []
    for cx in (0.1, 0.3, 0.5, 0.7, 0.9):
        lin, _ = retired([cx - 0.02, cy - 0.02, cx + 0.02, cy + 0.02])
        cells.append(f"{lin - azimuth_deg(cx, cy, 10.0, SIZE):>+9.2f}")
    print(f"cy {cy:.1f} " + "".join(cells))
