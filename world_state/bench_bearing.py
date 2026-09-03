#!/usr/bin/env python3
"""What the world state's pixels-times-a-gain bearing costs, on real boxes.

    python world_state/bench_bearing.py /tmp/run.db
    ssh orin 'cd ~/ugv/world_state && python3 bench_bearing.py'


`view._from_box` maps a box's horizontal centre to an angle with one
multiplication. `face_tracking/lens.py` holds a lens that was swept and fitted on
this rover, and `aiming.py` was moved off exactly that multiplication on
2026-08-19 because it is only right along the two centre lines. This asks what
the difference comes to for the boxes the rover has actually stored.
"""
from __future__ import annotations

import argparse
import json
import math
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

from world_state.view import _from_box       # noqa: E402
import lens as lensmod                       # noqa: E402

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
LENS = lensmod.lens_for(W, H)

def azimuth_deg(cx_frac, cy_frac, tilt_deg):
    """Where a pixel points in the world's horizontal plane, via the fitted lens.

    The gimbal pans about the world vertical and then tilts about its own
    horizontal, so the camera ray is rotated by the tilt before its azimuth is
    read off -- which is the step a separable model leaves out.
    """
    x, y, z = lensmod.ray_at(cx_frac * W, cy_frac * H, LENS)
    t = math.radians(tilt_deg)
    # Rotated about the camera's own horizontal axis, positive up. Only the
    # component out of the lens is wanted: the height the ray ends up at does not
    # change its bearing, and it is `z` that the tilt steals from.
    z_level = y * math.sin(t) + z * math.cos(t)
    # Positive to the camera's left, which is the map's convention and the
    # opposite of the gimbal's -- the same swap `view.ray` makes.
    return math.degrees(math.atan2(-x, z_level))

db = sqlite3.connect(f"file:{args.database}?mode=ro", uri=True)
rows = db.execute("SELECT bbox_json, observer_tilt_deg FROM observations").fetchall()
worst, errs = None, []
for bbox_json, tilt in rows:
    box = json.loads(bbox_json)
    linear, _span = _from_box(box, FOV)
    cx, cy = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
    fitted = azimuth_deg(cx, cy, tilt or 0.0)
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
print(f"  pixels-times-a-gain says {lin:+.1f} deg, the fitted lens says {fit:+.1f} deg")

print("\nwhere the error lives, at the recorded tilt of 10 deg:")
print(f"{'':>8}" + "".join(f"{c:>9.2f}" for c in (0.1, 0.3, 0.5, 0.7, 0.9)))
for cy in (0.1, 0.3, 0.5, 0.7, 0.9):
    cells = []
    for cx in (0.1, 0.3, 0.5, 0.7, 0.9):
        lin, _ = _from_box([cx - 0.02, cy - 0.02, cx + 0.02, cy + 0.02], FOV)
        cells.append(f"{lin - azimuth_deg(cx, cy, 10.0):>+9.2f}")
    print(f"cy {cy:.1f} " + "".join(cells))
