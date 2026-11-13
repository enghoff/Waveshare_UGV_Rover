"""A looks file in which every region with neither a depth-camera range nor a
wall place takes the monocular range, where the picture could be scaled.

    python mono_apply.py LOOKS.npz SCALED.npz OUT.npz [MAX_EXT] [--bearing-only]

The monocular place is given to version 4 as a depth box (`box`: placed, with
the 0.5 m spread version 4 gives a box). MAX_EXT (default 0.5) refuses a
reading further outside the picture's anchors' disparities than that share of
their spread. With --bearing-only, regions the lidar ranged keep the lidar.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.environ["VISIT_MODEL"])
import data  # noqa: E402

WALL_MIN_M = 2.0

looks, scaledp, outp = sys.argv[1:4]
max_ext = float(sys.argv[4]) if len(sys.argv) > 4 and not sys.argv[4].startswith("--") else 0.5
a = data.load(looks)
s0 = np.load(scaledp)
at = {int(i): k for k, i in enumerate(s0["id"])}
pick = np.array([at[int(i)] for i in a["id"]])  # a draw may leave regions out
s = {k: s0[k][pick] for k in ("range", "ext", "anchors", "sigma")}
# Strict mode (env STRICT=1): 3+ anchors, the picture's leave-one-out spread
# at most 0.4 m, and never nearer than a lidar return along the bearing allows.
STRICT = os.environ.get("STRICT") == "1"
src = np.array([str(x) for x in a["place_from"]])
posed = np.array([p is not None for p in a["pose_from"]])
oak = np.isin(src, ("outline", "box")) & np.isfinite(a["range"])
on_wall = np.isfinite(a["wx"]) & (np.nan_to_num(a["wr"]) >= WALL_MIN_M)
tgt = posed & ~oak & ~on_wall & np.isfinite(s["range"]) & (np.nan_to_num(s["ext"], nan=9) <= max_ext)
if STRICT:
    lidar_r = np.where(np.isfinite(a["range"]), a["range"], 0.0)
    tgt &= (s["anchors"] >= 3) & (np.nan_to_num(s["sigma"], nan=9) <= 0.4) & \
        (np.nan_to_num(s["range"]) >= lidar_r - 0.3)
if "--bearing-only" in sys.argv:
    tgt &= ~np.isfinite(a["range"])
r = s["range"]
b = np.radians(np.nan_to_num(a["bearing"]))
for k in np.nonzero(tgt)[0]:
    a["range"][k] = r[k]
    a["px"][k] = a["cx"][k] + r[k] * math.cos(b[k])
    a["py"][k] = a["cy"][k] + r[k] * math.sin(b[k])
    a["place_from"][k] = "box"
data.save(a, outp)
print("monocular range given to %d regions (%d were lidar-ranged, %d bearing only)" % (
    tgt.sum(), np.sum(tgt & (src == "recording lidar")) + np.sum(tgt & (src == "lidar")),
    np.sum(tgt & ~np.isin(src, ("recording lidar", "lidar")))))
