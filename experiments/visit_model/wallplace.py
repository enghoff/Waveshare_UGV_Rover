"""Where raised things hang: a region well above the lidar's plane is placed
where its sight line meets a wall.

    python wallplace.py LOOKS.npz WALLS.npz

Adds to LOOKS each region's lower edge in degrees above the horizontal
(`lower`, the tilt's bias out, as the deployed lidar rule reads it) and, for a
region whose lower edge is at least ELEVATED_DEG up, the first wall cell along
its sight line (`wx`, `wy`, `wr`: place and distance). A painting's depth
range is as noisy as any other's and its bearing puts it somewhere along a
line; the wall says where on that line it hangs. Whether to use it is the
model's choice: a raised thing a depth range puts well short of the wall is
not on it.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
import data  # noqa: E402
import walls  # noqa: E402

#: A region whose lower edge is this far above the horizontal hangs clear of
#: the floor's furniture: about 25 cm above the camera at 3 m.
ELEVATED_DEG = 5.0


def main(looks_path, walls_path):
    from world_state import locate, view
    a = data.load(looks_path)
    w = walls.load(walls_path)
    n = len(a["id"])
    lower = np.full(n, np.nan)
    wx, wy, wr = np.full(n, np.nan), np.full(n, np.nan), np.full(n, np.nan)
    posed = np.array([p is not None for p in a["pose_from"]])
    for i in np.flatnonzero(posed):
        x0, _y0, x1, y1 = a["bbox"][i]
        tilt = float(a["tilt"][i])
        e = view.elevation_deg((x0 + x1) / 2.0, y1, tilt, (640, 480), None)
        if e is None:
            continue
        lower[i] = e - locate.ELEVATION_BIAS_DEG.get(round(tilt), 0.0)
        if lower[i] < ELEVATED_DEG:
            continue
        d = walls.wall_hit(w, a["cx"][i], a["cy"][i], a["bearing"][i])
        if d is None:
            continue
        b = math.radians(a["bearing"][i])
        wx[i], wy[i], wr[i] = a["cx"][i] + d * math.cos(b), a["cy"][i] + d * math.sin(b), d
    a.update(lower=lower, wx=wx, wy=wy, wr=wr)
    data.save(a, looks_path)
    print("%d raised regions, %d placed on a wall" % (int(np.sum(lower >= ELEVATED_DEG)),
                                                       int(np.isfinite(wx).sum())))


if __name__ == "__main__":
    main(*sys.argv[1:3])
