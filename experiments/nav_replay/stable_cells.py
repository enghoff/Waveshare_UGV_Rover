"""Which cells of the map have stayed put, learned from drive recordings.

    python stable_cells.py OUT.npz REC.npz...

For the first-principles plan's first test (docs/plans/perception-from-
first-principles.md, "Localising on stable cells"): learned only from
recordings made before the shakedown, so the test does not already know the
armchairs moved. Each recording's scans are laid on the map at the pose
navigation believed (the controls of 2026-10-11 and the drives of 2026-10-10,
whose poses agreed with the stored looks to a few centimetres). A cell is
occupied in a recording when HIT_MIN returns landed in it. A cell is stable
when it was occupied in at least STABLE_SHARE of the recordings that saw it at
all -- a wall is there every time; an armchair moved between the days is not.

Writes the grid in the map frame: `stable`, `seen` (recordings that observed
the cell), `res`, `ox`, `oy`.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "visit_model"))
import recorded  # noqa: E402

RES = 0.05
OX, OY = -23.0, -25.0
W, H = 400, 400
EVERY = 5
HIT_MIN = 3
STABLE_SHARE = 0.8
SEEN_MIN = 3
MAX_M = 8.0


def lay(rec):
    hits = np.zeros((H, W), np.int32)
    passed = np.zeros((H, W), bool)
    for k in range(0, len(rec.scan_t), EVERY):
        p = rec.pose(rec.scan_t[k])
        if p is None:
            continue
        x, y, th = p
        _s, _r, amin, ainc, _ti, _st = rec.meta[k]
        r = rec.ranges[k].astype(float)
        ang = amin + ainc * np.arange(len(r)) + math.radians(th)
        ok = np.isfinite(r) & (r > 0.15) & (r < MAX_M)
        hx, hy = x + r[ok] * np.cos(ang[ok]), y + r[ok] * np.sin(ang[ok])
        j = ((hx - OX) / RES).astype(int)
        i = ((hy - OY) / RES).astype(int)
        inside = (i >= 0) & (i < H) & (j >= 0) & (j < W)
        np.add.at(hits, (i[inside], j[inside]), 1)
        # Cells along each beam short of its end were seen empty.
        for frac in (0.25, 0.5, 0.75, 0.9):
            fx, fy = x + frac * r[ok] * np.cos(ang[ok]), y + frac * r[ok] * np.sin(ang[ok])
            j2 = ((fx - OX) / RES).astype(int)
            i2 = ((fy - OY) / RES).astype(int)
            ins = (i2 >= 0) & (i2 < H) & (j2 >= 0) & (j2 < W)
            passed[i2[ins], j2[ins]] = True
    occ = hits >= HIT_MIN
    seen = occ | passed
    return occ, seen


def main(out, paths):
    occ_n = np.zeros((H, W), np.int32)
    seen_n = np.zeros((H, W), np.int32)
    for p in paths:
        occ, seen = lay(recorded.Recording(p))
        # One cell either way is the same wall.
        grown = occ.copy()
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                grown |= np.roll(np.roll(occ, di, 0), dj, 1)
        occ_n += grown
        seen_n += seen | grown
        print(os.path.basename(p), int(occ.sum()), "occupied")
    stable = (seen_n >= SEEN_MIN) & (occ_n >= STABLE_SHARE * np.maximum(seen_n, 1))
    np.savez_compressed(out, stable=stable, seen=seen_n, occupied=occ_n, res=RES, ox=OX, oy=OY)
    print("stable cells", int(stable.sum()), "from", len(paths), "recordings")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
