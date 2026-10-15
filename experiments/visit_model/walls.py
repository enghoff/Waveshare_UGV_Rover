"""The walls: cells the mapper calls occupied in every session's grid.

    python walls.py OUT.npz GRID.json...

Furniture that moved between the sessions drops out; what is left is the
building and whatever never moved. Each session script saved the grid as it
found it (`session.py start`, NAME-grid.json). `wall_hit` casts a sight line
across it.
"""
import base64
import json
import math
import sys
import zlib

import numpy as np


def _grid(path):
    g = json.load(open(path))
    cells = np.frombuffer(zlib.decompress(base64.b64decode(g["data"])), dtype=np.int8)
    return g, cells.reshape(g["height"], g["width"]) >= 50


def build(paths):
    g0, occ0 = _grid(paths[0])
    res, ox, oy = g0["resolution_m"], g0["origin_x_m"], g0["origin_y_m"]
    ii, jj = np.mgrid[0:occ0.shape[0], 0:occ0.shape[1]]
    x = ox + (jj + 0.5) * res
    y = oy + (ii + 0.5) * res
    stable = occ0.copy()
    for p in paths[1:]:
        g, occ = _grid(p)
        j = np.floor((x - g["origin_x_m"]) / g["resolution_m"]).astype(int)
        i = np.floor((y - g["origin_y_m"]) / g["resolution_m"]).astype(int)
        inside = (i >= 0) & (i < occ.shape[0]) & (j >= 0) & (j < occ.shape[1])
        hit = np.zeros_like(stable)
        hit[inside] = occ[i[inside], j[inside]]
        # A wall one cell off between two grids is still the wall.
        grown = hit.copy()
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                grown |= np.roll(np.roll(hit, di, 0), dj, 1)
        stable &= grown
    return {"stable": stable, "res": res, "ox": ox, "oy": oy}


def wall_hit(w, x, y, bearing_deg, near=0.3, far=8.0):
    """Distance along a sight line to the first wall cell, or None."""
    res = w["res"]
    ux, uy = math.cos(math.radians(bearing_deg)), math.sin(math.radians(bearing_deg))
    s = w["stable"]
    for k in range(int(near / (res / 2)), int(far / (res / 2))):
        d = k * res / 2
        j = int((x + d * ux - w["ox"]) / res)
        i = int((y + d * uy - w["oy"]) / res)
        if not (0 <= i < s.shape[0] and 0 <= j < s.shape[1]):
            return None
        if s[i, j]:
            return d
    return None


def load(path):
    d = np.load(path)
    return {"stable": d["stable"], "res": float(d["res"]), "ox": float(d["ox"]), "oy": float(d["oy"])}


if __name__ == "__main__":
    w = build(sys.argv[2:])
    np.savez_compressed(sys.argv[1], **w)
    print("stable cells", int(w["stable"].sum()))
