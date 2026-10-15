"""The believed pose through a session, scored against a known-good map.

    python pose_timeline.py REC.npz GRID.json [--every 5]

For every Nth scan: the pose the rover believed (map->odom composed with
odom->base_link at the scan's stamp), the share of returns that land within a
cell of a wall there, and the best pose within 0.5 m and 25 degrees with its
share. A believed pose whose best neighbour fits much better is a wrong pose.
"""
import argparse
import base64
import json
import math
import time
import zlib

import numpy as np
from scipy.ndimage import binary_dilation

a = argparse.ArgumentParser()
a.add_argument("rec")
a.add_argument("grid")
a.add_argument("--every", type=int, default=5)
a.add_argument("--map-tf", help="a replay's map->odom as graph_replay_tool.py writes it")
a.add_argument("--until", type=float, help="last scan stamp to score")
args = a.parse_args()

g = json.load(open(args.grid))
occ = np.frombuffer(zlib.decompress(base64.b64decode(g["data"])), dtype=np.int8).reshape(g["height"], g["width"]) >= 50
near = binary_dilation(occ, iterations=1)
res, ox, oy = g["resolution_m"], g["origin_x_m"], g["origin_y_m"]

d = np.load(args.rec)
ranges, meta = d["ranges"], d["scan_meta"]
otf = d["odom_tf"][np.argsort(d["odom_tf"][:, 0])]
if args.map_tf:
    rows = np.array(json.load(open(args.map_tf)), dtype=float)
    mtf = np.column_stack([rows[:, 0], rows[:, 0], rows[:, 1], rows[:, 2], rows[:, 3]])
else:
    mtf = d["map_tf"]
mtf = mtf[np.argsort(mtf[:, 0])]


def lookup(table, t):
    i = int(np.clip(np.searchsorted(table[:, 0], t), 1, len(table) - 1))
    return table[i - 1] if t - table[i - 1, 0] < table[i, 0] - t else table[i]


def score(px, py, ph, ang, rr):
    xs = px + rr * np.cos(ph + ang)
    ys = py + rr * np.sin(ph + ang)
    ix = ((xs - ox) / res).astype(int)
    iy = ((ys - oy) / res).astype(int)
    ok = (ix >= 0) & (iy >= 0) & (ix < near.shape[1]) & (iy < near.shape[0])
    hit = np.zeros(len(xs), bool)
    hit[ok] = near[iy[ok], ix[ok]]
    return hit.mean()


offsets = [(dx, dy) for dx in np.arange(-0.5, 0.51, 0.1) for dy in np.arange(-0.5, 0.51, 0.1)]
turns = np.radians(np.arange(-25, 26, 2.5))
print("%-9s %21s %6s %6s %21s" % ("time", "believed", "fit", "best", "best is off by"))
for k in range(0, len(ranges), args.every):
    t = meta[k, 0]
    if args.until and t > args.until:
        break
    o, m = lookup(otf, t), lookup(mtf, t)
    if abs(o[0] - t) > 0.2 or abs(m[0] - t) > 2.0:
        continue
    c, s = math.cos(m[4]), math.sin(m[4])
    px, py, ph = m[2] + c * o[2] - s * o[3], m[3] + s * o[2] + c * o[3], m[4] + o[4]
    r = ranges[k]
    ang = meta[k, 2] + np.arange(len(r)) * meta[k, 3]
    ok = np.isfinite(r) & (r > 0.15) & (r < 7.5)
    ang, rr = ang[ok], r[ok]
    if len(rr) < 60:
        continue
    here = score(px, py, ph, ang, rr)
    best, bo = here, (0.0, 0.0, 0.0)
    for dt in turns:
        for dx, dy in offsets:
            v = score(px + dx, py + dy, ph + dt, ang, rr)
            if v > best + 1e-9:
                best, bo = v, (dx, dy, math.degrees(dt))
    print("%-9s (%6.2f, %6.2f, %6.1f) %6.2f %6.2f  %4.1f m %+6.1f deg" % (
        time.strftime("%H:%M:%S", time.localtime(t)), px, py, math.degrees(ph) % 360 - 180 * 0,
        here, best, math.hypot(bo[0], bo[1]), bo[2]))
