"""Odometry's heading against the lidar's, through session3-1015.

For a rover turning on the spot the lidar's range profile shifts round by the angle
turned, so consecutive scans give a rotation that owes nothing to the gyro. Summed
over the stretches where the rover hardly moves its position, that is compared with
odometry's yaw over the same stretch; the mapper's map->odom correction is
printed beside it.
"""
import math
import sys
import time

import numpy as np

d = np.load(sys.argv[1])
ranges, meta, odom_tf, map_tf = d["ranges"], d["scan_meta"], d["odom_tf"], d["map_tf"]
t_scan = meta[:, 0]
order = np.argsort(odom_tf[:, 0])
odom_tf = odom_tf[order]
order = np.argsort(map_tf[:, 0])
map_tf = map_tf[order]


def at(table, t, col):
    i = np.searchsorted(table[:, 0], t)
    i = min(max(i, 1), len(table) - 1)
    return table[i - 1, col] if abs(table[i - 1, 0] - t) < abs(table[i, 0] - t) else table[i, col]


def unwrap_at(table, ts, col):
    i = np.clip(np.searchsorted(table[:, 0], ts), 0, len(table) - 1)
    return np.unwrap(table[:, col])[i]


def shift(a, b, span=25):
    """Bins b is a rotated by, best over +-span: b[i] ~ a[i + k]."""
    best, best_k = None, 0
    ok_a = np.isfinite(a) & (a > 0.15)
    for k in range(-span, span + 1):
        rb = np.roll(b, k)
        ok = ok_a & np.isfinite(rb) & (rb > 0.15)
        if ok.sum() < 60:
            continue
        err = np.median(np.abs(a[ok] - rb[ok]))
        if best is None or err < best:
            best, best_k = err, k
    return best_k, best


odom_yaw = unwrap_at(odom_tf, t_scan, 4)
ox = np.interp(t_scan, odom_tf[:, 0], odom_tf[:, 2])
oy = np.interp(t_scan, odom_tf[:, 0], odom_tf[:, 3])
map_yaw = unwrap_at(map_tf, t_scan, 4)
inc = meta[0, 3]

# Turning stretches: consecutive scans whose odometry moved under 3 cm while it turned
# over 1.5 degrees, or whose lidar shift says it turned.
step = 2
rows = []
for i in range(0, len(t_scan) - step, step):
    j = i + step
    moved = math.hypot(ox[j] - ox[i], oy[j] - oy[i])
    k, err = shift(ranges[i], ranges[j])
    rows.append((t_scan[i], t_scan[j], moved, math.degrees(odom_yaw[j] - odom_yaw[i]),
                 k * math.degrees(inc), err, math.degrees(map_yaw[j] - map_yaw[i])))
rows = np.array(rows)
still_pos = rows[:, 2] < 0.03
turning = still_pos & ((np.abs(rows[:, 3]) > 1.0) | (np.abs(rows[:, 4]) > 1.0))
print("pairs", len(rows), "turning on the spot", turning.sum())

# Group consecutive turning pairs into turns.
turns, cur = [], None
for r, flag in zip(rows, turning):
    if flag:
        if cur is None:
            cur = [r[0], r[1], 0.0, 0.0, 0.0]
        cur[1] = r[1]
        cur[2] += r[3]
        cur[3] += r[4]
        cur[4] += r[6]
    elif cur is not None:
        turns.append(cur)
        cur = None
if cur is not None:
    turns.append(cur)
print("%-9s %6s %9s %9s %9s %10s" % ("start", "secs", "odom", "lidar", "odom-lid", "map->odom"))
total = 0.0
for a, b, od, li, mo in turns:
    if abs(od) < 5 and abs(li) < 5:
        continue
    total += od - li
    print("%-9s %6.1f %+9.1f %+9.1f %+9.1f %+10.1f   running %+7.1f" % (
        time.strftime("%H:%M:%S", time.localtime(a)), b - a, od, li, od - li, mo, total))
