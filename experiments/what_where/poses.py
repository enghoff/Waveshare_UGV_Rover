"""The three pose helpers of experiments/m5_changes/change_detect.py, without
its scipy import, so that the shadow runner loads on the rover's ROS Python."""
import math

import numpy as np


def interp_pose(rows, t):
    """(x, y, yaw) from rows (stamp, recv, x, y, yaw) at time t, or None."""
    st = rows[:, 0]
    i = np.searchsorted(st, t)
    if i <= 0 or i >= len(st):
        return None
    a, b = rows[i - 1], rows[i]
    if b[0] - a[0] > 0.5:
        return None
    f = (t - a[0]) / max(1e-9, b[0] - a[0])
    dyaw = math.atan2(math.sin(b[4] - a[4]), math.cos(b[4] - a[4]))
    return a[2] + f * (b[2] - a[2]), a[3] + f * (b[3] - a[3]), a[4] + f * dyaw


def last_pose(rows, t):
    st = rows[:, 0]
    i = np.searchsorted(st, t) - 1
    return None if i < 0 else tuple(rows[i, 2:5])


def compose(a, b):
    x, y, th = a
    bx, by, bth = b
    return (x + bx * math.cos(th) - by * math.sin(th),
            y + bx * math.sin(th) + by * math.cos(th), th + bth)
