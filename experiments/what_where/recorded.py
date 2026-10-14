"""Navigation's own pose and lidar, read back from a drive recording.

The store gives a look a pose only when the heading check confirms it
(R-WS-16), and in a rearranged room it mostly does not. The recording kept
what navigation believed at every moment regardless, and the lidar's scans.
`bag_extract.py` (experiments/m5_changes) turns a recording into the arrays
read here.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for _where in (os.path.join(HERE, "..", "m5_changes"), os.path.expanduser("~/m5")):
    if os.path.exists(os.path.join(_where, "change_detect.py")):
        sys.path.insert(0, _where)
        break
from change_detect import compose, interp_pose, last_pose  # noqa: E402

#: A look taken while the rover turned faster than this gets no recorded pose:
#: the store's pose and the shutter disagree by up to 0.2 s, which at this rate
#: is 4 degrees of bearing.
MAX_TURN_DPS = 20.0
#: The lidar's return along a region's bearing: within this many degrees of it,
#: no farther than LIDAR_MAX_M, from the scan nearest the shutter.
LIDAR_HALF_DEG = 3.0
LIDAR_MIN_M = 0.15
LIDAR_MAX_M = 6.0


class Recording:
    def __init__(self, path=None, arrays=None):
        d = np.load(path) if path is not None else arrays
        self.odom = d["odom_tf"][np.argsort(d["odom_tf"][:, 0])]
        self.map = d["map_tf"][np.argsort(d["map_tf"][:, 0])]
        self.ranges = d["ranges"]
        self.meta = d["scan_meta"]
        self.scan_t = self.meta[:, 0] + self.meta[:, 5] / 2.0
        self.start, self.end = self.odom[0, 0], self.odom[-1, 0]

    def covers(self, t):
        return self.start <= t <= self.end

    def pose(self, t):
        """(x, y, heading_deg) navigation believed at t, or None."""
        m, o = last_pose(self.map, t), interp_pose(self.odom, t)
        if m is None or o is None:
            return None
        x, y, th = compose(m, o)
        return x, y, math.degrees(th)

    def turn_rate(self, t):
        a, b = interp_pose(self.odom, t - 0.1), interp_pose(self.odom, t + 0.1)
        if a is None or b is None:
            return None
        return abs(math.degrees(math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2])))) / 0.2

    def lidar_point(self, t, bearing_deg):
        """The nearest lidar return along a map bearing at t, as a map point."""
        k = int(np.argmin(np.abs(self.scan_t - t)))
        if abs(self.scan_t[k] - t) > 0.3:
            return None
        p = self.pose(self.scan_t[k])
        if p is None:
            return None
        x, y, heading = p
        _s, _r, amin, ainc, _ti, _st = self.meta[k]
        r = self.ranges[k]
        ang = np.degrees(amin + ainc * np.arange(len(r))) + heading
        near = np.abs((ang - bearing_deg + 180.0) % 360.0 - 180.0) <= LIDAR_HALF_DEG
        ok = near & np.isfinite(r) & (r > LIDAR_MIN_M) & (r < LIDAR_MAX_M)
        if not ok.any():
            return None
        d = float(np.min(r[ok]))
        b = math.radians(bearing_deg)
        return x + d * math.cos(b), y + d * math.sin(b)


def for_time(recordings, t):
    for rec in recordings:
        if rec.covers(t):
            return rec
    return None
