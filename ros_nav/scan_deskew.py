"""Put each point of a turning rover's scan where it was, at the sweep's start.

**A scan taken while the rover turns is smeared, and the mapper cannot tell.** The
D500 sweeps once in 0.1 s; at the 120 degrees a second this chassis turns on the
spot, the last point of a sweep is measured 12 degrees of rover heading after the
first. Published as though every point were measured at one instant, the room in
the scan is bent, and slam_toolbox matches the bent room against the map. On
2026-10-10, replaying twelve recorded turns into a second mapper
(captures/2026-10-10-turns), placing each point at its own moment took the
heading after a turn from 2.1 degrees typical, 4.5 at worst, to 0.6 and 1.0 --
with the mapper's heading gate on and odometry stamped 48 ms earlier (base_node's
`ODOM_LATENCY_S`) in both.

A point's moment comes from its bearing, not from its place in the list: the
sensor starts each revolution at bearing zero, which `slam2d`'s mount angle puts
at the rover's left, and sweeps clockwise, so a point at rover bearing `phi` was
measured `((mount - phi) mod 360) / 360` of a sweep after the start. How far the
rover had turned by then comes from odometry's heading history, interpolated, and
extrapolated at the last rate for the few tens of milliseconds odometry trails
the sensor.

No ROS here, so the arithmetic is tested on its own (test_scan.py).
"""

import bisect
import math

# How long a heading history is kept. Two sweeps and change; older is never asked.
HISTORY_S = 1.0
# How far past the newest odometry a heading may be extrapolated. Odometry trails
# the end of a sweep by about its 48 ms latency plus its 50 ms interval; past this
# the history has stopped and the scan is published as measured.
EXTRAPOLATE_S = 0.2
# Less than this much turn across a sweep is not turning: a twentieth of a degree,
# under a centimetre at the far wall.
STILL_RAD = math.radians(0.05)


class YawHistory:
    """Odometry's heading over the last second, unwrapped, in radians."""

    def __init__(self):
        self.t = []
        self.yaw = []

    def add(self, t, yaw):
        if self.t and t <= self.t[-1]:
            return
        if self.yaw:
            step = (yaw - self.yaw[-1] + math.pi) % (2.0 * math.pi) - math.pi
            yaw = self.yaw[-1] + step
        self.t.append(t)
        self.yaw.append(yaw)
        cut = bisect.bisect_left(self.t, t - HISTORY_S)
        if cut:
            del self.t[:cut]
            del self.yaw[:cut]

    def at(self, t):
        """Heading at time `t`, or None when the history does not reach it."""
        n = len(self.t)
        if n == 0 or t < self.t[0]:
            return None
        if t >= self.t[-1]:
            if t - self.t[-1] > EXTRAPOLATE_S:
                return None
            if n < 2:
                return self.yaw[-1]
            rate = (self.yaw[-1] - self.yaw[-2]) / max(self.t[-1] - self.t[-2], 1e-3)
            return self.yaw[-1] + rate * (t - self.t[-1])
        i = bisect.bisect_right(self.t, t)
        t0, t1 = self.t[i - 1], self.t[i]
        f = (t - t0) / (t1 - t0)
        return self.yaw[i - 1] + f * (self.yaw[i] - self.yaw[i - 1])


def sweep_offset(phi, mount_rad, scan_time):
    """Seconds after the sweep's start at which a point at rover bearing `phi` was measured."""
    return ((mount_rad - phi) % (2.0 * math.pi)) / (2.0 * math.pi) * scan_time


def deskew(points, start, history, mount_rad, scan_time):
    """`points` as (x, y) in the rover's frame at `start`, or unchanged if they cannot be.

    Returns (points, moved): `moved` is False when the history does not cover the
    sweep, in which case the scan is no worse than it was before this existed.
    """
    yaw_ref = history.at(start)
    yaw_end = history.at(start + scan_time)
    if yaw_ref is None or yaw_end is None:
        return points, False
    if abs(yaw_end - yaw_ref) < STILL_RAD:
        # Not turning: every point is where it was, and 420 interpolations a
        # sweep are spent only when they move something.
        return points, True
    out = []
    for x, y in points:
        phi = math.atan2(y, x)
        yaw = history.at(start + sweep_offset(phi, mount_rad, scan_time))
        turned = 0.0 if yaw is None else yaw - yaw_ref
        if turned:
            c, s = math.cos(turned), math.sin(turned)
            x, y = c * x - s * y, s * x + c * y
        out.append((x, y))
    return out, True
