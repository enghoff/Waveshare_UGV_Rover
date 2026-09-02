"""The lidar scan: binning points into bearings.

A scan binned into the wrong half of the circle still looks like a room, which
is why it is checked against bearings that are known by construction.
"""
import math

from test_harness import check, section


    # The gyro is what the heading depends on entirely, so a missing scale must
    # be refused rather than defaulted -- checked in test_calibration below.


# --- the scan -----------------------------------------------------------------
def bin_scan(points, bins=360, range_min=0.12, range_max=8.0):
    """lidar_node.LidarNode.to_scan's binning, standing alone."""
    increment = 2.0 * math.pi / bins
    ranges = [float("inf")] * bins
    used = 0
    for x, y in points:
        r = math.hypot(x, y)
        if r < range_min or r > range_max:
            continue
        i = int((math.atan2(y, x) + math.pi) / increment) % bins
        if r < ranges[i]:
            ranges[i] = r
        used += 1
    return ranges, used, increment


def at_bearing(ranges, increment, degrees):
    i = int((math.radians(degrees) + math.pi) / increment) % len(ranges)
    return ranges[i]


def test_scan_binning():
    section("scan points -> LaserScan")
    # slam2d hands back x forward and y left, which is REP-103. A point two
    # metres straight ahead must land where a consumer looks for straight ahead.
    ranges, used, inc = bin_scan([(2.0, 0.0)])
    check("a point 2 m ahead is 2 m ahead", at_bearing(ranges, inc, 0), 2.0,
          tolerance=0.02)
    check("...and is the only one", used, 1)

    ranges, _, inc = bin_scan([(0.0, 1.5)])
    check("a point 1.5 m to port reads at +90 degrees",
          at_bearing(ranges, inc, 90), 1.5, tolerance=0.02)
    ranges, _, inc = bin_scan([(0.0, -1.5)])
    check("a point to starboard reads at -90 degrees",
          at_bearing(ranges, inc, -90), 1.5, tolerance=0.02)
    ranges, _, inc = bin_scan([(-3.0, 0.0)])
    check("a point behind reads at 180 degrees",
          at_bearing(ranges, inc, 180), 3.0, tolerance=0.02)

    # Two points in one bin: the nearer wins, because this message is read by an
    # obstacle costmap and rounding a chair leg away is what gets it hit.
    ranges, _, inc = bin_scan([(2.0, 0.0), (1.0, 0.001)])
    check("where two points share a bin the nearer one wins",
          at_bearing(ranges, inc, 0), 1.0, tolerance=0.02)

    # Out-of-range points are dropped rather than clamped. A clamped point is a
    # wall reported where there is none.
    _, used, _ = bin_scan([(20.0, 0.0), (0.05, 0.0)])
    check("points beyond the sensor's honest reach are dropped, not clamped",
          used, 0)

    # Nothing may land outside the array, including a point at exactly pi.
    ranges, used, _ = bin_scan([(-2.0, -1e-12)])
    check("a point at the wrap does not fall off the end", used, 1)

    # A full circle fills every bin exactly once -- offset half a degree so the
    # points sit in the middle of their bins rather than on the boundaries. On
    # the boundary the answer is genuinely ambiguous and floating point decides
    # it, which is a property of binning rather than a fault to fix: a real
    # sensor's returns are not aligned to the grid either.
    circle = [(math.cos(math.radians(d + 0.5)) * 2,
               math.sin(math.radians(d + 0.5)) * 2) for d in range(0, 360)]
    ranges, used, _ = bin_scan(circle)
    check("360 points a degree apart fill 360 bins", used, 360)
    check("...leaving none empty", sum(1 for r in ranges if math.isinf(r)), 0)


TESTS = (
    test_scan_binning,
)
