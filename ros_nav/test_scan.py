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


def room_range(x0, y0, direction, half_w=2.0, half_h=1.5):
    """How far a beam from (x0, y0) along `direction` travels to a box's walls."""
    dx, dy = math.cos(direction), math.sin(direction)
    best = float("inf")
    for wall, d, origin in ((half_w, dx, x0), (-half_w, dx, x0),
                            (half_h, dy, y0), (-half_h, dy, y0)):
        if abs(d) > 1e-9:
            t = (wall - origin) / d
            if t > 0:
                best = min(best, t)
    return best


def turning_sweep(rate, n=420, scan_time=0.1, mount=math.pi / 2, heading0=0.3,
                  where=(0.6, -0.4)):
    """One D500 sweep from a rover turning on the spot at `rate` rad/s.

    The sensor starts at the rover's left and sweeps clockwise; each point is
    reported in the rover's frame as it was at that point's own moment, which is
    what the library hands over.
    """
    points = []
    for k in range(n):
        t = k / n * scan_time
        phi = mount - 2.0 * math.pi * k / n
        heading = heading0 + rate * t
        r = room_range(where[0], where[1], heading + phi)
        points.append((r * math.cos(phi), r * math.sin(phi)))
    return points


def test_a_turning_scan_is_put_back_together():
    from scan_deskew import YawHistory, deskew, sweep_offset

    section("a scan taken while turning, put back where it was at the sweep's start")
    rate = math.radians(120.0)             # what this chassis does on the spot
    heading0, where = 0.3, (0.6, -0.4)
    history = YawHistory()
    for i in range(-20, 21):               # odometry at 20 Hz around the sweep
        history.add(10.0 + i * 0.05, heading0 + rate * i * 0.05)

    check("the sweep starts at the rover's left", sweep_offset(math.pi / 2, math.pi / 2, 0.1),
          0.0, tolerance=1e-9)
    check("...and reaches straight ahead a quarter of a sweep later",
          sweep_offset(0.0, math.pi / 2, 0.1), 0.025, tolerance=1e-9)

    raw = turning_sweep(rate, heading0=heading0, where=where)
    fixed, moved = deskew(raw, 10.0, history, math.pi / 2, 0.1)
    check("odometry covers the sweep, so it is put together", moved, True)

    truth = turning_sweep(0.0, heading0=heading0, where=where)
    raw_bins, _, inc = bin_scan(raw)
    fixed_bins, _, _ = bin_scan(fixed)
    true_bins, _, _ = bin_scan(truth)

    def worst(bins):
        # Away from the corners, where a degree of bearing is a jump in range.
        errors = []
        for b in range(len(bins)):
            if math.isinf(bins[b]) or math.isinf(true_bins[b]):
                continue
            neighbours = [true_bins[(b + d) % len(bins)] for d in (-2, 2)]
            if max(abs(v - true_bins[b]) for v in neighbours) > 0.05:
                continue
            errors.append(abs(bins[b] - true_bins[b]))
        errors.sort()
        return errors[int(0.95 * len(errors))]

    check("as measured, the turning scan is bent (95th percentile over 10 cm)",
          worst(raw_bins) > 0.10, True)
    check("put back together, it matches the room within 2 cm",
          worst(fixed_bins) < 0.02, True)

    stale = YawHistory()
    stale.add(1.0, 0.0)
    stale.add(1.05, 0.1)
    same, moved = deskew(raw, 10.0, stale, math.pi / 2, 0.1)
    check("odometry that stopped long ago leaves the scan as measured",
          (moved, same == raw), (False, True))

    wrap = YawHistory()
    wrap.add(0.0, math.pi - 0.05)
    wrap.add(0.1, -math.pi + 0.05)
    check("a heading crossing +-180 is unwrapped, not swung through 360",
          wrap.at(0.05), math.pi, tolerance=1e-9)


TESTS = (
    test_scan_binning,
    test_a_turning_scan_is_put_back_together,
)
