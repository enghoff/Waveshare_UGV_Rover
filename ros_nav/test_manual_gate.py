"""The 20 cm rule for manual driving, against scans built by hand.

No ROS: `manual_gate.py` is plain arithmetic over a LaserScan's numbers. What
matters is that something in the way stops the rover only in the direction it is
driving, that nothing beside it does, that turning is never cut, and that a
missing or stale scan stops it driving rather than letting it drive blind.
"""
import math

import manual_gate as g
from test_harness import check


def scan(points, count=360):
    """A full turn of empty returns with a few placed: (bearing deg, range m)."""
    ranges = [math.inf] * count
    for bearing, distance in points:
        ranges[int(round(bearing)) % count] = distance
    return ranges, 0.0, 2 * math.pi / count, 0.05, 12.0


def edge(lidar_m, body_m):
    """How far from the body edge a return this far from the lidar is."""
    return round(lidar_m - body_m, 3)


def test_clearance_is_measured_from_the_body_in_its_own_width():
    front, rear = g.clearance(*scan([(0, 0.55), (180, 0.70)]))
    check("ahead, from the front edge", round(front, 3), edge(0.55, g.BODY_FRONT_M))
    check("behind, from the rear edge", round(rear, 3), edge(0.70, g.BODY_BACK_M))
    front, rear = g.clearance(*scan([(90, 0.25), (270, 0.25)]))
    check("something beside the rover is in neither", (front, rear), (None, None))
    front, _ = g.clearance(*scan([(15, 0.5)]))
    check("something ahead but off to one side counts while it is within the "
          "rover's width", front is not None, True)
    front, _ = g.clearance(*scan([(40, 0.5)]))
    check("...and not once it is outside it", front, None)
    front, _ = g.clearance(*scan([(0, 0.10)]))
    check("a return inside the assumed body is no room at all", front, 0.0)
    check("an empty room is nothing either way",
          g.clearance(*scan([])), (None, None))


def test_the_rover_stops_only_for_what_is_in_its_way():
    near = 0.15          # well inside 20 cm of the body
    stop = g.stop_at(0.5)
    check("the line moves out with speed",
          (g.stop_at(0.5) > g.STOP_M, g.stop_at(0.1) == g.stop_at(0.33)), (True, True))
    check("forward into something close is cut",
          g.gate(0.5, 0.0, near, None, 0.05), (0.0, 0.0, "ahead"))
    check("...but backing away from it is not",
          g.gate(-0.5, 0.0, near, None, 0.05), (-0.5, 0.0, ""))
    check("reversing into something close is cut",
          g.gate(-0.3, 0.0, None, near, 0.05), (0.0, 0.0, "behind"))
    check("...and something behind never stops it going forward",
          g.gate(0.5, 0.0, None, 0.0, 0.05), (0.5, 0.0, ""))
    check("turning is never cut, so a stopped rover can turn away",
          g.gate(0.5, 0.6, near, near, 0.05), (0.0, 0.6, "ahead"))
    check("just beyond the line it drives",
          g.gate(0.5, 0.0, stop + 0.01, None, 0.05)[2], "")
    check("...and on it it does not", g.gate(0.5, 0.0, stop, None, 0.05)[2], "ahead")
    check("no scan is no driving", g.gate(0.3, 0.2, None, None, None),
          (0.0, 0.2, "no fresh scan"))
    check("...and neither is an old one",
          g.gate(0.3, 0.0, 5.0, 5.0, g.SCAN_STALE_S + 0.1)[2], "no fresh scan")
    check("standing still is never refused", g.gate(0.0, 0.0, None, None, None),
          (0.0, 0.0, ""))


TESTS = (
    test_clearance_is_measured_from_the_body_in_its_own_width,
    test_the_rover_stops_only_for_what_is_in_its_way,
)
