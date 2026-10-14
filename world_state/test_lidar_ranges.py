"""A lidar range for a region the depth camera could not range (lidar_ranges.py,
InspectionRanges._lidar_fill)."""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test_harness import check  # noqa: E402
from world_state import lidar_ranges  # noqa: E402
from world_state.depth_client import Ranged  # noqa: E402
from world_state.inspection_ranges import InspectionRanges  # noqa: E402


def section(title):
    print()
    print(title)


def scan(near_deg=30.0, near_m=2.0, far_m=5.0, width_deg=8.0):
    """A sweep of 360 beams, a degree apart from -180, with something `near_m`
    away across `width_deg` centred `near_deg` left of the nose."""
    ranges = []
    for i in range(360):
        angle = -180.0 + i
        off = (angle - near_deg + 180.0) % 360.0 - 180.0
        ranges.append(near_m if abs(off) <= width_deg / 2 else far_m)
    return {"angle_min": math.radians(-180.0), "angle_increment": math.radians(1.0),
            "ranges": ranges, "from": [0.0, 0.0, 0.0]}


def test_range_along():
    section("lidar_ranges: how far the lidar saw across a region")
    s = scan()
    check("a region over the near thing reads its range",
          lidar_ranges.range_along(s, 30.0, 6.0), 2.0)
    check("one beside it reads what is beyond",
          lidar_ranges.range_along(s, 60.0, 6.0), 5.0)
    check("a region wider than the thing takes the nearer quarter of its returns",
          lidar_ranges.range_along(s, 30.0, 20.0), 2.0)
    s["ranges"] = [None] * 360
    check("nothing came back: no range", lidar_ranges.range_along(s, 30.0, 6.0), None)
    s = scan(near_m=7.5, far_m=7.5)
    check("returns past the lidar's band are not read",
          lidar_ranges.range_along(s, 30.0, 6.0), None)
    check("a malformed scan gives no range", lidar_ranges.range_along({}, 0.0, 5.0), None)
    check("a region whose lower edge is below the horizontal reaches the lidar's plane",
          lidar_ranges.reaches_down(-5.0), True)
    check("...as does one ending a couple of degrees above it",
          lidar_ranges.reaches_down(lidar_ranges.REACH_DEG), True)
    check("...but not one ending higher", lidar_ranges.reaches_down(2.5), False)
    check("...nor one with no elevation", lidar_ranges.reaches_down(None), False)


class Region(object):
    def __init__(self, bbox):
        self.bbox = bbox


class Heading(object):
    def __init__(self, last, at):
        self.last = last
        self.last_at = at


class Looker(InspectionRanges):
    def __init__(self, last, at):
        self.heading = Heading(last, at)


def test_fill():
    section("_lidar_fill: the depth camera's gaps filled from the look's own scan")
    # Straight ahead and level, a box in the middle of the picture: its middle is
    # 0 degrees off the aim. The near thing sits there.
    s = scan(near_deg=0.0, near_m=1.8, width_deg=40.0)
    now = time.monotonic()
    capture = {"pose": {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0}, "pan": 0.0,
               "tilt": 0.0, "frame_size": (640, 480), "lens": None}
    middle = Region([0.45, 0.6, 0.55, 0.9])
    high = Region([0.45, 0.05, 0.55, 0.3])
    depth = Ranged(range_m=1.7, sigma_m=0.05, valid=0.9, method="outline")
    found, note = Looker({"scan": s}, now)._lidar_fill(
        capture, [middle, high, middle], [None, None, depth], True)
    check("an unranged region reaching the floor takes the lidar's range",
          (found[0].range_m, found[0].method, found[0].sigma_m),
          (1.8, "lidar", lidar_ranges.SIGMA_M))
    check("...one high in the picture is left alone", found[1], None)
    check("...and a range the depth camera measured is kept", found[2] is depth, True)
    check("...and the look says so", note, "1 ranged by the lidar")
    found, _ = Looker({"scan": s}, now)._lidar_fill(capture, [middle], [None], False)
    check("a look taken on the move gets none", found, [None])
    found, _ = Looker({"scan": s}, now - 10.0)._lidar_fill(capture, [middle], [None], True)
    check("nor one whose scan is not its own check's", found, [None])
    found, _ = Looker({"scan": s}, now)._lidar_fill(dict(capture, pose=None), [middle],
                                                    [None], True)
    check("nor one with no pose", found, [None])
    found, _ = Looker({}, now)._lidar_fill(capture, [middle], [None], True)
    check("nor one whose check returned no scan", found, [None])
    # The armchair from the balcony door: a level look, and the box ends just
    # above the middle of the picture, a degree over the horizontal.
    level = Region([0.43, 0.32, 0.62, 0.46])
    found, _ = Looker({"scan": s}, now)._lidar_fill(capture, [level], [None], True)
    check("a level look's box ending a degree above the horizontal is ranged",
          found[0] is not None and found[0].range_m, 1.8)
    # Tilted up 20 degrees, a box ending at 0.6 of the picture reads 6.9 degrees
    # up, and 2.0 once the tilt's measured bias of 4.9 is taken off.
    found, _ = Looker({"scan": s}, now)._lidar_fill(
        dict(capture, tilt=20.0), [Region([0.45, 0.3, 0.55, 0.6])], [None], True)
    check("the tilted camera's measured elevation bias is taken off",
          found[0] is not None and found[0].range_m, 1.8)
    # Panned 30 degrees right, the thing 30 degrees right of the nose is ahead of
    # the camera.
    s = scan(near_deg=-30.0, near_m=2.4, width_deg=20.0)
    found, _ = Looker({"scan": s}, now)._lidar_fill(dict(capture, pan=30.0), [middle],
                                                    [], True)
    check("the gimbal's pan, positive right, is taken off the region's bearing",
          found[0].range_m, 2.4)


class Store(object):
    def __init__(self):
        self.saved = 0

    def save_depth(self, *_a, **_k):
        self.saved += 1
        return 1


class Ranger(object):
    def lens(self):
        return None

    def depth_map(self):
        return b"depth"


def test_depth_kept_only_for_depth():
    section("_keep_depth: a lidar range is not a reason to fetch a depth map")
    looker = Looker({}, None)
    looker.ranger, looker.store = Ranger(), Store()
    lidar = Ranged(range_m=2.0, sigma_m=0.4, valid=1.0, method="lidar")
    looker._keep_depth("f1", [lidar])
    check("only lidar ranges: nothing fetched", looker.store.saved, 0)
    looker._keep_depth("f2", [lidar, Ranged(range_m=1.0, sigma_m=0.05, valid=0.9,
                                             method="outline")])
    check("a depth range beside it: the map is kept", looker.store.saved, 1)


TESTS = (test_range_along, test_fill, test_depth_kept_only_for_depth)

if __name__ == "__main__":
    for test in TESTS:
        test()
