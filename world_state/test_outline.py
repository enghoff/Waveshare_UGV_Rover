"""A region's distance is read from the depth under its own outline, and its box only
where the outline cannot say.

On 2026-10-02 the landscape painting behind the dining chairs read 1.22 m against
2.54, because the depth service answers a box with its nearest surface and the
nearest surface in that box was a chair. These hold the replacement to the case that
went wrong, to the depth service's own arithmetic, and to the mount's own geometry.
"""
from __future__ import annotations

import math
import tempfile

from test_harness import check
from test_fakes import a_capture, a_pose, a_sighting, a_store
from world_state import oak, outline
from world_state.depth_client import DepthMap, FakeRanger, Lens, Ranged

LENS = Lens(fx=500.30, fy=500.17, cx=321.23, cy=190.69, width=640, height=360,
            hfov_deg=65.2, vfov_deg=39.6)
SIZE = (640, 480)
BOX = [0.45, 0.30, 0.55, 0.60]


def _np():
    import numpy
    return numpy


def a_painting_behind_a_chair(np, wall_mm=4000, painting_mm=2500, chair_mm=1200):
    """A depth map of a wall, a painting in the region's box, and a chair across the
    bottom 45% of that box: the depth service's nearest surface is the chair."""
    depth = np.full((180, 320), wall_mm, dtype=np.uint16)
    corners = outline.corners_of(BOX, SIZE)
    left, top, right, bottom = oak.box_for(corners, LENS, painting_mm / 1000.0)
    x0, x1 = int(left * 320), int(math.ceil(right * 320))
    y0, y1 = int(top * 180), int(math.ceil(bottom * 180))
    depth[y0:y1, x0:x1] = painting_mm
    depth[y1 - int(0.45 * (y1 - y0)):y1, x0:x1] = chair_mm
    return depth


def the_painting_alone(np, share=0.40, bbox=BOX):
    """The region finder's outline of the painting: the top of its box, which the
    chair does not reach."""
    mask = np.zeros((SIZE[1], SIZE[0]), dtype=bool)
    x0, y0, x1, y1 = outline.window(bbox, SIZE)
    mask[y0:y0 + int(share * (y1 - y0)), x0:x1] = True
    return mask


def test_an_outline_survives_being_packed() -> None:
    np = _np()
    mask = np.zeros((480, 640), dtype=bool)
    mask[150:290, 300:340] = True
    mask[200:210, 0:640] = True                   # beyond the box: cut off
    blob = outline.encode(np, mask, BOX)
    x0, y0, stride, piece = outline.decode(np, blob)
    check("the window starts at the box's corner", (x0, y0), (288, 144))
    check("...kept at half the frame's resolution", stride, outline.STRIDE)
    check("...and is exactly the mask inside the box, every other pixel",
          bool((piece == mask[144:288:2, 288:352:2]).all()), True)
    check("a few hundred bytes at most", len(blob) < 400, True)
    check("an empty outline packs to nothing",
          outline.encode(np, np.zeros((480, 640), dtype=bool), BOX), None)
    check("and bytes that do not read decode to nothing",
          outline.decode(np, b"\x00\x01garbage"), None)


def test_the_statistic_is_the_depth_services() -> None:
    """A low percentile finds the front of the thing and the band behind it is the
    answer -- not a median across the thing and whatever is behind it."""
    np = _np()
    near = np.array([1.0] * 30 + [3.0] * 70)
    check("the nearest surface, not the median of everything",
          outline.surface(np, near), (1.0, 0.0, 30))
    check("too few pixels is no answer", outline.surface(np, np.ones(11)), None)
    check("what a reading is worth is the service's formula",
          outline.sigma_m(2.0, 0.1), 0.055)


def test_the_projection_is_the_mounts_own() -> None:
    np = _np()
    for mount in (oak.MOUNT, oak.Mount(yaw_deg=3.0, pitch_deg=-2.0, roll_deg=1.5,
                                       forward_m=0.02, left_m=-0.01, up_m=0.05)):
        matrix, offset = outline.into_oak(np, mount)
        worst = 0.0
        for direction, distance in (((0.9, 0.3, 0.1), 2.0), ((0.8, -0.4, 0.3), 0.9),
                                    ((1.0, 0.0, 0.0), 4.5)):
            d = np.array(direction) / np.linalg.norm(direction)
            want = np.array(oak._in_oak(tuple(d), distance, mount))
            got = matrix @ (d * distance - offset)
            worst = max(worst, float(np.abs(want - got).max()))
        check(f"a pixel lands where oak puts it (yaw {mount.yaw_deg})", worst < 1e-9, True)


def test_a_chair_in_front_of_the_painting_is_not_its_distance() -> None:
    """The case that went wrong: the box reads the chair, the outline the painting."""
    np = _np()
    depth = a_painting_behind_a_chair(np)
    blob = outline.encode(np, the_painting_alone(np), BOX)
    boxed, drawn = outline.read(np, depth.tobytes(), 320, 180, LENS,
                                [(BOX, None), (BOX, blob)], SIZE)
    check("the box reads the chair, as the service does",
          (boxed["method"], abs(boxed["range_m"] - 1.2) < 0.1), ("box", True))
    check("the outline reads the painting",
          (drawn["method"], abs(drawn["range_m"] - 2.5) < 0.1), ("outline", True))


def test_an_outline_too_small_to_read_leaves_the_box() -> None:
    np = _np()
    depth = a_painting_behind_a_chair(np)
    sliver = np.zeros((480, 640), dtype=bool)
    sliver[150:152, 300:302] = True
    blob = outline.encode(np, sliver, BOX)
    answer = outline.read(np, depth.tobytes(), 320, 180, LENS, [(BOX, blob)], SIZE)[0]
    check("a few pixels of outline fall back to the box", answer["method"], "box")
    outside = outline.read(np, depth.tobytes(), 320, 180, LENS,
                           [([0.0, 0.0, 0.05, 0.1], blob)], SIZE)[0]
    check("and a box the depth camera cannot see says so",
          outside, {"absent": "outside the depth camera's view"})


class _DepthOf(FakeRanger):
    """A depth camera with a real-shaped map in it."""

    def __init__(self, depth):
        super().__init__(lens=LENS, answers=[[Ranged(range_m=1.2, sigma_m=0.02)]])
        self.depth = depth

    def depth_map(self):
        return DepthMap(millimetres=self.depth.tobytes(), width=320, height=180,
                        dtype="uint16", age_s=0.05)


def test_a_look_stores_the_outline_and_what_its_range_came_from() -> None:
    """End to end through one look: the outline is kept, the range is read under it,
    and the depth map kept is the one the range was read from."""
    from world_state.inspector import Inspector
    from world_state.perception_client import FakeEyes

    np = _np()
    depth = a_painting_behind_a_chair(np)
    sighting = a_sighting(bbox=list(BOX))
    sighting.outline = outline.encode(np, the_painting_alone(np), BOX)
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        ranger = _DepthOf(depth)
        Inspector(store, FakeEyes([[sighting]]), a_capture(), a_pose(heading=90.0),
                  fov_deg=100.0, ranger=ranger).inspect()
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("the look's range is the painting's", abs(row["range_m"] - 2.5) < 0.1, True)
        check("...read from its outline", row["range_from"], "outline")
        check("...which is kept with it", row["outline_blob"] == sighting.outline, True)
        check("the depth service was not asked", ranger.asked, [])
        kept, described = store.depth(row["frame_id"])
        check("the depth map kept is the one it was read from",
              (kept == depth.tobytes(), described.get("width")), (True, 320))
        store.close()

    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        ranger = FakeRanger(lens=LENS, answers=[[Ranged(range_m=1.2, sigma_m=0.02)]])
        Inspector(store, FakeEyes([[a_sighting(bbox=list(BOX))]]), a_capture(),
                  a_pose(heading=90.0), fov_deg=100.0, ranger=ranger).inspect()
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("with no depth map to read, the service reads the box, and says so",
              (row["range_from"], bool(ranger.asked)), ("service", True))
        store.close()


TESTS = (test_an_outline_survives_being_packed,
         test_the_statistic_is_the_depth_services,
         test_the_projection_is_the_mounts_own,
         test_a_chair_in_front_of_the_painting_is_not_its_distance,
         test_an_outline_too_small_to_read_leaves_the_box,
         test_a_look_stores_the_outline_and_what_its_range_came_from)
