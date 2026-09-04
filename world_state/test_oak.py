"""The second camera: its lens, where it is bolted, and what a range buys.

Two halves that fail differently. The geometry -- a pixel on the OAK becoming a
direction, a box on the gimbal camera being found in the OAK's picture, a range
measured from one lens becoming a length along a ray drawn from the other --
fails by being subtly wrong, so it is checked against directions worked out by
hand rather than against itself. The gates in `locate` fail by being *on*: a
range is a measurement most looks do not have, and a rover that refuses
everything it knew yesterday because a new column is null would be far worse than
one with no depth camera at all. So most of what follows checks that silence is
agreement.

Standalone as well as part of the selftest, because the mount it is about is
measured by a bench script rather than written down:

    python3 test_oak.py
"""
from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# The package is imported as a package, so what goes on the path is its parent --
# ~/ugv on the rover, the checkout root here. Set here rather than borrowed from
# `test_fakes`, because nothing in this file needs a store, a camera or a pose.
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from test_harness import check                                    # noqa: E402
from world_state import locate                                    # noqa: E402
from world_state import oak                                       # noqa: E402
from world_state import view                                      # noqa: E402
from world_state.depth_client import (                            # noqa: E402
    FakeRanger, Lens, Ranged, SidecarRanger, _size_of,
)

#: The lens the rover's own OAK reports for the frame it emits, read off the
#: device on 2026-09-04. Here rather than fetched because a test that needs a
#: camera on the USB bus is a test that does not run at a desk -- and because
#: what is being checked is the arithmetic, not the calibration.
OAK_LENS = Lens(fx=456.54, fy=456.43, cx=321.12, cy=189.75,
                width=640, height=360, hfov_deg=70.1, vfov_deg=43.0)


def _mounted(**fields):
    """Run something with a measured mount, and put the module back afterwards.

    `oak.MOUNT` is a module constant on purpose -- it describes this rover's
    hardware and nothing should be changing it at runtime -- so a test that needs
    one measured says so explicitly and tidies up.
    """
    class _Held:
        def __enter__(self):
            self.mount, self.measured = oak.MOUNT, oak.MEASURED
            oak.MOUNT = oak.Mount(**fields)
            oak.MEASURED = True
            return oak.MOUNT

        def __exit__(self, *_gone):
            oak.MOUNT, oak.MEASURED = self.mount, self.measured
            return False

    return _Held()


def _unmeasured():
    """And run something as a rover whose mount nobody has measured.

    The state this component shipped in and the one it has to be safe in: the
    checks below are about what happens *before* somebody runs the bench, so they
    force it rather than relying on the module still being in it.
    """
    class _Held:
        def __enter__(self):
            self.mount, self.measured = oak.MOUNT, oak.MEASURED
            oak.MOUNT, oak.MEASURED = oak.Mount(), False
            return oak.MOUNT

        def __exit__(self, *_gone):
            oak.MOUNT, oak.MEASURED = self.mount, self.measured
            return False

    return _Held()


# --- the lens ----------------------------------------------------------------

def test_a_pixel_survives_the_round_trip_through_the_mount() -> None:
    """The invariant that keeps three sign conventions honest at once.

    A direction in the rover's own frame goes into the OAK's picture as a pixel
    and comes back out as a bearing and an elevation, through `_in_oak`, the
    projection, `ray_at` and `view.chassis_direction` -- four pieces with a yaw, a
    pitch and a roll between them, each of which has a sign that can be wrong on
    its own. Checked end to end rather than piece by piece, because a pair of
    compensating sign errors passes every test that looks at one of them.

    Deliberately with a mount that is crooked in every axis at once: with any of
    the three at zero, two of the ways to get this wrong stop being visible.
    """
    with _mounted(yaw_deg=-1.53, pitch_deg=3.11, roll_deg=-2.12):
        for direction in ((1.0, 0.0, 0.0), (1.0, 0.25, 0.12),
                          (1.0, -0.30, -0.15), (1.0, 0.10, -0.20)):
            length = math.sqrt(sum(one * one for one in direction))
            unit = tuple(one / length for one in direction)
            placed = oak._in_oak(unit, 3.0)
            x_frac, y_frac = oak._project(placed, OAK_LENS)
            back = view.chassis_direction(x_frac, y_frac, oak.pan_deg(),
                                          oak.tilt_deg(), lens=OAK_LENS)
            apart = math.degrees(math.acos(min(1.0, max(-1.0, sum(
                a * b for a, b in zip(unit, back))))))
            check(f"a direction {tuple(round(one, 2) for one in unit)} comes "
                  f"back where it went in", round(apart, 3), 0.0)

    # And the roll on its own is invertible, which is the one pair of signs the
    # round trip above could hide by cancelling.
    with _mounted(roll_deg=-2.12):
        x, y = oak._rolled(0.3, -0.2)
        check("rolling and unrolling is the identity",
              tuple(round(one, 9) for one in oak._unrolled(x, y)), (0.3, -0.2))

        # **And the calibration reads the lens alone.** A bench that measured the
        # mount through the mount would report how far it had moved since the
        # last number was written down, and print that as the mount. So the two
        # have to differ by exactly the roll and by nothing else.
        raw = oak.pinhole_at(0.8, 0.3, OAK_LENS)
        drawn = oak.ray_at(0.8, 0.3, OAK_LENS)
        check("the calibration's view of a pixel does not move with the mount",
              raw, oak.pinhole_at(0.8, 0.3, OAK_LENS))
        check("...while the bearing's view of it does",
              tuple(round(one, 6) for one in drawn)
              == tuple(round(one, 6) for one in raw), False)
        check("...by the roll and nothing else",
              tuple(round(one, 9) for one in oak._rolled(drawn[0], drawn[1])),
              tuple(round(one, 9) for one in raw[:2]))


def test_a_pixel_on_the_oak_becomes_a_direction() -> None:
    """A pinhole, so the centre of the picture is straight ahead and the edge is
    half the field of view off it. Checked against the field the device reports
    rather than against the arithmetic that produced it."""
    middle = oak.ray_at(OAK_LENS.cx / OAK_LENS.width,
                        OAK_LENS.cy / OAK_LENS.height, OAK_LENS)
    check("the lens axis looks straight out of the lens",
          [round(one, 3) for one in middle], [0.0, 0.0, 1.0])

    def across(x_frac, y_frac):
        right, down, forward = oak.ray_at(x_frac, y_frac, OAK_LENS)
        return (math.degrees(math.atan2(right, forward)),
                math.degrees(math.atan2(-down, forward)))

    # With the mount's twist taken out, because this is about the lens rather
    # than about how it is bolted on: `ray_at` applies the roll, so on a rover
    # whose camera is two degrees out of true the frame's horizontal edges are
    # not horizontal and its field would not measure across them.
    with _unmeasured():
        middle_y = OAK_LENS.cy / OAK_LENS.height
        left_edge = across(0.0, middle_y)[0]
        right_edge = across(1.0, middle_y)[0]
        check("the two side edges span the field of view the device reports",
              round(right_edge - left_edge, 1), OAK_LENS.hfov_deg)

        middle_x = OAK_LENS.cx / OAK_LENS.width
        check("...and the top and bottom edges span the vertical one",
              round(across(middle_x, 0.0)[1] - across(middle_x, 1.0)[1], 1),
              OAK_LENS.vfov_deg)

    # **Not half each side, and that is the calibration rather than a slip.** The
    # device puts the principal point at (321.1, 189.8) on a 640x360 frame, which
    # is a pixel right of centre and ten below it -- so the picture reaches 22.6
    # degrees above the axis and 20.4 below. Reading either half as half the field
    # of view is the error a `hfov/2` model makes, and it is why nothing here uses
    # one.
        check("the lens axis is not the middle of the picture",
              round(across(middle_x, 0.0)[1], 1)
              == round(OAK_LENS.vfov_deg / 2, 1), False)


def test_the_oak_draws_a_bearing_through_its_own_lens() -> None:
    """The whole point of `view` taking a lens: the same box on the same rover
    means one angle through the fisheye and a different one through the OAK, and
    reading either through the other's optics is the error this prevents."""
    box = [0.70, 0.45, 0.80, 0.55]
    through_oak = view.ray({"pose": {"x_m": 0.0, "y_m": 0.0,
                                     "heading_deg": 0.0},
                            "observer_pan_deg": 0.0, "observer_tilt_deg": 0.0,
                            "bbox": box, "lens": OAK_LENS}, fov_deg=70.1)
    through_gimbal = view.ray({"pose": {"x_m": 0.0, "y_m": 0.0,
                                        "heading_deg": 0.0},
                               "observer_pan_deg": 0.0, "observer_tilt_deg": 0.0,
                               "bbox": box}, fov_deg=130.0)
    check("a box right of centre is right of the heading through either lens",
          (through_oak["bearing_deg"] < 0, through_gimbal["bearing_deg"] < 0),
          (True, True))
    check("but the narrow lens puts it much less far round",
          abs(through_oak["bearing_deg"]) < abs(through_gimbal["bearing_deg"]),
          True)
    check("and the fisheye calls the same box wider",
          through_gimbal["span_deg"] > through_oak["span_deg"], True)


def test_the_mount_is_where_an_oak_bearing_comes_from() -> None:
    """The OAK is modelled as a gimbal that never moves, so its yaw is what an
    observation stores as a pan -- which means every bearing it draws swings with
    the mount, exactly as a bearing through the gimbal swings with the servo."""
    with _mounted(yaw_deg=6.0):
        drawn = view.ray({"pose": {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0},
                          "observer_pan_deg": oak.pan_deg(),
                          "observer_tilt_deg": oak.tilt_deg(),
                          "bbox": [0.45, 0.45, 0.55, 0.55],
                          "lens": OAK_LENS}, fov_deg=70.1)
        check("a camera aimed six degrees right of the nose points there",
              round(drawn["bearing_deg"]), -6.0)


# --- where the camera is -----------------------------------------------------

def test_a_ray_from_the_oak_starts_where_the_oak_is() -> None:
    """Ten centimetres between the two lenses is three degrees of bearing at two
    metres, which is twice what the geometry is told to expect -- so the pose an
    OAK look is stored against is the OAK's own, not the rover's."""
    pose = {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0}
    with _unmeasured():
        check("an unmeasured mount moves nothing", oak.pose_at(pose), pose)

    with _mounted(forward_m=0.12, left_m=-0.03):
        moved = oak.pose_at(pose)
        check("facing north, forward is +y and left is -x",
              (moved["x_m"], moved["y_m"]), (1.03, 2.12))
        check("...and the heading is untouched", moved["heading_deg"], 90.0)


def test_the_two_cameras_agree_about_a_thing_straight_ahead() -> None:
    """The bench measures the mount by making these two agree; this checks the
    arithmetic that will spend it. A thing three metres down the rover's nose is
    dead centre of a camera bolted straight ahead, and off centre by the offset
    over the range for one that is not."""
    # A real box rather than four copies of one direction: a box with no width
    # is not a box, and `box_for` says so.
    ahead = [(1.0, 0.02, 0.02), (1.0, -0.02, 0.02),
             (1.0, 0.02, -0.02), (1.0, -0.02, -0.02)]
    with _mounted(yaw_deg=0.0, pitch_deg=0.0):
        box = oak.box_for(ahead, OAK_LENS, 3.0)
        check("a camera pointed where the rover points sees it in the middle",
              (round((box[0] + box[2]) / 2.0, 2),
               round((box[1] + box[3]) / 2.0, 2)),
              (round(OAK_LENS.cx / OAK_LENS.width, 2),
               round(OAK_LENS.cy / OAK_LENS.height, 2)))

    with _mounted(yaw_deg=0.0, pitch_deg=0.0, left_m=0.10):
        box = oak.box_for(ahead, OAK_LENS, 3.0)
        middle = (box[0] + box[2]) / 2.0
        check("a lens ten centimetres to the left sees it right of centre",
              middle > OAK_LENS.cx / OAK_LENS.width, True)
        offset = math.degrees(math.atan2(0.10, 3.0))
        seen = math.degrees(math.atan2(
            (middle * OAK_LENS.width - OAK_LENS.cx), OAK_LENS.fx))
        check("...by the offset over the range, which is the parallax",
              round(seen, 1), round(offset, 1))


def test_a_thing_the_oak_cannot_see_has_no_range() -> None:
    """The gimbal sees 122 degrees across and the OAK 70, so most of a gimbal
    frame has no depth behind it and a look over the rover's shoulder has none at
    all. Refusing is the answer; guessing would put a range on a ray pointed
    somewhere the camera never looked."""
    with _mounted():
        behind = [(-1.0, 0.0, 0.0)] * 4
        check("nothing behind the camera is in its picture",
              oak.box_for(behind, OAK_LENS, 2.0), None)
        beside = [(0.3, 0.95, 0.0)] * 4
        check("nor anything seventy degrees off its axis",
              oak.box_for(beside, OAK_LENS, 2.0), None)
        check("and a box with no width in it is not a box",
              oak.box_for([(1.0, 0.0, 0.0)] * 4, OAK_LENS, 2.0), None)


def test_an_unmeasured_mount_finds_nothing_rather_than_guessing() -> None:
    """**The one failure that would poison the store.** A yaw taken by eye off a
    bracket is worth about five degrees against a bearing believed to one and a
    half, and nothing downstream could detect it -- so an unmeasured mount does
    nothing at all rather than something approximate."""
    with _unmeasured():
        check("no box without a mount",
              oak.box_for([(1.0, 0.02, 0.02), (1.0, -0.02, 0.02),
                           (1.0, 0.02, -0.02), (1.0, -0.02, -0.02)],
                          OAK_LENS, 2.0), None)
        check("no range correction either",
              oak.range_from_gimbal([(1.0, 0.0, 0.0)] * 4, 2.0), None)
        check("and it says so out loud", "unmeasured" in oak.describe(), True)


def test_a_range_measured_from_one_lens_becomes_a_length_along_the_other() -> None:
    """A range is a length along a particular ray from a particular point. The
    OAK measures from itself; the observation's ray starts at the gimbal camera,
    and putting one on the other unchanged is wrong by the offset between them --
    a little at four metres and a lot at half of one."""
    with _mounted(forward_m=0.15):
        ahead = [(1.0, 0.0, 0.0)] * 4
        check("a lens fifteen centimetres ahead measures fifteen less",
              round(oak.range_from_gimbal(ahead, 2.0), 3), 2.15)
        check("...which is the same fifteen at any distance, head on",
              round(oak.range_from_gimbal(ahead, 0.5), 3), 0.65)

    with _mounted(left_m=0.15):
        # Sideways, the two lenses and the thing make a right-angled triangle
        # rather than a straight line, so the correction is the small one:
        # sqrt(2^2 - 0.15^2).
        check("a lens off to one side barely changes a head-on range",
              round(oak.range_from_gimbal(ahead, 2.0), 2), 1.99)
        check("and a range shorter than the two lenses are apart describes "
              "nothing", oak.range_from_gimbal(ahead, 0.10), None)


# --- what a range buys -------------------------------------------------------

def _placed(**fields):
    point = {"x_m": 2.0, "y_m": 0.0, "uncertainty_m": 0.1, "extent_m": 0.1,
             "error_major_m": 0.1, "error_minor_m": 0.1, "error_major_deg": 0.0}
    point.update(fields)
    return point


def _ray(**fields):
    ray = {"x_m": 0.0, "y_m": 0.0, "bearing_deg": 0.0, "span_deg": 6.0}
    ray.update(fields)
    return ray


def test_a_ray_with_no_range_says_nothing_about_the_distance() -> None:
    """**Silence is agreement.** Every look this rover took before it read the
    depth camera has no range, and so does every look since taken where the OAK
    was not pointing. A gate that refused those would empty the store."""
    check("no range, no opinion",
          locate.stands_at_range(_placed(), _ray()), True)
    check("a range of nothing is not a range of zero",
          locate.stands_at_range(_placed(), _ray(range_m=None)), True)
    check("and neither is something unreadable",
          locate.stands_at_range(_placed(), _ray(range_m="near")), True)
    check("two rays that measured nothing do not disagree",
          locate.range_disagreement(_placed(), _ray(), _ray()), None)
    check("nor does one that did against one that did not",
          locate.range_disagreement(_placed(), _ray(range_m=2.0), _ray()), None)


def test_a_look_joins_a_thing_only_at_the_distance_it_measured() -> None:
    """The axis a bearing cannot see. A ray pointed straight at a placed thing
    agrees with it perfectly in every angle and still says, in millimetres, that
    what it was looking at was somewhere else."""
    point = _placed()
    check("a range that agrees with where the thing is, joins",
          locate.stands_at_range(point, _ray(range_m=2.0, range_sigma_m=0.05)),
          True)
    check("a range half a metre short does not",
          locate.stands_at_range(point, _ray(range_m=1.2, range_sigma_m=0.05)),
          False)
    check("...and a bearing alone would have taken it",
          locate.agrees(point, _ray()), True)
    check("but with the range on it, it does not",
          locate.agrees(point, _ray(range_m=1.2, range_sigma_m=0.05)), False)


def test_a_thing_is_forgiven_its_own_depth() -> None:
    """A depth camera measures the front of a thing and a placement is its
    middle, so the two differ by half its depth before anything has gone wrong.
    A sideboard reads nearer than where the crossing puts it, every time."""
    narrow = _placed(extent_m=0.05)
    wide = _placed(extent_m=0.60)
    near = _ray(range_m=1.65, range_sigma_m=0.05)
    check("a thin thing measured 35 cm nearer than it sits is a different thing",
          locate.stands_at_range(narrow, near), False)
    check("a wide one is the same thing seen from the front",
          locate.stands_at_range(wide, near), True)
    check("and the allowance grows with the thing rather than with the ray",
          locate.range_tolerance_m(wide, near)
          > locate.range_tolerance_m(narrow, near), True)


def test_a_bad_range_is_worth_less_than_a_good_one() -> None:
    """The depth camera reports what each reading is worth, because stereo error
    grows with the square of the distance and with how ragged the surface was.
    A reading that says so is allowed to miss by more."""
    point = _placed()
    tight = locate.range_tolerance_m(point, _ray(range_m=2.0,
                                                 range_sigma_m=0.02))
    loose = locate.range_tolerance_m(point, _ray(range_m=2.0,
                                                 range_sigma_m=0.40))
    check("a ragged reading buys a wider allowance", loose > tight + 0.35, True)
    check("and one that does not say falls back to the constant",
          round(locate.range_tolerance_m(point, _ray(range_m=2.0))
                - locate.range_tolerance_m(point,
                                           _ray(range_m=2.0,
                                                range_sigma_m=locate.RANGE_SIGMA_M)),
                6), 0.0)


def test_two_bearings_at_two_different_things_no_longer_cross() -> None:
    """**The phantom, and it is what the depth camera was wanted for.** Two rays
    aimed at two different chairs meet at a point that is on neither of them, at
    a healthy parallax off a healthy baseline, and every guard in `locate`
    accepted it -- while both rays said in millimetres that what they were
    looking at was somewhere else."""
    left = _ray(x_m=0.0, y_m=0.0, bearing_deg=45.0)
    right = _ray(x_m=4.0, y_m=0.0, bearing_deg=135.0)
    crossing = locate.fix(left, right)
    check("two bearings cross wherever they cross",
          None if crossing is None else (round(crossing["x_m"]),
                                         round(crossing["y_m"])),
          (2.0, 2.0))

    honest = locate.fix(dict(left, range_m=2.83, range_sigma_m=0.05),
                        dict(right, range_m=2.83, range_sigma_m=0.05))
    check("two rays that agree about the distance still cross",
          honest is not None, True)

    phantom = locate.fix(dict(left, range_m=1.20, range_sigma_m=0.05),
                         dict(right, range_m=1.20, range_sigma_m=0.05))
    check("two that were both looking at something nearer do not",
          phantom, None)

    half = locate.fix(dict(left, range_m=2.83, range_sigma_m=0.05),
                      dict(right, range_m=1.20, range_sigma_m=0.05))
    check("and one right range does not rescue one wrong one", half, None)


def test_a_range_pins_the_axis_the_bearings_leave_open() -> None:
    """A bearing constrains the direction and says nothing at all about the
    distance, so a fit over bearings alone is precise across the line of sight
    and loose along it. One range constrains exactly that axis, which is why it
    is a residual in the fit as well as a gate on the crossing."""
    bearing_only = locate.residuals(2.0, 0.0, _ray())
    check("a ray with no range contributes one term", len(bearing_only), 1)
    ranged = locate.residuals(2.0, 0.0, _ray(range_m=2.0, range_sigma_m=0.05))
    check("one with a range contributes two", len(ranged), 2)
    check("and the range term is zero where the range is right",
          round(ranged[1][0], 6), 0.0)

    wrong = locate.residuals(2.0, 0.0, _ray(range_m=1.5, range_sigma_m=0.05))
    check("a half-metre miss at a 5 cm sigma is ten sigma out",
          round(wrong[1][0]), 10)


def test_a_stale_range_is_worth_less_on_a_moving_rover() -> None:
    """**A range is true of where the camera was when the frame was taken.** The
    depth camera holds each frame until the picture it belongs with has come
    through the encoder, so a reading is about two thirds of a second old when it
    is read -- thirty centimetres at the speed the rover explores at, against a
    stereo error of two to seven. Standing still it costs nothing."""
    from world_state.inspector import Inspector, _speed

    fresh = Ranged(range_m=2.0, sigma_m=0.04, age_s=0.65)
    check("a parked rover is charged nothing for a stale frame",
          Inspector._aged_sigma(fresh, 0.0), 0.04)
    moving = Inspector._aged_sigma(fresh, 0.47)
    check("a driving one is charged what it covered meanwhile",
          round(moving, 2), round(math.hypot(0.04, 0.47 * 0.65), 2))
    check("which is the larger term by a long way", moving > 0.29, True)
    check("and it only ever widens", moving > 0.04, True)

    check("speed comes off the bracket that already measures it",
          round(_speed(0.17, 100.0, 100.36), 2), 0.47)
    check("and an untimed bracket claims none of it",
          _speed(0.17, None, None), 0.0)


# --- the wire ----------------------------------------------------------------

def test_a_depth_camera_that_is_not_there_is_an_ordinary_answer() -> None:
    """The caller is an inspection inside the process that owns STOP. A camera
    that has been unplugged, a service restarting and a reply that is not JSON
    all have to come back as words rather than as an exception."""
    ranger = SidecarRanger(url="http://127.0.0.1:9")
    check("no lens from a port with nothing on it", ranger.lens(), None)
    check("...and it says it is unavailable", ranger.available()[0], False)
    frame = ranger.frame()
    check("no frame either", frame.ok, False)
    check("with a sentence rather than a stack trace",
          bool(frame.error) and "Error" in frame.error
          or "refused" in frame.error.lower(), True)
    answers, error = ranger.ranges([[0.1, 0.1, 0.2, 0.2]])
    check("and no ranges", (answers, bool(error)), ([], True))


def test_an_answer_that_is_short_is_padded_rather_than_misaligned() -> None:
    """A caller lines these up with the regions it asked about, one for one. A
    short list would quietly put the second thing's range on the third thing,
    which nothing downstream could notice."""
    ranger = FakeRanger(answers=[[Ranged(range_m=1.5, sigma_m=0.02)]])
    answers, error = ranger.ranges([[0, 0, 1, 1], [0, 0, 1, 1], [0, 0, 1, 1]])
    check("three boxes get three answers", (len(answers), error), (3, ""))
    check("the one that was measured keeps its range", answers[0].range_m, 1.5)
    check("and the rest abstain",
          [one.range_m for one in answers[1:]], [None, None])


def test_the_frame_header_says_how_far_apart_the_two_halves_were() -> None:
    """A picture and a set of ranges half a second apart on a moving rover are
    two different rooms, and a consumer has to be able to see that rather than
    assume it away."""
    check("a size header reads as two numbers", _size_of("640x360"), (640, 360))
    check("and a missing one as none at all", _size_of(None), (0, 0))


TESTS = (
    test_a_pixel_survives_the_round_trip_through_the_mount,
    test_a_pixel_on_the_oak_becomes_a_direction,
    test_the_oak_draws_a_bearing_through_its_own_lens,
    test_the_mount_is_where_an_oak_bearing_comes_from,
    test_a_ray_from_the_oak_starts_where_the_oak_is,
    test_the_two_cameras_agree_about_a_thing_straight_ahead,
    test_a_thing_the_oak_cannot_see_has_no_range,
    test_an_unmeasured_mount_finds_nothing_rather_than_guessing,
    test_a_range_measured_from_one_lens_becomes_a_length_along_the_other,
    test_a_ray_with_no_range_says_nothing_about_the_distance,
    test_a_look_joins_a_thing_only_at_the_distance_it_measured,
    test_a_thing_is_forgiven_its_own_depth,
    test_a_bad_range_is_worth_less_than_a_good_one,
    test_two_bearings_at_two_different_things_no_longer_cross,
    test_a_range_pins_the_axis_the_bearings_leave_open,
    test_a_stale_range_is_worth_less_on_a_moving_rover,
    test_a_depth_camera_that_is_not_there_is_an_ordinary_answer,
    test_an_answer_that_is_short_is_padded_rather_than_misaligned,
    test_the_frame_header_says_how_far_apart_the_two_halves_were,
)


def main() -> int:
    """Standalone, so this can be run before the selftest knows about it."""
    from test_harness import FAIL, PASS

    for one in TESTS:
        try:
            one()
        except Exception as error:                                # noqa: BLE001
            FAIL.append(f"{one.__name__} raised {type(error).__name__}: {error}")
    for line in FAIL:
        print("FAIL " + line)
    print(f"{len(PASS)} passed, {len(FAIL)} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
