"""The OAK as a second camera on this rover: where it is, and what its pixels see.

The rover has two cameras that point at the room and they are nothing alike. The
gimbal camera is a 130-degree fisheye on two servos, swept and fitted by
`usb_cameras/calibrate_fov.py`, and it is the one every bearing this component has
ever recorded was drawn through. The OAK sees 65 degrees of the room and is the
only thing on the rover that knows **how far away** what it is looking at is.

**Since 2026-09-30 the OAK rides the gimbal**, clamped to the rail on its tilt
platform by the printed mount in `cad/`, beside the fisheye and looking the same
way. Until then it was bolted to the chassis and could look nowhere else. The
move is what this module's shape follows from: the two cameras now turn together,
so where one of them is relative to the other is a single fixed transform, and
the servos -- whose pointing faults are recorded in
`docs/requirements/world-state.md#r-ws-10` -- no longer come between them.

This module is what lets the second one write into the world the first one built.
Three things have to be true for that, and each is a section below:

* **its pixels have to become directions**, which is a lens, and the lens is the
  device's own -- fetched by `depth_client`, never written down here;
* **it has to be somewhere**, which is `MOUNT`: where the OAK sits and which way
  it faces relative to the gimbal camera. Until that is measured this camera
  cannot draw a bearing at all, and `MEASURED` says so;
* **a box drawn on the other camera's picture has to be findable in this one**,
  which is `box_for`, and which is what lets a look taken through the gimbal
  carry a range without being taken through the OAK.

**A look through the OAK is a look through the gimbal with a different lens.**
`ray_at` turns an OAK pixel into a direction in the gimbal camera's own frame, so
an observation from it records the gimbal's pan and tilt like any other and
`view.ray` turns it into a bearing with the code it already had.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

#: What the store writes in an observation's `camera` column, and what the
#: capture dictionary carries so that everything downstream knows which optics a
#: box was drawn through. Two values and no more: the column exists to keep the
#: two cameras' bearings from being read through each other's lens.
GIMBAL = "gimbal"
OAK = "oak"


@dataclass(frozen=True)
class Mount:
    """Where the OAK is, relative to the gimbal camera.

    **In the gimbal camera's own frame, which turns with the gimbal.** At pan 0
    and tilt 0 that frame is the chassis frame, which is where the mount is
    measured and why the fields are named the way the chassis is: `forward_m`
    along the fisheye's axis, `left_m` to its left, `up_m` above it. At any other
    pan and tilt the whole transform turns with the platform -- which is the
    point of the rail -- and `pose_at`, `rise_of` and `camera_frame` are where
    that turning is done.

    Angles first, because they are what a bearing is made of: `yaw_deg` positive
    to the **right**, which is the gimbal's convention and the opposite of the
    map's, and `pitch_deg` positive **up**, which is the gimbal's tilt. Applied
    after the gimbal's own pan and tilt, so a wrong yaw here swings every bearing
    this camera records by the same amount.

    The offset is what makes the two cameras agree about a thing a metre away
    rather than about a thing at infinity: five centimetres of it is three
    degrees at a metre, which is twice what the geometry is told to expect from a
    bearing.

    **Relative to the gimbal camera and not to the rover's centre, deliberately.**
    Where the gimbal camera itself sits relative to the pose SLAM reports has
    never been measured on this rover, and every bearing in the store already
    carries that error. Expressing this one relative to the gimbal camera means
    the two cameras agree with *each other*, which is the thing that matters when
    both write into one world, and leaves the common unmeasured offset exactly
    where it already was.
    """

    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    #: And how far the camera is twisted about its own optical axis, positive
    #: the way a rotation from its x axis towards its y axis goes -- clockwise
    #: in the picture. Carried because a roll mixes a ray's bearing into its
    #: elevation and back, by the roll times how far off the axis the ray is,
    #: and on the chassis bracket it was two degrees.
    roll_deg: float = 0.0
    forward_m: float = 0.0
    left_m: float = 0.0
    up_m: float = 0.0
    #: Whether the OAK turns with the gimbal. False only for `CHASSIS_MOUNT`,
    #: the bracket it was on until 2026-09-30, which is kept so that depth maps
    #: recorded then can still be read.
    on_gimbal: bool = True


#: Where this rover's OAK is, measured on 2026-09-30 by `bench_oak.py --joint` in
#: two runs at fifteen gimbal positions between them -- pan -30, 0 and +30 at tilt
#: 0, 20 and 40, then pan -45, 0 and +45 at tilt 10 and 30 -- from 1417 feature
#: matches between the two cameras, ranged by the OAK itself, 2 to 6 m out.
#:
#:     component    adopted       first run   second run
#:     yaw         +1.92 deg       +1.89       +1.98
#:     pitch       +0.80 deg       +0.73       +0.92
#:     roll        -0.80 deg       -0.82       -0.71
#:     forward      0.000 m        held: the two sensors are in one plane
#:     left        -0.005 m        -0.007      -0.002
#:     up          +0.046 m        +0.051      +0.038
#:
#: Read the two runs, not the resampling intervals, as what these are good to:
#: the intervals within each run are half the width of the difference between
#: them, because the height and the pitch trade against each other through
#: whatever ranges the room offered. About a tenth of a degree, and a centimetre
#: of height.
#:
#: **The rail is rigid, and that is the measurement that says so.** Fitted
#: position by position with the offset held, the fifteen rotations agree to 0.13
#: degrees of yaw, with no trend in tilt, which is what a clamp sagging under the
#: camera's weight would show first. On the chassis bracket the same bench moved
#: by 4.5 degrees of yaw with the gimbal, because the gimbal's pointing faults
#: were in between.
#:
#: **The forward offset is the owner's, not the fit's.** The OAK was mounted with
#: its sensors in the plane of the fisheye's; the fit, left free, puts it 19 mm
#: ahead, but forward is the one direction this bench cannot separate from the
#: fisheye's angular scale (below), and holding it at nothing changes nothing
#: else by more than a tenth of a degree or 2 mm.
#:
#: **The fisheye's own lens is the largest term left, and it is not the mount's.**
#: The fit only closes -- a median of 0.23 degrees over all fifteen positions --
#: once the fisheye's angles off its axis are stretched by 7.2% (1.0719 and
#: 1.0716 in the two runs), meaning
#: `face_tracking/lens.py` puts a thing 30 degrees from the middle of the picture
#: at 28. The board calibration of 2026-09-07, which never trusted a servo, says
#: 5 to 7%. That is recorded against R-WS-10 and left for its own change; until
#: it is made, a box mapped from the fisheye onto this camera is off by about 7%
#: of how far it sits from the middle of the picture -- nothing at the centre, two
#: degrees at the OAK's edge. See `docs/progress/2026-09-30-oak-on-the-gimbal.md`.
MOUNT = Mount(
    yaw_deg=1.92,
    pitch_deg=0.80,
    roll_deg=-0.80,
    forward_m=0.0,
    left_m=-0.005,
    up_m=0.046,
)

#: Where the OAK was until 2026-09-30: bolted to the chassis, measured 2026-09-07
#: by `usb_cameras/calibrate_oak_mount.py` against a printed board with the gimbal
#: at pan 0 and tilt 0. Kept only for reading what the rover recorded then --
#: every depth map saved beside a look before `RAIL_SINCE` was taken through it.
#:
#: **Its forward offset is corrected here, and the correction is a measurement.**
#: The board fit read 87 mm at one target distance and 100 mm at another, a
#: systematic 12.4 mm nobody could account for; the OAK's lens as depthai
#: reported it was 9.6% short in focal length (see `oak_depth/colour_lens.py`),
#: which puts the board 9.6% too close to the OAK and the OAK too far forward by
#: 9.6% of the distance. Undone, the two distances agree on 42 and 44 mm, and a
#: tape had said 40. The angles and the other two offsets do not depend on the
#: focal length to first order and are as measured.
CHASSIS_MOUNT = Mount(
    yaw_deg=1.492,
    pitch_deg=6.256,
    roll_deg=-1.200,
    forward_m=0.043,
    left_m=-0.0031,
    up_m=-0.0937,
    on_gimbal=False,
)

#: When the OAK moved, as a Unix time: the rover's first boot with it on the rail,
#: 2026-09-30 14:22:24 +05:30. The rover was off from 2026-09-08 19:54 until then,
#: so nothing it recorded falls between the two mounts.
RAIL_SINCE = 1790758344.0


def mount_at(when: float | None) -> Mount:
    """The mount the OAK was on at this Unix time; the current one if unsaid."""
    if when is not None and float(when) < RAIL_SINCE:
        return CHASSIS_MOUNT
    return MOUNT


#: Whether `MOUNT` above holds measurements. **Everything this module can do is
#: gated on it**, in both directions: the OAK cannot draw a bearing without it,
#: and a box drawn on the gimbal camera cannot be found in the OAK's picture
#: without it either, so an unmeasured rover simply records what it always
#: recorded and says nothing about range.
#:
#: A flag rather than a check for zeros, because zero is a perfectly possible
#: measurement -- a camera mounted straight ahead has a yaw of zero, and the
#: difference between "measured as zero" and "never measured" is the whole point.
MEASURED = True

#: How far off the OAK's own axis a direction may lie before it is not in its
#: picture at all, as a fraction of the frame beyond the edge. A box mapped from
#: the gimbal camera lands wherever it lands, and plenty land outside: the OAK
#: takes in the middle half of the fisheye's picture across and two fifths of it
#: down, whichever way the gimbal points.
#:
#: A little beyond the edge is still allowed because a box that runs off the side
#: of the OAK's picture still has pixels inside it, and the part that is inside is
#: a perfectly good measurement of the part of the thing the camera can see.
#: Entirely outside is refused.
EDGE_SLACK = 0.02

#: What to assume a thing's range is while working out where to *look* for it,
#: in metres, before anything has been measured. Only the parallax between the
#: two cameras depends on it, and that is a few centimetres over a couple of
#: metres, so this only has to be the right order -- the answer is then computed
#: again with the range that came back. See `box_for`.
GUESS_RANGE_M = 2.5


# --- turning a direction by a gimbal-like pair of angles ----------------------


def _turn(vector: tuple[float, float, float], yaw_right_deg: float,
          pitch_up_deg: float) -> tuple[float, float, float]:
    """A forward-left-up vector turned the way a gimbal turns its camera.

    Pitched up about its own horizontal first and then yawed right about the
    vertical, which is the order a pan-tilt head composes them in: the pan
    carries the tilt axis round with it. Used twice over -- the mount within the
    gimbal camera's frame, and the gimbal within the chassis.
    """
    forward, left, up = vector
    pitch = math.radians(pitch_up_deg or 0.0)
    forward, up = (forward * math.cos(pitch) - up * math.sin(pitch),
                   forward * math.sin(pitch) + up * math.cos(pitch))
    # Right is positive for the gimbal and left for the map -- the same swap
    # `view.ray` makes for the pan.
    yaw = math.radians(-(yaw_right_deg or 0.0))
    return (forward * math.cos(yaw) - left * math.sin(yaw),
            forward * math.sin(yaw) + left * math.cos(yaw),
            up)


def _unturn(vector: tuple[float, float, float], yaw_right_deg: float,
            pitch_up_deg: float) -> tuple[float, float, float]:
    """`_turn` undone: the yaw taken off first, then the pitch."""
    x, y, z = vector
    yaw = math.radians(-(yaw_right_deg or 0.0))
    forward = x * math.cos(yaw) + y * math.sin(yaw)
    left = -x * math.sin(yaw) + y * math.cos(yaw)
    pitch = math.radians(pitch_up_deg or 0.0)
    return (forward * math.cos(pitch) + z * math.sin(pitch),
            left,
            -forward * math.sin(pitch) + z * math.cos(pitch))


def camera_frame(vector: tuple[float, float, float], pan_deg: float | None,
                 tilt_deg: float | None) -> tuple[float, float, float]:
    """A chassis-frame vector, as the gimbal camera saw it at this pan and tilt.

    The frame `box_for`, `range_from_gimbal` and `_in_oak` work in, for a mount
    that rides the gimbal: whatever the servos did, the OAK stands in the same
    place relative to this frame. A missing angle is taken as rest's zero, which
    is also what a look with no gimbal angle has had its bearing withheld for.
    """
    return _unturn(vector, pan_deg or 0.0, tilt_deg or 0.0)


# --- the lens -------------------------------------------------------------------


def ray_at(x_frac: float, y_frac: float, lens: Any) -> tuple[float, float, float]:
    """Where a point in the OAK's picture looks, **in the gimbal camera's frame**:
    x right, y down, z out of the fisheye's lens.

    The same convention `face_tracking/lens.ray_at` answers in, and the reason it
    answers in the other camera's frame rather than its own: an OAK look records
    the gimbal's pan and tilt exactly as a fisheye look does, so `view` can turn
    either camera's answer into a bearing with one piece of code. The mount's
    whole rotation -- roll, pitch and yaw -- is taken out here, and the gimbal's
    is taken out by `view` afterwards.

    A pinhole, and honestly one rather than for convenience -- see
    `depth_client.Lens`, where the measured reason is written down: this lens's
    rational distortion model has its numerator and denominator terms within a
    tenth of each other and they very nearly cancel.
    """
    right, down, along = pinhole_at(x_frac, y_frac, lens)
    right, down = _unrolled(right, down)
    forward, left, up = _turn((along, -right, -down),
                              MOUNT.yaw_deg, MOUNT.pitch_deg)
    return -left, -up, forward


def pinhole_at(x_frac: float, y_frac: float,
               lens: Any) -> tuple[float, float, float]:
    """The same pixel, through the lens alone and **not** through the mount.

    `ray_at` is what everything that draws a bearing wants: the direction with
    the mount taken out, so this camera can be treated as the gimbal camera with
    another lens. This is what the *calibration* wants, and the difference is
    the whole reason it is a separate function: a bench that measured the mount
    through `ray_at` would be measuring how far the mount has moved since the
    last time somebody wrote a number down, and would print that as if it were
    the mount. The one it prints has to be absolute.
    """
    x = (x_frac * lens.width - lens.cx) / lens.fx
    y = (y_frac * lens.height - lens.cy) / lens.fy
    length = math.sqrt(x * x + y * y + 1.0)
    return x / length, y / length, 1.0 / length


def _rolled(x: float, y: float, mount: Mount | None = None) -> tuple[float, float]:
    """Turn a direction from the frame the yaw and pitch live in into the
    sensor's own, which is the one a pixel is measured in."""
    roll = math.radians((mount or MOUNT).roll_deg)
    return (x * math.cos(roll) - y * math.sin(roll),
            x * math.sin(roll) + y * math.cos(roll))


def _unrolled(x: float, y: float, mount: Mount | None = None) -> tuple[float, float]:
    """And back the other way. The inverse of `_rolled` by construction, which is
    what `test_oak` checks rather than trusting the two signs to stay in step."""
    roll = math.radians((mount or MOUNT).roll_deg)
    return (x * math.cos(roll) + y * math.sin(roll),
            -x * math.sin(roll) + y * math.cos(roll))


def pose_at(pose: dict[str, Any] | None, pan_deg: float | None = None,
            tilt_deg: float | None = None) -> dict[str, Any] | None:
    """The rover's pose moved to where the OAK's optical centre actually is.

    **A ray has to start where the camera is**, and this camera is not where the
    other one is. The store keeps the pose an observation was taken from and
    `locate` treats it as the origin of the ray, so an OAK look whose pose was
    the gimbal camera's would put every crossing out by the offset between them
    -- and worse, the range the OAK measured is from the OAK, so the range and
    the origin would be describing different points.

    The offset lives in the gimbal camera's frame, so it is turned by the tilt
    and the pan the look was taken at, and then by the heading onto the map: the
    rover's nose points along `(cos h, sin h)` and its left along
    `(-sin h, cos h)`. Five centimetres above the lens is a centimetre and a half
    forward at a tilt of 20 and nothing sideways at any pan, which is why this is
    small -- and why it is done anyway rather than assumed.

    What is left vertical is not spent here and is not lost either: the map is
    flat, and what reads a height is `locate.rise_m` -- see `rise_of`.
    """
    if not isinstance(pose, dict) or not MEASURED:
        return pose
    offset = (MOUNT.forward_m, MOUNT.left_m, MOUNT.up_m)
    if not any(offset):
        return pose
    if MOUNT.on_gimbal:
        offset = _turn(offset, pan_deg or 0.0, tilt_deg or 0.0)
    try:
        heading = math.radians(float(pose.get("heading_deg", 0.0)))
        x_m, y_m = float(pose["x_m"]), float(pose["y_m"])
    except (KeyError, TypeError, ValueError):
        return pose
    forward, left, _up = offset
    return {**pose,
            "x_m": round(x_m + forward * math.cos(heading)
                         - left * math.sin(heading), 3),
            "y_m": round(y_m + forward * math.sin(heading)
                         + left * math.cos(heading), 3)}


# --- finding the other camera's box in this one -------------------------------


def box_for(corners: list[tuple[float, float, float]], lens: Any,
            range_m: float | None = None
            ) -> tuple[list[float], float] | None:
    """Where a thing the gimbal camera saw would be in the OAK's picture, and how
    far the OAK's answer would then be from the gimbal camera.

    `corners` are the box's four corners as directions in the **gimbal camera's
    own frame** -- x along its axis, y to its left, z up -- which is what
    `view.chassis_direction` answers at pan 0 and tilt 0. They arrive that way
    rather than as pixels because the two cameras share nothing else: a fisheye
    pixel and a pinhole pixel are not comparable, and the direction between them
    is. And in that frame rather than the rover's because the OAK turns with the
    gimbal: the answer is the same wherever the gimbal is pointed, and no servo
    error enters it.

    None when the thing is not in this camera's picture at all, which is the
    ordinary case rather than a failure -- the OAK takes in the middle half of
    the fisheye's picture across and two fifths of it down, so a box near the
    fisheye's edge has no depth behind it whichever way the gimbal points.

    **The offset between the two cameras is what makes this more than a
    rotation, and it is why a range goes in as well as coming out.** The two
    lenses are a few centimetres apart, so they see a thing two metres away in
    slightly different directions -- and how different depends on how far away it
    is, which is the thing being asked. So the box is worked out once at a
    guessed range, and the caller comes back with the range that produced and
    asks again. Two passes is enough: the correction is a few centimetres and the
    second pass moves the box by a fraction of a pixel.

    A box, in fractions of the OAK's picture, and nothing else. Turning the range
    that comes back into a length along the gimbal camera's own ray is
    `range_from_gimbal`, which needs no guess at all.
    """
    if not MEASURED or lens is None or not corners:
        return None
    assumed = GUESS_RANGE_M if range_m is None else max(0.05, float(range_m))
    seen = []
    for direction in corners:
        placed = _in_oak(direction, assumed)
        if placed is None:
            return None
        seen.append(_project(placed, lens))
    left = min(x for x, _ in seen)
    right = max(x for x, _ in seen)
    top = min(y for _, y in seen)
    bottom = max(y for _, y in seen)
    if (right < -EDGE_SLACK or left > 1.0 + EDGE_SLACK
            or bottom < -EDGE_SLACK or top > 1.0 + EDGE_SLACK):
        return None
    box = [max(0.0, left), max(0.0, top), min(1.0, right), min(1.0, bottom)]
    if box[2] - box[0] <= 0.0 or box[3] - box[1] <= 0.0:
        return None
    return box


def range_from_gimbal(corners: list[tuple[float, float, float]],
                      oak_range_m: float) -> float | None:
    """How far a thing the OAK measured at `oak_range_m` is from the gimbal camera.

    **A range is a length along a particular ray from a particular point**, and
    this one was measured from the other lens. The observation it is about to be
    stored on carries a ray that starts at the gimbal camera, so the number has to
    be converted rather than copied -- a few centimetres at three metres, and a
    good share of the answer at half of one.

    Exactly, and with no guess in it: the thing lies somewhere along the gimbal
    camera's own direction, and it lies on a sphere of radius `oak_range_m` about
    the OAK's lens. Where a line meets a sphere is a quadratic, so this is the far
    root of one -- which is also why `box_for` above may guess about *where to
    look* without that guess reaching the answer. `corners` are in the gimbal
    camera's own frame, the same as `box_for`'s, which is the frame the offset is
    written in.

    None when the mount is unmeasured, and None when the sphere does not reach the
    line at all: a range shorter than the distance between the two lenses,
    measured across them, describes nothing the gimbal camera could have been
    looking at.
    """
    if not MEASURED or not corners:
        return None
    direction = _middle(corners)
    towards = (MOUNT.forward_m * direction[0] + MOUNT.left_m * direction[1]
               + MOUNT.up_m * direction[2])
    apart = (MOUNT.forward_m ** 2 + MOUNT.left_m ** 2 + MOUNT.up_m ** 2)
    under = towards * towards - apart + float(oak_range_m) ** 2
    if under < 0.0:
        return None
    found = towards + math.sqrt(under)
    return found if found > 0.0 else None


def _middle(corners: list[tuple[float, float, float]]
            ) -> tuple[float, float, float]:
    """The average of these directions, renormalised. The box's own axis."""
    x = sum(one[0] for one in corners) / len(corners)
    y = sum(one[1] for one in corners) / len(corners)
    z = sum(one[2] for one in corners) / len(corners)
    length = math.sqrt(x * x + y * y + z * z) or 1.0
    return x / length, y / length, z / length


def _in_oak(direction: tuple[float, float, float], range_m: float,
            mount: Mount | None = None) -> tuple[float, float, float] | None:
    """A direction from the gimbal camera, as a direction in the OAK's optical
    frame.

    The direction is in the gimbal camera's own frame for a mount that rides the
    gimbal, and in the chassis frame for `CHASSIS_MOUNT` -- in both cases the
    frame the mount's own numbers are written in, which is what makes it one
    function. The point the gimbal camera is looking at is `range_m` along
    `direction` from its lens, which is that frame's origin by construction.
    Where the point lies from the OAK is the same point less the OAK's own
    position, turned by the mount's yaw and pitch.

    None when the point ends up behind this camera, which a thing over the
    rover's shoulder is.
    """
    mount = mount or MOUNT
    point = (direction[0] * range_m - mount.forward_m,
             direction[1] * range_m - mount.left_m,
             direction[2] * range_m - mount.up_m)
    along, left, up = _unturn(point, mount.yaw_deg, mount.pitch_deg)
    if along <= 1e-6:
        return None
    # Into the lens's own axes: x right, y down, z out -- and twisted by the
    # mount's roll, because that is the frame a pixel is measured in and
    # `_project` is a plain pinhole.
    right, down = _rolled(-left, -up, mount)
    return right, down, along


def _project(direction: tuple[float, float, float],
             lens: Any) -> tuple[float, float]:
    """A direction in the OAK's optical frame, as a fraction of its picture."""
    x, y, z = direction
    return ((lens.cx + lens.fx * x / z) / lens.width,
            (lens.cy + lens.fy * y / z) / lens.height)


def rise_of(camera: str | None, tilt_deg: float | None = None) -> float:
    """How high this camera's optical centre sits above the height datum, in metres.

    The datum is the gimbal camera, so its own answer is zero by construction and
    the OAK's is where the mount puts it at the tilt the look was taken at --
    five centimetres higher when level, a little less tilted, because the offset
    turns with the platform. This is the vertical half of the offset that
    `pose_at` deliberately does not spend: the map is flat, so the horizontal
    part belongs on the pose and the vertical part on the height.

    An unmeasured mount, or a camera nobody has heard of, is zero rather than a
    refusal. Zero is what a look through the gimbal deserves and what every look
    on this rover so far was, and a datum offset guessed wrong is worse than one
    not applied -- it would move heights that are currently right.
    """
    if camera != OAK or not MEASURED:
        return 0.0
    if not MOUNT.on_gimbal:
        return float(MOUNT.up_m)
    tilt = math.radians(tilt_deg or 0.0)
    return float(MOUNT.forward_m * math.sin(tilt) + MOUNT.up_m * math.cos(tilt))


def describe() -> str:
    """One line for the diagnostics row: where this camera is, or that nobody knows."""
    if not MEASURED:
        return "oak mount unmeasured -- run bench_oak.py"
    where = "on the gimbal rail" if MOUNT.on_gimbal else "on the chassis"
    return (f"oak {where} at yaw {MOUNT.yaw_deg:+.1f} pitch "
            f"{MOUNT.pitch_deg:+.1f} deg, {MOUNT.forward_m:+.3f} forward "
            f"{MOUNT.left_m:+.3f} left {MOUNT.up_m:+.3f} up of the gimbal camera")
