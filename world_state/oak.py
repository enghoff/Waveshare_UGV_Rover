"""The OAK as a second camera on this rover: where it is, and what its pixels see.

The rover has two cameras that point at the room and they are nothing alike. The
gimbal camera is a 130-degree fisheye on two servos, swept and fitted by
`usb_cameras/calibrate_fov.py`, and it is the one every bearing this component has
ever recorded was drawn through. The OAK is bolted to the chassis, sees 70 degrees
of the room and cannot look anywhere else -- and it is the only thing on the rover
that knows **how far away** what it is looking at is.

This module is what lets the second one write into the world the first one built.
Three things have to be true for that, and each is a section below:

* **its pixels have to become directions**, which is a lens, and the lens is the
  device's own -- fetched by `depth_client`, never written down here;
* **it has to be somewhere**, which is `MOUNT`: where the OAK sits and which way
  it faces, relative to the gimbal camera at rest. Until that is measured this
  camera cannot draw a bearing at all, and `MEASURED` says so;
* **a box drawn on the other camera's picture has to be findable in this one**,
  which is `box_for`, and which is what lets a look taken through the gimbal
  carry a range without being taken through the OAK.

**The OAK is modelled as a gimbal that never moves**, and that is what keeps the
arithmetic in one place rather than two. An observation from it records the
mount's yaw as `observer_pan_deg` and its pitch as `observer_tilt_deg`, so
`view.ray` turns it into a bearing with the code it already had -- the only thing
that differs between the two cameras is which lens a pixel goes through.
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
    """Where the OAK is, relative to the gimbal camera at pan 0 and tilt 0.

    Angles first, because they are what a bearing is made of: `yaw_deg` positive
    to the **right**, which is the gimbal's convention and the opposite of the
    map's, and `pitch_deg` positive **up**, which is the gimbal's tilt. An OAK
    observation stores these two as its pan and tilt, so a wrong yaw here swings
    every bearing this camera ever records by the same amount.

    Then the offset, in the chassis frame: `forward_m` along the rover's nose,
    `left_m` to its left, `up_m` above. This is where the OAK's optical centre
    sits relative to the gimbal camera's, and it is what makes the two cameras
    agree about a thing two metres away rather than about a thing at infinity --
    ten centimetres of it is three degrees of bearing at two metres, which is
    twice what the geometry is told to expect from a bearing.

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
    #: in the picture. **Carried because this rover's is two degrees**, which is
    #: past where it can be waved away: a roll mixes a ray's bearing into its
    #: elevation and back, by the roll times how far off the axis the ray is, so
    #: at the edge of this camera's field two degrees is more than a degree of
    #: elevation. It was left out of the first version on the argument that a
    #: bracket bolted to a flat plate has none, and the bracket disagreed.
    roll_deg: float = 0.0
    forward_m: float = 0.0
    left_m: float = 0.0
    up_m: float = 0.0


#: Where this rover's OAK is. **Which way it points was measured on 2026-09-04
#: by `bench_oak.py`; where it sits was not, and the two have very different
#: standing.**
#:
#: The rotation is what this bench is for and it is solid. Both cameras looked at
#: a dining table, the OAK's picture was warped into the fisheye's geometry, ORB
#: matched them, the depth camera ranged every match, and the rotation lining the
#: two sets of directions up was solved with its own inliers re-selected as it
#: went. What survives misses by 0.3 to 0.9 degrees against the 1.5
#: `locate.BEARING_SIGMA_DEG` allows a bearing, and -- the part that matters --
#: **it comes out the same whatever offset it is given**, within about half a
#: degree across everything from nothing to half a metre.
#:
#: **The offset is not measured and is deliberately left at nothing.** Two lenses
#: a few centimetres apart looking at a room metres away see it in almost the
#: same direction: five centimetres at three metres is one degree of parallax,
#: which is the size of what the fit leaves over anyway. So a solver handed both
#: at once spends the offset absorbing whatever else is systematic, and on this
#: rover it did exactly that -- it claimed 0.571 m forward and 0.181 m to the
#: right, fitted the data four times better for it, and was wrong: the rover's
#: owner reports the two lenses are a few centimetres apart with both on the
#: **centre axis**, so the lateral offset is zero by construction. That figure
#: was briefly adopted here on the strength of its own repeatability, which is
#: the trap this note exists to stop somebody walking into again.
#:
#: Nothing is the honest placeholder rather than a guess at "a few": it is what
#: every bearing before this was worked out as, and being out by a few
#: centimetres costs about a degree of bearing at two metres, which is inside the
#: 1.5 the geometry already expects. Being out by half a metre costs sixteen.
#:
#: **A ruler settles it, and nothing here can.** Measure how far ahead of the
#: gimbal camera's lens the OAK's sits and how far above it, and pass them to
#: `bench_oak.py --offset FORWARD LEFT UP` -- which then solves the rotation with
#: them held, and prints a block to paste back in here.
#:
#: **The roll is carried, because it turned out to be two degrees.** It was left
#: out at first on the argument that a bracket bolted to a flat plate has none,
#: and it read as half a degree -- but that reading came from the fit that was
#: also inventing half a metre of offset, and with the offset held at nothing it
#: is -2.1. Two degrees mixes a ray's bearing into its elevation by the roll
#: times how far off the axis the ray is, so at the edge of this camera's field
#: it is worth more than a degree.
MOUNT = Mount(
    yaw_deg=-1.53,
    pitch_deg=3.11,
    roll_deg=-2.12,
    forward_m=0.0,
    left_m=0.0,
    up_m=0.0,
)

#: Whether `MOUNT` above holds measurements. **Everything this module can do is
#: gated on it**, in both directions: the OAK cannot draw a bearing without it,
#: and a box drawn on the gimbal camera cannot be found in the OAK's picture
#: without it either, so an unmeasured rover simply records what it always
#: recorded and says nothing about range.
#:
#: A flag rather than a check for zeros, because zero is a perfectly possible
#: measurement -- a camera mounted straight ahead has a yaw of zero, and the
#: difference between "measured as zero" and "never measured" is the whole point.
#: It is on because the rotation is measured, which is the half that swings every
#: bearing; the offset sitting at nothing costs about a degree at two metres and
#: is a known, bounded, written-down gap rather than an unknown one.
MEASURED = True

#: How far off the OAK's own axis a direction may lie before it is not in its
#: picture at all, as a fraction of the frame beyond the edge. A box mapped from
#: the gimbal camera lands wherever it lands, and most of them land outside: the
#: gimbal sees 122 degrees across and the OAK 70, so about half of a centred
#: gimbal frame has no depth behind it at all and everything past +/-35 degrees of
#: pan has none whatever.
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


def ray_at(x_frac: float, y_frac: float, lens: Any) -> tuple[float, float, float]:
    """Where a point in the OAK's picture looks: x right, y down, z out of the lens.

    The same convention `face_tracking/lens.ray_at` answers in, so `view` can
    rotate either camera's answer with one piece of code -- and **with the
    mount's roll already taken out**, so that what comes back is in the frame the
    yaw and the pitch are defined in and the caller can go on treating this
    camera as a gimbal that never moves.

    A pinhole, and honestly one rather than for convenience -- see
    `depth_client.Lens`, where the measured reason is written down: this lens's
    rational distortion model has its numerator and denominator terms within a
    tenth of each other and they very nearly cancel.
    """
    x, y, z = pinhole_at(x_frac, y_frac, lens)
    x, y = _unrolled(x, y)
    return x, y, z


def pinhole_at(x_frac: float, y_frac: float,
               lens: Any) -> tuple[float, float, float]:
    """The same pixel, through the lens alone and **not** through the mount.

    `ray_at` is what everything that draws a bearing wants: the direction with
    the mount's roll already taken out, so this camera can be treated as a gimbal
    that never moves. This is what the *calibration* wants, and the difference is
    the whole reason it is a separate function: a bench that measured the mount
    through `ray_at` would be measuring how far the mount has moved since the
    last time somebody wrote a number down, and would print that as if it were
    the mount. The one it prints has to be absolute.
    """
    x = (x_frac * lens.width - lens.cx) / lens.fx
    y = (y_frac * lens.height - lens.cy) / lens.fy
    length = math.sqrt(x * x + y * y + 1.0)
    return x / length, y / length, 1.0 / length


def _rolled(x: float, y: float) -> tuple[float, float]:
    """Turn a direction from the frame the yaw and pitch live in into the
    sensor's own, which is the one a pixel is measured in."""
    roll = math.radians(MOUNT.roll_deg)
    return (x * math.cos(roll) - y * math.sin(roll),
            x * math.sin(roll) + y * math.cos(roll))


def _unrolled(x: float, y: float) -> tuple[float, float]:
    """And back the other way. The inverse of `_rolled` by construction, which is
    what `test_oak` checks rather than trusting the two signs to stay in step."""
    roll = math.radians(MOUNT.roll_deg)
    return (x * math.cos(roll) + y * math.sin(roll),
            -x * math.sin(roll) + y * math.cos(roll))


def pan_deg() -> float:
    """What an OAK observation records as its gimbal pan: the mount's yaw."""
    return MOUNT.yaw_deg


def tilt_deg() -> float:
    """And as its tilt: the mount's pitch."""
    return MOUNT.pitch_deg


def pose_at(pose: dict[str, Any] | None) -> dict[str, Any] | None:
    """The rover's pose moved to where the OAK's optical centre actually is.

    **A ray has to start where the camera is**, and this camera is not where the
    other one is. The store keeps the pose an observation was taken from and
    `locate` treats it as the origin of the ray, so an OAK look whose pose was
    the gimbal camera's would put every crossing out by the offset between them
    -- and worse, the range the OAK measured is from the OAK, so the range and
    the origin would be describing different points.

    The offset is in the chassis frame and the pose is on the map, so it is
    turned by the heading: the rover's nose points along `(cos h, sin h)` and its
    left along `(-sin h, cos h)`.

    `up_m` is not spent here and is not lost either: the map is flat, and what
    reads a height is `locate.rise_m`, which measures it relative to whatever the
    camera's own height is and never needs to know it -- see
    `locate.CAMERA_HEIGHT_M`. A difference in mounting height between the two
    cameras is the one term that does not cancel there, and it is recorded in
    `MOUNT` so that it can be spent when that constant is.
    """
    if not isinstance(pose, dict) or not MEASURED:
        return pose
    if not MOUNT.forward_m and not MOUNT.left_m:
        return pose
    try:
        heading = math.radians(float(pose.get("heading_deg", 0.0)))
        x_m, y_m = float(pose["x_m"]), float(pose["y_m"])
    except (KeyError, TypeError, ValueError):
        return pose
    return {**pose,
            "x_m": round(x_m + MOUNT.forward_m * math.cos(heading)
                         - MOUNT.left_m * math.sin(heading), 3),
            "y_m": round(y_m + MOUNT.forward_m * math.sin(heading)
                         + MOUNT.left_m * math.cos(heading), 3)}


# --- finding the other camera's box in this one -------------------------------


def box_for(corners: list[tuple[float, float, float]], lens: Any,
            range_m: float | None = None
            ) -> tuple[list[float], float] | None:
    """Where a thing the gimbal camera saw would be in the OAK's picture, and how
    far the OAK's answer would then be from the gimbal camera.

    `corners` are the box's four corners as directions in the **chassis frame** --
    x forward, y left, z up -- which is what `view.chassis_direction` answers.
    They arrive that way rather than as pixels because the two cameras share
    nothing else: a fisheye pixel and a pinhole pixel are not comparable, and the
    direction between them is.

    None when the thing is not in this camera's picture at all, which is the
    ordinary case rather than a failure -- the gimbal sees 122 degrees across and
    this camera 70, so about half of a centred frame has no depth behind it and a
    look taken over the rover's shoulder has none whatever.

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
    quarter of the answer at half of one.

    Exactly, and with no guess in it: the thing lies somewhere along the gimbal
    camera's own direction, and it lies on a sphere of radius `oak_range_m` about
    the OAK's lens. Where a line meets a sphere is a quadratic, so this is the far
    root of one -- which is also why `box_for` above may guess about *where to
    look* without that guess reaching the answer.

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


def _in_oak(direction: tuple[float, float, float],
            range_m: float) -> tuple[float, float, float] | None:
    """A chassis-frame direction, as a direction in the OAK's own optical frame.

    The point the gimbal camera is looking at is `range_m` along `direction` from
    the gimbal camera, which is the chassis frame's origin by construction. Where
    that point lies from the OAK is the same point less the OAK's own position,
    turned by the mount's yaw and pitch.

    None when the point ends up behind this camera, which a thing over the
    rover's shoulder is.
    """
    x = direction[0] * range_m - MOUNT.forward_m
    y = direction[1] * range_m - MOUNT.left_m
    z = direction[2] * range_m - MOUNT.up_m
    # Undo the mount's yaw. The OAK's axis is at chassis yaw `-yaw_deg` measured
    # positive to the left, because the mount's yaw is positive to the right --
    # the same swap `view.ray` makes for the gimbal's pan.
    yaw = math.radians(-MOUNT.yaw_deg)
    forward = x * math.cos(yaw) + y * math.sin(yaw)
    left = -x * math.sin(yaw) + y * math.cos(yaw)
    # Then its pitch, positive up, about the camera's own horizontal.
    pitch = math.radians(MOUNT.pitch_deg)
    along = forward * math.cos(pitch) + z * math.sin(pitch)
    up = -forward * math.sin(pitch) + z * math.cos(pitch)
    if along <= 1e-6:
        return None
    # Into the lens's own axes: x right, y down, z out -- and twisted by the
    # mount's roll, because that is the frame a pixel is measured in and
    # `_project` is a plain pinhole.
    right, down = _rolled(-left, -up)
    return right, down, along


def _project(direction: tuple[float, float, float],
             lens: Any) -> tuple[float, float]:
    """A direction in the OAK's optical frame, as a fraction of its picture."""
    x, y, z = direction
    return ((lens.cx + lens.fx * x / z) / lens.width,
            (lens.cy + lens.fy * y / z) / lens.height)


def describe() -> str:
    """One line for the diagnostics row: where this camera is, or that nobody knows."""
    if not MEASURED:
        return "oak mount unmeasured -- run bench_oak.py"
    return (f"oak at yaw {MOUNT.yaw_deg:+.1f} pitch {MOUNT.pitch_deg:+.1f} deg, "
            f"{MOUNT.forward_m:+.3f} forward {MOUNT.left_m:+.3f} left "
            f"{MOUNT.up_m:+.3f} up of the gimbal camera")
