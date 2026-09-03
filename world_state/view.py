"""Turning an observation's provenance into something that can be drawn on the map.

An entity here has no position and must not be given one. What it has is a set of
observations, and each of those records where the rover was standing and where the
gimbal was pointing when the model saw the thing -- both measured by the rover
rather than guessed by the model. That is enough to draw a **bearing from a
measured pose**: a ray from the observation point along the camera's direction,
with the bounding box's horizontal position refining the angle.

The point of drawing it is the experiment's question. Three observations of one
sofa from three places are three rays, and whether they converge on one corner of
the room is exactly what "did it stay the same entity" looks like from the outside;
two identifiers on rays pointing at the same thing is what a duplicate looks like.

No distance is claimed and none is stored. A ray on its own has a length only
because a line has to end somewhere on a picture.

**Once the thing has been placed, the ray stops being the point.** What a person
looking at the map then wants is not six stubs of arbitrary length but how each
look stands to the one position the application has settled on: where the rover
was standing, which way it was facing, how far the settled point is from there,
and whether the bearing it measured actually points at it. `relate` answers that
with the resolver's own arithmetic, so the page draws a decision rather than
re-deriving one -- and a look that disagrees with the thing it is attached to is
visible as a fork instead of being averaged into a bundle of arrows.
"""
from __future__ import annotations

import math
import os
import sys
from typing import Any

from . import locate

#: How long a drawn ray is, in metres. A drawing convention, not a measurement:
#: far enough to cross a room, short enough not to imply the far wall.
RAY_M = 2.5
#: The narrowest cone worth drawing. A bounding box a few pixels wide would
#: otherwise be a line, and a line reads as a precision this has nowhere near.
MIN_SPAN_DEG = 6.0
MAX_SPAN_DEG = 90.0
#: The capture mode to read the lens in when nothing says otherwise. The rover
#: captures at this size; `lens.lens_for` has its own documented rule for a mode
#: it has not been swept in, and this only decides which one it is asked about.
FRAME_SIZE = (640, 480)


def _lens_module():
    """`face_tracking/lens.py`, wherever this checkout or this rover keeps it.

    **One description of the optics and not two.** The lens is a property of the
    hardware rather than of any component, it was swept and fitted on this rover
    by `usb_cameras/calibrate_fov.py`, and `face_tracking/aiming.py` has aimed
    through it since 2026-08-19. A copy of those numbers here could only ever
    drift away from the ones the gimbal is driven with.

    Deployment flattens `face_tracking/` into `~/ugv/` while this package lands
    in `~/ugv/world_state/`, so the directory above this one is where it lives on
    the rover and `face_tracking/` beside it is where it lives in a checkout.
    Both go on the path and whichever exists wins -- the same dance
    `bench_bearing.py` has been doing.
    """
    global _LENS_MODULE
    if _LENS_MODULE is None:
        here = os.path.dirname(os.path.abspath(__file__))
        root = os.path.dirname(here)
        for path in (root, os.path.join(root, "face_tracking")):
            if os.path.isdir(path) and path not in sys.path:
                sys.path.append(path)
        import lens                                    # noqa: PLC0415

        _LENS_MODULE = lens
    return _LENS_MODULE


_LENS_MODULE = None
_LENSES: dict[tuple[int, int], Any] = {}


def _lens_for(size: tuple[int, int] | None):
    """The fitted lens for a capture mode, or None if it cannot be reached.

    None rather than a raise, and rather than falling back to a multiplication:
    a host that cannot find the lens has no business writing bearings, and the
    store already knows what to do with an observation that has none. It is the
    same answer this module gives for a missing pose or a missing gimbal angle,
    for the same reason.
    """
    key = tuple(size or FRAME_SIZE)
    got = _LENSES.get(key)
    if got is None:
        try:
            got = _lens_module().lens_for(key[0], key[1])
        except Exception:                              # noqa: BLE001
            return None
        _LENSES[key] = got
    return got


def azimuth_deg(cx_frac: float, cy_frac: float, tilt_deg: float,
                size: tuple[int, int] | None = None) -> float | None:
    """Where a point in the picture lies, in degrees left of where the camera
    was aimed, or None if the lens cannot be reached.

    **This replaces one multiplication, and the multiplication was wrong twice
    over.** What used to be here mapped the box's horizontal position across the
    frame straight onto an angle, which is only true along the two centre lines
    of a 130-degree fisheye -- and it dropped the gimbal's tilt entirely,
    although every observation records it. Measured over the 441 boxes of the
    drive of 2026-09-03, the two together put **184 of them outside the 1.5
    degrees `locate.BEARING_SIGMA_DEG` promises the geometry**, with a median of
    1.24 and a worst of 16.9. The error is almost all vertical: about a degree
    across the middle band of the picture and eleven to thirteen along the top,
    because that is where a tilted fisheye bends most.

    Two steps, and the second is the one a separable model leaves out. The pixel
    becomes a direction through the fitted projection; that direction is then
    rotated back by the tilt the gimbal was holding, because the gimbal pans
    about the world's vertical and tilts about its own horizontal, so a ray's
    bearing cannot be read off until the tilt is undone. Only the component out
    of the lens survives that: how high the ray ends up does not change which
    way it points.
    """
    lens = _lens_for(size)
    if lens is None:
        return None
    width, height = tuple(size or FRAME_SIZE)
    x, y, z = _lens_module().ray_at(cx_frac * width, cy_frac * height, lens)
    tilt = math.radians(tilt_deg or 0.0)
    z_level = y * math.sin(tilt) + z * math.cos(tilt)
    # Positive to the camera's left, which is the map's convention and the
    # opposite of the gimbal's -- the same swap `ray` makes for the pan.
    return math.degrees(math.atan2(-x, z_level))


def _wrap(degrees: float) -> float:
    """To (-180, 180], the same convention `locate` compares bearings in."""
    return (degrees + 180.0) % 360.0 - 180.0


def ray(observation: dict[str, Any], fov_deg: float,
        length_m: float = RAY_M,
        size: tuple[int, int] | None = None) -> dict[str, Any] | None:
    """One observation as a bearing from where the rover stood, or None.

    None whenever the rover did not measure enough for an honest answer: no pose
    means there is no point to draw from, and no gimbal angle means there is no
    direction to draw in. Falling back to the middle of the map or to straight
    ahead would be inventing the very geometry this experiment refuses to invent.
    A lens that cannot be reached is the same kind of silence and gets the same
    answer -- see `_lens_for`.

    **The two sign conventions are opposite and that is the whole of the
    arithmetic.** The gimbal takes pan positive to the *right*; the map, the lidar
    and everything under `ros_nav` take bearings positive to the *left*. Same
    conversion, same reason, as the camera cone the map already draws -- see
    `_camera_cone` in [rover_nav.py](../rover_daemon/rover_nav.py).

    **`fov_deg` no longer does the arithmetic and is kept as the switch.** It
    says the caller is in a position to know what the camera saw; what the angle
    is actually worked out through is the swept lens in `face_tracking/lens.py`,
    chosen by the frame's own size. A rover with a different camera wants
    `usb_cameras/calibrate_fov.py` run on it and an entry in `lens.LENS`, not a
    number passed in here -- the old multiplication is what this fixed.

    **A bearing the rover already measured is returned rather than recomputed.**
    The store works one out at the moment of the look, from the field of view the
    camera had then, and that is a measurement: a lens refitted afterwards must
    not silently rewrite every bearing the rover has ever recorded. So a row that
    carries `bearing_deg` keeps it, which also makes the sight line the console
    draws the same one `resolve.ray_of` reads -- until this was so, the page was
    quietly redrawing old looks through today's model while the resolver went on
    matching them through yesterday's.
    """
    pose = observation.get("pose")
    if not isinstance(pose, dict):
        return None
    try:
        x_m = float(pose["x_m"])
        y_m = float(pose["y_m"])
        heading_deg = float(pose.get("heading_deg", 0.0))
    except (KeyError, TypeError, ValueError):
        return None
    pan = observation.get("observer_pan_deg")
    if pan is None:
        return None
    try:
        pan_deg = float(pan)
    except (TypeError, ValueError):
        return None

    stored_bearing = observation.get("bearing_deg")
    stored_span = observation.get("span_deg")
    if stored_bearing is None:
        measured = _from_box(observation.get("bbox"),
                             _tilt_of(observation), size)
        if measured is None:
            return None
        offset_deg, span_deg = measured
        bearing_deg = _wrap(heading_deg - pan_deg + offset_deg)
    else:
        bearing_deg = _wrap(float(stored_bearing))
        span_deg = float(stored_span if stored_span is not None
                         else MIN_SPAN_DEG * 3)
    return {
        "id": observation.get("id"),
        "x_m": round(x_m, 3),
        "y_m": round(y_m, 3),
        # Where the rover itself was facing, and where the gimbal was turned to
        # from there. Both are already inside `bearing_deg` below; they are here
        # as well because a person reading the map is asking a different
        # question of them -- *the rover stood here, facing that way, and looked
        # over its shoulder* -- and a single arrow cannot say that. The gimbal's
        # sign is left as the gimbal reports it, positive to the right, and it is
        # labelled as such wherever it is shown.
        "heading_deg": round(heading_deg, 1),
        "pan_deg": round(pan_deg, 1),
        # How far out this ray's starting point is, carried through so that the
        # console's agreement test is the resolver's own and not a second
        # opinion: `relate` hands this dictionary to `locate.match_tolerance`,
        # which allows for it.
        "origin_sigma_m": float(observation.get("origin_sigma_m") or 0.0),
        # How well the bearing itself is known, carried for the same reason as
        # the line above: `relate` hands this dictionary to
        # `locate.match_tolerance`, and a look taken while the rover was turning
        # must be drawn against the width it actually earned rather than against
        # the width a look from a standstill earns. See `locate.sigma_of`.
        "bearing_sigma_deg": observation.get("bearing_sigma_deg"),
        # Where the rover's nose was, plus where the gimbal was turned to, plus
        # where in the picture the thing sat -- brought back into (-180, 180].
        # **The wrap is not cosmetic.** Three numbers added together run past
        # half a turn easily, and the rover measured it doing so: a real
        # inspection stored bearings of -205.9 and -208.6 degrees, which point
        # exactly where +154.1 and +151.4 do but compare with nothing. Every
        # bearing that is written down or compared has to be canonical.
        "bearing_deg": round(bearing_deg, 1),
        "span_deg": round(span_deg, 1),
        "length_m": length_m,
    }


def _tilt_of(observation: dict[str, Any]) -> float:
    """Where the gimbal was tilted to, in degrees, or level if it did not say.

    Level is the right silence here and not a guess: it is what a rover that
    never tilts records, and it is what every bearing written before this was
    read was worked out as. What it is *not* is a substitute for a missing pan --
    the pan says which way the camera was aimed and its absence means no ray at
    all, while the tilt only bends the answer.
    """
    try:
        return float(observation.get("observer_tilt_deg") or 0.0)
    except (TypeError, ValueError):
        return 0.0

def _from_box(bbox: Any, tilt_deg: float = 0.0,
              size: tuple[int, int] | None = None
              ) -> tuple[float, float] | None:
    """How far off the camera's aim the thing was, and how wide it was, both in
    degrees, or None if the lens cannot be reached.

    (0, a default cone) when there is no usable box: the camera direction is
    still measured, and only the refinement is missing.

    **Off the lens axis rather than off the middle of the picture**, which is a
    choice worth naming because the two are 0.8 degrees apart on this camera --
    the sweep put the principal point thirteen pixels above the centre of the
    frame. The axis is what the fitted projection calls forward and what
    `lens.ray_at` answers (0, 0, 1) for, so taking it needs no assumption the
    calibration did not make. What *is* unmeasured is where pan = 0 actually
    points relative to either, and it is worth more than this 0.8 degrees to
    anybody chasing it: the gimbal is already known to arrive about three degrees
    short of where it is sent, which no lens model can see.

    **The width is measured the same way as the direction**, as the angle between
    the box's two vertical edges taken at its own height in the frame, rather
    than as its pixel width times a field of view. The two answers differ for the
    same reason the centre's did -- a box of a given pixel width spans more angle
    at the edge of a fisheye than at the middle -- and `locate.match_tolerance`
    spends this number, so it has to come from the same optics as everything
    else.
    """
    default = MIN_SPAN_DEG * 3
    if _lens_for(size) is None:
        return None
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        return 0.0, default
    try:
        left, top, right, bottom = (float(value) for value in bbox)
    except (TypeError, ValueError):
        return 0.0, default
    cx, cy = (left + right) / 2.0, (top + bottom) / 2.0
    at_centre = azimuth_deg(cx, cy, tilt_deg, size)
    at_left = azimuth_deg(left, cy, tilt_deg, size)
    at_right = azimuth_deg(right, cy, tilt_deg, size)
    if at_centre is None or at_left is None or at_right is None:
        return None
    return at_centre, max(MIN_SPAN_DEG,
                          min(MAX_SPAN_DEG, abs(at_left - at_right)))


def relate(placement: dict[str, Any] | None,
           drawn: dict[str, Any]) -> dict[str, Any] | None:
    """How one look stands to the position the application has settled on.

    None when there is nothing to stand against -- an entity the rover has
    pointed at from one place only has no position, and that is the ordinary
    state of everything until the rover drives.

    **The numbers are the resolver's own and are not recomputed here.** Whether
    a bearing points at a placed thing is decided by `locate.agrees` against
    `locate.match_tolerance`, which is the same call `resolve._against_known`
    makes when it attaches a look in the first place. So a look drawn as
    disagreeing on the map is a look that would not be attached today, rather
    than a second opinion this module formed on its own -- and where the two
    would differ, the answer on the screen is the one the rover acts on.

    `miss_m` is measured across the line of sight rather than along it, for the
    reason `locate.cross_track` exists: error runs a long way down a shallow
    crossing and hardly at all across it, and *how far the bearing misses the
    thing* is the sideways question.
    """
    if not placement:
        return None
    try:
        range_m = math.hypot(float(placement["x_m"]) - float(drawn["x_m"]),
                             float(placement["y_m"]) - float(drawn["y_m"]))
        to_deg = locate.bearing_to(placement, drawn)
    except (KeyError, TypeError, ValueError):
        return None
    off_deg = _wrap(float(drawn["bearing_deg"]) - to_deg)
    tolerance_m = locate.match_tolerance(placement, drawn)
    return {
        "range_m": round(range_m, 2),
        # Where the settled thing actually lies from here, against where this
        # look said it was. Both are on the map, and the gap between them is
        # what the fork in the drawn sight line is.
        "to_deg": round(to_deg, 1),
        "off_deg": round(off_deg, 1),
        "miss_m": round(range_m * math.tan(math.radians(min(abs(off_deg), 89.9))), 2),
        "tolerance_m": round(tolerance_m, 2),
        "agrees": bool(locate.agrees(placement, drawn, tolerance_m)),
    }


def rays(observations: list[dict[str, Any]], fov_deg: float,
         limit: int = 6,
         placement: dict[str, Any] | None = None) -> list[dict[str, Any]]:
    """The newest few observations of one entity as rays, oldest of those first.

    Bounded because the picture is what has to stay readable: an entity seen thirty
    times would be thirty overlapping lines saying no more than six of them do.
    That bound is generous for a placed thing and tight for an unplaced one, which
    is why the caller sets it: sight lines that all end at one point stay legible
    in a way that six free-running stubs do not, and whether they converge is the
    question.

    `placement` is what the entity has settled on, and giving it is what turns a
    ray into a sighting -- see `relate`.
    """
    drawn = []
    for observation in observations[:limit]:
        one = ray(observation, fov_deg)
        if one is not None:
            one["observed_at"] = observation.get("observed_at")
            one["map_session"] = observation.get("map_session")
            one["frame_id"] = observation.get("frame_id")
            one["relation"] = relate(placement, one)
            drawn.append(one)
    drawn.reverse()
    return drawn
