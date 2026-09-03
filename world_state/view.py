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
from typing import Any

from . import locate

#: How long a drawn ray is, in metres. A drawing convention, not a measurement:
#: far enough to cross a room, short enough not to imply the far wall.
RAY_M = 2.5
#: The narrowest cone worth drawing. A bounding box a few pixels wide would
#: otherwise be a line, and a line reads as a precision this has nowhere near.
MIN_SPAN_DEG = 6.0
MAX_SPAN_DEG = 90.0


def _wrap(degrees: float) -> float:
    """To (-180, 180], the same convention `locate` compares bearings in."""
    return (degrees + 180.0) % 360.0 - 180.0


def ray(observation: dict[str, Any], fov_deg: float,
        length_m: float = RAY_M) -> dict[str, Any] | None:
    """One observation as a bearing from where the rover stood, or None.

    None whenever the rover did not measure enough for an honest answer: no pose
    means there is no point to draw from, and no gimbal angle means there is no
    direction to draw in. Falling back to the middle of the map or to straight
    ahead would be inventing the very geometry this experiment refuses to invent.

    **The two sign conventions are opposite and that is the whole of the
    arithmetic.** The gimbal takes pan positive to the *right*; the map, the lidar
    and everything under `ros_nav` take bearings positive to the *left*. Same
    conversion, same reason, as the camera cone the map already draws -- see
    `_camera_cone` in [rover_nav.py](../rover_daemon/rover_nav.py).
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

    offset_deg, span_deg = _from_box(observation.get("bbox"), fov_deg)
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
        # Where the rover's nose was, plus where the gimbal was turned to, plus
        # where in the picture the thing sat -- brought back into (-180, 180].
        # **The wrap is not cosmetic.** Three numbers added together run past
        # half a turn easily, and the rover measured it doing so: a real
        # inspection stored bearings of -205.9 and -208.6 degrees, which point
        # exactly where +154.1 and +151.4 do but compare with nothing. Every
        # bearing that is written down or compared has to be canonical.
        "bearing_deg": round(_wrap(heading_deg - pan_deg + offset_deg), 1),
        "span_deg": round(span_deg, 1),
        "length_m": length_m,
    }


def _from_box(bbox: Any, fov_deg: float) -> tuple[float, float]:
    """How far off the middle of the picture the thing was, and how wide it was,
    both in degrees. (0, a default cone) when there is no usable box: the camera
    direction is still measured, and only the refinement is missing."""
    default = MIN_SPAN_DEG * 3
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        return 0.0, default
    try:
        left, _top, right, _bottom = (float(value) for value in bbox)
    except (TypeError, ValueError):
        return 0.0, default
    centre = (left + right) / 2.0
    # Left of the middle of the picture is to the camera's left, which is positive
    # in the map's convention.
    offset_deg = (0.5 - centre) * fov_deg
    span_deg = max(MIN_SPAN_DEG, min(MAX_SPAN_DEG, abs(right - left) * fov_deg))
    return offset_deg, span_deg


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
