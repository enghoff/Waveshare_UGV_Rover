"""Where a thing is, from two bearings taken from two different places.

[view.py](view.py) turns one observation into a bearing from a measured pose and
deliberately stops there, because one bearing is a direction and not a position.
This module is the next step and the only honest way to get one without a range
sensor: **two bearings taken from places far enough apart intersect at a point.**

That is the whole of it, and it is worth being clear about why it is the answer
to the question the POC failed. Asking the model "is this the sofa you saw
before?" failed in both directions -- Cosmos Reason 2 never says yes and Cosmos 3
never says no -- and no model can do better from a single picture, because two
identical chairs at opposite ends of a room are identical *in the picture*. What
separates them is not what they look like. It is where they are, and the rover
already measures that: SLAM knows where the rover was standing and the gimbal
knows where it was pointing.

**Nothing here works from one place.** Rays from a rover that only turned on the
spot all start from the same point and meet nowhere useful, so an entity gets a
position only once the rover has driven far enough between two looks. That is a
precondition rather than a limitation: it is what exploring already does.

The uncertainty is reported rather than hidden, because it is what the resolver
has to gate on and because it is large. With bearings good to about five degrees
-- measured, see BEARING_SIGMA_DEG -- a metre of baseline at three metres of range
puts the point out by the better part of a metre along the line of sight. Two
chairs at opposite walls separate easily at that accuracy; two chairs a metre
apart do not, and the resolver must be told that rather than left to guess.
"""
from __future__ import annotations

import math
from typing import Any

#: How wrong a bearing is, in degrees, one standard deviation. Measured on the
#: rover on 2026-09-01 rather than assumed: the same sofa in two inspections of a
#: byte-identical frame came back 4.8 degrees apart, and the coffee table 2.6,
#: with the difference coming from the model drawing a slightly different box
#: each time. Better boxes would shrink this and it is the dominant term.
BEARING_SIGMA_DEG = 5.0
#: Below this angle between two bearings the intersection runs away down the line
#: of sight and the answer is noise wearing a number. Two rays this close are
#: better treated as one look than as a fix.
MIN_PARALLAX_DEG = 12.0
#: Two looks from closer together than this are one look. Rays from a rover that
#: only turned on the spot share an origin exactly, and no amount of parallax in
#: the arithmetic makes that a measurement.
MIN_BASELINE_M = 0.4
#: Beyond this, indoors, the fix is a pair of nearly parallel rays agreeing by
#: accident rather than a thing in a room.
MAX_RANGE_M = 12.0


def _unit(bearing_deg: float) -> tuple[float, float]:
    radians = math.radians(bearing_deg)
    return math.cos(radians), math.sin(radians)


def _wrap(degrees: float) -> float:
    """To (-180, 180]."""
    return (degrees + 180.0) % 360.0 - 180.0


def parallax_deg(first: dict[str, Any], second: dict[str, Any]) -> float:
    """The angle between two bearings, folded to [0, 90].

    Folded because two rays meeting head-on are as good a fix as two meeting at a
    right angle; what makes a fix bad is being *parallel*, at either end.
    """
    between = abs(_wrap(float(second["bearing_deg"]) - float(first["bearing_deg"])))
    return 180.0 - between if between > 90.0 else between


def baseline_m(first: dict[str, Any], second: dict[str, Any]) -> float:
    return math.hypot(float(second["x_m"]) - float(first["x_m"]),
                      float(second["y_m"]) - float(first["y_m"]))


def _cross(first: dict[str, Any], second: dict[str, Any]
           ) -> tuple[float, float] | None:
    """Where two rays cross, or None if they do not cross in front of both."""
    ax, ay = float(first["x_m"]), float(first["y_m"])
    bx, by = float(second["x_m"]), float(second["y_m"])
    dax, day = _unit(float(first["bearing_deg"]))
    dbx, dby = _unit(float(second["bearing_deg"]))

    determinant = dbx * day - dax * dby
    if abs(determinant) < 1e-9:
        return None
    along = (-(bx - ax) * dby + dbx * (by - ay)) / determinant
    other = (dax * (by - ay) - day * (bx - ax)) / determinant
    # Behind the camera is not a sighting. A negative parameter means the rays
    # would have crossed had the rover been looking the other way.
    if along <= 0.0 or other <= 0.0:
        return None
    if along > MAX_RANGE_M or other > MAX_RANGE_M:
        return None
    return ax + along * dax, ay + along * day


def fix(first: dict[str, Any], second: dict[str, Any]) -> dict[str, Any] | None:
    """A map point from two bearings, with how far out it might be, or None.

    None is the ordinary answer and not a failure: two looks from nearly the same
    place, or along nearly the same line, genuinely do not locate anything, and
    saying so is the point. The caller keeps the observations and waits for a look
    from somewhere else.

    The uncertainty is measured rather than modelled -- each bearing is nudged by
    `BEARING_SIGMA_DEG` either way and the fix recomputed, and the radius reported
    is the furthest the point moved. That captures the geometry that matters here,
    which is that error along the line of sight grows with range and shrinks with
    baseline, without pretending to a covariance nobody calibrated.
    """
    if baseline_m(first, second) < MIN_BASELINE_M:
        return None
    if parallax_deg(first, second) < MIN_PARALLAX_DEG:
        return None
    point = _cross(first, second)
    if point is None:
        return None

    worst = 0.0
    for da in (-BEARING_SIGMA_DEG, BEARING_SIGMA_DEG):
        for db in (-BEARING_SIGMA_DEG, BEARING_SIGMA_DEG):
            nudged = _cross({**first,
                             "bearing_deg": float(first["bearing_deg"]) + da},
                            {**second,
                             "bearing_deg": float(second["bearing_deg"]) + db})
            if nudged is None:
                # A nudge that destroys the fix means the fix was marginal.
                return None
            worst = max(worst, math.hypot(nudged[0] - point[0],
                                          nudged[1] - point[1]))
    return {
        "x_m": round(point[0], 3),
        "y_m": round(point[1], 3),
        "uncertainty_m": round(worst, 3),
        "baseline_m": round(baseline_m(first, second), 3),
        "parallax_deg": round(parallax_deg(first, second), 1),
    }


def best_fix(rays: list[dict[str, Any]]) -> dict[str, Any] | None:
    """The most trustworthy fix obtainable from a set of bearings, or None.

    The pair with the smallest uncertainty rather than an average of all of them:
    a least-squares fit over rays whose errors are dominated by one bad bounding
    box is worse than the best honest pair, and this stays explainable -- the
    popup can name the two looks that placed the thing.
    """
    best = None
    for index, first in enumerate(rays):
        for second in rays[index + 1:]:
            found = fix(first, second)
            if found is None:
                continue
            if best is None or found["uncertainty_m"] < best["uncertainty_m"]:
                best = found
    return best


def bearing_to(point: dict[str, Any], observation: dict[str, Any]) -> float:
    """What bearing the thing at `point` would have been seen on from here."""
    return math.degrees(math.atan2(float(point["y_m"]) - float(observation["y_m"]),
                                   float(point["x_m"]) - float(observation["x_m"])))


def agrees(point: dict[str, Any], ray: dict[str, Any],
           tolerance_m: float | None = None) -> bool:
    """Whether a new bearing is consistent with a thing already placed.

    The test is in metres across the line of sight rather than in degrees, because
    a five-degree error matters much more at five metres than at one, and the
    resolver's question is "could this be the same object" rather than "is this
    the same angle".
    """
    range_m = math.hypot(float(point["x_m"]) - float(ray["x_m"]),
                         float(point["y_m"]) - float(ray["y_m"]))
    if range_m > MAX_RANGE_M:
        return False
    if tolerance_m is None:
        # What the bearing noise alone allows at this range, plus however
        # uncertain the point already was.
        tolerance_m = (range_m * math.tan(math.radians(BEARING_SIGMA_DEG))
                       + float(point.get("uncertainty_m", 0.0)))
    off_deg = abs(_wrap(bearing_to(point, ray) - float(ray["bearing_deg"])))
    if off_deg >= 90.0:
        return False
    return range_m * math.tan(math.radians(off_deg)) <= tolerance_m
