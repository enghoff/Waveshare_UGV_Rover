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
has to gate on. With bearings good to a degree and a half -- measured, see
BEARING_SIGMA_DEG -- a metre of baseline at three metres of range puts the point
out by about a quarter of a metre along the line of sight. Two chairs at opposite
walls separate easily at that accuracy and so do two chairs a metre apart; two
books on the same shelf still do not, and the resolver must be told that rather
than left to guess.
"""
from __future__ import annotations

import math
from typing import Any

#: How wrong a bearing is, in degrees, one standard deviation. Re-measured on the
#: rover on 2026-09-02 after the region finder moved to the GPU, and it fell by
#: more than a factor of three -- it stood at 5.0, taken when a language model was
#: drawing the boxes and redrawing them differently every time.
#:
#: A bearing is three things added together, and all three were measured
#: separately, because knowing which one dominates is what says whether it is
#: worth trying to improve:
#:
#:   the box      eight inspections of an unchanging scene, regions matched
#:                between them by appearance: 0.13 deg of scatter, worst 0.16.
#:                FastSAM draws the same box every time, so this term is gone.
#:   the heading  the rover's own idea of which way it faces, over the two
#:                minutes of a gimbal sweep while it stood still: 0.2 deg.
#:   the gimbal   the same objects seen with the gimbal at -30, -15, 0, +15 and
#:                +30 degrees, which is what is left once the other two are ruled
#:                out: within 0.7 deg out to +/-15, and about 3 deg at -30, on two
#:                separate objects on opposite sides of the frame. That is the
#:                pan servo not arriving where it was told, and there is nothing
#:                to correct it with: the driver board's telemetry carries the
#:                inertial sensors and the wheel encoders but no gimbal feedback,
#:                so the commanded angle is all the rover knows.
#:
#: 1.5 is the root-mean-square of the ten sightings across that whole sweep. The
#: gimbal is now the dominant term by a long way, and a rover that inspected only
#: within +/-15 degrees of pan would do better than this number says.
#:
#: **What this does not cover: driving.** Every measurement above was taken with
#: the rover standing still, so the heading term here is drift over two minutes
#: and not the error SLAM accumulates over a few metres of travel -- which is
#: exactly the case a fix is taken in. That still wants measuring.
BEARING_SIGMA_DEG = 1.5
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
#: And nearer than this, from either of the two places that saw it, a fix is not a
#: thing in the room either. **This is the fault the validation drive of
#: 2026-09-02 found**, and it is worth stating because neither of the two guards
#: above catches it: two rays pointing *inward* from two nearby places cross in
#: the gap between them at a perfectly healthy parallax, off a perfectly healthy
#: baseline. Driven between three places, the rover placed six things and every
#: one of them landed between 0.13 and 0.59 m from the nearest camera that saw
#: it -- a doorway fourteen centimetres away, a floor lamp thirteen.
#:
#: Worse, such a crossing wins. The uncertainty is measured by nudging each
#: bearing, and a degree and a half barely moves a point a quarter of a metre
#: away, so a phantom on the lens reports a couple of centimetres of uncertainty
#: and outranks every real thing in the room.
#:
#: 0.75 m is a statement about the rover rather than about the arithmetic: it does
#: not drive closer than that to anything, its region finder throws away anything
#: filling more than a third of the frame, and a crop of something on the lens is
#: not a crop of a lamp. Refusing a real object that close costs a placement the
#: rover can make again from somewhere else; accepting a phantom writes a thing
#: that was never there into the world and keeps it.
MIN_RANGE_M = 0.75


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
    # Far enough from both, and not so far that two nearly parallel rays are
    # agreeing by accident. Measured from each observer rather than from their
    # midpoint, because the failure is asymmetric: the crossing sits on top of
    # one camera and a comfortable distance from the other.
    for observer in (first, second):
        range_m = math.hypot(point[0] - float(observer["x_m"]),
                             point[1] - float(observer["y_m"]))
        if range_m < MIN_RANGE_M or range_m > MAX_RANGE_M:
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


#: The most an object's own width may add to the tolerance for pointing at it.
#: A region spanning most of the frame is a wall or a floor, and letting that
#: claim three metres of slack would let it swallow the room.
MAX_EXTENT_M = 0.75


def match_tolerance(point: dict[str, Any], ray: dict[str, Any]) -> float:
    """How far off a bearing may be and still be pointing at this thing.

    Three terms, and **the third is the one that was missing**: how wrong the
    bearing is, how uncertain the thing's position is, and how big the thing
    actually is. An entity is stored as a point, but a television is a metre
    wide, and two looks from different sides of it centre on different parts of
    it -- so a bearing that lands anywhere within the thing's own silhouette is
    pointing at it, however precisely the bearing itself is known.

    Measured on the rover on 2026-09-02: with only the first two terms, matching
    a television at two and a half metres allowed 0.115 m, which is a tenth of
    the television. Looks that should have joined it were refused, fell through
    to the pairing pass, and made a *second* television eight centimetres from
    the first -- four in the end, and three people where there was one.

    The width comes from the observation's own `span_deg`, which is what the
    region measured, so a doorway gets more room than a mug without anything
    having to store a size.
    """
    range_m = math.hypot(float(point["x_m"]) - float(ray["x_m"]),
                         float(point["y_m"]) - float(ray["y_m"]))
    span_deg = float(ray.get("span_deg") or 0.0)
    extent_m = min(MAX_EXTENT_M,
                   range_m * math.tan(math.radians(min(span_deg, 90.0) / 2.0)))
    return (range_m * math.tan(math.radians(BEARING_SIGMA_DEG))
            + float(point.get("uncertainty_m", 0.0)) + extent_m)


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
