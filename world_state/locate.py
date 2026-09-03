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
#:                The region finder drew the same box every time, so this term
#:                is gone. Measured against FastSAM; YOLOE replaced it on
#:                2026-09-02 and carries 72% of its regions from one look at an
#:                unchanging scene to the next, against FastSAM's 64%, so if this
#:                term has moved at all it has moved the right way.
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


def sigma_of(ray: dict[str, object]) -> float:
    """How well this particular bearing is known, in degrees.

    **`BEARING_SIGMA_DEG` is what the gimbal and the heading are worth on a rover
    standing still, and a look taken while turning is worth much less.** Taking
    the picture is not instant: the shutter opens somewhere inside a grab that
    measures about a third of a second, and a rover turning at the 29 degrees a
    second this one manages in the median swings the whole bearing while it is
    open. That used to cost the look its bearing outright -- 71 of the 108 looks
    of the drive of 2026-09-03 stored no direction for anything they saw, every
    one of them for this reason.

    The frame carries its own timestamp, so the heading can be interpolated to
    the instant it was taken rather than guessed at from the middle of a bracket.
    What is left over is the turn rate multiplied by how well that instant is
    known, and it is carried here rather than being the reason to throw the look
    away. A fast turn now buys a wide answer instead of no answer, which is
    exactly what `origin_sigma_m` did for travel.

    Absent means the ray was measured on a rover this could not be asked of, and
    the answer is the constant -- which is what every bearing recorded before
    this existed was worked out as.
    """
    try:
        got = ray.get("bearing_sigma_deg")           # type: ignore[union-attr]
    except AttributeError:
        return BEARING_SIGMA_DEG
    if got is None:
        return BEARING_SIGMA_DEG
    try:
        return max(BEARING_SIGMA_DEG, float(got))
    except (TypeError, ValueError):
        return BEARING_SIGMA_DEG
#: Below this angle between two bearings the intersection runs away down the line
#: of sight and the answer is noise wearing a number. Two rays this close are
#: better treated as one look than as a fix.
MIN_PARALLAX_DEG = 12.0
#: Two looks from closer together than this are one look. Rays from a rover that
#: only turned on the spot share an origin exactly, and no amount of parallax in
#: the arithmetic makes that a measurement.
MIN_BASELINE_M = 0.4
#: How far a ray's own starting point might be out, when the observation does not
#: say. Zero, because a row written before the rover measured it was taken from a
#: rover standing still: the gate that produced those rows refused every look
#: taken on the move, so silence there really does mean "the rover was not going
#: anywhere". See `Inspector._where` for what writes it now.
NO_ORIGIN_ERROR_M = 0.0
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

#: How far past the first thing on its own bearing a sighting may still be, in
#: metres. **The thing itself is often the obstacle** -- a picture on a wall, a
#: doorway, a window -- so this cannot be zero, and the grid's own cell is 5 cm
#: before a bearing good to 1.5 degrees clips a corner early.
#:
#: **The generous end of a wide safe band, chosen for the lidar rather than for
#: the arithmetic.** The map is drawn by a 2D scanner at chassis height and the
#: camera is on a gimbal above it, so the two do not agree about what blocks a
#: view: a sofa two metres away is a wall to the lidar and something the camera
#: looks straight over. A metre of slack is the allowance for that, and it costs
#: nothing here -- measured on the run of 2026-09-02, every placement this refuses
#: is claiming a thing 3.3 to 9.6 m past its own first obstacle, and the whole
#: band from 0 to 3 m refuses exactly the same three placements. Past 5 m they
#: start coming back.
#:
#: What it cannot do is tell furniture from a wall, so a thing genuinely more than
#: a metre behind a sofa still goes unplaced until the rover looks from somewhere
#: with a clear view. That is the safe direction: the observation is kept, with
#: its picture and its bearing, and waits.
SEE_PAST_M = 1.0

#: How wrong an elevation is, in degrees, one standard deviation.
#:
#: **The bearing's own figure, because it is the same measurement.** An
#: elevation and a bearing come off one ray through one swept lens on one
#: gimbal, so the box term and the gimbal term behind `BEARING_SIGMA_DEG` apply
#: unchanged to both. What differs is what each is spared and what neither has
#: been asked:
#:
#:   the heading  belongs to the bearing alone. Which way the rover faces turns
#:                a ray about the world's vertical, which cannot change how high
#:                it points, so the 0.2 deg of heading drift is not in here --
#:                and neither is `sigma_of`'s turn-rate term, which is the same
#:                error taken while moving. An elevation measured on a rover
#:                spinning at ninety degrees a second is as good as one taken
#:                standing still, which is the one respect in which this is the
#:                better half of the ray.
#:   the tilt     is the pan servo's twin and has never been checked. The pan
#:                lands about three degrees short at the ends of its travel with
#:                no feedback to correct it, and there is no reason to think the
#:                tilt servo is better. **On every drive so far the tilt has been
#:                held at its rest position**, so whatever that error is, it is
#:                one constant shared by every observation -- it cancels out of
#:                any comparison between two of them, and does not cancel out of
#:                a height above the floor.
#:   the pitch    of the rover itself is not recorded anywhere. A flat floor
#:                makes it nothing; a threshold does not. The driver board's
#:                telemetry already carries it and nothing reads it.
#:
#: So the two terms that are measured are shared, one term is dropped in this
#: half's favour, and two are unmeasured -- which is why this is the bearing's
#: number rather than a smaller one. It is a separate name so that measuring the
#: tilt servo has somewhere to land.
ELEVATION_SIGMA_DEG = BEARING_SIGMA_DEG

#: Past this, an elevation stops saying anything useful about a height at a
#: horizontal range: the tangent runs away, and a ray pointing nearly at the
#: ceiling puts the thing anywhere from here to the roof. Rays this steep keep
#: their bearings and abstain from the vertical test.
MAX_ELEVATION_DEG = 80.0

#: The most an object's own height may forgive a rise, in metres, measured from
#: its middle. **Half a door**, which is the tallest thing in a room a rover can
#: see all of, and the vertical counterpart of `MAX_EXTENT_M` -- with the same
#: job, which is to stop a region spanning most of the frame claiming the whole
#: wall it was cut from.
MAX_RISE_EXTENT_M = 1.0

#: How high the camera sits above the floor, in metres, or None if nobody has
#: measured it.
#:
#: **None, and that is a missing tape measure rather than a missing idea.**
#: Nothing in this repository holds a height: `base_link` is defined at the
#: lidar, SLAM is two-dimensional, and no transform in the stack has a z in it.
#: So every height here is measured **from the camera** -- which is the quantity
#: the geometry actually uses, since two rays leaving the same mount disagree
#: vertically by the same amount whatever that mount's height is, and the
#: unknown cancels.
#:
#: What it does not do is let a person read "0.4 m above the floor" off the
#: console. Set this to the height of the gimbal's optical centre above the
#: ground and every height becomes floor-referenced; leave it None and they stay
#: camera-referenced and are labelled as such. The lever arm as the camera tilts
#: is a few centimetres and is deliberately not modelled: it is far below what
#: `ELEVATION_SIGMA_DEG` already allows.
CAMERA_HEIGHT_M = None


#: Where a bearing stops being evidence and starts being an outlier, in
#: standard deviations of its own noise.
#:
#: **The greedy pass already does this by hand, and doing it by hand is what it
#: is better at.** `locate.best_fix` picks the pair the other rays agree with and
#: `locate.refine` then fits over the agreeing rays only, which is an outlier
#: rejection written as a search. A plain least-squares fit has no such step, and
#: measured on the recording of 2026-09-03 that is the whole of its
#: disadvantage: one region drawn round a doorframe instead of the cabinet inside
#: it drags the centre, and the median bearing then misses by 2.3 degrees where
#: the greedy pass misses by 1.6.
#:
#: So the fit is given the standard version of the same idea -- a Huber loss,
#: which is what `gtsam::noiseModel::Robust` and Ceres' loss functions do -- and
#: the miss is measured from the edge of the thing's silhouette rather than from
#: its centre, so that a bearing landing legitimately on one end of a sideboard
#: is not mistaken for a bad box.
#:
#: 2.0 rather than the 1.345 that maximises efficiency against clean Gaussian
#: noise: the residuals here are not clean, and a two-sigma bearing on this rover
#: is an ordinary bearing rather than a suspect one.
HUBER_K = 2.0

#: How well a range measurement would be known, in metres, if a ray carried one.
#:
#: **Nothing produces one yet**, so this is unused and is here to be the shape of
#: the answer rather than a measurement. The figure is what the OAK-D-Lite's own
#: README implies for stereo at a few metres off a 7.5 cm baseline and should be
#: re-measured against the ground before it is believed. `_residuals` is the only
#: place it is read.
RANGE_SIGMA_M = 0.15


def beyond_reach(observer: dict[str, Any], point: tuple[float, float]) -> bool:
    """Whether this observer would have had to see through a wall.

    **This is the constraint the design was missing, and it comes from the map
    rather than from the picture.** A bearing has no range, so two bearings will
    cross *somewhere* -- and two cameras pointed at two different things a couple
    of metres away in two different rooms produce rays that cross ten metres away
    at a perfectly healthy angle off a perfectly healthy baseline. Every guard
    here passed such a crossing, because none of them asks the one question that
    settles it: could the rover have seen that far in that direction at all?

    It could not, and the rover already knows. The occupancy grid says where the
    first obstacle on a bearing is, and **you cannot see a thing through a
    wall** -- so the first obstacle bounds the range of every single sighting,
    from one look, with no triangulation involved. Measured on the run of
    2026-09-02: 19 of the 22 bearings that had been attached to a placed thing
    claimed something further away than the first wall along their own bearing,
    and two of the three things placed sat outside the edge of the map
    altogether. One of them was nine metres past a wall 55 cm in front of the
    rover.

    `reach_m` is put on the ray by whoever built it, because it belongs to the
    map at the moment of the decision rather than to the observation -- a map
    grows as the rover explores, and a bearing that could not be checked last
    time can be checked now. A ray without it is unbounded, which is what this
    module did before and is what the selftests get unless they say otherwise.
    """
    reach = observer.get("reach_m")
    if reach is None:
        return False
    range_m = math.hypot(point[0] - float(observer["x_m"]),
                         point[1] - float(observer["y_m"]))
    return range_m > float(reach) + SEE_PAST_M


def _unit(bearing_deg: float) -> tuple[float, float]:
    radians = math.radians(bearing_deg)
    return math.cos(radians), math.sin(radians)


def _range_to(x_m: float, y_m: float, ray: dict[str, Any]) -> float:
    """How far a point is from where a ray started, in metres."""
    return math.hypot(x_m - float(ray["x_m"]), y_m - float(ray["y_m"]))


def _bearing_to(x_m: float, y_m: float, ray: dict[str, Any]) -> float:
    """What bearing a point sits on, seen from where a ray started."""
    return math.degrees(math.atan2(y_m - float(ray["y_m"]),
                                   x_m - float(ray["x_m"])))


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

    **The shape of that error is reported as well as its size, and the difference
    between the two is what stopped one entity swallowing a room.** The four
    nudged points are not scattered evenly around the answer: a crossing taken at
    a shallow angle is uncertain a long way down the line of sight and precise
    across it, so the cloud is a cigar rather than a disc. `uncertainty_m` is the
    length of that cigar, and adding it to a tolerance measured *across* a later
    bearing charges the whole of a lengthways error to a sideways question. So
    the long axis, the width across it and the direction it points are recorded
    too, and `cross_track` is what reads them.
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
        # And not through a wall. See `beyond_reach`: this is the guard that
        # refuses a crossing invented out of two bearings pointed at two
        # different things in two different rooms, which every guard above
        # accepts.
        if beyond_reach(observer, point):
            return None

    # Each bearing is nudged by its *own* error rather than by the constant, so
    # a crossing made from a look taken while the rover was turning reports the
    # width that look actually earned. See `sigma_of`.
    first_sigma, second_sigma = sigma_of(first), sigma_of(second)
    moved = []
    for da in (-first_sigma, first_sigma):
        for db in (-second_sigma, second_sigma):
            nudged = _cross({**first,
                             "bearing_deg": float(first["bearing_deg"]) + da},
                            {**second,
                             "bearing_deg": float(second["bearing_deg"]) + db})
            if nudged is None:
                # A nudge that destroys the fix means the fix was marginal.
                return None
            moved.append((nudged[0] - point[0], nudged[1] - point[1]))
    # **Where the ray started is uncertain too, and it is charged here rather
    # than used to throw the look away.** A rover driving in a straight line
    # through a 0.36 s shutter covers 0.17 m, and the pose behind the bearing is
    # the midpoint of two readings, so the ray begins somewhere within half of
    # what it travelled. That shifts the whole ray sideways, which moves the
    # crossing by about as much in a direction nothing here can predict -- so it
    # widens the answer both ways rather than along one axis. See
    # `Inspector.MOVED_WHILE_LOOKING_M` for why this is now measured and kept
    # instead of being the reason 76% of a driven run recorded no bearing at all.
    origin_m = (float(first.get("origin_sigma_m") or NO_ORIGIN_ERROR_M)
                + float(second.get("origin_sigma_m") or NO_ORIGIN_ERROR_M))
    worst = max(math.hypot(dx, dy) for dx, dy in moved) + origin_m
    # The direction the error runs in is the furthest the point moved, and the
    # width is how far the cloud reaches either side of that. Four points is a
    # bounding box rather than a covariance, which is the honest amount of shape
    # to claim from four samples and errs towards the generous.
    long_dx, long_dy = max(moved, key=lambda one: math.hypot(*one))
    axis_deg = math.degrees(math.atan2(long_dy, long_dx))
    ux, uy = _unit(axis_deg)
    across = max(abs(-dx * uy + dy * ux) for dx, dy in moved) + origin_m
    placed = {
        "x_m": round(point[0], 3),
        "y_m": round(point[1], 3),
        "uncertainty_m": round(worst, 3),
        # The same error as a shape rather than a radius. `cross_track` reads
        # these; `uncertainty_m` stays because it is what the console shows and
        # what a person means by "to within".
        "error_major_m": round(worst, 3),
        "error_minor_m": round(across, 3),
        "error_major_deg": round(axis_deg, 1),
        "baseline_m": round(baseline_m(first, second), 3),
        "parallax_deg": round(parallax_deg(first, second), 1),
        # How wide the thing itself is, measured rather than assumed. See
        # `extent_of`: this is what a later bearing is allowed to be off by, and
        # it has to be a property of the thing rather than of whatever crop is
        # asking to join it.
        "extent_m": round(extent_of(point, first, second), 3),
    }

    # **And the two rays have to agree about how high it is, not only about
    # where on the floor it is.** Everything above this line is a plan view, in
    # which a bearing at a picture on the wall and a bearing at the sideboard
    # under it cross as convincingly as two bearings at one thing. The elevation
    # was measured off the same ray as the bearing and costs nothing to spend,
    # and it is an axis nothing else in here can see: appearance cannot separate
    # two objects on this rover, and a crossing cannot separate two heights.
    # Rays that recorded no elevation abstain rather than refuse, which is every
    # look the rover took before this was kept.
    vertical = rise_disagreement(placed, first, second)
    if vertical is not None:
        if vertical[0] > vertical[1]:
            return None
        placed.update(height_fields(placed, [first, second]))
    return placed


def noise_deg(x_m: float, y_m: float, ray: dict[str, Any]) -> float:
    """How wrong this bearing might be, in degrees. Measurement error only.

    Two terms: how well the bearing itself is known, and where the ray started.
    The second is a distance and becomes an angle at the range the thing sits at,
    so this is re-evaluated as the fit moves -- which makes the whole thing
    iteratively reweighted least squares, the ordinary way a range-dependent
    error is handled.

    **The thing's own width is deliberately not in here**, and keeping it out is
    a correction rather than a simplification. See `silhouette_deg`.
    """
    range_m = max(_range_to(x_m, y_m, ray), MIN_RANGE_M)
    origin = math.degrees(math.atan2(
        float(ray.get("origin_sigma_m") or NO_ORIGIN_ERROR_M), range_m))
    return math.hypot(sigma_of(ray), origin)


def silhouette_deg(x_m: float, y_m: float, ray: dict[str, Any],
                    extent_m: float) -> float:
    """How much of the bearing's own error is the thing simply being wide.

    Half the thing's width, as an angle at the range it sits at. A bearing
    landing anywhere within a wardrobe's silhouette is pointing at the wardrobe,
    and this is the allowance for that -- the same term `locate.match_tolerance`
    adds in metres, in the units a bearing residual lives in.

    **It is subtracted from the miss rather than added to the noise, and the
    difference is not cosmetic.** Added to the noise it also divides the
    likelihood, because a wider Gaussian is a lower one; so a wide thing close to
    the camera scored *worse* than the hypothesis that the region is nothing at
    all, however precisely the bearing landed on it. Measured on the recording of
    2026-09-03 that cost 12 of the 15 things the greedy pass places -- a
    three-quarter-metre object at a metre spreads over 41 degrees, and the peak
    of a 41-degree Gaussian is below `CLUTTER_PER_DEG`.

    The consequence, stated because it is a real departure from the textbook
    mixture: subtracting it leaves the likelihood unnormalised, so it is a score
    rather than a probability. Every ratio this module takes is between things
    measured the same way, so the ratios are unaffected, and the alternative is a
    model in which a sideboard cannot be seen.
    """
    range_m = max(_range_to(x_m, y_m, ray), MIN_RANGE_M)
    return math.degrees(math.atan2(max(0.0, extent_m) / 2.0, range_m))


# **Where the thing's own width is used, and where it deliberately is not.**
#
# It enters the association likelihood, as a miss the bearing is forgiven --
# `silhouette_deg`, and without that a sideboard loses its own rays to the
# hypothesis that they are scenery.
#
# It does **not** enter the position fit, and that is a decision rather than an
# omission. Weighting the least squares by noise-plus-silhouette is defensible
# on its own terms -- two looks at either end of a wardrobe really do disagree
# about which way it lies -- but the covariance that comes out then answers "how
# well is the centre of a thing this big known", which is a different quantity
# from the one the rest of the component calls `uncertainty_m`. Measured against
# the 15 entities the greedy pass places on the recording of 2026-09-03, it runs
# about twice their stated figure, and `locate.cross_track` and
# `match_tolerance` would spend the difference as slack the geometry never
# earned. That is the fault the README's own "the term this replaces was the
# largest in every match decision" section is about, arrived at from the other
# direction.
#
# So the fit is weighted by noise alone, `uncertainty_m` stays the same quantity
# it has always been, and `extent_m` is reported beside it for
# `match_tolerance` to add exactly as it does today.


def residuals(x_m: float, y_m: float, ray: dict[str, Any]
               ) -> list[tuple[float, float, float]]:
    """What this ray says about a thing here, as `(residual, d/dx, d/dy)` terms.

    Each term is already divided by its own standard deviation, so a sum of their
    squares is a chi-square and the normal matrix built from them inverts to a
    covariance without further scaling.

    **One term today and two when the depth camera is read.** The bearing term is
    the angle between where the thing would be and where the ray pointed. The
    range term -- how far the thing is against how far the ray said it was -- is
    written out below and is skipped whenever a ray carries no `range_m`, which
    is every ray this rover has ever recorded. It is here rather than in a plan
    because the whole argument for fitting positions this way instead of crossing
    pairs is that a range costs one residual, and an argument like that should be
    checked against the code.
    """
    dx = x_m - float(ray["x_m"])
    dy = y_m - float(ray["y_m"])
    squared = dx * dx + dy * dy
    if squared < 1e-12:
        return []
    terms = []

    # Which way the thing lies, against which way the ray pointed.
    sigma = noise_deg(x_m, y_m, ray)
    residual = _wrap(math.degrees(math.atan2(dy, dx))
                     - float(ray["bearing_deg"]))
    scale = math.degrees(1.0) / squared
    terms.append((residual / sigma,
                  (-dy * scale) / sigma,
                  (dx * scale) / sigma))

    # How far the thing is, against how far the ray said. Nothing writes
    # `range_m` yet; see the module docstring and `RANGE_SIGMA_M`.
    measured = ray.get("range_m")
    if measured is not None:
        range_m = math.sqrt(squared)
        sigma_m = float(ray.get("range_sigma_m") or RANGE_SIGMA_M)
        terms.append(((range_m - float(measured)) / sigma_m,
                      (dx / range_m) / sigma_m,
                      (dy / range_m) / sigma_m))
    return terms


def robust_weight(x_m: float, y_m: float, ray: dict[str, Any],
            extent_m: float) -> float:
    """How much to trust this bearing, given how far it misses. Huber.

    Full weight while the bearing lands within `HUBER_K` noise widths of the
    thing's silhouette, then falling away as the reciprocal of the miss, which is
    what makes one badly drawn box cost the fit a little instead of everything.

    **The miss is measured from the silhouette and the residual is not**, and
    both are deliberate. A sideboard seen from its two ends gives two bearings
    that genuinely straddle it, and the fit should settle between them -- so the
    residual it minimises is the full miss from the centre. What the silhouette
    decides is only whether such a bearing is *suspect*, and a bearing landing on
    the thing never is.
    """
    noise = noise_deg(x_m, y_m, ray)
    half = silhouette_deg(x_m, y_m, ray, extent_m)
    off = abs(_wrap(math.degrees(math.atan2(y_m - float(ray["y_m"]),
                                            x_m - float(ray["x_m"])))
                    - float(ray["bearing_deg"])))
    missed = max(0.0, off - half) / noise
    return 1.0 if missed <= HUBER_K else HUBER_K / missed


def fit_over(rays: list[dict[str, Any]], weights: list[float],
         start: tuple[float, float], extent_m: float = 0.0
         ) -> dict[str, Any] | None:
    """Where a thing must be for all these rays, with the covariance of that.

    A damped Gauss-Newton over two unknowns, written out rather than handed to
    `scipy.optimize.least_squares`. **That is a judgement and it went the other
    way for the assignment problem next door**, so it is worth saying why: two
    unknowns with an analytic Jacobian is a 2x2 normal matrix and a closed-form
    inverse, which is less code than wiring a general solver up to it, and the
    inverse *is* the covariance the caller needs. Bringing in a dependency here
    would buy nothing and would add a way for the rover to have no answer.
    `bench_cluster.py` checks this solver against scipy's at a desk, which is
    where a cross-check belongs.

    The damping matters more than it looks. Rays that are nearly parallel make
    the normal matrix nearly singular, and an undamped step then throws the point
    kilometres away and never comes back. Damped, it converges to the honest
    answer -- a position with an enormous long axis -- and the caller refuses it
    for that.
    """
    x_m, y_m = start
    lam = 1e-6
    previous = None
    for _round in range(64):
        hxx = hxy = hyy = gx = gy = 0.0
        chi = 0.0
        used = 0
        for ray, weight in zip(rays, weights):
            if weight <= 0.0:
                continue
            weight = weight * robust_weight(x_m, y_m, ray, extent_m)
            if weight <= 0.0:
                continue
            for residual, jx, jy in residuals(x_m, y_m, ray):
                root = math.sqrt(weight)
                residual, jx, jy = residual * root, jx * root, jy * root
                hxx += jx * jx
                hxy += jx * jy
                hyy += jy * jy
                gx += jx * residual
                gy += jy * residual
                chi += residual * residual
                used += 1
        if used < 2:
            return None
        if previous is not None and abs(previous - chi) < 1e-12:
            break
        previous = chi
        determinant = (hxx + lam) * (hyy + lam) - hxy * hxy
        if abs(determinant) < 1e-18:
            return None
        step_x = -((hyy + lam) * gx - hxy * gy) / determinant
        step_y = -((hxx + lam) * gy - hxy * gx) / determinant
        # A step longer than the room is the near-singular case above. Clamp it
        # rather than refuse: the next round with more damping usually recovers,
        # and if it does not the covariance says so.
        length = math.hypot(step_x, step_y)
        if length > MAX_RANGE_M:
            step_x *= MAX_RANGE_M / length
            step_y *= MAX_RANGE_M / length
            lam *= 10.0
        else:
            lam = max(1e-9, lam * 0.5)
        x_m += step_x
        y_m += step_y
        if length < 1e-5:
            break

    # The covariance, from the normal matrix evaluated **at the answer** rather
    # than at the point the last step started from. One step of difference is
    # usually nothing and is not always nothing: a fit that stopped because it
    # hit the step clamp is exactly the ill-conditioned case whose covariance
    # matters most, and there the two differ a great deal.
    hxx = hxy = hyy = chi = 0.0
    used = 0
    for ray, weight in zip(rays, weights):
        if weight <= 0.0:
            continue
        weight = weight * robust_weight(x_m, y_m, ray, extent_m)
        if weight <= 0.0:
            continue
        for residual, jx, jy in residuals(x_m, y_m, ray):
            root = math.sqrt(weight)
            residual, jx, jy = residual * root, jx * root, jy * root
            hxx += jx * jx
            hxy += jx * jy
            hyy += jy * jy
            chi += residual * residual
            used += 1
    if used < 2:
        return None
    # The residuals were divided by their own sigmas, so this needs no further
    # scaling -- and where it is enormous, that is the answer rather than a bug.
    determinant = hxx * hyy - hxy * hxy
    if abs(determinant) < 1e-18:
        return None
    cxx, cxy, cyy = hyy / determinant, -hxy / determinant, hxx / determinant
    # Eigenvalues of a symmetric 2x2, which are the squared axes of the error
    # ellipse, and the angle of the long one.
    middle = (cxx + cyy) / 2.0
    radius = math.hypot((cxx - cyy) / 2.0, cxy)
    major = math.sqrt(max(0.0, middle + radius))
    minor = math.sqrt(max(0.0, middle - radius))
    angle = 0.5 * math.degrees(math.atan2(2.0 * cxy, cxx - cyy))
    return {"x_m": x_m, "y_m": y_m,
            "error_major_m": major, "error_minor_m": minor,
            "error_major_deg": _wrap(angle),
            "chi_square": chi, "terms": used}


def cross_track(point: dict[str, Any], ray: dict[str, Any]) -> float:
    """How uncertain this thing's position is *across* this particular bearing.

    **The term this replaces was the largest in every match decision the rover
    made, and it was the wrong number.** A crossing taken at a shallow angle is
    uncertain a long way down its own line of sight and precise across it, and
    `uncertainty_m` reports the long way. Charged to a tolerance measured across
    a later bearing it buys slack the geometry never claimed: on the run of
    2026-09-03 an entity placed at 13.9 degrees of parallax carried 0.46 m of it,
    which at two metres is a cone 46 degrees wide against a bearing the geometry
    believes to a degree and a half. That cone is what collected a cabinet, two
    framed pictures, a doorway, a table and a person's head into one thing.

    A placement written before the shape was recorded has only the radius, and
    gets it -- the old behaviour, for rows the rover already holds.
    """
    major = point.get("error_major_m")
    if major is None:
        return float(point.get("uncertainty_m", 0.0))
    minor = float(point.get("error_minor_m", major))
    # The angle between the error's long axis and the line this ray looks along.
    # Side on, the ray sees the whole length of it; end on, only its width.
    between = (math.radians(float(point.get("error_major_deg", 0.0)))
               - math.atan2(float(point["y_m"]) - float(ray["y_m"]),
                            float(point["x_m"]) - float(ray["x_m"])))
    return math.hypot(float(major) * math.sin(between), minor * math.cos(between))


def along_track(point: dict[str, Any], ray: dict[str, Any]) -> float:
    """How uncertain this thing's position is *along* this particular bearing.

    `cross_track`'s other half, and it is wanted for the same reason: the error
    is a cigar rather than a disc, and a height read off an elevation is a range
    multiplied by a tangent, so what it spends is the doubt running down the
    sight line rather than the doubt across it.
    """
    major = point.get("error_major_m")
    if major is None:
        return float(point.get("uncertainty_m", 0.0))
    minor = float(point.get("error_minor_m", major))
    between = (math.radians(float(point.get("error_major_deg", 0.0)))
               - math.atan2(float(point["y_m"]) - float(ray["y_m"]),
                            float(point["x_m"]) - float(ray["x_m"])))
    return math.hypot(float(major) * math.cos(between), minor * math.sin(between))


def rise_m(point: dict[str, Any], ray: dict[str, Any]) -> float | None:
    """How far above the camera this ray says the thing at `point` is, or None.

    **A height needs a range, and a bearing has none** -- which is why this
    takes a point rather than a ray alone, and why the elevation is a
    measurement the resolver spends *after* something has been placed rather
    than a second way of placing it. Once the crossing says how far away the
    thing is, the angle the ray was pointing above the horizontal says how far
    above the camera it is, and that is one tangent.

    None whenever the answer would be dishonest: a look that recorded no
    elevation, which is every look taken before the vertical half of the ray was
    kept, and a ray steeper than `MAX_ELEVATION_DEG`, where the tangent runs
    away faster than the range is known.

    Above the *camera*, not above the floor. See `CAMERA_HEIGHT_M`.
    """
    elevation = ray.get("elevation_deg")
    if elevation is None:
        return None
    try:
        elevation = float(elevation)
    except (TypeError, ValueError):
        return None
    if abs(elevation) > MAX_ELEVATION_DEG:
        return None
    range_m = math.hypot(float(point["x_m"]) - float(ray["x_m"]),
                         float(point["y_m"]) - float(ray["y_m"]))
    return range_m * math.tan(math.radians(elevation))


def rise_tolerance_m(point: dict[str, Any], ray: dict[str, Any]) -> float:
    """How far off a rise may be and still be this thing, in metres.

    Four terms, and they are the vertical mirror of `match_tolerance`'s three.
    How wrong the elevation itself is, at the range the thing sits at. How
    uncertain the range is -- which is `along_track`, turned into a height by
    the same tangent, because being wrong about how far away a thing is moves
    where a sloping ray says it is. Where the ray started, for the same reason.
    And how tall the thing itself is, because two looks at either end of a
    doorway centre on different parts of it, exactly as two looks at either end
    of a sideboard do.

    **A box the frame cut the top or bottom off gets the whole allowance**, and
    that is the correction that keeps this honest. The camera stares level down
    the rover's nose at things taller than it, so a doorway or a wardrobe runs
    off the top of the picture routinely -- and a clipped box's middle is
    wherever the frame happened to cut, which moves as the rover drives towards
    it. Measuring a height off that centre and then believing it to a handspan
    would throw away the very looks that see a thing best. See
    `view.clipped_vertically`.
    """
    return rise_noise_m(point, ray) + rise_extent_m(point, ray)


def rise_noise_m(point: dict[str, Any], ray: dict[str, Any]) -> float:
    """How wrong a rise could be, in metres, on measurement error alone.

    Three of `rise_tolerance_m`'s four terms and deliberately not the fourth:
    how wrong the elevation is at this range, how wrong the range itself is
    turned into a height by the ray's own slope, and where the ray started. What
    is left out is how tall the thing is, which is not an error in anything --
    it is a miss the geometry forgives, and `rise_extent_m` is where it lives.

    **The split is what stops the thing's height being charged twice.** It is
    the vertical statement of the argument `match_tolerance` already makes
    horizontally: the allowance for a wide thing belongs to the ray asking to
    join it, once, and a placement's own doubt must not carry a copy of it. With
    both, an entity founded on a box the frame had cut claimed a full metre of
    slack in `height_sigma_m` and was then offered another metre by every ray
    that came near it -- and on the drive of 2026-09-03 that is how nine of
    thirty-eight things came to span more than a metre with the gate switched
    on.
    """
    range_m = math.hypot(float(point["x_m"]) - float(ray["x_m"]),
                         float(point["y_m"]) - float(ray["y_m"]))
    slope = abs(math.tan(math.radians(
        min(abs(float(ray.get("elevation_deg") or 0.0)), MAX_ELEVATION_DEG))))
    return (range_m * math.tan(math.radians(ELEVATION_SIGMA_DEG))
            + along_track(point, ray) * slope
            + float(ray.get("origin_sigma_m") or NO_ORIGIN_ERROR_M) * slope)


def rise_extent_m(point: dict[str, Any], ray: dict[str, Any]) -> float:
    """How much of a vertical miss this crop's own height forgives, in metres.

    Half of how tall the thing looked, at the range it sits at -- the vertical
    twin of the `extent_m` term in `match_tolerance`, and there for the same
    reason: two looks at either end of a doorway centre their boxes on different
    parts of it, and a bearing landing anywhere within a thing's silhouette is
    pointing at the thing.

    **A box the frame cut the top or bottom off gets the whole allowance.** The
    camera stares level down the rover's nose at things taller than it, so a
    doorway or a wardrobe runs off the top of the picture routinely, and a
    clipped box's middle sits wherever the frame happened to cut -- which moves
    as the rover drives towards it. Measuring a height off that centre and then
    believing it to a handspan would throw away the very looks that see a thing
    best. 77 of the 459 regions of the drive of 2026-09-03 are cut this way. See
    `view.clipped_vertically`.
    """
    if ray.get("elevation_clipped"):
        return MAX_RISE_EXTENT_M
    range_m = math.hypot(float(point["x_m"]) - float(ray["x_m"]),
                         float(point["y_m"]) - float(ray["y_m"]))
    span_deg = float(ray.get("elevation_span_deg") or 0.0)
    own = range_m * math.tan(math.radians(min(span_deg, 90.0) / 2.0))
    return min(MAX_RISE_EXTENT_M, max(0.0, own))


def rise_disagreement(point: dict[str, Any], first: dict[str, Any],
                      second: dict[str, Any]) -> tuple[float, float] | None:
    """How far apart two rays put a thing vertically, and how far apart they are
    allowed to be. None when either of them measured no height.

    **This is the test a plan-view crossing cannot make, and it is free.** Two
    bearings that cross beautifully seen from above can be pointing at things a
    metre apart in height -- a picture on the wall and the sideboard beneath it,
    a doorway and the floor in front of it -- and until this existed nothing
    looked. It costs the rover nothing further to answer, because the elevation
    was measured off the same ray as the bearing and thrown away.

    **The camera's own height is not needed and deliberately not used.** Both
    rays leave the same mount, so whatever height that mount is at cancels out
    of the difference between them. That is what makes this usable today rather
    than after somebody has been round the rover with a tape measure.
    """
    here, there = rise_m(point, first), rise_m(point, second)
    if here is None or there is None:
        return None
    return (abs(here - there),
            rise_tolerance_m(point, first) + rise_tolerance_m(point, second))


def height_over(point: dict[str, Any], rays: list[dict[str, Any]]
                ) -> tuple[float, float] | None:
    """How high the thing at `point` is above the camera, and how well that is
    known, from every ray that measured it. None if none did.

    The middle of what the rays say rather than a weighted fit: the spread
    between them is dominated by where on the object each box happened to be
    centred, which is not a Gaussian and is not independent between two looks at
    the same face of a wardrobe. The median is also what keeps one badly cut box
    from dragging the answer, which is the job `HUBER_K` does for the position.

    **The doubt is measurement error on the best-placed look, and deliberately
    neither the spread nor the thing's own height.** Both of the other two are
    self-defeating, because `stands_as_high` spends this figure as slack. The
    spread lets an entity that has admitted one look at the wrong height widen
    its own gate and admit the next; the thing's height is already forgiven, per
    ray, by `rise_extent_m`, so counting it again offers two metres of slack to
    anything founded on a box the frame had cut. Measured on the drive of
    2026-09-03, the two together left nine of thirty-eight things spanning more
    than a metre with the gate switched on, every one of them a look at
    something else rather than a tall thing seen properly.

    **A look the frame cut is the exception, and the rover found it rather than
    the replay.** Asked for a fresh inspection on 2026-09-04, the store came back
    with `object:14` standing 3.58 m above the camera give or take 0.23 -- off
    one box whose top edge was two thousandths of a frame from the ceiling of the
    picture. Where on that thing the box was centred is exactly what a clipped
    box does not know, and unlike every other ray's version of that doubt there
    is nothing later to forgive it: `rise_extent_m` allows for the crop that is
    *joining*, and this is the crop the height was taken from. So a clipped look
    carries its own allowance into the figure it claims, and an entity with any
    uncut look in it is unaffected -- the tightest still wins.
    """
    seen = []
    for ray in rays:
        got = rise_m(point, ray)
        if got is None:
            continue
        # See the docstring: what a clipped box cannot say about itself has to be
        # charged here, because no later ray charges it.
        cut = rise_extent_m(point, ray) if ray.get("elevation_clipped") else 0.0
        seen.append((got, rise_noise_m(point, ray) + cut))
    if not seen:
        return None
    heights = sorted(one for one, _ in seen)
    middle = (heights[len(heights) // 2] if len(heights) % 2
              else (heights[len(heights) // 2 - 1]
                    + heights[len(heights) // 2]) / 2.0)
    return middle, min(noise for _, noise in seen)


def stands_as_high(point: dict[str, Any], ray: dict[str, Any]) -> bool:
    """Whether this ray puts the thing at the height the placement claims.

    True whenever the question cannot be asked -- a placement with no height, a
    look that measured none, a ray too steep to read one off. **Silence is
    agreement here and that is deliberate**: this is a removal-only gate, the
    same shape as the appearance floor, and it must never be the reason a rover
    that has just been upgraded refuses everything it knew yesterday.

    What it is allowed to spend is `rise_tolerance_m` plus how well the
    placement's own height is known, which is the same pairing of "how wrong
    could this ray be" with "how wrong could the thing be" that
    `match_tolerance` makes horizontally.
    """
    claimed = point.get("height_m")
    if claimed is None:
        return True
    got = rise_m(point, ray)
    if got is None:
        return True
    allowed = (rise_tolerance_m(point, ray)
               + float(point.get("height_sigma_m") or 0.0))
    return abs(got - float(claimed)) <= allowed


def above_floor_m(height_m: float | None) -> float | None:
    """A height above the camera as a height above the floor, or None.

    None whenever `CAMERA_HEIGHT_M` has not been measured, which is what the
    console reads to decide which of the two it is showing. Answering the
    camera-referenced number under a floor-referenced label would be the one
    way this could lie.
    """
    if height_m is None or CAMERA_HEIGHT_M is None:
        return None
    return float(height_m) + float(CAMERA_HEIGHT_M)


def height_fields(point: dict[str, Any], rays: list[dict[str, Any]]) -> dict:
    """The height part of a placement, from every ray that measured one.

    Empty when none did, which is how a placement written from looks the rover
    took before the vertical half of the ray was kept comes out exactly as it
    always did -- and how `stands_as_high` knows to ask nothing of it.

    Two numbers or three. `height_m` is always above the camera, because that is
    the quantity the geometry has; `height_above_floor_m` appears only once
    somebody has measured `CAMERA_HEIGHT_M`, and the console shows whichever it
    is given under the label that is true of it.
    """
    found = height_over(point, rays)
    if found is None:
        return {}
    fields = {"height_m": round(found[0], 3),
              "height_sigma_m": round(found[1], 3)}
    floor = above_floor_m(found[0])
    if floor is not None:
        fields["height_above_floor_m"] = round(floor, 3)
    return fields


def extent_of(point: tuple[float, float], *observers: dict[str, Any]) -> float:
    """Half the width of the thing at `point`, from the crops that saw it.

    A region's angular width, at the range the crossing puts it, is a
    measurement of how wide the thing is. **The smaller of the two views wins**,
    which is the conservative direction and the deliberate one: the region finder
    segments parts as readily as wholes, so one view of a picture on a wall can
    come back as the picture and the other as the whole wall panel around it, and
    the tighter of the two is the better claim about the object.
    """
    widths = []
    for observer in observers:
        span_deg = float(observer.get("span_deg") or 0.0)
        if span_deg <= 0.0:
            continue
        range_m = math.hypot(point[0] - float(observer["x_m"]),
                             point[1] - float(observer["y_m"]))
        widths.append(range_m * math.tan(math.radians(min(span_deg, 90.0) / 2.0)))
    if not widths:
        return 0.0
    return min(MAX_EXTENT_M, min(widths))


#: How far past its own doubt a fit may move a placement before it is treated as a
#: different answer rather than a better one, in metres. A handspan, which is the
#: same statement about rooms `resolve.SAME_PLACE_M` makes and for the same
#: reason: two positions this close name the same chair whichever is right, and
#: further than this the rays being fitted are not all looking at one thing.
#: Choosing between answers is the resolver's job and not this function's.
REFINE_LIMIT_M = 0.5

#: The most an object's own width may add to the tolerance for pointing at it.
#: A region spanning most of the frame is a wall or a floor, and letting that
#: claim three metres of slack would let it swallow the room.
#:
#: **It used to be doing most of the work rather than capping it.** Measured over
#: the 54,607 tolerance decisions of one run, the median total was 1.00 m of which
#: 0.75 was this cap, saturated -- because the term was taken from whichever crop
#: was asking to join rather than from the thing it wanted to join. It is a cap
#: again now: `extent_of` measures the thing when it is placed and the tolerance
#: uses that.
MAX_EXTENT_M = 0.75


def match_tolerance(point: dict[str, Any], ray: dict[str, Any]) -> float:
    """How far off a bearing may be and still be pointing at this thing.

    Three terms: how wrong the bearing is, how uncertain the thing's position is,
    and how big the thing actually is. The third one matters because an entity is
    stored as a point while a television is a metre wide, and two looks from
    different sides of it centre on different parts of it -- so a bearing landing
    anywhere within the thing's own silhouette is pointing at it, however
    precisely the bearing itself is known. Without it, matching a television at
    two and a half metres allowed 0.115 m, a tenth of the television; looks that
    should have joined it were refused, fell through to the pairing pass and made
    a *second* television eight centimetres from the first.

    **The width is the thing's own and not the candidate's, which is a fix.** It
    used to come from whichever crop was asking to join, capped at
    `MAX_EXTENT_M` -- and the cap saturated, so more than half of every match
    decision on the rover was made with three quarters of a metre of slack
    whatever the thing was. Expressed as an angle that is a cone eleven degrees
    wide in the median against a bearing measured to one and a half, and it let
    any wide region claim any small thing in roughly its direction. On the driven
    run of 2026-09-02 it cost exactly that: one entity a metre and a half from a
    parked rover collected thirteen bearings spanning fifty-three degrees -- a
    ceiling corner, a dark doorway, a framed picture and a wall panel, all one
    thing.

    So `extent_m` is measured when the thing is placed and travels with the
    placement, and the candidate's own span is only a fallback for a placement
    written before this existed.

    **And the second term is the placement's error across this ray rather than
    its whole length**, which is `cross_track` and which is the fix for the run
    of 2026-09-03. Measured there, the tolerance was 0.82 m at two metres, of
    which the bearing -- the only term that is about this ray at all -- was 6%.
    """
    range_m = math.hypot(float(point["x_m"]) - float(ray["x_m"]),
                         float(point["y_m"]) - float(ray["y_m"]))
    own = point.get("extent_m")
    if own is None:
        span_deg = float(ray.get("span_deg") or 0.0)
        own = range_m * math.tan(math.radians(min(span_deg, 90.0) / 2.0))
    extent_m = min(MAX_EXTENT_M, max(0.0, float(own)))
    return (range_m * math.tan(math.radians(sigma_of(ray)))
            + cross_track(point, ray) + extent_m
            # And where this ray started, which is a sideways error like the
            # bearing's own and belongs on the same side of the comparison. A
            # look taken while the rover was driving must not be refused for
            # missing by less than its own starting point is known to.
            + float(ray.get("origin_sigma_m") or NO_ORIGIN_ERROR_M))


def best_fix(rays: list[dict[str, Any]]) -> dict[str, Any] | None:
    """The most trustworthy fix obtainable from a set of bearings, or None.

    A pair rather than a least-squares fit over all of them: a fit whose errors
    are dominated by one bad bounding box is worse than the best honest pair, and
    a pair stays explainable -- the popup can name the two looks that placed the
    thing.

    **Which pair is the one the rest of the rays agree with, and that is a fix
    rather than a preference.** It used to be the pair with the smallest
    uncertainty, which is a statement about two rays and about nothing else: an
    entity with a dozen looks behind it would move to wherever the luckiest two of
    them happened to cross, out from under everything already attached to it.
    Measured on the drive of 2026-09-02, 13 of 151 placements moved more than half
    a metre when a new look arrived, one of them 2.6 m in a single step, and
    afterwards 45% of every entity's own rays missed its own stated position.
    Counting agreement first takes that to 26% on the same recording.

    Agreement is counted in **rays and not in viewpoints**, which is the opposite
    of how `_place_one` counts support, and the two are answering different
    questions. There, a pool of unattached bearings is being searched for
    something to place, and a phantom close to the camera collects agreement from
    half the room -- so support has to be counted in looks. Here every ray already
    belongs to this one thing, and a second look at it from the same standstill is
    a real second opinion about which direction it lies in. Measured on the drive
    of 2026-09-02, counting rays leaves 26% of an entity's bearings missing its
    own position where counting viewpoints leaves 37%.

    Uncertainty still breaks the tie, so nothing changes for an entity with only
    two looks behind it.

    **How much independent evidence stands behind the answer travels with it**, as
    `rays_agreeing` and `viewpoints`. Until this was recorded there was no way to
    tell a sideboard photographed from eight places apart from two dark blobs seen
    from two -- the console showed a count of observations, which lumps six looks
    taken from one standstill together with two from opposite sides of a room. The
    two numbers are kept separate because they answer different questions: rays
    are how much the thing was looked at, and viewpoints are from how many places,
    which is the one that says whether the geometry was ever tested.
    """
    best = None
    for index, first in enumerate(rays):
        for second in rays[index + 1:]:
            found = fix(first, second)
            if found is None:
                continue
            agreeing = [ray for ray in rays if agrees(found, ray)]
            rank = (-len(agreeing), found["uncertainty_m"])
            if best is None or rank < best[0]:
                found["rays_agreeing"] = len(agreeing)
                found["viewpoints"] = standing_places(agreeing)
                best = (rank, found)
    return None if best is None else best[1]


def standing_places(rays: list[dict[str, Any]]) -> int:
    """From how many genuinely different places these rays were taken.

    **Counted in places and not in looks, and the difference is the whole point
    of recording it.** A count of observations says a thing was looked at a lot;
    what says whether its position was ever tested is how far apart the looks
    were, because rays from one standstill share an origin exactly and cross
    nowhere. On the drive of 2026-09-03 one entity carried ten agreeing rays from
    two places 0.42 m apart, which by any count of looks was the best-evidenced
    thing in the room and by this one is a thing seen twice from the doorway.

    Two rays closer together than `MIN_BASELINE_M` are one place, which is the
    same line `fix` already draws when it refuses to cross them.
    """
    places: list[tuple[float, float]] = []
    for ray in rays:
        x_m, y_m = float(ray["x_m"]), float(ray["y_m"])
        if not any(math.hypot(x_m - px, y_m - py) < MIN_BASELINE_M
                   for px, py in places):
            places.append((x_m, y_m))
    return len(places)


def refine(point: dict[str, Any], rays: list[dict[str, Any]]) -> dict[str, Any]:
    """Move a placement to the point every agreeing ray is nearest to.

    **The pair stays what chose it and this only adjusts where it landed**, which
    is the whole reason a fit is allowed here at all. `best_fix` argues against
    fitting over every ray, and it is right: a fit whose errors are dominated by
    one bad bounding box is worse than the best honest pair. What it is fitting
    over here is only the rays that already agree with the pair's answer, so a bad
    box has been excluded before this runs rather than being averaged in.

    **The arithmetic is `fit_over`, which is a change and it is the measured
    one.** What this did before was the least-squares point of closest approach
    to a set of lines -- each ray contributing the part of the error square that
    is across it -- and it had two faults that only showed up once there were
    entities with nine bearings behind them. It minimised a *distance* across
    each ray, so a look taken five metres away counted for eight times as much as
    one taken at one metre, when the error being minimised is an angle and is the
    same size at both. And it weighted every ray alike, so a bearing from a rover
    turning at ninety degrees a second counted as much as one taken standing
    still, which since the shutter fix is a thing that happens.

    Measured over the 15 entities of the recording of 2026-09-03, with the same
    rays and the same associations: the worst bearing missed its own entity's
    position by 48.9 degrees before and by 15.0 after, and the median was
    unchanged at 1.5. One entity, `object:4`, went from 49.0 to 8.7.

    **The uncertainty is still not narrowed for the extra rays, deliberately, and
    the fit's own covariance is deliberately not used for it.** Shrinking it by
    the root of how many there are would assume their errors are independent, and
    on this rover they are mostly not: a bearing is dominated by the gimbal not
    arriving where it was told and by the heading SLAM reports, which are one
    mistake per look rather than one per ray. `fit_over` returns a covariance
    that does assume independence, so what is taken from it is the *shape* of the
    error -- which way it runs and how flat it is, which the pair's four nudged
    copies could only guess at once the fit had moved -- while the size stays the
    measured spread with the pair's own figure as a floor. So the number can grow
    when the evidence disagrees and never shrinks on a promise.
    """
    usable = [ray for ray in rays if agrees(point, ray)]

    def with_height(where: dict[str, Any]) -> dict[str, Any]:
        """The same placement, with its height taken from everything that now
        agrees with it.

        **A height has to improve as the evidence does, exactly as the position
        does.** Left at whatever the founding pair said, it would be a claim from
        two looks that a dozen later ones were then measured against -- and since
        `stands_as_high` refuses a look that disagrees with it, a founding pair
        that centred its boxes low would go on refusing every honest look at the
        top of the thing for ever.
        """
        return {**where, **height_fields(where, usable or rays)}

    if len(usable) < 3:
        return with_height(point)
    fitted = fit_over(usable, [1.0] * len(usable),
                      (float(point["x_m"]), float(point["y_m"])),
                      float(point.get("extent_m") or 0.0))
    if fitted is None:
        return with_height(point)
    x_m, y_m = fitted["x_m"], fitted["y_m"]
    moved = math.hypot(x_m - float(point["x_m"]), y_m - float(point["y_m"]))
    if moved > float(point.get("uncertainty_m", 0.0)) + REFINE_LIMIT_M:
        # Further than the pair's own doubt plus a handspan is not a refinement,
        # it is a different answer, and this is not the function that chooses
        # between answers. Leave the pair's.
        return with_height(point)
    spread = math.sqrt(sum(
        cross_track_of(x_m, y_m, ray) ** 2 for ray in usable) / len(usable))
    major_m = max(point.get("error_major_m", point["uncertainty_m"]), spread)
    # The shape from the fit and the size from the spread. The ratio of the
    # covariance's two axes says how flat the error really is and which way it
    # runs, which is what `cross_track` reads and what the founding pair's four
    # nudged copies stopped describing the moment the point moved off them.
    flatness = 1.0
    if fitted["error_major_m"] > 1e-9:
        flatness = min(1.0, fitted["error_minor_m"] / fitted["error_major_m"])
    return with_height({
        **point, "x_m": round(x_m, 3), "y_m": round(y_m, 3),
        "uncertainty_m": round(max(point["uncertainty_m"], spread), 3),
        "error_major_m": round(major_m, 3),
        "error_minor_m": round(major_m * flatness, 3),
        "error_major_deg": round(fitted["error_major_deg"], 1),
        "refined_from": len(usable),
        "spread_m": round(spread, 3)})


def cross_track_of(x_m: float, y_m: float, ray: dict[str, Any]) -> float:
    """How far a point lies to the side of where a ray was pointing, in metres."""
    range_m = math.hypot(x_m - float(ray["x_m"]), y_m - float(ray["y_m"]))
    off_deg = abs(_wrap(math.degrees(math.atan2(y_m - float(ray["y_m"]),
                                                x_m - float(ray["x_m"])))
                        - float(ray["bearing_deg"])))
    if off_deg >= 90.0:
        return range_m
    return range_m * math.tan(math.radians(off_deg))


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

    **And in metres up and down as well, which is where the elevation earns its
    keep.** Gating the founding pair on it is the obvious half and it turned out
    to be the smaller one: measured over the drive of 2026-09-03, an entity is
    not usually built wrong, it is *joined* wrong afterwards, and every look that
    joins one comes through here. A bearing at a picture on a wall points at the
    sideboard beneath it as convincingly as at the picture, and until this line
    existed nothing in the resolver could say otherwise -- appearance cannot
    separate two objects on this rover and a plan view cannot separate two
    heights.

    A placement with no height, or a ray that measured none, skips it: that is
    every entity and every look the rover recorded before the vertical half of
    the ray was kept, and the horizontal test is exactly what it always was.
    """
    range_m = math.hypot(float(point["x_m"]) - float(ray["x_m"]),
                         float(point["y_m"]) - float(ray["y_m"]))
    if range_m > MAX_RANGE_M:
        return False
    # A bearing cannot be pointing at something behind a wall, however well the
    # angle lines up. Same guard as in `fix`, and it has to be in both: one
    # decides whether a crossing places a thing, this decides whether a later
    # look joins one, and the run of 2026-09-02 had both kinds of error.
    if beyond_reach(ray, (float(point["x_m"]), float(point["y_m"]))):
        return False
    if not stands_as_high(point, ray):
        return False
    if tolerance_m is None:
        # What the bearing noise alone allows at this range, plus however
        # uncertain the point already was.
        #
        # **The whole radius here, and only the cross-track half in
        # `match_tolerance`, and the difference is deliberate.** The two look
        # like the same question and are not. There, one bearing is asked
        # whether it could be pointing at a thing already placed, and charging
        # it for error running down someone else's line of sight is what let an
        # entity claim a 46-degree cone. Here, several placements built from the
        # same rays are being *ranked* by how many of those rays agree, and the
        # ranking is what stops an entity moving out from under its own
        # evidence. Narrowing it collapses the counts to a tie -- measured on
        # `object:14` of 2026-09-02, both candidates fall to two agreeing rays
        # -- and a tie is broken on uncertainty, which is the wandering the
        # count was introduced to stop.
        tolerance_m = (range_m * math.tan(math.radians(sigma_of(ray)))
                       + float(point.get("uncertainty_m", 0.0))
                       + float(ray.get("origin_sigma_m") or NO_ORIGIN_ERROR_M))
    off_deg = abs(_wrap(bearing_to(point, ray) - float(ray["bearing_deg"])))
    if off_deg >= 90.0:
        return False
    return range_m * math.tan(math.radians(off_deg)) <= tolerance_m
