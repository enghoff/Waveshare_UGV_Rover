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
    return {
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

    The arithmetic is the least-squares point of closest approach to a set of
    lines: each ray contributes the part of the error square that is *across* it,
    which for a unit direction `d` is `I - d dT`, and the sum is a two-by-two
    solve. Two rays give back their own crossing exactly, so an entity with one
    baseline behind it is not moved at all.

    **The uncertainty is not narrowed for the extra rays, deliberately.** Shrinking
    it by the root of how many there are would assume their errors are
    independent, and on this rover they are mostly not: a bearing is dominated by
    the gimbal not arriving where it was told and by the heading SLAM reports,
    which are one mistake per look rather than one per ray. What is recorded
    instead is the measured spread -- how far the agreeing rays actually fall from
    the fitted point -- and the pair's own figure is kept as a floor, so the number
    can grow when the evidence disagrees and never shrinks on a promise.
    """
    usable = [ray for ray in rays if agrees(point, ray)]
    if len(usable) < 3:
        return point
    axx = axy = ayy = bx = by = 0.0
    for ray in usable:
        dx, dy = _unit(float(ray["bearing_deg"]))
        # I - d dT, the part of a displacement that is across this ray.
        wxx, wxy, wyy = 1.0 - dx * dx, -dx * dy, 1.0 - dy * dy
        px, py = float(ray["x_m"]), float(ray["y_m"])
        axx += wxx; axy += wxy; ayy += wyy
        bx += wxx * px + wxy * py
        by += wxy * px + wyy * py
    determinant = axx * ayy - axy * axy
    if abs(determinant) < 1e-9:
        return point
    x_m = (ayy * bx - axy * by) / determinant
    y_m = (axx * by - axy * bx) / determinant
    moved = math.hypot(x_m - float(point["x_m"]), y_m - float(point["y_m"]))
    if moved > float(point.get("uncertainty_m", 0.0)) + REFINE_LIMIT_M:
        # Further than the pair's own doubt plus a handspan is not a refinement,
        # it is a different answer, and this is not the function that chooses
        # between answers. Leave the pair's.
        return point
    spread = math.sqrt(sum(
        cross_track_of(x_m, y_m, ray) ** 2 for ray in usable) / len(usable))
    return {**point, "x_m": round(x_m, 3), "y_m": round(y_m, 3),
            "uncertainty_m": round(max(point["uncertainty_m"], spread), 3),
            "error_major_m": round(max(point.get("error_major_m",
                                                 point["uncertainty_m"]),
                                       spread), 3),
            "refined_from": len(usable),
            "spread_m": round(spread, 3)}


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
