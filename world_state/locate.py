"""Triangulate and refine map positions from bearings, elevations and ranges.

One viewpoint cannot establish a position. A fix needs separated origins and
enough parallax; map visibility, height and measured range can reject it.
Measurement uncertainty is kept separate from an object's physical extent.
"""
from __future__ import annotations

import math
from typing import Any

# Stationary gimbal sweep, 2026-09-02: 1.5 degrees RMS, dominated by servo error.
# Driving SLAM error and tilt calibration remain unmeasured.
BEARING_SIGMA_DEG = 1.5
MIN_PARALLAX_DEG = 12.0
MIN_BASELINE_M = 0.4
# Older observations came from stationary captures; absent origin error is zero.
NO_ORIGIN_ERROR_M = 0.0
MAX_RANGE_M = 12.0
# Reject near-camera phantom intersections, even when their formal sigma is small.
MIN_RANGE_M = 0.75
# Lidar is below the camera: allow one metre beyond the first mapped obstacle.
SEE_PAST_M = 1.0
ELEVATION_SIGMA_DEG = BEARING_SIGMA_DEG
MAX_ELEVATION_DEG = 80.0
MAX_RISE_EXTENT_M = 1.0
# Unmeasured: report relative height only until the optical centre is surveyed.
CAMERA_HEIGHT_M = None
HUBER_K = 2.0
RANGE_SIGMA_M = 0.15

def sigma_of(ray: dict[str, object]) -> float:
    """Bearing sigma in degrees, never below the stationary calibration.

    Missing or invalid per-observation uncertainty uses BEARING_SIGMA_DEG."""
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
    """Cross two separated bearings and return a position with uncertainty.

    Reject poor baseline/parallax, out-of-range intersections, blocked sight
    lines and inconsistent elevations or measured ranges."""
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

    # **And both rays have to agree about how far away it is.** Everything above
    # is made of angles, and angles have one blind spot: a crossing where nothing
    # is lies exactly on the rays belonging to the real things either side of it.
    # Two bearings at two different chairs meet at a point that is on neither of
    # them, at a healthy parallax off a healthy baseline, and every guard above
    # accepts it -- while both rays say in millimetres that what they were
    # looking at was somewhere else. Rays that carry no range abstain rather than
    # refuse, which is every look taken through a camera the depth map does not
    # cover and every look this rover recorded before it had one.
    ranged = range_disagreement(placed, first, second)
    if ranged is not None and ranged[0] > ranged[1]:
        return None
    return placed


def noise_deg(x_m: float, y_m: float, ray: dict[str, Any]) -> float:
    """Combine bearing and origin error at this range, excluding object width."""
    range_m = max(_range_to(x_m, y_m, ray), MIN_RANGE_M)
    origin = math.degrees(math.atan2(
        float(ray.get("origin_sigma_m") or NO_ORIGIN_ERROR_M), range_m))
    return math.hypot(sigma_of(ray), origin)


def silhouette_deg(x_m: float, y_m: float, ray: dict[str, Any],
                    extent_m: float) -> float:
    """Half-width as an angle; residuals inside the silhouette are unpenalized."""
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
    """Whitened bearing and optional range residuals for one candidate position."""
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

    # How far the thing is, against how far the ray said. `world_state/oak.py`
    # is what puts one there; see `RANGE_SIGMA_M` for what it is worth when the
    # ray does not say.
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
    """Robust weighted least-squares fit using bearing and optional range residuals.

    Recompute range-dependent noise each iteration; use Huber weights for
    outliers and return None for a singular or unsupported fit."""
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
    """Cross-track measurement uncertainty in metres, excluding object extent."""
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
    """Placement uncertainty along a ray, projected from its covariance when known."""
    major = point.get("error_major_m")
    if major is None:
        return float(point.get("uncertainty_m", 0.0))
    minor = float(point.get("error_minor_m", major))
    between = (math.radians(float(point.get("error_major_deg", 0.0)))
               - math.atan2(float(point["y_m"]) - float(ray["y_m"]),
                            float(point["x_m"]) - float(ray["x_m"])))
    return math.hypot(float(major) * math.cos(between), minor * math.sin(between))


def rise_m(point: dict[str, Any], ray: dict[str, Any]) -> float | None:
    """Height relative to the camera at this horizontal range; None if unmeasured."""
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
    """Height allowance: measurement noise plus the visible vertical half-extent."""
    return rise_noise_m(point, ray) + rise_extent_m(point, ray)


def rise_noise_m(point: dict[str, Any], ray: dict[str, Any]) -> float:
    """Elevation, placement and origin uncertainty propagated to height."""
    range_m = math.hypot(float(point["x_m"]) - float(ray["x_m"]),
                         float(point["y_m"]) - float(ray["y_m"]))
    slope = abs(math.tan(math.radians(
        min(abs(float(ray.get("elevation_deg") or 0.0)), MAX_ELEVATION_DEG))))
    return (range_m * math.tan(math.radians(ELEVATION_SIGMA_DEG))
            + along_track(point, ray) * slope
            + float(ray.get("origin_sigma_m") or NO_ORIGIN_ERROR_M) * slope)


def rise_extent_m(point: dict[str, Any], ray: dict[str, Any]) -> float:
    """Visible vertical half-extent, capped; clipped boxes use the full allowance."""
    if ray.get("elevation_clipped"):
        return MAX_RISE_EXTENT_M
    range_m = math.hypot(float(point["x_m"]) - float(ray["x_m"]),
                         float(point["y_m"]) - float(ray["y_m"]))
    span_deg = float(ray.get("elevation_span_deg") or 0.0)
    own = range_m * math.tan(math.radians(min(span_deg, 90.0) / 2.0))
    return min(MAX_RISE_EXTENT_M, max(0.0, own))


def rise_disagreement(point: dict[str, Any], first: dict[str, Any],
                      second: dict[str, Any]) -> tuple[float, float] | None:
    """Height disagreement and total allowance; None without both elevations."""
    here, there = rise_m(point, first), rise_m(point, second)
    if here is None or there is None:
        return None
    return (abs(here - there),
            rise_tolerance_m(point, first) + rise_tolerance_m(point, second))


def height_over(point: dict[str, Any], rays: list[dict[str, Any]]
                ) -> tuple[float, float] | None:
    """Median relative height and best measurement sigma across supporting rays.

    Clipped boxes widen uncertainty; object extent does not average away."""
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
    """Accept a ray if its elevation agrees with the placement, or is unmeasured."""
    claimed = point.get("height_m")
    if claimed is None:
        return True
    got = rise_m(point, ray)
    if got is None:
        return True
    allowed = (rise_tolerance_m(point, ray)
               + float(point.get("height_sigma_m") or 0.0))
    return abs(got - float(claimed)) <= allowed


# --- how far away it said it was ---------------------------------------------
#
# **The one thing a bearing cannot say, and the rover has been measuring it for
# months with nothing reading it.** Everything above this line works from angles,
# and angles have one blind spot that no amount of care removes: a crossing where
# nothing is lies exactly on the rays belonging to the real things either side of
# it and fits them exactly as well. Three objects in a row seen from two places
# come back as one phantom, and the honest answer from two viewpoints is that it
# is not knowable.
#
# A range makes it knowable from one. The depth camera on the front of this rover
# has served millimetres on loopback 8770 since 2026-08-31, and `world_state/oak.py`
# is what puts one on a ray; what follows is the same three-part shape the
# elevation already has -- what a range is worth (`range_noise_m`), how much of a
# miss the thing's own size forgives (`range_extent_m`), whether two rays agree
# about it (`range_disagreement`, spent by `fix`), and whether a later look agrees
# with a thing already placed (`stands_at_range`, spent by `agrees`).
#
# It is a gate and a residual and deliberately **not** a third way of placing
# something. One ranged ray is a point in the room and it would be easy to let it
# found an entity, but everything this component knows about what identity costs
# was learnt from crossings, and a rule that lets one look place a thing is a
# different application from the one that was measured. `residuals` spends the
# range in the fit, where it pins the position along the sight line -- exactly
# where two bearings are weakest -- and that is the whole of the change.


def range_noise_m(point: dict[str, Any], ray: dict[str, Any]) -> float:
    """Measured range sigma plus placement and origin uncertainty."""
    sigma = ray.get("range_sigma_m")
    try:
        sigma = RANGE_SIGMA_M if sigma is None else max(0.0, float(sigma))
    except (TypeError, ValueError):
        sigma = RANGE_SIGMA_M
    return (sigma + along_track(point, ray)
            + float(ray.get("origin_sigma_m") or NO_ORIGIN_ERROR_M))


def range_extent_m(point: dict[str, Any], ray: dict[str, Any]) -> float:
    """Object half-extent along a range, capped by MAX_EXTENT_M."""
    own = point.get("extent_m")
    if own is None:
        span_deg = float(ray.get("span_deg") or 0.0)
        range_m = _range_to(float(point["x_m"]), float(point["y_m"]), ray)
        own = range_m * math.tan(math.radians(min(span_deg, 90.0) / 2.0))
    return min(MAX_EXTENT_M, max(0.0, float(own)))


def range_tolerance_m(point: dict[str, Any], ray: dict[str, Any]) -> float:
    """How far off a measured range may be and still be this thing, in metres."""
    return range_noise_m(point, ray) + range_extent_m(point, ray)


def range_disagreement(point: dict[str, Any], first: dict[str, Any],
                       second: dict[str, Any]) -> tuple[float, float] | None:
    """Worst range disagreement and tightest allowance; None when neither measured one.

    **A ray that measured nothing abstains; it does not silence the one that
    did.** Requiring both used to mean that the commonest pair on this rover --
    one look through the depth camera's picture and one outside it -- spent no
    range at all, and that is the pair the gate is most needed for. Measured on
    the store of 2026-09-05, only 11 of 251 placed things had a reading on every
    look and 172 had one on some, so the gate was abstaining almost everywhere it
    had something to say. `object:213` is what that costs: a crossing accepted
    1.06 m from a look that had measured 5.94 m to the thing, which is the whole
    length of a room in the one direction two bearings cannot see.

    This is now the pair-wise form of `stands_at_range`, which has always
    abstained per ray rather than per pair.
    """
    misses = []
    allowed = []
    for ray in (first, second):
        measured = ray.get("range_m")
        if measured is None:
            continue
        try:
            measured = float(measured)
        except (TypeError, ValueError):
            continue
        misses.append(abs(_range_to(float(point["x_m"]), float(point["y_m"]),
                                    ray) - measured))
        allowed.append(range_tolerance_m(point, ray))
    if not misses:
        return None
    return max(misses), min(allowed)


def stands_at_range(point: dict[str, Any], ray: dict[str, Any]) -> bool:
    """Accept absent ranges; otherwise require agreement within noise and extent."""
    measured = ray.get("range_m")
    if measured is None:
        return True
    try:
        measured = float(measured)
    except (TypeError, ValueError):
        return True
    if measured <= 0.0:
        return True
    got = _range_to(float(point["x_m"]), float(point["y_m"]), ray)
    return abs(got - measured) <= range_tolerance_m(point, ray)


def above_floor_m(height_m: float | None) -> float | None:
    """Convert relative height only when the camera mounting height is measured."""
    if height_m is None or CAMERA_HEIGHT_M is None:
        return None
    return float(height_m) + float(CAMERA_HEIGHT_M)


def height_fields(point: dict[str, Any], rays: list[dict[str, Any]]) -> dict:
    """Serialisable height fields, with absolute height only when calibrated."""
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
    """Smallest observed half-width in metres, capped by MAX_EXTENT_M."""
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
    """Association allowance in metres, combining measurement error and object width."""
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
    """Choose the best-supported pair crossing from independent viewpoints."""
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
    """Count sufficiently separated camera origins supporting this placement."""
    places: list[tuple[float, float]] = []
    for ray in rays:
        x_m, y_m = float(ray["x_m"]), float(ray["y_m"])
        if not any(math.hypot(x_m - px, y_m - py) < MIN_BASELINE_M
                   for px, py in places):
            places.append((x_m, y_m))
    return len(places)


def refine(point: dict[str, Any], rays: list[dict[str, Any]]) -> dict[str, Any]:
    """Refit agreeing bearings while preserving the pair crossing as a fallback.

    Bound displacement by REFINE_LIMIT_M and retain the measured object extent."""
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
    """Require forward visibility and agreement in bearing, height and measured range."""
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
    # And not at a distance this ray measured for itself and disagrees with. The
    # same removal-only gate as the height above it, spent in the one direction
    # a bearing has nothing to say about.
    if not stands_at_range(point, ray):
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
