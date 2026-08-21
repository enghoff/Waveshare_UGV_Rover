#!/usr/bin/env python3
"""A route through the occupancy grid, as a few waypoints rather than a cell path.

The map is the only picture of the room that reaches more than a couple of metres,
so going to a place the rover is not already facing means planning on it. The pose
drifts and the grid holds geometry that has since moved, so this is a sketch, not a
promise: the navigator follows the sketch with the live scan in the loop, and asks
for another one when the room disagrees.

What comes out is a short polyline. A* runs at cell resolution so it can thread a
gap; the caller does not want a hundred 5 cm hops, so the path is then thinned to
the corners that actually change heading, and then pulled straight -- most of the
corners a grid search reports are the grid talking, not the room. Following that is
a handful of turns and straight segments, which is what the rover can do.
"""
from __future__ import annotations

import heapq
import math

# Unknown (log-odds 0) is treated as blocked: a picture of never-seen is not a
# picture of empty, and driving into grey is how a rover finds a stair. Occupied
# is blocked too. Only cells the lidar has seen to be empty are a route.
#
# The inflate radius is the caller's business. It is a sideways gap, not the
# along-track brake: STANDOFF_M + REACT_MARGIN_M (0.45 m) asked for a 90 cm
# opening and sealed pinches the chassis still fits. Navigator uses 0.25 m as
# the floor, and tries 0.45 m first -- see `preferred_m`.
#
# A single hard ring is hugged, and a hugged corner is passed at exactly the
# distance the follower brakes at, so an ordinary pose error turns a legal
# route into a stop. A soft toll (`comfort_m`) was not enough: two extra cells
# of path is a cheap price to scrape a corner when going around costs metres.
# So `preferred_m` is a second keep-out, tried first. Only if it has no route
# at all does planning fall back to `inflate_m`. A corner gets the room the
# follower actually needs whenever the room has it, and a pinch is still taken
# when it does not.
#
# `start_yaw` matters because the rover may always turn, even with its nose in
# a wall. A* has no heading, so a start cell that is free will still step into
# the keep-out ahead and the follower will drive that chord. If the heading
# looks into the keep-out, the route starts from a free cell off that heading
# and keeps the hop as a waypoint, so the first thing the rover does is turn.

# Most of the corners in a grid route are the grid, not the room. A* moves in
# eight directions and measures in octile steps, so every monotone staircase
# between two cells costs exactly the same and which one comes back is decided
# by the order the heap happened to pop. On empty floor that produced a 4 m run
# straight ahead and then a 25 degree kink, and thinning cannot undo it: the
# kink is a real 40 cm departure from the straight line, and keeping departures
# that large is what thinning is for.
#
# So after thinning the route is pulled straight -- runs of corners are replaced
# by the line between their ends wherever that line is clear of the same
# keep-out A* used and is not a worse trade under the same proximity toll. Both
# halves of that test matter. Clearance is what makes it safe: the line is
# walked against the inflated mask, so a straightened route keeps every metre of
# room the cornered one had. The toll is what keeps it honest: a chord that
# scrapes a corner to save a few centimetres costs more toll than it saves in
# length and is refused, so the middle of a gap stays preferred.
#
# A corner is not free to the rover, though, so the comparison credits the ones
# a shortcut removes. Anything past the follower's turn-in-place threshold is a
# full stop and a dead-reckoned spin -- around 1.8 s for a right-angle at the
# fine PWM, once its settle is counted, which is 0.4 m of driving at the speed
# a route is followed at -- and even a shallow corner drops the rover to crawl
# speed until the nose comes round. KINK_CREDIT_M is that in metres of path, so
# a straighter route may be a little longer or run a little nearer the toll and
# still win. Only on the roomy attempt: on the fallback through a pinch the
# keep-out is already inside the distance the follower brakes at, so the toll is
# the last thing holding the route off the wall and nothing is credited against
# it.

# What a step next to the keep-out costs, on top of its length: 1 + this, fading
# linearly to 1 at comfort_m. This is a nudge toward the middle of a gap, not
# the thing that keeps corners at arm's length -- `preferred_m` is.
COMFORT_COST = 2.0
# Thin the cell path, but not enough to eat the keep-out. 0.22 m used to cut a
# 0.45 m ring down to the distance the follower brakes at.
SIMPLIFY_M = 0.12
# Off-axis enough that the follower's turn-in-place threshold (40 deg) sees it.
TURN_ESCAPE_DEG = 55.0
# How finely a candidate straight line is walked when asking whether it is clear,
# in cells. Half a cell cannot step over the keep-out: that mask is a dilation by
# `inflate_m / resolution` cells in every direction, so anything blocked is blocked
# in a band several cells thick, and a sample cannot land either side of it.
LOS_STEP_CELLS = 0.5
# What removing one corner is worth, as metres of path a shortcut may spend to
# do it -- see the straightening note above. Capped in total, because a run of
# ten corners is not a licence to take any line at all.
KINK_CREDIT_M = 0.30
KINK_CREDIT_MAX_M = 1.00


def plan(grid, resolution_m, occupied_at, start_xy, goal_xy, inflate_m,
         comfort_m=0.0, origin_cells=None, preferred_m=None, start_yaw=None):
    """World metres -> a polyline of world metres, or (None, why_not).

    `grid` is indexed [forward, left] the way slam2d's occupancy is, with the origin
    cell at `origin_cells` (defaults to the centre). `start_xy` and `goal_xy` are
    metres in that same frame. `comfort_m` is the distance from anything blocked
    at which travel stops costing extra; at or below `inflate_m` it does nothing.
    `preferred_m`, if larger than `inflate_m`, is tried as the keep-out first and
    `inflate_m` is used only when that has no route. `start_yaw` is radians in
    this frame (x forward, y left): if the heading looks into the keep-out, the
    route begins with a hop off that heading.
    """
    credit = KINK_CREDIT_M
    if preferred_m is not None and preferred_m > inflate_m:
        path, _why = _plan_once(grid, resolution_m, occupied_at, start_xy, goal_xy,
                                preferred_m, comfort_m, origin_cells, start_yaw,
                                kink_credit_m=credit)
        if path is not None:
            return path, None
        # Only a pinch is left. This keep-out is inside the distance the follower
        # brakes at, so the toll is the last thing holding the route off the wall
        # and straightening pays full price for it -- see the note above.
        credit = 0.0
    return _plan_once(grid, resolution_m, occupied_at, start_xy, goal_xy,
                      inflate_m, comfort_m, origin_cells, start_yaw,
                      kink_credit_m=credit)


def _plan_once(grid, resolution_m, occupied_at, start_xy, goal_xy, inflate_m,
               comfort_m, origin_cells, start_yaw, kink_credit_m=0.0):
    import numpy as np

    grid = np.asarray(grid)
    cells = grid.shape[0]
    if grid.shape != (cells, cells):
        return None, "the occupancy grid is not square"
    ox = oy = (cells // 2 if origin_cells is None else origin_cells)

    def to_cell(x, y):
        return int(round(x / resolution_m)) + ox, int(round(y / resolution_m)) + oy

    def to_world(ix, iy):
        return (ix - ox) * resolution_m, (iy - oy) * resolution_m

    sx, sy = to_cell(*start_xy)
    gx, gy = to_cell(*goal_xy)
    if not (0 <= gx < cells and 0 <= gy < cells):
        return None, "that place is off the map"
    if not (0 <= sx < cells and 0 <= sy < cells):
        return None, "the rover is off the map"

    goal_val = int(grid[gx, gy])
    if goal_val >= occupied_at:
        return None, "that place is solid"
    if goal_val == 0:
        return None, "that place has not been seen yet"

    # Crop to the two points plus a margin, so A* is a small search rather than a
    # walk of the whole 20 m grid. The margin is enough to go around a table.
    margin = max(12, int(math.ceil(2.5 / resolution_m)))
    x0 = max(0, min(sx, gx) - margin)
    x1 = min(cells, max(sx, gx) + margin + 1)
    y0 = max(0, min(sy, gy) - margin)
    y1 = min(cells, max(sy, gy) + margin + 1)
    crop = grid[x0:x1, y0:y1]
    blocked = (crop >= occupied_at) | (crop == 0)
    radius = max(0, int(math.ceil(inflate_m / resolution_m)))
    inflated = _inflate(blocked, radius)

    # The rover is already where it is, even if that cell is inside the inflate
    # radius of a wall. Plan from the nearest cell that is actually free: merely
    # unblocking the start cell looked like the same fix and was not, because the
    # rest of the inflation still walled that one cell in and every route out was
    # refused -- exactly the tight spot a planner most needs to leave. The true
    # pose is prepended below, and that hop is kept as a waypoint, so a rover
    # facing the wall turns onto the free cell instead of driving the chord.
    true_sx, true_sy = sx - x0, sy - y0
    lsx, lsy = true_sx, true_sy
    lgx, lgy = gx - x0, gy - y0
    search = max(radius + 2, 8)
    if inflated[lsx, lsy]:
        freed = _nearest_free(inflated, lsx, lsy, search)
        if freed is None:
            return None, "the map shows no room to move at all from here"
        lsx, lsy = freed
    elif start_yaw is not None and _heading_hits_keepout(
            inflated, lsx, lsy, start_yaw, resolution_m, inflate_m):
        escaped = _nearest_free_not_ahead(
            inflated, lsx, lsy, start_yaw, search)
        if escaped is not None:
            lsx, lsy = escaped

    true_gx, true_gy = gx - x0, gy - y0
    if inflated[lgx, lgy]:
        snapped = _nearest_free(inflated, lgx, lgy, search)
        if snapped is None:
            return None, "there is no room to stand at that place"
        lgx, lgy = snapped
    goal_snapped = (lgx, lgy) != (true_gx, true_gy)

    comfort_cells = int(math.ceil(max(0.0, comfort_m - inflate_m) / resolution_m))
    penalty = _proximity_penalty(inflated, comfort_cells)
    local = _astar(inflated, (lsx, lsy), (lgx, lgy), penalty)
    if local is None:
        return None, "no clear route through what the lidar has seen"

    world = [to_world(ix + x0, iy + y0) for ix, iy in local]
    # The cell centres of start and goal are not the poses that were asked for.
    # Keep the real endpoints so the follower is aiming at the tap, not at a
    # 5 cm rounding of it. If A* started somewhere else -- a snap out of the
    # keep-out, or a hop off a blocked heading -- that cell stays as the second
    # waypoint; thinning it away is how a turn-first route became a chord
    # through the wall.
    pin_hop = (lsx, lsy) != (true_sx, true_sy)
    if pin_hop:
        world.insert(0, tuple(start_xy))
    else:
        world[0] = tuple(start_xy)
    world[-1] = tuple(goal_xy)
    thinned = _simplify_pinned(world, SIMPLIFY_M, pin_hop)

    def to_local(x, y):
        """World metres -> fractional cells in the cropped frame."""
        return x / resolution_m + ox - x0, y / resolution_m + oy - y0

    # A goal inside the keep-out was snapped for the search and then put back, so
    # the last leg already ends by driving into that ring. Straightening is
    # allowed the same exemption and no more: `inflate_m` around the goal, which
    # is shorter than the leg it replaces.
    exempt = (inflate_m / resolution_m) if goal_snapped else 0.0
    return _straighten(thinned, inflated, penalty, to_local,
                       first=1 if pin_hop else 0, exempt_cells=exempt,
                       credit_cells=kink_credit_m / resolution_m,
                       credit_cap_cells=KINK_CREDIT_MAX_M / resolution_m), None


def length(points):
    """Metres along a polyline."""
    total = 0.0
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        total += math.hypot(bx - ax, by - ay)
    return total


def point_at(points, s):
    """The point `s` metres along the polyline, clamped to the ends."""
    if not points:
        return 0.0, 0.0
    if s <= 0.0:
        return points[0]
    walked = 0.0
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        step = math.hypot(bx - ax, by - ay)
        if walked + step >= s or step < 1e-9:
            t = 0.0 if step < 1e-9 else (s - walked) / step
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            return ax + (bx - ax) * t, ay + (by - ay) * t
        walked += step
    return points[-1]


def project(points, xy, s_min, slack_m):
    """Closest point on the polyline, as (s, cross_track_m).

    `s` is not allowed to run backwards by more than `slack_m` from `s_min`, so a
    rover that weaves a little beside the path does not have its progress wound
    back to a corner it has already passed. A rover that has actually gone back
    by more than the slack is off the route, and the caller should replan rather
    than pretend the closest point is still ahead.
    """
    if len(points) < 2:
        px, py = points[0] if points else xy
        return 0.0, math.hypot(xy[0] - px, xy[1] - py)
    floor = s_min - slack_m
    best_s, best_d = s_min, 1e9
    found = False
    walked = 0.0
    x, y = xy
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        vx, vy = bx - ax, by - ay
        step = math.hypot(vx, vy)
        if step < 1e-9:
            continue
        t = ((x - ax) * vx + (y - ay) * vy) / (step * step)
        t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
        s = walked + t * step
        if s >= floor:
            px, py = ax + vx * t, ay + vy * t
            d = math.hypot(x - px, y - py)
            if not found or d < best_d:
                best_s, best_d = s, d
                found = True
        walked += step
    if not found:
        return s_min, math.hypot(x - points[-1][0], y - points[-1][1])
    return best_s, best_d


def segment_end_s(points, s):
    """Metres along the polyline at the end of the segment that contains `s`.

    A carrot that looks past this vertex cuts the corner: the follower aims at a
    point on the next leg and drives the chord, which is how a route that gave a
    corner room still arrives at the brake distance. Looking only to the vertex
    makes a sharp corner the turn-in-place it already is. A progress value that
    sits exactly on a vertex belongs to the outgoing segment, so the carrot
    after arriving is the next heading, not the one just finished.
    """
    if len(points) < 2:
        return 0.0
    walked = 0.0
    s = max(0.0, s)
    eps = 1e-6
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        step = math.hypot(bx - ax, by - ay)
        if step < 1e-9:
            continue
        if s < walked + step - eps:
            return walked + step
        walked += step
    return walked


def carrot_at(points, s, lookahead_m, min_m, max_turn_deg):
    """The point to steer at from `s` metres along the route.

    `lookahead_m` ahead normally, and never past the vertex at the end of the
    segment being driven -- see `segment_end_s` for why looking onto the next leg
    drives the chord.

    But a vertex that is 3 cm away cannot be steered at. The bearing to a point
    that close is all cross-track error and pose wobble: 5 cm to the side of a
    carrot 5 cm ahead is 45 degrees of heading error out of nothing, which is past
    the follower's turn-in-place threshold, so the rover stops and spins a hand's
    breadth short of a corner it was tracking perfectly. Two thirds of the heading
    a simulated route threw away went on exactly that.

    So at a *shallow* vertex -- one the follower would drive through rather than
    stop and spin at, which is what `max_turn_deg` names -- the aim point runs on
    past it once it comes inside `min_m`, along this segment's own line and not
    the next one. That distinction is the whole safety of it: extending the line
    the rover is already driving cannot bend it towards the inside of the corner.

    A sharp vertex keeps the collapsing carrot it always had. There the rover is
    going to stop and turn whatever happens, so an early trigger costs a few
    centimetres of approach and nothing else -- while running on past a right
    angle would have it arrive at the corner still under power, and a corner is
    exactly where the route has the least room to spare.

    The last waypoint is left alone for a different reason: there the collapsing
    bearing is doing real work, swinging the rover round to a goal it would
    otherwise sail past a little to one side of.
    """
    if len(points) < 2:
        return points[0] if points else (0.0, 0.0)
    s = max(0.0, s)
    end_s = segment_end_s(points, s)
    if end_s - s >= min_m or end_s >= length(points) - 1e-6:
        return point_at(points, min(s + lookahead_m, end_s))
    seg = _segment_at(points, s)
    if seg is None or abs(_turn_after(points, end_s)) >= max_turn_deg:
        return point_at(points, end_s)
    (ax, ay), (bx, by), step = seg
    extra = min_m - (end_s - s)
    return bx + (bx - ax) / step * extra, by + (by - ay) / step * extra


def _turn_after(points, end_s):
    """Degrees the route turns at the vertex `end_s` metres along, 0 at the last.

    `walked` is the distance at the *start* of each segment, so the segment it
    matches is the one leaving the vertex and `heading` is still the one arriving.
    """
    heading = None
    walked = 0.0
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        step = math.hypot(bx - ax, by - ay)
        if step < 1e-9:
            continue
        leg = math.atan2(by - ay, bx - ax)
        if heading is not None and abs(walked - end_s) <= 1e-6:
            return math.degrees((leg - heading + math.pi) % (2 * math.pi) - math.pi)
        heading = leg
        walked += step
    return 0.0


def _segment_at(points, s):
    """The segment being driven at `s`: its ends and its length.

    Same boundary rule as `segment_end_s`: progress sitting exactly on a vertex
    belongs to the segment leaving it.
    """
    walked = 0.0
    eps = 1e-6
    last = None
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        step = math.hypot(bx - ax, by - ay)
        if step < 1e-9:
            continue
        last = ((ax, ay), (bx, by), step)
        if s < walked + step - eps:
            return last
        walked += step
    return last


def cells_occupied_along(grid, resolution_m, occupied_at, points, s_from,
                         origin_cells=None):
    """True if a remaining stretch of the polyline now sits on a solid cell.

    Unknown is not a reason to throw the route away: the planner already refused
    to go through it, and a cell flipping back to 0 is the matcher losing a hit,
    not a wall appearing. Solid is.
    """
    import numpy as np

    grid = np.asarray(grid)
    cells = grid.shape[0]
    ox = oy = (cells // 2 if origin_cells is None else origin_cells)
    s = max(0.0, s_from)
    end = length(points)
    while s <= end + 1e-6:
        x, y = point_at(points, s)
        ix = int(round(x / resolution_m)) + ox
        iy = int(round(y / resolution_m)) + oy
        if 0 <= ix < cells and 0 <= iy < cells and int(grid[ix, iy]) >= occupied_at:
            return True
        s += resolution_m
    return False


def _inflate(blocked, radius_cells):
    """Euclidean dilation of a boolean grid, in-place-safe (returns a copy).

    One whole-array OR per offset in the disc, not one neighbourhood write per
    blocked cell. The distinction decides whether this runs on the Pi: unknown
    counts as blocked here, so a typical crop has *thousands* of blocked cells,
    against ~113 offsets in a radius-6 disc -- and each OR is a single numpy pass
    where the per-cell version paid several interpreted numpy calls per cell.
    """
    if radius_cells <= 0:
        return blocked.copy()
    out = blocked.copy()
    h, w = blocked.shape
    r, r2 = radius_cells, radius_cells * radius_cells
    for du in range(-r, r + 1):
        for dv in range(-r, r + 1):
            if (du or dv) and du * du + dv * dv <= r2:
                out[max(0, du):h + min(0, du), max(0, dv):w + min(0, dv)] |= \
                    blocked[max(0, -du):h + min(0, -du),
                            max(0, -dv):w + min(0, -dv)]
    return out


def _dilate1(mask):
    """One cell of 8-connected dilation -- the same whole-array trick as _inflate,
    fixed at radius 1 so repeated calls walk outward one chessboard ring at a time."""
    out = mask.copy()
    h, w = mask.shape
    for du in (-1, 0, 1):
        for dv in (-1, 0, 1):
            if du or dv:
                out[max(0, du):h + min(0, du), max(0, dv):w + min(0, dv)] |= \
                    mask[max(0, -du):h + min(0, -du),
                         max(0, -dv):w + min(0, -dv)]
    return out


def _proximity_penalty(inflated, radius_cells, peak=COMFORT_COST):
    """Extra cost per step for travelling near the keep-out, or None for none.

    `peak` right against the keep-out, fading linearly to zero `radius_cells`
    out. Built as successive one-cell dilations of the keep-out mask -- each new
    ring is one distance band -- so the whole thing is a few dozen whole-array
    ORs rather than a distance transform this host cannot afford.
    """
    import numpy as np

    if radius_cells <= 0 or peak <= 0.0:
        return None
    penalty = np.zeros(inflated.shape, dtype=np.float32)
    ring = inflated
    for k in range(radius_cells):
        grown = _dilate1(ring)
        band = grown & ~ring
        penalty[band] = peak * (radius_cells - k) / radius_cells
        ring = grown
    return penalty


def _heading_hits_keepout(inflated, sx, sy, yaw, resolution_m, look_m):
    """True if the heading from this cell runs into the keep-out within `look_m`."""
    if look_m <= 0.0:
        return False
    h, w = inflated.shape
    steps = max(1, int(math.ceil(look_m / resolution_m)))
    for i in range(1, steps + 1):
        dist = i * resolution_m
        ix = sx + int(round(math.cos(yaw) * dist / resolution_m))
        iy = sy + int(round(math.sin(yaw) * dist / resolution_m))
        if not (0 <= ix < h and 0 <= iy < w) or inflated[ix, iy]:
            return True
    return False


def _nearest_free(inflated, gx, gy, limit):
    h, w = inflated.shape
    best, best_d = None, None
    for dx in range(-limit, limit + 1):
        for dy in range(-limit, limit + 1):
            ix, iy = gx + dx, gy + dy
            if not (0 <= ix < h and 0 <= iy < w) or inflated[ix, iy]:
                continue
            d = dx * dx + dy * dy
            if best is None or d < best_d:
                best, best_d = (ix, iy), d
    return best


def _nearest_free_not_ahead(inflated, gx, gy, yaw, limit):
    """Nearest free cell whose bearing is off the heading by TURN_ESCAPE_DEG.

    The rover may always turn, so a blocked nose is a reason to start the route
    to the side, not to refuse it. Cells still inside the forward cone are
    ignored: those are the hop into the wall this is here to avoid.
    """
    h, w = inflated.shape
    best, best_d = None, None
    min_rad = math.radians(TURN_ESCAPE_DEG)
    for dx in range(-limit, limit + 1):
        for dy in range(-limit, limit + 1):
            if dx == 0 and dy == 0:
                continue
            ix, iy = gx + dx, gy + dy
            if not (0 <= ix < h and 0 <= iy < w) or inflated[ix, iy]:
                continue
            bearing = math.atan2(dy, dx) - yaw
            bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
            if abs(bearing) < min_rad:
                continue
            d = dx * dx + dy * dy
            if best is None or d < best_d:
                best, best_d = (ix, iy), d
    return best


def _astar(blocked, start, goal, penalty=None):
    """8-connected A* on a boolean blocked grid. Returns a list of (ix, iy).

    Flat Python lists rather than numpy arrays, because the cost of A* is almost
    entirely element access and a numpy scalar read is several times the price of
    a list index -- numpy earns its keep on whole-array passes like _inflate, and
    this is the opposite of one. Measured on the Pi it is the difference between
    a route in well under a second and one the caller times out waiting for.

    `penalty` scales each step by 1 + the destination cell's value, so nearness
    to the keep-out is a toll rather than a wall. The heuristic stays the plain
    distance, which every real cost is at least, so it stays admissible and the
    route stays optimal under the tolled costs.
    """
    h, w = blocked.shape
    sx, sy = start
    gx, gy = goal
    solid = blocked.ravel().tolist()
    toll = None if penalty is None else penalty.ravel().tolist()
    start_i, goal_i = sx * w + sy, gx * w + gy
    if solid[goal_i] or solid[start_i]:
        return None
    if start == goal:
        return [start]

    rt2 = math.sqrt(2)
    nbrs = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, rt2), (1, -1, rt2), (-1, 1, rt2), (-1, -1, rt2))

    inf = 1e18
    g = [inf] * (h * w)
    came = [-1] * (h * w)
    closed = bytearray(h * w)
    g[start_i] = 0.0
    heap = [(math.hypot(gx - sx, gy - sy), 0.0, sx, sy)]

    while heap:
        _f, cost, x, y = heapq.heappop(heap)
        i = x * w + y
        if closed[i]:
            continue
        closed[i] = 1
        if i == goal_i:
            return _reconstruct(came, w, start_i, goal_i)
        for dx, dy, step in nbrs:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < h and 0 <= ny < w):
                continue
            j = nx * w + ny
            if solid[j] or closed[j]:
                continue
            # No cutting a corner through a blocked diagonal neighbour.
            if dx != 0 and dy != 0 and (solid[i + dx * w] or solid[i + dy]):
                continue
            nxt = cost + (step if toll is None else step * (1.0 + toll[j]))
            if nxt + 1e-9 < g[j]:
                g[j] = nxt
                came[j] = i
                heapq.heappush(heap, (nxt + math.hypot(gx - nx, gy - ny),
                                      nxt, nx, ny))
    return None


def _reconstruct(came, w, start_i, goal_i):
    path = [goal_i]
    i = goal_i
    for _ in range(len(came) + 1):
        if i == start_i:
            path.reverse()
            return [(k // w, k % w) for k in path]
        i = came[i]
        if i < 0:
            return None
        path.append(i)
    return None


def _straighten(points, inflated, penalty, to_local, first=0, exempt_cells=0.0,
                credit_cells=0.0, credit_cap_cells=float("inf")):
    """Replace runs of corners with the line between their ends, where it is as good.

    Two sweeps over what is by now a handful of waypoints, not the cell path: a
    greedy one that jumps as far along the route as a clear line will reach, and
    then a removal sweep that drops any remaining corner whose neighbours can see
    each other. The greedy pass collapses a staircase in one go; the sweep catches
    the corner the greedy jump landed on and then did not need.

    A run is only replaced when the straight line is clear of `inflated` -- the
    same keep-out A* was given -- and costs no more under `penalty` than the run
    it replaces, less `credit_cells` for each corner it removes (capped at
    `credit_cap_cells`, and zero on a route squeezing through a pinch). `first`
    is the index before which nothing may be shortcut, which is how the
    turn-off-a-wall hop survives. `exempt_cells` is a radius around the final
    waypoint inside which blockage is ignored, for a goal that stands in the
    keep-out.

    Flat lists rather than numpy arrays for the same reason `_astar` uses them: this
    is thousands of single-element reads, which is where a numpy scalar read costs
    several times a list index.
    """
    if len(points) < 3:
        return list(points)
    h, w = inflated.shape
    solid = inflated.ravel().tolist()
    toll = None if penalty is None else penalty.ravel().tolist()
    local = [to_local(x, y) for x, y in points]
    last = len(points) - 1

    def walk(a, b):
        """Sample a straight line at half-cell steps: (t, flat index, step) each."""
        au, av = a
        bu, bv = b
        span = math.hypot(bu - au, bv - av)
        steps = max(1, int(math.ceil(span / LOS_STEP_CELLS)))
        for k in range(steps + 1):
            t = k / steps
            iu = int(round(au + (bu - au) * t))
            iv = int(round(av + (bv - av) * t))
            inside = 0 <= iu < h and 0 <= iv < w
            yield t, (iu * w + iv if inside else -1), span / steps

    def clear(a, b, exempt):
        for t, i, _step in walk(a, b):
            if exempt > 0.0 and (1.0 - t) * math.hypot(b[0] - a[0],
                                                       b[1] - a[1]) <= exempt:
                continue
            if i < 0 or solid[i]:
                return False
        return True

    def tolled(a, b):
        """Length of the straight line in cells, with the proximity toll on it."""
        total = 0.0
        first_sample = True
        for _t, i, step in walk(a, b):
            if first_sample:          # the sample at t=0 is the previous segment's end
                first_sample = False
                continue
            total += step * (1.0 + (0.0 if toll is None or i < 0 else toll[i]))
        return total

    # Tolled length along the route as it stands, cumulative, so comparing a
    # shortcut against the run it would replace is a subtraction.
    cum = [0.0]
    for i in range(last):
        cum.append(cum[-1] + tolled(local[i], local[i + 1]))

    def reach(i, j):
        if not clear(local[i], local[j], exempt_cells if j == last else 0.0):
            return False
        credit = min(credit_cells * (j - i - 1), credit_cap_cells)
        return tolled(local[i], local[j]) <= cum[j] - cum[i] + credit + 1e-9

    kept = list(range(first + 1))
    i = first
    while i < last:
        j = last
        while j > i + 1 and not reach(i, j):
            j -= 1
        kept.append(j)
        i = j

    changed = True
    while changed and len(kept) > max(2, first + 2):
        changed = False
        k = first + 1
        while k < len(kept) - 1:
            if reach(kept[k - 1], kept[k + 1]):
                del kept[k]
                changed = True
            else:
                k += 1
    return [points[i] for i in kept]


def _simplify_pinned(points, epsilon_m, pin_first_hop):
    """Thin the route, keeping the first hop when that hop is the turn off a wall."""
    if pin_first_hop and len(points) >= 3:
        return [points[0]] + _simplify(points[1:], epsilon_m)
    return _simplify(points, epsilon_m)


def _simplify(points, epsilon_m):
    """Ramer–Douglas–Peucker, so a 3 m detour around a table is three corners
    rather than sixty cells."""
    if len(points) <= 2:
        return list(points)

    def farthest(seq):
        ax, ay = seq[0]
        bx, by = seq[-1]
        vx, vy = bx - ax, by - ay
        span = math.hypot(vx, vy)
        best_i, best_d = 0, -1.0
        for i, (x, y) in enumerate(seq[1:-1], start=1):
            if span < 1e-9:
                d = math.hypot(x - ax, y - ay)
            else:
                t = ((x - ax) * vx + (y - ay) * vy) / (span * span)
                t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
                d = math.hypot(x - (ax + vx * t), y - (ay + vy * t))
            if d > best_d:
                best_i, best_d = i, d
        return best_i, best_d

    index, dist = farthest(points)
    if dist > epsilon_m:
        left = _simplify(points[:index + 1], epsilon_m)
        right = _simplify(points[index:], epsilon_m)
        return left[:-1] + right
    return [points[0], points[-1]]


def _selftest():
    import numpy as np

    res, n, occ = 0.05, 80, 20
    grid = np.full((n, n), -10, dtype=np.int8)  # known free
    origin = n // 2

    def at(x, y):
        return origin + int(round(x / res)), origin + int(round(y / res))

    # A wall with a gap to the left, so the straight line is blocked and the
    # route has to go around.
    wx, _ = at(1.0, 0.0)
    grid[wx, :] = occ
    gap_y0 = origin + int(round(0.6 / res))
    gap_y1 = origin + int(round(1.1 / res))
    grid[wx, gap_y0:gap_y1 + 1] = -10

    path, why = plan(grid, res, occ, (0.0, 0.0), (1.8, 0.0), inflate_m=0.20)
    assert path is not None, why
    assert length(path) > 1.8, f"took the blocked straight line, length {length(path)}"
    assert any(y > 0.4 for _, y in path), f"did not detour left through the gap: {path}"
    # Thinned: a cell path would be ~40 points; a route is a handful of corners.
    assert len(path) <= 12, f"not thinned, {len(path)} waypoints: {path}"

    # Occupied target is refused rather than planned onto.
    path, why = plan(grid, res, occ, (0.0, 0.0), (1.0, 0.0), inflate_m=0.20)
    assert path is None and "solid" in why, why

    # A rover that has ended up inside the wall's inflation ring can still plan
    # its way out -- this is the wedged case, and refusing it strands the rover.
    path, why = plan(grid, res, occ, (0.85, 0.0), (-0.5, 0.0), inflate_m=0.20)
    assert path is not None, f"wedged start was refused: {why}"
    assert path[0] == (0.85, 0.0), f"route does not start at the rover: {path[0]}"

    # Unseen target is refused.
    grid[:, :] = 0
    path, why = plan(grid, res, occ, (0.0, 0.0), (1.0, 0.0), inflate_m=0.20)
    assert path is None and "seen" in why, why

    # Corners are given room when there is room to give. A wall ends at
    # (1.0, 0.25); the route from (0,0) to (2,0) has to round that end. The
    # preferred keep-out is a wall, not a toll: if a 0.45 m ring fits, the
    # route must stay that far out. A soft comfort still helps a little on a
    # single ring, but it is not what keeps the follower off the brake.
    def corner_distance(pts, cx, cy):
        best, s = 1e9, 0.0
        while s <= length(pts) + 1e-6:
            px, py = point_at(pts, s)
            best = min(best, math.hypot(px - cx, py - cy))
            s += 0.02
        return best

    big = np.full((120, 120), -10, dtype=np.int8)   # 6 m of known-free floor
    o = 60
    wx = o + int(round(1.0 / res))
    big[wx, :o + int(round(0.25 / res)) + 1] = occ  # wall along x=1.0, y <= 0.25
    hug, why = plan(big, res, occ, (0.0, 0.0), (2.0, 0.0), inflate_m=0.25)
    assert hug is not None, why
    wide, why = plan(big, res, occ, (0.0, 0.0), (2.0, 0.0),
                     inflate_m=0.25, comfort_m=0.55)
    assert wide is not None, why
    d_hug = corner_distance(hug, 1.0, 0.25)
    d_wide = corner_distance(wide, 1.0, 0.25)
    assert d_wide >= 0.35, f"still hugging the corner at {d_wide:.2f} m"
    assert d_wide > d_hug + 0.04, (
        f"comfort changed nothing: {d_hug:.2f} -> {d_wide:.2f}")
    assert length(wide) < length(hug) + 1.0, (
        f"the wide route ballooned: {length(hug):.2f} -> {length(wide):.2f}")

    pref, why = plan(big, res, occ, (0.0, 0.0), (2.0, 0.0),
                     inflate_m=0.25, preferred_m=0.45)
    assert pref is not None, why
    d_pref = corner_distance(pref, 1.0, 0.25)
    assert d_pref >= 0.40, f"preferred still grazed the corner at {d_pref:.2f} m"
    assert d_pref > d_hug + 0.10, (
        f"preferred changed nothing: {d_hug:.2f} -> {d_pref:.2f}")

    # ...but a narrow gap is still taken when it is the only way through. A
    # 0.6 m opening leaves one free cell after the 0.25 m keep-out; preferred
    # 0.45 m seals it, so this is the fallback, not a refusal.
    big[wx, :] = occ
    gap0 = o - int(round(0.30 / res))
    gap1 = o + int(round(0.30 / res))
    big[wx, gap0:gap1] = -10
    path, why = plan(big, res, occ, (0.0, 0.0), (2.0, 0.0),
                     inflate_m=0.25, preferred_m=0.45, comfort_m=0.55)
    assert path is not None, f"refused a gap the chassis fits through: {why}"
    assert all(abs(y) < 0.31 for x, y in
               [point_at(path, s * 0.05) for s in range(int(length(path) / 0.05))]
               if 0.9 < x < 1.1), f"did not go through the gap: {path}"

    # Facing a wall, goal on the other side: the nose is blocked and turning is
    # free, so the first hop must not be into the keep-out.
    faced = np.full((120, 120), -10, dtype=np.int8)
    wx = o + int(round(0.50 / res))
    faced[wx, :o + int(round(0.80 / res)) + 1] = occ
    path, why = plan(faced, res, occ, (0.0, 0.0), (1.2, 0.0),
                     inflate_m=0.25, preferred_m=0.45, start_yaw=0.0)
    assert path is not None, f"refused a turn-first route: {why}"
    assert len(path) >= 2, path
    hop = math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
    assert abs(hop) > math.radians(50), (
        f"first hop into the wall: {path[:4]} heading {math.degrees(hop):.0f} deg")

    # And a clear run ahead is still a clear run: yaw must not invent a turn.
    clear = np.full((120, 120), -10, dtype=np.int8)
    path, why = plan(clear, res, occ, (0.0, 0.0), (1.5, 0.0),
                     inflate_m=0.25, preferred_m=0.45, start_yaw=0.0)
    assert path is not None, why
    assert all(abs(y) < 0.15 for _, y in path), f"yaw forced a detour: {path}"

    # Straightening. On empty floor a route to anywhere is one segment: every
    # monotone staircase A* could have returned costs the same, and the corner it
    # picked between them is the grid talking. This is the case the whole thing
    # exists for -- before it, this route came back as a 4.25 m run straight ahead
    # and then a 25 degree kink.
    empty = np.full((160, 160), -10, dtype=np.int8)
    for goal in ((3.0, 0.8), (3.0, 2.0), (2.0, 3.0), (3.0, -1.7)):
        line, why = plan(empty, res, occ, (-3.0, 0.0), goal, inflate_m=0.25,
                         preferred_m=0.45, comfort_m=0.55)
        assert line is not None, why
        assert len(line) == 2, f"empty floor to {goal} came back with {line}"
        direct = math.hypot(goal[0] + 3.0, goal[1])
        assert abs(length(line) - direct) < 0.02, (
            f"straight route is {length(line):.2f} m, direct is {direct:.2f} m")

    # But it is a shortcut only where there is nothing in the way. The same skew
    # trip with a wall across the middle keeps the corner that gets round it, and
    # keeps its distance from the end of that wall.
    walled = np.full((160, 160), -10, dtype=np.int8)
    o2 = 80
    wcol = o2 + int(round(0.0 / res))
    walled[wcol, :o2 + int(round(1.5 / res))] = occ   # wall along x=0, y < 1.5
    bent, why = plan(walled, res, occ, (-3.0, 0.0), (3.0, 2.0), inflate_m=0.25,
                     preferred_m=0.45, comfort_m=0.55)
    assert bent is not None, why
    assert len(bent) >= 3, f"straightened through a wall: {bent}"
    assert corner_distance(bent, 0.0, 1.5) >= 0.40, (
        f"straightening grazed the wall end at "
        f"{corner_distance(bent, 0.0, 1.5):.2f} m")

    # And it never crosses the keep-out anywhere, which is the promise that makes
    # it safe to shorten a route at all.
    def min_clearance(pts, grid_, from_m=0.0, to_m=None):
        cells = grid_.shape[0]
        og = cells // 2
        solid = np.argwhere(grid_ >= occ)
        xs = (solid[:, 0] - og) * res
        ys = (solid[:, 1] - og) * res
        best, s2 = 1e9, from_m
        stop = length(pts) if to_m is None else to_m
        while s2 <= stop + 1e-9:
            px, py = point_at(pts, s2)
            best = min(best, float(np.min(np.hypot(xs - px, ys - py))))
            s2 += 0.02
        return best

    assert min_clearance(bent, walled) >= 0.44, (
        f"straightened route runs {min_clearance(bent, walled):.3f} m from a wall, "
        f"inside the 0.45 m ring it was planned with")

    # The turn-off-a-wall hop is not a corner to be shortcut away. Straightening
    # the route from the pose onto that first free cell is exactly the chord
    # through the wall that the hop exists to prevent.
    hopped, why = plan(faced, res, occ, (0.0, 0.0), (1.2, 0.0),
                       inflate_m=0.25, preferred_m=0.45, start_yaw=0.0)
    assert hopped is not None, why
    first_hop = math.atan2(hopped[1][1] - hopped[0][1],
                           hopped[1][0] - hopped[0][0])
    assert abs(first_hop) > math.radians(50), (
        f"straightening ate the turn-first hop: {hopped[:3]}")

    # carrot_at. Far from a vertex it is the plain lookahead...
    square = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]      # a right angle
    bend = [(0.0, 0.0), (2.0, 0.0), (3.0, 0.5)]        # 27 degrees, driven through
    cx, cy = carrot_at(square, 0.5, 0.80, 0.40, 40.0)
    assert abs(cx - 1.3) < 1e-6 and abs(cy) < 1e-6, (cx, cy)
    # ...and it never aims onto the next leg, which is what drives the chord.
    cx, cy = carrot_at(square, 1.6, 0.80, 0.0, 40.0)
    assert abs(cx - 2.0) < 1e-6 and abs(cy) < 1e-6, (cx, cy)
    # At a gentle corner it runs on along this leg's own line rather than
    # collapsing onto a vertex 5 cm away -- and stays on that line, not the next.
    cx, cy = carrot_at(bend, 1.95, 0.80, 0.40, 40.0)
    assert abs(cy) < 1e-6, f"the carrot bent onto the next leg: {(cx, cy)}"
    assert abs(cx - 2.35) < 1e-6, (cx, cy)
    assert math.hypot(cx - 1.95, cy) >= 0.40 - 1e-9, "carrot still inside the minimum"
    # A corner the rover is going to stop and spin at keeps the carrot it had.
    # Running on past a right angle arrives at the corner still under power, and
    # a corner is where the route has the least room to spare.
    cx, cy = carrot_at(square, 1.95, 0.80, 0.40, 40.0)
    assert abs(cx - 2.0) < 1e-6 and abs(cy) < 1e-6, (
        f"the carrot ran on past a right angle: {(cx, cy)}")
    # And the last waypoint is left alone, or a rover a little to one side of the
    # goal would be steered past it instead of round to it.
    cx, cy = carrot_at(square, 3.9, 0.80, 0.40, 40.0)
    assert abs(cx - 2.0) < 1e-6 and abs(cy - 2.0) < 1e-6, (
        f"the carrot ran on past the goal: {(cx, cy)}")
    assert abs(_turn_after(square, 2.0) - 90.0) < 1e-6, _turn_after(square, 2.0)
    assert abs(_turn_after(bend, 2.0) - 26.565) < 1e-3, _turn_after(bend, 2.0)
    assert _turn_after(square, length(square)) == 0.0, "a last vertex turns nowhere"

    # The two things straightening promises that a route through a clean room
    # does not exercise, checked on the function itself: that it will not shortcut
    # across the turn-off-a-wall hop, and that the corner credit is what decides a
    # shortcut which is straighter but runs nearer the toll.
    free = np.zeros((60, 60), dtype=bool)
    toll_field = np.zeros((60, 60), dtype=np.float32)
    toll_field[25:36, 28:33] = 0.8          # something to give a wide berth to

    def cellwise(x, y):
        return x / res + 30, y / res + 30

    dogleg = [(-1.0, 0.0), (0.0, -0.5), (1.0, 0.0)]
    tight = _straighten(dogleg, free, toll_field, cellwise, credit_cells=0.0,
                        credit_cap_cells=20.0)
    assert tight == dogleg, (
        f"took a more tolled line for nothing: {tight}")
    paid = _straighten(dogleg, free, toll_field, cellwise,
                       credit_cells=KINK_CREDIT_M / res,
                       credit_cap_cells=KINK_CREDIT_MAX_M / res)
    assert paid == [dogleg[0], dogleg[-1]], (
        f"the corner was worth more than the detour and was kept anyway: {paid}")

    # `first` holds the hop even where the line past it is perfectly clear.
    hop = [(0.0, 0.0), (0.0, 0.5), (1.0, 0.5)]
    assert _straighten(hop, free, None, cellwise, first=0) == [hop[0], hop[-1]], (
        "nothing was shortcut on an empty floor")
    assert _straighten(hop, free, None, cellwise, first=1) == hop, (
        "the hop that turns the rover off a wall was shortcut away")

    # Progress along a path does not rewind for a small sideways weave.
    line = [(0.0, 0.0), (2.0, 0.0)]
    s, d = project(line, (1.0, 0.12), 1.0, slack_m=0.35)
    assert abs(s - 1.0) < 0.05 and 0.10 < d < 0.15, (s, d)
    s, d = project(line, (0.7, 0.0), 1.0, slack_m=0.35)
    assert 0.65 < s < 0.75, f"should be allowed to slide back 30 cm, got s={s}"
    s, d = project(line, (0.4, 0.0), 1.0, slack_m=0.35)
    assert s >= 0.64, f"rewound past the slack to s={s}"

    corner = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    assert abs(segment_end_s(corner, 0.0) - 1.0) < 1e-9
    assert abs(segment_end_s(corner, 0.4) - 1.0) < 1e-9
    assert abs(segment_end_s(corner, 1.0) - 2.0) < 1e-9
    assert abs(segment_end_s(corner, 1.5) - 2.0) < 1e-9

    print("planner: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
