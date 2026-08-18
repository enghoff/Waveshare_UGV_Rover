#!/usr/bin/env python3
"""A route through the occupancy grid, as a few waypoints rather than a cell path.

The map is the only picture of the room that reaches more than a couple of metres,
so going to a place the rover is not already facing means planning on it. The pose
drifts and the grid holds geometry that has since moved, so this is a sketch, not a
promise: the navigator follows the sketch with the live scan in the loop, and asks
for another one when the room disagrees.

What comes out is a short polyline. A* runs at cell resolution so it can thread a
gap; the caller does not want a hundred 5 cm hops, so the path is then thinned to
the corners that actually change heading. Following that is a handful of turns and
straight segments, which is what the rover can do.
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
# opening and sealed pinches the chassis still fits. Navigator uses 0.25 m.
#
# Beyond that hard ring there is a soft one -- see `comfort_m`. A route hugging
# the keep-out boundary passes corners at exactly the distance the follower
# brakes at, so an ordinary pose error turns a legal route into a stop. Charging
# extra for travelling near anything blocked swings the route wide when there is
# room, and still takes a narrow gap when there is not, because in a squeeze
# every route pays the same toll and the shortest one is still through.

# What a step next to the keep-out costs, on top of its length: 1 + this, fading
# linearly to 1 at comfort_m. At 2.0 the planner will spend up to two extra cells
# of path to avoid one cell of wall-hugging -- enough to arc around a corner in
# the open, not enough to refuse a doorway both of whose sides charge it.
COMFORT_COST = 2.0


def plan(grid, resolution_m, occupied_at, start_xy, goal_xy, inflate_m,
         comfort_m=0.0, origin_cells=None):
    """World metres -> a polyline of world metres, or (None, why_not).

    `grid` is indexed [forward, left] the way slam2d's occupancy is, with the origin
    cell at `origin_cells` (defaults to the centre). `start_xy` and `goal_xy` are
    metres in that same frame. `comfort_m` is the distance from anything blocked
    at which travel stops costing extra; at or below `inflate_m` it does nothing.
    """
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
    # refused -- exactly the tight spot a planner most needs to leave. The first
    # waypoint is re-anchored to the true pose below, so the short blind hop from
    # the real start to free space is followed on the live scan like everything
    # else.
    lsx, lsy = sx - x0, sy - y0
    lgx, lgy = gx - x0, gy - y0
    if inflated[lsx, lsy]:
        freed = _nearest_free(inflated, lsx, lsy, max(radius + 2, 8))
        if freed is None:
            return None, "the map shows no room to move at all from here"
        lsx, lsy = freed

    if inflated[lgx, lgy]:
        snapped = _nearest_free(inflated, lgx, lgy, max(radius + 2, 8))
        if snapped is None:
            return None, "there is no room to stand at that place"
        lgx, lgy = snapped

    comfort_cells = int(math.ceil(max(0.0, comfort_m - inflate_m) / resolution_m))
    penalty = _proximity_penalty(inflated, comfort_cells)
    local = _astar(inflated, (lsx, lsy), (lgx, lgy), penalty)
    if local is None:
        return None, "no clear route through what the lidar has seen"

    world = [to_world(ix + x0, iy + y0) for ix, iy in local]
    # The cell centres of start and goal are not the poses that were asked for.
    # Keep the real endpoints so the follower is aiming at the tap, not at a
    # 5 cm rounding of it.
    world[0] = tuple(start_xy)
    world[-1] = tuple(goal_xy)
    return _simplify(world, 0.22), None


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
    # (1.0, 0.25); the route from (0,0) to (2,0) has to round that end, and with
    # open floor above it the comfort toll should swing it wide where a plain
    # keep-out would hug the ring at exactly inflate_m.
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

    # ...but a narrow gap is still taken when it is the only way through. A
    # 0.6 m opening leaves one free cell after the 0.25 m keep-out; the toll is
    # paid, not treated as a wall.
    big[wx, :] = occ
    gap0 = o - int(round(0.30 / res))
    gap1 = o + int(round(0.30 / res))
    big[wx, gap0:gap1] = -10
    path, why = plan(big, res, occ, (0.0, 0.0), (2.0, 0.0),
                     inflate_m=0.25, comfort_m=0.55)
    assert path is not None, f"refused a gap the chassis fits through: {why}"
    assert all(abs(y) < 0.31 for x, y in
               [point_at(path, s * 0.05) for s in range(int(length(path) / 0.05))]
               if 0.9 < x < 1.1), f"did not go through the gap: {path}"

    # Progress along a path does not rewind for a small sideways weave.
    line = [(0.0, 0.0), (2.0, 0.0)]
    s, d = project(line, (1.0, 0.12), 1.0, slack_m=0.35)
    assert abs(s - 1.0) < 0.05 and 0.10 < d < 0.15, (s, d)
    s, d = project(line, (0.7, 0.0), 1.0, slack_m=0.35)
    assert 0.65 < s < 0.75, f"should be allowed to slide back 30 cm, got s={s}"
    s, d = project(line, (0.4, 0.0), 1.0, slack_m=0.35)
    assert s >= 0.64, f"rewound past the slack to s={s}"

    print("planner: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
