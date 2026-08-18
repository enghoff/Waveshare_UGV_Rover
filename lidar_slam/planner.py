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
# The inflate radius is the same standoff the driving loop keeps, so a path that
# threads a gap the rover cannot actually take is not offered in the first place.


def plan(grid, resolution_m, occupied_at, start_xy, goal_xy, inflate_m,
         origin_cells=None):
    """World metres -> a polyline of world metres, or (None, why_not).

    `grid` is indexed [forward, left] the way slam2d's occupancy is, with the origin
    cell at `origin_cells` (defaults to the centre). `start_xy` and `goal_xy` are
    metres in that same frame.
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
    # radius of a wall. Leaving the start blocked is how a planner refuses to
    # extract itself from a tight spot.
    lsx, lsy = sx - x0, sy - y0
    lgx, lgy = gx - x0, gy - y0
    inflated[lsx, lsy] = False

    if inflated[lgx, lgy]:
        snapped = _nearest_free(inflated, lgx, lgy, max(radius + 2, 8))
        if snapped is None:
            return None, "there is no room to stand at that place"
        lgx, lgy = snapped

    local = _astar(inflated, (lsx, lsy), (lgx, lgy))
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
    """Euclidean dilation of a boolean grid, in-place-safe (returns a copy)."""
    import numpy as np

    if radius_cells <= 0:
        return blocked.copy()
    out = blocked.copy()
    ys, xs = np.nonzero(blocked)
    if len(ys) == 0:
        return out
    h, w = blocked.shape
    r2 = radius_cells * radius_cells
    for y, x in zip(ys.tolist(), xs.tolist()):
        y0, y1 = max(0, y - radius_cells), min(h, y + radius_cells + 1)
        x0, x1 = max(0, x - radius_cells), min(w, x + radius_cells + 1)
        yy = np.arange(y0, y1)[:, None]
        xx = np.arange(x0, x1)[None, :]
        out[y0:y1, x0:x1] |= ((yy - y) * (yy - y) + (xx - x) * (xx - x)) <= r2
    return out


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


def _astar(blocked, start, goal):
    """8-connected A* on a boolean blocked grid. Returns a list of (ix, iy)."""
    import numpy as np

    h, w = blocked.shape
    sx, sy = start
    gx, gy = goal
    if blocked[gx, gy] or blocked[sx, sy]:
        return None
    if start == goal:
        return [start]

    nbrs = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)))

    def h_cost(ix, iy):
        return math.hypot(gx - ix, gy - iy)

    inf = 1e18
    g = np.full((h, w), inf, dtype=np.float64)
    came = np.full((h, w, 2), -1, dtype=np.int32)
    g[sx, sy] = 0.0
    heap = [(h_cost(sx, sy), 0.0, sx, sy)]
    closed = np.zeros((h, w), dtype=bool)

    while heap:
        _f, cost, x, y = heapq.heappop(heap)
        if closed[x, y]:
            continue
        closed[x, y] = True
        if (x, y) == (gx, gy):
            return _reconstruct(came, start, goal)
        for dx, dy, step in nbrs:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < h and 0 <= ny < w) or blocked[nx, ny] or closed[nx, ny]:
                continue
            # No cutting a corner through a blocked diagonal neighbour.
            if dx != 0 and dy != 0 and (blocked[x + dx, y] or blocked[x, y + dy]):
                continue
            nxt = cost + step
            if nxt + 1e-9 < g[nx, ny]:
                g[nx, ny] = nxt
                came[nx, ny] = (x, y)
                heapq.heappush(heap, (nxt + h_cost(nx, ny), nxt, nx, ny))
    return None


def _reconstruct(came, start, goal):
    path = [goal]
    x, y = goal
    sx, sy = start
    for _ in range(came.shape[0] * came.shape[1] + 1):
        if (x, y) == (sx, sy):
            path.reverse()
            return path
        px, py = int(came[x, y, 0]), int(came[x, y, 1])
        if px < 0:
            return None
        path.append((px, py))
        x, y = px, py
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

    # Unseen target is refused.
    grid[:, :] = 0
    path, why = plan(grid, res, occ, (0.0, 0.0), (1.0, 0.0), inflate_m=0.20)
    assert path is None and "seen" in why, why

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
