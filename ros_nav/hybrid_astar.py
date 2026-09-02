#!/usr/bin/env python3
"""A Hybrid-A* search on this rover's costmap, with no ROS in it.

NavFn is a grid Dijkstra. It has no turning radius, so the path it traces around
a doorway is a polyline whose first metre can bend more than one DWB rollout can
follow. DWB then prefers a pivot -- standing still stays on the line -- and the
rover locks up in the passage. See docs/doorway-pivot.md.

Nav2's live answer on this rover is SmacPlannerLattice (see lattice.py): the
same costmap, searched over a differential control set that can pivot and
cannot turn tighter than 0.5 m while driving. This module is the Dubins
Hybrid-A* that was the first kinematic stand-in -- still the geometry the
doorway test is scored on (windows, densify, the 55 deg passage), and still
a useful comparison, but not the plugin the rover runs.

The radius is not a guess. DWB's only forward sample is max_vel_x, its fastest
turn is max_vel_theta, and the tightest arc that combination can draw is
max_vel_x / max_vel_theta. A path that respects that number is one the
controller can stay on without pivoting.
"""

from __future__ import annotations

import heapq
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import corridor_sim as dwb
import goal_fit

#: The tightest arc DWB can follow while driving. Pivot samples are a different
#: manoeuvre and are how this chassis changes heading when the path actually
#: asks it to; they are not a reason to draw a corner the forward samples cannot
#: take.
MIN_TURNING_RADIUS = dwb.MAX_VEL_X / dwb.MAX_VEL_THETA

#: One DWB forward rollout, in metres and in heading.
ROLLOUT_M = dwb.MAX_VEL_X * dwb.SIM_TIME
ROLLOUT_RAD = dwb.MAX_VEL_THETA * dwb.SIM_TIME

#: How far along a path to look when asking "does this set off with a bend".
#: The recorded doorway plans bent 44 to 67 degrees in this window.
FIRST_BEND_M = 1.2

#: Angular bins for the Hybrid-A* state. 32 is 11.25 deg, coarser than Nav2's
#: stock 72 and fine enough that a 0.51 m arc steps about two costmap cells.
ANGLE_BINS = 32

#: Stop searching. A metre-class L on a 5 cm grid is a few thousand expansions;
#: this is the cap that turns a bug into a failure rather than a hang.
MAX_EXPANSIONS = 200000

#: Goal xy tolerance, matching nav2.yaml's NavFn/Smac tolerance.
TOLERANCE_M = 0.25


def wrap(radians):
    return math.atan2(math.sin(radians), math.cos(radians))


def angle_bin(yaw, bins=ANGLE_BINS):
    step = 2.0 * math.pi / bins
    return int(round(wrap(yaw) / step) % bins)


def traversable(grid, col, row, allow_unknown=True):
    """Can the search step on this cell, the way NavFn would.

    253 is the inscribed ring and is a hard obstacle to both planners. Unknown
    is allowed because this rover maps as it drives -- a goal in a room it has
    not seen yet is a normal request.
    """
    if not (0 <= col < grid.width and 0 <= row < grid.height):
        return False
    cost = grid.cost(col, row)
    if cost >= goal_fit.INSCRIBED and cost != goal_fit.UNKNOWN:
        return False
    if cost == goal_fit.UNKNOWN and not allow_unknown:
        return False
    return True


def cell_blocked(grid, x, y, allow_unknown=True):
    col, row = grid.cell_of(x, y)
    return not traversable(grid, col, row, allow_unknown)


# --- path geometry ------------------------------------------------------------
def path_length(points):
    if len(points) < 2:
        return 0.0
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(points, points[1:]))


def headings(points):
    """Heading of each segment, length-weighted onto the vertices.

    A one-point path has no heading. The last vertex keeps the last segment's
    heading so a window that lands on the end still has a direction.
    """
    if len(points) < 2:
        return []
    out = []
    last = None
    for a, b in zip(points, points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        if math.hypot(dx, dy) < 1e-9:
            out.append(last if last is not None else 0.0)
            continue
        last = math.atan2(dy, dx)
        out.append(last)
    out.append(last if last is not None else 0.0)
    return out


def _yaw_of(a, b):
    """Heading of the segment a->b, preferring a stored yaw when the path has one.

    Hybrid-A* poses carry the Dubins heading. Using the chord instead turns a
    smooth 0.51 m arc into a 45-degree kink at every bin, which is the
    measurement that would condemn the planner for the artefact of how it was
    sampled.
    """
    if len(a) > 2:
        return a[2]
    return math.atan2(b[1] - a[1], b[0] - a[0])


def heading_at(points, distance):
    """Path heading after travelling `distance` metres from the start."""
    if len(points) < 2:
        return None
    remaining = distance
    last = _yaw_of(points[0], points[1])
    for a, b in zip(points, points[1:]):
        step = math.hypot(b[0] - a[0], b[1] - a[1])
        if step < 1e-9:
            continue
        last = _yaw_of(a, b)
        if remaining <= step:
            if len(a) > 2 and len(b) > 2:
                return wrap(a[2] + remaining / step * wrap(b[2] - a[2]))
            return last
        remaining -= step
    if len(points[-1]) > 2:
        return points[-1][2]
    return last


def pose_at(points, distance):
    """(x, y, heading) after travelling `distance` metres along the path."""
    if len(points) < 2:
        return None
    remaining = distance
    x, y = points[0][0], points[0][1]
    yaw = _yaw_of(points[0], points[1])
    for a, b in zip(points, points[1:]):
        step = math.hypot(b[0] - a[0], b[1] - a[1])
        if step < 1e-9:
            continue
        yaw = _yaw_of(a, b)
        if remaining <= step:
            share = remaining / step
            heading = yaw
            if len(a) > 2 and len(b) > 2:
                heading = wrap(a[2] + share * wrap(b[2] - a[2]))
            return (a[0] + share * (b[0] - a[0]),
                    a[1] + share * (b[1] - a[1]), heading)
        remaining -= step
        x, y = b[0], b[1]
        if len(b) > 2:
            yaw = b[2]
    return (x, y, yaw)


def densify(poses, ds=0.05, radius=MIN_TURNING_RADIUS):
    """Fill in Hybrid-A* primitive endpoints so a 0.32 m window is a real arc.

    Search nodes are one heading bin apart, about 10 cm. Measuring a window
    that short against the chords between them reports the bin size, not the
    curvature.
    """
    if len(poses) < 2:
        return list(poses)
    out = [poses[0]]
    for a, b in zip(poses, poses[1:]):
        if len(a) < 3 or len(b) < 3:
            out.append(b)
            continue
        dth = wrap(b[2] - a[2])
        if abs(dth) < 1e-6:
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            steps = max(1, int(math.ceil(length / ds)))
            for i in range(1, steps + 1):
                share = float(i) / steps
                out.append((a[0] + share * (b[0] - a[0]),
                            a[1] + share * (b[1] - a[1]), a[2]))
            continue
        length = abs(dth) * radius
        steps = max(1, int(math.ceil(length / ds)))
        for i in range(1, steps + 1):
            share = float(i) / steps
            out.append(_step(a[0], a[1], a[2], radius, dth * share, length * share))
    return out


def first_bend(points, window_m=FIRST_BEND_M):
    """Heading change over the first `window_m` of the path, in radians."""
    start = heading_at(points, 0.0)
    end = heading_at(points, window_m)
    if start is None or end is None:
        return 0.0
    return abs(wrap(end - start))


def windows(points, window_m):
    """(distance_along_path, heading_change) for every window of `window_m`."""
    length = path_length(points)
    if length < window_m or len(points) < 2:
        return []
    # Step by the costmap cell so a grid kink cannot hide between samples.
    step = max(0.05, window_m / 8.0)
    out = []
    at = 0.0
    while at + window_m <= length + 1e-9:
        a = heading_at(points, at)
        b = heading_at(points, at + window_m)
        if a is not None and b is not None:
            out.append((at, abs(wrap(b - a))))
        at += step
    return out


def tightest_window(points, window_m=ROLLOUT_M):
    """The sharpest heading change over any `window_m` stretch, in radians.

    This is the number that decides whether DWB can follow the path: one
    forward rollout is `ROLLOUT_M` long and can turn at most `ROLLOUT_RAD`.
    A window tighter than that is a corner the controller cannot take without
    leaving the line, which is when the pivot wins.
    """
    found = windows(points, window_m)
    if not found:
        return 0.0, 0.0
    at, bend = max(found, key=lambda item: item[1])
    return at, bend


# --- 8-connected A*, standing in for NavFn ------------------------------------
def grid_astar(grid, start, goal, allow_unknown=True, max_expansions=MAX_EXPANSIONS):
    """A point-robot A* on the cost grid, which is what NavFn is.

    Eight-connected, 253 is a wall, unknown is allowed. The path is cell
    centres. It has no heading in the state, so a doorway corner comes back as
    a diagonal cut the body cannot follow at speed.
    """
    sc, sr = grid.cell_of(start[0], start[1])
    gc, gr = grid.cell_of(goal[0], goal[1])
    if not traversable(grid, sc, sr, allow_unknown):
        return None
    if not traversable(grid, gc, gr, allow_unknown):
        return None

    def h(col, row):
        return math.hypot(col - gc, row - gr)

    start_key = (sc, sr)
    came = {start_key: None}
    g_score = {start_key: 0.0}
    heap = [(h(sc, sr), 0.0, sc, sr)]
    expanded = 0
    found = None
    while heap and expanded < max_expansions:
        _f, g, col, row = heapq.heappop(heap)
        if g > g_score.get((col, row), 1e18) + 1e-12:
            continue
        expanded += 1
        if (col, row) == (gc, gr) or (
                math.hypot(col - gc, row - gr) * grid.resolution <= TOLERANCE_M
                and traversable(grid, col, row, allow_unknown)):
            found = (col, row)
            break
        for dcol, drow in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nc, nr = col + dcol, row + drow
            if not traversable(grid, nc, nr, allow_unknown):
                continue
            step = math.hypot(dcol, drow)
            cost = grid.cost(nc, nr)
            if cost == goal_fit.UNKNOWN:
                cost = 0
            travel = step * (1.0 + cost / 252.0)
            ng = g + travel
            key = (nc, nr)
            if ng + 1e-12 < g_score.get(key, 1e18):
                g_score[key] = ng
                came[key] = (col, row)
                heapq.heappush(heap, (ng + h(nc, nr), ng, nc, nr))
    if found is None:
        return None
    cells = []
    node = found
    while node is not None:
        cells.append(node)
        node = came[node]
    cells.reverse()
    return [(grid.origin_x + (c + 0.5) * grid.resolution,
             grid.origin_y + (r + 0.5) * grid.resolution)
            for c, r in cells]


# --- Hybrid-A* (Dubins), the kinematic comparison lattice.py is scored against --
def _primitives(radius, bins=ANGLE_BINS):
    """The three Dubins motions from one heading bin: left, straight, right.

    One bin of heading at this radius is the step length, so a 0.51 m turn
    on a 5 cm map moves about two cells. Matching Smac's search model rather
    than inventing a denser one: denser looks smoother and is a different
    planner.
    """
    dtheta = 2.0 * math.pi / bins
    length = radius * dtheta
    return (
        ("left", length, dtheta),
        ("straight", length, 0.0),
        ("right", length, -dtheta),
    )


def _step(x, y, yaw, radius, dtheta, length):
    if abs(dtheta) < 1e-12:
        return (x + length * math.cos(yaw),
                y + length * math.sin(yaw),
                wrap(yaw))
    # Left of the heading is (-sin, cos). The sign of dtheta picks the side.
    side = 1.0 if dtheta > 0.0 else -1.0
    cx = x - side * radius * math.sin(yaw)
    cy = y + side * radius * math.cos(yaw)
    nyaw = wrap(yaw + dtheta)
    nx = cx + side * radius * math.sin(nyaw)
    ny = cy - side * radius * math.cos(nyaw)
    return nx, ny, nyaw


def _segment_clear(grid, x, y, yaw, radius, dtheta, length, allow_unknown):
    """Sample the primitive at costmap resolution so a wall between bins is seen."""
    samples = max(2, int(math.ceil(length / grid.resolution)))
    for i in range(1, samples + 1):
        share = float(i) / samples
        px, py, _ = _step(x, y, yaw, radius, dtheta * share, length * share)
        if cell_blocked(grid, px, py, allow_unknown):
            return False
    return True


def hybrid_astar(grid, start, goal, radius=MIN_TURNING_RADIUS,
                 allow_unknown=True, max_expansions=MAX_EXPANSIONS,
                 bins=ANGLE_BINS, tolerance=TOLERANCE_M):
    """SE2 A* with Dubins primitives, which is what SmacPlannerHybrid is.

    Not the live plugin -- that is SmacPlannerLattice -- but the same envelope
    test, without in-place rotations. Kept because a lattice path that is
    followable only because it pivoted is a different claim from one that
    took the arc.

    `start` and `goal` are (x, y, yaw). The goal heading is a preference: a
    pose within `tolerance` of the goal xy is accepted even if the heading
    is off, because that is how NavFn was allowed to finish and a comparison
    that demands a heading NavFn never had is not a comparison.
    """
    sx, sy, syaw = start
    gx, gy, gyaw = goal
    if cell_blocked(grid, sx, sy, allow_unknown):
        return None
    primitives = _primitives(radius, bins)
    start_key = (int(math.floor((sx - grid.origin_x) / grid.resolution)),
                 int(math.floor((sy - grid.origin_y) / grid.resolution)),
                 angle_bin(syaw, bins))

    def heuristic(x, y, yaw):
        # Euclidean is admissible for Dubins-with-any-heading. Adding a heading
        # term would not be, once we accept any arrival heading.
        return math.hypot(gx - x, gy - y)

    came = {start_key: None}
    pose_of = {start_key: (sx, sy, wrap(syaw))}
    g_score = {start_key: 0.0}
    heap = [(heuristic(sx, sy, syaw), 0.0, start_key)]
    expanded = 0
    found = None
    while heap and expanded < max_expansions:
        _f, g, key = heapq.heappop(heap)
        if g > g_score.get(key, 1e18) + 1e-12:
            continue
        x, y, yaw = pose_of[key]
        expanded += 1
        if math.hypot(gx - x, gy - y) <= tolerance:
            found = key
            break
        for _name, length, dtheta in primitives:
            if not _segment_clear(grid, x, y, yaw, radius, dtheta, length,
                                  allow_unknown):
                continue
            nx, ny, nyaw = _step(x, y, yaw, radius, dtheta, length)
            nkey = (int(math.floor((nx - grid.origin_x) / grid.resolution)),
                    int(math.floor((ny - grid.origin_y) / grid.resolution)),
                    angle_bin(nyaw, bins))
            col, row, _ = nkey
            if not traversable(grid, col, row, allow_unknown):
                continue
            cost = grid.cost(col, row)
            if cost == goal_fit.UNKNOWN:
                cost = 0
            ng = g + length * (1.0 + cost / 252.0)
            if ng + 1e-12 < g_score.get(nkey, 1e18):
                g_score[nkey] = ng
                came[nkey] = key
                pose_of[nkey] = (nx, ny, nyaw)
                heapq.heappush(heap, (ng + heuristic(nx, ny, nyaw), ng, nkey))
    if found is None:
        return None
    keys = []
    node = found
    while node is not None:
        keys.append(node)
        node = came[node]
    keys.reverse()
    return [pose_of[k] for k in keys]


# --- the doorway geometry this exists to reproduce ----------------------------
def _dist_to_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 < 1e-18:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def bent_passage(width_m=1.0, bend_deg=55.0, leg_m=2.5):
    """A metre-wide passage that turns `bend_deg` left.

    The recorded doorway plans bent 44 to 67 degrees in their first 1.2 m.
    Fifty-five is the middle of that range, in the same width of passage, so
    a grid search still cuts the inner corner and a turning-radius search
    still has to swing.
    """
    res = dwb.RESOLUTION
    margin = 0.6
    yaw = math.radians(bend_deg)
    ax, ay = -leg_m, 0.0
    bx, by = 0.0, 0.0
    cx, cy = leg_m * math.cos(yaw), leg_m * math.sin(yaw)
    xs = (ax, bx, cx)
    ys = (ay, by, cy)
    origin_x = min(xs) - width_m / 2.0 - margin
    origin_y = min(ys) - width_m / 2.0 - margin
    span_x = max(xs) - min(xs) + width_m + 2.0 * margin
    span_y = max(ys) - min(ys) + width_m + 2.0 * margin
    width = int(round(span_x / res))
    height = int(round(span_y / res))
    half = width_m / 2.0
    lethal = []
    for col in range(width):
        x = origin_x + (col + 0.5) * res
        for row in range(height):
            y = origin_y + (row + 0.5) * res
            inside = (
                _dist_to_segment(x, y, ax, ay, bx, by) <= half
                or _dist_to_segment(x, y, bx, by, cx, cy) <= half)
            if not inside:
                lethal.append((col, row))
    data = dwb.inflate(width, height, lethal)
    grid = goal_fit.CostGrid(width, height, res, origin_x, origin_y, data)
    start = (ax + 0.45, 0.0, 0.0)
    goal = (cx - 0.45 * math.cos(yaw), cy - 0.45 * math.sin(yaw), yaw)
    return grid, start, goal


def xy_of(path):
    return [(p[0], p[1]) for p in path]


def dwb_families(kept):
    """Best pivot and best forward among a scored sample set. Lowest wins."""
    pivot = forward = None
    for row in kept:
        score, vx, wz = row[0], row[1], row[2]
        if abs(vx) < 1e-6:
            if pivot is None:
                pivot = (score, vx, wz)
        else:
            if forward is None:
                forward = (score, vx, wz)
    return pivot, forward


def dwb_at(grid, path, pose, path_look=None, goal_look=None):
    """What DWB would pick at `pose` following `path`, on this costmap.

    Returns (pivot_score, forward_score, margin) where a positive margin means
    the forward arc wins. None scores mean that family had no legal candidate.
    """
    if len(path) < 2:
        return None
    x, y, yaw = pose
    points = xy_of(path) if len(path[0]) > 2 else path
    local = _trim_path(points, grid, x, y)
    if len(local) < 2:
        return None
    kept, _refused = dwb.evaluate(grid, local, local[-1], x, y, yaw,
                                 path_look=path_look, goal_look=goal_look)
    pivot, forward = dwb_families(kept)
    p = None if pivot is None else pivot[0]
    f = None if forward is None else forward[0]
    margin = None if (p is None or f is None) else (p - f)
    return {"pivot": p, "forward": f, "margin": margin,
            "legal": len(kept), "chose": ("forward" if (margin is not None and margin > 0)
                                          else "pivot" if margin is not None
                                          else "none")}


def _trim_path(points, grid, x, y):
    """The part of the path DWB would still score, without importing dwb_replay."""
    if len(points) < 2:
        return points
    best_i, best_d = 0, 1e18
    for i, (px, py) in enumerate(points):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < best_d:
            best_d, best_i = d, i
    horizon = max(grid.width, grid.height) * grid.resolution / 2.0
    out = []
    for px, py in points[best_i:]:
        if math.hypot(px - x, py - y) > horizon:
            break
        out.append((px, py))
    if len(out) < 2:
        out = points[best_i:best_i + 2]
    return out


def describe_path(path, label):
    if not path or len(path) < 2:
        return {"label": label, "length_m": 0.0, "first_bend_deg": 0.0,
                "tightest_deg": 0.0, "tightest_at_m": 0.0, "poses": 0}
    at, bend = tightest_window(path)
    return {
        "label": label,
        "length_m": path_length(path),
        "first_bend_deg": math.degrees(first_bend(path)),
        "tightest_deg": math.degrees(bend),
        "tightest_at_m": at,
        "poses": len(path),
        "followable": bend <= ROLLOUT_RAD + math.radians(1.0),
    }


def approach_pose(path, look_ahead=0.80):
    """Where the rover is standing when the corner is still ahead of it.

    Scoring DWB at the kink itself is the other half of the frozen-map trap:
    the heading there already matches the path, so every candidate looks
    fine. The recorded lock-up was a rover that had not yet turned, with
    1.2 m of plan in front of it that already bent 44 to 67 degrees.
    """
    at, _ = tightest_window(path)
    stand = max(0.0, at - look_ahead)
    return pose_at(path, stand), stand


def midcourse_stats(grid, path, step=0.05):
    """DWB's choices along a path, stopping before the arrival circle.

    Near the goal `RotateToGoal` throws every forward sample, which is the
    controller arriving, not the doorway lock-up. Those ticks are dropped so
    a path that is followable through its corner is not condemned for parking.
    """
    if not path or len(path) < 2:
        return {"ticks": 0, "forward": 0, "pivot": 0, "no_forward": 0,
                "longest_stall_m": 0.0}
    goal = path[-1]
    s = 0.0
    length = path_length(path)
    forward = pivot = no_forward = 0
    streak = longest = 0.0
    while s < length:
        pose = pose_at(path, s)
        if pose is None:
            break
        if math.hypot(pose[0] - goal[0], pose[1] - goal[1]) <= dwb.XY_GOAL_TOLERANCE:
            break
        got = dwb_at(grid, path, pose)
        s += step
        if not got:
            continue
        if got["forward"] is None:
            no_forward += 1
            stalled = True
        elif got["chose"] == "pivot":
            pivot += 1
            stalled = True
        else:
            forward += 1
            stalled = False
        if stalled:
            streak += step
            longest = max(longest, streak)
        else:
            streak = 0.0
    return {"ticks": forward + pivot + no_forward, "forward": forward,
            "pivot": pivot, "no_forward": no_forward,
            "longest_stall_m": longest}


def doorway_reproduction():
    """NavFn vs Hybrid-A* on a 55-degree metre-wide bend, and DWB on each path.

    This is the test the first doorway fix did not have: the path is what
    changes, the costmap is the same, and each controller is asked to follow
    the path it would actually have been handed. A frozen-map closed loop that
    starts from a NavFn deadlock is not this, and is how a look-ahead change
    shipped on worthless evidence.
    """
    grid, start, goal = bent_passage()
    navfn = grid_astar(grid, (start[0], start[1]), (goal[0], goal[1]))
    hybrid = hybrid_astar(grid, start, goal)
    if hybrid:
        hybrid = densify(hybrid)
    result = {
        "radius_m": MIN_TURNING_RADIUS,
        "rollout_m": ROLLOUT_M,
        "rollout_deg": math.degrees(ROLLOUT_RAD),
        "navfn": None if navfn is None else describe_path(navfn, "navfn"),
        "hybrid": None if hybrid is None else describe_path(hybrid, "hybrid"),
        "navfn_dwb": None,
        "hybrid_dwb": None,
        "navfn_mid": None,
        "hybrid_mid": None,
    }
    if navfn:
        pose, stand = approach_pose(navfn, look_ahead=0.25)
        if pose is not None:
            result["navfn"]["approach_bend_deg"] = math.degrees(abs(wrap(
                heading_at(navfn, stand + FIRST_BEND_M) - pose[2])))
            result["navfn_dwb"] = dwb_at(grid, navfn, pose)
            result["navfn_dwb"]["stand_m"] = stand
        result["navfn_mid"] = midcourse_stats(grid, navfn)
    if hybrid:
        pose, stand = approach_pose(hybrid, look_ahead=0.25)
        if pose is not None:
            result["hybrid"]["approach_bend_deg"] = math.degrees(abs(wrap(
                heading_at(hybrid, stand + FIRST_BEND_M) - pose[2])))
            result["hybrid_dwb"] = dwb_at(grid, hybrid, pose)
            result["hybrid_dwb"]["stand_m"] = stand
        result["hybrid_mid"] = midcourse_stats(grid, hybrid)
    return result
