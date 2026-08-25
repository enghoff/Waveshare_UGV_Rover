#!/usr/bin/env python3
"""State-lattice search using Nav2's differential-drive control set.

SmacPlannerLattice is the plugin this rover should run: a skid-steer can turn
on the spot, which Hybrid-A* Dubins cannot write into a path, and it must not
draw a corner tighter than one DWB rollout, which NavFn cannot see. The control
set in config/lattices/ is Nav2's own 5 cm, 0.5 m, differential sample -- 0.5 m
is this chassis's forward envelope (max_vel_x / max_vel_theta) to a centimetre.

No ROS in it, so the selftest can run the same search the plugin will, against
the same doorway geometry hybrid_astar.py already holds.
"""

from __future__ import annotations

import json
import math
import os
import sys
import heapq

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import goal_fit
import hybrid_astar as geo

LATTICE_PATH = os.path.join(HERE, "config", "lattices", "diff_5cm_0.5m.json")

#: Nav2 defaults that matter on this chassis. Reverse is off: the lidar looks
#: forwards. Rotation is expensive so a doorway gets a 0.5 m arc when one fits
#: and a pivot only when it does not.
ROTATION_PENALTY = 5.0
NON_STRAIGHT_PENALTY = 1.05
# In-place primitives have length 0; without a floor they are free and the
# search will spin at every cell. 0.15 m is the straight primitive in this set.
ROTATION_LENGTH_M = 0.15

_CACHE = None


def load_lattice(path=None):
    """The control set, grouped by the heading bin it may be applied from."""
    global _CACHE
    path = path or LATTICE_PATH
    if _CACHE is not None and _CACHE[0] == path:
        return _CACHE[1], _CACHE[2]
    with open(path) as handle:
        document = json.load(handle)
    meta = document["lattice_metadata"]
    by_start = {}
    for prim in document["primitives"]:
        by_start.setdefault(prim["start_angle_index"], []).append(prim)
    _CACHE = (path, meta, by_start)
    return meta, by_start


def nearest_bin(yaw, headings):
    best_i, best_d = 0, 99.0
    for i, heading in enumerate(headings):
        delta = abs(geo.wrap(yaw - heading))
        if delta < best_d:
            best_i, best_d = i, delta
    return best_i


def _clear(grid, x, y, poses, allow_unknown):
    """Every sample of the primitive, translated onto the rover's pose."""
    for px, py, _yaw in poses:
        if geo.cell_blocked(grid, x + px, y + py, allow_unknown):
            return False
    return True


def driving_stretches(path):
    """Contiguous forward legs, with in-place rotations cut out.

    The doorway test is whether a *driving* corner fits one DWB rollout. A
    lattice path is allowed to pivot -- that is the point of the differential
    control set -- and a 0.32 m window that swallowed the pivot would condemn
    a legal arc for a heading change the rover was meant to take standing
    still. Hybrid-A* Dubins never had this problem: it cannot write a pivot.
    """
    if not path:
        return []
    stretches, current = [], [path[0]]
    for a, b in zip(path, path[1:]):
        if math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-9:
            if len(current) >= 2:
                stretches.append(current)
            current = [b]
        else:
            current.append(b)
    if len(current) >= 2:
        stretches.append(current)
    return stretches


def tightest_driving_window(path, window_m=geo.ROLLOUT_M):
    """Sharpest heading change over `window_m` of forward travel, in radians."""
    best_at, best_bend = 0.0, 0.0
    offset = 0.0
    for stretch in driving_stretches(path):
        at, bend = geo.tightest_window(stretch, window_m)
        if bend > best_bend:
            best_at, best_bend = offset + at, bend
        offset += geo.path_length(stretch)
    return best_at, best_bend


def describe_path(path, label):
    """Same numbers as hybrid_astar.describe_path, on driving stretches only."""
    if not path or len(path) < 2:
        return {"label": label, "length_m": 0.0, "first_bend_deg": 0.0,
                "tightest_deg": 0.0, "tightest_at_m": 0.0, "poses": 0,
                "followable": False, "pivots": 0}
    at, bend = tightest_driving_window(path)
    stretches = driving_stretches(path)
    return {
        "label": label,
        "length_m": geo.path_length(path),
        "first_bend_deg": math.degrees(geo.first_bend(path)),
        "tightest_deg": math.degrees(bend),
        "tightest_at_m": at,
        "poses": len(path),
        "followable": bend <= geo.ROLLOUT_RAD + math.radians(1.0),
        "pivots": max(0, len(stretches) - 1),
    }


def lattice_astar(grid, start, goal, lattice_path=None, allow_unknown=True,
                  max_expansions=geo.MAX_EXPANSIONS, tolerance=geo.TOLERANCE_M,
                  rotation_penalty=ROTATION_PENALTY):
    """SE2 A* over the differential control set, which is SmacPlannerLattice.

    `start` and `goal` are (x, y, yaw). Arrival heading is a preference, not a
    requirement -- the same concession hybrid_astar makes, because NavFn never
    had one and a comparison that demands it is not a comparison.
    """
    meta, by_start = load_lattice(lattice_path)
    headings = meta["heading_angles"]
    sx, sy, syaw = start
    gx, gy, _gyaw = goal
    if geo.cell_blocked(grid, sx, sy, allow_unknown):
        return None
    start_bin = nearest_bin(syaw, headings)
    start_key = (int(math.floor((sx - grid.origin_x) / grid.resolution)),
                 int(math.floor((sy - grid.origin_y) / grid.resolution)),
                 start_bin)

    def heuristic(x, y):
        return math.hypot(gx - x, gy - y)

    came = {start_key: None}
    pose_of = {start_key: (sx, sy, headings[start_bin])}
    g_score = {start_key: 0.0}
    heap = [(heuristic(sx, sy), 0.0, start_key)]
    expanded = 0
    found = None
    while heap and expanded < max_expansions:
        _f, g, key = heapq.heappop(heap)
        if g > g_score.get(key, 1e18) + 1e-12:
            continue
        x, y, _yaw = pose_of[key]
        expanded += 1
        if math.hypot(gx - x, gy - y) <= tolerance:
            found = key
            break
        _, _, heading_i = key
        for prim in by_start.get(heading_i, ()):
            poses = prim["poses"]
            if not poses:
                continue
            if not _clear(grid, x, y, poses, allow_unknown):
                continue
            last = poses[-1]
            nx, ny = x + last[0], y + last[1]
            nbin = prim["end_angle_index"]
            nkey = (int(math.floor((nx - grid.origin_x) / grid.resolution)),
                    int(math.floor((ny - grid.origin_y) / grid.resolution)),
                    nbin)
            col, row, _ = nkey
            if not geo.traversable(grid, col, row, allow_unknown):
                continue
            length = prim["trajectory_length"]
            if length < 1e-6:
                travel = ROTATION_LENGTH_M * rotation_penalty
            elif abs(prim.get("arc_length") or 0.0) > 1e-6:
                travel = length * NON_STRAIGHT_PENALTY
            else:
                travel = length
            cost = grid.cost(col, row)
            if cost == goal_fit.UNKNOWN:
                cost = 0
            ng = g + travel * (1.0 + cost / 252.0)
            if ng + 1e-12 < g_score.get(nkey, 1e18):
                g_score[nkey] = ng
                came[nkey] = (key, prim)
                pose_of[nkey] = (nx, ny, headings[nbin])
                heapq.heappush(heap, (ng + heuristic(nx, ny), ng, nkey))
    if found is None:
        return None
    chain = []
    node = found
    while node is not None:
        chain.append(node)
        parent = came.get(node)
        node = None if parent is None else parent[0]
    chain.reverse()
    path = [pose_of[chain[0]]]
    for nxt in chain[1:]:
        parent, prim = came[nxt]
        x, y, _ = pose_of[parent]
        for px, py, pyaw in prim["poses"]:
            path.append((x + px, y + py, pyaw))
    return path


def doorway_reproduction(dwb=False):
    """NavFn vs the differential lattice on the same 55-degree metre-wide bend.

    Hybrid-A* Dubins is the other kinematic search in this tree; it is not the
    live plugin. Lattice is, because this chassis can pivot and Hybrid-A*
    cannot write that into a path. The number that decides the doorway is
    still the driving corner against one DWB rollout, not a closed loop on a
    frozen map.
    """
    grid, start, goal = geo.bent_passage()
    navfn = geo.grid_astar(grid, (start[0], start[1]), (goal[0], goal[1]))
    lattice = lattice_astar(grid, start, goal)
    result = {
        "turning_radius_m": geo.MIN_TURNING_RADIUS,
        "lattice_radius_m": load_lattice()[0]["turning_radius"],
        "rollout_m": geo.ROLLOUT_M,
        "rollout_deg": math.degrees(geo.ROLLOUT_RAD),
        "navfn": None if navfn is None else geo.describe_path(navfn, "navfn"),
        "lattice": None if lattice is None else describe_path(lattice, "lattice"),
        "navfn_dwb": None,
        "lattice_dwb": None,
        "navfn_mid": None,
        "lattice_mid": None,
    }
    if not dwb:
        return result
    if navfn:
        pose, stand = geo.approach_pose(navfn, look_ahead=0.25)
        if pose is not None:
            result["navfn"]["approach_bend_deg"] = math.degrees(abs(geo.wrap(
                geo.heading_at(navfn, stand + geo.FIRST_BEND_M) - pose[2])))
            result["navfn_dwb"] = geo.dwb_at(grid, navfn, pose)
            result["navfn_dwb"]["stand_m"] = stand
        result["navfn_mid"] = geo.midcourse_stats(grid, navfn)
    if lattice:
        pose, stand = geo.approach_pose(lattice, look_ahead=0.25)
        if pose is not None:
            result["lattice"]["approach_bend_deg"] = math.degrees(abs(geo.wrap(
                geo.heading_at(lattice, stand + geo.FIRST_BEND_M) - pose[2])))
            result["lattice_dwb"] = geo.dwb_at(grid, lattice, pose)
            result["lattice_dwb"]["stand_m"] = stand
        result["lattice_mid"] = geo.midcourse_stats(grid, lattice)
    return result
