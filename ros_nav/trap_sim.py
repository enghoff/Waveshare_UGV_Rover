#!/usr/bin/env python3
"""Why the rover ends up somewhere it cannot turn round, and what decides that.

    python3 trap_sim.py --turn        # where a pivot does not fit
    python3 trap_sim.py --weights     # what actually decides a tick
    python3 trap_sim.py --look        # sweep the align look-ahead
    python3 trap_sim.py --bias        # what driving costs more than turning
    python3 trap_sim.py --rings       # does a smaller body get it out? (no)
    python3 trap_sim.py --aim         # how far along the plan it aims  (yes)
    python3 trap_sim.py               # all four

**The three complaints this exists to explain.** The rover gets stuck turning
in a corridor; it drives too close to a corner and stops there; and once it is
near a surface it will not turn away from it. They are one fault seen from
three angles, and the fault is in the controller rather than the planner.

**The geometry, which is fixed and worth knowing.** The body is a measured
rectangle 0.36 m long and 0.28 m wide, so its *inscribed* radius -- from
`base_link` to the nearest edge -- is 0.14 m, while its *circumscribed* radius,
which is what a turn on the spot sweeps, is 0.244 m. The inflation layer paints
its hard 253 ring at the inscribed radius. So there is a ten-centimetre band
around every wall where the costmap says the rover fits and a pivot does not.
On the map this rover really built, 23% of the poses the body fits in are poses
it cannot turn all the way round in, and 7% fit at four of sixteen headings or
fewer. `--turn` measures that.

**What lets it get there, which is not fixed.** `--weights` scores every
candidate of every recorded tick and reports how many points separate the best
from the worst, critic by critic. On `recordings/trap-2026-08-25-spin.json`,
the drive where the rover turned 3038 degrees and finished 28 cm from where it
started, the median tick reads:

    BaseObstacle             1.38 points
    PathDist                 2.40 points
    PathAlign                6.40 points

**A correction, and it is the largest one this file has carried.** Those three
numbers used to be 0.02, 2.40 and 2880.80, and the third was the whole story:
the align critics were believed to charge `unreachable_score_` -- 2881 points
after scaling -- for a judged point the flood could not reach, which made every
other critic rounding. That charge does not happen. The flood in the
`libdwb_critics.so` installed on this rover is not stopped by walls, so a
critic with a seed on its window reaches every cell of it and there is no
unreachable cell to land on: 0 of 8687 driving candidates and 0 of 6132 pivots
over that recording. `corridor_sim.flood` has the disassembly. What follows
from it is that the four map-grid critics are back to doing the ordinary thing
-- measuring how far a rollout ends from the plan -- and the margins between
candidates are single points rather than thousands.

**So what makes the rover turn instead of drive is two things, and `--bias`
measures both.** On 41% of that drive's ticks not one forward candidate was
legal: the rover stood where every rollout that moved ended on a cell the
obstacle critic refuses, and sixteen ways of turning on the spot were the only
choices it had. On the rest, driving lost to pivoting by a median 4.6 points,
and the bill is `PathAlign` +4.0 and `PathDist` +3.2 -- both saying the rover
would end further from the planned path than it is now -- against `GoalDist`
and `GoalAlign` pulling the other way by 1.8 each.

**A smaller body is not the answer, and `--rings` is how that was settled.**
The obvious reading of the paragraph above is that the 20 cm inscribed ring is
what refuses the forward moves, so a smaller body would free the rover. It does
not. Swept from a 0.20 m circle down to a 0.10 m one, and through the measured
rectangle, on `recordings/trap-2026-08-25-spin.json`:

    body                    got somewhere   forward legal   forward wins
    circle 0.20 (as run)      0 of 12        33 of 52          0
    circle 0.16               0 of 12        37 of 52          0
    rectangle (measured)      0 of 12        32 of 52          0
    circle 0.12               0 of 12        42 of 52          0
    circle 0.10               1 of 12        43 of 52          1

Shrinking the body makes a forward move *legal* far more often -- 33 ticks of 52
becomes 43 -- and the rover still never chooses one, because the margin against
it only falls from 4.3 points to 3.4. The single escape at 0.10 m takes the
rover's centre within 0.16 m of a cell the lidar saw something in, and the real
body is 0.14 m to its nearest edge and 0.24 m to a corner: that escape is a
collision. Two other candidates were killed the same way. Removing
`PreferForward`'s rate charge makes the rover turn faster and further -- 604
degrees against 422 -- and it escapes 0 of 12. Turning it to the plan's heading
first, which is what a `Spin` recovery would do, escapes 1 of 12 and ends 97
degrees off the plan again, because DWB turns back out of the alignment as soon
as it is handed control.

**What is actually holding it is where the goal field points.** `GoalDist` and
`GoalAlign` flood from the last plan point on the window, and this build's flood
runs through walls, so the direction they reward is the straight-line direction
to a goal that is on the far side of one. Measured at the rover's own poses,
with the same seed flooded both ways, the best nose bearing under the library's
flood and under the wall-respecting flood upstream intends are 75 and 120
degrees apart -- and on many ticks the seed sits on an inscribed cell where the
upstream flood would have no answer at all. So the controller is aiming the
rover at a wall, every forward move that way is refused, and the pivots it is
left with are separated by 0.4 points of aiming signal against 3.4 points of
turn-rate charge. It turns for ever.

**What does get it out is cutting the plan by driving distance, and `--aim`
is where that was found.** DWB keeps plan points while they are within half
the local costmap of the rover. That is a *radius*, and where the route turns
a corner the plan doubles back inside it, so the piece kept can be far longer
than the radius: on this drive it is a median 3.34 m of driving whose far end
is only 1.48 m away as the crow flies. The end of that piece is the goal every
goal critic floods from, and on 34 of 52 ticks it is a cell the rover's centre
may not occupy -- around the corner, behind a wall. The flood in this build
runs through walls, so the direction the critics reward is the straight line
to a point on the far side of one.

Cut the same plan at a distance of *driving* instead and the seed stops being
a wall, on a threshold that is sharp:

    plan cut at            the seed is a wall   got somewhere   moved
    wherever the costmap    34 of 52 ticks        0 of 12       0.00 m
    ends (as run)
    2.0 m of driving         5 of 52             11 of 12       1.15 m
    1.5 m of driving         2 of 52             11 of 12       1.15 m
    1.0 m of driving        13 of 52              8 of 12       1.13 m
    0.4 m of driving         0 of 52              2 of 12       0.12 m

It falls away again at the short end, where the seed lands inside the rover's
own inflated ring instead. The escapes clear a real lidar return by 0.20 to
0.23 m against a body whose corner is 0.24 m out, so this gets the rover
moving without getting it through the gap cleanly -- centring in the gap is a
separate question from being aimed at somewhere reachable.

**This is a change and not a missing piece of the model.** `dwb_core` does
have a forward-prune parameter, and modelling one as always present is the
obvious explanation for the numbers above -- but it is wrong: with the cut
modelled at DWB's 2.0 m default the model's agreement with the recorded drive
falls from 84% to 47%, so the controller the rover actually ran was not
applying one. `dwb_replay.FORWARD_AIM_M` therefore stays `None`.

**The look-ahead sweep is a measurement of the model that was, not the rover.**
`--look` chose 0.325 over 0.8 by counting nose points that missed the flood,
and that count is now structurally zero, so the sweep no longer says anything
about the look-ahead. Nothing in the config has been changed back on the
strength of that: what it means is that the reason recorded for 0.325 is not a
reason any more, and a fresh case has to be made from a drive.
"""

from __future__ import annotations

import argparse
import base64
import collections
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import corridor_sim as dwb
import dwb_replay
import goal_fit

#: The default episode, which is the drive the complaints were recorded on.
EPISODE = os.path.join(HERE, "recordings", "doorway-2026-08-25-after-floor.json")

#: Sixteen headings, because that is what the lattice control set has and so
#: what a planned turn on the spot is actually made of.
HEADINGS = [i * 2.0 * math.pi / 16 for i in range(16)]

#: What a pivot sweeps. Not the inscribed radius the inflation layer uses, and
#: the gap between the two is the whole subject of `--turn`.
CIRCUMSCRIBED_M = max(math.hypot(x, y) for x, y in dwb.FOOTPRINT)


def global_grid(episode):
    """The map the planner was working from, as a `goal_fit.CostGrid`."""
    snapshot = episode["global_costmap"]
    return goal_fit.CostGrid(
        snapshot["width"], snapshot["height"], snapshot["resolution"],
        snapshot["origin"][0], snapshot["origin"][1],
        list(base64.b64decode(snapshot["data"])))


def hits_obstacle(grid, x, y, yaw):
    """The rule the planner really uses, which is 254 and not 253.

    Read out of `libnav2_smac_planner_lattice.so`, where
    `GridCollisionChecker::inCollision` compares the oriented footprint's cost
    against 0x437e0000 -- 254.0f -- with `cset ge`, having already compared it
    against 255.0f for unknown. The 253 ring stops the *centre* through the
    inflation fast path; it does not by itself condemn a footprint that merely
    overlaps it.
    """
    for col, row in goal_fit.covered(grid, dwb.FOOTPRINT, x, y, yaw):
        if grid.cost(col, row) == goal_fit.LETHAL:
            return True
    return False


def turn_survey(episode):
    """Every pose the body fits in, and whether it can turn round there."""
    grid = global_grid(episode)
    fits_somewhere = can_turn = wedged = 0
    partial = []
    for row in range(grid.height):
        for col in range(grid.width):
            cost = grid.cost(col, row)
            if cost in (goal_fit.UNKNOWN, goal_fit.LETHAL):
                continue
            x = grid.origin_x + (col + 0.5) * grid.resolution
            y = grid.origin_y + (row + 0.5) * grid.resolution
            ok = sum(1 for yaw in HEADINGS
                     if not hits_obstacle(grid, x, y, yaw))
            if not ok:
                continue
            fits_somewhere += 1
            if ok == len(HEADINGS):
                can_turn += 1
            else:
                partial.append(ok)
                if ok <= 4:
                    wedged += 1
    partial.sort()
    return {"fits": fits_somewhere, "turns": can_turn, "wedged": wedged,
            "median_headings": partial[len(partial) // 2] if partial else 0}


def recorded_ticks(episode):
    """Each tick as the inputs the controller had, with the latch carried along.

    The oscillation latch is stateful and is wiped by every replan, so it has to
    be walked forward in recorded order rather than rebuilt per tick.
    """
    grids = {}
    oscillation = dwb.Oscillation()
    last_plan = None
    last_vx = last_wz = 0.0
    out = []
    for row in dwb_replay.ticks(episode):
        if last_plan is None or row["plan"]["t"] != last_plan:
            oscillation.reset()
            last_plan = row["plan"]["t"]
        key = row["costmap"]["t"]
        if key not in grids:
            grids.clear()
            grids[key] = dwb_replay.grid_of(row["costmap"])
        grid = grids[key]
        x, y, yaw = row["pose"]["odom"]
        path = dwb_replay.transform_plan(
            dwb_replay.plan_in_odom(row["plan"], row["pose"]["map"],
                                    row["pose"]["odom"]), grid, x, y)
        if len(path) < 2:
            continue
        out.append((grid, path, x, y, yaw, last_vx, last_wz, oscillation))
        oscillation.debrief(x, y, yaw, row["command"]["vx"],
                            row["command"]["wz"])
        last_vx, last_wz = row["command"]["vx"], row["command"]["wz"]
    return out


def clearance(grid, x, y, cap=0.60):
    """Metres from a point to the nearest lethal cell, given up on past `cap`."""
    best = cap
    col0, row0 = grid.cell_of(x, y)
    span = int(cap / grid.resolution) + 1
    for drow in range(-span, span + 1):
        for dcol in range(-span, span + 1):
            if grid.cost(col0 + dcol, row0 + drow) == goal_fit.LETHAL:
                gap = math.hypot(dcol, drow) * grid.resolution
                if gap < best:
                    best = gap
    return best


def weights(episode):
    """How many points each critic can move the answer by, within one tick."""
    spread = collections.defaultdict(list)
    unreachable = []
    for grid, path, x, y, yaw, vx0, wz0, _osc in recorded_ticks(episode):
        values = dwb.flood(grid, [grid.cell_of(px, py) for px, py in path])
        big = grid.width * grid.height
        obstacle, align, near = [], [], []
        missed = seen = 0
        for vx, wz in dwb.twists(vx0, wz0):
            poses = dwb.rollout(x, y, yaw, vx, wz, vx0, wz0)
            end_x, end_y, end_yaw = poses[-1]
            # Whichever obstacle critic the configured body takes, so this
            # table names the one the rover is running rather than the one it
            # used to run.
            if dwb.CIRCULAR and dwb.CIRCULAR_USES_BASE_OBSTACLE:
                cost, reason = dwb.base_obstacle(grid, poses)
            else:
                cost, reason = dwb.obstacle_footprint(grid, poses)
            if reason:
                continue
            obstacle.append(dwb.OBSTACLE_RESCALE * dwb.OBSTACLE_SCALE * cost)
            nose = dwb.forward_pose(end_x, end_y, end_yaw,
                                    dwb.FORWARD_POINT_DISTANCE)
            value, _ = dwb.map_grid_score(grid, values, nose[0], nose[1],
                                          "PathAlign", False)
            if value is None:
                continue
            seen += 1
            if value >= big:
                missed += 1
            align.append(dwb.MAP_GRID_RESCALE * dwb.PATH_ALIGN_SCALE * value)
            close, _ = dwb.map_grid_score(grid, values, end_x, end_y,
                                          "PathDist", False)
            if close is not None:
                near.append(dwb.MAP_GRID_RESCALE * dwb.PATH_DIST_SCALE * close)
        if align and obstacle:
            spread["BaseObstacle" if dwb.CIRCULAR and dwb.CIRCULAR_USES_BASE_OBSTACLE
                   else "ObstacleFootprint"].append(max(obstacle) - min(obstacle))
            spread["PathAlign"].append(max(align) - min(align))
            spread["PathDist"].append(max(near) - min(near) if near else 0.0)
            unreachable.append(missed / float(seen or 1))
    return spread, sorted(unreachable)


def look_sweep(episode, looks):
    """Clearance and nose-point misses, as the align look-ahead changes."""
    rows = recorded_ticks(episode)
    table = []
    for look in looks:
        missed_share, clear, distinct = [], [], []
        too_close = ticks = 0
        for grid, path, x, y, yaw, vx0, wz0, osc in rows:
            values = dwb.flood(grid, [grid.cell_of(px, py) for px, py in path])
            big = grid.width * grid.height
            missed = seen = 0
            pivots = set()
            for vx, wz in dwb.twists(vx0, wz0):
                poses = dwb.rollout(x, y, yaw, vx, wz, vx0, wz0)
                end_x, end_y, end_yaw = poses[-1]
                nose = dwb.forward_pose(end_x, end_y, end_yaw, look)
                value, _ = dwb.map_grid_score(grid, values, nose[0], nose[1],
                                              "PathAlign", False)
                if value is None:
                    continue
                seen += 1
                if value >= big:
                    missed += 1
                if abs(vx) < 1e-9:
                    pivots.add(round(value, 3))
            if seen:
                missed_share.append(missed / float(seen))
            distinct.append(len(pivots))
            kept, _ = dwb.evaluate(grid, path, path[-1], x, y, yaw, vx_now=vx0,
                                   wz_now=wz0, oscillation=osc,
                                   path_look=look, goal_look=look)
            if kept:
                ticks += 1
                end = dwb.rollout(x, y, yaw, kept[0][1], kept[0][2], vx0, wz0)[-1]
                gap = clearance(grid, end[0], end[1])
                clear.append(gap)
                if gap < CIRCUMSCRIBED_M:
                    too_close += 1
        missed_share.sort()
        clear.sort()
        distinct.sort()
        table.append({
            "look": look,
            "missed": missed_share[len(missed_share) // 2] if missed_share else 0.0,
            "clearance": clear[len(clear) // 2] if clear else 0.0,
            "distinct": distinct[len(distinct) // 2] if distinct else 0,
            "too_close": too_close, "ticks": ticks})
    return table


#: The critics whose difference between driving and turning is worth naming,
#: in the order `config/nav2.yaml` lists them.
BIAS_CRITICS = ("RotateToGoal", "BaseObstacle", "GoalAlign", "PathAlign",
                "PathDist", "GoalDist", "PreferForward")


def score_parts(grid, path, x, y, yaw, vx, wz, vx0, wz0, path_values,
                goal_values, look):
    """Every critic's contribution to one candidate, or None if it is refused.

    The same arithmetic `corridor_sim.evaluate` does, kept apart from it
    because that one returns a total and the question here is which critic the
    total came from.
    """
    poses = dwb.rollout(x, y, yaw, vx, wz, vx0, wz0)
    end_x, end_y, end_yaw = poses[-1]
    turn, reason = dwb.rotate_to_goal(x, y, yaw, path[-1], vx, wz, end_yaw,
                                      None)
    if reason:
        return None
    if dwb.CIRCULAR and dwb.CIRCULAR_USES_BASE_OBSTACLE:
        obstacle, reason = dwb.base_obstacle(grid, poses)
    else:
        obstacle, reason = dwb.obstacle_footprint(grid, poses)
    if reason:
        return None
    parts = {
        "RotateToGoal": dwb.ROTATE_TO_GOAL_SCALE * turn,
        "BaseObstacle": dwb.OBSTACLE_RESCALE * dwb.OBSTACLE_SCALE * obstacle,
        "PreferForward": dwb.PREFER_FORWARD_SCALE * dwb.prefer_forward(vx, wz),
    }
    near_goal = math.hypot(path[-1][0] - x, path[-1][1] - y) <= look
    for name, values, scale, stops in (
            ("GoalAlign", goal_values, dwb.GOAL_ALIGN_SCALE, False),
            ("PathAlign", path_values,
             0.0 if near_goal else dwb.PATH_ALIGN_SCALE, False),
            ("PathDist", path_values, dwb.PATH_DIST_SCALE, True),
            ("GoalDist", goal_values, dwb.GOAL_DIST_SCALE, True)):
        if stops:
            value = 0.0
            for px, py, _pyaw in poses:
                value, reason = dwb.map_grid_score(grid, values, px, py, name,
                                                   True)
                if reason:
                    return None
        else:
            point = dwb.forward_pose(end_x, end_y, end_yaw, look)
            value, reason = dwb.map_grid_score(grid, values, point[0],
                                               point[1], name, False)
            if reason:
                return None
        parts[name] = dwb.MAP_GRID_RESCALE * scale * value
    return parts


def drive_bias(episode, look=None):
    """What does going forward cost more than turning on the spot, and who charges it?

    **This used to count a charge that cannot happen, and the count is now
    always zero.** The four map-grid critics were believed to charge
    `unreachable_score_` -- 2881 points after scaling -- for a judged point the
    flood could not reach, and a sweep of that charge across the align
    look-ahead is what chose 0.325 over 0.8. The flood in the
    `libdwb_critics.so` this rover runs is not stopped by walls at all (see
    `corridor_sim.flood`), so once a critic has a seed on the window there is
    no unreachable cell to land on: measured over
    `recordings/trap-2026-08-25-spin.json`, 0 of 8687 driving candidates and 0
    of 6132 pivots. The charge survives only for a critic given no seed at all,
    and then it lands on every candidate of the tick equally and decides
    nothing.

    So the question is asked directly instead. For every tick where both a
    forward candidate and a pivot were legal, this takes the best of each by
    the full objective and reports the difference critic by critic. A positive
    number is a critic that prefers the rover stayed where it is.
    """
    look = dwb.FORWARD_POINT_DISTANCE if look is None else look
    gaps = collections.defaultdict(list)
    no_forward = ticks = 0
    for grid, path, x, y, yaw, vx0, wz0, _osc in recorded_ticks(episode):
        path_values = dwb.flood(grid, [grid.cell_of(px, py) for px, py in path])
        seed = dwb.last_pose_on_costmap(grid, path)
        goal_values = dwb.flood(grid, [seed] if seed else [])
        best = {}
        for vx, wz in dwb.twists(vx0, wz0):
            parts = score_parts(grid, path, x, y, yaw, vx, wz, vx0, wz0,
                                path_values, goal_values, look)
            if parts is None:
                continue
            kind = "forward" if abs(vx) > 1e-9 else "pivot"
            total = sum(parts.values())
            if kind not in best or total < best[kind][0]:
                best[kind] = (total, parts)
        ticks += 1
        if "forward" not in best:
            no_forward += 1
        if "forward" in best and "pivot" in best:
            for name in BIAS_CRITICS:
                gaps[name].append(best["forward"][1].get(name, 0.0)
                                  - best["pivot"][1].get(name, 0.0))
            gaps["TOTAL"].append(best["forward"][0] - best["pivot"][0])
    return {"gaps": gaps, "ticks": ticks, "no_forward": no_forward,
            "compared": len(gaps["TOTAL"])}


#: The body as `lidar_slam/slam2d.c` masks it, which is the rectangle the rover
#: was described by before it went back to a circle.
MEASURED_RECT = [(0.20, 0.14), (0.20, -0.14), (-0.16, -0.14), (-0.16, 0.14)]

#: What to sweep. The two ends are deliberately absurd -- a 0.10 m rover is not
#: this rover -- because the point of the sweep is to show what shape *cannot*
#: buy, and a range that stops at plausible bodies cannot show that.
#: How far along the plan the controller is allowed to aim, in metres of
#: driving. `None` is the rover as it stands, which aims at whatever the
#: costmap radius happens to leave -- a median 3.34 m of plan on this drive.
AIM_LIMITS = (None, 2.0, 1.5, 1.2, 1.0, 0.8, 0.6, 0.4)

SHAPES = (("circle 0.20 (as run)", 0.200),
          ("circle 0.175", 0.175),
          ("circle 0.16", 0.160),
          ("rectangle (measured)", None),
          ("circle 0.12", 0.120),
          ("circle 0.10", 0.100))


def set_shape(radius):
    """Describe the rover to the model as a circle, or as the rectangle.

    Everything downstream of the shape has to move with it: which obstacle
    critic is correct, what `getScale` does to it, and the inscribed radius the
    inflation layer paints its 253 ring at. Changing the radius alone would
    score a circular body against a rectangle's ring.
    """
    if radius is None:
        dwb.CIRCULAR = False
        dwb.FOOTPRINT = list(MEASURED_RECT)
        dwb.INSCRIBED_M = 0.14
        dwb.ROBOT_RADIUS_M = 0.14
        dwb.OBSTACLE_RESCALE = dwb.RESOLUTION
    else:
        dwb.CIRCULAR = True
        dwb.ROBOT_RADIUS_M = radius
        dwb.INSCRIBED_M = radius
        dwb.FOOTPRINT = [(radius * math.cos(i * math.pi / 6.0),
                          radius * math.sin(i * math.pi / 6.0))
                         for i in range(12)]
        dwb.OBSTACLE_RESCALE = 1.0
    dwb.CIRCUMSCRIBED_M = max(math.hypot(px, py) for px, py in dwb.FOOTPRINT)


def ring_sweep(episode, starts=12, seconds=12.0, every=10):
    """Does a smaller body get the rover out, and what does the escape cost?

    Two measurements, because either alone misleads.

    **Driven**, from `starts` poses of the recording, with the costmap
    re-inflated at each shape: how often the model gets anywhere, and how close
    its centre passed to a cell the lidar really saw something in. That second
    number is the one that stops the sweep being a machine for choosing an
    absurd body -- the model knows only what the costmap forbids, so shrinking
    the rover always escapes more, and an escape that took less clearance than
    the rover's real half-width is a collision rather than a fix.

    **Standing still**, at the rover's own recorded poses: whether a forward
    move is legal at all, and what it costs against the best pivot. This is the
    apples-to-apples half. The driven runs diverge into different poses under
    each shape, so their legality counts are not comparable across rows; these
    are.
    """
    rows = dwb_replay.ticks(episode)
    if not rows:
        return None
    picks = [int(i * (len(rows) - 1) / float(starts - 1)) for i in range(starts)]
    out = []
    for label, radius in SHAPES:
        set_shape(radius)
        dwb_replay._REINFLATED.clear()
        runs = [dwb_replay.closed_loop(episode, dwb.TURN_GAIN, dwb.DEAD_TIME_S,
                                       seconds=seconds, start=start,
                                       reinflate=True)
                for start in picks]
        runs = [run for run in runs if run]
        cache = {}
        legal = wins = ticks = 0
        margins = []
        for row in rows[::every]:
            key = row["costmap"]["t"]
            if key not in cache:
                cache.clear()
                cache[key] = dwb.reinflate(dwb_replay.grid_of(row["costmap"]))
            grid = cache[key]
            x, y, yaw = row["pose"]["odom"]
            path = dwb_replay.transform_plan(
                dwb_replay.plan_in_odom(row["plan"], row["pose"]["map"],
                                        row["pose"]["odom"]), grid, x, y)
            if len(path) < 2:
                continue
            ticks += 1
            kept, _ = dwb.evaluate(grid, path, path[-1], x, y, yaw)
            forward = [k for k in kept if k[1] > 1e-6]
            pivot = [k for k in kept if abs(k[1]) < 1e-6]
            if forward:
                legal += 1
            if kept and kept[0][1] > 1e-6:
                wins += 1
            if forward and pivot:
                margins.append(forward[0][0] - pivot[0][0])
        margins.sort()
        nets = sorted(run["net"] for run in runs)
        clears = sorted(run["clearance"] for run in runs)
        out.append({
            "label": label,
            "escaped": sum(1 for run in runs if not run["stuck"]),
            "runs": len(runs),
            "net": nets[len(nets) // 2],
            "clearance": clears[0],
            "legal": legal, "ticks": ticks, "wins": wins,
            "margin": margins[len(margins) // 2] if margins else float("nan"),
        })
    set_shape(dwb.ROBOT_RADIUS_CONFIGURED if dwb.CIRCULAR else None)
    return out


def aim_sweep(episode, starts=12, seconds=12.0, every=10):
    """How far along the plan may the controller aim, and does shortening it help?

    DWB keeps the plan while it is within half the local costmap of the rover,
    which is a *radius*. Where the route turns a corner the plan doubles back
    inside that radius, so the piece kept can be much longer than the radius
    itself, and its far end -- which is the goal every goal critic flods from
    -- ends up around the corner and behind a wall. Cutting the same plan at a
    given distance of *driving* is the change being priced here.

    `escaped` is measured the way `ring_sweep` measures it and carries the same
    warning: read `clearance` beside it. Unlike the body sweep the shape is not
    being changed, so an escape here is not bought by describing a smaller
    rover, but the model still knows nothing about hitting anything.
    """
    rows = dwb_replay.ticks(episode)
    if not rows:
        return None
    picks = [int(i * (len(rows) - 1) / float(starts - 1)) for i in range(starts)]
    out = []
    for limit in AIM_LIMITS:
        runs = [dwb_replay.closed_loop(episode, dwb.TURN_GAIN, dwb.DEAD_TIME_S,
                                       seconds=seconds, start=start,
                                       forward_aim=limit)
                for start in picks]
        runs = [run for run in runs if run]
        ticks = blocked = 0
        aims = []
        margins = []
        for row in rows[::every]:
            grid = dwb_replay.grid_of(row["costmap"])
            x, y, yaw = row["pose"]["odom"]
            path = dwb_replay.transform_plan(
                dwb_replay.plan_in_odom(row["plan"], row["pose"]["map"],
                                        row["pose"]["odom"]), grid, x, y, limit)
            if len(path) < 2:
                continue
            ticks += 1
            aims.append(math.hypot(path[-1][0] - x, path[-1][1] - y))
            col, row_i = grid.cell_of(path[-1][0], path[-1][1])
            if (0 <= col < grid.width and 0 <= row_i < grid.height
                    and grid.cost(col, row_i) >= goal_fit.INSCRIBED):
                blocked += 1
            kept, _ = dwb.evaluate(grid, path, path[-1], x, y, yaw)
            forward = [k for k in kept if k[1] > 1e-6]
            pivot = [k for k in kept if abs(k[1]) < 1e-6]
            if forward and pivot:
                margins.append(forward[0][0] - pivot[0][0])
        margins.sort()
        nets = sorted(run["net"] for run in runs)
        clears = sorted(run["clearance"] for run in runs)
        aims.sort()
        out.append({
            "limit": limit,
            "escaped": sum(1 for run in runs if not run["stuck"]),
            "runs": len(runs),
            "net": nets[len(nets) // 2],
            "clearance": clears[0],
            "typical": clears[len(clears) // 2],
            "margin": margins[len(margins) // 2] if margins else float("nan"),
            "ticks": ticks, "blocked": blocked,
            "aim": aims[len(aims) // 2] if aims else float("nan"),
        })
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--episode", default=EPISODE)
    parser.add_argument("--turn", action="store_true")
    parser.add_argument("--weights", action="store_true")
    parser.add_argument("--look", action="store_true")
    parser.add_argument("--bias", action="store_true")
    parser.add_argument("--aim", action="store_true",
                        help="sweep how far along the plan the controller aims")
    parser.add_argument("--rings", action="store_true",
                        help="sweep the body the controller is told it has")
    parser.add_argument("--looks", default="0.325,0.45,0.60,0.80")
    args = parser.parse_args(argv)
    everything = not (args.turn or args.weights or args.look
                      or args.bias or args.rings or args.aim)
    episode = dwb_replay.load(args.episode)

    if args.turn or everything:
        print("where a turn on the spot does not fit")
        print("   the inflation layer paints its hard ring at the inscribed")
        print("   radius, %.2f m, but a pivot sweeps the circumscribed one,"
              % dwb.INSCRIBED_M)
        print("   %.3f m -- a circle %.2f m across. So there is a %.0f cm band"
              % (CIRCUMSCRIBED_M, 2 * CIRCUMSCRIBED_M,
                 100 * (CIRCUMSCRIBED_M - dwb.INSCRIBED_M)))
        print("   around every wall where standing is legal and turning is not.")
        survey = turn_survey(episode)
        fits = survey["fits"] or 1
        print()
        print("   on the map this rover really built:")
        print("      %5d poses the body fits in at some heading" % survey["fits"])
        print("      %5d it can turn the whole way round in   (%4.1f%%)"
              % (survey["turns"], 100.0 * survey["turns"] / fits))
        print("      %5d it cannot                            (%4.1f%%)"
              % (survey["fits"] - survey["turns"],
                 100.0 * (survey["fits"] - survey["turns"]) / fits))
        print("      %5d fit at 4 headings of 16 or fewer     (%4.1f%%)"
              % (survey["wedged"], 100.0 * survey["wedged"] / fits))
        print()

    if args.weights or everything:
        spread, unreachable = weights(episode)
        print("what decides a tick, in points between the best and worst candidate")
        obstacle_critic = ("BaseObstacle"
                           if dwb.CIRCULAR and dwb.CIRCULAR_USES_BASE_OBSTACLE
                           else "ObstacleFootprint")
        for name in (obstacle_critic, "PathDist", "PathAlign"):
            values = sorted(spread[name])
            if values:
                print("   %-20s %9.2f" % (name, values[len(values) // 2]))
        if unreachable:
            print()
            worst = 100 * unreachable[-1]
            if worst < 0.5:
                print("   no candidate on any tick was charged the %.0f-point "
                      "unreachable score:" % (dwb.MAP_GRID_RESCALE
                                              * dwb.PATH_ALIGN_SCALE
                                              * (60 * 60 + 1)))
                print("   this build's flood is not stopped by walls, so a "
                      "critic with a seed reaches")
                print("   every cell of its window. See corridor_sim.flood.")
            else:
                print("   the nose point lands where the flood never reached on a")
                print("   median %.0f%% of the candidates in a tick, worst tick %.0f%%,"
                      % (100 * unreachable[len(unreachable) // 2], worst))
                print("   and every one of those is charged %.0f points."
                      % (dwb.MAP_GRID_RESCALE * dwb.PATH_ALIGN_SCALE
                         * (60 * 60 + 1)))
        print()

    if args.bias or everything:
        bias = drive_bias(episode)
        print("why the rover turns instead of going")
        print("   on %d of %d ticks not one forward candidate was legal at all "
              "(%.0f%%):" % (bias["no_forward"], bias["ticks"],
                             100.0 * bias["no_forward"] / max(bias["ticks"], 1)))
        print("   every rollout that moved ended on a cell the obstacle critic "
              "refuses, so")
        print("   the only thing left to choose between was sixteen ways of "
              "turning on the spot.")
        if bias["compared"]:
            print()
            print("   on the other %d, what the best forward candidate cost "
                  "MORE than the best pivot:" % bias["compared"])
            rows = []
            for name in ("TOTAL",) + BIAS_CRITICS:
                values = sorted(bias["gaps"][name])
                if not values:
                    continue
                rows.append((name, values[len(values) // 2],
                             values[len(values) // 10],
                             values[9 * len(values) // 10]))
            head = rows[:1]
            rest = sorted(rows[1:], key=lambda row: -abs(row[1]))
            for name, mid, low, high in head + rest:
                print("      %-14s %+7.2f points   (a tenth of ticks below "
                      "%+.2f, a tenth above %+.2f)" % (name, mid, low, high))
            print("   positive is a critic that would rather the rover stayed "
                  "where it is.")
        print()

    if args.aim:
        table = aim_sweep(episode)
        print("how far along the plan the controller is allowed to aim")
        print("   %-26s %-10s %-13s %-13s %-14s %s"
              % ("cut the plan at", "aim point", "it is a wall",
                 "got somewhere", "and moved", "clearance"))
        for row in table:
            label = ("wherever the costmap ends" if row["limit"] is None
                     else "%.1f m of driving" % row["limit"])
            print("   %-26s %5.2f m    %2d of %2d      %2d of %2d       %5.2f m       "
                  "%.2f m worst, %.2f m typical"
                  % (label, row["aim"], row["blocked"], row["ticks"],
                     row["escaped"], row["runs"], row["net"],
                     row["clearance"], row["typical"]))
        print("   'aim point' is how far the thing it steers at ends up, as the crow")
        print("   flies; 'it is a wall' counts the ticks where that point is a cell the")
        print("   rover's centre may not occupy, which is what the goal critics then")
        print("   flood from, and it is the column that carries the mechanism.")
        print("   'and moved' is the median straight-line distance the model got from")
        print("   where it started in twelve seconds. 'clearance' is the rover's centre")
        print("   against a real lidar return, and its own corner is 0.24 m out, so")
        print("   these escapes clip it: they get the rover moving, not through cleanly.")
        print()

    if args.rings:
        table = ring_sweep(episode)
        print("the body the controller is told it has, swept against this drive")
        print("   %-22s %-14s %-8s %-26s %s"
              % ("", "driven:", "", "standing at the rover's own poses:", ""))
        print("   %-22s %-14s %-8s %-14s %-11s %s"
              % ("body", "got somewhere", "closest", "forward legal",
                 "forward wins", "what forward costs"))
        for row in table:
            print("   %-22s %2d of %2d       %.2f m   %2d of %2d       %2d          %+.2f points"
                  % (row["label"], row["escaped"], row["runs"], row["clearance"],
                     row["legal"], row["ticks"], row["wins"], row["margin"]))
        print("   'closest' is how near the rover's centre passed to a cell the lidar")
        print("   saw something in. The real body is 0.14 m to its nearest edge and")
        print("   0.24 m to a corner, so an escape under that is a collision, not a fix.")
        print()

    if args.look or everything:
        looks = [float(v) for v in args.looks.split(",")]
        print("the align look-ahead, swept on the recorded drive")
        print("   look   nose missed   pivots told apart   clearance   "
              "ends too close to turn")
        for row in look_sweep(episode, looks):
            print("   %.3f     %4.0f%%       %2d of 12 values     %.2f m      "
                  "%3d of %d (%2.0f%%)"
                  % (row["look"], 100 * row["missed"], row["distinct"],
                     row["clearance"], row["too_close"], row["ticks"],
                     100.0 * row["too_close"] / max(row["ticks"], 1)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
