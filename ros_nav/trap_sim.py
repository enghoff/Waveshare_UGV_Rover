#!/usr/bin/env python3
"""Why the rover ends up somewhere it cannot turn round, and what decides that.

    python3 trap_sim.py --turn        # where a pivot does not fit
    python3 trap_sim.py --weights     # what actually decides a tick
    python3 trap_sim.py --look        # sweep the align look-ahead
    python3 trap_sim.py --bias        # what driving costs more than turning
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--episode", default=EPISODE)
    parser.add_argument("--turn", action="store_true")
    parser.add_argument("--weights", action="store_true")
    parser.add_argument("--look", action="store_true")
    parser.add_argument("--bias", action="store_true")
    parser.add_argument("--looks", default="0.325,0.45,0.60,0.80")
    args = parser.parse_args(argv)
    everything = not (args.turn or args.weights or args.look or args.bias)
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
