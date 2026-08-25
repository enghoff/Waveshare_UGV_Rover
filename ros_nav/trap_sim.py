#!/usr/bin/env python3
"""Why the rover ends up somewhere it cannot turn round, and what decides that.

    python3 trap_sim.py --turn        # where a pivot does not fit
    python3 trap_sim.py --weights     # what actually decides a tick
    python3 trap_sim.py --look        # sweep the align look-ahead
    python3 trap_sim.py --bias        # is driving or turning being punished?
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
from the worst, critic by critic. The median tick reads:

    ObstacleFootprint        0.02 points
    PathDist                 2.40 points
    PathAlign             2880.80 points

`PathAlign` is not winning by being important. `MapGridCritic::reset` sets
`unreachable_score_` to the cell count plus one -- 3601 on this rover's 60x60
window -- and the align critics *return* that number instead of refusing the
candidate, because both clear `stop_on_failure_` in their `onInit`. At
`resolution * 0.5 * 32` that is a charge of 2881 points for a nose point that
landed where the flood never reached, against about 5 points for the entire
range of obstacle cost. So the controller is not choosing between routes that
are more or less close to a wall. It is choosing whichever candidate happens to
put a point 0.8 m ahead of the nose in a cell the flood reached, and everything
else -- including how close the body passes to the corner -- is rounding.

That is all three complaints at once: in a passage the flood is thin so most
nose points miss it, at a corner the nose sweeps across the wall, and beside a
surface up to 86% of the candidates carry the same 2881-point charge, leaving
the choice between them to be settled below the size of one cell.

**Read the sweep before changing the look-ahead back.** The 0.8 m that was in
the config was chosen against a model that treated an unreachable nose point as
a refusal rather than as a score, so the table that justified it was describing
a controller nobody has run. `--look` re-runs that sweep with the critics doing
what `libdwb_critics.so` actually does, and the answer inverts: driven round
the loop from twelve starts in each of the two recordings, 0.325 m gets out of
24 of 24 and 0.8 m gets out of 5.
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
            spread["ObstacleFootprint"].append(max(obstacle) - min(obstacle))
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


def drive_bias(episode, look=None):
    """Does the unreachable charge fall harder on driving than on pivoting?

    **This is a tuning question and not a law, and the answer flips.** A
    driving rollout ends `sim_time * max_vel_x` -- 0.32 m -- further into the
    room, so the point the align critics judge it on is that much further out;
    a pivot ends where the rover already stands, which is on the path by
    definition. Which of the two ends up over a wall therefore depends on the
    look-ahead and on the geometry in front of the rover, and on the corridor
    recording it reverses between 0.45 and 0.60:

        look     driving   pivoting
        0.200      74%        1%
        0.325      77%       16%
        0.450      74%       65%
        0.600      13%       73%
        0.800       5%       60%

    Whichever side is being punished is the side the rover stops choosing. At
    0.325 in that corridor it was driving, so the rover turned on the spot for
    93% of a minute with clear floor ahead. On the doorway recording the same
    sweep never exceeds eight points either way, so do not read a number off
    one recording and treat it as the rover's character.
    """
    look = dwb.FORWARD_POINT_DISTANCE if look is None else look
    drive_hit = drive_n = pivot_hit = pivot_n = 0
    drive_worse = ticks = 0
    for grid, path, x, y, yaw, vx0, wz0, _osc in recorded_ticks(episode):
        values = dwb.flood(grid, [grid.cell_of(px, py) for px, py in path])
        big = grid.width * grid.height
        d_hit = d_n = p_hit = p_n = 0
        for vx, wz in dwb.twists(vx0, wz0):
            end_x, end_y, end_yaw = dwb.rollout(x, y, yaw, vx, wz, vx0, wz0)[-1]
            nose = dwb.forward_pose(end_x, end_y, end_yaw, look)
            value, _ = dwb.map_grid_score(grid, values, nose[0], nose[1],
                                          "PathAlign", False)
            if value is None:
                continue
            charged = 1 if value >= big else 0
            if abs(vx) > 1e-9:
                d_n += 1
                d_hit += charged
            else:
                p_n += 1
                p_hit += charged
        drive_hit += d_hit
        drive_n += d_n
        pivot_hit += p_hit
        pivot_n += p_n
        if d_n and p_n:
            ticks += 1
            if d_hit / float(d_n) > p_hit / float(p_n):
                drive_worse += 1
    return {"drive": (drive_hit, drive_n), "pivot": (pivot_hit, pivot_n),
            "drive_worse": drive_worse, "ticks": ticks}


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
        for name in ("ObstacleFootprint", "PathDist", "PathAlign"):
            values = sorted(spread[name])
            if values:
                print("   %-20s %9.2f" % (name, values[len(values) // 2]))
        if unreachable:
            print()
            print("   the nose point lands where the flood never reached on a")
            print("   median %.0f%% of the candidates in a tick, worst tick %.0f%%,"
                  % (100 * unreachable[len(unreachable) // 2],
                     100 * unreachable[-1]))
            print("   and every one of those is charged %.0f points."
                  % (dwb.MAP_GRID_RESCALE * dwb.PATH_ALIGN_SCALE * (60 * 60 + 1)))
        print()

    if args.bias or everything:
        bias = drive_bias(episode)
        hit, n = bias["drive"]
        phit, pn = bias["pivot"]
        print("who pays the unreachable charge, driving or turning")
        print("   a candidate that DRIVES forward is charged  %5d of %5d  (%2.0f%%)"
              % (hit, n, 100.0 * hit / max(n, 1)))
        print("   a candidate that TURNS on the spot          %5d of %5d  (%2.0f%%)"
              % (phit, pn, 100.0 * phit / max(pn, 1)))
        print("   driving is the worse-treated of the two on %d of %d ticks (%.0f%%)"
              % (bias["drive_worse"], bias["ticks"],
                 100.0 * bias["drive_worse"] / max(bias["ticks"], 1)))
        print("   -- which is why the rover turns instead of going.")
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
