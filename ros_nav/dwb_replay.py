#!/usr/bin/env python3
"""Re-score a recorded drive offline, and check the model against the rover.

    python3 dwb_replay.py episode.json            # does the model match DWB?
    python3 dwb_replay.py episode.json --drive    # then: would a change help?

**The check comes first, and it is the only reason to trust anything here.**
`nav_record.py` saved what the controller was given (the plan, the local
costmap, the pose) and what it decided (`/cmd_vel_nav`), tick by tick. This
re-runs the decision from the same inputs and compares. Three previous attempts
at this rover's "drives but goes nowhere" fault were built on models nobody had
ever checked this way, and each reproduced something the rover was not doing.

So `--drive` refuses to run until the agreement is good enough to mean
something. An honest simulation of a controller is one that picks what the
controller picked.

The critic model, the sample set, the costmap inflation thresholds and the
footprint check all come from corridor_sim.py, which took them from the rover's
own libraries rather than from the documentation.
"""

from __future__ import annotations

import argparse
import base64
import collections
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import corridor_sim as dwb
import goal_fit

#: How close a replayed command has to be to count as the same candidate. DWB's
#: sample set is discrete, so the question is whether the model lands on the
#: same one of the thirty-three, not whether it agrees to three decimals.
VX_TOLERANCE = 0.05
WZ_TOLERANCE = 0.10

#: **The test that matters, and it is not "did the model pick the same twist".**
#:
#: Measured on this rover, the thirty-three candidates in a tick score within
#: about 4.9 of each other out of roughly 45, and the best handful are within a
#: quarter of a point. That is not a defect in the model, it is what this
#: objective is: the four map-grid critics score integer counts of costmap cells
#: at 0.6 and 0.8 a cell, and a rotation on the spot does not move the rover out
#: of its own cell, so all sixteen pivots get *identical* path and goal
#: distances. The only thing separating them is which cell a point 10 cm ahead
#: of the nose falls in, and that sweeps about two cells across the whole turn
#: range.
#:
#: So which candidate comes top is settled below the resolution of the costmap
#: being scored, and asking a model to reproduce that is asking it to agree to
#: finer than one cell. Whether the rover's own choice was near-optimal *by the
#: model's own reckoning* is a fair question, and this is the tolerance for it:
#: less than half a cell of GoalDist, so it cannot hide a whole cell of
#: disagreement.
SCORE_TOLERANCE = 0.25

#: How much of a drive has to be near-optimal before a fix may be tested on it.
AGREEMENT_GATE = 70.0

#: What the rover was running before anybody went looking, for recordings made
#: before `nav_record.py` learned to save the settings alongside the drive.
#: Every recording made since carries its own and this is not consulted.
LEGACY_LOOK_AHEAD = 0.1


def settings_of(episode):
    """The look-ahead this drive was actually made under, not today's.

    The check in this file only means anything if the model is scoring the way
    the controller scored *at the time*. Replaying an old drive with a new
    setting compares a rover against a controller it never ran, and the
    agreement collapses for a reason that has nothing to do with either being
    wrong.
    """
    params = episode.get("params") or {}
    path_look = params.get("FollowPath.PathAlign.forward_point_distance")
    goal_look = params.get("FollowPath.GoalAlign.forward_point_distance")
    if path_look is None:
        path_look = LEGACY_LOOK_AHEAD
    if goal_look is None:
        goal_look = LEGACY_LOOK_AHEAD
    return path_look, goal_look, bool(params)


def turning_of(poses, floor=0.01):
    """Heading actually turned, with the noise floor taken out.

    Summing every |delta yaw| looks right and is not: at 10 Hz over five
    minutes it adds up three thousand samples of gyro noise and reports
    thirty-four degrees of turning for a rover that never moved. That would
    quietly break the one test that says whether a recording is worth
    analysing. Anything under half a degree between samples is not a turn.
    """
    total = 0.0
    for a, b in zip(poses, poses[1:]):
        step = abs(math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2])))
        if step >= floor:
            total += step
    return total


def load(path):
    with open(path) as handle:
        return json.load(handle)


def grid_of(snapshot):
    raw = base64.b64decode(snapshot["data"])
    return goal_fit.CostGrid(snapshot["width"], snapshot["height"],
                             snapshot["resolution"], snapshot["origin"][0],
                             snapshot["origin"][1], list(raw))


def nearest(rows, when, key=None):
    """The recorded sample in force at a moment: the last one at or before it."""
    best = None
    for row in rows:
        if row["t"] > when:
            break
        if key is None or key in row:
            best = row
    return best


def plan_in_odom(plan, pose_map, pose_odom):
    """The plan is recorded in `map`; the costmap and the rover are in `odom`.

    Rather than carry a transform, the offset between the two recorded poses at
    the same instant *is* the transform, to within the drift that is the point
    of keeping them apart. A rotation as well as a translation, because the two
    frames differ by a yaw whenever the scan matcher has corrected a turn.
    """
    dx = pose_odom[0] - pose_map[0]
    dy = pose_odom[1] - pose_map[1]
    dyaw = pose_odom[2] - pose_map[2]
    cos_d, sin_d = math.cos(dyaw), math.sin(dyaw)
    out = []
    for px, py in ((p[0], p[1]) for p in plan["poses"]):
        rx, ry = px - pose_map[0], py - pose_map[1]
        out.append((pose_map[0] + dx + rx * cos_d - ry * sin_d,
                    pose_map[1] + dy + rx * sin_d + ry * cos_d))
    return out


def transform_plan(path, grid, x, y):
    """`DWBLocalPlanner::transformGlobalPlan`, which is where the goal comes from.

    This was the model's first real error and it invalidated everything built
    on it. DWB does not score against the goal the operator asked for: it
    prunes the plan to what fits in the local costmap and treats the *end of
    that* as the goal. Two steps, both of which matter:

      * points behind the rover are dropped, from the closest point on the
        plan onward, so the route never pulls the rover backwards;
      * points further from the rover than half the costmap's width are
        dropped, because the critics cannot score a cell they cannot index.

    With a 3 m rolling window that horizon is 1.5 m. Seeding GoalDist from the
    true end of a two metre plan instead puts the seed off the grid, the flood
    comes back empty, every cell reads "unreachable", and the model refuses all
    thirty-three candidates on almost every tick -- which is exactly what it
    did: 2201 refusals, none of them anything the rover agreed with.
    """
    if len(path) < 2:
        return path
    best_i, best_d = 0, 1e18
    for i, (px, py) in enumerate(path):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < best_d:
            best_d, best_i = d, i
    horizon = max(grid.width, grid.height) * grid.resolution / 2.0
    out = []
    for px, py in path[best_i:]:
        if math.hypot(px - x, py - y) > horizon:
            break
        out.append((px, py))
    if len(out) < 2:
        out = path[best_i:best_i + 2]
    return out


def ticks(episode):
    """Every commanded velocity, with the inputs that were in force for it."""
    out = []
    for command in episode["commands"]:
        when = command["t"]
        pose = nearest(episode["poses"], when, "odom")
        plan = nearest(episode["plans"], when)
        grid = nearest(episode["costmaps"], when)
        if pose is None or plan is None or grid is None:
            continue
        if "map" not in pose or len(plan["poses"]) < 2:
            continue
        out.append({"t": when, "command": command, "pose": pose,
                    "plan": plan, "costmap": grid})
    return out


def same(a, b):
    return (abs(a[0] - b[0]) <= VX_TOLERANCE
            and abs(a[1] - b[1]) <= WZ_TOLERANCE)


def replay(episode, verbose=False, limit=None):
    """Score every recorded tick and compare with what DWB actually sent."""
    path_look, goal_look, known = settings_of(episode)
    rows = ticks(episode)
    if limit:
        rows = rows[:limit]
    agreed = 0
    same_twist = 0
    both_empty = 0
    model_empty = 0
    model_refused_it = 0
    rover_stopped_model_moved = 0
    gaps = []
    spreads = []
    grids = {}
    shown = 0
    # The Oscillation critic's latch is state that survives between ticks, and
    # it is fed by what was *commanded*, not by what the model would have
    # chosen -- otherwise the replay diverges from the rover and then blames
    # the rover. The recorded commands are the truth here.
    oscillation = dwb.Oscillation()
    blame = collections.Counter()
    last_vx = last_wz = 0.0
    last_plan = None
    replans = 0
    for row in rows:
        if row["plan"]["t"] != last_plan:
            # **Every replan wipes the oscillation critic.**
            # `DWBLocalPlanner::setPlan` walks its critic list and calls
            # `reset()` on each one -- vtable slot at byte 24, which is
            # `reset` for `OscillationCritic`, read out of libdwb_core.so and
            # libdwb_critics.so rather than assumed. The behaviour tree replans
            # about once a second, so the critic that exists to remember a
            # reversal is handed a blank memory ten times more often than it
            # can fill it.
            if last_plan is not None:
                replans += 1
            oscillation.reset()
            last_plan = row["plan"]["t"]
        key = row["costmap"]["t"]
        if key not in grids:
            grids.clear()
            grids[key] = grid_of(row["costmap"])
        grid = grids[key]
        x, y, yaw = row["pose"]["odom"]
        path = transform_plan(
            plan_in_odom(row["plan"], row["pose"]["map"], row["pose"]["odom"]),
            grid, x, y)
        if len(path) < 2:
            continue
        kept, refused = dwb.evaluate(grid, path, path[-1], x, y, yaw,
                                     vx_now=last_vx, wz_now=last_wz,
                                     oscillation=oscillation,
                                     path_look=path_look,
                                     goal_look=goal_look)
        want = (row["command"]["vx"], row["command"]["wz"])
        blame.update(refused)
        if not kept:
            model_empty += 1
            if abs(want[0]) < 1e-6 and abs(want[1]) < 1e-6:
                both_empty += 1
                agreed += 1
            continue
        got = (kept[0][1], kept[0][2])
        if same(got, want):
            same_twist += 1
        elif abs(want[0]) < 1e-6 and abs(want[1]) < 1e-6:
            rover_stopped_model_moved += 1
        # What the model makes of the candidate the rover actually took. One the
        # model threw out is a real disagreement and is counted apart; one it
        # merely ranked second is not.
        score = next((sc for sc, vx, wz in kept if same((vx, wz), want)), None)
        if score is None:
            model_refused_it += 1
        else:
            gaps.append(score - kept[0][0])
            if score - kept[0][0] <= SCORE_TOLERANCE:
                agreed += 1
        spreads.append(kept[-1][0] - kept[0][0])
        oscillation.debrief(x, y, yaw, want[0], want[1])
        last_vx, last_wz = want
        if verbose and shown < 25:
            shown += 1
            print("  %6.1fs  rover vx %+.2f wz %+.2f   model vx %+.2f wz %+.2f"
                  "   %2d/%d legal  %s"
                  % (row["t"], want[0], want[1], got[0], got[1], len(kept),
                     dwb.CANDIDATES, "" if same(got, want) else "<- differ"))
    gaps.sort()
    spreads.sort()
    return {"look": (path_look, goal_look), "look_known": known,
            "ticks": len(rows), "agreed": agreed, "model_empty": model_empty,
            "both_empty": both_empty, "blame": blame, "replans": replans,
            "latch_resets": oscillation.resets, "same_twist": same_twist,
            "model_refused_it": model_refused_it, "gaps": gaps,
            "spreads": spreads,
            "rover_stopped_model_moved": rover_stopped_model_moved}


def closed_loop(episode, gain, dead_time, seconds=12.0, start=0,
                path_look=None, goal_look=None):
    """Let the model drive, on the costmap and plan the rover really had.

    `replay` asks whether the model scores the way DWB scores, one recorded
    tick at a time. It cannot ask the question that matters, because a rover
    that oscillates is not making one bad decision -- it is making a
    *sequence* of decisions each of which is defensible on its own. Only a
    closed loop shows that, so this one takes the rover's real local costmap
    and its real plan, puts the model in charge, and lets it drive.

    The costmap and the plan are held fixed for the run. That is fair here and
    would not be everywhere: in the recording being modelled the rover moved
    two centimetres in eight seconds, so its rolling window did not roll and
    its plan was replanned to nearly the same thing eight times over. It would
    not be fair on a drive that went anywhere.
    """
    rows = ticks(episode)
    if not rows:
        return None
    row = rows[start]
    grid = grid_of(row["costmap"])
    x, y, yaw = row["pose"]["odom"]
    origin = (x, y)
    full = plan_in_odom(row["plan"], row["pose"]["map"], row["pose"]["odom"])
    chassis = dwb.Chassis(gain, dead_time)
    oscillation = dwb.Oscillation()
    dt = 1.0 / dwb.CONTROLLER_FREQUENCY
    replan_every = int(round(dwb.CONTROLLER_FREQUENCY))
    vx_now = wz_now = 0.0
    turned = travelled = 0.0
    reversals = stalled = 0
    last_sign = 0
    for step in range(int(seconds / dt)):
        if step % replan_every == 0:
            # A new path resets every critic, as `setPlan` does on the rover.
            oscillation.reset()
        path = transform_plan(full, grid, x, y)
        if len(path) < 2:
            break
        kept, _ = dwb.evaluate(grid, path, path[-1], x, y, yaw,
                               vx_now=vx_now, wz_now=wz_now,
                               oscillation=oscillation,
                               path_look=path_look, goal_look=goal_look)
        if not kept:
            stalled += 1
            vx_cmd = wz_cmd = 0.0
        else:
            _, vx_cmd, wz_cmd = kept[0]
        oscillation.debrief(x, y, yaw, vx_cmd, wz_cmd)
        vx, wz = chassis.step(vx_cmd, wz_cmd)
        sign = 0 if abs(wz) < 1e-6 else (1 if wz > 0 else -1)
        if sign and last_sign and sign != last_sign:
            reversals += 1
        if sign:
            last_sign = sign
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt
        yaw = math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt))
        travelled += abs(vx) * dt
        turned += abs(wz) * dt
        vx_now, wz_now = vx, wz
    net = math.hypot(x - origin[0], y - origin[1])
    return {"net": net, "travelled": travelled,
            "turned_deg": math.degrees(turned), "reversals": reversals,
            "stalled": stalled,
            "stuck": net < 0.25 and math.degrees(turned) > 90.0}


def describe(episode):
    poses = [p["odom"] for p in episode["poses"] if "odom" in p]
    if not poses:
        return "no poses recorded"
    path = sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(poses, poses[1:]))
    net = math.hypot(poses[-1][0] - poses[0][0], poses[-1][1] - poses[0][1])
    turned = turning_of(poses)
    shapes = []
    for plan in episode["plans"]:
        pts = [(p[0], p[1]) for p in plan["poses"]]
        if len(pts) < 3:
            continue
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                     for a, b in zip(pts, pts[1:]))
        turning = 0.0
        for a, b, c in zip(pts, pts[1:], pts[2:]):
            h1 = math.atan2(b[1] - a[1], b[0] - a[0])
            h2 = math.atan2(c[1] - b[1], c[0] - b[0])
            turning += abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1)))
        if length > 0.05:
            shapes.append(math.degrees(turning) / length)
    print("the recording")
    print("   %d plans, %d costmaps, %d poses, %d commands"
          % (len(episode["plans"]), len(episode["costmaps"]),
             len(episode["poses"]), len(episode["commands"])))
    print("   the rover drove %.2f m of path, turned %.0f deg, and finished "
          "%.2f m from where it started" % (path, math.degrees(turned), net))
    if shapes:
        print("   plans handed to the controller: %.0f to %.0f deg of turning "
              "per metre (median %.0f)"
              % (min(shapes), max(shapes), sorted(shapes)[len(shapes) // 2]))
    if net < 0.25 and (path > 0.3 or math.degrees(turned) > 90.0):
        print("   this is the fault: it moved %.2f m and turned %.0f deg and "
              "finished where it started" % (path, math.degrees(turned)))
    elif path < 0.02 and math.degrees(turned) < 5.0:
        print("   NOTE: the rover never moved -- nothing to analyse here")
    else:
        print("   NOTE: the rover went somewhere, so this is a working drive")
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("episode")
    parser.add_argument("--verbose", action="store_true",
                        help="print the first 25 ticks side by side")
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--drive", action="store_true",
                        help="integrate the model forward (needs agreement)")
    parser.add_argument("--loop", action="store_true",
                        help="close the loop on this costmap and plan, with "
                             "an obedient chassis and with the measured one")
    args = parser.parse_args()

    episode = load(args.episode)
    describe(episode)
    print()
    result = replay(episode, args.verbose, args.limit)
    if not result["ticks"]:
        print("no tick had a plan, a costmap and a pose all at once -- the "
              "recording is too short or the rover was never given a goal")
        return 1
    share = 100.0 * result["agreed"] / result["ticks"]
    gaps, spreads = result["gaps"], result["spreads"]

    def pick(seq, quantile):
        if not seq:
            return 0.0
        return seq[min(len(seq) - 1, int(quantile * len(seq)))]

    print()
    print("how flat the choice is")
    print("   one tick's candidates span %.1f points at the median and %.1f at "
          "worst, out of about 45,"
          % (pick(spreads, 0.5), spreads[-1] if spreads else 0.0))
    print("   so the top is a plateau and which candidate wins is settled "
          "below the size of one cell")
    print()
    print("the model against the rover")
    print("   %d ticks compared, scored the way the controller scored at the "
          "time:" % result["ticks"])
    print("   align look-ahead %.2f / %.2f m%s"
          % (result["look"][0], result["look"][1],
             "" if result["look_known"] else
             "  (assumed -- this recording predates saving them)"))
    print("   the rover's own choice scored within %.2f of the model's best on "
          "%d of them (%.0f%%)" % (SCORE_TOLERANCE, result["agreed"], share))
    print("   median gap %.3f, p90 %.3f, worst %.3f"
          % (pick(gaps, 0.5), pick(gaps, 0.9), gaps[-1] if gaps else 0.0))
    print("   the model refused the rover's choice outright on %d ticks"
          % result["model_refused_it"])
    print("   it landed on the very same twist %d times, which on a plateau "
          "this flat is not the test" % result["same_twist"])
    print("   %d ticks where the model found nothing legal (%d of them the "
          "rover also stopped)" % (result["model_empty"], result["both_empty"]))
    print("   the oscillation latch was wiped %d times by a replan and %d "
          "times by the rover having turned far enough"
          % (result["replans"], result["latch_resets"]))
    if result["blame"]:
        print("   what the model refused candidates for, over every tick:")
        for reason, count in result["blame"].most_common(6):
            print("      %-52s %d" % (reason, count))
    print()
    if share < AGREEMENT_GATE:
        print("The model does not match the controller, so it cannot be used "
              "to test a fix.")
        print("Fix the model against these ticks first -- run with --verbose "
              "to see where it diverges.")
        return 1
    print("The model rates what the rover did as near-best on %.0f%% of ticks, "
          "so it is a fair copy of" % share)
    print("the score function. It is not a copy of the *loop*: it re-scores "
          "recorded ticks one at a")
    print("time, so nothing here says anything about the delay between a "
          "command and the rover")
    print("obeying it, which is the other half of this fault.")
    if args.loop or args.drive:
        print()
        was = settings_of(episode)
        print("the same costmap and the same plan, driven by the model at the "
              "look-ahead")
        print("in corridor_sim.py now (%.2f m), against the %.2f m the rover "
              "was running" % (dwb.FORWARD_POINT_DISTANCE, was[0]))
        print("   %-34s %8s %8s %10s %s"
              % ("chassis", "went", "turned", "reversals", ""))
        for label, gain, dead in (
                ("obeys exactly, no delay", 1.0, 0.0),
                ("obeys exactly, 0.2 s late", 1.0, dwb.DEAD_TIME_S),
                ("2.4x too fast, no delay", dwb.TURN_GAIN, 0.0),
                ("2.4x too fast, 0.2 s late  <- the real one",
                 dwb.TURN_GAIN, dwb.DEAD_TIME_S)):
            out = closed_loop(episode, gain, dead)
            if out is None:
                continue
            print("   %-34s %6.2f m %6.0f deg %8d   %s"
                  % (label, out["net"], out["turned_deg"], out["reversals"],
                     "STUCK" if out["stuck"] else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
