#!/usr/bin/env python3
"""DWB will drive the body into a door-frame halo, then the action dies.

    python3 jam_repro.py

The mixer floor stopped the doorway pivoting. The recording after that
(`recordings/doorway-2026-08-25-after-floor.json`) is a different fault:
the rover drove 3.5 m, then sat next to a door frame for fifty seconds.
The last driving command was 0.40 m/s, the rectangle already covering
the inscribed ring, the nose four centimetres from lethal, and 1.1 m
still to the goal. Then one (0, 0) and silence.

Two halves, kept apart because they are different faults:

  1. On a synthetic 0.80 m door -- a real interior opening, not a pinch
     the body cannot fit -- a rover that has drifted 14 cm off centre
     at 20 degrees still has DWB pick 0.40 m/s. ObstacleFootprint does
     not veto 253. The centre cell is a legal planner step. Reverse is
     not in the sample set. Thirty centimetres behind is clear. That is
     the last driving tick of the recording, without needing the file.

  2. On the recording, PoseProgressChecker (10 cm or 20 deg in 15 s)
     would have called the sit stuck, and on the tick the rover
     published (0, 0) the model still had a legal forward candidate.
     So the sitting is not DWB running out of trajectories and not the
     progress checker being too kind. FollowPath had already ended,
     and BackUp never ran.

A frozen-map closed loop is not this. The 0.8 m look-ahead shipped on
one of those. The synthetic pose is scored once, against a costmap
built for that pose. The recording is scored against the costmap that
was in force at that tick.
"""
from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import corridor_sim as dwb
import goal_fit

RECORDING = os.path.join(HERE, "recordings",
                         "doorway-2026-08-25-after-floor.json")

#: A typical interior door, wide enough that a body on the centreline
#: fits with room to spare. The trap is drifting toward one jamb, not
#: squeezing through a gap the chassis cannot physically take.
GAP_M = 0.80
WALL_THICK_M = 0.40
WALL_AT_M = 1.00

#: Where the last driving command of the recording sat, in this geometry:
#: 14 cm off the centreline, 20 degrees toward the near jamb, the nose
#: about 4 cm from lethal. Measured to match that tick, not tuned to
#: make DWB fail.
APPROACH_X = 0.95
APPROACH_Y = 0.14
APPROACH_YAW = math.radians(20.0)

# PoseProgressChecker in config/nav2.yaml. Copied rather than parsed so
# a desk run without the yaml still grades the recording the same way.
PROGRESS_RADIUS_M = 0.10
PROGRESS_ANGLE_RAD = 0.35
PROGRESS_ALLOWANCE_S = 15.0


def door_frame(gap_m=GAP_M, wall_thick_m=WALL_THICK_M, wall_at_m=WALL_AT_M,
               length_m=4.0, clear_at=None):
    """A partition with a gap in it, running across the rover's path.

    Lethal only in the wall, not a tube: once the rover is through, the
    far side is open floor. Footprint clearing is the live obstacle
    layer (`footprint_clearing_enabled` is on).
    """
    res = dwb.RESOLUTION
    height_m = 3.0
    width = int(round(length_m / res))
    height = int(round(height_m / res))
    origin_x, origin_y = -1.0, -height_m / 2.0
    blank = goal_fit.CostGrid(width, height, res, origin_x, origin_y,
                              [0] * (width * height))
    spared = set() if clear_at is None else goal_fit.covered(
        blank, list(dwb.FOOTPRINT), *clear_at)
    half = gap_m / 2.0
    lethal = []
    for col in range(width):
        x = origin_x + (col + 0.5) * res
        for row in range(height):
            y = origin_y + (row + 0.5) * res
            if (wall_at_m <= x <= wall_at_m + wall_thick_m
                    and abs(y) > half
                    and (col, row) not in spared):
                lethal.append((col, row))
    return goal_fit.CostGrid(width, height, res, origin_x, origin_y,
                             dwb.inflate(width, height, lethal))


def body_stats(grid, x, y, yaw):
    costs = [grid.cost(c, r)
             for c, r in goal_fit.covered(grid, list(dwb.FOOTPRINT), x, y, yaw)]
    return {
        "lethal": sum(1 for c in costs if c == goal_fit.LETHAL),
        "ring": sum(1 for c in costs if c == goal_fit.INSCRIBED),
        "max": max(costs) if costs else 0,
        "centre": grid.cost(*grid.cell_of(x, y)),
        "fits": goal_fit.fits(grid, list(dwb.FOOTPRINT), x, y, yaw),
    }


def nearest_lethal(grid, x, y):
    best = None
    for col in range(grid.width):
        for row in range(grid.height):
            if grid.cost(col, row) != goal_fit.LETHAL:
                continue
            cx = grid.origin_x + (col + 0.5) * grid.resolution
            cy = grid.origin_y + (row + 0.5) * grid.resolution
            d = math.hypot(cx - x, cy - y)
            if best is None or d < best:
                best = d
    return best


def through_path(x0=0.0, ahead=2.4):
    """A route down the gap, the shape a point search draws."""
    n = int(ahead / dwb.RESOLUTION)
    return [(x0 + i * dwb.RESOLUTION, 0.0) for i in range(n + 1)]


def score_approach(x=APPROACH_X, y=APPROACH_Y, yaw=APPROACH_YAW, path=None):
    """What DWB does at the pose that matches the last driving command."""
    grid = door_frame(clear_at=(x, y, yaw))
    path = path if path is not None else through_path()
    kept, refused = dwb.evaluate(grid, path, path[-1], x, y, yaw)
    nose_x = x + dwb.FOOTPRINT[0][0] * math.cos(yaw)
    nose_y = y + dwb.FOOTPRINT[0][0] * math.sin(yaw)
    stats = body_stats(grid, x, y, yaw)
    stats.update({
        "legal": len(kept),
        "candidates": dwb.CANDIDATES,
        "best_vx": None if not kept else kept[0][1],
        "best_wz": None if not kept else kept[0][2],
        "refused": dict(refused),
        "nose_lethal_m": nearest_lethal(grid, nose_x, nose_y),
        "origin_lethal_m": nearest_lethal(grid, x, y),
        "room_behind_m": dwb.room_behind(grid, x, y, yaw),
        "reverse_samples": sum(1 for vx, _wz in dwb.twists() if vx < -1e-9),
        "x": x, "y": y, "yaw": yaw,
    })
    return stats


def progress_stuck_at(poses, t0, radius_m=PROGRESS_RADIUS_M,
                      angle_rad=PROGRESS_ANGLE_RAD,
                      allowance_s=PROGRESS_ALLOWANCE_S):
    """When PoseProgressChecker would first call this trace stuck.

    Baseline resets on 10 cm of travel *or* 20 degrees of turn, which is
    why a legitimate pivot is not a jam. A rover that does neither for
    15 s is. Returns the time of the first failure, or None.
    """
    baseline = None
    for pose in poses:
        if "odom" not in pose or pose["t"] < t0:
            continue
        x, y, yaw = pose["odom"]
        if baseline is None:
            baseline = (pose["t"], x, y, yaw)
            continue
        bt, bx, by, byaw = baseline
        moved = math.hypot(x - bx, y - by)
        turned = abs(math.atan2(math.sin(yaw - byaw), math.cos(yaw - byaw)))
        if moved >= radius_m or turned >= angle_rad:
            baseline = (pose["t"], x, y, yaw)
        elif pose["t"] - bt >= allowance_s:
            return pose["t"]
    return None


def recording_sit(path=RECORDING):
    """The live episode, as numbers a test can fail against.

    Skips DWB on every tick: the last 15 of this file agree with the
    model only 40 percent of the time, which is below the gate
    `dwb_replay.py` uses before a fix may be tried on a recording.
    What *is* fair is the body, the last command, and the pose trace
    after commands stop -- those are measured, not modelled.
    """
    if not os.path.isfile(path):
        return None
    import dwb_replay as replay
    episode = replay.load(path)
    commands = episode["commands"]
    if not commands:
        return None
    nonzero = [c for c in commands
               if abs(c["vx"]) > 0.02 or abs(c["wz"]) > 0.02]
    last_drive = nonzero[-1] if nonzero else commands[-1]
    last = commands[-1]
    pose = replay.nearest(episode["poses"], last_drive["t"], "odom")
    plan = replay.nearest(episode["plans"], last_drive["t"])
    cmap = replay.nearest(episode["costmaps"], last_drive["t"])
    if pose is None or cmap is None or "map" not in pose:
        return None
    grid = replay.grid_of(cmap)
    x, y, yaw = pose["odom"]
    stats = body_stats(grid, x, y, yaw)
    mx, my = pose["map"][0], pose["map"][1]
    goal = plan["poses"][-1] if plan and plan["poses"] else (mx, my)
    after = [p for p in episode["poses"]
             if "odom" in p and p["t"] >= last_drive["t"]]
    end = after[-1] if after else pose
    dx = end["odom"][0] - x
    dy = end["odom"][1] - y
    dyaw = abs(math.atan2(math.sin(end["odom"][2] - yaw),
                          math.cos(end["odom"][2] - yaw)))
    # The tick the rover published (0, 0): did the model still want to move?
    model_legal = None
    if last is not last_drive:
        stop_pose = replay.nearest(episode["poses"], last["t"], "odom")
        stop_plan = replay.nearest(episode["plans"], last["t"])
        stop_map = replay.nearest(episode["costmaps"], last["t"])
        if (stop_pose is not None and stop_plan is not None
                and stop_map is not None and "map" in stop_pose
                and len(stop_plan["poses"]) >= 2):
            sgrid = replay.grid_of(stop_map)
            sx, sy, syaw = stop_pose["odom"]
            spath = replay.transform_plan(
                replay.plan_in_odom(stop_plan, stop_pose["map"],
                                    stop_pose["odom"]),
                sgrid, sx, sy)
            if len(spath) >= 2:
                kept, _ = dwb.evaluate(sgrid, spath, spath[-1], sx, sy, syaw)
                model_legal = len(kept)
    return {
        "last_drive_vx": last_drive["vx"],
        "last_drive_wz": last_drive["wz"],
        "last_drive_t": last_drive["t"],
        "last_cmd_vx": last["vx"],
        "last_cmd_wz": last["wz"],
        "last_cmd_t": last["t"],
        "ring": stats["ring"],
        "lethal": stats["lethal"],
        "centre": stats["centre"],
        "fits": stats["fits"],
        "goal_m": math.hypot(goal[0] - mx, goal[1] - my),
        "sat_m": math.hypot(dx, dy),
        "sat_deg": math.degrees(dyaw),
        "sat_s": end["t"] - last_drive["t"],
        "stuck_at": progress_stuck_at(episode["poses"], last_drive["t"]),
        "model_legal_at_stop": model_legal,
    }


def jam_reproduction():
    """The numbers the selftest holds this fault to."""
    approach = score_approach()
    live = recording_sit()
    return {"approach": approach, "recording": live}


def main():
    result = jam_reproduction()
    a = result["approach"]
    print("synthetic 0.80 m door, rover 14 cm off centre at 20 deg")
    print("  body: %d ring, %d lethal, centre %d, fits=%s"
          % (a["ring"], a["lethal"], a["centre"], a["fits"]))
    print("  nose %.3f m from lethal, origin %.3f m"
          % (a["nose_lethal_m"], a["origin_lethal_m"]))
    print("  DWB %d/%d legal, best vx=%s wz=%s"
          % (a["legal"], a["candidates"],
             ("%.2f" % a["best_vx"]) if a["best_vx"] is not None else "none",
             ("%.2f" % a["best_wz"]) if a["best_wz"] is not None else "none"))
    print("  reverse samples %d, room behind %.2f m"
          % (a["reverse_samples"], a["room_behind_m"]))
    live = result["recording"]
    if live is None:
        print()
        print("no recordings/doorway-2026-08-25-after-floor.json")
        return 0
    print()
    print("recording after the mixer floor")
    print("  last drive t=%.2f vx=%+.3f wz=%+.3f"
          % (live["last_drive_t"], live["last_drive_vx"], live["last_drive_wz"]))
    print("  last command t=%.2f vx=%+.3f wz=%+.3f"
          % (live["last_cmd_t"], live["last_cmd_vx"], live["last_cmd_wz"]))
    print("  body: %d ring, %d lethal, centre %d, fits=%s, %.2f m from goal"
          % (live["ring"], live["lethal"], live["centre"], live["fits"],
             live["goal_m"]))
    print("  then sat %.2f m / %.1f deg in %.0f s"
          % (live["sat_m"], live["sat_deg"], live["sat_s"]))
    if live["stuck_at"] is None:
        print("  PoseProgressChecker would not have called it stuck")
    else:
        print("  PoseProgressChecker would have called it stuck at t=%.1f s"
              % live["stuck_at"])
    if live["model_legal_at_stop"] is None:
        print("  no model score for the stop tick")
    else:
        print("  on the (0, 0) tick the model still had %d legal candidates"
              % live["model_legal_at_stop"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
