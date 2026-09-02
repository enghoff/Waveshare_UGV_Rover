#!/usr/bin/env python3
"""Why DWB refuses all thirty-three of its candidates in a narrow passage.

    python3 corridor_sim.py --plan             # the one that reproduces it
    python3 corridor_sim.py                    # poses, at 1.0 m
    python3 corridor_sim.py --widths 0.9,1.0,1.2,1.5
    python3 corridor_sim.py --run              # the abort-and-clear cycle

**The line this exists to explain.** The rover's own log, a hundred and
seventy-five times, and once for twelve unbroken seconds ending in a client
cancel:

    [controller_server]: Could not find a legal trajectory: No valid
    trajectories out of 33!

Thirty-three is not a round number, it is *this* rover: `vx_samples: 2` and
`vtheta_samples: 16` from config/nav2.yaml, iterated the way DWB iterates
them. When all thirty-three are thrown out the controller returns no command;
`failure_tolerance: 0.3` allows three tenths of a second of that before
`follow_path` aborts; the tree then clears a local costmap that was telling
the truth, replans, and hands back the same situation with the controller's
memory wiped.

**What reproduces it: the rover's own cell, not the plan.** Run with no
arguments and sweep a rover across a passage. Where its centre is more than
the inscribed radius from a wall every candidate survives; closer in, every
forward rollout ends on a cell the obstacle critic refuses and only the twelve
pivots are left; closer still, the pivots go too and the tick is dead. Twenty
of the hundred and five poses the body fits in lose all twenty-nine. That is
the log line, and it is a statement about where the rover is standing rather
than about the room.

**What was believed to reproduce it, and does not.** `--plan` was the mode
that mattered: `PathDist` floods outward from the plan, that flood was thought
to stop at the 253 ring, and a plan lying inside the ring therefore left the
critic nothing to flood from and refused all thirty-three wherever the rover
stood. The flood in the `libdwb_critics.so` this rover runs is not stopped by
anything -- `MapGridQueue::validCellToQueue` is `mov w0, #1; ret` and nothing
calls `setAsObstacle`. So `--plan` now comes back nearly empty, and it is kept
because a mode that used to reproduce a fault and no longer does is worth
being able to re-run. `flood()` below carries the disassembly and what
correcting it did to the model's agreement with a recorded drive: 15% to 84%.

Nor does a reverse sample rescue a dead tick (`min_vel_x` below zero rescues
six of fifty-six), nor a shorter rollout (`sim_time` from 0.8 s down to 0.3 s
changes nothing), nor moving `PathAlign.forward_point_distance` from 0.1 m to
0.32 m, which is what the rolled-back attempt did.

**The two costmaps still disagree, and that is worth watching.** The planner
works on the *global* costmap in the `map` frame and will not route through
its own inflated ring; the critics test that plan against the *local* costmap
in the `odom` frame. The two only agree while `map -> odom` does. In
`recordings/trap-2026-08-25-spin.json` the last twenty-five points of the
pruned plan lie on cells the local costmap calls 253 or 254. That no longer
refuses anything, but it does mean the critics are measuring distance to a
line drawn through a wall.

**A veto is not a score.** Three of the seven critics refuse candidates by
throwing, and tuning their *scales* does not touch that -- worth saying
plainly, because an obstacle scale of 0.02 reads like a decision to care very
little about obstacles and is nothing of the kind. The three, as the rover's
own `libdwb_critics.so` has them:

  * `BaseObstacle` reads the one cell the rover's centre is in and throws on
    253, 254 or 255; `ObstacleFootprint`, which a rectangular body takes
    instead, walks the footprint outline and throws only on 254 or 255. Which
    of the two is configured follows from whether the body is a circle.
  * `PathDist` and `GoalDist` read the flood at the rover's centre point
    rather than at its body, and can only throw for a pose off the window --
    or, if their critic never got a seed at all, for every candidate at once.
  * `PathAlign` and `GoalAlign` read the same flood at a point
    `forward_point_distance` in front of the pose, and clear
    `stop_on_failure_`, so they charge rather than refuse.
  * `Oscillation` latches the sign of the turn once the rover is pivoting and
    throws every candidate that turns back until it has moved 5 cm or turned
    0.2 rad -- and what clears that latch is a new `follow_path` goal, which
    is what the abort above produces.

**What is not modelled.** The scores themselves rank survivors and cannot
rescue a tick with none. `--run` does pick a winner by the config's weights,
so the cycle it prints is the shape of the real one rather than its exact
heading history.
"""

from __future__ import annotations

import argparse
import collections
import math
import os
import sys

import goal_fit

# Re-exported so that `import corridor_sim as dwb` still reaches every
# setting and every helper by the name it always had. dwb_replay.py,
# hybrid_astar.py, jam_repro.py and the ros_nav checks all rely on that.
from dwb_config import (
    ACC_LIM_THETA, ACC_LIM_X, ANGULAR_GRANULARITY, BACKUP_DIST, BACKUP_SPEED,
    CANDIDATES, CIRCULAR, CIRCULAR_USES_BASE_OBSTACLE, CIRCUMSCRIBED_M,
    CONTROLLER_FREQUENCY, COST_SCALING_FACTOR, CYCLE_FREQUENCY, DEAD_TIME_S,
    DECEL_LIM_THETA, DECEL_LIM_X, FAILURE_TOLERANCE_S, FOOTPRINT,
    FORWARD_POINT_DISTANCE, GOAL_ALIGN_SCALE, GOAL_DIST_SCALE, HERE, INFLATION_RADIUS,
    INSCRIBED_M, LINEAR_GRANULARITY, MAP_GRID_RESCALE, MAX_ROTATIONAL_VEL,
    MAX_VEL_THETA, MAX_VEL_X, MIN_ROTATIONAL_VEL, MIN_SPEED_THETA, MIN_SPEED_XY,
    MIN_TURN_DPS, MIN_VEL_X, OBSTACLE_RESCALE, OBSTACLE_SCALE, OBSTACLE_SCORE,
    OSCILLATION_RESET_ANGLE, OSCILLATION_RESET_DIST, PATH_ALIGN_SCALE, PATH_DIST_SCALE,
    PIVOTS, PREFER_FORWARD_PENALTY, PREFER_FORWARD_SCALE, PREFER_FORWARD_THETA_SCALE,
    RESOLUTION, ROBOT_RADIUS_CONFIGURED, ROBOT_RADIUS_M, ROTATE_TO_GOAL_SCALE,
    ROTATIONAL_ACC_LIM, SIMULATE_AHEAD_S, SIM_TIME, SLOWING_FACTOR, SPIN_DIST,
    TURN_GAIN, UNREACHABLE_SCORE, VTHETA_SAMPLES, VX_SAMPLES, XY_GOAL_TOLERANCE,
    X_ONLY_THRESHOLD, special_scores, wrap
)
from dwb_grid import (
    base_obstacle, cleared, collision_free, corridor, flood, inflate, line_cells,
    map_grid_score, obstacle_footprint, reinflate
)
from dwb_recoveries import (
    backup_recovery, drive_on_heading, escape_drive_on_heading, escape_spin,
    recovery_sweep, room_behind, room_to_turn, spin_recovery
)


# --- DWB's sample set ---------------------------------------------------------
def project_velocity(v0, acc_limit, decel_limit, dt, target):
    """`dwb_plugins::projectVelocity`, the reachable velocity after `dt`."""
    if v0 < target:
        return min(target, v0 + acc_limit * dt)
    return max(target, v0 + decel_limit * dt)


def one_d_velocities(current, low, high, acc_limit, decel_limit, acc_time,
                     samples):
    """`OneDVelocityIterator`, and the window it uses is not the control period.

    **The acceleration window is `sim_time`, not one tick of the controller**,
    and that single constant decides what the whole sample set looks like. The
    model had 1/10 s here, which clipped the band to whatever one tick of
    acceleration could reach, and every candidate then depended on how fast the
    rover happened to be turning. The recorded drive says otherwise: all
    eighty-three commands land exactly on the sixteen-way split of the full
    -0.78..+0.78 range, with nothing in between, so the band was never clipped
    at all. `StandardTrajectoryGenerator` hands `sim_time_` to the iterator,
    and at 8 rad/s^2 for 0.8 s the reachable band is 6.4 rad/s wide -- five
    times the whole range. `LimitedAccelGenerator` is the plugin that would
    have used the control period, and this rover does not use it.

    So the sample set is *static*: two forward speeds, seventeen turn rates.
    Which matters for reading the fault, because it means the rover is not
    being denied the forward sample by its own momentum. The 0.40 m/s
    candidate is offered on every single tick, and loses on score.

    The zero is inserted rather than sampled: sixteen even samples across a
    range straddling zero never land on it, so the iterator emits one extra.
    """
    current = min(max(current, low), high)
    top = project_velocity(current, acc_limit, decel_limit, acc_time, high)
    bottom = project_velocity(current, acc_limit, decel_limit, acc_time, low)
    if abs(top - bottom) < 1e-9:
        return [bottom]
    samples = max(2, samples)
    step = (top - bottom) / max(1, samples - 1)
    out = [bottom + i * step for i in range(samples)]
    if bottom < 0.0 < top and not any(abs(v) < 1e-9 for v in out):
        out.append(0.0)
    return out


def is_valid_speed(vx, wz, min_speed_xy=None, min_speed_theta=None):
    """`KinematicsHandler::isValidSpeed`: too slow in xy *and* in theta is out.

    A standing turn under the mixer floor fails both tests and is dropped. A
    0.40 m/s sample with a 3 deg/s steer fails only the theta test, so it stays
    -- steering while rolling has no stiction floor of that kind.

    The two floors are arguments and not just constants because they are the
    setting most likely to differ between a recording and the config in front
    of you. They decide which candidates *exist*, so a replay that assumes
    today's pair against a drive recorded under yesterday's is not scoring the
    same controller at all -- it is asking why the rover chose a twist the
    model never offered.
    """
    floor_xy = MIN_SPEED_XY if min_speed_xy is None else min_speed_xy
    floor_theta = MIN_SPEED_THETA if min_speed_theta is None else min_speed_theta
    if math.hypot(vx, 0.0) < floor_xy and abs(wz) < floor_theta:
        return False
    return True


def twists(vx_now=0.0, wz_now=0.0, min_speed_xy=None, min_speed_theta=None):
    """Every candidate DWB scores this tick, after the mixer floor.

    The pair (0, 0) is dropped by `nav_2d_utils::isValidSpeed`. Standing
    perfectly still is not a candidate: whatever DWB picks, the rover either
    turns or drives. The four slowest standing turns go the same way, because
    this chassis will not hold them.
    """
    xs = one_d_velocities(vx_now, MIN_VEL_X, MAX_VEL_X, ACC_LIM_X, DECEL_LIM_X,
                          SIM_TIME, VX_SAMPLES)
    thetas = one_d_velocities(wz_now, -MAX_VEL_THETA, MAX_VEL_THETA,
                              ACC_LIM_THETA, DECEL_LIM_THETA, SIM_TIME,
                              VTHETA_SAMPLES)
    return [(vx, wz) for vx in xs for wz in thetas
            if (abs(vx) > 1e-9 or abs(wz) > 1e-9)
            and is_valid_speed(vx, wz, min_speed_xy, min_speed_theta)]


def rollout(x, y, yaw, vx, wz, vx_now=None, wz_now=None):
    """`StandardTrajectoryGenerator::generateTrajectory`.

    Not `discretize_by_time`, which is off, so how many poses a candidate has
    comes from how far it travels and how far it turns rather than from the
    clock. A 0.40 m/s sample is seven poses and a full-rate pivot is
    twenty-five, and `ObstacleFootprint` checks every one of them.

    Two details that the obvious constant-velocity version gets wrong. **The
    pose the rover is standing in is the first pose of every trajectory**, so a
    rover whose own footprint is already over a lethal cell has all
    thirty-three candidates refused, whatever they were going to do about it.
    And the velocity *ramps* from what the rover is actually doing towards the
    candidate rather than starting there, so a pivot commanded at 0.78 rad/s
    from a standstill turns about six per cent less than 0.78 x 0.8 rad.
    """
    steps = max(1, int(math.ceil(max(
        abs(vx) * SIM_TIME / LINEAR_GRANULARITY,
        abs(wz) * SIM_TIME / ANGULAR_GRANULARITY))))
    dt = SIM_TIME / steps
    poses = [(x, y, yaw)]
    px, py, pyaw = x, y, yaw
    cur_x = vx if vx_now is None else vx_now
    cur_t = wz if wz_now is None else wz_now
    for _ in range(steps):
        cur_x = project_velocity(cur_x, ACC_LIM_X, DECEL_LIM_X, dt, vx)
        cur_t = project_velocity(cur_t, ACC_LIM_THETA, DECEL_LIM_THETA, dt, wz)
        px += cur_x * math.cos(pyaw) * dt
        py += cur_x * math.sin(pyaw) * dt
        pyaw = wrap(pyaw + cur_t * dt)
        poses.append((px, py, pyaw))
    return poses


def forward_pose(x, y, yaw, distance=None):
    """Where `PathAlign` and `GoalAlign` actually look: ahead of the nose.

    **This distance is the whole fault.** Everything else about a pivot is
    invisible to the four map-grid critics: they read the costmap at a point,
    the flood values in it are whole numbers of cells, and a rover turning on
    the spot does not leave its own cell. So the only thing that separates one
    pivot from another is where *this* point lands -- and at the configured
    0.1 m it sweeps about two cells across the entire turn range, which the
    integer flood cannot resolve. Sixteen pivots therefore score identically,
    the rover has no way to tell left from right, and it dithers.

    The two critics take the distance separately, as they do in nav2, because
    they flood from different seeds and a long reach costs them differently.
    """
    if distance is None:
        distance = FORWARD_POINT_DISTANCE
    return x + distance * math.cos(yaw), y + distance * math.sin(yaw)


class CommandTrend(object):
    """One dimension of `OscillationCritic`, read off the rover's own library.

    This class was wrong twice before it was disassembled, and both times the
    error made the model refuse turns the rover was happily taking. The name
    invites the wrong guess: it sounds as though commanding a left turn
    forbids a right one. It does not. **A restriction is placed only at the
    moment the sign actually reverses**, and it forbids going back the way it
    just came from:

        sign   +   +   +   -    <- the reversal, here and nowhere else
        flag   .   .   .   negative_only

    So after a left-then-right, right is all that is allowed until the flag
    clears -- and until that reversal happens nothing is forbidden at all.

    Three details from `objdump` that no amount of reading the name would give:

      * a commanded zero does nothing whatever. It does not clear the flags and
        it does not even record a sign, so a stopped tick is invisible here.
      * `update` never clears the opposite flag. Two reversals without a reset
        in between leave *both* set, and then every candidate that turns either
        way is refused -- which is the one state in which this critic can wedge
        a rover on its own.
      * only `reset()` clears them, and it clears the remembered sign too.
    """

    ZERO, POSITIVE, NEGATIVE = 0, 1, 2

    def __init__(self):
        self.reset()

    def reset(self):
        self.sign = self.ZERO
        self.positive_only = False
        self.negative_only = False

    def update(self, velocity):
        """`CommandTrend::update`: true when a restriction was just placed."""
        placed = False
        if velocity < 0.0:
            if self.sign == self.POSITIVE:
                self.negative_only = True
                placed = True
            self.sign = self.NEGATIVE
        elif velocity > 0.0:
            if self.sign == self.NEGATIVE:
                self.positive_only = True
                placed = True
            self.sign = self.POSITIVE
        return placed

    def is_oscillating(self, velocity):
        return ((self.positive_only and velocity < 0.0)
                or (self.negative_only and velocity > 0.0))

    def restricted(self):
        """`hasSignFlipped`, which despite the name is "a flag is in force"."""
        return self.positive_only or self.negative_only


class Oscillation(object):
    """`OscillationCritic`, and the two reasons it cannot stop this rover.

    **The first is the reset distance.** The flags clear as soon as the rover
    has moved 5 cm or turned 0.2 rad from wherever it last reversed, and a
    rover turning at the 0.68 rad/s this one commands covers 0.2 rad in three
    control ticks. So a restriction lives for about three hundred
    milliseconds, which is no obstacle at all to something reversing its turn
    once a second.

    **The second is worse and is not in this class**: `DWBLocalPlanner::setPlan`
    calls `reset()` on every critic, and the behaviour tree hands the
    controller a freshly planned path once a second. Every replan therefore
    wipes the memory of the critic whose entire job is remembering. The replay
    models that, because without it no honest copy of this controller is
    possible.

    Faithful to the aarch64 build on the rover: `setOscillationFlags` updates
    the forward trend always and the rotational trend only while the forward
    command is at or below `x_only_threshold`; the pose to measure from moves
    only when a restriction was just placed; and the reset is consulted only
    while some restriction is in force.
    """

    def __init__(self):
        self.x = CommandTrend()
        self.y = CommandTrend()
        self.theta = CommandTrend()
        self.prev_stationary = (0.0, 0.0, 0.0)
        self.resets = 0

    def reset(self):
        self.x.reset()
        self.y.reset()
        self.theta.reset()

    def veto(self, vx, wz):
        """`scoreTrajectory`: all three dimensions, ungated by the threshold."""
        if self.x.is_oscillating(vx) or self.theta.is_oscillating(wz):
            return "Oscillation: trajectory is oscillating"
        return ""

    def _reset_available(self, x, y, yaw):
        px, py, pyaw = self.prev_stationary
        if OSCILLATION_RESET_DIST >= 0.0:
            if (x - px) ** 2 + (y - py) ** 2 > OSCILLATION_RESET_DIST ** 2:
                return True
        if OSCILLATION_RESET_ANGLE >= 0.0:
            # Deliberately unwrapped, because nav2 does not wrap it either: it
            # subtracts two yaws straight and takes the magnitude. A rover
            # crossing +/-pi therefore gets a free reset out of arithmetic.
            if abs(yaw - pyaw) > OSCILLATION_RESET_ANGLE:
                return True
        return False

    def debrief(self, x, y, yaw, vx, wz):
        """`debrief`, in the order the library does it."""
        placed = self.x.update(vx)
        if X_ONLY_THRESHOLD < 0.0 or abs(vx) <= X_ONLY_THRESHOLD:
            placed = self.y.update(0.0) or placed
            placed = self.theta.update(wz) or placed
        if placed:
            self.prev_stationary = (x, y, yaw)
        if not (self.x.restricted() or self.y.restricted()
                or self.theta.restricted()):
            return
        if self._reset_available(x, y, yaw):
            self.reset()
            self.resets += 1


def rotate_to_goal(x, y, yaw, goal, vx, wz, end_yaw, goal_yaw=None):
    """`RotateToGoalCritic`: inside the goal circle, only turning is allowed.

    Its `prepare` sets an in-window flag when the rover is closer to the goal
    than `xy_goal_tolerance`, and while that is set every candidate with any
    translation at all is thrown out, so within 22 cm of the goal the sample
    set collapses from thirty-three to the sixteen pivots. nav2.yaml already
    records that as the reason a goal 23 cm out sat and shuffled; it has to be
    modelled here or a replay of the last seconds of any drive disagrees with
    the rover for a reason that has nothing to do with the fault.

    Returns (score, veto reason). Outside the window it is silent.
    """
    if math.hypot(goal[0] - x, goal[1] - y) >= XY_GOAL_TOLERANCE:
        return 0.0, ""
    if abs(vx) > 1e-6:
        return None, "RotateToGoal: nonrotation command near goal"
    if goal_yaw is None:
        return 0.0, ""
    return abs(wrap(end_yaw - goal_yaw)) / SLOWING_FACTOR, ""


def last_pose_on_costmap(grid, path):
    """`GoalDistCritic::getLastPoseOnCostmap`, and leaving it out broke the model.

    `GoalDist` and `GoalAlign` do not flood from the end of the plan. They flood
    from the last point of it that `worldToMap` can still convert -- walking
    forward, skipping any lead-in that is off the window, and stopping at the
    first point that falls off it again.

    The distinction is not academic on this rover. `transformGlobalPlan` keeps
    plan points within `max(width, height) * resolution / 2` of the robot, which
    is 1.5 m, while the window those points have to land in is a 3 m *square*
    that the rolling costmap re-centres on cell boundaries rather than on the
    robot. So the far end of the pruned plan sits exactly on the boundary and
    routinely falls just outside it: on the recorded drive, 63 of 89 ticks.
    Seeding the flood from that point instead means seeding it from nothing,
    every cell comes back unreachable, and a model that then let those critics
    refuse candidates threw out all of them -- on exactly those 63 ticks.

    A cell that reads *unknown* is skipped as though it were off the window,
    and that is the only cost this function looks at: the one comparison in
    `GoalDistCritic::getLastPoseOnCostmap` is `cmp w0, #0xff`. A plan point
    lying on the inflated ring, or on the wall itself, is a perfectly good
    seed.
    """
    found = None
    started = False
    for px, py in path:
        col, row = grid.cell_of(px, py)
        if 0 <= col < grid.width and 0 <= row < grid.height \
                and grid.cost(col, row) != goal_fit.UNKNOWN:
            found = (col, row)
            started = True
        elif started:
            break
    return found


def prefer_forward(vx, wz):
    """`PreferForwardCritic::scoreTrajectory`, on this chassis's samples."""
    if vx < 0.0:
        return PREFER_FORWARD_PENALTY
    return abs(wz) * PREFER_FORWARD_THETA_SCALE


def evaluate(grid, path, goal, x, y, yaw, vx_now=0.0, wz_now=0.0,
             oscillation=None, path_values=None, goal_values=None,
             goal_yaw=None, path_look=None, goal_look=None,
             min_speed_xy=None, min_speed_theta=None):
    """Score every candidate, or say which critic threw it out.

    Returns the survivors as (score, vx, wz) and a tally of refusals by
    critic. `aggregation_type` is "last" for the four map-grid critics, which
    is their default and matters: a pivot's last pose is where the rover
    already stands, so a rover whose centre is inside the 253 ring has every
    one of its sixteen pivots refused before the wall is even consulted.
    """
    if path_values is None:
        path_values = flood(grid, [grid.cell_of(px, py) for px, py in path])
    if goal_values is None:
        seed = last_pose_on_costmap(grid, path)
        goal_values = flood(grid, [seed] if seed else [])
    near_goal = math.hypot(goal[0] - x, goal[1] - y) <= FORWARD_POINT_DISTANCE
    kept = []
    refused = collections.Counter()
    for vx, wz in twists(vx_now, wz_now, min_speed_xy, min_speed_theta):
        if oscillation is not None:
            reason = oscillation.veto(vx, wz)
            if reason:
                refused[reason] += 1
                continue
        poses = rollout(x, y, yaw, vx, wz, vx_now, wz_now)
        end_x, end_y, end_yaw = poses[-1]
        # The config's critic order is RotateToGoal, Oscillation,
        # ObstacleFootprint, then the four map-grid ones, and the order is not
        # decoration: `short_circuit_trajectory_evaluation` is true, so the
        # first critic to throw is the one the log would name.
        turn_score, reason = rotate_to_goal(x, y, yaw, goal, vx, wz, end_yaw,
                                            goal_yaw)
        if reason:
            refused[reason] += 1
            continue
        if CIRCULAR and CIRCULAR_USES_BASE_OBSTACLE:
            obstacle, reason = base_obstacle(grid, poses)
        else:
            obstacle, reason = obstacle_footprint(grid, poses)
        if reason:
            refused[reason] += 1
            continue
        total = (OBSTACLE_RESCALE * OBSTACLE_SCALE * obstacle
                 + ROTATE_TO_GOAL_SCALE * turn_score
                 + PREFER_FORWARD_SCALE * prefer_forward(vx, wz))
        failed = ""
        # The order is the one nav2.yaml lists, because
        # `short_circuit_trajectory_evaluation` is on and the first critic to
        # throw is the one that gets the blame in the log and in the tally.
        # `PathAlign` switches itself off within `forward_point_distance` of
        # the local goal, because past the end of the path there is nothing
        # left to line up with. Its scale, and only its scale, goes to zero.
        align_scale = 0.0 if near_goal else PATH_ALIGN_SCALE
        # `stops` is `stop_on_failure_`, and it changes both what a critic may
        # refuse and how much of the rollout it reads -- see `map_grid_score`.
        for name, values, scale, stops, look in (
                ("GoalAlign", goal_values, GOAL_ALIGN_SCALE, False, goal_look),
                ("PathAlign", path_values, align_scale, False, path_look),
                ("PathDist", path_values, PATH_DIST_SCALE, True, None),
                ("GoalDist", goal_values, GOAL_DIST_SCALE, True, None)):
            value = 0.0
            if stops:
                # Every pose of the rollout is tested and any one of them can
                # refuse the candidate; the score kept is the last one's.
                for px, py, _pyaw in poses:
                    value, reason = map_grid_score(grid, values, px, py, name,
                                                   True)
                    if reason:
                        failed = reason
                        break
            else:
                # The flag is clear, so the loop starts at `size - 1`: the last
                # pose, read at the forward point, and nothing it finds there
                # can refuse the candidate.
                point = forward_pose(end_x, end_y, end_yaw, look)
                value, reason = map_grid_score(grid, values, point[0], point[1],
                                               name, False)
                if reason:
                    failed = reason
            if failed:
                break
            total += MAP_GRID_RESCALE * scale * value
        if failed:
            refused[failed] += 1
            continue
        kept.append((total, vx, wz))
    kept.sort()
    return kept, refused


# --- the sweep ------------------------------------------------------------------
def straight_path(grid, x, y, ahead=2.0):
    """A route down the middle of the passage, which is what NavFn produces."""
    steps = int(ahead / RESOLUTION)
    return [(x + i * RESOLUTION, 0.0) for i in range(steps + 1)]


def sweep(width_m, headings, doorway=False):
    """How many candidates survive at each pose, footprint clearing and all.

    The costmap is rebuilt for every pose because the obstacle layer clears
    whatever the body is standing on before the inflation layer runs, so the
    map a rover sees genuinely depends on where the rover is. Leaving that out
    is what makes a clean corridor look like a trap.
    """
    limit = width_m / 2.0 - 0.14
    offsets = [round(-0.35 + 0.05 * i, 2) for i in range(15)]
    offsets = [o for o in offsets if abs(o) < limit]
    rows = []
    for offset in offsets:
        counts, blames = [], []
        for heading in headings:
            yaw = math.radians(heading)
            grid = corridor(width_m, doorway=doorway,
                            clear_at=(0.0, offset, yaw))
            path = straight_path(grid, 0.0, 0.0)
            kept, refused = evaluate(grid, path, (path[-1][0], 0.0),
                                     0.0, offset, yaw)
            counts.append(len(kept))
            blames.append(refused)
        rows.append((offset, counts, blames))
    return rows


def render(width_m, headings, rows):
    print("a %.2f m passage, thirty-three candidates a tick, no oscillation "
          "latch yet" % width_m)
    print("      offset  " + "".join("%5d" % h for h in headings)
          + "   deg off the passage")
    for offset, counts, _ in rows:
        marks = "".join(("    ." if c == 0 else "%5d" % c) for c in counts)
        print("      %+5.2f m %s" % (offset, marks))
    print("      '.' is a tick with nothing to drive -- the log's "
          "'No valid trajectories out of 33'")
    forward_only = [(o, h) for o, counts, _ in rows
                    for h, c in zip(headings, counts) if c == PIVOTS]
    if forward_only:
        print("      a count of %d is the remaining pivots surviving and no "
              "forward candidate at all" % PIVOTS)


def blame(rows):
    """Which critic did the refusing, over the whole sweep."""
    tally = collections.Counter()
    for _, _, blames in rows:
        for refused in blames:
            tally.update(refused)
    return tally


# --- the cycle ------------------------------------------------------------------
def plant(vx, wz):
    """`drive_mixer`: a standing turn under 12 deg/s does not clear stiction.

    So it is lifted to 12, and a rover asked for a two-degree correction gets
    a twelve-degree one. On a rover with no creep forward speed either, this
    is why "nearly lined up" is not a state this chassis can hold.
    """
    if abs(vx) < 0.05:
        dps = math.degrees(abs(wz))
        if dps < 1e-3:
            return 0.0, 0.0
        return 0.0, math.copysign(math.radians(max(MIN_TURN_DPS, dps)), wz)
    return vx, wz


class Chassis(object):
    """What the rover does with a command, rather than what it was told to do.

    The model used to assume the wheels obeyed instantly and exactly, and with
    that assumption the controller is stable at every corridor width and start
    pose worth trying -- which is why three attempts at this fault found
    nothing. **Two tenths of a second of delay and a factor of two and a half
    are the whole difference**, and neither is a Nav2 setting, so no amount of
    reading nav2.yaml would have turned them up.

    Set `gain` to 1 and `dead_time` to 0 to get the old, obedient rover back;
    that comparison is the experiment, not a fallback.
    """

    def __init__(self, gain=TURN_GAIN, dead_time=DEAD_TIME_S,
                 dt=1.0 / CONTROLLER_FREQUENCY):
        self.gain = gain
        self.dt = dt
        self.queue = [(0.0, 0.0)] * max(0, int(round(dead_time / dt)))

    def step(self, vx_cmd, wz_cmd):
        """The twist on the wheels now, given the one just commanded."""
        # The mixer's floors first, because they act on the command; then the
        # delay, because it is downstream of everything; then the gain, which
        # is the chassis itself out-running its own calibration.
        vx, wz = plant(vx_cmd, wz_cmd)
        self.queue.append((vx, wz * self.gain))
        return self.queue.pop(0)


def run(width_m, seconds=12.0, doorway=False, start_offset=0.30,
        start_heading_deg=90.0, gain=TURN_GAIN, dead_time=DEAD_TIME_S):
    """The controller, the abort and the costmap clear, at 10 Hz.

    The default start is the one the sweep says is the trap: 30 cm off the
    centreline of a metre-wide passage, which leaves 6 cm between the body and
    the wall, facing across it. That is a pose the rover reaches by drifting,
    not one anybody drives it to on purpose, and once it is there the
    controller cannot get it out.

    Everything with a clock in it runs at the rate the config gives it -- the
    controller at 10 Hz, the behaviour tree's replan at 1 Hz, the obstacle
    layer refilling a cleared costmap in one 5 Hz update, and `follow_path`
    aborting after 0.3 s with no command at all.
    """
    path = straight_path(corridor(width_m, doorway=doorway), 0.0, 0.0)
    goal = (path[-1][0], 0.0)
    oscillation = Oscillation()

    x, y, yaw = 0.0, start_offset, math.radians(start_heading_deg)
    start = (x, y)
    real = corridor(width_m, doorway=doorway, clear_at=(x, y, yaw))
    grid = real
    chassis = Chassis(gain, dead_time)
    turned = 0.0
    travelled = 0.0
    vx_now = wz_now = 0.0
    dt = 1.0 / CONTROLLER_FREQUENCY
    dead_since = None
    refill_at = None
    aborts = clears = dead_ticks = flips = 0
    last_sign = 0
    events = []
    t = 0.0
    while t < seconds:
        # The obstacle layer clears the footprint every update, so the map
        # follows the rover rather than standing still.
        real = corridor(width_m, doorway=doorway, clear_at=(x, y, yaw))
        if refill_at is not None and t >= refill_at:
            refill_at = None
        grid = cleared(real) if refill_at is not None else real
        path_values = flood(grid, [grid.cell_of(px, py) for px, py in path])
        goal_values = flood(grid, [grid.cell_of(*goal)])
        kept, refused = evaluate(grid, path, goal, x, y, yaw, vx_now, wz_now,
                                 oscillation, path_values, goal_values)
        if not kept:
            dead_ticks += 1
            if dead_since is None:
                dead_since = t
            elif t - dead_since >= FAILURE_TOLERANCE_S:
                aborts += 1
                clears += 1
                worst = refused.most_common(1)[0][0] if refused else "?"
                events.append("%5.1f s  no command for %.1f s -- %s; "
                              "follow_path aborts, local costmap cleared"
                              % (t, t - dead_since, worst))
                # The recovery: clear the costmap, and start a fresh
                # follow_path, which resets every critic's memory.
                refill_at = t + 0.2
                oscillation.reset()
                dead_since = None
            vx_cmd = wz_cmd = 0.0
        else:
            dead_since = None
            _, vx_cmd, wz_cmd = kept[0]
        vx, wz = chassis.step(vx_cmd, wz_cmd)
        sign = 0 if abs(wz) < 1e-6 else (1 if wz > 0 else -1)
        if sign and last_sign and sign != last_sign:
            flips += 1
            events.append("%5.1f s  turn reverses, now %s at %.0f deg/s"
                          % (t, "left" if sign > 0 else "right",
                             math.degrees(abs(wz))))
        if sign:
            last_sign = sign
        # nav2 debriefs the critic with what it *commanded*, not with what the
        # wheels did about it -- the critic never sees the plant at all.
        oscillation.debrief(x, y, yaw, vx_cmd, wz_cmd)
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt
        yaw = wrap(yaw + wz * dt)
        travelled += abs(vx) * dt
        turned += abs(wz) * dt
        vx_now, wz_now = vx, wz
        t += dt
    net = math.hypot(x - start[0], y - start[1])
    return {"x": x, "y": y, "yaw_deg": math.degrees(yaw), "aborts": aborts,
            "clears": clears, "dead_ticks": dead_ticks, "flips": flips,
            "events": events, "ticks": int(seconds * CONTROLLER_FREQUENCY),
            "net": net, "travelled": travelled,
            "turned_deg": math.degrees(turned),
            "stuck": net < 0.25 and math.degrees(turned) > 90.0}


def render_run(width_m, result, offset, heading):
    print("%.0f s in a %.2f m passage, starting %+.2f m off the centreline "
          "and %.0f deg off it" % (result["ticks"] / CONTROLLER_FREQUENCY,
                                   width_m, offset, heading))
    for line in result["events"][:14]:
        print("  " + line)
    if len(result["events"]) > 14:
        print("  ... and %d more" % (len(result["events"]) - 14))
    print("  ticks with no command at all   %d of %d"
          % (result["dead_ticks"], result["ticks"]))
    print("  follow_path aborts             %d" % result["aborts"])
    print("  local costmaps cleared         %d" % result["clears"])
    print("  turn reversals                 %d" % result["flips"])
    print("  ended at (%.2f, %.2f) facing %.0f deg"
          % (result["x"], result["y"], result["yaw_deg"]))


def plan_sweep(widths, offsets):
    """What happens when the plan itself runs inside the inflated ring.

    `PathDist` floods outward from the plan and is stopped by 253. When every
    cell of the plan is *already* 253 there is nowhere to flood from, so every
    cell of the local costmap comes back "unreachable" and the critic refuses
    all thirty-three -- wherever the rover is standing and whichever way it is
    facing. This is the one failure that does not care about the rover's pose
    at all, which is why it looks like a rover that cannot move rather than a
    rover that cannot fit.
    """
    print("all thirty-three refused, by how far the plan runs off the centreline")
    print("      plan off centre  " + "".join("%8.1f m" % w for w in widths))
    for plan_y in offsets:
        cells = []
        for width_m in widths:
            if plan_y >= width_m / 2.0:
                cells.append("       -")
                continue
            dead = total = 0
            for offset in (-0.2, -0.1, 0.0, 0.1, 0.2):
                if abs(offset) >= width_m / 2.0 - 0.14:
                    continue
                for degrees in range(-180, 180, 30):
                    yaw = math.radians(degrees)
                    grid = corridor(width_m, clear_at=(0.0, offset, yaw))
                    path = [(i * RESOLUTION, plan_y) for i in range(31)]
                    kept, _ = evaluate(grid, path, path[-1], 0.0, offset, yaw)
                    total += 1
                    if not kept:
                        dead += 1
            cells.append("%5d/%2d" % (dead, total))
        print("         %+.2f m       %s" % (plan_y, "".join("%8s" % c
                                                             for c in cells)))
    print("      this used to be a wall of refusals and is now nearly empty:")
    print("      a plan inside the inscribed ring was thought to leave PathDist")
    print("      nothing to flood from, and the flood in the installed library")
    print("      is not stopped by the ring at all -- see flood() above. What")
    print("      is left here is the obstacle critic refusing a rover that is")
    print("      already touching the wall, which is the default sweep's fault")
    print("      and not this one's")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--width", type=float, default=1.0)
    parser.add_argument("--widths", default=None)
    parser.add_argument("--doorway", action="store_true",
                        help="a gap in a wall rather than a tube")
    parser.add_argument("--run", action="store_true",
                        help="drive the cycle rather than sweep the poses")
    parser.add_argument("--plan", action="store_true",
                        help="sweep how far off centre the plan runs")
    parser.add_argument("--recover", action="store_true",
                        help="what Spin and BackUp manage, by look-ahead")
    parser.add_argument("--horizons", default="2.0,0.5",
                        help="simulate_ahead_time values to compare")
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--offset", type=float, default=0.30,
                        help="metres off the passage centreline to start")
    parser.add_argument("--heading", type=float, default=90.0,
                        help="degrees off the passage to start")
    args = parser.parse_args()

    if len(twists()) != CANDIDATES:
        print("the sample set is %d, not the %d the rover's log names -- "
              "config/nav2.yaml has moved" % (len(twists()), CANDIDATES))
        return 1

    if args.recover:
        horizons = [float(h) for h in args.horizons.split(",")]
        wasted = recovery_sweep(args.width, horizons)
        print()
        print("%d of the poses swept have room to turn and get nothing at "
              "%.1f s of look-ahead." % (wasted, horizons[0]))
        return 0

    if args.plan:
        plan_sweep([0.8, 0.9, 1.0, 1.2, 1.5],
                   [round(0.20 + 0.04 * i, 2) for i in range(8)])
        return 0

    if args.run:
        result = run(args.width, args.seconds, args.doorway,
                     args.offset, args.heading)
        render_run(args.width, result, args.offset, args.heading)
        if result["dead_ticks"]:
            print("\nreproduced: %d ticks with nothing to drive, %d aborts, "
                  "%d turn reversals" % (result["dead_ticks"],
                                         result["aborts"], result["flips"]))
            return 0
        print("\nnot reproduced: the controller had a command every tick")
        return 1

    widths = [float(w) for w in args.widths.split(",")] if args.widths \
        else [args.width]
    headings = list(range(0, 91, 15))
    dead = 0
    for width_m in widths:
        rows = sweep(width_m, headings, args.doorway)
        render(width_m, headings, rows)
        stuck = sum(1 for _, counts, _ in rows for c in counts if c == 0)
        total = sum(len(counts) for _, counts, _ in rows)
        dead += stuck
        print("      dead ticks %d of %d poses the body fits in" % (stuck, total))
        for reason, count in blame(rows).most_common(6):
            print("      %-52s %d" % (reason, count))
        print()

    print("%d of the poses swept lose all %d candidates. A clean passage is "
          "not the fault:" % (dead, CANDIDATES))
    print("the pose has to be all but touching a wall, and the obstacle layer "
          "clears the footprint")
    print("every update, so the rover carves itself free. Run --plan for the "
          "one that reproduces.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
