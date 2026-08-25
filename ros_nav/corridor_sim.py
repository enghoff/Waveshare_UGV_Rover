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

**What reproduces it: the plan, not the passage.** `--plan` is the mode that
matters. `PathDist` floods the local costmap outward from the plan and that
flood is stopped by 253, the inscribed ring the inflation layer paints 14 cm
deep along every wall. When *every* cell of the plan is already inside that
ring there is nowhere to flood from, every cell of the costmap comes back
"unreachable", and the critic refuses all thirty-three -- wherever the rover
is standing and whichever way it is facing. It is total, it is independent of
the rover's pose, and it recurs on every replan. A plan within about 6 cm of
a wall does it, which a 0.8 m passage reaches at 0.32 m off the centreline and
a 1.2 m passage does not reach at all.

**What does not reproduce it, having been tried.** A narrow passage on its
own: once the obstacle layer's footprint clearing is modelled -- and it is on,
`footprint_clearing_enabled` was read off the running node -- a rover in a
clean 0.9 m corridor loses all thirty-three at two poses in three hundred and
twelve. Nor does a reverse sample (`min_vel_x` below zero rescues six of
fifty-six), nor a shorter rollout (`sim_time` from 0.8 s down to 0.3 s changes
nothing), nor moving `PathAlign.forward_point_distance` from 0.1 m to 0.32 m,
which is what the rolled-back attempt did.

**The link still to be closed on the rover.** NavFn plans on the *global*
costmap, in the `map` frame, and would not route through its own inflated
ring. `PathDist` tests that plan against the *local* costmap, in the `odom`
frame. The two only agree while `map -> odom` does, and in a metre-wide
passage a few centimetres of disagreement is the whole margin. Confirming
that means watching the transformed plan against the live local costmap
during an episode, or turning `debug_trajectory_details` on and reading which
critic names itself -- `dwb_bench.py` already starts a shadow controller that
does the second without moving the wheels.

**A veto is not a score.** Four of the seven critics refuse candidates by
throwing, and tuning their *scales* does not touch that -- worth saying
plainly, because `ObstacleFootprint.scale: 0.005` reads like a decision to
care very little about obstacles and is nothing of the kind. The four, as the
rover's own `libdwb_critics.so` has them:

  * `ObstacleFootprint` walks the footprint outline at every pose of the
    rollout and throws on 254 or 255. Its `pointCost` compares the cell
    against 0xfe and 0xff and has no third comparison, so the 253 ring is a
    cost here and not contact.
  * `PathDist` and `GoalDist` are the flood described above, and they test the
    rover's centre point rather than its body.
  * `PathAlign` and `GoalAlign` run the same test at a point
    `forward_point_distance` in front of the pose.
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
import heapq
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import goal_fit

# --- config/nav2.yaml, controller_server -------------------------------------
# Copied rather than parsed, the way dwb_bench.py and steering_sim.py copy them.
# Named after the keys so a change there that is not made here is findable.
MIN_VEL_X = 0.0
MAX_VEL_X = 0.40
MAX_VEL_THETA = 0.78
VX_SAMPLES = 2
VTHETA_SAMPLES = 16
SIM_TIME = 0.8
LINEAR_GRANULARITY = 0.05
ANGULAR_GRANULARITY = 0.025
ACC_LIM_X = 4.0
ACC_LIM_THETA = 8.0
DECEL_LIM_X = -4.0
DECEL_LIM_THETA = -8.0
CONTROLLER_FREQUENCY = 10.0
FAILURE_TOLERANCE_S = 0.3
XY_GOAL_TOLERANCE = 0.22

# **The four map-grid critics do not weigh what nav2.yaml says they weigh.**
# `MapGridCritic::getScale` returns `resolution * 0.5 * scale`, so at 5 cm cells
# the 32 below is really 0.8 and the 24 is really 0.6 -- a fortieth of the
# configured number. `ObstacleFootprint` is not a map-grid critic and is not
# rescaled, so its 0.005 is the real 0.005. Reading the numbers off the config
# as written puts the path critics forty times too heavy against the obstacle
# cost, which is the difference between a landscape where the wall matters and
# one where it is a rounding error. Taken from the rover's own
# `include/dwb_critics/map_grid.hpp`.
PATH_ALIGN_SCALE = 32.0
PATH_DIST_SCALE = 32.0
GOAL_ALIGN_SCALE = 24.0
GOAL_DIST_SCALE = 24.0
OBSTACLE_SCALE = 0.005
# Tracks config/nav2.yaml. A stale copy here is not a cosmetic drift: this
# distance decides which candidates carry the unreachable charge, and on some
# geometry it decides whether driving or turning is the one penalised.
FORWARD_POINT_DISTANCE = 0.325

# dwb_critics defaults, which the config does not override.
ROTATE_TO_GOAL_SCALE = 32.0
SLOWING_FACTOR = 5.0

OSCILLATION_RESET_DIST = 0.05
OSCILLATION_RESET_ANGLE = 0.2
X_ONLY_THRESHOLD = 0.05

#: local_costmap: a 3 m rolling window at 5 cm, obstacle + inflation.
RESOLUTION = 0.05
#: What `MapGridCritic::getScale` does to every one of the four.
MAP_GRID_RESCALE = RESOLUTION * 0.5
#: The footprint, and the inscribed radius nav2 derives from it -- the distance
#: from the body's origin to the nearest edge, which is what gets inflated to
#: 253 and therefore what stops the PathDist flood. This is the body as
#: measured off the chassis by `lidar_slam/slam2d.c`, and it is what the rover
#: runs.
FOOTPRINT = [(0.20, 0.14), (0.20, -0.14), (-0.16, -0.14), (-0.16, 0.14)]
#: **Describe the rover as a circle instead. False, because that is what the
#: rover runs -- the circle was tried and rolled back.**
#:
#: The argument for a circle is real and worth keeping: a pivot sweeps the
#: circumscribed radius, 0.244 m, while the inflation layer paints its hard 253
#: ring at the inscribed one, 0.14 m. The ten centimetres between them are a
#: band around every wall the rover may legally drive into and then not turn out
#: of, and the rover was once found wedged in exactly that band -- 0.21 m off a
#: wall, five centimetres from a legal pose, the body fitting at five of sixteen
#: headings in the local costmap and none at all in the planner's. A radius
#: makes the two numbers one number, so anywhere it may stand is somewhere it
#: may turn.
#:
#: What it costs is doorways. Set this True and pass `reinflate=True` to
#: `dwb_replay.closed_loop` -- which rebuilds a recording's inflation at the new
#: radius, so an old drive can judge a shape change fairly -- to re-run the
#: comparison. The answer, on all three recordings, is the table in
#: config/nav2.yaml beside the `footprint` line. Only 0.244 closes the band, and
#: it is the worst row of the four -- five escapes of eight on the corridor
#: where the rectangle makes eight of eight.
CIRCULAR = False
#: The radius to describe it with when CIRCULAR. Not the circumscribed radius of
#: the measured rectangle (0.244 m), which closes the band completely and costs
#: the doorways; this is the value the rover's owner measured off the chassis.
ROBOT_RADIUS_CONFIGURED = 0.175
#: Which obstacle critic goes with it. `BaseObstacle` refuses at 253, so the
#: controller may not put its centre in the ring at all; `ObstacleFootprint`
#: traces the outline and refuses only on contact at 254. The first is far
#: stricter and is what a circular body would normally use, and it is only a
#: collision test at all while the ring is as big as the whole robot -- which is
#: why the rectangle above must keep `ObstacleFootprint`.
CIRCULAR_USES_BASE_OBSTACLE = True
CIRCUMSCRIBED_M = max(math.hypot(x, y) for x, y in FOOTPRINT)
ROBOT_RADIUS_M = ROBOT_RADIUS_CONFIGURED
INSCRIBED_M = ROBOT_RADIUS_M if CIRCULAR else 0.14
if CIRCULAR:
    # nav2 turns a radius into a twelve-sided polygon, not a square: a square
    # drawn round a circle is 27% too big at the corners.
    FOOTPRINT = [(ROBOT_RADIUS_M * math.cos(i * math.pi / 6.0),
                  ROBOT_RADIUS_M * math.sin(i * math.pi / 6.0))
                 for i in range(12)]

#: And what `ObstacleFootprintCritic::getScale` does to the obstacle critic,
#: which is *not* the same thing and was the model's other scale error. Its
#: header on the rover reads `return costmap_->getResolution() * scale_;`,
#: where the `BaseObstacleCritic` it replaced inherited the plain `scale_`.
#: So the swap to a footprint check quietly divided the weight on obstacle cost
#: by twenty, on top of the deliberate 0.02 -> 0.005 cut. `BaseObstacle` is
#: correct only with a circular body -- see CIRCULAR above.
OBSTACLE_RESCALE = (1.0 if (CIRCULAR and CIRCULAR_USES_BASE_OBSTACLE)
                    else RESOLUTION)
INFLATION_RADIUS = 0.45
COST_SCALING_FACTOR = 3.0

#: lidar_slam/nav_types.py, applied by drive_mixer: a standing turn slower than
#: this does not clear stiction, so it is lifted to this. It is why a two-degree
#: correction leaves as a twelve-degree one.
MIN_TURN_DPS = 12.0

# --- the chassis, as measured off a recorded drive ----------------------------
# **These two numbers are the other half of this fault, and neither is in any
# config file.** They were fitted to `episode-fault.json` by cross-correlating
# the turn Nav2 commanded against the turn the gyro then reported:
#
#     lag   0.0s  0.1s  0.2s  0.3s  0.4s  0.5s  0.6s  0.7s
#     corr  0.15  0.46  0.85  0.67  0.27 -0.06 -0.41 -0.50
#
# The peak is unambiguous and it is *positive*, which is worth saying because a
# rover whose yaw moves the opposite way to its command looks exactly like
# miswired motors or an inverted gyro. It is neither: the sign is right and the
# response is simply two ticks late. The correlation goes negative at 0.6-0.7 s,
# which is the half-period of the swing.
#
#: Two control ticks between Nav2 asking and the gyro showing it. Assembled out
#: of six small delays, none of them wrong on its own: the controller's own
#: 10 Hz, the velocity smoother's 10 Hz, `base_node` at 50 Hz, the loopback
#: bridge, the ESP32's 17 Hz telemetry coming back, and the chassis reversing.
DEAD_TIME_S = 0.2
#: And it turns two and a half times faster than it was asked to. The mixer's
#: own arithmetic accounts for only 1.14 of that on this command sequence -- the
#: 12 deg/s from-rest floor over-serving the two smallest samples, which are
#: also the two most often chosen. The rest is the pivot curve itself: it was
#: measured by timing *bursts from a standstill*, so its rate is an average that
#: includes the spin-up, and the sustained rate at the same PWM is higher.
TURN_GAIN = 2.4

#: What the rover's log prints when it gives up, so a sample set that stops
#: matching it is noticed rather than quietly simulated. Thirty-three was the
#: set with every standing turn included (2 x 17, less (0, 0)). The mixer floor
#: drops the four slowest pivots (±0.052, ±0.156 rad/s); the live set is 29.
CANDIDATES = 29
#: Standing turns that survive min_speed_xy / min_speed_theta. Used to read the
#: pose-sweep table: a count of this many and no more is "pivots only".
PIVOTS = 12

#: behavior_server, read off the running node. `simulate_ahead_time` is the
#: number that decides whether a wedged rover turns at all: Spin and BackUp
#: project their whole motion this far forward and return FAILED if any of it
#: touches a lethal cell, rather than moving as far as they safely can.
CYCLE_FREQUENCY = 10.0
SIMULATE_AHEAD_S = 2.0
MAX_ROTATIONAL_VEL = 0.5
MIN_ROTATIONAL_VEL = 0.2
ROTATIONAL_ACC_LIM = 1.5

#: What the recovery subtree of the behaviour tree asks those two for.
SPIN_DIST = 1.57
BACKUP_DIST = 0.30
BACKUP_SPEED = 0.15

#: **`MapGridCritic`'s two special cell values, which are not sentinels.**
#:
#: Read out of the rover's own `libdwb_critics.so` rather than assumed.
#: `MapGridCritic::reset` converts the costmap's cell count to a double, adds
#: one, and stores the pair::
#:
#:     ucvtf d0, x1            ; obstacle_score_   = number of cells
#:     fmov  d30, #1.0
#:     fadd  d30, d0, d30      ; unreachable_score_ = cells + 1
#:     stp   d0, d30, [x19, #160]
#:
#: So on this rover's 3 m window at 5 cm they are 3600 and 3601, and they are
#: *scores* rather than flags. That matters because two of the four map-grid
#: critics do not refuse a pose that lands on one -- they return the number.
#: A nose pointing into a wall is then ruinously expensive and still legal,
#: which is the difference between a controller that picks the least bad option
#: and a model that finds nothing to pick at all.
def special_scores(grid):
    """`obstacle_score_`, `unreachable_score_` for a costmap of this size."""
    cells = float(grid.width * grid.height)
    return cells, cells + 1.0


#: Kept only so the flood can fill an array before it knows better; every
#: comparison against them goes through `special_scores` for the real values.
OBSTACLE_SCORE = -1.0
UNREACHABLE_SCORE = -2.0


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


# Copied from config/nav2.yaml. Nav2's isValidSpeed is an AND of the two, so a
# theta floor with xy left at zero does not drop anything.
MIN_SPEED_XY = 0.1
MIN_SPEED_THETA = 0.21


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


def wrap(radians):
    return math.atan2(math.sin(radians), math.cos(radians))


# --- the costmap --------------------------------------------------------------
def _lower_envelope(values):
    """One dimension of Felzenszwalb's exact distance transform.

    An approximate transform is not good enough here. The threshold that
    matters is 253, which nav2 paints at exactly the inscribed radius, and a
    cell put one centimetre the wrong side of it is the difference between a
    candidate DWB refuses and one it scores. A nearest-source flood gets some
    of those cells wrong; this does not.
    """
    n = len(values)
    hull = [0] * n
    edge = [0.0] * (n + 1)
    k = 0
    hull[0] = 0
    edge[0] = -1e20
    edge[1] = 1e20
    for q in range(1, n):
        while True:
            p = hull[k]
            crossing = ((values[q] + q * q) - (values[p] + p * p)) / (2.0 * q - 2.0 * p)
            if crossing <= edge[k]:
                k -= 1
                if k < 0:
                    k = 0
                    break
            else:
                break
        k += 1
        hull[k] = q
        edge[k] = crossing
        edge[k + 1] = 1e20
    out = [0.0] * n
    k = 0
    for q in range(n):
        while edge[k + 1] < q:
            k += 1
        p = hull[k]
        out[q] = (q - p) * (q - p) + values[p]
    return out


def inflate(width, height, lethal):
    """`nav2_costmap_2d::InflationLayer`, over a set of lethal cells.

    Everything within the inscribed radius becomes 253 and everything out to
    the inflation radius decays from 252. In a metre-wide passage the whole
    floor is inside the second and a strip 14 cm deep along each wall is
    inside the first -- a preference and a barrier respectively, and the
    difference between the two is most of this file.

    The distance every cell is inflated by is exact, and cheap enough to
    rebuild for every pose -- which this has to do, because the obstacle layer
    clears whatever the footprint is standing on before the inflation layer
    ever runs.
    """
    far = float(width * width + height * height)
    field = [far] * (width * height)
    for col, row in lethal:
        field[row * width + col] = 0.0
    for col in range(width):
        column = _lower_envelope([field[row * width + col] for row in range(height)])
        for row in range(height):
            field[row * width + col] = column[row]
    data = [0] * (width * height)
    reach_cells = INFLATION_RADIUS / RESOLUTION
    inscribed_cells = INSCRIBED_M / RESOLUTION
    for row in range(height):
        base = row * width
        line = _lower_envelope(field[base:base + width])
        for col in range(width):
            away = math.sqrt(line[col])
            if away == 0.0:
                data[base + col] = goal_fit.LETHAL
            elif away <= inscribed_cells:
                data[base + col] = goal_fit.INSCRIBED
            elif away <= reach_cells:
                data[base + col] = int((goal_fit.INSCRIBED - 1) * math.exp(
                    -COST_SCALING_FACTOR * (away * RESOLUTION - INSCRIBED_M)))
    return data


def corridor(width_m, length_m=4.0, doorway=False, clear_at=None):
    """A passage of that width running east, walls both sides.

    The rover's y is measured from the centreline, so an offset of zero is a
    body exactly between the walls. `clear_at` is a pose whose footprint the
    obstacle layer has set to free space, which the live costmap does
    (`footprint_clearing_enabled` is true on this rover, checked) and which
    matters more than it sounds: without it a body that has clipped a wall
    stays clipped for ever, and a simulation that leaves it out invents a
    deadlock the real rover does not have.
    """
    margin = 0.9
    height_m = width_m + 2.0 * margin
    width = int(round(length_m / RESOLUTION))
    height = int(round(height_m / RESOLUTION))
    origin_x, origin_y = -1.2, -height_m / 2.0
    blank = goal_fit.CostGrid(width, height, RESOLUTION, origin_x, origin_y,
                              [0] * (width * height))
    spared = set() if clear_at is None else goal_fit.covered(
        blank, FOOTPRINT, clear_at[0], clear_at[1], clear_at[2])
    lethal = []
    for col in range(width):
        x = origin_x + (col + 0.5) * RESOLUTION
        for row in range(height):
            y = origin_y + (row + 0.5) * RESOLUTION
            outside = abs(y) > width_m / 2.0
            if doorway:
                outside = 0.95 <= x <= 1.05 and abs(y) > width_m / 2.0
            if outside and (col, row) not in spared:
                lethal.append((col, row))
    return goal_fit.CostGrid(width, height, RESOLUTION, origin_x, origin_y,
                             inflate(width, height, lethal))


def cleared(grid):
    """The costmap the behaviour tree's recovery leaves behind: empty.

    `ClearEntireCostmap` does not clear "what was wrong"; it clears
    everything, and the obstacle layer then refills from the next scan. For
    one update period every candidate is legal, which is why the rover twitches
    rather than standing still.
    """
    return goal_fit.CostGrid(grid.width, grid.height, grid.resolution,
                             grid.origin_x, grid.origin_y,
                             [0] * (grid.width * grid.height))


# --- the critics that can throw -----------------------------------------------
def line_cells(col0, row0, col1, row1):
    """Bresenham, as `nav2_util::LineIterator` walks a footprint edge."""
    cells = []
    dc, dr = abs(col1 - col0), abs(row1 - row0)
    step_c = 1 if col1 > col0 else -1
    step_r = 1 if row1 > row0 else -1
    err = dc - dr
    col, row = col0, row0
    while True:
        cells.append((col, row))
        if col == col1 and row == row1:
            return cells
        err2 = 2 * err
        if err2 > -dr:
            err -= dr
            col += step_c
        if err2 < dc:
            err += dc
            row += step_r


def base_obstacle(grid, poses):
    """`BaseObstacleCritic`, which is the right critic once the body is a circle.

    It reads the one cell the robot's centre is in and refuses anything that is
    not free. `isValidCost` in `libdwb_critics.so` is a single comparison --
    `cmp w1, #0xfc; cset w0, ls` -- so a cost is valid exactly when it is 252 or
    less, and 253, 254 and 255 are all refusals.

    That is a *point* test, and it is only a collision test while the inflation
    layer's inscribed ring is as big as the whole robot. With the measured
    rectangle it was not, which is why `ObstacleFootprint` replaced it. With a
    circular `robot_radius` it is again, exactly -- and it is now stronger than
    the footprint critic ever was, because that one only refused on contact at
    254 while this refuses at 253, the ring. The ring is 0.244 m deep, so the
    controller can no longer drive the rover into the band where it would not
    be able to turn round.

    `aggregation_type` is "last" and `sum_scores` is off, so every pose can
    refuse the candidate but only the final one's cost is scored.
    """
    last = 0.0
    for x, y, _yaw in poses:
        col, row = grid.cell_of(x, y)
        if not (0 <= col < grid.width and 0 <= row < grid.height):
            return None, "BaseObstacle: trajectory goes off grid"
        cost = grid.cost(col, row)
        if cost > 252:
            return None, "BaseObstacle: trajectory hits obstacle"
        last = float(cost)
    return last, ""


def reinflate(grid, inscribed_m=None, inflation_m=None, scaling=None):
    """Rebuild a recorded costmap's inflation at a different inscribed radius.

    A recording carries the costmap the rover had, inflated under the settings
    of the day. Testing a change to the robot's *shape* against it means
    redoing that arithmetic, because the shape is what sets the ring. Only the
    lethal cells are kept -- they are the observations; everything else in the
    grid was derived from them and can be derived again.

    The formula is `InflationLayer::computeCost` and it was checked against a
    recorded map before being trusted: fitting cost against distance over 2931
    gradient cells returned a scaling factor of 3.00 and an inscribed radius of
    0.149 m, against the 3.0 and 0.14 the config asked for.
    """
    inscribed_m = INSCRIBED_M if inscribed_m is None else inscribed_m
    inflation_m = INFLATION_RADIUS if inflation_m is None else inflation_m
    scaling = COST_SCALING_FACTOR if scaling is None else scaling
    lethal = [(c, r) for r in range(grid.height) for c in range(grid.width)
              if grid.cost(c, r) == goal_fit.LETHAL]
    unknown = set((c, r) for r in range(grid.height) for c in range(grid.width)
                  if grid.cost(c, r) == goal_fit.UNKNOWN)
    reach = int(math.ceil(inflation_m / grid.resolution))
    data = [0] * (grid.width * grid.height)
    best = {}
    for lc, lr in lethal:
        for dr in range(-reach, reach + 1):
            for dc in range(-reach, reach + 1):
                nc, nr = lc + dc, lr + dr
                if not (0 <= nc < grid.width and 0 <= nr < grid.height):
                    continue
                away = math.hypot(dc, dr) * grid.resolution
                if away > inflation_m:
                    continue
                if away < best.get((nc, nr), 1e9):
                    best[(nc, nr)] = away
    for (c, r), away in best.items():
        if away == 0.0:
            data[r * grid.width + c] = goal_fit.LETHAL
        elif away <= inscribed_m:
            data[r * grid.width + c] = goal_fit.INSCRIBED
        else:
            data[r * grid.width + c] = int((goal_fit.INSCRIBED - 1) * math.exp(
                -scaling * (away - inscribed_m)))
    for c, r in unknown:
        data[r * grid.width + c] = goal_fit.UNKNOWN
    return goal_fit.CostGrid(grid.width, grid.height, grid.resolution,
                             grid.origin_x, grid.origin_y, data)


def obstacle_footprint(grid, poses):
    """`ObstacleFootprintCritic`: the outline, every pose, 254 and 255 only.

    Verified against the rover's own `libdwb_critics.so` rather than assumed:
    `pointCost` compares the cell against 0xfe and 0xff and has no third
    comparison, so the 253 ring the inflation layer paints along every wall is
    a cost here and not a refusal. The refusal comes from the map-grid critics
    below, which is a much easier thing to miss.
    """
    # `BaseObstacleCritic::scoreTrajectory` with `sum_scores` off -- which is
    # the default and is what this rover runs -- keeps only the *last* pose's
    # cost, however bad the ones on the way there were. Every pose can still
    # refuse the candidate outright; only the score is the last one's. Taking
    # the worst instead quietly makes the wall matter along the whole rollout.
    last = 0.0
    for x, y, yaw in poses:
        worst = 0.0
        col, row = grid.cell_of(x, y)
        if not (0 <= col < grid.width and 0 <= row < grid.height):
            return None, "ObstacleFootprint: trajectory goes off grid"
        corners = goal_fit.corners(FOOTPRINT, x, y, yaw)
        for i, (ax, ay) in enumerate(corners):
            bx, by = corners[(i + 1) % len(corners)]
            c0, r0 = grid.cell_of(ax, ay)
            c1, r1 = grid.cell_of(bx, by)
            for col, row in line_cells(c0, r0, c1, r1):
                if not (0 <= col < grid.width and 0 <= row < grid.height):
                    return None, "ObstacleFootprint: footprint goes off grid"
                cost = grid.cost(col, row)
                if cost == goal_fit.LETHAL:
                    return None, "ObstacleFootprint: trajectory hits obstacle"
                if cost == goal_fit.UNKNOWN:
                    return None, "ObstacleFootprint: trajectory hits unknown"
                worst = max(worst, float(cost))
        last = worst
    return last, ""


def flood(grid, seeds):
    """`MapGridCritic`'s propagation: distance in cells, stopped by 253.

    Seeded from the path or from the goal and flooded four-connected, exactly
    as `CostmapQueue` does it. A cell it refuses to enter is marked obstacle
    and a cell it never reaches stays unreachable, and the critic throws on
    both -- so a strip 14 cm deep along every wall is a hard refusal for the
    rover's *centre point*, and anything the flood cannot get to behind it is
    a refusal too.
    """
    values = [UNREACHABLE_SCORE] * (grid.width * grid.height)
    queue = collections.deque()
    for col, row in seeds:
        if not (0 <= col < grid.width and 0 <= row < grid.height):
            continue
        index = row * grid.width + col
        if goal_fit.blocked(grid.cost(col, row)) \
                or grid.cost(col, row) == goal_fit.UNKNOWN:
            values[index] = OBSTACLE_SCORE
            continue
        if values[index] == UNREACHABLE_SCORE:
            values[index] = 0.0
            queue.append((col, row, 0.0))
    while queue:
        col, row, distance = queue.popleft()
        for dcol, drow in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nc, nr = col + dcol, row + drow
            if not (0 <= nc < grid.width and 0 <= nr < grid.height):
                continue
            index = nr * grid.width + nc
            if values[index] != UNREACHABLE_SCORE:
                continue
            cost = grid.cost(nc, nr)
            if goal_fit.blocked(cost) or cost == goal_fit.UNKNOWN:
                values[index] = OBSTACLE_SCORE
                continue
            values[index] = distance + 1.0
            queue.append((nc, nr, distance + 1.0))
    return values


def map_grid_score(grid, values, x, y, name, stop_on_failure=True):
    """`MapGridCritic::scorePose`, which is a point test, not a body test.

    **Only two of the four map-grid critics can refuse anything, and that was
    the model's largest error.** `MapGridCritic::onInit` sets `stop_on_failure_`
    true, and both align critics overwrite it with zero immediately after
    calling their base -- in `libdwb_critics.so`, at offset 176 of the critic::

        MapGridCritic::onInit    mov  w0, #0x1 ; strb w0, [x19, #176]
        PathAlignCritic::onInit  blr  x1      ; strb wzr, [x19, #176]
        GoalAlignCritic::onInit  blr  x1      ; strb wzr, [x19, #176]

    `scoreTrajectory` reads that byte twice. It decides whether the obstacle
    and unreachable values throw, and it decides how much of the rollout is
    looked at: with the flag clear the loop starts at `size - 1`, so an align
    critic sees the last pose and nothing else.

    Going off the grid is different and refuses either way, because that throw
    lives in `scorePose` where the flag cannot reach it.
    """
    col, row = grid.cell_of(x, y)
    if not (0 <= col < grid.width and 0 <= row < grid.height):
        return None, "%s: trajectory goes off grid" % name
    value = values[row * grid.width + col]
    obstacle, unreachable = special_scores(grid)
    if value == OBSTACLE_SCORE:
        if stop_on_failure:
            return None, "%s: trajectory hits obstacle" % name
        return obstacle, ""
    if value == UNREACHABLE_SCORE:
        if stop_on_failure:
            return None, "%s: trajectory hits unreachable area" % name
        return unreachable, ""
    return value, ""


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
    """
    found = None
    started = False
    for px, py in path:
        col, row = grid.cell_of(px, py)
        if 0 <= col < grid.width and 0 <= row < grid.height:
            found = (col, row)
            started = True
        elif started:
            break
    return found


#: `PreferForwardCritic`, which is not in the rover's critic list and is
#: modelled here so it can be tried before it is. It is the only critic
#: available that says anything about turning *as such*: the four map-grid ones
#: read the costmap in whole cells, so of the twelve standing turns this
#: chassis can command only about five come back with distinct scores and the
#: rest are exact ties settled by rounding. Its score is
#: `fabs(velocity.theta) * theta_scale` for a forward trajectory and a flat
#: `penalty` for a reverse or strafing one, so it is symmetric in left and
#: right -- it cannot say which way to turn, only that turning costs more than
#: going. That is the right shape for a rover that pivots 93% of the time with
#: clear floor in front of it, and the wrong shape for one that needs to pivot
#: to get out of somewhere.
PREFER_FORWARD_THETA_SCALE = 10.0
PREFER_FORWARD_PENALTY = 1.0
#: 0.0 keeps it out of the sum, which is what the rover runs today.
PREFER_FORWARD_SCALE = 0.5


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
    print("      a plan inside the 14 cm inscribed ring leaves PathDist nothing")
    print("      to flood from, and every candidate is refused everywhere")


# --- the recoveries -------------------------------------------------------------
def collision_free(grid, x, y, yaw):
    """`CostmapTopicCollisionChecker::isCollisionFree`, at one pose.

    The footprint outline, refused at 254 and at nothing lower. Read off the
    rover's own `libnav2_costmap_2d_client.so`, where the double compared
    against is 0x406FC00000000000 -- 254.0 -- so the recoveries use the same
    true-lethal threshold the controller does, and are not being stopped by
    inflation they could safely sit in.
    """
    worst, _ = obstacle_footprint(grid, [(x, y, yaw)])
    return worst is not None


def room_to_turn(grid, x, y, yaw, limit=SPIN_DIST):
    """How far the body could rotate from here before it actually touched."""
    step = MAX_ROTATIONAL_VEL / CYCLE_FREQUENCY
    turned = 0.0
    while turned < limit:
        if not collision_free(grid, x, y, wrap(yaw + turned + step)):
            return turned
        turned += step
    return limit


def room_behind(grid, x, y, yaw, limit=BACKUP_DIST):
    """How far the body could reverse from here before it actually touched."""
    step = BACKUP_SPEED / CYCLE_FREQUENCY
    moved = 0.0
    while moved < limit:
        back = moved + step
        if not collision_free(grid, x - back * math.cos(yaw),
                              y - back * math.sin(yaw), yaw):
            return moved
        moved += step
    return limit


def spin_recovery(grid, x, y, yaw, target=SPIN_DIST,
                  simulate_ahead_s=SIMULATE_AHEAD_S):
    """`nav2_behaviors::Spin`, and why a wedged rover does not turn at all.

    Each cycle it works out the speed it wants, then projects that rotation
    forward `cycle_frequency * simulate_ahead_time` cycles and tests the
    footprint at every projected heading. One collision anywhere in that
    projection returns FAILED for the whole behaviour -- it does not turn as
    far as it can and stop there. At 0.5 rad/s over a 2 s horizon the
    projection is 0.95 rad, so a rover with twenty degrees of room is asked
    whether it has fifty-four, and told no.
    """
    max_cycles = int(CYCLE_FREQUENCY * simulate_ahead_s)
    turned = 0.0
    for _ in range(int(CYCLE_FREQUENCY * 60)):
        remaining = target - turned
        if remaining < 1e-6:
            return turned, "completed"
        speed = math.sqrt(2.0 * ROTATIONAL_ACC_LIM * remaining)
        speed = min(max(speed, MIN_ROTATIONAL_VEL), MAX_ROTATIONAL_VEL)
        for cycle in range(max_cycles):
            ahead = speed * (cycle / CYCLE_FREQUENCY)
            if remaining - abs(ahead) <= 0.0:
                break
            if not collision_free(grid, x, y, wrap(yaw + turned + ahead)):
                return turned, "collision ahead"
        turned += speed / CYCLE_FREQUENCY
    return turned, "out of time"


def backup_recovery(grid, x, y, yaw, target=BACKUP_DIST,
                    simulate_ahead_s=SIMULATE_AHEAD_S):
    """`nav2_behaviors::BackUp`, which is `DriveOnHeading` with a sign.

    The same shape and the same trap: 0.15 m/s over a 2 s horizon projects
    0.29 m, which is the whole 0.30 m the tree asked for, so a rover with ten
    centimetres behind it reverses none of them.
    """
    max_cycles = int(CYCLE_FREQUENCY * simulate_ahead_s)
    moved = 0.0
    for _ in range(int(CYCLE_FREQUENCY * 60)):
        remaining = target - moved
        if remaining < 1e-6:
            return moved, "completed"
        for cycle in range(max_cycles):
            ahead = BACKUP_SPEED * (cycle / CYCLE_FREQUENCY)
            if remaining - abs(ahead) <= 0.0:
                break
            back = moved + ahead
            if not collision_free(grid, x - back * math.cos(yaw),
                                  y - back * math.sin(yaw), yaw):
                return moved, "collision ahead"
        moved += BACKUP_SPEED / CYCLE_FREQUENCY
    return moved, "out of time"


def recovery_sweep(width_m, horizons):
    """What the recoveries get out of a rover pressed up against a wall.

    The offsets are the ones a rover in a metre-wide passage actually reaches:
    the body is 0.28 m across and the walls are a metre apart, so 0.30 m off
    the centreline leaves six centimetres, which is a drift rather than a
    driving error. `room` is what the geometry allows; the columns beside it
    are what each look-ahead lets the behaviour take of it.
    """
    limit = width_m / 2.0 - 0.14
    offsets = [o for o in (round(0.10 + 0.05 * i, 2) for i in range(9))
               if o < limit]
    wasted = 0
    print("a %.2f m passage, body 0.28 m across, walls at %.2f m"
          % (width_m, width_m / 2.0))
    print()
    print("  Spin, asked for 90 deg")
    print("      off centre  clearance   room  " +
          " ".join("  @%.1fs" % h for h in horizons))
    for offset in offsets:
        yaw = 0.0
        grid = corridor(width_m, clear_at=(0.0, offset, yaw))
        room = math.degrees(room_to_turn(grid, 0.0, offset, yaw))
        cells = []
        for horizon in horizons:
            turned, _ = spin_recovery(grid, 0.0, offset, yaw,
                                      simulate_ahead_s=horizon)
            cells.append("%5.0f  " % math.degrees(turned))
        first = math.degrees(spin_recovery(grid, 0.0, offset, yaw,
                                           simulate_ahead_s=horizons[0])[0])
        if room > 5.0 and first < 1.0:
            wasted += 1
        print("      %+5.2f m     %.2f m   %4.0f deg %s"
              % (offset, width_m / 2.0 - offset - 0.14, room, "".join(cells)))
    print()
    print("  BackUp, asked for 0.30 m, rover across the passage")
    print("      off centre  clearance   room  " +
          " ".join("  @%.1fs" % h for h in horizons))
    for offset in offsets:
        yaw = math.pi / 2.0
        grid = corridor(width_m, clear_at=(0.0, offset, yaw))
        room = room_behind(grid, 0.0, offset, yaw)
        cells = []
        for horizon in horizons:
            moved, _ = backup_recovery(grid, 0.0, offset, yaw,
                                       simulate_ahead_s=horizon)
            cells.append("%5.2f  " % moved)
        print("      %+5.2f m     %.2f m   %.2f m %s"
              % (offset, width_m / 2.0 - offset - 0.14, room, "".join(cells)))
    print()
    print("      'room' is what the body could do; the columns are what the")
    print("      behaviour takes of it at each simulate_ahead_time. A row where")
    print("      room is real and the first column is zero is a rover that")
    print("      could have moved and reported 'Collision Ahead' instead.")
    return wasted


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
