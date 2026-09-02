#!/usr/bin/env python3
"""The rover's DWB and costmap settings, mirrored from config/nav2.yaml.

Every number here is read off the configuration the rover actually runs, and the
comment beside it says what it is and where it came from. They are in a file of
their own because three modules need them -- the grid, the recoveries and the
simulation itself -- and a constant that has drifted from the YAML is the one
fault this whole reproduction cannot survive.

Change the YAML and change these together, or the model stops being a model of
this rover.
"""

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


OBSTACLE_SCALE = 0.02


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


#: **Describe the rover as a circle. True, because that is what the rover runs.**
#:
#: The rectangle above is not a measurement of the body: it is `slam2d.c`'s
#: lidar self-return mask, where the returns span 8.5-11.2 cm behind and
#: 8.2-10.7 cm to each side over 397 revolutions, plus about 5 cm of masking
#: margin -- and its forward 0.20 was never measured at all, because the lidar
#: sees straight past the body that way.
#:
#: More to the point it is the wrong *kind* of shape for a rover that pivots.
#: nav2 paints its hard 253 ring at the inscribed radius, 0.14 m for that
#: rectangle, while a turn on the spot sweeps the circumscribed one. The ten
#: centimetres between them are a band around every wall the rover may legally
#: drive into and then not turn out of, and the rover was found wedged in
#: exactly that band: 0.21 m off a wall, five centimetres from a legal pose,
#: with the body fitting at five of sixteen headings locally and none at all in
#: the planner's map. This chassis's whole control set is 32 in-place rotations,
#: so "may I stand here" and "may I turn here" have to be one question, and a
#: radius is the only shape that makes them one.
#:
#: Set this False to model the rectangle again, and pass `reinflate=True` to
#: `dwb_replay.closed_loop` whenever the shape changes, so a recording's
#: inflation is rebuilt at the radius under test rather than judging a new body
#: against the old ring.
CIRCULAR = True


#: The radius to describe it with. **0.20 m, measured off the chassis with a
#: tape rather than inferred from the lidar's view of it.**
#:
#: Do not pick this number off the escape counts. Driven from twelve starts in
#: each of the three recordings they run 36/36 at a five-centimetre body, 36/36
#: at the rectangle, 35 at 0.175, 33 at 0.200, 33 at 0.213 and 31 at 0.244 --
#: monotone, and an absurd body wins. The replay knows what the costmap forbids
#: and nothing about the rover hitting anything, so shrinking the body always
#: scores better. That column prices what a shape costs; it cannot choose one.
#: 0.20 costs three escapes in 36 against the rectangle, which is the price of
#: closing a trap that has actually been watched to happen.
#:
#: If the body is ever properly measured the number is `hypot(length/2,
#: width/2)`. Near 0.23-0.25 and this house is too tight for a circle -- the
#: doorway escapes fall away -- and the rectangle wins on pragmatism.
ROBOT_RADIUS_CONFIGURED = 0.200


#: Which obstacle critic goes with it. `BaseObstacle` refuses at 253, so the
#: controller may not put its centre in the ring at all; `ObstacleFootprint`
#: traces the outline and refuses only on contact at 254. The first is far
#: stricter and is what a circular body takes, and it is only a collision test
#: at all while the ring is as big as the whole robot -- which is why the
#: rectangle needed `ObstacleFootprint` and why this is correct now.
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


# Copied from config/nav2.yaml. Nav2's isValidSpeed is an AND of the two, so a
# theta floor with xy left at zero does not drop anything.
MIN_SPEED_XY = 0.1


MIN_SPEED_THETA = 0.21


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
