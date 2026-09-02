#!/usr/bin/env python3
"""What the navigation bridge is held to, and the three conversions it needs.

Every number the bridge decides with, in one file with its reasoning beside it,
because three modules read them now -- the node itself, the moves and the
exploring -- and a limit that means one thing in one of them and another in the
next is a rover that behaves differently depending on which call it arrived
through.

`yaw_of`, `wrap` and `duration` are here for the same reason and no other: they
are three lines each, all three halves use them, and a second copy of `wrap` is
how a rover told to turn ten degrees turns three hundred and fifty.
"""

import math

from builtin_interfaces.msg import Duration as DurationMsg


# A scan older than this means the sensor has stopped, not slowed: at 10 Hz it is
# ten missed revolutions. Same number as lidar_slam/nav_types.py, and for the same
# reason -- everything that could move the rover checks it first.
SCAN_STALE_S = 1.0

# The map -> odom transform going stale is slam_toolbox having stopped, which is a
# different fault from the lidar having stopped and needs its own threshold. It is
# republished every 50 ms by the config, so a second is twenty missed.
TRANSFORM_STALE_S = 1.0

# How often a running move says where it has got to. Three times a second, which
# is the rate the console polls the daemon at -- faster would be lines nobody
# reads, slower would be a poll that finds nothing new.
PROGRESS_S = 0.33

# Where the rover has been, for drawing on the map. Same shape as the figures the
# old navigator kept: a pose every five centimetres, four thousand of them, which
# is two hundred metres of pottering about.
TRAIL_STEP_M = 0.05

TRAIL_MAX = 4000

# The square grid the map is presented on, so that the daemon's existing renderer
# can draw it unchanged. 800 cells at 5 cm is 40 m across with the rover's
# starting point at the middle, which is exactly the grid the daemon's own SLAM
# used -- so the console's zoom controls behave as they always did.
GRID_CELLS = 800

# Nav2 will not take a goal with no time allowance -- the behaviour server reads a
# zero as "already out of time" and returns TIMEOUT on the first cycle. So every
# move gets one, worked out from what it is being asked to do and multiplied,
# because the allowance is a backstop against a wedged rover and not a schedule.
TIME_ALLOWANCE_SLACK = 3.0

TIME_ALLOWANCE_FLOOR_S = 8.0

# What to assume when the caller does not say. Below the measured 0.33 m/s floor
# of this chassis there is no motion at all, so a "slow" default has to be a real
# speed rather than a small number.
DEFAULT_SPEED_MS = 0.35

DEFAULT_TURN_DPS = 45.0

# **A route is as long as the route, not as long as the straight line.** This is
# the number a 3 m goal used to time out on. `drive_to` budgeted its allowance
# from the distance to the goal as the crow flies, and NavFn does not fly: sent
# 2.95 m to a spot with a wall in between, it returned a perfectly correct 8.81 m
# detour -- out west, round, and back -- and the rover was cancelled 53 seconds
# into a route that needed about 42 seconds of driving and turning even if
# nothing went wrong. The console reported "timed out", which reads as a rover
# that could not find its way, and it had found its way and was driving it.
#
# So the budget is rebuilt from the route as soon as the planner publishes one,
# and again on every replan, out of the two things a route costs:
#
#   - its length, at the speed the rover really holds, and
#   - its corners. Every direction change on a skid-steer chassis is a stop and
#     a pivot, and this route had six of them; at the rate DWB pivots that is
#     about as much of the clock as the driving.
#
# Sampled at a quarter of a metre because a 5 cm grid path's heading is quantised
# to eight compass points, so measuring the turns pose by pose counts a straight
# line as a staircase and charges for 45 degrees at every step.
# The rate the controller really pivots at. DWB's rotation samples run to
# 44.7 deg/s and it picks one of the larger ones for a corner, but a corner is
# also a stop, a turn and a start, so the average rate a *route* turns at is
# lower than the peak rate a pivot reaches. Measured off the sample set the
# config offers, this is about the middle of it.
ROUTE_TURN_DPS = 27.0

# The allowance no route goes below, and it is set by Nav2's recovery ladder
# rather than by any distance. `SimpleProgressChecker` gives a move 15 seconds to
# cover 10 cm; on the second failure the behaviour tree clears both costmaps, and
# only on the third does it try the spin that might actually help. A rover
# cancelled before then has had none of the recoveries it carries -- which is what
# happened here: the log has three progress-checker failures, two costmap clears,
# and no spin, because the bridge cancelled at 53 seconds. Two windows, a spin and
# a wait is about 40 seconds, so this is the point of having recoveries at all.
TIME_ALLOWANCE_MIN_ROUTE_S = 45.0

# **How far the rover will reverse before it would rather turn round.** The lidar
# is the only thing aboard that sees where it is going and it faces forwards, so
# every centimetre of reverse is driven blind. Half a metre is a little over one
# body length -- enough to back off something it has nosed into, and short enough
# that whatever it is reversing towards was in view moments ago. Past that,
# `drive` turns the rover round and drives it forwards instead, which covers the
# same ground looking at it. The controller has no reverse at all: see
# `min_vel_x` in config/nav2.yaml.
REVERSE_LIMIT_M = 0.5

# The costmap query behind the goal check. Fetched per goal rather than cached,
# because the whole value of the check is that it uses the costmap Nav2 is about
# to plan on; a stale one would pass goals into furniture that has since been
# seen. Two seconds is generous for a 300 x 300 grid over loopback.
COSTMAP_TIMEOUT_S = 2.0

# --- exploring -----------------------------------------------------------------
# How long an `explore` runs before it stops of its own accord. Ten minutes is
# most of a floor of a house at this rover's speed, and it is a backstop rather
# than a target: exploring normally ends because there is nothing left to drive
# to, and the caller can pass its own budget. It matters that there *is* one --
# this is the only thing the rover does that keeps giving itself new work, so
# without a limit a mapper that has started hallucinating frontiers has the
# wheels until the battery goes.
EXPLORE_BUDGET_S = 600.0

# How many frontiers to price in one round before looking at the map again.
# Each rejection costs a planner call and puts that frontier on the blacklist, so
# four is four chances to find something reachable for a couple of seconds, and
# then a fresh survey rather than a longer queue of stale candidates.
#
# **It is not how many chances the run gets.** It used to be: four refusals ended
# the whole explore with "everything still unmapped is behind something the rover
# cannot get through", which is a claim about ten frontiers made from four
# planner calls, and on 2026-09-01 the rover made it with 73% of the map unknown
# and three of those four frontiers demonstrably reachable. A round that finds
# nothing now writes those four off and looks again; the run ends when the map
# has nothing left on it, not when a sample of it was refused.
EXPLORE_TRIES = 4

# How many times one run will stop and shuffle the rover before accepting that it
# is stuck. Each one is a turn and a short drive -- ten seconds or so -- and the
# situation it answers is real but not usually repeated: if three back-offs have
# not given the planner a start it will accept, a fourth is the rover pacing.
EXPLORE_SHUFFLES = 3

# The back-off itself: a slow half-speed nudge, because it is a few tens of
# centimetres onto ground the rover is already touching, and a quarter turn when
# there is nothing better to go on.
ESCAPE_SPEED_MS = 0.2

ESCAPE_TURN_DEG = 90.0

# How long to give `ComputePathToPose` to answer. This is the planner doing
# exactly the work it would do for a real goal, on a map-sized grid, so it is the
# planner's own frequency rather than a network timeout: at the configured 1 Hz a
# plan that has not appeared in five seconds is not coming.
PLAN_TIMEOUT_S = 5.0

# A frontier this much further round than it looked is one the walk across the
# map was wrong about, and worth saying out loud rather than only driving. The
# grid walk in frontier.py and the planner agree closely when the route is open;
# where they disagree it is because the planner refused a gap the walk went
# through, and it took the long way instead.
EXPLORE_DETOUR_NOTE = 1.5


def yaw_of(quaternion):
    """Yaw in radians from a quaternion that is only ever a yaw.

    The rover is on a floor, so roll and pitch are noise and the general
    conversion would only launder that noise into the answer.
    """
    z, w = quaternion.z, quaternion.w
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def wrap(radians):
    """Into (-pi, pi]. A heading difference that is not wrapped is how a rover
    told to turn ten degrees turns three hundred and fifty."""
    return math.atan2(math.sin(radians), math.cos(radians))


def duration(seconds):
    whole = int(seconds)
    return DurationMsg(sec=whole, nanosec=int((seconds - whole) * 1e9))
