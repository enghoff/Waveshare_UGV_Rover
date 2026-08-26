#!/usr/bin/env python3
"""Serve the ROS 2 navigation stack to the rover daemon, over loopback.

The daemon cannot import any of this. It runs under the board's system Python
because that is what has the serial port, the camera and OpenCV; ROS 2 lives in a
conda environment with its own Python, and the two cannot be made into one
process. So the daemon reaches Nav2 the same way ROS reaches the driver board:
through a small server on loopback that speaks newline-delimited JSON.

    daemon  --8772-->  board_bridge   the encoders, the gyro, the motors
    daemon  <--8773--  this           goals, the map, the pose, the room

Those two are the whole of the interface between the two halves of this rover, and
they run in opposite directions, which is worth holding on to: 8772 is the daemon
lending hardware out, and 8773 is the daemon borrowing navigation back.

What each request maps onto is deliberately a Nav2 action rather than anything
written here. `drive` is `DriveOnHeading`, `turn_in_place` is `Spin`, `drive_to` is
`NavigateToPose` -- so the collision checking, the recovery behaviours and the
costmap are Nav2's, and this file has no control loop in it at all. That is the
point of the migration: the rover used to carry its own planner and its own
follower, and both are now somebody else's problem.

The protocol. One request per line, and every reply carries a `kind`:

    -> {"op": "status", "since_seq": 12}
    <- {"kind": "reply", "ok": true, ...}

    -> {"op": "drive", "distance_m": 1.0, "speed_ms": 0.3}
    <- {"kind": "progress", "phase": "driving", "travelled_m": 0.2, ...}
    <- {"kind": "progress", "phase": "driving", "travelled_m": 0.7, ...}
    <- {"kind": "outcome", "reason": "arrived", "travelled_m": 1.0, ...}

    -> {"op": "retarget", "x_m": 2.0, "y_m": -1.0}
    <- {"kind": "reply", "ok": true, "reason": "handed over"}

`retarget` is the one write that reaches into a running move, and it is the
reason the rover no longer pauses when somebody changes their mind. It hands
Nav2 a replacement `NavigateToPose` goal without cancelling the one in flight,
which Nav2 takes as a preemption: it swaps the target on the behaviour tree's
blackboard and never halts the tree, so the controller keeps driving the route
it already has until the planner produces the new one. See `NavBridge.retarget`.

A move is one request that can last a minute, and it narrates itself while it
runs. That is not decoration: the drive console polls the daemon three times a
second while somebody watches a move, and without the running commentary the only
thing it could show for the whole minute is a stopwatch. The daemon turns those
progress lines into the `MoveReport` its clients already know how to read.

Moves want their own connection. `status` has to be answerable *while* a move is
blocked, and the daemon opens a second connection for exactly that reason -- so
this server is threaded, with one thread per connection, and only the move ops
take the mutex.

    python3 nav_bridge.py --help
"""

import argparse
import base64
import json
import math
import socket
import socketserver
import sys
import threading
import time
import zlib

import rclpy
import rclpy.duration
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose, Spin
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import Reset
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

# Beside this file, and with no ROS in it, so that the selftest on a
# workstation reads the same table this does rather than a copy of it.
from nav_codes import phrase_for, reason_for
# Likewise, and for the stronger version of the same reason: this one is a
# geometry test rather than a table, and a drifted copy of it would be a rover
# that thinks it fits somewhere it does not.
import goal_fit
# And likewise: what a route costs to drive is what the time allowance is built
# from, and dwb_bench.py reports the same figure before anybody drives, so the
# two have to be one function.
import route_cost

# Loopback, for the same reason board_bridge.py is: this hands out the wheels,
# and nothing on it authenticates. 8769 is the daemon, 8770 the depth camera,
# 8771 the drive console, 8772 the board going the other way.
HOST = "127.0.0.1"
PORT = 8773

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


class NavBridge(Node):
    """Everything the daemon needs to know about, kept current by subscription.

    The node holds no navigation logic. It caches what arrives -- the map, the
    pose, the scan's age, what the lidar node says about the room -- and it owns
    four action clients. A request either reads the cache, which is instant, or
    sends a goal and waits, which is not.
    """

    def __init__(self, args):
        super().__init__("nav_bridge")
        self.args = args
        self.group = ReentrantCallbackGroup()
        self._lock = threading.Lock()

        # --- what the stack is saying
        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                durability=QoSDurabilityPolicy.VOLATILE,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        # The map is latched -- slam_toolbox publishes it transient-local so that a
        # subscriber joining late is given the current one rather than waiting for
        # the next update, which is two seconds away by the config and could be
        # minutes if nothing is moving. Matching that durability is what makes the
        # first map_png after a restart answer instead of failing.
        latched_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(LaserScan, "scan", self.on_scan, sensor_qos,
                                 callback_group=self.group)
        self.create_subscription(OccupancyGrid, "map", self.on_map, latched_qos,
                                 callback_group=self.group)
        self.create_subscription(Odometry, "odom", self.on_odom, 10,
                                 callback_group=self.group)
        self.create_subscription(Path, "plan", self.on_plan, 10,
                                 callback_group=self.group)
        self.create_subscription(String, "surroundings", self.on_surroundings,
                                 latched_qos, callback_group=self.group)
        self.create_subscription(String, "base_state", self.on_base_state,
                                 latched_qos, callback_group=self.group)

        # Zero velocity on a stop, published here as well as cancelling the goal.
        # Cancelling is the correct act and this is the fast one: a cancel has to
        # go to the behaviour server, be accepted, and stop the controller, and a
        # stop button should not wait for three hops when it can also just say so.
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.reset_client = self.create_client(
            Reset, "/slam_toolbox/reset", callback_group=self.group)
        # The costmap the planner is about to use, and the body the controller is
        # about to check it against. Asking the running stack for both is what
        # keeps this file from carrying a second opinion about either: the
        # footprint in particular is a measurement, and a copy of it here would go
        # stale the first time somebody re-measured the rover.
        self.costmap_client = self.create_client(
            GetCostmap, "/global_costmap/get_costmap",
            callback_group=self.group)
        self.footprint_client = self.create_client(
            GetParameters, "/global_costmap/global_costmap/get_parameters",
            callback_group=self.group)
        self.body = None

        self.actions = {
            "goto": ActionClient(self, NavigateToPose, "navigate_to_pose",
                                 callback_group=self.group),
            "spin": ActionClient(self, Spin, "spin", callback_group=self.group),
            "forward": ActionClient(self, DriveOnHeading, "drive_on_heading",
                                    callback_group=self.group),
            "back": ActionClient(self, BackUp, "backup",
                                 callback_group=self.group),
        }

        # --- the cache
        self.scan_at = None
        self.scan_count = 0
        self.map_msg = None
        self.map_at = None
        self.odom = None
        self.plan = None
        self.surroundings = None
        self.base_state = None
        self.trail = []

        # --- what a move is doing
        self.move_mutex = threading.Lock()
        self.driving = False
        self.estop = False
        self.remaining_m = None
        self.active_goal = None
        self.active_kind = None
        self.cancelled = False
        # A target that arrived while a move was already running. Held here
        # rather than sent from the connection that brought it, because every
        # call on the action client belongs to the thread running the move --
        # `run_goal` picks this up on its next pass. See `retarget`.
        self.pending_retarget = None

        self.create_timer(0.5, self.sample_trail, callback_group=self.group)
        self.get_logger().info("nav bridge ready; serving %s:%d"
                               % (args.bind, args.port))

    # --- subscriptions --------------------------------------------------------
    def on_scan(self, msg):
        with self._lock:
            self.scan_at = time.monotonic()
            self.scan_count += 1

    def on_map(self, msg):
        with self._lock:
            self.map_msg = msg
            self.map_at = time.monotonic()

    def on_odom(self, msg):
        with self._lock:
            self.odom = msg

    def on_plan(self, msg):
        with self._lock:
            self.plan = msg

    def on_surroundings(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self.surroundings = payload

    def on_base_state(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self.base_state = payload

    # --- where the rover is ---------------------------------------------------
    def dead_reckoned(self):
        """`(x, y, yaw)` in the *odom* frame, for measuring what a move did.

        Not the map frame, and the difference is not academic -- it was measured
        on the rover. `odom -> base_link` is pure dead reckoning and is never
        corrected; `map -> odom` is the pose graph's correction on top, and it
        moves in steps whenever slam_toolbox matches a scan.

        The trap is that slam_toolbox only adds a scan once the rover has
        travelled `minimum_travel_distance`, so a rover standing still
        accumulates gyro drift in `odom` with nothing correcting it -- measured
        here at about 0.1 deg/s while the bias estimate is still converging. The
        correction for all of it then lands in one step on the first scan after
        the rover moves. Measured in the map frame, a 0.3 m straight drive
        therefore reported 19 degrees of turn: the drive was being charged with
        several minutes of standing still.

        So a move is measured against dead reckoning, which is what the rover did
        relative to itself, and the map frame is used for where the rover *is*.
        """
        try:
            at = self.tf_buffer.lookup_transform(
                self.args.odom_frame, self.args.base_frame,
                rclpy.time.Time(), rclpy.duration.Duration(seconds=0.2))
        except Exception:
            return None
        t = at.transform.translation
        return t.x, t.y, yaw_of(at.transform.rotation)

    def pose(self):
        """`(x, y, yaw)` in the map frame, or None if nothing has said yet.

        Read off the transform tree rather than a topic, because that is the one
        answer both halves of the stack agree on: slam_toolbox corrects `map ->
        odom` and base_node integrates `odom -> base_link`, and the product of the
        two is where the rover is according to a map with loop closure in it. The
        pose topic slam_toolbox also publishes is the same number arriving less
        often.
        """
        try:
            at = self.tf_buffer.lookup_transform(
                self.args.map_frame, self.args.base_frame,
                rclpy.time.Time(), rclpy.duration.Duration(seconds=0.2))
        except Exception:
            return None
        t = at.transform.translation
        return t.x, t.y, yaw_of(at.transform.rotation)

    def transform_age(self):
        """How long ago `map -> base_link` was last published, or None.

        This is the honest reading of "is the position trusted": there is no
        per-scan match score to report the way the old scan matcher had, and a
        pose graph that has stopped publishing is the failure that number was
        being watched for anyway.
        """
        try:
            at = self.tf_buffer.lookup_transform(
                self.args.map_frame, self.args.base_frame, rclpy.time.Time())
        except Exception:
            return None
        stamp = at.header.stamp
        then = stamp.sec + stamp.nanosec / 1e9
        now = self.get_clock().now().nanoseconds / 1e9
        return max(0.0, now - then)

    def sample_trail(self):
        """Where the rover has been, thinned to one pose every five centimetres.

        Kept here rather than by the daemon because it is a property of the map's
        frame: a `clear_map` throws the graph away and the trail with it, and the
        two have to happen together or the next picture draws a history of a room
        that no longer exists in those coordinates.
        """
        where = self.pose()
        if where is None:
            return
        x, y, _ = where
        with self._lock:
            if self.trail:
                last = self.trail[-1]
                if math.hypot(x - last[0], y - last[1]) < TRAIL_STEP_M:
                    return
            self.trail.append((round(x, 3), round(y, 3)))
            if len(self.trail) > TRAIL_MAX:
                del self.trail[:len(self.trail) - TRAIL_MAX]

    # --- reads ----------------------------------------------------------------
    def status(self):
        """Every number the driving half of this rover has.

        The field names are the old navigator's, because the console and the voice
        console both read them by name and neither should have to know which
        planner is underneath. Several of them have no equivalent here and come
        back None, which those clients already render as a dash -- there is no
        per-revolution match score in a pose graph, and no steering angle in a
        controller that emits velocities.
        """
        with self._lock:
            scan_at, scans = self.scan_at, self.scan_count
            odom, plan, room = self.odom, self.plan, self.surroundings
            base = self.base_state
            driving, estop, remaining = self.driving, self.estop, self.remaining_m
        now = time.monotonic()
        scan_age = None if scan_at is None else now - scan_at
        tf_age = self.transform_age()
        # Two clocks, and they mean different things: the sensor can be spinning
        # happily while the pose graph has stopped, and the pair is what tells a
        # dead mapper apart from a dead lidar.
        lidar_live = scan_age is not None and scan_age < SCAN_STALE_S
        mapped = tf_age is not None and tf_age < TRANSFORM_STALE_S

        where = self.pose()
        speed = turn = None
        if odom is not None:
            speed = odom.twist.twist.linear.x
            turn = math.degrees(odom.twist.twist.angular.z)

        return {
            "driving": driving,
            "estop": estop,
            "pose": None if where is None else {
                "x_m": round(where[0], 3), "y_m": round(where[1], 3),
                "heading_deg": round(math.degrees(where[2]), 1)},
            "speed_ms": None if speed is None else round(speed, 3),
            "turn_dps": None if turn is None else round(turn, 1),
            "clearance_m": None if not room else room.get("clear_ahead_m"),
            "steering_deg": self.steering(where, plan),
            "remaining_m": remaining,
            # Nothing here scores a single scan against the map, so the panel row
            # that used to show one is empty on purpose rather than by omission.
            "match_score": None,
            "position_trusted": mapped,
            "mapping": mapped,
            "scans": scans,
            "dropped_scans": None if not room else room.get("thin"),
            "pwm": None if not base else base.get("pwm"),
            "gyro_bias_dps": None if not base else base.get("gyro_bias_dps"),
            "board_ok": None if not base else base.get("board_ok"),
            "lidar_ok": lidar_live and mapped,
            "lidar_live": lidar_live,
            "lidar_port": None if not room else room.get("port"),
            "scan_age_s": None if scan_age is None else round(scan_age, 2),
            "transform_age_s": None if tf_age is None else round(tf_age, 2),
            "map_age_s": None if self.map_at is None
                         else round(now - self.map_at, 1),
            "nav2_ready": self.actions["goto"].server_is_ready(),
        }

    def steering(self, where, plan):
        """Which way the rover is trying to go, in degrees off its own nose.

        The old number came out of a follower that scored candidate arcs and could
        therefore name the one it picked. A velocity controller has no such thing,
        so this is the next best honest answer: the bearing to the point on the
        planned route about a lookahead ahead of the rover. Absent when nothing is
        planned, which is most of the time.
        """
        if where is None or plan is None or not plan.poses:
            return None
        x, y, yaw = where
        for stamped in plan.poses:
            p = stamped.pose.position
            if math.hypot(p.x - x, p.y - y) >= 1.0:
                return round(math.degrees(wrap(
                    math.atan2(p.y - y, p.x - x) - yaw)), 1)
        # The whole route is inside the lookahead, so the end of it is the answer.
        p = plan.poses[-1].pose.position
        if math.hypot(p.x - x, p.y - y) < 0.05:
            return None
        return round(math.degrees(wrap(math.atan2(p.y - y, p.x - x) - yaw)), 1)

    def describe(self):
        """What the lidar node says is around the rover, with the real pose in it.

        The description is computed where the raw revolutions are -- see
        lidar_node.py -- because the library that can turn a scan into walls and
        gaps is already there with the scan in it. What it cannot know is where
        the rover is, since it never runs the matcher, so the pose it reports is
        the origin and is replaced here.
        """
        with self._lock:
            room = dict(self.surroundings) if self.surroundings else None
            scan_at, scans = self.scan_at, self.scan_count
        if room is None:
            return {"ok": False,
                    "error": "the lidar node has not described the room yet"}
        where = self.pose()
        if where is not None:
            room["pose"] = {"x_m": round(where[0], 3), "y_m": round(where[1], 3),
                            "heading_deg": round(math.degrees(where[2]), 1)}
        tf_age = self.transform_age()
        room["position_trusted"] = tf_age is not None and tf_age < TRANSFORM_STALE_S
        # Revolutions this bridge has seen arrive, which is the number the lidar
        # node deliberately does not send: its own counter belongs to a matcher
        # that never runs.
        room["scans"] = scans
        age = None if scan_at is None else time.monotonic() - scan_at
        room["scan_age_s"] = None if age is None else round(age, 2)
        room["lidar_ok"] = age is not None and age < SCAN_STALE_S
        if not room["lidar_ok"]:
            room["text"] = ("The lidar is not reporting, so nothing here is "
                            "current and the rover will not drive. "
                            + room.get("text", ""))
        room["ok"] = True
        return room

    def grid(self):
        """The occupancy map, as the bytes it arrived as plus where they belong.

        Sent raw rather than rendered. The daemon already has a map renderer that
        draws the rover, its track and the camera's cone and writes the caption
        that goes with the picture, and the alternative to shipping the grid is a
        second renderer that would slowly come to disagree with the first.

        zlib because most of a room-sized map is unknown or free and both are long
        runs of one byte: measured on this rover a 172x164 map goes from 28 kB to
        under 2, which is the difference between a JSON line worth streaming and
        one worth thinking about.
        """
        with self._lock:
            msg = self.map_msg
            trail = list(self.trail)
        if msg is None:
            return {"ok": False,
                    "error": "slam_toolbox has not published a map yet"}
        raw = bytes(bytearray((v & 0xFF) for v in msg.data))
        where = self.pose()
        return {
            "ok": True,
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution_m": msg.info.resolution,
            "origin_x_m": msg.info.origin.position.x,
            "origin_y_m": msg.info.origin.position.y,
            "cells": GRID_CELLS,
            "data": base64.b64encode(zlib.compress(raw, 6)).decode("ascii"),
            "pose": None if where is None else {
                "x_m": where[0], "y_m": where[1], "heading_deg":
                math.degrees(where[2])},
            "trail": trail,
        }

    # --- writes ---------------------------------------------------------------
    def halt(self, latch=False):
        """Stop, by both roads at once. Never refused and never blocked.

        Cancelling the goal is the correct act -- it stops the controller
        publishing, which is what would otherwise keep driving -- and the zero
        Twist is the quick one, because a cancel has three hops to make and a stop
        button should not wait for them. Sent three times because `/cmd_vel` is
        best-effort by the time it reaches the wheels and one lost message here is
        a rover that did not stop.
        """
        with self._lock:
            goal = self.active_goal
            self.cancelled = True
            # A stop beats a handover that has not happened yet. Without this a
            # target clicked a moment before the stop button would be picked up
            # by the move as it wound down and the rover would set off again.
            self.pending_retarget = None
            if latch:
                self.estop = True
        stop = Twist()
        for _ in range(3):
            self.cmd_pub.publish(stop)
        if goal is not None:
            try:
                goal.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn("cancel failed: %s" % exc)
        with self._lock:
            return {"stopped": True, "latched": self.estop}

    def clear_estop(self):
        with self._lock:
            self.estop = False
        return {"latched": False}

    def clear_map(self):
        """Throw the pose graph away and start again where the rover stands.

        Refused while a move is running, for the reason the old one was: the route
        being followed is a list of places in the frame this is about to discard.
        Stopping is never refused, so the answer is to stop first.

        The trail goes with it. A track drawn in coordinates that have just been
        redefined is an invented history laid over an empty room.
        """
        if not self.move_mutex.acquire(blocking=False):
            return {"cleared": False,
                    "reason": "the rover is moving, and the route it is following "
                              "is in the frame this would throw away -- stop it "
                              "first"}
        try:
            if not self.reset_client.wait_for_service(timeout_sec=2.0):
                return {"cleared": False,
                        "reason": "slam_toolbox is not answering, so there is no "
                                  "map to clear"}
            future = self.reset_client.call_async(Reset.Request())
            if not self.wait(future, 10.0):
                return {"cleared": False,
                        "reason": "slam_toolbox did not answer the reset in ten "
                                  "seconds"}
            with self._lock:
                self.trail = []
                self.map_msg = None
                self.map_at = None
            return {"cleared": True,
                    "reason": "the pose graph is empty and the rover is at its "
                              "origin"}
        finally:
            self.move_mutex.release()

    # --- moves ----------------------------------------------------------------
    def wait(self, future, limit_s):
        """Wait for a future without spinning: the executor is already doing that.

        `spin_until_future_complete` is the usual answer and is wrong here. This
        runs on a connection thread, not on the executor's, and calling spin from
        two threads at once is how rclpy deadlocks. The executor services the
        future; this only has to notice.
        """
        deadline = time.monotonic() + limit_s
        while not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.02)
        return True

    def run_goal(self, kind, goal_msg, limit_s, say, measure, motion="driving",
                 budget=None):
        """Send one Nav2 goal and narrate it until it ends.

        `say` publishes a progress line and `measure` turns the action's own
        feedback into the numbers the daemon reports, because each action counts
        something different -- degrees for a spin, metres for a drive, metres
        remaining for a navigation.

        `budget`, where there is one, is asked on every pass how many seconds the
        move now deserves, and the deadline moves out to match. `drive` and
        `turn_in_place` do not need it -- what they were asked for is what they
        will do -- but a navigation does: the route is not known when the goal is
        sent, and it is the route rather than the goal that has to be driven.

        `motion` is the word for what the rover is doing once the goal is
        accepted, and it is a parameter because the consoles read it: both turn
        the phase into a sentence, and a spin narrating itself as "driving +45
        deg" is a rover describing something it is not doing.
        """
        client = self.actions[kind]
        if not client.wait_for_server(timeout_sec=2.0):
            return {"reason": "refused", "travelled_m": 0.0, "turned_deg": 0.0,
                    "detail": "Nav2 is not running, so the rover will not drive "
                              "itself. Only the mapping half of the stack is up."}
        with self._lock:
            if self.estop:
                return {"reason": "blocked", "travelled_m": 0.0,
                        "turned_deg": 0.0,
                        "detail": "the stop is latched; clear it first"}
            self.cancelled = False

        # Dead reckoning, not the map frame: see dead_reckoned() and
        # finish() for the 19 degrees that cost.
        started = self.dead_reckoned()
        feedback = {}
        recoveries = [0]

        def on_feedback(message):
            fields = measure(message.feedback)
            # Kept outside `feedback`, which is overwritten each time: the count
            # only matters once the move has failed, and by then Nav2's last
            # feedback may have reset it.
            recoveries[0] = max(recoveries[0], int(fields.get("recoveries") or 0))
            feedback.update(fields)

        say("planning", "the goal is with Nav2")
        send = client.send_goal_async(goal_msg, feedback_callback=on_feedback)
        if not self.wait(send, 10.0):
            return {"reason": "failed", "travelled_m": 0.0, "turned_deg": 0.0,
                    "detail": "Nav2 did not answer the goal in ten seconds"}
        handle = send.result()
        if handle is None or not handle.accepted:
            return {"reason": "refused", "travelled_m": 0.0, "turned_deg": 0.0,
                    "detail": "Nav2 would not accept the goal, which usually means "
                              "the rover is standing inside something the costmap "
                              "believes in"}

        with self._lock:
            self.active_goal = handle
            self.active_kind = kind
            self.driving = True
        try:
            result_future = handle.get_result_async()
            began = time.monotonic()
            deadline = began + limit_s
            said_at = 0.0
            while True:
                # **A new target takes over without the wheels stopping.**
                # `NavigateToPose` preempts: `NavigateToPoseNavigator::onPreempt`
                # accepts the pending goal and `initializeGoalPose` only writes it
                # to the blackboard, so the tree is never halted and `FollowPath`
                # keeps driving the route it has until the planner answers with
                # the new one. Nav2 aborts the goal it replaced, which is why this
                # is checked before the result: the abort and the handover are the
                # same event, and reading the result first would report a failure.
                waiting = None
                with self._lock:
                    if self.pending_retarget is not None:
                        waiting, self.pending_retarget = self.pending_retarget, None
                if waiting is not None:
                    send = client.send_goal_async(waiting["goal"],
                                                  feedback_callback=on_feedback)
                    if self.wait(send, 10.0) and send.result() is not None                             and send.result().accepted:
                        handle = send.result()
                        result_future = handle.get_result_async()
                        with self._lock:
                            self.active_goal = handle
                        # The new target gets its own clock. Keeping the old one
                        # would have a handover late in a long move inherit a
                        # deadline that has nearly run out.
                        began = time.monotonic()
                        deadline = began + waiting["limit"]
                        say(motion, "a new target took over")
                    else:
                        say(motion, "the new target was refused, so the rover is "
                                    "still going to the old one")
                    continue
                if result_future.done():
                    break
                now = time.monotonic()
                if budget is not None:
                    # Re-asked every pass rather than once at the start, because
                    # the route does not exist yet when the goal is sent and it
                    # changes at every replan. Only ever pushed outwards: a
                    # replan that happens to come back shorter must not pull the
                    # deadline back past where the rover has already got to.
                    deadline = max(deadline, began + budget())
                if now > deadline:
                    handle.cancel_goal_async()
                    self.wait(result_future, 5.0)
                    break
                if now - said_at > PROGRESS_S:
                    said_at = now
                    say(motion, "", **dict(feedback))
                time.sleep(0.05)
            outcome = self.finish(result_future, started, feedback)
            # What it tried before giving up. A bare "blocked" sends somebody to
            # look at the rover; "blocked after 10 recoveries, and the planner
            # could not find a route" sends them to look at the map, which is
            # where the answer is.
            if recoveries[0] and outcome.get("reason") != "arrived":
                outcome["detail"] = (
                    "%s -- Nav2 gave up after %d recovery attempt%s"
                    % (outcome.get("detail") or "no route",
                       recoveries[0], "" if recoveries[0] == 1 else "s"))
        finally:
            with self._lock:
                self.active_goal = None
                self.active_kind = None
                self.driving = False
                self.remaining_m = None
                # Anything staged but never picked up dies with the move that
                # would have driven it, rather than surprising the next one.
                self.pending_retarget = None
        return outcome

    def finish(self, result_future, started, feedback):
        """What the move did, measured against dead reckoning.

        `started` is an odom-frame pose -- see `dead_reckoned` for why it must not
        be a map-frame one. Two refinements on top of the plain difference, and
        both matter:

        A wrapped heading difference cannot tell 200 degrees from -160, so where
        the behaviour has been counting rotation of its own -- `Spin` does -- its
        accumulating figure wins whenever it is the larger of the two. And a
        straight drive is credited with the distance `DriveOnHeading` measured
        rather than the straight line between its ends, because a rover that
        wandered a little covers more ground than the chord between where it
        started and where it stopped.
        """
        travelled = turned = 0.0
        ended = self.dead_reckoned()
        if started is not None and ended is not None:
            travelled = math.hypot(ended[0] - started[0], ended[1] - started[1])
            turned = math.degrees(wrap(ended[2] - started[2]))
        if "turned_deg" in feedback and abs(feedback["turned_deg"]) > abs(turned):
            turned = feedback["turned_deg"]
        if "travelled_m" in feedback and feedback["travelled_m"] > travelled:
            travelled = feedback["travelled_m"]

        with self._lock:
            cancelled = self.cancelled
        if not result_future.done():
            return {"reason": "timed out", "travelled_m": travelled,
                    "turned_deg": turned,
                    "detail": "Nav2 was still working when the time allowance ran "
                              "out, and the goal was cancelled"}
        wrapped = result_future.result()
        status = getattr(wrapped, "status", None)
        result = getattr(wrapped, "result", None)
        code = getattr(result, "error_code", 0) or 0
        message = (getattr(result, "error_msg", "") or "").strip()

        if status == GoalStatus.STATUS_SUCCEEDED and not code:
            return {"reason": "arrived", "travelled_m": travelled,
                    "turned_deg": turned}
        if status == GoalStatus.STATUS_CANCELED:
            return {"reason": "stopped" if cancelled else "timed out",
                    "travelled_m": travelled, "turned_deg": turned,
                    "detail": "a stop was asked for" if cancelled
                              else "the time allowance ran out"}
        # Code 0 is NONE, and NONE only means "arrived" beside a SUCCEEDED
        # status, which the branch above has already taken. Down here the goal
        # was aborted, and an abort that carries no code is one `bt_navigator`
        # ended without filling in a reason -- which it does when a server under
        # it stops answering. Reading the table for 0 here turned exactly that
        # into "arrived", so a rover that gave up 0.7 m into a 1.5 m drive
        # reported success, twice, while somebody was trying to work out why it
        # was not driving properly.
        if not code:
            return {"reason": "failed", "travelled_m": travelled,
                    "turned_deg": turned,
                    "detail": message or ("Nav2 abandoned the goal without saying "
                                          "why, which usually means a server under "
                                          "it stopped answering in time")}
        return {"reason": reason_for(code), "travelled_m": travelled,
                "turned_deg": turned,
                "detail": (phrase_for(code, message)
                           or "Nav2 gave up without saying why (code %s)" % code)}

    def drive(self, distance_m, speed_ms, say):
        """Straight ahead or straight back, and stop rather than hit anything.

        `DriveOnHeading` and `BackUp` are the same behaviour in two directions,
        and neither steers: they drive the heading they were given and abort with
        COLLISION_AHEAD when the costmap says the footprint would hit something.
        That is a narrower promise than the old `drive` made -- it used to weave
        around obstacles -- and the honest place to want weaving is `drive_to`,
        which has a planner behind it.
        """
        speed = abs(speed_ms or DEFAULT_SPEED_MS)
        reach = abs(distance_m)
        if distance_m < -REVERSE_LIMIT_M:
            return self.reverse_by_turning(reach, speed, say)
        limit = max(TIME_ALLOWANCE_FLOOR_S,
                    TIME_ALLOWANCE_SLACK * reach / max(speed, 0.05))
        if distance_m >= 0:
            goal = DriveOnHeading.Goal()
            kind = "forward"
        else:
            goal = BackUp.Goal()
            kind = "back"
        goal.target = Point(x=reach, y=0.0, z=0.0)
        goal.speed = float(speed)
        goal.time_allowance = duration(limit)
        return self.run_goal(
            kind, goal, limit + 5.0, say,
            lambda fb: {"travelled_m": round(abs(fb.distance_traveled), 3)})

    def reverse_by_turning(self, reach, speed, say):
        """A long way backwards, driven forwards, because the lidar faces one way.

        The rover sees with a lidar bolted on looking ahead of it, so anything
        behind it is unmapped and unwatched, and `BackUp` will drive into it at
        full speed reporting nothing wrong -- its collision check reads the same
        costmap, and the costmap behind the rover is whatever was there when it
        last faced that way. A short reverse is fine on those terms because the
        rover was looking at that ground moments ago; REVERSE_LIMIT_M is where
        that stops being true.

        So this turns round and drives forwards, which covers the same ground
        with the sensor pointed at it. The rover ends up facing the other way,
        which is the honest cost of the manoeuvre and is why the reply says so.
        """
        about = self.turn(180.0, say)
        if about.get("reason") != "arrived":
            about["detail"] = (
                "%s -- the rover was turning round first, because %0.1f m is "
                "further than it will reverse blind"
                % (about.get("detail") or "the turn did not finish", reach))
            return about
        onward = self.drive(reach, speed, say)
        onward["turned_deg"] = round(
            (about.get("turned_deg") or 0.0) + (onward.get("turned_deg") or 0.0),
            1)
        onward["detail"] = (
            "%s -- " % onward["detail"] if onward.get("detail") else "") + (
            "the rover turned round and drove forwards rather than reversing "
            "%0.1f m blind, so it is now facing the other way" % reach)
        return onward

    def turn(self, angle_deg, say):
        """On the spot, by `Spin`, which is collision-checked like everything else.

        Not refused when the rover is boxed in, unlike a navigation goal: rotating
        is how something that has got too close to a wall gets away from it, and
        Nav2's spin only aborts if the rotation itself would sweep through an
        obstacle.
        """
        limit = max(TIME_ALLOWANCE_FLOOR_S,
                    TIME_ALLOWANCE_SLACK * abs(angle_deg) / DEFAULT_TURN_DPS)
        goal = Spin.Goal()
        goal.target_yaw = float(math.radians(angle_deg))
        goal.time_allowance = duration(limit)
        return self.run_goal(
            "spin", goal, limit + 5.0, say,
            lambda fb: {"turned_deg": round(
                math.copysign(math.degrees(abs(fb.angular_distance_traveled)),
                              angle_deg), 1)},
            motion="turning")

    def footprint(self):
        """The body outline the costmap node is configured with, asked for once.

        A parameter query rather than a constant in this file, because the
        footprint is a measurement of the rover -- `lidar_slam/slam2d.c` has the
        same rectangle -- and somebody re-measuring it in config/nav2.yaml should
        not have to know that a second copy exists here. Cached after the first
        answer: costmap footprints do not change while a node is running.
        """
        if self.body is not None:
            return self.body
        if not self.footprint_client.wait_for_service(timeout_sec=1.0):
            return None
        request = GetParameters.Request()
        request.names = ["footprint", "robot_radius"]
        future = self.footprint_client.call_async(request)
        if not self.wait(future, COSTMAP_TIMEOUT_S):
            return None
        answer = future.result()
        if answer is None or len(answer.values) < 2:
            return None
        self.body = goal_fit.polygon_from(answer.values[0].string_value,
                                          answer.values[1].double_value)
        return self.body

    def costmap(self):
        """The global costmap as the planner currently holds it, or None.

        `GetCostmap` rather than the published topic on purpose: the topic sends
        one full grid and then deltas, so a subscriber that joined late or missed
        an update holds something subtly wrong, and subtly wrong is the failure
        this whole check exists to catch.
        """
        if not self.costmap_client.wait_for_service(timeout_sec=1.0):
            return None
        future = self.costmap_client.call_async(GetCostmap.Request())
        if not self.wait(future, COSTMAP_TIMEOUT_S):
            return None
        answer = future.result()
        if answer is None:
            return None
        grid = answer.map
        return goal_fit.CostGrid(grid.metadata.size_x, grid.metadata.size_y,
                                 grid.metadata.resolution,
                                 grid.metadata.origin.position.x,
                                 grid.metadata.origin.position.y,
                                 bytes(bytearray(grid.data)))

    def fit_goal(self, gx, gy, yaw):
        """Move a goal to the nearest place the rover's body will actually go.

        Returns the pose to send and a sentence about it, or None for the goal
        and a sentence saying why when there is nowhere near it that fits.

        Nav2 will not do this for itself, and the two halves of it disagree in a
        way that reads as a broken rover: NavFn plans for a point, so a cell five
        centimetres from a wall is a fine destination and it returns a clean
        straight path to it, while DWB checks the real rectangle and will not end
        a rollout there. What that looked like on the rover was twenty-five
        seconds of small heading corrections and then a timeout, with nothing
        anywhere saying the goal had been inside a wall the whole time. See
        goal_fit.py.

        A failure to ask -- the costmap service missing, the parameters not
        answering -- sends the goal unchanged. This is a check that improves a
        goal, not one the rover depends on to move, and a stack half way through
        starting up should not mean a refusal to drive.
        """
        body = self.footprint()
        grid = self.costmap() if body else None
        if grid is None:
            return (gx, gy, yaw), None
        placed = goal_fit.fit(grid, body, gx, gy, yaw)
        if placed is None:
            return None, ("there is nowhere within half a metre of that spot "
                          "where the rover's body fits -- it is inside a wall "
                          "or under something")
        if placed["moved_m"] < grid.resolution / 2.0:
            return (gx, gy, yaw), None
        return ((placed["x"], placed["y"], placed["yaw"]),
                "the spot asked for is too close to something for the rover to "
                "stand in, so the goal was moved %d cm to the nearest one it "
                "fits" % round(placed["moved_m"] * 100))

    def goal_for(self, where, yaw_deg):
        """One `NavigateToPose` goal, fitted to the map. Shared by `goto` and
        `retarget` so a target that arrives mid-drive is placed by the same
        rules as one that starts a drive.

        Returns `(goal, note, straight_line_metres)`, or `(None, (reason,
        detail), 0.0)` when there is nowhere to send the rover.
        """
        start = self.pose()
        if start is None:
            return None, ("lost", "nothing is publishing the rover's position, "
                                  "so there is no frame to drive in"), 0.0
        gx, gy = where
        if yaw_deg is None:
            yaw = math.atan2(gy - start[1], gx - start[0])
        else:
            yaw = math.radians(yaw_deg)
        placed, note = self.fit_goal(gx, gy, yaw)
        if placed is None:
            return None, ("blocked", note), 0.0
        gx, gy, yaw = placed
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.args.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal, note, math.hypot(gx - start[0], gy - start[1])

    def retarget(self, where, yaw_deg):
        """Send the rover somewhere else without stopping it first.

        **Why this is not just another move.** A second move is refused, and has
        to be: the connection carrying the first one cannot be overtaken on it,
        and two callers driving at once is two callers arguing through the
        wheels. So a console with a new destination used to stop the rover, wait
        for the wheels to come free, and then start again -- which is a visible
        pause every time somebody changes their mind.

        Nav2 does not need that. `NavigateToPose` preempts: a second goal is
        taken as a replacement for the first, `initializeGoalPose` writes it to
        the behaviour tree's blackboard and nothing halts the tree, so
        `FollowPath` carries on driving the route it already has until the
        planner answers with the new one. The wheels never stop.

        The goal is only *staged* here. Every call on the action client belongs
        to the thread running the move -- `run_goal` picks it up on its next
        pass, within a twentieth of a second -- because sending it from this
        connection would race that thread for the goal handle.
        """
        with self._lock:
            if self.estop:
                return {"ok": False, "reason": "blocked",
                        "detail": "the stop is latched; clear it first"}
            if not self.driving or self.active_kind != "goto":
                return {"ok": False, "reason": "idle",
                        "detail": "nothing is driving to a place, so there is "
                                  "nothing to redirect"}
        goal, note, straight = self.goal_for(where, yaw_deg)
        if goal is None:
            return {"ok": False, "reason": note[0], "detail": note[1]}
        with self._lock:
            # Still driving? The move may have ended while the goal was being
            # fitted, and staging it then would leave it for the *next* move.
            if not self.driving or self.active_kind != "goto":
                return {"ok": False, "reason": "idle",
                        "detail": "the move ended while the new target was "
                                  "being placed"}
            replaced = self.pending_retarget is not None
            self.pending_retarget = {"goal": goal,
                                     "limit": allowance_for(straight)}
        return {"ok": True, "reason": "handed over", "replaced": replaced,
                "detail": note or "the rover keeps driving until the new route "
                                  "is ready"}

    def goto(self, where, yaw_deg, say):
        """Somewhere on the map, with a planner and a costmap between.

        `where` is already in map coordinates -- the daemon converts an offset into
        one, because it is the daemon that knows the pose the map picture was drawn
        at. See `drive_to` in rover_nav.py for why a model is never shown map
        coordinates.

        With no `yaw_deg` the goal faces along the way it travelled, which is what
        the old planner left the rover doing and what makes a series of goals read
        as a journey rather than a set of arrivals in random directions.
        """
        goal, note, straight = self.goal_for(where, yaw_deg)
        if goal is None:
            return {"reason": note[0], "travelled_m": 0.0, "turned_deg": 0.0,
                    "detail": note[1]}

        # Generous, and a backstop rather than a schedule: Nav2 may legitimately
        # spend a while backing out of a corner and trying again, and a limit tight
        # enough to be a schedule would cancel exactly the recoveries that were
        # about to work. It starts from the straight line only because that is all
        # there is to go on before the planner has answered; `budget` below
        # replaces it with the route as soon as there is one. See
        # ROUTE_SAMPLE_M for the 3 m goal that used to time out on 8.8 m of route.
        limit = allowance_for(straight)
        # The longest route seen while this move has been running, kept rather
        # than recomputed from the current plan alone: the plan shortens as the
        # rover eats into it, and an allowance that shortened with it would
        # tighten exactly as the rover ran out of time.
        longest = [0.0, 0.0]

        def budget():
            with self._lock:
                plan = self.plan
            metres, turning = route_cost.from_path(plan)
            if metres > longest[0]:
                longest[0], longest[1] = metres, turning
            if longest[0] <= 0.0:
                return limit
            return route_cost.seconds_for(
                longest[0], longest[1], DEFAULT_SPEED_MS, ROUTE_TURN_DPS,
                slack=TIME_ALLOWANCE_SLACK, floor=limit)

        def measure(fb):
            with self._lock:
                self.remaining_m = round(float(fb.distance_remaining), 2)
                plan = self.plan
            # How many poses the planner produced, so the console can say what
            # route was accepted rather than only how far is left. Nav2's feedback
            # does not carry it; the plan it publishes does.
            return {"remaining_m": self.remaining_m,
                    "waypoints": len(plan.poses) if plan is not None else 0,
                    "route_m": round(longest[0], 2) or None,
                    "recoveries": int(fb.number_of_recoveries)}

        outcome = self.run_goal("goto", goal, limit, say, measure, budget=budget)
        # **How far the route was, said out loud.** A move that ran out of time on
        # a route three times the length of the straight line is a different event
        # from one that ran out of time going nowhere, and the console could not
        # tell them apart: both said "timed out".
        if longest[0] > straight * 1.3 and outcome.get("reason") != "arrived":
            outcome["detail"] = (
                "%s -- the route round was %.1f m for a goal %.1f m away"
                % (outcome.get("detail") or "no route", longest[0], straight))
        # Said whatever happened, including on arrival: a rover that stopped 20 cm
        # from where somebody pointed has done the right thing, and the console
        # saying so is the difference between that and a rover that missed.
        if note:
            outcome["detail"] = ("%s -- %s" % (outcome["detail"], note)
                                 if outcome.get("detail") else note)
        return outcome


class Handler(socketserver.StreamRequestHandler):
    """One connection. Reads requests, writes replies, and streams a move.

    A move blocks this thread for as long as it lasts, which is why the daemon
    opens a second connection for status: the server is threaded, and the two do
    not queue behind each other.
    """

    def handle(self):
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        node = self.server.node
        for raw in self.rfile:
            raw = raw.strip()
            if not raw:
                continue
            try:
                request = json.loads(raw)
            except ValueError:
                self.write({"kind": "reply", "ok": False, "error": "not JSON"})
                continue
            if not isinstance(request, dict):
                self.write({"kind": "reply", "ok": False,
                            "error": "a request is an object"})
                continue
            try:
                self.dispatch(node, request)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
            except Exception as exc:                 # never take the node down
                node.get_logger().warn("%s: %s" % (request.get("op"), exc))
                self.write({"kind": "reply", "ok": False,
                            "error": "%s: %s" % (type(exc).__name__, exc)})

    def write(self, payload):
        self.wfile.write(json.dumps(payload, separators=(",", ":")).encode()
                         + b"\n")

    def dispatch(self, node, request):
        op = request.get("op")
        if op == "status":
            self.write({"kind": "reply", "ok": True, **node.status()})
        elif op == "describe":
            self.write({"kind": "reply", **node.describe()})
        elif op == "map":
            self.write({"kind": "reply", **node.grid()})
        elif op == "stop":
            self.write({"kind": "reply", "ok": True,
                        **node.halt(bool(request.get("latch")))})
        elif op == "clear_estop":
            self.write({"kind": "reply", "ok": True, **node.clear_estop()})
        elif op == "clear_map":
            result = node.clear_map()
            self.write({"kind": "reply", "ok": bool(result.get("cleared")),
                        **result})
        elif op == "retarget":
            # Not a move, so not behind the move mutex: it is answered on the
            # connection that brought it while the move it redirects carries on
            # driving. Same shape as `stop` for the same reason.
            result = node.retarget((float(request["x_m"]), float(request["y_m"])),
                                   request.get("yaw_deg"))
            self.write({"kind": "reply", **result})
        elif op in ("drive", "turn", "goto"):
            self.move(node, op, request)
        else:
            self.write({"kind": "reply", "ok": False,
                        "error": "no such op: %r" % (op,)})

    def move(self, node, op, request):
        """One move, narrated. The mutex is what makes two callers safe.

        Refused rather than queued, like the old navigator did, and for its
        reason: a move that waited its turn would start by driving to somewhere
        the caller ahead of it has since made wrong.
        """
        if not node.move_mutex.acquire(blocking=False):
            self.write({"kind": "outcome", "reason": "busy", "travelled_m": 0.0,
                        "turned_deg": 0.0,
                        "detail": "a move is already running"})
            return

        def say(phase, why="", **fields):
            self.write({"kind": "progress", "phase": phase, "why": why, **fields})

        try:
            if op == "drive":
                outcome = node.drive(float(request.get("distance_m", 0.0)),
                                     request.get("speed_ms"), say)
            elif op == "turn":
                outcome = node.turn(float(request.get("angle_deg", 0.0)), say)
            else:
                outcome = node.goto((float(request["x_m"]), float(request["y_m"])),
                                    request.get("yaw_deg"), say)
        finally:
            node.move_mutex.release()
        outcome["travelled_m"] = round(float(outcome.get("travelled_m", 0.0)), 3)
        outcome["turned_deg"] = round(float(outcome.get("turned_deg", 0.0)), 1)
        self.write({"kind": "outcome", **outcome})


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--bind", default=HOST,
                   help="loopback by default, and it should stay that way")
    p.add_argument("--port", type=int, default=PORT)
    p.add_argument("--map-frame", default="map")
    p.add_argument("--odom-frame", default="odom")
    p.add_argument("--base-frame", default="base_link")
    return p.parse_args(strip_ros_args(argv))


def allowance_for(straight_m):
    """How long a goal that far away in a straight line is allowed to take.

    A backstop rather than a schedule; `budget` inside `goto` replaces it with
    the real route as soon as the planner has produced one.
    """
    return max(TIME_ALLOWANCE_MIN_ROUTE_S,
               TIME_ALLOWANCE_SLACK * straight_m / DEFAULT_SPEED_MS)


def strip_ros_args(argv):
    """Drop the `--ros-args ...` tail `ros2 launch` appends; it runs to the end of
    the command line and belongs to rclpy rather than to argparse."""
    if "--ros-args" in argv:
        return argv[:argv.index("--ros-args")]
    return list(argv)


def main():
    rclpy.init()
    args = parse_args(sys.argv[1:])
    node = NavBridge(args)

    server = Server((args.bind, args.port), Handler)
    server.node = node
    threading.Thread(target=server.serve_forever, daemon=True,
                     name="nav-bridge").start()

    # Multi-threaded, and it has to be: a move waits on an action result from a
    # connection thread while timers and subscriptions must go on running, and a
    # single-threaded executor would have the callbacks that deliver that result
    # queued behind whatever else is due. Three threads is enough for the handful
    # of low-rate callbacks here and keeps the cost off a board with four cores
    # already running a SLAM.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
