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

`explore` is the one op that is not a single action, and it does not break that
rule so much as stack on it: it looks at the map for the nearest worthwhile gap
in it, sends the rover there with `NavigateToPose`, and does it again until there
are no gaps left. Choosing the gap is `frontier.py`; every metre of the driving
is still Nav2's.

The protocol. One request per line, and every reply carries a `kind`:

    -> {"op": "status", "since_seq": 12}
    <- {"kind": "reply", "ok": true, ...}

    -> {"op": "drive", "distance_m": 1.0, "speed_ms": 0.3}
    <- {"kind": "progress", "phase": "driving", "travelled_m": 0.2, ...}
    <- {"kind": "progress", "phase": "driving", "travelled_m": 0.7, ...}
    <- {"kind": "outcome", "reason": "arrived", "travelled_m": 1.0, ...}

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
from geometry_msgs.msg import Twist
from nav2_msgs.action import (BackUp, ComputePathToPose, DriveOnHeading,
                              NavigateToPose, Spin)
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

# Likewise, and for the stronger version of the same reason: this one is a
# geometry test rather than a table, and a drifted copy of it would be a rover
# that thinks it fits somewhere it does not.
# And likewise, for the same reason again: which gap in the map is worth driving
# to is grid arithmetic with no ROS in it, and the selftest argues with it against
# a real map saved off the rover rather than against a second copy of it.
import frontier
# And likewise: what a route costs to drive is what the time allowance is built
# from, and dwb_bench.py reports the same figure before anybody drives, so the
# two have to be one function.

from nav_limits import (
    EXPLORE_BUDGET_S, GRID_CELLS, SCAN_STALE_S, TRAIL_MAX, TRAIL_STEP_M,
    TRANSFORM_STALE_S, duration, wrap, yaw_of,
)
from nav_explore import NavExplore
from nav_moves import NavMoves

# Loopback, for the same reason board_bridge.py is: this hands out the wheels,
# and nothing on it authenticates. 8769 is the daemon, 8770 the depth camera,
# 8771 the drive console, 8772 the board going the other way.
HOST = "127.0.0.1"
PORT = 8773


class NavBridge(NavMoves, NavExplore, Node):
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

        # Throwing the map away is one call to the mapper's own reset service.
        # This briefly held a client for each of two mappers and asked whichever
        # answered, which was the right shape while there were two; there is one
        # again, and a client for a service nothing advertises is a way for
        # `clear_map` to fail slowly instead of at once.
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
        # Not in `actions`, because that dict is what `run_goal` drives and this
        # is not a move: it asks the planner for a route and nothing turns a
        # wheel. `explore` uses it to find out whether a frontier is reachable
        # before committing the rover to a minute of driving towards it.
        self.plan_client = ActionClient(self, ComputePathToPose,
                                        "compute_path_to_pose",
                                        callback_group=self.group)

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
        # Separate from `driving`, which means the wheels have a goal right now.
        # An explore is a sequence of goals with thinking in between, so `driving`
        # goes false several times during one, and a console reading it alone
        # would show a rover that had stopped every time it chose where to go
        # next.
        self.exploring = False
        self.estop = False
        self.remaining_m = None
        self.active_goal = None
        self.cancelled = False
        # How many times a stop has been asked for, ever. `cancelled` cannot
        # answer that question, because every move clears it on the way in --
        # which is right for a move, and wrong for `explore`, which is a run of
        # several moves and has to notice a stop that landed between two of them.
        # See `explore`, where a swallowed stop meant a rover setting off again.
        self.stop_seq = 0

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
            exploring = self.exploring
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
            "exploring": exploring,
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
            self.stop_seq += 1
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


    # --- exploring ------------------------------------------------------------


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
        elif op in ("drive", "turn", "goto", "explore"):
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
            elif op == "explore":
                outcome = node.explore(
                    say,
                    budget_s=float(request.get("budget_s") or EXPLORE_BUDGET_S),
                    min_frontier_m=(
                        None if request.get("min_frontier_m") is None
                        else float(request["min_frontier_m"])))
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
