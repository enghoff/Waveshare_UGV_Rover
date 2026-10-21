#!/usr/bin/env python3
"""The rover's place on its map, kept scan by scan: the one publisher of `map -> odom`.

    python3 localizer_node.py        (started by slam.launch.py)

**Why the mapper no longer says where the rover is.** slam_toolbox in mapping
mode places each new scan against the rover's own recent scans, so an error that
gets into those -- a few degrees left over from a turn, odometry counting wheel
travel the rover never made -- is carried along rather than taken out. On
2026-10-21 that left the rover 20-26 cm sideways after one turn and 8 m, and
0.8 m out after a wheel slip, and both times the mapper never pulled it back.
`localizer.Tracker` matches every fifth of a second against the map itself, and
on the same recordings held to a few centimetres; see localizer.py and
docs/plans/continuous-localization.md.

**The mapper still maps, and the map keeps updating itself.** It is now fed this
node's answer as its odometry: slam_toolbox's `odom_frame` is `map` and its own
frame is `slam_map` (config/slam_toolbox.yaml), so the `map -> base_link` it reads
is the tracked pose and the scans it adds land where the rover really was. It
publishes nothing on the transform tree (`transform_publish_period: 0`) and its
grid on `slam_map`, which this node republishes on `map` under the `map` frame,
so every reader of the map is unchanged.

**A mapper that parts company with this answer is stopped from drawing.** If
its own estimate (`/pose`, in `slam_map`) and the tracked pose disagree by more
than `GUARD_M` or `GUARD_DEG` for `GUARD_S`, it is asked to pause new
measurements, because every scan it then adds is a wall in the wrong place. It
stays paused until navigation is restarted, which re-anchors it from the saved
map and costs the rover nothing, since its position is this node's.

**What it takes from outside is a pose on `initialpose`**, from the map keeper's
restore or from a person's refit, in the `map` frame; that is the only way the
answer moves further than the tracker's window. Until one arrives the correction
is the identity, which is what a mapper starting a fresh map assumes too.
"""

import argparse
import json
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from slam_toolbox.srv import Pause
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

import frontier
import localizer

#: Scans matched a second. The lidar gives ten; the mapper itself processes five
#: (`minimum_time_interval`), and a step is 9 ms on a workstation.
RATE_HZ = 5.0
#: How often `map -> odom` is republished, and how far ahead it is stamped so
#: that a reader asking for "now" never has to extrapolate past it -- AMCL's
#: `transform_tolerance`. 20 Hz is what slam_toolbox published.
TF_HZ = 20.0
POST_DATE_S = 0.1

#: The mapper's own estimate may differ from the tracked pose by this much, for
#: this long, before it is stopped from adding to the map.
GUARD_M = 0.30
GUARD_DEG = 8.0
GUARD_S = 3.0


def yaw_deg(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def inverse(p):
    t = math.radians(p[2])
    c, s = math.cos(t), math.sin(t)
    return (-c * p[0] - s * p[1], s * p[0] - c * p[1], -p[2])


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class LocalizerNode(Node):
    def __init__(self, args):
        super().__init__("localizer")
        self.args = args
        self.lock = threading.Lock()
        self.tracker = localizer.Tracker()
        #: map -> odom, as (x, y, deg). The identity until a pose is handed over.
        self.correction = (0.0, 0.0, 0.0)
        self.started = False
        self.scan = None
        self.last_step = None
        self.map_info = None
        self.guard_since = None
        self.mapper_paused = False
        self.mapper_off = None
        self.counts = {"steps": 0, "matched": 0, "kept": 0}

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.tf_out = TransformBroadcaster(self)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, args.map_topic, latched)
        self.status_pub = self.create_publisher(String, "localizer/status", 10)
        group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(LaserScan, args.scan_topic, self.on_scan,
                                 qos_profile_sensor_data, callback_group=group)
        self.create_subscription(OccupancyGrid, args.slam_map_topic, self.on_map, latched,
                                 callback_group=group)
        self.create_subscription(PoseWithCovarianceStamped, "initialpose", self.on_initial_pose,
                                 latched, callback_group=group)
        self.create_subscription(PoseWithCovarianceStamped, args.slam_pose_topic,
                                 self.on_slam_pose, 10, callback_group=group)
        self.pause_client = self.create_client(Pause, "/slam_toolbox/pause_new_measurements")
        self.create_timer(1.0 / TF_HZ, self.publish_tf)
        self.create_timer(1.0, self.publish_status)
        threading.Thread(target=self.work, name="localizer", daemon=True).start()
        self.get_logger().info("localizer: owning %s -> %s, matching %s against %s at %.0f Hz"
                               % (args.map_frame, args.odom_frame, args.scan_topic,
                                  args.slam_map_topic, RATE_HZ))

    # --- inputs -----------------------------------------------------------------

    def odom_at(self, t=None):
        """odom -> base_link as (x, y, deg), at `t` (a rclpy Time) or the latest."""
        try:
            at = self.tf_buffer.lookup_transform(
                self.args.odom_frame, self.args.base_frame,
                t if t is not None else Time(), rclpy.duration.Duration(seconds=0.1))
        except Exception:
            return None
        v = at.transform.translation
        return (v.x, v.y, yaw_deg(at.transform.rotation))

    def on_scan(self, msg):
        with self.lock:
            self.scan = msg

    def on_map(self, msg):
        # The mapper republishes its grid every two seconds whether or not it
        # changed, and rebuilding the field for an unchanged one was most of
        # this node's cost while parked.
        info = msg.info
        key = (info.width, info.height, info.origin.position.x, info.origin.position.y,
               hash(bytes(np.asarray(msg.data, dtype=np.int8))))
        if key == getattr(self, "map_key", None):
            return
        self.map_key = key
        out = OccupancyGrid()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.args.map_frame
        out.info = msg.info
        out.data = msg.data
        self.map_pub.publish(out)
        if abs(info.origin.orientation.z) > 1e-6:
            self.get_logger().warn("localizer: the map is rotated; it is used as if it were not")
        data = np.asarray(msg.data, dtype=np.int8)
        grid = frontier.Grid(info.width, info.height, info.resolution,
                             info.origin.position.x, info.origin.position.y, data)
        with self.lock:
            self.tracker.set_map(grid)
            self.map_info = (info.width, info.height, round(info.resolution, 3))
            if not self.started:
                # A fresh map: the mapper starts its frame where odometry is, and
                # so does this until somebody hands over a pose.
                odom = self.odom_at()
                if odom is not None:
                    self.tracker.set_pose(localizer.compose(self.correction, odom), odom)
                    self.started = True

    def on_initial_pose(self, msg):
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_deg(p.orientation))
        t = Time.from_msg(msg.header.stamp) if stamp_s(msg.header.stamp) > 0 else None
        odom = self.odom_at(t) or self.odom_at()
        if odom is None:
            self.get_logger().warn("localizer: a pose was handed over but odometry has none "
                                   "to anchor it to; ignored")
            return
        with self.lock:
            self.tracker.set_pose(pose, odom)
            self.correction = localizer.compose(pose, inverse(odom))
            self.started = True
            self.guard_since = None
        self.get_logger().info("localizer: put at (%.2f, %.2f, %.1f deg)" % pose)

    def on_slam_pose(self, msg):
        """The mapper's own estimate, in its own frame: compared with ours."""
        p = msg.pose.pose
        theirs = (p.position.x, p.position.y, yaw_deg(p.orientation))
        odom = self.odom_at(Time.from_msg(msg.header.stamp))
        if odom is None:
            return
        with self.lock:
            if not self.started:
                return
            ours = localizer.compose(self.correction, odom)
        off_m = math.hypot(theirs[0] - ours[0], theirs[1] - ours[1])
        off_deg = abs((theirs[2] - ours[2] + 180.0) % 360.0 - 180.0)
        self.mapper_off = (round(off_m, 3), round(off_deg, 1))
        now = time.monotonic()
        if off_m <= GUARD_M and off_deg <= GUARD_DEG:
            self.guard_since = None
            return
        if self.guard_since is None:
            self.guard_since = now
        if now - self.guard_since >= GUARD_S and not self.mapper_paused:
            self.pause_mapper("its own estimate is %.0f cm and %.0f degrees from the tracked "
                              "pose" % (100 * off_m, off_deg))

    def pause_mapper(self, why):
        if not self.pause_client.service_is_ready():
            return
        # A toggle, and it is only ever pressed once from here: there is no
        # resume, because a mapper whose own frame has slid off the map cannot be
        # trusted again until it is re-anchored, and the clean way to do that is a
        # navigation restart, which no longer costs the rover its position.
        self.pause_client.call_async(Pause.Request())
        self.mapper_paused = True
        self.get_logger().warn("localizer: the mapper was paused, %s, so that it adds no "
                               "walls in the wrong place; the map stops updating until "
                               "navigation is restarted" % why)

    # --- the work ---------------------------------------------------------------

    def work(self):
        period = 1.0 / RATE_HZ
        last = None
        while rclpy.ok():
            began = time.monotonic()
            with self.lock:
                scan = self.scan
                ready = self.started and self.tracker.grid is not None
            if scan is not None and ready and scan is not last:
                last = scan
                try:
                    self.step(scan)
                except Exception as error:                 # a loop: never past here
                    self.get_logger().warn("localizer: %s: %s" % (type(error).__name__, error))
            time.sleep(max(0.0, period - (time.monotonic() - began)))

    def step(self, scan):
        odom = self.odom_at(Time.from_msg(scan.header.stamp))
        if odom is None:
            return
        points = localizer_points(scan)
        with self.lock:
            step = self.tracker.update(points, odom, stamp_s(scan.header.stamp))
            if step is None:
                return
            self.correction = localizer.compose(step.pose, inverse(odom))
            self.last_step = step
            self.counts["steps"] += 1
            self.counts["matched" if step.accepted else "kept"] += 1

    # --- outputs ----------------------------------------------------------------

    def publish_tf(self):
        with self.lock:
            c = self.correction
        t = TransformStamped()
        t.header.stamp = (self.get_clock().now()
                          + rclpy.duration.Duration(seconds=POST_DATE_S)).to_msg()
        t.header.frame_id = self.args.map_frame
        t.child_frame_id = self.args.odom_frame
        t.transform.translation.x = float(c[0])
        t.transform.translation.y = float(c[1])
        half = math.radians(c[2]) / 2.0
        t.transform.rotation.z = math.sin(half)
        t.transform.rotation.w = math.cos(half)
        self.tf_out.sendTransform(t)

    def publish_status(self):
        with self.lock:
            step = self.last_step.as_dict() if self.last_step is not None else None
            out = {"started": self.started, "map": self.map_info, "last": step,
                   "counts": dict(self.counts), "mapper_off": self.mapper_off,
                   "mapper_paused": self.mapper_paused}
        self.status_pub.publish(String(data=json.dumps(out)))


def localizer_points(scan):
    """A LaserScan as (n, 2) points in the rover's frame, with refit's rules
    about what counts as a return."""
    r = np.asarray(scan.ranges, dtype=np.float64)
    a = scan.angle_min + scan.angle_increment * np.arange(len(r))
    low = max(float(scan.range_min), 0.15)
    high = min(float(scan.range_max) or 8.0, 8.0)
    keep = np.isfinite(r) & (r > low) & (r < high)
    return np.c_[r[keep] * np.cos(a[keep]), r[keep] * np.sin(a[keep])]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--odom-frame", default="odom")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--slam-map-topic", default="slam_map")
    ap.add_argument("--slam-pose-topic", default="/pose")
    ap.add_argument("--map-topic", default="map")
    args, _ros = ap.parse_known_args()
    rclpy.init()
    node = LocalizerNode(args)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
