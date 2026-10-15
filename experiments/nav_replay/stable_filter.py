"""Scans with the returns from things that move taken out: run on the rover,
on the replay's domain, beside graph_replay.sh.

    python3 stable_filter.py STABLE.npz

Subscribes to the recording's scans (played as /scan_bag) and publishes /scan
for the replay mapper with every return set to no-return unless it lands
within NEAR_M of a stable cell (stable_cells.py) or somewhere the recordings
never saw. Where a return lands is worked out at the pose the replay mapper
itself believes at the scan's stamp (map -> laser through tf), as a live
filter would have to: the recording's own correction is the thing under test.
Before the replay mapper publishes a pose, scans pass unchanged.
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros

NEAR_CELLS = 2  # within 0.1 m of a stable cell
SEEN_MIN = 3


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Filter(Node):
    def __init__(self, path):
        super().__init__("stable_filter", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        d = np.load(path)
        stable = d["stable"]
        grown = stable.copy()
        for di in range(-NEAR_CELLS, NEAR_CELLS + 1):
            for dj in range(-NEAR_CELLS, NEAR_CELLS + 1):
                grown |= np.roll(np.roll(stable, di, 0), dj, 1)
        unseen = d["seen"] < SEEN_MIN
        self.keep = grown | unseen
        self.res, self.ox, self.oy = float(d["res"]), float(d["ox"]), float(d["oy"])
        self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.pub = self.create_publisher(LaserScan, "/scan", 50)
        # The recording plays its scans best-effort, as the lidar driver sends them.
        self.create_subscription(LaserScan, "/scan_bag", self.on_scan, qos_profile_sensor_data)
        self.n, self.dropped, self.passed = 0, 0, 0

    def on_scan(self, msg):
        self.n += 1
        try:
            t = self.buf.lookup_transform("map", msg.header.frame_id, Time.from_msg(msg.header.stamp),
                                          timeout=Duration(seconds=0.05))
        except Exception:
            self.passed += 1
            self.pub.publish(msg)
            return
        x, y = t.transform.translation.x, t.transform.translation.y
        th = yaw_of(t.transform.rotation)
        r = np.array(msg.ranges, dtype=np.float32)
        ang = msg.angle_min + msg.angle_increment * np.arange(len(r)) + th
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        hx, hy = x + r * np.cos(ang), y + r * np.sin(ang)
        j = np.clip(((hx - self.ox) / self.res).astype(int), 0, self.keep.shape[1] - 1)
        i = np.clip(((hy - self.oy) / self.res).astype(int), 0, self.keep.shape[0] - 1)
        drop = ok & ~self.keep[i, j]
        r[drop] = float("inf")
        self.dropped += int(drop.sum())
        msg.ranges = r.tolist()
        self.pub.publish(msg)
        if self.n % 600 == 0:
            self.get_logger().info("scans %d, unfiltered %d, returns dropped %d" % (self.n, self.passed, self.dropped))


def main():
    rclpy.init()
    node = Filter(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
