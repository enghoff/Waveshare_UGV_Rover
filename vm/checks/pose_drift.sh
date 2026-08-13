#!/usr/bin/env bash
# Drift of the robot's estimated pose in the MAP, which is the number that
# actually matters: odom drift is forgivable if SLAM corrects it, so measure
# map -> base_link rather than odom -> base_link.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

python3 - <<'PYEOF'
import math
import time

import rclpy
import tf2_ros
from rclpy.node import Node


class P(Node):
    def __init__(self):
        super().__init__("pose_drift")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)


rclpy.init()
n = P()
samples = []
t0 = time.monotonic()
while time.monotonic() - t0 < 40.0:
    rclpy.spin_once(n, timeout_sec=0.2)
    try:
        tf = n.buf.lookup_transform("map", "base_link", rclpy.time.Time())
    except Exception:
        continue
    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    samples.append((tf.transform.translation.x, tf.transform.translation.y, yaw))

if len(samples) < 5:
    print(f"only {len(samples)} samples -- is map -> base_link published?")
else:
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    yaws = [math.degrees(s[2]) for s in samples]
    el = time.monotonic() - t0
    span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    print(f"samples {len(samples)} over {el:.0f} s")
    print(f"position spread: {span * 1000:.0f} mm   "
          f"net {math.hypot(xs[-1] - xs[0], ys[-1] - ys[0]) * 1000:.0f} mm")
    print(f"yaw spread:      {max(yaws) - min(yaws):.2f} deg   "
          f"net {yaws[-1] - yaws[0]:+.2f} deg")
rclpy.shutdown()
PYEOF
