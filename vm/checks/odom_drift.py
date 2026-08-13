#!/usr/bin/env python3
"""How far does each odometry source think a stationary rover has travelled?

    checks/odom_drift.py [seconds]        default 120

Run it while the rover is genuinely still, with odom_source:=ekf. Both estimates
are then published at once -- /odom_rf2o unfused, /odometry/filtered fused with
the gyro -- so this is a controlled comparison rather than two runs in different
conditions, which for a scan matcher in a room where nothing is guaranteed to
stay put is the difference between a measurement and an anecdote.

Drift is unavoidable in dead reckoning; the question is only whether it is slow
enough to be a useful prior for slam_toolbox. Much more than a degree a minute
while genuinely still and the map smears.
"""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
SOURCES = [("rf2o (unfused)", "/odom_rf2o"), ("EKF (fused)", "/odometry/filtered")]

# Above this the rover really turned, and the run is not a drift measurement at
# all. The de-biased gyro's stationary noise is about 0.09 deg/s, so 1.0 is more
# than ten standard deviations -- comfortably clear of noise, while still
# catching a nudge far too small to notice by eye.
MOVED_DEG_S = 1.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Drift(Node):
    def __init__(self):
        super().__init__("odom_drift")
        self.state = {}
        for label, topic in SOURCES:
            self.state[label] = {"first": None, "last": None, "n": 0, "yaw_unwrapped": 0.0}
            self.create_subscription(
                Odometry, topic, lambda m, l=label: self.cb(l, m), 10
            )
        # The independent witness. rf2o cannot tell its own drift from the rover
        # being nudged, and neither can the filter downstream of it -- but the
        # gyro can, because it responds to the rover turning and to nothing else
        # in the room.
        self.gyro_peak = 0.0
        self.gyro_n = 0
        self.create_subscription(
            Imu, "/imu/data_unbiased", self.on_imu, QoSPresetProfiles.SENSOR_DATA.value
        )

    def on_imu(self, msg):
        self.gyro_peak = max(self.gyro_peak, abs(msg.angular_velocity.y))
        self.gyro_n += 1

    def cb(self, label, msg):
        s = self.state[label]
        p = msg.pose.pose
        yaw = yaw_of(p.orientation)
        if s["first"] is None:
            s["first"] = (p.position.x, p.position.y, yaw)
            s["prev_yaw"] = yaw
        # Unwrap: a heading that crosses +/-pi would otherwise register as a 360
        # degree jump and make a badly drifting source look stable.
        d = yaw - s["prev_yaw"]
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        s["yaw_unwrapped"] += d
        s["prev_yaw"] = yaw
        s["last"] = (p.position.x, p.position.y, yaw)
        s["n"] += 1
        # Track the excursion, not just the endpoints. A net drift of zero can
        # mean a well-behaved estimate or a state that is not moving at all, and
        # those need telling apart -- see checks/ekf_response.py for why.
        s["yaw_min"] = min(s.get("yaw_min", 0.0), s["yaw_unwrapped"])
        s["yaw_max"] = max(s.get("yaw_max", 0.0), s["yaw_unwrapped"])


def main():
    rclpy.init()
    node = Drift()
    print(f"measuring {DUR:.0f} s -- keep the rover still")
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < DUR:
        rclpy.spin_once(node, timeout_sec=0.2)
    el = time.monotonic() - t0

    print(f"{'source':<18}{'msgs':>7}{'drift':>12}{'rate':>13}{'yaw':>11}{'rate':>13}")
    for label, topic in SOURCES:
        s = node.state[label]
        if s["first"] is None:
            print(f"{label:<18}{'-':>7}   no messages on {topic}")
            continue
        dist = math.hypot(s["last"][0] - s["first"][0], s["last"][1] - s["first"][1])
        dyaw = math.degrees(s["yaw_unwrapped"])
        span = math.degrees(s["yaw_max"] - s["yaw_min"])
        print(
            f"{label:<18}{s['n']:>7}"
            f"{dist * 1000:>9.1f} mm{dist / el * 60000:>9.1f} mm/min"
            f"{dyaw:>+9.3f} deg{dyaw / el * 60:>+9.3f} deg/min"
            f"   (swept {span:.3f} deg)"
        )

    peak = math.degrees(node.gyro_peak)
    if not node.gyro_n:
        print("\nno gyro seen -- cannot tell drift from real motion. Is the camera running?")
    elif peak > MOVED_DEG_S:
        print(
            f"\nINVALID: the rover TURNED during this window (gyro peaked at "
            f"{peak:.2f} deg/s). These numbers are motion, not drift -- re-run undisturbed."
        )
    else:
        print(f"\nrover was still (gyro peaked at {peak:.2f} deg/s). Drift figures are valid.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
