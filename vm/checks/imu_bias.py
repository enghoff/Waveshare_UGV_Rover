#!/usr/bin/env python3
"""Measure what the gyro and rf2o each report while the rover is NOT moving.

This is the measurement that decides whether fusing the IMU is worth anything.
A MEMS gyro has a constant offset, and integrating it produces heading drift
exactly the way rf2o's scan-matching noise does. If the BMI270's bias is larger
than rf2o's drift then fusing it unchanged makes the estimate worse, not better,
and the bias has to be subtracted before the EKF ever sees it.

Reports, for a stationary rover:
  gyro z   mean -> the bias, in deg/s and the deg/min of heading it invents
           std  -> the noise the EKF can actually average away
  rf2o     mean angular z, for the same window, as the thing to beat

KEEP THE ROVER STILL for the whole window, and stay out of the lidar's view --
a person shifting weight nearby moves rf2o but not the gyro, which corrupts the
comparison in rf2o's favour.
"""

import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu

WINDOW_S = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0


class Collector(Node):
    def __init__(self):
        super().__init__("imu_bias")
        self.gyro = []
        self.accel = []
        self.rf2o_w = []
        self.rf2o_vx = []
        self.odom_cov_nonzero = None
        self.create_subscription(
            Imu, "/oak/imu/data", self.on_imu, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_subscription(Odometry, "/odom_rf2o", self.on_odom, 10)

    def on_imu(self, m):
        self.gyro.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
        self.accel.append(
            (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)
        )

    def on_odom(self, m):
        self.rf2o_w.append(m.twist.twist.angular.z)
        self.rf2o_vx.append(m.twist.twist.linear.x)
        if self.odom_cov_nonzero is None:
            self.odom_cov_nonzero = any(abs(c) > 0.0 for c in m.twist.covariance)


def stats(xs):
    n = len(xs)
    mean = sum(xs) / n
    var = sum((x - mean) ** 2 for x in xs) / max(n - 1, 1)
    return mean, math.sqrt(var)


def main():
    rclpy.init()
    node = Collector()
    end = node.get_clock().now().nanoseconds + int(WINDOW_S * 1e9)
    print(f"collecting {WINDOW_S:.0f} s -- keep the rover still")
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    if not node.gyro:
        print("NO IMU DATA. The IMU comes from the OAK, so the camera must be running.")
        return 1
    print(f"samples: imu {len(node.gyro)}, rf2o {len(node.rf2o_w)}")

    print("=== gyro bias (stationary) ===")
    for axis, i in (("x", 0), ("y", 1), ("z", 2)):
        mean, sd = stats([g[i] for g in node.gyro])
        line = f"  {axis}: mean {math.degrees(mean):+8.4f} deg/s   sd {math.degrees(sd):7.4f} deg/s"
        if axis == "z":
            line += f"   -> {math.degrees(mean) * 60:+.1f} deg/min of invented heading"
        print(line)

    print("=== accel (sanity: |a| should be ~9.81) ===")
    mags = [math.sqrt(sum(c * c for c in a)) for a in node.accel]
    mean, sd = stats(mags)
    print(f"  |a| mean {mean:.3f} m/s^2   sd {sd:.3f}")

    # A stationary accelerometer reads +g along whichever of its own axes points
    # up, so the gravity vector names the vertical axis -- and that is the axis
    # whose gyro channel measures yaw in the world. Worth deriving rather than
    # assuming: depthai stamps the IMU with oak_imu_frame, a frame its own URDF
    # never publishes, so nothing else in the system states the chip's mounting.
    g = [stats([a[i] for a in node.accel])[0] for i in range(3)]
    print(f"  gravity in IMU axes: x {g[0]:+.3f}  y {g[1]:+.3f}  z {g[2]:+.3f}")
    up = max(range(3), key=lambda i: abs(g[i]))
    print(
        f"  -> IMU {'+' if g[up] > 0 else '-'}{'xyz'[up]} points UP; "
        f"yaw rate is gyro {'xyz'[up]}"
        f"{' (negated)' if g[up] < 0 else ''}"
    )
    tilt = math.degrees(math.acos(min(1.0, abs(g[up]) / mean)))
    print(f"  off-vertical tilt of that axis: {tilt:.1f} deg")

    if node.rf2o_w:
        print("=== rf2o, same window (the thing to beat) ===")
        mean, sd = stats(node.rf2o_w)
        print(
            f"  angular z: mean {math.degrees(mean):+8.4f} deg/s  sd {math.degrees(sd):7.4f}"
            f"   -> {math.degrees(mean) * 60:+.1f} deg/min"
        )
        mean, sd = stats(node.rf2o_vx)
        print(f"  linear  x: mean {mean:+.5f} m/s     sd {sd:.5f}   -> {mean * 60:+.3f} m/min")
        print(f"  twist covariance populated: {node.odom_cov_nonzero}")

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
