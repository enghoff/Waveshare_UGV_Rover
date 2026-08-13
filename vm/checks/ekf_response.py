#!/usr/bin/env python3
"""Is the EKF actually tracking the gyro, or just reporting zero?

A stationary rover makes these two indistinguishable: a filter fusing a
well-calibrated gyro reports no rotation, and so does a filter silently dropping
every IMU message -- which is the failure mode here, because robot_localization
discards measurements whose frame it cannot transform and says nothing about it.
"Zero drift" is therefore not evidence of anything on its own.

So compare the signals instead of the totals. The gyro's yaw channel is its y
axis (checks/imu_bias.py shows +y is the vertical one), and the filter's output
is twist.angular.z in base_link. If the filter is genuinely consuming the IMU,
those two track each other sample for sample, including the noise. If it is
ignoring it, the output is flat while the input is not.
"""

import math
import statistics
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0


class R(Node):
    def __init__(self):
        super().__init__("ekf_response")
        self.imu = []
        self.ekf = []
        self.create_subscription(
            Imu, "/imu/data_unbiased", self.on_imu, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_subscription(Odometry, "/odometry/filtered", self.on_ekf, 20)

    def on_imu(self, m):
        self.imu.append(m.angular_velocity.y)

    def on_ekf(self, m):
        self.ekf.append(m.twist.twist.angular.z)


def main():
    rclpy.init()
    n = R()
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.2)

    if not n.imu or not n.ekf:
        print(f"missing data: imu {len(n.imu)}, ekf {len(n.ekf)}")
        return 1

    isd = math.degrees(statistics.pstdev(n.imu))
    esd = math.degrees(statistics.pstdev(n.ekf))
    print(f"gyro y  (input) : sd {isd:.4f} deg/s   n={len(n.imu)}")
    print(f"ekf   z (output): sd {esd:.4f} deg/s   n={len(n.ekf)}")

    if esd < 1e-4:
        print("VERDICT: output is flat -- the EKF is NOT consuming the gyro.")
        return 1
    ratio = esd / isd if isd else float("inf")
    print(f"output/input noise ratio: {ratio:.2f}")
    print(
        "VERDICT: the filter is tracking the gyro. A ratio below 1 is expected "
        "and healthy -- that is the filter smoothing, not ignoring."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
