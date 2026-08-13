#!/usr/bin/env python3
"""Watch a turn-and-stop by hand, and say whether the estimate settles.

    checks/spin_watch.py [seconds]        default 90

Every check here until now measured a rover standing still, which is exactly the
condition under which the two worst failures were invisible. This one measures
the case that actually broke: rotate the rover by hand, put it down, and let it
sit. A healthy run shows rotation while you turn it and a rate returning to zero
within a second or two of you letting go. A phantom spin shows a rate that stays
non-zero -- usually with the opposite sign to the turn you just made, because the
bias tracker absorbed the turn and is now subtracting it from nothing.

Reports per second: the raw gyro, what the EKF believes, and how much heading has
accumulated since the last time the rover was judged still. The summary is the
part to read: after motion ends, how long until the rate settles, and how much
heading leaked away afterwards.

Turn it roughly 90 degrees, take about two seconds over it, then stop and stand
clear of the lidar.
"""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
MOVING = math.radians(1.0)     # rad/s, above which we call it real motion
SETTLED = math.radians(0.25)   # rad/s, below which we call the estimate at rest


class Watch(Node):
    def __init__(self):
        super().__init__("spin_watch")
        self.raw = 0.0
        self.ekf_rate = 0.0
        self.ekf_yaw = None
        self.samples = []
        self.create_subscription(
            Imu, "/imu/data_unbiased", self.on_imu, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_subscription(Odometry, "/odometry/filtered", self.on_ekf, 20)

    def on_imu(self, m):
        self.raw = m.angular_velocity.y

    def on_ekf(self, m):
        self.ekf_rate = m.twist.twist.angular.z
        q = m.pose.pose.orientation
        self.ekf_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
        )


def main():
    rclpy.init()
    n = Watch()
    t0 = time.monotonic()
    next_tick = 0.0
    print("turn the rover ~90 deg by hand, then stop and stand clear")
    print(f"{'t':>5}  {'gyro deg/s':>11}  {'ekf deg/s':>10}  {'ekf yaw':>9}")

    while rclpy.ok() and time.monotonic() - t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.05)
        el = time.monotonic() - t0
        if el >= next_tick and n.ekf_yaw is not None:
            next_tick = el + 1.0
            n.samples.append((el, n.raw, n.ekf_rate, n.ekf_yaw))
            print(
                f"{el:5.0f}  {math.degrees(n.raw):11.3f}  "
                f"{math.degrees(n.ekf_rate):10.3f}  {math.degrees(n.ekf_yaw):9.2f}"
            )

    if not n.samples:
        print("no data -- is the stack running with odom_source:=ekf?")
        return 1

    moving = [s for s in n.samples if abs(s[1]) > MOVING]
    if not moving:
        print("\nno motion seen. Turn the rover during the window, or this proves nothing.")
        return 1

    end = moving[-1][0]
    after = [s for s in n.samples if s[0] > end]
    print(f"\nmotion ended at t={end:.0f}s (peak {math.degrees(max(abs(s[1]) for s in moving)):.1f} deg/s)")

    settle = next((s[0] - end for s in after if abs(s[2]) < SETTLED), None)
    if settle is None:
        worst = max(abs(s[2]) for s in after) if after else 0.0
        print(
            f"FAIL: the estimate never settled -- still turning at "
            f"{math.degrees(worst):.3f} deg/s when the window ended. Phantom spin."
        )
        return 1

    leaked = math.degrees(after[-1][3] - after[0][3]) if len(after) > 1 else 0.0
    print(f"settled {settle:.0f} s after motion ended")
    print(f"heading that leaked away while stationary afterwards: {leaked:+.2f} deg")
    if abs(leaked) > 2.0:
        print("FAIL: too much. The bias tracker absorbed part of the turn.")
        return 1
    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
