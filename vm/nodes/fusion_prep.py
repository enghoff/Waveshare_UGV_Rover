#!/usr/bin/env python3
"""Make the two odometry inputs fit to hand to an EKF.

Both upstream publishers ship all-zero covariance matrices. robot_localization
reads a zero covariance as "infinitely certain", so fed raw it would trust rf2o's
rotation absolutely and the gyro would change nothing -- the fusion would appear
to run and do nothing. This node exists to fix that, and one related defect:

  1. /odom_rf2o        -> /odom_rf2o_cov      covariance stamped
  2. /oak/imu/data     -> /imu/data_unbiased  gyro bias removed, covariance
                                              stamped, header re-stamped

The re-stamping is the third defect and the one that actually stopped the fusion
working. depthai converts the OAK's device clock to ROS time against an offset
captured at startup, and VMware's tools.syncTime keeps stepping the guest clock
out from under it, so the stamps drift into the FUTURE -- measured at +0.49 s
mean, +0.91 s worst, while rf2o's arrive 0.06 s in the past. robot_localization
will not fuse a measurement dated after its own filter time, so it dropped every
IMU message silently and the EKF reported exactly zero rotation, which on a
stationary rover looks precisely like success. The driver's own
imu.i_update_ros_base_time_on_ros_msg and i_get_base_device_timestamp were both
set and verified applied; neither changed the skew.

Stamping on arrival costs the true sensor-to-stamp latency, at most a few
milliseconds at 200 Hz, and buys stamps that are monotonic with every other
clock in the system. For fusing a yaw RATE -- as opposed to integrating
acceleration, where sample timing is everything -- that trade is heavily
favourable.

The numbers come from checks/imu_bias.py against a stationary rover, and they are
the whole argument for doing this at all. Over 60 s, rf2o's yaw rate had a
standard deviation of 7.2 deg/s while the gyro's was 0.09 deg/s -- eighty times
quieter. Declaring those two honestly is what makes the EKF prefer the gyro.

The bias matters as much as the noise. The BMI270 reads a constant offset on
every axis, and integrating that is drift indistinguishable from real rotation:
the raw z channel invents 11.9 deg/min. So the first CALIBRATION_S of samples are
averaged and subtracted from everything after. That requires the rover to be
still at startup, which is checked rather than assumed -- if the sample variance
during calibration is too high the node says so and refuses to apply a bias it
cannot trust.

That startup average is necessary and not sufficient. The offset is not constant:
measured at startup on three consecutive runs it came out -0.044, -0.150 and
-0.154 deg/s, and it keeps moving as the OAK warms. Calibrating once and holding
the value gave a filter that drifted +3.5 deg/min -- worse than the rf2o it was
brought in to fix, and worse in a nastier way, because the error was smooth and
monotonic rather than noisy.

So the bias is tracked continuously, by the standard zero-velocity-update trick:
whenever the rover can be shown to be still, the current reading IS the bias, so
ease the estimate toward it. Stillness is established from two independent
sources -- the gyro's own short-term spread and rf2o's linear speed -- because
the gyro alone cannot distinguish a slow steady turn from an offset. While
anything is moving the estimate is frozen, so real rotation is never quietly
absorbed into the correction.
"""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu


class FusionPrep(Node):
    def __init__(self):
        super().__init__("fusion_prep")

        self.declare_parameter("calibration_s", 15.0)
        # Measured standard deviations, squared. Stated as variances because that
        # is what the message carries and what the filter weights by.
        self.declare_parameter("gyro_variance", 2.2e-6)      # (0.085 deg/s)^2
        self.declare_parameter("rf2o_vx_variance", 2.0e-4)   # (0.014 m/s)^2
        self.declare_parameter("rf2o_vy_variance", 1.0e-2)   # rf2o reports vy=0; weak
        self.declare_parameter("rf2o_vyaw_variance", 2.0e-2) # (8 deg/s)^2
        self.declare_parameter("max_calibration_sd", 0.02)   # rad/s
        self.declare_parameter("restamp", True)

        # Continuous bias tracking. tau is deliberately long: the bias wanders
        # over minutes, so following it faster would mostly follow noise, and
        # every bit of genuine slow rotation that slipped past the stillness gate
        # would be absorbed that much quicker.
        self.declare_parameter("track_bias", True)
        self.declare_parameter("bias_tau_s", 30.0)
        # 0.3 deg/s, about 3.5x the gyro's stationary noise: loose enough not to
        # trip on noise, tight enough that any turn worth calling a turn fails it.
        self.declare_parameter("still_gyro_sd", 0.0052)      # rad/s
        # 0.06 m/s. rf2o's stationary speed noise has sd 0.015 m/s and spikes
        # past 0.02 several times a second, so the obvious tight threshold makes
        # the gate chatter rather than gate. A person pushing the rover manages
        # about 0.2 m/s, so this sits clear of both.
        self.declare_parameter("still_speed", 0.06)          # m/s, from rf2o
        self.declare_parameter("min_dwell_s", 1.0)
        # 0.3 deg/s. Above genuine bias wander (~0.05 deg/s over minutes), far
        # below any hand rotation (tens of deg/s).
        self.declare_parameter("still_offset", 0.0052)       # rad/s
        # 0.6 deg/s of total excursion from the startup measurement.
        self.declare_parameter("max_bias_drift", 0.0105)     # rad/s

        self.cal_s = self.get_parameter("calibration_s").value
        self.gyro_var = self.get_parameter("gyro_variance").value
        self.max_cal_sd = self.get_parameter("max_calibration_sd").value
        self.restamp = self.get_parameter("restamp").value

        self.track = self.get_parameter("track_bias").value
        self.still_gyro_sd = self.get_parameter("still_gyro_sd").value
        self.still_speed = self.get_parameter("still_speed").value
        self.still_offset = self.get_parameter("still_offset").value
        self.max_bias_drift = self.get_parameter("max_bias_drift").value
        self.initial_bias = None
        self.last_clamp_warn = 0.0

        self.samples = []
        self.bias = None
        self.t0 = None
        self.skew_seen = []
        self.recent = deque(maxlen=400)   # ~2 s at 200 Hz
        self.speeds = deque(maxlen=10)    # ~1 s at 10 Hz
        self.rf2o_seen = False
        self.still = False
        self.pending = None
        self.pending_since = 0.0
        self.last_report = 0.0

        self.pub_imu = self.create_publisher(Imu, "/imu/data_unbiased", 20)
        self.pub_odom = self.create_publisher(Odometry, "/odom_rf2o_cov", 20)
        self.create_subscription(
            Imu, "/oak/imu/data", self.on_imu, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_subscription(Odometry, "/odom_rf2o", self.on_odom, 20)

        self.get_logger().info(f"calibrating gyro bias for {self.cal_s:.0f} s -- keep the rover still")

    # -- IMU ---------------------------------------------------------------
    def on_imu(self, msg):
        w = msg.angular_velocity
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.bias is None:
            if self.t0 is None:
                self.t0 = now
            self.samples.append((w.x, w.y, w.z))
            if now - self.t0 < self.cal_s:
                return
            self.finish_calibration()

        out = Imu()
        out.header = msg.header
        if self.restamp:
            skew = now - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            self.skew_seen.append(skew)
            if len(self.skew_seen) == 2000:
                worst = min(self.skew_seen)
                mean = sum(self.skew_seen) / len(self.skew_seen)
                self.get_logger().info(
                    f"re-stamping IMU: source stamps ran {-mean:+.3f} s into the "
                    f"future on average ({-worst:+.3f} s worst)"
                )
            out.header.stamp = self.get_clock().now().to_msg()

        if self.track:
            self.track_bias((w.x, w.y, w.z), now)
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        out.angular_velocity = Vector3(
            x=w.x - self.bias[0], y=w.y - self.bias[1], z=w.z - self.bias[2]
        )
        out.angular_velocity_covariance = [
            self.gyro_var, 0.0, 0.0,
            0.0, self.gyro_var, 0.0,
            0.0, 0.0, self.gyro_var,
        ]
        # Orientation is passed through untouched but explicitly marked unusable:
        # with no magnetometer there is no absolute heading, and a filter told
        # otherwise would lock onto a yaw that means nothing.
        out.orientation_covariance = [-1.0] + [0.0] * 8
        self.pub_imu.publish(out)

    def finish_calibration(self):
        n = len(self.samples)
        means = [sum(s[i] for s in self.samples) / n for i in range(3)]
        sds = [
            math.sqrt(sum((s[i] - means[i]) ** 2 for s in self.samples) / max(n - 1, 1))
            for i in range(3)
        ]
        worst = max(sds)
        if worst > self.max_cal_sd:
            self.bias = (0.0, 0.0, 0.0)
            # Still needs a reference for the clamp, and zero is the honest one:
            # nothing was established, so tracking is allowed to find the offset
            # from scratch within the usual excursion limit.
            self.initial_bias = (0.0, 0.0, 0.0)
            self.get_logger().error(
                f"NOT still during calibration (worst sd {math.degrees(worst):.3f} deg/s > "
                f"{math.degrees(self.max_cal_sd):.3f}). Publishing UNCORRECTED gyro -- "
                "restart with the rover stationary."
            )
            return
        self.bias = tuple(means)
        self.initial_bias = tuple(means)
        self.get_logger().info(
            "gyro bias removed (deg/s): "
            + "  ".join(f"{a} {math.degrees(m):+.4f}" for a, m in zip("xyz", means))
            + f"   [{n} samples]"
        )

    def track_bias(self, w, now):
        """Ease the bias toward the current reading, but only while truly still."""
        self.recent.append(w)
        if len(self.recent) < self.recent.maxlen:
            return

        # Spread about the mean, not about zero: a genuine steady turn shows a
        # large mean with a small spread, and testing the magnitude instead would
        # wave it straight through as "quiet, must be bias".
        n = len(self.recent)
        sd = []
        for i in range(3):
            m = sum(s[i] for s in self.recent) / n
            sd.append(math.sqrt(sum((s[i] - m) ** 2 for s in self.recent) / n))

        quiet = max(sd) < self.still_gyro_sd

        # The decisive test, and the one whose absence made this unusable: how far
        # the current reading sits from the bias we already believe.
        #
        # Spread alone cannot see a turn. Rotate the rover steadily by hand and
        # the gyro reads a large CONSTANT value -- low spread, so "quiet" -- while
        # turning on the spot produces almost no translation, so rf2o's linear
        # speed says "slow" too. Both gates opened, the bias absorbed the turn
        # rate, and when the rover stopped the corrected gyro read minus that
        # rate: a phantom spin in the opposite direction, decaying only as fast as
        # bias_tau_s. That is what "it keeps spinning after I stop" was.
        #
        # Offset from the established bias catches it, because the two quantities
        # live on completely different scales. Genuine bias wanders by about
        # 0.05 deg/s over minutes; hand rotation is tens of deg/s. Anything in
        # between is slow enough that absorbing it costs little.
        means = [sum(s[i] for s in self.recent) / n for i in range(3)]
        offset = max(abs(m - b) for m, b in zip(means, self.bias))
        settled = offset < self.still_offset

        # rf2o is a third opinion and still worth having: it is the only signal
        # here that watches the room rather than the chassis, so it catches
        # translation the gyro cannot see at all. Averaged over ~1 s, or its own
        # noise would decide the gate.
        speed = sum(self.speeds) / len(self.speeds) if self.speeds else 0.0
        slow = (not self.rf2o_seen) or speed < self.still_speed
        candidate = quiet and settled and slow

        # Require the new state to hold before acting on it, so a single noisy
        # window neither freezes tracking nor resumes it mid-push.
        if candidate != self.still:
            if self.pending != candidate:
                self.pending, self.pending_since = candidate, now
            elif now - self.pending_since >= self.get_parameter("min_dwell_s").value:
                self.still = candidate
                self.pending = None
                self.get_logger().info(
                    f"bias tracking {'RESUMED (rover still)' if candidate else 'FROZEN (moving)'}"
                )
        else:
            self.pending = None

        if not self.still:
            return

        # One-pole toward the current mean of the window.
        tau = self.get_parameter("bias_tau_s").value
        alpha = min(1.0, (n / 200.0) / max(tau, 1e-3))
        moved = [b + alpha * (m - b) for b, m in zip(self.bias, means)]

        # Backstop. However the gates are tuned, the tracked bias may never wander
        # far from what was measured at startup with the rover verifiably still.
        # Real offset drift is a fraction of a deg/s; anything approaching this
        # limit is rotation that got past the gates, and clamping caps the size of
        # the phantom spin that would follow rather than letting it grow without
        # bound. Hitting the clamp is a fault, so it is reported as one.
        clamped = []
        for i, (v, ref) in enumerate(zip(moved, self.initial_bias)):
            lo, hi = ref - self.max_bias_drift, ref + self.max_bias_drift
            if v < lo or v > hi:
                v = min(max(v, lo), hi)
                if now - self.last_clamp_warn > 10.0:
                    self.last_clamp_warn = now
                    self.get_logger().warning(
                        f"bias on {'xyz'[i]} hit its clamp "
                        f"({math.degrees(self.max_bias_drift):.2f} deg/s from startup) -- "
                        "rotation is leaking past the stillness gate"
                    )
            clamped.append(v)
        self.bias = tuple(clamped)

        if now - self.last_report > 60.0:
            self.last_report = now
            self.get_logger().info(
                "bias now (deg/s): "
                + "  ".join(f"{a} {math.degrees(b):+.4f}" for a, b in zip("xyz", self.bias))
            )

    # -- rf2o --------------------------------------------------------------
    def on_odom(self, msg):
        self.speeds.append(
            math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        )
        self.rf2o_seen = True

        out = msg
        cov = [0.0] * 36
        # Pose covariance is deliberately huge: this pose is dead reckoning with
        # unbounded error, and nothing downstream should treat it as a position
        # fix. Only the velocities below are meant to be fused.
        for i in (0, 7, 14, 21, 28, 35):
            cov[i] = 1e6
        out.pose.covariance = cov

        tw = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            tw[i] = 1e6
        tw[0] = self.get_parameter("rf2o_vx_variance").value
        tw[7] = self.get_parameter("rf2o_vy_variance").value
        tw[35] = self.get_parameter("rf2o_vyaw_variance").value
        out.twist.covariance = tw
        self.pub_odom.publish(out)


def main():
    rclpy.init()
    node = FusionPrep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
