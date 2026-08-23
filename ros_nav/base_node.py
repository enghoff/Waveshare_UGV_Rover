#!/usr/bin/env python3
"""The rover's wheels and gyro, as a ROS 2 base controller.

Two jobs, both against the driver board that the daemon owns. Outwards, it turns
`/cmd_vel` into the motor PWM pair the ESP32 takes. Inwards, it integrates the
board's gyro and wheel encoders into `/odom` and the `odom` -> `base_link`
transform, which is the motion prior `slam_toolbox` matches scans against and the
frame Nav2 does its local planning in.

It does not open a serial port. The board is on the GPIO UART, the daemon holds
that port because it is also the lights and the gimbal and the pack voltage, and
two processes on one UART is two half-conversations. So this is a client of
`rover_daemon/board_bridge.py` on loopback, which hands out exactly these two
things at a rate a control loop can use.

**Nothing here invents a number.** Every constant comes from
`~/ugv/odometry.json`: the gyro's scale, measured over eighteen turns, and the
PWM-to-turn-rate and PWM-to-speed curves and the tick scale, all measured by
`calibrate_chassis.py` on this chassis. The constants in `lidar_slam/nav_types.py`
are only a fallback, and a bad one -- they describe the rover as it was on a
different board, floor and battery, and following them asked for two and a half
times too little PWM. So a missing curve is warned about loudly rather than
papered over, because a drive model that is quietly wrong by a scale factor is
worse than one that is absent: everything downstream believes it.

    python3 base_node.py --help
"""

import argparse
import json
import math
import os
import socket
import sys
import threading
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

_HERE = os.path.dirname(os.path.abspath(__file__))
for _candidate in (os.path.join(_HERE, "..", "lidar_slam"),
                   os.path.join(_HERE, "..", "..", "lidar_slam")):
    if os.path.isdir(_candidate):
        _candidate = os.path.abspath(_candidate)
        if _candidate not in sys.path:
            sys.path.insert(0, _candidate)

# The measured drive model, imported rather than copied. If somebody re-measures
# the chassis on a different floor, this follows.
from nav_types import (CMD_HEARTBEAT, CMD_PWM, HEARTBEAT_MS, MAX_SPEED_MS,   # noqa: E402
                       MAX_TURN_DPS, MIN_PWM, MIN_TURN_DPS, TOP_PWM,
                       TURN_RATES)

# Where the daemon stores what it has learned about this chassis.
ODOMETRY_STORE = os.path.expanduser("~/ugv/odometry.json")

BRIDGE = ("127.0.0.1", 8772)
# A command older than this is not a command. Nav2's controller runs at 20 Hz, so
# half a second of silence is it having stopped, crashed, or lost the network --
# and in all three cases the wheels should not still be turning. The board's own
# heartbeat is the backstop under this one, not a replacement for it: it catches
# this process dying, and this catches the controller dying while this lives.
CMD_TIMEOUT_S = 0.5

# Estimating the gyro's zero-offset while the rover is still. See
# BaseNode.debias, which explains why this is not optional.
#
# STILL_TICKS is a hair above zero rather than zero: the encoder mean is a float
# and jitters in the last place even when nothing is moving.
STILL_TICKS = 0.5
STILL_SETTLE_S = 1.0
# Slow. The offset drifts with temperature over minutes, so the average has to be
# long compared with the noise and short compared with that drift; at ~18 samples
# a second this is a time constant of about a minute.
BIAS_GAIN = 0.001


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def to_pwm(value):
    """A normalised -1..1 wheel command as the firmware's PWM.

    The same curve as `lidar_slam/nav_drive.py`, and deliberately so: below
    MIN_PWM the motors buzz and do not turn, so the usable range starts there
    rather than at zero, and a controller that has not been told this spends the
    bottom quarter of its output commanding a noise.
    """
    if abs(value) < 1e-3:
        return 0
    magnitude = MIN_PWM + abs(value) * (TOP_PWM - MIN_PWM)
    return int(round(magnitude if value > 0 else -magnitude))


# The two points somebody actually measured on this chassis, from
# lidar_slam/nav_types.py: `{180: (170.0 deg/s, 9.0 deg of coast),
# 80: (31.6, 2.0)}`, taken by timing fixed-PWM bursts and reading the angle back
# off the lidar. Sorted so the fit below does not depend on dict order.
_TURN_FIT = sorted((pwm, rate) for pwm, (rate, _coast) in TURN_RATES.items())
_TURN_LO, _TURN_HI = _TURN_FIT[0], _TURN_FIT[-1]
# deg/s per unit of PWM: (170.0 - 31.6) / (180 - 80) = 1.384
_TURN_SLOPE = ((_TURN_HI[1] - _TURN_LO[1]) / (_TURN_HI[0] - _TURN_LO[0])
               if _TURN_HI[0] != _TURN_LO[0] else 1.0)
TURN_PWM_MAX = _TURN_HI[0]


def pwm_for(points, wanted, floor=MIN_PWM, ceiling=TURN_PWM_MAX):
    """Invert a measured [[pwm, value], ...] curve: what PWM gives `wanted`?

    Piecewise linear between the measured points, and extrapolated from the two
    nearest ones outside them, so a curve with a knee in it is followed rather
    than averaged away. This exists because the alternatives were both tried on
    the rover and both failed in opposite directions:

      - Scaling PWM proportionally from zero ignores that the motors deliver
        nothing below MIN_PWM and then climb steeply. Asking for 20 deg/s gave
        PWM 93 and a measured 25 deg/s.
      - Fitting a straight line to the two constants in `nav_types.py` gave
        PWM 72 and a measured 8 deg/s -- because those constants describe the
        rover as it was, on a different board, floor and battery.

    Neither model was wrong about arithmetic. Both were wrong about this chassis,
    which is why the points come from a file that `calibrate_chassis.py` writes
    rather than from anything in the source.
    """
    if not points or wanted <= 0:
        return 0
    # Below the slowest thing measured, the honest answer is the slowest thing
    # measured: a PWM under it is one the motors ignore, which is a movement that
    # never happens and a controller waiting for it.
    if wanted <= points[0][1]:
        return int(round(max(floor, min(ceiling, points[0][0]))))
    for (p0, v0), (p1, v1) in zip(points, points[1:]):
        if wanted <= v1:
            if v1 == v0:
                return int(round(p1))
            share = (wanted - v0) / (v1 - v0)
            return int(round(max(floor, min(ceiling, p0 + share * (p1 - p0)))))
    # Past the fastest measured point: extrapolate from the last pair, but never
    # past what the firmware was measured to take.
    (p0, v0), (p1, v1) = points[-2], points[-1]
    if v1 == v0:
        return int(round(min(ceiling, p1)))
    slope = (p1 - p0) / (v1 - v0)
    return int(round(max(floor, min(ceiling, p1 + (wanted - v1) * slope))))


# The fallback, used only until calibrate_chassis.py has run. Kept because a
# rover with no calibration at all should still move, and flagged loudly at
# startup because these numbers are known to be wrong for this chassis.
FALLBACK_TURN_POINTS = [[pwm, rate] for pwm, rate in _TURN_FIT]


def turn_to_pwm(dps, points=None):
    """The per-wheel PWM magnitude that turns this chassis at `dps` degrees/s."""
    wanted = abs(dps)
    if wanted < 1e-3:
        return 0
    return pwm_for(points or FALLBACK_TURN_POINTS, max(wanted, MIN_TURN_DPS))


def mix(linear, angular, turn_points=None, drive_points=None):
    """A `/cmd_vel` as the firmware's left and right PWM pair.

    Done in PWM rather than in normalised units, because that is where both
    calibrations live: each curve is a measured map from PWM to motion, and mixing
    normalised numbers and converting once at the end -- which is what
    `lidar_slam/nav_drive.py` does -- would apply the motors' floor to the sum
    instead of to each part.

    Positive `angular.z` is counter-clockwise by REP-103, which is a left turn, so
    the left wheel goes backwards. That is the one sign here worth checking
    against the rover rather than against the code, and it was: a commanded
    +20 deg/s turned it anticlockwise.
    """
    if drive_points:
        # Clamped to the fastest speed actually measured, not to MAX_SPEED_MS.
        # That constant says 0.35 m/s and this chassis was measured at 0.33 at its
        # slowest usable PWM and 0.68 at PWM 140 -- so the "maximum" is very nearly
        # the minimum, and clamping to it pinned every Nav2 command to the slowest
        # PWM the motors will turn at. There was no speed control at all.
        speed = min(drive_points[-1][1], abs(linear))
        throttle = pwm_for(drive_points, speed, ceiling=TOP_PWM)
    else:
        speed = min(MAX_SPEED_MS, abs(linear))
        throttle = to_pwm(speed / MAX_SPEED_MS)
    if linear < 0:
        throttle = -throttle

    turn = turn_to_pwm(math.degrees(min(math.radians(MAX_TURN_DPS), abs(angular))),
                       turn_points)
    if angular < 0:
        turn = -turn
    left, right = throttle - turn, throttle + turn
    # Scale rather than clip when the pair runs past what the firmware takes.
    # Clipping one wheel and not the other changes the turn the caller asked for
    # into a different one; scaling both keeps the ratio and only slows it down.
    peak = max(abs(left), abs(right))
    if peak > TURN_PWM_MAX:
        scale = TURN_PWM_MAX / peak
        left, right = left * scale, right * scale
    return int(round(left)), int(round(right))


class Bridge:
    """The daemon's board bridge, as a background reader with a latest-value hold.

    Latest-value rather than a queue: this is a sensor, and a subscriber that has
    fallen behind wants the current reading rather than the backlog. The
    reconnect loop matters more than it looks -- the daemon is restarted by
    `restart.sh` as a matter of routine, and a base controller that needed
    restarting alongside it would make every daemon deploy a rover deploy.
    """

    def __init__(self, address, log):
        self.address = address
        self.log = log
        self.sock = None
        self.latest = None
        self.latest_at = 0.0
        self.connects = 0
        self.drops = 0
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_forever, daemon=True,
                                        name="board-bridge-client")

    def start(self):
        self._thread.start()

    def close(self):
        self._stop.set()
        self._drop()

    def _drop(self):
        with self._send_lock:
            try:
                if self.sock is not None:
                    self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _read_forever(self):
        pending = b""
        while not self._stop.is_set():
            if self.sock is None:
                try:
                    sock = socket.create_connection(self.address, timeout=2.0)
                    sock.settimeout(1.0)
                    with self._send_lock:
                        self.sock = sock
                    pending = b""
                    self.connects += 1
                    self.log.info("board bridge connected to %s:%d" % self.address)
                except OSError:
                    self._stop.wait(2.0)
                    continue
            try:
                chunk = self.sock.recv(8192)
                if not chunk:
                    raise OSError("bridge closed the connection")
                pending += chunk
            except socket.timeout:
                continue
            except OSError as exc:
                if self._stop.is_set():
                    break        # our own close(), not the bridge going away
                self.drops += 1
                self.log.warn("board bridge lost (%s); reconnecting" % exc)
                self._drop()
                self._stop.wait(1.0)
                continue
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    record = json.loads(line)
                except ValueError:
                    continue
                if record.get("kind") == "motion":
                    with self._lock:
                        self.latest = record
                        self.latest_at = time.monotonic()

    def read(self):
        with self._lock:
            return self.latest, self.latest_at

    def send(self, command):
        """Push one board command. False if there is nowhere to push it."""
        with self._send_lock:
            if self.sock is None:
                return False
            try:
                self.sock.sendall(
                    json.dumps({"send": command}, separators=(",", ":")).encode()
                    + b"\n")
                return True
            except OSError:
                return False


class BaseNode(Node):

    def __init__(self, args):
        super().__init__("base_node")
        self.args = args
        self.calibration = self.load_calibration()

        self.bridge = Bridge((args.bridge_host, args.bridge_port), self.get_logger())
        self.bridge.start()

        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.odom_pub = self.create_publisher(Odometry, "odom", qos)
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", qos)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)

        self.publish_static_transforms()

        # Pose in the odom frame. It starts at the origin by definition -- odom is
        # "wherever this node was started" -- and drifts from there, which is what
        # the frame is for. slam_toolbox is what corrects it, by publishing
        # map -> odom on top.
        self.x = self.y = self.yaw = 0.0
        self.speed = self.turn_rate = 0.0
        self._last_ticks = None
        self._last_gz = None
        self._last_breaks = None
        self._last_at = None
        self._cmd = (0.0, 0.0)
        self._cmd_at = 0.0
        self._commanded = None
        self._heartbeat_at = 0.0
        self._no_board_since = None
        self._warned_ticks = False
        self._bias = None
        self._bias_samples = 0
        self._still_for = 0.0

        self.turn_points = self.calibration["turn_pwm_points"]
        self.drive_points = self.calibration["drive_pwm_points"]

        self.timer = self.create_timer(1.0 / args.rate, self.tick)
        self.get_logger().info(
            "gyro %.4f LSB/deg-per-s, wheels %s"
            % (self.calibration["gyro_lsb_per_dps"],
               ("%.1f ticks/m" % self.calibration["ticks_per_metre"])
               if self.calibration["ticks_per_metre"] else
               "NOT CALIBRATED -- falling back to the commanded speed"))
        if self.turn_points:
            self.get_logger().info(
                "turn curve measured on this chassis: %s"
                % ", ".join("PWM %d=%.1f deg/s" % (p, v) for p, v in self.turn_points))
        else:
            # Loud, because the fallback is known to be wrong here rather than
            # merely unverified: measured on the rover, the constants in
            # nav_types.py ask for a PWM that turns it at 8 deg/s when 20 was
            # wanted. Nav2 will still drive; it will rotate slowly and overshoot.
            self.get_logger().warn(
                "no measured turn curve -- using nav_types.py, which was measured "
                "on the previous rover and asks for about 2.5x too little PWM. "
                "Run ros_nav/calibrate_chassis.py --turns")
        if not self.drive_points:
            self.get_logger().warn(
                "no measured speed curve -- assuming PWM is proportional to speed. "
                "Run ros_nav/calibrate_chassis.py")
        self.create_timer(30.0, self.report_bias)

    def report_bias(self):
        if self._bias is None:
            self.get_logger().info("gyro bias: not estimated yet (rover not still)")
        else:
            self.get_logger().info(
                "gyro bias %+.3f deg/s over %d still samples -- %+.0f deg an hour "
                "if left uncorrected" % (math.degrees(self._bias),
                                         self._bias_samples,
                                         math.degrees(self._bias) * 3600))

    # --- what the chassis is known to do -------------------------------------
    def load_calibration(self):
        """The learned constants, from where the daemon keeps them.

        `ticks_per_metre` being absent is the expected case rather than an error:
        the store records it as null until something has driven a measured
        straight line, and nothing had by the time this was written. Run
        `calibrate_chassis.py` to fill it in.
        """
        store = {"gyro_lsb_per_dps": None, "ticks_per_metre": None,
                 "turn_pwm_points": None, "drive_pwm_points": None}
        try:
            with open(self.args.calibration) as fh:
                loaded = json.load(fh)
            for key in ("gyro_lsb_per_dps", "ticks_per_metre"):
                if isinstance(loaded.get(key), (int, float)):
                    store[key] = float(loaded[key])
            for key in ("turn_pwm_points", "drive_pwm_points"):
                points = loaded.get(key)
                if isinstance(points, list) and len(points) >= 2:
                    # Sorted by the measured value, which is what pwm_for walks.
                    store[key] = sorted(
                        [[float(a), float(b)] for a, b in points],
                        key=lambda pair: pair[1])
        except (OSError, ValueError, TypeError) as exc:
            self.get_logger().warn("no calibration from %s (%s)"
                                   % (self.args.calibration, exc))
        if self.args.gyro_lsb_per_dps:
            store["gyro_lsb_per_dps"] = self.args.gyro_lsb_per_dps
        if self.args.ticks_per_metre:
            store["ticks_per_metre"] = self.args.ticks_per_metre
        if not store["gyro_lsb_per_dps"]:
            # Without this every turn is unmeasured, and a mapper handed a rover
            # that never turns will fold the room in on itself. Refusing is the
            # honest failure.
            raise SystemExit(
                "no gyro scale in %s and none given: run lidar_slam/calibrate_turn.py"
                % self.args.calibration)
        return store

    def publish_static_transforms(self):
        """base_link -> laser, and it is deliberately the identity.

        `base_link` is defined *at the lidar* on this rover rather than at the
        centre of the chassis, because the offset between the two has never been
        measured and a made-up one is worse than none: it would put every wall in
        the map a few centimetres from where it is, consistently, which is exactly
        the error a scan matcher cannot see and cannot correct. The 90-degree
        mount rotation is already undone inside the parser, so the axes agree.

        The consequence is that Nav2's footprint is measured from the sensor and
        not from the middle of the rover -- see nav2.yaml, which says so again
        where the number is.
        """
        self.static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.args.base_frame
        t.child_frame_id = self.args.laser_frame
        t.transform.rotation.w = 1.0
        self.static_tf.sendTransform([t])

    # --- outward: cmd_vel -> PWM ---------------------------------------------
    def on_cmd_vel(self, msg):
        self._cmd = (float(msg.linear.x), float(msg.angular.z))
        self._cmd_at = time.monotonic()

    def drive(self):
        """Send the wheels whatever the newest `/cmd_vel` asks for, or stop them.

        Open loop against the measured PWM curves. That is the right shape here
        even though the encoders are right there: Nav2 closes its own loop on the
        pose, at 20 Hz, and a second velocity servo underneath it is two
        controllers fighting over the same actuator with different ideas of the
        truth. What the encoders are for is `/odom`, which is how Nav2 sees the
        result.
        """
        now = time.monotonic()
        live = self._cmd_at > 0.0 and now - self._cmd_at <= CMD_TIMEOUT_S

        if not live:
            # Send one stop on the falling edge, then go quiet. Repeating zeroes
            # for ever would be the safe-looking choice and is the wrong one: it
            # means ROS holds the wheels at a standstill whenever nobody is
            # navigating, so nothing else can ever drive this rover. Somebody
            # mapping the house with the game pad would find the board obeying a
            # stop command six times a second and the rover refusing to move,
            # with nothing in any log to say why.
            #
            # Going quiet is also not a risk here, because the firmware's own
            # heartbeat is underneath: it stops the base if it hears nothing for
            # HEARTBEAT_MS, so silence brakes the rover rather than freeing it.
            if self._commanded not in (None, (0, 0)):
                self.bridge.send({"T": CMD_PWM, "L": 0, "R": 0})
                self._commanded = (0, 0)
            return

        linear, angular = self._cmd
        pair = mix(linear, angular, self.turn_points, self.drive_points)

        # Only when it changes, plus a keepalive while a command is live. The
        # board's heartbeat stops the wheels if it hears nothing, so a rover that
        # is meant to be moving has to keep saying so -- but resending an
        # identical pair fifty times a second is most of the traffic on the bridge
        # for no effect on the motors.
        if pair != self._commanded or now - self._heartbeat_at > HEARTBEAT_MS / 3000.0:
            self.bridge.send({"T": CMD_PWM, "L": pair[0], "R": pair[1]})
            self._commanded = pair
            self._heartbeat_at = now

    # --- inward: the board -> odom -------------------------------------------
    def tick(self):
        record, _ = self.bridge.read()
        self.drive()
        if record is None:
            return
        motion = record.get("motion")
        if not motion:
            return
        # Keyed on the *board's* own timestamp, not on when the bridge handed this
        # over. The bridge polls at 50 Hz and the ESP32 speaks at about 17, so two
        # ticks in three carry the reading that was already published. Treating
        # those as new samples was wrong twice over: it put duplicate poses on
        # /odom, and it computed velocity over an assumed 20 ms when the real
        # interval was nearer 60 -- so every speed and turn rate this published
        # was about three times what the rover was doing.
        at = motion.get("at")
        if at is None or at == self._last_at:
            return
        previous = self._last_at
        self._last_at = at
        self.integrate(motion, record,
                       dt=(at - previous) if previous is not None else None)

    def debias(self, d_yaw, dt, ticks):
        """Take the gyro's zero-offset out of one interval's rotation.

        This is the difference between odometry that is usable over a run and
        odometry that is not, and it is invisible in any short measurement. A
        stationary rover here reports 0.008 rad/s on `angular.z` -- 0.46 deg/s
        that is not happening. Over a four-second calibration burst that is two
        degrees and disappears into the noise; over a three-minute circuit it is
        eighty, and it was: two loop tests running the same route left dead
        reckoning 34 and 37 degrees from where the map put it, while the gyro's
        *scale* measured accurate to half a percent against the walls, and the
        saved map showed straight single walls and square corners rather than the
        doubling a mis-rotated map would have. Scale was exonerated by
        measurement and the map by inspection, which leaves the offset.

        Estimated only while the rover is genuinely still -- nothing commanded and
        the wheels not turning -- because that is the only time the true rate is
        known to be zero. A slow exponential average, because the offset drifts
        with temperature over minutes and a fast one would chase the noise it is
        supposed to be averaging out.

        This is what `robot_localization` would do properly, with a filter that
        also knows about the accelerometer. Until that is fitted, this is the part
        of it that matters.
        """
        if not dt or dt <= 0:
            return d_yaw
        rate = d_yaw / dt
        moving = (self._cmd_at > 0.0
                  and time.monotonic() - self._cmd_at <= CMD_TIMEOUT_S)
        if ticks is not None and self._last_ticks is not None:
            moving = moving or abs(ticks - self._last_ticks) > STILL_TICKS
        if not moving:
            self._still_for += dt
            # A moment's grace after stopping, so the coast is not averaged in as
            # though it were bias.
            if self._still_for > STILL_SETTLE_S:
                if self._bias is None:
                    self._bias = rate
                else:
                    self._bias += BIAS_GAIN * (rate - self._bias)
                self._bias_samples += 1
        else:
            self._still_for = 0.0
        if self._bias is None:
            return d_yaw
        return d_yaw - self._bias * dt

    def integrate(self, motion, record, dt=None):
        """Fold one motion sample into the pose.

        Differences, not absolutes: the board's counters are free-running and
        restart when it does. `breaks` is the board's own count of intervals it
        could not vouch for, so a step in it is a hole -- the right response is to
        drop that one interval rather than to integrate across it, because the
        counters on the far side are a different origin.
        """
        gz = motion.get("gz_lsb_s")
        ticks = motion.get("ticks")
        breaks = motion.get("breaks")
        stamp = self.get_clock().now()

        broken = (self._last_breaks is not None and breaks != self._last_breaks)
        self._last_breaks = breaks

        d_yaw = 0.0
        if gz is not None and self._last_gz is not None and not broken:
            # LSB-seconds since the last sample, over LSB per degree-per-second,
            # is degrees.
            d_yaw = math.radians((gz - self._last_gz)
                                 / self.calibration["gyro_lsb_per_dps"])
            d_yaw = self.debias(d_yaw, dt, ticks)
        self._last_gz = gz

        d_s = 0.0
        scale = self.calibration["ticks_per_metre"]
        if scale and ticks is not None and self._last_ticks is not None and not broken:
            d_s = (ticks - self._last_ticks) / scale
        elif not scale:
            # No tick scale: integrate what was asked for instead, and say so once.
            # This is good enough to keep slam_toolbox's search window centred on
            # something better than nothing, and it is not good enough to navigate
            # on -- which is why it warns rather than passing silently.
            if not self._warned_ticks:
                self.get_logger().warn(
                    "no ticks/metre: odometry distance is the commanded speed, not "
                    "a measurement. Run ros_nav/calibrate_chassis.py")
                self._warned_ticks = True
            d_s = self._cmd[0] * (dt or 0.0)
        self._last_ticks = ticks

        # Midpoint of the heading over the interval, which is the difference
        # between a circle and a polygon when the rover is turning while moving.
        heading = self.yaw + d_yaw / 2.0
        self.x += d_s * math.cos(heading)
        self.y += d_s * math.sin(heading)
        self.yaw = (self.yaw + d_yaw + math.pi) % (2 * math.pi) - math.pi
        # A very short or absent interval leaves the last velocity standing rather
        # than dividing by something near zero. The pose is already correct either
        # way -- it is built from differences, not from rates -- so this only
        # affects what is reported, and reporting a spike would have Nav2's
        # progress checker believe the rover bolted.
        if dt and dt > 1e-3:
            self.speed = d_s / dt
            self.turn_rate = d_yaw / dt

        self.publish_odom(stamp)
        self.publish_imu(stamp, record.get("telemetry"))

    def publish_odom(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.args.odom_frame
        t.child_frame_id = self.args.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = yaw_to_quaternion(self.yaw)
        self.tf.sendTransform(t)

        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.args.odom_frame
        msg.child_frame_id = self.args.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        msg.twist.twist.linear.x = self.speed
        msg.twist.twist.angular.z = self.turn_rate
        # Honest covariances, and they are not small. The heading is a calibrated
        # gyro and is the good one; x and y are wheel ticks on a tracked chassis
        # that skids every time it turns, so they are trusted about a tenth as
        # much. Getting this wrong in the optimistic direction is how a mapper
        # decides the scan match must be mistaken.
        var_xy = 0.05 ** 2
        var_yaw = 0.02 ** 2
        msg.pose.covariance[0] = var_xy
        msg.pose.covariance[7] = var_xy
        msg.pose.covariance[35] = var_yaw
        msg.twist.covariance[0] = var_xy
        msg.twist.covariance[35] = var_yaw
        self.odom_pub.publish(msg)

    def publish_imu(self, stamp, telemetry):
        """The board's raw IMU, when a full telemetry line came with the sample.

        Raw, hence `imu/data_raw` and hence the -1 orientation covariance: the
        ESP32 reports accelerometer and gyro counts, not a fused attitude, and
        claiming an orientation this does not have would be believed. Nothing in
        the stack consumes this yet -- it is here for `robot_localization`, which
        is the next thing to fit.
        """
        if not telemetry:
            return
        msg = Imu()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.args.base_frame
        msg.orientation_covariance[0] = -1.0          # "there is no orientation here"
        gz = telemetry.get("gz")
        if gz is not None:
            msg.angular_velocity.z = math.radians(
                float(gz) / self.calibration["gyro_lsb_per_dps"])
        for axis, key in (("x", "ax"), ("y", "ay"), ("z", "az")):
            value = telemetry.get(key)
            if value is not None:
                # The board reports milli-g on these axes.
                setattr(msg.linear_acceleration, axis, float(value) * 9.80665e-3)
        self.imu_pub.publish(msg)


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--bridge-host", default=BRIDGE[0])
    p.add_argument("--bridge-port", type=int, default=BRIDGE[1])
    p.add_argument("--calibration", default=ODOMETRY_STORE,
                   help="where the learned chassis constants are kept")
    p.add_argument("--gyro-lsb-per-dps", type=float, default=None,
                   help="override the stored gyro scale")
    p.add_argument("--ticks-per-metre", type=float, default=None,
                   help="override the stored wheel scale")
    p.add_argument("--rate", type=float, default=50.0, help="odometry rate, Hz")
    p.add_argument("--odom-frame", default="odom")
    p.add_argument("--base-frame", default="base_link")
    p.add_argument("--laser-frame", default="laser")
    if "--ros-args" in argv:
        argv = argv[:argv.index("--ros-args")]
    return p.parse_args(argv)


def main():
    rclpy.init()
    node = BaseNode(parse_args(sys.argv[1:]))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass                       # a SIGTERM from `ros2 launch`; see lidar_node.py
    finally:
        # Stop the wheels on the way out. This is the one thing in this file that
        # must happen even when everything else has gone wrong.
        try:
            node.bridge.send({"T": CMD_PWM, "L": 0, "R": 0})
        except Exception:
            pass
        node.bridge.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
