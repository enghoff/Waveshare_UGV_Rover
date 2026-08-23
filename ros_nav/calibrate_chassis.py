#!/usr/bin/env python3
"""Measure what this chassis actually does when told to move, and write it down.

Three things, all of which the base node needs and none of which it may guess:

  **turn_pwm_points**   what turn rate each motor PWM really produces
  **drive_pwm_points**  what forward speed each motor PWM really produces
  **ticks_per_metre**   how many wheel-encoder ticks a metre costs

They go into `~/ugv/odometry.json` beside the gyro scale that is already there,
and `base_node.py` interpolates between them. Until they exist it falls back to
the constants in `lidar_slam/nav_types.py`, which were measured on the rover as it
was before -- a different board, a different floor and a different battery -- and
which are demonstrably no longer true here.

    python3 calibrate_chassis.py --dry-run    # check everything, move nothing
    python3 calibrate_chassis.py --turns      # just the turn curve, no floor needed
    python3 calibrate_chassis.py              # everything

## Why the old numbers had to be replaced

`nav_types.py` records `{180: (170.0 deg/s, 9.0 deg of coast), 80: (31.6, 2.0)}`
and warns, in as many words, that the signature of stale numbers is a consistent
over- or under-shoot in the same direction on every turn. That is exactly what
turned up. Commanding 20 deg/s through a straight proportional map produced PWM 93
and a measured 25 deg/s; commanding it through a straight-line fit to those two
points produced PWM 72 and a measured 8. Two honest models, both wrong, because
the data underneath them describes a different rover.

The first sweep looked as though the two directions differed -- 8.2 deg/s
anticlockwise against 4.4 clockwise at one PWM -- and the second sweep put the
difference the other way round at the same PWM. Two runs disagreeing on the sign
of a difference is noise, not a drivetrain, so both directions are measured and
averaged together, several times per point. What is real is the *floor*: below
about PWM 85 this chassis does not turn, it shuffles, and those points are
reported but deliberately kept out of the curve.

## How it measures

Turn rate comes from the **gyro**, through `/odom`, which is independent of the
motors and was itself calibrated over eighteen turns. Forward speed and the tick
scale come from the **map**, through the `map` -> `base_link` transform, which is
`slam_toolbox` fixing the rover against the walls -- also independent of the
wheels, which is what a wheel calibration requires.

Rates are taken from the steady middle of each burst, not from the total angle
divided by the total time. That is what removes the ramp at the start and the
coast at the end without having to model either: the repository's older
`calibrate_turn.py` separated them by timing bursts of three different lengths,
and sampling the middle is the same information for a third of the driving.

**This moves the rover.** Turning happens on the spot and needs only room to
rotate; the forward runs need clear floor and stop the moment anything is closer
than the margin. Somebody should be watching either way.
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
import rclpy.duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from base_node import Bridge, ODOMETRY_STORE                    # noqa: E402

for _candidate in (os.path.join(_HERE, "..", "lidar_slam"),
                   os.path.join(_HERE, "..", "..", "lidar_slam")):
    if os.path.isdir(_candidate):
        sys.path.insert(0, os.path.abspath(_candidate))
        break
from nav_types import CMD_HEARTBEAT, CMD_PWM, HEARTBEAT_MS      # noqa: E402

# The PWM values to characterise. Spread across the usable range rather than
# evenly, because the interesting part is the bottom: near the motors' floor a
# few units of PWM are the difference between not moving and moving, and that is
# where a controller asking for a gentle correction lives.
TURN_PWMS = (75, 85, 90, 95, 105, 120, 145, 170)
# Starting at 80, not 70: PWM 70 was measured here and the rover did not move at
# all, which is the same stiction floor the turn sweep found at 75. A point that
# reports zero teaches the curve nothing it does not already refuse to
# extrapolate into.
DRIVE_PWMS = (80, 95, 115, 140)

# How many times each point is measured, in each direction. More than one because
# a single 1.6 s burst on a skidding chassis is noisy enough to invent an
# asymmetry: the first sweep here had PWM 90 turning faster anticlockwise than
# clockwise (34.4 against 28.7) and the second had it the other way round (23.9
# against 27.8). Two runs disagreeing on the sign of the difference is how you
# know it is noise and not the drivetrain, so the directions are averaged
# together rather than stored apart -- and averaged over repeats, which is the
# only thing that actually shrinks the error.
REPEATS = 2

# Below this a burst did not turn the rover, it shuffled it. Points under it are
# reported -- they are a real measurement of where the motors give up -- but kept
# out of the curve, because a curve that starts at "PWM 75 gives 2 deg/s" invites
# the base node to ask for PWM 75 when something wants a gentle correction, and
# what it gets is stiction and a rover that does not move at all.
TURN_FLOOR_DPS = 5.0

# A burst, and the slice of it that counts. The first 0.7 s is the motors getting
# up to speed and the chassis taking up its own slack; the last 0.2 s is trimmed
# so that nothing from the release is inside the window.
BURST_S = 2.5
SETTLE_S = 0.7
TRIM_S = 0.2
# Long enough for the coast to finish and the scan matcher to settle on the new
# pose before the next burst starts from it.
REST_S = 2.0

AHEAD_HALF_DEG = 25.0
# How much room a turn on the spot needs. The rover is 34 cm across, so its
# corners sweep a circle of about 24 cm radius; this is that with room to spare.
TURN_CLEARANCE_M = 0.35

# Backing up to recover floor: slow, and stopping well clear of whatever is
# behind. Reversing is the one move here made without the map watching closely,
# so it is made at a crawl.
RECENTRE_PWM = 80
RECENTRE_MS = 0.15               # roughly, for working out how long to allow
RECENTRE_MARGIN_M = 0.45

# How far a forward run may wander off straight before its distance stops meaning
# anything. The distance is measured as the range to a wall ahead shrinking, which
# is the length of a straight line -- so a run that curved has measured the chord
# and not the path, and reports too little.
MAX_DRIVE_DRIFT_DEG = 8.0


def normalise(degrees):
    return (degrees + 180.0) % 360.0 - 180.0


class Chassis(Node):

    def __init__(self, args):
        super().__init__("calibrate_chassis")
        self.args = args
        self.scan = None
        self.odom = None
        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan, "scan", self._on_scan, sensor_qos)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.bridge = Bridge((args.bridge_host, args.bridge_port), self.get_logger())
        self.bridge.start()

    def _on_scan(self, msg):
        self.scan = msg

    def _on_odom(self, msg):
        self.odom = msg

    # --- reading the world ----------------------------------------------------
    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def yaw(self):
        """Heading in degrees, from the gyro by way of /odom."""
        if self.odom is None:
            return None
        q = self.odom.pose.pose.orientation
        return math.degrees(2.0 * math.atan2(q.z, q.w))

    def pose(self):
        try:
            t = self.buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def clearance(self, half_deg=AHEAD_HALF_DEG, bearing_deg=0.0):
        """Nearest return inside a cone about `bearing_deg`. 0 is straight ahead,
        180 straight behind."""
        msg = self.scan
        if msg is None:
            return None
        half = math.radians(half_deg)
        centre = math.radians(bearing_deg)
        nearest = float("inf")
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(normalise(math.degrees(angle - centre))) <= math.degrees(half):
                if r < nearest:
                    nearest = r
        return nearest

    def nearest_anything(self):
        msg = self.scan
        if msg is None:
            return None
        finite = [r for r in msg.ranges if math.isfinite(r) and r >= msg.range_min]
        return min(finite) if finite else float("inf")

    def ticks(self):
        record, _ = self.bridge.read()
        if not record or not record.get("motion"):
            return None, None
        return record["motion"].get("ticks"), record["motion"].get("breaks")

    # --- driving --------------------------------------------------------------
    def send(self, left, right):
        self.bridge.send({"T": CMD_PWM, "L": int(left), "R": int(right)})

    def stop(self):
        self.send(0, 0)

    def burst(self, left, right, sample, guard=None):
        """Hold a PWM pair for BURST_S, sampling `sample()` through the middle.

        Returns a list of (time, value) through the steady window, or None if
        `guard` asked to stop. Every sample is kept rather than just the two ends,
        which matters for the turn measurement: at PWM 160 this chassis turns
        about 100 deg/s, so a 1.6 s window is more than half a revolution, and a
        heading difference taken between two endpoints cannot tell 200 degrees
        anticlockwise from 160 clockwise. It reported the latter. Summing the
        small differences between consecutive samples has no such ambiguity.

        The PWM goes straight to the board rather than through `/cmd_vel`, because
        what is being measured is the map from PWM to motion, and putting the
        thing under test in the path would measure it against itself.
        """
        self.bridge.send({"T": CMD_HEARTBEAT, "cmd": HEARTBEAT_MS})
        start = time.monotonic()
        samples = []
        aborted = None
        last_kick = 0.0
        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed >= BURST_S:
                break
            if guard is not None:
                why = guard()
                if why:
                    aborted = why
                    break
            # Re-sent well inside the firmware's heartbeat, which stops the base
            # if it hears nothing for HEARTBEAT_MS.
            if now - last_kick > HEARTBEAT_MS / 3000.0:
                self.send(left, right)
                last_kick = now
            rclpy.spin_once(self, timeout_sec=0.02)
            if SETTLE_S <= elapsed <= BURST_S - TRIM_S:
                value = sample()
                if value is not None:
                    samples.append((now, value))
        self.stop()
        self.spin_for(REST_S)
        # A guard that fires is not automatically a wasted run. Stopping because
        # something came inside the margin is the *right* outcome and it usually
        # happens near the end, by which point the steady window has already been
        # sampled -- so the measurement is kept if there is enough of it, and only
        # discarded if the burst was cut short before it began. Throwing away every
        # guarded run is what made the whole forward half of this unmeasurable in a
        # room 1.5 m wide: each run stopped legitimately, and each was binned.
        enough = len(samples) >= 6 and samples[-1][0] - samples[0][0] >= 0.6
        if aborted and not enough:
            return None, aborted
        if len(samples) < 4 or samples[-1][0] - samples[0][0] < 0.3:
            return None, "no readings through the burst"
        return samples, None

    def recentre(self, needed):
        """Back up until there is `needed` metres of floor ahead again.

        This is not a convenience. Turning a skid-steer chassis on the spot does
        not happen on the spot: measured here, a sweep of sixteen two-second turn
        bursts walked the rover 2.4 metres forward and left it 34 cm from a wall,
        with the entire forward half of the calibration then skipped for want of
        room. A script that consumes the floor it needs can only ever be run once
        per repositioning by hand.

        Reversing is the safe direction to recover in because the lidar sees all
        the way round, so "behind" is measured rather than assumed -- and it is
        checked every tick of the way back, not once at the start.
        """
        ahead = self.clearance()
        if ahead is None or ahead >= needed:
            return True, None
        behind = self.clearance(bearing_deg=180.0)
        if behind is None:
            return False, "no scan"
        room = needed - ahead + 0.2
        if behind < room + RECENTRE_MARGIN_M:
            return False, ("%.2f m ahead and only %.2f m behind -- no room to back "
                           "into" % (ahead, behind))
        self.get_logger().info(
            "backing up about %.2f m: %.2f m ahead, %.2f m behind"
            % (room, ahead, behind))
        self.bridge.send({"T": CMD_HEARTBEAT, "cmd": HEARTBEAT_MS})
        deadline = time.monotonic() + room / RECENTRE_MS + 4.0
        last_kick = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            back = self.clearance(bearing_deg=180.0)
            if back is not None and back < RECENTRE_MARGIN_M:
                break
            if self.clearance() >= needed:
                break
            if now - last_kick > HEARTBEAT_MS / 3000.0:
                self.send(-RECENTRE_PWM, -RECENTRE_PWM)
                last_kick = now
            rclpy.spin_once(self, timeout_sec=0.02)
        self.stop()
        self.spin_for(REST_S)
        ahead = self.clearance()
        if ahead is None or ahead < needed:
            return False, ("still only %.2f m ahead after backing up"
                           % (ahead if ahead is not None else float("nan")))
        return True, None

    # --- the turn curve -------------------------------------------------------
    def measure_turn(self, pwm, direction):
        """Degrees per second at this PWM, turning `direction` (+1 = anticlockwise).

        Anticlockwise is left wheel back and right wheel forward, which is the
        sign checked against the rover: a commanded positive yaw rate turned it
        anticlockwise.
        """
        room = self.nearest_anything()
        if room is None:
            return None, "no scan"
        if room < TURN_CLEARANCE_M:
            return None, ("only %.2f m to the nearest thing, needs %.2f to turn"
                          % (room, TURN_CLEARANCE_M))
        left, right = (-pwm, pwm) if direction > 0 else (pwm, -pwm)
        samples, why = self.burst(left, right, self.yaw)
        if samples is None:
            return None, why
        # Accumulated from consecutive differences, each of which is far under
        # half a turn, so the total is unambiguous however far the rover went.
        turned = sum(normalise(b - a) for (_, a), (_, b) in zip(samples, samples[1:]))
        seconds = samples[-1][0] - samples[0][0]
        rate = turned / seconds
        # Under a couple of degrees a second the rover has not turned, it has
        # shuffled -- and at that size the sign is noise. That is a real
        # measurement of the motors' floor and belongs in the curve, so it is
        # reported rather than treated as a wrong-way error.
        if abs(rate) < 2.0:
            return abs(rate), None
        if direction * rate <= 0:
            return None, ("it turned the wrong way (%+.1f deg/s) -- check the sign"
                          % rate)
        return abs(rate), None

    # --- the drive curve ------------------------------------------------------
    def measure_drive(self, pwm):
        """Metres per second and ticks per metre at this PWM, going forward.

        Distance comes from the **lidar**: how much closer the thing ahead got.
        Not from the map, which is where this started and where it could not
        finish, because that route is circular. `slam_toolbox` only adds a scan to
        its pose graph once odometry says the rover has moved
        `minimum_travel_distance`, and odometry's distance is exactly the number
        being calibrated -- with no tick scale it reports the commanded speed,
        which is zero here because this drives the board directly. So the map sat
        frozen at the origin and every run reported moving 0.000 m while the rover
        was visibly crossing the room.

        The range to a wall ahead depends on nothing but the wall. What it does
        depend on is going straight, so the gyro is watched too and a run that
        curved is thrown away rather than quietly recording the chord of an arc as
        though it were the arc.
        """
        needed = self.args.margin + 0.5
        room = self.clearance()
        if room is None:
            return None, "no scan"
        if room < needed:
            return None, ("only %.2f m ahead, needs %.2f" % (room, needed))
        if not math.isfinite(room):
            return None, ("nothing within range ahead to measure against -- point "
                          "the rover at a wall")

        def guard():
            clear = self.clearance()
            if clear is None:
                return "lost the scan"
            if clear < self.args.margin:
                return "%.2f m ahead" % clear
            return None

        def sample():
            """Range ahead, encoder count and heading, all at the same instant."""
            ahead = self.clearance()
            ticks, breaks = self.ticks()
            heading = self.yaw()
            if ahead is None or not math.isfinite(ahead) or ticks is None:
                return None
            return (ahead, ticks, breaks, heading)

        samples, why = self.burst(pwm, pwm, sample, guard=guard)
        if samples is None:
            return None, why
        (a0, t0, b0, y0) = samples[0][1]
        (a1, t1, b1, y1) = samples[-1][1]
        if b0 != b1:
            return None, "the board's counters broke mid-run"
        seconds = samples[-1][0] - samples[0][0]
        # Closing on the wall, so the range shrinks by the distance travelled.
        travelled = a0 - a1
        if travelled < 0.05:
            return None, ("the wall ahead only got %.3f m closer -- it did not move"
                          % travelled)
        drift = abs(normalise((y1 or 0.0) - (y0 or 0.0)))
        if drift > MAX_DRIVE_DRIFT_DEG:
            return None, ("it curved %.0f degrees over the run, so the distance is "
                          "not a straight line" % drift)
        return (travelled / seconds, travelled, t1 - t0, drift), None


def fit_points(pairs):
    """Sorted [pwm, value] pairs, averaging repeats at the same PWM."""
    grouped = {}
    for pwm, value in pairs:
        grouped.setdefault(pwm, []).append(value)
    return [[pwm, round(sum(v) / len(v), 4)] for pwm, v in sorted(grouped.items())]


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--turns", action="store_true", help="the turn curve only")
    p.add_argument("--drives", action="store_true", help="the drive curve only")
    p.add_argument("--margin", type=float, default=0.55,
                   help="stop a forward run if anything is this close ahead. "
                        "0.55 m rather than 0.8: the rover is 0.34 m wide, stops "
                        "inside a few centimetres at these speeds, and a bigger "
                        "margin means an ordinary room has no measurable run in it")
    p.add_argument("--store", default=ODOMETRY_STORE)
    p.add_argument("--bridge-host", default="127.0.0.1")
    p.add_argument("--bridge-port", type=int, default=8772)
    p.add_argument("--repeats", type=int, default=REPEATS,
                   help="measurements per point per direction; averaging is the "
                        "only thing that shrinks the error on a skidding chassis")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()
    do_turns = args.turns or not args.drives
    do_drives = args.drives or not args.turns

    rclpy.init()
    node = Chassis(args)
    turn_points, drive_points, tick_ratios = [], [], []
    try:
        node.get_logger().info("waiting for the scan, the map and the board...")
        node.spin_for(5.0)
        if node.scan is None:
            print("no /scan -- is the stack running?", file=sys.stderr)
            return 1
        print("nearest anything: %.2f m; clear ahead: %.2f m"
              % (node.nearest_anything(), node.clearance()))
        if args.dry_run:
            print("dry run: everything is connected, nothing was moved")
            return 0

        if do_drives:
            print("\n--- drive curve (forward, %d PWM values)" % len(DRIVE_PWMS))
            print("    first, because a forward run needs 1.3 m of floor where a")
            print("    turn needs 0.35 -- and the turn sweep drifts the rover along")
            for pwm in DRIVE_PWMS:
                # Room for the whole run, not just its first step: the rover
                # travels up to about 0.7 m inside a burst, and recentring only
                # to the margin means the guard fires before the steady window
                # has been sampled.
                ok, why = node.recentre(node.args.margin + 1.0)
                if not ok:
                    print("  PWM %3d skipped: %s" % (pwm, why))
                    continue
                result, why = node.measure_drive(pwm)
                if result is None:
                    print("  PWM %3d skipped: %s" % (pwm, why))
                    continue
                speed, travelled, spent, drift = result
                print("  PWM %3d  %.3f m/s  (%.3f m by the lidar, %.1f ticks, "
                      "%.0f deg of curve)" % (pwm, speed, travelled, spent, drift))
                drive_points.append((pwm, speed))
                if travelled > 0.05 and abs(spent) > 1e-6:
                    tick_ratios.append(abs(spent) / travelled)

        if do_turns:
            print("\n--- turn curve (on the spot, %d PWM values, both ways, x%d)"
                  % (len(TURN_PWMS), args.repeats))
            for pwm in TURN_PWMS:
                rates = []
                # Turning walks the rover along, so recover the room it needs
                # before each point rather than discovering halfway through the
                # sweep that there is none left.
                ok, why = node.recentre(TURN_CLEARANCE_M + 0.4)
                if not ok:
                    node.get_logger().info("no room to recentre (%s); carrying on"
                                           % why)
                for _ in range(args.repeats):
                    for direction, name in ((+1, "anticlockwise"), (-1, "clockwise")):
                        rate, why = node.measure_turn(pwm, direction)
                        if rate is None:
                            print("  PWM %3d %-13s skipped: %s" % (pwm, name, why))
                            continue
                        rates.append(rate)
                if not rates:
                    continue
                mean = sum(rates) / len(rates)
                spread = (max(rates) - min(rates)) if len(rates) > 1 else 0.0
                flag = ""
                if mean < TURN_FLOOR_DPS:
                    flag = "  <- below the motors' floor, kept out of the curve"
                print("  PWM %3d  %6.1f deg/s  (%d runs, spread %.1f)%s"
                      % (pwm, mean, len(rates), spread, flag))
                if mean >= TURN_FLOOR_DPS:
                    turn_points.extend((pwm, r) for r in rates)

    except KeyboardInterrupt:
        pass
    finally:
        # The rover must not be left driving because this script had a bad day.
        try:
            node.stop()
            node.spin_for(0.3)
            node.stop()
        except Exception:
            pass

    store = {}
    try:
        with open(args.store) as fh:
            store = json.load(fh)
    except (OSError, ValueError):
        pass

    changed = []
    if turn_points:
        store["turn_pwm_points"] = fit_points(turn_points)
        store["turns_measured"] = len(turn_points)
        changed.append("turn_pwm_points")
    if drive_points:
        store["drive_pwm_points"] = fit_points(drive_points)
        store["drives_measured"] = len(drive_points)
        changed.append("drive_pwm_points")
    if tick_ratios:
        # The median, properly, rather than the upper of the two middles: these
        # ratios span 108 to 173 on four runs, so which middle is taken moves the
        # answer by 12%.
        store["ticks_per_metre"] = round(median(tick_ratios), 3)
        store["ticks_spread"] = [round(min(tick_ratios), 1), round(max(tick_ratios), 1)]
        changed.append("ticks_per_metre")

    if not changed:
        print("\nnothing measured; %s unchanged" % args.store, file=sys.stderr)
        rclpy.shutdown()
        return 1

    with open(args.store, "w") as fh:
        json.dump(store, fh, indent=1)
        fh.write("\n")
    # ext4 here is mounted commit=120, so a file written and not flushed can be
    # two minutes behind a power cut -- and this one cost driving the rover about.
    os.sync()
    print("\nwrote %s to %s" % (", ".join(changed), args.store))
    for key in changed:
        print("  %-18s %s" % (key, store[key]))
    print("restart the stack for the base node to pick these up:")
    print("  ~/ugv/ros_nav/restart.sh")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
