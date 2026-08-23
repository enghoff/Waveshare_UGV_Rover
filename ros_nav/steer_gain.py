#!/usr/bin/env python3
"""Measure what this chassis actually does when asked to steer while rolling.

    python3 steer_gain.py --dry-run          # say what it would drive, drive nothing
    python3 steer_gain.py                    # ~2 minutes, returns to where it started

**Why this exists.** Everything the mixer knows about rotation was measured with
the wheels running *opposite* each other -- `calibrate_chassis.py` times fixed-PWM
bursts of a pivot on the spot against the gyro. That is a real measurement of a
real manoeuvre, and it is the wrong one for steering.

A tracked chassis pivoting on the spot is dragging its whole contact patch
sideways, and the numbers say so: the turn curve implies an effective track width
of 4.16 m at PWM 85 falling to 1.09 m at PWM 170, on a rover 0.22 m wide. That
factor of five to nineteen is scrub, and it is not a property of the chassis --
it is a property of *pivoting*. When the rover is rolling forwards and one track
runs a little faster than the other, almost none of that scrub is present, so the
same PWM difference should turn it very much harder.

Nobody has measured how much harder, which means the steering channel's gain is
unknown. This measures it, by asking for a rotation through the same path Nav2
uses -- `/cmd_vel` into `base_node` into the mixer into the wheels -- and reading
what the gyro says came out.

**How it stays inside a small room.** Every sample is a short forward burst, then
a straight reverse of the same duration, then a pivot to undo the heading the
burst put on. Net displacement per sample is close to zero, and the turn requests
alternate in sign so what does not cancel exactly cancels on the average. It also
watches `/scan` and refuses to start a burst with anything close ahead.
"""

import argparse
import json
import math
import sys
import time

import os

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from drive_mixer import steer_pwm

STORE = os.path.expanduser("~/ugv/odometry.json")

TICK_HZ = 20.0             # how often /cmd_vel is republished; base_node times out at 0.5 s
STOP_S = 0.6               # quiet between manoeuvres, so each starts from rest
SETTLE_FRACTION = 0.4      # the leading share of a burst thrown away as acceleration
# Half the width of the corridor ahead that has to be clear. The rover's body is
# 0.14 m each side of centre, measured by the lidar's own returns off it, and this
# leaves a hand's width of margin on top.
GUARD_CORRIDOR_HALF_M = 0.25
GUARD_MIN_M = 0.45         # closer than this ahead and no burst is started
YAW_TOLERANCE_DEG = 3.0    # good enough when undoing a burst's heading
YAW_RECOVER_MAX_S = 8.0
# Below this a sample is not a measurement of steering, it is a measurement of
# noise. The chassis's own pull is about 1.3 deg/s and repeats of the same
# differential land half a degree a second apart, so a steering response smaller
# than this cannot be told from either. Such points are dropped rather than
# recorded, and the curve runs linearly to the origin from the smallest one that
# survives -- which it has to pass through anyway.
MIN_MEASURABLE_DPS = 0.5


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def stored_curves():
    """The pivot curve and, once measured, the rolling steering curve.

    Both matter here, and which one the running `base_node` is using decides how a
    request on `/cmd_vel` becomes a PWM difference. Reading the same store it
    reads is what keeps `--differentials` aiming at the differential it names
    rather than at whatever the previous calibration would have produced.
    """
    try:
        with open(STORE) as fh:
            store = json.load(fh)
    except (OSError, ValueError):
        return None, None

    def points(key):
        got = store.get(key)
        return [[float(p), float(v)] for p, v in got] if got else None

    return points("turn_pwm_points"), points("steer_pwm_points")


def request_for_differential(target, points, steer_points=None):
    """The `/cmd_vel` rotation that makes the mixer emit `target` PWM of steering.

    Going in through `/cmd_vel` keeps the measurement on the path Nav2 uses, but
    it means the requests land wherever the current mixer happens to put them --
    which left the small end, the end that matters, with one point in it. The
    mixer's steering term is monotonic, so the request that produces a chosen
    differential can simply be searched for. What is being measured is unchanged:
    a PWM pair goes to the wheels and the gyro says what the chassis did.

    The search has to use the same curve the running `base_node` is using, or it
    aims at a differential and the rover is given a quite different one.
    """
    lo, hi = 0.0, 200.0
    for _ in range(60):
        mid = (lo + hi) / 2.0
        if steer_pwm(mid, points, driving=True,
                     steer_points=steer_points) < target:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


class SteerGain(Node):
    """Drives the bursts and records what the gyro made of them."""

    def __init__(self, args):
        super().__init__("steer_gain")
        self.args = args
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_subscription(String, "base_state", self.on_state, 10)
        self.create_subscription(LaserScan, "scan", self.on_scan,
                                 qos_profile_sensor_data)
        self.yaw = None
        self.rate = 0.0
        self.speed = 0.0
        self.pwm = None
        self.ahead_m = None
        self.odom_at = 0.0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def map_yaw(self):
        """Heading in the *map* frame, which is the lidar's opinion rather than
        the gyro's.

        `odom -> base_link` is the gyro integrated and nothing else. `map ->
        base_link` has slam_toolbox's scan matching on top of it, so the two
        disagreeing over a drive is the gyro being wrong about that drive. That is
        the whole point of asking: a rover can curve because it really curved, or
        because its gyro said it did, and only one of those is fixed by steering.
        """
        try:
            at = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.3))
        except tf2_ros.TransformException:
            return None
        q = at.transform.rotation
        return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))

    # --- what the rover tells us ------------------------------------------------
    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
        self.rate = float(msg.twist.twist.angular.z)
        self.speed = float(msg.twist.twist.linear.x)
        self.odom_at = time.monotonic()

    def on_state(self, msg):
        try:
            self.pwm = json.loads(msg.data).get("pwm")
        except ValueError:
            pass

    def on_scan(self, msg):
        """How far the rover could go before hitting something, for the guard.

        A *corridor* the width of the rover, not a cone. A cone takes the nearest
        return anywhere within it, so at a half-angle wide enough to be safe close
        up it also catches a wall two metres off to the side and calls the rover
        blocked -- which it did, refusing bursts with four and a half metres of
        clear floor in front of it. What matters is whether something is in the
        way, and that is a question about lateral offset, not about bearing.
        """
        best = None
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x, y = r * math.cos(angle), r * math.sin(angle)
            if x > 0 and abs(y) <= GUARD_CORRIDOR_HALF_M:
                if best is None or x < best:
                    best = x
        self.ahead_m = best

    # --- driving ----------------------------------------------------------------
    def send(self, linear, angular_dps):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = math.radians(angular_dps)
        self.pub.publish(msg)

    def spin_for(self, seconds, linear, angular_dps, collect=False, settle=0.0):
        """Hold one command for `seconds`, optionally averaging the gyro.

        Returns the mean measured rate and the PWM pair that was on the motors,
        both taken only after `settle` so that the chassis accelerating does not
        get counted as the steady-state response.
        """
        end = time.monotonic() + seconds
        started = time.monotonic()
        rates, speeds, pairs = [], [], []
        while time.monotonic() < end:
            self.send(linear, angular_dps)
            rclpy.spin_once(self, timeout_sec=1.0 / TICK_HZ)
            if collect and time.monotonic() - started >= settle:
                if time.monotonic() - self.odom_at < 0.5:
                    rates.append(self.rate)
                    speeds.append(self.speed)
                    if self.pwm:
                        pairs.append(tuple(self.pwm))
        pair = pairs[-1] if pairs else None
        if not rates:
            return None, None, pair
        return (math.degrees(sum(rates) / len(rates)),
                sum(speeds) / len(speeds), pair)

    def halt(self, seconds=STOP_S):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=1.0 / TICK_HZ)

    def clear_ahead(self):
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.ahead_m is not None:
                break
        return self.ahead_m

    def recover_yaw(self, target):
        """Pivot back onto `target`, so the next burst starts where this one did."""
        deadline = time.monotonic() + YAW_RECOVER_MAX_S
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=1.0 / TICK_HZ)
            if self.yaw is None:
                continue
            error = math.degrees(wrap(target - self.yaw))
            if abs(error) <= YAW_TOLERANCE_DEG:
                break
            # Proportional, floored at something the chassis will actually pivot
            # at and capped well under the lidar's smear limit.
            wanted = max(14.0, min(35.0, abs(error) * 1.2))
            self.send(0.0, math.copysign(wanted, error))
        self.halt(0.4)

    # --- the run ----------------------------------------------------------------
    def sample(self, request_dps, sign):
        """One burst: forward with a steering request, then undo it."""
        asked = request_dps * sign
        near = self.clear_ahead()
        if near is not None and near < GUARD_MIN_M:
            return {"asked_dps": asked, "skipped": "only %.2f m ahead" % near}

        start_yaw = self.yaw
        settle = self.args.burst * SETTLE_FRACTION
        got, rolled, pair = self.spin_for(self.args.burst, self.args.speed, asked,
                                          collect=True, settle=settle)
        self.halt()
        # Straight back the way we came, then pivot off whatever heading the burst
        # left behind. Reversing straight does not retrace the arc exactly, which
        # is why the requests alternate in sign.
        self.spin_for(self.args.burst, -self.args.speed, 0.0)
        self.halt()
        if start_yaw is not None:
            self.recover_yaw(start_yaw)

        row = {"asked_dps": asked, "measured_dps": got, "pwm": pair,
               "rolled_ms": rolled}
        if pair:
            row["differential"] = (pair[1] - pair[0]) / 2.0
        if got is not None and abs(asked) > 1e-6:
            row["ratio"] = abs(got) / abs(asked)
        return row

    def straight(self, metres):
        """Drive straight and ask both the gyro and the lidar what happened.

        No steering request at all, so anything that comes out is the chassis
        going its own way -- or the gyro saying it did.
        """
        near = self.clear_ahead()
        if near is not None and near < metres + GUARD_MIN_M:
            print("only %.2f m ahead, need %.2f" % (near, metres + GUARD_MIN_M))
            return None
        for _ in range(40):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.yaw is not None and self.map_yaw() is not None:
                break
        start_odom, start_map = self.yaw, self.map_yaw()
        seconds = metres / self.args.speed
        self.spin_for(seconds, self.args.speed, 0.0)
        self.halt(1.5)
        for _ in range(40):          # let slam_toolbox match on the new scan
            rclpy.spin_once(self, timeout_sec=0.05)
        end_odom, end_map = self.yaw, self.map_yaw()
        # Straight back to roughly where this started, so the check can repeat
        # without walking the rover across the room.
        self.spin_for(seconds, -self.args.speed, 0.0)
        self.halt()
        if None in (start_odom, start_map, end_odom, end_map):
            print("could not read both frames")
            return None
        d_odom = math.degrees(wrap(end_odom - start_odom))
        d_map = math.degrees(wrap(end_map - start_map))
        print("  gyro %+6.2f deg (%+.2f deg/s)   lidar %+6.2f deg (%+.2f deg/s)"
              "   differ %+5.2f"
              % (d_odom, d_odom / seconds, d_map, d_map / seconds,
                 d_odom - d_map))
        return {"metres": metres, "seconds": seconds,
                "gyro_deg": d_odom, "lidar_deg": d_map,
                "gyro_dps": d_odom / seconds, "lidar_dps": d_map / seconds}

    def run(self):
        rows = []
        sign = 1
        for request in self.args.requests:
            row = self.sample(request, sign)
            rows.append(row)
            if "skipped" in row:
                print("  %5.1f deg/s asked -- skipped, %s"
                      % (row["asked_dps"], row["skipped"]))
            elif row["measured_dps"] is None:
                print("  %5.1f deg/s asked -- no odometry came back"
                      % row["asked_dps"])
            else:
                print("  %+6.1f asked  PWM %-10s diff %+6.1f  ->  %+7.1f measured"
                      "  at %.2f m/s   (%.1fx)"
                      % (row["asked_dps"], row["pwm"], row["differential"],
                         row["measured_dps"], row["rolled_ms"] or 0.0,
                         row["ratio"]))
            sign = -sign
        return rows


def save_calibration(rows, args, write=True):
    """Merge what was measured into the store the mixer reads.

    Two things go in, and they are separate measurements of separate faults.
    `steer_pwm_points` is the differential-to-rotation curve while rolling, which
    replaces the pivot curve the mixer had been steering on. It is built from the
    *symmetric* half of each pair of samples -- one left, one right, at the same
    differential -- so that the chassis's own pull does not end up baked into the
    curve as a lopsided response.

    `straight_bias_deg_per_m` is that pull, taken from the common half of the
    same pairs, or from a `--straight` run if one was done. Only the `--straight`
    figure is corroborated by the lidar; the paired figure is the gyro alone.
    """
    store = {}
    try:
        with open(STORE) as fh:
            store = json.load(fh)
    except (OSError, ValueError):
        pass

    straight = [r for r in rows if r.get("lidar_dps") is not None]
    if straight:
        # Median, not mean. Scan matching occasionally jumps -- one run in four
        # here reported the rover turning the other way, against three that agreed
        # with the gyro to a tenth of a degree a second -- and a mean lets that one
        # run move the calibration by a third. The median simply ignores it.
        got = sorted(r["lidar_dps"] for r in straight)
        middle = len(got) // 2
        lidar = got[middle] if len(got) % 2 else (got[middle - 1] + got[middle]) / 2
        store["straight_bias_deg_per_m"] = round(lidar / args.speed, 4)
        store["straight_bias_measured_at_ms"] = args.speed
        print("saved straight-line bias %+.2f deg per metre, the median of "
              "%d runs spanning %+.2f to %+.2f deg/s"
              % (store["straight_bias_deg_per_m"], len(got), got[0], got[-1]))

    # Pair samples that share a differential magnitude but not its sign, keyed on
    # the sign that was *asked for* rather than the sign that came back. Those are
    # not the same thing and assuming they were is a trap: at the smallest
    # differentials this chassis's own pull is larger than the steering being
    # measured, so a request to turn right still turns the rover left, and keying
    # on the result quietly drops such a pair into the unpaired path -- where the
    # pull it was supposed to cancel gets recorded as steering instead.
    by_diff = {}
    for r in rows:
        if r.get("measured_dps") is None or not r.get("differential"):
            continue
        d = r["differential"]
        entry = by_diff.setdefault(abs(d), {"+": [], "-": []})
        entry["+" if d > 0 else "-"].append(r["measured_dps"])

    def mean(xs):
        return sum(xs) / len(xs)

    paired, unpaired, bias = {}, {}, []
    for diff, got in by_diff.items():
        if got["+"] and got["-"]:
            hi, lo = mean(got["+"]), mean(got["-"])
            paired[diff] = (hi - lo) / 2.0
            bias.append((hi + lo) / 2.0)
        else:
            side = got["+"] or got["-"]
            unpaired[diff] = (mean(side), 1.0 if got["+"] else -1.0)

    # An unpaired sample still carries the pull, so take it off using the pull the
    # pairs measured. Without any pairs to measure it against there is nothing
    # honest to subtract, so such a sample is dropped rather than guessed at.
    pull = mean(bias) if bias else None
    points = [[d, round(v, 3)] for d, v in paired.items()]
    for diff, (value, sign) in unpaired.items():
        if pull is None:
            print("dropping %g PWM: measured once, and with no pair there is no "
                  "way to separate steering from the chassis's own pull" % diff,
                  file=sys.stderr)
            continue
        points.append([diff, round(sign * (value - pull), 3)])
    kept = [[d, v] for d, v in points if v >= MIN_MEASURABLE_DPS]
    for d, v in points:
        if v < MIN_MEASURABLE_DPS:
            print("dropping %g PWM: %.2f deg/s is below what this can measure "
                  "against a %.1f deg/s pull" % (d, v, pull or 0.0))
    points = sorted(kept)
    if points:
        merged = {} if args.sources else {
            p[0]: p[1] for p in store.get("steer_pwm_points") or []}
        merged.update({p[0]: p[1] for p in points})
        curve = [[d, merged[d]] for d in sorted(merged)]
        # A curve that is not monotonic cannot be inverted, and inverting it is
        # the only thing it is for. Noise at the small end is the likely cause,
        # so say which point rather than silently sorting it away.
        for (d0, v0), (d1, v1) in zip(curve, curve[1:]):
            if v1 <= v0:
                print("refusing to save: %g PWM gives %.2f deg/s but %g gives "
                      "%.2f -- re-measure, the curve has to climb"
                      % (d0, v0, d1, v1), file=sys.stderr)
                return
        store["steer_pwm_points"] = curve
        print("saved steering curve: %s"
              % ", ".join("%g=%.2f deg/s" % (d, v) for d, v in curve))
        if bias:
            print("(the pairs also imply a pull of %+.2f deg/s, gyro only)"
                  % (sum(bias) / len(bias)))

    if not write:
        return
    with open(STORE, "w") as fh:
        json.dump(store, fh, indent=1)
    print("written to %s" % STORE)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--speed", type=float, default=0.35,
                   help="m/s to roll at while steering")
    p.add_argument("--burst", type=float, default=1.0,
                   help="seconds of rolling per sample")
    p.add_argument("--requests", default="2,5,10,20,45",
                   help="steering requests to try, deg/s")
    p.add_argument("--differentials", default="",
                   help="target these PWM differentials instead, which is how to "
                        "get even coverage of the small end")
    p.add_argument("--out", default="",
                   help="write the measurements here as JSON")
    p.add_argument("--from", dest="sources", default="",
                   help="comma-separated JSON files from earlier --out runs; "
                        "recompute the calibration from those instead of driving")
    p.add_argument("--save", action="store_true",
                   help="merge what was measured into %s, which is where the "
                        "mixer reads it from" % STORE)
    p.add_argument("--straight", type=float, default=0.0,
                   help="instead, drive this many metres straight and compare "
                        "what the gyro and the lidar say the heading did")
    p.add_argument("--repeat", type=int, default=1,
                   help="how many times to repeat the straight-line check")
    p.add_argument("--dry-run", action="store_true",
                   help="say what would be driven and drive nothing")
    args = p.parse_args()
    args.requests = [float(x) for x in args.requests.split(",") if x.strip()]
    if args.differentials:
        points, steer_points = stored_curves()
        if not points:
            print("no turn curve in %s to solve against" % STORE, file=sys.stderr)
            return 1
        wanted = [float(x) for x in args.differentials.split(",") if x.strip()]
        args.requests = [request_for_differential(d, points, steer_points)
                         for d in wanted]
        print("targeting PWM differentials %s"
              % ", ".join("%g" % d for d in wanted))

    if args.sources:
        # Every sample is a PWM pair and the rotation the gyro measured for it, so
        # runs done at the same speed on the same day combine straightforwardly --
        # and combining them is better than any one of them, because the repeats
        # are what average the noise out of the small end.
        rows = []
        for path in args.sources.split(","):
            with open(path.strip()) as fh:
                loaded = json.load(fh)
            if abs(loaded.get("speed_ms", args.speed) - args.speed) > 0.01:
                print("%s was measured at %.2f m/s, not %.2f -- skipping"
                      % (path, loaded["speed_ms"], args.speed), file=sys.stderr)
                continue
            rows.extend(loaded.get("samples", []))
        print("recomputing from %d samples across %d runs"
              % (len(rows), len(args.sources.split(","))))
        if not args.save:
            print("(add --save to write it to %s)" % STORE)
        save_calibration(rows, args, write=args.save)
        return 0

    reach = args.speed * args.burst
    if args.straight > 0:
        print("straight-line check: %.2f m at %.2f m/s, no rotation requested"
              % (args.straight, args.speed))
    else:
        print("steering gain: %d samples at %.2f m/s, %.1f s each"
              % (len(args.requests), args.speed, args.burst))
        print("each sample rolls %.2f m forward, reverses %.2f m, then pivots back"
              % (reach, reach))
        print("requests: %s deg/s, alternating direction"
              % ", ".join("%g" % r for r in args.requests))
    if args.dry_run:
        print("dry run -- nothing driven")
        return 0

    rclpy.init()
    node = SteerGain(args)
    rows = []
    try:
        if node.clear_ahead() is None:
            print("no /scan yet -- is the stack up?", file=sys.stderr)
        else:
            print("clear ahead: %.2f m" % node.ahead_m)
        print()
        if args.straight > 0:
            print("driving %.2f m straight, asking for no rotation at all:"
                  % args.straight)
            rows = [r for r in (node.straight(args.straight)
                                for _ in range(args.repeat)) if r]
            if rows:
                gyro = sum(r["gyro_dps"] for r in rows) / len(rows)
                lidar = sum(r["lidar_dps"] for r in rows) / len(rows)
                print()
                print("over %d runs the lidar says the chassis really curved "
                      "%+.2f deg/s" % (len(rows), lidar))
                print("and the gyro reported %+.2f deg/s, so the gyro is wrong by "
                      "%+.2f" % (gyro, gyro - lidar))
                print("and the remaining %+.2f deg/s is the rover genuinely "
                      "pulling to one side." % lidar)
                print("at %.2f m/s that is %+.2f degrees per metre driven."
                      % (node.args.speed, lidar / node.args.speed))
        else:
            rows = node.run()
    finally:
        try:
            node.halt(0.8)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    good = [r for r in rows if r.get("ratio")]
    if good:
        print()
        print("the steering channel is over-responding by %.1fx on average"
              % (sum(r["ratio"] for r in good) / len(good)))
        print("(1.0 would mean the rover turns at the rate it was asked to)")
    if args.save:
        save_calibration(rows, args)
    if args.out:
        with open(args.out, "w") as fh:
            json.dump({"speed_ms": args.speed, "burst_s": args.burst,
                       "samples": rows}, fh, indent=1)
        print("written to %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
