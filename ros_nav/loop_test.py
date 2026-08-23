#!/usr/bin/env python3
"""Drive a closed circuit and measure whether the map closes it.

This is the test the whole `ros_nav/` stack exists to pass. The scan matcher it
replaced has no loop closure and says so: the pose drifts monotonically and for
ever, so a rover driven round a circuit believes it has ended up somewhere it has
not, and "go back to where you started" is not a thing it can do. `slam_toolbox`
keeps a pose graph, recognises somewhere it has been, and bends the graph to
agree. Whether that actually happens on this rover, on this floor, is a question
only the rover can answer.

    python3 loop_test.py --dry-run          # check everything, move nothing
    python3 loop_test.py                    # a 1.5 m square, four corners
    python3 loop_test.py --side 2.0 --laps 2

## What is being compared

Three frames, and the whole test is in the difference between two of them:

  `odom` -> `base_link`   dead reckoning: wheel ticks and the gyro, integrated.
                          It has no idea it has been here before. Its closure
                          error is the drift, and it only grows.
  `map` -> `base_link`     where slam_toolbox thinks the rover is, having matched
                          every scan against the map and optimised the graph.
  `map` -> `odom`          the correction between them. **This is the interesting
                          one.** It is what loop closure moves, and a step change
                          in it is a closure firing.

So the measurement is: drive back to the start, then compare how far each of the
two poses thinks it is from where it began. If the map's error is materially
smaller than dead reckoning's, the graph is doing its job. If they are the same,
it is not -- and no amount of map that *looks* right changes that.

## What this deliberately does not use

Nav2. The rover is driven here by commanding `/cmd_vel` directly in a fixed
pattern, because a test of SLAM should not depend on a controller: if Nav2 cut a
corner or a recovery behaviour spun the rover, the circuit would not be the
circuit and the closure error would be measuring the controller. Driving Nav2
round the same square is a good test *of Nav2*, and a separate one.

**This moves the rover several metres.** Every leg is watched by the lidar and
stops early if anything comes inside the margin -- a leg cut short is fine, the
test is the returning, not the distance -- and the wheels are stopped on every
exit path including a crash. Somebody should be watching it.
"""

import argparse
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
from calibrate_chassis import AHEAD_HALF_DEG, normalise            # noqa: E402

# Driven well under the measured envelope. The point is a circuit the rover can
# follow accurately, not a fast one, and a skid-steer chassis that is pushed
# leaves more of its odometry on the floor as slip.
DRIVE_MS = 0.35
TURN_DPS = 25.0
MARGIN_M = 0.55
SETTLE_S = 2.0


def yaw_of(transform):
    q = transform.transform.rotation
    return math.degrees(2.0 * math.atan2(q.z, q.w))


class LoopTest(Node):

    def __init__(self, args):
        super().__init__("loop_test")
        self.args = args
        self.scan = None
        self.odom = None
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan, "scan", self._on_scan, qos)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        # Every correction seen during the run, so a closure can be spotted as the
        # step it is rather than inferred from the endpoints.
        self.corrections = []

    def _on_scan(self, msg):
        self.scan = msg

    def _on_odom(self, msg):
        self.odom = msg

    # --- reading ---------------------------------------------------------------
    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            self.note_correction()

    def lookup(self, parent, child):
        try:
            return self.buffer.lookup_transform(
                parent, child, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.3))
        except Exception:
            return None

    def where(self, frame):
        """(x, y, yaw degrees) of base_link in `frame`, or None."""
        t = self.lookup(frame, "base_link")
        if t is None:
            return None
        return (t.transform.translation.x, t.transform.translation.y, yaw_of(t))

    def note_correction(self):
        """Record map -> odom, which is what a loop closure moves."""
        t = self.lookup("map", "odom")
        if t is None:
            return
        self.corrections.append((time.monotonic(), t.transform.translation.x,
                                 t.transform.translation.y, yaw_of(t)))

    def clearance(self, bearing_deg=0.0):
        msg = self.scan
        if msg is None:
            return None
        nearest = float("inf")
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min:
                continue
            angle = math.degrees(msg.angle_min + i * msg.angle_increment)
            if abs(normalise(angle - bearing_deg)) <= AHEAD_HALF_DEG:
                nearest = min(nearest, r)
        return nearest

    # --- driving ---------------------------------------------------------------
    def stop(self):
        self.cmd.publish(Twist())

    def hold(self, linear, angular, seconds, guard=None):
        """Publish one velocity for `seconds`, stopping early if `guard` says so."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        end = time.monotonic() + seconds
        stopped = None
        while time.monotonic() < end:
            if guard is not None:
                why = guard()
                if why:
                    stopped = why
                    break
            self.cmd.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            self.note_correction()
        self.stop()
        self.spin_for(SETTLE_S)
        return stopped

    def leg(self, metres):
        """Drive forward, watching what is in front the whole way."""
        def guard():
            clear = self.clearance()
            if clear is None:
                return "lost the scan"
            if clear < MARGIN_M:
                return "%.2f m ahead" % clear
            return None

        clear = self.clearance()
        if clear is not None and clear < MARGIN_M:
            return "already only %.2f m ahead" % clear
        return self.hold(DRIVE_MS, 0.0, metres / DRIVE_MS, guard=guard)

    def survey(self):
        """Clearance in a cone swept all the way round, as (bearing, metres).

        Worth printing before a circuit as well as using: it says how big a square
        the room will actually take, which is otherwise discovered one aborted leg
        at a time.
        """
        out = []
        for bearing in range(-180, 180, 15):
            clear = self.clearance(bearing)
            if clear is not None:
                out.append((bearing, clear))
        return out

    def face_open(self, wanted):
        """Turn to the most open bearing, if the rover is not already on one.

        A circuit has to start pointing somewhere it can drive. Turning to find
        that is a legitimate part of the test rather than something a person
        should have to do: it costs one corner's worth of driving and it is the
        difference between a run that measures closure and a run whose first leg
        aborts against a wall 40 cm away.
        """
        ahead = self.clearance()
        if ahead is not None and ahead >= wanted:
            return None
        options = [(clear, bearing) for bearing, clear in self.survey()]
        if not options:
            return "no scan to choose a heading from"
        best_clear, best_bearing = max(options)
        if best_clear < wanted:
            return ("the most open direction has only %.2f m, and the circuit needs "
                    "%.2f -- move the rover somewhere with more floor"
                    % (best_clear, wanted))
        self.get_logger().info(
            "turning %+d degrees to face the open floor (%.2f m there, %.2f m ahead "
            "now)" % (best_bearing, best_clear, ahead if ahead else 0.0))
        return self.corner(best_bearing)

    def corner(self, degrees):
        """Turn on the spot by roughly `degrees`, closing the loop on the gyro.

        Servoed rather than timed, because the corners are what decide whether a
        square is a square: four timed turns that are each 10% short leave the
        rover 36 degrees off its own start, and the closure error then measures
        the turns rather than the mapping.
        """
        start = self.where("odom")
        if start is None:
            return "no odom transform"
        target = degrees
        turned = 0.0
        last = start[2]
        rate = math.radians(TURN_DPS) * (1 if degrees > 0 else -1)
        deadline = time.monotonic() + abs(degrees) / TURN_DPS * 3.0 + 6.0
        while abs(turned) < abs(target) - 2.0 and time.monotonic() < deadline:
            msg = Twist()
            msg.angular.z = rate
            self.cmd.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            self.note_correction()
            now = self.where("odom")
            if now is not None:
                turned += normalise(now[2] - last)
                last = now[2]
        self.stop()
        self.spin_for(SETTLE_S)
        return None if abs(turned) >= abs(target) - 8.0 else (
            "turned %.0f of %.0f degrees" % (turned, target))


def report(label, start, end):
    if start is None or end is None:
        print("  %-24s could not be read" % label)
        return None
    dx, dy = end[0] - start[0], end[1] - start[1]
    dist = math.hypot(dx, dy)
    dyaw = normalise(end[2] - start[2])
    print("  %-24s %.3f m and %+.1f deg from where it started" % (label, dist, dyaw))
    return dist


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--side", type=float, default=1.5, help="metres per leg")
    p.add_argument("--corners", type=int, default=4)
    p.add_argument("--laps", type=int, default=1)
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()

    rclpy.init()
    node = LoopTest(args)
    try:
        node.get_logger().info("waiting for the scan, the map and odometry...")
        node.spin_for(5.0)
        if node.scan is None:
            print("no /scan -- is the stack running?", file=sys.stderr)
            return 1
        start_map = node.where("map")
        start_odom = node.where("odom")
        if start_map is None or start_odom is None:
            print("no map or odom transform -- is slam_toolbox active?",
                  file=sys.stderr)
            return 1
        print("start: map %.3f,%.3f at %+.0f deg; clear ahead %.2f m"
              % (start_map[0], start_map[1], start_map[2], node.clearance()))
        print("room, by bearing: " + ", ".join(
            "%+d:%s" % (b, ("%.1f" % c) if math.isfinite(c) else "8+")
            for b, c in node.survey()))
        if args.dry_run:
            print("dry run: everything is connected, nothing was moved")
            return 0

        # Point it somewhere it can actually drive before the circuit starts, and
        # take the start pose *after* that turn -- the closure error is measured
        # from where the circuit began, not from where the rover happened to be
        # parked when the script was run.
        why = node.face_open(args.side * 0.6 + MARGIN_M)
        if why:
            print("cannot start: %s" % why, file=sys.stderr)
            return 1
        start_map = node.where("map")
        start_odom = node.where("odom")
        node.corrections = []

        turn = 360.0 / args.corners
        for lap in range(1, args.laps + 1):
            for corner in range(args.corners):
                cut = node.leg(args.side)
                print("  lap %d leg %d: %s" % (lap, corner + 1,
                                               ("cut short, %s" % cut) if cut
                                               else "%.1f m" % args.side))
                short = node.corner(turn)
                if short:
                    print("  lap %d corner %d: %s" % (lap, corner + 1, short))
        print()
        end_map = node.where("map")
        end_odom = node.where("odom")
        print("CLOSURE ERROR -- how far each pose thinks it is from the start")
        map_err = report("the map says", start_map, end_map)
        odom_err = report("dead reckoning says", start_odom, end_odom)

        # The correction is the whole argument. If it never moved, slam_toolbox
        # accepted dead reckoning unchanged and the two errors above will agree.
        if node.corrections:
            xs = [c[1] for c in node.corrections]
            ys = [c[2] for c in node.corrections]
            yaws = [c[3] for c in node.corrections]
            steps = [math.hypot(b[1] - a[1], b[2] - a[2])
                     for a, b in zip(node.corrections, node.corrections[1:])]
            print()
            print("MAP -> ODOM CORRECTION  (%d samples)" % len(node.corrections))
            print("  moved over the run     %.3f m, %+.1f deg"
                  % (math.hypot(xs[-1] - xs[0], ys[-1] - ys[0]),
                     normalise(yaws[-1] - yaws[0])))
            print("  biggest single step    %.3f m" % (max(steps) if steps else 0.0))
            print("  a step of a few centimetres is the scan match nudging the pose;")
            print("  a step of tens is a loop closure bending the graph.")

        if map_err is not None and odom_err is not None:
            print()
            if odom_err < 0.05:
                print("VERDICT: dead reckoning barely drifted over this circuit, so")
                print("  it does not test closure. Drive a longer or twistier one.")
            elif map_err < odom_err * 0.6:
                print("VERDICT: the map closed the loop. Dead reckoning was %.2f m"
                      % odom_err)
                print("  out and the map %.2f m, so the graph removed %.0f%% of the"
                      % (map_err, 100 * (1 - map_err / odom_err)))
                print("  drift. This is what lidar_slam/ could not do.")
            else:
                print("VERDICT: the map did NOT meaningfully improve on dead")
                print("  reckoning (%.2f m against %.2f m). Either no closure fired"
                      % (map_err, odom_err))
                print("  or the circuit did not return close enough to be")
                print("  recognised -- check loop_search_maximum_distance.")
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
            node.spin_for(0.5)
            node.stop()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
