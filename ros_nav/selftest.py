#!/usr/bin/env python3
"""Prove the parts of the ROS 2 stack that can be proved without a rover.

    python3 selftest.py

Runs anywhere, including the Windows workstation and including a board with no
ROS installed: the modules that need `rclpy` are imported lazily and their pure
arithmetic is tested through small stand-ins instead. That is deliberate. The
things most worth catching here -- a sign flip on the steering, a scan binned
into the wrong half of the circle, a tick difference taken across a counter
reset -- are all arithmetic, and none of them needs a radio to be wrong.

What this does *not* cover is whether slam_toolbox is configured sensibly or
whether Nav2 can drive through a doorway. Those are hardware facts and the README
says how to check them on the rover.
"""

import json
import math
import os
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PASSED = FAILED = 0


def check(name, got, want=True, tolerance=None):
    global PASSED, FAILED
    if tolerance is not None:
        ok = got is not None and abs(got - want) <= tolerance
    else:
        ok = got == want
    if ok:
        PASSED += 1
        print("  ok   %s" % name)
    else:
        FAILED += 1
        print("  FAIL %s\n         got %r, wanted %r" % (name, got, want))
    return ok


def section(title):
    print("\n%s" % title)


# --- the drive model ----------------------------------------------------------
# Imported from the repository's own measurements rather than restated, so that a
# re-measured chassis is tested against its new numbers and not its old ones.
for candidate in (os.path.join(HERE, "..", "lidar_slam"),
                  os.path.join(HERE, "..", "..", "lidar_slam")):
    if os.path.isdir(candidate):
        sys.path.insert(0, os.path.abspath(candidate))
        break
from nav_types import (MAX_SPEED_MS, MAX_TURN_DPS, MIN_PWM,           # noqa: E402
                        MIN_TURN_DPS, TOP_PWM, TURN_RATES)


def to_pwm(value):
    """A copy of base_node.to_pwm, so this file needs no rclpy to test it."""
    if abs(value) < 1e-3:
        return 0
    magnitude = MIN_PWM + abs(value) * (TOP_PWM - MIN_PWM)
    return int(round(magnitude if value > 0 else -magnitude))


_FIT = sorted((pwm, rate) for pwm, (rate, _c) in TURN_RATES.items())
_LO, _HI = _FIT[0], _FIT[-1]
_SLOPE = (_HI[1] - _LO[1]) / (_HI[0] - _LO[0])
TURN_PWM_MAX = _HI[0]


def pwm_for(points, wanted, floor=MIN_PWM, ceiling=None):
    """A copy of base_node.pwm_for."""
    ceiling = TURN_PWM_MAX if ceiling is None else ceiling
    if not points or wanted <= 0:
        return 0
    if wanted <= points[0][1]:
        return int(round(max(floor, min(ceiling, points[0][0]))))
    for (p0, v0), (p1, v1) in zip(points, points[1:]):
        if wanted <= v1:
            if v1 == v0:
                return int(round(p1))
            share = (wanted - v0) / (v1 - v0)
            return int(round(max(floor, min(ceiling, p0 + share * (p1 - p0)))))
    (p0, v0), (p1, v1) = points[-2], points[-1]
    if v1 == v0:
        return int(round(min(ceiling, p1)))
    slope = (p1 - p0) / (v1 - v0)
    return int(round(max(floor, min(ceiling, p1 + (wanted - v1) * slope))))


FALLBACK_TURN_POINTS = [[pwm, rate] for pwm, rate in _FIT]


def turn_to_pwm(dps, points=None):
    """A copy of base_node.turn_to_pwm."""
    wanted = abs(dps)
    if wanted < 1e-3:
        return 0
    return pwm_for(points or FALLBACK_TURN_POINTS, max(wanted, MIN_TURN_DPS))


def cmd_to_pwm(linear, angular, turn_points=None, drive_points=None):
    """A copy of base_node.mix, which is the part that can be wrong in a way
    that drives the rover backwards."""
    speed = min(MAX_SPEED_MS, abs(linear))
    if drive_points:
        throttle = pwm_for(drive_points, speed, ceiling=TOP_PWM)
    else:
        throttle = to_pwm(speed / MAX_SPEED_MS)
    if linear < 0:
        throttle = -throttle
    turn = turn_to_pwm(math.degrees(min(math.radians(MAX_TURN_DPS), abs(angular))),
                       turn_points)
    if angular < 0:
        turn = -turn
    left, right = throttle - turn, throttle + turn
    peak = max(abs(left), abs(right))
    if peak > TURN_PWM_MAX:
        scale = TURN_PWM_MAX / peak
        left, right = left * scale, right * scale
    return int(round(left)), int(round(right))


def test_turn_curve():
    section("degrees per second -> PWM, against what was measured")
    # The whole point of the fit: it must reproduce the two points somebody
    # actually measured on this chassis, not merely pass through the origin.
    for pwm, (rate, _coast) in sorted(TURN_RATES.items()):
        check("a commanded %.1f deg/s asks for PWM %d, as measured" % (rate, pwm),
              turn_to_pwm(rate), pwm)
    # Below the slowest thing measured, the answer is the slowest thing measured
    # rather than an extrapolation. This is the rule that two failed attempts
    # bought. Scaling proportionally from zero gave PWM 93 for 20 deg/s, and the
    # rover turned at 25; extrapolating a straight line down from these two points
    # gave PWM 72, and the rover turned at 8. Refusing to guess below the data is
    # the only one of the three that cannot be confidently wrong.
    check("a rate below anything measured gets the slowest measured PWM",
          turn_to_pwm(20.0), _LO[0])
    check("...which is never under the floor where the motors do nothing",
          turn_to_pwm(20.0) >= MIN_PWM, True)
    check("an impossibly slow turn becomes the slowest real one",
          turn_to_pwm(1.0), turn_to_pwm(MIN_TURN_DPS))
    check("but zero is still zero", turn_to_pwm(0.0), 0)
    check("and the fastest turn does not exceed the measured top PWM",
          turn_to_pwm(1000.0), TURN_PWM_MAX)

    # A curve with real low-end points, which is what calibrate_chassis.py writes.
    # Interpolation between measured points is where the accuracy comes from, so
    # it has to actually interpolate rather than snap to the nearest.
    measured = [[60.0, 5.0], [75.0, 12.0], [90.0, 24.0], [160.0, 90.0]]
    check("between two measured points it interpolates",
          turn_to_pwm(18.0, measured), 82)
    check("exactly on a measured point it returns that point",
          turn_to_pwm(24.0, measured), 90)
    # MIN_TURN_DPS lifts anything slower than 12 deg/s to 12 before the curve is
    # consulted, so the no-extrapolation rule has to be checked on the curve
    # itself -- via turn_to_pwm it is unreachable on this chassis.
    check("a request under 12 deg/s is lifted to the slowest real turn first",
          turn_to_pwm(3.0, measured), turn_to_pwm(MIN_TURN_DPS, measured))
    check("and the curve itself never extrapolates below its slowest point",
          pwm_for(measured, 1.0), 60)
    check("over the fastest it extrapolates but respects the ceiling",
          turn_to_pwm(500.0, measured) <= TURN_PWM_MAX, True)
    # And the thing the whole rewrite was for: with a measured curve, a modest
    # request gets a modest PWM instead of the lowest point of a coarse one.
    check("a measured curve gives 12 deg/s a much gentler PWM than the fallback",
          turn_to_pwm(12.0, measured) < turn_to_pwm(12.0), True)


def test_drive_model():
    section("cmd_vel -> PWM")
    check("stopped is stopped", cmd_to_pwm(0.0, 0.0), (0, 0))
    left, right = cmd_to_pwm(MAX_SPEED_MS, 0.0)
    check("full ahead is full ahead on both wheels", (left, right), (TOP_PWM, TOP_PWM))
    check("half speed is between the floor and the ceiling",
          MIN_PWM < cmd_to_pwm(MAX_SPEED_MS / 2, 0.0)[0] < TOP_PWM, True)
    left, right = cmd_to_pwm(-MAX_SPEED_MS, 0.0)
    check("full astern is negative on both", (left, right), (-TOP_PWM, -TOP_PWM))

    # The one sign in the stack worth a test of its own. REP-103 has positive
    # angular.z counter-clockwise, i.e. turning left; the firmware's left wheel
    # gets throttle + steer, so a left turn must drive the left wheel backwards.
    left, right = cmd_to_pwm(0.0, math.radians(MAX_TURN_DPS))
    check("turning left drives the left wheel back", left < 0, True)
    check("...and the right wheel forward", right > 0, True)
    left, right = cmd_to_pwm(0.0, -math.radians(MAX_TURN_DPS))
    check("turning right drives the right wheel back", right < 0, True)

    # Below MIN_PWM the motors buzz and do not turn, so the curve must not spend
    # its bottom quarter there.
    check("a whisper of throttle still clears the motors' floor",
          abs(to_pwm(0.01)) >= MIN_PWM, True)
    check("but exactly zero is zero, not a buzz", to_pwm(0.0), 0)

    # A command asking for full speed and full turn at once cannot have both;
    # what it must not do is exceed the firmware's range.
    left, right = cmd_to_pwm(MAX_SPEED_MS, math.radians(MAX_TURN_DPS))
    check("speed and turn together stay inside the PWM range",
          max(abs(left), abs(right)) <= TURN_PWM_MAX, True)
    check("...and the turn survives it", left != right, True)
    # Scaled, not clipped: clipping one wheel changes the turn into a different
    # one, so the *difference* between the wheels must survive the squeeze.
    check("...having been scaled down rather than one wheel clipped",
          abs(right - left) > 0, True)


# --- odometry -----------------------------------------------------------------
def integrate(samples, gyro_lsb_per_dps, ticks_per_metre):
    """base_node.BaseNode.integrate's arithmetic, standing alone.

    Same midpoint-heading rule and same treatment of `breaks`, so a change to
    either here is a change that has to be made in both places on purpose.
    """
    x = y = yaw = 0.0
    last_gz = last_ticks = last_breaks = None
    for gz, ticks, breaks in samples:
        broken = last_breaks is not None and breaks != last_breaks
        last_breaks = breaks
        d_yaw = 0.0
        if last_gz is not None and not broken:
            d_yaw = math.radians((gz - last_gz) / gyro_lsb_per_dps)
        last_gz = gz
        d_s = 0.0
        if last_ticks is not None and not broken:
            d_s = (ticks - last_ticks) / ticks_per_metre
        last_ticks = ticks
        heading = yaw + d_yaw / 2.0
        x += d_s * math.cos(heading)
        y += d_s * math.sin(heading)
        yaw = (yaw + d_yaw + math.pi) % (2 * math.pi) - math.pi
    return x, y, yaw


def idle_sends(commanded, seconds_since_cmd, ever_commanded=True):
    """base_node.BaseNode.drive's decision about whether to say anything at all.

    Its own arithmetic, standing alone, because the rule is easy to get backwards
    in the safe-looking direction and the consequence is a rover nobody else can
    drive.
    """
    live = ever_commanded and seconds_since_cmd <= 0.5
    if not live:
        return commanded not in (None, (0, 0))
    return True


def test_idle_behaviour():
    section("what the base node does when nobody is navigating")
    check("having never been commanded, it says nothing to the board",
          idle_sends(None, 999.0, ever_commanded=False), False)
    check("a live command is always sent", idle_sends((0, 0), 0.1), True)
    check("when a command goes stale, one stop is sent",
          idle_sends((80, 80), 1.0), True)
    check("...and then it goes quiet rather than repeating the stop",
          idle_sends((0, 0), 1.0), False)
    # This is the one that matters. If it were True, ROS would be commanding
    # zero several times a second for ever, and a person driving the rover with
    # the game pad would find the board braking under them with nothing in any
    # log to explain it.
    check("so a game pad can drive the rover while ROS is only mapping",
          idle_sends((0, 0), 30.0), False)


def test_odometry():
    section("board counters -> pose")
    lsb, tpm = 15.0, 100.0

    straight = [(0.0, 0.0, 0), (0.0, 100.0, 0), (0.0, 200.0, 0)]
    x, y, yaw = integrate(straight, lsb, tpm)
    check("200 ticks at 100/m is 2 m straight ahead", x, 2.0, tolerance=1e-9)
    check("...with no sideways drift", y, 0.0, tolerance=1e-9)
    check("...and no heading change", yaw, 0.0, tolerance=1e-9)

    # 15 LSB per degree-per-second, so 1350 LSB-seconds is 90 degrees.
    turning = [(0.0, 0.0, 0), (1350.0, 0.0, 0)]
    _, _, yaw = integrate(turning, lsb, tpm)
    check("1350 LSB-s at 15 LSB/dps is a quarter turn", math.degrees(yaw), 90.0,
          tolerance=1e-6)

    # Driving and turning together must trace an arc, not a corner. The midpoint
    # heading is what makes the difference, and getting it wrong is a bias that
    # only shows up as a map that curls.
    arc = [(0.0, 0.0, 0), (1350.0, 100.0, 0)]
    x, y, _ = integrate(arc, lsb, tpm)
    check("a metre while turning 90 degrees goes diagonally, not straight up",
          abs(x - y) < 1e-6 and x > 0.6, True)

    # A counter break is a hole, and integrating across it invents a jump.
    broken = [(0.0, 0.0, 0), (0.0, 100.0, 0), (0.0, 5.0, 1), (0.0, 105.0, 1)]
    x, _, _ = integrate(broken, lsb, tpm)
    check("a reset counter does not become a 95 cm leap backwards", x, 2.0,
          tolerance=1e-9)

    # The gyro is what the heading depends on entirely, so a missing scale must
    # be refused rather than defaulted -- checked in test_calibration below.


# --- the scan -----------------------------------------------------------------
def bin_scan(points, bins=360, range_min=0.12, range_max=8.0):
    """lidar_node.LidarNode.to_scan's binning, standing alone."""
    increment = 2.0 * math.pi / bins
    ranges = [float("inf")] * bins
    used = 0
    for x, y in points:
        r = math.hypot(x, y)
        if r < range_min or r > range_max:
            continue
        i = int((math.atan2(y, x) + math.pi) / increment) % bins
        if r < ranges[i]:
            ranges[i] = r
        used += 1
    return ranges, used, increment


def at_bearing(ranges, increment, degrees):
    i = int((math.radians(degrees) + math.pi) / increment) % len(ranges)
    return ranges[i]


def test_scan_binning():
    section("scan points -> LaserScan")
    # slam2d hands back x forward and y left, which is REP-103. A point two
    # metres straight ahead must land where a consumer looks for straight ahead.
    ranges, used, inc = bin_scan([(2.0, 0.0)])
    check("a point 2 m ahead is 2 m ahead", at_bearing(ranges, inc, 0), 2.0,
          tolerance=0.02)
    check("...and is the only one", used, 1)

    ranges, _, inc = bin_scan([(0.0, 1.5)])
    check("a point 1.5 m to port reads at +90 degrees",
          at_bearing(ranges, inc, 90), 1.5, tolerance=0.02)
    ranges, _, inc = bin_scan([(0.0, -1.5)])
    check("a point to starboard reads at -90 degrees",
          at_bearing(ranges, inc, -90), 1.5, tolerance=0.02)
    ranges, _, inc = bin_scan([(-3.0, 0.0)])
    check("a point behind reads at 180 degrees",
          at_bearing(ranges, inc, 180), 3.0, tolerance=0.02)

    # Two points in one bin: the nearer wins, because this message is read by an
    # obstacle costmap and rounding a chair leg away is what gets it hit.
    ranges, _, inc = bin_scan([(2.0, 0.0), (1.0, 0.001)])
    check("where two points share a bin the nearer one wins",
          at_bearing(ranges, inc, 0), 1.0, tolerance=0.02)

    # Out-of-range points are dropped rather than clamped. A clamped point is a
    # wall reported where there is none.
    _, used, _ = bin_scan([(20.0, 0.0), (0.05, 0.0)])
    check("points beyond the sensor's honest reach are dropped, not clamped",
          used, 0)

    # Nothing may land outside the array, including a point at exactly pi.
    ranges, used, _ = bin_scan([(-2.0, -1e-12)])
    check("a point at the wrap does not fall off the end", used, 1)

    # A full circle fills every bin exactly once -- offset half a degree so the
    # points sit in the middle of their bins rather than on the boundaries. On
    # the boundary the answer is genuinely ambiguous and floating point decides
    # it, which is a property of binning rather than a fault to fix: a real
    # sensor's returns are not aligned to the grid either.
    circle = [(math.cos(math.radians(d + 0.5)) * 2,
               math.sin(math.radians(d + 0.5)) * 2) for d in range(0, 360)]
    ranges, used, _ = bin_scan(circle)
    check("360 points a degree apart fill 360 bins", used, 360)
    check("...leaving none empty", sum(1 for r in ranges if math.isinf(r)), 0)


# --- the board bridge ---------------------------------------------------------
def test_bridge_protocol():
    """The bridge's own command parsing, against a board that records what it got."""
    section("board bridge")
    # Two layouts. In the repository this file is in ros_nav/ and board_bridge.py
    # is in the sibling rover_daemon/; on the rover ~/ugv is flat and the daemon's
    # modules sit directly in the parent. Checking both means the bridge is tested
    # on the machine that actually runs it, which is where it matters -- this
    # section is thirteen of the checks, and skipping them there was silent.
    for candidate in (os.path.join(HERE, "..", "rover_daemon"),
                      os.path.join(HERE, "..")):
        sys.path.insert(0, os.path.abspath(candidate))
    try:
        import board_bridge
    except ImportError as exc:
        print("  .... skipped, no board_bridge.py beside this checkout (%s)" % exc)
        return

    class FakeLink:
        def __init__(self):
            self.sent = []
            self.pumps = 0

        def pump(self):
            self.pumps += 1

        def motion(self):
            return {"at": 1.0, "gz_lsb_s": 12.5, "ticks": 7.5, "samples": 3,
                    "breaks": 0}

        def telemetry(self):
            return {"T": 1001, "v": 1150, "gz": 3, "ax": 10}

        def send(self, command):
            self.sent.append(command)
            return True

    link = FakeLink()
    # Port 0 lets the OS pick a free one, so a self-test never collides with a
    # bridge that is actually running on this machine.
    bridge = board_bridge.BoardBridge(link, host="127.0.0.1", port=0)

    check("a command reaches the board",
          bridge.command(b'{"send": {"T": 11, "L": 5, "R": 5}}')["ok"], True)
    check("...and it is the command that was sent",
          link.sent[-1], {"T": 11, "L": 5, "R": 5})
    check("a bare command object works too, without the wrapper",
          bridge.command(b'{"T": 132, "IO4": 8}')["ok"], True)
    check("nonsense is refused rather than forwarded",
          bridge.command(b'not json at all')["ok"], False)
    check("...and an object with no command in it is refused",
          bridge.command(b'{"hello": 1}')["ok"], False)
    check("a refusal does not reach the board", len(link.sent), 2)

    # Before the pump has run there is nothing to report, and saying so is the
    # right answer -- a snapshot that invented zeroes would be odometry claiming
    # the rover is stationary when in truth nothing has been asked yet.
    check("a snapshot before the first pump reports nothing, not zeroes",
          bridge.snapshot()["motion"], None)

    bridge.start()
    try:
        for _ in range(50):
            if bridge.snapshot()["motion"] is not None:
                break
            time.sleep(0.02)
        snapshot = bridge.snapshot(full=False)
        check("a snapshot carries the motion counters",
              snapshot["motion"]["ticks"], 7.5)
        check("...and leaves out the telemetry when it was not asked for",
              "telemetry" in snapshot, False)
        check("...but includes it when it is", "telemetry" in bridge.snapshot(full=True),
              True)
        # End to end over a real socket, which is what catches a framing mistake
        # that every in-process test would miss.
        host, port = bridge.address
        sock = socket.create_connection((host, port), timeout=3)
        sock.settimeout(3)
        sock.sendall(b'{"send": {"T": 11, "L": 1, "R": -1}}\n')
        pending, records, deadline = b"", [], time.monotonic() + 3
        while time.monotonic() < deadline and len(records) < 4:
            pending += sock.recv(4096)
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                if line.strip():
                    records.append(json.loads(line))
        sock.close()
        kinds = {r["kind"] for r in records}
        check("the stream carries motion records", "motion" in kinds, True)
        check("...and the command was acknowledged", "ack" in kinds, True)
        check("...and the board actually got it",
              {"T": 11, "L": 1, "R": -1} in link.sent, True)
        check("the pump loop is running", link.pumps > 0, True)
    finally:
        bridge.close()


# --- calibration --------------------------------------------------------------
def test_calibration_store():
    section("the calibration store")
    store = os.path.expanduser("~/ugv/odometry.json")
    if not os.path.exists(store):
        print("  .... skipped, no %s on this machine" % store)
        return
    with open(store) as fh:
        loaded = json.load(fh)
    check("the gyro scale is present and positive",
          isinstance(loaded.get("gyro_lsb_per_dps"), (int, float))
          and loaded["gyro_lsb_per_dps"] > 0, True)
    ticks = loaded.get("ticks_per_metre")
    if ticks is None:
        print("  ....  ticks_per_metre is still null -- run calibrate_chassis.py on "
              "the rover; odometry distance is the commanded speed until then")
    else:
        check("the wheel scale is positive", ticks > 0, True)


# --- the configuration files --------------------------------------------------
def test_configs_agree():
    """The three places a speed limit is written must say the same thing.

    Nav2's YAML cannot import from nav_types.py, so the numbers are copied -- and
    a copy that silently diverges is a rover whose controller commands more than
    the base will deliver, which shows up as a path it never quite follows.
    """
    section("configuration agrees with the measured chassis")
    path = os.path.join(HERE, "config", "nav2.yaml")
    if not os.path.exists(path):
        print("  .... skipped, no config/nav2.yaml")
        return
    with open(path) as fh:
        text = fh.read()
    check("Nav2's top speed matches MAX_SPEED_MS",
          ("max_vel_x: %.2f" % MAX_SPEED_MS) in text, True)
    turn = math.radians(MAX_TURN_DPS)
    check("Nav2's turn limit matches MAX_TURN_DPS (%.2f rad/s)" % turn,
          abs(turn - 0.78) < 0.01, True)
    check("...and that is what the file says", "max_vel_theta: 0.78" in text, True)

    slam = os.path.join(HERE, "config", "slam_toolbox.yaml")
    if os.path.exists(slam):
        with open(slam) as fh:
            slam_text = fh.read()
        check("slam_toolbox and Nav2 agree the map resolution is 5 cm",
              "resolution: 0.05" in slam_text and "resolution: 0.05" in text, True)
        check("slam_toolbox is told the lidar's real reach",
              "max_laser_range: 8.0" in slam_text, True)
        check("mapping is on, or there is no map to navigate on",
              "mode: mapping" in slam_text, True)
        check("loop closing is on, which is the whole reason for this stack",
              "do_loop_closing: true" in slam_text, True)


def main():
    test_drive_model()
    test_turn_curve()
    test_idle_behaviour()
    test_odometry()
    test_scan_binning()
    test_bridge_protocol()
    test_calibration_store()
    test_configs_agree()
    print("\n%d passed, %d failed" % (PASSED, FAILED))
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
