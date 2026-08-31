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
import re
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


# Imported, not copied. This is the control law that drives the rover, and it now
# lives in a module with no ROS in it precisely so that this file can test the
# real thing. It used to be restated here, which is exactly the arrangement that
# lets a fix land in one copy and not the other -- and a control law that has
# drifted from its test looks like a passing test.
sys.path.insert(0, HERE)
from drive_mixer import (FALLBACK_TURN_POINTS, TURN_PWM_MAX,     # noqa: E402
                         mix as cmd_to_pwm, pwm_for, steer_pwm, to_pwm,
                         turn_to_pwm)

_FIT = sorted((pwm, rate) for pwm, (rate, _c) in TURN_RATES.items())
_LO, _HI = _FIT[0], _FIT[-1]

# This chassis as calibrate_chassis.py measured it on 2026-08-23, so the steering
# tests below run the same numbers the rover runs and do not depend on a store
# being present. Kept beside the tests rather than read from ~/ugv/odometry.json
# because a test that changes its own expectations when somebody re-calibrates is
# not a test.
MEASURED_TURN = [[85, 9.1718], [90, 12.625], [95, 14.8219], [105, 24.4707],
                 [120, 40.78], [145, 65.573], [170, 95.8467]]
MEASURED_DRIVE = [[85, 0.3326], [90, 0.3606], [95, 0.396], [115, 0.4851],
                  [140, 0.678]]


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
    # And the squeeze must come out of the speed, not out of the rotation. The
    # first version scaled both, so a commanded 45 deg/s arrived as 25 -- fair
    # looking, and wrong: a rover that advances too slowly still follows its
    # route, and one that turns too slowly leaves it.
    turn_only = cmd_to_pwm(0.0, math.radians(MAX_TURN_DPS), MEASURED_TURN,
                           MEASURED_DRIVE)
    both = cmd_to_pwm(0.5, math.radians(MAX_TURN_DPS), MEASURED_TURN,
                      MEASURED_DRIVE)
    check("asking for speed as well does not cost rotation",
          abs(both[1] - both[0]) >= abs(turn_only[1] - turn_only[0]) - 1, True)


def test_steering_has_a_small_end():
    """A gentle steering request must produce a gentle turn.

    This is the test that was missing, and its absence is why a rover zig-zagged
    every route it drove for a week. Everything about the drive model was tested
    except the regime a path follower spends nearly all of its time in: nearly on
    the path, asking for a fraction of a degree a second of correction.

    The floors are the cause. `MIN_TURN_DPS` lifts any request under 12 deg/s to
    12, and the curve then refuses to go below the slowest PWM anybody measured --
    both correct rules about a wheel starting from *rest*, and both wrong about the
    difference between two wheels already turning. Measured on this chassis's own
    curves, requests of 0.5, 1, 2, 5 and 10 deg/s all came out as the identical
    pair (-1, 177): one wheel stopped, the other at full. A follower handed that
    cannot steer, only swerve.
    """
    section("steering while driving has a small end")
    speed = 0.35
    pairs = [cmd_to_pwm(speed, math.radians(d), MEASURED_TURN, MEASURED_DRIVE)
             for d in (0.5, 1.0, 2.0, 5.0, 10.0)]
    diffs = [(r - l) / 2.0 for l, r in pairs]
    check("five requests spanning a factor of twenty are not one output",
          len(set(diffs)) == len(diffs), True)
    check("...and they increase with the request", diffs == sorted(diffs), True)
    check("half a degree a second is a gentle differential, not a pivot",
          diffs[0] < 20, True)
    check("...and both wheels still drive forward",
          pairs[0][0] > 0 and pairs[0][1] > 0, True)
    check("ten degrees a second is a firm one", diffs[-1] > 60, True)

    # Continuity: no step in the output as the request crosses the old floor. A
    # step anywhere is a request the follower cannot make, and a limit cycle
    # around it.
    worst, at = 0.0, 0.0
    previous = None
    for i in range(0, 301):
        dps = i * 0.1
        left, right = cmd_to_pwm(speed, math.radians(dps), MEASURED_TURN,
                                 MEASURED_DRIVE)
        diff = (right - left) / 2.0
        if previous is not None and abs(diff - previous) > worst:
            worst, at = abs(diff - previous), dps
        previous = diff
    check("the steering curve has no step in it (worst %.1f PWM at %.1f deg/s)"
          % (worst, at), worst <= 6.0, True)

    # Standing still is the other half of the rule, and it must not change: from
    # rest both wheels have to clear stiction, so the floors still apply.
    left, right = cmd_to_pwm(0.0, math.radians(1.0), MEASURED_TURN, MEASURED_DRIVE)
    check("a turn on the spot still gets the from-rest floor",
          abs(right - left) / 2.0 >= MEASURED_TURN[0][0], True)
    check("...which is what steer_pwm says when it is not driving",
          steer_pwm(1.0, MEASURED_TURN, driving=False),
          float(turn_to_pwm(1.0, MEASURED_TURN)))
    check("...and driving, the same request is far gentler",
          steer_pwm(1.0, MEASURED_TURN, driving=True)
          < steer_pwm(1.0, MEASURED_TURN, driving=False), True)
    check("no request at all is no differential, driving or not",
          (steer_pwm(0.0, MEASURED_TURN, driving=True),
           steer_pwm(0.0, MEASURED_TURN, driving=False)), (0.0, 0.0))


def test_the_simulated_chassis_is_not_the_mixer_inverted():
    """Guard the thing that made this simulation useless the first time.

    Its rover used to derive rotation from the PWM difference with the very curve
    the mixer used to choose that difference. Plant and controller were exact
    inverses, loop gain was 1.0 whatever either of them believed, and a mixer
    steering on entirely the wrong curve simulated perfectly -- which is how a
    mixer that over-responded by up to nine times passed. A simulation that
    cannot fail is worse than none, so this asserts that it can.
    """
    section("the simulated chassis is measured, not the mixer turned around")
    try:
        import steering_sim
    except Exception as exc:                       # pragma: no cover
        print("  .... skipped, cannot import steering_sim: %s" % exc)
        return
    from drive_mixer import steer_pwm
    worst = 0.0
    for dps in (1.0, 2.0, 5.0, 10.0, 20.0):
        differential = steer_pwm(dps, MEASURED_TURN, driving=True)
        real = steering_sim.curve(steering_sim.STEER_POINTS, differential)
        worst = max(worst, abs(real - dps) / dps)
    check("steering on the pivot curve is visibly wrong in simulation "
          "(worst %.0fx off)" % (1 + worst), worst > 1.0, True)


def test_the_rover_does_not_wander_down_a_straight_line():
    """Close the loop in simulation and count how often the steering reverses.

    Three mixers against one measured chassis. The middle one is the interesting
    one: it is the fix for the zig-zag, it can express a gentle request, and
    against a plant that is not its own inverse it still wanders, because the
    amount of rotation it buys with that request is read off the wrong curve.
    That is the state the rover was in when its trail was still visibly curved,
    and this is the test that would have said so.
    """
    section("a simulated rover follows a straight line without hunting")
    try:
        import steering_sim
    except Exception as exc:                       # pragma: no cover
        print("  .... skipped, cannot import steering_sim: %s" % exc)
        return
    runs = {}
    for name, mixer in steering_sim.MIXERS:
        runs[name] = steering_sim.run(mixer, MEASURED_TURN, MEASURED_DRIVE,
                                      metres=4.0, start_offset=0.10)
    floored, pivot, measured = (runs["floored"], runs["pivot-curve"],
                                runs["measured"])
    check("the floored mixer hunts (%.1f steering reversals per metre)"
          % floored["reversals_per_m"], floored["reversals_per_m"] > 2.0, True)
    check("steering on the pivot curve still wanders (%.1f cm of swing)"
          % (pivot["settled_swing_m"] * 100),
          pivot["settled_swing_m"] > 0.03, True)
    check("the measured curve settles (%.1f per metre)"
          % measured["reversals_per_m"], measured["reversals_per_m"] < 1.0, True)
    check("and holds the line to under a centimetre (%.1f cm of swing)"
          % (measured["settled_swing_m"] * 100),
          measured["settled_swing_m"] < 0.01, True)
    check("all three still reach the line they were following",
          max(r["final_offset_m"] for r in runs.values()) < 0.05, True)


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


def debias_run(samples, gain=0.001, settle=1.0, still_ticks=0.5):
    """base_node.BaseNode.debias, standing alone.

    Each sample is (d_yaw, dt, ticks, commanded). Returns the corrected total and
    the bias it settled on.
    """
    bias = None
    still_for = 0.0
    last_ticks = None
    total = 0.0
    for d_yaw, dt, ticks, commanded in samples:
        rate = d_yaw / dt
        moving = bool(commanded)
        if ticks is not None and last_ticks is not None:
            moving = moving or abs(ticks - last_ticks) > still_ticks
        if not moving:
            still_for += dt
            if still_for > settle:
                bias = rate if bias is None else bias + gain * (rate - bias)
        else:
            still_for = 0.0
        total += d_yaw if bias is None else d_yaw - bias * dt
        last_ticks = ticks
    return total, bias


def test_gyro_bias():
    section("the gyro's zero-offset")
    dt = 1.0 / 18.0
    drift = math.radians(0.46)          # measured on this rover, standing still

    # Standing still for two minutes with a 0.46 deg/s offset. Uncorrected that is
    # 55 degrees of rotation that never happened.
    still = [(drift * dt, dt, 100.0, False) for _ in range(18 * 120)]
    raw = sum(d for d, _, _, _ in still)
    check("uncorrected, a still rover invents %.0f degrees in two minutes"
          % math.degrees(raw), math.degrees(raw) > 40, True)
    total, bias = debias_run(still)
    check("the offset is found", bias is not None, True)
    check("...to within a twentieth of a degree per second",
          math.degrees(bias), math.degrees(drift), tolerance=0.05)
    check("...and most of the invented rotation is removed",
          abs(math.degrees(total)) < abs(math.degrees(raw)) * 0.25, True)

    # A real turn must survive. The rover is commanded and the wheels are moving,
    # so nothing is learned during it and the rotation passes through.
    warmup = [(drift * dt, dt, 100.0, False) for _ in range(18 * 60)]
    turning = [(math.radians(25.0) * dt, dt, 100.0 + i, True) for i in range(18 * 4)]
    # Both totals corrected, and differenced. Subtracting the *raw* warmup from a
    # corrected total would charge the turn with the bias the warmup removed.
    settled, _ = debias_run(warmup)
    total, _ = debias_run(warmup + turning)
    turned = math.degrees(total - settled)
    check("a commanded 25 deg/s turn for 4 s still reads about 100 degrees",
          turned, 100.0, tolerance=6.0)

    # Wheels turning with nothing commanded -- somebody pushing the rover, or a
    # game pad driving it -- must not be mistaken for stillness.
    pushed = [(math.radians(20.0) * dt, dt, 100.0 + i * 3, False)
              for i in range(18 * 30)]
    _, bias = debias_run(pushed)
    check("a pushed rover is not treated as a still one", bias, None)

    # And the settle window keeps the coast out of the estimate.
    coasting = [(math.radians(15.0) * dt, dt, 100.0, False) for _ in range(9)]
    _, bias = debias_run(coasting)
    check("half a second of coasting teaches it nothing", bias, None)


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
    # The measured envelope has to contain what Nav2 is allowed to ask for, or
    # the controller spends its life commanding a speed the base cannot deliver.
    points = loaded.get("drive_pwm_points")
    if isinstance(points, list) and len(points) >= 2:
        speeds = sorted(v for _, v in points)
        check("the measured speed curve rises with PWM, so it can be inverted",
              [v for _, v in points] == speeds, True)
        check("Nav2's 0.40 m/s limit is inside the measured range (%.2f-%.2f)"
              % (speeds[0], speeds[-1]), speeds[0] <= 0.40 <= speeds[-1], True)
        # The one that was missing, and its absence is what let the controller
        # spend a third of every drive commanding speeds the wheels cannot
        # produce. There is no creep on this chassis: below the slowest measured
        # PWM the motors do not turn, so anything Nav2 asks for between zero and
        # that speed arrives at the wheels as that speed.
        floor = speeds[0]
        cfg = os.path.join(HERE, "config", "nav2.yaml")
        if os.path.exists(cfg):
            with open(cfg) as fh:
                nav_text = fh.read()
            check("Nav2's top speed clears the chassis's %.2f m/s floor" % floor,
                  "max_vel_x: 0.40" in nav_text and floor < 0.40, True)
            check("...and it samples only the two speeds the chassis has",
                  "vx_samples: 2" in nav_text, True)
            check("...and its acceleration window spans the whole range, or the "
                  "samples collapse to a creep",
                  "acc_lim_x: 4.0" in nav_text, True)
    else:
        print("  ....  no drive_pwm_points yet -- run calibrate_chassis.py")

    ticks = loaded.get("ticks_per_metre")
    if ticks is None:
        print("  ....  ticks_per_metre is still null -- run calibrate_chassis.py on "
              "the rover; odometry distance is the commanded speed until then")
    else:
        check("the wheel scale is positive", ticks > 0, True)


# --- the configuration files --------------------------------------------------
def _costmap_sections(text):
    """nav2.yaml split into the global costmap's block and the local one's.

    Crude, and deliberately so: this file cannot import yaml on every machine it
    runs on, and what the checks below ask is only whether a plugin name appears
    on one side of the file or the other. Returns (global, local) as text.
    """
    lines = text.splitlines(True)
    where, out = None, {"global_costmap:": [], "local_costmap:": []}
    for line in lines:
        if line.rstrip() in out:
            where = line.rstrip()
        elif line and not line[0].isspace() and not line.startswith("#"):
            where = None
        if where:
            out[where].append(line)
    return "".join(out["global_costmap:"]), "".join(out["local_costmap:"])


def settings_of(text):
    """Just the settings out of a YAML file, with the comments dropped.

    Everything below is checked by looking for a string, and both of the names
    that matter -- the critic that was replaced and the shim that was removed --
    go on appearing in the comments that explain why they are not there. A search
    over the whole file finds the explanation and calls it the setting.
    """
    keep = [line for line in text.splitlines()
            if line.strip() and not line.lstrip().startswith("#")]
    return chr(10).join(keep)


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
    # Deliberately *not* MAX_SPEED_MS. That constant is 0.35 m/s and this chassis
    # was measured at 0.33 at its slowest usable PWM, so putting it here pins
    # every command to the bottom of the range. What must hold is that Nav2's
    # limit lies inside the speeds actually measured, which is checked against
    # the store below where the store exists.
    check("Nav2's top speed is not the stale MAX_SPEED_MS",
          ("max_vel_x: %.2f" % MAX_SPEED_MS) in text, False)
    check("...and is a speed this chassis can actually reach",
          "max_vel_x: 0.40" in text, True)
    # A heading change tighter than an arc can absorb has to be a turn on the
    # spot, and a bare velocity controller will only ever approximate one.
    # The shim was tried and taken out again -- it cannot transform on a rover
    # whose transform tree runs at the driver board's 17 Hz, and it cost the
    # control loop a third of its rate. The comment in nav2.yaml has the whole
    # story; this keeps somebody from re-adding it without reading it.
    check("no rotation shim, which this rover's transform rate cannot support",
          "RotationShimController" in settings_of(text), False)
    # The footprint and the critic that reads it have to change together. A
    # rectangle with a 0.14 m inscribed radius under a critic that only tests the
    # centre cell is a rover free to swing its corners into the furniture.
    # Settings only. Both names go on appearing in the comments beside them,
    # which is where the reasoning lives, so a search over the whole file finds
    # the explanation and calls it the setting.
    settings = settings_of(text)
    # **A circle, because this rover pivots.** nav2 clears the robot at the
    # inscribed radius and the robot sweeps the circumscribed one, so any
    # non-circular body has a band round every wall it may legally stand in and
    # not turn out of. The rectangle that was here had a 0.14 m ring and 0.21 m
    # corners, and the rover was found wedged in exactly those ten centimetres.
    # A radius makes the two one number. 0.20 m is the chassis measured with a
    # tape; the old footprint was `slam2d.c`'s lidar self-return mask plus 5 cm
    # of margin, with an unmeasured guess forwards. See config/nav2.yaml.
    check("the body is a circle, so standing somewhere implies turning there",
          "robot_radius: 0.200" in settings, True)
    check("...in both costmaps, because two shapes is a planner that routes "
          "through gaps the controller will not drive",
          settings.count("robot_radius: 0.200") == 2, True)
    check("...and the rectangle that could not turn where it stood is gone",
          'footprint: "[[0.20' in settings, False)
    # The point test is a collision test again, and only because of the above.
    check("the obstacle critic is the point test a circular body takes",
          "BaseObstacle" in settings and "ObstacleFootprint" not in settings, True)
    # The arrival circle has to be bigger than the smallest move the rover has.
    # One forward sample at 0.40 m/s over a 0.8 s rollout is 32 cm, so a 15 cm
    # circle was a target DWB could not aim at: it sat 23 cm from a goal for
    # 25 s and timed out, because everything that closed the gap overshot it.
    # The two copies are the goal checker's and the controller's, and they have
    # to be the same number or RotateToGoal switches at a different radius from
    # the one that ends the goal.
    check("the arrival circle clears the chassis's 32 cm minimum move",
          settings.count("xy_goal_tolerance: 0.22") == 2, True)
    # The planner has to know the turning radius DWB can follow while driving.
    # NavFn does not, and that is the doorway lock-up: a 45 deg kink in one
    # rollout, every forward sample leaves the line, the rover pivots. The
    # plugin name GridBased is unchanged so the behaviour tree does not have
    # to move; the class behind it does. Lattice, not Hybrid: this chassis
    # can pivot, and Dubins Hybrid-A* cannot write that into a path.
    check("the planner is the state lattice, which is the one that knows a turning radius",
          "nav2_smac_planner::SmacPlannerLattice" in settings, True)
    check("NavFn is not the configured planner",
          "nav2_navfn_planner::NavfnPlanner" in settings, False)
    check("Hybrid-A* is not the configured planner either",
          "nav2_smac_planner::SmacPlannerHybrid" in settings, False)
    # **The align look-ahead, which is a controller setting but belongs next to
    # the planner ones because it decides whether the plan gets followed.**
    # 0.325 is nav2's own default and these two checks only hold it there.
    #
    # **The reason recorded for it has since been withdrawn**, so read them as
    # "this is the default and nothing has argued it away" rather than as a
    # measured choice. The argument was that an align critic charges
    # `unreachable_score_` -- 2881 points once scaled -- for a nose point the
    # flood could not reach, and that at 0.8 m this landed on a median 26% of
    # the candidates in a tick. The flood in the installed `libdwb_critics.so`
    # is not stopped by walls at all, so no candidate is ever charged it: 0 of
    # 8687 driving candidates and 0 of 6132 pivots over
    # recordings/trap-2026-08-25-spin.json. See corridor_sim.flood and
    # trap_sim.py --bias, and make a fresh case from a drive before moving it.
    #
    # `PreferForward` is the only critic in the set that prices turning as
    # such, and it was added on the same withdrawn measurement. It is left in
    # because the rover does still choose to turn far more often than to drive
    # -- on the recorded trap, on 481 of 496 ticks the model could score.
    check("something in the critic set prices turning, or the rover will spin",
          "PreferForward" in settings, True)
    check("the align look-ahead is back at nav2's default, not the 0.8 that trapped it",
          "PathAlign.forward_point_distance: 0.325" in settings
          and "GoalAlign.forward_point_distance: 0.325" in settings, True)
    check("...and neither align critic is left at 0.8",
          "forward_point_distance: 0.8" in settings, False)
    check("in-place turns are expensive, so a doorway that takes an arc gets one",
          "rotation_penalty: 5.0" in settings, True)
    # **The budget, and it is the one that was failing long goals.** A route
    # of eight to twelve metres across a mapped house costs this board one to
    # two and a half seconds, so a 2 s budget cut a large share of them off
    # mid-search -- and Nav2 reports that as `NoValidPathCouldBeFound`, which
    # reaches the operator as "there is no route to there". Measured with
    # plan_bench.py: one query at one start heading, ten times, planned 4 and
    # refused 6 at 2 s with every success landing at 2.01-2.09 s; at 3 s the
    # whole sixteen-heading sweep planned, none of it needing over 2.27 s.
    check("the planner has 4 s, because a house-sized route costs this board 2 to 3",
          "max_planning_time: 4.0" in settings, True)
    check("...and reverse expansion is off, because the lidar looks forwards",
          "allow_reverse_expansion: false" in settings, True)
    check("...and the lattice may enter unknown, because this rover maps as it drives",
          "allow_unknown: true" in settings, True)
    lattice_json = os.path.join(HERE, "config", "lattices", "diff_5cm_0.5m.json")
    check("the differential control set is in the tree, not left on a share path",
          os.path.isfile(lattice_json), True)
    if os.path.isfile(lattice_json):
        with open(lattice_json) as handle:
            meta = json.load(handle)["lattice_metadata"]
        check("...and is the 0.5 m differential sample, DWB's envelope to a centimetre",
              meta.get("motion_model") == "diff" and abs(meta.get("turning_radius", 0) - 0.5) < 1e-9,
              True)
    with open(os.path.join(HERE, "nav.launch.py")) as handle:
        launch = handle.read()
    check("launch injects an absolute lattice path; yaml cannot resolve a relative one",
          "lattice_filepath" in launch and "diff_5cm_0.5m.json" in launch, True)
    with open(os.path.join(HERE, "slam.launch.py")) as handle:
        slam_launch = handle.read()
    check("the python nodes are started by the interpreter, so a 644 checkout still runs",
          "sys.executable" in slam_launch
          and 'os.path.join(HERE, "lidar_node.py")' in slam_launch, True)

    # The lidar looks forwards, so a reverse leg is driven blind. DWB is left
    # with no reverse sample at all; backing out of a corner is the behaviour
    # server's `backup`, which the behaviour tree bounds to 30 cm.
    check("the controller has no reverse, since the rover cannot see behind it",
          "min_vel_x: 0.0" in settings and "min_vel_x: -0.40" not in settings,
          True)
    check("...but the smoother still passes one, or the recovery cannot back up",
          "min_velocity: [-0.40, 0.0, -0.78]" in settings, True)
    turn = math.radians(MAX_TURN_DPS)
    check("Nav2's turn limit matches MAX_TURN_DPS (%.2f rad/s)" % turn,
          abs(turn - 0.78) < 0.01, True)
    check("...and that is what the file says", "max_vel_theta: 0.78" in text, True)
    # Both floors have to move. Nav2's isValidSpeed is an AND: a theta floor
    # with min_speed_xy left at 0 never drops a sample. 0.21 rad/s is the
    # mixer's 12 deg/s; 0.1 m/s is below the only forward sample, so driving
    # is untouched.
    check("DWB will not sample a standing turn slower than the mixer can hold",
          "min_speed_theta: 0.21" in settings, True)
    check("...and min_speed_xy is not zero, or that theta floor is a no-op",
          "min_speed_xy: 0.1" in settings, True)
    check("...and the old zero floors are gone",
          "min_speed_xy: 0.0" in settings or "min_speed_theta: 0.0" in settings,
          False)

    # The two costmap rules that a rover running SLAM cannot break. Both were
    # broken at once, and between them they closed 61% of the mapped floor to the
    # planner, which is what a route four times longer than it needed to be is
    # made of. See the comments beside each of them in config/nav2.yaml.
    globals_, locals_ = _costmap_sections(text)
    check("the global costmap has no obstacle layer, which SLAM would ghost",
          "obstacle_layer" in globals_, False)
    check("...and still has the static layer, or it has nothing to plan on",
          "static_layer" in globals_, True)
    check("the local costmap does have one, since something must see a chair",
          "obstacle_layer" in locals_, True)
    check("...and clears the bearings that got nothing back",
          "inf_is_valid: true" in locals_, True)

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


def test_goal_fits_before_it_is_sent():
    """The goal check, on the real geometry rather than a stand-in.

    goal_fit.py has no ROS in it for exactly this reason, so what runs here is
    the code the rover runs. The numbers in the last two checks are the recorded
    failure: a goal at (4.34, -0.98) on a costmap where the body covered a lethal
    cell at the heading the bridge would have sent, which Nav2 accepted, planned
    a clean straight path to, and then spent thirty seconds failing to reach.
    """
    section("a goal is checked against the body before it is sent")
    sys.path.insert(0, HERE)
    try:
        import goal_fit
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import goal_fit: %s" % exc)
        return

    body = goal_fit.polygon_from(
        '[[0.20, 0.14], [0.20, -0.14], [-0.16, -0.14], [-0.16, 0.14]]', 0.0)
    check("the footprint parses out of the string nav2.yaml holds",
          body == [(0.20, 0.14), (0.20, -0.14), (-0.16, -0.14), (-0.16, 0.14)],
          True)
    check("...and a bare radius still gives a polygon rather than nothing",
          len(goal_fit.polygon_from("", 0.25) or []) >= 8, True)

    # Two metres square of clear floor, with a wall down the right-hand side:
    # lethal from 1.70 m, and the inscribed ring reaching back to 1.50.
    width = height = 40
    data = [0] * (width * height)
    for row in range(height):
        for col in range(30, width):
            data[row * width + col] = 254 if col >= 34 else 253
    floor = goal_fit.CostGrid(width, height, 0.05, 0.0, 0.0, data)

    check("a goal in open floor fits", goal_fit.fits(floor, body, 0.5, 1.0, 0.0),
          True)
    check("...and one in the wall does not",
          goal_fit.fits(floor, body, 1.6, 1.0, 0.0), False)
    check("...and is left exactly where it was asked for",
          goal_fit.fit(floor, body, 0.5, 1.0, 0.0)["moved_m"] == 0.0, True)

    moved = goal_fit.fit(floor, body, 1.6, 1.0, 0.0)
    check("a goal in the wall is moved to somewhere the body fits",
          moved is not None and goal_fit.fits(floor, body, moved["x"],
                                              moved["y"], moved["yaw"]), True)
    check("...and not moved further than it has to be",
          moved is not None and moved["moved_m"] <= goal_fit.REACH_M, True)
    check("...and a goal with no way out at all is refused rather than tried",
          goal_fit.fit(floor, body, 1.9, 1.0, 0.0, reach_m=0.10), None)

    # Unknown is not an obstacle. The planner is configured with allow_unknown
    # because this rover maps as it drives, so a goal in a room it has not seen
    # yet has to be allowed through -- refusing it would stop exploration dead.
    unseen = goal_fit.CostGrid(width, height, 0.05, 0.0, 0.0,
                               [255] * (width * height))
    check("unknown floor does not block a goal, or the rover stops exploring",
          goal_fit.fits(unseen, body, 1.0, 1.0, 0.0), True)

    # The outline matters as well as the interior: a body can straddle a wall
    # one cell thick without any cell centre landing inside the polygon.
    thin = [0] * (width * height)
    for row in range(height):
        thin[row * width + 20] = 254
    check("a wall one cell thick is not stepped over by the interior test",
          goal_fit.fits(goal_fit.CostGrid(width, height, 0.05, 0.0, 0.0, thin),
                        body, 1.0, 1.0, 0.0), False)

    # And that the bridge actually asks. The geometry being right is no use if
    # `goto` never calls it, and this file cannot import nav_bridge to find out.
    bridge = os.path.join(HERE, "nav_bridge.py")
    if os.path.exists(bridge):
        with open(bridge) as fh:
            source = fh.read()
        check("the bridge checks a goal before sending it",
              "self.fit_goal(gx, gy, yaw)" in source, True)
        check("...and turns round rather than reversing the length of a room",
              "REVERSE_LIMIT_M" in source and "reverse_by_turning" in source,
              True)


# --- the navigation bridge ----------------------------------------------------
# Stand-ins for nav_bridge.py, which cannot be imported without rclpy. The same
# arrangement as the drive model above, and for the same reason: a sign flip in a
# bearing does not need a radio to be wrong.
#
# The result-code table is the exception, and it is imported rather than copied.
# It is a table and not arithmetic, so a stand-in could agree with itself
# perfectly while disagreeing with the bridge -- which is how the first version of
# it shipped a mapping that read a blocked drive as a timeout. `nav_codes.py`
# has no ROS in it precisely so that this file can read the real thing.
sys.path.insert(0, HERE)
from nav_codes import PHRASES, REASONS, phrase_for, reason_for   # noqa: E402


def wrap(radians):
    """A copy of nav_bridge.wrap."""
    return math.atan2(math.sin(radians), math.cos(radians))


def yaw_of_zw(z, w):
    """A copy of nav_bridge.yaw_of, taking the two components that matter."""
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def steering(where, poses, lookahead=1.0):
    """A copy of nav_bridge.NavBridge.steering, over plain (x, y) tuples."""
    if where is None or not poses:
        return None
    x, y, yaw = where
    for px, py in poses:
        if math.hypot(px - x, py - y) >= lookahead:
            return round(math.degrees(wrap(math.atan2(py - y, px - x) - yaw)), 1)
    px, py = poses[-1]
    if math.hypot(px - x, py - y) < 0.05:
        return None
    return round(math.degrees(wrap(math.atan2(py - y, px - x) - yaw)), 1)


def test_nav2_error_codes():
    """Nav2's result codes, as the words the daemon's `Outcome` already uses.

    The whole reason `nav_codes.py` lists every code instead of doing arithmetic
    on it: the numbers look systematic and are not. BackUp's 713 is invalid input
    and its 714 is a collision; DriveOnHeading's 723 is a collision and its 724 is
    invalid input -- the same two meanings, swapped, in adjacent blocks. A version
    of this that matched on the last digit passed every test written for it and
    reported a rover stopped by a wall as one that had timed out.
    """
    section("Nav2 result codes read as English")
    check("zero is an arrival", reason_for(0), "arrived")
    check("701, Spin timing out", reason_for(701), "timed out")
    check("703, Spin into something", reason_for(703), "blocked")
    check("713, BackUp given a nonsense distance", reason_for(713), "refused")
    check("714, BackUp into something", reason_for(714), "blocked")
    check("723, DriveOnHeading into something -- note it is not 724",
          reason_for(723), "blocked")
    check("724, DriveOnHeading given a nonsense distance",
          reason_for(724), "refused")
    check("702, a transform failure, which is being lost",
          reason_for(702), "lost")
    check("208, the planner finding no route, is being blocked",
          reason_for(208), "blocked")
    check("206, a goal with something in it, is a refusal",
          reason_for(206), "refused")
    check("105, the controller stuck, is being blocked",
          reason_for(105), "blocked")
    # 700 is Spin's UNKNOWN, and it caught the last-digit version red-handed:
    # 700 % 10 is 0, so a behaviour that failed for a reason it could not name was
    # reported as having arrived. The rover would have said it had turned.
    check("700 -- plain unknown -- is a failure and not an arrival",
          reason_for(700), "failed")
    check("a code nobody has heard of falls back rather than raising",
          reason_for(795), "failed")
    check("...and specifically not to an arrival, so a Nav2 upgrade that adds a "
          "failure does not have it read as a success",
          reason_for(795) == "arrived", False)

    # Every reason has to be one the daemon's callers understand, because
    # `_tool_drive` decides `ok` by testing the word.
    known = {"arrived", "blocked", "timed out", "lost", "refused", "failed"}
    check("every code maps to a word Outcome's readers know",
          sorted(set(REASONS.values()) - known), [])
    check("every phrase belongs to a code that exists",
          sorted(set(PHRASES) - set(REASONS)), [])
    check("Nav2's own words win over ours when it gives any",
          phrase_for(723, "the local costmap says no"),
          "the local costmap says no")
    check("...and ours are there for when it does not",
          phrase_for(723, "  ") != "", True)
    check("a code with neither says nothing rather than something made up",
          phrase_for(700, ""), "")


def test_nav2_error_codes_match_the_installed_nav2():
    """On the rover, check the numbers against the .action files themselves.

    The table was copied by hand out of `share/nav2_msgs/action/`, and a Nav2
    upgrade that renumbered anything would leave it quietly describing the
    previous version. Skipped where there is no ROS, which is most machines.
    """
    section("the code table matches the Nav2 that is installed")
    import glob
    import re as _re

    roots = glob.glob(os.path.expanduser(
        "~/miniforge3/envs/*/share/nav2_msgs/action"))
    if not roots:
        print("  .... skipped, no nav2_msgs on this machine")
        return
    wanted = {"UNKNOWN": "failed", "TIMEOUT": "timed out", "TF_ERROR": "lost",
              "COLLISION_AHEAD": "blocked", "INVALID_INPUT": "refused",
              "NO_VALID_PATH": "blocked", "GOAL_OCCUPIED": "refused",
              "START_OCCUPIED": "blocked", "GOAL_OUTSIDE_MAP": "refused",
              "START_OUTSIDE_MAP": "lost", "FAILED_TO_MAKE_PROGRESS": "blocked",
              "NO_VALID_CONTROL": "blocked", "PATIENCE_EXCEEDED": "blocked",
              "CONTROLLER_TIMED_OUT": "timed out",
              "INVALID_CONTROLLER": "refused", "INVALID_PLANNER": "refused",
              "INVALID_PATH": "refused"}
    interesting = ("Spin", "BackUp", "DriveOnHeading", "FollowPath",
                   "ComputePathToPose")
    for name in interesting:
        path = os.path.join(roots[0], "%s.action" % name)
        if not os.path.exists(path):
            continue
        with open(path, encoding="utf-8", errors="replace") as fh:
            declared = _re.findall(r"^uint16 ([A-Z_]+)=(\d+)",
                                   fh.read(), _re.MULTILINE)
        for label, number in declared:
            if label == "NONE":
                continue
            code = int(number)
            if label not in wanted:
                check("%s.%s (%d) is a meaning this table has an opinion about"
                      % (name, label, code), label, "one of %s" % sorted(wanted))
                continue
            check("%s.%s is %d and reads as '%s'" % (name, label, code,
                                                     wanted[label]),
                  reason_for(code), wanted[label])


def test_the_transform_budget_survives_a_scan():
    """The controller has to tolerate a scan going missing, and only just does.

    `map -> odom` is stamped with the last scan slam_toolbox picked up plus
    `transform_timeout`, and DWB refuses a pose that is older than
    `transform_tolerance` -- so between the two configs there is a fixed number of
    seconds that `laserCallback` may be busy before the next control tick throws
    ControllerTFError and the goal comes back as code 102, which is "lost".
    tf_stall_sim.py explains where that subtraction comes from.

    Two things are checked. The budget must be longer than the gap between scans,
    or every ordinary revolution would abort a goal; and the simulation must
    report a healthy stack as healthy, which is the guard the steering simulation
    had to learn the hard way.
    """
    section("the transform budget between slam_toolbox and the controller")
    try:
        import tf_stall_sim
    except ImportError:
        print("  .... skipped, no tf_stall_sim.py")
        return
    cfg = tf_stall_sim.config()
    budget = (cfg["transform_timeout"] + cfg["transform_tolerance"]
              - cfg["scan_stamp_offset"])
    scan_gap = 1.0 / tf_stall_sim.SCAN_HZ
    check("the budget is longer than one revolution", budget > scan_gap, True)
    print("       %.2f s of budget, %.0f ms between scans" % (budget, scan_gap * 1000))
    quiet = tf_stall_sim.run(cfg, seconds=6.0)
    check("nothing aborts when nothing stalls", len(quiet["aborts"]), 0)
    stalled = tf_stall_sim.run(cfg, seconds=6.0, stall=budget + 0.3, stall_at=2.0)
    check("...and a stall past the budget does", len(stalled["aborts"]) > 0, True)
    # And the way out of it, which is the only reason the budget above is
    # survivable on a route long enough to close a loop: with this on,
    # `map -> odom` is stamped when it is published rather than when the mapper
    # last picked up a scan, so a busy mapper no longer expires the correction
    # the controller steers on.
    slam = os.path.join(HERE, "config", "slam_toolbox.yaml")
    if os.path.exists(slam):
        check("slam_toolbox restamps map -> odom, so a busy mapper cannot expire it",
              "restamp_tf: true" in open(slam).read(), True)


def test_heading_arithmetic():
    """Wrapping, and reading a yaw back out of a quaternion.

    Both are one line and both have a failure that looks like the rover being
    possessed: an unwrapped difference turns "ten degrees to the left" into "three
    hundred and fifty to the right", and a quaternion read with the wrong sign
    turns every heading in the map upside down.
    """
    section("headings wrap and quaternions come back out")
    check("just under half a turn stays positive",
          math.degrees(wrap(math.radians(179))), 179.0, tolerance=0.001)
    check("just over half a turn comes back as the short way round",
          math.degrees(wrap(math.radians(181))), -179.0, tolerance=0.001)
    check("a full turn is no turn",
          math.degrees(wrap(math.radians(360))), 0.0, tolerance=0.001)
    check("two and a bit turns anticlockwise is still ten degrees",
          math.degrees(wrap(math.radians(730))), 10.0, tolerance=0.001)
    for degrees in (-179.0, -90.0, -1.0, 0.0, 45.0, 90.0, 179.0):
        half = math.radians(degrees) / 2.0
        check("a yaw of %+.0f survives the round trip" % degrees,
              math.degrees(yaw_of_zw(math.sin(half), math.cos(half))),
              degrees, tolerance=0.001)


def test_steering_bearing():
    """Which way the rover is trying to go, off its own nose.

    This replaced a number the old follower could name directly, because it chose
    between candidate arcs. A velocity controller chooses no such thing, so the
    honest substitute is the bearing to the route a lookahead ahead -- and the one
    thing it must get right is the sign, because a panel that says "left" while
    the rover goes right is worse than a panel that says nothing.
    """
    section("steering points at the route, on the correct side")
    # Facing along +x at the origin, with a route that turns left.
    route = [(0.2, 0.0), (0.6, 0.1), (1.2, 0.6)]
    check("a route bending left reads as a left bearing",
          steering((0.0, 0.0, 0.0), route) > 0, True)
    check("...and the same route mirrored reads as a right one",
          steering((0.0, 0.0, 0.0), [(x, -y) for x, y in route]) < 0, True)
    check("a route straight ahead is zero degrees",
          steering((0.0, 0.0, 0.0), [(2.0, 0.0)]), 0.0)
    # Facing +y now: a point at (0, 2) is dead ahead rather than 90 degrees off.
    check("the bearing is relative to the rover, not to the map",
          steering((0.0, 0.0, math.pi / 2), [(0.0, 2.0)]), 0.0)
    check("a route entirely underneath the rover says nothing at all",
          steering((0.0, 0.0, 0.0), [(0.01, 0.0)]), None)
    check("no route at all says nothing", steering((0.0, 0.0, 0.0), []), None)
    check("no pose says nothing", steering(None, route), None)


def test_the_two_halves_agree_on_the_port():
    """The bridge's port is written in four files and nothing links them.

    A mismatch is the quietest failure in this whole stack: the daemon offers
    every driving tool, each one connects to a port nothing is listening on, and
    every tool call comes back "the ROS navigation stack is not answering" on a
    rover where it plainly is.
    """
    section("both halves agree where the bridge is")
    wanted = "8773"
    places = {
        "ros_nav/nav_bridge.py": ("PORT = " + wanted),
        "rover_daemon/ros_navigator.py": ("PORT = " + wanted),
        "rover_daemon/rover_daemon.py": ("ROS_NAV_PORT = " + wanted),
        "ros_nav/slam.launch.py": ('"nav_port", default_value="%s"' % wanted),
    }
    root = os.path.dirname(HERE)
    for relative, needle in places.items():
        path = os.path.join(root, relative)
        if not os.path.exists(path):
            # On the rover everything is deployed flat, so the repository layout
            # is not there to check. Saying so beats a failure that means nothing.
            print("  .... skipped, no %s" % relative)
            continue
        with open(path, encoding="utf-8", errors="replace") as fh:
            check("%s says %s" % (relative, wanted), needle in fh.read(), True)


def test_a_route_is_budgeted_on_the_route():
    """A goal 3 m away round a wall must not be cancelled as though it were 3 m.

    The numbers here are the route the planner really returned on 2026-08-24 from
    (0.19, -11.34) to (2.74, -9.86), read off the rover: 346 poses, 8.81 m, and a
    straight line of 2.95 m with a wall across it. The old allowance came to
    50.6 s against about 44 s of flawless driving -- 15% of headroom on a stack
    that replans every second and spends 15 s on each rung of its recovery ladder.
    Three progress-checker windows is 45 s by itself, and there were three,
    because the checker kept calling a legitimate pivot a jam.
    """
    section("a route is budgeted on the route, not on the straight line")
    straight = 2.95
    # The real route, as the eighteen waypoints it was sampled at on the rover.
    route = [(0.19, -11.34), (-0.3, -11.3), (-0.8, -11.3), (-1.0, -11.0),
             (-0.9, -10.7), (-0.6, -10.3), (-0.1, -10.2), (0.3, -10.1),
             (0.8, -10.1), (1.0, -9.7), (0.9, -9.2), (0.8, -8.8), (1.1, -8.6),
             (1.5, -8.4), (2.0, -8.3), (2.4, -8.3), (2.6, -8.8), (2.8, -9.2),
             (2.8, -9.7), (2.74, -9.86)]
    sys.path.insert(0, HERE)
    try:
        import route_cost
    except Exception as exc:
        print("  .... skipped, cannot import route_cost: %s" % exc)
        return
    metres, turning = route_cost.length_and_turning(route)
    check("the route is longer than the straight line", metres > 2 * straight)
    check("and it is measured as such", round(metres, 1), 8.3, tolerance=0.6)
    check("its turning is counted, not ignored", turning > 200.0)

    was = max(30.0, 6.0 * straight / 0.35)
    now = route_cost.seconds_for(metres, turning, 0.35, 27.0, slack=3.0,
                                 floor=45.0)
    need = metres / 0.40 + turning / 27.0
    # **Not "the old allowance was too short to drive the route" -- it was not.**
    # 50.6 s against about 44 s of flawless execution is 15% of margin, and 15%
    # is nothing on a stack that replans once a second and whose recovery ladder
    # spends 15 s a rung. Three progress-checker windows is 45 s on its own. So
    # what the old number lacked was not length, it was headroom, and that is what
    # this asserts.
    check("the old allowance left no room for a single recovery",
          was < 1.5 * need)
    check("the new allowance leaves room for the recovery ladder",
          now > 2.5 * need)

    # A grid staircase is not a route full of corners. This is the guard on the
    # sampling: a dead straight line stored at 5 cm must cost no turning at all.
    staircase = [(0.05 * k, 0.0) for k in range(60)]
    check("a straight line asks for no turning",
          route_cost.length_and_turning(staircase)[1], 0.0, tolerance=1.0)
    # And the reverse guard: a real right angle must survive the sampling.
    corner = [(0.05 * k, 0.0) for k in range(40)] +              [(1.95, 0.05 * k) for k in range(1, 40)]
    check("a real right angle is still a right angle",
          route_cost.length_and_turning(corner)[1], 90.0, tolerance=10.0)
    # Nothing to budget on is not a licence to budget on nothing.
    check("an empty plan costs nothing", route_cost.length_and_turning([]),
          (0.0, 0.0))
    check("and falls back to the floor",
          route_cost.seconds_for(0.0, 0.0, 0.35, 27.0, floor=45.0), 45.0)


def test_progress_is_not_only_translation():
    """The progress checker has to accept a pivot, on a chassis that pivots.

    `SimpleProgressChecker` measures displacement alone, and every direction
    change this rover makes is a turn on the spot. That is what aborted a
    perfectly good route three times in fifteen-second slices, cleared two
    costmaps that had nothing wrong with them, and let the replan come back the
    other way round the wall so the rover turned back -- the wiggle somebody
    watched.
    """
    section("progress means moving or turning, not only moving")
    path = os.path.join(HERE, "config", "nav2.yaml")
    if not os.path.exists(path):
        print("  .... skipped, no config/nav2.yaml")
        return
    with open(path) as fh:
        text = fh.read()
    # Comments stripped first, and that matters more here than anywhere else in
    # this file: the block explaining *why* SimpleProgressChecker was replaced
    # names it four times, so a search over the raw text finds the explanation
    # and calls it the setting.
    settings = settings_of(text)
    check("the progress checker is PoseProgressChecker",
          "nav2_controller::PoseProgressChecker" in settings)
    check("SimpleProgressChecker is not the one configured",
          "nav2_controller::SimpleProgressChecker" in settings, False)

    def number(name):
        found = re.search(r"^\s*%s:\s*([-\d.]+)" % name, settings,
                          re.MULTILINE)
        return float(found.group(1)) if found else None

    angle = number("required_movement_angle")
    allowance = number("movement_time_allowance") or 15.0
    check("a turn counts as progress", angle is not None)
    if angle is not None:
        # Above the gyro's standing bias over the whole window, or a rover
        # standing perfectly still would report progress for ever. base_node
        # measures that bias at about +0.43 deg/s.
        drift = math.radians(0.43 * allowance)
        check("and the threshold clears the gyro's own drift over the window",
              angle > 2 * drift)
        # And well below any real turn, or a legitimate pivot would fail to
        # register. The slowest pivot this chassis holds is about 9 deg/s.
    check("while staying far inside the slowest real pivot",
          angle < math.radians(9.0 * allowance) / 4.0)


def test_dwb_will_not_sample_a_turn_the_wheels_cannot_hold():
    """The 2026-08-25 doorway recording, as a sample-set test.

    Eight in ten commands were standing turns, most of them 3 deg/s. The mixer
    lifts those to 12 deg/s, the rover overshoots, and it swaps sides. Nav2's
    isValidSpeed only drops a sample when *both* floors fail, so this checks
    the generated set, not just the yaml strings.
    """
    section("DWB does not sample a standing turn slower than the mixer")
    sys.path.insert(0, HERE)
    try:
        import corridor_sim as dwb
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import corridor_sim: %s" % exc)
        return

    samples = dwb.twists()
    slow = [(vx, wz) for vx, wz in samples
            if abs(vx) < 0.05 and abs(wz) < dwb.MIN_SPEED_THETA]
    check("the sample count matches the 29 the live controller will log",
          len(samples), dwb.CANDIDATES)
    check("...and that is 12 standing turns, not the old 16",
          dwb.PIVOTS, 12)
    check("no standing turn is slower than 0.21 rad/s", slow, [])
    rolling = [(vx, wz) for vx, wz in samples if abs(vx) >= 0.05]
    check("...while the forward sample still has every steer, including slow ones",
          any(abs(wz) < dwb.MIN_SPEED_THETA for vx, wz in rolling), True)

    episode_path = os.path.join(HERE, "recordings", "doorway-2026-08-25.json")
    if os.path.isfile(episode_path):
        with open(episode_path) as handle:
            episode = json.load(handle)
        below = sum(1 for c in episode["commands"]
                    if abs(c["vx"]) < 0.05 and 0.05 <= abs(c["wz"]) < 0.21)
        check("the doorway recording still shows the fault this floor deletes",
              below > 100, True)
    else:
        print("  .... no recordings/doorway-2026-08-25.json, sample set only")


def test_lattice_respects_the_dwb_envelope():
    """The doorway corner, on a map that does not need a recording.

    NavFn's grid search has no turning radius, so the path it traces through
    a metre-wide 55 deg bend kinks more in 0.32 m than DWB can follow at
    speed. The differential lattice is given a 0.5 m control set (this
    chassis's max_vel_x / max_vel_theta, to a centimetre) and has to stay
    inside that envelope on the driving stretches. This is the reproduction
    docs/doorway-pivot.md asked for before SmacPlanner replaced NavFn: the
    same costmap, both searches, the path geometry -- not a closed loop
    started from a NavFn deadlock.
    """
    section("the state lattice will not draw a corner DWB cannot follow")
    sys.path.insert(0, HERE)
    try:
        import hybrid_astar as geo
        import lattice
        import corridor_sim as dwb
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import lattice: %s" % exc)
        return

    radius = dwb.MAX_VEL_X / dwb.MAX_VEL_THETA
    check("DWB's forward envelope is max_vel_x over max_vel_theta",
          geo.MIN_TURNING_RADIUS, radius, tolerance=1e-9)
    check("...and is 0.51 m with the numbers in nav2.yaml",
          radius, 0.51, tolerance=0.005)
    meta, _ = lattice.load_lattice()
    check("the control set's radius is that envelope to a centimetre",
          meta["turning_radius"], 0.5, tolerance=1e-9)
    check("...and the motion model is differential, not ackermann",
          meta["motion_model"], "diff")

    result = lattice.doorway_reproduction()
    ninfo = result["navfn"]
    linfo = result["lattice"]
    check("the grid search still finds a route through the doorway",
          ninfo is not None)
    check("the lattice finds one too", linfo is not None)
    if ninfo is None or linfo is None:
        return
    check("NavFn's doorway corner is tighter than one DWB rollout (%.1f deg > %.1f)"
          % (ninfo["tightest_deg"], result["rollout_deg"]),
          ninfo["followable"], False)
    check("the lattice stays inside that envelope (%.1f deg)"
          % linfo["tightest_deg"],
          linfo["followable"], True)
    check("...without needing a pivot on a doorway that takes an arc",
          linfo["pivots"], 0)
    check("...and does not wander off: the route is within 2x the grid one",
          linfo["length_m"] < 2.0 * ninfo["length_m"], True)


def test_dwb_drives_the_body_into_a_door_frame():
    """The after-floor doorway recording, as a pose test, not a closed loop.

    Mixer floor stopped the pivoting. The rover then drove 3.5 m and sat
    next to a door frame for fifty seconds. Last driving command was
    0.40 m/s with the rectangle already in the inscribed ring, the nose
    4 cm from lethal, and 1.1 m still to the goal. Then one (0, 0).

    The synthetic 0.80 m door is that last driving tick without the
    recording: 14 cm off centre, 20 degrees toward the jamb. Frozen-map
    closed loops are how the 0.8 m look-ahead shipped on worthless
    evidence; this scores the pose once. The recording, when present,
    is the sit: PoseProgressChecker would have fired, and the model
    still had a forward candidate on the stop tick, so FollowPath had
    already ended.
    """
    section("DWB will drive the body into a door-frame halo")
    sys.path.insert(0, HERE)
    try:
        import jam_repro
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import jam_repro: %s" % exc)
        return

    result = jam_repro.jam_reproduction()
    a = result["approach"]
    check("the body already covers the inscribed ring of the jamb",
          a["ring"] > 0, True)
    check("...without covering a lethal cell", a["lethal"], 0)
    check("...while the centre cell is still a legal planner step",
          a["centre"] < 253, True)
    check("DWB still has a legal candidate", a["legal"] > 0, True)
    check("...and the one it picks is 0.40 m/s forward",
          a["best_vx"], 0.40, tolerance=0.01)
    check("there is no reverse sample to back out with",
          a["reverse_samples"], 0)
    check("...even though 30 cm behind is clear",
          a["room_behind_m"], 0.30, tolerance=0.01)
    check("the nose is about 4 cm from lethal, as on the rover",
          a["nose_lethal_m"], 0.22, tolerance=0.03)

    live = result["recording"]
    if live is None:
        print("  .... no recordings/doorway-2026-08-25-after-floor.json")
        return
    check("the recording's last drive is still 0.40 m/s",
          live["last_drive_vx"], 0.40, tolerance=0.01)
    check("...with the body already in the ring and not in lethal",
          live["ring"] > 0 and live["lethal"] == 0, True)
    check("...and more than a goal-tolerance from the plan's end",
          live["goal_m"] > 0.22, True)
    check("then it sat less than 20 cm in fifty seconds",
          live["sat_m"] < 0.20 and live["sat_s"] > 40.0, True)
    check("PoseProgressChecker would have called that stuck",
          live["stuck_at"] is not None, True)
    if live["stuck_at"] is not None:
        check("...inside one 15 s window of the last drive",
              live["stuck_at"] - live["last_drive_t"] < 16.0, True)
    check("on the (0, 0) tick the model still had a legal forward",
          (live["model_legal_at_stop"] or 0) > 0, True)


def test_discovery_stays_on_this_board():
    """A dead radio must not be able to take the ROS graph with it.

    The console saying "only the mapping half is up" with every process still
    listed is CycloneDDS writing to a leftover address (wlan0's .139 after a
    failover onto the dongle). RoboStack's activate hook sets discovery to the
    subnet; dds.sh has to override that after env.sh, in every launcher, or the
    next interface change looks like Nav2 crashing.
    """
    section("discovery stays on this board")
    dds_path = os.path.join(HERE, "dds.sh")
    if not os.path.isfile(dds_path):
        print("  .... skipped, no dds.sh")
        return
    with open(dds_path, encoding="utf-8", errors="replace") as fh:
        dds = fh.read()
    check("dds.sh pins discovery to localhost",
          "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in dds, True)
    check("...and ROS_LOCALHOST_ONLY, so CycloneDDS will not keep LAN peers",
          "ROS_LOCALHOST_ONLY=1" in dds, True)
    for name in ("run_ros_nav.sh", "restart.sh", "run_record.sh"):
        path = os.path.join(HERE, name)
        with open(path, encoding="utf-8", errors="replace") as fh:
            text = fh.read()
        check("%s sources dds.sh after env.sh" % name,
              'DIR/dds.sh' in text and text.find('DIR/env.sh') < text.find('DIR/dds.sh'),
              True)
    with open(os.path.join(HERE, "sweep.sh"), encoding="utf-8", errors="replace") as fh:
        sweep = fh.read()
    check("sweep SIGKILLs what ignored SIGTERM, or the leftover keeps the port",
          "pkill -9 -f" in sweep, True)

    # --- the second mapper ----------------------------------------------------
    # RTAB-Map runs beside slam_toolbox so the two can be measured against each
    # other, and every check below is a fault that was met on the rover while
    # getting it there. They are static reads of the files because that is what
    # catches them: each one fails silently at runtime and looks like something
    # else.
    check("sweep.sh takes RTAB-Map down too, or the next launch runs two",
          "run_rtabmap[.]sh" in sweep and "/lib/rtabmap_slam/rtabmap" in sweep, True)
    check("...and it names it by path, so the pattern cannot match an ssh session",
          "pkill -f 'rtabmap'" not in sweep and 'pkill -f "rtabmap"' not in sweep, True)
    with open(os.path.join(HERE, "run_rtabmap.sh"), encoding="utf-8",
              errors="replace") as fh:
        runrtab = fh.read()
    # The one that matters. A frame has a single parent, so a second publisher of
    # `map -> odom` does not give the controller a second opinion -- it gives it
    # one transform flickering between two, and the rover steers on whichever
    # landed last.
    check("compare mode forbids RTAB-Map the map -> odom transform",
          "PUBLISH_TF=false" in runrtab and "MODE=compare" in runrtab, True)
    check("...and only --primary turns it on",
          runrtab.find("MODE = primary") < 0
          and 'if [ "$MODE" = primary ]' in runrtab, True)
    check("RTAB-Map keeps its grid out of /map, or Nav2 reads two maps",
          "__ns:=/rtabmap" in runrtab, True)
    with open(os.path.join(HERE, "slam.launch.py"), encoding="utf-8",
              errors="replace") as fh:
        slam_launch_text = fh.read()
    check("the launch never starts both mappers, whatever rtabmap:= is set to",
          "!= 'primary'" in slam_launch_text
          and "== 'compare'" in slam_launch_text, True)
    check("RTAB-Map is launched through its wrapper, not as a conda Node()",
          "run_rtabmap.sh" in slam_launch_text
          and 'package="rtabmap' not in slam_launch_text, True)
    rtab_cfg_path = os.path.join(HERE, "config", "rtabmap.yaml")
    if os.path.isfile(rtab_cfg_path):
        with open(rtab_cfg_path, encoding="utf-8", errors="replace") as fh:
            rtab_cfg = fh.read()
        # lidar_node publishes /scan best-effort on purpose. A reliable
        # subscriber against a best-effort publisher is *incompatible* in DDS,
        # not merely mismatched: both sides list the topic and not one message is
        # ever delivered, which reads as RTAB-Map being broken.
        check("RTAB-Map subscribes to /scan best-effort, or it receives nothing",
              "qos_scan: 2" in rtab_cfg, True)
        # Every 2D-lidar recipe written before RTAB-Map 0.21 says `Icp/PM`. This
        # build has `Icp/Strategy`, does not map a parameter it does not know,
        # and logs nothing -- so the file says libpointmatcher and the node
        # quietly runs PCL's ICP with stock settings.
        #
        # Read off the settings rather than the whole file: the comment above
        # that line names the old parameter in order to warn about it, and a
        # check that cannot tell a warning from a setting is worse than none.
        rtab_settings = "\n".join(
            line for line in rtab_cfg.splitlines()
            if line.strip() and not line.lstrip().startswith("#"))
        check("...and uses 0.22's ICP parameter names, which are not the old ones",
              "Icp/Strategy" in rtab_settings and "Icp/PM" not in rtab_settings, True)
        check("...and RTAB-Map's own loop closure is switched on",
              "RGBD/ProximityBySpace" in rtab_cfg, True)
    with open(os.path.join(HERE, "native.sh"), encoding="utf-8",
              errors="replace") as fh:
        native = fh.read()
    # ROS's setup.bash reads $AMENT_TRACE_SETUP_FILES with no default, so under
    # `set -u` it dies naming ROS's file rather than ours.
    check("native.sh survives set -u across ROS's own setup.bash",
          "set +u" in native and "AMENT_TRACE_SETUP_FILES" in native, True)
    check("...and leaves the conda environment rather than layering on it",
          "env -i" in native, True)

    # --- getting off something the rover is touching --------------------------
    # Nav2's Spin, DriveOnHeading and BackUp start their look-ahead projection at
    # the pose the rover is standing in, so a rover in contact is refused every
    # motion in every direction -- it will not drive off the obstacle and it will
    # not turn. `behaviors/` replaces all three with subclasses that differ only
    # in that state. These checks are the fix and its two guard rails: a motion
    # into an obstacle must still be refused, and the reasoning behind the spin
    # only holds while the footprint is a circle.
    import corridor_sim
    import goal_fit as _goal_fit

    def _wall(behind=None, ahead=None, span=4.0):
        """Open ground with a wall that far in front of or behind the rover.

        The rover stands at the origin facing +x. 0.12 m behind is inside the
        chassis, which reaches 0.16 m back from `base_link` -- so the rover is
        genuinely touching, which is the case that matters. The narrow band
        where the costmap says collision and the body is actually clear is only
        about a centimetre wide and is not what this is about.
        """
        res = corridor_sim.RESOLUTION
        cells = int(round(span / res))
        origin = -span / 2.0
        lethal = []
        for col in range(cells):
            x = origin + (col + 0.5) * res
            if (behind is not None and x <= -behind) or \
               (ahead is not None and x >= ahead):
                lethal.extend((col, row) for row in range(cells))
        return _goal_fit.CostGrid(cells, cells, res, origin, origin,
                                  corridor_sim.inflate(cells, cells, lethal))

    wall = _wall(behind=0.12)
    stock_spin = corridor_sim.spin_recovery(wall, 0.0, 0.0, 0.0,
                                            target=math.radians(90))
    escape_spin = corridor_sim.escape_spin(wall, 0.0, 0.0, 0.0,
                                           target=math.radians(90))
    check("a rover touching something behind cannot turn under stock Nav2",
          round(math.degrees(stock_spin[0]), 1), 0.0)
    check("...and can under the escape behaviours, which is the whole point",
          round(math.degrees(escape_spin[0])), 90)
    stock_fwd = corridor_sim.drive_on_heading(wall, 0.0, 0.0, 0.0,
                                              target=0.5, sign=1.0)
    escape_fwd = corridor_sim.escape_drive_on_heading(wall, 0.0, 0.0, 0.0,
                                                      target=0.5, sign=1.0)
    check("...nor drive away from it under stock Nav2", round(stock_fwd[0], 2), 0.0)
    check("...and can drive away from it under the escape behaviours",
          round(escape_fwd[0], 2), 0.5)
    escape_back = corridor_sim.escape_drive_on_heading(wall, 0.0, 0.0, 0.0,
                                                       target=0.3, sign=-1.0)
    check("but reversing *into* the thing it is touching is still refused",
          round(escape_back[0], 2), 0.0)
    ahead = _wall(ahead=0.12)
    escape_into = corridor_sim.escape_drive_on_heading(ahead, 0.0, 0.0, 0.0,
                                                       target=0.5, sign=1.0)
    check("...and so is driving forward into a wall, which is the safety that "
          "must survive all of this", round(escape_into[0], 2), 0.0)
    nav2_path = os.path.join(HERE, "config", "nav2.yaml")
    with open(nav2_path, encoding="utf-8", errors="replace") as fh:
        nav2_cfg = fh.read()
    check("the behaviour server actually loads the escape behaviours",
          "ugv_behaviors::EscapeSpin" in nav2_cfg
          and "ugv_behaviors::EscapeDriveOnHeadingAction" in nav2_cfg
          and "ugv_behaviors::EscapeBackUpAction" in nav2_cfg, True)
    # EscapeSpin is only sound because rotating a circle about its own centre
    # sweeps no new ground. A footprint polygon would break that silently.
    nav2_settings = "\n".join(
        line for line in nav2_cfg.splitlines()
        if line.strip() and not line.lstrip().startswith("#"))
    check("...and the circular footprint EscapeSpin's soundness rests on is "
          "still a circle", "robot_radius:" in nav2_settings
          and "footprint:" not in nav2_settings, True)
    with open(os.path.join(os.path.dirname(HERE), "deploy", "manifest.json"),
              encoding="utf-8") as fh:
        deploy_cfg = json.load(fh)
    ros_nav_cmds = next((c.get("commands") or [] for c in deploy_cfg["components"]
                         if c.get("name") == "ros_nav"), [])
    check("a deploy rebuilds the plugin, or the rover runs last week's .so",
          any("behaviors/build.sh" in cmd for cmd in ros_nav_cmds), True)
    check("...and it builds before it restarts, not after",
          ([i for i, c in enumerate(ros_nav_cmds) if "behaviors/build.sh" in c] or [99])[0]
          < ([i for i, c in enumerate(ros_nav_cmds) if "restart.sh" in c] or [-1])[0],
          True)
    with open(os.path.join(HERE, "restart.sh"), encoding="utf-8", errors="replace") as fh:
        restart = fh.read()
    check("restart.sh will not hang SSH on a wedged ros2 node list",
          "timeout 15 ros2 node list" in restart, True)
    with open(os.path.join(HERE, "nav_record.py"), encoding="utf-8", errors="replace") as fh:
        recorder = fh.read()
    check("a hung nav_record cannot sit in spin_once past the recording window",
          "threading.Timer" in recorder and "os._exit" in recorder, True)
    with open(os.path.join(HERE, "run_record.sh"), encoding="utf-8", errors="replace") as fh:
        wrapper = fh.read()
    check("...and the shell wrapper still fires if Python itself is stuck",
          "timeout --kill-after=15" in wrapper, True)
    manifest_path = os.path.join(os.path.dirname(HERE), "deploy", "manifest.json")
    if os.path.isfile(manifest_path):
        with open(manifest_path, encoding="utf-8") as fh:
            manifest = json.load(fh)
        ros_nav = next((c for c in manifest["components"]
                        if c.get("name") == "ros_nav"), None)
        when = []
        for rule in (ros_nav or {}).get("special_commands") or []:
            when.extend(rule.get("when") or [])
        check("deploying dds.sh replaces the supervisor, or boot still has SUBNET",
              "ros_nav/dds.sh" in when, True)
    else:
        print("  .... skipped, no deploy/manifest.json")


def main():
    test_drive_model()
    test_steering_has_a_small_end()
    test_the_simulated_chassis_is_not_the_mixer_inverted()
    test_the_rover_does_not_wander_down_a_straight_line()
    test_turn_curve()
    test_idle_behaviour()
    test_gyro_bias()
    test_odometry()
    test_scan_binning()
    test_bridge_protocol()
    test_nav2_error_codes()
    test_nav2_error_codes_match_the_installed_nav2()
    test_the_transform_budget_survives_a_scan()
    test_heading_arithmetic()
    test_steering_bearing()
    test_the_two_halves_agree_on_the_port()
    test_calibration_store()
    test_configs_agree()
    test_goal_fits_before_it_is_sent()
    test_a_route_is_budgeted_on_the_route()
    test_progress_is_not_only_translation()
    test_dwb_will_not_sample_a_turn_the_wheels_cannot_hold()
    test_lattice_respects_the_dwb_envelope()
    test_dwb_drives_the_body_into_a_door_frame()
    test_discovery_stays_on_this_board()
    print("\n%d passed, %d failed" % (PASSED, FAILED))
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
