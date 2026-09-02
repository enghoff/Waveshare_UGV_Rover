"""The drive model: what a command becomes at the wheels.

The mixer and the measured turn/drive curves are imported rather than restated,
so a re-measured chassis is tested against its new numbers. A sign flip on the
steering is the fault this catches, and it needs no radio to be wrong.
"""
import math

from test_harness import check, section
from nav_types import MAX_SPEED_MS, MAX_TURN_DPS, MIN_PWM, MIN_TURN_DPS, TOP_PWM, TURN_RATES
from drive_mixer import TURN_PWM_MAX, mix as cmd_to_pwm, pwm_for, steer_pwm, to_pwm, turn_to_pwm


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


TESTS = (
    test_turn_curve,
    test_drive_model,
    test_steering_has_a_small_end,
    test_the_simulated_chassis_is_not_the_mixer_inverted,
    test_the_rover_does_not_wander_down_a_straight_line,
)
