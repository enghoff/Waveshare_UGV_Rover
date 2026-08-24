#!/usr/bin/env python3
"""Turn a `/cmd_vel` into the driver board's left and right PWM pair.

Its own module, with no ROS in it, because three things need this arithmetic and
only one of them can import `rclpy`: the base node that runs it on the rover, the
selftest that runs on a workstation with no ROS at all, and `steering_sim.py`,
which drives a simulated rover down a straight line to see whether it wanders.
They used to carry copies. A copy of a *table* drifts visibly; a copy of a control
law drifts invisibly, and this is the control law.

**Everything here is measured, nothing is assumed.** Both curves come from
`~/ugv/odometry.json`, written by `calibrate_chassis.py` against this chassis on
this floor: PWM to metres a second by driving at fixed PWM and reading the
distance off the lidar, and PWM to degrees a second by timing fixed-PWM bursts
against the gyro. The constants in `lidar_slam/nav_types.py` are a fallback and a
bad one -- they describe the previous rover -- so a missing curve is warned about
loudly rather than papered over.

**The rule that matters is the one about the small end.** A wheel starting from
rest does nothing below about PWM 40; it buzzes. So the drive curve refuses to
extrapolate below the slowest thing anybody measured, and `MIN_TURN_DPS` refuses
to attempt a rotation too small to be a move. Both are right, and both are rules
about *starting from rest*.

Neither governs the difference between two wheels that are already turning. That
distinction was missing, and it was the whole of a fault: with the from-rest floor
applied to the steering term, every steering request under 12 deg/s -- which is
most of them, because a follower that is nearly on its path asks for very little
-- came out as one wheel stopped and the other at full. Nav2 asking for half a
degree a second and Nav2 asking for ten degrees a second produced the identical
PWM pair. The rover could not make a gentle correction, so it corrected hard,
overshot, corrected hard the other way, and zig-zagged the length of every route.

    python3 steering_sim.py --trace     # the fault and the fix, side by side
"""

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
for _candidate in (os.path.join(_HERE, "..", "lidar_slam"),
                   os.path.join(_HERE, "..", "..", "lidar_slam")):
    if os.path.isdir(_candidate):
        _candidate = os.path.abspath(_candidate)
        if _candidate not in sys.path:
            sys.path.insert(0, _candidate)
        break

from nav_types import (MAX_SPEED_MS, MAX_TURN_DPS, MIN_PWM,       # noqa: E402
                       MIN_TURN_DPS, TOP_PWM, TURN_RATES)

# The two points somebody actually measured on the previous chassis, from
# lidar_slam/nav_types.py: `{180: (170.0 deg/s, 9.0 deg of coast),
# 80: (31.6, 2.0)}`. Sorted so nothing here depends on dict order.
_TURN_FIT = sorted((pwm, rate) for pwm, (rate, _coast) in TURN_RATES.items())
_TURN_LO, _TURN_HI = _TURN_FIT[0], _TURN_FIT[-1]
TURN_PWM_MAX = _TURN_HI[0]

#: The fallback curve, used only until calibrate_chassis.py has run. Kept because
#: a rover with no calibration should still move, and flagged loudly at startup
#: because these numbers are known to be wrong for this chassis.
FALLBACK_TURN_POINTS = [[pwm, rate] for pwm, rate in _TURN_FIT]


def to_pwm(value):
    """A normalised -1..1 wheel command as the firmware's PWM.

    The same curve the rover's own drive controller used, and deliberately so: below
    MIN_PWM the motors buzz and do not turn, so the usable range starts there
    rather than at zero, and a controller that has not been told this spends the
    bottom quarter of its output commanding a noise.
    """
    if abs(value) < 1e-3:
        return 0
    magnitude = MIN_PWM + abs(value) * (TOP_PWM - MIN_PWM)
    return int(round(magnitude if value > 0 else -magnitude))


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
    # never happens and a controller waiting for it. See `steer_pwm` for the one
    # case where that rule does not apply.
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


def interp(points, wanted):
    """Invert a measured curve to a *float*, with no floors of any kind.

    `pwm_for` is the right thing for a wheel starting from rest and the wrong
    thing for a steering differential: it clamps its answer up to MIN_PWM, which
    for a difference between two wheels means the smallest steering correction
    available is a large one. It also rounds, and at this end of the curve the
    difference between 4 and 5 PWM is the difference between a gentle correction
    and a noticeable one.

    Below the slowest measured point the curve runs straight to the origin, which
    it has to pass through: no difference between the wheels is no rotation.
    """
    if not points or wanted <= 0:
        return 0.0
    if wanted <= points[0][1]:
        return points[0][0] * wanted / points[0][1]
    for (p0, v0), (p1, v1) in zip(points, points[1:]):
        if wanted <= v1:
            if v1 == v0:
                return float(p1)
            return p0 + (wanted - v0) / (v1 - v0) * (p1 - p0)
    (p0, v0), (p1, v1) = points[-2], points[-1]
    if v1 == v0:
        return float(p1)
    return min(float(TURN_PWM_MAX), p1 + (wanted - v1) * (p1 - p0) / (v1 - v0))


def turn_to_pwm(dps, points=None):
    """The per-wheel PWM that turns this chassis at `dps` *from a standstill*.

    For a rotation on the spot. Both wheels are starting from rest and have to
    clear stiction, so the floors apply: a request under `MIN_TURN_DPS` is lifted
    to it, and the curve will not go below the slowest PWM anybody measured.
    Steering while already driving is `steer_pwm`, which is a different question.
    """
    wanted = abs(dps)
    if wanted < 1e-3:
        return 0
    return pwm_for(points or FALLBACK_TURN_POINTS, max(wanted, MIN_TURN_DPS))


def steer_pwm(dps, points=None, driving=False, steer_points=None):
    """The PWM *difference* between the wheels that turns the chassis at `dps`.

    Standing still, this is `turn_to_pwm` and the from-rest floors apply. Driving,
    they must not -- both wheels are already well above stiction, so ten PWM
    between them is a perfectly achievable gentle curve, and applying the
    from-rest floor to that difference is what made every small correction come
    out as a violent pivot.

    **Driving also uses a different curve, and that is the larger of the two
    corrections.** `points` is the pivot curve, measured by spinning the wheels
    against each other on the spot, and a tracked chassis pivoting on the spot is
    dragging its whole contact patch sideways. The numbers say how much: that
    curve implies an effective track width between 1.09 m and 4.16 m on a rover
    0.22 m wide. Almost none of that scrub is present when the rover is rolling
    forwards and one track simply runs faster than the other, so the same
    differential turns it very much harder.

    Measured on this chassis with `steer_gain.py`, the pivot curve asked for 86
    PWM of differential to get 10 deg/s and the rover turned at 85.6. Every
    steering request was being over-served by between two and nine times, which
    a follower can only answer by correcting back, and that is a weave rather
    than a route.

    So `steer_points` -- the differential-to-rotation curve measured while
    actually rolling -- is used whenever there is one. Without it this falls back
    to the pivot curve, which is wrong by the factors above and is why
    `base_node` says so loudly at startup.
    """
    wanted = abs(dps)
    if wanted < 1e-3:
        return 0.0
    if not driving:
        return float(turn_to_pwm(wanted, points or FALLBACK_TURN_POINTS))
    if steer_points:
        return interp(steer_points, wanted)
    points = points or FALLBACK_TURN_POINTS
    slow_pwm, slow_dps = points[0]
    if wanted < slow_dps:
        return slow_pwm * wanted / slow_dps
    return float(pwm_for(points, wanted))


def mix(linear, angular, turn_points=None, drive_points=None,
        steer_points=None, straight_bias_deg_per_m=0.0):
    """A `/cmd_vel` as the firmware's left and right PWM pair.

    Done in PWM rather than in normalised units, because that is where both
    calibrations live: each curve is a measured map from PWM to motion, and mixing
    normalised numbers and converting once at the end -- which is what the rover's
    own drive controller did -- would apply the motors' floor to the sum instead of
    to each part.

    Positive `angular.z` is counter-clockwise by REP-103, which is a left turn, so
    the left wheel goes backwards. That is the one sign here worth checking
    against the rover rather than against the code, and it was: a commanded
    +20 deg/s turned it anticlockwise.

    **When the pair will not fit, speed is given up and rotation is kept.** A
    rover that advances more slowly than it was told still follows its route; one
    that turns more slowly than it was told leaves it. The first version scaled
    both ends of the pair to fit, which reads as fair and is not -- it quietly
    reduced the rotation as well, so a commanded 45 deg/s came out as 25.

    **`straight_bias_deg_per_m` is what this chassis does when asked for nothing.**
    Told to drive straight it curves left, and both the gyro and the lidar agree
    on it: four runs of 1.3 m measured +0.93 deg/s against +0.98 from the gyro, so
    it is the rover and not its instruments. Left uncorrected it is a heading
    error the controller has to keep paying off for the whole length of a route,
    and paying it off through a steering channel that was itself over-responding
    is most of what a weaving trail is made of.

    It is held as degrees per metre driven rather than as degrees a second,
    because a small mismatch between two wheels is a constant *curvature* -- go
    twice as fast and you turn twice as fast through the same arc. That is the
    physical shape of the fault, but it is worth saying it was measured at one
    speed only, so the scaling is reasoned rather than observed.
    """
    if drive_points:
        # Clamped to the fastest speed actually measured, not to MAX_SPEED_MS.
        # That constant says 0.35 m/s and this chassis was measured at 0.33 at its
        # slowest usable PWM and 0.68 at PWM 140 -- so the "maximum" is very nearly
        # the minimum, and clamping to it pinned every Nav2 command to the slowest
        # PWM the motors will turn at. There was no speed control at all.
        speed = min(drive_points[-1][1], abs(linear))
        throttle = float(pwm_for(drive_points, speed, ceiling=TOP_PWM)) \
            if speed > 0 else 0.0
    else:
        speed = min(MAX_SPEED_MS, abs(linear))
        throttle = float(to_pwm(speed / MAX_SPEED_MS))
    if linear < 0:
        throttle = -throttle

    driving = abs(throttle) > 0
    wanted_dps = math.copysign(
        math.degrees(min(math.radians(MAX_TURN_DPS), abs(angular))), angular)
    if driving and straight_bias_deg_per_m:
        # Ask the chassis for the rotation wanted *less* the rotation it makes on
        # its own, which is proportional to how fast it is going. Only while
        # driving: this is a rolling asymmetry between the two tracks and it has
        # nothing to say about a pivot on the spot.
        # Signed with the direction of travel: a track that runs slow turns the
        # rover one way going forwards and the other way going backwards.
        wanted_dps -= straight_bias_deg_per_m * math.copysign(speed, linear)
        wanted_dps = max(-MAX_TURN_DPS, min(MAX_TURN_DPS, wanted_dps))
    turn = steer_pwm(abs(wanted_dps), turn_points, driving=driving,
                     steer_points=steer_points)
    if wanted_dps < 0:
        turn = -turn

    left, right = throttle - turn, throttle + turn
    peak = max(abs(left), abs(right))
    if peak > TURN_PWM_MAX:
        # Take it out of the throttle first, which is the half that can be given
        # up without leaving the route.
        room = max(0.0, TURN_PWM_MAX - abs(turn))
        throttle = math.copysign(min(abs(throttle), room), throttle)
        left, right = throttle - turn, throttle + turn
        peak = max(abs(left), abs(right))
        if peak > TURN_PWM_MAX:
            # The rotation alone is past the ceiling, so there is nothing left to
            # give up. Scale rather than clip: clipping one wheel and not the
            # other turns the request into a different one.
            scale = TURN_PWM_MAX / peak
            left, right = left * scale, right * scale
    return int(round(left)), int(round(right))
