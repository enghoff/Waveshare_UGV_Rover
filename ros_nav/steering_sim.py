#!/usr/bin/env python3
"""Drive a simulated rover down a straight line and see whether it wanders.

    python3 steering_sim.py                 # all three mixers, side by side
    python3 steering_sim.py --trace         # and draw what the path looked like

**Read this before trusting a number out of it.** The first version of this
simulation could not fail. Its rover derived rotation from the PWM difference
using the very curve the mixer used to choose that difference, so plant and
controller were exact inverses and the loop gain was 1.0 by construction. Started
exactly on the line it drove a perfect straight line for ever -- and so did the
*broken* mixer, the one that had been seen zig-zagging the length of every route.
The only thing that could push that rover off its path was the follower choosing
to.

So it could detect exactly one class of fault: a mixer that cannot *express* a
small steering request. That was a real fault and it was really there, and fixing
it really did take the hard zig-zag out. It could not detect a mixer that
expresses the request and then gets a different amount of rotation than it asked
for, which is the larger fault underneath and the reason the rover still curved.

The plant here is now measured independently of the mixer, by `steer_gain.py`,
which asks for a steering request through `/cmd_vel` exactly as Nav2 does and
reads what the gyro says came out:

- **The steering curve.** How much the chassis actually rotates for a given PWM
  difference *while rolling*. This is not the pivot curve. A tracked chassis
  spinning on the spot is dragging its whole contact patch sideways and the pivot
  curve is mostly a measurement of that scrub; rolling, almost none of it is
  present. Measured, the pivot curve wanted 86 PWM of difference for 10 deg/s and
  the rover turned at 85.6.
- **The pull.** Asked for no rotation at all, this rover curves left at about
  0.93 deg/s at 0.35 m/s. Four runs, and the lidar and the gyro agree to within
  0.07 deg/s, so it is the chassis and not the instruments.

Two assumptions remain, both about dynamics rather than about gain, and both are
swept in the output because the argument rests on them: a first-order lag of
0.15 s on rotation, and 0.2 s of delay between measuring a pose and acting on it.
"""

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for _candidate in (os.path.join(HERE, "..", "lidar_slam"),
                   os.path.join(HERE, "..", "..", "lidar_slam")):
    if os.path.isdir(_candidate):
        sys.path.insert(0, os.path.abspath(_candidate))
        break

from nav_types import MAX_TURN_DPS, MIN_TURN_DPS, TOP_PWM     # noqa: E402

STORE = os.path.expanduser("~/ugv/odometry.json")

# The chassis as it was measured on 2026-08-23, so this runs on a workstation
# with no rover attached. Overridden by the store when there is one.
TURN_POINTS = [[85, 9.1718], [90, 12.625], [95, 14.8219], [105, 24.4707],
               [120, 40.78], [145, 65.573], [170, 95.8467]]
DRIVE_POINTS = [[85, 0.3326], [90, 0.3606], [95, 0.396], [115, 0.4851],
                [140, 0.678]]
#: PWM difference between the wheels against the rotation it really produces
#: while rolling, from steer_gain.py. The points below 35 are the symmetric half
#: of a matched left/right pair, so the chassis's own pull is not baked in.
STEER_POINTS = [[4, 0.50], [8, 1.30], [12, 2.25], [18, 4.30], [25, 6.75],
                [35, 14.55], [50, 32.3], [70, 60.9], [86, 84.5],
                [100, 106.0], [124, 110.6]]
#: Degrees of unasked-for turn per metre driven: +0.93 deg/s at 0.35 m/s.
STRAIGHT_BIAS_DEG_PER_M = 2.66

DT = 0.1                 # the controller's period; Nav2's controller_frequency
LAG_S = 0.15             # first-order lag on rotation -- see the module docstring
LOOKAHEAD_M = 0.6        # pure pursuit's carrot
DELAY_S = 0.2            # how stale the pose the follower steers on is
SPEED_MS = 0.35


from drive_mixer import TURN_PWM_MAX, mix, pwm_for


# --- the three mixers -----------------------------------------------------------
# All three are the *same* function where they can be. `mix` is the code that
# actually runs on the rover, and simulating a paraphrase of it would prove
# nothing; what separates the last two is only what they have been told about the
# chassis. The first is kept because it is what the zig-zag was.
def mix_floored(linear, angular, turn_points, drive_points):
    """The mixer as it was when the rover zig-zagged, kept for comparison.

    The steering term is floored at `MIN_TURN_DPS` and then at the slowest PWM
    anybody measured, so every request under 12 deg/s produces the same violent
    pivot, and the ceiling scales both wheels so a commanded 45 deg/s arrives
    as 25.
    """
    speed = min(drive_points[-1][1], abs(linear))
    throttle = pwm_for(drive_points, speed, ceiling=TOP_PWM)
    if linear < 0:
        throttle = -throttle
    dps = math.degrees(min(math.radians(MAX_TURN_DPS), abs(angular)))
    turn = 0 if dps < 1e-3 else pwm_for(turn_points, max(dps, MIN_TURN_DPS))
    if angular < 0:
        turn = -turn
    left, right = throttle - turn, throttle + turn
    peak = max(abs(left), abs(right))
    if peak > TURN_PWM_MAX:
        scale = TURN_PWM_MAX / peak
        left, right = left * scale, right * scale
    return int(round(left)), int(round(right))


def mix_pivot(linear, angular, turn_points, drive_points):
    """The fix for the zig-zag alone: small requests survive, but the amount of
    rotation they buy is still read off the pivot curve, and nothing corrects the
    chassis's own pull. This is what was on the rover when the trail was still
    visibly curved."""
    return mix(linear, angular, turn_points, drive_points)


def mix_measured(linear, angular, turn_points, drive_points):
    """Steering on the curve measured while rolling, and trimmed for the pull."""
    return mix(linear, angular, turn_points, drive_points,
               steer_points=STEER_POINTS,
               straight_bias_deg_per_m=STRAIGHT_BIAS_DEG_PER_M)


# --- the chassis ----------------------------------------------------------------
def curve(points, pwm):
    """What the chassis does at this PWM, by its own measurements.

    Linear to the origin below the slowest measured point: no PWM is no motion, so
    the curve passes through zero, and the shape between is not measured.
    """
    p = abs(pwm)
    if p <= 0:
        return 0.0
    if p <= points[0][0]:
        return points[0][1] * p / points[0][0]
    for (p0, v0), (p1, v1) in zip(points, points[1:]):
        if p <= p1:
            return v0 + (p - p0) * (v1 - v0) / (p1 - p0)
    (p0, v0), (p1, v1) = points[-2], points[-1]
    return v1 + (p - p1) * (v1 - v0) / (p1 - p0)


class Chassis:
    """The rover as its measured curves, with a lag on rotation and its own pull.

    **The steering curve here is a measurement, not the mixer's model.** That is
    the whole point: if this used the same curve the mixer inverts, the loop gain
    would be exactly one whatever either of them said, and a mixer steering on
    entirely the wrong curve would simulate perfectly. It is what makes this able
    to fail.
    """

    def __init__(self, turn_points, drive_points, lag_s=LAG_S,
                 steer_points=None, bias_deg_per_m=STRAIGHT_BIAS_DEG_PER_M):
        self.turn_points = turn_points
        self.drive_points = drive_points
        self.steer_points = steer_points or STEER_POINTS
        self.bias_deg_per_m = bias_deg_per_m
        self.lag_s = lag_s
        self.x = self.y = self.yaw = 0.0
        self.rate = 0.0                       # rad/s, lagged

    def step(self, left, right, dt):
        throttle = (left + right) / 2.0
        differential = (right - left) / 2.0
        speed = math.copysign(curve(self.drive_points, throttle), throttle or 1.0)
        if throttle == 0:
            speed = 0.0
        # Rolling and pivoting are different manoeuvres on a tracked chassis, and
        # the two curves differ by nearly an order of magnitude, so which one
        # applies depends on whether the rover is going anywhere.
        points = self.steer_points if throttle else self.turn_points
        wanted = math.radians(curve(points, differential))
        if differential < 0:
            wanted = -wanted
        if throttle:
            wanted += math.radians(self.bias_deg_per_m * speed)
        alpha = dt / max(dt, self.lag_s)
        self.rate += alpha * (wanted - self.rate)
        self.yaw += self.rate * dt
        self.x += speed * math.cos(self.yaw) * dt
        self.y += speed * math.sin(self.yaw) * dt
        return speed, self.rate


# --- the follower ---------------------------------------------------------------
def pursue(chassis, goal_x, lookahead=LOOKAHEAD_M, speed=SPEED_MS):
    """Pure pursuit along the x axis: the carrot is a lookahead ahead, on y=0.

    Not DWB, and it does not need to be. What every path follower has in common is
    that it asks for a small angular velocity when it is nearly on the path, and
    what happens to that request is the subject here.
    """
    cx = min(goal_x, chassis.x + lookahead)
    dx, dy = cx - chassis.x, 0.0 - chassis.y
    if math.hypot(dx, dy) < 1e-6:
        return 0.0, 0.0
    bearing = math.atan2(dy, dx) - chassis.yaw
    bearing = math.atan2(math.sin(bearing), math.cos(bearing))
    curvature = 2.0 * math.sin(bearing) / max(1e-3, math.hypot(dx, dy))
    angular = max(-math.radians(MAX_TURN_DPS),
                  min(math.radians(MAX_TURN_DPS), speed * curvature))
    return speed, angular


def run(mixer, turn_points, drive_points, metres=4.0, start_offset=0.0,
        start_yaw=0.0, lag_s=LAG_S, delay_s=DELAY_S, steer_points=None,
        bias_deg_per_m=STRAIGHT_BIAS_DEG_PER_M):
    """Follow y=0 for `metres` and report how well. Returns the track and stats."""
    chassis = Chassis(turn_points, drive_points, lag_s, steer_points,
                      bias_deg_per_m)
    chassis.y = start_offset
    chassis.yaw = start_yaw
    track = [(chassis.x, chassis.y)]
    commands = []
    held = int(round(delay_s / DT))
    queue = []
    steps = int(metres / (SPEED_MS * DT)) + 40
    for _ in range(steps):
        if chassis.x >= metres:
            break
        linear, angular = pursue(chassis, metres)
        queue.append(mixer(linear, angular, turn_points, drive_points))
        left, right = queue.pop(0) if len(queue) > held else (0, 0)
        chassis.step(left, right, DT)
        track.append((chassis.x, chassis.y))
        commands.append((angular, (right - left) / 2.0, chassis.rate))

    offsets = [abs(y) for _, y in track]
    rms = math.sqrt(sum(o * o for o in offsets) / max(1, len(offsets)))
    reversals = 0
    previous = 0
    for _, _, rate in commands:
        sign = 0 if abs(rate) < math.radians(0.5) else (1 if rate > 0 else -1)
        if sign and previous and sign != previous:
            reversals += 1
        if sign:
            previous = sign
    travelled = track[-1][0] - track[0][0]
    settled = track[int(len(track) * 0.4):] or track
    settled_y = [y for _, y in settled]
    settled_rms = math.sqrt(sum(y * y for y in settled_y) / max(1, len(settled_y)))
    return {
        "track": track,
        "rms_offset_m": rms,
        "worst_offset_m": max(offsets),
        "settled_rms_m": settled_rms,
        "settled_swing_m": max(settled_y) - min(settled_y),
        "reversals": reversals,
        "reversals_per_m": reversals / max(0.1, travelled),
        "final_offset_m": abs(track[-1][1]),
        "asked_vs_got": [(math.degrees(a), d, math.degrees(r))
                         for a, d, r in commands[:6]],
    }


def trace(track, width=74, height=17, span=None):
    """The path as characters, because a picture of a wiggle is worth a number."""
    xs = [x for x, _ in track]
    ys = [y for _, y in track]
    span = span or max(0.05, max(abs(min(ys)), abs(max(ys))) * 1.15)
    rows = [[" "] * width for _ in range(height)]
    mid = height // 2
    for i in range(width):
        rows[mid][i] = "-"
    for x, y in track:
        cx = int((x - xs[0]) / max(1e-6, xs[-1] - xs[0]) * (width - 1))
        cy = int(mid - (y / span) * mid)
        if 0 <= cy < height and 0 <= cx < width:
            rows[cy][cx] = "#"
    out = ["    +%s+" % ("-" * width)]
    for i, row in enumerate(rows):
        label = "%+5.2f" % (span * (mid - i) / mid) if i in (0, mid, height - 1) \
            else "     "
        out.append("%s|%s|" % (label, "".join(row)))
    out.append("    +%s+  %.1f m of route" % ("-" * width, xs[-1] - xs[0]))
    return "\n".join(out)


def load_points():
    try:
        with open(STORE) as fh:
            store = json.load(fh)
        return (store.get("turn_pwm_points") or TURN_POINTS,
                store.get("drive_pwm_points") or DRIVE_POINTS,
                store.get("steer_pwm_points") or STEER_POINTS,
                store.get("straight_bias_deg_per_m", STRAIGHT_BIAS_DEG_PER_M),
                STORE)
    except (OSError, ValueError):
        return (TURN_POINTS, DRIVE_POINTS, STEER_POINTS,
                STRAIGHT_BIAS_DEG_PER_M, "the values measured on 2026-08-23")


MIXERS = (("floored", mix_floored), ("pivot-curve", mix_pivot),
          ("measured", mix_measured))


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--trace", action="store_true", help="draw the paths")
    p.add_argument("--metres", type=float, default=4.0)
    p.add_argument("--offset", type=float, default=0.10,
                   help="how far off the line to start, in metres")
    p.add_argument("--lag", type=float, default=LAG_S)
    p.add_argument("--delay", type=float, default=DELAY_S,
                   help="seconds of loop delay between measuring and acting")
    args = p.parse_args()

    turn_points, drive_points, steer_points, bias, where = load_points()
    print("chassis from %s" % where)
    print("  turn   %s" % ", ".join("PWM %d=%.1f deg/s" % (p_, v)
                                    for p_, v in turn_points))
    print("  drive  %s" % ", ".join("PWM %d=%.2f m/s" % (p_, v)
                                    for p_, v in drive_points))
    print("  steer  %s" % ", ".join("%g=%.1f deg/s" % (p_, v)
                                    for p_, v in steer_points[:6]))
    print("  pull   %+.2f deg per metre driven" % bias)
    print()

    print("a steering request at 0.35 m/s, and what the rover would really do:")
    print("  asked   floored          pivot-curve       measured")
    print("  deg/s   diff  ->real     diff  ->real      diff  ->real")
    for dps in (0.5, 1, 2, 5, 10, 20, 45):
        w = math.radians(dps)
        cells = []
        for _, mixer in MIXERS:
            l, r = mixer(0.35, w, turn_points, drive_points)
            d = (r - l) / 2.0
            cells.append((d, curve(steer_points, d)))
        print("  %5.1f   %5.0f %6.1f    %5.0f %6.1f     %5.0f %6.1f"
              % (dps, cells[0][0], cells[0][1], cells[1][0], cells[1][1],
                 cells[2][0], cells[2][1]))
    print()

    results = {}
    for name, mixer in MIXERS:
        results[name] = run(mixer, turn_points, drive_points,
                            metres=args.metres, start_offset=args.offset,
                            lag_s=args.lag, delay_s=args.delay,
                            steer_points=steer_points, bias_deg_per_m=bias)

    print("following a straight line for %.1f m, starting %.0f cm off it:"
          % (args.metres, args.offset * 100))
    print("               once settled, the rover...")
    print("               wanders +/-   swings over   reverses its steering")
    for name, _ in MIXERS:
        r = results[name]
        print("  %-11s  %6.1f cm      %6.1f cm      %5.1f times per metre"
              % (name, r["settled_rms_m"] * 100, r["settled_swing_m"] * 100,
                 r["reversals_per_m"]))
    print()

    print("sensitivity to what is assumed rather than measured (settled swing, cm):")
    print("   rotational lag                    loop delay")
    print("   lag   floor  pivot  meas          delay  floor  pivot  meas")
    for lag, delay in zip((0.05, 0.10, 0.15, 0.25, 0.40),
                          (0.0, 0.1, 0.2, 0.3, 0.4)):
        row = []
        for held, use_lag, use_delay in ((True, lag, args.delay),
                                         (False, args.lag, delay)):
            for _, mixer in MIXERS:
                row.append(run(mixer, turn_points, drive_points,
                               metres=args.metres, start_offset=args.offset,
                               lag_s=use_lag, delay_s=use_delay,
                               steer_points=steer_points,
                               bias_deg_per_m=bias)["settled_swing_m"] * 100)
        print("  %.2fs %6.1f %6.1f %5.1f          %.2fs %6.1f %6.1f %5.1f"
              % (lag, row[0], row[1], row[2], delay, row[3], row[4], row[5]))
    print()

    print("what the follower asked for, and what the wheels did (first steps):")
    for name, _ in MIXERS:
        print("  %s:" % name)
        for asked, diff, got in results[name]["asked_vs_got"]:
            print("    asked %+6.1f deg/s -> differential %+6.1f -> turned %+6.1f"
                  % (asked, diff, got))

    if args.trace:
        span = max(0.01, results["pivot-curve"]["settled_swing_m"] * 0.6)
        for name, _ in MIXERS:
            track = results[name]["track"]
            settled = track[int(len(track) * 0.4):]
            print("\n%s, the settled part of the route, %.0f cm across:"
                  % (name, span * 200))
            print(trace(settled, span=span))
    return 0


if __name__ == "__main__":
    sys.exit(main())
