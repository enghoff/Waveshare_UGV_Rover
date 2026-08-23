#!/usr/bin/env python3
"""Drive a simulated rover down a straight line and see whether it wanders.

    python3 steering_sim.py                 # both mixers, side by side
    python3 steering_sim.py --trace         # and draw what the path looked like

This exists because the rover was seen zig-zagging the whole length of every
route, and the rover was tethered to a charger at the time. It reproduces that
without moving anything, which is worth doing anyway: a control loop is far easier
to understand at a thousand runs a second than at one run per battery charge.

**What is simulated and what is not.** The loop here is a pure-pursuit follower at
10 Hz, which is not DWB -- but the fault does not depend on which controller is
upstream. Any follower that asks for a small heading correction when it is nearly
on the path will do, and every follower does that; DWB is one of them. What
matters is the *plant*, and the plant here is this chassis's own measured curves
from `~/ugv/odometry.json`.

Two assumptions are made about the plant and both are stated because the argument
rests on them:

- **Below the slowest measured turn, the response is assumed linear to zero.** A
  differential of nothing produces a rotation of nothing, so the curve must pass
  through the origin; the shape between there and the slowest measured point
  (PWM 85, 9.2 deg/s) is not measured. Linear is the least assuming choice.
- **The rover is given a first-order lag of 0.15 s on its rotation.** The chassis
  has real inertia -- `nav_types.py` records 9 degrees of coast at PWM 180 -- and
  a lag makes an oscillation worse, never better. Omitting it would flatter the
  result, so it is here.

Neither assumption is doing the work. The zig-zag comes from the *mixer*, which
maps a continuum of requests onto a step: see what it prints.
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

from nav_types import (MAX_TURN_DPS, MIN_TURN_DPS,               # noqa: E402
                       TOP_PWM)

STORE = os.path.expanduser("~/ugv/odometry.json")

# The chassis as it was measured on 2026-08-23, so this runs on a workstation with
# no rover attached. Overridden by the store when there is one.
TURN_POINTS = [[85, 9.1718], [90, 12.625], [95, 14.8219], [105, 24.4707],
               [120, 40.78], [145, 65.573], [170, 95.8467]]
DRIVE_POINTS = [[85, 0.3326], [90, 0.3606], [95, 0.396], [115, 0.4851],
                [140, 0.678]]

DT = 0.1                 # the controller's period; Nav2's controller_frequency
LAG_S = 0.15             # first-order lag on rotation -- see the module docstring
LOOKAHEAD_M = 0.6        # pure pursuit's carrot
# How stale the pose the follower steers on is, end to end. Nav2's
# controller at 10 Hz, the velocity smoother, the board bridge and an ESP32
# that reports at about 17 Hz each add their share. Two tenths is a
# conservative reading of that chain.
DELAY_S = 0.2
SPEED_MS = 0.35


# --- the two mixers -------------------------------------------------------------
# The new one is imported, not copied: it is the code that actually runs on the
# rover, and a simulation of a different mixer would prove nothing. The old one is
# kept here, and only here, because its whole purpose is to be the thing the fix
# is compared against -- there is no other reason for it to exist any more.
from drive_mixer import (TURN_PWM_MAX, mix as mix_new,           # noqa: E402
                         pwm_for, steer_pwm)


def mix_old(linear, angular, turn_points, drive_points):
    """The mixer as it was, kept for comparison and nothing else.

    Two faults, and the first is the one that made the rover wander. The steering
    term is floored at `MIN_TURN_DPS` and then at the slowest PWM anybody ever
    measured, so every request under 12 deg/s produces the same violent pivot.
    The second shows at the other end: when the pair runs past the firmware's
    ceiling both wheels are scaled to fit, which reduces the rotation as well as
    the speed, so a commanded 45 deg/s comes out as 25.
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
    """The rover as its two measured curves, with a lag on rotation.

    A PWM pair is decomposed the way the mixer composes it -- the mean advances,
    the difference rotates -- because that decomposition is what both curves were
    measured under: the drive curve by running both wheels together, the turn curve
    by running them opposite.
    """

    def __init__(self, turn_points, drive_points, lag_s=LAG_S):
        self.turn_points = turn_points
        self.drive_points = drive_points
        self.lag_s = lag_s
        self.x = self.y = self.yaw = 0.0
        self.rate = 0.0                       # rad/s, lagged

    def step(self, left, right, dt):
        throttle = (left + right) / 2.0
        differential = (right - left) / 2.0
        speed = math.copysign(curve(self.drive_points, throttle), throttle or 1.0)
        if throttle == 0:
            speed = 0.0
        wanted = math.radians(curve(self.turn_points, differential))
        if differential < 0:
            wanted = -wanted
        # First order towards what the wheels are asking for.
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
    that is the request the old mixer could not deliver.
    """
    cx = min(goal_x, chassis.x + lookahead)
    dx, dy = cx - chassis.x, 0.0 - chassis.y
    if math.hypot(dx, dy) < 1e-6:
        return 0.0, 0.0
    bearing = math.atan2(dy, dx) - chassis.yaw
    bearing = math.atan2(math.sin(bearing), math.cos(bearing))
    # The standard pure-pursuit curvature, capped at what Nav2 is allowed to ask.
    curvature = 2.0 * math.sin(bearing) / max(1e-3, math.hypot(dx, dy))
    angular = max(-math.radians(MAX_TURN_DPS),
                  min(math.radians(MAX_TURN_DPS), speed * curvature))
    return speed, angular


def run(mixer, turn_points, drive_points, metres=4.0, start_offset=0.0,
        start_yaw=0.0, lag_s=LAG_S, delay_s=DELAY_S):
    """Follow y=0 for `metres` and report how well. Returns the track and stats.

    `delay_s` is how stale the pose the follower steers on is. The real loop has
    several helpings of this and none of them are optional: Nav2's controller runs
    at 10 Hz on odometry that reaches it through the velocity smoother and the
    board bridge, and the driver board itself only speaks at about 17 Hz. Delay is
    what turns a controller that merely overshoots into one that oscillates, so
    leaving it out would understate the fault rather than the fix.
    """
    chassis = Chassis(turn_points, drive_points, lag_s)
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
        # The wheels act on a command computed `held` steps ago.
        left, right = queue.pop(0) if len(queue) > held else (0, 0)
        chassis.step(left, right, DT)
        track.append((chassis.x, chassis.y))
        commands.append((angular, (right - left) / 2.0, chassis.rate))

    offsets = [abs(y) for _, y in track]
    rms = math.sqrt(sum(o * o for o in offsets) / max(1, len(offsets)))
    # How many times the rover changed which way it was turning. This is the
    # number that says "zig-zag" rather than "drifted": a rover following a
    # straight line should reverse its steering a handful of times, not once a
    # step.
    reversals = 0
    previous = 0
    for _, _, rate in commands:
        sign = 0 if abs(rate) < math.radians(0.5) else (1 if rate > 0 else -1)
        if sign and previous and sign != previous:
            reversals += 1
        if sign:
            previous = sign
    travelled = track[-1][0] - track[0][0]
    # The numbers that matter are the *settled* ones. Both mixers converge onto
    # the line -- a follower that could not would be a different bug -- and what
    # the rover was seen doing is wandering about it for the whole route
    # afterwards. So the interesting window is once the initial approach is over,
    # taken here as the last 60% of the run.
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
        # What the follower asked for against what it got, which is where the
        # fault actually lives.
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
        turn = store.get("turn_pwm_points") or TURN_POINTS
        drive = store.get("drive_pwm_points") or DRIVE_POINTS
        return turn, drive, STORE
    except (OSError, ValueError):
        return TURN_POINTS, DRIVE_POINTS, "the values measured on 2026-08-23"


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

    turn_points, drive_points, where = load_points()
    print("chassis from %s" % where)
    print("  turn  %s" % ", ".join("PWM %d=%.1f deg/s" % (p_, v)
                                   for p_, v in turn_points))
    print("  drive %s" % ", ".join("PWM %d=%.2f m/s" % (p_, v)
                                   for p_, v in drive_points))
    print()

    print("what the mixers do with a steering request, at 0.35 m/s:")
    print("  asked      old mixer          new mixer")
    print("  deg/s    left right  diff    left right  diff")
    for dps in (0.5, 1, 2, 5, 10, 15, 25, 45):
        w = math.radians(dps)
        ol, orr = mix_old(0.35, w, turn_points, drive_points)
        nl, nr = mix_new(0.35, w, turn_points, drive_points)
        print("  %5.1f    %4d %4d  %4d    %4d %4d  %4d"
              % (dps, ol, orr, (orr - ol) // 2, nl, nr, (nr - nl) // 2))
    print()

    results = {}
    for name, mixer in (("old", mix_old), ("new", mix_new)):
        results[name] = run(mixer, turn_points, drive_points,
                            metres=args.metres, start_offset=args.offset,
                            lag_s=args.lag, delay_s=args.delay)

    print("following a straight line for %.1f m, starting %.0f cm off it:"
          % (args.metres, args.offset * 100))
    print("            once settled, the rover...")
    print("            wanders +/-   swings over   reverses its steering")
    for name in ("old", "new"):
        r = results[name]
        print("  %-8s  %6.1f cm      %6.1f cm      %5.1f times per metre"
              % (name, r["settled_rms_m"] * 100, r["settled_swing_m"] * 100,
                 r["reversals_per_m"]))
    print()

    # How much of this rests on the one assumption that is not measured -- how much
    # rotational lag the chassis has. If the answer moved with it, the simulation
    # would be describing the assumption rather than the rover.
    print("sensitivity to what is assumed rather than measured:")
    print("  rotational lag           loop delay")
    print("   lag    old    new       delay   old    new")
    delays = (0.0, 0.1, 0.2, 0.3, 0.4)
    lags = (0.05, 0.10, 0.15, 0.25, 0.40)
    for lag, delay in zip(lags, delays):
        a = run(mix_old, turn_points, drive_points, metres=args.metres,
                start_offset=args.offset, lag_s=lag, delay_s=args.delay)
        b = run(mix_new, turn_points, drive_points, metres=args.metres,
                start_offset=args.offset, lag_s=lag, delay_s=args.delay)
        c = run(mix_old, turn_points, drive_points, metres=args.metres,
                start_offset=args.offset, lag_s=args.lag, delay_s=delay)
        d = run(mix_new, turn_points, drive_points, metres=args.metres,
                start_offset=args.offset, lag_s=args.lag, delay_s=delay)
        print("  %.2fs %6.1f %6.1f       %.2fs %6.1f %6.1f"
              % (lag, a["settled_swing_m"] * 100, b["settled_swing_m"] * 100,
                 delay, c["settled_swing_m"] * 100, d["settled_swing_m"] * 100))
    print("  (settled swing in cm; delay is what turns overshoot into oscillation)")
    print()
    print("what the follower asked for, and what the wheels did (first steps):")
    for name in ("old", "new"):
        print("  %s:" % name)
        for asked, diff, got in results[name]["asked_vs_got"]:
            print("    asked %+6.1f deg/s -> differential %+6.1f -> turned %+6.1f"
                  % (asked, diff, got))

    if args.trace:
        # Both drawn to the same ruler -- the old mixer's own wander -- so that the
        # new one is not flattered by being given a bigger one.
        span = max(0.01, results["old"]["settled_swing_m"] * 0.6)
        for name in ("old", "new"):
            track = results[name]["track"]
            settled = track[int(len(track) * 0.4):]
            print("\n%s mixer, the settled part of the route, %.0f cm across:"
                  % (name, span * 200))
            print(trace(settled, span=span))
    return 0


if __name__ == "__main__":
    sys.exit(main())
