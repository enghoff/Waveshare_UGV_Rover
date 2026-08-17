#!/usr/bin/env python3
"""Exercise the whole driving stack against the real lidar without moving anything.

The link is a stub that records commands instead of sending them, so the control
loop, the clearance checks, the heading choice and the PWM arithmetic all run on
live scans while the rover stays exactly where it is. That is the only honest way
to test this indoors on a desk, and it is worth doing before the first real move:
everything except the motors is under test here.

    ssh rpi 'cd ~/ugv/lidar_slam && python3 dryrun.py'
"""
import argparse
import sys
import time

import navigator
from navigator import Navigator


class FakeLink:
    """Looks like the daemon's SerialLink, reaches nothing."""

    def __init__(self):
        self.sent = []

    def send(self, command):
        self.sent.append((round(time.monotonic(), 3), command))
        return True

    def describe(self):
        return "a stub that goes nowhere"

    def close(self):
        pass

    def pwm_pairs(self):
        return [(c["L"], c["R"]) for _, c in self.sent if c.get("T") == navigator.CMD_PWM]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--lidar", default="/dev/ttyACM0")
    ap.add_argument("--settle", type=float, default=4.0,
                    help="seconds of mapping before asking it to drive")
    ap.add_argument("--map", default=None, metavar="FILE.png")
    args = ap.parse_args(argv)

    link = FakeLink()
    events = []
    nav = Navigator(link, args.lidar,
                    on_drive_start=lambda: events.append("tracking paused"),
                    on_drive_end=lambda: events.append("tracking resumed"))
    nav.start()
    print(f"settling for {args.settle:.0f}s to build a map...")
    time.sleep(args.settle)

    st = nav.status()
    print(f"  {st['scans']} scans, {st['dropped_scans']} dropped, "
          f"match {st['match_score']}")
    if st["scans"] < 10:
        nav.close()
        return ("the lidar produced almost nothing -- if the port is live but silent, "
                "the rover's power switch is off")

    print("\n--- what it can see ---")
    described = nav.describe()
    print(described["text"])
    print(f"  walls {len(described['walls'])}, objects {len(described['objects'])}, "
          f"gaps {len(described['gaps'])}")

    print("\n--- clearance by steering angle ---")
    import math
    for deg in range(-40, 41, 10):
        curvature = 2.0 * math.sin(math.radians(deg)) / navigator.LOOKAHEAD_M
        print(f"  {deg:+4d} deg  {nav._headroom(curvature):5.2f} m")
    chosen, clear = nav._choose_heading(0.0)
    print(f"  asked for straight, chose {chosen:+.0f} deg with {clear:.2f} m")

    print("\n--- a 1 m forward request (motors disconnected) ---")
    before = len(link.sent)
    result = nav.drive(distance_m=1.0, speed_ms=0.20)
    print(f"  outcome: {result.asdict()}")
    print(f"  lifecycle: {events}")
    pwm = link.pwm_pairs()
    print(f"  {len(link.sent) - before} commands sent, {len(pwm)} of them PWM")
    if pwm:
        moving = [p for p in pwm if p != (0, 0)]
        print(f"  first {pwm[0]}, peak {max(pwm, key=lambda p: abs(p[0]) + abs(p[1]))}, "
              f"last {pwm[-1]}, {len(moving)} non-zero")
        if moving:
            lo = min(min(abs(a), abs(b)) for a, b in moving if a or b)
            print(f"  smallest non-zero magnitude {lo} "
                  f"(must be >= {navigator.MIN_PWM}, below that they only buzz)")
    heartbeat = [c for _, c in link.sent if c.get("T") == navigator.CMD_HEARTBEAT]
    print(f"  heartbeat set {len(heartbeat)}x: {heartbeat[:1]}")
    print(f"  ends stopped: {pwm[-1] == (0, 0) if pwm else 'no PWM at all'}")

    print("\n--- a turn request ---")
    before = len(link.pwm_pairs())
    result = nav.turn_in_place(90)
    print(f"  outcome: {result.asdict()}")
    turn_pwm = link.pwm_pairs()[before:]
    spinning = [p for p in turn_pwm if p != (0, 0)]
    print(f"  {len(turn_pwm)} PWM commands, {len(spinning)} of them spinning")
    if spinning:
        # A counter-clockwise turn drives the left track back and the right forward,
        # so L must be negative and R positive. A pair with the same sign would be
        # the steering convention inverted, which is the one bug here that cannot be
        # caught by reading the code.
        left, right = spinning[0]
        print(f"  first spinning pair {spinning[0]}  "
              f"({'ccw, correct for +90' if left < 0 < right else 'WRONG SENSE'})")

    print("\n--- latched stop ---")
    print(f"  {nav.stop(latch=True)}")
    blocked = nav.drive(distance_m=0.5)
    print(f"  drive while latched: {blocked.asdict()}")
    print(f"  cleared: {nav.clear_estop()}")

    print("\n--- the map as a picture ---")
    png, caption = nav.map_png()
    print(f"  {len(png)} bytes of PNG, header {png[:8]!r}")
    print(f"  caption: {caption[:120]}...")
    if args.map:
        with open(args.map, "wb") as f:
            f.write(png)
        print(f"  wrote {args.map}")

    final = nav.status()
    print(f"\nfinal: {final['scans']} scans, {final['dropped_scans']} dropped, "
          f"pose {final['pose']}, trusted {final['position_trusted']}")
    nav.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
