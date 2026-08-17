#!/usr/bin/env python3
"""Measure how far the rover really turns, without asking the thing under test.

Turning is closed on the scan matcher's heading, so if the matcher under-reports
rotation the rover overshoots and every reading agrees that it did not. Checking
that needs a reference the matcher has no part in.

This is that reference. Rotating the rover shifts its entire range-versus-bearing
profile by exactly the angle turned, so the profile captured before a turn and the
one captured after are the same curve at a different offset -- and the offset falls
out of a cross-correlation over the full circle. No search window, nothing
incremental, and no dependence on anything the matcher did along the way.

It drives the rover. Turning on the spot only, never forward, and it refuses to
start without room to turn.

    ssh rpi 'cd ~/ugv/lidar_slam && python3 calibrate_turn.py --angles 45,90,-90'
"""
import argparse
import math
import sys
import time

BINS = 360          # one degree of resolution, which is finer than the sensor's
                    # 0.86 degree spacing and costs nothing here
MIN_OVERLAP = BINS // 4
SETTLE_S = 1.5      # let it stop, and let a couple of clean revolutions arrive


def profile(nav):
    """Range against bearing, as a list with None where nothing came back."""
    return nav.slam.sectors(BINS)


def rotation_between(before, after):
    """Degrees turned, from how far the profile shifted. Counter-clockwise positive.

    Compared on the bins where *both* captures got a return, because a bin that is
    unknown in one of them says nothing about the shift and averaging a missing
    value in as zero would drag every candidate towards whichever shift hides the
    most gaps.
    """
    import numpy as np

    a = np.array([np.nan if v is None else v for v in before], dtype=np.float64)
    b = np.array([np.nan if v is None else v for v in after], dtype=np.float64)

    best_shift, best_cost = None, None
    for shift in range(BINS):
        diff = np.abs(a - np.roll(b, -shift))
        n = int(np.count_nonzero(~np.isnan(diff)))
        if n < MIN_OVERLAP:
            continue
        cost = float(np.nanmean(diff))
        if best_cost is None or cost < best_cost:
            best_cost, best_shift = cost, shift
    if best_shift is None:
        return None, None

    # after[i] holds what before[i + shift] held, and a bearing that has *decreased*
    # is the rover having turned counter-clockwise, so the turn is minus the shift.
    degrees = -best_shift * (360.0 / BINS)
    if degrees <= -180.0:
        degrees += 360.0
    return degrees, best_cost


def burst(link, pwm, seconds):
    """Fixed PWM for a fixed time, then stop. No feedback of any kind -- this is the
    open-loop turn being characterised, not a controlled one.

    Left track back and right track forward is counter-clockwise, which matches
    drive_gamepad.py's left = throttle + steer with a positive steer turning right.
    """
    from navigator import CMD_HEARTBEAT, CMD_PWM, HEARTBEAT_MS
    link.send({"T": CMD_HEARTBEAT, "cmd": HEARTBEAT_MS})
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        link.send({"T": CMD_PWM, "L": -pwm, "R": pwm})
        time.sleep(0.08)          # inside the 500 ms heartbeat, and cheap
    for _ in range(3):
        link.send({"T": CMD_PWM, "L": 0, "R": 0})


def characterise(nav, link, pwm, durations):
    """Degrees per second and coast at one PWM, from bursts of differing length.

    Angle turned is rate * duration + coast, so a straight line through two or more
    burst lengths separates the two. The coast is the part a fixed-time turn cannot
    control and has to subtract up front.
    """
    print(f"\nopen-loop bursts at PWM {pwm}")
    print(f"{'seconds':>8} {'measured':>9} {'fit':>7}")
    print("-" * 27)
    points = []
    for seconds in durations:
        before = profile(nav)
        burst(link, pwm, seconds)
        time.sleep(SETTLE_S)
        after = profile(nav)
        measured, fit = rotation_between(before, after)
        if measured is None:
            print(f"{seconds:8.2f}   no overlap to measure")
            continue
        # Bursts are always counter-clockwise here, so unwrap onto 0..360 rather
        # than +/-180: a 200 degree spin must not read as -160.
        if measured < -10.0:
            measured += 360.0
        print(f"{seconds:8.2f} {measured:9.1f} {fit:7.3f}")
        points.append((seconds, measured))

    if len(points) < 2:
        print("not enough measurements to fit a rate")
        return None
    n = len(points)
    sx = sum(t for t, _ in points)
    sy = sum(a for _, a in points)
    sxx = sum(t * t for t, _ in points)
    sxy = sum(t * a for t, a in points)
    denom = n * sxx - sx * sx
    if abs(denom) < 1e-9:
        return None
    rate = (n * sxy - sx * sy) / denom
    coast = (sy - rate * sx) / n
    print(f"\n  rate  {rate:6.1f} deg/s at PWM {pwm}")
    print(f"  coast {coast:6.1f} deg carried after the power comes off")
    print(f"  -> for A degrees: hold for (A - {coast:.1f}) / {rate:.1f} seconds")
    return rate, coast


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--angles", default="45,90,-90",
                    help="comma-separated degrees to try (default: %(default)s)")
    ap.add_argument("--deadreckon", metavar="PWM", type=int, default=None,
                    help="instead, characterise open-loop turning at this PWM")
    ap.add_argument("--durations", default="0.4,0.8,1.2,1.6",
                    help="burst lengths in seconds for --deadreckon")
    ap.add_argument("--serial", default="/dev/ttyAMA0")
    ap.add_argument("--lidar", default=None)
    ap.add_argument("--clearance", type=float, default=0.35,
                    help="metres that must be free all round before it will turn")
    args = ap.parse_args(argv)

    sys.path.insert(0, "/home/admin/ugv")
    from rover_daemon import SerialLink
    from navigator import Navigator

    link = SerialLink(args.serial)
    nav = Navigator(link, args.lidar)
    nav.start()
    print("waiting for the lidar...")
    for _ in range(100):
        if nav.lidar_ok():
            break
        time.sleep(0.2)
    if not nav.lidar_ok():
        nav.close()
        return "the lidar never reported; is the rover's power switch on?"

    time.sleep(SETTLE_S)
    near = nav._nearest_recent()
    print(f"nearest thing: {near:.2f} m" if near else "nothing in range")
    if near is not None and near < args.clearance:
        nav.close()
        return (f"only {near:.2f} m of room and {args.clearance:.2f} is wanted -- "
                f"move the rover somewhere clearer, or lower --clearance knowing "
                f"the rover sweeps its corners as it turns")

    if args.deadreckon:
        try:
            characterise(nav, link,
                         args.deadreckon,
                         [float(t) for t in args.durations.split(",")])
        finally:
            nav.stop()
            nav.close()
            link.close()
        return 0

    print(f"\n{'asked':>8} {'matcher':>9} {'measured':>9} {'ratio':>7}  {'fit':>6} "
          f"{'secs':>6}")
    print("-" * 54)
    results = []
    try:
        for text in args.angles.split(","):
            angle = float(text)
            before = profile(nav)
            mark = nav._heading_accum

            began = time.monotonic()
            outcome = nav.turn_in_place(angle)
            elapsed = time.monotonic() - began
            time.sleep(SETTLE_S)

            reported = math.degrees(nav._heading_accum - mark)
            after = profile(nav)
            measured, fit = rotation_between(before, after)

            if measured is None:
                # Say why the *move* ended as well. Reporting only that the
                # measurement failed hides the far more useful fact that the turn
                # was abandoned, which is what happened the first time this ran.
                print(f"{angle:8.0f} {reported:9.1f}   no overlap to measure  "
                      f"[{outcome.reason}: {outcome.detail or '-'}]")
                continue
            # Unwrap onto the same turn as the request, so a measured -170 against a
            # requested +190 is not read as a wild miss.
            while measured - angle > 180.0:
                measured -= 360.0
            while measured - angle < -180.0:
                measured += 360.0
            ratio = measured / angle if angle else float("nan")
            print(f"{angle:8.0f} {reported:9.1f} {measured:9.1f} {ratio:7.2f}  "
                  f"{fit:6.3f} {elapsed:6.1f}"
                  + ("" if outcome.reason == "arrived"
                     else f"   [{outcome.reason}: {outcome.detail or '-'}]"))
            results.append((angle, reported, measured, ratio))
    finally:
        nav.stop()
        nav.close()
        link.close()

    if results:
        ratios = [r for _, _, _, r in results if math.isfinite(r)]
        mean = sum(ratios) / len(ratios)
        print(f"\nmean ratio of real turn to requested: {mean:.2f}")
        if abs(mean - 1.0) < 0.10:
            print("within 10%, so the turn is honest and needs no correction")
        else:
            over = (mean - 1.0) * 100.0
            print(f"the rover turns {abs(over):.0f}% "
                  f"{'further than' if over > 0 else 'short of'} it is asked")
            # Reported against measured is the diagnosis. If the matcher agreed with
            # the reference the fault is downstream of it -- coasting, or stopping
            # late -- and if it did not, the matcher is under-reading and no amount
            # of controller tuning will fix it.
            agree = [abs(rep - meas) for _, rep, meas, _ in results]
            print(f"matcher against reference, worst disagreement: "
                  f"{max(agree):.1f} degrees")
            if max(agree) > 10.0:
                print("  -> the scan matcher is the problem, not the controller")
            else:
                print("  -> the matcher is right, so this is coasting after the stop")
    return 0


if __name__ == "__main__":
    sys.exit(main())
