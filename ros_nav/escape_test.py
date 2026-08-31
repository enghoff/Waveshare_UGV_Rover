#!/usr/bin/env python3
"""Prove on the hardware that a rover touching something can still get off it.

    python3 ~/ugv/escape_test.py              # measure and report, moves the rover
    python3 ~/ugv/escape_test.py --dry-run    # look at the room, move nothing

**Put an obstacle close behind the rover first** -- a box, a wall, a chair leg,
anything solid within about 15 cm of the back of the chassis -- and point the
rover at open floor. Then run this. It asks the rover to do the three things the
fault was about and prints what each one did.

## What is being tested

Nav2's `Spin`, `DriveOnHeading` and `BackUp` each begin their look-ahead
projection at the pose the rover is standing in, so a rover in contact used to be
refused *every* motion in every direction -- it would not turn, and it would not
drive forward off the thing behind it. `ros_nav/behaviors/` replaces all three
with subclasses that defer to Nav2 except in exactly that state.

So the three answers below are the whole test, and the third matters as much as
the first two:

    turn in place        must succeed. A circular footprint rotated about its
                         own centre sweeps no new ground, so this is always safe
                         and refusing it leaves the rover no way out.
    drive forward        must succeed. This is moving *away* from the obstacle,
                         and open floor ahead is checked before asking.
    reverse              must be refused. This is driving *into* the obstacle,
                         and the escape behaviours are supposed to keep saying no.

A run where all three succeed is as much a failure as one where none do: it means
the safety half is gone. The model this was built against is
`ros_nav/corridor_sim.py`, and `ros_nav/selftest.py` holds the same three
expectations at a desk.

## Why it is a separate script

`selftest.py` proves the arithmetic and the deploy proves the plugins load. Only
a rover with something actually behind it can prove the escape, and no deploy can
put it there. This is the part a person has to do.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys

DAEMON = ("127.0.0.1", 8769)

#: How far to turn, and back again afterwards. Big enough to be unambiguous on a
#: chassis whose standing turns are lifted to a 12 deg/s floor by stiction, small
#: enough to be safe beside an obstacle.
TURN_DEG = 30.0
#: How far to drive away from the obstacle, and how much clear floor to insist on
#: before asking. The margin is the rover's own body length again.
DRIVE_M = 0.35
NEED_CLEAR_M = 0.9
#: A short reverse, which must be refused. Kept under nav_bridge's REVERSE_LIMIT_M
#: so that it stays a `BackUp` rather than becoming a turn-round-and-drive.
REVERSE_M = 0.2


class Daemon:
    def __init__(self):
        self.sock = socket.create_connection(DAEMON, 5)
        self.stream = self.sock.makefile("rwb")

    def call(self, name, **arguments):
        self.stream.write(
            (json.dumps({"call": name, "arguments": arguments}) + "\n").encode())
        self.stream.flush()
        return json.loads(self.stream.readline())

    def close(self):
        try:
            self.stream.close()
            self.sock.close()
        except OSError:
            pass


def describe(rover):
    room = rover.call("describe_surroundings")
    return room.get("clear_ahead_m"), (room.get("surroundings")
                                       or room.get("text") or "")


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--dry-run", action="store_true",
                   help="report the room and move nothing")
    p.add_argument("--turn-deg", type=float, default=TURN_DEG)
    p.add_argument("--drive-m", type=float, default=DRIVE_M)
    args = p.parse_args()

    try:
        rover = Daemon()
    except OSError as error:
        print("cannot reach the rover daemon on %s:%s -- %s" % (*DAEMON, error))
        return 1

    clear, text = describe(rover)
    print()
    print("  the room, as the rover sees it")
    print("    clear ahead: %s m" % ("unknown" if clear is None else round(clear, 2)))
    for line in (text or "").split(". "):
        if line.strip():
            print("      %s" % line.strip().rstrip("."))
    print()

    if clear is None:
        print("  the rover cannot say what is around it, so nothing was driven.")
        print("  Check that the ROS stack is up: ~/ugv/ros_nav/restart.sh")
        rover.close()
        return 1

    behind = [s for s in (text or "").split(". ") if "behind" in s]
    if behind:
        print("  something behind it:")
        for line in behind:
            print("      %s" % line.strip().rstrip("."))
    else:
        print("  **nothing is reported behind the rover.** This test needs an")
        print("  obstacle within about 15 cm of the back of the chassis, or all")
        print("  three answers below are just Nav2 behaving normally and prove")
        print("  nothing about the escape.")
    print()

    if args.dry_run:
        print("  --dry-run, so nothing was driven.")
        rover.close()
        return 0

    if clear < NEED_CLEAR_M:
        print("  only %.2f m clear ahead and this wants %.1f m to drive into."
              % (clear, NEED_CLEAR_M))
        print("  Turn the rover to face open floor, keeping the obstacle behind it.")
        rover.close()
        return 1

    results = {}

    print("  turning %+.0f deg -- must succeed" % args.turn_deg)
    r = rover.call("turn_in_place", angle_deg=args.turn_deg)
    results["turn"] = r
    print("      %s, turned %s deg  %s"
          % (r.get("reason"), r.get("turned_deg"), (r.get("detail") or "")[:90]))

    print("  turning back")
    back = rover.call("turn_in_place", angle_deg=-args.turn_deg)
    print("      %s, turned %s deg" % (back.get("reason"), back.get("turned_deg")))

    print("  driving %.2f m forward, away from it -- must succeed" % args.drive_m)
    r = rover.call("drive", distance_m=args.drive_m)
    results["forward"] = r
    print("      %s, travelled %s m  %s"
          % (r.get("reason"), r.get("travelled_m"), (r.get("detail") or "")[:90]))

    # After driving forward the obstacle is no longer against the rover, so the
    # reverse below is testing ordinary Nav2 again rather than the escape. Say
    # so rather than letting the result be read as more than it is.
    print("  reversing %.2f m -- must be REFUSED while it is still in contact"
          % REVERSE_M)
    r = rover.call("drive", distance_m=-REVERSE_M)
    results["reverse"] = r
    print("      %s, travelled %s m  %s"
          % (r.get("reason"), r.get("travelled_m"), (r.get("detail") or "")[:90]))

    print()
    print("  verdict")
    turned = abs(float(results["turn"].get("turned_deg") or 0.0))
    drove = abs(float(results["forward"].get("travelled_m") or 0.0))
    reversed_m = abs(float(results["reverse"].get("travelled_m") or 0.0))
    ok_turn = turned > 0.5 * abs(args.turn_deg)
    ok_drive = drove > 0.5 * args.drive_m
    print("    turned off the obstacle      %s (%.1f deg)"
          % ("yes" if ok_turn else "NO", turned))
    print("    drove away from it           %s (%.2f m)"
          % ("yes" if ok_drive else "NO", drove))
    print("    still refuses to reverse     %s (%.2f m)"
          % ("yes" if reversed_m < 0.05 else "NO -- it reversed into it",
             reversed_m))
    print()
    if ok_turn and ok_drive and reversed_m < 0.05:
        print("    the escape behaviours are doing both halves of their job.")
    elif not (ok_turn or ok_drive):
        print("    the rover is still frozen. If nothing was reported behind it")
        print("    then it was never in contact and this proves nothing; if")
        print("    something was, check ros_nav.log for the escape warnings.")
    else:
        print("    mixed -- read the three lines above rather than a summary.")
    print()

    rover.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
