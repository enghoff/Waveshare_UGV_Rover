#!/usr/bin/env python3
"""The counters the ROS checks share, and the paths that let them import.

`check` and `section` print as they go and keep the tally, so a module of checks
needs nothing but this. Importing it also puts `lidar_slam/` and this directory
on `sys.path` -- the first for the chassis measurements in `nav_types.py`, the
second for `drive_mixer.py` and `nav_codes.py`, which are the real control law
and the real error codes rather than restatements of them. Both are found from
the repository and from the rover's `~/ugv/ros_nav`, which are different shapes.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for candidate in (os.path.join(HERE, "..", "lidar_slam"),
                  os.path.join(HERE, "..", "..", "lidar_slam")):
    if os.path.isdir(candidate):
        sys.path.insert(0, os.path.abspath(candidate))
        break
if HERE not in sys.path:
    sys.path.insert(0, HERE)

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
