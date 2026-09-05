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

This file is the runner. The checks live beside it, one module per part of the
stack, and each exports a `TESTS` tuple: the drive model, odometry and the IMU,
the lidar scan, the two bridges, the configuration Nav2 is given, where the rover
decides to go, how it follows a route once it has decided, and how it keeps its
map between sessions and finds itself on it again. `test_harness.py` holds the
tally they share and the `sys.path` setup they all need.
"""
import sys

import test_harness
from test_bridge import TESTS as BRIDGE_TESTS
from test_chassis import TESTS as CHASSIS_TESTS
from test_config import TESTS as CONFIG_TESTS
from test_control import TESTS as CONTROL_TESTS
from test_odometry import TESTS as ODOMETRY_TESTS
from test_planning import TESTS as PLANNING_TESTS
from test_refit import TESTS as REFIT_TESTS
from test_scan import TESTS as SCAN_TESTS


def main():
    for test in (*CHASSIS_TESTS, *ODOMETRY_TESTS, *SCAN_TESTS, *BRIDGE_TESTS,
                 *CONFIG_TESTS, *PLANNING_TESTS, *CONTROL_TESTS, *REFIT_TESTS):
        test()
    print("\n%d passed, %d failed" % (test_harness.PASSED, test_harness.FAILED))
    return 1 if test_harness.FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
