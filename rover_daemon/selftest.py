"""Offline checks for the rover daemon: no board, no camera, no detector.

What is covered is the part where a bug is silent rather than loud -- argument
coercion, the limits the gimbal is held to, and the fact that every schema the
model is shown corresponds to something that will actually run. A tool whose
name does not match its handler fails as "no such tool" out loud, in the middle
of a conversation, which is a poor place to find out.

    python3 selftest.py                  # on the rover, where everything is flat
    python rover_daemon/selftest.py      # in the repo

The hardware paths are not covered and cannot be: they need the rover.

This file is the runner. The checks live beside it, one module per area of the
daemon, and each exports a `TESTS` tuple: the board and its battery, the gimbal
camera, the map, the radios, the semantic world state, scripting, and the tool
surface itself. `test_fakes.py` holds the driver board they all stand up and the
`sys.path` dance that lets them import the daemon from either layout.
"""
from __future__ import annotations

import sys

from test_fakes import HERE  # noqa: F401  -- importing it sets sys.path up

from test_aiming import (
    test_aiming_through_a_missed_frame, test_one_move_puts_a_face_in_the_middle,
    test_the_approach_to_a_face_never_turns_back,
    test_the_camera_settles_ahead_instead_of_sweeping,
)
from test_api import TESTS as API_TESTS
from test_board import TESTS as BOARD_TESTS
from test_camera import TESTS as CAMERA_TESTS
from test_harness import FAIL, PASS, SKIP
from test_map import TESTS as MAP_TESTS
from test_ros_nav import TESTS as ROS_NAV_TESTS
from test_scripting import TESTS as SCRIPTING_TESTS
from test_wifi import TESTS as WIFI_TESTS
from test_world import TESTS as WORLD_TESTS

AIMING_TESTS = (
    test_aiming_through_a_missed_frame,
    test_one_move_puts_a_face_in_the_middle,
    test_the_approach_to_a_face_never_turns_back,
    test_the_camera_settles_ahead_instead_of_sweeping,
)


def main():
    for test in (*BOARD_TESTS, *API_TESTS, *CAMERA_TESTS, *MAP_TESTS,
                 *WIFI_TESTS, *WORLD_TESTS, *SCRIPTING_TESTS,
                 *AIMING_TESTS, *ROS_NAV_TESTS):
        try:
            test()
        except Exception as exc:
            FAIL.append(f"{test.__name__} raised {type(exc).__name__}: {exc}")

    for name in PASS:
        print(f"  ok   {name}")
    for name in SKIP:
        print(f"  skip {name}")
    for name in FAIL:
        print(f"  FAIL {name}")
    print(f"\n{len(PASS)} passed, {len(FAIL)} failed, {len(SKIP)} skipped")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
