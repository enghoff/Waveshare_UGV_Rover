#!/usr/bin/env python3
"""Offline checks for the drive console. No rover and no browser.

    python drive_web/selftest.py
    ssh bpi-m4zero 'cd ~/ugv/drive_web && python3 selftest.py'
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import _paths  # noqa: F401

from test_drive_web import (
    test_a_browser_leaving, test_a_second_click_takes_over,
    test_choosing_a_network, test_finding_the_rover_again,
    test_idle_console_waits_for_a_browser, test_map_size_for_a_panel,
    test_one_console_at_a_time, test_pictures_are_not_replayed,
    test_signal_verdict, test_stopping_an_unwatched_rover,
    test_the_audio_socket, test_web_console, test_what_the_browser_heard,
)
from test_harness import FAIL, PASS, SKIP


def main() -> int:
    test_choosing_a_network()
    test_signal_verdict()
    test_map_size_for_a_panel()
    test_web_console()
    test_stopping_an_unwatched_rover()
    test_idle_console_waits_for_a_browser()
    test_a_second_click_takes_over()
    test_finding_the_rover_again()
    test_a_browser_leaving()
    test_one_console_at_a_time()
    test_pictures_are_not_replayed()
    test_the_audio_socket()
    test_what_the_browser_heard()

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
