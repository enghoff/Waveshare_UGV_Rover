#!/usr/bin/env python3
"""Offline checks for the drive console. No rover and no browser.

    python drive_web/selftest.py
    ssh orin 'cd ~/ugv/drive_web && python3 selftest.py'

This file is the runner. The checks live beside it, one module per part of the
console, and each exports a `TESTS` tuple: the network panel, the session a
browser holds, the frames it is sent, the audio socket, the world-state panel,
and the page itself read as text.
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import _paths  # noqa: F401

from test_audio import TESTS as AUDIO_TESTS
from test_harness import FAIL, PASS, SKIP
from test_network import TESTS as NETWORK_TESTS
from test_page import TESTS as PAGE_TESTS
from test_pictures import TESTS as PICTURE_TESTS
from test_session import TESTS as SESSION_TESTS
from test_world_panel import TESTS as WORLD_TESTS


def main() -> int:
    for test in (*NETWORK_TESTS, *PICTURE_TESTS, *SESSION_TESTS, *AUDIO_TESTS,
                 *WORLD_TESTS, *PAGE_TESTS):
        test()

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
