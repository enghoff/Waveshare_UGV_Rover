"""Offline checks for the current Alibaba realtime voice/session helpers.

These tests need no GPU and make no live Alibaba call. They exercise the parts of
the browser-to-rover-to-cloud path whose failures can otherwise look like a dead
rover or a model that ignored a tool: rover connection/reconnect, prompt/schema
assembly, frame handoff, move commentary and realtime session event handling.
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from test_harness import FAIL, PASS, SKIP, check  # noqa: F401 — check is for callers


def main() -> int:
    from test_talk import (
        test_connect_errors,
        test_frames,
        test_move_commentary,
        test_prompts,
        test_rover_client,
        test_talk_session,
    )

    test_rover_client()
    test_connect_errors()
    test_prompts()
    test_frames()
    test_move_commentary()
    test_talk_session()

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
