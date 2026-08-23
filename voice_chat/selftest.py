"""Offline checks for the parts of the voice stack that need no GPU and no mic.

Two halves, deliberately runnable independently, because they have different
dependencies and live on different machines:

    python voice_chat/selftest.py          # client half, needs only numpy
    ssh root@media /opt/voice_chat/.venv/bin/python /opt/voice_chat/selftest.py

Whichever half cannot import its dependencies is reported as skipped rather than
failing, so each machine runs the part that is actually deployed on it.

What is covered is the logic that decides *when* to speak and *what* to hand the
synthesiser -- the places where a bug is silent rather than loud. Tool calls are
covered for the same reason from both ends: the sniffer that keeps a call from
being read out loud, and the dispatch that turns one into a command to the
board. The models themselves are not covered here; they need the card.
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from test_harness import FAIL, PASS, SKIP, check  # noqa: F401 — check is for callers


def main() -> int:
    from test_drive_web import (
        test_a_browser_leaving, test_a_second_click_takes_over,
        test_choosing_a_network, test_finding_the_rover_again,
        test_map_size_for_a_panel, test_one_console_at_a_time,
        test_pictures_are_not_replayed, test_signal_verdict,
        test_stopping_an_unwatched_rover, test_web_console,
    )
    from test_server import (
        test_sentences, test_tool_sniffer, test_trim, test_vision,
    )
    from test_talk import (
        test_connect_errors, test_echo_guard, test_endpointer, test_frames,
        test_indicator, test_move_commentary, test_pointing_the_camera,
        test_prompts, test_rover_client, test_speaker, test_speculation,
        test_talk_session,
    )

    test_sentences()
    test_tool_sniffer()
    test_trim()
    test_vision()
    test_rover_client()
    test_connect_errors()
    test_indicator()
    test_endpointer()
    test_speculation()
    test_prompts()
    test_frames()
    test_speaker()
    test_echo_guard()
    test_pointing_the_camera()
    test_move_commentary()
    test_choosing_a_network()
    test_signal_verdict()
    test_map_size_for_a_panel()
    test_web_console()
    test_stopping_an_unwatched_rover()
    test_a_second_click_takes_over()
    test_finding_the_rover_again()
    test_a_browser_leaving()
    test_one_console_at_a_time()
    test_pictures_are_not_replayed()
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
