"""The recording page's route: it carries a session's calls to the daemon and nothing else.

`POST /recording` reaches the daemon with whatever the page sends, so what
matters is that nothing but the seven session calls can travel down it -- not a
drive, not a stop, not a model tool -- and that the page itself is served.
"""
from __future__ import annotations

import json
import os

import _paths  # noqa: F401 -- puts drive_web and voice_chat on the path
from test_harness import check


def test_the_route_carries_only_a_sessions_calls():
    import drive_recording

    for name in ("drive", "stop_driving", "look_at", "autonomy_start", "power_hold"):
        got = drive_recording.forward("127.0.0.1:1", json.dumps(
            {"call": name, "arguments": {}}).encode())
        check(f"{name} is refused on the recording route", got["ok"], False)
    check("...and so is something that is not JSON",
          drive_recording.forward("127.0.0.1:1", b"nonsense")["ok"], False)
    got = drive_recording.forward("127.0.0.1:1", json.dumps(
        {"call": "recording_status", "arguments": {}}).encode())
    check("a session call goes to the daemon, and a daemon that is not there says so",
          (got["ok"], "no answer" in got.get("error", "")), (False, True))
    check("the page is beside it", os.path.exists(drive_recording.PAGE), True)


TESTS = (test_the_route_carries_only_a_sessions_calls,)
