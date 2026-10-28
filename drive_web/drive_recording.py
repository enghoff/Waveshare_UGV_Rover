"""The recording page: starting and stopping a session for Perception, and annotating it.

A page of its own at `/recording`, not panels on the drive console: what is
pressed here belongs to a session (rover_daemon/rover_recording.py) and to the
person staging changes in front of the rover, and the drive console keeps the
controls that drive it. The page talks to the daemon through `POST /recording`,
which forwards the session's calls and nothing else, and waits for each answer
-- an annotation is a press at a moment, and the page shows what the daemon
made of it.
"""
from __future__ import annotations

import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(HERE, "drive_recording.html")
#: The only calls this route carries.
SESSION_CALLS = ("recording_start", "recording_stop", "recording_status",
                 "recording_annotate", "recording_still", "recording_schedule",
                 "recording_schedule_cancel")
#: Stopping hashes every file the session wrote, which is gigabytes after a long one.
STOP_TIMEOUT_S = 180.0
TIMEOUT_S = 15.0


def forward(address: str, body: bytes) -> dict:
    """One session call, as the page sent it, and the daemon's answer."""
    try:
        request = json.loads(body or b"{}")
    except ValueError:
        return {"ok": False, "error": "not JSON"}
    if not isinstance(request, dict):
        return {"ok": False, "error": "a request is an object"}
    name = request.get("call")
    if name not in SESSION_CALLS:
        return {"ok": False, "error": "this page only carries a session's calls"}
    arguments = request.get("arguments") or {}
    if not isinstance(arguments, dict):
        return {"ok": False, "error": "`arguments` is an object"}
    from rover_tools import RoverClient

    client = RoverClient(address, timeout=STOP_TIMEOUT_S if name == "recording_stop"
                         else TIMEOUT_S)
    try:
        return client.call(name, arguments)
    except Exception as error:
        return {"ok": False, "error": "no answer from the rover daemon: %s" % error}
    finally:
        client.close()
