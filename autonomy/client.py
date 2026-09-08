"""The only way this component may reach the rover, and it cannot drive it.

**The refusal is the point.** It would be easy enough to write a recorder that
simply never calls `drive`, and that promise would be worth exactly as much as
the next person's care. This client holds a list of the calls it will make and
raises on anything else, so a movement call is not merely unused here -- it is
unavailable, and the test that proves it does so by trying.

Everything on the list is a read. `world_inspect` is not on it, and that is the
distinction worth understanding: it reads nothing, it *acts*, turning the gimbal
and taking a picture. A recorder that triggered looks in order to have something
to record would no longer be watching the rover do its work; it would be part of
the work.

A connection per call rather than one held open. At a poll every few seconds the
cost is nothing, and it removes an entire class of problem -- a half-read socket
after the daemon restarts under a recorder that has been running for an hour.
"""
from __future__ import annotations

import json
import socket
from typing import Any

#: The calls this component may make. Every one is a read: it returns what the
#: rover already knows and changes nothing about what it is doing.
ALLOWED = frozenset({
    "world_state_summary",       # counts, the map session, the generation
    "world_state_entities",      # the things, with their placements
    "world_state_observations",  # the history, newest first, a page at a time
    "world_state_entity",        # one thing in detail
    "world_state_frame",         # a stored picture, as base64
    "world_building",            # whether the rover is looking, and how often
    "nav_status",                # every number the driving loop has
    "battery",                   # volts, for the measured record
})

#: Named so that the error a mistake produces says what is wrong rather than
#: "not allowed". These are the calls somebody would most plausibly reach for.
MOVES = frozenset({
    "drive", "drive_to", "drive_to_map_point", "turn_in_place", "explore",
    "go_to_thing", "stop_driving", "look", "look_at", "center_camera",
    "start_tracking", "track_next", "run_script", "start_script",
})


class Refused(Exception):
    """Raised when something asks this client for a call it may not make."""


class Unreachable(Exception):
    """The daemon did not answer. Not a refusal -- the rover may simply be busy."""


class ReadOnly:
    """A door onto the rover that only opens outwards.

    `host` and `port` are the daemon's loopback address on the rover itself; a
    recorder runs beside it rather than across the network, because the frames it
    copies are large and the network is the rover's wifi.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 8769,
                 timeout: float = 10.0) -> None:
        self.host, self.port, self.timeout = host, port, timeout
        self.calls = 0

    def call(self, name: str, arguments: dict[str, Any] | None = None) -> dict:
        """Make one read, or refuse.

        The refusal happens here, before a socket is opened, so a client pointed
        at a daemon that would happily drive the rover still cannot ask it to.
        """
        if name not in ALLOWED:
            raise Refused(
                f"{name} is not a call this component may make"
                + (" -- it would move the rover" if name in MOVES else "")
                + f". The list is: {', '.join(sorted(ALLOWED))}")
        return self._ask(name, arguments or {})

    def _ask(self, name: str, arguments: dict[str, Any]) -> dict:
        """The socket half, alone, so that a test can replace it.

        Split out rather than mocked around, so that a fake rover goes through
        the real refusal above: a test that bypassed `call` would prove nothing
        about what this client will not do.
        """
        payload = json.dumps({"call": name, "arguments": arguments})
        try:
            with socket.create_connection((self.host, self.port),
                                          self.timeout) as sock:
                sock.settimeout(self.timeout)
                handle = sock.makefile("rwb")
                handle.write(payload.encode() + b"\n")
                handle.flush()
                line = handle.readline()
        except OSError as exc:
            raise Unreachable(f"{name}: {exc}") from exc
        if not line:
            raise Unreachable(f"{name}: the daemon closed without answering")
        self.calls += 1
        try:
            return json.loads(line)
        except ValueError as exc:
            raise Unreachable(f"{name}: unreadable answer") from exc
