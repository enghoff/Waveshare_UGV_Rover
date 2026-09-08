"""The two ways this component may reach the rover, and what neither of them can do.

**The refusal is the point.** It would be easy enough to write a recorder that
simply never calls `drive`, and that promise would be worth exactly as much as
the next person's care. A client here holds a list of the calls it will make and
raises on anything else, so a movement call is not merely unused -- it is
unavailable, and the test that proves it does so by trying.

`ReadOnly` is every call that reads. `world_inspect` is not on it, and that is
the distinction worth understanding: it reads nothing, it *acts*, turning the
gimbal and taking a picture. A recorder that triggered looks in order to have
something to record would no longer be watching the rover do its work; it would
be part of the work.

`Acting` is the executive's, and **it still cannot drive the rover**. It adds
three calls and not one of them is a move: ask for permission, hand permission
back, and do one thing *with* permission. Every autonomous action therefore
arrives at the daemon carrying a permit, an episode and an identifier, and is
checked against the run's budgets before anything turns -- see
[rover_daemon/permission.py](permission.py), which is deployed beside this file
so that what the executive expects and what the rover enforces are one set of
rules.

Two calls are missing from `Acting` on purpose. `autonomy_enable` is the human
act that opens a run and clears a stop, so an executive that could make it could
give itself back the authority a person had just taken away -- which is the
whole of what [R-SAFE-11](../docs/requirements/safety.md#r-safe-11) is about.
`autonomy_stop` is the person's stop, which latches; what the executive has
instead is `autonomy_act(stop)`, which stops the wheels without pretending a
person asked.

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
    "nav_grid",                  # the occupancy map, as numbers rather than a picture
    "battery",                   # volts, for the measured record
    "autonomy_status",           # whether the rover may move by itself, and why not
})

#: What an executive may make on top of those. Three calls, none of them a move:
#: the first asks the daemon for permission and renews it, the second gives it
#: back, and the third is how one admitted operation is dispatched under it.
ACTING = frozenset({
    "autonomy_permit",           # give me permission to act, or renew it
    "autonomy_release",          # I have finished; close the run
    "autonomy_act",              # do this one thing, under this permission
})

#: Named so that the error a mistake produces says what is wrong rather than
#: "not allowed". These are the calls somebody would most plausibly reach for.
MOVES = frozenset({
    "drive", "drive_to", "drive_to_map_point", "turn_in_place", "explore",
    "go_to_thing", "stop_driving", "look", "look_at", "center_camera",
    "start_tracking", "track_next", "run_script", "start_script",
    "world_inspect",
})

#: The two calls that belong to the person at the rover, named for the same
#: reason `MOVES` is: so that the refusal says what is wrong rather than "not
#: allowed". Enabling autonomy is how a stop is cleared, and no client here may
#: make it however the code above it is written.
HUMAN = frozenset({"autonomy_enable", "autonomy_stop"})


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

    #: What this kind of client may call. A class attribute rather than the
    #: module constant read directly, so that `Acting` widens it by saying which
    #: three calls it adds -- and so that every subclass still goes through the
    #: one refusal below.
    allowed = ALLOWED

    def __init__(self, host: str = "127.0.0.1", port: int = 8769,
                 timeout: float = 10.0) -> None:
        self.host, self.port, self.timeout = host, port, timeout
        self.calls = 0

    def call(self, name: str, arguments: dict[str, Any] | None = None) -> dict:
        """Make one call, or refuse.

        The refusal happens here, before a socket is opened, so a client pointed
        at a daemon that would happily drive the rover still cannot ask it to.
        """
        if name not in self.allowed:
            raise Refused(
                f"{name} is not a call this component may make"
                + (" -- it would move the rover" if name in MOVES else "")
                + (" -- enabling autonomy is a person's act, and nothing here "
                   "may take back the authority a person removed"
                   if name in HUMAN else "")
                + f". The list is: {', '.join(sorted(self.allowed))}")
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


class Acting(ReadOnly):
    """The executive's door: every read, plus permission, and still no move.

    The three calls it adds are the whole of its authority, and each of them is
    checked on the other side by the daemon rather than here: this class decides
    what may be *asked for*, and `rover_daemon/permission.py` decides what may
    happen. Both halves matter and neither is enough on its own -- an executive
    that could call `drive_to` directly would be one bug away from driving with
    no permit at all, and a daemon that trusted whatever asked would be one
    stolen connection away from the same thing.

    It is a subclass rather than a flag, so that everything written against
    `ReadOnly` -- the recorder, the situation, the deliberation -- goes on
    working unchanged with one of these in its hand, and so that the check that
    a movement call is refused is the same check for both.
    """

    allowed = ALLOWED | ACTING
