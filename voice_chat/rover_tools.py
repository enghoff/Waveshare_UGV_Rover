"""The rover's tools, as seen by a chat client: a thin line to the daemon.

Nothing here knows what the rover can do. It asks. `rover_daemon.py` owns the
board, the camera and the tracking loop, and it is the only thing that should be
describing them -- so this fetches the schemas over the wire with `list_tools`
and hands them straight to the voice service, which puts them in the prompt.
Adding a tool is then a change to the daemon alone: no client is redeployed, and
there is no second copy of a schema to drift out of step with the code that
honours it.

    rover = RoverClient("127.0.0.1:8769")   # on the rover itself
    rover = RoverClient("rpi.local:8769")  # from a desk on the same LAN

[talk.py](talk.py) uses this across the LAN, from whichever desk has the
microphone. The loopback default is for anything that ends up running on the
rover itself.

Every call answers with a JSON object, and a failure is an answer too --
`{"ok": false, "error": ...}` rather than an exception. That shape is not for
this file's convenience: the result goes into the model's context verbatim and
is paraphrased out loud, so it has to read as an explanation rather than a
traceback.
"""

from __future__ import annotations

import json
import socket
import threading
from typing import Any

DEFAULT_ADDRESS = "127.0.0.1:8769"  # the daemon, on the rover itself
DEFAULT_PORT = 8769

# Where to look for the rover, in order, when nobody has named it.
#
# The name first, because it is the only one of the three that is always right.
# The rover has two addresses and which of them exists depends on whether it is
# plugged in -- `eth0` is primary at route metric 100 when docked, and a rover
# that has driven off has no `eth0` at all -- while `rpi.local` resolves to
# whichever interface is actually up. Defaulting to the wired address was a real
# bug: the daemon was serving all nine tools on wlan0 while the client reported
# no rover, and the model spent the conversation insisting it had no lights.
#
# The addresses stay as fallbacks for a LAN with no mDNS, and they are second
# because a *failed* name lookup is slow -- measured at 7.3s here, against
# milliseconds for an address that simply refuses. A name that works costs
# ~150ms. See docs/hosts.md.
DEFAULT_CANDIDATES = ("rpi.local", "192.168.1.47", "192.168.1.4")

# Long enough for the slowest tool. `count_faces` with the camera cold has to
# start v4l2-ctl and wait for its first buffer, which is seconds; everything
# else is a JSON line down a UART. The voice service has its own, shorter
# patience -- see VOICE_TOOL_TIMEOUT -- and that is the one a user notices.
TIMEOUT_S = 12.0
CONNECT_TIMEOUT_S = 3.0
# Shorter, because this one is paid per candidate before anybody has spoken. An
# address on this LAN either answers in milliseconds or is not there.
DISCOVER_TIMEOUT_S = 1.5

def discover(candidates=DEFAULT_CANDIDATES, port: int = DEFAULT_PORT):
    """The first candidate with a daemon answering on it, or None.

    Tried in order rather than in parallel: the list is short, a refusal on this
    LAN is immediate, and an ordered answer is easier to explain to whoever is
    wondering which address their rover came up on.
    """
    for candidate in candidates:
        address = candidate if ":" in candidate else f"{candidate}:{port}"
        client = RoverClient(address)
        client._connect_timeout = DISCOVER_TIMEOUT_S
        if client.probe():
            client._connect_timeout = CONNECT_TIMEOUT_S
            return client
        client.close()
    return None


class RoverClient:
    """One connection to the daemon, remade if it goes.

    Serialised with a lock rather than pipelined. A tool call is a physical act
    on a rover -- there is no throughput to win by having two in flight, and
    ordering matters more than speed when both of them move the camera.
    """

    def __init__(self, address: str = DEFAULT_ADDRESS, timeout: float = TIMEOUT_S) -> None:
        host, _, port = address.partition(":")
        self.host = host
        self.port = int(port) if port else DEFAULT_PORT
        self.timeout = timeout
        self._connect_timeout = CONNECT_TIMEOUT_S
        self._sock: socket.socket | None = None
        self._file = None
        self._lock = threading.Lock()

    def describe(self) -> str:
        return f"{self.host}:{self.port}"

    def _connect(self) -> None:
        sock = socket.create_connection((self.host, self.port), self._connect_timeout)
        sock.settimeout(self.timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock = sock
        self._file = sock.makefile("rwb")

    def _drop(self) -> None:
        for handle in (self._file, self._sock):
            try:
                if handle is not None:
                    handle.close()
            except OSError:
                pass
        self._sock = self._file = None

    def _exchange(self, request: dict[str, Any]) -> dict[str, Any]:
        """One request, one reply. Raises only if the daemon cannot be reached."""
        line = json.dumps(request).encode() + b"\n"
        # Two attempts, because the first may be spent discovering that a
        # connection kept open since the last question has since been closed --
        # by a daemon restart, most often. That costs a reconnect, not a tool.
        #
        # Which is all the second attempt is for, so it is spent only where there
        # was such a connection to discard, and never after a timeout. A timeout is
        # not a connection that has gone: it is a daemon that has the request and is
        # still working on it, and sending it again has the rover do the thing
        # twice. One console scan was costing two -- a scan runs ~15 s on the Pi
        # against the 12 s above -- and the same retry on a move would have driven
        # twice.
        for attempt in (1, 2):
            reused = self._sock is not None
            try:
                if self._sock is None:
                    self._connect()
                self._file.write(line)
                self._file.flush()
                reply = self._file.readline()
                if not reply:
                    raise ConnectionError("the rover daemon closed the connection")
                return json.loads(reply)
            except (OSError, ValueError) as error:
                self._drop()
                if (attempt == 2 or not reused
                        or isinstance(error, (socket.timeout, TimeoutError))):
                    raise ConnectionError(
                        f"no answer from the rover daemon at {self.describe()}: {error}")
        raise ConnectionError("unreachable")

    def probe(self) -> bool:
        """Is the daemon there? Cheap, and it is what decides whether to offer tools."""
        try:
            with self._lock:
                return bool(self._exchange({"call": "list_tools"}).get("ok"))
        except ConnectionError:
            return False

    def tools(self) -> list[dict[str, Any]]:
        """The schemas this rover is currently offering, straight from the daemon."""
        with self._lock:
            reply = self._exchange({"call": "list_tools"})
        tools = reply.get("tools")
        return tools if isinstance(tools, list) else []

    def local_address(self) -> str | None:
        """This machine's address, as the rover sees it. None if not connected.

        Taken from the socket rather than from `gethostbyname` or a guess,
        because a desk has several addresses and only one of them is on the way
        to the rover -- and which one that is changes with the rover, which
        answers on eth0 while docked and on wlan0 once it has driven off. The
        kernel already chose the right interface to make this connection; this
        just reads back what it chose.

        Used to tell the daemon where to post pictures. See `set_vision` in
        [rover_daemon.py](../rover_daemon/rover_daemon.py).
        """
        with self._lock:
            try:
                if self._sock is None:
                    self._connect()
                return self._sock.getsockname()[0]
            except OSError:
                return None

    def call(self, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
        """Perform one tool call. Never raises -- a failure is a result."""
        try:
            with self._lock:
                return self._exchange({"call": name, "arguments": arguments})
        except ConnectionError as error:
            return {"ok": False, "error": str(error)}

    def close(self) -> None:
        with self._lock:
            self._drop()
