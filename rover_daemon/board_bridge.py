#!/usr/bin/env python3
"""Lend the driver board to a second process, without giving up the port.

The ESP32 is on the GPIO UART and only one process can hold that port. The daemon
holds it, because it is what switches the lights, aims the gimbal and reads the
pack voltage, and none of that should have to stop for anything else. But the ROS
2 stack needs the same board for the two things it cannot do without: the wheel
encoders and the gyro, which are its odometry, and the motor commands that Nav2's
controller produces.

So this is a small server inside the daemon that hands both of those out over
loopback. It is not a general remote control -- it binds to 127.0.0.1 only, so
reaching it already means being on the rover, at which point the daemon's own port
offers strictly more. What it adds is a *rate*: the daemon's tools are called
every few seconds by a language model, and an odometry consumer needs the board
fifty times a second.

    {"send": {"T": 1, "L": 0.2, "R": 0.2}}      ->  {"kind": "ack", "ok": true}
                                                <-  {"kind": "motion", ...} x50/s

It is off unless `--board-bridge` is passed, and it is off in the crontab entry
that starts the rover, so a board that is being shared is a board somebody decided
to share.
"""
from __future__ import annotations

import json
import socket
import socketserver
import threading
import time
from typing import Any

# Loopback on purpose -- see the module docstring. Changing this to 0.0.0.0 puts
# the wheels on the LAN with no authentication in front of them.
HOST = "127.0.0.1"
PORT = 8772

# Fifty a second. The board itself only speaks at about 17 Hz, so this is not a
# sampling rate -- it is how promptly a command gets out and how soon after a new
# telemetry line a subscriber hears about it. Polling faster than the source
# costs one dict copy per tick and removes a sixty-millisecond lump of latency
# from a control loop.
TICK_S = 0.02

# How often the full telemetry line goes out rather than just the motion counters.
# The pack voltage and the accelerometer do not need fifty a second, and sending
# them at that rate is most of the bytes on this socket for none of the value.
FULL_EVERY = 25


class _Handler(socketserver.StreamRequestHandler):
    """One subscriber. Reads commands on the way in, streams motion on the way out.

    Both directions run on this one thread, which is why the read side is
    non-blocking: a client that only listens -- a plain odometry consumer, with
    nothing to say -- must not stall the stream waiting for a command it will
    never send.
    """

    def handle(self) -> None:
        bridge: BoardBridge = self.server.bridge          # type: ignore[attr-defined]
        self.connection.settimeout(0.0)
        bridge.note("subscriber from %s" % (self.client_address,))
        pending = b""
        seq = 0
        try:
            while not bridge.stopping:
                # --- anything to say to the board?
                try:
                    chunk = self.connection.recv(4096)
                    if not chunk:
                        break                     # the client hung up
                    pending += chunk
                except (BlockingIOError, socket.timeout):
                    pass
                except OSError:
                    break
                while b"\n" in pending:
                    line, pending = pending.split(b"\n", 1)
                    if line.strip():
                        self._reply(bridge.command(line))

                # --- and what the board has to say back
                snapshot = bridge.snapshot(full=(seq % FULL_EVERY == 0))
                seq += 1
                self._reply(snapshot)
                time.sleep(TICK_S)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            bridge.note("subscriber gone %s" % (self.client_address,))

    def _reply(self, payload: dict[str, Any]) -> None:
        self.connection.sendall(
            json.dumps(payload, separators=(",", ":")).encode() + b"\n")


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class BoardBridge:
    """The server, plus the loop that keeps the board's counters current.

    That loop is the part that is easy to leave out and then spend an afternoon
    on. `board_link` folds telemetry in only when somebody calls `pump()`, and
    with the lidar off nobody does except a slow backstop thread -- so without
    this, a subscriber gets a motion record that updates twice a second and
    odometry that lurches. `pump()` is documented as safe from several threads at
    once, and this is the loop the documentation is talking about.
    """

    def __init__(self, link, host: str = HOST, port: int = PORT,
                 verbose: bool = False) -> None:
        self.link = link
        self.verbose = verbose
        self.stopping = False
        self._lock = threading.Lock()
        self._motion: dict[str, Any] | None = None
        self._telemetry: dict[str, Any] | None = None
        self._telemetry_at = 0.0
        self._sent = 0
        self._refused = 0
        self.server = _Server((host, port), _Handler)
        self.server.bridge = self                        # type: ignore[attr-defined]
        self.address = self.server.server_address
        self._pump = threading.Thread(target=self._pump_forever, daemon=True,
                                      name="board-bridge-pump")
        self._serve = threading.Thread(target=self.server.serve_forever,
                                       daemon=True, name="board-bridge")

    def start(self) -> None:
        self._pump.start()
        self._serve.start()

    def close(self) -> None:
        self.stopping = True
        try:
            self.server.shutdown()
            self.server.server_close()
        except Exception:
            pass

    def describe(self) -> str:
        return "%s:%d" % (self.address[0], self.address[1])

    def note(self, what: str) -> None:
        if self.verbose:
            print("[bridge] %s" % what, flush=True)

    # --- the board ------------------------------------------------------------
    def _pump_forever(self) -> None:
        while not self.stopping:
            try:
                self.link.pump()
                motion = self.link.motion()
                telemetry = None
                # `telemetry()` waits for a fresh line and would spend most of a
                # tick doing it, so it is asked only as often as it is sent -- and
                # the cached answer covers the ticks in between.
                if time.monotonic() - self._telemetry_at > (FULL_EVERY * TICK_S):
                    telemetry = self.link.telemetry()
                with self._lock:
                    self._motion = motion
                    if telemetry is not None:
                        self._telemetry = telemetry
                        self._telemetry_at = time.monotonic()
            except Exception as error:
                self.note("pump failed: %s" % error)
            time.sleep(TICK_S)

    def snapshot(self, full: bool = False) -> dict[str, Any]:
        """What the board has got to, as one record.

        `motion` is raw and stays raw -- the board's own gyro LSB-seconds and its
        own encoder ticks -- for the reason `board_link.motion` gives: turning
        those into radians and metres needs scale factors that are measured
        elsewhere, and guessing them here would produce odometry that looks
        plausible and is quietly wrong.
        """
        with self._lock:
            out: dict[str, Any] = {"kind": "motion", "t": time.monotonic(),
                                   "motion": self._motion}
            if full:
                out["telemetry"] = self._telemetry
                out["telemetry_age"] = (round(time.monotonic() - self._telemetry_at, 2)
                                        if self._telemetry_at else None)
        return out

    def command(self, line: bytes) -> dict[str, Any]:
        """Forward one command to the board. Everything else is refused.

        The refusals are deliberately narrow -- a malformed line, or one that is
        not a JSON object with a `T` in it -- because this is a passthrough and
        guessing what the firmware will accept is how a passthrough starts
        silently dropping the commands that matter.
        """
        try:
            message = json.loads(line)
        except (ValueError, UnicodeDecodeError) as error:
            self._refused += 1
            return {"kind": "ack", "ok": False, "error": "not JSON: %s" % error}
        if not isinstance(message, dict):
            self._refused += 1
            return {"kind": "ack", "ok": False, "error": "not an object"}
        body = message.get("send", message)
        if not isinstance(body, dict) or "T" not in body:
            self._refused += 1
            return {"kind": "ack", "ok": False,
                    "error": "no command in it -- expected {\"send\": {\"T\": ...}}"}
        ok = bool(self.link.send(body))
        self._sent += 1
        return {"kind": "ack", "ok": ok, "T": body.get("T")}

    def stats(self) -> dict[str, Any]:
        return {"sent": self._sent, "refused": self._refused,
                "address": self.describe()}
