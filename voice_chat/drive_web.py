#!/usr/bin/env python3
"""Drive the rover from a browser: the driving tools, in a window that resizes.

This began as a tkinter window, and that window is where everything here was
measured -- five connections, the pacing, the sentences the navigator publishes
mid-move. What it could never do is fit on a screen. Its panels were laid out at
fixed sizes because tk has no notion of reflowing them, so the log and the turns
table sat below the bottom edge of a 1080p display with no scrollbar to reach
them, and widening the window only added empty space to the right of the camera. A
browser solves that in about ten lines of CSS, and solves it properly: the page
scrolls, the columns rewrap as the window narrows, and on a phone it comes out as
one column in the right order.

    python voice_chat/drive_web.py                      # finds the rover, opens a tab
    python voice_chat/drive_web.py --rover rpi.local:8769
    python voice_chat/drive_web.py --bind 0.0.0.0       # ...and let the phone in
    python voice_chat/mock_rover.py --drive             # ...with no rover at all

**This server runs on your desk, not on the rover, and that is the whole answer to
whether the Pi can afford a web console.** It cannot afford one and it is never
asked to. What is on the Pi is `rover_daemon.py`, exactly as before, answering the
same six TCP connections with the same JSON it has always answered; the HTTP, the
event stream and the page are all at this end. The rover cannot tell that the
thing calling `nav_status` three times a second is a browser rather than a desk
program with a window in it, because in the only sense that matters to a 700 MHz
ARMv6 core running SLAM, it is not.

**And the browser gives two things back for free.** It reads JPEG, so the frame
from the camera goes straight into an `<img>` -- which deleted the one dependency
this console used to have, the OpenCV decode that existed solely because tk reads
PNG, GIF and PPM and nothing else, along with the fallback that wrote the frame to
a file and told you where. It also scales pictures, so the map can be drawn at
whatever size the Pi can afford and then fitted to whatever width the panel
happens to have, with `image-rendering: pixelated` -- which on a picture made of
5 cm squares with no antialiasing in it loses nothing at all.

**The page holds no state of its own.** Everything on screen is rendered from one
JSON object this server pushes down a `text/event-stream`, and every button posts
an action back and renders nothing until the state says so. That is not a taste in
architectures: it is the same reason face tracking is polled rather than
remembered. A button that greys itself out because you
pressed it is a button that lies when the rover refuses, and here there can be two
browsers open on the same rover, so a page that believed its own clicks would
disagree with the room.

**The pictures do not travel in that stream.** A map is 40-200 kB of base64 and
the stream carries a fresh state ten times a second, so the map and the camera
frame are kept back as ordinary HTTP resources -- `/map.png`, `/frame.jpg` -- and
the state carries a counter that goes up when a new one arrives. The page changes
the `src` when the counter moves, the browser fetches it once, and everything in
between is a few kilobytes of numbers.

**Closing the tab stops the rover**, which a desktop window gets almost for free
from its close handler and is harder to keep here: a browser tab that goes away
says nothing, and the server outlives it. So the rule is on this side instead --
when the last event stream has been gone for a couple of seconds and a move is
still running, the stop goes out on the connection that carries nothing else. A
reload drops the stream for a fraction of a second and is covered by the grace;
two tabs open means the count never reaches zero. Ctrl-C stops it too, for the
same reason.

Only the standard library, plus [rover_tools.py](rover_tools.py) for the wire and
[console_model.py](console_model.py) for everything both consoles agree about --
which is nearly all of it. The page is [drive_web.html](drive_web.html) beside
this file, read from disk on every request so that editing it needs no restart.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import queue
import socket
import sys
import uuid
import tempfile
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import rover_tools
from console_model import (
    ALARM_WHEN_FALSE, ALARM_WHEN_TRUE, BATTERY_NOTES, BATTERY_POLL_S,
    BATTERY_STALE_S, CAMERA_AUTO_S, CLEAR_ARM_S, Channel, LIGHT_MAX, LOG_LINES,
    LOUD_PHASES, MAP_AUTO_S, MAP_EXTENTS_M, MAP_LEGEND, MAP_SIZES_PX,
    MOVE_TIMEOUT_S, POLL_S, Reply, STATUS_FIELDS, TRACK_POLL_S, TURN_PRESETS_DEG,
    TURN_ROWS, WIFI_POLL_S, WIFI_REJOIN_S, WIFI_SCAN_TIMEOUT_S, move_sentence,
    or_dash, rung, size_for_panel, tap_to_relative, wifi_verdict, worth_logging)

from drive_session import (
    DEFAULT_HTTP_PORT, KEEPALIVE_S, LINK_LOST_S, ORPHAN_GRACE_S,
    RECONNECT_MAX_S, RECONNECT_S, Session, TICK_S,
)
from drive_show import _number, _png_width

PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drive_web.html")

class Handler(BaseHTTPRequestHandler):
    """The page, the stream, the two pictures, and one POST.

    `protocol_version` is HTTP/1.1 so that the event stream is a connection the
    browser keeps rather than one it has to re-open, and every reply therefore has
    to carry an accurate `Content-Length` or a chunked body -- which is why the
    bodies here are always assembled before the headers go out.
    """

    protocol_version = "HTTP/1.1"
    session: Session = None          # type: ignore[assignment]
    verbose = False

    def log_message(self, fmt: str, *args) -> None:
        if Handler.verbose:
            sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

    # --- replies --------------------------------------------------------------
    def _send(self, body: bytes, kind: str, cache: str = "no-store") -> None:
        self.send_response(200)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", cache)
        self.end_headers()
        self.wfile.write(body)

    def _missing(self, why: str) -> None:
        body = why.encode()
        self.send_response(404)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:            # noqa: N802 - http.server's spelling
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            # Read per request rather than held in memory, so that editing the page
            # and pressing reload is the whole edit cycle. It is a local file on the
            # machine running this; there is nothing to save by caching it.
            try:
                with open(PAGE, "rb") as handle:
                    page = handle.read()
            except OSError as error:
                self._missing(f"{PAGE} is missing: {error}")
                return
            self._send(page, "text/html; charset=utf-8")
        elif path == "/map.png":
            if not self.session.map_png:
                self._missing("no map yet")
                return
            # Immutable because the URL carries the generation: a new map is a new
            # URL, and the one already fetched can never change under it.
            self._send(self.session.map_png, "image/png",
                       "public, max-age=31536000, immutable")
        elif path == "/frame.jpg":
            if not self.session.frame_jpeg:
                self._missing("no frame yet")
                return
            self._send(self.session.frame_jpeg, "image/jpeg",
                       "public, max-age=31536000, immutable")
        elif path == "/setup":
            self._send(json.dumps(setup()).encode(),
                       "application/json; charset=utf-8")
        elif path == "/events":
            self._events()
        else:
            self._missing("no such thing here")

    def do_POST(self) -> None:           # noqa: N802 - http.server's spelling
        if self.path.split("?", 1)[0] != "/do":
            self._missing("no such thing here")
            return
        length = int(self.headers.get("Content-Length") or 0)
        try:
            action = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            action = {}
        # Queued, never executed here. Two browsers and a keyboard shortcut can all
        # post at once, and the pump is the only thread allowed to touch the rover's
        # state -- which a single-threaded GUI event loop would have given for
        # nothing.
        if isinstance(action, dict):
            self.session.actions.put(action)
        self._send(b'{"ok":true}', "application/json; charset=utf-8")

    # --- the stream -----------------------------------------------------------
    def _events(self) -> None:
        """One `text/event-stream` per browser: the state when it changes, and the
        transcript lines this browser has not had.

        Chunked rather than length-delimited, because it never ends. Each browser
        holds its own cursor into the log and its own idea of which version of the
        state it has, so a page opened an hour in gets the whole transcript that has
        survived trimming and a page that has been open all along gets one line.
        """
        session = self.session
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.send_header("Transfer-Encoding", "chunked")
        self.end_headers()

        with session.lock:
            session.listeners += 1
        seen_version, cursor = -1, 0
        try:
            while session.running:
                with session.lock:
                    if session.version == seen_version:
                        session.lock.wait(KEEPALIVE_S)
                    state, seen_version = session.published, session.version
                    lines = [line for line in session.log if line["seq"] > cursor]
                    if lines:
                        cursor = lines[-1]["seq"]
                out = ""
                if state:
                    out += f"event: state\ndata: {state}\n\n"
                if lines:
                    out += ("event: log\ndata: "
                            + json.dumps(lines, separators=(",", ":")) + "\n\n")
                self._chunk(out or ": keepalive\n\n")
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass                       # the tab went away, which is not an error
        finally:
            with session.lock:
                session.listeners -= 1
                session.lock.notify_all()

    def _chunk(self, text: str) -> None:
        body = text.encode("utf-8")
        self.wfile.write(b"%x\r\n%s\r\n" % (len(body), body))
        self.wfile.flush()


class Console(ThreadingHTTPServer):
    """The HTTP server, with one thing changed: a browser leaving is not an error.

    `socketserver` prints a full traceback for any exception that reaches it out of
    a handler, and a browser closing a kept-alive connection reaches it as one. On
    Windows it arrives as `ConnectionAbortedError [WinError 10053]` from the read of
    the *next* request line, which is nobody's bug: the page was reloaded, or the tab
    was closed, or this process was stopped, and the connection did what connections
    do. Elsewhere it is `ConnectionResetError` or a `TimeoutError` from the idle
    handler timeout, for the same reasons.

    Left alone it printed twenty lines of traceback per reload into the window
    somebody is watching the rover in, and that is worse than untidy: it teaches
    whoever is watching to scroll past tracebacks, in the one window where a real one
    would appear. So the ordinary disconnects are swallowed and everything else is
    still printed exactly as it was.
    """

    # Not reusable, deliberately, and this is the one place where the usual advice
    # is backwards. On Windows `SO_REUSEADDR` does not mean "reclaim a port left in
    # TIME_WAIT", it means *share*: a second process binds the same port happily and
    # which of the two a given connection reaches is anyone's guess. So the browser
    # is served its page by one console and posts its buttons to the other, which is
    # not a confusing console -- it is two consoles, one of them showing an earlier
    # session's transcript and map while the rover ignores everything you press.
    # `talk.py` found this on the frame server first; the same answer, for the same
    # reason: refuse to start, and say so.
    allow_reuse_address = False

    def handle_error(self, request, client_address) -> None:
        kind = sys.exc_info()[0]
        if kind is not None and issubclass(kind, (ConnectionError, TimeoutError)):
            return
        super().handle_error(request, client_address)


class OnlyOne:
    """An exclusive lock held for as long as this process lives, so that there is
    only ever one drive console on this machine.

    The port guard above catches the same command typed twice. It does not catch the
    same command typed twice with different ports, and that is the worse case rather
    than the safer one: two consoles on two ports are two clients of one rover, each
    polling three times a second and each asking for a map that costs the Pi's single
    core two and a half seconds to draw. Measured with three of them attached, the
    daemon sat at 48% of the core drawing maps for windows nobody was looking at, and
    a rover that is busy drawing maps is a rover that answers slowly when told to
    stop.

    An OS lock rather than a pid file, because the interesting case is the console
    that died without tidying up: a lock is dropped by the kernel when the process
    goes, however it goes, where a file has to be deleted by something still running.
    The pid is written into the file as *content*, outside the locked region, purely
    so the refusal can name what to close.
    """

    #: Locked a long way past any content, so that reading the pid never contends
    #: with the lock itself. Windows locks byte ranges; nothing reads this one.
    REGION = 1 << 20

    def __init__(self, path: str) -> None:
        self.path = path
        self._fd: int | None = None

    def claim(self) -> str:
        """"" if this process now holds it, or a sentence about who does."""
        try:
            fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o644)
        except OSError as error:
            # A desk that will not let us make a lock file is not a reason to
            # refuse to drive a rover.
            print(f"note: cannot use {self.path} to check for another console "
                  f"({error})", file=sys.stderr)
            return ""
        try:
            self._take(fd)
        except OSError:
            held = self._whoever()
            os.close(fd)
            return (f"another drive console is already running on this machine"
                    f"{held}. Two of them are two clients of one rover, each asking "
                    f"a single-core Pi for maps, and the browser cannot tell which "
                    f"one it is talking to. Close that one first.")
        os.ftruncate(fd, 0)
        os.write(fd, f"{os.getpid()}\n".encode())
        self._fd = fd
        return ""

    def _take(self, fd: int) -> None:
        """The lock itself, which is the one part that is not the same on both."""
        if os.name == "nt":
            import msvcrt

            os.lseek(fd, self.REGION, os.SEEK_SET)
            msvcrt.locking(fd, msvcrt.LK_NBLCK, 1)
            os.lseek(fd, 0, os.SEEK_SET)
        else:
            import fcntl

            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def _whoever(self) -> str:
        try:
            with open(self.path) as handle:
                pid = int(handle.read().split()[0])
        except (OSError, ValueError, IndexError):
            return ""
        return f" (process {pid})"

    def release(self) -> None:
        if self._fd is not None:
            os.close(self._fd)          # which drops the lock with it
            self._fd = None


def setup() -> dict[str, Any]:
    """The handful of things the page needs once and never again: the preset turns
    it draws buttons for, what the daemon calls full brightness, and the colour key
    -- which comes from the renderer on the rover's side of the repository rather
    than being written out again in CSS, for the reason it always has. A key that
    has drifted from the picture is worse than no key at all."""
    return {"presets_deg": list(TURN_PRESETS_DEG),
            "light_max": LIGHT_MAX,
            "legend": [list(entry) for entry in MAP_LEGEND]}


def main(argv=None) -> int | str:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--rover", default=None, metavar="HOST[:PORT]",
                        help="the daemon; omit to look for it (see rover_tools.py)")
    parser.add_argument("--half-extent", type=float, default=3.0, metavar="M",
                        help="metres each way shown in the map (default: %(default)s)")
    parser.add_argument("--map-size", type=int, default=480, metavar="PX",
                        help="how big a map to ask for before the panel has a "
                             "width to go on (default: %(default)s)")
    parser.add_argument("--port", type=int, default=DEFAULT_HTTP_PORT,
                        help="where to serve the page (default: %(default)s)")
    parser.add_argument("--bind", default="127.0.0.1", metavar="ADDRESS",
                        help="0.0.0.0 to let other machines on the LAN drive it; "
                             "there is no password on this (default: %(default)s)")
    parser.add_argument("--no-browser", action="store_true",
                        help="do not open a tab")
    parser.add_argument("--verbose", action="store_true",
                        help="log every HTTP request")
    args = parser.parse_args(argv)

    # Before the session, and before the port: a console that is about to refuse to
    # run should not have opened six connections to the rover on its way to finding
    # out.
    alone = OnlyOne(os.path.join(tempfile.gettempdir(), "rover-drive-console.lock"))
    taken = alone.claim()
    if taken:
        return taken

    session = Session(args.rover, args.half_extent, args.map_size)
    Handler.session = session
    Handler.verbose = args.verbose
    try:
        server = Console((args.bind, args.port), Handler)
    except OSError as error:
        alone.release()
        return (f"cannot serve on {args.bind}:{args.port}: {error}. Something else "
                f"is on that port -- another console, or another program.")
    # Every event stream is a thread that blocks until its browser goes away, so
    # they have to be daemons or Ctrl-C would wait for every open tab to close.
    server.daemon_threads = True

    threading.Thread(target=session.run, daemon=True, name="rover-pump").start()
    where = f"http://{'127.0.0.1' if args.bind == '0.0.0.0' else args.bind}:{args.port}/"
    print(f"drive console on {where}")
    if args.bind == "0.0.0.0":
        print(f"    and on http://{_lan_address()}:{args.port}/ from the LAN -- "
              f"anyone who can reach it can drive the rover")
    if not args.no_browser:
        threading.Thread(target=webbrowser.open, args=(where,), daemon=True).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping the rover and shutting down")
    finally:
        session.close()
        server.shutdown()
        alone.release()
    return 0


def _lan_address() -> str:
    """This machine's address on the LAN, for printing. No packet is sent -- a
    connected UDP socket only picks the route -- so this works with nothing
    listening at the far end."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("192.168.1.1", 9))
        return probe.getsockname()[0]
    except OSError:
        return "this machine"
    finally:
        probe.close()


if __name__ == "__main__":
    sys.exit(main())
