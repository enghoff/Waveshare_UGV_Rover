#!/usr/bin/env python3
"""Drive the rover from a browser: the driving tools, in a window that resizes.

This began as a tkinter window, and that window is where everything here was
measured -- five connections, the pacing, the sentences the navigator publishes
mid-move. What it could never do is fit on a screen. Its panels were laid out at
fixed sizes because tk has no notion of reflowing them, so the panels along the
bottom sat below the edge of a 1080p display with no scrollbar to reach them, and
widening the window only added empty space to the right of the camera. A
browser solves that in about ten lines of CSS, and solves it properly: the page
scrolls, the columns rewrap as the window narrows, and on a phone it comes out as
one column in the right order.

    python3 drive_web.py                 # on the rover, from boot; page on :8771
    python drive_web/drive_web.py --no-idle --bind 127.0.0.1 --rover 127.0.0.1:8769
    python voice_chat/mock_rover.py --drive             # ...with no rover at all

Started from boot by [run_drive_web.sh](run_drive_web.sh). The page is
`http://<the rover>:8771/`. A Pi 1 could not afford this and was never asked to;
the Banana Pi M4 Zero can. What the daemon sees is unchanged: the same six TCP
connections, the same JSON. `--idle` (the default) is why a process that lives
from boot is not a client overnight -- it talks to the daemon only while a
browser is open.

**And the browser gives two things back for free.** It reads JPEG, so the frame
from the camera goes straight into an `<img>` -- which deleted the one dependency
this console used to have, the OpenCV decode that existed solely because tk reads
PNG, GIF and PPM and nothing else, along with the fallback that wrote the frame to
a file and told you where. It also scales pictures, so the map can be drawn at
whatever size the rover can afford and then fitted to whatever width the panel
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

Only the standard library, plus [rover_tools.py](../voice_chat/rover_tools.py)
for the wire and [console_model.py](../voice_chat/console_model.py) for the
pacing and the English. The page is [drive_web.html](drive_web.html) beside
this file, read from disk on every request so that editing it needs no restart.
"""

from __future__ import annotations

import argparse
import json
import os
import select
import socket
import ssl
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import _paths  # noqa: F401 — console_model, rover_tools
import wsframe
from console_model import (
    LIGHT_MAX, MAP_LEGEND, MAP_SIZE_PX, Reply, TURN_PRESETS_DEG,
)
from drive_session import (
    KEEPALIVE_S, LINK_LOST_S, ORPHAN_GRACE_S, RECONNECT_MAX_S, RECONNECT_S,
    ROVER_HTTP_PORT, Session,
)
from drive_show import _png_width

PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drive_web.html")

# **The microphone is optional and the console is not.** `omni_bridge` pulls in
# the voice client, which needs `websockets` and numpy, and a desk that has
# neither should still be able to drive the rover -- that is what this console is
# for, and it was true of every version before this one. So the import is allowed
# to fail, the failure is kept, and the page is told why the button is not there
# rather than being given a button that cannot work.
try:
    import omni_bridge
    OMNI_MISSING = ""
except Exception as _error:                       # noqa: BLE001 - reported to the page
    omni_bridge = None
    OMNI_MISSING = f"{type(_error).__name__}: {_error}"

# Where make_cert.sh puts what it makes. Outside ~/ugv because a deploy lands on
# ~/ugv, and a private key a deploy can overwrite -- or carry back off the rover
# into the repository -- is a key in the wrong place.
TLS_DIR = os.path.join(os.path.expanduser("~"), ".ugv", "tls")
TLS_CERT = os.path.join(TLS_DIR, "console.crt")
TLS_KEY = os.path.join(TLS_DIR, "console.key")

# How long a connection has to say whether it is TLS before it is dropped. A
# browser opens connections speculatively and sometimes sends nothing down them,
# and each of those parks a thread inside the peek below until this expires.
HANDSHAKE_S = 10.0

# How much of the rover the kernel may hold on one browser's behalf.
#
# **A live picture is worth nothing late, and TCP will not let it be late alone.**
# Every byte written to a browser is a promise to deliver it, in order, ahead of
# everything written after it. So a link that cannot keep up does not thin the
# stream out, it delays all of it, and whatever the kernel is willing to hold
# becomes the lag. Left alone Linux grows that buffer into the hundreds of
# kilobytes: on 2026-08-26 this rover's radio had begun dropping every packet over
# about 1100 bytes, the console was writing 55 kB/s of state into a link carrying
# 20, and 580 kB stood queued for one browser. The page was drawing a map
# twenty-nine generations old and a photo fifty-seven -- about a minute of rover
# either way -- while its own age readout said the map was drawn 0.8 s ago, because
# that number is taken here as we publish and cannot see the minute that follows.
#
# Sixteen kilobytes is three states: more than any ordinary link will ever notice,
# little enough that one falling behind is found out within an update or two
# instead of half a megabyte later.
STREAM_SNDBUF = 16 * 1024
# How long one write to a browser may take before that browser is taken as gone.
# Only ever reached by a tab that stopped reading altogether -- a slow link still
# moves a 5 kB state in well under a second -- and such a tab used to hold its
# thread inside a write, and its place in the watcher count, for as long as this
# process lived.
STREAM_WRITE_S = 30.0
# How long to hold off after throwing a state away, so that a jammed socket is not
# retried in a tight loop while the rover keeps publishing into it.
DROPPED_PAUSE_S = 0.05

class Handler(BaseHTTPRequestHandler):
    """The page, the stream, the two pictures, and one POST.

    `protocol_version` is HTTP/1.1 so that the event stream is a connection the
    browser keeps rather than one it has to re-open, and every reply therefore has
    to carry an accurate `Content-Length` or a chunked body -- which is why the
    bodies here are always assembled before the headers go out.
    """

    protocol_version = "HTTP/1.1"
    session: Session = None          # type: ignore[assignment]
    omni: Any = None                 # an omni_bridge.Omni, or None
    token: str = ""                  # what /audio wants; see omni_bridge.token
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
        elif path == "/health":
            self._send(json.dumps(health()).encode(),
                       "application/json; charset=utf-8")
        elif path == "/events":
            self._events()
        elif path == "/audio":
            self._audio()
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
        # The microphone is answered here rather than queued, because the queue is
        # the rover's: everything in it is a tool call the pump makes on the
        # daemon's connections, in order, and starting a conversation is neither a
        # tool call nor something to do behind a move that is still running.
        if isinstance(action, dict) and "omni" in action:
            self._omni(action)
            return
        # Queued, never executed here. Two browsers and a keyboard shortcut can all
        # post at once, and the pump is the only thread allowed to touch the rover's
        # state -- which a single-threaded GUI event loop would have given for
        # nothing.
        if isinstance(action, dict):
            self.session.actions.put(action)
        self._send(b'{"ok":true}', "application/json; charset=utf-8")

    # --- the microphone -------------------------------------------------------
    def _omni(self, action: dict) -> None:
        """Turn the conversation on or off. The token is checked here and at
        /audio, because these are two different doors to the same account."""
        if Handler.omni is None:
            self._send(json.dumps({"ok": False, "error":
                                   f"this console has no microphone: {OMNI_MISSING}"}
                                  ).encode(), "application/json; charset=utf-8")
            return
        if not self._allowed(str(action.get("token") or "")):
            self._send(json.dumps({"ok": False, "error": "wrong or missing token"}
                                  ).encode(), "application/json; charset=utf-8")
            return
        if action.get("omni"):
            why = Handler.omni.turn_on()
            body = {"ok": not why, "error": why}
        else:
            Handler.omni.turn_off()
            body = {"ok": True, "error": ""}
        self.session.publish_soon()
        self._send(json.dumps(body).encode(), "application/json; charset=utf-8")

    def _allowed(self, given: str) -> bool:
        """Constant-time, because the alternative leaks the token one character at
        a time to anything patient enough to time the answers."""
        import hmac

        return bool(Handler.token) and hmac.compare_digest(given, Handler.token)

    def _audio(self) -> None:
        """One browser's microphone and speaker, as a WebSocket on this port.

        The handshake is four lines of the standard and the framing is in
        [wsframe.py](wsframe.py); what is here is the loop, and it runs on this
        connection's own thread for as long as the tab is open. Audio arrives as
        binary frames -- PCM16 at MIC_RATE, already gated by the browser's echo
        canceller -- and everything else is a JSON line, of which there is
        currently one: where the browser's playback has actually got to.
        """
        query = self.path.split("?", 1)[1] if "?" in self.path else ""
        given = ""
        for part in query.split("&"):
            if part.startswith("k="):
                from urllib.parse import unquote

                given = unquote(part[2:])
        if Handler.omni is None:
            return self._missing(f"no microphone here: {OMNI_MISSING}")
        if not self._allowed(given):
            body = b"wrong or missing token"
            self.send_response(403)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        key = self.headers.get("Sec-WebSocket-Key", "")
        if "websocket" not in self.headers.get("Upgrade", "").lower() or not key:
            return self._missing("/audio is a WebSocket")

        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", wsframe.accept(key))
        self.end_headers()
        self.wfile.flush()
        self.close_connection = True   # nothing follows this on the connection

        wire = Wire(self.wfile)
        Handler.omni.attach(wire)
        self.session.publish_soon()
        try:
            while wire.alive:
                opcode, data = wsframe.read_message(self.rfile)
                if opcode == wsframe.BINARY:
                    Handler.omni.on_audio(data)
                elif opcode == wsframe.TEXT:
                    self._from_page(data)
                elif opcode == wsframe.PING:
                    wsframe.send(self.wfile, wire.lock, wsframe.PONG, data)
                elif opcode == wsframe.CLOSE:
                    break
        except (OSError, ConnectionError, wsframe.ProtocolError, ValueError):
            pass                       # the tab went away, or spoke nonsense
        finally:
            wire.alive = False
            Handler.omni.detach(wire)
            self.session.publish_soon()

    def _from_page(self, data: bytes) -> None:
        try:
            message = json.loads(data)
        except ValueError:
            return
        if not isinstance(message, dict):
            return
        if message.get("t") == "played":
            Handler.omni.on_played(int(message.get("gen") or 0),
                                   float(message.get("ms") or 0))

    # --- the stream -----------------------------------------------------------
    def _events(self) -> None:
        """One `text/event-stream` per browser: the state, whenever it changes.

        Chunked rather than length-delimited, because it never ends. Each browser
        holds its own idea of which version of the state it has, so a page that has
        just opened is sent the current one and a page that has been open all along
        is sent nothing until something moves. There is no second kind of message:
        the console's own notice line rides in the state like every other panel,
        which is what lets a browser that missed a few states still be correct
        rather than missing a line of history for good.
        """
        session = self.session
        self._bound_the_backlog()
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.send_header("Transfer-Encoding", "chunked")
        self.end_headers()

        with session.lock:
            session.listeners += 1
        seen_version = -1
        try:
            while session.running:
                with session.lock:
                    if session.version == seen_version:
                        session.lock.wait(KEEPALIVE_S)
                    state, version = session.published, session.version
                out = ""
                if not state:
                    # Nothing published yet, or a publish invalidated and being
                    # rebuilt. There is no picture here to be late with.
                    seen_version = version
                elif version != seen_version:
                    if not self._room_for_one_more():
                        # The link is behind, so this state is already history.
                        # Dropping it costs the page one update. Writing it would
                        # cost the page every update after it, because the newest
                        # cannot be delivered until this one has been. seen_version
                        # stays put, so whatever is current when there is room next
                        # is what goes out in its place.
                        time.sleep(DROPPED_PAUSE_S)
                        continue
                    out = f"event: state\ndata: {state}\n\n"
                    seen_version = version
                self._chunk(out or ": keepalive\n\n")
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass                       # the tab went away, which is not an error
        finally:
            with session.lock:
                session.listeners -= 1
                session.lock.notify_all()

    def _bound_the_backlog(self) -> None:
        """Stop the kernel becoming a queue of where the rover used to be.

        See `STREAM_SNDBUF` for what this is defending against and what it cost.
        The write timeout is the other half of the same thought: a tab that stops
        reading altogether used to hold this thread inside a single write, and its
        place in the watcher count, for as long as the console ran.
        """
        try:
            self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF,
                                       STREAM_SNDBUF)
            self.connection.settimeout(STREAM_WRITE_S)
        except OSError:
            pass            # an unusual socket is not a reason to refuse a page

    def _room_for_one_more(self) -> bool:
        """Whether this socket can take a state now rather than owe it later."""
        try:
            return bool(select.select([], [self.connection], [], 0)[1])
        except (OSError, ValueError):
            return True     # let the write find out, and report it as it always did

    def _chunk(self, text: str) -> None:
        body = text.encode("utf-8")
        self.wfile.write(b"%x\r\n%s\r\n" % (len(body), body))
        self.wfile.flush()


class Wire:
    """One browser's audio socket, as the session sees it.

    Two threads write down this socket -- the page's own thread answering pings,
    and the session's thread pushing the model's voice -- so every write goes
    through one lock. Without it the frames interleave, which is not an error
    anybody sees: the browser simply closes the connection, and the microphone
    appears to have stopped working.
    """

    def __init__(self, wfile) -> None:
        self.wfile = wfile
        self.lock = threading.Lock()
        self.alive = True

    def audio(self, pcm: bytes) -> None:
        self._send(wsframe.BINARY, pcm)

    def control(self, payload: dict) -> None:
        self._send(wsframe.TEXT, json.dumps(payload).encode())

    def evict(self, why: str) -> None:
        self.control({"t": "evicted", "why": why})
        self.alive = False

    def _send(self, opcode: int, payload: bytes) -> None:
        if not self.alive:
            return
        try:
            wsframe.send(self.wfile, self.lock, opcode, payload)
        except (OSError, ValueError):
            self.alive = False     # the tab went; the reader will notice too


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

    # On Windows `SO_REUSEADDR` means *share*, not "reclaim TIME_WAIT": a second
    # process binds the same port and which of the two a connection reaches is
    # anyone's guess. On Linux it is the usual reclaim, and without it a reload
    # fails for a minute as EADDRINUSE after the previous process has gone. So
    # Windows refuses and Linux reclaims. OnlyOne still stops a second console
    # on any port.
    allow_reuse_address = os.name != "nt"

    #: An `ssl.SSLContext` once a certificate has been found, None otherwise.
    #: See :func:`tls_context` for why this console speaks TLS at all.
    tls: "ssl.SSLContext | None" = None

    def handle_error(self, request, client_address) -> None:
        kind = sys.exc_info()[0]
        if kind is not None and issubclass(kind, (ConnectionError, TimeoutError,
                                                  ssl.SSLError)):
            return
        super().handle_error(request, client_address)

    def finish_request(self, request, client_address) -> None:
        """One connection, either wrapped in TLS or redirected into it.

        Both schemes arrive on the same port, and which one this is can be read
        off the first byte: a TLS connection opens with a handshake record, which
        is 0x16, and no HTTP request line begins with that. So the byte is peeked
        at without being consumed, and the connection either becomes TLS or is
        sent a redirect and closed.

        **The alternative was breaking every bookmark**, in the way that reads as
        the rover being down rather than as a moved page: a browser that speaks
        HTTP to a TLS socket gets neither a redirect nor an error page, it gets a
        handshake failure, and the tab says the site sent an invalid response.
        Serving both on one port costs the peek below and keeps
        `http://<the rover>:8771/` working for as long as anyone has it written
        down.

        This runs on the connection's own thread -- `ThreadingHTTPServer` spawns
        first and calls this second -- so a peek that blocks holds up nothing but
        the connection waiting on it. The same work in `get_request` would block
        the accept loop, which is the whole server.
        """
        if self.tls is None:
            return super().finish_request(request, client_address)
        try:
            request.settimeout(HANDSHAKE_S)
            first = request.recv(1, socket.MSG_PEEK)
            request.settimeout(None)
            if first != b"\x16":
                return _redirect_to_tls(request, self.server_address[1])
            request = self.tls.wrap_socket(request, server_side=True)
        except (OSError, ssl.SSLError):
            return                     # a connection that never really started
        self.RequestHandlerClass(request, client_address, self)


def _redirect_to_tls(request, port: int) -> None:
    """Answer one plain-HTTP request with 308 Permanent Redirect, to https.

    Deliberately not a `BaseHTTPRequestHandler`: this connection is finished
    either way, and the only header that matters is `Host`, because the address
    the browser typed is the one it has to be sent back to. A phone that reached
    the rover by address and was redirected to its hostname would have to resolve
    a name it may not have; the certificate covers both, so neither is preferred
    here.
    """
    try:
        request.settimeout(HANDSHAKE_S)
        head = b""
        while b"\r\n\r\n" not in head and len(head) < 8192:
            block = request.recv(4096)
            if not block:
                break
            head += block
        lines = head.split(b"\r\n")
        target = b"/"
        if lines and len(lines[0].split(b" ")) >= 2:
            target = lines[0].split(b" ")[1]
        authority = ""
        for line in lines[1:]:
            if line.lower().startswith(b"host:"):
                authority = line.split(b":", 1)[1].strip().decode("latin-1")
                break
        if not authority:
            authority = f"{_lan_address()}:{port}"
        where = f"https://{authority}{target.decode('latin-1')}"
        body = (f"the drive console is at {where} now, because a browser will "
                f"only open a microphone for an https page").encode()
        request.sendall(
            b"HTTP/1.1 308 Permanent Redirect\r\n"
            b"Location: " + where.encode("latin-1") + b"\r\n"
            b"Content-Type: text/plain; charset=utf-8\r\n"
            b"Content-Length: " + str(len(body)).encode() + b"\r\n"
            b"Connection: close\r\n\r\n" + body)
    except OSError:
        pass
    finally:
        try:
            request.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        request.close()


def tls_context(cert: str, key: str) -> "ssl.SSLContext | None":
    """The certificate make_cert.sh wrote, or None and a reason on stderr.

    **A missing certificate is not a refusal to serve.** This console ran over
    plain HTTP for months and everything on it still works that way; what a
    certificate buys is the microphone, because `getUserMedia` is refused outside
    a secure context and no amount of asking changes that. So a board that has
    not run make_cert.sh gets a console with no voice in it, rather than no
    console.
    """
    if not (os.path.exists(cert) and os.path.exists(key)):
        print(f"note: no certificate at {cert}, so this is plain HTTP and no "
              f"browser will open a microphone for it. Run make_cert.sh.",
              file=sys.stderr)
        return None
    try:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(cert, key)
    except (ssl.SSLError, OSError) as error:
        print(f"note: {cert} would not load ({error}), so this is plain HTTP "
              f"and there will be no microphone", file=sys.stderr)
        return None
    return context


class OnlyOne:
    """An exclusive lock held for as long as this process lives, so that there is
    only ever one drive console on this machine.

    The port guard above catches the same command typed twice. It does not catch the
    same command typed twice with different ports, and that is the worse case rather
    than the safer one: two consoles on two ports are two clients of one rover, each
    polling three times a second and each asking for a map that, on the Pi 1, cost
    the single core two and a half seconds to draw. Measured with three of them attached, the
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


def health() -> dict[str, Any]:
    """Whether this process is serving, how many browsers it has, and whether it
    has the rover. Restart scripts wait on this the way oak_depth waits on its
    own `/health` -- the page answering is not enough, because a console that
    has not bound yet is not a console."""
    session = Handler.session
    if session is None:
        return {"ok": False, "watching": 0, "rover": "", "idle": False}
    return {"ok": True, "watching": session.listeners,
            "rover": session.address, "idle": session.idle}


def omni_state() -> dict[str, Any]:
    """What the page needs to draw the microphone button, whether or not there is
    one. `available` false with a reason beats a button that does nothing."""
    if Handler.omni is None:
        return {"available": False, "why": OMNI_MISSING or "not configured",
                "state": "off"}
    status = Handler.omni.status()
    status["available"] = True
    status["why"] = ""
    return status


def setup() -> dict[str, Any]:
    """The handful of things the page needs once and never again: the preset turns
    it draws buttons for, what the daemon calls full brightness, and the colour key
    -- which comes from the renderer on the rover's side of the repository rather
    than being written out again in CSS, for the reason it always has. A key that
    has drifted from the picture is worse than no key at all."""
    return {"presets_deg": list(TURN_PRESETS_DEG),
            "light_max": LIGHT_MAX,
            "legend": [list(entry) for entry in MAP_LEGEND],
            # The audio rates come from the module that talks to the service, so
            # that the page resamples to what is actually wanted rather than to a
            # number written down twice.
            "mic_rate": getattr(omni_bridge, "MIC_RATE", 16000),
            "play_rate": getattr(omni_bridge, "PLAY_RATE", 24000)}


def main(argv=None) -> int | str:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--rover", default="127.0.0.1:8769", metavar="HOST[:PORT]",
                        help="the daemon (default: %(default)s)")
    parser.add_argument("--half-extent", type=float, default=3.0, metavar="M",
                        help="metres each way shown in the map (default: %(default)s)")
    parser.add_argument("--map-size", type=int, default=MAP_SIZE_PX, metavar="PX",
                        help="how big a map to ask the rover for; the browser "
                             "scales it into the panel (default: %(default)s)")
    parser.add_argument("--port", type=int, default=ROVER_HTTP_PORT,
                        help="where to serve the page (default: %(default)s)")
    parser.add_argument("--bind", default="0.0.0.0", metavar="ADDRESS",
                        help="0.0.0.0 lets the LAN in; there is no password on "
                             "this (default: %(default)s)")
    parser.add_argument("--idle", dest="idle", action="store_true", default=True,
                        help="do not talk to the rover until a browser is open "
                             "(default)")
    parser.add_argument("--no-idle", dest="idle", action="store_false",
                        help="talk to the rover even with no browser (mock / desk)")
    parser.add_argument("--no-browser", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--verbose", action="store_true",
                        help="log every HTTP request")
    parser.add_argument("--tls-cert", default=TLS_CERT, metavar="FILE",
                        help="the certificate to serve (default: %(default)s, "
                             "which is what make_cert.sh writes)")
    parser.add_argument("--tls-key", default=TLS_KEY, metavar="FILE",
                        help="its private key (default: %(default)s)")
    parser.add_argument("--no-omni", action="store_true",
                        help="do not offer the microphone at all, even where a "
                             "key and a certificate would allow it")
    parser.add_argument("--no-tls", dest="tls", action="store_false", default=True,
                        help="serve plain HTTP even where a certificate exists; "
                             "the page loads and the microphone does not")
    args = parser.parse_args(argv)

    # Before the session, and before the port: a console that is about to refuse to
    # run should not have opened six connections to the rover on its way to finding
    # out.
    alone = OnlyOne(os.path.join(tempfile.gettempdir(), "rover-drive-console.lock"))
    taken = alone.claim()
    if taken:
        return taken

    context = tls_context(args.tls_cert, args.tls_key) if args.tls else None
    session = Session(args.rover, args.half_extent, args.map_size, idle=args.idle)
    Handler.session = session
    if omni_bridge is not None and not args.no_omni:
        Handler.token = omni_bridge.token()
        Handler.omni = omni_bridge.Omni(
            args.rover, lambda text, err=False: session.say(text + "\n",
                                                            "bad" if err else "quiet"))
        session.omni = omni_state
    elif omni_bridge is None:
        print(f"note: no microphone on this console ({OMNI_MISSING})",
              file=sys.stderr)
    Handler.verbose = args.verbose
    try:
        server = Console((args.bind, args.port), Handler)
    except OSError as error:
        alone.release()
        return (f"cannot serve on {args.bind}:{args.port}: {error}. Something else "
                f"is on that port -- another console, or another program.")
    server.tls = context
    # Every event stream is a thread that blocks until its browser goes away, so
    # they have to be daemons or Ctrl-C would wait for every open tab to close.
    server.daemon_threads = True

    threading.Thread(target=session.run, daemon=True, name="rover-pump").start()
    scheme = "https" if context else "http"
    where = f"{scheme}://{'127.0.0.1' if args.bind == '0.0.0.0' else args.bind}:{args.port}/"
    print(f"drive console on {where}")
    if args.idle:
        print("    idle until a browser opens")
    if args.bind == "0.0.0.0":
        print(f"    and on {scheme}://{_lan_address()}:{args.port}/ from the LAN -- "
              f"anyone who can reach it can drive the rover")
    if context is not None:
        print("    plain http on the same port is redirected here, and the "
              "warning about the certificate is expected")
    if Handler.omni is not None:
        print(f"    microphone: on demand, token in {omni_bridge.TOKEN_PATH}")
        if context is None:
            print("    ...but no certificate, so no browser will open one")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping the rover and shutting down")
    finally:
        if Handler.omni is not None:
            Handler.omni.close()
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
