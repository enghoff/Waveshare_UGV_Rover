"""Local /frame receiver so look can post pictures to the model's host."""
from __future__ import annotations

import http.server
import json
import struct
import threading
import time
from typing import Any

FRAME_TTL_S = 60.0
MAX_FRAMES = 4
MAX_FRAME_BYTES = 180 * 1024

def _jpeg_size(data: bytes) -> tuple[int | None, int | None]:
    """Width and height out of a JPEG's frame header, without decoding it.

    Only so the log line can say what was posted. Deliberately not PIL: this
    client's whole dependency list is three packages, and reading two big-endian
    shorts out of a marker is not worth a fourth.
    """
    index = 2
    while index + 9 < len(data):
        if data[index] != 0xFF:
            index += 1
            continue
        marker = data[index + 1]
        # SOF0..SOF15, skipping the four that are not start-of-frame markers.
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            height, width = struct.unpack(">HH", data[index + 5:index + 9])
            return width, height
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
            index += 2
            continue
        length = struct.unpack(">H", data[index + 2:index + 4])[0]
        index += 2 + length
    return None, None


class Frames(http.server.ThreadingHTTPServer):
    """`POST /frame` on this machine, because the service that used to serve it is gone.

    The rover's `look` does not send the picture through the conversation. It
    posts the JPEG straight to the model's host and returns nothing but the name
    it was filed under. That road led to MEDIA, then to a desk client; on this
    path the host is the rover itself, loopback, while a conversation is open.

    The contract is the voice service's, unchanged, so the daemon needs no edit:

        POST /frame   body: one JPEG
          -> {"ok": true, "image": "frame-7", "w": 640, "h": 480}

    **Threading here is not about throughput.** The rover posts over one
    kept-open connection, deliberately, and a plain `HTTPServer` handles requests
    one at a time inside `serve_forever` -- so after a single picture it is
    parked inside that connection's handler, blocked on a request line that will
    not arrive until the next `look`. Nothing else can be accepted, and
    `shutdown()` never returns, because the loop it is waiting on is the one that
    is blocked. What that looks like from outside is a conversation that ends
    fine until somebody asks the rover what it can see, after which Ctrl-C hangs
    the terminal.
    """

    # Not reusable, deliberately, and this is the one place where the usual
    # advice is backwards. On Windows SO_REUSEADDR does not mean "reclaim a port
    # left in TIME_WAIT", it means *share*: a second process binds the same port
    # happily and which of the two a given connection reaches is anyone's guess.
    # A leftover client from an earlier run therefore steals the rover's
    # pictures, and the running one is handed a frame name it is not holding.
    # Refusing to start is the better failure, and `main` prints it.
    allow_reuse_address = False
    daemon_threads = True  # inherited, and load-bearing: see above

    def __init__(self, port: int = 8767, host: str = "0.0.0.0") -> None:
        super().__init__((host, port), _FrameHandler)
        self._frames: dict[str, tuple[bytes, float]] = {}
        self._seq = 0
        self._lock = threading.Lock()
        self.posted = 0

    def stash(self, jpeg: bytes) -> str:
        with self._lock:
            now = time.monotonic()
            for name, (_data, at) in list(self._frames.items()):
                if now - at > FRAME_TTL_S:
                    del self._frames[name]
            while len(self._frames) >= MAX_FRAMES:
                del self._frames[min(self._frames, key=lambda n: self._frames[n][1])]
            self._seq += 1
            self.posted += 1
            name = f"frame-{self._seq}"
            self._frames[name] = (jpeg, now)
            return name

    def take(self, name: str) -> bytes | None:
        """The frame under this name, removed. One picture answers one question."""
        with self._lock:
            found = self._frames.pop(name, None)
        return found[0] if found else None

    def serve_in_background(self) -> None:
        threading.Thread(target=self.serve_forever, daemon=True).start()


class _FrameHandler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    # An idle kept-open connection costs a parked thread, and a rover that is
    # restarted a few times over an afternoon leaves one behind each time. On
    # timeout the handler simply closes the connection, and the rover's next
    # picture reconnects -- which its VisionLink already expects, since a stale
    # keep-alive is a retry there rather than a lost frame.
    timeout = 300

    def log_message(self, *_args) -> None:
        pass  # the conversation owns the terminal

    def _reply(self, status: int, payload: dict) -> None:
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:
        if self.path.rstrip("/") not in ("/frame", ""):
            self._reply(404, {"ok": False, "error": "only /frame"})
            return
        length = int(self.headers.get("Content-Length") or 0)
        data = self.rfile.read(length) if length else b""
        # JPEG or PNG. `look` sends the first and `show_map` sends the
        # second, and rejecting the map here was invisible in the only way that
        # matters: the tool still answered, with its caption and its description,
        # and the note saying the picture had not been accepted read as a
        # transport problem rather than as this check.
        if not (data.startswith(b"\xff\xd8") or data.startswith(b"\x89PNG")):
            self._reply(400, {"ok": False, "error": "not a JPEG or a PNG"})
            return
        if len(data) > MAX_FRAME_BYTES:
            self._reply(413, {"ok": False,
                              "error": f"{len(data)} bytes is over the "
                                       f"{MAX_FRAME_BYTES} the model accepts"})
            return
        name = self.server.stash(data)
        width, height = _jpeg_size(data)
        self._reply(200, {"ok": True, "image": name, "w": width, "h": height,
                          "bytes": len(data)})

    def do_GET(self) -> None:
        if self.path.rstrip("/") == "/health":
            self._reply(200, {"ok": True, "frames": self.server.posted})
        else:
            self._reply(404, {"ok": False, "error": "only /frame and /health"})

