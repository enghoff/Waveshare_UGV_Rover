"""Face detection as a service, on MEDIA. JPEG in, boxes out.

The rover no longer needs this: a Banana Pi M4 Zero runs the same YuNet in
process at 146 ms a frame. The service stays because a Pi 1 could not -- an
ARM1176 at 700 MHz with **no NEON**, conv-shaped GEMMs at 0.039 GFLOP/s, YuNet
about a second even before the JPEG was decoded (93 ms at 640x480). It costs
5.8 ms here. `--service` on the tracking loop is how a host that cannot detect
for itself still can.

**This runs on the CPU, deliberately, and that is the whole point.** MEDIA's card
is an 8 GB 3070 that Windows has already taken a share of, and the three model
services in /opt -- grounding-dino, qwen3-vl, voice-chat -- share what is left and
are mutually exclusive because of it; ~/switch_service.sh exists to stop one
before starting another. With voice-chat up the card reads 6656 of 8192 MiB used.
A GPU face detector would join that queue and the rover could then only see while
nobody was talking to it. YuNet is a 230 kB CNN that a 5700G swallows without
noticing, so on the CPU it is simply always available, concurrent with whatever
owns the card. **Do not add this to switch_service.sh**, and do not be tempted to
answer this with grounding-dino instead: it would work, and it would put face
tracking back into the interlock.

For the same reason it is enabled at boot. Only qwen3-vl is, among the others, so
an instance that has restarted comes back on vision -- this must not depend on
somebody having switched anything.

    POST /detect?ts=...&score=...&width=...    body: one JPEG
    -> {"ts": ..., "w": 640, "h": 480, "faces": [[x, y, w, h, score], ...], ...}

`ts` is opaque and echoed back untouched. That is the point of it: the caller
stamps each frame with whatever clock it aims by -- track_face_pi.py uses V4L2's
start-of-exposure time, CLOCK_MONOTONIC on the rover -- and gets it back attached
to the boxes, so the control loop knows exactly which moment those boxes describe
rather than assuming a fixed dead time. Nothing here interprets it, and no clock
sync between the two machines is needed or wanted.

`score` is the caller's keep threshold, not a policy of this service's. The rover
runs two thresholds -- a high one to acquire a face, a low one to keep following
one, both in face_tracking/aiming.py -- and only it can know which applies to a
given detection, so it asks for everything above its low bar and decides. The
default here matches KEEP_SCORE, for a caller that has no opinion.

Unlike the three GPU services this binds the LAN rather than loopback, because
the thing that calls it is a rover and there is no sitting at it to open a tunnel.
On MEDIA that also needs a hole in the firewall: WSL runs in mirrored networking
mode behind a Hyper-V firewall whose DefaultInboundAction is Block, so a listening
socket alone is reached by nobody. See the README -- one New-NetFirewallRule,
scoped to LocalSubnet, alongside the one that already lets SSH in.

No FastAPI and no uvicorn, unlike voice_chat/server.py. A turn here is one request
and one response with nothing to stream and nothing to overlap, so the stdlib's
threading server answers it in a tenth of the dependencies -- which matters on a
box where the other three services each carry a multi-gigabyte CUDA stack.
"""

import json
import os
import statistics
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

HOST = os.environ.get("FACE_HOST", "0.0.0.0")
PORT = int(os.environ.get("FACE_PORT", "8768"))

MODEL_FILE = "face_detection_yunet.onnx"
MODEL_URL = (
    "https://github.com/opencv/opencv_zoo/raw/main/models/"
    "face_detection_yunet/face_detection_yunet_2023mar.onnx"
)
MODEL_BYTES = 232589  # what the fetch above should produce; a short file is a bad one
MODEL_PATH = os.environ.get(
    "FACE_MODEL", os.path.join(os.path.dirname(os.path.abspath(__file__)), MODEL_FILE))

# Detection runs on a reduced copy: YuNet's accuracy is set by the model's own
# 640-ish working size, not by handing it more pixels. The rover sends 640x480
# already, so this normally does nothing -- it is here so that a caller sending
# 720p is downscaled rather than made slow.
DEFAULT_WIDTH = int(os.environ.get("FACE_WIDTH", "640"))
# Matches KEEP_SCORE in face_tracking/aiming.py. The rover overrides it per
# request; see the module docstring for why the policy lives there and not here.
DEFAULT_SCORE = float(os.environ.get("FACE_SCORE", "0.60"))
NMS_THRESHOLD = 0.3
TOP_K = 5000

# How many cores YuNet may have. Measured here, on a 640x480 frame:
#
#     1 thread   14.5 ms        320x240:  3.6 ms
#     2          9.5                      2.2
#     4          6.5                      1.7
#     8          5.9                      1.4
#
# Four, because that is where the curve flattens: the eighth thread buys 0.6 ms
# and costs the box half of what is left of it. The point of running on the CPU
# is to be concurrent with whichever model service owns the card, and a detector
# that takes the whole machine to save six milliseconds is not being a good
# neighbour. At 6.5 ms against a rover loop that comes round every ~130 ms, this
# is not remotely what limits anything.
THREADS = int(os.environ.get("FACE_THREADS", "4"))

# A frame is tens of kilobytes. Anything approaching this is not one.
MAX_BODY = 8 << 20

_stats_lock = threading.Lock()
_stats = {"frames": 0, "faces": 0, "errors": 0, "detect_ms": []}
STAT_WINDOW = 200


def ensure_model(path):
    """The ONNX file, fetched once if it is not already here.

    Deployment puts it beside this script, so this normally does nothing. Kept
    anyway because the alternative is a service that fails to start on a fresh
    box for want of 230 kB. Deliberately a copy of the same logic in
    face_tracking/track_face.py rather than an import: that file lives on the
    workstation and this one on MEDIA, and they are not deployed together.
    """
    if os.path.exists(path) and os.path.getsize(path) > MODEL_BYTES // 2:
        return path
    print(f"fetching the face detector ({MODEL_BYTES // 1024} kB, once) -> {path}",
          flush=True)
    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        partial = path + ".part"
        with urllib.request.urlopen(MODEL_URL, timeout=30) as response:
            data = response.read()
        if len(data) < MODEL_BYTES // 2:
            raise ValueError(f"got {len(data)} bytes, expected about {MODEL_BYTES}")
        with open(partial, "wb") as handle:
            handle.write(data)
        os.replace(partial, path)
    except Exception as error:
        sys.exit(f"Could not fetch the face detector: {error}\n"
                 f"Download it from {MODEL_URL} and put it at {path}.")
    return path


_pool_lock = threading.Lock()
_pool = {}  # (size, score) -> [idle detectors]


class Borrowed:
    """One YuNet, on loan for the length of a request.

    Pooled rather than shared, because cv2.FaceDetectorYN carries the input size
    as state and is not documented as safe to call from two threads at once;
    pooled rather than thread-local, because ThreadingHTTPServer gives each
    *connection* a thread, and a client that does not keep the connection open
    would otherwise build a fresh detector for every frame. That is not
    theoretical -- it doubled the measured detection time here, 9.5 ms to 20,
    which is what this class exists to prevent.

    Keyed by size as well, since setInputSize reallocates: a caller alternating
    between two resolutions gets two detectors rather than a rebuild per frame.
    """

    def __init__(self, size, score):
        self.key = (size, round(score, 3))

    def __enter__(self):
        with _pool_lock:
            idle = _pool.setdefault(self.key, [])
            self.net = idle.pop() if idle else None
        if self.net is None:
            size, score = self.key
            self.net = cv2.FaceDetectorYN_create(
                MODEL_PATH, "", size, score, NMS_THRESHOLD, TOP_K)
        return self.net

    def __exit__(self, *exc):
        with _pool_lock:
            _pool[self.key].append(self.net)


def detect(jpeg, score, width):
    """One JPEG -> faces as (x, y, w, h, score) in full-frame pixels, and timings."""
    t0 = time.perf_counter()
    frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("not a decodable image")
    t1 = time.perf_counter()

    height, full_width = frame.shape[:2]
    scale = min(width / full_width, 1.0) if width else 1.0
    if scale < 1.0:
        size = (int(full_width * scale), int(height * scale))
        frame = cv2.resize(frame, size)
    else:
        size = (full_width, height)

    with Borrowed(size, score) as net:
        _, raw = net.detect(frame)
    t2 = time.perf_counter()

    faces = []
    if raw is not None:
        back = 1.0 / scale
        for row in raw:
            x, y, w, h = (round(float(v) * back, 1) for v in row[:4])
            faces.append([x, y, w, h, round(float(row[-1]), 4)])
    return faces, (full_width, height), (t1 - t0) * 1e3, (t2 - t1) * 1e3


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"  # keep-alive: the rover sends 20+ frames a second
    server_version = "face-detect"
    # Worth 45 ms a request, which is most of what this service costs anybody.
    #
    # BaseHTTPRequestHandler flushes its headers and then writes the body, two
    # sends -- and with Nagle on, the second waits for the first to be
    # acknowledged while the client's delayed ACK sits on its hands for 40 ms.
    # Both ends idle, by the book. MEASURED from the rover, GET /health: 55 ms
    # with this left at its default, 8 ms with it set. Detection is 6 ms, so the
    # default was costing seven times the work being asked for.
    disable_nagle_algorithm = True

    def log_message(self, fmt, *args):
        pass  # one line per frame at 20 fps is not a log, it is a fire

    def _reply(self, code, payload):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _query(self):
        from urllib.parse import parse_qs, urlparse

        parsed = urlparse(self.path)
        return parsed.path, parse_qs(parsed.query)

    def do_GET(self):
        path, _ = self._query()
        if path != "/health":
            return self._reply(404, {"error": "GET /health, POST /detect"})
        with _stats_lock:
            recent = list(_stats["detect_ms"])
            frames, faces, errors = _stats["frames"], _stats["faces"], _stats["errors"]
        self._reply(200, {
            "ok": True,
            "model": os.path.basename(MODEL_PATH),
            "opencv": cv2.__version__,
            "threads": THREADS,
            "frames": frames,
            "faces": faces,
            "errors": errors,
            "detect_ms_median": round(statistics.median(recent), 2) if recent else None,
            "detect_ms_max": round(max(recent), 2) if recent else None,
        })

    def do_POST(self):
        path, query = self._query()
        if path != "/detect":
            return self._reply(404, {"error": "GET /health, POST /detect"})
        try:
            length = int(self.headers.get("Content-Length", 0))
        except ValueError:
            length = 0
        if not 0 < length <= MAX_BODY:
            return self._reply(400, {"error": f"Content-Length {length} out of range"})
        jpeg = self.rfile.read(length)

        # Echoed verbatim, never parsed: it is the caller's clock, not ours.
        ts = query.get("ts", [None])[0]
        try:
            score = float(query.get("score", [DEFAULT_SCORE])[0])
            width = int(query.get("width", [DEFAULT_WIDTH])[0])
        except ValueError:
            return self._reply(400, {"error": "score and width must be numbers"})

        try:
            faces, (w, h), decode_ms, detect_ms = detect(jpeg, score, width)
        except Exception as error:
            with _stats_lock:
                _stats["errors"] += 1
            return self._reply(400, {"ts": ts, "error": str(error)})

        with _stats_lock:
            _stats["frames"] += 1
            _stats["faces"] += len(faces)
            _stats["detect_ms"].append(detect_ms)
            del _stats["detect_ms"][:-STAT_WINDOW]

        self._reply(200, {
            "ts": ts, "w": w, "h": h, "faces": faces,
            "decode_ms": round(decode_ms, 2), "detect_ms": round(detect_ms, 2),
        })


def main():
    cv2.setNumThreads(THREADS)
    ensure_model(MODEL_PATH)
    # Built and run once here so a broken model file is a refusal to start rather
    # than a 400 on every frame, and so the first real frame does not pay for the
    # load. It goes back into the pool warm.
    size = (DEFAULT_WIDTH, DEFAULT_WIDTH * 3 // 4)
    blank = np.zeros((size[1], size[0], 3), dtype=np.uint8)
    started = time.perf_counter()
    with Borrowed(size, DEFAULT_SCORE) as warm:
        warm.detect(blank)
    print(f"YuNet ready in {(time.perf_counter() - started) * 1e3:.0f} ms "
          f"(OpenCV {cv2.__version__}, {THREADS} threads)", flush=True)

    server = ThreadingHTTPServer((HOST, PORT), Handler)
    server.daemon_threads = True
    print(f"listening on {HOST}:{PORT}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
