"""Face detection on the rover itself, on the OAK's VPU. JPEG in, boxes out.

This is face_detect/server.py's protocol, answered from the rover instead of from
MEDIA -- same query string, same JSON, same opaque `ts` -- so
face_tracking/track_face_pi.py needs nothing but a different address:

    python3 track_face_pi.py --service 127.0.0.1:8768

The detector is not YuNet and not on this CPU. A Pi 1 cannot run either, but it
turns out it does not have to: the camera bolted to the rover has an Intel Myriad
X in it, the same chip Intel sold as a Neural Compute Stick 2, and it will accept
Intel's firmware and run a compiled graph without depthai ever being involved.
oak.py boots it and holds it open; everything here is the HTTP shape around that.

    POST /detect?ts=...&score=...&width=...    body: one JPEG
    -> {"ts": ..., "w": 640, "h": 480, "faces": [[x, y, w, h, score], ...], ...}

`width` is accepted and ignored, unlike on MEDIA. There, it decides how far the
frame is downscaled before YuNet sees it; here the graph is compiled for a fixed
320x240 input and every frame arrives at that size whatever the caller asks --
that being exactly half of the camera's 640x480, which is what lets the JPEG
decoder land on it without a resize afterwards. Boxes still come back in
full-frame pixels, which is the part the caller actually depends on.

`score` is still the caller's threshold rather than a policy here -- the rover
runs a high one to acquire a face and a low one to keep following it, both in
face_tracking/aiming.py -- so this returns everything above what it is given.

**Binds loopback by default**, unlike MEDIA's copy of this. There the caller is a
rover across the network; here it is a process on the same machine, and nothing
about this authenticates.

One frame at a time, enforced here rather than assumed: the device has a single
executor and one graph, so a second request would have to wait anyway. The lock
makes that explicit and keeps the reused input buffer safe.
"""

import argparse
import json
import os
import statistics
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oak import Oak, OakError  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
BLOB = os.path.join(HERE, "face-detection-retail-0004-320x240.blob")

DEFAULT_SCORE = 0.5
MAX_BODY = 8 << 20
STAT_WINDOW = 100
RECENT_WINDOW = 60

# The SSD's output is a fixed 200 rows of seven float16s -- image id, label,
# confidence, then the corners as fractions of the frame. A row whose image id is
# negative ends the list, which is how a frame with two faces in it costs the same
# 2.8 kB on the wire as a frame with none.
ROW_FLOATS = 7

_device = None
_input = None
_save_dir = None
_lock = threading.Lock()
_stats_lock = threading.Lock()
_stats = {"frames": 0, "faces": 0, "errors": 0, "detect_ms": [], "decode_ms": [],
          "last_error": None, "recent": []}


def detect(jpeg, score):
    """One JPEG -> faces as (x, y, w, h, score) in full-frame pixels, and timings."""
    import ctypes

    with _lock:
        t0 = time.perf_counter()
        sizes = _device.jpeg_to_input(jpeg, _input)
        if sizes is None:
            raise ValueError("not a decodable image")
        t1 = time.perf_counter()
        raw = _device.infer(_input)
        t2 = time.perf_counter()

    full_width, height, _, _ = sizes
    rows = len(raw) // (ROW_FLOATS * 2)
    values = struct.unpack("<%de" % (rows * ROW_FLOATS), raw[:rows * ROW_FLOATS * 2])

    faces = []
    for row in range(rows):
        image_id, _label, confidence, xmin, ymin, xmax, ymax = \
            values[row * ROW_FLOATS:(row + 1) * ROW_FLOATS]
        if image_id < 0:
            break                      # the terminator, not a detection
        if confidence < score:
            continue
        x, y = xmin * full_width, ymin * height
        faces.append([round(x, 1), round(y, 1),
                      round(xmax * full_width - x, 1), round(ymax * height - y, 1),
                      round(float(confidence), 4)])
    return faces, (full_width, height), (t1 - t0) * 1e3, (t2 - t1) * 1e3


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"      # keep-alive: the caller sends every frame
    server_version = "oak-detect"
    # The same 40 ms that MEDIA's copy of this documents, and it is worth more
    # here rather than less: over loopback a round trip is a couple of
    # milliseconds, so a Nagle stall would be twenty times the request.
    disable_nagle_algorithm = True

    def log_message(self, fmt, *args):
        pass

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
            detect_ms = list(_stats["detect_ms"])
            decode_ms = list(_stats["decode_ms"])
            frames, faces, errors = _stats["frames"], _stats["faces"], _stats["errors"]
        self._reply(200, {
            "ok": True,
            "model": os.path.basename(BLOB),
            "device": "Myriad X, %dx%d input" % (_device.input_shape[2],
                                                 _device.input_shape[1]),
            "frames": frames,
            "faces": faces,
            "errors": errors,
            # Kept because the two kinds of error need telling apart and the
            # count alone cannot: a frame the camera cut short is routine and the
            # caller simply sends the next one, while anything from the device is
            # the service needing a restart to boot it again.
            "last_error": _stats["last_error"],
            "recent": list(_stats["recent"]),
            "decode_ms_median": round(statistics.median(decode_ms), 2) if decode_ms else None,
            "detect_ms_median": round(statistics.median(detect_ms), 2) if detect_ms else None,
            "detect_ms_max": round(max(detect_ms), 2) if detect_ms else None,
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
            return self._reply(400, {"error": "Content-Length %d out of range" % length})
        jpeg = self.rfile.read(length)

        ts = query.get("ts", [None])[0]        # echoed verbatim, never parsed
        try:
            score = float(query.get("score", [DEFAULT_SCORE])[0])
        except ValueError:
            return self._reply(400, {"error": "score must be a number"})

        try:
            faces, (w, h), decode_ms, detect_ms = detect(jpeg, score)
        except Exception as error:
            with _stats_lock:
                _stats["errors"] += 1
                _stats["last_error"] = str(error)
            return self._reply(400, {"ts": ts, "error": str(error)})

        with _stats_lock:
            _stats["frames"] += 1
            _stats["faces"] += len(faces)
            # The top score and the count for each of the last few frames. This is
            # the only view of what the tracking loop is actually being told: from
            # outside, a loop that never locks on looks the same whether nothing
            # scores high enough or something else scored higher.
            _stats["recent"].append(
                [round(max(f[4] for f in faces), 3) if faces else 0.0, len(faces),
                 int(max(faces, key=lambda f: f[2] * f[3])[2]) if faces else 0])
            del _stats["recent"][:-RECENT_WINDOW]
            _stats["detect_ms"].append(detect_ms)
            _stats["decode_ms"].append(decode_ms)
            del _stats["detect_ms"][:-STAT_WINDOW]
            del _stats["decode_ms"][:-STAT_WINDOW]

        if _save_dir:
            name = "face.jpg" if faces else "no_face.jpg"
            try:
                with open(os.path.join(_save_dir, name), "wb") as handle:
                    handle.write(jpeg)
            except OSError:
                pass

        self._reply(200, {
            "ts": ts, "w": w, "h": h, "faces": faces,
            "decode_ms": round(decode_ms, 2), "detect_ms": round(detect_ms, 2),
        })


def main():
    global _device, _input
    import ctypes

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8768)
    parser.add_argument("--blob", default=BLOB)
    # A diagnostic, off unless asked for: writes the most recent frame that found
    # no face and the most recent that did, so "the loop never detects anything"
    # can be told apart from "the loop is sending something unexpected". Costs one
    # file write per frame, which is why it is not on by default.
    parser.add_argument("--save-frames", metavar="DIR",
                        help="keep the last frame with and without a face, for looking at")
    args = parser.parse_args()
    global _save_dir
    _save_dir = args.save_frames

    # Booted before the socket is bound, so a camera that is not there is a
    # refusal to start rather than a 400 on every frame -- and so the first real
    # frame does not pay for the firmware upload.
    started = time.perf_counter()
    _device = Oak(args.blob).open()
    _input = ctypes.create_string_buffer(_device.input_bytes)
    print("VPU booted and graph loaded in %.1f s: %dx%d input, %d bytes in, %d out"
          % (time.perf_counter() - started, _device.input_shape[2],
             _device.input_shape[1], _device.input_bytes, _device.output_bytes),
          flush=True)

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print("listening on %s:%d" % (args.host, args.port), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        _device.close()


if __name__ == "__main__":
    main()
