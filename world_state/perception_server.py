#!/usr/bin/env python3
"""The perception sidecar: three models in a process of their own, on loopback.

    ~/ugv/world_state/run_perception.sh          # started at boot by crontab
    curl -s 127.0.0.1:8776/health

**Why a separate process at all.** `rover_daemon` owns the driver-board UART, the
gimbal camera and STOP. A model that runs out of memory or takes a fault must not
be able to take those with it, and half a gigabyte of ONNX graphs on a board with
eight is exactly the kind of thing that gets chosen by the out-of-memory killer.
So the daemon asks a question over loopback and treats no answer as an ordinary
outcome, the same arrangement the language model already has one port along.

**Loopback only, and that is the security argument in full.** This port
authenticates nothing and takes arbitrary images; bound to 127.0.0.1 it grants
what an ssh session on this board already grants, and the only client is the
daemon in the next process along.

The models are loaded on the first look rather than at startup, so a rover that
never inspects anything never pays the seven seconds or the memory.
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from world_state.perceive import Perception, Unavailable   # noqa: E402

#: 8769 is the daemon, 8770 the depth camera, 8771 the console, 8772 and 8773 the
#: ROS bridges, 8774 the frame service the voice session uses, and 8775 was the
#: language model that used to answer inspections and is gone. This is the next
#: one along, and it stays where it is rather than moving into the gap.
DEFAULT_PORT = 8776
DEFAULT_BIND = "127.0.0.1"

#: The largest picture this will accept, which is about thirty times the size of
#: a frame from this camera. A cap rather than a limit anyone should reach: the
#: body is read into memory before anything looks at it.
MAX_BODY = 8 * 1024 * 1024


class Handler(BaseHTTPRequestHandler):
    """Two endpoints and no more.

    `GET /health` says whether the models are installed and whether they are
    loaded, without loading them -- the daemon calls it before it opens the
    camera, so it must be free.

    `POST /look` takes a JPEG, as raw bytes or as base64 in JSON, and answers
    with regions and their vectors. Nothing is named: `POST /embed` turns a
    phrase somebody typed into a vector in the same space, and the comparison
    happens in the daemon.
    """

    protocol_version = "HTTP/1.1"
    server_version = "ugv-perception/1"

    def log_message(self, fmt, *args):     # noqa: A003 - BaseHTTPRequestHandler's
        """One line per request, to stdout, which run_perception.sh appends to a
        log. The default writes to stderr with a timestamp of its own."""
        if self.server.quiet:
            return
        sys.stdout.write(f"{time.strftime('%H:%M:%S')} {fmt % args}\n")
        sys.stdout.flush()

    # --- the endpoints --------------------------------------------------------

    def do_GET(self):                      # noqa: N802 - the base class names it
        if self.path.split("?")[0] != "/health":
            return self._send(404, {"ok": False, "error": f"no {self.path} here"})
        ready, why = self.server.perception.available()
        backend, missing_gpu = self.server.perception.chosen()
        return self._send(200, {
            "ok": True,
            "status": "ok" if ready else "no models",
            "ready": ready,
            "detail": why,
            "loaded": self.server.perception._loaded,
            # Which backend this host would use, and -- when it is the slower,
            # blunter one -- why. Worth saying in health rather than only in a
            # look, because a rover quietly running on the CPU is a rover whose
            # vectors will not compare with the ones already stored.
            "backend": backend,
            "no_gpu_because": missing_gpu,
            "fallback": self.server.perception.fallback,
            "load_s": self.server.perception.load_s,
            "looks": self.server.looks,
            "busy": self.server.busy,
            "uptime_s": round(time.monotonic() - self.server.started, 1),
        })

    def do_POST(self):                     # noqa: N802
        route = self.path.split("?")[0]
        if route == "/embed":
            return self._embed()
        if route != "/look":
            return self._send(404, {"ok": False, "error": f"no {self.path} here"})
        jpeg, error = self._body()
        if error:
            return self._send(400, {"ok": False, "error": error})
        try:
            answer = self.server.look(jpeg)
        except Unavailable as unavailable:
            # An ordinary state, not a fault: deployed to but not installed on.
            return self._send(503, {"ok": False, "error": str(unavailable)})
        except Exception as failure:       # never past here: the client is a robot
            return self._send(500, {"ok": False,
                                    "error": f"{type(failure).__name__}: {failure}"})
        return self._send(200, {"ok": True, **answer})

    def _embed(self):
        """Text vectors for arbitrary phrases, which is what a search is made of.

        The same model whose image tower produced every stored region vector, so
        a query lands in the same space and the comparison is a dot product. On
        the GPU the first phrase after a start-up opens the text engine, which
        takes a couple of seconds; every one after that finds it open and costs
        about ten milliseconds.
        """
        raw = self.rfile.read(int(self.headers.get("Content-Length") or 0))
        try:
            asked = json.loads(raw.decode("utf-8", "replace"))
            phrases = [str(one) for one in asked["phrases"]]
        except (KeyError, TypeError, ValueError) as bad:
            return self._send(400, {"ok": False,
                                    "error": f"expected {{\"phrases\": [...]}}: {bad}"})
        if not phrases or len(phrases) > 64:
            return self._send(400, {"ok": False,
                                    "error": "between one and sixty-four phrases"})
        try:
            vectors = self.server.embed(phrases)
        except Unavailable as unavailable:
            return self._send(503, {"ok": False, "error": str(unavailable)})
        except Exception as failure:       # never past here
            return self._send(500, {"ok": False,
                                    "error": f"{type(failure).__name__}: {failure}"})
        return self._send(200, {"ok": True, "phrases": phrases,
                                "vectors": vectors})

    # --- the wire -------------------------------------------------------------

    def _body(self) -> tuple[bytes, str]:
        """The picture out of the request, whichever way it was sent.

        Raw bytes are what the daemon uses, because it already holds a JPEG and
        base64 would be a third more bytes across a loopback socket for nothing.
        JSON is accepted because it is what a person with curl will reach for.
        """
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            return b"", "a Content-Length that was not a number"
        if length <= 0:
            return b"", "no picture in the request"
        if length > MAX_BODY:
            return b"", f"{length} bytes is more picture than this accepts"
        raw = self.rfile.read(length)
        kind = (self.headers.get("Content-Type") or "").split(";")[0].strip()
        if kind != "application/json":
            return raw, ""
        try:
            payload = json.loads(raw.decode("utf-8", "replace"))
            return base64.b64decode(payload["jpeg_base64"]), ""
        except (ValueError, KeyError, TypeError) as error:
            return b"", f"that JSON had no readable jpeg_base64: {error}"

    def _send(self, status: int, payload: dict) -> None:
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class Server(ThreadingHTTPServer):
    """One perception, one lock, and a count of how many looks it has served.

    Threading so that `/health` answers while a look is running -- the daemon
    asks that before it opens the camera and should not wait two seconds for it.
    The look itself is serialised inside `Perception`, because two at once on six
    cores is two slow looks.
    """

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, perception: Perception, quiet: bool = False):
        super().__init__(address, Handler)
        self.perception = perception
        self.quiet = quiet
        self.looks = 0
        self.busy = False
        self.started = time.monotonic()
        self._lock = threading.Lock()

    def look(self, jpeg: bytes) -> dict:
        with self._lock:
            self.busy = True
            try:
                answer = self.perception.look(jpeg)
            finally:
                self.busy = False
            self.looks += 1
        # The vectors are bytes and this is JSON, so they go as base64. A 384-
        # float and a 768-float vector is 6 kB a region before encoding, which is
        # nothing beside the JPEG that arrived, and the store wants the bytes
        # rather than a list of numbers it would have to pack again.
        regions = []
        for region in answer["regions"]:
            regions.append({**region,
                            "dino": base64.b64encode(region["dino"]).decode(),
                            "siglip": base64.b64encode(region["siglip"]).decode()})
        return {**answer, "regions": regions}

    def embed(self, phrases: list) -> list:
        """Text vectors, base64, in the order the phrases were given."""
        with self._lock:
            self.busy = True
            try:
                vectors = self.perception.embed_text(phrases)
            finally:
                self.busy = False
        return [base64.b64encode(
            vector.astype("float32").tobytes()).decode() for vector in vectors]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--bind", default=DEFAULT_BIND)
    parser.add_argument("--threads", type=int, default=0,
                        help="onnxruntime intra-op threads; 0 leaves its default, "
                             "which measured fastest on this board")
    parser.add_argument("--preload", action="store_true",
                        help="load the models now rather than on the first look")
    parser.add_argument("--quiet", action="store_true")
    arguments = parser.parse_args()

    perception = Perception(threads=arguments.threads)
    ready, why = perception.available()
    print(f"perception sidecar on {arguments.bind}:{arguments.port} -- "
          + ("models ready" if ready else why), flush=True)
    if arguments.preload and ready:
        began = time.monotonic()
        perception.load()
        print(f"loaded in {time.monotonic() - began:.1f} s on "
              f"{perception.backend}", flush=True)

    server = Server((arguments.bind, arguments.port), perception,
                    quiet=arguments.quiet)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("stopping", flush=True)
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
