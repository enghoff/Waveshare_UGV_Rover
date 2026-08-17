"""A rover that is not there, answering as though it were.

The daemon owns a serial port, a camera and a gimbal, so it only runs on the Pi
that is bolted to the rover. That makes the whole voice path -- prompt, schemas,
tool dispatch, the picture -- untestable anywhere else, which is the wrong way
round: the part most likely to be wrong is the conversation, and the conversation
needs no hardware.

So this speaks the daemon's wire protocol and lies about the hardware. It holds
the state a real rover would hold, because that is what makes a conversation
worth having -- ask it to turn the lights on and then ask whether they are on,
and the second answer depends on the first. Everything else is invented.

    python voice_chat/mock_rover.py                    # on 127.0.0.1:8769
    python voice_chat/mock_rover.py --picture room.jpg # ...and `look` sees that

The schemas are the real ones, read out of the daemon's source by
[prompts.py](prompts.py) rather than written out again here. A mock that
described its tools in its own words would be a mock of a different rover, and
every measurement taken through it would be measuring this file's prose.

`look` behaves like the real one in the way that matters: it posts a JPEG to
whatever vision service it was pointed at and returns nothing but the name that
came back. That is the path worth exercising, because it is the one that has to
be rebuilt when the model moves off the machine holding the picture.
"""

from __future__ import annotations

import argparse
import http.client
import json
import os
import socket
import socketserver
import sys
import threading
from typing import Any

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import prompts

DEFAULT_PORT = 8769
LIGHT_MAX = 255

# What the invented camera sees. Two faces, because one is the boring case: the
# tracker's "next" is only meaningful where there is somebody else to move to.
FACES = [
    {"where": "on your left, quite close"},
    {"where": "in the centre, further away"},
]


def _test_card() -> bytes | None:
    """A picture to hand back when nobody supplied one, if OpenCV is here.

    Deliberately something with describable content rather than a blank frame:
    the question this exists to answer is whether the model receives an image and
    says what is in it, and a grey rectangle cannot tell those two failures apart.
    """
    try:
        import cv2
        import numpy as np
    except Exception:
        return None
    image = np.full((240, 320, 3), 235, dtype="uint8")
    cv2.rectangle(image, (30, 60), (130, 180), (40, 40, 200), -1)   # a red box
    cv2.circle(image, (230, 120), 55, (60, 160, 40), -1)            # a green ball
    cv2.putText(image, "ROVER", (28, 40), cv2.FONT_HERSHEY_SIMPLEX,
                0.9, (30, 30, 30), 2, cv2.LINE_AA)
    ok, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return bytes(buffer) if ok else None


class Rover:
    """The state a real rover would have, and the answers that come out of it."""

    def __init__(self, vision: str | None, picture: bytes | None) -> None:
        self.lights = 0
        self.pan = 0
        self.tilt = 0
        self.tracking = False
        self.target = 0
        self.vision = vision
        self.picture = picture
        self.calls: list[tuple[str, dict[str, Any]]] = []
        self._lock = threading.Lock()

    # --- the tools ----------------------------------------------------------

    def set_lights(self, arguments: dict[str, Any]) -> dict[str, Any]:
        level = arguments.get("level")
        if not isinstance(level, int) or not 0 <= level <= LIGHT_MAX:
            return {"ok": False, "error": f"level must be a whole number from 0 to {LIGHT_MAX}"}
        self.lights = level
        return {"ok": True, "level": level}

    def get_lights(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        return {"ok": True, "level": self.lights, "on": self.lights > 0}

    def look_at(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self.pan = int(arguments.get("pan", self.pan))
        self.tilt = int(arguments.get("tilt", self.tilt))
        return {"ok": True, "pan": self.pan, "tilt": self.tilt}

    def center_camera(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        self.pan = self.tilt = 0
        return {"ok": True, "pan": 0, "tilt": 0}

    def count_faces(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        return {"ok": True, "faces": len(FACES),
                "where": [face["where"] for face in FACES]}

    def start_tracking(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        already, self.tracking = self.tracking, True
        return {"ok": True, "tracking": True, **({"already": True} if already else {})}

    def stop_tracking(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        already, self.tracking = not self.tracking, False
        return {"ok": True, "tracking": False, **({"already": True} if already else {})}

    def track_next(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        if not self.tracking:
            return {"ok": False, "error": "face tracking is not running"}
        self.target = (self.target + 1) % len(FACES)
        return {"ok": True, "target": self.target, "of": len(FACES),
                "where": FACES[self.target]["where"]}

    def tracking_status(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        if not self.tracking:
            return {"ok": True, "tracking": False}
        return {"ok": True, "tracking": True, "faces": len(FACES),
                "where": FACES[self.target]["where"]}

    def set_vision(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Where `look` posts its pictures. A control call, as on the real daemon.

        Faithful to the real one in the part that matters: naming no address
        switches the picture path off, which withdraws `look` from the tool list.
        A client that gets this wrong against the mock gets it wrong against the
        rover, which is the whole reason the mock exists.
        """
        address = arguments.get("address")
        if address is None or (isinstance(address, str) and not address.strip()):
            self.vision = None
            return {"ok": True, "vision": None, "tools": prompts.names(self.tools())}
        if not isinstance(address, str):
            return {"ok": False, "error": "set_vision wants an address like host:port"}
        self.vision = address.strip()
        # A mock started with no picture path has no picture either. Draw one
        # now, so that turning the path on turns the camera on with it.
        if self.picture is None:
            self.picture = _test_card()
        return {"ok": True, "vision": f"http://{self.vision}/frame",
                "tools": prompts.names(self.tools())}

    def look(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        if self.vision is None:
            return {"ok": False, "error": "this rover cannot show you a picture"}
        if self.picture is None:
            return {"ok": False, "error": "the camera gave nothing: no picture to send"}
        host, _, port = self.vision.partition(":")
        try:
            connection = http.client.HTTPConnection(host, int(port or 8767), timeout=6.0)
            connection.request("POST", "/frame", body=self.picture,
                               headers={"Content-Type": "image/jpeg",
                                        "Content-Length": str(len(self.picture))})
            payload = json.loads(connection.getresponse().read())
            connection.close()
        except Exception as error:
            return {"ok": False,
                    "error": f"could not send the picture to {self.vision}: "
                             f"{type(error).__name__}: {error}"}
        if not isinstance(payload, dict) or not payload.get("image"):
            return {"ok": False, "error": "the picture was not accepted"}
        # Nothing but the name, exactly as the daemon does it. A tool result that
        # says anything about the picture is read as an instruction for the turn.
        return {"ok": True, "image": payload["image"]}

    # --- dispatch -----------------------------------------------------------

    def call(self, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            self.calls.append((name, arguments))
            handler = getattr(self, name, None)
            if handler is None or name.startswith("_") or name == "call":
                return {"ok": False, "error": f"this rover has no tool called {name}"}
            try:
                return handler(arguments)
            except Exception as error:  # a failure is an answer, never an exception
                return {"ok": False, "error": f"{type(error).__name__}: {error}"}

    def tools(self) -> list[dict[str, Any]]:
        return prompts.tools(vision=self.vision is not None)


def serve(rover: Rover, host: str = "127.0.0.1", port: int = DEFAULT_PORT,
          quiet: bool = False) -> socketserver.ThreadingTCPServer:
    """Start answering on `host:port`. Returns the server; caller shuts it down."""

    class Handler(socketserver.StreamRequestHandler):
        def handle(self) -> None:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            for raw in self.rfile:
                try:
                    request = json.loads(raw)
                except ValueError:
                    continue
                name = request.get("call")
                if name == "list_tools":
                    reply: dict[str, Any] = {"ok": True, "tools": rover.tools()}
                else:
                    reply = rover.call(name, request.get("arguments") or {})
                if not quiet and name != "list_tools":
                    print(f"  {name}{json.dumps(request.get('arguments') or {})}"
                          f" -> {json.dumps(reply)}", flush=True)
                self.wfile.write(json.dumps(reply).encode() + b"\n")

    class Server(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True

    server = Server((host, port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--vision", nargs="?", default=None, const="127.0.0.1:8767",
                        metavar="HOST:PORT",
                        help="offer 'look' and post pictures here (bare --vision "
                             "means 127.0.0.1:8767)")
    parser.add_argument("--picture", metavar="FILE",
                        help="the JPEG 'look' hands over; without one a test card "
                             "is drawn, if OpenCV is installed")
    args = parser.parse_args()

    picture = None
    if args.vision is not None:
        if args.picture:
            picture = open(args.picture, "rb").read()
        else:
            picture = _test_card()
            if picture is None:
                print("  no --picture and no OpenCV to draw one; 'look' will fail",
                      file=sys.stderr)

    rover = Rover(args.vision, picture)
    server = serve(rover, args.host, args.port)
    names = ", ".join(prompts.names(rover.tools()))
    print(f"mock rover on {args.host}:{args.port}\n"
          f"  tools: {names}\n"
          f"  vision: {args.vision or 'off'}"
          + (f", {len(picture)} bytes of JPEG" if picture else "")
          + "\nCtrl-C to stop.", flush=True)
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        server.shutdown()
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
