"""A rover that is not there, answering as though it were.

The daemon owns a serial port, a camera and a gimbal, so it only runs on the
board that is bolted to the rover. That makes the whole voice path -- prompt, schemas,
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
import base64
import http.client
import json
import math
import os
import socket
import socketserver
import sys
import threading
import time
from typing import Any

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import prompts

DEFAULT_PORT = 8769
LIGHT_MAX = 255

# A pack that empties while the mock runs. Nothing like a real rover, whose pack
# lasts hours, and that is deliberate: a panel whose number never moves cannot be
# told apart from a panel that has stopped being updated, and telling those two
# apart is most of what this mock is for.
MOCK_BATTERY_FULL_V = 12.5
MOCK_BATTERY_DROP_V_PER_MIN = 0.1

# An invented neighbourhood, for the console's network panel. The three the rover
# has passphrases for and two it has not, because "this one you can join and that
# one you cannot" is the distinction the panel exists to draw and a list where
# every row is joinable would not exercise it. Signals wander a few points per
# reading for the reason the battery drains: a panel that never changes cannot be
# told from a panel that has stopped being updated.
MOCK_NETWORKS = (("TheGreatLord", 82, True), ("TheMaharaja", 61, True),
                 ("TheGreatViking", 47, True), ("Alister", 66, False),
                 ("Sandy Hall (5GHz)", 31, False))
MOCK_WIFI_IFACE = "wlan0"



def _move_report():
    """The rover's own `MoveReport`, or None if `lidar_slam/` is not beside us.

    Borrowed rather than reimplemented: it is what the real rover publishes into
    `nav_status` while a move runs, and a mock that made up its own field names
    would let [drive_web.py](../drive_web/drive_web.py) pass against this and fail
    against the rover. Imported at first use, because this file's whole point is to
    run where the rover's code may not.
    """
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "lidar_slam"))
    try:
        from nav_types import MoveReport
    except Exception:
        return None
    return MoveReport()


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


from mock_room import RoverRoom


class Rover(RoverRoom):
    """The state a real rover would have, and the answers that come out of it."""

    def __init__(self, vision: str | None, picture: bytes | None,
                 drive: bool = False) -> None:
        self.lights = 0
        self.pan = 0
        self.tilt = 0
        self.tracking = False
        self.target = 0
        # The access point it pretends to be on -- the strongest of the invented
        # ones, so a console opened against this mock starts where a rover sitting
        # in its usual spot would. Not `wifi_join`: that name is the method, and
        # calls are dispatched by looking one up on this object.
        self.wifi = MOCK_NETWORKS[0][0]
        self._last_join: dict[str, Any] | None = None
        self.vision = vision
        self.picture = picture
        self.driving = drive
        self.x = self.y = self.heading = 0.0
        self.started = time.monotonic()
        self.trail: list[tuple[float, float]] = []
        self.calls: list[tuple[str, dict[str, Any]]] = []
        self._lock = threading.Lock()
        # The same running commentary the real navigator keeps, from the same
        # class rather than a copy of its shape -- a mock that invented its own
        # field names would let the console pass here and fail on the rover.
        # Absent if lidar_slam is not beside this checkout, which is also the
        # state a rover running an older daemon is in, and the console handles it.
        self.report = _move_report()
        # The map picture last handed to the model, and what it takes to read a
        # place on it back out. Mirrors the daemon's `_map_shown` field for field,
        # because `drive_to_map_point` is refused here for the same four reasons it
        # is refused there and a client cannot tell the two apart.
        self._map_shown: dict[str, Any] | None = None

    # --- what the move is doing, for anything polling nav_status --------------
    # Three lines around each move rather than one wrapper, because a mock's moves
    # do not share a shape the way the rover's do -- one of them is a loop over a
    # route and the other two are arithmetic.

    def _begin(self, kind: str, asked: dict[str, Any], phase: str) -> None:
        if self.report is not None:
            self.report.begin(kind, asked, phase)

    def _say(self, phase: str, why: str = "", **fields: Any) -> None:
        if self.report is not None:
            self.report.say(phase, why, **fields)

    def _say_end(self, reason: str, why: str, result: Any = None) -> Any:
        """Ends the commentary and hands `result` straight back, so it can be
        used in a `return` without a spare line."""
        if self.report is not None:
            self.report.finish(reason, why)
        return result

    # --- the tools ----------------------------------------------------------

    def set_lights(self, arguments: dict[str, Any]) -> dict[str, Any]:
        level = arguments.get("level")
        if not isinstance(level, int) or not 0 <= level <= LIGHT_MAX:
            return {"ok": False, "error": f"level must be a whole number from 0 to {LIGHT_MAX}"}
        self.lights = level
        return {"ok": True, "level": level}

    def get_lights(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        return {"ok": True, "level": self.lights, "on": self.lights > 0}

    def battery(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """The pack, emptying as the mock runs. Same shape the daemon answers in."""
        volts = max(9.6, MOCK_BATTERY_FULL_V
                    - MOCK_BATTERY_DROP_V_PER_MIN
                    * (time.monotonic() - self.started) / 60.0)
        percent = round(max(0.0, min(100.0, (volts - 9.9) / (12.6 - 9.9) * 100)) / 5) * 5
        state = ("full" if volts >= 12.45 else "critical" if volts < 10.8
                 else "low" if volts < 11.2 else "ok")
        return {"ok": True, "volts": round(volts, 2), "percent": percent,
                "state": state, "cells": 3,
                "volts_per_cell": round(volts / 3, 2), "reading_age_s": 0.4,
                "summary": f"The battery is at about {percent}%, "
                           f"or {volts:.1f} volts."}

    def wifi_status(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """The invented neighbourhood, in the shape the daemon answers in.

        A mock of the *unscanned* case as well as the scanned one, because that is
        the state a real rover is usually in: nothing scans while the link is
        healthy, so NetworkManager's list decays to the access point it is on, and
        a console that only ever saw a full list would have no reason to offer a
        button that goes and looks.
        """
        scan = bool(arguments.get("scan"))
        drift = int((time.monotonic() - self.started) / 3) % 7 - 3
        networks = []
        for ssid, signal, configured in MOCK_NETWORKS:
            if not scan and ssid != self.wifi:
                continue
            networks.append({"ssid": ssid,
                             "signal": max(1, min(100, signal + drift)),
                             "security": "WPA2", "in_use": ssid == self.wifi,
                             "configured": configured})
        reading = {"ok": True, "interface": MOCK_WIFI_IFACE,
                   "connected": self.wifi,
                   # Around -45 dBm when the AP reads 82, which is roughly the
                   # relationship the rover's dongle shows.
                   "level_dbm": -90 + (dict((n, s) for n, s, _ in MOCK_NETWORKS)
                                       .get(self.wifi, 50) + drift) // 2,
                   "address": "192.168.1.47",
                   "networks": networks,
                   "configured": [n for n, _, c in MOCK_NETWORKS if c],
                   "scanned": scan, "list_age_s": 0.0}
        if self._last_join is not None:
            reading["last_join"] = dict(self._last_join)
        return reading

    def wifi_join(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Switch networks, without the part where the connection dies.

        The real one answers before it has done anything, because the switch takes
        the link down and the reply would go out over it. This answers in the same
        shape and then simply is on the other network, which is the one way a mock
        of this is easier to drive than the rover: nothing here has to be
        reconnected to.
        """
        ssid = arguments.get("ssid")
        if not isinstance(ssid, str) or not ssid.strip():
            return {"ok": False, "error": "wifi_join wants an ssid"}
        ssid = ssid.strip()
        configured = [n for n, _, c in MOCK_NETWORKS if c]
        if ssid not in configured:
            return {"ok": False,
                    "error": f"there is no passphrase for {ssid} on this rover, so "
                             f"it cannot join it. Configured networks: "
                             f"{', '.join(configured)}"}
        self.wifi = ssid
        self._last_join = {"ssid": ssid, "ok": True, "at": round(time.time(), 1),
                           "seconds": 8.0, "said": ""}
        return {"ok": True, "joining": ssid,
                "note": (f"joining {ssid}. Every connection to this rover is about "
                         f"to drop, including this one; reconnect in a few seconds "
                         f"and wifi_status will say how it went.")}

    def look_at(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self.pan = int(arguments.get("pan", self.pan))
        self.tilt = int(arguments.get("tilt", self.tilt))
        return {"ok": True, "pan": self.pan, "tilt": self.tilt}

    def center_camera(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        # Rest is ten degrees above level on the real rover; see REST_TILT_DEG.
        self.pan, self.tilt = 0, 10
        return {"ok": True, "pan": self.pan, "tilt": self.tilt}

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

    def camera_jpeg(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """A frame in the reply, the way the daemon's control call returns one.

        Not gated on a vision host, because the real one is not: `look` needs
        somewhere to post a picture and this needs only a camera, which is what lets
        a window take pictures from a daemon started without `--vision`.
        """
        if self.picture is None:
            self.picture = _test_card()
        if self.picture is None:
            return {"ok": False,
                    "error": "the camera gave nothing: no picture to send, and "
                             "OpenCV is not here to draw a test card"}
        return {"ok": True, "bytes": len(self.picture), "width": 320, "height": 240,
                "live": self.tracking, "pan": self.pan, "tilt": self.tilt,
                "jpeg_base64": base64.b64encode(self.picture).decode("ascii")}

    def clear_map(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """Throw the map away -- as far as an invented room can.

        The real one empties an occupancy grid that was built up scan by scan and
        stands the rover at the origin of it. This room is not built up; it is
        evaluated from its own geometry every time a map is drawn, so it cannot be
        un-seen and the walls come straight back. What does go is the driven track,
        which is the part a client can see disappear -- and the pose stays where it
        is, because teleporting the rover to the middle of the room would move the
        room around it, which is the one thing clearing a real map does not do.
        """
        had = len(self.trail)
        self.trail = []
        return {"ok": True, "cleared": True,
                "reason": f"the track of {had} places is gone; the invented room "
                          f"itself cannot be un-seen"}

    def look(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        if self.vision is None:
            return {"ok": False, "error": "this rover cannot show you a picture"}
        if self.picture is None:
            return {"ok": False, "error": "the camera gave nothing: no picture to send"}
        name, error = self._post(self.picture)
        if error:
            return {"ok": False, "error": error}
        # Nothing but the name, exactly as the daemon does it. A tool result that
        # says anything about the picture is read as an instruction for the turn.
        return {"ok": True, "image": name}

    def _post(self, image: bytes, kind: str = "image/jpeg"):
        """Push a picture at the vision service. Returns (name, error), one of them.

        Shared by `look` and `show_map` rather than written twice, because the thing
        being exercised is the path -- and two copies of it would be two paths.
        """
        host, _, port = (self.vision or "").partition(":")
        try:
            connection = http.client.HTTPConnection(host, int(port or 8767), timeout=6.0)
            connection.request("POST", "/frame", body=image,
                               headers={"Content-Type": kind,
                                        "Content-Length": str(len(image))})
            payload = json.loads(connection.getresponse().read())
            connection.close()
        except Exception as error:
            return None, (f"could not send the picture to {self.vision}: "
                          f"{type(error).__name__}: {error}")
        if not isinstance(payload, dict) or not payload.get("image"):
            return None, "the picture was not accepted"
        return payload["image"], None

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
        return prompts.tools(vision=self.vision is not None, nav=self.driving)


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
    parser.add_argument("--drive", action="store_true",
                        help="also offer the driving tools, in an invented room. "
                             "For exercising a client -- drive_web/drive_web.py, or a "
                             "conversation -- and not for measuring anything")
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

    rover = Rover(args.vision, picture, args.drive)
    server = serve(rover, args.host, args.port)
    names = ", ".join(prompts.names(rover.tools()))
    print(f"mock rover on {args.host}:{args.port}\n"
          f"  tools: {names}\n"
          f"  vision: {args.vision or 'off'}"
          + (f", {len(picture)} bytes of JPEG" if picture else "")
          + (f"\n  driving: an invented {ROOM_FORWARD_M + ROOM_BACK_M:.0f} by "
             f"{ROOM_LEFT_M + ROOM_RIGHT_M:.0f} m room with a table in it; "
             f"nothing here is measured" if args.drive else "")
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
