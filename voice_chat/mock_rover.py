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

# The invented room, in metres from wherever the rover started, with forward as +x
# and left as +y -- the frame lidar_slam works in. A table sits in front and to
# both sides of it, because a table is the thing the rover is asked to go round and
# the interesting property of one is that a lidar sees four thin legs and no top.
ROOM_FORWARD_M, ROOM_BACK_M = 2.5, 1.5
ROOM_LEFT_M, ROOM_RIGHT_M = 2.0, 2.0
LEGS = ((1.2, 0.4), (1.2, -0.4), (2.0, 0.4), (2.0, -0.4))
LEG_RADIUS_M = 0.05
STANDOFF_M = 0.30          # the real navigator's rule, mirrored here
MAX_RANGE_M = 12.0
# The daemon's own map limits, read out of its source the same way the schemas are.
# A mock that had its own copy of these would drift from the rover, and the whole
# point of clamping here is that a client which shows what it *got* can be tested
# against a rover that says no.
MAP_MAX_HALF_EXTENT_M = prompts._literal(prompts.DAEMON, "MAP_MAX_HALF_EXTENT_M")
MAP_MAX_SCALE = prompts._literal(prompts.DAEMON, "MAP_MAX_SCALE")
MAP_MIN_PIXELS = prompts._literal(prompts.DAEMON, "MAP_MIN_PIXELS")
MAP_MAX_PIXELS = prompts._literal(prompts.DAEMON, "MAP_MAX_PIXELS")
MAP_PIXELS = prompts._literal(prompts.DAEMON, "MAP_PIXELS")
CAMERA_FOV_DEG = prompts._literal(prompts.DAEMON, "CAMERA_FOV_DEG")


def _wrap(radians: float) -> float:
    return (radians + math.pi) % (2 * math.pi) - math.pi

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

    def __init__(self, vision: str | None, picture: bytes | None,
                 drive: bool = False) -> None:
        self.lights = 0
        self.pan = 0
        self.tilt = 0
        self.tracking = False
        self.target = 0
        self.vision = vision
        self.picture = picture
        self.driving = drive
        self.x = self.y = self.heading = 0.0
        self.trail: list[tuple[float, float]] = []
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

    # --- driving, in an invented room ---------------------------------------
    #
    # Offered only under --drive, and worth being precise about what it is for.
    # These answers exercise the *shape* of the driving path -- the schemas, the
    # dispatch, the standoff refusing a move, the map arriving as a picture, a
    # client's buttons and tables -- and they are the wrong thing to draw any
    # conclusion from about how the rover moves. This room has no floor, no track
    # slip, no coast after the power comes off and no lidar that browns out when
    # the motors pull, and those are the four things that make real driving hard.
    # A turn here is exact because arithmetic is exact. On the rover it is not, and
    # that is measured with lidar_slam/calibrate_turn.py, on the rover.

    def drive(self, arguments: dict[str, Any]) -> dict[str, Any]:
        distance = float(arguments.get("distance_m", 0.5))
        speed = float(arguments.get("speed_ms") or 0.2)
        if distance <= 0.0:
            return {"ok": False, "error": "distance_m has to be positive"}

        # Walked in small steps rather than solved, so that stopping at the
        # standoff falls out of the same code that moves -- which is how the real
        # one works, and it means a drive into a wall stops where it meets it.
        step, travelled, reason = 0.02, 0.0, "arrived"
        while travelled < distance:
            hop = min(step, distance - travelled)
            ahead = self._range_at(self.x, self.y, self.heading)
            if ahead < STANDOFF_M + 0.02:
                reason = "blocked"
                break
            self.x += hop * math.cos(self.heading)
            self.y += hop * math.sin(self.heading)
            travelled += hop
            self.trail.append((self.x, self.y))

        detail = ""
        if reason == "blocked":
            detail = (f"stopped {STANDOFF_M:.2f} m short of something after "
                      f"{travelled:.2f} m of the {distance:.2f} m asked for")
        return {"ok": True, "reason": reason, "travelled_m": round(travelled, 3),
                "turned_deg": 0.0,
                **({"detail": detail} if detail else {}),
                **self._nav_context(speed)}

    def drive_to(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Around the table, not through it -- the point of the invented room."""
        ahead = float(arguments.get("ahead_m", 0.0) or 0.0)
        left = float(arguments.get("left_m", 0.0) or 0.0)
        speed = float(arguments.get("speed_ms") or 0.2)
        range_m = math.hypot(ahead, left)
        if range_m < 0.08:
            return {"ok": True, "reason": "arrived", "travelled_m": 0.0,
                    "turned_deg": 0.0, **self._nav_context(speed)}
        if range_m > 8.0:
            return {"ok": False, "error": (
                f"that is {range_m:.1f} m away and a single route is capped at 8 m")}

        target = (self.x + ahead * math.cos(self.heading) - left * math.sin(self.heading),
                  self.y + ahead * math.sin(self.heading) + left * math.cos(self.heading))
        start_heading = self.heading
        travelled, replans = 0.0, 0
        last_why = "no clear route through what the lidar has seen"
        while replans <= 8:
            path, last_why = self._plan_to(target)
            if not path:
                break
            blocked = False
            for wx, wy in path[1:]:
                desired = math.atan2(wy - self.y, wx - self.x)
                delta = _wrap(desired - self.heading)
                if abs(math.degrees(delta)) > 35.0:
                    self.heading = _wrap(self.heading + delta)
                while math.hypot(wx - self.x, wy - self.y) > 0.12:
                    heading = math.atan2(wy - self.y, wx - self.x)
                    if self._range_at(self.x, self.y, heading) < STANDOFF_M + 0.02:
                        blocked = True
                        break
                    hop = min(0.02, math.hypot(wx - self.x, wy - self.y) - 0.10)
                    if hop <= 0.0:
                        break
                    self.x += hop * math.cos(heading)
                    self.y += hop * math.sin(heading)
                    self.heading = heading
                    travelled += hop
                    self.trail.append((self.x, self.y))
                if blocked:
                    break
            remaining = math.hypot(target[0] - self.x, target[1] - self.y)
            if remaining <= 0.16:
                turned = math.degrees(_wrap(self.heading - start_heading))
                extra = (f"replanned {replans} time"
                         f"{'' if replans == 1 else 's'}" if replans else "")
                return {"ok": True, "reason": "arrived",
                        "travelled_m": round(travelled, 3),
                        "turned_deg": round(turned, 1),
                        **({"detail": extra} if extra else {}),
                        **self._nav_context(speed)}
            replans += 1
            if not blocked:
                last_why = "the route did not reach that place"
        turned = math.degrees(_wrap(self.heading - start_heading))
        return {"ok": False, "reason": "blocked",
                "travelled_m": round(travelled, 3),
                "turned_deg": round(turned, 1),
                "detail": last_why, **self._nav_context(speed)}

    def _plan_to(self, target: tuple[float, float]):
        """The invented room as an occupancy grid, then the real planner."""
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), "lidar_slam"))
        import numpy as np
        import planner

        res, n, occ = 0.05, 120, 20
        origin = n // 2
        grid = np.zeros((n, n), dtype=np.int8)
        wall = res * 1.5
        for ix in range(n):
            for iy in range(n):
                wx, wy = (ix - origin) * res, (iy - origin) * res
                inside = (-ROOM_BACK_M < wx < ROOM_FORWARD_M
                          and -ROOM_RIGHT_M < wy < ROOM_LEFT_M)
                on_wall = (min(abs(wx + ROOM_BACK_M), abs(wx - ROOM_FORWARD_M),
                               abs(wy + ROOM_RIGHT_M), abs(wy - ROOM_LEFT_M)) < wall)
                near_leg = any((wx - lx) ** 2 + (wy - ly) ** 2
                               <= (LEG_RADIUS_M + STANDOFF_M * 0.15) ** 2
                               for lx, ly in LEGS)
                if near_leg or on_wall:
                    grid[ix, iy] = occ
                elif inside:
                    grid[ix, iy] = -10
        return planner.plan(grid, res, occ, (self.x, self.y), target,
                            inflate_m=STANDOFF_M)

    def turn_in_place(self, arguments: dict[str, Any]) -> dict[str, Any]:
        angle = float(arguments.get("angle_deg", 0.0))
        self.heading = _wrap(self.heading + math.radians(angle))
        return {"ok": True, "reason": "arrived", "travelled_m": 0.0,
                "turned_deg": round(angle, 1), **self._nav_context(0.0)}

    def stop_driving(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        return {"ok": True, "stopped": True, "latched": False}

    def describe_surroundings(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        return {"ok": True, **self._nav_context(0.0), **self._described()}

    def nav_status(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """The engineering numbers, as the real daemon's control call returns them."""
        return {"ok": True, "driving": False, "estop": False,
                "pose": {"x_m": round(self.x, 3), "y_m": round(self.y, 3),
                         "heading_deg": round(math.degrees(self.heading), 1)},
                "speed_ms": 0.0, "turn_dps": 0.0,
                "clearance_m": round(self._range_at(self.x, self.y, self.heading), 2),
                "steering_deg": 0.0, "match_score": 0.95,
                "position_trusted": True, "scans": len(self.trail) + 100,
                "dropped_scans": 0, "pwm": [0, 0], "lidar_ok": True,
                "lidar_live": True, "lidar_port": "invented", "scan_age_s": 0.05,
                "remaining_m": None}

    def map_png(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Zoomed, clamped and reported the way the real one does.

        Including working pixels per cell out from the extent and the wanted picture
        size rather than taking it as an argument, so that widening the view here also
        keeps the picture the same size. A client that displays what it *got* needs a
        mock that can hand back something other than what was asked for.
        """
        half = min(MAP_MAX_HALF_EXTENT_M, max(0.5, float(
            arguments.get("half_extent_m", 3.0))))
        cells = 2 * max(8, int(half / 0.05)) + 1
        if arguments.get("scale") is not None:
            scale = int(min(MAP_MAX_SCALE, max(1, arguments["scale"])))
        else:
            pixels = min(MAP_MAX_PIXELS, max(MAP_MIN_PIXELS, float(
                arguments.get("pixels", MAP_PIXELS))))
            scale = int(min(MAP_MAX_SCALE, max(1, round(pixels / cells))))
        while scale > 1 and cells * scale > MAP_MAX_PIXELS:
            scale -= 1
        rover_up = bool(arguments.get("rover_up", False))
        started = time.monotonic()
        png, caption = self._map(half, scale, rover_up)
        return {"ok": True, "caption": caption, "bytes": len(png),
                "half_extent_m": round(half, 2), "scale": scale,
                "pixels": int.from_bytes(png[16:20], "big"), "rover_up": rover_up,
                "pose": {"x_m": round(self.x, 3), "y_m": round(self.y, 3),
                         "heading_deg": round(math.degrees(self.heading), 1)},
                "render_s": round(time.monotonic() - started, 2),
                "png_base64": base64.b64encode(png).decode("ascii")}

    def show_map(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """The model's version: the picture goes to the vision host, not the reply."""
        if self.vision is None:
            return {"ok": False, "error": "there is nowhere to send a picture"}
        png, caption = self._map(3.0, 3)
        _name, error = self._post(png, "image/png")
        result = {"ok": True, "caption": caption, **self._described()}
        if error:
            result["note"] = ("the map could not be sent as a picture, so answer "
                              "from the description alone")
        return result

    # --- the invented room --------------------------------------------------

    def _range_at(self, x: float, y: float, heading: float) -> float:
        """How far a ray from here gets before it meets the room or a table leg."""
        dx, dy = math.cos(heading), math.sin(heading)
        best = MAX_RANGE_M
        for delta, position, low, high in ((dx, x, -ROOM_BACK_M, ROOM_FORWARD_M),
                                          (dy, y, -ROOM_RIGHT_M, ROOM_LEFT_M)):
            if abs(delta) > 1e-9:
                edge = (high - position) if delta > 0 else (low - position)
                best = min(best, edge / delta)
        for leg_x, leg_y in LEGS:
            # Ray against a circle: the near root, when there is one in front.
            ox, oy = x - leg_x, y - leg_y
            b = ox * dx + oy * dy
            c = ox * ox + oy * oy - LEG_RADIUS_M * LEG_RADIUS_M
            disc = b * b - c
            if disc < 0.0:
                continue
            hit = -b - math.sqrt(disc)
            if hit > 0.0:
                best = min(best, hit)
        return max(0.0, best)

    def _nav_context(self, _speed: float) -> dict[str, Any]:
        ahead = self._range_at(self.x, self.y, self.heading)
        return {"clear_ahead_m": round(ahead, 2), "surroundings": self._text()}

    def _described(self) -> dict[str, Any]:
        return {"text": self._text(), "pose": {
            "x_m": round(self.x, 3), "y_m": round(self.y, 3),
            "heading_deg": round(math.degrees(self.heading), 1)}}

    def _text(self) -> str:
        """Bearings the way the real describe_surroundings words them."""
        parts = []
        for bearing, name in ((0, "straight ahead"), (45, "to the left"),
                              (90, "hard left"), (-45, "to the right"),
                              (-90, "hard right"), (180, "behind")):
            span = self._range_at(self.x, self.y,
                                 self.heading + math.radians(bearing))
            parts.append(f"{span:.2f} m {name}")
        return ("An invented room, so nothing here was measured: "
                + ", ".join(parts)
                + ". There is a table with four legs in it, and the lidar sees the "
                  "legs rather than the top.")

    def _map(self, half_extent_m: float, scale: int, rover_up: bool = False):
        """The room as a colour PNG, drawn with the rover's own encoder and palette.

        `mapimg` is imported from the rover's tree rather than reimplemented: this
        exists to exercise a client's picture path, and a second PNG writer here
        would be testing this file's encoder instead of the rover's. The palette and
        the arrow come from there for the same reason -- a console that looks one way
        against the mock and another against the rover is a console that hides
        exactly the kind of drawing bug this is here to catch.

        `rover_up` turns the page with the rover, as the real renderer does. Here that
        is one rotation in `to_pixels` and the cells sampled through it, rather than
        the array-sampling the real one needs, because this room is a formula and can
        be evaluated at any point rather than looked up in a grid.
        """
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), "lidar_slam"))
        import mapimg

        res = 0.05
        half = max(8, int(half_extent_m / res))
        size = half * 2 + 1
        canvas = mapimg.Canvas(size * scale, size * scale, mapimg.C_UNKNOWN)
        ahead_cos, ahead_sin = ((math.cos(self.heading), math.sin(self.heading))
                                if rover_up else (1.0, 0.0))

        def to_pixels(px: float, py: float):
            # Up the page is either the start heading or the rover's own, exactly as
            # mapimg.render arranges it; left is to the left in both.
            dx, dy = (px - self.x) / res, (py - self.y) / res
            forward = dx * ahead_cos + dy * ahead_sin
            sideways = -dx * ahead_sin + dy * ahead_cos
            return (half - sideways) * scale, (half - forward) * scale

        def at_cell(iy: int, ix: int):
            """The world point a cell of the picture looks at. The inverse of
            to_pixels, so the room and everything drawn on it agree however the page
            is turned -- which is the bug this mock exists to make visible."""
            forward, sideways = ix - half, iy - half
            return (self.x + (forward * ahead_cos - sideways * ahead_sin) * res,
                    self.y + (forward * ahead_sin + sideways * ahead_cos) * res)

        # Filled from the cell indices rather than by rounding to_pixels back to a
        # block: the two agree to within float error, and that error is enough to
        # drop a one-pixel line between neighbouring cells, which drew a faint grid
        # over the whole picture.
        wall = res * 1.5
        for iy in range(size):
            for ix in range(size):
                wx, wy = at_cell(iy, ix)
                inside = (-ROOM_BACK_M < wx < ROOM_FORWARD_M
                          and -ROOM_RIGHT_M < wy < ROOM_LEFT_M)
                on_wall = (min(abs(wx + ROOM_BACK_M), abs(wx - ROOM_FORWARD_M),
                               abs(wy + ROOM_RIGHT_M), abs(wy - ROOM_LEFT_M)) < wall)
                near_leg = any((wx - lx) ** 2 + (wy - ly) ** 2
                               <= (LEG_RADIUS_M + res) ** 2 for lx, ly in LEGS)
                if near_leg or on_wall:
                    value = mapimg.C_OCCUPIED
                elif inside:
                    value = mapimg.C_FREE
                else:
                    # Beyond the walls the lidar has seen nothing, and a mock that
                    # painted that solid would be inviting the reader to read black
                    # as "outside" rather than as "something is there".
                    value = mapimg.C_UNKNOWN
                col, row = (size - 1 - iy) * scale, (size - 1 - ix) * scale
                for dy in range(scale):
                    for dx in range(scale):
                        canvas.put(col + dx, row + dy, value)

        prev = None
        for tx, ty in list(self.trail)[-400:]:
            cur = to_pixels(tx, ty)
            if prev is not None:
                canvas.line(prev[0], prev[1], cur[0], cur[1], mapimg.C_TRACK,
                            thickness=max(1, scale // 2))
            prev = cur

        # The arrow turns with the heading, which the old dot-and-whisker did not:
        # it was drawn straight up whatever the rover had done, so every turn looked
        # like it had not happened.
        #
        # The camera's cone first, so the arrow is never crossed by it -- and drawn
        # by the rover's own `draw_camera`, including the minus sign that turns a
        # gimbal pan into a bearing in the map's frame. Two copies of that sign would
        # be two chances to get it backwards, and this file exists to catch exactly
        # the kind of drawing bug that looks right from either side.
        mapimg.draw_camera(canvas, to_pixels, self.x, self.y, self.heading,
                           -self.pan, CAMERA_FOV_DEG,
                           half_extent_m * mapimg.CAMERA_REACH)

        forward = (math.cos(self.heading), math.sin(self.heading))
        side = (-math.sin(self.heading), math.cos(self.heading))

        def offset(along: float, across: float):
            return to_pixels(self.x + forward[0] * along + side[0] * across,
                             self.y + forward[1] * along + side[1] * across)

        canvas.triangle(offset(0.30, 0.0), offset(-0.15, 0.16), offset(-0.15, -0.16),
                        mapimg.C_ROVER)
        centre = half * scale
        canvas.disc(centre, centre, max(1.0, scale * 0.5), mapimg.C_ANCHOR)

        facing = ("Up the page is the direction the rover is facing now" if rover_up
                  else "Up the page is the direction the rover started facing")
        caption = (f"An invented top-down map of roughly {2 * half_extent_m:.0f} by "
                   f"{2 * half_extent_m:.0f} metres. {facing} and the rover's left is "
                   f"to the left. The red triangle is the rover and its tip points the "
                   f"way it is facing, with a yellow dot at its exact position, and "
                   f"the blue line is the path it has driven. "
                   + mapimg.camera_caption(-self.pan, CAMERA_FOV_DEG)
                   + " Nothing in it was measured.")
        return mapimg.png_rgb(canvas.rows), caption

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
                             "For exercising a client -- drive_console.py, or a "
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
