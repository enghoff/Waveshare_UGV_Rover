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
MAP_MAX_HALF_EXTENT_M = prompts._literal(prompts.ROVER_NAV, "MAP_MAX_HALF_EXTENT_M")
MAP_MAX_SCALE = prompts._literal(prompts.ROVER_NAV, "MAP_MAX_SCALE")
MAP_MIN_PIXELS = prompts._literal(prompts.ROVER_NAV, "MAP_MIN_PIXELS")
MAP_MAX_PIXELS = prompts._literal(prompts.ROVER_NAV, "MAP_MAX_PIXELS")
MAP_PIXELS = prompts._literal(prompts.ROVER_NAV, "MAP_PIXELS")
CAMERA_FOV_DEG = prompts._literal(prompts.ROVER_NAV, "CAMERA_FOV_DEG")
MAP_POINT_MAX_AGE_S = prompts._literal(prompts.ROVER_NAV, "MAP_POINT_MAX_AGE_S")
MAP_POINT_CLEAR_M = prompts._literal(prompts.ROVER_NAV, "MAP_POINT_CLEAR_M")
MAP_POINT_HINT = prompts._literal(prompts.ROVER_NAV, "MAP_POINT_HINT")
# The longest single route the mock will accept. This used to be read out of the
# rover's own navigator so the two could not drift apart; that navigator is gone
# and Nav2 has no equivalent ceiling, so this is now simply the mock's own limit,
# kept because a console that never sees a refusal is a console whose refusal path
# is untested.
MAX_GOTO_M = 15.0


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


def _length(path) -> float:
    """How long a route is: the sum of its legs."""
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(path, path[1:]))


def _wrap(radians: float) -> float:
    return (radians + math.pi) % (2 * math.pi) - math.pi


def _fraction(value: Any, what: str) -> float:
    """A place on the map picture, 0 to 1. Refused outside it, as the rover does."""
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise ValueError(f"{what} must be a number")
    number = float(value)
    if not 0.0 <= number <= 1.0:
        raise ValueError(
            f"{what} is a fraction of the map picture, from 0 at one edge to 1 at "
            f"the other, and {number:g} is off the picture")
    return number

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
    # that is measured with ros_nav/calibrate_chassis.py, on the rover.

    def drive(self, arguments: dict[str, Any]) -> dict[str, Any]:
        distance = float(arguments.get("distance_m", 0.5))
        speed = float(arguments.get("speed_ms") or 0.2)
        self._begin("drive", {"distance_m": distance, "speed_ms": speed}, "driving")
        if distance <= 0.0:
            return self._say_end("blocked", "distance_m has to be positive",
                                 {"ok": False,
                                  "error": "distance_m has to be positive"})

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
        self._say_end(reason, detail)
        return {"ok": True, "reason": reason, "travelled_m": round(travelled, 3),
                "turned_deg": 0.0,
                **({"detail": detail} if detail else {}),
                **self._nav_context(speed)}

    def drive_to(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Around the table, not through it -- the point of the invented room.

        Takes a place either way the daemon does: `ahead_m`/`left_m` from where the
        rover is standing, or `x_m`/`y_m` as a point on the map. A console that
        relies on the second against the rover -- which the drive console does for
        every tap, so that a click keeps its meaning while the rover is still moving
        -- has to be able to rely on it here.
        """
        speed = float(arguments.get("speed_ms") or 0.2)
        x_m, y_m = arguments.get("x_m"), arguments.get("y_m")
        if (x_m is None) != (y_m is None):
            # Refused here because it is refused on the rover, and a mock that is
            # more forgiving than the thing it stands in for is a mock that hides
            # the client bug it exists to catch. Half a coordinate read as an
            # offset of zero is "already there", which is the wrong answer given
            # confidently.
            return {"ok": False, "error": "a place on the map needs both x_m and "
                                          "y_m; one on its own is not a place"}
        if x_m is not None and y_m is not None:
            target = (float(x_m), float(y_m))
            asked = {"x_m": round(target[0], 2), "y_m": round(target[1], 2)}
            range_m = math.hypot(target[0] - self.x, target[1] - self.y)
        else:
            ahead = float(arguments.get("ahead_m", 0.0) or 0.0)
            left = float(arguments.get("left_m", 0.0) or 0.0)
            asked = {"ahead_m": round(ahead, 2), "left_m": round(left, 2)}
            range_m = math.hypot(ahead, left)
            target = (
                self.x + ahead * math.cos(self.heading) - left * math.sin(self.heading),
                self.y + ahead * math.sin(self.heading) + left * math.cos(self.heading))
        self._begin("drive_to", asked, "planning")
        if range_m < 0.08:
            self._say_end("arrived", "already there")
            return {"ok": True, "reason": "arrived", "travelled_m": 0.0,
                    "turned_deg": 0.0, **self._nav_context(speed)}
        if range_m > MAX_GOTO_M:
            why = (f"that is {range_m:.1f} m away and a single route is "
                   f"capped at {MAX_GOTO_M:.0f} m")
            self._say_end("blocked", why)
            return {"ok": False, "error": why}

        start_heading = self.heading
        travelled, replans = 0.0, 0
        last_why = "no clear route through what the lidar has seen"
        while replans <= 8:
            path, last_why = self._plan_to(target)
            if not path:
                break
            self._say("driving", route_m=round(_length(path), 2),
                      waypoints=len(path), replans=replans)
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
                self._say_end("arrived", extra)
                return {"ok": True, "reason": "arrived",
                        "travelled_m": round(travelled, 3),
                        "turned_deg": round(turned, 1),
                        **({"detail": extra} if extra else {}),
                        **self._nav_context(speed)}
            replans += 1
            if not blocked:
                last_why = "the route did not reach that place"
            self._say("replanning", last_why, replans=replans,
                      route_m=None, waypoints=None)
        turned = math.degrees(_wrap(self.heading - start_heading))
        self._say_end("blocked", last_why)
        return {"ok": False, "reason": "blocked",
                "travelled_m": round(travelled, 3),
                "turned_deg": round(turned, 1),
                "detail": last_why, **self._nav_context(speed)}

    def _clear_run(self, a: tuple[float, float], b: tuple[float, float]) -> bool:
        """Can the rover go straight from a to b without fouling anything?

        A leg counts as fouled if the segment passes within the standoff of it, and
        the room counts as fouled if either end is outside it. Distance from a point
        to a segment, clamped to the segment, is the whole of the test.
        """
        for point in (a, b):
            if not (-ROOM_BACK_M + STANDOFF_M < point[0] < ROOM_FORWARD_M - STANDOFF_M
                    and -ROOM_RIGHT_M + STANDOFF_M < point[1] < ROOM_LEFT_M - STANDOFF_M):
                return False
        ex, ey = b[0] - a[0], b[1] - a[1]
        span = ex * ex + ey * ey
        for leg_x, leg_y in LEGS:
            if span < 1e-12:
                near = a
            else:
                t = ((leg_x - a[0]) * ex + (leg_y - a[1]) * ey) / span
                t = max(0.0, min(1.0, t))
                near = (a[0] + t * ex, a[1] + t * ey)
            if math.hypot(leg_x - near[0], leg_y - near[1]) < LEG_RADIUS_M + STANDOFF_M:
                return False
        return True

    def _plan_to(self, target: tuple[float, float]):
        """A route through the invented room, as (waypoints, why-not).

        Deliberately not a planner. This used to call the rover's own A* over an
        occupancy grid built from the room above, and that made sense while the
        rover had one; Nav2 plans on the rover now, and reaching across the
        repository for 900 lines of grid search so that a mock can draw three
        waypoints is paying for precision nothing here measures.

        What the tests actually need of a route is that it exists when the way is
        open, that it bends round the table rather than through it, and that it
        comes back as None with a sentence when there is no way at all -- because
        those are the three things a console has to render. So: go straight if the
        straight line is clear, and otherwise try one waypoint out to the side of
        each leg in the way, nearest first.
        """
        start = (self.x, self.y)
        if self._clear_run(start, target):
            return [start, target], ""

        # One detour waypoint per candidate, placed abeam a leg at a comfortable
        # radius. Both sides of every leg are tried and the shortest route that
        # works wins, which is a visibility graph with exactly one hop in it.
        stand = LEG_RADIUS_M + STANDOFF_M + 0.10
        best, best_len = None, float("inf")
        for leg_x, leg_y in LEGS:
            for angle in range(0, 360, 30):
                theta = math.radians(angle)
                via = (leg_x + stand * math.cos(theta),
                       leg_y + stand * math.sin(theta))
                if not (self._clear_run(start, via) and self._clear_run(via, target)):
                    continue
                total = _length([start, via, target])
                if total < best_len:
                    best, best_len = [start, via, target], total
        if best is not None:
            return best, ""

        # Nothing reaches it. Which of the two reasons it is matters to the caller:
        # a place inside the table is a different conversation from a place behind
        # it that this router is simply not clever enough to reach.
        if not self._clear_run(target, target):
            return None, "there is no room to stand there"
        return None, "no clear route through what the lidar has seen"

    def drive_to_map_point(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """A place named as a fraction of the last map picture, driven to.

        Refused here for the same four reasons the rover refuses it -- no picture
        taken, a picture too old to still be in front of the model, a fraction off
        the edge of it, and a point the rover could not stand on -- because a
        conversation that gets past this mock and then hits one of those against the
        rover is a conversation this mock failed to test. The conversion is the
        rover's own `mapimg.tap_to_point`, so a fraction means the same place here
        as it does there; what differs is only how the room is decided, which is a
        formula here and an occupancy grid there.
        """
        shown = self._map_shown
        if shown is None:
            return {"ok": False,
                    "error": "you have not looked at the map yet, so there is no "
                             "picture to point at. Take one with show_map first, "
                             "then point at a place on it"}
        age = time.monotonic() - shown["at"]
        if age > MAP_POINT_MAX_AGE_S:
            return {"ok": False,
                    "error": "the map you are pointing at was drawn %.0f seconds "
                             "ago and is not in front of you any more, so a place "
                             "on it would be a guess. Take a fresh one with "
                             "show_map and point at that" % age}
        try:
            across = _fraction(arguments.get("across"), "across")
            down = _fraction(arguments.get("down"), "down")
        except ValueError as error:
            return {"ok": False, "error": str(error)}

        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), "lidar_slam"))
        import mapimg

        pixels = shown["pixels"]
        x_m, y_m = mapimg.tap_to_point(
            across * pixels, down * pixels, shown["half_extent_m"], shown["scale"],
            shown["resolution_m"], rover_up=shown["rover_up"], pose=shown["pose"])

        px, py, heading = shown["pose"]
        dx, dy = x_m - px, y_m - py
        cos, sin = math.cos(heading), math.sin(heading)
        pointed = {"ahead_m": round(dx * cos + dy * sin, 2),
                   "left_m": round(-dx * sin + dy * cos, 2),
                   "range_m": round(math.hypot(dx, dy), 2),
                   "x_m": round(x_m, 2), "y_m": round(y_m, 2)}
        blocked = self._point_blocked(x_m, y_m)
        if blocked is not None:
            return {"ok": False, "error": blocked, "pointed_at": pointed}
        result = self.drive_to({"x_m": x_m, "y_m": y_m,
                                **({"speed_ms": arguments["speed_ms"]}
                                   if arguments.get("speed_ms") is not None
                                   else {})})
        return {**result, "pointed_at": pointed}

    def _point_blocked(self, x_m: float, y_m: float) -> str | None:
        """Why the rover could not stand here, in the map's own vocabulary.

        The same two refusals the rover makes off its occupancy grid, decided from
        the formula this room is instead: a wall or a table leg is what comes out
        black, and anything outside the room is what comes out grey. The wording is
        the rover's, because it is the wording a model has to be able to act on.
        """
        near_solid = (
            min(abs(x_m + ROOM_BACK_M), abs(x_m - ROOM_FORWARD_M),
                abs(y_m + ROOM_RIGHT_M), abs(y_m - ROOM_LEFT_M)) < MAP_POINT_CLEAR_M
            or any(math.hypot(x_m - lx, y_m - ly) < LEG_RADIUS_M + MAP_POINT_CLEAR_M
                   for lx, ly in LEGS))
        inside = (-ROOM_BACK_M < x_m < ROOM_FORWARD_M
                  and -ROOM_RIGHT_M < y_m < ROOM_LEFT_M)
        if near_solid and inside:
            return ("that point is on or within %.0f cm of something solid -- black "
                    "on the map -- so the rover cannot stand there. Point at green "
                    "floor instead" % (MAP_POINT_CLEAR_M * 100))
        if not inside:
            return ("that point is grey on the map, which means the rover has never "
                    "seen it rather than that it is empty, so there is no route to "
                    "plan there. Point at green floor instead")
        return None

    def turn_in_place(self, arguments: dict[str, Any]) -> dict[str, Any]:
        angle = float(arguments.get("angle_deg", 0.0))
        self._begin("turn_in_place", {"angle_deg": round(angle, 1)}, "turning")
        self.heading = _wrap(self.heading + math.radians(angle))
        self._say_end("arrived", "")
        return {"ok": True, "reason": "arrived", "travelled_m": 0.0,
                "turned_deg": round(angle, 1), **self._nav_context(0.0)}

    def stop_driving(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        return {"ok": True, "stopped": True, "latched": False}

    def describe_surroundings(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        return {"ok": True, **self._nav_context(0.0), **self._described()}

    def nav_status(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """The engineering numbers, as the real daemon's control call returns them.

        `since_seq` included, since a client that relies on it against the rover
        has to be able to rely on it here."""
        since = arguments.get("since_seq")
        move = ({"move": self.report.snapshot(
                    since_seq=None if since is None else int(since))}
                if self.report is not None else {})
        return {"ok": True, "driving": False, "estop": False, **move,
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

    def show_map(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """The model's version: the picture goes to the vision host, not the reply.

        The same two knobs as the console -- how many metres across, and how big a
        picture -- because a mock that ignored them would let a conversation that
        asked for a floor view pass here and fail against the rover. `across_m` is
        halved into `map_png`'s half-extent; nothing asked for is a room at the
        default picture size, same as the real handler.
        """
        if self.vision is None:
            return {"ok": False, "error": "there is nowhere to send a picture"}
        mapped: dict[str, Any] = {}
        if arguments.get("pixels") is not None:
            mapped["pixels"] = arguments["pixels"]
        if arguments.get("across_m") is not None:
            mapped["half_extent_m"] = float(arguments["across_m"]) / 2.0
        body = self.map_png(mapped)
        png = base64.b64decode(body["png_base64"])
        _name, error = self._post(png, "image/png")
        # What `drive_to_map_point` needs to turn a place on this picture back into
        # a place in the room, taken from what was actually drawn rather than what
        # was asked for -- whole cells at whole pixels do not reach every size, and
        # a fraction of the wrong width is a point in the wrong place.
        self._map_shown = {
            "half_extent_m": body["half_extent_m"], "scale": body["scale"],
            "rover_up": body["rover_up"], "resolution_m": 0.05,
            "pixels": body["pixels"],
            "pose": (self.x, self.y, self.heading), "at": time.monotonic(),
        }
        result = {"ok": True, "caption": body["caption"] + MAP_POINT_HINT,
                  **self._described()}
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
        import numpy as np
        import mapimg

        res = 0.05
        half = max(8, int(half_extent_m / res))
        size = half * 2 + 1
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
        # over the whole picture. Occupancy is coded the way the real renderer
        # reads it, then coloured by `colour_occupancy`, so reachable floor is
        # green here for the same reason it is green on the rover.
        wall = res * 1.5
        shown = np.zeros((size, size), dtype=np.int8)
        for iy in range(size):
            for ix in range(size):
                wx, wy = at_cell(iy, ix)
                inside = (-ROOM_BACK_M < wx < ROOM_FORWARD_M
                          and -ROOM_RIGHT_M < wy < ROOM_LEFT_M)
                on_wall = (min(abs(wx + ROOM_BACK_M), abs(wx - ROOM_FORWARD_M),
                               abs(wy + ROOM_RIGHT_M), abs(wy - ROOM_LEFT_M)) < wall)
                near_leg = any((wx - lx) ** 2 + (wy - ly) ** 2
                               <= (LEG_RADIUS_M + res) ** 2 for lx, ly in LEGS)
                row, col = size - 1 - ix, size - 1 - iy
                if near_leg or on_wall:
                    shown[row, col] = 60
                elif inside:
                    shown[row, col] = -1
        rgb = mapimg.colour_occupancy(shown, occupied_at=20, origin=(half, half))
        big = np.repeat(np.repeat(rgb, scale, axis=0), scale, axis=1)
        canvas = mapimg.Canvas.over([bytearray(row.tobytes()) for row in big], 3)

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
                   + " Green is empty space the rover can reach from where it is. "
                     "Nothing in it was measured.")
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
