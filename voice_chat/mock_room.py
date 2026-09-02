"""The room the mock rover drives in, and the map it draws of it.

An invented rectangular room with legs of furniture in it, a doorway, and enough
of a planner to refuse a route that would drive through something. None of it is
a model of anywhere real: the point is that a console driving this sees the same
*shapes* of answer it would see from the rover, refusals included, with no rover
present.

`map_png` and `show_map` render through the real `mapimg`, so the picture the
console draws from here is drawn by the code that draws the rover's own maps. It
is imported inside the methods that use it, as it was before this split, because
it lives in `lidar_slam/` and the path to it is worked out at the call.

Split from mock_rover.py because this is the half with the arithmetic in it --
ranges, plans, and what is blocked. What is left there is the tool surface, which
is a list of small answers, and the dispatch that picks between them.
"""
from __future__ import annotations

import base64
import math
import os
import sys
import time
from typing import Any

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import prompts


# The invented room, in metres from wherever the rover started, with forward as +x
# and left as +y -- the frame lidar_slam works in. A table sits in front and to
# both sides of it, because a table is the thing the rover is asked to go round and
# the interesting property of one is that a lidar sees four thin legs and no top.
ROOM_FORWARD_M, ROOM_BACK_M = 2.5, 1.5
ROOM_LEFT_M, ROOM_RIGHT_M = 2.0, 2.0

CAMERA_FOV_DEG = prompts._literal(prompts.ROVER_NAV, "CAMERA_FOV_DEG")


LEGS = ((1.2, 0.4), (1.2, -0.4), (2.0, 0.4), (2.0, -0.4))


LEG_RADIUS_M = 0.05


# The daemon's own map limits, read out of its source the same way the schemas are.
# A mock that had its own copy of these would drift from the rover, and the whole
# point of clamping here is that a client which shows what it *got* can be tested
# against a rover that says no.
MAP_MAX_HALF_EXTENT_M = prompts._literal(prompts.ROVER_NAV, "MAP_MAX_HALF_EXTENT_M")


MAP_MAX_PIXELS = prompts._literal(prompts.ROVER_NAV, "MAP_MAX_PIXELS")


MAP_MAX_SCALE = prompts._literal(prompts.ROVER_NAV, "MAP_MAX_SCALE")


MAP_MIN_PIXELS = prompts._literal(prompts.ROVER_NAV, "MAP_MIN_PIXELS")


MAP_PIXELS = prompts._literal(prompts.ROVER_NAV, "MAP_PIXELS")


MAP_POINT_CLEAR_M = prompts._literal(prompts.ROVER_NAV, "MAP_POINT_CLEAR_M")


MAP_POINT_HINT = prompts._literal(prompts.ROVER_NAV, "MAP_POINT_HINT")


MAP_POINT_MAX_AGE_S = prompts._literal(prompts.ROVER_NAV, "MAP_POINT_MAX_AGE_S")


# The longest single route the mock will accept. This used to be read out of the
# rover's own navigator so the two could not drift apart; that navigator is gone
# and Nav2 has no equivalent ceiling, so this is now simply the mock's own limit,
# kept because a console that never sees a refusal is a console whose refusal path
# is untested.
MAX_GOTO_M = 15.0


MAX_RANGE_M = 12.0


STANDOFF_M = 0.30          # the real navigator's rule, mirrored here


def _length(path) -> float:
    """How long a route is: the sum of its legs."""
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(path, path[1:]))


def _wrap(radians: float) -> float:
    return (radians + math.pi) % (2 * math.pi) - math.pi


def _fraction(value: Any, what: str) -> float:
    """A place on the map picture, 0 to 1. Refused outside it, as the rover does."""
    if value is None:
        raise ValueError(f"{what} is missing; a place on the map picture needs "
                         f"both across and down")
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise ValueError(f"{what} must be a number")
    number = float(value)
    if not 0.0 <= number <= 1.0:
        raise ValueError(
            f"{what} is a fraction of the map picture, from 0 at one edge to 1 at "
            f"the other, and {number:g} is off the picture")
    return number


class RoverRoom:
    """The half of the mock rover that is a place rather than a tool."""

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
