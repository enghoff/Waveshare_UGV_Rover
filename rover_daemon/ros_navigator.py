"""The daemon's driving tools, backed by ROS 2.

There used to be a `Navigator` under `lidar_slam/` that owned the lidar, a scan
matcher and a 10 Hz control loop, and the daemon's driving tools were thin
wrappers over it. All three of those jobs belong to the ROS 2 stack now --
slam_toolbox maps with loop closure, Nav2 plans and follows -- so that navigator
has been deleted and this took its place: the same methods, returning the same
shapes, with a socket where the control loop used to be.

What survived the deletion is the interface, because the daemon's tools and its
two consoles already speak it: `Outcome` and `MoveReport` still live in
`lidar_slam/nav_types.py` and are imported from there rather than reinvented, and
the field names in `nav_status` are unchanged.

**Why the map is drawn here and not on the ROS side.** The bridge hands over the
occupancy grid as the bytes it arrived as, and this turns them into a picture with
[lidar_slam/mapimg.py](../lidar_slam/mapimg.py), which is the renderer that was
already drawing this rover's maps -- the rover's own arrow, the track it has
driven, the camera's cone, and the caption that tells a model what it is looking
at. Rendering on the ROS side would have meant a second renderer, and two
renderers become two different pictures of one room.

**What is genuinely lost.** A pose graph has no per-revolution match score, and a
velocity controller has no chosen steering arc, so `nav_status` reports None for
both and the consoles show a dash. What replaces the first is better rather than
worse: `position_trusted` now means "slam_toolbox is still publishing where we
are", and the thing it was really watching for was the mapper having stopped.
"""
from __future__ import annotations

import base64
import json
import math
import os
import socket
import sys
import threading
import zlib
from typing import Any

# lidar_slam/ is a sibling in the repository and a subdirectory of the rover's
# flat ~/ugv. The same two-layout dance rover_daemon.py does for the old
# navigator, and for the same reason: a deployment that moves must not silently
# lose the driving tools.
_HERE = os.path.dirname(os.path.abspath(__file__))
for _candidate in (os.path.join(_HERE, "..", "lidar_slam"),
                   os.path.join(_HERE, "lidar_slam")):
    if os.path.isdir(_candidate):
        _candidate = os.path.abspath(_candidate)
        if _candidate not in sys.path:
            sys.path.insert(0, _candidate)

from nav_types import MoveReport, Outcome                       # noqa: E402

HOST = "127.0.0.1"
PORT = 8773

# How long to wait for each kind of request. A status poll that has not answered
# in a second has found a bridge that is wedged rather than busy, and the console
# asking three times a second must not stack up behind it. The map is slower
# because it is tens of kilobytes of grid being compressed.
STATUS_TIMEOUT_S = 2.0
MAP_TIMEOUT_S = 8.0
STOP_TIMEOUT_S = 3.0
# A move is answered by Nav2 when Nav2 is done, and a long route across a house
# legitimately takes minutes. This is not a schedule -- the bridge has its own
# time allowance per goal, worked out from the goal -- it is the point at which a
# silent socket is a dead bridge rather than a patient one.
MOVE_QUIET_S = 240.0

# --- how a ROS occupancy grid becomes the grid mapimg expects -------------------
# The renderer reads four states off an int8 array: below zero is free, zero is
# never-seen, between zero and `occupied_at` is seen-but-uncertain, and at or above
# it is occupied. ROS says the same thing on a different scale -- -1 unknown, and
# 0 to 100 probability -- so these are the two thresholds that translate one into
# the other. slam_toolbox emits mostly 0 and 100, having already made up its mind,
# so the middle band is rare and is drawn as the dim shade it is.
ROS_FREE_BELOW = 25
ROS_OCCUPIED_AT = 65
GRID_FREE = -100
GRID_DIM = 10
GRID_OCCUPIED = 100
GRID_OCCUPIED_AT = 50
# 800 cells at 5 cm is 40 m across with the rover's starting point in the middle,
# which is the grid the daemon's own SLAM presented -- so the console's zoom
# buttons and `MAP_MAX_HALF_EXTENT_M` mean what they always meant.
GRID_CELLS = 800
DEFAULT_RESOLUTION_M = 0.05


class _Config:
    """The three fields of `slam2d.Config` that the renderer reads.

    Not the real structure, which is a ctypes mirror of the C library's and would
    mean loading `libslam2d.so` to carry three numbers across a function call.
    """

    def __init__(self, resolution_m: float, grid_cells: int,
                 occupied_at: int) -> None:
        self.resolution_m = resolution_m
        self.grid_cells = grid_cells
        self.occupied_at = occupied_at


class _GridSlam:
    """A ROS occupancy grid wearing enough of `Slam2D` to be rendered.

    `mapimg.render` asks a map object for five things: a lock, the grid, the pose,
    and three numbers of configuration. All five are in the message slam_toolbox
    published. It used to ask for the live scan as well and then not use it, which
    meant carrying an empty list here to be fetched and dropped; the renderer no
    longer asks.
    """

    def __init__(self, payload: dict[str, Any] | None = None) -> None:
        self.lock = threading.Lock()
        if not payload or not payload.get("data"):
            self.config = _Config(DEFAULT_RESOLUTION_M, GRID_CELLS,
                                  GRID_OCCUPIED_AT)
            self.pose = (0.0, 0.0, 0.0)
            self._grid = None
            self.trail: tuple = ()
            return

        cells = int(payload.get("cells") or GRID_CELLS)
        resolution = float(payload.get("resolution_m") or DEFAULT_RESOLUTION_M)
        self.config = _Config(resolution, cells, GRID_OCCUPIED_AT)
        where = payload.get("pose") or {}
        self.pose = (float(where.get("x_m") or 0.0),
                     float(where.get("y_m") or 0.0),
                     math.radians(float(where.get("heading_deg") or 0.0)))
        self.trail = tuple((float(x), float(y))
                           for x, y in payload.get("trail") or ())
        self._grid = self._place(payload)

    def _place(self, payload: dict[str, Any]):
        """The ROS map, recoded and dropped into the square grid at its own place.

        Two conversions, and both are the kind that is invisible when wrong. The
        axes first: ROS packs a row at a time with x running fastest, so the array
        it reshapes to is indexed [y, x], while the renderer indexes [forward,
        left]. Map +x *is* forward and map +y *is* left on this rover -- the map
        frame is where it started -- so the axes agree once the array is
        transposed, and nothing else needs turning.

        Then the offset. slam_toolbox's origin is wherever the map happens to have
        grown to and is not a whole number of cells from anywhere, so placing it
        rounds to the nearest cell. That is up to two and a half centimetres, half
        the resolution of the thing being drawn, and it is the reason this rounds
        rather than truncating: truncating would bias every map the same way.
        """
        import numpy as np

        width = int(payload["width"])
        height = int(payload["height"])
        cells = self.config.grid_cells
        resolution = self.config.resolution_m
        raw = zlib.decompress(base64.b64decode(payload["data"]))
        occupancy = np.frombuffer(raw, dtype=np.int8)
        if occupancy.size != width * height:
            raise ValueError("the map is %d cells but says it is %dx%d"
                             % (occupancy.size, width, height))
        occupancy = occupancy.reshape(height, width).T          # -> [x, y]

        coded = np.zeros(occupancy.shape, dtype=np.int8)
        coded[(occupancy >= 0) & (occupancy < ROS_FREE_BELOW)] = GRID_FREE
        coded[(occupancy >= ROS_FREE_BELOW)
              & (occupancy < ROS_OCCUPIED_AT)] = GRID_DIM
        coded[occupancy >= ROS_OCCUPIED_AT] = GRID_OCCUPIED

        grid = np.zeros((cells, cells), dtype=np.int8)
        ox = int(round(float(payload["origin_x_m"]) / resolution)) + cells // 2
        oy = int(round(float(payload["origin_y_m"]) / resolution)) + cells // 2
        # Clipped both ends, because a map that has grown past 40 m across is a
        # real thing and the alternative to clipping is an exception in the middle
        # of drawing a picture somebody asked for.
        sx, sy = max(0, -ox), max(0, -oy)
        ex = min(width, cells - ox)
        ey = min(height, cells - oy)
        if ex > sx and ey > sy:
            grid[ox + sx:ox + ex, oy + sy:oy + ey] = coded[sx:ex, sy:ey]
        return grid

    def grid(self):
        import numpy as np

        if self._grid is None:
            # Never-seen everywhere, which is what an empty map is. Allocated here
            # rather than in the constructor so that the common case -- somebody
            # reading `config.resolution_m` off the placeholder -- costs nothing.
            return np.zeros((self.config.grid_cells, self.config.grid_cells),
                            dtype="int8")
        return self._grid


class RosNavigator:
    """Nav2 and slam_toolbox, presented as the navigator the daemon already has.

    Every request is its own short-lived connection to the bridge, which is worth
    a word because the obvious design is one kept open. Two things fall out of the
    per-request choice and both matter here: a stop can never be stuck behind a
    map that is still being compressed on the other side, and a bridge that is
    down is discovered instantly as a refused connection rather than as a
    persistent socket that has quietly stopped answering. Loopback connections
    cost tens of microseconds, and the busiest caller is a console polling three
    times a second.

    The exception is a move, which holds its connection for as long as the move
    lasts and reads the progress lines Nav2's feedback becomes.
    """

    def __init__(self, host: str = HOST, port: int = PORT) -> None:
        self.host = host
        self.port = port

        #: The wheels have one owner at a time. Refused rather than queued, for
        #: the reason the old navigator refused: a move that waited its turn would
        #: begin by driving to somewhere the caller ahead of it has since made
        #: wrong. The bridge holds one of these too, against a second client.
        self._move_mutex = threading.Lock()
        #: The running commentary, which is the only account of a move that
        #: arrives before the move is over. See MoveReport.
        self.report = MoveReport()
        #: The last map fetched, kept because `map_png`'s caller reads the
        #: resolution off it before asking for the picture.
        self._slam = _GridSlam()
        self._lidar_port: str | None = None
        #: How many times somebody has replugged the lidar in software from here,
        #: and what came of the last one. The ROS lidar node has no recovery
        #: ladder of its own -- it simply reopens the port every three seconds --
        #: so this is the button and not an automatic act.
        self._resets = 0
        self._reset_note = ""
        self._reachable = False

    # --- lifecycle ------------------------------------------------------------
    def start(self) -> None:
        """Ask the bridge once, so the startup banner can say whether ROS is up.

        Deliberately not fatal and deliberately not retried here. The ROS stack
        and this daemon are both started by the same crontab and the stack takes
        the best part of a minute to come up, so at boot this will always fail;
        what makes the tools work anyway is that every one of them connects when
        it is called.
        """
        answer = self.ask({"op": "status"}, STATUS_TIMEOUT_S)
        self._reachable = bool(answer.get("ok"))
        if self._reachable:
            self._lidar_port = answer.get("lidar_port")

    def close(self) -> None:
        """Stop the wheels on the way down, and do not wait long for it.

        The daemon calls this while shutting down, so a bridge that has already
        gone is the ordinary case rather than a fault.
        """
        try:
            self.ask({"op": "stop"}, STOP_TIMEOUT_S)
        except Exception:
            pass

    @property
    def driving(self) -> bool:
        """True from the moment a move takes the wheels until it has let go.

        The mutex is the fact rather than a flag kept beside it, so this cannot
        drift out of step with what is actually running. Read by anything that has
        to behave differently while the rover is under way -- the lidar reset
        refuses, and face tracking stops sweeping -- neither of which is worth a
        callback into: a caller that asks each time it acts is never left holding a
        stale answer from a move that ended while it was busy.
        """
        return self._move_mutex.locked()

    @property
    def reachable(self) -> bool:
        """Whether the last thing said to the bridge was answered. For the startup
        banner, and not a health check: it is only as fresh as the last request."""
        return self._reachable

    @property
    def slam(self):
        return self._slam

    # --- talking to the bridge ------------------------------------------------
    def ask(self, request: dict[str, Any], timeout_s: float) -> dict[str, Any]:
        """One request, one reply, one connection. Never raises.

        An unreachable bridge is a sentence rather than an exception, because
        every caller here is a tool whose result goes into a model's context or
        onto a console panel, and both need words.
        """
        try:
            with socket.create_connection((self.host, self.port),
                                          timeout=timeout_s) as link:
                link.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                link.sendall(json.dumps(request).encode() + b"\n")
                with link.makefile("r") as stream:
                    line = stream.readline()
        except (OSError, socket.timeout) as error:
            self._reachable = False
            return {"ok": False, "error": self._down(error)}
        if not line:
            self._reachable = False
            return {"ok": False,
                    "error": "the ROS navigation bridge closed the connection "
                             "without answering"}
        try:
            reply = json.loads(line)
        except ValueError:
            return {"ok": False,
                    "error": "the ROS navigation bridge sent something that is "
                             "not JSON"}
        self._reachable = True
        return reply if isinstance(reply, dict) else {"ok": False,
                                                      "error": "not an object"}

    def _down(self, error: Exception) -> str:
        return ("the ROS navigation stack is not answering on %s:%d (%s). It is "
                "started from the crontab and takes about half a minute to come "
                "up; ~/ugv/ros_nav/restart.sh brings it back."
                % (self.host, self.port, error))

    # --- reading --------------------------------------------------------------
    def status(self, since_seq: int | None = None) -> dict[str, Any]:
        """Every number the driving half of the rover has, plus the move's own
        commentary, which lives on this side because it is this side that narrates
        a move as it runs."""
        answer = self.ask({"op": "status"}, STATUS_TIMEOUT_S)
        move = self.report.snapshot(since_seq)
        if not answer.get("ok"):
            # Shaped like a real status even when there is nothing behind it, so
            # that a console draws a panel of dashes and one honest sentence
            # instead of losing the rover entirely.
            return {"driving": False, "estop": False, "pose": None,
                    "speed_ms": None, "turn_dps": None, "clearance_m": None,
                    "steering_deg": None, "remaining_m": None,
                    "match_score": None, "position_trusted": False,
                    "mapping": False, "scans": None, "dropped_scans": None,
                    "pwm": None, "lidar_ok": False, "lidar_live": False,
                    "lidar_port": self._lidar_port, "scan_age_s": None,
                    "lidar_resets": self._resets,
                    "lidar_reset_note": self._reset_note,
                    "move": move, "nav_error": answer.get("error")}
        answer.pop("kind", None)
        answer.pop("ok", None)
        if answer.get("lidar_port"):
            self._lidar_port = answer["lidar_port"]
        answer["move"] = move
        answer["lidar_resets"] = self._resets
        answer["lidar_reset_note"] = self._reset_note
        return answer

    def describe(self) -> dict[str, Any]:
        """What is around the rover.

        Guaranteed to carry `text` and `clear_ahead_m` whatever happened, because
        the daemon puts both into the result of every move -- and a KeyError there
        would throw away the outcome of a drive that had already happened.
        """
        answer = self.ask({"op": "describe"}, STATUS_TIMEOUT_S)
        if not answer.get("ok"):
            return {"text": str(answer.get("error")
                                or "the rover cannot say what is around it"),
                    "clear_ahead_m": None, "position_trusted": False,
                    "lidar_ok": False, "walls": [], "objects": [], "gaps": []}
        answer.pop("kind", None)
        answer.pop("ok", None)
        answer.setdefault("clear_ahead_m", None)
        answer.setdefault("text", "nothing was said about the room")
        return answer

    def map_png(self, half_extent_m: float = 3.0, scale: int = 3,
                rover_up: bool = False, camera=None):
        """The map as PNG bytes and a caption, drawn by the renderer that has
        always drawn this rover's maps.

        Raises rather than returning a sentence, because both callers already turn
        an exception into an honest tool error and neither can do anything useful
        with half a picture. `ValueError` specifically: the daemon's dispatcher
        reports those as the sentence alone and everything else with the exception
        class in front of it, and "no map yet" is a reason rather than a bug.
        """
        import mapimg

        answer = self.ask({"op": "map"}, MAP_TIMEOUT_S)
        if not answer.get("ok"):
            raise ValueError(str(answer.get("error") or "there is no map yet"))
        self._slam = _GridSlam(answer)
        return mapimg.render(self._slam, half_extent_m, scale, self._slam.trail,
                             rover_up=rover_up, camera=camera)

    # --- writing --------------------------------------------------------------
    def stop(self, latch: bool = False) -> dict[str, Any]:
        """Stop now. Never refused, and answered even when the bridge is gone.

        A stop that reports failure is the worst reply this whole interface can
        give, because the person reading it has already decided the rover should
        not be moving. So a bridge that cannot be reached still comes back
        `stopped` -- and it is not a lie: the driver board stops itself when it
        stops hearing commands, and a bridge that is not answering is a bridge
        that is not sending any.
        """
        answer = self.ask({"op": "stop", "latch": bool(latch)}, STOP_TIMEOUT_S)
        if self.report.snapshot().get("phase") not in ("idle", "ended"):
            self.report.say("stopping", "a stop was asked for"
                                        + (" and latched" if latch else ""))
        if not answer.get("ok"):
            return {"stopped": True, "latched": False,
                    "note": "the ROS stack is not answering, so nothing was told "
                            "to stop -- but the driver board stops itself within "
                            "half a second of the commands ceasing, which they "
                            "have"}
        return {"stopped": True, "latched": bool(answer.get("latched"))}

    def clear_estop(self) -> dict[str, Any]:
        answer = self.ask({"op": "clear_estop"}, STOP_TIMEOUT_S)
        return {"latched": bool(answer.get("latched"))}

    def clear_map(self) -> dict[str, Any]:
        """Throw the pose graph away and start again where the rover stands.

        Worth keeping now that there is loop closure, and for a different reason
        than before. The old map drifted permanently and this was how a room that
        had come out of true with itself got fixed; a pose graph mostly fixes that
        by itself. What it cannot undo is a *wrong* loop closure -- two different
        corridors matched to each other -- and that is unrecoverable by driving,
        so the button stays.
        """
        answer = self.ask({"op": "clear_map"}, 15.0)
        if not answer.get("ok"):
            return {"cleared": False,
                    "reason": str(answer.get("reason")
                                  or answer.get("error") or "the map was kept")}
        return {"cleared": True, "reason": str(answer.get("reason") or "")}

    def reset_lidar(self) -> dict[str, Any]:
        """Replug the lidar's USB device in software.

        Done here rather than through the bridge, and that is not an oversight:
        resetting a USB device is an ioctl on a sysfs node, so it needs neither
        the serial port nor ROS, and the process that *does* hold the port is the
        one that must not be inside a blocking write when the device disappears.
        The ROS lidar node reopens the port by itself within three seconds, which
        is the whole of the recovery it needs.

        Refused while a move is running: the reset reaches the hub the lidar is
        under, which takes the camera and the OAK with it for a few seconds, and a
        rover that is driving should not go blind.
        """
        import usbreset

        if self.driving:
            return {"ok": False,
                    "error": "the rover is moving; the reset takes the camera "
                             "down with it, so stop first"}
        attempt = usbreset.revive()
        self._resets += 1
        self._reset_note = attempt.why
        return {"ok": bool(attempt.ok), "what": attempt.what,
                "why": attempt.why, "rung": attempt.rung,
                "rungs": attempt.rungs, "more": attempt.more,
                "resets": self._resets,
                "note": "the ROS lidar node reopens the port by itself, within "
                        "about three seconds of the device coming back"}

    # --- moves ----------------------------------------------------------------
    def drive(self, distance_m: float, speed_ms: float | None = None) -> Outcome:
        return self.move("drive", {"distance_m": distance_m},
                         {"op": "drive", "distance_m": float(distance_m),
                          "speed_ms": None if speed_ms is None
                                      else float(speed_ms)})

    def turn_in_place(self, angle_deg: float) -> Outcome:
        return self.move("turn_in_place", {"angle_deg": angle_deg},
                         {"op": "turn", "angle_deg": float(angle_deg)},
                         phase="turning")

    def drive_to(self, ahead_m: float | None = None, left_m: float | None = None,
                 x_m: float | None = None, y_m: float | None = None,
                 speed_ms: float | None = None) -> Outcome:
        """Somewhere on the map, given either as an offset or as a point.

        The offset is converted here rather than on the ROS side, because that is
        where the pose is: a click on the map names a point, and an offset names
        one relative to wherever the rover is at the moment the request is made.
        Converting late -- after the request has crossed the socket -- would move
        the destination by however far the rover travelled in between, which for a
        rover already driving is most of a metre.
        """
        if x_m is None:
            where = self.pose_now()
            if where is None:
                return Outcome("lost", 0.0, 0.0,
                               "nothing is publishing the rover's position, so "
                               "an offset from it has nowhere to start")
            px, py, yaw = where
            ahead = float(ahead_m or 0.0)
            left = float(left_m or 0.0)
            x_m = px + ahead * math.cos(yaw) - left * math.sin(yaw)
            y_m = py + ahead * math.sin(yaw) + left * math.cos(yaw)
            asked = {"ahead_m": ahead, "left_m": left}
        else:
            asked = {"x_m": float(x_m), "y_m": float(y_m or 0.0)}
        return self.move("drive_to", asked,
                         {"op": "goto", "x_m": float(x_m),
                          "y_m": float(y_m or 0.0)})

    def explore(self, budget_s: float | None = None,
                min_frontier_m: float | None = None) -> Outcome:
        """Map the rest of the room, by driving to the edges of what is mapped.

        One call that lasts minutes rather than seconds, and it is a move like
        any other here: it holds the same mutex, so anything else that would
        drive is refused as busy while it runs, and `stop` ends it. The choosing
        is on the ROS side because that is where the map is -- see `explore` in
        `ros_nav/nav_bridge.py`.

        The socket's quiet timeout is what makes this work at all without a
        special case: the bridge narrates every third of a second throughout,
        including while it is thinking about where to go next, so a run of ten
        minutes never looks like a connection that has died.
        """
        asked: dict[str, Any] = {}
        if budget_s is not None:
            asked["budget_s"] = float(budget_s)
        if min_frontier_m is not None:
            asked["min_frontier_m"] = float(min_frontier_m)
        return self.move("explore", asked, {"op": "explore", **asked},
                         phase="choosing")

    def pose_now(self) -> tuple[float, float, float] | None:
        """Where the rover is, in the map frame, for converting an offset."""
        answer = self.ask({"op": "status"}, STATUS_TIMEOUT_S)
        where = answer.get("pose") if answer.get("ok") else None
        if not where:
            return None
        return (float(where["x_m"]), float(where["y_m"]),
                math.radians(float(where["heading_deg"])))

    def move(self, kind: str, asked: dict[str, Any], request: dict[str, Any],
             phase: str = "planning") -> Outcome:
        """Send one move and narrate it until it ends.

        The connection is held for the whole move, which is what makes the
        commentary possible: Nav2's feedback becomes progress lines on this socket,
        each one a sentence in the `MoveReport` that a console is polling for. The
        move itself is answered by the last line, and nothing else on this socket
        matters.
        """
        if not self._move_mutex.acquire(blocking=False):
            return Outcome("busy", 0.0, 0.0, "a move is already running")
        try:
            self.report.begin(kind, asked, phase)
            return self.stream(request, phase)
        finally:
            self._move_mutex.release()

    def stream(self, request: dict[str, Any], phase: str) -> Outcome:
        """The socket half of a move: write once, then read until the outcome."""
        try:
            link = socket.create_connection((self.host, self.port), timeout=10.0)
        except OSError as error:
            self.report.finish("blocked", "the ROS stack is not answering")
            return Outcome("blocked", 0.0, 0.0, self._down(error))
        try:
            link.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            link.settimeout(MOVE_QUIET_S)
            link.sendall(json.dumps(request).encode() + b"\n")
            with link.makefile("r") as stream:
                return self.follow(stream)
        except (OSError, socket.timeout) as error:
            # The bridge stopped talking in the middle of a move, so the rover may
            # still be driving. Say stop on a fresh connection before answering.
            self.stop()
            self.report.finish("failed", "the ROS stack went quiet mid-move")
            return Outcome("failed", 0.0, 0.0,
                           "the ROS navigation bridge went quiet during the move "
                           "(%s); a stop was sent" % error)
        finally:
            try:
                link.close()
            except OSError:
                pass

    def follow(self, stream) -> Outcome:
        """Turn the bridge's progress lines into a commentary and an `Outcome`.

        Nav2's recoveries are reported as replans, which is what they are from
        outside: the navigator has decided the route it had is not working and is
        doing something else. Saying so is the difference between a console that
        shows a rover thinking and one that shows a rover stuck.
        """
        replans = 0
        announced = False
        for raw in stream:
            raw = raw.strip()
            if not raw:
                continue
            try:
                line = json.loads(raw)
            except ValueError:
                continue
            kind = line.get("kind")
            if kind == "progress":
                recoveries = int(line.get("recoveries") or 0)
                if recoveries > replans:
                    replans = recoveries
                    self.report.say("replanning",
                                    "Nav2 is trying to recover", replans=replans)
                    continue
                # A new frontier is a new route, so the announcement is armed
                # again. Without this an `explore` announces the route to its
                # first frontier and then drives eight more in silence, because
                # `announced` was written for a move that has one route in it.
                if line.get("phase") == "choosing":
                    announced = False
                fields: dict[str, Any] = {}
                # The route, said once. `route_m` is what the console turns into
                # "route accepted: 2.4 m through 48 waypoints", so it is the length
                # of the route when it was accepted -- not how much of it is left,
                # which has its own row on the panel. Repeating it every third of a
                # second would have the rover announcing a fresh route all the way
                # down the old one.
                # ...and only when there is one. Nav2 reports zero distance and no
                # waypoints while its planner is failing, and "route accepted:
                # 0.00 m through 0 waypoints" is a sentence that claims the
                # opposite of what is happening. Observed on a goal the planner
                # refused outright, where the console cheerfully announced a route
                # ten times over while the rover went nowhere.
                if (not announced and float(line.get("remaining_m") or 0.0) > 0.0
                        and int(line.get("waypoints") or 0) > 0):
                    announced = True
                    fields["route_m"] = float(line["remaining_m"])
                    fields["waypoints"] = int(line["waypoints"])
                # How much unmapped edge is left, carried through unchanged so
                # that a panel watching an explore can say "9 left" rather than
                # only "driving". Absent from every other move, which is why it
                # is forwarded when present rather than always.
                if line.get("frontiers_left") is not None:
                    fields["frontiers_left"] = int(line["frontiers_left"])
                self.report.say(str(line.get("phase") or "driving"),
                                str(line.get("why") or ""), **fields)
            elif kind == "outcome":
                reason = str(line.get("reason") or "failed")
                detail = str(line.get("detail") or "")
                self.report.finish(reason, detail)
                return Outcome(reason, float(line.get("travelled_m") or 0.0),
                               float(line.get("turned_deg") or 0.0), detail)
            elif kind == "reply" and not line.get("ok"):
                detail = str(line.get("error") or "the move was refused")
                self.report.finish("blocked", detail)
                return Outcome("blocked", 0.0, 0.0, detail)
        # The stream ended without an outcome, which means the bridge died holding
        # the move. Same treatment as a timeout: stop first, explain second.
        self.stop()
        self.report.finish("failed", "the ROS stack closed the connection")
        return Outcome("failed", 0.0, 0.0,
                       "the ROS navigation bridge closed the connection during "
                       "the move; a stop was sent")
