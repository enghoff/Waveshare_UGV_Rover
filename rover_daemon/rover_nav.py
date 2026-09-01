"""Driving and map tools, plus the helpers that size a map picture."""
from __future__ import annotations

import base64
import math
import time
from typing import Any

import scripting
from rover_util import _flag, _number

# What every tool here says when there is no navigator behind it. It deliberately
# does not name the lidar: the sensor belongs to the ROS stack now, so on a rover
# whose lidar is plugged in and spinning "this rover has no lidar attached" is a
# sentence that sends somebody to check a cable that is fine.
NO_DRIVING = ("this rover is not set up to drive or map itself, so it has no "
              "driving tools. The daemon needs --ros-nav, which points it at the "
              "ROS 2 stack")
# How much of the map goes into a picture when nobody asks. A room, not the whole
# grid: the pose drifts over a long run, so a picture wide enough to invite planning
# a route home is a picture that will mislead. The model can ask for more via
# `across_m`; the caption still says not to navigate back off a wide view.
MAP_HALF_EXTENT_M = 3.0
# Magnification `map_png` still accepts directly. The model and the console both
# go through `_map_view` instead, which derives this from extent and picture size.
MAP_SCALE = 3
# What a hand-driven client may ask for, so the window can zoom.
#
# Zooming asks for an extent and a picture size, not an extent and a magnification.
# Pixels per cell is derived from the two, because that is what zooming means: the
# picture stays the size it was and what fits inside it changes. Asking for extent
# and magnification separately -- which is what this did first -- resized the picture
# every time the view widened, which is not zooming, it is rescaling the window.
#
# The bounds are what this host will attempt. Measured here at 5 cm cells, a
# 480 px map is about half a second and a 1200 px one about three, and past that the
# caller holds a connection open for longer than the map stays true.
# How wide a slice of the room the camera takes in, across the picture. It is drawn
# on the map as the gimbal's cone, so that a picture of the room says which part of
# the room the photographs are of -- the two sensors point in different directions
# most of the time, and the rover's own arrow says nothing about where the camera got
# to.
#
# Measured, on this rover's camera, by usb_cameras/calibrate_fov.py: 130 degrees
# across and 96 down, at 640x480, re-measured 2026-08-19 by a pan sweep and a tilt
# sweep that agreed to half a percent. It stood at 65 for a long time as a guess at a
# generic webcam, and the guess was wrong by more than a factor of two -- the module
# fitted here is a fisheye, and the cone was claiming a third of what was actually in
# shot. Two independent references agree to within a degree, the pan servo's own
# degrees and the lidar's scan-matched heading while the whole chassis turns, which
# is also what says the servo is honest. Re-measure it if the camera is ever changed;
# `--camera-fov` is there for a rover wearing a different lens.
CAMERA_FOV_DEG = 130.0

# Twelve, which is 24 m across, because the grid is 40 m across and a view has to
# be able to hold a whole run rather than the middle of one. It was 10 while the grid
# was 20 m, where asking for the ceiling asked for the entire map and got a picture
# whose outer ring could only ever be off-grid grey.
MAP_MAX_HALF_EXTENT_M = 12.0
MAP_MAX_SCALE = 16
MAP_MIN_PIXELS = 200
MAP_MAX_PIXELS = 1200
MAP_PIXELS = 480           # the default picture size, and what the console asks for

# How long a map picture stays something the model can point at. The picture does
# not outlive its exchange -- the conversation drops the image the way it drops the
# one `look` takes -- so past this the model is not pointing at a map, it is
# recalling one, and "the doorway was up and to the left" recalled is a guess with
# a fifteen-metre drive on the end of it. Refusing costs a second of redrawing and
# says exactly what to do instead. Generous rather than tight, because a spoken
# exchange about a room legitimately runs a minute or two before anybody decides
# where to go.
MAP_POINT_MAX_AGE_S = 120.0
# How much room around the named point has to be free of anything solid before the
# rover is sent to it. Roughly the chassis's own half-width, so a point aimed at a
# wall a couple of cells thick is caught here -- with a single cell it is possible
# to slip between the pixels of a wall and have the refusal come back a minute
# later from Nav2 instead of immediately and in the map's own vocabulary.
MAP_POINT_CLEAR_M = 0.15
# Added to the caption of the model's own map, and not to the renderer's, because
# the drive console draws the same picture and has a mouse for this. The rule about
# who states a fact still holds: the tool schema says what the two numbers mean,
# and this says only that the picture in front of the model is a thing it may point
# at, which is a fact about this reply.
MAP_POINT_HINT = (
    " You can drive to anywhere green on this picture with drive_to_map_point, by "
    "saying how far across it is from the left and how far down from the top, each "
    "as a fraction from 0 to 1. The rover is in the middle, at 0.5 and 0.5."
)
# What `explore` may be asked for, in seconds, whatever it is asked for. The
# bounds are here rather than in the schema because a schema is a description and
# this is a rule: a model that has talked itself into an hour of unsupervised
# driving must be given ten minutes, not an hour, and told plainly that is what
# it got. The floor exists for the same reason from the other end -- an explore
# of thirty seconds is one goal and a cancellation, which is worse than not
# starting.
EXPLORE_MIN_S = 60.0
EXPLORE_MAX_S = 900.0
# And what a model that names no time gets. **Not a copy of the bridge's
# `EXPLORE_BUDGET_S`**, which would be the kind of copy this repository keeps
# being bitten by; it is a different number with a different owner that happens
# to agree today. The bridge's is the fallback for somebody poking TCP 8773 by
# hand, and this is the policy about how long a *model* may set the rover off
# for unattended. Sent explicitly on every call, so which one applied is never a
# question about who defaulted.
EXPLORE_DEFAULT_S = 600.0

def _fraction(value: Any, what: str) -> float:
    """A place on the map picture, as a fraction of its width or height.

    Refused outside the picture rather than clamped, which is the opposite of what
    the rest of this file does with a number out of range. Every other bound here
    is a limit of the hardware, where the nearest legal value is the honest answer
    to what was asked for. This one is not a limit at all: a fraction outside nought
    to one is a model that has read the convention the wrong way round, and clamping
    would answer a misunderstanding by driving to the edge of the room.
    """
    if value is None:
        # Named rather than left to `_number`, whose "down must be a number" is
        # true and unhelpful: what has gone wrong is a place with only half of
        # itself given, and the model has to be told the other half is wanted.
        raise ValueError(f"{what} is missing; a place on the map picture needs "
                         f"both across and down")
    number = _number(value, what)
    if not 0.0 <= number <= 1.0:
        raise ValueError(
            f"{what} is a fraction of the map picture, from 0 at one edge to 1 at "
            f"the other, and {number:g} is off the picture")
    return number


def _map_cells(half_extent_m: float, resolution_m: float) -> int:
    """How many cells across the crop will be. Mirrors `mapimg.render`, which centres
    an odd number of cells on the rover's own cell -- rounding rather than truncating,
    because the resolution is a float32 and three metres over it is 59.999999."""
    return 2 * max(8, int(round(half_extent_m / resolution_m))) + 1


def _map_view(half: float, pixels: float, resolution_m: float) -> tuple[float, int]:
    """What the rover will actually draw: an extent, and pixels per cell for it.

    The caller says how much room it wants in frame and how big a picture it wants
    back, and this works out the magnification. That is the way round it has to be.
    Pixels per cell is not a thing anyone wants to choose -- choosing it means the
    picture changes size whenever the view widens, so a zoom control resizes the
    window instead of zooming.

    The size is honoured as closely as whole pixels per cell allow, which is within
    a few percent: the crop is a whole number of cells and each cell is a whole
    number of pixels, so not every size is reachable exactly. Sizes are bounded
    because a picture costs roughly its own area here, drawing being interpreted
    Python, and a 3000 px map took half a minute.
    """
    half = min(MAP_MAX_HALF_EXTENT_M, max(0.5, half))
    pixels = min(MAP_MAX_PIXELS, max(MAP_MIN_PIXELS, pixels))
    cells = _map_cells(half, resolution_m)
    scale = int(min(MAP_MAX_SCALE, max(1, round(pixels / cells))))
    # Rounding up can still overshoot the ceiling on a wide view; the ceiling wins.
    while scale > 1 and cells * scale > MAP_MAX_PIXELS:
        scale -= 1
    return half, scale


def _model_map_view(arguments: dict[str, Any],
                    resolution_m: float) -> tuple[float, int]:
    """Extent and pixels-per-cell for the model's map.

    `across_m` is how many metres of room to show, the way a person would say it,
    not how far each way from the rover -- that half-extent is what `map_png` and
    the renderer take, and a model handed the half would pass six meaning six
    metres across and get twelve. `pixels` is how big a picture. Leave both out
    and this is a room at the same size the console asks for by default, with
    pixels per cell derived rather than fixed, so widening the view shows more
    room instead of a bigger picture.
    """
    if arguments.get("across_m") is not None:
        half = _number(arguments["across_m"], "across_m") / 2.0
    else:
        half = MAP_HALF_EXTENT_M
    pixels = arguments.get("pixels", MAP_PIXELS)
    return _map_view(half, _number(pixels, "pixels"), resolution_m)


class RoverNav:
    """Lidar tools and scripting, mixed into Rover."""

    @property
    def driving(self) -> bool:
        """True while a move has the wheels, for the parts of the rover that care.

        False on a daemon with no navigator, which is the honest answer there: a
        rover that cannot be told to drive is not driving.
        """
        return self.nav is not None and self.nav.driving

    def _tool_drive(self, arguments: dict[str, Any]) -> dict[str, Any]:
        if self.nav is None:
            return {"ok": False, "error": NO_DRIVING}
        distance = _number(arguments.get("distance_m", 0.5), "distance_m")
        speed = arguments.get("speed_ms")
        outcome = self.nav.drive(distance_m=distance,
                                 speed_ms=None if speed is None
                                 else _number(speed, "speed_ms"))
        return {"ok": outcome.reason in ("arrived", "timed out"), **outcome.asdict(),
                **self._nav_context()}

    def _tool_drive_to(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Drive to a place, given either as an offset or as a point on the map.

        `x_m` and `y_m` are deliberately absent from this tool's schema, so a model
        is never shown them. Nothing a model can see says where the rover is in that
        frame -- `describe_surroundings` and the map picture are both relative, and
        the pose only comes back from control calls -- so a model offered a pair of
        map coordinates has no way to arrive at one except by inventing it, and an
        invented pair is a fifteen-metre drive to a place nobody chose. What wants
        them is a console with the map on screen, which knows the pose the picture
        was drawn at and can therefore name the point that was clicked.
        """
        if self.nav is None:
            return {"ok": False, "error": NO_DRIVING}
        x, y = arguments.get("x_m"), arguments.get("y_m")
        if (x is None) != (y is None):
            return {"ok": False, "error": "a place on the map needs both x_m and "
                                          "y_m; one on its own is not a place"}
        if x is None:
            where = {"ahead_m": _number(arguments.get("ahead_m", 0.0), "ahead_m"),
                     "left_m": _number(arguments.get("left_m", 0.0), "left_m")}
        else:
            where = {"x_m": _number(x, "x_m"), "y_m": _number(y, "y_m")}
        speed = arguments.get("speed_ms")
        outcome = self.nav.drive_to(speed_ms=None if speed is None
                                    else _number(speed, "speed_ms"), **where)
        return {"ok": outcome.reason in ("arrived", "timed out"), **outcome.asdict(),
                **self._nav_context()}

    def _tool_turn_in_place(self, arguments: dict[str, Any]) -> dict[str, Any]:
        if self.nav is None:
            return {"ok": False, "error": NO_DRIVING}
        angle = _number(arguments.get("angle_deg", 0.0), "angle_deg")
        outcome = self.nav.turn_in_place(angle)
        return {"ok": outcome.reason == "arrived", **outcome.asdict(),
                **self._nav_context()}

    def _tool_explore(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Set the rover off mapping the rest of the place, and answer at once.

        **It does not wait for the run, and that is about the voice model rather
        than about exploring.** Every client of this daemon holds one connection
        with one lock on it, so a tool call that blocks for ten minutes blocks
        every other tool call for ten minutes -- `stop_driving` included. A model
        that started an explore that way could not stop it, and neither could the
        person in the room asking it to. So this starts the run and comes back;
        `stop_driving` ends it, exactly as it ends anything else.

        **Asking again while it is running reports rather than stops.** A model
        unsure whether its call landed will call again, and a tool that toggled
        would answer that by stopping the rover -- the opposite of what was
        asked, at the moment nobody would notice. Stopping has its own verb.

        `minutes` is capped here rather than in the schema, because a schema
        describes and this is a rule: a model that has talked itself into an hour
        of unsupervised driving gets fifteen minutes.
        """
        if self.nav is None:
            return {"ok": False, "error": NO_DRIVING}
        minutes = arguments.get("minutes")
        budget_s = None
        if minutes is not None:
            budget_s = max(EXPLORE_MIN_S,
                           min(EXPLORE_MAX_S,
                               _number(minutes, "minutes") * 60.0))
        if budget_s is None:
            budget_s = EXPLORE_DEFAULT_S
        started = self.nav.explore_in_background(budget_s=budget_s)

        if started.get("busy"):
            return {"ok": False, "exploring": False,
                    "error": "the rover is already driving somewhere, so it "
                             "cannot go exploring until that has finished or "
                             "been stopped"}
        if not started.get("started"):
            return {"ok": True, "exploring": True,
                    "note": "it is already exploring, and has been for %d "
                            "seconds -- stop_driving ends it"
                            % round(started.get("running_s") or 0)}

        # How the previous run ended, said as the next one sets off. Nothing
        # waits for one of these, so this is the only moment anybody is told --
        # and a model that has just been asked to explore again is exactly who
        # wants to know that last time it stopped after two minutes with half the
        # place unmapped.
        before = self.nav.explored
        return {"ok": True, "exploring": True,
                "last_run": None if before is None else before.detail,
                "note": "the rover has set off to map what it has not seen yet, "
                        "for up to %d minutes. It chooses where to go and stops "
                        "when there is nothing unmapped left it can reach. Say "
                        "so out loud; use stop_driving to end it, and explore "
                        "again to hear how it is getting on."
                        % round(budget_s / 60.0)}

    def _tool_stop_driving(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        if self.nav is None:
            return {"ok": True, "stopped": True,
                    "note": "this rover does not drive itself, so it was not moving"}
        return {"ok": True, **self.nav.stop()}

    def _nav_context(self) -> dict[str, Any]:
        """What the model needs after a move: how much room is left, so it can decide
        what to do next without a second tool call."""
        described = self.nav.describe()
        return {"clear_ahead_m": described["clear_ahead_m"],
                "surroundings": described["text"]}

    def _tool_describe_surroundings(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        if self.nav is None:
            return {"ok": False, "error": NO_DRIVING}
        return {"ok": True, **self.nav.describe()}

    def _camera_cone(self) -> tuple[float, float] | None:
        """The gimbal as `(bearing_deg, fov_deg)` for the map, or None with no camera.

        **The two conventions are opposite, and this minus sign is the whole of the
        conversion.** The gimbal takes pan positive to the *right* (see `look_at`);
        the lidar, the map and everything under [ros_nav/](../ros_nav) take bearings
        positive to the *left*, counter-clockwise from straight ahead. Get
        it backwards and the map draws a perfectly ordinary cone over the wrong half
        of the room, which nothing about the picture would give away.

        None when there is no camera on this rover, because a cone drawn for a lens
        that does not exist is a picture making a claim the hardware cannot keep.
        """
        if self.device is None:
            return None
        with self._lock:
            return -self.pan, self.camera_fov_deg

    def _tool_show_map(self, arguments: dict[str, Any]) -> dict[str, Any]:
        if self.nav is None:
            return {"ok": False, "error": NO_DRIVING}
        if self.vision is None:
            return {"ok": False, "error": "there is nowhere to send a picture"}
        half, scale = _model_map_view(arguments, self.nav.slam.config.resolution_m)
        png, caption = self.nav.map_png(half, scale, camera=self._camera_cone())
        self._remember_map(half, scale, png)
        sent = self.vision.post(png)
        # The caption is the answer whether or not the picture arrives. The frame
        # server stashes bytes without decoding them and the upload declares no
        # media type, so a PNG should be as acceptable as the JPEGs `look` sends --
        # but that has not been confirmed at the model itself, and a tool that says
        # nothing when the image is refused would leave the model inventing a map.
        result = {"ok": True, "caption": caption + MAP_POINT_HINT,
                  **self.nav.describe()}
        if not sent.get("ok"):
            result["note"] = ("the map could not be sent as a picture, so answer "
                              "from the description alone: " + str(sent.get("error")))
        return result

    def _remember_map(self, half_extent_m: float, scale: int, png: bytes) -> None:
        """Keep what was just drawn, so a point on the picture can be read back.

        The five numbers are what turns a place on the image into a place in the
        room: how much floor is in frame, how many pixels a cell came out as, how
        big the picture is, which way the page is turned, and -- the one that is
        not a property of the picture at all -- the pose the rover was at when it
        was drawn. That last one is why this is remembered rather than worked out
        later. A point on a map is a fixed place in the room, and it stays the
        place that was pointed at however long the model spends talking about it
        before it asks to go there; recovering it from the pose *now* would move
        the destination by however far the rover has driven in the meantime.

        The resolution is read after the render and not before, because the render
        is what fetched the map that carries it, and `tap_to_point` has to be the
        exact inverse of the sampling that drew this picture rather than of the one
        before it.

        The picture size comes out of the PNG header, which is where `map_png`'s
        own reply reads it from too: whole cells at whole pixels cannot reach every
        size exactly, so the size asked for is not reliably the size drawn, and a
        fraction of the wrong width is a point in the wrong place.
        """
        self._map_shown = {
            "half_extent_m": half_extent_m,
            "scale": scale,
            # `show_map` never turns the page; only a console asks for that. Kept
            # as a field rather than assumed, so that the conversion below stays
            # correct if the model's map ever gains the option.
            "rover_up": False,
            "resolution_m": self.nav.slam.config.resolution_m,
            "pixels": int.from_bytes(png[16:20], "big"),
            # Read back off the navigator rather than returned by the render. A
            # console fetching its own map in the same instant would replace this
            # object first and leave the pose a fraction of a second late, which is
            # centimetres on a rover that is driving and nothing at all on one that
            # is being spoken to -- and both are far inside how accurately anybody
            # can point at a picture.
            "pose": self.nav.slam.pose,
            "at": time.monotonic(),
        }

    def _map_point(self, shown: dict[str, Any],
                   across: float, down: float) -> tuple[float, float]:
        """A fraction of the picture -> a point in the map's own frame.

        Straight through `mapimg.tap_to_point`, which is the function the drive
        console puts every mouse click through. Sharing it is the point: the model
        pointing at the middle of a doorway and a person clicking the same pixel
        have to arrive at the same place in the room, and two copies of this
        arithmetic would eventually disagree about which way `left` runs.
        """
        import mapimg

        pixels = shown["pixels"]
        return mapimg.tap_to_point(
            across * pixels, down * pixels, shown["half_extent_m"], shown["scale"],
            shown["resolution_m"], rover_up=shown["rover_up"], pose=shown["pose"])

    def _map_point_blocked(self, x_m: float, y_m: float) -> str | None:
        """Why the rover cannot be sent to this point, or None if it can.

        Read off the same occupancy grid the picture was drawn from, so a refusal
        can be phrased in the colour the model was looking at. That matters more
        than it sounds: "that point is black on the map" is a sentence the model
        can act on by pointing somewhere else, where Nav2's own "no valid path" a
        minute later is not, and nothing connects it back to the picture.

        Two states are refused and a third deliberately is not. Solid is refused
        because the rover cannot stand there, and never-seen because grey on this
        map means the lidar has not been there rather than that the floor is clear
        -- which is exactly the reading a model shown a flat grey area is most
        likely to get wrong. What is *not* checked here is whether a route exists:
        floor the rover has seen but that is walled off from where it stands looks
        the same in the grid, and telling the two apart means flooding the map,
        which is tens of milliseconds on a room and seconds on a floor. Nav2 owns
        that question, answers it properly and says so in words, so it keeps it.
        """
        import numpy as np

        slam = self.nav.slam
        with slam.lock:
            grid = np.asarray(slam.grid())
            resolution = slam.config.resolution_m
            cells = slam.config.grid_cells
            occupied_at = slam.config.occupied_at
        ix = int(round(x_m / resolution)) + cells // 2
        iy = int(round(y_m / resolution)) + cells // 2
        if not (0 <= ix < cells and 0 <= iy < cells):
            return ("that point is off the edge of the map the rover keeps, which "
                    "is about %.0f metres across" % (cells * resolution))
        margin = max(1, int(round(MAP_POINT_CLEAR_M / resolution)))
        patch = grid[max(0, ix - margin):ix + margin + 1,
                     max(0, iy - margin):iy + margin + 1]
        if (patch >= occupied_at).any():
            return ("that point is on or within %.0f cm of something solid -- black "
                    "on the map -- so the rover cannot stand there. Point at green "
                    "floor instead" % (MAP_POINT_CLEAR_M * 100))
        if grid[ix, iy] == 0:
            return ("that point is grey on the map, which means the rover has never "
                    "seen it rather than that it is empty, so there is no route to "
                    "plan there. Point at green floor instead")
        return None

    def _tool_drive_to_map_point(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Drive to a place the model has pointed at on the map picture.

        The tool that lets a model name a place on the map at all, and the reason
        it can is that it names it in the frame it can actually see. `drive_to` has
        always taken a point in the map's own frame and has always hidden that pair
        from models, because a model has no way to learn where the rover is in that
        frame and would have to invent the numbers. A fraction of the picture needs
        no such knowledge: the model has the image in front of it, the rover is in
        the middle of it by construction, and the daemon holds the pose the picture
        was drawn at and does the conversion.

        So the picture is really the argument, and everything below is about making
        sure the model is pointing at one that exists: taken at all, taken recently
        enough to still be in front of it, pointed at within its edges, and at
        somewhere the rover could stand.
        """
        if self.nav is None:
            return {"ok": False, "error": NO_DRIVING}
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
        across = _fraction(arguments.get("across"), "across")
        down = _fraction(arguments.get("down"), "down")
        x_m, y_m = self._map_point(shown, across, down)
        # Where the point sits relative to the rover, said in the terms the other
        # driving tools use. It is the model's own check on itself: a doorway it
        # believes is a couple of metres away, coming back as eleven metres behind
        # it, is a misread picture, and the sentence says so before the wheels turn.
        px, py, heading = shown["pose"]
        dx, dy = x_m - px, y_m - py
        cos, sin = math.cos(heading), math.sin(heading)
        pointed = {"ahead_m": round(dx * cos + dy * sin, 2),
                   "left_m": round(-dx * sin + dy * cos, 2),
                   "range_m": round(math.hypot(dx, dy), 2),
                   "x_m": round(x_m, 2), "y_m": round(y_m, 2)}
        blocked = self._map_point_blocked(x_m, y_m)
        if blocked is not None:
            return {"ok": False, "error": blocked, "pointed_at": pointed}
        speed = arguments.get("speed_ms")
        outcome = self.nav.drive_to(x_m=x_m, y_m=y_m,
                                    speed_ms=None if speed is None
                                    else _number(speed, "speed_ms"))
        return {"ok": outcome.reason in ("arrived", "timed out"), **outcome.asdict(),
                "pointed_at": pointed, **self._nav_context()}

    def _tool_nav_status(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Every number the driving loop has. A control call, not a model tool.

        Dispatched like a tool because that is the only protocol here, and absent
        from :meth:`tools` so no model is shown it. What is in this and not in
        `describe_surroundings` is the machinery rather than the room -- the PWM
        actually on the motors, the turn rate the matcher measures, how stale the
        last scan is. That is what tells you why a move went wrong, and it is of no
        use whatsoever to something that has to say the answer out loud.

        Written for [drive_web/drive_web.py](../drive_web/drive_web.py), which
        polls it a few times a second while somebody drives by hand.

        `move` in the reply is the odd one out: not a number off the rover but what
        the request currently running says it is doing -- planning, the route it
        accepted, a replan and what provoked it, how it ended. A move is one
        blocking call that can last a minute, so until it returns this is the only
        account of it there is, and a plan the navigator refused shows up here
        before the refusal itself arrives. See MoveReport in `lidar_slam/nav_types.py`.

        `since_seq` in the arguments is the last of those sentences the caller
        already has; anything said since comes back under `move.missed`. A poller
        that passes it cannot miss a phase for being briefer than its own interval,
        which a replan routinely is.
        """
        if self.nav is None:
            return {"ok": False, "error": NO_DRIVING}
        since = arguments.get("since_seq")
        # `board_reopens` comes from this daemon and not from the bridge, because
        # this daemon is the only thing that holds the board's port. It is the same
        # kind of number as `lidar_resets` beside it and is read the same way: not
        # a fault on its own, but a count that has climbed over an afternoon is a
        # connector working loose, and nothing else on this rover would say so.
        return {"ok": True, **self.nav.status(
            since_seq=None if since is None else int(since)),
            "board_reopens": getattr(self.link, "reopens", 0),
            "board_reopen_note": getattr(self.link, "reopen_note", None)}

    def _tool_map_png(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """The map as base64 PNG in the reply. A control call, not a model tool.

        `show_map` exists for the model and posts the picture to the model's host
        instead, because a tool result cannot carry an image into a conversation.
        A GUI has no such problem, and routing a picture through a frame server to
        get it onto the screen of the machine that asked for it would be silly.

        Zooming is `half_extent_m` -- how much room is in frame -- together with
        `pixels`, how big a picture to send back. Pixels per cell is worked out from
        the two by `_map_view` rather than asked for, so widening the view shows more
        room at the same picture size instead of returning a bigger picture. `scale`
        is still accepted for a caller that really does want to fix the
        magnification. The model is not shown that knob: `show_map` takes metres
        across and a picture size, and `_model_map_view` turns them into these.

        `rover_up` turns the page so that straight up is straight ahead of the rover,
        instead of the direction it was facing when it started.

        The reply says what was drawn rather than what was asked for -- the extent,
        the pixels per cell, the size, what it cost -- because whole cells at whole
        pixels cannot hit every size exactly, and a client that displayed its own
        request would be describing a picture that does not exist.
        """
        if self.nav is None:
            return {"ok": False, "error": NO_DRIVING}
        half = _number(arguments.get("half_extent_m", MAP_HALF_EXTENT_M),
                       "half_extent_m")
        resolution = self.nav.slam.config.resolution_m
        if arguments.get("scale") is not None:
            half = min(MAP_MAX_HALF_EXTENT_M, max(0.5, half))
            scale = int(min(MAP_MAX_SCALE, max(
                1, _number(arguments["scale"], "scale"))))
        else:
            half, scale = _map_view(
                half, _number(arguments.get("pixels", MAP_PIXELS), "pixels"),
                resolution)
        rover_up = _flag(arguments.get("rover_up", False), "rover_up")

        started = time.monotonic()
        png, caption = self.nav.map_png(half, scale, rover_up=rover_up,
                                        camera=self._camera_cone())
        # Read the size out of the PNG rather than working it out again: this is the
        # number the caller is going to display, and it should be the real one.
        width = int.from_bytes(png[16:20], "big")
        x, y, th = self.nav.slam.pose
        return {"ok": True, "caption": caption, "bytes": len(png),
                "half_extent_m": round(half, 2), "scale": scale, "pixels": width,
                "rover_up": rover_up,
                "pose": {"x_m": round(x, 3), "y_m": round(y, 3),
                         "heading_deg": round(math.degrees(th), 1)},
                "render_s": round(time.monotonic() - started, 2),
                "png_base64": base64.b64encode(png).decode("ascii")}

    def _tool_clear_map(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """Throw the SLAM map away and start again. A control call, not a model tool.

        Kept away from models deliberately, and not because it is dangerous -- the
        rover fills a map back in within a revolution or two. It is that a model
        handed this will reach for it. Asked to go somewhere and told there is no
        route, the obliging thing to do is clear the map and try again, and that
        throws away the only account anyone has of the room, including the walls the
        route was refused for. Whether the map has drifted past being worth keeping
        is a judgement made by looking at it, which is a thing a person does.

        The refusal while driving comes from the navigator, where the route being
        followed is: see `clear_map` there.
        """
        if self.nav is None:
            return {"ok": False, "error": NO_DRIVING}
        result = self.nav.clear_map()
        return {"ok": bool(result.get("cleared")), **result}

    def _tool_reset_lidar(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """Reset the lidar's USB device. A control call, not a model tool.

        The rover's lidar hangs off two hubs and drops off the bus from time to
        time, and when it goes the kernel's own port power cycle sometimes cannot
        get it back -- at which point the rover is blind until somebody reaches
        over and replugs it. `usbreset.py` is what does the replug in software, the
        navigator reaches for it by itself after half a minute of silence, and this
        is the same act on demand, so that a person watching the scan age climb has
        a button rather than an ssh session.

        Not offered to models. Not because it is dangerous -- it is the opposite,
        it is the thing that makes a blind rover see again -- but because it takes
        the camera and the OAK down with it for a few seconds, and a model told the
        map is stale would reach for it in preference to waiting the two seconds the
        ordinary reopen needs.
        """
        if self.nav is None:
            return {"ok": False, "error": NO_DRIVING}
        return self.nav.reset_lidar()

    # --- scripts ------------------------------------------------------------
    #
    # Five calls, all of them refused on anything but the loopback interface --
    # see `Handler` -- and one of them, `run_script`, offered to the model as a
    # tool by :meth:`Rover.tools` when that is where the asking is coming from.
    #
    # Loopback is still the whole security argument: this port authenticates
    # nothing, "run this code" is a different proposition from "turn the lights
    # on", and bound to loopback it grants exactly what an ssh session on this
    # board already grants. What has changed is who is on loopback. The rover
    # holds its own conversation now, so a model composing a program in the
    # middle of one is a local client, and the gate that was doing two jobs is
    # back to doing the one it was built for.
    #
    # `start_script` and `script_stop` are offered the same way and for a reason
    # that arrived with them: a behaviour has no time limit any more, so what
    # ends one is somebody stopping it, and the client that can start a thing
    # which outlives the question has to be able to end it too. A blocking run
    # is still the shape that fits a conversation best -- fifteen seconds, and
    # the answer is what the program printed -- and it is what to reach for when
    # the thing being asked for has an end in it.
    #
    # `script_status` and `list_api` remain control calls and stay out of
    # :meth:`Rover.tools`: watching a run is what a console wants, and the
    # catalogue is written into `run_script`'s own description.

    def _tool_run_script(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Run a script and wait for it. One of the three a model is offered.

        For something that finishes while the caller holds the connection. A
        behaviour that keeps going is `start_script`; the difference is who does
        the waiting, and that only this one has a deadline.

        The waiting is longer than a tool call, and the clients know it: fifteen
        seconds of script, plus the interpreter starting, plus the two graces of
        a kill, is half a minute in the worst case against the twelve seconds a
        conversation client allows an ordinary call. `RUN_SCRIPT_TIMEOUT_S` in
        [rover_tools.py](../voice_chat/rover_tools.py) is that arithmetic on the
        other side of the wire, and it is there so that a script stopped at its
        limit is reported as a script stopped at its limit rather than read as a
        daemon that has died.
        """
        if self.scripts is None:
            return {"ok": False, "error": "this daemon is not running scripts"}
        return self.scripts.run(arguments.get("source"), arguments.get("limit_s"))

    def _tool_start_script(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Start a behaviour and return its handle. A model tool, on loopback.

        No deadline unless the caller asks for one with `limit_s`, so what comes
        back is a handle to something that will still be running when the next
        thing is said. Refused while another script holds the slot, which is the
        reply that names it -- and `script_stop` is how the slot is given back.
        """
        if self.scripts is None:
            return {"ok": False, "error": "this daemon is not running scripts"}
        return self.scripts.start(arguments.get("source"), arguments.get("limit_s"))

    def _tool_script_status(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """How a run is going, or how the last one went. A control call."""
        if self.scripts is None:
            return {"ok": False, "error": "this daemon is not running scripts"}
        return self.scripts.status(arguments.get("id"))

    def _tool_script_stop(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """Stop the running script. A model tool now, and never refused.

        Never refused is what makes it safe to offer: with nothing running it
        answers that nothing was, so a model that reaches for it after being
        told to stop cannot be wrong to have tried. It is also the only thing
        that ends an unbounded behaviour, short of the script itself.
        """
        if self.scripts is None:
            return {"ok": False, "error": "this daemon is not running scripts"}
        return self.scripts.stop()

    def _tool_list_api(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """The primitives a script may use, generated from the module itself.

        `list_tools` for programs, and for the same reason: the rover is the only
        thing that knows what it can do, so nothing that writes a behaviour should
        be carrying its own copy of the answer.
        """
        import rover_api

        # No `start_limit_s` among these, and its absence is the fact: a
        # behaviour is bounded by being stopped rather than by a clock, so there
        # is no number to report and reporting one would invent a deadline.
        return {"ok": True, "reference": rover_api.reference(),
                "run_limit_s": scripting.RUN_LIMIT_S,
                "memory_mb": scripting.MEMORY_MB}
