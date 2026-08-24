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
# How much of the map goes into a picture for the model. A few metres, not the whole
# grid: the pose drifts over a long run, so a picture wide enough to invite planning
# a route home is a picture that will mislead.
MAP_HALF_EXTENT_M = 3.0
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


class RoverNav:
    """Lidar tools and scripting, mixed into Rover."""

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

    def _tool_show_map(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        if self.nav is None:
            return {"ok": False, "error": NO_DRIVING}
        if self.vision is None:
            return {"ok": False, "error": "there is nowhere to send a picture"}
        png, caption = self.nav.map_png(MAP_HALF_EXTENT_M, MAP_SCALE,
                                        camera=self._camera_cone())
        sent = self.vision.post(png)
        # The caption is the answer whether or not the picture arrives. The frame
        # server stashes bytes without decoding them and the upload declares no
        # media type, so a PNG should be as acceptable as the JPEGs `look` sends --
        # but that has not been confirmed at the model itself, and a tool that says
        # nothing when the image is refused would leave the model inventing a map.
        result = {"ok": True, "caption": caption, **self.nav.describe()}
        if not sent.get("ok"):
            result["note"] = ("the map could not be sent as a picture, so answer "
                              "from the description alone: " + str(sent.get("error")))
        return result

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
        magnification, which is how `show_map` asks.

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
    # Five control calls, none of them in :meth:`tools`, and all five refused on
    # anything but the loopback interface -- see `Handler`. A model is not shown
    # them and could not reach them if it were, because the clients that hold a
    # conversation are on a desk across the LAN.
    #
    # That is the MVP's answer to the obvious objection: this port authenticates
    # nothing, and "run this code" is a different proposition from "turn the
    # lights on". Bound to loopback it grants exactly what an ssh session on this
    # Pi already grants, and it is reached the same way -- a tunnel, or an agent
    # working on the rover itself. What lets a model use a behaviour later is
    # `run_behaviour`, which runs something already written and reviewed rather
    # than something composed in the middle of a conversation.

    def _tool_run_script(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Run a script and wait for it. A control call, not a model tool.

        For something that finishes while the caller holds the connection. A
        behaviour that runs for minutes is `start_script`; the difference is only
        who does the waiting, and this one is bounded well inside the clients'
        12 s patience so that "no answer" cannot mean "still working".
        """
        if self.scripts is None:
            return {"ok": False, "error": "this daemon is not running scripts"}
        return self.scripts.run(arguments.get("source"), arguments.get("limit_s"))

    def _tool_start_script(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Start a script and return its handle. A control call, not a model tool."""
        if self.scripts is None:
            return {"ok": False, "error": "this daemon is not running scripts"}
        return self.scripts.start(arguments.get("source"), arguments.get("limit_s"))

    def _tool_script_status(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """How a run is going, or how the last one went. A control call."""
        if self.scripts is None:
            return {"ok": False, "error": "this daemon is not running scripts"}
        return self.scripts.status(arguments.get("id"))

    def _tool_script_stop(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """Stop the running script. A control call, and never refused."""
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

        return {"ok": True, "reference": rover_api.reference(),
                "run_limit_s": scripting.RUN_LIMIT_S,
                "start_limit_s": scripting.START_LIMIT_S,
                "memory_mb": scripting.MEMORY_MB}
