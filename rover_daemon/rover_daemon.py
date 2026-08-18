"""The rover's control plane: one process owning the board, the camera and the loop.

Everything that touches the rover's hardware goes through here. That is not
tidiness, it is the only arrangement that works: the ESP32 hangs off a single
UART and the camera can be opened by one process at a time, so two programs that
both want to command servos or look through the lens are two programs corrupting
each other. `drive_gamepad_pi.py` takes the UART for the wheels and the lights,
and `track_face_pi.py` takes it for the gimbal; running both means interleaved
JSON on one wire, and nothing at all could then also want the camera.

    python3 rover_daemon.py                    # ttyAMA0, camera, detector on the OAK
    python3 rover_daemon.py --host 192.168.1.22    # board over WiFi instead
    python3 rover_daemon.py --no-camera        # lights and gimbal only
    python3 rover_daemon.py --vision 192.168.1.3:8767   # ...and it can be looked through

`--vision` adds one tool, `look`, which takes a picture and posts it to the
voice service's `/frame` so that a vision-language model can be asked about it.
The frame goes straight from here to that card -- the same road the detector's
frames already take -- and never through the client holding the conversation.
It is a flag rather than a default because it only works against a service
running a model that can take an image: without one, `look` is a tool that can
only fail, and offering it is worse than not having it. Dropping the flag is
therefore the whole rollback on this side, since clients ask `list_tools` afresh
every time they connect.

Clients speak newline-delimited JSON over TCP -- one request, one reply:

    -> {"call": "set_lights", "arguments": {"level": 255}}
    <- {"ok": true, "level": 255, "on": true}
    -> {"call": "list_tools"}
    <- {"ok": true, "tools": [ ...JSON schemas... ]}

Four calls in that protocol are for the client rather than for the model, and none
of them appears in `list_tools`, so no model is ever shown them. `list_tools`
itself is one. `set_vision` says where `look` should post its pictures:

    -> {"call": "set_vision", "arguments": {"address": "192.168.1.7:8767"}}
    <- {"ok": true, "vision": "http://192.168.1.7:8767/frame", "tools": [...]}

The last two are for driving the rover by hand rather than by conversation, and
exist for [voice_chat/drive_console.py](../voice_chat/drive_console.py):
`nav_status` returns every number the driving loop has, and `map_png` returns the
map as base64 in the reply instead of posting it away. Both are things a person
watching a move needs and a model asked to narrate one does not.

`list_tools` is why the clients carry no schemas of their own. The daemon is the
only thing that knows what this rover can do, so it is the only thing that should
be describing it -- [voice_chat/talk.py](../voice_chat/talk.py) asks, and
hands the answer straight to the model. Adding a tool is a change to this file
alone, with nothing to redeploy anywhere else.

**Why the tracking loop lives here rather than staying a separate script.**
`track_face_pi.py` is still the right thing to run when face tracking is all you
want; it is standalone, it prints a status line, and it is where the loop was
worked out. This runs the same loop -- importing the same `aiming.py`, so the two
cannot become different robots -- but under a switch, sharing the board with
everything else, so that a conversation can start and stop it.

The client is not on this machine. Speech runs on whatever desk has a
microphone, and [voice_chat/talk.py](../voice_chat/talk.py) reaches this over the
LAN like any other client. That is why this binds an address rather than a Unix
socket, and it is why forwarding frames at 30% of the Pi's core is simply the
cost of tracking rather than something that has to be budgeted against anything
else running here.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import re
import socket
import socketserver
import sys
import threading
import time
from pathlib import Path
from typing import Any

DEFAULT_SERIAL = "/dev/ttyAMA0"
# The ESP32, by address: it is the one device here that advertises no mDNS
# name, so there is nothing to call it by.
DEFAULT_BOARD_HOST = "192.168.1.22"
# The detector is on this Pi now, not on MEDIA -- oak_detect/server.py, running
# the graph on the OAK camera's Myriad X. It is still reached over HTTP rather
# than called in-process, because the protocol was already there and a separate
# process is what lets the device be restarted without restarting the rover.
# Loopback and by address, so nothing about a frame's round trip can depend on
# the network or on mDNS: this sits in a control loop with a 1 s timeout, and
# what used to be here was a 5 s outlier resolving `media.local`.
DEFAULT_SERVICE = "local"  # the OAK, in this process -- see oak_detect/local.py
# Where a picture goes to be looked at, when --vision is given. By address for
# the same reason as the detector above: this is on a control path, and a 5s
# mDNS outlier is a tool call that times out.
DEFAULT_VISION = "192.168.1.3:8767"  # voice-chat.service on MEDIA
DEFAULT_DEVICE = "/dev/video0"
BAUD = 115200

# Bound to the LAN, not loopback: the client is `talk.py` on whichever desk has a
# microphone, not anything on this box. Nothing here authenticates -- the same
# trade face-detect makes, and the same warning applies. This is a home LAN.
HOST = "0.0.0.0"
PORT = 8769

CMD_LIGHTS = 132  # CMD_LED_CTRL, both channels driven together as one headlight
CMD_PROBE = 130   # a harmless query; the board answers with its usual telemetry
LIGHT_MAX = 255

# How long a "show me somebody else" suppression lasts, and how wide it is in
# multiples of the face's own width. Long enough to let the sweep carry the
# camera off the person it was on, short enough that they are not banished.
SKIP_FOR_S = 6.0
SKIP_RADIUS = 1.5

# How long the picture-taking service gets. A frame is ~35kB and the POST is
# milliseconds on this LAN, but the model host is also the one holding a
# conversation, so this is sized to outlast a busy moment rather than to be
# tight. The voice service's own patience for a tool is 12s and includes this.
VISION_TIMEOUT_S = 6.0
# How long to spend finding out whether the face detector is there before
# starting to track. Short, because this is paid on the way into a tool call the
# model is waiting on, and because the answer on a LAN is immediate either way --
# except when the host is off rather than refusing, which is the case worth
# bounding. See `_detector_ready` for why this check exists at all.
DETECT_PROBE_S = 2.0
# A frame this old is not what the camera is looking at any more. Only reached
# while the tracking loop owns the camera, where the loop's newest frame is used
# rather than opening a second one -- which is impossible anyway.
FRAME_STALE_S = 2.0

# The camera feed is closed this long after the last thing that needed it. Only
# face tracking opens the feed now -- one-shot pictures do not, see `_snapshot` --
# and tracking closes it on its way out, so in ordinary running nothing reaches
# this. It stays as the backstop for the case it was always really for: an open
# camera is a camera nothing else can open, and a feed left behind by a crash
# between `_open_camera` and tracking's own `finally` would otherwise sit there
# taking a quarter of the core off the scan matcher until the daemon restarted.
CAMERA_IDLE_S = 20.0
# How many frames a one-shot picture asks the camera for. Named here as well as in
# track_face_pi because it is in two error messages the model reads out loud.
SNAPSHOT_FRAMES = 3

# The lidar, when this daemon is asked to drive. A separate port from the driver
# board: the board is on the GPIO UART and the lidar is a CH343 the cdc_acm driver
# claims, so there is no /dev/ttyUSB* to look for. See docs/hosts.md.
#
# "auto" rather than a device node, because the node is not stable. This adapter
# re-enumerated as ttyACM1 under a running daemon and left it holding a dead
# ttyACM0, still answering questions from a scan that had stopped updating. The
# navigator prefers the /dev/serial/by-id name, which carries the serial number.
DEFAULT_LIDAR = "auto"
# How much of the map goes into a picture for the model. A few metres, not the whole
# 20 m grid: the pose drifts over a long run, so a picture wide enough to invite
# planning a route home is a picture that will mislead.
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
# The bounds are what the Pi will attempt. Measured on this host at 5 cm cells, a
# 480 px map is about half a second and a 1200 px one about three, and past that the
# caller holds a connection open for longer than the map stays true.
# How wide a slice of the room the camera takes in, across the picture. It is drawn
# on the map as the gimbal's cone, so that a picture of the room says which part of
# the room the photographs are of -- the two sensors point in different directions
# most of the time, and the rover's own arrow says nothing about where the camera got
# to.
#
# Measured, on this rover's camera, by usb_cameras/calibrate_fov.py: 132 degrees
# across and 98 down, at 640x480. It stood at 65 for a long time as a guess at a
# generic webcam, and the guess was wrong by more than a factor of two -- the module
# fitted here is a fisheye, and the cone was claiming a third of what was actually in
# shot. Two independent references agree to within a degree, the pan servo's own
# degrees and the lidar's scan-matched heading while the whole chassis turns, which
# is also what says the servo is honest. Re-measure it if the camera is ever changed;
# `--camera-fov` is there for a rover wearing a different lens.
CAMERA_FOV_DEG = 132.0

MAP_MAX_HALF_EXTENT_M = 10.0
MAP_MAX_SCALE = 16
MAP_MIN_PIXELS = 200
MAP_MAX_PIXELS = 1200
MAP_PIXELS = 480           # the default picture size, and what the console asks for


def _level(value: Any) -> int:
    """Whatever the model produced -> a brightness, or ValueError.

    Tolerant on purpose. A small quantised model will hand over "255", 255.0 or
    "on" about as often as it hands over 255, and refusing those means the user
    hears "I could not do that" over a difference the tool does not care about.
    """
    if isinstance(value, bool):  # before int: bool is an int in Python
        return LIGHT_MAX if value else 0
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("on", "true", "full", "max"):
            return LIGHT_MAX
        if text in ("off", "false", "none"):
            return 0
        if text.endswith("%"):
            return round(float(text[:-1]) * LIGHT_MAX / 100)
        value = float(text)
    if not isinstance(value, (int, float)):
        raise ValueError(f"level must be a number from 0 to {LIGHT_MAX}")
    return int(min(max(round(value), 0), LIGHT_MAX))


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


def _number(value: Any, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise ValueError(f"{what} must be a number")
    try:
        return float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{what} must be a number, not {value!r}")


def _flag(value: Any, what: str) -> bool:
    """Whatever the caller produced -> a yes or a no, or ValueError.

    Loose in the same way `_level` is, and for the same reason: a small quantised
    model writes "true", "yes" or 1 about as often as it writes a JSON boolean, and
    refusing those means refusing the tool. Only genuinely ambiguous input raises --
    silently reading an unrecognised word as False would turn a mistake into a picture
    that looks fine and faces the wrong way.
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("true", "yes", "on", "1"):
            return True
        if text in ("false", "no", "off", "0", ""):
            return False
    raise ValueError(f"{what} must be true or false, not {value!r}")


class SerialLink:
    """JSON commands down the GPIO UART to the ESP32.

    Locked, unlike the copies in the other scripts, because this is the one that
    is genuinely shared: a tool call arrives on a connection thread while the
    tracking loop is commanding servos on its own, and two interleaved writes are
    one line of JSON the board cannot parse.
    """

    def __init__(self, port: str) -> None:
        import serial

        self.port = port
        self.link = serial.Serial(port, BAUD, timeout=0.1)
        self._lock = threading.Lock()

    def describe(self) -> str:
        return f"{self.port} at {BAUD}"

    def send(self, command: dict[str, Any]) -> bool:
        line = json.dumps(command, separators=(",", ":")).encode() + b"\n"
        with self._lock:
            try:
                self.link.write(line)
                # The board streams T:1001 telemetry continuously at ~2.6 kB/s
                # and nothing here reads it; left alone it fills within seconds.
                self.link.reset_input_buffer()
                return True
            except Exception:
                return False

    def close(self) -> None:
        try:
            self.link.close()
        except Exception:
            pass


class HttpLink:
    """JSON commands over the ESP32's own `/js` endpoint, for a board on WiFi."""

    def __init__(self, host: str, timeout: float = 1.0) -> None:
        import http.client
        from urllib.parse import quote

        self._client = http.client
        self._quote = quote
        self.host = host
        self.timeout = timeout
        self.connection = None
        self._lock = threading.Lock()

    def describe(self) -> str:
        return f"http://{self.host}/js"

    def send(self, command: dict[str, Any]) -> bool:
        path = "/js?json=" + self._quote(
            json.dumps(command, separators=(",", ":")), safe="")
        with self._lock:
            for attempt in (1, 2):  # a stale keep-alive costs one retry
                if self.connection is None:
                    self.connection = self._client.HTTPConnection(
                        self.host, timeout=self.timeout)
                try:
                    self.connection.request("GET", path)
                    self.connection.getresponse().read()
                    return True
                except Exception:
                    self._close()
                    if attempt == 2:
                        return False
        return False

    def _close(self) -> None:
        if self.connection is not None:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None

    def close(self) -> None:
        with self._lock:
            self._close()


class VisionLink:
    """The voice service's `/frame`, over one kept-open connection.

    Modelled on `track_face_pi.Detector` rather than on anything new: this
    machine already POSTs a JPEG for every tracked frame, and this is the same
    POST to a different host and port. One request at a time, a stale keep-alive
    costs a retry rather than a picture, and a service that is not there comes
    back as a failure to report rather than an exception to raise -- the model
    has to be told it could not see, in words it can repeat.
    """

    def __init__(self, address: str, timeout: float = VISION_TIMEOUT_S) -> None:
        import http.client

        self._client = http.client
        host, _, port = address.partition(":")
        self.host = host
        self.port = int(port) if port else 8767
        self.timeout = timeout
        self.connection = None
        self._lock = threading.Lock()

    def describe(self) -> str:
        return f"http://{self.host}:{self.port}/frame"

    def post(self, jpeg: bytes) -> dict[str, Any]:
        """Send one frame. Returns the service's answer, or {"ok": false, ...}."""
        with self._lock:
            for attempt in (1, 2):
                try:
                    if self.connection is None:
                        self.connection = self._client.HTTPConnection(
                            self.host, self.port, timeout=self.timeout)
                        self.connection.connect()
                        # As in Detector: headers and body are two writes, and
                        # Nagle can hold the second until the first is acked.
                        self.connection.sock.setsockopt(
                            socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.connection.request(
                        "POST", "/frame", body=jpeg,
                        headers={"Content-Type": "image/jpeg",
                                 "Content-Length": str(len(jpeg))})
                    payload = json.loads(self.connection.getresponse().read())
                except Exception as error:
                    self.close()
                    if attempt == 2:
                        return {"ok": False,
                                "error": f"could not send the picture to {self.describe()}: "
                                         f"{type(error).__name__}: {error}"}
                    continue
                if not isinstance(payload, dict) or not payload.get("image"):
                    return {"ok": False,
                            "error": str(payload.get("error") if isinstance(payload, dict)
                                         else "the vision service gave no answer")}
                return {"ok": True, "image": payload["image"],
                        "w": payload.get("w"), "h": payload.get("h")}
        return {"ok": False, "error": "unreachable"}

    def close(self) -> None:
        if self.connection is not None:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None


def open_link(serial_port: str | None, host: str | None):
    if host:
        return HttpLink(host)
    serial_port = serial_port or DEFAULT_SERIAL
    if not serial_port.startswith("/") and not re.fullmatch(r"COM\d+", serial_port, re.I):
        return HttpLink(serial_port)
    return SerialLink(serial_port)


# --- what the model is shown -------------------------------------------------
# Wordier than they look because a 4B model at int4 reads these descriptions and
# nothing else. Anything left implicit is invented.

TOOLS: list[dict[str, Any]] = [
    {
        "type": "function",
        "function": {
            "name": "set_lights",
            "description": (
                "Switch or dim the rover's white headlights. The level is 0 for "
                "off, 255 for full brightness, and anything between for dimmer. "
                "Use 255 when asked to turn the lights on and 0 to turn them off."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "level": {"type": "integer", "minimum": 0, "maximum": LIGHT_MAX,
                              "description": "Brightness from 0 (off) to 255 (full)."},
                },
                "required": ["level"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_lights",
            "description": (
                "Report the headlight brightness as a level from 0 to 255 and "
                "whether they are on. The board cannot be read back, so this is "
                "the last level that was set."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "look_at",
            "description": (
                "Point the rover's camera. pan is degrees left or right of "
                "straight ahead, negative for left and positive for right; tilt "
                "is degrees up or down, negative for down and positive for up. "
                "This stops face tracking if it is running, since both cannot "
                "aim the camera at once."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "pan": {"type": "number", "description": "Degrees; negative left."},
                    "tilt": {"type": "number", "description": "Degrees; negative down."},
                },
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "center_camera",
            "description": "Point the camera straight ahead and level. Stops face tracking.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "count_faces",
            "description": (
                # Reworded 2026-08-16 and measured: the old wording opened "Look "
                # through the camera once", and beside a tool that actually looks
                # it stopped being called at all -- "how many people can you see"
                # called nothing, 0/6. Naming what it is *not* for is what fixed
                # it. See voice_chat/README.md.
                "Count the people in front of the rover and say roughly where "
                "each one is: left, centre or right, and near or far. Use this "
                "only for counting people, not for seeing what something is. "
                "Does not move the camera."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "start_tracking",
            "description": (
                "Start following a face with the camera. The rover keeps whoever "
                "it finds centred in view, and sweeps to look for somebody if "
                "there is nobody about. It picks the largest face it can see, "
                "which is normally the nearest person."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "stop_tracking",
            "description": "Stop following a face and return the camera to centre.",
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "track_next",
            "description": (
                "Let go of the person being followed and look for a different "
                "one. The rover cannot tell people apart, so this ignores "
                "whoever is currently being followed for a few seconds and takes "
                "the next face it finds -- which may be the same person again if "
                "nobody else is there. Starts tracking if it was not running."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "tracking_status",
            "description": (
                "Report whether face tracking is running, whether a face is "
                "currently being followed, how many are in view, and where the "
                "camera is pointing."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
]


# Offered only when --vision is given, since without somewhere to send the
# picture this can do nothing but fail. Worded for a model that will otherwise
# answer from its imagination: the failure being designed against is a rover
# that describes a room it has never looked at, which sounds exactly like one
# that has.
#
# Every word of this description was measured, because the first one was not
# called at all -- 0/6 on "what can you see right now" beside the other nine
# schemas, while the same tool alone scored 6/6. A tool is not read on its own:
# it is read against its neighbours, and "take a photograph and look at it" lost
# to a list already full of looking. Naming it as the *only* way to see, and
# pointing the counting question at the tool that counts, took it to 6/6. The
# table is in voice_chat/README.md; change this wording only with numbers.
#
# The opening sentence was measured too, and so was its position. Without it the
# model answers the plainest questions there are -- "what can you see", "check
# your camera", "can you describe what is in front of you" -- with "I'll take a
# picture to see what's in front of me" and takes none: 0/6 each. Naming those
# questions takes them to 6/6, and naming them *first* is worth the last of it
# ("check your camera" is 0/6 with the same sentence at the end). Renaming the
# tool to take_picture, which is the model's own phrase for it, was tried and is
# much worse -- it collides with look_at, so "look around" aims the camera
# instead of photographing, and "what do you see now" falls 6/6 -> 0/6.
#
# The second sentence is about the questions that come *after* a picture, and it
# is here because the picture stopped outliving its turn. Once the looking
# exchange is dropped, "what else is on the table?" is a fresh question with no
# view behind it -- and it was answered "I can't see what's on the table without
# taking a picture" by a rover that could have taken one, 0/6. Naming those
# questions too, at 6 samples a cell:
#
#   "What else is on the table?"   0/6 -> 3/6
#   "What else is there?"          0/6 -> 3/6
#   "Is there anything else?"      0/6 -> 5/6
#   "How many people can you see?" 4/6 -> 6/6
#
# Position again, and again not the obvious one: in *front* of the opening
# sentence it totals higher still but takes "check your camera" 6/6 -> 3/6 and
# "what colour is the box" 6/6 -> 3/6, because it displaces the list that was
# put first for exactly that reason. Second is the only placement measured that
# costs no cell. Folding both lists into one sentence is worse than either
# (55/72 against 65/72) -- the follow-ups need their own sentence, not a longer
# list. Still only a partial fix: two of those cells are 3/6, not 6/6.
LOOK_TOOL: dict[str, Any] = {
    "type": "function",
    "function": {
        "name": "look",
        "description": (
            "Call it when you are asked what you can see, what is in front of "
            "you, to check your camera, or to describe or read anything. "
            "Call it again for a follow-up about the same view -- what else is "
            "there, what else is on something, whether there is anything else, "
            "or what colour or shape something is. You keep no picture between "
            "questions, so answering one of those means taking a new one. "
            "See what is in front of the rover. This is the only way you can see "
            "anything at all: it takes a photograph through the camera and shows "
            "it to you. Use it for every question about what is there, what "
            "something is, what it looks like, what it says, or what the rover "
            "can see. To count how many people are there, use the counting tool "
            "instead. It does not move the camera."
        ),
        "parameters": {"type": "object", "properties": {}},
    },
}


# Offered only when the daemon has a lidar, for the same reason `look` is offered
# only when there is somewhere to send a picture: a tool that cannot reach its
# hardware is worse than a missing one, because the model reports success and
# nothing happens.
NAV_TOOLS: list[dict[str, Any]] = [
    {
        "type": "function",
        "function": {
            "name": "drive",
            "description": (
                "Drive the rover forward. It watches its lidar the whole way and "
                "stops itself rather than hitting anything, steering around "
                "obstacles when it can. Always says how far it actually got and why "
                "it stopped, which will often be less than asked for. Pauses face "
                "tracking while it moves and resumes it afterwards. It cannot see "
                "steps, drops, or anything above or below the height of its lidar, "
                "so do not drive it near a stair or a table edge on the strength of "
                "this. To change heading, turn on the spot first."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "distance_m": {
                        "type": "number", "minimum": 0.05, "maximum": 3.0,
                        "description": "How far to go, in metres.",
                    },
                    "speed_ms": {
                        "type": "number", "minimum": 0.05, "maximum": 0.35,
                        "description": "Metres per second. Leave it out for a "
                                       "sensible walking crawl.",
                    },
                },
                "required": ["distance_m"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "drive_to",
            "description": (
                "Drive to a place a given distance ahead and to the left of the "
                "rover, in metres, going around obstacles. It plans a route of "
                "straight segments and turns, follows it without needing to hit "
                "the line exactly, and plans again if something gets in the way "
                "or the room has changed. Distances are from where the rover is "
                "now, not from where it started: positive ahead is forward, "
                "positive left is to its left, negatives are behind and right. "
                "Always says how far it actually got and why it stopped. Prefer "
                "this over a series of drive and turn calls when you know where "
                "you want to end up. It cannot see steps, drops, or table tops. "
                "This can take tens of seconds."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "ahead_m": {
                        "type": "number", "minimum": -8.0, "maximum": 8.0,
                        "description": "Metres forward of the rover; negative is "
                                       "behind.",
                    },
                    "left_m": {
                        "type": "number", "minimum": -8.0, "maximum": 8.0,
                        "description": "Metres to the rover's left; negative is "
                                       "right.",
                    },
                    "speed_ms": {
                        "type": "number", "minimum": 0.05, "maximum": 0.35,
                        "description": "Metres per second. Leave it out for a "
                                       "sensible walking crawl.",
                    },
                },
                "required": ["ahead_m", "left_m"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "turn_in_place",
            "description": (
                "Turn the rover on the spot without going anywhere, by a number of "
                "degrees: positive turns left, negative turns right. Use this to "
                "face something before driving to it, and use it to get out of a "
                "tight spot: turning is never refused, because rotating is how a "
                "rover that has got too close to something gets away from it. It "
                "turns more slowly when something is within about 25 cm, and says so."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "angle_deg": {
                        "type": "number", "minimum": -180, "maximum": 180,
                        "description": "Degrees to turn; positive is left.",
                    },
                },
                "required": ["angle_deg"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "stop_driving",
            "description": (
                "Stop the rover moving immediately. Use this the moment anyone asks "
                "it to stop, or if something sounds wrong."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "describe_surroundings",
            "description": (
                # Named for what it answers rather than for the sensor, on the same
                # reasoning as count_faces: a tool called "read the lidar" does not
                # get called when somebody asks what is around the rover.
                "Say what is around the rover and how much room it has, measured "
                "with its lidar rather than seen with its camera. Gives the walls, "
                "any free-standing objects, the gaps between them and how far it "
                "can go forward. Use this to answer questions about space, room, "
                "distance and what is in the way, and before driving somewhere. It "
                "does not use the camera and cannot tell you what anything is -- the "
                "lidar measures one flat slice at its own height, so a table appears "
                "only as its legs."
            ),
            "parameters": {"type": "object", "properties": {}},
        },
    },
]

MAP_TOOL: dict[str, Any] = {
    "type": "function",
    "function": {
        "name": "show_map",
        "description": (
            "Take a top-down map of the few metres around the rover, built up from "
            "its lidar as it has driven, and look at it. Use this for questions "
            "about the shape of the space or about getting from one place to "
            "another. For a plain question about what is nearby, "
            "describe_surroundings is quicker and more precise."
        ),
        "parameters": {"type": "object", "properties": {}},
    },
}


def _where(face, width: int, height: int) -> dict[str, Any]:
    """One box described the way a person would say it, not in pixels.

    The model is going to read this out loud, so "on your left, quite close" has
    to be derivable from it without the model doing arithmetic on coordinates --
    which a 4B model at int4 will get wrong, confidently.
    """
    x, y, w, h = face[0], face[1], face[2], face[3]
    centre_x = (x + w / 2) / max(width, 1)
    side = "left" if centre_x < 0.38 else "right" if centre_x > 0.62 else "centre"
    # By apparent width. A face is about 16 cm across, so this is a crude range
    # estimate and is deliberately reported in words rather than metres.
    share = w / max(width, 1)
    distance = "near" if share > 0.22 else "far" if share < 0.10 else "mid"
    return {"where": side, "distance": distance,
            "score": round(float(face[4]), 3) if len(face) > 4 else None}


class Rover:
    """The rover's state and everything that may be done to it.

    One lock covers the board and the model of where things are pointed. The
    tracking loop runs on its own thread and takes the same lock for as long as
    it takes to send a servo command, so a tool call arriving mid-sweep is
    ordered against it rather than interleaved with it.
    """

    def __init__(self, link, service: str, device: str | None, size=(640, 480),
                 vision: str | None = None,
                 camera_fov_deg: float = CAMERA_FOV_DEG) -> None:
        self.link = link
        self.service = service
        self.device = device
        self.camera_fov_deg = camera_fov_deg
        self.size = size
        self.vision = VisionLink(vision) if vision and device else None

        self._lock = threading.RLock()
        self.level = 0
        self.pan = 0.0
        self.tilt = 0.0

        self._camera = None
        self._camera_used = 0.0
        self._detector = None
        self._loop_fps = 0.0
        # Overridden by whichever detector is opened, since the bar for
        # starting a lock is a property of that detector's scores.
        self._acquire_score = None
        # The tracking loop's newest frame, kept so that `look` has something to
        # send while the loop owns the camera. One 35kB JPEG, replaced in place.
        self._frame: tuple[bytes, float] | None = None

        self._tracking = threading.Event()
        self._thread: threading.Thread | None = None
        # Set while driving has taken face tracking away from itself, so that the
        # end of the move can hand it back. Only the navigator's callbacks touch it,
        # and they run on whichever thread asked for the move.
        self._tracking_parked = False
        self.nav = None
        self._skip_centre = None
        self._skip_until = 0.0
        # What the loop last saw, for tracking_status and count_faces while it
        # is running -- the loop owns the camera then, so nothing else may look.
        self._seen = {"faces": 0, "locked": False, "at": 0.0, "where": []}
        # The detector is one kept-open connection that takes strictly one request
        # at a time, so two `count_faces` arriving together must not both be in it.
        # Its own lock rather than the board's: waiting on a detector on another
        # host is no reason a light cannot be switched or the wheels stopped, and
        # this call used to hold the board lock for exactly that wait.
        self._detector_lock = threading.Lock()

    # --- the board ----------------------------------------------------------

    def tools(self) -> list[dict[str, Any]]:
        """What this rover can do, as this rover is currently configured.

        Built rather than constant, because `look` exists only when there is
        somewhere to send a picture. A tool that cannot reach the hardware it
        describes is worse than a missing one -- the model says it has done the
        thing, and nothing happens -- and the same is true of one that cannot
        reach the model's own host.

        The same rule covers driving, which needs a lidar, and the map, which needs
        both a lidar to build it and somewhere to send the picture.
        """
        tools = list(TOOLS)
        if self.vision is not None:
            tools.append(LOOK_TOOL)
        if self.nav is not None:
            tools += NAV_TOOLS
            if self.vision is not None:
                tools.append(MAP_TOOL)
        return tools

    def describe(self) -> str:
        return self.link.describe()

    def probe(self) -> bool:
        return self.link.send({"T": CMD_PROBE})

    def _send_lights(self, level: int) -> bool:
        return self.link.send({"T": CMD_LIGHTS, "IO4": level, "IO5": level})

    def _send_gimbal(self) -> bool:
        # T:133 is the simple absolute form, and deliberately does not feed the
        # firmware's heartbeat: aiming is not driving, and must not be mistaken
        # for it by a base that stops itself when commands stop arriving.
        return self.link.send({"T": 133, "X": round(self.pan), "Y": round(self.tilt),
                               "SPD": 0, "ACC": 0})

    def centre_gimbal(self) -> bool:
        with self._lock:
            self.pan = self.tilt = 0.0
            return self._send_gimbal()

    # --- the camera ---------------------------------------------------------
    #
    # There are two ways to get a picture here and the difference is not the
    # picture, it is what is still running afterwards.
    #
    # `_open_camera` is the 30 fps feed, and only face tracking uses it: the loop
    # wants every frame it can get, and pays for them. Everything else -- `look`,
    # `camera_jpeg`, `count_faces` -- goes through `_snapshot`, which opens the
    # camera for three frames and closes it. That is not a micro-optimisation. The
    # feed costs this one core about a quarter of the lidar's revolutions, and
    # leaving it warm for CAMERA_IDLE_S meant one photograph degraded the scan
    # matcher for twenty seconds; since the matcher is this rover's only odometer,
    # that is twenty seconds of a drive measuring itself wrong. The numbers are in
    # `track_face_pi.snapshot`, which is also where the capture lives.

    def _open_camera(self):
        """The shared camera feed, opened on demand. Caller holds the lock.

        **Only call this from a thread that will outlive the camera.** v4l2-ctl is
        started with PR_SET_PDEATHSIG, and the kernel counts the *thread* that
        started it as the parent, so a feed opened on a connection thread dies when
        that request finishes. Tracking's loop thread is the one caller that
        qualifies, and it is now the only caller.
        """
        from track_face_pi import Camera

        if self._camera is not None and self._camera.alive():
            self._camera_used = time.monotonic()
            return self._camera
        if self._camera is not None:
            self._camera.close()
        # YUYV when the detector is in this process, because it can take pixels
        # directly and decoding a JPEG here costs 85 ms a frame -- more than the
        # inference. MJPEG when the detector is over HTTP, which wants a picture.
        camera = Camera(self.device, self.size,
                        "YUYV" if self.service == "local" else "MJPG")
        self._camera = camera
        self._camera_used = time.monotonic()
        return camera

    def _snapshot(self, frames: int = SNAPSHOT_FRAMES):
        """A few whole frames from a camera that is shut again straight away.

        The seam the self-test replaces, and the reason it is a method rather than a
        bare import at each call site. Nothing is held: no lock, no camera, no
        thread -- so this is safe to call while the rover is driving, which is the
        whole point of it.
        """
        from track_face_pi import snapshot

        return snapshot(self.device, self.size, frames=frames)

    def _close_camera(self) -> None:
        if self._camera is not None:
            self._camera.close()
            self._camera = None

    def _open_detector(self):
        """The face detector, in this process or over HTTP.

        `--service local` opens the OAK here and feeds it raw frames, which is
        what makes the loop run at a useful rate. Anything host:port keeps the
        old path, which is how the detector can be put back on another machine
        without changing anything else.
        """
        if self._detector is not None:
            return self._detector
        if self.service == "local":
            sys.path.insert(0, str(Path(__file__).resolve().parent / "oak_detect"))
            from local import ACQUIRE_SCORE, KEEP_SCORE, LocalDetector

            # This detector's own thresholds, not aiming.py's -- see local.py.
            self._detector = LocalDetector(score=KEEP_SCORE, size=self.size)
            self._acquire_score = ACQUIRE_SCORE
        else:
            from aiming import ACQUIRE_SCORE
            from track_face_pi import Detector

            # YuNet's bar, for YuNet behind the HTTP service.
            self._detector = Detector(self.service)
            self._acquire_score = ACQUIRE_SCORE
        return self._detector

    # --- tools --------------------------------------------------------------

    def call(self, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
        """Perform one tool call. Never raises: an error is a result too.

        What comes back goes into the model's context verbatim, so a failure has
        to read as an explanation rather than a traceback -- the model repeats
        the gist of it out loud.
        """
        handler = getattr(self, f"_tool_{name}", None)
        if handler is None:
            return {"ok": False, "error": f"no such tool: {name}"}
        try:
            return handler(arguments)
        except (TypeError, ValueError) as error:
            return {"ok": False, "error": str(error)}
        except Exception as error:  # a bug here must not take the daemon down
            return {"ok": False, "error": f"{type(error).__name__}: {error}"}

    def _tool_set_vision(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Point `look` at whoever is asking. A control call, not a model tool.

        It is dispatched like a tool because that is the only protocol this
        daemon speaks, and it is deliberately absent from :meth:`tools`, so no
        model is ever shown it or can call it.

        It exists because the picture's destination was a constant, and a
        constant was wrong. `look` posts the JPEG straight to the model's host
        rather than passing it back through the client, which is what keeps a
        35kB frame off a desk that only has a microphone on it -- but it means
        this daemon has to be told an address, and it was told one at startup, by
        whoever last edited a crontab. When the model moved off that host the
        pictures kept going to it, and `look` failed with "No route to host"
        while everything else on the rover worked perfectly.

        So the client that is about to hold a conversation says where it is
        listening, every time it connects. The address it gives is the one its
        own socket to this daemon is bound to, so it is right by construction on
        a rover that has moved between eth0 and wlan0.

        Naming no address switches the picture path off, which also withdraws
        `look` from the tool list -- a tool that cannot reach the model's host is
        worse than a missing one.
        """
        address = arguments.get("address")
        if address is None or (isinstance(address, str) and not address.strip()):
            was, self.vision = self.vision, None
            if was is not None:
                was.close()
            return {"ok": True, "vision": None, "tools": [t["function"]["name"]
                                                          for t in self.tools()]}
        if not isinstance(address, str):
            return {"ok": False, "error": "set_vision wants an address like host:port"}
        host, _, port = address.strip().partition(":")
        if not host or (port and not port.isdigit()):
            return {"ok": False, "error": f"{address!r} is not a host:port"}
        link = VisionLink(address.strip())
        was, self.vision = self.vision, link
        if was is not None and was is not link:
            was.close()
        print(f"[rover] pictures now go to {link.describe()}", flush=True)
        return {"ok": True, "vision": link.describe(),
                "tools": [t["function"]["name"] for t in self.tools()]}

    def _tool_set_lights(self, arguments: dict[str, Any]) -> dict[str, Any]:
        level = _level(arguments.get("level"))
        with self._lock:
            if not self._send_lights(level):
                return {"ok": False, "error": "the driver board did not answer"}
            self.level = level
        return self._tool_get_lights({})

    def _tool_get_lights(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        return {"ok": True, "level": self.level, "on": self.level > 0}

    def _tool_look_at(self, arguments: dict[str, Any]) -> dict[str, Any]:
        from aiming import PAN_LIMIT, TILT_LIMITS

        stopped = self.stop_tracking()
        with self._lock:
            if "pan" in arguments and arguments["pan"] is not None:
                self.pan = min(max(_number(arguments["pan"], "pan"), -PAN_LIMIT), PAN_LIMIT)
            if "tilt" in arguments and arguments["tilt"] is not None:
                self.tilt = min(max(_number(arguments["tilt"], "tilt"),
                                    TILT_LIMITS[0]), TILT_LIMITS[1])
            if not self._send_gimbal():
                return {"ok": False, "error": "the driver board did not answer"}
            return {"ok": True, "pan": round(self.pan), "tilt": round(self.tilt),
                    "stopped_tracking": stopped}

    def _tool_center_camera(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        stopped = self.stop_tracking()
        if not self.centre_gimbal():
            return {"ok": False, "error": "the driver board did not answer"}
        return {"ok": True, "pan": 0, "tilt": 0, "stopped_tracking": stopped}

    def _tool_count_faces(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        if self.device is None:
            return {"ok": False, "error": "this rover has no camera attached"}
        # While the loop is running it owns the camera, so the honest answer is
        # what the loop last saw rather than a second opinion nobody can take.
        if self._tracking.is_set():
            seen = dict(self._seen)
            age = time.monotonic() - seen["at"] if seen["at"] else None
            if age is None or age > 2.0:
                return {"ok": False, "error": "tracking is running but has not seen a frame yet"}
            return {"ok": True, "count": seen["faces"], "faces": seen["where"],
                    "from": "the tracking loop"}
        # Deliberately outside the lock, and without opening the feed: a capture
        # holds nothing, so counting faces cannot delay a stop or a gimbal command
        # the way waiting on a camera under the lock could.
        got, why = self._snapshot()
        if not got:
            return {"ok": False, "error": f"the camera gave nothing: {why}"}
        # Newest first. More than one frame is worth having because the detector
        # rejecting a frame arrives here as the same None a dead service gives, so
        # a single undecodable picture used to read as "the host is away" -- but the
        # old reason for it has gone: these frames are whole buffers rather than
        # whatever a reader starting mid-stream happened to find, so the fragment
        # that made a cold `count_faces` fail every time cannot occur here.
        faces = None
        with self._detector_lock:
            detector = self._open_detector()
            for jpeg, at in reversed(got):
                faces = detector.detect(jpeg, at)
                if faces is not None:
                    break
        if faces is None:
            return {"ok": False,
                    "error": f"the face detector did not answer, or "
                             f"rejected {len(got)} frames running"}
        width, height = self.size
        where = [_where(face, width, height) for face in faces]
        return {"ok": True, "count": len(faces), "faces": where}

    def _whole_jpeg(self) -> tuple[bytes | None, str]:
        """One complete frame from the camera, or (None, why).

        Complete is checked rather than assumed, and it is still worth checking even
        though the capture below now hands back whole buffers: a picture is about to
        cross a network and be looked at by a model, and two bytes is a cheaper way
        to find out it is a fragment than either of those. Nothing is decoded on
        this machine; that costs 93 ms here and the picture is not for us.

        Newest of the three, because that is the frame the camera settled on -- see
        SNAPSHOT_FRAMES for why a cold camera's first frame is not the one to send.
        """
        if self._tracking.is_set():
            # The loop has the camera. Nothing else can open it, so the honest
            # answer is the newest frame the loop has seen -- which is also the
            # frame the camera is actually pointing at.
            frame = self._frame
            if frame is None:
                return None, "tracking is running but has not seen a frame yet"
            jpeg, at = frame
            if time.monotonic() - at > FRAME_STALE_S:
                return None, "tracking is running but its last frame is stale"
            # Raw when the local detector is in use, so this is where a picture
            # gets made. Only reached when somebody asks to see one, never in the
            # loop -- encoding costs about what decoding used to.
            if not jpeg.startswith(b"\xff\xd8"):
                encoder = self._detector
                jpeg = encoder.encode_jpeg(jpeg) if encoder is not None else None
                if jpeg is None:
                    return None, "the frame could not be turned into a picture"
            return jpeg, ""
        got, why = self._snapshot()
        if not got:
            return None, f"the camera gave nothing: {why}"
        for jpeg, _at in reversed(got):
            if jpeg.startswith(b"\xff\xd8"):
                return jpeg, ""
        return None, (f"the camera gave {len(got)} frames running that were not "
                      f"whole pictures")

    def _tool_look(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        if self.vision is None:
            return {"ok": False, "error": "this rover cannot show you a picture"}
        jpeg, why = self._whole_jpeg()
        if jpeg is None:
            return {"ok": False, "error": why}
        # The picture goes straight to the model's host; what comes back is the
        # name it was filed under, and that name is the whole of this result.
        # The client between here and there never sees the frame.
        sent = self.vision.post(jpeg)
        if not sent.get("ok"):
            return {"ok": False, "error": sent.get("error", "the picture was not accepted")}
        # Nothing but the name. This result carried a note once -- "the picture
        # is in front of you; describe what is actually in it" -- and that one
        # sentence, arriving immediately before the picture on every single
        # look, was read as an instruction for the turn: the model described the
        # whole picture whatever it had been asked, and took a fresh one for
        # every follow-up, 3/3 against 0/3 with the note removed. A tool result
        # is context, and context that reads like an order is an order. What the
        # model should do with a picture belongs in the system prompt, where it
        # is said once. See voice_chat/README.md.
        return {"ok": True, "image": sent["image"]}

    def _tool_camera_jpeg(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """One frame, as base64 JPEG in the reply. A control call, not a model tool.

        `look` is the model's version and posts the picture to the model's host,
        because a tool result cannot carry an image into a conversation. A window on
        a desk has no such problem, and routing a frame through a frame server to get
        it onto the screen of the machine that asked for it would be silly -- the
        same argument that gives `map_png` its own existence beside `show_map`.

        It needs a camera and not a vision host, which is the practical difference:
        a daemon started without `--vision` cannot `look` at all, and can still be
        asked for a picture from here.

        The bytes are the camera's own, undecoded. There is no image library on this
        Pi -- see [face_tracking/track_face_pi.py](../face_tracking/track_face_pi.py),
        where v4l2-ctl does the capturing precisely because of that -- so JPEG is the
        only thing this end can send, and turning it into something a widget can show
        is the caller's problem. `_whole_jpeg` still checks it is a whole picture and
        not the tail of one, which costs two bytes rather than the 93 ms a decode
        would.
        """
        if self.device is None:
            return {"ok": False, "error": "this rover has no camera attached"}
        jpeg, why = self._whole_jpeg()
        if jpeg is None:
            return {"ok": False, "error": why}
        width, height = self.size
        return {"ok": True, "bytes": len(jpeg), "width": width, "height": height,
                # Which of the two paths it came off, because they mean different
                # things: the loop's newest frame is what the camera is pointing at
                # while it sweeps, and a fresh grab is a camera opened for this call.
                "live": self._tracking.is_set(),
                "pan": round(self.pan), "tilt": round(self.tilt),
                "jpeg_base64": base64.b64encode(jpeg).decode("ascii")}

    def _detector_ready(self) -> str:
        """Empty if the face detector answers, otherwise why not, in a sentence.

        Face tracking needs the detector service, and its being away is still an
        expected state even now that it is on this same Pi -- it holds a USB
        device that can be unplugged, and it takes several seconds to boot that
        device after a restart. The loop is written to hold still through that
        rather than to die. Which is right for a loop already running
        and wrong for one being started: the loop starts, holds still, reports
        itself as tracking, and the model says "I started tracking people" while
        the camera never moves. That is the failure this whole directory's prompt
        wording exists to prevent, arriving from underneath the prompt.

        A refusal is instant and a host that is off takes the timeout, which is
        why this is bounded rather than left to the first detect call.

        For the detector in this process there is no socket to probe, so the
        readiness question is whether the device opens -- which is the same
        question, and answering it here means the several seconds of firmware and
        graph upload are paid once, on the first call, rather than being mistaken
        for a camera that will not answer.
        """
        if self.service == "local":
            try:
                self._open_detector()
                return ""
            except Exception as error:
                return (f"the OAK could not be opened ({error}), so tracking a "
                        f"face is not possible right now")
        host, _, port = self.service.partition(":")
        try:
            with socket.create_connection((host, int(port or 8768)), DETECT_PROBE_S):
                return ""
        except OSError as error:
            return (f"the face detector at {self.service} is not answering "
                    f"({error.strerror or type(error).__name__}), so tracking a "
                    f"face is not possible right now")

    def _tool_start_tracking(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        if self.device is None:
            return {"ok": False, "error": "this rover has no camera attached"}
        if self._tracking.is_set():
            return {"ok": True, "tracking": True, "already": True}
        if (why := self._detector_ready()):
            return {"ok": False, "error": why}
        self._tracking.set()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return {"ok": True, "tracking": True}

    def _tool_stop_tracking(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        was = self.stop_tracking()
        return {"ok": True, "tracking": False, "was_tracking": was}

    def _tool_track_next(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        started = False
        if not self._tracking.is_set():
            result = self._tool_start_tracking({})
            if not result.get("ok"):
                return result
            started = True
        with self._lock:
            centre = self._seen.get("centre")
            if centre is None and not started:
                return {"ok": True, "skipped": False,
                        "note": "nobody is being followed at the moment, so there is "
                                "nobody to let go of"}
            self._skip_centre = centre
            self._skip_until = time.monotonic() + SKIP_FOR_S
        return {"ok": True, "skipped": True, "started": started,
                "note": "the rover cannot tell people apart, so it may find the same "
                        "person again if nobody else is in view"}

    def _tool_tracking_status(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        seen = dict(self._seen)
        fresh = seen["at"] and time.monotonic() - seen["at"] < 2.0
        status = {
            "ok": True,
            "tracking": self._tracking.is_set(),
            "following_someone": bool(seen["locked"]) if fresh else False,
            "faces_in_view": seen["faces"] if fresh else 0,
            "pan": round(self.pan), "tilt": round(self.tilt),
        }
        # How fast the loop is actually going round, which is the difference
        # between "it cannot see me" and "it sees me twice a second". Only the
        # in-process detector keeps these; over HTTP the service's own /health
        # has them instead.
        detector = self._detector
        if detector is not None and hasattr(detector, "convert_ms"):
            status["loop_fps"] = round(self._loop_fps, 1)
            status["frame_ms"] = {"convert": round(detector.convert_ms, 1),
                                  "detect": round(detector.detect_ms, 1)}
            status["acquire_at"] = self._acquire_score
            status["recent_scores"] = list(detector.recent)
        return status

    # --- driving --------------------------------------------------------------

    def park_tracking(self) -> None:
        """Give the core to the scan matcher, because the wheels are about to turn.

        Called by the navigator the instant before anything moves, and the one place
        that decides what driving outranks. Face tracking and driving cannot both
        run. Face tracking holds the camera and now decodes every frame here as
        well as posting it -- measured at 64-87 ms of JPEG per frame since the
        detector moved onto the rover, well over the 30% of this core that
        forwarding alone used to cost -- and SLAM is another 33%; run both and the
        scan matcher starts dropping revolutions, which degrades exactly the thing
        that is keeping the rover off the walls. Aiming the camera while driving is also a good way
        to be looking at somebody's face when something appears in front of the
        tracks.

        The camera feed is released as well as the loop that was reading it, and not
        only the loop, because those are two different things and the second used to
        be missed. Tracking closes its own camera on the way out, so ordinarily this
        finds nothing left to do -- but a feed nobody is reading costs the matcher
        just as much as one somebody is, and it is exactly what a crash between
        opening the camera and entering the loop leaves behind. Twenty seconds of
        CAMERA_IDLE_S is a long time to be driving on a degraded map.
        """
        self._tracking_parked = self.stop_tracking()
        with self._lock:
            self._close_camera()

    def unpark_tracking(self) -> None:
        """Give it back, but only if driving is what took it."""
        if not self._tracking_parked:
            return
        self._tracking_parked = False
        result = self._tool_start_tracking({})
        if not result.get("ok"):
            print(f"[rover] could not resume face tracking after driving: "
                  f"{result.get('error')}", file=sys.stderr, flush=True)

    def _tool_drive(self, arguments: dict[str, Any]) -> dict[str, Any]:
        if self.nav is None:
            return {"ok": False, "error": "this rover has no lidar, so it will not "
                                          "drive itself"}
        distance = _number(arguments.get("distance_m", 0.5), "distance_m")
        speed = arguments.get("speed_ms")
        outcome = self.nav.drive(distance_m=distance,
                                 speed_ms=None if speed is None
                                 else _number(speed, "speed_ms"))
        return {"ok": outcome.reason in ("arrived", "timed out"), **outcome.asdict(),
                **self._nav_context()}

    def _tool_drive_to(self, arguments: dict[str, Any]) -> dict[str, Any]:
        if self.nav is None:
            return {"ok": False, "error": "this rover has no lidar, so it will not "
                                          "drive itself"}
        ahead = _number(arguments.get("ahead_m", 0.0), "ahead_m")
        left = _number(arguments.get("left_m", 0.0), "left_m")
        speed = arguments.get("speed_ms")
        outcome = self.nav.drive_to(ahead, left,
                                    speed_ms=None if speed is None
                                    else _number(speed, "speed_ms"))
        return {"ok": outcome.reason in ("arrived", "timed out"), **outcome.asdict(),
                **self._nav_context()}

    def _tool_turn_in_place(self, arguments: dict[str, Any]) -> dict[str, Any]:
        if self.nav is None:
            return {"ok": False, "error": "this rover has no lidar, so it will not "
                                          "drive itself"}
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
            return {"ok": False, "error": "this rover has no lidar attached"}
        return {"ok": True, **self.nav.describe()}

    def _camera_cone(self) -> tuple[float, float] | None:
        """The gimbal as `(bearing_deg, fov_deg)` for the map, or None with no camera.

        **The two conventions are opposite, and this minus sign is the whole of the
        conversion.** The gimbal takes pan positive to the *right* (see `look_at`);
        the lidar, the map and everything in [lidar_slam/](../lidar_slam) take
        bearings positive to the *left*, counter-clockwise from straight ahead. Get
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
            return {"ok": False, "error": "this rover has no lidar attached"}
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

    def _tool_nav_status(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """Every number the driving loop has. A control call, not a model tool.

        Dispatched like a tool because that is the only protocol here, and absent
        from :meth:`tools` so no model is shown it. What is in this and not in
        `describe_surroundings` is the machinery rather than the room -- the PWM
        actually on the motors, the turn rate the matcher measures, how stale the
        last scan is. That is what tells you why a move went wrong, and it is of no
        use whatsoever to something that has to say the answer out loud.

        Written for [voice_chat/drive_console.py](../voice_chat/drive_console.py),
        which polls it a few times a second while somebody drives by hand.
        """
        if self.nav is None:
            return {"ok": False, "error": "this rover has no lidar attached"}
        return {"ok": True, **self.nav.status()}

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
            return {"ok": False, "error": "this rover has no lidar attached"}
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
            return {"ok": False, "error": "this rover has no lidar attached"}
        result = self.nav.clear_map()
        return {"ok": bool(result.get("cleared")), **result}

    # --- the loop -----------------------------------------------------------

    def stop_tracking(self) -> bool:
        """Stop the loop and wait for it to let go. True if it had been running."""
        if not self._tracking.is_set():
            return False
        self._tracking.clear()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=5.0)
        self.centre_gimbal()
        return True

    def _loop(self) -> None:
        """The face-tracking control loop, from track_face_pi.py's main().

        Deliberately the same `aiming.py` as the standalone script rather than a
        second control law: two implementations of how this rover aims would
        become two different robots, which is the reason aiming.py exists.
        """
        from aiming import (
            GAIN, GRACE_FRAMES, LOST_GRACE_S, MAX_DT, SCAN_AFTER_S, SCAN_RATE,
            Gimbal, Scan, Target, clamp, scan_rate_for,
        )

        width, height = self.size
        try:
            with self._lock:
                camera = self._open_camera()
            detector = self._open_detector()
        except Exception as error:
            print(f"[rover] cannot start tracking: {error}", file=sys.stderr, flush=True)
            self._tracking.clear()
            return

        gimbal = Gimbal(clamp(GAIN, 0.05, 1.0), self.size)
        # The angles are a model; this is what makes the model true. Start it
        # from wherever the camera actually is rather than assuming centre.
        gimbal.pan, gimbal.tilt = self.pan, self.tilt
        target = Target(self._acquire_score)
        scan = None
        last_tick = time.monotonic()
        self._loop_fps = 0.0
        service_ok_at = time.monotonic()
        stalled = False

        try:
            while self._tracking.is_set():
                got = camera.latest(timeout=1.0)
                now = time.monotonic()
                if got is None:
                    if not camera.alive():
                        why = "; ".join(camera.complaints) or "it stopped without saying why"
                        print(f"[rover] the camera stopped: {why}", file=sys.stderr, flush=True)
                        break
                    continue
                frame, exposed_at = got
                # Kept before the detector is consulted rather than after, so
                # that `look` still has a picture during the seconds when the
                # detector is not answering and this loop is only holding still.
                self._frame = (frame, now)
                # Clamped: a frame that took a second to arrive must not be
                # answered with a second's worth of sweep.
                dt, last_tick = min(now - last_tick, MAX_DT), now
                # Exponentially smoothed rather than instantaneous: one slow frame
                # is not news, a loop that has halved is.
                if dt > 0:
                    self._loop_fps += 0.2 * (1.0 / dt - self._loop_fps)
                # Hold a lock for whichever is longer, the measured 0.7 s or four
                # frames. This loop runs at four frames a second with the detector
                # on the rover, where 0.7 s is not "a frame or two" but two and a
                # half, and one turn of a head then drops somebody the camera is
                # still pointing straight at.
                target.grace = max(LOST_GRACE_S, GRACE_FRAMES * dt)

                faces = detector.detect(frame, exposed_at)
                if faces is None:
                    if now - service_ok_at > 3.0 and not stalled:
                        print("[rover] the face detector is not answering; holding still",
                              file=sys.stderr, flush=True)
                        target.drop()
                        scan = None
                        stalled = True
                    continue
                if stalled:
                    stalled, last_tick, dt = False, now, 0.0
                service_ok_at = now

                # "Show me somebody else": drop detections near whoever was being
                # followed, so Target acquires the next largest face instead. Done
                # here rather than in aiming.py, which is shared with the desktop
                # script and has no business knowing about conversations.
                visible = faces
                if self._skip_centre is not None and now < self._skip_until:
                    x0, y0 = self._skip_centre
                    visible = [
                        face for face in faces
                        if math.hypot(face[0] + face[2] / 2 - x0,
                                      face[1] + face[3] / 2 - y0)
                        >= max(face[2], 60) * SKIP_RADIUS
                    ]
                elif self._skip_centre is not None:
                    self._skip_centre = None

                tracking = target.update(visible, now)
                if tracking:
                    scan = None
                    # Positive x is right of centre and positive y is *above* it,
                    # which is not the picture's own row order.
                    error_x = (target.centre[0] - width / 2) / (width / 2)
                    error_y = (height / 2 - target.centre[1]) / (height / 2)
                    gimbal.track(error_x, error_y, dt, now, exposed_at=exposed_at)
                else:
                    if target.centre is not None:
                        target.drop()
                    if now - target.seen_at > SCAN_AFTER_S:
                        if scan is None:
                            scan = Scan(gimbal)
                        scan.step(gimbal, scan_rate_for(dt), dt)
                gimbal.record(now)

                with self._lock:
                    if gimbal.changed():
                        self.pan, self.tilt = gimbal.pan, gimbal.tilt
                        self._send_gimbal()
                    self._seen = {
                        "faces": len(faces),
                        "locked": tracking,
                        "at": now,
                        "centre": target.centre if tracking else None,
                        "where": [_where(face, width, height) for face in faces],
                    }
        finally:
            self._tracking.clear()
            self._seen = {"faces": 0, "locked": False, "at": 0.0, "where": []}
            self._frame = None
            with self._lock:
                self._close_camera()

    def idle_tick(self) -> None:
        """Let go of the camera when nothing has wanted it for a while."""
        with self._lock:
            if (self._camera is not None and not self._tracking.is_set()
                    and time.monotonic() - self._camera_used > CAMERA_IDLE_S):
                self._close_camera()

    def close(self) -> None:
        # The navigator first, and outside the lock: it stops the wheels and joins
        # its own loop, and nothing else here matters until the rover is still.
        if self.nav is not None:
            self.nav.close()
            self.nav = None
        self.stop_tracking()
        with self._lock:
            self._close_camera()
            if self._detector is not None:
                self._detector.close()
            if self.vision is not None:
                self.vision.close()
            self.link.close()


class Handler(socketserver.StreamRequestHandler):
    """One client connection: newline-delimited JSON, one reply per request."""

    def handle(self) -> None:
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        rover: Rover = self.server.rover
        for raw in self.rfile:
            raw = raw.strip()
            if not raw:
                continue
            try:
                request = json.loads(raw)
            except ValueError:
                reply = {"ok": False, "error": "not JSON"}
            else:
                name = request.get("call")
                if name == "list_tools":
                    reply = {"ok": True, "tools": rover.tools()}
                elif not isinstance(name, str):
                    reply = {"ok": False, "error": "every request needs a 'call'"}
                else:
                    reply = rover.call(name, request.get("arguments") or {})
            try:
                self.wfile.write(json.dumps(reply).encode() + b"\n")
            except OSError:
                return  # the client went away mid-reply; nothing to say about it


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main() -> int | str:
    """Returns 0, or a message -- sys.exit prints a string and exits non-zero."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--serial", default=DEFAULT_SERIAL, metavar="PORT",
                        help=f"the ESP32's serial port (default {DEFAULT_SERIAL})")
    parser.add_argument("--host", default=None, metavar="ADDRESS",
                        help="command the board over WiFi at this address instead")
    parser.add_argument("--service", default=DEFAULT_SERVICE, metavar="HOST[:PORT]",
                        help=f"the face detector: 'local' for the OAK in this "
                             f"process, or host:port for one over HTTP "
                             f"(default {DEFAULT_SERVICE})")
    parser.add_argument("--device", default=DEFAULT_DEVICE, metavar="PATH",
                        help=f"the camera (default {DEFAULT_DEVICE})")
    parser.add_argument("--vision", nargs="?", default=None, const=DEFAULT_VISION,
                        metavar="HOST[:PORT]",
                        help="offer the 'look' tool, posting frames to this vision "
                             f"service (bare --vision means {DEFAULT_VISION})")
    parser.add_argument("--no-camera", dest="camera", action="store_false",
                        help="lights and gimbal only, for a rover with no camera fitted")
    parser.add_argument("--lidar", nargs="?", default=None, const=DEFAULT_LIDAR,
                        metavar="PORT",
                        help="offer the driving and mapping tools, using the lidar on "
                             "this port; bare --lidar finds it by its stable "
                             "/dev/serial/by-id name. Without this the rover will "
                             "not move itself.")
    parser.add_argument("--camera-fov", type=float, default=CAMERA_FOV_DEG,
                        metavar="DEGREES",
                        help="how wide a slice of the room the camera sees across "
                             "the picture, drawn on the map as the gimbal's cone. "
                             "The default is a guess -- measure it by panning until "
                             "a known object just leaves the frame.")
    parser.add_argument("--bind", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    try:
        link = open_link(args.serial, args.host)
    except Exception as error:
        return f"Cannot reach the driver board: {error}"

    rover = Rover(link, args.service, args.device if args.camera else None,
                  vision=args.vision, camera_fov_deg=args.camera_fov)
    if not rover.probe():
        link.close()
        return f"No answer from the driver board on {link.describe()}. Is it powered?"
    # The one thing that moves before anybody asks: the gimbal angles are a model
    # kept true by putting the camera where this thinks it is, since it cannot ask.
    rover.centre_gimbal()

    if args.lidar:
        # Two layouts to satisfy: in the repository this file is in rover_daemon/ and
        # lidar_slam/ is its sibling, while the Pi's ~/ugv is flat with lidar_slam/
        # inside it. Checking for the directory rather than assuming either means a
        # deployment that moves does not silently lose the driving tools.
        here = Path(__file__).resolve().parent
        for candidate in (here.parent / "lidar_slam", here / "lidar_slam"):
            if candidate.is_dir():
                sys.path.insert(0, str(candidate))
                break
        try:
            from navigator import Navigator
            rover.nav = Navigator(link,
                                  None if args.lidar == "auto" else args.lidar,
                                  on_drive_start=rover.park_tracking,
                                  on_drive_end=rover.unpark_tracking)
            # The port is opened by its loop, not here, and retried until it turns
            # up: on this Pi the lidar enumerates 93 s after the kernel starts, long
            # after cron has run this, so insisting on it now would mean every
            # reboot came up without the driving tools.
            rover.nav.start()
        except Exception as error:
            # Not fatal. A rover that cannot drive itself is still a rover that can
            # light up, aim its camera and hold a conversation, and the driving tools
            # simply will not be offered.
            rover.nav = None
            print(f"[rover] no driving or mapping: {error}", file=sys.stderr,
                  flush=True)

    server = Server((args.bind, args.port), Handler)
    server.rover = rover
    print(f"rover daemon on {args.bind}:{args.port} -- board {rover.describe()}, "
          f"camera {args.device if args.camera else 'none'}, detector {args.service}, "
          f"vision {rover.vision.describe() if rover.vision else 'off'}, "
          f"lidar {(rover.nav.lidar_path or 'waiting for it') if rover.nav else 'off'} "
          f"({len(rover.tools())} tools)",
          flush=True)

    def release_idle_camera() -> None:
        while True:
            time.sleep(5.0)
            rover.idle_tick()

    threading.Thread(target=release_idle_camera, daemon=True).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        print("\nstopping")
        server.server_close()
        rover.close()
        print("camera released, gimbal centred")
    return 0


if __name__ == "__main__":
    sys.exit(main())
