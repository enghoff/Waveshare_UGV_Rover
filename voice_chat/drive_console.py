#!/usr/bin/env python3
"""Drive the rover by hand through the daemon, and watch what it says back.

[talk.py](talk.py) moves the rover by talking to a
model that decides which tool to call. That is the point of it, and it is also
what makes them the wrong instrument for finding out whether a 90 degree turn
turns 90 degrees: every number arrives paraphrased, seconds late, from a model
that may have asked for something else entirely and will describe whatever
happened as a success.

This is the same tools with the model taken out. Buttons in, JSON out, and the
numbers shown as they arrive. A turn that comes back a third of what was asked for
shows up here as a ratio in a table rather than as a rover that looked a bit
wrong.

    python voice_chat/drive_console.py                     # finds the rover
    python voice_chat/drive_console.py --rover rpi.local:8769
    python voice_chat/drive_console.py --half-extent 4.0   # a wider map to start on

Only the standard library, plus [rover_tools.py](rover_tools.py) for the wire
protocol. tkinter ships with Python, which matters because this is a diagnostic,
and a diagnostic that needs something installed first is one you will not have when
you need it.

**The map zooms**, on two separate knobs, because they are two separate questions.
"Across" changes how many metres are in frame and has to be asked of the rover,
since the cropping happens where the grid lives; `-` and `+` step it through a
fixed ladder so the same extents come back and one picture can be compared with an
earlier one. "Detail" changes pixels per 5 cm cell, which is the one that costs:
the picture's area goes as its square and there is no drawing library on the Pi, so
the daemon caps the total and lowers the detail rather than spend half a minute on
one map. The line under the buttons reports what came back -- size, kilobytes, time
to draw, and whether the detail was capped -- so a setting that the rover declined
does not look like a setting that did nothing.

**Five connections, deliberately.** A `drive` call does not answer until the move
has finished, and `RoverClient` serialises calls on one socket, so anything sharing
that socket would queue behind the move. Stop must never queue, and neither should
the status poll that tells you what the move is doing while it does it -- so moves,
stop, and watching each get their own. The daemon is a threading server and holds
no lock across a move, so the others are answered while the first is driving.

Clicking the map sends `drive_to`: the rover plans a route of segments and turns
to that point, relative to where it is now, and follows it with the lidar in the
loop. Stop still interrupts it.

The map gets a fourth for the same reason one step down: drawing it costs the Pi a
second and a half, sometimes several, and it shared the status connection until that
was measured -- so the numbers went stale exactly while the picture was being
drawn. It is the slowest thing here and it is the least urgent, which is a good
argument for its own socket and a poor one for sharing.

The camera gets a fifth, and it is the slowest of the lot: opening the camera and
waiting for its first buffer takes the rover up to four seconds, so a picture on
the status connection would blank the numbers for longer than a map ever did.

**The other sensor, and the board.** Beside the map there is a panel for the
camera, for face tracking and for the headlights. The picture is worth having next
to the map rather than instead of it: the map draws the camera's cone, and the two
together are what say which part of the room a photograph is of. Tracking is polled
rather than remembered, because the daemon puts it down by itself -- driving parks
it, since the tracking loop and SLAM cannot share this Pi -- so a window that only
updated when you pressed something would go on claiming the camera was following
somebody long after a drive took it away.

Showing a frame is the one thing here that needs a library. The rover sends JPEG
because that is all it can send -- there is no image library on that Pi, which is
why face detection happens on another host -- and tkinter reads PNG, GIF and PPM.
OpenCV does the decode, it is already in the repo's requirements.txt, and where it
is missing the frame is written to a file and the window says where.

**Clearing the map** throws the occupancy grid away and stands the rover at the
origin of an empty one. Drift here is permanent -- there is no loop closure on this
hardware and never will be -- so a map that has come out of true with itself will
stay that way, and an empty map that fills back in over a few revolutions is worth
more than a confident wrong one. It takes two presses rather than a confirmation
box, because a modal dialog stops the event loop this window's stop button lives in.

**What it will not do.** There is no continuous teleop here, because the daemon
offers none: every move it exposes is bounded, in metres or in degrees, and it
watches the lidar for the whole of it. Holding a key down to creep forward would
mean a stream of short moves, which is worse driving than one long one and a poor
way to measure anything. For seat-of-the-pants teleop,
[driver_board/drive_gamepad.py](../driver_board/drive_gamepad.py) talks straight to
the ESP32 and has no Pi, no SLAM and no standoff in it at all.

Closing the window sends a stop. The rover only moves for as long as one bounded
command lasts, but a window that can start a move and cannot be trusted to end one
is not a thing to leave lying around.
"""

from __future__ import annotations

import argparse
import base64
import math
import os
import queue
import sys
import tempfile
import threading
import time
import tkinter as tk
from tkinter import ttk
from typing import Any

import rover_tools

POLL_S = 0.3               # how often to ask for nav_status while connected
MAP_AUTO_S = 2.0           # how often to refresh the map when auto is ticked
TICK_MS = 100              # the reply pump, and the in-flight timer
LOG_LINES = 500            # trimmed, so an afternoon of testing does not grow forever
TURN_ROWS = 40
# drive_to can take a minute of segments and turns; the default client timeout is
# 12 s, which is right for a single hop and wrong for a route.
MOVE_TIMEOUT_S = 90.0

# Preset turns, as magnitudes. Laid out as two columns with the left turns in the
# left one and the right turns in the right one, increasing downwards, so a button's
# place on screen matches the way the rover is about to go.
#
# The sizes cover the range that matters rather than making a keypad: the small ones
# are where the coast after the power comes off is a large fraction of the whole
# turn, and 90 is what a model asks for when it wants to face another way.
TURN_PRESETS_DEG = (15, 45, 90)

# How far each way the map covers, as a ladder rather than a zoom multiplier, so the
# same handful of extents come back and one picture can be compared with an earlier
# one.
#
# It stops at 6 m -- twelve metres across -- for two reasons. The pose drifts, so a
# wider map invites planning a route home that the map cannot support; and past there
# the picture can no longer be held at a steady size, because a cell must be a whole
# number of pixels and at 12 m across it is already down to two. The daemon will go
# to 10 m for a caller that asks, and says how big what it drew came out.
MAP_EXTENTS_M = (0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0)

# How big a picture to ask for. This is a separate control from the zoom, and the
# separation is the point: widening the view must show more room in the same picture,
# not send back a bigger picture. The rover works out pixels per cell from the two.
MAP_SIZES_PX = (320, 480, 640, 800)

# The camera panel. Frames arrive at whatever the daemon captures -- 640x480 on this
# rover -- and are scaled to this to sit beside the map without dominating it.
CAMERA_BOX_PX = 320
# Auto-refresh for the picture, and it is off until asked for. Face tracking and
# SLAM already cannot share this one core, and a picture every few seconds opens the
# camera on the same core the driving loop is using -- so continuous frames are a
# thing to switch on while looking at something, not a thing to leave running while
# measuring a turn.
CAMERA_AUTO_S = 3.0

# How often to ask what face tracking is doing. It reads state the daemon already
# has -- no camera, no detector, no lock -- so it is cheap, and it has to be asked
# rather than remembered because the daemon parks tracking by itself when the wheels
# turn.
TRACK_POLL_S = 2.0

# How long "clear map" stays armed after the first press. Deliberately not a
# confirmation dialog: a modal window stops the tk event loop, and that loop is where
# the stop button and the status poll live, so the rover would be unstoppable from
# here for as long as somebody left the box open. A map thrown away by accident costs
# a minute of driving; that costs whatever the rover hits.
CLEAR_ARM_S = 4.0

LIGHT_MAX = 255            # what the daemon calls full brightness


def _legend():
    """Swatch colours and labels for the map key, taken from the renderer itself.

    The rover's own `mapimg` is the authority on what colour means what, and a key
    that has drifted from the picture is worse than no key at all -- so the palette
    is imported rather than copied, and if it cannot be found the key is simply
    omitted. It is a label on a picture; it is not worth failing to start over.
    """
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "lidar_slam"))
    try:
        import mapimg
    except ImportError:
        return ()
    return tuple(("#%02x%02x%02x" % colour, label) for colour, label in (
        (mapimg.C_ROVER, "rover"),
        (mapimg.C_CAMERA, "camera"),
        (mapimg.C_TRACK, "driven"),
        (mapimg.C_OCCUPIED, "solid"),
        (mapimg.C_FREE, "empty"),
        (mapimg.C_DIM, "unsure"),
        (mapimg.C_UNKNOWN, "unseen")))


MAP_LEGEND = _legend()


def _photo(jpeg, box):
    """A JPEG as something tkinter can show, scaled to `box` across, or (None, why).

    Tk reads PNG, GIF and PPM, and does not read JPEG. The rover cannot send anything
    else: there is no image library on that Pi at all -- v4l2-ctl produces MJPEG and
    the daemon forwards those bytes without decoding them, which is the same reason
    face detection runs on another host. So the decode lands at this end, and it is
    the one thing in this window that needs something installed. OpenCV is already in
    the repo's requirements.txt, so a machine that runs anything else here has it, and
    where it is missing the caller writes the frame to a file instead -- which beats a
    blank panel and keeps every other control in the window working.

    PPM rather than PNG on the way out because PPM needs no encoder: the bytes are the
    pixels. They go to Tk raw, not base64 -- Tk's PPM reader does not accept base64,
    only the compressed formats do, and a base64 PPM comes back as "couldn't recognize
    image data".
    """
    try:
        import cv2
        import numpy as np
    except ImportError:
        return None, ("OpenCV is not installed here, so the picture cannot be put on "
                      "screen (pip install opencv-python)")
    image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        return None, "those bytes did not decode as a picture"
    height, width = image.shape[:2]
    if width > box:
        image = cv2.resize(image, (box, max(1, round(height * box / width))),
                           interpolation=cv2.INTER_AREA)
        height, width = image.shape[:2]
    # OpenCV is BGR and PPM is RGB. Getting this backwards produces a picture that
    # looks entirely plausible until somebody wears a red jumper.
    ppm = b"P6\n%d %d\n255\n" % (width, height) + image[:, :, ::-1].tobytes()
    return tk.PhotoImage(data=ppm), ""


def _save_frame(jpeg):
    """The frame on disk, for when it cannot go on screen. One file, overwritten:
    a window left open for an afternoon should not fill a disk with pictures of a
    carpet."""
    path = os.path.join(tempfile.gettempdir(), "rover-frame.jpg")
    with open(path, "wb") as handle:
        handle.write(jpeg)
    return path


def _f(value, spec="{}"):
    """Format, or a dash. Every number in nav_status may legitimately be absent."""
    return "-" if value is None else spec.format(value)


# The fields of nav_status that get a permanent place on screen, in reading order.
STATUS_FIELDS = (
    # Two lidar rows, because they mean different things and the difference is the
    # one thing a dead-reckoned turn makes visible: the sensor keeps reporting all
    # the way through a turn while the map is suspended, so "matched" goes stale on
    # a rover whose lidar is perfectly healthy. Both false together is the sensor
    # actually having died -- which this one does, under motor load.
    ("lidar_live", "lidar", lambda v: "reporting" if v else "SILENT"),
    ("lidar_ok", "matched", lambda v: "current" if v else "stale"),
    ("scan_age_s", "scan age", lambda v: _f(v, "{:.2f} s")),
    ("driving", "driving", lambda v: "yes" if v else "no"),
    ("estop", "e-stop", lambda v: "LATCHED" if v else "clear"),
    ("speed_ms", "speed", lambda v: _f(v, "{:.3f} m/s")),
    ("turn_dps", "turn rate", lambda v: _f(v, "{:.1f} deg/s")),
    ("pwm", "pwm L,R", lambda v: "-" if not v else f"{v[0]}, {v[1]}"),
    ("clearance_m", "clearance", lambda v: _f(v, "{:.2f} m")),
    ("steering_deg", "steering", lambda v: _f(v, "{:.1f} deg")),
    ("remaining_m", "to go", lambda v: _f(v, "{:.2f} m")),
    ("match_score", "match", lambda v: _f(v, "{:.3f}")),
    ("position_trusted", "position", lambda v: "trusted" if v else "NOT TRUSTED"),
    ("scans", "scans", lambda v: _f(v)),
    ("dropped_scans", "dropped", lambda v: _f(v)),
)

# Which fields shout when they are wrong. A silent lidar and an untrusted position
# each make every other number on the panel a lie, so neither is left as a quiet
# "False" among a dozen other rows.
ALARM_WHEN_FALSE = ("lidar_live", "position_trusted")
ALARM_WHEN_TRUE = ("estop",)


class Reply:
    """One answered call, on its way back to the window."""

    def __init__(self, name, arguments, body, seconds):
        self.name = name
        self.arguments = arguments
        self.body = body
        self.seconds = seconds


class Channel:
    """A `RoverClient` on its own thread: submit a call, get a `Reply` on a queue.

    One call at a time and in order, which is what the daemon expects anyway -- a
    tool call is a physical act on a rover, and there is no throughput to win by
    having two of them in flight.
    """

    def __init__(self, label: str, address: str, replies: queue.Queue,
                 timeout: float | None = None) -> None:
        self.label = label
        self.client = rover_tools.RoverClient(
            address, timeout=rover_tools.TIMEOUT_S if timeout is None else timeout)
        self._replies = replies
        self._work: queue.Queue = queue.Queue()
        threading.Thread(target=self._run, daemon=True,
                         name=f"rover-{label}").start()

    def submit(self, name: str, arguments: dict[str, Any] | None = None) -> None:
        self._work.put((name, arguments or {}))

    def _run(self) -> None:
        while True:
            item = self._work.get()
            if item is None:
                return
            name, arguments = item
            began = time.monotonic()
            if name == "list_tools":
                # Not a tool, so it does not go through `call` -- and `tools` raises
                # where `call` returns, so the two shapes are reconciled here.
                try:
                    body = {"ok": True, "tools": self.client.tools()}
                except ConnectionError as error:
                    body = {"ok": False, "error": str(error)}
            else:
                body = self.client.call(name, arguments)
            self._replies.put(Reply(name, arguments, body,
                                    time.monotonic() - began))

    def close(self) -> None:
        self._work.put(None)
        self.client.close()


class Console:
    """The window, and the four channels behind it."""

    def __init__(self, root: tk.Tk, address: str | None, half_extent: float,
                 map_size: int) -> None:
        self.root = root
        self.half_extent = half_extent
        self.map_size = map_size
        self.replies: queue.Queue = queue.Queue()

        self.moves: Channel | None = None     # blocking, one bounded move at a time
        self.halt: Channel | None = None      # stop, and nothing else, ever
        self.watch: Channel | None = None     # status, questions, tool list
        self.picture: Channel | None = None   # the map, which is slow enough to matter
        self.camera: Channel | None = None    # frames, which are slower still
        self.channels: list[Channel] = []

        self.tools: list[str] = []
        self.can_drive = False
        self.busy_since: float | None = None
        self.busy_name = ""
        self.poll_outstanding = False
        self.poll_at = 0.0
        self.map_outstanding = False
        self.map_wanted = False        # the view moved while one was already in flight
        self.map_at = 0.0
        self.map_cost = 0.0            # how long the rover said the last one took
        self.map_image: tk.PhotoImage | None = None
        self.map_view: dict[str, Any] | None = None
        self._heading_rad = 0.0
        self.frame_outstanding = False
        self.frame_at = 0.0
        self.frame_cost = 0.0          # how long the last one took to arrive
        self.frame_image: tk.PhotoImage | None = None
        self.track_outstanding = False
        self.track_at = 0.0
        self.light_level: int | None = None
        self.clear_armed_until = 0.0

        root.title("rover drive console")
        # Wide enough for the numbers, the map and the camera side by side. They earn
        # the width: the map draws the cone the camera is looking down, so reading one
        # against the other is the point of having both.
        root.minsize(1220, 700)
        self._build()

        self.address_var.set(address or "")
        self._connect()
        root.protocol("WM_DELETE_WINDOW", self._quit)
        root.after(TICK_MS, self._tick)

    # --- layout ---------------------------------------------------------------
    def _build(self) -> None:
        root = self.root
        root.columnconfigure(1, weight=1)
        root.rowconfigure(1, weight=1)

        bar = ttk.Frame(root, padding=(8, 6))
        bar.grid(row=0, column=0, columnspan=2, sticky="ew")
        ttk.Label(bar, text="rover").pack(side="left")
        self.address_var = tk.StringVar()
        entry = ttk.Entry(bar, textvariable=self.address_var, width=22)
        entry.pack(side="left", padx=6)
        entry.bind("<Return>", lambda _e: self._connect())
        ttk.Button(bar, text="connect", command=self._connect).pack(side="left")
        self.link_var = tk.StringVar(value="not connected")
        ttk.Label(bar, textvariable=self.link_var).pack(side="left", padx=12)
        self.busy_var = tk.StringVar(value="")
        ttk.Label(bar, textvariable=self.busy_var,
                  font=("TkDefaultFont", 10, "bold")).pack(side="right")

        left = ttk.Frame(root, padding=(8, 0, 4, 8))
        left.grid(row=1, column=0, sticky="ns")
        right = ttk.Frame(root, padding=(4, 0, 8, 8))
        right.grid(row=1, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(2, weight=1)

        self._build_controls(left)
        self._build_status(right)
        self._build_log(right)

    def _build_controls(self, parent: ttk.Frame) -> None:
        # Only buttons are disabled while a move runs, never the entry boxes: a
        # greyed-out field you cannot type the next distance into is a nuisance, and
        # typing a number is not an act on the rover.
        self.buttons: list[tk.Widget] = []

        # Stop first and at the top, because it is the one control here that has to
        # be findable without looking for it.
        tk.Button(parent, text="STOP", command=self._stop, bg="#b02020", fg="white",
                  height=2, font=("TkDefaultFont", 12, "bold")
                  ).pack(fill="x", pady=(0, 10))

        turn = ttk.LabelFrame(parent, text="turn in place", padding=8)
        turn.pack(fill="x")
        grid = ttk.Frame(turn)
        grid.pack(fill="x")
        # Positive is left in this module, and left is column 0.
        for row, magnitude in enumerate(TURN_PRESETS_DEG):
            for column, sign in ((0, 1), (1, -1)):
                angle = sign * magnitude
                button = ttk.Button(
                    grid, text=f"{magnitude} {'left' if sign > 0 else 'right'}",
                    width=10, command=lambda a=angle: self._turn(a))
                button.grid(row=row, column=column, padx=2, pady=2)
                self.buttons.append(button)
        row = ttk.Frame(turn)
        row.pack(fill="x", pady=(6, 0))
        ttk.Label(row, text="angle").pack(side="left")
        self.angle_var = tk.StringVar(value="90")
        ttk.Entry(row, textvariable=self.angle_var, width=7).pack(side="left", padx=4)
        button = ttk.Button(row, text="turn", command=self._turn_entered)
        button.pack(side="left")
        self.buttons.append(button)
        ttk.Label(turn, text="positive is left", foreground="#666"
                  ).pack(anchor="w", pady=(4, 0))

        drive = ttk.LabelFrame(parent, text="drive", padding=8)
        drive.pack(fill="x", pady=(10, 0))
        self.distance_var = tk.StringVar(value="0.5")
        self.speed_var = tk.StringVar(value="")
        for label, var, hint in (("distance", self.distance_var, "m"),
                                 ("speed", self.speed_var, "m/s, blank for auto")):
            row = ttk.Frame(drive)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=label, width=8).pack(side="left")
            ttk.Entry(row, textvariable=var, width=7).pack(side="left")
            ttk.Label(row, text=hint, foreground="#666").pack(side="left", padx=4)
        button = ttk.Button(drive, text="drive", command=self._drive)
        button.pack(fill="x", pady=(6, 0))
        self.buttons.append(button)

        ask = ttk.LabelFrame(parent, text="ask", padding=8)
        ask.pack(fill="x", pady=(10, 0))
        ttk.Button(ask, text="describe surroundings",
                   command=lambda: self._watch_call("describe_surroundings")
                   ).pack(fill="x", pady=1)
        ttk.Button(ask, text="refresh map", command=self._refresh_map
                   ).pack(fill="x", pady=1)
        self.auto_map = tk.BooleanVar(value=True)
        ttk.Checkbutton(ask, text=f"map every {MAP_AUTO_S:.0f} s",
                        variable=self.auto_map).pack(anchor="w")

        # How much is in frame, and how big the picture is. Two knobs rather than one
        # because they are different questions, and keeping them apart is what makes
        # the first one a zoom: widening the view shows more room in the same picture
        # rather than sending back a bigger one. The size is the one that costs the
        # rover, since a picture costs roughly its own area to draw.
        zoom = ttk.LabelFrame(parent, text="map", padding=8)
        zoom.pack(fill="x", pady=(10, 0))
        # The left button of each pair always steps down its ladder and the right one
        # always steps up, so the two rows behave the same way round.
        for label, down_text, up_text, handler in (
                ("across", "closer", "wider", self._zoom),
                ("size", "smaller", "bigger", self._resize)):
            row = ttk.Frame(zoom)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=label, width=7).pack(side="left")
            ttk.Button(row, text=down_text, width=8,
                       command=lambda h=handler: h(-1)).pack(side="left")
            ttk.Button(row, text=up_text, width=8,
                       command=lambda h=handler: h(1)).pack(side="left", padx=2)
        # Which way is up. Off, the page keeps the heading the rover started with, so
        # the room holds still and the arrow turns -- right for watching where the
        # rover has got to. On, the page turns with the rover, so ahead is always up
        # and the room swings instead, which is what you want when the question is
        # whether it will fit through the gap in front of it.
        self.rover_up = tk.BooleanVar(value=False)
        ttk.Checkbutton(zoom, text="rover up (ahead is up the page)",
                        variable=self.rover_up,
                        command=self._reorient).pack(anchor="w", pady=(4, 0))

        self.zoom_var = tk.StringVar(value="")
        ttk.Label(zoom, textvariable=self.zoom_var, foreground="#444").pack(anchor="w")
        self.cost_var = tk.StringVar(value="")
        ttk.Label(zoom, textvariable=self.cost_var, foreground="#666").pack(anchor="w")
        self._show_zoom()

        # Throwing the map away is a map control, so it lives with them rather than
        # with the moves. In the same disabled-while-driving list as the moves, though,
        # because the navigator refuses it mid-move for the same reason it exists: the
        # route being followed is written in the frame this discards.
        ttk.Separator(zoom, orient="horizontal").pack(fill="x", pady=(8, 6))
        self.clear_button = ttk.Button(zoom, text="clear map", command=self._clear_map)
        self.clear_button.pack(fill="x")
        self.buttons.append(self.clear_button)
        ttk.Label(zoom, text="empties the grid and re-centres the rover",
                  foreground="#666").pack(anchor="w", pady=(2, 0))

        keys = ttk.LabelFrame(parent, text="keys", padding=8)
        keys.pack(fill="x", pady=(10, 0))
        ttk.Label(keys, justify="left", foreground="#444",
                  text=("space  stop\n"
                        "up     drive the distance above\n"
                        "left   turn left by the angle above\n"
                        "right  turn right by the angle above\n"
                        "+ -    map closer or wider\n"
                        "click the map to drive there")).pack(anchor="w")
        for sequence, handler in (
                ("<space>", lambda _e: self._stop()),
                ("<Escape>", lambda _e: self._stop()),
                ("<Up>", lambda _e: self._typing() or self._drive()),
                ("<Left>", lambda _e: self._typing() or self._turn(self._angle())),
                ("<Right>", lambda _e: self._typing() or self._turn(-self._angle())),
                # Plus zooms in, the way it does everywhere else, which means asking
                # for a smaller extent -- a step down the ladder, not up it.
                ("<plus>", lambda _e: self._typing() or self._zoom(-1)),
                ("<KP_Add>", lambda _e: self._typing() or self._zoom(-1)),
                ("<equal>", lambda _e: self._typing() or self._zoom(-1)),
                ("<minus>", lambda _e: self._typing() or self._zoom(1)),
                ("<KP_Subtract>", lambda _e: self._typing() or self._zoom(1))):
            self.root.bind(sequence, handler)

    def _build_status(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.grid(row=0, column=0, sticky="ew")
        # The slack goes to the right of the camera rather than between it and the
        # map, so the two pictures stay side by side however wide the window is.
        top.columnconfigure(2, weight=1)

        panel = ttk.LabelFrame(top, text="nav_status", padding=8)
        panel.grid(row=0, column=0, sticky="nw")
        self.status_labels: dict[str, ttk.Label] = {}
        for index, (key, label, _fmt) in enumerate(STATUS_FIELDS):
            ttk.Label(panel, text=label, foreground="#666").grid(
                row=index, column=0, sticky="w", padx=(0, 8))
            value = ttk.Label(panel, text="-", font=("TkFixedFont", 9))
            value.grid(row=index, column=1, sticky="w")
            self.status_labels[key] = value
        self.pose_var = tk.StringVar(value="-")
        ttk.Label(panel, text="pose", foreground="#666").grid(
            row=len(STATUS_FIELDS), column=0, sticky="w", padx=(0, 8))
        ttk.Label(panel, textvariable=self.pose_var, font=("TkFixedFont", 9)).grid(
            row=len(STATUS_FIELDS), column=1, sticky="w")

        picture = ttk.LabelFrame(top, text="map — click to drive there", padding=8)
        picture.grid(row=0, column=1, sticky="nw", padx=(8, 0))
        # The picture sits in a box of its own fixed size rather than setting the size
        # of everything around it. Whole cells at whole pixels cannot land on exactly
        # the size asked for, and without this the few pixels of difference between
        # one zoom step and the next would shuffle the whole window sideways.
        self.map_box = tk.Frame(picture, width=self.map_size, height=self.map_size)
        self.map_box.pack()
        self.map_box.pack_propagate(False)
        self.map_label = ttk.Label(self.map_box, text="no map yet", anchor="center",
                                   cursor="crosshair")
        self.map_label.place(relx=0.5, rely=0.5, anchor="center")
        self.map_label.bind("<Button-1>", self._map_tap)

        # A key for the colours. The map itself cannot carry one: there is no font on
        # the rover and no image library to draw text with, so the picture names its
        # colours in the caption for the model and here in real widgets for us.
        legend = ttk.Frame(picture)
        legend.pack(anchor="w", pady=(6, 0))
        for column, (colour, label) in enumerate(MAP_LEGEND):
            swatch = tk.Frame(legend, background=colour, width=13, height=13,
                              highlightthickness=1, highlightbackground="#999")
            swatch.grid(row=0, column=2 * column, padx=(0 if column == 0 else 8, 3))
            swatch.grid_propagate(False)
            ttk.Label(legend, text=label, foreground="#444").grid(
                row=0, column=2 * column + 1, sticky="w")

        self.caption_var = tk.StringVar(value="")
        ttk.Label(picture, textvariable=self.caption_var, wraplength=400,
                  foreground="#444", justify="left").pack(anchor="w", pady=(4, 0))

        self._build_camera(top)

        # Asked against achieved, one row per turn, newest at the top. This table is
        # much of the reason the window exists: a turn is the one move whose result
        # you cannot judge by watching the rover, and a column of ratios makes a
        # systematic shortfall obvious in three attempts instead of ten.
        turns = ttk.LabelFrame(parent, text="turns: asked against achieved",
                               padding=(8, 4))
        turns.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        turns.columnconfigure(0, weight=1)
        self.turns = ttk.Treeview(
            turns, columns=("asked", "turned", "ratio", "secs", "reason"),
            show="headings", height=5)
        for column, label, width in (("asked", "asked", 60),
                                     ("turned", "turned", 70),
                                     ("ratio", "ratio", 60),
                                     ("secs", "seconds", 70),
                                     ("reason", "reason", 260)):
            self.turns.heading(column, text=label)
            self.turns.column(column, width=width, anchor="w")
        self.turns.grid(row=0, column=0, sticky="ew")

    def _build_camera(self, parent: ttk.Frame) -> None:
        """The other sensor, and the two board controls that go with it.

        Beside the map rather than under the driving buttons because the two pictures
        answer the same question from opposite ends -- the map says where the rover is
        in the room, the frame says what is in front of the lens -- and the map draws
        the camera's cone, so having both in view is what makes that violet wedge mean
        anything.
        """
        column = ttk.Frame(parent)
        column.grid(row=0, column=2, sticky="nw", padx=(8, 0))

        camera = ttk.LabelFrame(column, text="camera", padding=8)
        camera.pack(fill="x")
        # A box of its own fixed size, for the reason the map has one: a picture that
        # sets the size of everything around it makes the whole window twitch when a
        # frame fails to arrive and the label falls back to a line of text.
        self.frame_box = tk.Frame(camera, width=CAMERA_BOX_PX,
                                  height=CAMERA_BOX_PX * 3 // 4)
        self.frame_box.pack()
        self.frame_box.pack_propagate(False)
        self.frame_label = ttk.Label(self.frame_box, text="no picture yet",
                                     anchor="center", justify="center",
                                     wraplength=CAMERA_BOX_PX - 12)
        self.frame_label.place(relx=0.5, rely=0.5, anchor="center")
        row = ttk.Frame(camera)
        row.pack(fill="x", pady=(6, 0))
        ttk.Button(row, text="take a picture", command=self._take_picture
                   ).pack(side="left")
        self.auto_camera = tk.BooleanVar(value=False)
        ttk.Checkbutton(row, text=f"every {CAMERA_AUTO_S:.0f} s",
                        variable=self.auto_camera).pack(side="left", padx=6)
        self.frame_var = tk.StringVar(value="")
        ttk.Label(camera, textvariable=self.frame_var, foreground="#666"
                  ).pack(anchor="w", pady=(4, 0))

        # Face tracking. Started here it does the same thing the model's tool does:
        # the loop takes the camera, sweeps for a face and follows it.
        tracking = ttk.LabelFrame(column, text="face tracking", padding=8)
        tracking.pack(fill="x", pady=(10, 0))
        row = ttk.Frame(tracking)
        row.pack(fill="x")
        for label, name in (("start", "start_tracking"), ("stop", "stop_tracking")):
            ttk.Button(row, text=label, width=8,
                       command=lambda n=name: self._track(n)).pack(side="left", padx=2)
        self.tracking_var = tk.StringVar(value="-")
        ttk.Label(tracking, textvariable=self.tracking_var, font=("TkFixedFont", 9)
                  ).pack(anchor="w", pady=(4, 0))
        ttk.Label(tracking, text="driving stops it: they cannot share the core",
                  foreground="#666").pack(anchor="w")

        lights = ttk.LabelFrame(column, text="headlights", padding=8)
        lights.pack(fill="x", pady=(10, 0))
        row = ttk.Frame(lights)
        row.pack(fill="x")
        for label, level in (("on", LIGHT_MAX), ("off", 0)):
            ttk.Button(row, text=label, width=8,
                       command=lambda v=level: self._set_lights(v)
                       ).pack(side="left", padx=2)
        self.lights_var = tk.StringVar(value="-")
        ttk.Label(lights, textvariable=self.lights_var, font=("TkFixedFont", 9)
                  ).pack(anchor="w", pady=(4, 0))

    def _build_log(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="calls and replies", padding=(4, 4))
        frame.grid(row=2, column=0, sticky="nsew", pady=(8, 0))
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)
        self.log = tk.Text(frame, height=12, wrap="word", font=("TkFixedFont", 9),
                           state="disabled", background="#fbfbfb")
        self.log.grid(row=0, column=0, sticky="nsew")
        bar = ttk.Scrollbar(frame, orient="vertical", command=self.log.yview)
        bar.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=bar.set)
        self.log.tag_configure("sent", foreground="#0a4f9c")
        self.log.tag_configure("good", foreground="#136b13")
        self.log.tag_configure("bad", foreground="#a01010")
        self.log.tag_configure("quiet", foreground="#777")

    # --- connecting -----------------------------------------------------------
    def _connect(self) -> None:
        for channel in self.channels:
            channel.close()
        self.channels = []
        # All of them, including the two the reconnect path used to leave pointing at
        # a closed socket: a submit on a closed channel is queued to a thread that has
        # already returned, so the map simply never came back and the "one at a time"
        # flag stayed set for good.
        self.moves = self.halt = self.watch = self.picture = self.camera = None
        self.frame_outstanding = False
        self.map_outstanding = False
        self.tools = []
        self.can_drive = False
        self.busy_since = None
        self._enable_buttons(False)
        self.link_var.set("looking for the rover...")

        wanted = self.address_var.get().strip()

        def find() -> None:
            # On a thread, because a failed name lookup costs seconds -- 7.3 of them
            # on this LAN, measured -- and a window that freezes while it looks is a
            # window nobody should trust with a stop button.
            if wanted:
                address = (wanted if ":" in wanted
                           else f"{wanted}:{rover_tools.DEFAULT_PORT}")
                probe = rover_tools.RoverClient(address)
                found = address if probe.probe() else None
                probe.close()
            else:
                client = rover_tools.discover()
                found = client.describe() if client else None
                if client:
                    client.close()
            self.replies.put(Reply("__found__", {},
                                   {"ok": found is not None, "address": found}, 0.0))

        threading.Thread(target=find, daemon=True, name="rover-find").start()

    def _connected(self, address: str) -> None:
        self.address_var.set(address)
        self.moves = Channel("move", address, self.replies, timeout=MOVE_TIMEOUT_S)
        self.halt = Channel("stop", address, self.replies)
        self.watch = Channel("watch", address, self.replies)
        self.picture = Channel("map", address, self.replies)
        self.camera = Channel("camera", address, self.replies)
        self.channels = [self.moves, self.halt, self.watch, self.picture, self.camera]
        self.link_var.set(f"{address}: asking what it can do")
        self.watch.submit("list_tools")
        # The board cannot be read back, so the daemon only knows the level it last
        # set. Ask once on connect and the panel starts out true rather than blank.
        self.watch.submit("get_lights")

    # --- issuing calls --------------------------------------------------------
    def _watch_call(self, name: str, arguments: dict[str, Any] | None = None) -> None:
        if self.watch is None:
            self._say(f"not connected, so {name} was not sent\n", "bad")
            return
        self._log_sent(name, arguments or {})
        self.watch.submit(name, arguments)

    def _move(self, name: str, arguments: dict[str, Any]) -> None:
        """A bounded move, one at a time.

        Refused here rather than sent and refused by the daemon. The daemon's answer
        would be `busy`, which is correct and tells you nothing, and it would land in
        the log between the move and its result, where it reads like the move having
        failed.
        """
        if self.moves is None or not self.can_drive:
            self._say(f"no driving tools on this rover, so {name} was not sent\n",
                      "bad")
            return
        if self.busy_since is not None:
            self._say(f"{self.busy_name} is still running; stop it or wait\n", "quiet")
            return
        self.busy_since = time.monotonic()
        self.busy_name = name
        self._enable_buttons(False)
        self._log_sent(name, arguments)
        self.moves.submit(name, arguments)

    def _turn(self, angle: float) -> None:
        self._move("turn_in_place", {"angle_deg": float(angle)})

    def _turn_entered(self) -> None:
        self._turn(self._angle())

    def _drive(self) -> None:
        arguments: dict[str, Any] = {"distance_m": self._float(self.distance_var, 0.5)}
        if self.speed_var.get().strip():
            arguments["speed_ms"] = self._float(self.speed_var, 0.2)
        self._move("drive", arguments)

    def _map_tap(self, event: tk.Event) -> str:
        """A click on the picture is a place relative to the rover, not a pixel."""
        if self.map_image is None or self.map_view is None:
            return "break"
        width, height = self.map_image.width(), self.map_image.height()
        if not (0 <= event.x < width and 0 <= event.y < height):
            return "break"
        if "drive_to" not in self.tools:
            self._say("this rover has no drive_to tool, so the tap was not sent\n",
                      "quiet")
            return "break"
        view = self.map_view
        pose = view.get("pose") or {}
        heading = math.radians(float(pose.get("heading_deg", math.degrees(
            self._heading_rad))))
        try:
            import mapimg
        except ImportError:
            self._say("cannot convert a tap without mapimg\n", "bad")
            return "break"
        ahead, left = mapimg.tap_to_relative(
            event.x, event.y, view["half_extent_m"], view["scale"],
            rover_up=bool(view.get("rover_up")), heading_rad=heading)
        arguments: dict[str, Any] = {
            "ahead_m": round(ahead, 2), "left_m": round(left, 2)}
        if self.speed_var.get().strip():
            arguments["speed_ms"] = self._float(self.speed_var, 0.2)
        self._move("drive_to", arguments)
        return "break"

    def _take_picture(self) -> None:
        """On its own connection, because it is the slowest call here: a camera that
        has to be opened takes the rover up to four seconds to deliver a first
        buffer, and while it is doing that nothing else on that socket is answered."""
        if self.camera is None:
            self._say("not connected, so no picture was asked for\n", "bad")
            return
        if self.frame_outstanding:
            return
        self.frame_outstanding = True
        self.frame_at = time.monotonic()
        self.frame_var.set("taking one...")
        self.camera.submit("camera_jpeg")

    def _track(self, name: str) -> None:
        """Start or stop face tracking, and ask straight away what came of it rather
        than believing the button."""
        self._watch_call(name)
        self.track_at = 0.0

    def _set_lights(self, level: int) -> None:
        self._watch_call("set_lights", {"level": level})

    def _clear_map(self) -> None:
        """Two presses, and no dialog between them.

        A modal confirmation box stops the tk event loop, and that loop is where the
        stop button, the key bindings and the status poll live -- so the window would
        be unable to stop a rover for as long as somebody left the box sitting there.
        Arming the button costs one extra press and takes nothing away. It disarms
        itself after CLEAR_ARM_S, so a press forgotten about does not lie in wait.
        """
        now = time.monotonic()
        if now > self.clear_armed_until:
            self.clear_armed_until = now + CLEAR_ARM_S
            self.clear_button.configure(text="clear map -- press again")
            return
        self._disarm_clear()
        if self.picture is None:
            self._say("not connected, so the map was not cleared\n", "bad")
            return
        # On the map's connection, so that a picture already being drawn comes back
        # before the clear rather than after it. The other way round shows an empty
        # map and then replaces it with the old one, which reads as the clear having
        # failed.
        self._log_sent("clear_map", {})
        self.picture.submit("clear_map")

    def _disarm_clear(self) -> None:
        self.clear_armed_until = 0.0
        self.clear_button.configure(text="clear map")

    def _stop(self) -> None:
        """Always allowed, and on the connection that carries nothing else."""
        if self.halt is None:
            self._say("not connected, so there was nothing to stop\n", "quiet")
            return
        self._log_sent("stop_driving", {})
        self.halt.submit("stop_driving")

    def _refresh_map(self) -> None:
        """On its own connection, because the map is the slowest thing here.

        It shared the status connection at first, which was wrong once the cost was
        measured: a map at the default settings takes a couple of seconds on the Pi,
        and `RoverClient` serialises, so every refresh held up a status poll that is
        meant to arrive three times a second. The numbers went stale exactly while
        the picture was being drawn.
        """
        if self.picture is None:
            return
        if self.map_outstanding:
            # One at a time, but do not lose the request: the map takes seconds, and a
            # zoom pressed while one is in flight would otherwise be dropped on the
            # floor -- silently, and for good if auto-refresh is off. Remember that the
            # settings moved and ask again as soon as this one lands.
            self.map_wanted = True
            return
        self.map_wanted = False
        self.map_outstanding = True
        self.map_at = time.monotonic()
        # Say that the picture on screen is the old one. The buttons change the labels
        # at once and the rover takes a second or more to answer, so without this the
        # settings shown and the picture shown disagree for as long as the draw takes.
        self.cost_var.set("drawing...")
        # An extent and a size, never a magnification: the rover derives pixels per
        # cell from the two, which is what keeps the picture the same size when the
        # view widens. Asking for a magnification instead resized the picture on every
        # zoom, which is not zooming.
        self.picture.submit("map_png", {"half_extent_m": self.half_extent,
                                        "pixels": self.map_size,
                                        "rover_up": bool(self.rover_up.get())})

    def _step(self, ladder: tuple, value: float) -> int:
        """Where `value` sits in a ladder, tolerating its not being on a rung."""
        try:
            return ladder.index(value)
        except ValueError:
            return min(range(len(ladder)), key=lambda i: abs(ladder[i] - value))

    def _zoom(self, step: int) -> None:
        """Wider or closer, through a fixed ladder rather than a multiplier, so the
        same few extents come back and one picture can be compared with an earlier
        one. The picture stays the size it was."""
        index = self._step(MAP_EXTENTS_M, self.half_extent)
        self.half_extent = MAP_EXTENTS_M[
            max(0, min(len(MAP_EXTENTS_M) - 1, index + step))]
        self._show_zoom()
        self._refresh_map()

    def _resize(self, step: int) -> None:
        """How big a picture to ask for, which is a different question from what is in
        it. This is the one that costs the rover: the area goes as its square, and
        drawing it is interpreted Python on a Pi 1."""
        index = self._step(MAP_SIZES_PX, self.map_size)
        self.map_size = MAP_SIZES_PX[max(0, min(len(MAP_SIZES_PX) - 1, index + step))]
        self.map_box.configure(width=self.map_size, height=self.map_size)
        self._show_zoom()
        self._refresh_map()

    def _reorient(self) -> None:
        self._show_zoom()
        self._refresh_map()

    def _show_zoom(self) -> None:
        facing = "rover up" if self.rover_up.get() else "start heading up"
        self.zoom_var.set(f"{2 * self.half_extent:.0f} m across, "
                          f"{self.map_size} px picture, {facing}")

    # --- the pump -------------------------------------------------------------
    def _tick(self) -> None:
        while True:
            try:
                reply = self.replies.get_nowait()
            except queue.Empty:
                break
            self._handle(reply)

        now = time.monotonic()
        self.busy_var.set("" if self.busy_since is None
                          else f"{self.busy_name}  {now - self.busy_since:.1f} s")

        if self.watch is not None and not self.poll_outstanding and now - self.poll_at > POLL_S:
            self.poll_outstanding = True
            self.poll_at = now
            self.watch.submit("nav_status")
        # Auto-refresh leaves the rover as long to breathe as the last map took to
        # draw. Asking every two seconds for a picture that takes two and a half is
        # how you keep a single-core Pi permanently drawing maps while it is also
        # running the SLAM it is drawing them from.
        if (self.picture is not None and self.auto_map.get()
                and not self.map_outstanding
                and now - self.map_at > max(MAP_AUTO_S, self.map_cost)):
            self._refresh_map()
        # Paced off what the last one cost, like the map: a cold camera takes seconds
        # to produce a frame, and asking again while it is still opening would keep a
        # single-core Pi taking pictures for the whole time it is meant to be driving.
        if (self.camera is not None and self.auto_camera.get()
                and not self.frame_outstanding
                and now - self.frame_at > max(CAMERA_AUTO_S, self.frame_cost)):
            self._take_picture()
        # Asked, not remembered: the daemon parks tracking by itself when the wheels
        # turn, so the only honest source for this panel is the daemon.
        if (self.watch is not None and not self.track_outstanding
                and now - self.track_at > TRACK_POLL_S):
            self.track_outstanding = True
            self.track_at = now
            self.watch.submit("tracking_status")
        if self.clear_armed_until and now > self.clear_armed_until:
            self._disarm_clear()

        self.root.after(TICK_MS, self._tick)

    def _handle(self, reply: Reply) -> None:
        name, body = reply.name, reply.body

        if name == "__found__":
            if body.get("ok"):
                self._connected(body["address"])
            else:
                self.link_var.set("no daemon answered")
                self._say("no rover daemon answered. Is it running, and is the "
                          "address right?\n", "bad")
            return
        if name == "nav_status":
            self.poll_outstanding = False
            self._show_status(body)
            return
        if name == "map_png":
            self.map_outstanding = False
            self._show_map(body)
            if self.map_wanted:
                self._refresh_map()
            return
        if name == "list_tools":
            self._show_tools(body)
            return
        if name == "camera_jpeg":
            self.frame_outstanding = False
            self.frame_cost = reply.seconds
            self._show_picture(body)
            return
        if name == "tracking_status":
            # Polled by the window rather than asked for by a person, so it updates
            # the panel and stays out of the transcript.
            self.track_outstanding = False
            self._show_tracking(body)
            return
        if name in ("get_lights", "set_lights"):
            if body.get("ok"):
                self.light_level = body.get("level")
                self._show_lights()
            if name == "get_lights":
                return          # the window asked for this one, on connect

        # What is left is something a person asked for, so it is logged.
        moved = name in ("drive", "turn_in_place", "drive_to")
        if moved:
            self.busy_since = None
            self.busy_name = ""
            self._enable_buttons(self.can_drive)
        self._log_reply(reply)
        if name == "turn_in_place":
            self._tally_turn(reply)
        if name in ("start_tracking", "stop_tracking") and body.get("ok"):
            self._show_tracking(body)
        if moved or name == "clear_map":
            self._refresh_map()

    # --- showing it -----------------------------------------------------------
    def _show_tools(self, body: dict[str, Any]) -> None:
        if not body.get("ok"):
            self.link_var.set("connected, but it would not say what it can do")
            self._say(f"list_tools failed: {body.get('error')}\n", "bad")
            return
        self.tools = [t.get("function", {}).get("name", "?") for t in body["tools"]]
        self.can_drive = {"drive", "turn_in_place"} <= set(self.tools)
        address = self.moves.client.describe() if self.moves else "?"
        self.link_var.set(f"{address}: {len(self.tools)} tools"
                          + ("" if self.can_drive else ", none of them driving"))
        self._say("tools: " + ", ".join(self.tools) + "\n", "quiet")
        if not self.can_drive:
            # Said plainly, because the failure here is a window of live buttons on
            # a rover with no navigator behind them, and the cause is nearly always
            # a daemon started without --lidar.
            self._say("this daemon is offering no driving tools. It was probably "
                      "started without --lidar; or its lidar has not enumerated yet, "
                      "in which case the tools appear when the sensor does and "
                      "connecting again will pick them up.\n", "bad")
        self._enable_buttons(self.can_drive)

    def _show_status(self, body: dict[str, Any]) -> None:
        if not body.get("ok"):
            for label in self.status_labels.values():
                label.configure(text="-", foreground="black")
            self.pose_var.set(str(body.get("error", "no status")))
            return
        for key, _label, fmt in STATUS_FIELDS:
            value = body.get(key)
            alarm = ((key in ALARM_WHEN_FALSE and not value)
                     or (key in ALARM_WHEN_TRUE and value))
            self.status_labels[key].configure(
                text=fmt(value), foreground="#a01010" if alarm else "black")
        pose = body.get("pose") or {}
        self._heading_rad = math.radians(float(pose.get("heading_deg", 0.0)))
        self.pose_var.set("x {:+.2f}  y {:+.2f}  {:+.1f} deg".format(
            pose.get("x_m", 0.0), pose.get("y_m", 0.0), pose.get("heading_deg", 0.0)))

    def _show_map(self, body: dict[str, Any]) -> None:
        if not body.get("ok"):
            self.map_label.configure(text=str(body.get("error", "no map")), image="")
            return
        try:
            # Tk 8.6 reads PNG natively, truecolour included, and takes base64
            # directly, so the map the rover already renders for the model goes on
            # screen with nothing decoding it at this end.
            image = tk.PhotoImage(data=body["png_base64"])
        except (tk.TclError, KeyError) as error:
            self.map_label.configure(text=f"cannot show the map: {error}", image="")
            return
        self.map_image = image          # Tk keeps no reference of its own
        self.map_label.configure(image=image, text="")
        self.caption_var.set(body.get("caption", ""))
        self.map_view = {
            "half_extent_m": float(body.get("half_extent_m", self.half_extent)),
            "scale": int(body.get("scale") or 1),
            "rover_up": bool(body.get("rover_up")),
            "pose": body.get("pose") or {"heading_deg": math.degrees(self._heading_rad)},
        }

        # What the rover actually drew, which is not always the size asked for: a cell
        # has to be a whole number of pixels, so most sizes are only reachable to
        # within a few percent, and a very wide view cannot reach a large one at all.
        # Worth saying, because otherwise "bigger" appearing to do nothing looks like a
        # broken button rather than a picture already as big as that view can be drawn.
        took = body.get("render_s")
        self.map_cost = float(took or 0.0)
        cost = f"{image.width()} px at {body.get('scale', '?')} px/cell"
        if body.get("bytes"):
            cost += f", {body['bytes'] / 1000:.0f} kB"
        if took is not None:
            cost += f", {took:.1f} s to draw"
        if abs(image.width() - self.map_size) > self.map_size * 0.1:
            cost += f" -- {self.map_size} px was not reachable here"
        self.cost_var.set(cost)

    def _show_picture(self, body: dict[str, Any]) -> None:
        if not body.get("ok"):
            self.frame_label.configure(text=str(body.get("error", "no picture")),
                                       image="")
            self.frame_var.set("")
            return
        jpeg = base64.b64decode(body.get("jpeg_base64", ""))
        photo, why = _photo(jpeg, CAMERA_BOX_PX)
        if photo is None:
            # The frame arrived and only the showing of it failed, so it is put
            # somewhere it can be looked at rather than dropped on the floor.
            self.frame_label.configure(
                text=f"{why}.\nThe frame itself is at {_save_frame(jpeg)}", image="")
        else:
            self.frame_image = photo        # Tk keeps no reference of its own
            self.frame_label.configure(image=photo, text="")
        where = f"pan {_f(body.get('pan'))}, tilt {_f(body.get('tilt'))}"
        size = f"{_f(body.get('width'))}x{_f(body.get('height'))}"
        # Which of the two paths it came off. They mean different things: while
        # tracking runs the loop owns the camera and this is its newest frame, which
        # is also the one the gimbal is actually pointed at.
        source = "tracking's own frame" if body.get("live") else "fresh"
        self.frame_var.set(f"{size}, {body.get('bytes', 0) / 1000:.0f} kB, {where}, "
                           f"{source}, {self.frame_cost:.1f} s")

    def _show_tracking(self, body: dict[str, Any]) -> None:
        if not body.get("ok"):
            self.tracking_var.set(str(body.get("error", "-")))
            return
        if not body.get("tracking"):
            self.tracking_var.set("off")
            return
        # "Running" and "following somebody" are different states and the difference
        # is the whole question: a loop that is running and has locked onto nobody is
        # sweeping, which looks identical from here and quite different on the rover.
        who = ("following someone" if body.get("following_someone")
               else "sweeping, nobody yet")
        faces = body.get("faces_in_view")
        self.tracking_var.set(
            f"on, {who}" + ("" if faces is None else f", {faces} in view"))

    def _show_lights(self) -> None:
        level = self.light_level
        if level is None:
            self.lights_var.set("-")
            return
        self.lights_var.set(f"{'on' if level else 'off'} ({level})")

    def _tally_turn(self, reply: Reply) -> None:
        asked = float(reply.arguments.get("angle_deg", 0.0))
        turned = reply.body.get("turned_deg")
        ratio = (turned / asked) if (turned is not None and asked) else None
        self.turns.insert("", 0, values=(
            f"{asked:+.0f}",
            "-" if turned is None else f"{turned:+.1f}",
            "-" if ratio is None else f"{ratio:.2f}",
            f"{reply.seconds:.1f}",
            str(reply.body.get("reason") or reply.body.get("error") or "-")))
        rows = self.turns.get_children()
        if len(rows) > TURN_ROWS:
            self.turns.delete(rows[-1])

    def _log_sent(self, name: str, arguments: dict[str, Any]) -> None:
        shown = ", ".join(f"{k}={v}" for k, v in arguments.items())
        self._say(f"{time.strftime('%H:%M:%S')}  -> {name}({shown})\n", "sent")

    def _log_reply(self, reply: Reply) -> None:
        body = reply.body
        ok = bool(body.get("ok"))
        head = f"{'':10}   <- {reply.seconds:5.2f}s  "
        if not ok and "error" in body:
            self._say(head + f"failed: {body['error']}\n", "bad")
            return
        summary = str(body.get("reason", "ok" if ok else "failed"))
        for key, unit in (("travelled_m", " m"), ("turned_deg", " deg"),
                          ("remaining_m", " m to go"),
                          ("clear_ahead_m", " m clear ahead")):
            if body.get(key) is not None:
                summary += f", {body[key]}{unit}"
        self._say(head + summary + "\n", "good" if ok else "bad")
        for key in ("detail", "note", "surroundings", "text"):
            if body.get(key):
                self._say(f"{'':17}{body[key]}\n", "quiet")

    def _say(self, text: str, tag: str = "") -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text, tag)
        # Trimmed from the top rather than left to grow: this window is meant to be
        # left open for an afternoon of test moves.
        excess = int(self.log.index("end-1c").split(".")[0]) - LOG_LINES
        if excess > 0:
            self.log.delete("1.0", f"{excess}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    # --- odds and ends --------------------------------------------------------
    def _enable_buttons(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        for widget in self.buttons:
            widget.configure(state=state)

    def _typing(self) -> bool:
        """True if the keyboard belongs to an entry box rather than to the rover.

        Without this, typing a distance of 0.5 sends an arrow key to the motors on
        the way to the decimal point.
        """
        return isinstance(self.root.focus_get(), (tk.Entry, ttk.Entry, ttk.Combobox))

    def _angle(self) -> float:
        return self._float(self.angle_var, 90.0)

    def _float(self, var: tk.StringVar, fallback: float) -> float:
        try:
            return float(var.get().strip())
        except ValueError:
            self._say(f"{var.get()!r} is not a number, using {fallback}\n", "quiet")
            var.set(str(fallback))
            return fallback

    def _quit(self) -> None:
        # A window that can start a move has to be able to end one, including by
        # being closed. Sent on the connection that carries nothing else, and sent
        # inline rather than submitted, because the channel threads die with the
        # process and a queued stop would go nowhere.
        if self.halt is not None:
            self.halt.client.call("stop_driving", {})
        for channel in self.channels:
            channel.close()
        self.root.destroy()


def main(argv=None) -> int | str:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--rover", default=None, metavar="HOST[:PORT]",
                        help="the daemon; omit to look for it (see rover_tools.py)")
    parser.add_argument("--half-extent", type=float, default=3.0, metavar="M",
                        help="metres each way shown in the map (default: %(default)s)")
    parser.add_argument("--map-size", type=int, default=480, metavar="PX",
                        help="how big a map to ask for (default: %(default)s)")
    args = parser.parse_args(argv)

    try:
        root = tk.Tk()
    except tk.TclError as error:
        return f"cannot open a window: {error}"
    Console(root, args.rover, args.half_extent, args.map_size)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
