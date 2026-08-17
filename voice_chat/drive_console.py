#!/usr/bin/env python3
"""Drive the rover by hand through the daemon, and watch what it says back.

[realtime.py](realtime.py) and [talk.py](talk.py) move the rover by talking to a
model that decides which tool to call. That is the point of them, and it is also
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
    python voice_chat/drive_console.py --half-extent 4.0   # a wider map

Only the standard library, plus [rover_tools.py](rover_tools.py) for the wire
protocol. tkinter ships with Python, which matters because this is a diagnostic,
and a diagnostic that needs something installed first is one you will not have when
you need it.

**Three connections, deliberately.** A `drive` call does not answer until the move
has finished, and `RoverClient` serialises calls on one socket, so anything sharing
that socket would queue behind the move. Stop must never queue, and neither should
the status poll that tells you what the move is doing while it does it -- so moves,
stop, and watching each get their own. The daemon is a threading server and holds
no lock across a move, so the other two are answered while the first is driving.

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
import queue
import sys
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

# Preset turns, as magnitudes. Laid out as two columns with the left turns in the
# left one and the right turns in the right one, increasing downwards, so a button's
# place on screen matches the way the rover is about to go.
#
# The sizes cover the range that matters rather than making a keypad: the small ones
# are where the coast after the power comes off is a large fraction of the whole
# turn, and 90 is what a model asks for when it wants to face another way.
TURN_PRESETS_DEG = (15, 45, 90)


def _legend():
    """Swatch colours and labels for the map key, taken from the renderer itself.

    The rover's own `mapimg` is the authority on what colour means what, and a key
    that has drifted from the picture is worse than no key at all -- so the palette
    is imported rather than copied, and if it cannot be found the key is simply
    omitted. It is a label on a picture; it is not worth failing to start over.
    """
    import os

    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "lidar_slam"))
    try:
        import mapimg
    except ImportError:
        return ()
    return tuple(("#%02x%02x%02x" % colour, label) for colour, label in (
        (mapimg.C_ROVER, "rover"),
        (mapimg.C_TRACK, "driven"),
        (mapimg.C_OCCUPIED, "solid"),
        (mapimg.C_FREE, "empty"),
        (mapimg.C_DIM, "unsure"),
        (mapimg.C_UNKNOWN, "unseen")))


MAP_LEGEND = _legend()


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

    def __init__(self, label: str, address: str, replies: queue.Queue) -> None:
        self.label = label
        self.client = rover_tools.RoverClient(address)
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
    """The window, and the three channels behind it."""

    def __init__(self, root: tk.Tk, address: str | None, half_extent: float,
                 scale: int) -> None:
        self.root = root
        self.half_extent = half_extent
        self.scale = scale
        self.replies: queue.Queue = queue.Queue()

        self.moves: Channel | None = None     # blocking, one bounded move at a time
        self.halt: Channel | None = None      # stop, and nothing else, ever
        self.watch: Channel | None = None     # status, map, tool list
        self.channels: list[Channel] = []

        self.tools: list[str] = []
        self.can_drive = False
        self.busy_since: float | None = None
        self.busy_name = ""
        self.poll_outstanding = False
        self.poll_at = 0.0
        self.map_outstanding = False
        self.map_at = 0.0
        self.map_image: tk.PhotoImage | None = None

        root.title("rover drive console")
        root.minsize(960, 640)
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
        self.curve_var = tk.StringVar(value="0")
        self.speed_var = tk.StringVar(value="")
        for label, var, hint in (("distance", self.distance_var, "m"),
                                 ("turn over the move", self.curve_var, "deg"),
                                 ("speed", self.speed_var, "m/s, blank for auto")):
            row = ttk.Frame(drive)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=label, width=17).pack(side="left")
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

        keys = ttk.LabelFrame(parent, text="keys", padding=8)
        keys.pack(fill="x", pady=(10, 0))
        ttk.Label(keys, justify="left", foreground="#444",
                  text=("space  stop\n"
                        "up     drive the distance above\n"
                        "left   turn left by the angle above\n"
                        "right  turn right by the angle above")).pack(anchor="w")
        for sequence, handler in (
                ("<space>", lambda _e: self._stop()),
                ("<Escape>", lambda _e: self._stop()),
                ("<Up>", lambda _e: self._typing() or self._drive()),
                ("<Left>", lambda _e: self._typing() or self._turn(self._angle())),
                ("<Right>", lambda _e: self._typing() or self._turn(-self._angle()))):
            self.root.bind(sequence, handler)

    def _build_status(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(1, weight=1)

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

        picture = ttk.LabelFrame(top, text="map", padding=8)
        picture.grid(row=0, column=1, sticky="nw", padx=(8, 0))
        self.map_label = ttk.Label(picture, text="no map yet", anchor="center")
        self.map_label.pack()

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
        self.moves = self.halt = self.watch = None
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
        self.moves = Channel("move", address, self.replies)
        self.halt = Channel("stop", address, self.replies)
        self.watch = Channel("watch", address, self.replies)
        self.channels = [self.moves, self.halt, self.watch]
        self.link_var.set(f"{address}: asking what it can do")
        self.watch.submit("list_tools")

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
        curve = self._float(self.curve_var, 0.0)
        if abs(curve) > 0.01:
            arguments["turn_deg"] = curve
        if self.speed_var.get().strip():
            arguments["speed_ms"] = self._float(self.speed_var, 0.2)
        self._move("drive", arguments)

    def _stop(self) -> None:
        """Always allowed, and on the connection that carries nothing else."""
        if self.halt is None:
            self._say("not connected, so there was nothing to stop\n", "quiet")
            return
        self._log_sent("stop_driving", {})
        self.halt.submit("stop_driving")

    def _refresh_map(self) -> None:
        if self.watch is None or self.map_outstanding:
            return
        self.map_outstanding = True
        self.map_at = time.monotonic()
        self.watch.submit("map_png", {"half_extent_m": self.half_extent,
                                      "scale": self.scale})

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
        if (self.watch is not None and self.auto_map.get()
                and not self.map_outstanding and now - self.map_at > MAP_AUTO_S):
            self._refresh_map()

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
            return
        if name == "list_tools":
            self._show_tools(body)
            return

        # What is left is something a person asked for, so it is logged.
        moved = name in ("drive", "turn_in_place")
        if moved:
            self.busy_since = None
            self.busy_name = ""
            self._enable_buttons(self.can_drive)
        self._log_reply(reply)
        if name == "turn_in_place":
            self._tally_turn(reply)
        if moved:
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
    parser.add_argument("--half-extent", type=float, default=2.5, metavar="M",
                        help="metres each way shown in the map (default: %(default)s)")
    parser.add_argument("--scale", type=int, default=3, metavar="PX",
                        help="screen pixels per 5 cm cell (default: %(default)s)")
    args = parser.parse_args(argv)

    try:
        root = tk.Tk()
    except tk.TclError as error:
        return f"cannot open a window: {error}"
    Console(root, args.rover, args.half_extent, args.scale)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
