#!/usr/bin/env python3
"""What the drive console knows: the wire, the pacing, and the English.

[drive_web.py](drive_web.py) is a browser page, and almost nothing that matters
about it is HTML. Which connection a call goes down, how often it is safe to ask
the Pi for a map, what "replanning (#2) -- the corridor closed" is made of, and
which of those sentences is worth a line in the transcript are all questions about
a rover, and they belong on this side of the wire rather than in a page.

The temptation is to write the sentence logic again in JavaScript because it is
only twenty lines. That is how a console starts disagreeing with itself about what
the rover said, and the disagreement is invisible: both versions look plausible and
only one is right. So the browser is sent its English from here, over the wire,
already assembled -- the same rule this repository applies to tool schemas, which
the clients fetch from the daemon rather than keep a copy of.

Nothing here imports a toolkit, and nothing here touches a socket except through
[rover_tools.py](rover_tools.py). It is covered in [selftest.py](selftest.py) for
the same reason it exists: a GUI is a miserable place to debug a sentence, and a
browser is worse.
"""

from __future__ import annotations

import math
import os
import queue
import sys
import threading
import time
from typing import Any

import rover_tools

# --- how often to ask the rover for things ----------------------------------
#
# Every number here is a load on one 700 MHz ARMv6 core that is also running SLAM,
# so none of them is a taste in refresh rates. They are what the Pi can answer
# without the driving loop noticing.

POLL_S = 0.3               # how often to ask for nav_status while connected
MAP_AUTO_S = 2.0           # how often to refresh the map when auto is on
LOG_LINES = 500            # trimmed, so an afternoon of testing does not grow forever
TURN_ROWS = 40
# drive_to can take minutes of segments and turns -- the navigator allows a route
# 15 m and 200 s -- while the default client timeout is 12 s, which is right for a
# single hop and wrong for a route. Comfortably past the navigator's own budget, so
# that a route which ran out of time is reported by the rover rather than abandoned
# by the console, which cannot tell the two apart.
MOVE_TIMEOUT_S = 240.0

# Preset turns, as magnitudes, laid out with the left turns in one column and the
# right turns in the other so a button's place on screen matches the way the rover
# is about to go.
#
# The sizes cover the range that matters rather than making a keypad: the small ones
# are where the coast after the power comes off is a large fraction of the whole
# turn, and 90 is what a model asks for when it wants to face another way.
TURN_PRESETS_DEG = (15, 45, 90)

# How far each way the map covers, as a ladder rather than a zoom multiplier, so the
# same handful of extents come back and one picture can be compared with an earlier
# one. The steps go up by a third and then by a half, alternately, which is a
# noticeable change in what is in frame without being a jump.
#
# The top two rungs are for finding your way around a whole floor rather than a
# room, and they cost something the ones below them do not. A cell must be a whole
# number of pixels, so past about 6 m the picture can no longer be held at the size
# that was asked for and comes back smaller and coarser -- two pixels a cell at 16 m
# across, and the note under the map says so. Read them for the shape of the place
# and the way back; read the close rungs for anything the rover is about to drive
# into. The pose still drifts over a long run, which is the real reason not to plan a
# route home off the widest picture, and the caption says that too.
MAP_EXTENTS_M = (0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0)

# How big a picture to ask for. This is a separate control from the zoom, and the
# separation is the point: widening the view must show more room in the same picture,
# not send back a bigger picture. The rover works out pixels per cell from the two.
MAP_SIZES_PX = (320, 480, 640, 800)

# Auto-refresh for the camera, and it is off until asked for. Face tracking and SLAM
# already cannot share this one core, and a picture every few seconds opens the
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
# confirmation dialog: a blocking confirm halts the same script that is meant to be
# receiving status. A map thrown away by accident costs a minute of driving; a stop
# button behind a dialog costs whatever the rover hits.
CLEAR_ARM_S = 4.0

LIGHT_MAX = 255            # what the daemon calls full brightness

# How often to ask about the battery. Far slower than anything else here, because
# it is the slowest-moving number on the rover and the daemon caches it anyway: a
# poll inside its cache window is answered without the board being read at all.
BATTERY_POLL_S = 10.0
# Past this the reading has stopped being refreshed. The daemon serves one for
# five seconds, so anything much older than that means the board went quiet.
BATTERY_STALE_S = 20.0
# What each of the daemon's five words means on screen. "absent" is not a flat
# battery and must not read like one: it is the board running from USB with the
# pack out or the main switch off, which is something to go and look at rather
# than something to charge.
BATTERY_COLOURS = {"full": "#136b13", "ok": "black", "low": "#a05a10",
                   "critical": "#a01010", "absent": "#a01010"}
BATTERY_NOTES = {"full": "off the charger", "ok": "plenty left",
                 "low": "getting low", "critical": "nearly flat -- charge it",
                 "absent": "no pack fitted, or the main switch is off"}

# How often to ask which access point the rover is on. The signal strength in that
# answer is live -- the daemon reads it out of /proc for nothing -- while the list
# of networks behind it is cached for twenty seconds, so this is paced to the part
# that moves. It never asks for a scan: that costs the rover several seconds off
# channel and interrupts the link, so looking around is a button.
WIFI_POLL_S = 5.0
# And how long to wait for that button, which is its own connection's worth of
# patience. Measured on the Pi itself, over loopback where nothing can be blamed
# on the radio being off channel: 15.2 s for a scan through the daemon, of which
# nmcli's rescan is 9.8 s. That is past the 12 s the rest of these calls get, so
# every scan a console asked for timed out and the panel reported a rover that was
# in fact answering. It is also why the scan is not on the status connection: the
# daemon holds one lock across it, so the five-second polls queued behind a scan
# would take the whole status column down with it.
WIFI_SCAN_TIMEOUT_S = 45.0
# What the driver's dBm means for the link, and where the panel starts to say so.
# Not a signal ladder out of a phone: these are this rover's own numbers, which
# sits at -35 to -44 dBm in the lab, and the wifi keeper on the Pi calls the link
# failing at -78.
WIFI_GOOD_DBM = -60
WIFI_POOR_DBM = -72
WIFI_COLOURS = {"good": "#136b13", "fair": "#a05a10", "poor": "#a01010"}
# How long to leave the rover alone after asking it to switch networks before
# reconnecting. The daemon answers the request immediately and *then* takes the
# link down -- it has to, since the reply would otherwise be written into the
# connection the switch is breaking -- so this is the association and a DHCP round,
# measured at eight to twenty seconds on this hardware, plus room to spare.
WIFI_REJOIN_S = 25.0
WIFI_REJOIN_MS = int(WIFI_REJOIN_S * 1000)   # browser timers count milliseconds


def wifi_verdict(level: Any) -> str:
    """One word for a dBm reading, or "poor" for no reading at all.

    No reading means the interface is not reporting a signal, which is not a good
    sign and must not be coloured like one.
    """
    if not isinstance(level, (int, float)):
        return "poor"
    if level >= WIFI_GOOD_DBM:
        return "good"
    return "fair" if level >= WIFI_POOR_DBM else "poor"


def renderer():
    """`lidar_slam/mapimg.py`, or None where the repository is not beside us.

    Two things in the consoles need it and neither can be reimplemented honestly:
    the colour key, and turning a click on the picture back into a place in the
    room. Both are inverses of code that lives in the renderer, so both ask the
    renderer rather than keeping a second copy of its arithmetic.
    """
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "lidar_slam"))
    try:
        import mapimg
    except ImportError:
        return None
    return mapimg


def legend():
    """Swatch colours and labels for the map key, taken from the renderer itself.

    The rover's own `mapimg` is the authority on what colour means what, and a key
    that has drifted from the picture is worse than no key at all -- so the palette
    is imported rather than copied, and if it cannot be found the key is simply
    omitted. It is a label on a picture; it is not worth failing to start over.
    """
    mapimg = renderer()
    if mapimg is None:
        return ()
    return tuple(("#%02x%02x%02x" % colour, label) for colour, label in (
        (mapimg.C_ROVER, "rover"),
        (mapimg.C_CAMERA, "camera"),
        (mapimg.C_TRACK, "driven"),
        (mapimg.C_OCCUPIED, "solid"),
        (mapimg.C_FREE, "empty"),
        (mapimg.C_DIM, "unsure"),
        (mapimg.C_UNKNOWN, "unseen")))


MAP_LEGEND = legend()


def or_dash(value, spec="{}"):
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
    ("scan_age_s", "scan age", lambda v: or_dash(v, "{:.2f} s")),
    ("driving", "driving", lambda v: "yes" if v else "no"),
    ("estop", "e-stop", lambda v: "LATCHED" if v else "clear"),
    ("speed_ms", "speed", lambda v: or_dash(v, "{:.3f} m/s")),
    ("turn_dps", "turn rate", lambda v: or_dash(v, "{:.1f} deg/s")),
    ("pwm", "pwm L,R", lambda v: "-" if not v else f"{v[0]}, {v[1]}"),
    ("clearance_m", "clearance", lambda v: or_dash(v, "{:.2f} m")),
    ("steering_deg", "steering", lambda v: or_dash(v, "{:.1f} deg")),
    ("remaining_m", "to go", lambda v: or_dash(v, "{:.2f} m")),
    ("match_score", "match", lambda v: or_dash(v, "{:.3f}")),
    ("position_trusted", "position", lambda v: "trusted" if v else "NOT TRUSTED"),
    ("scans", "scans", lambda v: or_dash(v)),
    ("dropped_scans", "dropped", lambda v: or_dash(v)),
    # How many times this rover has had to replug its own lidar in software. Shown
    # only once it has happened, because on a healthy rover it is a permanent zero
    # among rows that all mean something -- and once it is not zero it is the most
    # interesting number on the panel: a count that climbs over an afternoon is a
    # cable working loose, and nothing else here would ever say so.
    ("lidar_resets", "usb resets", lambda v: or_dash(v or None)),
)

# Which fields shout when they are wrong. A silent lidar and an untrusted position
# each make every other number on the panel a lie, so neither is left as a quiet
# "False" among a dozen other rows.
ALARM_WHEN_FALSE = ("lidar_live", "position_trusted")
ALARM_WHEN_TRUE = ("estop",)

# The phases worth noticing when they do reach the transcript. A replan is not a
# failure, so it is not red -- but it is the moment the rover changed its mind, and
# it should not read like ordinary progress either.
LOUD_PHASES = ("replanning", "stopping")


def worth_logging(move):
    """Whether this sentence belongs in the transcript as well as on the panel.

    The test is whether it says anything the `-> drive(distance_m=0.5)` line just
    above it did not. A plain drive or turn announcing itself as driving or turning
    says nothing -- the request is already on screen a line higher, and echoing it
    back is how a log becomes a thing people stop reading. What earns a line is the
    planner's verdict on a request, and anything the rover decides for itself once
    it is under way.

    The ending earns none, for a different reason: the move's own reply is already
    on its way carrying the distances, and two accounts of one ending a tenth of a
    second apart read like two things having happened.
    """
    phase = (move or {}).get("phase")
    if phase in ("planning", "replanning", "stopping"):
        return True
    # `driving` covers both "the wheels are turning" and "the planner came back
    # with this route". Only the second is news.
    return phase == "driving" and move.get("route_m") is not None


def asked_for(move):
    """The request, in the units it was made in: what to put after "planning a
    route to". Every kind states its own, because "1.20, -0.40" means nothing."""
    asked = move.get("asked") or {}
    kind = move.get("kind")
    if kind == "drive_to":
        # A tap on the map asks for a point on the map, so that the click means one
        # place even though the rover is still moving when it is sent. Said as such:
        # "ahead +0.00 m" for a place the rover has already driven past would be a
        # sentence about the wrong thing entirely.
        if asked.get("x_m") is not None:
            return "the point x {:+.2f}, y {:+.2f} on the map".format(
                float(asked["x_m"]), float(asked.get("y_m") or 0.0))
        return "ahead {:+.2f} m, left {:+.2f} m".format(
            float(asked.get("ahead_m") or 0.0), float(asked.get("left_m") or 0.0))
    if kind == "turn_in_place":
        return "{:+.0f} deg".format(float(asked.get("angle_deg") or 0.0))
    if kind == "drive":
        distance = asked.get("distance_m")
        return "as far as it can" if distance is None else f"{float(distance):.2f} m"
    return ""


def move_sentence(move):
    """What the rover says the move in flight is doing, as one line for a person.

    The navigator publishes this into `nav_status` as the move runs, which is the
    only way to hear about a move before it is over: `drive_to` is one blocking
    call that plans, drives, and may plan again several times, and it does not
    answer until all of that has happened. Without this a click on the map bought
    a stopwatch and nothing else, and a route the planner had refused outright
    looked exactly like a route still being driven.

    Pure, and self-tested in [selftest.py](selftest.py), because it is the whole of
    what either window has to say about a move in progress and a GUI is a miserable
    place to debug a sentence.
    """
    if not move:
        return ""
    phase = move.get("phase") or ""
    why = move.get("why") or ""
    where = asked_for(move)
    if phase in ("", "idle"):
        return ""
    if phase == "planning":
        line = f"planning a route to {where}"
    elif phase == "driving" and move.get("route_m") is not None:
        count = move.get("waypoints") or 0
        line = (f"route accepted: {move['route_m']:.2f} m "
                f"through {count} waypoint{'' if count == 1 else 's'}")
    elif phase == "driving":
        line = f"driving {where}"
    elif phase == "turning":
        line = f"turning {where}"
    elif phase == "replanning":
        line = f"replanning (#{move.get('replans') or 1})"
    elif phase == "stopping":
        line = "stopping"
    elif phase == "ended":
        # The reason the navigator gives is the same vocabulary the reply uses --
        # arrived, blocked, stopped, busy -- so the panel and the transcript agree.
        line = str(move.get("reason") or "ended")
        if move.get("replans"):
            line += f", after {move['replans']} replan"
            line += "" if move["replans"] == 1 else "s"
    else:
        line = phase
    return line + (f" -- {why}" if why else "")


def rung(ladder: tuple, value: float) -> int:
    """Where `value` sits in a ladder, tolerating its not being on a rung."""
    try:
        return ladder.index(value)
    except ValueError:
        return min(range(len(ladder)), key=lambda i: abs(ladder[i] - value))


def size_for_panel(width_px: float) -> int:
    """Which rung of MAP_SIZES_PX to ask for when a panel this wide is what there is.

    Rounded down rather than to nearest, and that is the whole of the policy. The
    picture costs the rover roughly its own area to draw, so overshooting a 500 px
    column with an 800 px map spends seconds of a single ARMv6 core producing detail
    the browser immediately throws away. Undershooting costs less and is scaled back
    up by the browser, and on a picture made of 5 cm squares drawn with no
    antialiasing there is nothing there to lose.
    """
    fits = [size for size in MAP_SIZES_PX if size <= width_px]
    return fits[-1] if fits else MAP_SIZES_PX[0]


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


def tap_to_relative(col: float, row: float, view: dict):
    """A pixel on the map picture as (ahead_m, left_m) from where the rover is now.

    The arithmetic is the renderer's, not ours: `mapimg.tap_to_relative` is the
    exact inverse of the sampling `render` does, so a console that worked it out for
    itself would be keeping a second copy of the map's geometry -- wrong the first
    time the resolution or the centring changed. None where the renderer is not
    beside us, which the caller reports rather than guessing.
    """
    mapimg = renderer()
    if mapimg is None:
        return None
    pose = view.get("pose") or {}
    heading = math.radians(float(pose.get("heading_deg", 0.0)))
    return mapimg.tap_to_relative(
        col, row, view["half_extent_m"], view["scale"],
        rover_up=bool(view.get("rover_up")), heading_rad=heading)


def tap_to_point(col: float, row: float, view: dict):
    """A pixel on the map picture as (x_m, y_m) on the map itself.

    The same click as `tap_to_relative`, read as a fixed place rather than as an
    offset from the rover. That is the difference that lets a click land while the
    rover is still driving: an offset is measured from wherever it has got to by
    the time the call arrives, and a place on the map stays where it was clicked
    however long the stop takes to land.

    The pose is the one the picture was drawn at, which the daemon returns with
    every map -- so a click on a two-second-old picture means the point in the room
    that was under the cursor, not that point translated by two seconds of driving.
    """
    mapimg = renderer()
    if mapimg is None:
        return None
    pose = view.get("pose") or {}
    return mapimg.tap_to_point(
        col, row, view["half_extent_m"], view["scale"],
        rover_up=bool(view.get("rover_up")),
        pose=(float(pose.get("x_m", 0.0)), float(pose.get("y_m", 0.0)),
              math.radians(float(pose.get("heading_deg", 0.0)))))
