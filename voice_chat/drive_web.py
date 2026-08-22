#!/usr/bin/env python3
"""Drive the rover from a browser: the driving tools, in a window that resizes.

This began as a tkinter window, and that window is where everything here was
measured -- five connections, the pacing, the sentences the navigator publishes
mid-move. What it could never do is fit on a screen. Its panels were laid out at
fixed sizes because tk has no notion of reflowing them, so the log and the turns
table sat below the bottom edge of a 1080p display with no scrollbar to reach
them, and widening the window only added empty space to the right of the camera. A
browser solves that in about ten lines of CSS, and solves it properly: the page
scrolls, the columns rewrap as the window narrows, and on a phone it comes out as
one column in the right order.

    python voice_chat/drive_web.py                      # finds the rover, opens a tab
    python voice_chat/drive_web.py --rover rpi.local:8769
    python voice_chat/drive_web.py --bind 0.0.0.0       # ...and let the phone in
    python voice_chat/mock_rover.py --drive             # ...with no rover at all

**This server runs on your desk, not on the rover, and that is the whole answer to
whether the Pi can afford a web console.** It cannot afford one and it is never
asked to. What is on the Pi is `rover_daemon.py`, exactly as before, answering the
same six TCP connections with the same JSON it has always answered; the HTTP, the
event stream and the page are all at this end. The rover cannot tell that the
thing calling `nav_status` three times a second is a browser rather than a desk
program with a window in it, because in the only sense that matters to a 700 MHz
ARMv6 core running SLAM, it is not.

**And the browser gives two things back for free.** It reads JPEG, so the frame
from the camera goes straight into an `<img>` -- which deleted the one dependency
this console used to have, the OpenCV decode that existed solely because tk reads
PNG, GIF and PPM and nothing else, along with the fallback that wrote the frame to
a file and told you where. It also scales pictures, so the map can be drawn at
whatever size the Pi can afford and then fitted to whatever width the panel
happens to have, with `image-rendering: pixelated` -- which on a picture made of
5 cm squares with no antialiasing in it loses nothing at all.

**The page holds no state of its own.** Everything on screen is rendered from one
JSON object this server pushes down a `text/event-stream`, and every button posts
an action back and renders nothing until the state says so. That is not a taste in
architectures: it is the same reason face tracking is polled rather than
remembered. A button that greys itself out because you
pressed it is a button that lies when the rover refuses, and here there can be two
browsers open on the same rover, so a page that believed its own clicks would
disagree with the room.

**The pictures do not travel in that stream.** A map is 40-200 kB of base64 and
the stream carries a fresh state ten times a second, so the map and the camera
frame are kept back as ordinary HTTP resources -- `/map.png`, `/frame.jpg` -- and
the state carries a counter that goes up when a new one arrives. The page changes
the `src` when the counter moves, the browser fetches it once, and everything in
between is a few kilobytes of numbers.

**Closing the tab stops the rover**, which a desktop window gets almost for free
from its close handler and is harder to keep here: a browser tab that goes away
says nothing, and the server outlives it. So the rule is on this side instead --
when the last event stream has been gone for a couple of seconds and a move is
still running, the stop goes out on the connection that carries nothing else. A
reload drops the stream for a fraction of a second and is covered by the grace;
two tabs open means the count never reaches zero. Ctrl-C stops it too, for the
same reason.

Only the standard library, plus [rover_tools.py](rover_tools.py) for the wire and
[console_model.py](console_model.py) for everything both consoles agree about --
which is nearly all of it. The page is [drive_web.html](drive_web.html) beside
this file, read from disk on every request so that editing it needs no restart.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import queue
import socket
import sys
import uuid
import tempfile
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import rover_tools
from console_model import (
    ALARM_WHEN_FALSE, ALARM_WHEN_TRUE, BATTERY_NOTES, BATTERY_POLL_S,
    BATTERY_STALE_S, CAMERA_AUTO_S, CLEAR_ARM_S, Channel, LIGHT_MAX, LOG_LINES,
    LOUD_PHASES, MAP_AUTO_S, MAP_EXTENTS_M, MAP_LEGEND, MAP_SIZES_PX,
    MOVE_TIMEOUT_S, POLL_S, Reply, STATUS_FIELDS, TRACK_POLL_S, TURN_PRESETS_DEG,
    TURN_ROWS, WIFI_POLL_S, WIFI_REJOIN_S, WIFI_SCAN_TIMEOUT_S, move_sentence,
    or_dash, rung, size_for_panel, tap_to_relative, wifi_verdict, worth_logging)

# How often the pump wakes: drain what came back, decide what to ask for next, and
# publish the state if it changed. The tkinter window this grew out of ran its loop
# at the same rate for the same reason -- fast enough that the in-flight timer
# reads like a stopwatch, and slow enough that a state pushed on every tick is ten
# a second rather than a thousand.
TICK_S = 0.1
# A comment line down an idle stream, so that a proxy or a laptop suspending itself
# is noticed rather than leaving a page that has quietly stopped updating.
KEEPALIVE_S = 15.0
# How long the last browser has to come back before a move in flight is stopped.
# A reload takes a fraction of this; a closed tab never comes back.
ORPHAN_GRACE_S = 2.0
# How long after a search that found nothing to look again, multiplied by the number
# of tries and held at the ceiling. Two seconds so that opening the page a moment
# before the daemon is up costs nothing, fifteen so that a rover switched off for the
# evening is not being dialled all night.
RECONNECT_S = 2.0
RECONNECT_MAX_S = 15.0
# How long the rover may leave the status poll unanswered before the six connections
# are thrown away and remade. Well past a wifi hiccup on purpose: the client under
# each connection already remakes its own socket per call, so a link that merely
# stumbled needs nothing from here. What needs the reconnect is a rover that came
# back *different* -- restarted, or on another address -- because the tool list and
# the light level are asked once, on connect, and would otherwise stay stale.
LINK_LOST_S = 8.0
DEFAULT_HTTP_PORT = 8770

PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drive_web.html")


class Session:
    """The rover, as one object a browser can render: the six connections, the
    pacing, and one dict that is everything on screen.

    Every field is written by the pump thread and read under a lock by whichever
    HTTP thread is serving an event stream. Actions arrive the other way, on a
    queue, and are executed by the pump -- so there is exactly one writer, which a
    single-threaded GUI event loop would have given for free and which would
    otherwise be the first thing to go wrong here.
    """

    def __init__(self, address: str | None, half_extent: float,
                 map_size: int) -> None:
        self.half_extent = half_extent
        self.map_size = map_size
        self.wanted_address = address or ""
        #: Unique to this console process, and mixed into the URL of every picture
        #: it publishes. Without it the pictures of one run are served at exactly
        #: the URLs of the last one -- `/map.png?gen=1`, `?gen=2` -- because the
        #: counter starts again at 1 in every new process. They are sent
        #: `immutable` for a year, so a browser that has seen a run does not ask
        #: about those URLs again: it draws the *previous* run's pictures, in order,
        #: as the new counter climbs past the numbers it already holds. That is a
        #: whole recorded run played back over a live rover, the same one every
        #: time, and nothing about it is cleared by restarting or rebooting -- it is
        #: on disk in the browser's cache. Reproduced against the mock and the real
        #: rover in turn: the second console served a different picture at
        #: `?gen=1` and the browser never asked it for one.
        self.run_id = uuid.uuid4().hex[:8]
        self.replies: queue.Queue = queue.Queue()
        self.actions: queue.Queue = queue.Queue()

        self.moves: Channel | None = None     # blocking, one bounded move at a time
        self.halt: Channel | None = None      # stop, and nothing else, ever
        self.watch: Channel | None = None     # status, questions, tool list
        self.picture: Channel | None = None   # the map, which is slow enough to matter
        self.camera: Channel | None = None    # frames, which are slower still
        self.scanner: Channel | None = None   # one scan, slower than all of them
        self.channels: list[Channel] = []

        self.address = ""
        self.link_text = "not connected"
        self.tools: list[str] = []
        self.can_drive = False
        self.busy_since: float | None = None
        self.busy_name = ""
        # The navigator's own count of the sentences it has published about the move
        # it is running. Kept so that polling three times a second writes one line
        # per thing the rover said rather than thirty. See move_sentence.
        self.move_seq: int | None = None
        # Set when a move's reply has been printed, and cleared again by the record
        # that says that move ended. Everything in between is commentary the reply
        # has already overtaken -- see _show_move.
        self.move_answered = False
        self.poll_outstanding = False
        self.poll_at = 0.0
        # The search for the rover, which runs on its own thread and is tried again
        # for as long as it keeps failing -- see mind_the_link. `find_at` is when the
        # last one started, `find_tries` how many have failed in a row, and
        # `said_lost` whether the log has been told, so that a rover switched off for
        # an hour costs one line rather than one line every fifteen seconds.
        self.find_outstanding = False
        self.find_at = 0.0
        self.find_tries = 0
        self.said_lost = False
        # When the status poll was last answered. Measured from the answer rather
        # than from the failure, because a rover that has been unplugged does not
        # refuse the call -- the socket sits there until it times out twelve seconds
        # later, and a clock started then finds out about it twenty seconds late. The
        # poll goes out three times a second, so anything past a few seconds of this
        # is silence whether or not a refusal has arrived to prove it.
        self.answered_at = 0.0

        self.status_rows: list[list[Any]] = []
        self.status_error = ""
        # What the rover last said about its own sensor, kept so the reset button
        # can be offered exactly when it is worth pressing. `lidar_live` is the
        # honest test rather than `lidar_ok`: the map is suspended through a
        # dead-reckoned turn, so `lidar_ok` goes false on a sensor that is fine.
        self.lidar_live: bool | None = None
        self.lidar_note = ""
        self.pose_text = "-"
        self.plan_text = "-"
        self.heading_deg = 0.0

        self.map_outstanding = False
        self.map_wanted = False        # the view moved while one was already in flight
        self.map_at = 0.0
        # When the picture on screen was drawn, as against when one was last asked
        # for. The two differ by however long the rover took, and the page shows the
        # first: a map is a photograph of a moment, and a console that displays one
        # without saying how old it is invites reading a stale picture as the room
        # the rover is in now.
        self.map_drawn_at = 0.0
        self.map_cost = 0.0            # how long the rover said the last one took
        self.map_png: bytes = b""
        self.map_gen = 0
        self.map_shape = (0, 0)
        self.map_view: dict[str, Any] | None = None
        self.map_error = ""
        self.map_note = ""
        self.map_caption = ""
        self.map_auto = True
        # Whether the size asked for follows the panel. On, because the whole reason
        # this console is a page is that the panel has a size and the fixed box it
        # replaced did not; off is for pinning a size to compare two pictures at.
        self.map_fit = True
        self.panel_px = 0.0
        # Which way is up. Off, the page keeps the heading the rover started with, so
        # the room holds still and the arrow turns -- right for watching where the
        # rover has got to. On, the page turns with the rover, so ahead is always up
        # and the room swings instead, which is what you want when the question is
        # whether it will fit through the gap in front of it.
        self.rover_up = False

        self.frame_outstanding = False
        self.frame_at = 0.0
        self.frame_cost = 0.0          # how long the last one took to arrive
        self.frame_jpeg: bytes = b""
        self.frame_gen = 0
        self.frame_note = ""
        self.frame_error = ""
        self.frame_auto = False

        self.track_outstanding = False
        self.track_at = 0.0
        self.track_text = "-"
        self.light_level: int | None = None
        self.battery_outstanding = False
        self.battery_at = 0.0
        self.battery: dict[str, Any] = {"text": "-", "state": "", "note": ""}

        # None until the rover has been asked once. The network calls are not in
        # `list_tools` -- no model is offered them, since one that switched networks
        # would be cutting the wire its own conversation arrives on -- so support is
        # discovered by asking rather than read off the tool list, and a rover
        # running an older daemon leaves this False and the panel quiet.
        self.wifi_ok: bool | None = None
        self.wifi_outstanding = False
        self.wifi_at = 0.0
        self.wifi_joining: str | None = None
        self.rejoin_at = 0.0
        self.wifi: dict[str, Any] = {"supported": None, "text": "-", "verdict": "",
                                     "where": "", "note": "", "networks": [],
                                     "scanning": False, "joining": None}

        self.turns: list[dict[str, str]] = []
        self.clear_armed_until = 0.0

        self.log: list[dict[str, Any]] = []
        self.log_seq = 0

        # What the browsers are told, and how they are woken. `published` is the
        # JSON of the last state pushed, kept so that a tick which changed nothing
        # pushes nothing.
        self.lock = threading.Condition()
        self.published = ""
        self.version = 0
        self.listeners = 0
        self.alone_since = 0.0
        self.stopped_orphan = False
        self.running = True

    def tag(self, count: int) -> str:
        """The name a picture is published under: this run, and which picture.

        Empty while there is no picture, because the page reads a falsy generation
        as "nothing to show yet" and would otherwise ask for a map that has never
        been drawn.
        """
        return f"{self.run_id}-{count}" if count else ""

    # --- what the browser sees ------------------------------------------------
    def snapshot(self) -> dict[str, Any]:
        """Everything on screen, as JSON. Rebuilt whole on every tick and compared
        with the last one, rather than each writer remembering to mark the state
        dirty -- the state is a few kilobytes and this is a desk, and a missed dirty
        flag is a panel that silently stops updating."""
        busy = None
        if self.busy_since is not None:
            busy = {"name": self.busy_name,
                    "seconds": round(time.monotonic() - self.busy_since, 1)}
        facing = "rover up" if self.rover_up else "start heading up"
        return {
            "link": {"address": self.address or self.wanted_address,
                     "text": self.link_text,
                     "connected": bool(self.channels),
                     "can_drive": self.can_drive,
                     "tools": self.tools},
            "busy": busy,
            "status": {"rows": self.status_rows, "pose": self.pose_text,
                       "error": self.status_error},
            "lidar": {"offer": self.lidar_live is False, "note": self.lidar_note},
            "plan": self.plan_text,
            "map": {"gen": self.tag(self.map_gen), "width": self.map_shape[0],
                    "height": self.map_shape[1], "note": self.map_note,
                    "caption": self.map_caption, "error": self.map_error,
                    "drawing": self.map_outstanding,
                    "half_extent_m": self.half_extent, "size_px": self.map_size,
                    "rover_up": self.rover_up, "auto": self.map_auto,
                    "fit": self.map_fit,
                    "age_s": (None if not self.map_gen
                              else round(time.monotonic() - self.map_drawn_at, 1)),
                    "settings": f"{2 * self.half_extent:.0f} m across, "
                                f"{self.map_size} px picture, {facing}"},
            "frame": {"gen": self.tag(self.frame_gen), "note": self.frame_note,
                      "error": self.frame_error, "auto": self.frame_auto,
                      "taking": self.frame_outstanding},
            "tracking": self.track_text,
            "lights": {"level": self.light_level,
                       "text": "-" if self.light_level is None else
                               f"{'on' if self.light_level else 'off'} "
                               f"({self.light_level})"},
            "battery": self.battery,
            "wifi": self.wifi,
            "turns": self.turns,
            "clear_armed": self.clear_armed_until > time.monotonic(),
            "watching": self.listeners,
        }

    # --- the pump -------------------------------------------------------------
    def run(self) -> None:
        self.connect()
        while self.running:
            began = time.monotonic()
            self.pump()
            time.sleep(max(0.0, TICK_S - (time.monotonic() - began)))

    def pump(self) -> None:
        while True:
            try:
                self.act(self.actions.get_nowait())
            except queue.Empty:
                break
        while True:
            try:
                reply = self.replies.get_nowait()
            except queue.Empty:
                break
            self.handle(reply)

        now = time.monotonic()
        if (self.watch is not None and not self.poll_outstanding
                and now - self.poll_at > POLL_S):
            self.poll_outstanding = True
            self.poll_at = now
            # Saying which sentence about the move we already have is what makes a
            # third-of-a-second poll safe: a replan lasts about as long as the
            # planner takes and would otherwise come and go between two of these.
            self.watch.submit("nav_status", {"since_seq": self.move_seq})
        # Auto-refresh leaves the rover as long to breathe as the last map took to
        # draw. Asking every two seconds for a picture that takes two and a half is
        # how you keep a single-core Pi permanently drawing maps while it is also
        # running the SLAM it is drawing them from.
        if (self.picture is not None and self.map_auto and not self.map_outstanding
                and now - self.map_at > max(MAP_AUTO_S, self.map_cost)):
            self.refresh_map()
        # Paced off what the last one cost, like the map: a cold camera takes seconds
        # to produce a frame, and asking again while it is still opening would keep a
        # single-core Pi taking pictures for the whole time it is meant to be driving.
        if (self.camera is not None and self.frame_auto and not self.frame_outstanding
                and now - self.frame_at > max(CAMERA_AUTO_S, self.frame_cost)):
            self.take_picture()
        # Asked, not remembered: the daemon parks tracking by itself when the wheels
        # turn, so the only honest source for this panel is the daemon.
        if (self.watch is not None and not self.track_outstanding
                and now - self.track_at > TRACK_POLL_S):
            self.track_outstanding = True
            self.track_at = now
            self.watch.submit("tracking_status")
        # The battery, on the status connection and at a thirtieth of its rate. Only
        # once the daemon has said it offers it, so a rover still running an older
        # daemon shows an empty panel rather than a red error every ten seconds.
        if (self.watch is not None and "battery" in self.tools
                and not self.battery_outstanding
                and now - self.battery_at > BATTERY_POLL_S):
            self.battery_outstanding = True
            self.battery_at = now
            self.watch.submit("battery")
        # The network, on the same connection. Only once the rover has answered one
        # of these successfully, so a daemon that has never heard of them is asked
        # exactly once rather than every five seconds for the rest of the session.
        if (self.watch is not None and self.wifi_ok and not self.wifi_outstanding
                and now - self.wifi_at > WIFI_POLL_S):
            self.wifi_outstanding = True
            self.wifi_at = now
            self.watch.submit("wifi_status")
        if self.rejoin_at and now > self.rejoin_at:
            self.rejoin_at = 0.0
            self.rejoined()
        if self.clear_armed_until and now > self.clear_armed_until:
            self.clear_armed_until = 0.0
        self.mind_the_link(now)
        self.mind_the_watchers(now)
        self.publish()

    def retry_in(self) -> float:
        """How long to leave it before looking for the rover again."""
        return min(RECONNECT_MAX_S, RECONNECT_S * max(1, self.find_tries))

    def mind_the_link(self, now: float) -> None:
        """Keep looking for the rover instead of waiting to be asked again.

        There are two ways this console loses its rover and neither is the user's
        doing. It may never have had one -- the page opens before the daemon is up,
        or before the Pi has finished enumerating its lidar 93 seconds into a boot
        -- and it may lose one mid-session, which for a rover driving around a house
        on wifi is ordinary rather than exceptional. Both used to end at "no daemon
        answered" and a connect button, which is the wrong thing to need at the
        moment the stop button has stopped working.

        So a search that found nothing is tried again, backing off to every fifteen
        seconds, and a link that has gone quiet for LINK_LOST_S is thrown away so
        that the search picks it up. Not while a move is in flight: the move channel
        waits longer than this does, and remaking the connections under it would
        abandon the one reply that says what the rover did. A move that is genuinely
        gone ends at its own timeout, and the reconnect follows a tick later.
        """
        if self.wifi_joining or self.rejoin_at:
            # A join takes the rover off this network deliberately, and `rejoined`
            # is already scheduled to reconnect whatever came of it.
            return
        if self.channels:
            if (self.busy_since is None
                    and now - self.answered_at > LINK_LOST_S):
                self.say(f"no answer from the rover for "
                         f"{now - self.answered_at:.0f} s, so reconnecting\n", "bad")
                self.said_lost = True
                self.find_tries = 0
                self.connect()
            return
        if not self.find_outstanding and now - self.find_at > self.retry_in():
            self.connect()

    def mind_the_watchers(self, now: float) -> None:
        """Stop the rover once the last browser has been gone a couple of seconds.

        A desktop window sends a stop from its close handler, and a closed tab
        has no equivalent -- it simply stops reading, and this process carries on
        driving a rover nobody is looking at. So the promise is kept from this side:
        the event stream is the browser being present, and losing the last one while
        a move is running is treated exactly like closing the window.

        The grace matters. A reload tears the stream down and puts it back within a
        few hundred milliseconds, and stopping a move for that would make the page
        unreloadable during the only minute it is interesting.
        """
        if self.listeners:
            self.alone_since = 0.0
            self.stopped_orphan = False
            return
        if not self.alone_since:
            self.alone_since = now
            return
        if (self.busy_since is not None and not self.stopped_orphan
                and now - self.alone_since > ORPHAN_GRACE_S):
            self.stopped_orphan = True
            self.say("nobody is watching and a move is running, so it is being "
                     "stopped\n", "bad")
            self.stop()

    def publish(self) -> None:
        text = json.dumps(self.snapshot(), separators=(",", ":"))
        with self.lock:
            if text == self.published:
                return
            self.published = text
            self.version += 1
            self.lock.notify_all()

    # --- connecting -----------------------------------------------------------
    def connect(self) -> None:
        # Dropped on a thread of their own, because closing one of these can block
        # for as long as the call in flight on it. The socket lock is held by the
        # thread waiting on the reply, so closing a connection to a rover that has
        # been unplugged waits out the twelve-second read timeout first -- six times
        # over, on the pump thread, which is the thread that reads the stop button.
        # That is the wrong thing to be doing at the moment the rover has vanished.
        # Nothing refers to these once `channels` is emptied below, so they can be
        # left to die in their own time; the worst of it is a reply from the old link
        # arriving after the new one is up, which the next tick asks again for anyway.
        abandoned, self.channels = self.channels, []
        if abandoned:
            threading.Thread(target=lambda: [c.close() for c in abandoned],
                             daemon=True, name="rover-abandon").start()
        # All of them, including the two an earlier reconnect path left pointing at a
        # closed socket: a submit on a closed channel is queued to a thread that has
        # already returned, so the map simply never comes back and the "one at a
        # time" flag stays set for good.
        self.moves = self.halt = self.watch = self.picture = self.camera = None
        self.scanner = None
        self.frame_outstanding = False
        self.map_outstanding = False
        self.wifi_outstanding = False
        # Not forgotten across a reconnect: a reconnect is mostly what happens
        # *because* of a join, and the panel's job at that moment is to say whether
        # the rover came back on the network it was asked for.
        self.wifi_ok = None
        # Forgotten across a reconnect, so that a rover found mid-move says once
        # what it is doing instead of staying silent until the next phase.
        self.move_seq = None
        self.tools = []
        self.can_drive = False
        self.busy_since = None
        self.link_text = "looking for the rover..."
        self.find_outstanding = True
        self.find_at = time.monotonic()
        # A fresh link is given the same patience as an established one, counted
        # from now: the first poll cannot be answered before it has been sent.
        self.answered_at = self.find_at

        wanted = self.wanted_address.strip()

        def find() -> None:
            # On a thread, because a failed name lookup costs seconds -- 7.3 of them
            # on this LAN, measured -- and a console that freezes while it looks is
            # a console nobody should trust with a stop button.
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

    def connected(self, address: str) -> None:
        self.address = address
        self.wanted_address = address
        self.moves = Channel("move", address, self.replies, timeout=MOVE_TIMEOUT_S)
        self.halt = Channel("stop", address, self.replies)
        self.watch = Channel("watch", address, self.replies)
        self.picture = Channel("map", address, self.replies)
        self.camera = Channel("camera", address, self.replies)
        # Its own connection because a scan outlasts everything else here by a
        # factor of three, and its own patience because it outlasts the default.
        self.scanner = Channel("scan", address, self.replies,
                               timeout=WIFI_SCAN_TIMEOUT_S)
        self.channels = [self.moves, self.halt, self.watch, self.picture,
                         self.camera, self.scanner]
        self.link_text = f"{address}: asking what it can do"
        self.watch.submit("list_tools")
        # The board cannot be read back, so the daemon only knows the level it last
        # set. Ask once on connect and the panel starts out true rather than blank.
        self.watch.submit("get_lights")
        # And once for the network, which is also how this finds out whether the
        # rover has the calls for it at all.
        self.wifi_outstanding = True
        self.watch.submit("wifi_status")

    # --- what the buttons ask for ---------------------------------------------
    def act(self, action: dict[str, Any]) -> None:
        """One posted action, run on the pump thread so that nothing else is."""
        what = action.get("do")
        if what == "connect":
            self.wanted_address = str(action.get("address") or "")
            self.connect()
        elif what == "stop":
            self.stop()
        elif what == "drive":
            arguments: dict[str, Any] = {"distance_m": _number(
                action.get("distance_m"), 0.5)}
            speed = _number(action.get("speed_ms"), None)
            if speed is not None:
                arguments["speed_ms"] = speed
            self.move("drive", arguments)
        elif what == "turn":
            self.move("turn_in_place",
                      {"angle_deg": _number(action.get("angle_deg"), 90.0)})
        elif what == "tap":
            self.tap(action)
        elif what == "describe":
            self.watch_call("describe_surroundings")
        elif what == "map":
            self.map_settings(action)
        elif what == "picture":
            self.take_picture()
        elif what == "camera_auto":
            self.frame_auto = bool(action.get("on"))
        elif what == "track":
            name = action.get("name")
            if name in ("start_tracking", "stop_tracking"):
                self.watch_call(name)
                self.track_at = 0.0
        elif what == "lights":
            self.watch_call("set_lights",
                            {"level": int(_number(action.get("level"), 0))})
        elif what == "reset_lidar":
            self.reset_lidar()
        elif what == "clear_map":
            self.clear_map()
        elif what == "wifi_scan":
            self.wifi_scan()
        elif what == "wifi_join":
            self.wifi_join(str(action.get("ssid") or ""))
        elif what == "panel":
            self.panel_px = _number(action.get("map_px"), 0.0) or 0.0
            self.fit_map()

    def watch_call(self, name: str, arguments: dict[str, Any] | None = None) -> None:
        if self.watch is None:
            self.say(f"not connected, so {name} was not sent\n", "bad")
            return
        self.log_sent(name, arguments or {})
        self.watch.submit(name, arguments)

    def move(self, name: str, arguments: dict[str, Any]) -> None:
        """A bounded move, one at a time.

        Refused here rather than sent and refused by the daemon. The daemon's answer
        would be `busy`, which is correct and tells you nothing, and it would land in
        the log between the move and its result, where it reads like the move having
        failed.
        """
        if self.moves is None or not self.can_drive:
            self.say(f"no driving tools on this rover, so {name} was not sent\n",
                     "bad")
            return
        if self.busy_since is not None:
            self.say(f"{self.busy_name} is still running; stop it or wait\n", "quiet")
            return
        self.busy_since = time.monotonic()
        self.busy_name = name
        self.move_answered = False      # a new move's commentary is wanted again
        self.log_sent(name, arguments)
        self.moves.submit(name, arguments)

    def tap(self, action: dict[str, Any]) -> None:
        """A click on the picture is a place relative to the rover, not a pixel.

        The page sends the pixel in the picture's own coordinates -- it divides out
        whatever CSS scaling the panel applied, which is the one piece of arithmetic
        it does -- and the conversion into metres happens here, in the renderer's own
        code. A browser that worked that out for itself would be a third copy of the
        map's geometry.
        """
        if self.map_view is None:
            return
        if "drive_to" not in self.tools:
            self.say("this rover has no drive_to tool, so the tap was not sent\n",
                     "quiet")
            return
        where = tap_to_relative(_number(action.get("col"), 0.0),
                                _number(action.get("row"), 0.0), self.map_view)
        if where is None:
            self.say("cannot convert a tap without mapimg\n", "bad")
            return
        ahead, left = where
        arguments: dict[str, Any] = {"ahead_m": round(ahead, 2),
                                     "left_m": round(left, 2)}
        speed = _number(action.get("speed_ms"), None)
        if speed is not None:
            arguments["speed_ms"] = speed
        self.move("drive_to", arguments)

    def stop(self) -> None:
        """Always allowed, and on the connection that carries nothing else."""
        if self.halt is None:
            self.say("not connected, so there was nothing to stop\n", "quiet")
            return
        self.log_sent("stop_driving", {})
        self.halt.submit("stop_driving")

    def take_picture(self) -> None:
        """On its own connection, because it is the slowest call here: a camera that
        has to be opened takes the rover up to four seconds to deliver a first
        buffer, and while it is doing that nothing else on that socket is answered."""
        if self.camera is None:
            self.say("not connected, so no picture was asked for\n", "bad")
            return
        if self.frame_outstanding:
            return
        self.frame_outstanding = True
        self.frame_at = time.monotonic()
        self.frame_note = "taking one..."
        self.camera.submit("camera_jpeg")

    def reset_lidar(self) -> None:
        """Ask the rover to replug its own lidar.

        On the watch connection rather than the move one, because the point of it is
        to work when the rover is otherwise doing nothing useful -- and because it
        answers immediately: the reset is issued and the device takes a second or
        two to come back on its own, which the scan age will show.
        """
        if self.watch is None:
            return
        self.say("asking the rover to reset the lidar's USB device; the camera and "
                 "the face detector go with it for a few seconds\n", "note")
        self.watch_call("reset_lidar")

    def clear_map(self) -> None:
        """Two presses, and no dialog between them.

        `confirm()` blocks the page's script, which is the script receiving status
        and holding the stop button -- the same objection a desktop window has to a
        modal dialog sitting on its event loop, for the same reason. Arming the
        button costs one extra press and takes nothing away. It disarms itself
        after CLEAR_ARM_S, so a press forgotten about does not lie in wait.
        """
        now = time.monotonic()
        if now > self.clear_armed_until:
            self.clear_armed_until = now + CLEAR_ARM_S
            return
        self.clear_armed_until = 0.0
        if self.picture is None:
            self.say("not connected, so the map was not cleared\n", "bad")
            return
        # On the map's connection, so that a picture already being drawn comes back
        # before the clear rather than after it. The other way round shows an empty
        # map and then replaces it with the old one, which reads as the clear having
        # failed.
        self.log_sent("clear_map", {})
        self.picture.submit("clear_map")

    def map_settings(self, action: dict[str, Any]) -> None:
        """Zoom, size and which way is up, from the buttons that step the ladders.

        An extent and a size, never a magnification: the rover derives pixels per
        cell from the two, which is what keeps the picture the same size when the
        view widens. Asking for a magnification instead resized the picture on every
        zoom, which is not zooming.
        """
        if "zoom" in action:
            index = rung(MAP_EXTENTS_M, self.half_extent) + int(action["zoom"])
            self.half_extent = MAP_EXTENTS_M[
                max(0, min(len(MAP_EXTENTS_M) - 1, index))]
        if "size" in action:
            # Stepping the size by hand is a statement that the panel is not to
            # choose it, or the next resize would undo the press.
            self.map_fit = False
            index = rung(MAP_SIZES_PX, self.map_size) + int(action["size"])
            self.map_size = MAP_SIZES_PX[max(0, min(len(MAP_SIZES_PX) - 1, index))]
        if "rover_up" in action:
            self.rover_up = bool(action["rover_up"])
        if "auto" in action:
            self.map_auto = bool(action["auto"])
        if "fit" in action:
            self.map_fit = bool(action["fit"])
            if self.map_fit and self.fit_map():
                return
        self.refresh_map()

    def fit_map(self) -> bool:
        """Match the size asked for to the width the panel turned out to have.

        This is the part the tkinter window this replaced could not do at all: its
        map sat in a box of a fixed size, so the only way to fill a wider window
        was to press "bigger" and hope. Here the page reports what the column came
        out as and the rung below it is asked for -- rounded down, because the
        picture costs the rover its own area to draw and the browser scales what
        arrives.

        Only when the rung actually changes, and that matters more than it looks:
        dragging a window edge produces a resize event per frame, and a map request
        per frame would be a minute of a single ARMv6 core spent on one drag.
        """
        if not self.map_fit or self.panel_px <= 0:
            return False
        wanted = size_for_panel(self.panel_px)
        if wanted == self.map_size:
            return False
        self.map_size = wanted
        self.refresh_map()
        return True

    def refresh_map(self) -> None:
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
            # One at a time, but do not lose the request: the map takes seconds, and
            # a zoom pressed while one is in flight would otherwise be dropped on the
            # floor -- silently, and for good if auto-refresh is off. Remember that
            # the settings moved and ask again as soon as this one lands.
            self.map_wanted = True
            return
        self.map_wanted = False
        self.map_outstanding = True
        self.map_at = time.monotonic()
        self.picture.submit("map_png", {"half_extent_m": self.half_extent,
                                        "pixels": self.map_size,
                                        "rover_up": self.rover_up})

    def wifi_scan(self) -> None:
        """Ask the radio to look around, which costs the rover the link for a moment.

        Said out loud in the panel before it happens, because it is fifteen seconds
        of a rover that answers nothing -- including a stop -- and a button that
        appears to have done nothing for that long is a button people press again.
        Which is also why it goes out until the answer arrives: pressing it twice
        buys two scans and half a minute off channel, not a quicker one.
        """
        if self.scanner is None:
            self.say("not connected, so no scan was sent\n", "bad")
            return
        self.wifi["note"] = "scanning -- the rover is off channel for a few seconds"
        self.wifi["scanning"] = True
        self.wifi_outstanding = True
        self.wifi_at = time.monotonic()
        self.log_sent("wifi_status", {"scan": True})
        self.scanner.submit("wifi_status", {"scan": True})

    def wifi_join(self, ssid: str) -> None:
        """Move the rover onto another network, and expect to lose it.

        The daemon answers this before it acts, so the reply arriving means the
        request was accepted and nothing more. What follows is the link going down
        under all six connections, so the reconnect is scheduled here rather than
        waited for: there is nothing left to be told on.
        """
        if not ssid or self.watch is None:
            return
        self.wifi_joining = ssid
        self.wifi["joining"] = ssid
        self.wifi["note"] = (f"joining {ssid}; the rover will be unreachable for a "
                             f"few seconds")
        self.watch_call("wifi_join", {"ssid": ssid})
        self.rejoin_at = time.monotonic() + WIFI_REJOIN_S

    def rejoined(self) -> None:
        """Reconnect after a join, whatever became of the request.

        Unconditional on purpose. The switch may have worked, may have failed and
        left the rover where it was, or may have left it on a network this desk
        cannot reach -- and the first two are indistinguishable from here until
        something reconnects and asks.
        """
        asked, self.wifi_joining = self.wifi_joining, None
        self.wifi["joining"] = None
        self.wifi["note"] = f"reconnecting after asking for {asked}"
        self.connect()

    # --- what came back -------------------------------------------------------
    def handle(self, reply: Reply) -> None:
        name, body = reply.name, reply.body

        if name == "__found__":
            self.find_outstanding = False
            if body.get("ok"):
                if self.said_lost:
                    self.say(f"the rover answered again on {body['address']}\n",
                             "good")
                self.said_lost = False
                self.find_tries = 0
                self.connected(body["address"])
                return
            self.find_tries += 1
            self.link_text = (f"no daemon answered; looking again every "
                              f"{self.retry_in():.0f} s")
            # Once, however long the outage lasts. The link line is what says this
            # is still trying, and a line per try would bury everything else.
            if not self.said_lost:
                self.said_lost = True
                self.say("no rover daemon answered. Is it running, and is the "
                         "address right? This will keep looking.\n", "bad")
            return
        if name == "nav_status":
            self.poll_outstanding = False
            self.show_status(body)
            return
        if name == "map_png":
            self.map_outstanding = False
            self.show_map(body)
            if self.map_wanted:
                self.refresh_map()
            return
        if name == "list_tools":
            self.show_tools(body)
            return
        if name == "camera_jpeg":
            self.frame_outstanding = False
            self.frame_cost = reply.seconds
            self.show_picture(body)
            return
        if name == "battery":
            self.battery_outstanding = False
            self.show_battery(body)
            return
        if name == "wifi_status":
            self.wifi_outstanding = False
            self.show_wifi(body)
            # A scan is somebody pressing a button and waiting several seconds for
            # an answer, so it goes in the transcript; the five-second poll behind
            # it does not, or the log would be nothing else. The note has been
            # saying a scan is in flight all that time, so it is also what says how
            # it went -- until it did, a panel went on claiming to be scanning for
            # the rest of the session.
            if reply.arguments.get("scan"):
                self.log_reply(reply)
                self.wifi["scanning"] = False
                if body.get("ok"):
                    heard = len(body.get("networks") or [])
                    self.wifi["note"] = (
                        f"heard {heard} network{'' if heard == 1 else 's'} "
                        f"in {reply.seconds:.0f} s")
                else:
                    self.wifi["note"] = (f"the scan did not come back: "
                                         f"{body.get('error', 'no answer')}")
            return
        if name == "tracking_status":
            # Polled by the console rather than asked for by a person, so it updates
            # the panel and stays out of the transcript.
            self.track_outstanding = False
            self.show_tracking(body)
            return
        if name in ("get_lights", "set_lights"):
            if body.get("ok"):
                self.light_level = body.get("level")
            if name == "get_lights":
                return          # asked for on connect, not by a person

        # What is left is something a person asked for, so it is logged.
        moved = name in ("drive", "turn_in_place", "drive_to")
        if moved:
            self.busy_since = None
            self.busy_name = ""
            # The outcome is about to be printed, so anything the poll has not yet
            # caught up with is commentary on a move the log has already finished
            # telling. See show_move.
            self.move_answered = True
        self.log_reply(reply)
        if name == "turn_in_place":
            self.tally_turn(reply)
        if name in ("start_tracking", "stop_tracking") and body.get("ok"):
            self.show_tracking(body)
        if moved or name == "clear_map":
            self.refresh_map()

    def show_tools(self, body: dict[str, Any]) -> None:
        if not body.get("ok"):
            self.link_text = "connected, but it would not say what it can do"
            self.say(f"list_tools failed: {body.get('error')}\n", "bad")
            return
        self.tools = [t.get("function", {}).get("name", "?") for t in body["tools"]]
        self.can_drive = {"drive", "turn_in_place"} <= set(self.tools)
        self.link_text = (f"{self.address}: {len(self.tools)} tools"
                          + ("" if self.can_drive else ", none of them driving"))
        self.say("tools: " + ", ".join(self.tools) + "\n", "quiet")
        if not self.can_drive:
            # Said plainly, because the failure here is a page of live buttons on a
            # rover with no navigator behind them, and the cause is nearly always a
            # daemon started without --lidar.
            self.say("this daemon is offering no driving tools. It was probably "
                     "started without --lidar; or its lidar has not enumerated yet, "
                     "in which case the tools appear when the sensor does and "
                     "connecting again will pick them up.\n", "bad")

    def show_status(self, body: dict[str, Any]) -> None:
        if not body.get("ok"):
            self.status_rows = [[label, "-", False]
                                for _key, label, _fmt in STATUS_FIELDS]
            self.status_error = str(body.get("error", "no status"))
            self.pose_text = "-"
            # Unknown, not dead. A rover that is not answering says nothing about
            # its lidar, and offering to reset one over a link that is down would
            # be a button that cannot do anything.
            self.lidar_live = None
            self.lidar_note = ""
            return
        # The rover is there. Only a reply that says something resets this, so a
        # refusal does not read as an answer -- see mind_the_link.
        self.answered_at = time.monotonic()
        self.status_error = ""
        rows = []
        for key, label, fmt in STATUS_FIELDS:
            value = body.get(key)
            alarm = ((key in ALARM_WHEN_FALSE and not value)
                     or (key in ALARM_WHEN_TRUE and bool(value)))
            rows.append([label, fmt(value), bool(alarm)])
        self.status_rows = rows
        pose = body.get("pose") or {}
        self.heading_deg = float(pose.get("heading_deg", 0.0))
        self.pose_text = "x {:+.2f}  y {:+.2f}  {:+.1f} deg".format(
            pose.get("x_m", 0.0), pose.get("y_m", 0.0), pose.get("heading_deg", 0.0))
        self.show_lidar(body)
        self.show_move(body.get("move") or {})

    def show_lidar(self, body: dict[str, Any]) -> None:
        """Whether the sensor is talking, and what the rover has done about it.

        Two states worth a sentence rather than a row. A sensor that has stopped
        reporting is the one fault that makes every other number on the panel a
        lie, and the rover now tries to fix it by itself -- so the line has to say
        both how long it has been quiet and whether the fixing has been tried,
        or an unattended reset looks like the rover having done nothing.
        """
        self.lidar_live = bool(body.get("lidar_live"))
        note = body.get("lidar_reset_note") or ""
        resets = int(body.get("lidar_resets") or 0)
        if self.lidar_live:
            # Only worth saying once it has happened, and then worth saying: a
            # rover that has replugged its own lidar twice this afternoon has a
            # cable working loose and this is the only place that would show it.
            self.lidar_note = (f"the lidar has been reset {resets} time"
                               f"{'' if resets == 1 else 's'} this session"
                               if resets else "")
            return
        age = body.get("scan_age_s")
        quiet = "not reporting" if age is None else f"quiet for {age:.0f} s"
        self.lidar_note = f"the lidar is {quiet}." + (f" Last reset: {note}"
                                                      if note else "")

    def show_move(self, move: dict[str, Any]) -> None:
        """The line under the map always; the transcript only when the rover has
        said something it has not said before.

        `seq` is the navigator's own counter of the sentences it has published, and
        it is the whole reason this can be polled: without it there is no way to
        tell a phase that has just started from the same phase read again a tenth
        of a second later, and the log would fill with the same line.

        `missed` holds anything the rover said between the last poll and this one,
        oldest first, because a phase can be shorter than the gap between two polls
        -- and the phase that usually is happens to be the replan, which is the one
        worth reading. Those go to the transcript in order; only the newest reaches
        the panel, which is a statement about now.

        A move quicker than the poll is answered before any of this arrives, and its
        commentary would then read as news about something already reported -- the
        planning line printed underneath the outcome it led to. So once a move's
        reply has gone into the log, what the rover said during that move is dropped
        rather than printed late, up to and including the record that ends it.
        Commentary about a move this console did not start is never in that state and
        is always printed, which is how a rover being driven by something else -- or
        from the other browser -- can still be watched here.
        """
        sentence = move_sentence(move)
        self.plan_text = sentence or "-"
        seq = move.get("seq")
        if seq is None or seq == self.move_seq:
            return
        self.move_seq = seq
        for record in (move.get("missed") or []) + [move]:
            if self.move_answered:
                # Still working through what the reply overtook. The ending is the
                # last of it, and anything after belongs to a move not yet answered.
                self.move_answered = record.get("phase") != "ended"
                continue
            line = move_sentence(record)
            if line and worth_logging(record):
                self.say(f"{'':10}   <~ {line}\n",
                         "note" if record.get("phase") in LOUD_PHASES else "quiet")

    def show_map(self, body: dict[str, Any]) -> None:
        if not body.get("ok"):
            self.map_error = str(body.get("error", "no map"))
            return
        try:
            self.map_png = base64.b64decode(body["png_base64"])
        except (KeyError, ValueError) as error:
            self.map_error = f"cannot show the map: {error}"
            return
        self.map_error = ""
        self.map_gen += 1
        self.map_drawn_at = time.monotonic()
        # The daemon says how big what it drew came out, under `pixels`. Where it
        # does not -- an older daemon, or the mock -- the PNG says so itself in its
        # header, which is where the daemon reads it from too. The page needs a real
        # number either way: it sets the panel's aspect ratio from it, and a wrong
        # one puts the click somewhere else in the room.
        width = int(body.get("pixels") or 0) or _png_width(self.map_png)
        self.map_shape = (width, width)
        self.map_caption = str(body.get("caption", ""))
        self.map_view = {
            "half_extent_m": float(body.get("half_extent_m", self.half_extent)),
            "scale": int(body.get("scale") or 1),
            "rover_up": bool(body.get("rover_up")),
            "pose": body.get("pose") or {"heading_deg": self.heading_deg},
        }

        # What the rover actually drew, which is not always the size asked for: a
        # cell has to be a whole number of pixels, so most sizes are only reachable
        # to within a few percent, and a very wide view cannot reach a large one at
        # all. Worth saying, because otherwise "bigger" appearing to do nothing looks
        # like a broken button rather than a picture already as big as that view can
        # be drawn.
        took = body.get("render_s")
        self.map_cost = float(took or 0.0)
        note = f"{width} px at {body.get('scale', '?')} px/cell"
        if body.get("bytes"):
            note += f", {body['bytes'] / 1000:.0f} kB"
        if took is not None:
            note += f", {took:.1f} s to draw"
        if width and abs(width - self.map_size) > self.map_size * 0.1:
            note += f" -- {self.map_size} px was not reachable here"
        self.map_note = note

    def show_picture(self, body: dict[str, Any]) -> None:
        """The frame, straight through to an `<img>`.

        The tkinter window this replaced needed OpenCV at this point, because the
        rover can only send JPEG -- there is no image library on that Pi, which is
        the same reason face detection runs on another host -- and tk reads PNG, GIF
        and PPM. A browser reads JPEG, so the decode, the resize, the BGR-to-RGB and
        the fallback that wrote the frame to a file and said where all went away,
        and the console stopped having a dependency.
        """
        if not body.get("ok"):
            self.frame_error = str(body.get("error", "no picture"))
            self.frame_note = ""
            return
        try:
            self.frame_jpeg = base64.b64decode(body.get("jpeg_base64", ""))
        except ValueError as error:
            self.frame_error = f"those bytes did not decode: {error}"
            return
        self.frame_error = ""
        self.frame_gen += 1
        where = f"pan {or_dash(body.get('pan'))}, tilt {or_dash(body.get('tilt'))}"
        size = f"{or_dash(body.get('width'))}x{or_dash(body.get('height'))}"
        # Which of the two paths it came off. They mean different things: while
        # tracking runs the loop owns the camera and this is its newest frame, which
        # is also the one the gimbal is actually pointed at.
        source = "tracking's own frame" if body.get("live") else "fresh"
        self.frame_note = (f"{size}, {body.get('bytes', 0) / 1000:.0f} kB, {where}, "
                           f"{source}, {self.frame_cost:.1f} s")

    def show_battery(self, body: dict[str, Any]) -> None:
        """Volts and percent, and how much trouble the pack is in.

        The age of the reading is shown only once it is older than the daemon's own
        cache. Inside that window every reading is a few seconds old by design, and
        a number that always carries a caveat is a number nobody reads; past it, the
        board has stopped answering, which is the one thing this panel has to be able
        to say.
        """
        if not body.get("ok"):
            self.battery = {"text": "-", "state": "",
                            "note": str(body.get("error", "no reading"))}
            return
        state = str(body.get("state", "?"))
        percent = body.get("percent")
        text = or_dash(body.get("volts"), "{:.2f} V")
        if percent is not None:
            text += f"   {percent}%"
        note = BATTERY_NOTES.get(state, state)
        age = body.get("reading_age_s") or 0.0
        if age > BATTERY_STALE_S:
            note += f", and read {age:.0f} s ago"
        self.battery = {"text": text, "state": state, "note": note}

    def show_wifi(self, body: dict[str, Any]) -> None:
        """The access point, its strength, and what else was last heard.

        The strength is the driver's dBm rather than the 0-100 figure beside each row
        in the list, and the difference is not cosmetic: measured on this rover's
        dongle, consecutive scans put the *same* association anywhere from 74 to 88
        while the driver held steady within a couple of dB. So the number that gets a
        colour and a verdict is the one worth trusting, and the column in the list is
        only there to rank the alternatives against each other.
        """
        if not body.get("ok"):
            error = str(body.get("error", "no answer"))
            if "no such tool" in error:
                # An older daemon. Say so once, in the panel, and stop asking.
                self.wifi_ok = False
                self.wifi.update({"supported": False, "text": "-", "verdict": "",
                                  "where": "this rover's daemon does not offer the "
                                           "network calls yet", "networks": [],
                                  "scanning": False})
                return
            self.wifi_ok = True         # it knows the call; it just could not answer
            self.wifi.update({"supported": True, "text": "-", "verdict": "",
                              "where": error, "scanning": False})
            return

        self.wifi_ok = True
        ssid = body.get("connected")
        level = body.get("level_dbm")
        if ssid is None:
            text, verdict = "not associated", "poor"
        else:
            text = str(ssid)
            if isinstance(level, (int, float)):
                text += f"   {level:.0f} dBm"
            verdict = wifi_verdict(level)

        where = []
        address = body.get("address")
        # An association with no address is the failure worth naming: every panel on
        # this page has gone blank and the rover looks connected from the outside.
        where.append(str(address) if address else "no address -- DHCP has not answered")
        age = body.get("list_age_s")
        if isinstance(age, (int, float)) and age > WIFI_POLL_S:
            where.append(f"list heard {age:.0f} s ago")
        join = body.get("last_join")
        if isinstance(join, dict):
            got = join.get("ssid")
            where.append(f"joined {got}" if join.get("ok")
                         else f"could not join {got}")

        networks = []
        for entry in body.get("networks") or []:
            in_use, configured = bool(entry.get("in_use")), bool(entry.get("configured"))
            networks.append({
                "ssid": str(entry.get("ssid", "?")),
                "signal": entry.get("signal", "-"),
                "in_use": in_use, "configured": configured,
                # Joinable means configured and not already the one in use.
                "joinable": configured and not in_use,
                "note": "on it" if in_use else ("" if configured
                                                else "no passphrase")})
        self.wifi.update({"supported": True, "text": text, "verdict": verdict,
                          "where": ", ".join(where), "networks": networks,
                          "scanning": False})

    def show_tracking(self, body: dict[str, Any]) -> None:
        if not body.get("ok"):
            self.track_text = str(body.get("error", "-"))
            return
        if not body.get("tracking"):
            self.track_text = "off"
            return
        # "Running" and "following somebody" are different states and the difference
        # is the whole question: a loop that is running and has locked onto nobody is
        # sweeping, which looks identical from here and quite different on the rover.
        who = ("following someone" if body.get("following_someone")
               else "sweeping, nobody yet")
        faces = body.get("faces_in_view")
        self.track_text = (f"on, {who}"
                           + ("" if faces is None else f", {faces} in view"))

    def tally_turn(self, reply: Reply) -> None:
        asked = float(reply.arguments.get("angle_deg", 0.0))
        turned = reply.body.get("turned_deg")
        ratio = (turned / asked) if (turned is not None and asked) else None
        self.turns.insert(0, {
            "asked": f"{asked:+.0f}",
            "turned": "-" if turned is None else f"{turned:+.1f}",
            "ratio": "-" if ratio is None else f"{ratio:.2f}",
            "secs": f"{reply.seconds:.1f}",
            "reason": str(reply.body.get("reason")
                          or reply.body.get("error") or "-")})
        del self.turns[TURN_ROWS:]

    # --- the transcript -------------------------------------------------------
    def log_sent(self, name: str, arguments: dict[str, Any]) -> None:
        shown = ", ".join(f"{k}={v}" for k, v in arguments.items())
        self.say(f"{time.strftime('%H:%M:%S')}  -> {name}({shown})\n", "sent")

    def log_reply(self, reply: Reply) -> None:
        body = reply.body
        ok = bool(body.get("ok"))
        head = f"{'':10}   <- {reply.seconds:5.2f}s  "
        if not ok and "error" in body:
            self.say(head + f"failed: {body['error']}\n", "bad")
            return
        summary = str(body.get("reason", "ok" if ok else "failed"))
        for key, unit in (("travelled_m", " m"), ("turned_deg", " deg"),
                          ("remaining_m", " m to go"),
                          ("clear_ahead_m", " m clear ahead")):
            if body.get(key) is not None:
                summary += f", {body[key]}{unit}"
        self.say(head + summary + "\n", "good" if ok else "bad")
        for key in ("detail", "note", "surroundings", "text"):
            if body.get(key):
                self.say(f"{'':17}{body[key]}\n", "quiet")

    def say(self, text: str, tag: str = "") -> None:
        """One line into the transcript, numbered.

        The number is what lets a browser that has been open for an hour and one
        that just arrived be served from the same list: each stream remembers how
        far it has read and is sent the rest. Trimmed from the front for the reason
        any log window is -- this is meant to be left open for an afternoon of test
        moves.
        """
        with self.lock:
            self.log_seq += 1
            self.log.append({"seq": self.log_seq, "text": text, "tag": tag})
            del self.log[:-LOG_LINES]

    # --- shutting down --------------------------------------------------------
    def close(self) -> None:
        """A console that can start a move has to be able to end one, including by
        being shut down. Sent inline rather than submitted, because the channel
        threads die with the process and a queued stop would go nowhere."""
        self.running = False
        if self.halt is not None:
            try:
                self.halt.client.call("stop_driving", {})
            except Exception:
                pass
        for channel in self.channels:
            channel.close()


def _png_width(png: bytes) -> int:
    """The width out of a PNG's IHDR, which is bytes 16 to 20 of any PNG there is.

    Here so that a map still gets a size when the reply does not carry one, and it
    is the same four bytes `rover_daemon` reads to fill in `pixels`: a whole number
    of cells at a whole number of pixels rarely lands on the size that was asked
    for, so the only honest source is the picture."""
    return int.from_bytes(png[16:20], "big") if len(png) >= 20 else 0


def _number(value: Any, fallback):
    """A number out of JSON, or the fallback. The page sends what was typed into a
    box, and what was typed into a box is whatever somebody typed."""
    try:
        if value is None or value == "":
            return fallback
        return float(value)
    except (TypeError, ValueError):
        return fallback


class Handler(BaseHTTPRequestHandler):
    """The page, the stream, the two pictures, and one POST.

    `protocol_version` is HTTP/1.1 so that the event stream is a connection the
    browser keeps rather than one it has to re-open, and every reply therefore has
    to carry an accurate `Content-Length` or a chunked body -- which is why the
    bodies here are always assembled before the headers go out.
    """

    protocol_version = "HTTP/1.1"
    session: Session = None          # type: ignore[assignment]
    verbose = False

    def log_message(self, fmt: str, *args) -> None:
        if Handler.verbose:
            sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

    # --- replies --------------------------------------------------------------
    def _send(self, body: bytes, kind: str, cache: str = "no-store") -> None:
        self.send_response(200)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", cache)
        self.end_headers()
        self.wfile.write(body)

    def _missing(self, why: str) -> None:
        body = why.encode()
        self.send_response(404)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:            # noqa: N802 - http.server's spelling
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            # Read per request rather than held in memory, so that editing the page
            # and pressing reload is the whole edit cycle. It is a local file on the
            # machine running this; there is nothing to save by caching it.
            try:
                with open(PAGE, "rb") as handle:
                    page = handle.read()
            except OSError as error:
                self._missing(f"{PAGE} is missing: {error}")
                return
            self._send(page, "text/html; charset=utf-8")
        elif path == "/map.png":
            if not self.session.map_png:
                self._missing("no map yet")
                return
            # Immutable because the URL carries the generation: a new map is a new
            # URL, and the one already fetched can never change under it.
            self._send(self.session.map_png, "image/png",
                       "public, max-age=31536000, immutable")
        elif path == "/frame.jpg":
            if not self.session.frame_jpeg:
                self._missing("no frame yet")
                return
            self._send(self.session.frame_jpeg, "image/jpeg",
                       "public, max-age=31536000, immutable")
        elif path == "/setup":
            self._send(json.dumps(setup()).encode(),
                       "application/json; charset=utf-8")
        elif path == "/events":
            self._events()
        else:
            self._missing("no such thing here")

    def do_POST(self) -> None:           # noqa: N802 - http.server's spelling
        if self.path.split("?", 1)[0] != "/do":
            self._missing("no such thing here")
            return
        length = int(self.headers.get("Content-Length") or 0)
        try:
            action = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            action = {}
        # Queued, never executed here. Two browsers and a keyboard shortcut can all
        # post at once, and the pump is the only thread allowed to touch the rover's
        # state -- which a single-threaded GUI event loop would have given for
        # nothing.
        if isinstance(action, dict):
            self.session.actions.put(action)
        self._send(b'{"ok":true}', "application/json; charset=utf-8")

    # --- the stream -----------------------------------------------------------
    def _events(self) -> None:
        """One `text/event-stream` per browser: the state when it changes, and the
        transcript lines this browser has not had.

        Chunked rather than length-delimited, because it never ends. Each browser
        holds its own cursor into the log and its own idea of which version of the
        state it has, so a page opened an hour in gets the whole transcript that has
        survived trimming and a page that has been open all along gets one line.
        """
        session = self.session
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.send_header("Transfer-Encoding", "chunked")
        self.end_headers()

        with session.lock:
            session.listeners += 1
        seen_version, cursor = -1, 0
        try:
            while session.running:
                with session.lock:
                    if session.version == seen_version:
                        session.lock.wait(KEEPALIVE_S)
                    state, seen_version = session.published, session.version
                    lines = [line for line in session.log if line["seq"] > cursor]
                    if lines:
                        cursor = lines[-1]["seq"]
                out = ""
                if state:
                    out += f"event: state\ndata: {state}\n\n"
                if lines:
                    out += ("event: log\ndata: "
                            + json.dumps(lines, separators=(",", ":")) + "\n\n")
                self._chunk(out or ": keepalive\n\n")
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass                       # the tab went away, which is not an error
        finally:
            with session.lock:
                session.listeners -= 1
                session.lock.notify_all()

    def _chunk(self, text: str) -> None:
        body = text.encode("utf-8")
        self.wfile.write(b"%x\r\n%s\r\n" % (len(body), body))
        self.wfile.flush()


class Console(ThreadingHTTPServer):
    """The HTTP server, with one thing changed: a browser leaving is not an error.

    `socketserver` prints a full traceback for any exception that reaches it out of
    a handler, and a browser closing a kept-alive connection reaches it as one. On
    Windows it arrives as `ConnectionAbortedError [WinError 10053]` from the read of
    the *next* request line, which is nobody's bug: the page was reloaded, or the tab
    was closed, or this process was stopped, and the connection did what connections
    do. Elsewhere it is `ConnectionResetError` or a `TimeoutError` from the idle
    handler timeout, for the same reasons.

    Left alone it printed twenty lines of traceback per reload into the window
    somebody is watching the rover in, and that is worse than untidy: it teaches
    whoever is watching to scroll past tracebacks, in the one window where a real one
    would appear. So the ordinary disconnects are swallowed and everything else is
    still printed exactly as it was.
    """

    # Not reusable, deliberately, and this is the one place where the usual advice
    # is backwards. On Windows `SO_REUSEADDR` does not mean "reclaim a port left in
    # TIME_WAIT", it means *share*: a second process binds the same port happily and
    # which of the two a given connection reaches is anyone's guess. So the browser
    # is served its page by one console and posts its buttons to the other, which is
    # not a confusing console -- it is two consoles, one of them showing an earlier
    # session's transcript and map while the rover ignores everything you press.
    # `talk.py` found this on the frame server first; the same answer, for the same
    # reason: refuse to start, and say so.
    allow_reuse_address = False

    def handle_error(self, request, client_address) -> None:
        kind = sys.exc_info()[0]
        if kind is not None and issubclass(kind, (ConnectionError, TimeoutError)):
            return
        super().handle_error(request, client_address)


class OnlyOne:
    """An exclusive lock held for as long as this process lives, so that there is
    only ever one drive console on this machine.

    The port guard above catches the same command typed twice. It does not catch the
    same command typed twice with different ports, and that is the worse case rather
    than the safer one: two consoles on two ports are two clients of one rover, each
    polling three times a second and each asking for a map that costs the Pi's single
    core two and a half seconds to draw. Measured with three of them attached, the
    daemon sat at 48% of the core drawing maps for windows nobody was looking at, and
    a rover that is busy drawing maps is a rover that answers slowly when told to
    stop.

    An OS lock rather than a pid file, because the interesting case is the console
    that died without tidying up: a lock is dropped by the kernel when the process
    goes, however it goes, where a file has to be deleted by something still running.
    The pid is written into the file as *content*, outside the locked region, purely
    so the refusal can name what to close.
    """

    #: Locked a long way past any content, so that reading the pid never contends
    #: with the lock itself. Windows locks byte ranges; nothing reads this one.
    REGION = 1 << 20

    def __init__(self, path: str) -> None:
        self.path = path
        self._fd: int | None = None

    def claim(self) -> str:
        """"" if this process now holds it, or a sentence about who does."""
        try:
            fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o644)
        except OSError as error:
            # A desk that will not let us make a lock file is not a reason to
            # refuse to drive a rover.
            print(f"note: cannot use {self.path} to check for another console "
                  f"({error})", file=sys.stderr)
            return ""
        try:
            self._take(fd)
        except OSError:
            held = self._whoever()
            os.close(fd)
            return (f"another drive console is already running on this machine"
                    f"{held}. Two of them are two clients of one rover, each asking "
                    f"a single-core Pi for maps, and the browser cannot tell which "
                    f"one it is talking to. Close that one first.")
        os.ftruncate(fd, 0)
        os.write(fd, f"{os.getpid()}\n".encode())
        self._fd = fd
        return ""

    def _take(self, fd: int) -> None:
        """The lock itself, which is the one part that is not the same on both."""
        if os.name == "nt":
            import msvcrt

            os.lseek(fd, self.REGION, os.SEEK_SET)
            msvcrt.locking(fd, msvcrt.LK_NBLCK, 1)
            os.lseek(fd, 0, os.SEEK_SET)
        else:
            import fcntl

            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def _whoever(self) -> str:
        try:
            with open(self.path) as handle:
                pid = int(handle.read().split()[0])
        except (OSError, ValueError, IndexError):
            return ""
        return f" (process {pid})"

    def release(self) -> None:
        if self._fd is not None:
            os.close(self._fd)          # which drops the lock with it
            self._fd = None


def setup() -> dict[str, Any]:
    """The handful of things the page needs once and never again: the preset turns
    it draws buttons for, what the daemon calls full brightness, and the colour key
    -- which comes from the renderer on the rover's side of the repository rather
    than being written out again in CSS, for the reason it always has. A key that
    has drifted from the picture is worse than no key at all."""
    return {"presets_deg": list(TURN_PRESETS_DEG),
            "light_max": LIGHT_MAX,
            "legend": [list(entry) for entry in MAP_LEGEND]}


def main(argv=None) -> int | str:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--rover", default=None, metavar="HOST[:PORT]",
                        help="the daemon; omit to look for it (see rover_tools.py)")
    parser.add_argument("--half-extent", type=float, default=3.0, metavar="M",
                        help="metres each way shown in the map (default: %(default)s)")
    parser.add_argument("--map-size", type=int, default=480, metavar="PX",
                        help="how big a map to ask for before the panel has a "
                             "width to go on (default: %(default)s)")
    parser.add_argument("--port", type=int, default=DEFAULT_HTTP_PORT,
                        help="where to serve the page (default: %(default)s)")
    parser.add_argument("--bind", default="127.0.0.1", metavar="ADDRESS",
                        help="0.0.0.0 to let other machines on the LAN drive it; "
                             "there is no password on this (default: %(default)s)")
    parser.add_argument("--no-browser", action="store_true",
                        help="do not open a tab")
    parser.add_argument("--verbose", action="store_true",
                        help="log every HTTP request")
    args = parser.parse_args(argv)

    # Before the session, and before the port: a console that is about to refuse to
    # run should not have opened six connections to the rover on its way to finding
    # out.
    alone = OnlyOne(os.path.join(tempfile.gettempdir(), "rover-drive-console.lock"))
    taken = alone.claim()
    if taken:
        return taken

    session = Session(args.rover, args.half_extent, args.map_size)
    Handler.session = session
    Handler.verbose = args.verbose
    try:
        server = Console((args.bind, args.port), Handler)
    except OSError as error:
        alone.release()
        return (f"cannot serve on {args.bind}:{args.port}: {error}. Something else "
                f"is on that port -- another console, or another program.")
    # Every event stream is a thread that blocks until its browser goes away, so
    # they have to be daemons or Ctrl-C would wait for every open tab to close.
    server.daemon_threads = True

    threading.Thread(target=session.run, daemon=True, name="rover-pump").start()
    where = f"http://{'127.0.0.1' if args.bind == '0.0.0.0' else args.bind}:{args.port}/"
    print(f"drive console on {where}")
    if args.bind == "0.0.0.0":
        print(f"    and on http://{_lan_address()}:{args.port}/ from the LAN -- "
              f"anyone who can reach it can drive the rover")
    if not args.no_browser:
        threading.Thread(target=webbrowser.open, args=(where,), daemon=True).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping the rover and shutting down")
    finally:
        session.close()
        server.shutdown()
        alone.release()
    return 0


def _lan_address() -> str:
    """This machine's address on the LAN, for printing. No packet is sent -- a
    connected UDP socket only picks the route -- so this works with nothing
    listening at the far end."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("192.168.1.1", 9))
        return probe.getsockname()[0]
    except OSError:
        return "this machine"
    finally:
        probe.close()


if __name__ == "__main__":
    sys.exit(main())
