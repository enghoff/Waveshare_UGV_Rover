"""The rover as one object a browser can render: links, pacing, state."""
from __future__ import annotations

import json
import queue
import threading
import time
import uuid
from typing import Any

import _paths  # noqa: F401 — rover_tools and console_model
import rover_tools
from console_model import (
    BATTERY_POLL_S, BATTERY_STALE_S, CLEAR_ARM_S,
    Channel, LIGHT_MAX, MAP_EXTENTS_M, MAP_STALE_S, MOVE_TIMEOUT_S,
    PARKED_FRAME_GAP_S, PARKED_MAP_GAP_S, PARKED_POLL_S, PICTURE_GAP_S,
    SLOW_PICTURE_S,
    POLL_S, Reply, TRACK_POLL_S, TURN_PRESETS_DEG, WIFI_POLL_S, WIFI_REJOIN_S,
    WIFI_SCAN_TIMEOUT_S, WORLD_TIMEOUT_S, rung, tap_to_point,
)
from drive_show import SessionShow, _number, _png_width  # noqa: F401
from drive_world import SessionWorld

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
# How long a click that interrupted a move waits for the wheels to come free. The
# handover itself is inside a second, so this is only ever reached by a stop that
# did not land -- see mind_the_target for why waiting out the move channel's own
# four minutes instead would be worse than giving up.
TARGET_HANDOVER_S = 6.0
DEFAULT_HTTP_PORT = 8770
# On the rover. 8770 is already oak_depth's depth server; the desk still uses
# DEFAULT_HTTP_PORT because that collision does not exist there.
ROVER_HTTP_PORT = 8771


class Session(SessionShow, SessionWorld):
    """The rover, as one object a browser can render: the six connections, the
    pacing, and one dict that is everything on screen.

    Every field is written by the pump thread and read under a lock by whichever
    HTTP thread is serving an event stream. Actions arrive the other way, on a
    queue, and are executed by the pump -- so there is exactly one writer, which a
    single-threaded GUI event loop would have given for free and which would
    otherwise be the first thing to go wrong here.
    """

    def __init__(self, address: str | None, half_extent: float,
                 map_size: int, idle: bool = False) -> None:
        self.half_extent = half_extent
        self.map_size = map_size
        self.wanted_address = address or ""
        # Hosted on the rover: do not become a client until a browser is open,
        # and stop being one when the last tab has been gone for the orphan
        # grace. A desk process is started to drive and is killed when you are
        # done, so it connects at once; a process that lives from boot would
        # otherwise poll nav_status three times a second and draw a map every
        # two, overnight, for nobody.
        self.idle = idle
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
        # Slower than all of them again: one world-state inspection is about a
        # minute on this board, so it gets a connection nothing else can be stuck
        # behind -- least of all the status poll or a stop.
        self.world_link: Channel | None = None
        self.channels: list[Channel] = []

        self.address = ""
        self.link_text = "not connected"
        self.tools: list[str] = []
        self.can_drive = False
        self.busy_since: float | None = None
        self.busy_name = ""
        #: Whether the rover says it is off exploring. Not a thing this console
        #: does -- it is a thing the rover is doing, possibly at somebody else's
        #: asking -- so it is read off `nav_status` and never set from a click.
        self.exploring = False
        # A click on the map that is waiting for the move it interrupted to let go
        # of the wheels: the `drive_to` arguments, and when to give up on them. The
        # place is held in map coordinates rather than as an offset, so waiting does
        # not move it -- see `tap`.
        self.pending_target: dict[str, Any] | None = None
        self.pending_until = 0.0
        # The navigator's own count of the sentences it has published about the move
        # it is running. Kept so that polling three times a second writes one line
        # per thing the rover said rather than thirty. See move_sentence.
        self.move_seq: int | None = None
        self.poll_outstanding = False
        self.poll_at = 0.0
        # The search for the rover, which runs on its own thread and is tried again
        # for as long as it keeps failing -- see mind_the_link. `find_at` is when the
        # last one started, `find_tries` how many have failed in a row, and
        # `said_lost` whether the notice line has been told, so that a rover switched
        # off for an hour is said once rather than every fifteen seconds.
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
        # When the last map finished arriving, which is where the gap before the
        # next one is measured from. It is deliberately not when the last one was
        # asked for: the rover takes seconds to draw one, and a clock started at
        # the request has usually run out before the picture it was pacing lands.
        self.map_done_at = 0.0
        # When the picture on screen was drawn, as against when one was last asked
        # for. The two differ by however long the rover took, and the page shows the
        # first: a map is a photograph of a moment, and a console that displays one
        # without saying how old it is invites reading a stale picture as the room
        # the rover is in now.
        self.map_drawn_at = 0.0
        self.map_png: bytes = b""
        self.map_gen = 0
        self.map_shape = (0, 0)
        self.map_view: dict[str, Any] | None = None
        self.map_error = ""
        self.map_note = ""
        self.map_caption = ""
        # Which way is up. Off, the page keeps the heading the rover started with, so
        # the room holds still and the arrow turns -- right for watching where the
        # rover has got to. On, the page turns with the rover, so ahead is always up
        # and the room swings instead, which is what you want when the question is
        # whether it will fit through the gap in front of it.
        self.rover_up = False

        self.frame_outstanding = False
        self.frame_done_at = 0.0       # when the last frame landed; see map_done_at
        self.frame_cost = 0.0          # how long the last one took to arrive
        self.frame_jpeg: bytes = b""
        self.frame_gen = 0
        self.frame_note = ""
        # When each picture was asked for, so that "drawing" and "taking" mean
        # "slower than usual" rather than "in flight". See `slow`.
        self.map_asked_at = 0.0
        self.frame_asked_at = 0.0
        self.frame_error = ""

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
                                     "where": "", "note": "",
                                     "scanning": False, "joining": None}
        # Served from /wifi.json rather than pushed -- see `set_networks`.
        self.wifi_networks: list[dict[str, Any]] = []
        self.wifi_networks_gen = 0

        self.clear_armed_until = 0.0
        # Whether the rover's own face-tracking loop is running, which is the only
        # reason a parked rover's camera is still worth two frames a second.
        self.tracking_on = False

        # The one line the console says for itself, and the count that makes a new
        # one a new one. See `say`: there is no transcript behind this, so a notice
        # stands until something else is worth saying.
        self.notice: dict[str, Any] = {"seq": 0, "text": "", "tag": ""}
        self.notice_seq = 0
        # Filled in by drive_web when there is a microphone: a callable returning
        # what the page should draw for it. A callable rather than a value because
        # the session it reports on lives in another thread and is replaced every
        # time somebody presses the button.
        self.omni = None

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
        self.world_reset()

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
        dirty -- the state is a few kilobytes, and a missed dirty flag is a
        panel that silently stops updating."""
        busy = None
        if self.busy_since is not None:
            busy = {"name": self.busy_name,
                    # Whole seconds. A tenth was a stopwatch nobody could read at
                    # that speed, and it made the state a new state ten times a
                    # second for as long as the move ran.
                    "seconds": round(time.monotonic() - self.busy_since),
                    # A move that has been superseded is still the move in flight,
                    # so the panel goes on naming it -- but a second of apparently
                    # nothing happening after a click is what makes a console look
                    # as though it dropped the click.
                    "superseded": self.pending_target is not None}
        facing = "rover up" if self.rover_up else "start heading up"
        return {
            "link": {"address": self.address or self.wanted_address,
                     "text": self.link_text,
                     "connected": bool(self.channels),
                     "can_drive": self.can_drive,
                     "tools": self.tools},
            "busy": busy,
            "exploring": self.exploring,
            "status": {"rows": self.status_rows, "pose": self.pose_text,
                       "error": self.status_error},
            "lidar": {"offer": self.lidar_live is False, "note": self.lidar_note},
            "plan": self.plan_text,
            "map": {"gen": self.tag(self.map_gen), "width": self.map_shape[0],
                    "height": self.map_shape[1], "note": self.map_note,
                    "caption": self.map_caption, "error": self.map_error,
                    # Only once it is taking longer than a render always takes --
                    # see SLOW_PICTURE_S. Otherwise this flag alone was two states
                    # per map, saying the console was drawing a map.
                    "drawing": self.slow(self.map_outstanding, self.map_asked_at),
                    "half_extent_m": self.half_extent, "size_px": self.map_size,
                    "rover_up": self.rover_up,
                    # What the picture was drawn at, which the world-state popup
                    # needs to put a bearing on it. Free, because this block is
                    # already a new block whenever there is a new map to say it
                    # about -- the generation beside it has just changed.
                    "view": self.map_view,
                    # Only once it is old enough to be news -- see MAP_STALE_S. A
                    # map that is arriving normally is always a second or two behind
                    # and saying so in tenths was, on its own, most of what this
                    # console put on the wi-fi.
                    "age_s": self.map_age(),
                    "settings": f"{2 * self.half_extent:.0f} m across, {facing}"},
            "frame": {"gen": self.tag(self.frame_gen), "note": self.frame_note,
                      "error": self.frame_error,
                      "taking": self.slow(self.frame_outstanding,
                                          self.frame_asked_at)},
            "tracking": self.track_text,
            "lights": {"level": self.light_level,
                       "text": "-" if self.light_level is None else
                               f"{'on' if self.light_level else 'off'} "
                               f"({self.light_level})"},
            "battery": self.battery,
            # The list of networks is fetched rather than pushed, like the pictures
            # and for the same reason: it is three and a half kilobytes, it changes
            # a few times an hour, and it was riding in every state.
            "wifi": dict(self.wifi, networks_gen=self.tag(self.wifi_networks_gen)),
            # Counts and a generation tag; the body of it is fetched from
            # /world.json when that tag moves, like the network list and for the
            # same reason -- it is tens of kilobytes and this goes out ten times a
            # second.
            "world": self.world_state(),
            "notice": self.notice,
            "clear_armed": self.clear_armed_until > time.monotonic(),
            "watching": self.listeners,
            "omni": self.omni() if self.omni else {"available": False,
                                                   "state": "off", "why": ""},
        }

    def slow(self, outstanding: bool, asked_at: float) -> bool:
        """Whether a picture in flight has been in flight long enough to say so."""
        return outstanding and time.monotonic() - asked_at > SLOW_PICTURE_S

    def map_age(self) -> float | None:
        """How long ago the map on screen was drawn, when that is worth saying."""
        if not self.map_gen:
            return None
        age = time.monotonic() - self.map_drawn_at
        return round(age) if age > MAP_STALE_S else None

    def moving(self) -> bool:
        """Whether the rover is doing something this console asked it to do.

        Everything paced by this answers faster while it is true: the status poll,
        because a move publishes a sentence at a time and they are wanted in order,
        and the two pictures, because a map drawn around a pose that is changing is
        a different picture each time. A click waiting for the wheels counts -- it is
        about to be a move, and the panel should not go quiet in between.
        """
        return self.busy_since is not None or self.pending_target is not None

    # --- the pump -------------------------------------------------------------
    def run(self) -> None:
        if self.idle:
            self.link_text = "waiting for a browser"
        else:
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
        self.mind_the_target(now)
        self.mind_the_watchers(now)
        if self.idle and not self.listeners:
            # Keep the orphan stop above, then drop the six connections so an
            # unwatched console is not a client of the daemon overnight.
            if (self.alone_since
                    and now - self.alone_since > ORPHAN_GRACE_S):
                self.rest()
            self.publish()
            return
        if (self.watch is not None and not self.poll_outstanding
                and now - self.poll_at > (POLL_S if self.moving()
                                          else PARKED_POLL_S)):
            self.poll_outstanding = True
            self.poll_at = now
            # Saying which sentence about the move we already have is what makes a
            # third-of-a-second poll safe: a replan lasts about as long as the
            # planner takes and would otherwise come and go between two of these.
            self.watch.submit("nav_status", {"since_seq": self.move_seq})
        # Both pictures ask again a fixed gap after the last one arrived, rather
        # than on a clock of their own. Whatever the rover charged for the last one
        # is therefore already spent before the gap starts, which is the only way
        # to leave a host that is also running SLAM any room at all: a two-second
        # timer measured from the request left none whenever the map took longer
        # than two seconds to draw, and it usually does.
        #
        # Which gap depends on whether anything is happening. Half a second of
        # 28 kB camera frames and 7 kB map renders is about 370 kbit/s of somebody
        # else's wi-fi, and a rover standing still spends all of it redrawing the
        # same picture.
        map_gap = PICTURE_GAP_S if self.moving() else PARKED_MAP_GAP_S
        frame_gap = (PICTURE_GAP_S if self.moving() or self.tracking_on
                     else PARKED_FRAME_GAP_S)
        if (self.picture is not None and not self.map_outstanding
                and now - self.map_done_at > map_gap):
            self.refresh_map()
        if (self.camera is not None and not self.frame_outstanding
                and now - self.frame_done_at > frame_gap):
            self.take_picture()
        # Asked, not remembered: the voice session and any other console can start
        # or stop tracking, so the only honest source for this panel is the daemon.
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
        self.publish()

    def retry_in(self) -> float:
        """How long to leave it before looking for the rover again."""
        return min(RECONNECT_MAX_S, RECONNECT_S * max(1, self.find_tries))

    def mind_the_link(self, now: float) -> None:
        """Keep looking for the rover instead of waiting to be asked again.

        There are two ways this console loses its rover and neither is the user's
        doing. It may never have had one -- the page opens before the daemon is up,
        or before the rover has finished enumerating its lidar 93 seconds into a boot
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
                         f"{now - self.answered_at:.0f} s, so reconnecting", "bad")
                self.said_lost = True
                self.find_tries = 0
                self.connect()
            return
        if not self.find_outstanding and now - self.find_at > self.retry_in():
            self.connect()

    def mind_the_target(self, now: float) -> None:
        """Give up on a click that the move it interrupted never made room for.

        The ordinary handover is inside a second and happens in `handle`: the stop
        goes out on its own connection, the navigator drops the goal within a
        control cycle, and the move call answers as soon as it does. This is for the
        stop that never landed at all -- and then the move channel says nothing for
        as long as its own patience, which is four minutes. Driving off to a place
        somebody clicked four minutes ago is a rover acting on an intention that has
        expired, so the click is forgotten and the notice line says so.
        """
        if self.pending_target is not None and now > self.pending_until:
            self.forget_target("the move it interrupted did not let go of the "
                               "wheels")

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
                     "stopped", "bad")
            self.stop()

    def publish_soon(self) -> None:
        """Wake the streams now rather than at the next tick.

        The pump republishes ten times a second, which is right for a rover that
        is moving and wrong for a button: pressing the microphone and watching the
        page do nothing for a tenth of a second is exactly the lag that makes
        somebody press it a second time. This does not build the state -- it
        invalidates it, so the next tick is immediate rather than scheduled.
        """
        with self.lock:
            self.published = ""
            self.version += 1
            self.lock.notify_all()

    def publish(self) -> None:
        text = json.dumps(self.snapshot(), separators=(",", ":"))
        with self.lock:
            if text == self.published:
                return
            self.published = text
            self.version += 1
            self.lock.notify_all()

    # --- connecting -----------------------------------------------------------
    def abandon(self) -> None:
        """Drop the six connections without starting a search.

        Closing is on a thread of its own, because closing one of these can block
        for as long as the call in flight on it. The socket lock is held by the
        thread waiting on the reply, so closing a connection to a rover that has
        been unplugged waits out the twelve-second read timeout first -- six times
        over, on the pump thread, which is the thread that reads the stop button.
        That is the wrong thing to be doing at the moment the rover has vanished.
        Nothing refers to these once `channels` is emptied, so they can be left to
        die in their own time; the worst of it is a reply from the old link arriving
        after the new one is up, which the next tick asks again for anyway.
        """
        abandoned, self.channels = self.channels, []
        if abandoned:
            threading.Thread(target=lambda: [c.close() for c in abandoned],
                             daemon=True, name="rover-abandon").start()
        # All of them, including the two an earlier reconnect path left pointing at a
        # closed socket: a submit on a closed channel is queued to a thread that has
        # already returned, so the map simply never comes back and the "one at a
        # time" flag stays set for good.
        self.moves = self.halt = self.watch = self.picture = self.camera = None
        self.scanner = self.world_link = None
        self.frame_outstanding = False
        self.map_outstanding = False
        self.wifi_outstanding = False
        self.poll_outstanding = False
        self.track_outstanding = False
        self.battery_outstanding = False

    def rest(self) -> None:
        """Stop being a client. `--idle` calls this once nobody is watching."""
        if not self.channels and not self.find_outstanding:
            if self.link_text != "waiting for a browser":
                self.link_text = "waiting for a browser"
            return
        self.abandon()
        self.find_outstanding = False
        self.link_text = "waiting for a browser"

    def connect(self) -> None:
        self.abandon()
        # Not forgotten across a reconnect: a reconnect is mostly what happens
        # *because* of a join, and the panel's job at that moment is to say whether
        # the rover came back on the network it was asked for.
        self.wifi_ok = None
        # Forgotten across a reconnect, so that a rover found mid-move says once
        # what it is doing instead of staying silent until the next phase.
        self.move_seq = None
        # Asked again on the next tick, and until it is answered a rover this console
        # is no longer talking to does not get the fast camera gap.
        self.tracking_on = False
        self.tools = []
        self.can_drive = False
        self.world_outstanding = 0
        self.busy_since = None
        # The move connection has just been thrown away, so the reply that would
        # have handed the wheels over is never coming. Said out loud rather than
        # left to time out, because a reconnect is already a confusing minute.
        self.forget_target("the link to the rover was remade")
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
        # The slowest thing this console can ask for, and the reason it has a
        # connection at all: an inspection is tens of seconds of a model looking at
        # a picture, and a status poll queued behind one would leave the lights,
        # the tracking panel and the map stopped for that whole minute.
        self.world_link = Channel("world", address, self.replies,
                                  timeout=WORLD_TIMEOUT_S)
        self.channels = [self.moves, self.halt, self.watch, self.picture,
                         self.camera, self.scanner, self.world_link]
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
        elif what == "explore":
            # Not `move`, which is for calls that hold the wheels until they
            # answer. This one answers in a moment and leaves the rover driving,
            # so it goes out on the status connection like any other button --
            # and what the header toggle then shows is the rover's own
            # `exploring`, not a call this console is waiting for.
            #
            # No arguments: the budget is the rover's default, because the one
            # thing a console is for is watching, and somebody watching can stop
            # it whenever they like. A box to type minutes into would be a
            # setting nobody has a reason to change with the STOP button in view.
            self.watch_call("explore")
        elif what == "tap":
            self.tap(action)
        elif what == "describe":
            self.watch_call("describe_surroundings")
        elif what == "map":
            self.map_settings(action)
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
        elif what == "world":
            self.world_act(action)

    def watch_call(self, name: str, arguments: dict[str, Any] | None = None) -> None:
        if self.watch is None:
            self.say(f"not connected, so {name} was not sent", "bad")
            return
        self.watch.submit(name, arguments)

    def move(self, name: str, arguments: dict[str, Any]) -> None:
        """A bounded move, one at a time.

        Refused here rather than sent and refused by the daemon. The daemon's answer
        would be `busy`, which is correct and tells you nothing, and it would arrive
        as the notice on a move that is running perfectly well, where it reads like
        that move having failed.
        """
        if self.moves is None or not self.can_drive:
            self.say(f"no driving tools on this rover, so {name} was not sent", "bad")
            return
        if self.busy_since is not None:
            self.say(f"{self.busy_name} is still running; stop it or wait", "quiet")
            return
        self.busy_since = time.monotonic()
        self.busy_name = name
        self.moves.submit(name, arguments)

    def tap(self, action: dict[str, Any]) -> None:
        """A click on the picture is a place in the room, not a pixel -- and it
        outranks whatever the rover is doing when it lands.

        The page sends the pixel in the picture's own coordinates -- it divides out
        whatever CSS scaling the panel applied, which is the one piece of arithmetic
        it does -- and the conversion into metres happens here, in the renderer's own
        code. A browser that worked that out for itself would be a third copy of the
        map's geometry.

        **The place is asked for in map coordinates rather than as an offset from
        the rover, and that is what makes interrupting possible at all.** An offset
        is measured from wherever the rover has got to when the call arrives, and a
        click that interrupts a move arrives late by construction: the running move
        has to be stopped first, and the rover keeps driving until the stop lands.
        Sent as an offset, the click would mean a place most of a metre from the one
        under the cursor, and further out the faster the rover was going. Sent as a
        point on the map it means the same place however late it arrives.

        A click while something is already running is therefore not refused. It
        stops what is running and takes its place, because somebody clicking a
        second time is saying the rover is going to the wrong place, and "stop it or
        wait" is a console arguing with the only instruction it has.
        """
        if self.map_view is None:
            return
        if "drive_to" not in self.tools:
            self.say("this rover has no drive_to tool, so the tap was not sent", "quiet")
            return
        where = tap_to_point(_number(action.get("col"), 0.0),
                             _number(action.get("row"), 0.0), self.map_view)
        if where is None:
            self.say("cannot convert a tap without mapimg", "bad")
            return
        x_m, y_m = where
        arguments: dict[str, Any] = {"x_m": round(x_m, 2), "y_m": round(y_m, 2)}
        speed = _number(action.get("speed_ms"), None)
        if speed is not None:
            arguments["speed_ms"] = speed
        if self.busy_since is None:
            self.move("drive_to", arguments)
            return
        # Something is running. Stop it, and hold this until the wheels are free:
        # the running call occupies the move connection and cannot be overtaken on
        # it, and the daemon would refuse a second move as "busy" in any case. The
        # stop goes out on the connection that carries nothing else, and the move it
        # cancels answers within a control cycle of it landing, which is where the
        # waiting target is picked up. See `handle`.
        replacing = self.pending_target is not None
        self.pending_target = arguments
        self.pending_until = time.monotonic() + TARGET_HANDOVER_S
        if replacing:
            # The stop from the first click is already in flight, so a second one
            # would only say the same thing again.
            self.say(f"{self.new_target()} instead", "note")
            return
        self.say(f"{self.new_target()}, so the {self.busy_name} in flight is being "
                 f"stopped first", "note")
        self.stop(keep_target=True)

    def new_target(self) -> str:
        """The waiting click as a phrase, for the notice line."""
        target = self.pending_target or {}
        return ("a new target at x {:+.2f}, y {:+.2f}".format(
            float(target.get("x_m") or 0.0), float(target.get("y_m") or 0.0)))

    def hand_over(self) -> None:
        """Send the click that was waiting for the wheels, now that they are free."""
        arguments, self.pending_target = self.pending_target, None
        self.pending_until = 0.0
        if arguments is not None:
            self.move("drive_to", arguments)

    def forget_target(self, why: str) -> None:
        """Drop a waiting click, and say so. Silence is the bad outcome here: a
        click that quietly evaporated looks exactly like a console that ignores
        clicks."""
        if self.pending_target is None:
            return
        self.say(f"{self.new_target()} was dropped: {why}", "quiet")
        self.pending_target = None
        self.pending_until = 0.0

    def stop(self, keep_target: bool = False) -> None:
        """Always allowed, and on the connection that carries nothing else.

        A stop throws away a waiting click along with the move in flight, unless it
        *is* that click's own stop. Pressing STOP after clicking somewhere and then
        watching the rover set off for that place is the one behaviour nobody would
        forgive -- and the same goes for the stop that follows the last browser
        leaving, where the target was queued by a tab that has since been closed.
        """
        if not keep_target:
            self.forget_target("the rover was stopped")
        if self.halt is None:
            self.say("not connected, so there was nothing to stop", "quiet")
            return
        self.halt.submit("stop_driving")

    def take_picture(self) -> None:
        """On its own connection, because it is the slowest call here: a camera that
        has to be opened takes the rover up to four seconds to deliver a first
        buffer, and while it is doing that nothing else on that socket is answered."""
        if self.camera is None or self.frame_outstanding:
            return
        self.frame_outstanding = True
        self.frame_asked_at = time.monotonic()
        # The note is left saying what the last picture was. Replacing it with
        # "taking one..." for the second each capture takes was a change of state
        # twice a picture, and the panel it wrote to already has the picture on it.
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
                 "the face detector go with it for a few seconds", "note")
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
            self.say("not connected, so the map was not cleared", "bad")
            return
        # On the map's connection, so that a picture already being drawn comes back
        # before the clear rather than after it. The other way round shows an empty
        # map and then replaces it with the old one, which reads as the clear having
        # failed.
        self.picture.submit("clear_map")

    def map_settings(self, action: dict[str, Any]) -> None:
        """Zoom and which way is up, from the two controls left under the map.

        An extent, never a magnification: the picture is always the same number of
        pixels and the rover derives pixels per cell from the extent, which is what
        keeps the picture the same size when the view widens. Asking for a
        magnification instead resized the picture on every zoom, which is not
        zooming.
        """
        if "zoom" in action:
            index = rung(MAP_EXTENTS_M, self.half_extent) + int(action["zoom"])
            self.half_extent = MAP_EXTENTS_M[
                max(0, min(len(MAP_EXTENTS_M) - 1, index))]
        if "rover_up" in action:
            self.rover_up = bool(action["rover_up"])
        self.refresh_map()

    def refresh_map(self) -> None:
        """On its own connection, because the map is the slowest thing here.

        It shared the status connection at first, which was wrong once the cost was
        measured: a map at the default settings takes a couple of seconds on the rover,
        and `RoverClient` serialises, so every refresh held up a status poll that is
        meant to arrive three times a second. The numbers went stale exactly while
        the picture was being drawn.
        """
        if self.picture is None:
            return
        if self.map_outstanding:
            # One at a time, but do not lose the request: the map takes seconds, and
            # a zoom pressed while one is in flight would otherwise be dropped on
            # the floor and the next picture would come back at the old extent.
            # Remember that the settings moved and ask again as soon as this lands.
            self.map_wanted = True
            return
        self.map_wanted = False
        self.map_outstanding = True
        self.map_asked_at = time.monotonic()
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
            self.say("not connected, so no scan was sent", "bad")
            return
        self.wifi["note"] = "scanning -- the rover is off channel for a few seconds"
        self.wifi["scanning"] = True
        self.wifi_outstanding = True
        self.wifi_at = time.monotonic()
        self.scanner.submit("wifi_status", {"scan": True})

    def wifi_join(self, ssid: str) -> None:
        """Move the rover onto another network.

        The rover has one radio, so this costs the link. The daemon answers
        before it acts -- the reply arriving means the request was accepted and
        nothing more -- and what follows is the link going down under all six of
        this page's connections. The reconnect is therefore scheduled rather
        than waited for, because there is nothing left to be told on.
        """
        if not ssid or self.watch is None:
            return
        self.wifi["joining"] = ssid
        self.wifi_joining = ssid
        self.wifi["note"] = (f"joining {ssid}; the rover will be unreachable "
                             f"for a few seconds")
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
                    self.say(f"the rover answered again on {body['address']}", "good")
                self.said_lost = False
                self.find_tries = 0
                self.connected(body["address"])
                return
            self.find_tries += 1
            self.link_text = (f"no daemon answered; looking again every "
                              f"{self.retry_in():.0f} s")
            # Once, however long the outage lasts. The link line is what says this
            # is still trying, and saying it per try would bury everything else.
            if not self.said_lost:
                self.said_lost = True
                self.say("no rover daemon answered. Is it running, and is the "
                         "address right? This will keep looking.", "bad")
            return
        if name == "nav_status":
            self.poll_outstanding = False
            self.show_status(body)
            return
        if name == "map_png":
            self.map_outstanding = False
            self.map_done_at = time.monotonic()
            self.show_map(body)
            if self.map_wanted:
                self.refresh_map()
            return
        if name == "list_tools":
            self.show_tools(body)
            return
        if name == "camera_jpeg":
            self.frame_outstanding = False
            self.frame_done_at = time.monotonic()
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
            # The note has been saying a scan is in flight for several seconds, so
            # it is also what says how it went -- until it did, a panel went on
            # claiming to be scanning for the rest of the session.
            if reply.arguments.get("scan"):
                self.wifi["scanning"] = False
                if body.get("ok"):
                    heard = len(body.get("networks") or [])
                    self.wifi["note"] = (
                        f"heard {heard} network{'' if heard == 1 else 's'} "
                        f"in {reply.seconds:.0f} s")
                    extra = body.get("note")
                    if isinstance(extra, str) and extra.strip():
                        # The daemon already knows why a scan came back as one
                        # row in no time -- helper missing, no NetworkManager --
                        # and the panel used to throw that sentence away.
                        self.wifi["note"] += f" -- {extra.strip()}"
                else:
                    self.wifi["note"] = (f"the scan did not come back: "
                                         f"{body.get('error', 'no answer')}")
            return
        if name.startswith("world_"):
            # Its own connection, its own panel and its own errors, so none of
            # this goes near the notice line: a popup that is open shows what
            # happened, and one that is shut has nothing to say.
            self.world_handle(name, body, reply.seconds)
            return
        if name == "tracking_status":
            # Polled by the console rather than asked for by a person, so it updates
            # the panel and says nothing.
            self.track_outstanding = False
            self.show_tracking(body)
            return
        if name in ("get_lights", "set_lights"):
            if body.get("ok"):
                self.light_level = body.get("level")
            if name == "get_lights":
                return          # asked for on connect, not by a person

        # What is left is something a person asked for, so it gets the notice line
        # if there is anything about it a panel does not already show.
        moved = name in ("drive", "turn_in_place", "drive_to")
        if moved:
            self.busy_since = None
            self.busy_name = ""
        self.show_outcome(reply)
        if name in ("start_tracking", "stop_tracking") and body.get("ok"):
            self.show_tracking(body)
        if moved or name == "clear_map":
            self.refresh_map()
        if name == "clear_map" and body.get("ok"):
            # The semantic world is deliberately not cleared with the map -- an
            # entity outlives the map it was seen under -- but anything positional
            # recorded against the old map has to stay recognisable as belonging to
            # a map that no longer exists, so the store starts a new session.
            self.world_map_cleared()
        # The move that was in flight has answered, so the wheels are free and a
        # click that was waiting for them goes now. After that move's own outcome
        # has been said, so the notice line reads as one thing ending and the next
        # beginning rather than the new move claiming the old one's result.
        if moved and self.pending_target is not None:
            self.hand_over()

