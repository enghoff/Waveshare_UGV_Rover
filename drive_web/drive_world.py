"""The console's side of the semantic world state: one slow connection, one popup.

Almost everything here follows from one measurement taken on the rover: an
inspection is about a minute. That is far too long to share a connection with
anything else -- the status poll behind it would stall for a minute, taking the
lights, the tracking panel and the map with it, and a STOP queued behind it would
be a stop button that did not work. So the world gets its own channel with its own
patience, exactly as the wi-fi scan does and for the same reason, and the console
goes on being a console throughout.

The world state itself is fetched rather than pushed, like the network list and the
pictures: it is tens of kilobytes against a state that goes out ten times a second.
What rides in the state is a handful of counts and a generation tag; the body is
served from `/world.json` when that tag moves.

**Nobody presses refresh any more, and the tag is what made that affordable.**
The rover records a look a second and settles identities every ten, so a popup
that only changed when it was asked to was a still photograph of a store that had
moved on -- and the person watching had no way to tell those apart. While the
popup is open the pump asks the rover for the counts every `WORLD_OPEN_POLL_S`,
the body is asked for only once those have moved, and the tag only moves when
something in the body is genuinely different. A rover that has recorded nothing
since the last look therefore costs 7 kB every two seconds and no redraw at all;
one that is looking costs the body, once per change, which is what the person is
there to see.

**The pictures do not go through that channel at all, and they used to.** The
popup asked the rover ahead of time for the frames it thought would be wanted --
one per entity, the newest four of whichever entity was open -- and told the page
which ones it held, so an observation whose frame had not been asked for drew the
words "not fetched" instead of a picture. Every observation in the stream read
that way, which is every row a person opens the panel to look at, and the rover
now records a look a second so there is no fixed handful to guess at any more.
So `/world_frame.jpg` fetches on a miss instead: the page asks for the picture it
is about to draw, the browser asks only for the ones on screen, and each is
fetched once because a stored frame never changes under its name.

**The observations older than the newest forty come the same way.** The body
carries a window rather than the history because the body is re-sent every time
the rover records; the stream in the popup is not capped at that window all the
same. `/world_observations.json` hands back a page of whatever is below the
oldest row the browser already has, so scrolling to the bottom of the tiles walks
back through the store a page at a time, and a day of looking is something a
person can reach the end of.
"""
from __future__ import annotations

import base64
import math
import threading
import time
from typing import Any

import _paths  # noqa: F401 -- console_model
import rover_tools
from console_model import MAP_EXTENTS_M
from drive_show import _png_width

#: How many stored frames the console keeps in memory for the popup to draw.
#: Bounded because these are the rover's own JPEGs and this process is on the
#: rover: a session that fetched every frame of a long experiment would hold the
#: whole experiment in RAM beside SLAM. At the 28 kB a frame averages, this is
#: about 1.3 MB.
FRAME_CACHE = 48
#: How long to wait for the rover to hand over one stored frame. It is a file
#: read on the same machine, so this is a bound on a fault rather than on the
#: work: the browser is holding a connection open on the answer, and a frame that
#: is not coming should become a missing picture rather than a hung tab.
FRAME_TIMEOUT_S = 8.0
#: How many observations one page of the stream carries. The same forty the
#: payload's newest window holds, so that one scroll of the tiles is one ask.
STREAM_PAGE = 40
#: How big a picture the popup draws its map on, against the 480 px the console
#: asks for to drive by. This is the one panel that closes in: it shows a metre
#: or two of a room that can be twenty-four metres across, and a picture sized
#: for the drive card would be four cells wide by the time it got there. Pixels
#: per cell is derived by the daemon from this and the extent, so a wider view
#: comes back as more room at the same picture size rather than a bigger file --
#: measured on the Orin, the widest one is 962 px, 16 kB and 0.45 s.
WORLD_MAP_PX = 960
#: How often to draw it again while the popup is open, and never when it is
#: shut. Slower than the drive map's five seconds because nothing here is a
#: driving decision: what the picture sits under is a store of looks taken over
#: minutes, and the map itself only grows when the rover goes somewhere new.
WORLD_MAP_GAP_S = 10.0
#: Room to leave round the outermost thing the popup might draw. The extent is
#: worked out from where the rover was when the *last* picture was drawn, so
#: this is what covers a rover that has rolled a little since -- and it keeps a
#: thing sitting exactly on the edge off the edge.
WORLD_MAP_MARGIN_M = 0.75


def _metres(value: Any, fallback: float | None = None) -> float | None:
    """A coordinate out of the rover's JSON, or the fallback.

    Every number the popup's map is sized from crossed a socket, so nothing here
    may assume one is present or is a number: a single missing `x_m` must cost
    that one mark and not the whole picture.
    """
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback


class SessionWorld:
    """World-state actions, replies and payload, mixed into Session."""

    def world_reset(self) -> None:
        """The fields, in one place so the constructor and a reconnect agree."""
        #: What rides in the pushed state: small, and enough for the button.
        self.world: dict[str, Any] = {
            "available": None,          # None until the rover has been asked
            "open": False,
            "busy": False,
            "note": "",
            "error": "",
            "entities": 0,
            "observations": 0,
            "backend": "",
            "searching": False,
            # Which thing the rover is being sent to look at, so the row that
            # asked for it says so while it happens. Empty the moment any move
            # ends, because from then on the popup is not what is steering.
            "going": "",
            "gen": 0,
        }
        #: The destination that went with it, kept so that an outcome can be
        #: matched to the move it belongs to: a move this one interrupted answers
        #: first, and its verdict is not this one's.
        self.world_target: dict[str, Any] | None = None
        #: What `/world.json` serves: everything the popup draws.
        self.world_payload: dict[str, Any] = {}
        #: Frames by identifier, so `/world_frame.jpg` can answer without going
        #: back to the rover for a picture it already has.
        self.world_frames: dict[str, bytes] = {}
        #: One connection for the two things the page fetches for itself -- the
        #: pictures, and the older pages of the observation stream -- and one
        #: fetch at a time over it. Its own because the world channel carries
        #: the inspections, which a browser waiting on a picture must not queue
        #: behind.
        self._aside_lock = threading.Lock()
        self._aside_client = None
        self.world_selected = ""
        #: The phrase the search box last sent, kept so that an answer arriving
        #: after somebody has typed something else can be recognised as stale.
        self.world_query = ""
        #: When that phrase went out, or None when nothing is in flight. The
        #: search waits on the text tower, which is seconds rather than
        #: milliseconds, so the pane counts them off rather than sitting still.
        self.world_search_since: float | None = None
        self.world_outstanding = 0
        self.world_asked_at = 0.0
        #: When the counts were last asked for while the popup was open, and
        #: whether the answer now coming back is that ask. See `world_watch`,
        #: which is what keeps an open popup current.
        self.world_watched_at = 0.0
        self.world_watching = False
        #: The popup's own picture of the map: wide enough to hold every bearing
        #: and every settled position it draws, which the console's driving map
        #: is not. See `world_map_extent`.
        self.world_map_png: bytes = b""
        self.world_map_gen = 0
        self.world_map_view: dict[str, Any] | None = None
        self.world_map_shape = (0, 0)
        #: What was asked for and what came back, which are not always the same
        #: number: the daemon has a ceiling of its own. The picture is drawn
        #: again when the room the popup needs stops matching what it asked for,
        #: so it is the request that is remembered rather than the answer.
        self.world_map_asked = 0.0
        self.world_map_outstanding = False
        self.world_map_done_at = 0.0

    # --- what the buttons ask for --------------------------------------------

    def world_act(self, action: dict[str, Any]) -> None:
        """One posted world action, on the pump thread like every other."""
        what = str(action.get("what") or "")
        if what == "open":
            self.world["open"] = True
            self.world_refresh()
        elif what == "close":
            self.world["open"] = False
        elif what == "select":
            self.world_select(str(action.get("id") or ""))
        elif what == "inspect":
            self.world_inspect()
        elif what == "approach":
            self.world_approach(str(action.get("id") or ""))
        elif what == "search":
            self.world_search(str(action.get("query") or ""))

    def world_call(self, name: str, arguments: dict[str, Any] | None = None) -> None:
        if self.world_link is None:
            self.world["error"] = f"not connected, so {name} was not sent"
            return
        self.world_outstanding += 1
        self.world_link.submit(name, arguments)

    def world_refresh(self) -> None:
        """Everything the popup draws, asked for again.

        **There is no refresh button on the page any more, and this is
        why.** Opening the popup asks for the whole of it, and `world_watch`
        keeps it that way for as long as it is open; a button that asked again
        could only ever fetch what the console already had a moment ago. What it
        used to be good for besides was un-sticking a console that had decided
        the rover has no world state, and that is handled where it belongs --
        the asking slows down after a refusal rather than stopping.
        """
        self.world_call("world_state_entities")
        self.world_call("world_state_summary")
        if self.world_selected:
            self.world_call("world_state_entity", {"id": self.world_selected})

    def world_watch(self) -> None:
        """Ask what the rover has now, while somebody is looking at the popup.

        **This is why nobody has to press refresh any more.** The rover records
        a look a second and settles identities every ten, so a panel that only
        changed when it was asked to was a still photograph of a store that had
        moved on -- and the person watching it had no way to tell the two apart.
        The pump calls this every `WORLD_OPEN_POLL_S` for as long as the popup is
        open, and never when it is shut.

        What goes out is the counts alone. They are 7 kB and under 16 ms against
        the 74 kB and 50-95 ms the entity list costs, and the counts move whenever
        anything in the store does -- so `world_handle` asks for the body only
        once they have, and a rover that has recorded nothing since the last look
        costs nothing but the counts.
        """
        # Only this ask goes on to fetch the body. `world_refresh` asks for the
        # counts too, having already asked for the body beside them, and a rover
        # that recorded something between those two calls -- which at a look a
        # second is most of them -- would otherwise fetch it twice.
        self.world_watching = True
        self.world_call("world_state_summary")

    def world_select(self, entity_id: str) -> None:
        self.world_selected = entity_id
        if entity_id:
            self.world_call("world_state_entity", {"id": entity_id})
        elif self.world_payload.pop("selected", None) is not None:
            self.world_payload.pop("selected_observations", None)
            self.world_payload.pop("selected_rays", None)
            self.world_bump()

    def world_inspect(self) -> None:
        """Take a picture and ask the model about it.

        Nothing is disabled while it runs and nothing else waits for it. The button
        goes into a waiting state from the rover's own `busy` rather than from a
        call this console has in flight, for the reason the explore toggle is drawn
        that way: a second browser or a later version of this could start one too,
        and a button lit from "we are waiting" would disagree with the room.
        """
        if self.world_link is None:
            self.world["error"] = "not connected"
            return
        if self.world["busy"]:
            self.world["note"] = "an inspection is already running"
            return
        self.world["busy"] = True
        self.world["error"] = ""
        self.world["note"] = "looking..."
        self.world_asked_at = time.monotonic()
        self.world_call("world_inspect")

    def world_approach(self, entity_id: str) -> None:
        """Go and look at that thing.

        Two calls and not one, on two connections, because they are two different
        kinds of thing. Where to stand is arithmetic over the map and answers in
        milliseconds, so it goes out on the world channel with the rest of the
        popup's questions; the drive that follows lasts minutes and belongs on
        the move connection, under the same STOP and the same "a new destination
        outranks what is running" that a click on the map has. What joins them is
        `world_handle`, where the answer arrives and becomes a destination.

        **The rover is not asked twice.** The point comes back with the entity's
        name on it and is driven to as it stands; nothing here recomputes it from
        a position the console holds, because the console's copy of the world is
        as old as the last body it fetched and the map underneath it moves.
        """
        if self.world_link is None:
            self.world["error"] = "not connected"
            return
        if not self.can_drive or "drive_to" not in self.tools:
            self.world["error"] = ("this rover has no driving tools, so there "
                                   "is nowhere to send it")
            return
        self.world["going"] = entity_id
        # Whatever the last press was going to, this is not it. Held onto, the
        # move now being stopped would answer against the new thing's name and
        # write its verdict under a row the rover was never driving to.
        self.world_target = None
        self.world["error"] = ""
        self.world["note"] = f"working out where to stand to see {entity_id}..."
        self.world_call("world_state_viewpoint", {"id": entity_id})

    def world_going(self, body: dict[str, Any]) -> None:
        """The rover has said where to stand, so go there.

        Through `head_for`, which is the tail of a click on the map: it stops
        whatever is running, holds the destination until the wheels are free and
        drops it if they never come. A viewpoint is a place on the map like any
        other and there is no reason for it to reach the navigator by a second
        road.

        The one thing it carries that a click does not is which way to be facing
        on arrival. A click is somewhere to be; this is somewhere to look *from*,
        and a rover that parked in exactly the right spot with its back to the
        thing would have done everything asked of it and nothing wanted.
        """
        going = str(body.get("id") or "")
        self.world_target = {"x_m": body["x_m"], "y_m": body["y_m"],
                             "heading_deg": body["heading_deg"]}
        self.world["going"] = going
        self.world["note"] = (
            f"{going} is {body['range_m']:.1f} m from where the rover is being "
            f"sent, {body['travel_m']:.1f} m away")
        self.head_for(dict(self.world_target))

    def world_arrived(self, arguments: dict[str, Any],
                      body: dict[str, Any]) -> None:
        """What became of a move the popup started, in the popup's own note.

        The notice line under the console header says this too, and says it for
        every move however it was started. But it is behind the popup: somebody
        who pressed "go to" is looking at the entity list over a page they cannot
        read, and a drive that ended against a doorway would otherwise be a rover
        that silently stopped.

        Only the popup's own destination gets an outcome written under it. A move
        it interrupted answers first -- that is how the wheels come free -- and
        reporting "stopped" under the thing the rover is about to set off for
        would be the popup claiming a verdict on somebody else's drive.

        Another move ending does take the row's flag off, but only once this
        destination is no longer waiting to be sent: a click on the map that
        replaced it, or a `drive` somebody pressed instead. While it is still
        queued the rover really is on its way, and `hand_over` is about to send
        it.
        """
        going = self.world.get("going") or ""
        if not going:
            return
        if arguments != self.world_target:
            if self.pending_target != self.world_target:
                self.world["going"] = ""
            return
        self.world["going"] = ""
        self.world_target = None
        if not body.get("ok"):
            self.world["note"] = ""
            self.world["error"] = (f"the drive to {going} failed: "
                                   f"{body.get('error') or body.get('reason')}")
            return
        detail = body.get("detail")
        self.world["note"] = (f"{going}: {body.get('reason') or 'done'}"
                              + (f" -- {detail}" if detail else ""))

    def world_dropped(self, arguments: dict[str, Any], why: str) -> None:
        """The destination never got the wheels, so the row stops claiming it will.

        A waiting destination is thrown away when the move it interrupted does not
        let go -- see `mind_the_target` -- and silence there is the bad outcome:
        a row that says the rover is on its way to a thing it gave up on an hour
        ago is worse than no row at all.
        """
        going, self.world["going"] = self.world.get("going") or "", ""
        if not going or arguments != self.world_target:
            return
        self.world_target = None
        self.world["note"] = ""
        self.world["error"] = f"the drive to {going} was dropped: {why}"

    def world_search(self, query: str) -> None:
        """Find me the thing I described.

        The answer is what the popup narrows its entity list, its map and its
        observation stream down to, so the phrase is a filter and not a fourth
        view of the store. An empty one takes the filter off, which is why it is
        handled here rather than refused: nothing goes to the rover for it.

        Still not instant, and that is why the box says it is asking: the phrase
        has to go through the same text tower that named every region. Since
        2026-09-04 the rover keeps that model open, so the first search after a
        start-up is a couple of seconds and the rest are a fraction of one --
        short enough that the pulse is now a flicker, and long enough that
        without it a slow one would read as a box that swallowed the phrase.
        """
        query = query.strip()
        if self.world_link is None:
            self.world["error"] = "not connected"
            return
        if not query:
            self.world_query = ""
            self.world_payload.pop("search", None)
            self.world_bump()
            return
        self.world_query = query
        self.world["searching"] = True
        self.world_search_since = time.monotonic()
        self.world["error"] = ""
        # Enough looks that a thing the rover has seen a dozen times cannot fill
        # the answer by itself and hide every other candidate, which matters
        # more now that this list is the filtered view rather than a ranking
        # read on its own. The rover's own ceiling is 50.
        self.world_call("world_state_search", {"query": query, "limit": 24})

    def world_found(self, body: dict[str, Any]) -> None:
        """The best-scoring thing the search turned up, chosen without a click.

        A search is somebody asking where one thing is, and the answer to that is
        one thing. Left unselected, the phrase narrowed three views and then made
        the person pick the top row out of them by hand before the map would draw
        its sightings or "go to" would have anything to send the rover to -- a
        click that had no decision in it, since the list is already in the rover's
        own order and the top of it is the answer.

        **The first match with a thing behind it, which is not always the first
        match.** The ranking is over looks and a look need not belong to anything
        yet: the ordinary state of something seen once is a row with no entity,
        and choosing nothing because the best look was one of those would leave
        the second-best -- a real, placed thing -- unselected on screen.

        The verdict is deliberately not consulted. Below the floor the answer is
        "nothing here matches", the list says so on every row and the line under
        the box says it in words; what is selected is still the nearest thing the
        rover has, and hiding it would leave a person who wanted to see what it
        settled for with nothing to look at.
        """
        for match in body.get("matches") or []:
            entity_id = str(match.get("entity_id") or "")
            if entity_id:
                if entity_id != self.world_selected:
                    self.world_select(entity_id)
                return
        # Nothing the phrase matched has been made a thing of. Whatever was
        # chosen before is not in the narrowed list, so leaving it selected would
        # be a detail pane describing something the list beside it no longer
        # shows.
        self.world_select("")

    def world_map_cleared(self) -> None:
        """The SLAM map was thrown away, so the world state goes with it.

        **It used to survive**, on the argument that an entity outlives the map it
        was seen under: only the session stamp moved, so that positions measured
        against a map that no longer exists were visible as such rather than
        compared. That is defensible and it is not what happens. Everything the
        world state holds is a position or a bearing measured in the old map's
        frame, so what survived a clear was a list of things with nowhere to be,
        and in practice the two were always cleared together -- the second press
        being a separate button was friction, not a safeguard.

        So there is one button now, and it is the map's. The session is still
        started afresh afterwards, because a clear that half fails must not leave
        old coordinates comparable with new ones.
        """
        if self.world_link is not None and self.world.get("available"):
            self.world_frames.clear()
            self.world_selected = ""
            self.world_call("world_state_clear")
            self.world_call("world_map_session")

    # --- the map it draws on ---------------------------------------------------

    def world_map_extent(self) -> float | None:
        """How far each way the popup's map has to reach, or None to ask for none.

        **The driving map is the wrong picture for this panel, and that is what
        this exists to fix.** The card behind the popup is drawn a few metres
        around wherever the rover is standing, because that is what driving
        needs. The popup draws bearings taken from all over a flat, and against
        that picture a thing perfectly well placed six metres away sat on black
        with "outside the drawn map" written underneath it -- which was true, and
        was not the reader's fault. So the popup asks for a map of its own, wide
        enough to hold exactly what it is about to draw on it.

        That is the chosen thing's own looks while a thing is chosen, since the
        panel draws nothing else then and a room-wide picture would throw away
        the resolution that makes the fork between a bearing and a position
        readable; and every thing in the store while nothing is chosen, which is
        the overview. The answer is a rung off the console's own zoom ladder
        rather than the exact number, so a rover shuffling about on the spot does
        not buy a new picture every time the popup polls.

        None where there is nothing to cover, or nowhere to measure it from. The
        popup then goes on drawing over the driving map, which is what it did
        before this existed.
        """
        pose = (self.map_view or {}).get("pose") or {}
        at_x, at_y = _metres(pose.get("x_m")), _metres(pose.get("y_m"))
        if at_x is None or at_y is None:
            return None
        reach = 0.0
        for x, y in self.world_marks():
            reach = max(reach, abs(x - at_x), abs(y - at_y))
        if not reach:
            return None
        reach += WORLD_MAP_MARGIN_M
        return next((rung for rung in MAP_EXTENTS_M if rung >= reach),
                    MAP_EXTENTS_M[-1])

    def world_marks(self):
        """Every point the popup's map draws, in the map's own metres.

        The same three the page puts through `wPointToPx`: where each look was
        taken from, how far along its bearing the line actually runs, and the
        ring round the position the thing was settled at. Deliberately the same
        marks `wWindow` in drive_world.js gathers, because the two answer one
        question about one set of points -- how much room is needed -- and a
        picture sized from fewer of them than are drawn on it is the fault this
        is here to prevent.
        """
        payload = self.world_payload
        chosen = payload.get("selected") or {}
        if self.world_selected and chosen.get("id") == self.world_selected:
            # The chosen thing's own reply carries more of its looks than the
            # entity list does, and the drawing prefers it for the same reason.
            entities = [dict(chosen, rays=(payload.get("selected_rays")
                                           or chosen.get("rays") or []))]
        else:
            entities = payload.get("entities") or []
        for entity in entities:
            place = entity.get("placement") or {}
            px, py = _metres(place.get("x_m")), _metres(place.get("y_m"))
            if px is not None and py is not None:
                # The wider of the two rings drawn round a settled position: how
                # unsure the crossing was, and how wide the thing itself is.
                ring = max(_metres(place.get("error_major_m"), 0.0),
                           _metres(place.get("extent_m"), 0.0), 0.25)
                yield px - ring, py - ring
                yield px + ring, py + ring
            for ray in entity.get("rays") or []:
                ox, oy = _metres(ray.get("x_m")), _metres(ray.get("y_m"))
                if ox is None or oy is None:
                    continue
                yield ox, oy
                # As far out as the line is really drawn -- to the thing where
                # there is one, and to the length the look itself claims where
                # there is not. `wReach`, in Python.
                relation = ray.get("relation") or {}
                reach = (max(0.3, _metres(relation.get("range_m"), 0.0))
                         if px is not None and relation
                         else _metres(ray.get("length_m"), 2.5))
                bearing = math.radians(_metres(ray.get("bearing_deg"), 0.0))
                yield ox + reach * math.cos(bearing), oy + reach * math.sin(bearing)

    def world_map_due(self, now: float) -> float | None:
        """The extent to ask for now, or None to leave the picture alone.

        Two reasons to draw it again and no others: the room the popup needs has
        stopped matching the room the picture covers -- somebody chose a
        different thing, or the rover placed one further out than the edge -- or
        the picture has simply grown old while somebody watched it. Both only
        while the popup is open, because a shut popup draws nothing and this is
        the most expensive thing the console asks the rover for.
        """
        if (self.picture is None or not self.world["open"]
                or self.world_map_outstanding):
            return None
        half = self.world_map_extent()
        if half is None:
            return None
        if half != self.world_map_asked:
            return half
        return half if now - self.world_map_done_at > WORLD_MAP_GAP_S else None

    def world_map_refresh(self, half: float) -> None:
        """Ask for it, on the connection the driving map already uses.

        The same one deliberately. These are the two most expensive pictures this
        console asks for, and there is no reason for the rover to be drawing both
        at once -- nor anybody to see the drive map while the popup is over it.
        Tagged, because both calls are `map_png` and the window has to know which
        of its two pictures has arrived.
        """
        self.world_map_outstanding = True
        self.world_map_asked = half
        self.picture.submit("map_png", {"half_extent_m": half,
                                        "pixels": WORLD_MAP_PX}, tag="world")

    def world_map_arrived(self, body: dict[str, Any]) -> None:
        """The popup's map, or a refusal that says nothing on the page.

        A picture that did not come back is deliberately not reported. The panel
        falls back to the driving map -- the picture it drew over before this
        existed -- and a red line about a render would be the popup complaining
        about its own backdrop rather than about the world it is there to show.
        """
        self.world_map_outstanding = False
        self.world_map_done_at = time.monotonic()
        if not body.get("ok"):
            return
        try:
            png = base64.b64decode(body["png_base64"])
        except (KeyError, ValueError):
            return
        if not png:
            return
        self.world_map_png = png
        self.world_map_gen += 1
        width = int(body.get("pixels") or 0) or _png_width(png)
        self.world_map_shape = (width, width)
        # What it was really drawn at, which is what lets the page put a bearing
        # on it: the extent and the pixels per cell it came out as, and the pose
        # it was centred on. The pose it was drawn from and not the pose now -- a
        # map is a photograph of a moment, and every mark laid over this one is
        # placed against the moment it was taken.
        self.world_map_view = {
            "half_extent_m": _metres(body.get("half_extent_m"),
                                     self.world_map_asked),
            "scale": int(body.get("scale") or 1),
            # Never turned. The popup has no such switch, and a map that swung
            # round under a panel about where things are would be unreadable.
            "rover_up": False,
            "pose": body.get("pose") or {},
        }

    # --- what comes back ------------------------------------------------------

    def world_handle(self, name: str, body: dict[str, Any], seconds: float) -> None:
        self.world_outstanding = max(0, self.world_outstanding - 1)
        # `world_watch` only asks with nothing else in flight, so whatever comes
        # back next is its answer -- and cleared here whatever that answer is,
        # so a refusal does not leave the next reply looking like a watch.
        watching, self.world_watching = self.world_watching, False
        if not body.get("ok"):
            error = str(body.get("error") or "no answer")
            if name in ("world_state_summary", "world_state_entities"):
                # This is also how the console finds out whether the rover has a
                # world-state component at all, and the panel says so rather than
                # showing an error over and over. It is not remembered for the
                # session, though: the popup goes on asking at `WORLD_RETRY_S`
                # while it is open, so a rover that has just been given the
                # component, or a store that was locked while the daemon
                # restarted, comes back on its own.
                self.world["available"] = False
            self.world["error"] = error
            if name == "world_inspect":
                self.world["busy"] = False
                self.world["note"] = ""
            if name == "world_state_search":
                self.world["searching"] = False
                self.world_search_since = None
            if name == "world_state_viewpoint":
                # The refusal is the answer here rather than a fault: a thing
                # with no position, a thing whose position belongs to a map that
                # has been cleared, and a thing there is nowhere to see it from
                # are all things the rover knows and the popup cannot work out
                # for itself.
                self.world["going"] = ""
                self.world["note"] = ""
            # No bump: everything said here rides in the pushed state, which is
            # compared whole on every tick. Moving the tag would send the browser
            # back for 74 kB it already has, and a poll every two seconds means a
            # rover that is refusing would do that every two seconds.
            return

        self.world["available"] = True
        self.world["error"] = ""
        moved = False
        if name == "world_state_summary":
            moved = self.world_put(
                summary=body.get("summary") or {},
                inferences=body.get("inferences") or [],
                backend=body.get("backend") or "",
                camera_fov_deg=body.get("camera_fov_deg"))
            self.world["backend"] = body.get("backend") or ""
            self.world["busy"] = bool(body.get("busy"))
            # The looking loop's own last complaint. There is no line for it on
            # the page any more, so this is the only place a rover that has
            # quietly stopped recording can say so, and it belongs on the error
            # line of the popup that is showing the store it has stopped filling.
            if body.get("building_error"):
                self.world["error"] = str(body["building_error"])
            self.world_counts()
            # The counts are how an open popup finds out there is anything new to
            # draw: they are what `world_watch` asks for every couple of seconds,
            # and they move whenever the store does. So the body is fetched here,
            # once, on the strength of them, rather than on a timer of its own.
            #
            # And the tag is left alone while that is in flight, or every change
            # in the store would fetch the payload twice: once for these counts
            # and again half a second later for the body they sent for. What the
            # header shows does not wait on it -- the counts ride in the pushed
            # state, which is compared and sent on its own.
            if moved and watching and self.world["open"]:
                moved = False
                self.world_call("world_state_entities")
                if self.world_selected:
                    self.world_call("world_state_entity",
                                    {"id": self.world_selected})
        elif name == "world_state_entities":
            moved = self.world_put(
                entities=body.get("entities") or [],
                summary=body.get("summary") or {},
                recent=body.get("recent") or [])
            self.world_counts()
        elif name == "world_state_entity":
            moved = self.world_put(
                selected=body.get("entity") or {},
                selected_observations=body.get("observations") or [],
                selected_rays=body.get("rays") or [])
        elif name == "world_state_search":
            self.world["searching"] = False
            self.world_search_since = None
            # Stale if the box has moved on since this was asked. Dropped rather
            # than drawn, because a five-second answer arriving under a different
            # phrase reads as the search having got it wrong.
            if str(body.get("query") or "") == self.world_query:
                moved = self.world_put(search=body)
                self.world_found(body)
        elif name == "world_state_viewpoint":
            # Where to stand has come back, so this becomes a destination on the
            # move connection -- through the same path a click on the map takes,
            # so that it stops what is running, waits for the wheels and is
            # dropped if they never come free, exactly as a click is.
            self.world_going(body)
        elif name == "world_state_clear":
            self.world["note"] = (
                f"cleared -- {body.get('entities', 0)} entities, "
                f"{body.get('observations', 0)} observations")
            moved = bool(self.world_payload)
            self.world_payload = {}
            self.world_refresh()
        elif name == "world_map_session":
            # Second half of the same button, and it has nothing of its own to
            # say: the clear above has already reported what went.
            self.world_refresh()
        elif name == "world_inspect":
            self.world["busy"] = False
            self.world["note"] = _inspection_note(body, seconds)
            self.world_refresh()
        if moved:
            self.world_bump()

    def world_put(self, **fields: Any) -> bool:
        """Hold these in the payload, and say whether any of them is new.

        The tag `/world.json` is published under only moves when the answer is
        yes, and that is what makes asking the rover every two seconds
        affordable. The body is 74 kB, the browser holds it under a URL that is
        served `immutable`, and it re-fetches the lot the moment that tag
        changes -- so a console that bumped on every reply would put 74 kB on the
        wi-fi every two seconds to redraw a panel that had not changed. It used
        to bump on every reply, which cost nothing only because nothing asked
        unless a person pressed the button.
        """
        moved = False
        for field, value in fields.items():
            if self.world_payload.get(field) != value:
                self.world_payload[field] = value
                moved = True
        return moved

    def world_counts(self) -> None:
        summary = self.world_payload.get("summary") or {}
        self.world["entities"] = summary.get("entities", 0)
        self.world["observations"] = summary.get("observations", 0)

    def world_frame(self, frame_id: str) -> bytes | None:
        """The stored picture behind one observation, fetched if it is not held.

        Called from whichever thread is serving `/world_frame.jpg`, so it goes to
        the rover on a connection of its own rather than through the world channel
        the popup's other calls queue on: an inspection on that channel can be
        half a minute, and a picture is a file read that must not wait behind one.
        A `RoverClient` serialises its own calls, so several pictures at once
        become several quick calls in a row rather than a race.

        None where the rover has no such frame -- which is an ordinary answer, not
        a fault: the row outlives the file whenever the world is cleared, and the
        popup exists to show what happened rather than to fall over on it.
        """
        held = self.world_frames.get(frame_id)
        if held is not None:
            return held
        if not frame_id or not self.address:
            return None
        with self._aside_lock:
            held = self.world_frames.get(frame_id)
            if held is not None:
                return held
            body = self._aside_call("world_state_frame", {"frame_id": frame_id})
            if not body.get("ok"):
                return None
            try:
                jpeg = base64.b64decode(body.get("jpeg_base64", ""))
            except ValueError:
                return None
            if not jpeg:
                return None
            self.world_frames[frame_id] = jpeg
            while len(self.world_frames) > FRAME_CACHE:
                # Oldest first. A dict preserves insertion order, which is the
                # order they were asked for, which is near enough to the order
                # they will stop being looked at.
                self.world_frames.pop(next(iter(self.world_frames)))
            return jpeg

    def world_observations(self, before: tuple[float, int] | None) -> dict[str, Any]:
        """One page of the observation stream, older than the row the page names.

        **The whole history is readable, and it does not ride in the payload.**
        That payload is re-sent whenever the rover records anything, which while
        it is looking is every second or so, so what goes in it is the newest
        forty and nothing else; everything before that is fetched here as
        somebody scrolls back, once each, and the browser keeps what it has been
        given. A store holding a day of looking is therefore a stream a person
        can reach the bottom of, at the cost of the pages they actually asked
        for.

        `before` is the oldest row the page already holds -- a place in the
        history rather than a count of rows to skip, because the rover records
        while it is being read. None starts at the newest.
        """
        if not self.address:
            return {"ok": False, "error": "not connected"}
        arguments: dict[str, Any] = {"limit": STREAM_PAGE}
        if before is not None:
            arguments["before_at"], arguments["before_id"] = before
        with self._aside_lock:
            return self._aside_call("world_state_observations", arguments)

    def _aside_call(self, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
        """One quick call to the rover, off the world channel. Lock held.

        Both callers are serving a browser that is holding a connection open, and
        both are a file read or an index lookup on the rover: neither may queue
        behind the inspection the world channel exists to carry. A
        `RoverClient` serialises its own calls anyway, so the lock here is what
        keeps two page threads from building two connections at once.
        """
        client = self._aside_client
        if client is None or client.describe() != self.address:
            if client is not None:
                client.close()
            client = rover_tools.RoverClient(self.address, timeout=FRAME_TIMEOUT_S)
            self._aside_client = client
        return client.call(name, arguments)

    def world_bump(self) -> None:
        """Say that `/world.json` has changed, so the page fetches it again.

        A counter rather than the body, for the reason the network list is served
        this way: the payload is tens of kilobytes and the state it would ride in
        goes out ten times a second.
        """
        self.world["gen"] = self.world.get("gen", 0) + 1

    def world_state(self) -> dict[str, Any]:
        """The part of the world that rides in every pushed state."""
        # Whole seconds, for the reason the move stopwatch is whole seconds: a
        # tenth would make this a new state ten times a second for as long as
        # the model was thinking, and nobody can read it at that speed anyway.
        asking = self.world_search_since
        return dict(self.world, gen=self.tag(self.world["gen"]),
                    selected=self.world_selected,
                    # The popup's own map, which is a picture and so rides here
                    # as a generation and the geometry to lay marks over it by --
                    # exactly as the driving map's does, and fetched from
                    # `/world_map.png` for the same reason. Free: this block is
                    # already a new block whenever there is a new picture to say
                    # it about.
                    map={"gen": self.tag(self.world_map_gen),
                         "width": self.world_map_shape[0],
                         "view": self.world_map_view},
                    searched_s=(0 if asking is None
                                else round(time.monotonic() - asking)))


def _inspection_note(body: dict[str, Any], seconds: float) -> str:
    """What one inspection did, in a line, for the popup's header.

    Written so that the four outcomes a person has to tell apart read
    differently: the model found new things, the model recognised things it had
    seen, the model answered but nothing came of it, and the picture was the one
    the rover already had -- which is not the same as an empty room, and would
    otherwise read as one.
    """
    if not body.get("ok"):
        return f"the inspection failed: {body.get('error', 'no answer')}"
    if body.get("unchanged"):
        return f"{body.get('detail', 'the same picture as the last look')}" \
               f" -- {seconds:.0f} s"
    parts = []
    created, matched = body.get("created", 0), body.get("matched", 0)
    if created:
        parts.append(f"{created} new")
    if matched:
        parts.append(f"{matched} recognised")
    if body.get("rejected"):
        parts.append(f"{body['rejected']} not stored")
    if not parts:
        parts.append("nothing worth recording in view")
    note = ", ".join(parts) + f" -- {seconds:.0f} s"
    detail = body.get("detail")
    if detail:
        note += f" ({detail})"
    return note
