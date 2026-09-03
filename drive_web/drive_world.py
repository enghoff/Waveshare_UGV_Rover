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
"""
from __future__ import annotations

import base64
import threading
import time
from typing import Any

import rover_tools

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
            #: None until the rover has been asked. The panel says "-" rather than
            #: claiming the rover is doing something it may not be.
            "building": None,
            "built_looks": 0,
            #: What the last resolver pass did. Looking and settling run on
            #: separate clocks now, so a rover recording steadily and placing
            #: nothing is a state the panel has to be able to show -- it is the
            #: one this whole change came out of.
            "settled": {},
            "gen": 0,
        }
        #: What `/world.json` serves: everything the popup draws.
        self.world_payload: dict[str, Any] = {}
        #: Frames by identifier, so `/world_frame.jpg` can answer without going
        #: back to the rover for a picture it already has.
        self.world_frames: dict[str, bytes] = {}
        #: One connection for pictures, and one fetch at a time over it. Its own
        #: because the world channel carries the inspections.
        self._frame_lock = threading.Lock()
        self._frame_client = None
        self.world_selected = ""
        #: The phrase the search box last sent, kept so that an answer arriving
        #: after somebody has typed something else can be recognised as stale.
        self.world_query = ""
        self.world_outstanding = 0
        self.world_asked_at = 0.0
        #: When the counts were last asked for while the popup was open, and
        #: whether the answer now coming back is that ask. See `world_watch`,
        #: which is what keeps an open popup current.
        self.world_watched_at = 0.0
        self.world_watching = False

    # --- what the buttons ask for --------------------------------------------

    def world_act(self, action: dict[str, Any]) -> None:
        """One posted world action, on the pump thread like every other."""
        what = str(action.get("what") or "")
        if what == "open":
            self.world["open"] = True
            self.world_refresh()
        elif what == "close":
            self.world["open"] = False
        elif what == "refresh":
            self.world_refresh()
        elif what == "select":
            self.world_select(str(action.get("id") or ""))
        elif what == "inspect":
            self.world_inspect()
        elif what == "search":
            self.world_search(str(action.get("query") or ""))
        elif what == "build":
            self.world_build(bool(action.get("on")))

    def world_call(self, name: str, arguments: dict[str, Any] | None = None) -> None:
        if self.world_link is None:
            self.world["error"] = f"not connected, so {name} was not sent"
            return
        self.world_outstanding += 1
        self.world_link.submit(name, arguments)

    def world_refresh(self) -> None:
        """Everything the popup draws, asked for again. What the button does."""
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

    def world_build(self, on: bool) -> None:
        """Turn the rover's own looking on or off.

        Sent on the status connection like the poll, so that pressing it while an
        inspection is running does not wait for that inspection to finish. The
        panel is not updated here: what it shows comes back from the rover, the
        same way the tracking panel works, because this console is not the only
        thing that can change it.
        """
        if self.watch is None:
            self.world["error"] = "not connected"
            return
        self.watch.submit("world_building", {"on": on})

    def world_search(self, query: str) -> None:
        """Find me the thing I described.

        Slower than it looks and worth saying so in the note: the phrase has to
        go through the same text tower that named every region, and on the rover
        that means loading a model for the call and giving it back afterwards,
        which is several seconds. A search is something a person types, so that
        is the right way round, but a box that looks frozen for five seconds is
        not.
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
        self.world["error"] = ""
        self.world_call("world_state_search", {"query": query, "limit": 12})

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

    # --- what comes back ------------------------------------------------------

    def world_handle(self, name: str, body: dict[str, Any], seconds: float) -> None:
        self.world_outstanding = max(0, self.world_outstanding - 1)
        # `world_watch` only asks with nothing else in flight, so whatever comes
        # back next is its answer -- and cleared here whatever that answer is,
        # so a refusal does not leave the next reply looking like a watch.
        watching, self.world_watching = self.world_watching, False
        if not body.get("ok"):
            error = str(body.get("error") or "no answer")
            if name == "world_building":
                self.world_build_outstanding = False
            if name in ("world_state_summary", "world_state_entities",
                        "world_building"):
                # The first ask is also how this console finds out whether the
                # rover has a world-state component at all. A daemon without one
                # says so once and the button stays away, rather than the popup
                # showing an error every few seconds for the rest of the session.
                self.world["available"] = False
            self.world["error"] = error
            if name == "world_inspect":
                self.world["busy"] = False
                self.world["note"] = ""
            if name == "world_state_search":
                self.world["searching"] = False
            # No bump: everything said here rides in the pushed state, which is
            # compared whole on every tick. Moving the tag would send the browser
            # back for 74 kB it already has, and a poll every two seconds means a
            # rover that is refusing would do that every two seconds.
            return

        self.world["available"] = True
        self.world["error"] = ""
        if name == "world_building":
            self.world_build_outstanding = False
            self.world["building"] = bool(body.get("building"))
            self.world["built_looks"] = body.get("looks") or 0
            self.world["settled"] = body.get("settled") or {}
            # The loop's own last complaint, which is the only place a rover that
            # has quietly stopped recording would ever say so.
            if body.get("error"):
                self.world["error"] = str(body["error"])
            return
        moved = False
        if name == "world_state_summary":
            moved = self.world_put(
                summary=body.get("summary") or {},
                inferences=body.get("inferences") or [],
                backend=body.get("backend") or "",
                camera_fov_deg=body.get("camera_fov_deg"))
            self.world["backend"] = body.get("backend") or ""
            self.world["busy"] = bool(body.get("busy"))
            self.world["settled"] = body.get("settled") or {}
            self.world_counts()
            # The counts are how an open popup finds out there is anything new to
            # draw: they are what `world_watch` asks for every couple of seconds,
            # and they move whenever the store does. So the body is fetched here,
            # once, on the strength of them, rather than on a timer of its own.
            if moved and watching and self.world["open"]:
                self.world_call("world_state_entities")
                if self.world_selected:
                    self.world_call("world_state_entity",
                                    {"id": self.world_selected})
        elif name == "world_state_entities":
            moved = self.world_put(
                entities=body.get("entities") or [],
                unmatched=body.get("unmatched") or [],
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
            # Stale if the box has moved on since this was asked. Dropped rather
            # than drawn, because a five-second answer arriving under a different
            # phrase reads as the search having got it wrong.
            if str(body.get("query") or "") == self.world_query:
                moved = self.world_put(search=body)
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
        with self._frame_lock:
            held = self.world_frames.get(frame_id)
            if held is not None:
                return held
            client = self._frame_client
            if client is None or client.describe() != self.address:
                if client is not None:
                    client.close()
                client = rover_tools.RoverClient(self.address,
                                                 timeout=FRAME_TIMEOUT_S)
                self._frame_client = client
            body = client.call("world_state_frame", {"frame_id": frame_id})
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

    def world_bump(self) -> None:
        """Say that `/world.json` has changed, so the page fetches it again.

        A counter rather than the body, for the reason the network list is served
        this way: the payload is tens of kilobytes and the state it would ride in
        goes out ten times a second.
        """
        self.world["gen"] = self.world.get("gen", 0) + 1

    def world_state(self) -> dict[str, Any]:
        """The part of the world that rides in every pushed state."""
        return dict(self.world, gen=self.tag(self.world["gen"]),
                    selected=self.world_selected)


def _inspection_note(body: dict[str, Any], seconds: float) -> str:
    """What one inspection did, in a line, for the popup's header.

    Written so that the three outcomes a person has to tell apart read
    differently: the model found new things, the model recognised things it had
    seen, and the model answered but nothing came of it.
    """
    if not body.get("ok"):
        return f"the inspection failed: {body.get('error', 'no answer')}"
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
