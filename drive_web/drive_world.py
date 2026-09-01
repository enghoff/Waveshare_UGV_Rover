"""The console's side of the semantic world state: one slow connection, one popup.

Almost everything here follows from one measurement taken on the rover: an
inspection is about a minute. That is far too long to share a connection with
anything else -- the status poll behind it would stall for a minute, taking the
lights, the tracking panel and the map with it, and a STOP queued behind it would
be a stop button that did not work. So the world gets its own channel with its own
patience, exactly as the wi-fi scan does and for the same reason, and the console
goes on being a console throughout.

The world state itself is fetched rather than pushed, like the network list and the
pictures: it is tens of kilobytes, it changes when somebody presses a button, and
the state on the event stream goes out many times a second. What rides in the state
is a handful of counts and a generation tag; the body is served from `/world.json`
when that tag moves.
"""
from __future__ import annotations

import base64
import time
from typing import Any

#: How many stored frames the console keeps in memory for the popup to draw.
#: Bounded because these are the rover's own JPEGs and this process is on the
#: rover: a session that fetched every frame of a long experiment would hold the
#: whole experiment in RAM beside SLAM.
FRAME_CACHE = 24
#: How many of a selected entity's frames to fetch. The newest few, because the
#: question the popup is asked is "is this the same thing as last time".
FRAMES_PER_ENTITY = 4


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
            "gen": 0,
        }
        #: What `/world.json` serves: everything the popup draws.
        self.world_payload: dict[str, Any] = {}
        #: Frames by identifier, so `/world_frame.jpg` can answer without going
        #: back to the rover for a picture it already has.
        self.world_frames: dict[str, bytes] = {}
        self.world_selected = ""
        self.world_outstanding = 0
        self.world_asked_at = 0.0
        self.world_clear_armed_until = 0.0

    # --- what the buttons ask for --------------------------------------------

    def world_act(self, action: dict[str, Any]) -> None:
        """One posted world action, on the pump thread like every other."""
        what = str(action.get("what") or "")
        if what == "open":
            self.world["open"] = True
            self.world_refresh()
        elif what == "close":
            self.world["open"] = False
            self.world_clear_armed_until = 0.0
        elif what == "refresh":
            self.world_refresh()
        elif what == "select":
            self.world_select(str(action.get("id") or ""))
        elif what == "inspect":
            self.world_inspect()
        elif what == "clear":
            self.world_clear()

    def world_call(self, name: str, arguments: dict[str, Any] | None = None) -> None:
        if self.world_link is None:
            self.world["error"] = f"not connected, so {name} was not sent"
            return
        self.world_outstanding += 1
        self.world_link.submit(name, arguments)

    def world_refresh(self) -> None:
        self.world_call("world_state_entities")
        self.world_call("world_state_summary")
        if self.world_selected:
            self.world_call("world_state_entity", {"id": self.world_selected})

    def world_select(self, entity_id: str) -> None:
        self.world_selected = entity_id
        if entity_id:
            self.world_call("world_state_entity", {"id": entity_id})
        else:
            self.world_payload.pop("selected", None)
            self.world_payload.pop("selected_observations", None)
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

    def world_clear(self) -> None:
        """Two presses, and no dialog between them.

        The same arming the map's clear uses, for the same reason: `confirm()`
        blocks the page's script, and the page's script is what holds the stop
        button. This one is separate from that one on purpose -- clearing the
        semantic world must not be reachable by somebody aiming at the map, and
        clearing the map must not take the semantic world with it.
        """
        now = time.monotonic()
        if now > self.world_clear_armed_until:
            self.world_clear_armed_until = now + 5.0
            self.world["note"] = ("press again to throw away every entity and "
                                  "observation; the map is not touched")
            return
        self.world_clear_armed_until = 0.0
        self.world_frames.clear()
        self.world_selected = ""
        self.world_call("world_state_clear")

    def world_map_cleared(self) -> None:
        """The SLAM map was thrown away, so tell the store to start a new session.

        The console owns that button, so it says so rather than the store polling
        for it. Nothing semantic is deleted: entities outlive maps, and only the
        stamp on new observations moves.
        """
        if self.world_link is not None and self.world.get("available"):
            self.world_call("world_map_session")

    # --- what comes back ------------------------------------------------------

    def world_handle(self, name: str, body: dict[str, Any], seconds: float) -> None:
        self.world_outstanding = max(0, self.world_outstanding - 1)
        if not body.get("ok"):
            error = str(body.get("error") or "no answer")
            if name in ("world_state_summary", "world_state_entities"):
                # The first ask is also how this console finds out whether the
                # rover has a world-state component at all. A daemon without one
                # says so once and the button stays away, rather than the popup
                # showing an error every few seconds for the rest of the session.
                self.world["available"] = False
            self.world["error"] = error
            if name == "world_inspect":
                self.world["busy"] = False
                self.world["note"] = ""
            self.world_bump()
            return

        self.world["available"] = True
        self.world["error"] = ""
        if name == "world_state_summary":
            self.world_payload["summary"] = body.get("summary") or {}
            self.world_payload["inferences"] = body.get("inferences") or []
            self.world_payload["backend"] = body.get("backend") or ""
            self.world_payload["camera_fov_deg"] = body.get("camera_fov_deg")
            self.world["backend"] = body.get("backend") or ""
            self.world["busy"] = bool(body.get("busy"))
            self.world_counts()
        elif name == "world_state_entities":
            self.world_payload["entities"] = body.get("entities") or []
            self.world_payload["unmatched"] = body.get("unmatched") or []
            self.world_payload["summary"] = body.get("summary") or {}
            self.world_payload["recent"] = body.get("recent") or []
            self.world_counts()
            self.world_want_frames(self.world_payload["entities"])
        elif name == "world_state_entity":
            self.world_payload["selected"] = body.get("entity") or {}
            self.world_payload["selected_observations"] = body.get("observations") or []
            self.world_payload["selected_rays"] = body.get("rays") or []
            self.world_want_frames_for(body.get("observations") or [],
                                       FRAMES_PER_ENTITY)
        elif name == "world_state_frame":
            self.world_keep_frame(body)
        elif name == "world_state_clear":
            self.world["note"] = (
                f"cleared {body.get('entities', 0)} entities and "
                f"{body.get('observations', 0)} observations; the map is untouched")
            self.world_payload = {}
            self.world_refresh()
        elif name == "world_map_session":
            self.world["note"] = (f"the map was cleared; new observations are "
                                  f"session {body.get('map_session')}")
            self.world_refresh()
        elif name == "world_inspect":
            self.world["busy"] = False
            self.world["note"] = _inspection_note(body, seconds)
            self.world_refresh()
        self.world_bump()

    def world_counts(self) -> None:
        summary = self.world_payload.get("summary") or {}
        self.world["entities"] = summary.get("entities", 0)
        self.world["observations"] = summary.get("observations", 0)

    def world_want_frames(self, entities: list[dict[str, Any]]) -> None:
        """One frame per entity is enough for a list; the detail asks for more."""
        for entity in entities[:FRAME_CACHE]:
            frame_id = entity.get("last_frame_id")
            if frame_id and frame_id not in self.world_frames:
                self.world_call("world_state_frame", {"frame_id": frame_id})

    def world_want_frames_for(self, observations: list[dict[str, Any]],
                              limit: int) -> None:
        wanted = []
        for observation in observations:
            frame_id = observation.get("frame_id")
            if (frame_id and frame_id not in self.world_frames
                    and frame_id not in wanted):
                wanted.append(frame_id)
            if len(wanted) >= limit:
                break
        for frame_id in wanted:
            self.world_call("world_state_frame", {"frame_id": frame_id})

    def world_keep_frame(self, body: dict[str, Any]) -> None:
        try:
            jpeg = base64.b64decode(body.get("jpeg_base64", ""))
        except ValueError:
            return
        frame_id = str(body.get("frame_id") or "")
        if not frame_id or not jpeg:
            return
        self.world_frames[frame_id] = jpeg
        while len(self.world_frames) > FRAME_CACHE:
            # Oldest first. A dict preserves insertion order, which is the order
            # they were asked for, which is near enough to the order they will stop
            # being looked at.
            self.world_frames.pop(next(iter(self.world_frames)))

    def world_bump(self) -> None:
        """Say that `/world.json` has changed, so the page fetches it again.

        A counter rather than the body, for the reason the network list is served
        this way: the payload is tens of kilobytes and the state it would ride in
        goes out ten times a second.
        """
        self.world_payload["frames"] = sorted(self.world_frames)
        self.world["gen"] = self.world.get("gen", 0) + 1

    def world_state(self) -> dict[str, Any]:
        """The part of the world that rides in every pushed state."""
        return dict(self.world, gen=self.tag(self.world["gen"]),
                    clear_armed=self.world_clear_armed_until > time.monotonic(),
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
