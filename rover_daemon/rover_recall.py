"""What a voice model may ask of the room the rover has already looked at.

[rover_world.py](rover_world.py) is the console's side of the semantic world:
identifiers, map coordinates, scores, the whole observation history. **None of
that is answerable out loud**, and the two audiences are why this is a separate
file rather than three more methods over there. A person at the console is
holding a map and wants `object:12` at (4.31, 2.09) with a cosine of 0.137
beside it; a person in the room is asking "can you find the bed", and the answer
is two metres away, ahead and to your left, seen a minute ago.

So nothing here takes an identifier or a coordinate as an *argument*, and that is
a rule rather than a tidiness. The argument is `_tool_drive_to`'s, in
rover_nav.py: a model has no way to arrive at a number in the map's frame except
by inventing one, and an invented pair is a drive to a place nobody chose. A
phrase is the one handle a model genuinely holds, so a phrase is what both of
these tools take -- the same phrase every time, answered by the same ranking, so
"find the desk" and "go to the desk" a minute later are about the same desk
without anything being carried between them.

    find_thing   has the rover seen this, where is it, and what does it know
                 about it
    go_to_thing  drive to somewhere it can be seen from, and say so at once

**A coordinate coming back is a different thing from a coordinate going in.** A
found thing answers with its position on the map, and that is deliberate: it is
what lets one thing be compared with another. "How far is the bed from the desk"
is two calls and the distance between two pairs of numbers, which is a better
tool than a third one that measured it, because the same two pairs also answer
"which of them is nearer the door" and everything else nobody thought to write a
tool for. What the model must not do with them is read them out loud or try to
drive to them, and the schema says so.

**Read-only, except for the wheels.** rover_world.py says the model is shown
none of the world state because the question of whether the world state is worth
trusting has to be answered before a model is given authority over it. That
still holds for writing: nothing here records, attaches, places or clears
anything, and there is no model tool that does. What has changed is reading it,
and driving to what was read.

**Nothing here decides identity.** The ranking is `world_state.search`'s, the
floor it is judged against is the one measured on this rover, and where to stand
is `world_state.approach`'s. This file chooses which row of the ranking to
answer about and puts the answer into words.
"""
from __future__ import annotations

import math
import time
from typing import Any

#: How far down the ranking to look for something the rover has actually made a
#: thing of. The best-scoring *look* often belongs to nothing yet -- that is the
#: ordinary state of anything seen once -- and the answer to "where is the bed"
#: is a thing with a position, not the highest number in the list.
RECALL_LIMIT = 10
#: How long to wait for the wheels after telling a move it is being replaced.
#: A stop reaches the driver board in well under a second, and what is being
#: waited for after that is the thread holding the move letting go of the mutex.
#: Long enough for that, short enough to be inside a tool call somebody is
#: listening to the silence of.
HANDOVER_S = 3.0
HANDOVER_POLL_S = 0.05

#: Which way something is, in the words a model can say without doing arithmetic
#: on an angle -- the same choice `_where` makes for a face in a picture, for the
#: same reason. Degrees off the rover's nose, positive to its left, and the last
#: entry catches everything beyond the one before it.
WHICH_WAY = (
    (20.0, "straight ahead"),
    (65.0, "ahead and to your {side}"),
    (115.0, "to your {side}"),
    (160.0, "behind you and to your {side}"),
    (180.0, "straight behind you"),
)


def which_way(relative_deg: float) -> str:
    """Where something is relative to the way the rover is facing, in words."""
    turn = abs(float(relative_deg))
    side = "left" if float(relative_deg) > 0.0 else "right"
    for limit, words in WHICH_WAY:
        if turn <= limit:
            return words.format(side=side)
    return "straight behind you"


def how_long_ago(when: float | None) -> str:
    """When something was last seen, said the way a person says it.

    Vague on purpose past the first minute. The rover records a look a second, so
    "last seen 143 seconds ago" is a precision about the *recording* rather than
    about the room, and out loud it sounds like the rover is reading a log.
    """
    if not when:
        return "never"
    gap = max(0.0, time.time() - float(when))
    if gap < 45.0:
        return "just now"
    if gap < 90.0:
        return "a minute ago"
    if gap < 3300.0:
        return "%d minutes ago" % round(gap / 60.0)
    if gap < 5400.0:
        return "an hour ago"
    return "%d hours ago" % round(gap / 3600.0)


class RoverRecall:
    """The two world-state tools a model is shown. Mixed into Rover."""

    # --- finding one thing ----------------------------------------------------

    def _recall(self, description: str) -> dict[str, Any]:
        """The one thing a phrase found, with everything the tools need about it.

        `found` is whether anything cleared the floor, and it is deliberately
        judged row by row rather than off the answer's own verdict: the verdict
        is about the best *look*, and the look this settles on may be further down
        the list because the ones above it belong to nothing yet.

        `entity` is the store's row and `placement` its position, both of which
        stay in this file -- what leaves it is metres and words.
        """
        why = self._world_ready()
        if why:
            return {"ok": False, "error": why}
        answer = self._tool_world_state_search({"query": description,
                                                "limit": RECALL_LIMIT})
        if not answer.get("ok"):
            return answer
        floor = answer.get("floor")
        store = self._world_store()
        for match in answer.get("matches") or []:
            entity_id = str(match.get("entity_id") or "")
            if not entity_id:
                continue
            if floor is not None and float(match.get("score") or 0.0) < floor:
                break
            entity = store.entity(entity_id)
            if entity is None:                      # cleared while we ranked it
                continue
            return {"ok": True, "found": True, "entity": entity,
                    "id": entity_id, "score": match.get("score"),
                    "last_seen_at": match.get("observed_at")
                                    or entity.get("last_seen_at")}
        # Nothing that cleared the bar has been made a thing of. The sentence is
        # the search's own, because the search is what knows whether this is "not
        # in this room" or "the rover has barely looked at anything yet".
        return {"ok": True, "found": False,
                "detail": str(answer.get("detail") or
                              "nothing the rover has looked at matches that")}

    def _placed_now(self, entity: dict[str, Any]) -> dict[str, Any] | None:
        """Where the thing is on the map the rover is on now, or None.

        None covers the two ways a position is not one: never crossed, which is
        the ordinary state of everything seen from a single place, and measured
        under a map that has since been thrown away, whose coordinates name a
        place in this one only by coincidence.
        """
        place = entity.get("placement")
        if not place:
            return None
        if entity.get("placement_map_session") != self._world_store().map_session():
            return None
        return place

    def _from_here(self, place: dict[str, Any]) -> dict[str, Any]:
        """How far away the thing is and which way, or an empty answer.

        Empty when nothing is publishing where the rover is: the thing has a
        position and the rover does not, so there is no distance to give and
        saying one would be inventing the rover's own.
        """
        pose = self._world_pose()
        if pose is None:
            return {}
        dx = float(place["x_m"]) - pose["x_m"]
        dy = float(place["y_m"]) - pose["y_m"]
        turn = (math.degrees(math.atan2(dy, dx)) - pose["heading_deg"] + 180.0)
        return {"distance_m": round(math.hypot(dx, dy), 1),
                "direction": which_way(turn % 360.0 - 180.0)}

    def _measured(self, place: dict[str, Any]) -> dict[str, Any]:
        """What the rover has measured about a placed thing, besides where it is.

        **The position is in the map's own frame and that is the point of it.**
        Everything else here is a distance or a count, which mean the same thing
        wherever they are read; a coordinate only means something against other
        coordinates, and that is exactly what it is for -- two of these, from two
        calls, are what "how far is the bed from the desk" is the distance
        between. They are always in the map the rover is on now, because
        `_placed_now` refuses a placement measured under any other, so two
        answers in one conversation are comparable unless somebody clears the map
        between them.

        `known_to_m` is how far out the position may be, and it belongs beside
        the position rather than in a footnote: on this rover it is tens of
        centimetres, so a distance worked out from two of these is worth saying
        as "about three metres" and not as 3.14.

        `width_m` is the thing's own width, and it saturates. `locate.MAX_EXTENT_M`
        caps the stored half-width at 0.75 m so that a region spanning most of a
        frame cannot claim the room, which means anything from a sofa upwards
        comes back as a metre and a half. Reported anyway, because "about a metre
        and a half" is the right answer for a sofa and only wrong for a wall.
        """
        found = {
            "map_x_m": round(float(place["x_m"]), 2),
            "map_y_m": round(float(place["y_m"]), 2),
            "known_to_m": round(float(place.get("uncertainty_m") or 0.0), 1),
            # How many separate places agreed about it, which is different
            # evidence from how many looks did: seven looks from one doorway are
            # one opinion. `seen_times` beside it is the raw count.
            "seen_from_places": place.get("viewpoints"),
        }
        # Doubled because `locate.extent_of` measures the half-width -- the
        # angular span at the range the crossing put it, which is the distance
        # from the middle to the edge.
        extent = place.get("extent_m")
        if extent:
            found["width_m"] = round(2.0 * float(extent), 1)
        # Only ever present once somebody has measured how high the camera is;
        # until then the store's height is above the lens, which is not a thing
        # to say to a person. See `locate.above_floor_m`.
        if place.get("height_above_floor_m") is not None:
            found["height_m"] = round(float(place["height_above_floor_m"]), 1)
        return {name: value for name, value in found.items() if value is not None}

    def _tool_find_thing(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Has the rover seen this, where is it, and what does it know about it.

        Four answers, and telling them apart is most of the value: it has never
        seen anything like that; it has seen one but has only ever looked at it
        from one place, so it has no position for it; it has a position but does
        not know where it is itself; and it knows both, which is a distance, a
        direction and the rest of what was measured.
        """
        described = str(arguments.get("description") or "").strip()
        if not described:
            return {"ok": False, "error": "say what to look for"}
        recalled = self._recall(described)
        if not recalled.get("ok"):
            return recalled
        if not recalled["found"]:
            return {"ok": True, "found": False, "description": described,
                    "note": "the rover has not seen anything matching that: "
                            + recalled["detail"]}

        entity = recalled["entity"]
        answer = {"ok": True, "found": True, "description": described,
                  "seen_times": entity.get("observation_count") or 0,
                  "last_seen": how_long_ago(recalled.get("last_seen_at")),
                  "first_seen": how_long_ago(entity.get("created_at"))}
        place = self._placed_now(entity)
        if place is None:
            return {**answer, "placed": False,
                    "note": "the rover has seen it but has only looked at it "
                            "from one place, so it cannot say where it is; "
                            "driving somewhere else and looking again is what "
                            "gives it a position"}
        answer = {**answer, "placed": True, **self._measured(place)}
        here = self._from_here(place)
        if not here:
            return {**answer,
                    "note": "the rover knows where it is on the map but not "
                            "where the rover itself is, so it cannot say how "
                            "far away it is from here"}
        return {**answer, **here}

    # --- going to one thing ---------------------------------------------------

    def _tool_go_to_thing(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Drive to somewhere the thing can be seen from, and answer at once.

        **The move is not waited for**, for the reason `explore` is not: one
        client holds one connection, so a tool call that blocks for two minutes
        blocks `stop_driving` for two minutes and neither the model nor the person
        in the room can stop the rover. So this starts the trip and comes back,
        and `ok` means it set off rather than that it arrived.

        `ok` is false for everything that did not end in the rover moving, which
        is the one thing that matters about the shape of this result: the model
        reads it and says out loud what it did, and a refusal that came back
        looking like a success is a rover claiming to be on its way to a bed it
        has never seen.

        Asked again for the same thing, this reports rather than restarts --
        `explore`'s rule, and for its reason: a model unsure whether its call
        landed calls again, and a rover that answered that by starting over would
        be a rover that never got anywhere. Asked for something *different* it
        stops what is running and goes, because that is not an echo, it is
        somebody changing their mind, and it is the rule the drive console has
        had all along.
        """
        if self.nav is None:
            return {"ok": False, "error": "this rover cannot drive itself"}
        described = str(arguments.get("description") or "").strip()
        if not described:
            return {"ok": False, "error": "say what to go to"}
        recalled = self._recall(described)
        if not recalled.get("ok"):
            return recalled
        if not recalled["found"]:
            return {"ok": False,
                    "error": "the rover has not seen anything matching that, so "
                             "there is nowhere to go: " + recalled["detail"]}

        going = self.nav.errand
        if going and going.get("id") == recalled["id"]:
            return {"ok": True, "going": True, "description": described,
                    "note": "it is already on its way there, and has been for "
                            "%d seconds" % round(self.nav.away_for)}

        found = self._tool_world_state_viewpoint({"id": recalled["id"]})
        if not found.get("ok"):
            return {"ok": False, "error": str(found.get("error") or
                                              "there is nowhere to see it from")}
        handed = self._take_the_wheels()
        if handed:
            return {"ok": False, "error": handed}
        started = self.nav.drive_to_in_background(
            found["x_m"], found["y_m"], heading_deg=found["heading_deg"],
            for_what={"id": recalled["id"], "said": described})
        if not started.get("started"):
            return {"ok": False,
                    "error": "the rover is already driving somewhere, so it "
                             "cannot set off for that until it has finished or "
                             "been stopped"}
        # How the last trip ended, said as the next one sets off, because nothing
        # waits for one of these and this is the only moment anybody is told.
        ended = self.nav.ran_errand
        return {
            "ok": True, "going": True, "description": described,
            "distance_m": round(float(found["travel_m"]), 1),
            "note": "the rover has set off, about %.1f metres, and will stop "
                    "where it can see it. Say so out loud; it takes a minute or "
                    "so, stop_driving ends it, and asking again says how it is "
                    "getting on." % float(found["travel_m"]),
            "last_trip": None if ended is None else
                         "going to %s: %s" % (ended[0].get("said") or "somewhere",
                                              ended[1].detail),
        }

    def _take_the_wheels(self) -> str:
        """Stop whatever move is running, or say why the wheels cannot be had.

        Empty when there is nothing to stop or the stop worked, and a sentence
        otherwise. The wait is for the thread holding the move to let go of the
        navigator's mutex: the stop itself reaches the driver board in well under
        a second, but the move that was told to stop is still winding up, and
        starting the next one before it has finished would be refused as busy.
        """
        if not self.nav.driving:
            return ""
        self.nav.stop()
        deadline = time.monotonic() + HANDOVER_S
        while self.nav.driving and time.monotonic() < deadline:
            time.sleep(HANDOVER_POLL_S)
        if self.nav.driving:
            return ("the rover was told to stop what it was doing and has not "
                    "let go of the wheels yet, so it has not set off")
        return ""

