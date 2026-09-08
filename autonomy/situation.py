"""Everything a decision is made from, in one object that can be written down.

A choice is only reconstructable if what it was choosing between can be
reconstructed, so nothing downstream of here is allowed to ask the rover
anything: `goals.py` and `scoring.py` take a `Situation` and are pure functions
of it. Read the rover once, write the reading into the record, and the same
inputs produce the same candidates and the same ranking a month later on a
different machine. That is the whole of what "deterministic under replay" means
here, and it is a property of the shape of these modules rather than of anybody
remembering to be careful.

**What is in it is what a decision may look at.** The world state's things with
their placements and how their distances have gone, the occupancy map, where the
rover is standing, whether it is already driving, what the battery reads, and
whether the perception loop is running at all. Adding an input means adding it
here, which is also what makes it show up in the snapshot without anybody
remembering to add it there too.

Two things it deliberately does not hold. It has no clock beyond the moment it
was taken -- a scorer that asked what time it is now would not replay -- and it
holds no live handle on anything, so a `Situation` read off a rover in September
and one loaded from a JSON file on a desk are the same kind of object.
"""
from __future__ import annotations

import json
import time
from typing import Any

import client as client_mod
import mapgrid
import refs

#: What the world state must have done recently for its answers to be worth
#: reasoning about. A perception loop that last recorded twenty minutes ago is
#: not a rover that has seen nothing; it is a rover whose looking has stopped,
#: and a decision to go and inspect something rests on a look that will never
#: come.
#:
#: **Ten minutes, because a parked rover looks every five.** The daemon holds a
#: gate that stops a stationary rover recording the same wall every second --
#: rightly, since observations from one spot can never be triangulated against
#: each other -- and the interval it falls back to is five minutes. Three
#: minutes was the first number here and it called a perfectly healthy parked
#: rover unfit within four minutes of it stopping.
WORLD_STALE_S = 600.0

#: The fields of an entity a decision may use. A short list rather than the
#: whole row, because the row carries the appearance vectors' sizes, the console's
#: drawing hints and a `rays` listing that is several kilobytes per thing -- and
#: everything in a `Situation` is copied into the record every time one is taken.
ENTITY_FIELDS = ("id", "kind", "label", "observation_count", "created_at",
                 "last_seen_at", "placement", "placement_uncertainty_m",
                 "placement_map_session", "last_map_session", "ranging",
                 "exemplar_count")


class Situation:
    """One reading of the rover, and the arithmetic that reads it back.

    Immutable by convention: nothing here writes to it after it is built, and
    the copy in the record is taken from `as_dict` at the moment the decision was
    made. `reach` is the one thing computed lazily, because the walk over the map
    costs a few tenths of a second and a situation that never gets as far as
    scoring a movement goal should not pay for it.
    """

    def __init__(self, body: dict[str, Any]) -> None:
        self.body = body
        self._reach: mapgrid.Reach | None = None
        self._grid: Any = None
        self._grid_error: str = ""

    # --- what was read ---------------------------------------------------------

    @property
    def at(self) -> float:
        return float(self.body.get("at") or 0.0)

    @property
    def world_generation(self) -> str:
        return str(self.body.get("world_generation") or refs.UNKNOWN)

    @property
    def map_session(self) -> int | None:
        session = self.body.get("map_session")
        return None if session is None else int(session)

    @property
    def entities(self) -> list[dict[str, Any]]:
        return list(self.body.get("entities") or [])

    @property
    def world(self) -> dict[str, Any]:
        return dict(self.body.get("world") or {})

    @property
    def nav(self) -> dict[str, Any]:
        return dict(self.body.get("nav") or {})

    @property
    def pose(self) -> dict[str, Any] | None:
        pose = self.nav.get("pose")
        return dict(pose) if isinstance(pose, dict) else None

    @property
    def where(self) -> tuple[float, float] | None:
        pose = self.pose
        if not pose or pose.get("x_m") is None or pose.get("y_m") is None:
            return None
        return float(pose["x_m"]), float(pose["y_m"])

    @property
    def heading_deg(self) -> float | None:
        pose = self.pose
        return None if not pose else pose.get("heading_deg")

    @property
    def battery_v(self) -> float | None:
        volts = self.body.get("battery_v")
        return None if volts is None else float(volts)

    @property
    def building(self) -> dict[str, Any]:
        """Whether the rover is looking at all, and how often."""
        return dict(self.body.get("building") or {})

    @property
    def cooled(self) -> list[dict[str, Any]]:
        """Things put aside because looking at them again was getting nowhere.

        Part of the situation rather than something remembered elsewhere, so
        that a decision's inputs contain every reason it could have refused a
        candidate -- see [`cooling.py`](cooling.py).
        """
        return list(self.body.get("cooled") or [])

    @property
    def previous_goal(self) -> dict[str, Any]:
        """What the last deliberation would have done, if it wanted anything.

        Carried in the situation for the reason the cooling list is: the switch
        away from it costs something, and a cost that came from outside the
        recorded inputs would not replay.

        A small record rather than an identifier -- `{"id", "type", "target",
        "goal"}` -- because an identifier carries the viewpoint the generator
        happened to pick, and the same goal an hour later, over a map that has
        been refined by a centimetre, has a different one. Recognising "the same
        goal" by what it is about is the whole point of charging for a change.
        A bare string is still accepted and read as an identifier.
        """
        previous = self.body.get("previous_goal") or {}
        if isinstance(previous, str):
            return {"id": previous} if previous else {}
        return dict(previous)

    # --- the map ---------------------------------------------------------------

    @property
    def grid(self):
        """The occupancy grid, or None with `grid_error` saying why not."""
        if self._grid is None and not self._grid_error:
            try:
                self._grid = mapgrid.grid_of(self.body.get("map") or {})
            except mapgrid.NoMap as exc:
                self._grid_error = str(exc)
            except (ValueError, TypeError, KeyError) as exc:
                self._grid_error = f"the occupancy map did not decode: {exc}"
        return self._grid

    @property
    def grid_error(self) -> str:
        self.grid                                    # noqa: B018 -- fills it in
        return self._grid_error

    @property
    def reach(self) -> mapgrid.Reach | None:
        """Which floor the rover can walk to from where it is standing.

        None when there is no map or no pose on it, which is a refusal for every
        goal that needs the rover to go somewhere and not a reason to rank one
        badly.
        """
        if self._reach is None:
            grid, where = self.grid, self.where
            if grid is not None and where is not None:
                self._reach = mapgrid.Reach(grid, where)
        return self._reach

    # --- how the rover is ------------------------------------------------------

    def health(self) -> dict[str, Any]:
        """What is wrong with the rover as a thing that could be asked to act.

        Each entry is a sentence a person could be told, keyed by the short name
        the record uses. Empty means nothing is wrong, which is the only state in
        which an executive would be allowed to choose anything.
        """
        wrong: dict[str, str] = {}
        nav = self.nav
        if nav.get("error"):
            wrong["navigation"] = ("the navigation bridge did not answer: "
                                   f"{nav['error']}")
        elif nav.get("estop"):
            wrong["estop"] = "the rover is stopped and latched"
        if nav.get("position_trusted") is False:
            wrong["pose"] = ("the rover does not know where it is on the map "
                             "well enough to be given a place to drive to")
        if nav.get("map_settled") is False:
            wrong["map"] = ("the map has not settled since the last restart, so "
                            "map coordinates cannot be trusted yet")
        world = self.world
        if world.get("last_status") not in (None, "", "ok"):
            wrong["world_state"] = ("the perception loop's last look failed: "
                                    f"{world.get('last_detail') or world['last_status']}")
        last_at = world.get("last_at")
        if last_at and self.at and self.at - float(last_at) > WORLD_STALE_S:
            wrong["looking"] = (f"nothing has been recorded for "
                                f"{int(self.at - float(last_at))} s, so the "
                                f"perception loop is not running")
        if self.grid is None:
            wrong["no_map"] = self.grid_error or "there is no occupancy map"
        elif self.where is None:
            wrong["no_pose"] = "the rover did not report where it is"
        elif self.reach is not None and self.reach.standing is None:
            wrong["off_the_floor"] = ("the rover is not standing on floor the "
                                      "map calls free, so no route can be "
                                      "walked from where it is")
        return wrong

    def busy(self) -> str:
        """What the rover is already doing, if anything. A sentence or ''."""
        nav = self.nav
        if nav.get("exploring"):
            return "the rover is already exploring"
        if nav.get("driving"):
            return "the rover is already driving somewhere"
        return ""

    # --- writing it down -------------------------------------------------------

    def as_dict(self) -> dict[str, Any]:
        """The whole situation, as what goes into the record.

        Returned as it is held rather than rebuilt, so that a snapshot and the
        object that made the decision cannot be two different things.
        """
        return self.body

    def digest_body(self) -> dict[str, Any]:
        return self.body

    @classmethod
    def from_dict(cls, body: dict[str, Any]) -> "Situation":
        return cls(dict(body or {}))

    @classmethod
    def from_json(cls, text: str) -> "Situation":
        return cls.from_dict(json.loads(text))

    # --- reading the rover -----------------------------------------------------

    @classmethod
    def read(cls, client: client_mod.ReadOnly, *, at: float | None = None
             ) -> "Situation":
        """One reading, through the door that only opens outwards.

        Every call here is on `client.ReadOnly`'s allow-list, so this cannot ask
        the rover to do anything even by accident. What does not answer is
        recorded as not having answered rather than left out: a missing battery
        and a flat one must not read alike, and neither must a navigation bridge
        that is down and one that says the rover is fine.
        """
        body: dict[str, Any] = {"at": float(at if at is not None else time.time())}

        world = _try(client, "world_state_summary")
        summary = dict(world.get("summary") or {}) if world.get("ok") else {}
        if not world.get("ok"):
            summary["error"] = world.get("error") or "the world state did not answer"
        body["world"] = summary
        body["world_generation"] = summary.get("world_generation") or refs.UNKNOWN
        body["map_session"] = summary.get("map_session")

        listing = _try(client, "world_state_entities")
        body["entities"] = [
            {field: one.get(field) for field in ENTITY_FIELDS if field in one}
            for one in (listing.get("entities") or [])]

        nav = _try(client, "nav_status")
        body["nav"] = ({key: nav.get(key) for key in
                        ("driving", "exploring", "estop", "pose", "map_id",
                         "map_settled", "map_kept", "position_trusted",
                         "match_score", "speed_ms", "clearance_m")}
                       if nav.get("ok") else
                       {"error": nav.get("error") or "the navigator did not answer"})

        battery = _try(client, "battery")
        body["battery_v"] = battery.get("volts") if battery.get("ok") else None

        building = _try(client, "world_building")
        body["building"] = ({"building": building.get("building"),
                             "looks": building.get("looks"),
                             "every_s": building.get("every_s")}
                            if building.get("ok") else {})

        body["map"] = _try(client, "nav_grid")
        return cls(body)


#: The two parts of a situation that are large and change slowly: the things
#: the rover holds, and the occupancy map. Everything else -- the pose, the
#: battery, what was cooling, what it wanted last time -- is a few hundred bytes
#: and changes every time.
HEAVY = ("entities", "map")


def snapshot(store: Any, here: "Situation") -> str:
    """Write a situation into the record, in two parts, and name the whole.

    **Content addressing only saves anything if the thing addressed actually
    repeats.** A whole situation never does: the battery moves by a hundredth of
    a volt and the clock moves at all, so every reading is a new row -- and
    measured on the rover that is 97 kB a minute, of which 96 kB is a listing of
    123 things and a map that did not change. A parked rover deliberating for an
    afternoon would add a quarter of a gigabyte of identical listings, and
    nothing prunes them, because there is no DELETE anywhere in the store.

    So the parts that repeat are stored as their own row and referenced. An
    unchanged world costs one row however many decisions are made from it, which
    is what the addressing was for. `restore` puts the two halves back.
    """
    heavy = store.snapshot("world_and_map",
                           {name: here.body.get(name) for name in HEAVY})
    light = {name: value for name, value in here.body.items()
             if name not in HEAVY}
    light["world_and_map"] = heavy
    return store.snapshot("situation", light)


def restore(store: Any, digest: str) -> "Situation | None":
    """The situation a decision was made from, both halves, or None.

    None rather than a situation missing its things: a re-ranking against an
    empty world would produce a confident, different answer and nothing about it
    would look broken.
    """
    body = store.snapshot_body(digest)
    if not isinstance(body, dict):
        return None
    heavy = store.snapshot_body(body.get("world_and_map") or "")
    if not isinstance(heavy, dict):
        return None
    merged = {name: value for name, value in body.items()
              if name != "world_and_map"}
    merged.update(heavy)
    return Situation(merged)


def _try(client: client_mod.ReadOnly, name: str,
         arguments: dict[str, Any] | None = None) -> dict[str, Any]:
    """One read, with an unreachable daemon reported rather than raised.

    A situation assembled from four answers and one silence is worth having --
    it can still say the rover is unhealthy, which is the honest conclusion --
    and raising here would mean a shadow run that stops recording the first time
    a deploy restarts the daemon underneath it.
    """
    try:
        got = client.call(name, arguments or {})
    except client_mod.Unreachable as exc:
        return {"ok": False, "error": str(exc)}
    except client_mod.Refused as exc:                          # pragma: no cover
        return {"ok": False, "error": str(exc)}
    return got if isinstance(got, dict) else {"ok": False, "error": "no answer"}
