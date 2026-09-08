"""The store, the pictures and the one worked episode the checks share.

`an_episode` is a shadow run of the kind Phase 1 expects: the rover notices a
thing it has seen once and never placed, considers two goals, chooses one, looks
at it, and closes without having moved -- because in shadow mode there is nothing
it is allowed to drive with. Every check that needs an episode uses this one, so
that a change to what an episode looks like shows up in one place.
"""
from __future__ import annotations

import base64
from typing import Any

import client
import events
import refs
from store import EpisodeStore

#: A generation standing in for a world store, and a second one standing in for
#: what that store becomes after somebody clears it.
WORLD = "9f2a1c04ffab3d21"
CLEARED = "0011223344556677"

#: Two bytes' worth of "picture". Nothing here decodes them.
PICTURE = b"\xff\xd8\xff\xe0 a frame the rover kept"
DEPTH = b"\x1f\x8b a depth map the rover kept"


def a_store(directory: str) -> EpisodeStore:
    return EpisodeStore(directory)


def a_world(generation: str = WORLD) -> dict[str, Any]:
    """What the world state would have reported when the decision was made."""
    return {
        "world_generation": generation,
        "entities": 2,
        "observations": 31,
        "map_session": 7,
        "things": [
            {"id": "object:8", "looks": 1, "placed": False,
             "last_seen_at": 1757320000.0},
            {"id": "object:42", "looks": 36, "placed": True,
             "placement_uncertainty_m": 0.263},
        ],
    }


def an_episode(store: EpisodeStore, *, generation: str | None = WORLD,
               keep_evidence: bool = True) -> str:
    """One shadow run, recorded the way an executive would record it."""
    world = a_world(generation or refs.UNKNOWN)
    episode = store.open_episode(
        "nothing_to_do", world_generation=generation, map_session=7,
        detail={"idle_s": 45.0})

    kept = []
    if keep_evidence:
        kept.append(store.keep_evidence(
            "frame", PICTURE,
            source={"world": refs.world(generation, "object:8"),
                    "frame_id": "35022"}))
        kept.append(store.keep_evidence("depth", DEPTH,
                                        source={"frame_id": "35022"}))

    store.append(episode, events.candidate(
        "look_at(object:8)", "seen once and never placed", score=0.81,
        params={"entity": "object:8"},
        refs=[refs.world(generation, "object:8")]))
    store.append(episode, events.candidate(
        "look_at(object:42)", "placed already, and placed well", score=0.12,
        params={"entity": "object:42"},
        refs=[refs.world(generation, "object:42")]))

    snapshot = store.snapshot("world_state", world)
    store.append(episode, events.decision(
        "look_at(object:8)", "one look is a bearing and not a position",
        snapshot, rejected=["look_at(object:42)"],
        refs=[refs.world(generation, "object:8")]))

    store.append(episode, events.model(
        "alibaba", "qwen-omni-realtime", "phrasing what it was about to do",
        duration_s=0.42))
    store.append(episode, events.call(
        "look_at", {"entity": "object:8", "pan_deg": -20.0}, ok=True,
        result={"observations": 1, "ranged": False}, duration_s=1.9,
        refs=[refs.world(generation, "object:8")]))
    store.append(episode, events.world_change(
        "one more look at a thing that is still not placed",
        matched=[refs.world(generation, "object:8")],
        refs=[refs.world(generation, "object:8")],
        evidence=kept))
    store.append(episode, events.measured(
        "the attempt", duration_s=2.4, travel_m=0.0, battery_v=11.8))
    store.close_episode(episode, "abandoned",
                        detail="shadow mode: no movement authority")
    return episode


# --- a rover that answers without being one ---------------------------------

class FakeRover(client.ReadOnly):
    """A daemon's answers, without a daemon.

    Subclasses the real client and replaces only the socket, so everything a
    test does goes through the real allow-list: a check that this refuses
    `drive` is a check about the client the rover actually uses, not about a
    stand-in that happens to agree with it.
    """

    def __init__(self, *, generation: str = WORLD, rows: list | None = None,
                 said: list | None = None, frames: dict | None = None,
                 entities: list | None = None) -> None:
        super().__init__()
        self.generation = generation
        self.rows = list(rows or [])
        #: The driving loop's running commentary, oldest first. The last is what
        #: it is saying now; the ones before it are what a poller that named a
        #: sequence number gets back under `missed`.
        self.said = _commentary(said or [{"seq": 0, "phase": "idle"}])
        self.frames = dict(frames or {})
        self.entities = list(entities or [])
        self.asked: list[str] = []
        self.down = False

    def _ask(self, name, arguments):
        self.asked.append(name)
        if self.down:
            raise client.Unreachable(f"{name}: nothing is listening")
        self.calls += 1
        return getattr(self, "_" + name)(arguments)

    # the calls, in the shapes the daemon really returns

    def _world_state_summary(self, _arguments):
        summary = {"entities": len(self.entities), "observations": len(self.rows),
                   "unmatched": 0, "inspections": 1, "map_session": 7,
                   "world_generation": self.generation}
        if self.generation is None:
            summary.pop("world_generation")
        return {"ok": True, "summary": summary, "backend": "fake"}

    def _world_state_entities(self, _arguments):
        return {"ok": True, "entities": self.entities}

    def _world_state_entity(self, arguments):
        wanted = arguments.get("entity_id")
        for one in self.entities:
            if one.get("id") == wanted:
                return {"ok": True, "entity": one}
        return {"ok": False, "error": "no such thing"}

    def _world_state_observations(self, arguments):
        """Newest first, paged the way the daemon pages: below a given row."""
        limit = int(arguments.get("limit") or 20)
        rows = sorted(self.rows, key=lambda r: r["id"], reverse=True)
        before_at = arguments.get("before_at")
        if before_at not in (None, ""):
            edge = (float(before_at), int(arguments.get("before_id") or 0))
            rows = [r for r in rows
                    if (r["observed_at"], r["id"]) < edge]
        page = rows[:limit]
        return {"ok": True, "observations": page, "more": len(page) == limit}

    def _world_state_frame(self, arguments):
        jpeg = self.frames.get(arguments.get("frame_id"))
        if jpeg is None:
            return {"ok": False, "error": "no stored frame"}
        return {"ok": True, "frame_id": arguments.get("frame_id"),
                "bytes": len(jpeg),
                "jpeg_base64": base64.b64encode(jpeg).decode("ascii")}

    def _world_building(self, _arguments):
        return {"ok": True, "building": True, "looks": 9, "every_s": 1.0}

    def _nav_status(self, arguments):
        since = arguments.get("since_seq")
        missed = [] if since is None else [one for one in self.said[:-1]
                                           if one["seq"] > int(since)]
        move = {**self.said[-1], "missed": missed}
        return {"ok": True, "move": move, "driving": False,
                "exploring": False, "estop": False, "map_settled": True,
                "map_kept": True, "position_trusted": True, "map_id": "m1",
                "match_score": 0.8,
                "pose": {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0}}

    def _battery(self, _arguments):
        return {"ok": True, "volts": 12.07, "percent": 85}


def _commentary(said: list) -> list:
    """Carry a sentence's fields into the next, the way the real loop does.

    `MoveReport.say` copies the whole state and updates it, so `kind` and what
    was asked for persist through a move and only change when `begin` starts a
    new one; `why` is cleared unless the new phase gives a reason, because a
    reason left over from the previous phase is a lie about this one. A fake
    that dropped them instead would split one move into several, and the test
    written against it would be testing the fake.
    """
    out: list = []
    for one in said:
        carried = dict(out[-1]) if out else {}
        carried.pop("reason", None)
        carried["why"] = ""
        carried.update(one)
        out.append(carried)
    return out


def a_look(inference_id: int, first_row_id: int, *, regions: int = 2,
           at: float = 1757320000.0, frame_id: str = "",
           attached: bool = True) -> list[dict]:
    """The rows one inspection leaves behind, as the daemon reports them."""
    frame = frame_id or f"frame-{inference_id}"
    return [{
        "id": first_row_id + n,
        "inference_id": inference_id,
        "observed_at": at + n * 0.01,
        "frame_id": frame,
        "entity_id": f"object:{10 + n}" if attached else None,
        "map_session": 7,
        "observer_pan_deg": -20.0,
        "observer_tilt_deg": 0.0,
        "camera": "gimbal",
        "bearing_deg": -18.5,
        "bearing_sigma_deg": 1.5,
        "range_m": 1.371 if n == 0 else None,
        "pose": {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0},
    } for n in range(regions)]
