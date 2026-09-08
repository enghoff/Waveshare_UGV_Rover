"""The store, the pictures and the one worked episode the checks share.

`an_episode` is a shadow run of the kind Phase 1 expects: the rover notices a
thing it has seen once and never placed, considers two goals, chooses one, looks
at it, and closes without having moved -- because in shadow mode there is nothing
it is allowed to drive with. Every check that needs an episode uses this one, so
that a change to what an episode looks like shows up in one place.
"""
from __future__ import annotations

from typing import Any

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
