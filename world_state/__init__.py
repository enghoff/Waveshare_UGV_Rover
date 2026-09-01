"""Semantic world state: what the rover has been told is in the room.

SLAM Toolbox and Nav2 own the map, the pose and the routes. What lives here is
what a model said was visible, when, and from exactly where -- and, in the end,
which lasting thing in the room that was. The model proposes; the code in here
allocates the identifiers, validates the answer and owns the database.

**Identity is measured, not asked for.** The model is shown one picture and
nothing about the world it has already seen, because two of them were asked which
thing they were looking at and both failed: one never recognises anything, the
other recognises things that are not in the room. What separates two identical
chairs is not what they look like but where they are, so identity comes from a
bearing taken from a measured pose, and a second bearing from somewhere far enough
away to cross it.

    store.py      SQLite: entities, their observation history, and the frames
    contract.py   the JSON the model is asked for, and what is done to it before
                  any of it is believed
    reasoner.py   the replaceable model boundary, and the local Cosmos client
    inspector.py  one inspection end to end: frame, model, store
    view.py       an observation's measured provenance as a bearing to draw
    locate.py     two bearings from two places as a point on the map

The rover deploys this to ``~/ugv/world_state`` and the daemon imports it from
there; the database and the frames live under ``~/.ugv/world``, where no deploy
can reach them.
"""
from __future__ import annotations

from .contract import KINDS, PROMPT_VERSION, Result, Seen, build_prompt, validate
from .inspector import Inspector
from .locate import agrees, best_fix, fix
from .reasoner import (
    Answer, CosmosReasoner, FakeReasoner, PhysicalReasoner, describe_backend,
)
from .store import WorldStore, world_dir
from .view import ray, rays

__all__ = [
    "Answer", "CosmosReasoner", "FakeReasoner", "Inspector", "KINDS",
    "PROMPT_VERSION", "PhysicalReasoner", "Result", "Seen", "WorldStore",
    "agrees", "best_fix", "build_prompt", "describe_backend", "fix", "ray",
    "rays", "validate", "world_dir",
]
