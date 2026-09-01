"""Semantic world state: what the rover has been told is in the room.

This package is deliberately not about geometry. SLAM Toolbox and Nav2 own where
things are; what lives here is what a physical-reasoning model said was visible,
when, from where, and which lasting thing in the room the application decided that
was. The model proposes; the code in here allocates the identifiers, validates the
answer and owns the database.

    store.py      SQLite: entities, their observation history, and the frames
    contract.py   the JSON the model is asked for, and what is done to it before
                  any of it is believed
    reasoner.py   the replaceable model boundary, and the local Cosmos client
    inspector.py  one inspection end to end: frame, context, model, store
    view.py       an observation's measured provenance as a bearing to draw

The rover deploys this to ``~/ugv/world_state`` and the daemon imports it from
there; the database and the frames live under ``~/.ugv/world``, where no deploy
can reach them.
"""
from __future__ import annotations

from .contract import KINDS, PROMPT_VERSION, Result, Seen, build_prompt, validate
from .inspector import Inspector
from .reasoner import (
    Answer, CosmosReasoner, FakeReasoner, PhysicalReasoner, describe_backend,
)
from .store import WorldStore, world_dir
from .view import ray, rays

__all__ = [
    "Answer", "CosmosReasoner", "FakeReasoner", "Inspector", "KINDS",
    "PROMPT_VERSION", "PhysicalReasoner", "Result", "Seen", "WorldStore",
    "build_prompt", "describe_backend", "ray", "rays", "validate", "world_dir",
]
