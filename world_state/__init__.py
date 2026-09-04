"""Semantic world state: what the rover has measured about the room it is in.

SLAM Toolbox and Nav2 own the map, the pose and the routes. What lives here is
what the camera saw, when, and from exactly where -- and, in the end, which
lasting thing in the room that was. The code in here allocates the identifiers
and owns the database.

**Identity is measured, not asked for.** A language model used to be shown the
picture and asked which thing it was looking at; two of them were tried on the
rover's own frames and both failed, one never recognising anything and the other
recognising things that were not in the room. What separates two identical chairs
is not what they look like but where they are, so identity comes from a bearing
taken from a measured pose, and a second bearing from somewhere far enough away
to cross it. Nothing here asks a model anything.

    store.py             SQLite: entities, their observation history, the frames
    perception_client.py the encoders behind one call: regions, and two vectors
                         for each
    inspector.py         one inspection end to end: frame, encoders, store
    resolve.py           which lasting thing a pending observation belongs to
    search.py            a typed phrase against the stored semantic vectors
    view.py              an observation's measured provenance as a bearing to draw
    locate.py            two bearings from two places as a point on the map
    approach.py          where the rover would have to stand to look at one
    replay.py            a recorded run back through the resolver, at a desk

The rover deploys this to ``~/ugv/world_state`` and the daemon imports it from
there; the database and the frames live under ``~/.ugv/world``, where no deploy
can reach them.
"""
from __future__ import annotations

from .inspector import Inspector
from . import approach
from . import resolve
from . import search
from .locate import agrees, best_fix, fix
from .perception_client import (
    Eyes, FakeEyes, Look, SidecarEyes, Sighting, describe_eyes,
)
from .store import WorldStore, world_dir
from .view import ray, rays

__all__ = [
    "Eyes", "FakeEyes", "Inspector", "Look", "SidecarEyes", "Sighting",
    "WorldStore", "agrees", "approach", "best_fix", "describe_eyes", "fix",
    "ray", "rays", "resolve", "search", "world_dir",
]
