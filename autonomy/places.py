"""Places on a map where a run may drive but must not choose to stop.

**Not a no-go zone.** Navigation still plans across these, and a person can
still send the rover onto one; what they refuse is a run *choosing* one as the
place to drive to. The first is the rug under the dining table: the owner
reported on 2026-10-07 that the rover often gets stuck on it, and in M3
session 12 it did, pivoting at full turning power without turning. The lidar
scans one plane about 20 cm up and cannot see a rug, so nothing the rover
senses would keep it off; the owner asked for it to be no place to plan a
drive target, rather than a hard boundary.

The areas live in `places.json` beside this file, keyed by map identity: a box
drawn on one map means nothing on the next, so a new map starts with none
until somebody draws them again. Each area is a box in that map's frame, with
the name the record gives it and why it is there.
"""

from __future__ import annotations

import json
import os
from typing import Any

HERE = os.path.dirname(os.path.abspath(__file__))
PLACES = os.path.join(HERE, "places.json")

_cache: dict[str, Any] = {"mtime": None, "areas": {}}


def areas_for(map_id: str | None, path: str = PLACES) -> list[dict[str, Any]]:
    """The areas drawn on this map, or none. Read again when the file changes."""
    if not map_id:
        return []
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        return []
    if _cache["mtime"] != (path, mtime):
        with open(path, encoding="utf-8") as fh:
            _cache["areas"] = json.load(fh)
        _cache["mtime"] = (path, mtime)
    return list(_cache["areas"].get(map_id) or [])


def refusal(goal: dict[str, Any] | None,
            areas: list[dict[str, Any]]) -> str:
    """Why a drive target may not be chosen here, or "" when it may."""
    if not goal or goal.get("x_m") is None or goal.get("y_m") is None:
        return ""
    x, y = float(goal["x_m"]), float(goal["y_m"])
    for area in areas:
        if (area["min_x_m"] <= x <= area["max_x_m"]
                and area["min_y_m"] <= y <= area["max_y_m"]):
            return ("the place it would drive to is on %s, which is not a place "
                    "to stop: %s" % (area.get("name", "a marked area"),
                                     area.get("why", "marked by the owner")))
    return ""
