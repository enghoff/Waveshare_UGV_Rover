#!/usr/bin/env python3
"""Replay the viewpoint chooser over a recorded world, at a desk.

    scp orin:'~/.ugv/world/world.db' /tmp/run.db
    python3 world_state/bench_approach.py /tmp/run.db --grid /tmp/world_replay.json

**Why this exists.** `approach.py` decides where the rover is sent when somebody
presses "go to", and every rule in it is a judgement about real rooms. The
selftests draw those rooms by hand, which proves the arithmetic and settles
nothing about the room the rover is actually in -- and the one measurement this
module has ever had (60 placed things, 2026-09-04, in world_state/README.md) was
a script somebody wrote once and threw away. This is that script, kept.

What it reports is the difference the *sight lines* make: for every thing the
recording placed, the directions it was really looked at from, the median of
those, and whether the answer that comes out is the one the plain ring would have
given. A run where the two agree everywhere means the preference is costing
nothing and buying nothing on that recording; a run where they differ says by how
far, which is how much driving the preference is spending.

**The grid is the part a recording does not carry.** The store keeps poses and
bearings, not the occupancy map they were measured against, so a replay of a
recording whose map has since been cleared has no walls to test against. Without
`--grid` this uses open floor over the whole scene and says so: what it then
measures is the *choice of direction*, which is the whole of what sight lines
change, and not whether that direction is clear on any particular day's map.
`--grid` takes the JSON that `collect_world.py` writes, and is only meaningful
when the recording's map session is the one still on the rover.

Nothing here touches the rover.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import sqlite3
import sys
import zlib

if __package__ in (None, ""):                       # run as a script
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "world_state"

from . import approach                              # noqa: E402
from . import view as world_view                    # noqa: E402

#: How many of a thing's newest looks are read, matching `rover_world.SIGHT_LIMIT`
#: so that what is measured here is what the rover would do.
SIGHT_LIMIT = 60
#: The field of view a recording's rays are drawn through when the row does not
#: carry a stored bearing. Every row the rover has written since bearings were
#: stored does carry one, so this is only reached by very old recordings.
FOV_DEG = 130.0
#: How much floor to leave around the placements when there is no real grid, and
#: how big a cell to draw it in. The cell is the rover's own; the margin is more
#: than `approach.FAR_M` so that no candidate falls off the edge of the invention.
OPEN_MARGIN_M = 4.0
OPEN_CELL_M = 0.05


def placed(path: str) -> list[dict]:
    """Every placed thing in the recording, with its looks, newest first."""
    db = sqlite3.connect(path)
    db.row_factory = sqlite3.Row
    things = []
    for row in db.execute("SELECT * FROM entities"
                          " WHERE placement_json IS NOT NULL"):
        entity = dict(row)
        entity["placement"] = json.loads(entity["placement_json"])
        looks = [_readable(dict(one)) for one in db.execute(
            "SELECT * FROM observations WHERE entity_id = ?"
            " ORDER BY observed_at DESC, id DESC LIMIT ?",
            (entity["id"], SIGHT_LIMIT))]
        entity["looks"] = world_view.rays(looks, FOV_DEG, limit=SIGHT_LIMIT,
                                          placement=entity["placement"])
        things.append(entity)
    db.close()
    return things


def _readable(row: dict) -> dict:
    """One stored observation in the shape `view.ray` reads, which is the shape
    `store.observations` hands back: the JSON columns decoded under their plain
    names."""
    for column, name in (("observer_pose_json", "pose"), ("bbox_json", "bbox"),
                         ("raw_json", "raw")):
        text = row.pop(column, None)
        try:
            row[name] = json.loads(text) if text else None
        except (TypeError, ValueError):
            row[name] = None
    if isinstance(row.get("raw"), dict):
        for name in ("bearing_deg", "span_deg", "elevation_deg",
                     "elevation_span_deg", "bearing_sigma_deg",
                     "origin_sigma_m", "range_m", "range_sigma_m", "lens"):
            if name in row["raw"] and row.get(name) is None:
                row[name] = row["raw"][name]
    return row


def open_floor(things: list[dict]) -> approach.Grid:
    """Mapped floor over everything in the recording, and nothing solid in it.

    An invention, and named as one wherever it is used. It is here so that the
    direction the chooser picks can be measured on a recording whose map is gone,
    which is every recording older than the last time anybody cleared the map.
    """
    xs = [float(one["placement"]["x_m"]) for one in things]
    ys = [float(one["placement"]["y_m"]) for one in things]
    for one in things:
        xs += [float(look["x_m"]) for look in one["looks"]]
        ys += [float(look["y_m"]) for look in one["looks"]]
    x0, y0 = min(xs) - OPEN_MARGIN_M, min(ys) - OPEN_MARGIN_M
    width = int(math.ceil((max(xs) + OPEN_MARGIN_M - x0) / OPEN_CELL_M))
    height = int(math.ceil((max(ys) + OPEN_MARGIN_M - y0) / OPEN_CELL_M))
    return approach.Grid(width, height, OPEN_CELL_M, x0, y0,
                         [[0] * width for _ in range(height)])


def real_grid(path: str) -> tuple[approach.Grid, dict | None, int | None]:
    """The rover's own grid, pose and map session out of a `collect_world.py` file."""
    import numpy

    with open(path) as handle:
        held = json.load(handle)
    grid = held["grid"]
    cells = numpy.frombuffer(zlib.decompress(base64.b64decode(grid["data"])),
                             dtype=numpy.int8)
    cells = cells.reshape(int(grid["height"]), int(grid["width"]))
    return (approach.Grid(int(grid["width"]), int(grid["height"]),
                          float(grid["resolution_m"]), float(grid["origin_x_m"]),
                          float(grid["origin_y_m"]), cells),
            held.get("pose"), held.get("map_session"))


def _turn(degrees: float) -> float:
    return (degrees + 180.0) % 360.0 - 180.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("database", help="a world.db the rover wrote")
    parser.add_argument("--grid", metavar="JSON",
                        help="a collect_world.py file, for the real occupancy "
                             "grid and the rover's pose")
    parser.add_argument("--from", dest="stand", metavar="X,Y",
                        help="where the rover is standing, in metres; the "
                             "default is where it took its newest look")
    parser.add_argument("--detail", action="store_true",
                        help="one line per thing rather than the summary")
    args = parser.parse_args()

    things = placed(args.database)
    if not things:
        print("nothing in that recording has a position, so there is nowhere "
              "to be sent to look at anything")
        return 1

    pose = None
    if args.grid:
        grid, pose, session = real_grid(args.grid)
        room = "the rover's own map"
        sessions = {one.get("placement_map_session") for one in things}
        if session is not None and sessions != {session}:
            # Coordinates measured under a map that has been cleared name a place
            # in the new one only by coincidence, so a grid from a different
            # session would test these placements against somebody else's walls.
            print("the recording placed things under map %s and that grid is "
                  "map %s, so they are not the same room; use --grid only on a "
                  "recording whose map is still the rover's"
                  % (sorted(str(one) for one in sessions), session))
            return 1
    else:
        grid = open_floor(things)
        room = "open floor (invented: this recording carries no map)"

    if args.stand:
        stand = tuple(float(part) for part in args.stand.split(","))
    elif pose:
        stand = (float(pose["x_m"]), float(pose["y_m"]))
    else:
        newest = max(things, key=lambda one: one["last_seen_at"])
        stand = ((float(newest["looks"][-1]["x_m"]),
                  float(newest["looks"][-1]["y_m"]))
                 if newest["looks"] else
                 (float(newest["placement"]["x_m"]),
                  float(newest["placement"]["y_m"])))

    print("%d placed things, %d looks read, in %s"
          % (len(things), sum(len(one["looks"]) for one in things), room))
    print("the rover is standing at %.2f, %.2f\n" % stand)

    counted = {"no sight line": 0, "median": 0, "another line": 0, "ring": 0,
               "nowhere": 0}
    moved, spreads, extra = [], [], []
    for one in things:
        seen = approach.seen_from(one["placement"], one["looks"])
        middle = approach.middle_of(seen)
        was = approach.viewpoint(one["placement"], grid, stand)
        now = approach.viewpoint(one["placement"], grid, stand, one["looks"])
        if not seen:
            counted["no sight line"] += 1
        elif not now.get("ok"):
            counted["nowhere"] += 1
        elif now["along"].startswith("the line it has most"):
            counted["median"] += 1
        elif now["along"].startswith("a line"):
            counted["another line"] += 1
        else:
            counted["ring"] += 1
        if seen:
            spreads.append(max(abs(_turn(bearing - middle))
                               for bearing in seen))
        if was.get("ok") and now.get("ok"):
            apart = math.hypot(now["x_m"] - was["x_m"], now["y_m"] - was["y_m"])
            moved.append(apart)
            extra.append(now["travel_m"] - was["travel_m"])
        if args.detail:
            print("%-12s %2d looks %2d lines  median %7s  %s  moved %.2f m"
                  % (one["id"], one["observation_count"], len(seen),
                     "-" if middle is None else "%.1f" % middle,
                     now.get("along", now.get("why", "?"))[:44],
                     moved[-1] if moved and was.get("ok") and now.get("ok")
                     else 0.0))

    print("\nwhere the answer came from")
    for name, count in counted.items():
        print("  %-14s %3d" % (name, count))
    if spreads:
        spreads.sort()
        print("\nhow far a thing's looks spread around their own median, in "
              "degrees\n  median %.0f, worst %.0f, %d of %d within 20"
              % (spreads[len(spreads) // 2], spreads[-1],
                 sum(1 for one in spreads if one <= 20.0), len(spreads)))
    if moved:
        moved.sort()
        extra.sort()
        print("\nhow far the sight line moved the standing point, in metres\n"
              "  median %.2f, worst %.2f, %d of %d unchanged"
              % (moved[len(moved) // 2], moved[-1],
                 sum(1 for one in moved if one < 0.01), len(moved)))
        print("what that costs in driving, in metres\n"
              "  median %+.2f, worst %+.2f" % (extra[len(extra) // 2], extra[-1]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
