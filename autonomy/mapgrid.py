"""The occupancy map, and the rover's own chooser reading it.

**This module imports `frontier.py` rather than reimplementing it, and that is
the whole point of the file.** Which gap in the map is worth driving to is a
question the rover already answers -- `ros_nav/frontier.py` is what `explore`
ranks with -- and a second copy here would answer it plausibly, differently, and
invisibly. So the module is deployed into this component as well as into
`ros_nav/`, from one file in the repository, and both copies come off the same
commit; `deploy/manifest.json` is where that is arranged.

It is a module that can be imported here at all only because it was written to
be: no ROS, no numpy, and a `Grid` class with the arithmetic in it, so that
`ros_nav/selftest.py` can drive the policy on a workstation. This is the second
caller that property was worth having.

## Where the grid comes from

`nav_grid` on the daemon, which forwards the navigation bridge's own map: the
occupancy bytes as slam_toolbox published them, zlib'd and base64'd, plus the
resolution and where the grid's corner sits in the map frame. Raw rather than
rendered, for the reason the bridge gives for sending it that way -- the daemon
already has a renderer, and the alternative to shipping the numbers is a second
one that slowly disagrees with the first.

The bytes arrive unsigned because that is what fits in a byte, and unknown is
-1, so the decode below is not decoration: read as unsigned, every unknown cell
becomes 255, which is above the obstacle threshold, and the whole unexplored
half of the house reads as a wall. There would be no frontiers at all and
nothing would look broken.
"""
from __future__ import annotations

import base64
import os
import sys
import zlib
from typing import Any

# The rover has this beside us in ~/ugv/autonomy; a checkout has it in the
# component it belongs to. Both are the same file at the same commit.
#
# **The fallback loads that one file and adds no directory to the search path**,
# which is not fussiness: `ros_nav/` has its own `test_harness.py` and its own
# `selftest.py`, and putting it on the path would quietly hand this component
# somebody else's modules under names it uses itself. The failure looks like a
# test that cannot find its own helper, and it takes a while to see why.
try:
    import frontier
except ImportError:                                            # pragma: no cover
    import importlib.util

    _path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         os.pardir, "ros_nav", "frontier.py")
    _spec = importlib.util.spec_from_file_location("frontier", _path)
    if _spec is None or _spec.loader is None:
        raise
    frontier = importlib.util.module_from_spec(_spec)
    sys.modules["frontier"] = frontier
    _spec.loader.exec_module(frontier)

#: How far the lidar is worth believing for "standing there would reveal this
#: much". The scanner reaches 8 m and the costmap marks obstacles to 6, but the
#: number wanted here is neither: it is how much new floor a frontier actually
#: buys in a house, where the next wall is the limit long before the sensor is.
#: Four metres is a room's width and it keeps the estimate conservative, which
#: is the direction an estimate that decides where to drive should err in.
SENSE_DEPTH_M = 4.0


class NoMap(Exception):
    """There is no occupancy map to reason about. Not an error, a state."""


def grid_of(payload: dict[str, Any]) -> "frontier.Grid":
    """A `frontier.Grid` from what `nav_grid` returned.

    Raises `NoMap` rather than returning an empty grid, because "the mapper has
    not published yet" and "the house is one big unknown" must not read alike:
    the first is a rover that cannot be asked yet and the second is a rover with
    everything to explore.
    """
    if not payload or not payload.get("ok", True) or not payload.get("data"):
        raise NoMap(str((payload or {}).get("error")
                        or "the rover sent no occupancy map"))
    width = int(payload["width"])
    height = int(payload["height"])
    raw = zlib.decompress(base64.b64decode(payload["data"]))
    if len(raw) != width * height:
        raise NoMap(f"the map is {len(raw)} cells but says it is "
                    f"{width}x{height}")
    return frontier.Grid(width, height, float(payload["resolution_m"]),
                         float(payload["origin_x_m"]),
                         float(payload["origin_y_m"]),
                         [value - 256 if value > 127 else value
                          for value in raw])


class Reach:
    """Which floor the rover can walk to from where it stands, and how far.

    One breadth-first walk over cells the mapper calls confidently free, held so
    that the two things that need it -- ranking frontiers, and finding somewhere
    to stand and look at a thing -- ask for it once between them. The walk is
    `frontier.py`'s, four-connected and conservative for its reason: an
    eight-connected walk slips diagonally through a 7 cm gap this rover is
    30 cm too wide for.

    `standing` is None when the rover is not on floor the map calls free, which
    happens for real -- a rover parked half under a sofa, or a pose that has
    drifted into a wall -- and every movement goal has to be refused while it
    lasts, because nothing here can plan a route out of a place the walk cannot
    start from.
    """

    def __init__(self, grid: "frontier.Grid", where: tuple[float, float]) -> None:
        self.grid = grid
        self.where = where
        self.free, self.unknown = frontier.classify(grid)
        self.standing = frontier.standing_on(grid, self.free, where)
        self.distance = ([-1] * (grid.width * grid.height) if self.standing is None
                         else frontier.reachable_from(grid, self.free,
                                                      self.standing))

    def reachable(self, x: float, y: float) -> float | None:
        """How far the rover must walk over known floor to stand at a point.

        None when it cannot get there at all -- through a wall, or across ground
        nobody has seen -- which is the answer that vetoes a goal rather than
        one that ranks it badly.
        """
        col, row = self.grid.cell_of(x, y)
        if not self.grid.inside(col, row):
            return None
        steps = self.distance[row * self.grid.width + col]
        return None if steps < 0 else steps * self.grid.resolution

    def is_free(self, x: float, y: float) -> bool:
        """Does the mapper call this point confidently free floor?"""
        col, row = self.grid.cell_of(x, y)
        if not self.grid.inside(col, row):
            return False
        return bool(self.free[row * self.grid.width + col])

    def unknown_area_m2(self, x: float, y: float,
                        radius_m: float = SENSE_DEPTH_M) -> float:
        """How much unmapped floor lies within reach of a place, in square metres.

        **An upper bound and honestly so: there is no visibility test here.**
        Unknown ground behind the wall the rover would be standing against is
        counted, because ray-casting every cell of a disc for every candidate
        costs more than the ranking is worth. What keeps it from being silly is
        that it is used as a *cap* on the boundary-times-depth estimate rather
        than on its own -- a frontier that is one cell wide cannot reveal a room
        however much unknown ground is behind it.
        """
        grid = self.grid
        col, row = grid.cell_of(x, y)
        span = int(radius_m / grid.resolution)
        limit = span * span
        found = 0
        for dy in range(-span, span + 1):
            here = row + dy
            if not 0 <= here < grid.height:
                continue
            base = here * grid.width
            reach = int((limit - dy * dy) ** 0.5)
            for dx in range(-reach, reach + 1):
                there = col + dx
                if 0 <= there < grid.width and self.unknown[base + there]:
                    found += 1
        return found * grid.resolution * grid.resolution

    def clear_line(self, from_x: float, from_y: float,
                   to_x: float, to_y: float) -> bool:
        """Is there a wall between these two points, as far as the map knows?

        A cell-by-cell walk refusing anything the mapper calls occupied, which is
        the cheap half of "could the camera see the thing from there". It cannot
        answer the other half -- a table the lidar never saw because it is above
        the scan plane hides nothing here and everything in the picture -- so a
        candidate that passes this is a viewpoint worth trying rather than one
        that has been shown to work. Unknown cells are allowed through: the thing
        being looked at is usually on ground the scanner has not painted, and
        refusing on unknown would refuse nearly every real viewpoint.
        """
        grid = self.grid
        col, row = grid.cell_of(from_x, from_y)
        last_col, last_row = grid.cell_of(to_x, to_y)
        steps = max(abs(last_col - col), abs(last_row - row))
        if steps == 0:
            return True
        for step in range(1, steps + 1):
            here_col = col + int(round((last_col - col) * step / steps))
            here_row = row + int(round((last_row - row) * step / steps))
            if (here_col, here_row) == (last_col, last_row):
                break
            if grid.at(here_col, here_row) >= frontier.OCCUPIED_AT:
                return False
        return True

    def ring(self, x: float, y: float, near_m: float, far_m: float
             ) -> list[tuple[float, float, float]]:
        """Reachable places to stand between two distances of a point.

        `(x, y, walk_m)` per cell, nearest walk first and then by position, so
        that two runs over one map produce the same list in the same order --
        which is what makes a decision built on it replayable. Cells the rover
        cannot walk to are left out rather than ranked last: a viewpoint it
        cannot reach is not a worse viewpoint, it is not a viewpoint.
        """
        grid = self.grid
        col, row = grid.cell_of(x, y)
        span = int(far_m / grid.resolution) + 1
        out: list[tuple[float, float, float]] = []
        for dy in range(-span, span + 1):
            here = row + dy
            if not 0 <= here < grid.height:
                continue
            for dx in range(-span, span + 1):
                there = col + dx
                if not 0 <= there < grid.width:
                    continue
                if not self.free[here * grid.width + there]:
                    continue
                px, py = grid.point_of(there, here)
                gap = ((px - x) ** 2 + (py - y) ** 2) ** 0.5
                if not near_m <= gap <= far_m:
                    continue
                walk = self.distance[here * grid.width + there]
                if walk < 0:
                    continue
                out.append((px, py, walk * grid.resolution))
        out.sort(key=lambda one: (round(one[2], 3), round(one[0], 3),
                                  round(one[1], 3)))
        return out


def frontiers(grid: "frontier.Grid", where: tuple[float, float],
              min_frontier_m: float | None = None) -> tuple[list[dict], dict]:
    """Everywhere worth driving to next, best first, and what the map looks like.

    `frontier.survey`'s own answer, with no blacklist and no previous goal --
    both of those belong to a running `explore`, which is the loop that watches
    goals fail, and nothing here is running one.
    """
    return frontier.survey(grid, where,
                           min_frontier_m=(frontier.MIN_FRONTIER_M
                                           if min_frontier_m is None
                                           else min_frontier_m))


def unknown_share(summary: dict) -> float | None:
    """How much of the map is still unknown, as `frontier.py` measures it."""
    return frontier.unknown_share(summary)
