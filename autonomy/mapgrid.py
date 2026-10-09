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
import collections
import math
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
#
# `goal_fit.py` comes the same way and for the same reason: it is the bridge's
# own test of whether the body fits where a goal was set, and a goal this module
# calls reachable has to be one the bridge will accept.
def _from_ros_nav(name: str):
    try:
        return __import__(name)
    except ImportError:                                        # pragma: no cover
        import importlib.util

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            os.pardir, "ros_nav", name + ".py")
        spec = importlib.util.spec_from_file_location(name, path)
        if spec is None or spec.loader is None:
            raise
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        return module


frontier = _from_ros_nav("frontier")
goal_fit = _from_ros_nav("goal_fit")

#: How far the lidar is worth believing for "standing there would reveal this
#: much". The scanner reaches 8 m and the costmap marks obstacles to 6, but the
#: number wanted here is neither: it is how much new floor a frontier actually
#: buys in a house, where the next wall is the limit long before the sensor is.
#: Four metres is a room's width and it keeps the estimate conservative, which
#: is the direction an estimate that decides where to drive should err in.
SENSE_DEPTH_M = 4.0

#: How far from a cell's middle a goal's own point may sit, in cells, for the
#: body stamp in `_could_cover`.
SLACK_CELLS = 0.05


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


def body_of(payload: dict[str, Any]) -> tuple[float, float]:
    """The clearance a route needs, and how far a goal may be moved, from `nav_grid`.

    Both are the navigation bridge's own (`walking_body` in ros_nav/nav_moves.py):
    the distance from a wall the planner keeps the rover's centre, and how far
    the bridge moves a goal onto floor where the body fits before it plans.
    `(0.0, 0.0)` when the map came without them -- a bridge from before
    2026-10-05, or a costmap that has not said what the body is -- which is the
    walk as a point, the way this module walked before then.
    """
    def number(name: str) -> float:
        try:
            return max(0.0, float(payload.get(name) or 0.0))
        except (TypeError, ValueError):
            return 0.0
    return number("inscribed_radius_m"), number("goal_fit_reach_m")


class Reach:
    """Which floor the rover can walk to from where it stands, and how far.

    One breadth-first walk, held so that the two things that need it -- ranking
    frontiers, and finding somewhere to stand and look at a thing -- ask for it
    once between them. The walk is `frontier.py`'s, four-connected and
    conservative for its reason: an eight-connected walk slips diagonally
    through a 7 cm gap this rover is 30 cm too wide for.

    **It walks with the rover's body, not as a point.** Walked as a point over
    every free cell, it went through gaps the planner will not: Nav2 lays the
    rover's centre on its costmap and refuses any cell within `inscribed_m` of a
    wall, so a gap narrower than twice that is closed to it. Every frontier an
    autonomous run chose before 2026-10-05 -- seven, on two days -- was in a
    pocket by the charger reached only through a 30-40 cm gap; the walk called
    each about five metres away, and each failed, twice after Nav2 had found a
    38 m way round and driven off along it. So the walk crosses only free cells
    at least that far from anything occupied, which is the planner's own test
    for the centre of the body.

    **A goal is where the bridge will send the rover, not where it was set.**
    Before it plans, the bridge moves a goal to the nearest place within
    `goal_reach_m` where the whole body fits (`goal_fit.py`), and refuses it if
    there is none. That floor is narrower than the walk's: no part of the body
    may lie over the planner's 253 ring, so the centre keeps about twice the
    clearance a route needs. A goal counts as reachable only if that place
    exists and the walk reaches it. A frontier is always next to unknown ground
    and often next to a wall, so without the move nearly none would be.

    Replayed over every drive the autonomy record held on 2026-10-05, the body's
    walk kept all 183 that arrived and refused all seven frontiers in the
    pocket. Replayed again on 2026-10-06 over all 230 drives, the bridge's own
    fit refuses all ten goals the bridge refused for want of room for the body
    -- seven of which the walk alone had let through -- and keeps all 200 that
    arrived.

    `standing` is None when there is no such floor within half a metre of the
    rover -- a rover parked half under a sofa, or a pose that has drifted into a
    wall -- and every movement goal has to be refused while it lasts, because
    nothing here can plan a route out of a place the walk cannot start from.
    The bridge's own back-off looks the same half metre for a place to go.
    """

    def __init__(self, grid: "frontier.Grid", where: tuple[float, float],
                 inscribed_m: float = 0.0, goal_reach_m: float = 0.0) -> None:
        self.grid = grid
        self.where = where
        self.inscribed_m = inscribed_m
        self.goal_reach_m = goal_reach_m
        self.free, self.unknown = frontier.classify(grid)
        self.roomy = _clear_of_walls(grid, self.free, inscribed_m)
        self.standing = frontier.standing_on(grid, self.roomy, where)
        if self.standing is None:
            self.distance = [-1] * (grid.width * grid.height)
        else:
            walked = frontier.reachable_from(grid, self.roomy, self.standing)
            self.distance = _moved_by_the_bridge(grid, self.free, walked,
                                                 inscribed_m, goal_reach_m)

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
                   to_x: float, to_y: float, *,
                   ignore_within_m: float = 0.0) -> bool:
        """Is there a wall between these two points, as far as the map knows?

        A cell-by-cell walk refusing anything the mapper calls occupied, which is
        the cheap half of "could the camera see the thing from there". It cannot
        answer the other half -- a table the lidar never saw because it is above
        the scan plane hides nothing here and everything in the picture -- so a
        candidate that passes this is a viewpoint worth trying rather than one
        that has been shown to work. Unknown cells are allowed through: the thing
        being looked at is usually on ground the scanner has not painted, and
        refusing on unknown would refuse nearly every real viewpoint.

        `ignore_within_m` lets through the thing's own surface: a thing on a
        wall or a piece of furniture is itself on cells the map calls occupied,
        and so is the wall or cabinet it is on. Only the cells running straight
        back from the thing that are not free floor, and no further than that,
        are let through. Once the line has crossed free floor on its way to the
        thing, a wall before it is a wall in the way: a thing shut in a cupboard
        is still behind the cupboard's wall.
        """
        grid = self.grid
        col, row = grid.cell_of(from_x, from_y)
        last_col, last_row = grid.cell_of(to_x, to_y)
        steps = max(abs(last_col - col), abs(last_row - row))
        if steps == 0:
            return True
        cells = []
        for step in range(1, steps + 1):
            here_col = col + int(round((last_col - col) * step / steps))
            here_row = row + int(round((last_row - row) * step / steps))
            if (here_col, here_row) == (last_col, last_row):
                break
            cells.append((here_col, here_row))
        while cells and ignore_within_m > 0.0:
            here_col, here_row = cells[-1]
            if (math.hypot(here_col - last_col, here_row - last_row)
                    * grid.resolution > ignore_within_m
                    or 0 <= grid.at(here_col, here_row) <= frontier.FREE_AT):
                break
            cells.pop()
        return all(grid.at(here_col, here_row) < frontier.OCCUPIED_AT
                   for here_col, here_row in cells)

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


def _clear_of_walls(grid: "frontier.Grid", free: bytearray,
                    radius_m: float) -> bytearray:
    """The free cells whose centre is further than `radius_m` from anything occupied.

    The inflation layer's 253 ring, drawn on the occupancy grid: a cell is closed
    if an occupied cell's centre is within the radius of its own, measured
    centre to centre as the costmap measures it. The tolerance is for the map's
    resolution arriving as a 32-bit float -- four cells at 0.050000001 m is a
    hair over 0.2, and the costmap counts that cell inside the ring.
    """
    if radius_m <= 0.0:
        return free
    limit = radius_m / grid.resolution + 1e-3
    reach = int(limit)
    disc = [(dcol, drow) for drow in range(-reach, reach + 1)
            for dcol in range(-reach, reach + 1)
            if math.hypot(dcol, drow) <= limit]
    width, height = grid.width, grid.height
    out = bytearray(free)
    for here, value in enumerate(grid.data):
        if value < frontier.OCCUPIED_AT:
            continue
        row, col = divmod(here, width)
        for dcol, drow in disc:
            c, r = col + dcol, row + drow
            if 0 <= c < width and 0 <= r < height:
                out[r * width + c] = 0
    return out


def _moved_by_the_bridge(grid: "frontier.Grid", free: bytearray, walked: list,
                         inscribed_m: float, reach_m: float) -> list:
    """The walk, as the steps to wherever the bridge would send a goal at each cell.

    Every free cell is a goal somebody might set. The bridge sends one to the
    nearest place within `reach_m` where the body fits, in `goal_fit.fit`'s own
    order of trying, and the walk to that place, plus the cells it was moved,
    is what the goal costs. No such place, or one the walk does not reach --
    which is how the bridge sends a goal through a wall into a room the rover
    cannot get to -- and the cell is -1.

    The body is the circle the rover runs (`ros_nav/dwb_config.py`), at the
    inscribed radius the bridge sends, stamped as every cell it could touch at
    any heading (`_could_cover`). So a place called fit here is one the bridge
    would accept, and the bridge may accept a few this refuses -- the cheap side
    to be wrong on, since a refused goal costs a choice and a failed drive costs
    one of three failures in a row.
    """
    if inscribed_m <= 0.0 or reach_m <= 0.0:
        return walked
    fits = _where_the_body_fits(grid, inscribed_m)
    width, height, res = grid.width, grid.height, grid.resolution
    order = [(round(away / res), round(dx / res), round(dy / res))
             for away, dx, dy in goal_fit.candidates(
                 goal_fit.CostGrid(1, 1, res, 0.0, 0.0, b"\0"), 0.0, 0.0,
                 reach_m)]
    span = max(abs(dcol) for _shift, dcol, _drow in order)
    flat = [(shift, drow * width + dcol) for shift, dcol, drow in order]
    # Only a cell with walkable fitting floor within reach can come out
    # reachable, and the search is longest exactly where there is none, so those
    # are ruled out first, a row run at a time.
    runs: dict[int, list[int]] = {}
    for _shift, dcol, drow in order:
        low, high = runs.get(drow, (dcol, dcol))
        runs[drow] = (min(low, dcol), max(high, dcol))
    near = bytearray(len(walked))
    for here, steps in enumerate(walked):
        if steps < 0 or not fits[here]:
            continue
        row, col = divmod(here, width)
        for drow, (first, last) in runs.items():
            r = row + drow
            if 0 <= r < height:
                lo, hi = max(0, col + first), min(width - 1, col + last)
                near[r * width + lo:r * width + hi + 1] = b"\1" * (hi - lo + 1)
    out = [-1] * len(walked)
    for here, is_free in enumerate(free):
        if not is_free or not near[here]:
            continue
        row, col = divmod(here, width)
        if span <= col < width - span and span <= row < height - span:
            for shift, delta in flat:
                there = here + delta
                if fits[there]:
                    if walked[there] >= 0:
                        out[here] = walked[there] + shift
                    break
            continue
        for shift, dcol, drow in order:
            c, r = col + dcol, row + drow
            if not (0 <= c < width and 0 <= r < height):
                continue
            there = r * width + c
            if fits[there]:
                if walked[there] >= 0:
                    out[here] = walked[there] + shift
                break
    return out


def _could_cover(radius_cells: float, slack_cells: float = SLACK_CELLS
                 ) -> list[tuple[int, int]]:
    """Every cell a body of this radius could touch, centred on the middle cell.

    The bridge's body is a twelve-sided polygon with its corners on the circle,
    so nothing it covers lies outside the disc. Which edge cells it touches
    changes with a millimetre's shift or a few degrees' turn, so the stamp takes
    every cell the disc reaches with its centre up to `slack_cells` from the
    middle of the cell, which is never fewer than the bridge counts.
    """
    reach = int(math.ceil(radius_cells + slack_cells)) + 1
    out = []
    for drow in range(-reach, reach + 1):
        for dcol in range(-reach, reach + 1):
            gap_x = max(0.0, abs(dcol) - 0.5 - slack_cells)
            gap_y = max(0.0, abs(drow) - 0.5 - slack_cells)
            if math.hypot(gap_x, gap_y) < radius_cells:
                out.append((dcol, drow))
    return out


def _where_the_body_fits(grid: "frontier.Grid", inscribed_m: float) -> bytearray:
    """Cells where the bridge's goal fit would let the rover stand.

    `goal_fit.fits` on the costmap the planner would build from this map: the
    body may cover no occupied cell and no cell within `inscribed_m` of one (the
    253 ring, as `_clear_of_walls` draws it). The cells a body covers when it is
    centred on a cell are the same set wherever the cell is, so both rings fold
    into one stamp, laid once round every occupied cell as runs along each row --
    a per-cell polygon test would cost the Orin seconds per decision. Unknown and
    off-map ground do not block, as they do not for the bridge.
    """
    res = grid.resolution
    covered = _could_cover(inscribed_m / res)
    limit = inscribed_m / res + 1e-3
    reach = int(limit)
    ring = [(dcol, drow) for drow in range(-reach, reach + 1)
            for dcol in range(-reach, reach + 1) if math.hypot(dcol, drow) <= limit]
    # A cell is ruled out when an occupied cell lies within the ring of any cell
    # its body covers: the occupied cell minus that offset, for every pair.
    stamp: dict[int, set] = collections.defaultdict(set)
    for bcol, brow in covered:
        for rcol, rrow in ring:
            stamp[-(brow + rrow)].add(-(bcol + rcol))
    runs = []
    for drow, cols in sorted(stamp.items()):
        cols = sorted(cols)
        start = previous = cols[0]
        for dcol in cols[1:] + [None]:
            if dcol is not None and dcol == previous + 1:
                previous = dcol
                continue
            runs.append((drow, start, previous))
            if dcol is not None:
                start = previous = dcol
    width, height = grid.width, grid.height
    out = bytearray(b"\1") * (width * height)
    for here, value in enumerate(grid.data):
        if value < frontier.OCCUPIED_AT:
            continue
        row, col = divmod(here, width)
        for drow, first, last in runs:
            r = row + drow
            if not 0 <= r < height:
                continue
            lo, hi = max(0, col + first), min(width - 1, col + last)
            if lo <= hi:
                out[r * width + lo:r * width + hi + 1] = bytes(hi - lo + 1)
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


def unknown_m2(grid: "frontier.Grid") -> float:
    """How much floor the map still calls unknown, in square metres.

    One number, for the one question an executive asks after it has driven
    somewhere to see more of the house: is there less of the house left unseen
    than there was. It counts every unknown cell rather than only the reachable
    ones, deliberately -- the cheaper measure moves when the rover's *route*
    changes as well as when the map does, which would credit a drive with
    revealing ground that was always there.
    """
    _free, unknown = frontier.classify(grid)
    # A flag per cell rather than a list of them, so this is a sum and not a
    # length -- `len` here would answer with the whole map, every time, and
    # would look perfectly reasonable doing it.
    return sum(unknown) * grid.resolution * grid.resolution
