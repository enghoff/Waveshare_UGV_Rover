#!/usr/bin/env python3
"""Where the map stops, and which of those places is worth driving to.

Its own module, with no ROS in it, for the reason `goal_fit.py` and
`route_cost.py` are: the bridge runs this on the rover inside the conda
environment, and `selftest.py` runs it on a workstation that has no ROS at all.
A copy in the test would be a copy that drifts, and this one would drift
invisibly -- a frontier chooser that has quietly stopped agreeing with the one
on the rover still returns plausible-looking goals.

## What a frontier is here

A cell the mapper calls free, next to a cell it calls unknown. Stand there and
the lidar sees into the unknown, so the map grows. Everything below is about
which of them to drive to and which to leave alone.

**Reachability is decided first, and by walking rather than by measuring.** The
obvious ranking is straight-line distance, and on this rover it is wrong often
enough to matter: a house map is full of frontiers two metres away through a
wall and eleven metres away round the corridor. So the first pass is a breadth-
first walk out from the rover across cells the mapper has actually seen to be
free. It costs one sweep of the grid and pays for itself twice -- a frontier the
walk never reaches is dropped rather than ranked, and every frontier it does
reach arrives with the real distance round the furniture instead of through it.

**That walk is not the planner, and the difference is the whole reason the
bridge verifies its choice.** This counts cells; Nav2 plans with the rover's
inflated body, so a walk that squeezes through a 5 cm gap between a table leg
and a wall reports a frontier the rover cannot actually reach. The walk is the
cheap ranking of fifty candidates and the planner is the expensive check of the
one that won -- see `explore` in `nav_bridge.py`.

## What this deliberately does not do

**It does not check whether the rover's body fits at the goal.** `goal_fit.py`
does that, against the costmap the planner is about to use, with the footprint
read off the running costmap node. A second opinion here would be a second copy
of a measurement, which is the failure that module's docstring exists to
describe. So a candidate coming out of here is a *place worth going*, not a
place the rover has been shown to fit, and the bridge passes every one through
`fit_goal` before it sends it.

**It has no memory.** Blacklists and hysteresis are arguments, so that the loop
that owns them is the loop that can see what happened. Nothing here is stateful
between calls, which is what makes it testable against a map on disk.
"""

import collections
import gzip
import math

#: Occupancy values, as slam_toolbox publishes them: -1 unknown, otherwise a
#: probability in 0..100.
#:
#: Free is the *confident* end of that range rather than everything below the
#: obstacle threshold, and the asymmetry is deliberate. This walk decides where
#: the rover may drive, and a cell the mapper is halfway unsure about is not a
#: floor it should be routed across on the strength of a guess. The 65 that
#: divides obstacle from not is `map_score.py`'s and the ROS map server's, so
#: two files in this directory cannot come to different views of one grid.
FREE_AT = 25
OCCUPIED_AT = 65

#: How much boundary a frontier needs before it is worth a drive, in metres of
#: cells along it. Below this it is usually not a room at all -- it is the ragged
#: edge where two scans disagreed by a cell, or the shadow behind a chair leg,
#: and driving to it buys nothing the next scan from here would not have given
#: for free. Ten cells at 5 cm; a doorway is fifteen.
#:
#: **This number is the whole difference between exploring a house and tidying
#: one, and it was measured rather than guessed.** `explore_sim.py` runs the
#: policy round the map the recorded `kitchen-loop` drive produced:
#:
#:      threshold   goals   driven   floor found
#:        0.30 m      23     99.8 m     99.9%
#:        0.50 m      16     82.4 m     99.6%
#:        0.75 m      10     65.8 m     98.3%
#:        1.00 m       6     54.8 m     94.7%
#:
#: Four goals get 93% of that house. Everything after them is chasing slivers,
#: and at 0.30 m nineteen of the twenty-three goals and seventy of the hundred
#: metres buy the last 0.3% -- which at this rover's speed is more than the ten
#: minutes an `explore` is given, so the run would end on its budget having left
#: something real unexplored while it perfected a corner. 0.50 m keeps
#: essentially all of the coverage for four-fifths of the driving. The tail is
#: not lost either way: what is left is reported, and a second `explore` starts
#: with an empty blacklist.
MIN_FRONTIER_M = 0.50

#: What a metre of driving costs against what a metre of new boundary is worth.
#: `explore_lite`'s two scales under different names, and the ratio is what
#: matters: at 1.0 and 2.0 the rover will cross three metres of floor it has
#: already seen to reach a frontier two metres wider than the one at its feet.
#: That is the trade wanted in a house, where the near frontier is usually the
#: last unseen corner of the room it is standing in and the far one is the next
#: room.
DISTANCE_WEIGHT = 1.0
SIZE_WEIGHT = 2.0

#: Sticking with the frontier already being driven to, expressed as the metres
#: of head start a candidate near the previous goal is given.
#:
#: **Without it the rover dithers, and the dithering is not a tuning problem.**
#: Every goal that completes redraws the map, which reorders the candidates,
#: and two frontiers within a few tenths of each other in cost will trade places
#: as the rover approaches either one. What that looks like on the floor is a
#: rover that turns round halfway across a room, and then turns round again.
#: A metre of head start is more than the reordering noise and less than the
#: distance to anything in another room.
HYSTERESIS_M = 1.0

#: How near a blacklisted point a candidate has to be before it counts as the
#: same frontier. Half a metre is the width of the rover plus a little: two
#: goals closer together than that are the same doorway, and having failed to
#: get through it once there is no sense in queueing the cell next door.
BLACKLIST_M = 0.5


class Grid(object):
    """An occupancy grid, with the arithmetic to index it.

    Deliberately not `goal_fit.CostGrid`, which looks almost identical and means
    something else: that one holds *costs*, where 255 is unknown and 253 is a
    cell the body may not be laid over, and this one holds *occupancy*, where -1
    is unknown and there are no body semantics at all. One class serving both
    would need a flag saying which convention its numbers were in, and a flag
    like that is read wrongly exactly once.

    Deliberately not numpy either, for `goal_fit.py`'s reason: this has to run
    in the selftest on any machine, and the two sweeps below are a few tenths of
    a second on the real map at 5 cm.
    """

    def __init__(self, width, height, resolution, origin_x, origin_y, data):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.data = data

    def cell_of(self, x, y):
        """The (column, row) a point in the map frame lands in."""
        return (int(math.floor((x - self.origin_x) / self.resolution)),
                int(math.floor((y - self.origin_y) / self.resolution)))

    def point_of(self, col, row):
        """The centre of a cell, in the map frame.

        The half-cell is not decoration: a goal at a cell's corner is up to 3.5 cm
        from where the cell actually is, and the arrival tolerance this rover
        drives to is not so wide that a systematic bias in one corner's direction
        is free.
        """
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def inside(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height

    def at(self, col, row):
        """The occupancy at a cell; off the edge reads as unknown.

        Off the edge genuinely is unknown -- the grid is only as big as the map so
        far, and it grows as the rover drives -- so this is the honest answer
        rather than a convenience. It also means the frontier detector finds the
        edge of the grid, which is correct: past it is somewhere nobody has been.
        """
        if not self.inside(col, row):
            return -1
        return self.data[row * self.width + col]


def classify(grid):
    """Two flags per cell: is it confidently free, is it unknown.

    Done once, into flat `bytearray`s, because both sweeps below ask these
    questions of every cell several times and re-deriving them from the raw
    occupancy each time roughly triples the work.
    """
    free = bytearray(grid.width * grid.height)
    unknown = bytearray(grid.width * grid.height)
    for i, value in enumerate(grid.data):
        if value < 0:
            unknown[i] = 1
        elif value <= FREE_AT:
            free[i] = 1
    return free, unknown


def reachable_from(grid, free, start_cell):
    """How far every free cell is from the rover, in cells, walking free ground.

    Returns a list with -1 in every cell the rover cannot walk to. Four-connected
    rather than eight, and that is the conservative choice on purpose: an
    eight-connected walk slips diagonally between two cells that touch at a
    corner, which on a 5 cm grid is a 7 cm gap the rover is 30 cm too wide for.
    """
    size = grid.width * grid.height
    distance = [-1] * size
    col0, row0 = start_cell
    if not grid.inside(col0, row0) or not free[row0 * grid.width + col0]:
        return distance
    start = row0 * grid.width + col0
    distance[start] = 0
    queue = collections.deque((start,))
    width, height = grid.width, grid.height
    while queue:
        here = queue.popleft()
        step = distance[here] + 1
        row, col = divmod(here, width)
        if col > 0 and free[here - 1] and distance[here - 1] < 0:
            distance[here - 1] = step
            queue.append(here - 1)
        if col + 1 < width and free[here + 1] and distance[here + 1] < 0:
            distance[here + 1] = step
            queue.append(here + 1)
        if row > 0 and free[here - width] and distance[here - width] < 0:
            distance[here - width] = step
            queue.append(here - width)
        if row + 1 < height and free[here + width] and distance[here + width] < 0:
            distance[here + width] = step
            queue.append(here + width)
    return distance


def standing_on(grid, free, where, look_m=0.5):
    """The cell to start the walk from, which is not always the one under the rover.

    Usually it is: the rover is on floor it has driven over and the mapper agrees.
    The two times it is not are worth handling rather than failing on, because
    both are ordinary. Just after a `clear_map` the graph is empty and the rover
    stands in a cell nothing has been said about yet; and a rover that has nosed
    into something can be sitting in a cell the mapper has painted as an
    obstacle. In both cases there is free ground within half a metre, and
    starting the walk there gives an answer instead of an error.

    Returns None when there is genuinely no free ground nearby, which means the
    map is empty and the honest reply is that there is nothing to explore yet.
    """
    col0, row0 = grid.cell_of(where[0], where[1])
    if grid.inside(col0, row0) and free[row0 * grid.width + col0]:
        return (col0, row0)
    span = int(math.ceil(look_m / grid.resolution))
    best = None
    for drow in range(-span, span + 1):
        for dcol in range(-span, span + 1):
            col, row = col0 + dcol, row0 + drow
            if not grid.inside(col, row) or not free[row * grid.width + col]:
                continue
            away = math.hypot(dcol, drow)
            if best is None or away < best[0]:
                best = (away, col, row)
    return None if best is None else (best[1], best[2])


def frontier_cells(grid, free, unknown, distance):
    """Every reachable free cell with unknown ground next to it.

    Four-connected again, and here it changes what gets found rather than only
    how far away it is: a free cell touching unknown only at a corner is not a
    place the lidar can see through, it is the far side of a wall's end.
    """
    width, height = grid.width, grid.height
    out = []
    for row in range(height):
        base = row * width
        for col in range(width):
            here = base + col
            if not free[here] or distance[here] < 0:
                continue
            if (col > 0 and unknown[here - 1]) or \
               (col + 1 < width and unknown[here + 1]) or \
               (row > 0 and unknown[here - width]) or \
               (row + 1 < height and unknown[here + width]) or \
               col == 0 or col + 1 == width or row == 0 or row + 1 == height:
                out.append(here)
    return out


def unknown_bearing(grid, unknown, cells):
    """Which way the unknown lies from a clump of frontier cells, in radians.

    This becomes the goal's heading, so that the rover arrives facing what it
    came to look at rather than with its back to it. It matters more on this
    rover than on most: the lidar is the only thing aboard that sees where it is
    going, and an arrival heading is free -- Nav2 turns to it as the last act of
    the goal either way, so it may as well be turning towards the new room.

    Summed as a vector over every unknown neighbour of every cell in the clump,
    which handles the awkward shapes correctly: a frontier wrapped round the
    inside of a doorway points through the doorway, not along the wall.
    """
    width = grid.width
    dx = dy = 0.0
    for here in cells:
        row, col = divmod(here, width)
        for dcol, drow in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            col2, row2 = col + dcol, row + drow
            if not grid.inside(col2, row2) or unknown[row2 * width + col2]:
                dx += dcol
                dy += drow
    if dx == 0.0 and dy == 0.0:
        return None
    return math.atan2(dy, dx)


def clump(grid, cells):
    """Frontier cells grouped into the frontiers they belong to.

    Eight-connected, unlike everything else here, and the difference is not an
    oversight. This is not deciding where the rover may drive -- it is deciding
    whether two cells describe the same gap in the map, and a boundary that runs
    diagonally across the grid is one boundary. Four-connected clumping chops
    every diagonal frontier into a string of single cells, all of which are then
    thrown away for being under `MIN_FRONTIER_M`, and the rover stands in a room
    with an open door reporting that there is nothing left to explore.
    """
    width = grid.width
    members = set(cells)
    out = []
    while members:
        seed = members.pop()
        group = [seed]
        queue = collections.deque((seed,))
        while queue:
            here = queue.popleft()
            row, col = divmod(here, width)
            for drow in (-1, 0, 1):
                for dcol in (-1, 0, 1):
                    col2, row2 = col + dcol, row + drow
                    if not grid.inside(col2, row2):
                        continue
                    other = row2 * width + col2
                    if other in members:
                        members.discard(other)
                        group.append(other)
                        queue.append(other)
        out.append(group)
    return out


def survey(grid, where, min_frontier_m=MIN_FRONTIER_M, blacklist=(),
           previous=None, distance_weight=DISTANCE_WEIGHT,
           size_weight=SIZE_WEIGHT, hysteresis_m=HYSTERESIS_M,
           blacklist_m=BLACKLIST_M):
    """Everywhere worth driving to next, best first.

    `where` is the rover's `(x, y)` in the map frame; `blacklist` is the points
    already tried and failed, and `previous` the goal currently being driven to,
    if any. Both belong to the caller because the loop that watches goals fail is
    the only thing that knows which ones did.

    Each candidate is a dict:

        x, y        where to send the rover, on a cell the mapper calls free
        yaw         facing the unknown, so it arrives looking at the new ground
        size_m      how much boundary this frontier has, in metres of cells
        distance_m  how far the rover must walk over known floor to reach it
        cost        what it is ranked on; lower is better
        cells       how many cells the frontier is made of

    Plus a summary dict of what the map currently looks like, because the caller
    has to say when it has finished and "no candidates" on its own does not
    distinguish a finished house from a mapper that has stopped publishing.
    """
    free, unknown = classify(grid)
    known_free = sum(free)
    summary = {"free_cells": known_free,
               "unknown_cells": sum(unknown),
               "reachable_cells": 0,
               "frontiers": 0,
               "rejected_small": 0,
               "rejected_blacklisted": 0}
    if not known_free:
        return [], summary

    start = standing_on(grid, free, where)
    if start is None:
        return [], summary

    distance = reachable_from(grid, free, start)
    summary["reachable_cells"] = sum(1 for d in distance if d >= 0)

    cells = frontier_cells(grid, free, unknown, distance)
    groups = clump(grid, cells)

    out = []
    for group in groups:
        size_m = len(group) * grid.resolution
        if size_m < min_frontier_m:
            summary["rejected_small"] += 1
            continue

        # The goal is the member cell nearest the clump's centre of mass, not
        # the centre of mass itself. A frontier bent round a doorway or the
        # inside of a corner has its centroid in the wall, and a goal there is
        # the exact fault `goal_fit.py` was written for -- worth not creating in
        # the first place rather than relying on being rescued from.
        mean_col = sum(here % grid.width for here in group) / float(len(group))
        mean_row = sum(here // grid.width for here in group) / float(len(group))
        best = min(group, key=lambda here: (
            (here % grid.width - mean_col) ** 2
            + (here // grid.width - mean_row) ** 2))
        col, row = best % grid.width, best // grid.width
        x, y = grid.point_of(col, row)

        if any(math.hypot(x - bx, y - by) <= blacklist_m
               for bx, by in blacklist):
            summary["rejected_blacklisted"] += 1
            continue

        distance_m = distance[best] * grid.resolution
        cost = distance_weight * distance_m - size_weight * size_m
        if previous is not None and \
                math.hypot(x - previous[0], y - previous[1]) <= hysteresis_m:
            cost -= distance_weight * hysteresis_m

        yaw = unknown_bearing(grid, unknown, group)
        if yaw is None:
            yaw = math.atan2(y - where[1], x - where[0])

        out.append({"x": x, "y": y, "yaw": yaw,
                    "size_m": round(size_m, 2),
                    "distance_m": round(distance_m, 2),
                    "cost": round(cost, 3),
                    "cells": len(group)})

    out.sort(key=lambda c: c["cost"])
    summary["frontiers"] = len(out)
    return out, summary


#: How long a goal may go nowhere before an explore abandons it, and how far
#: "somewhere" is.
#:
#: **Measured against the two recorded drives where this rover was stuck.** Over
#: the whole minute of `recordings/trap-2026-08-25-spin.json` -- 1.22 m of path,
#: 3038 degrees of turning, 28 cm between the ends -- the furthest the rover got
#: from where it had been 20 seconds earlier is 0.20 m, and on
#: `corridor-2026-08-25-spin.json` it is 0.26 m. The two doorway recordings, which
#: are the same rover driving properly, reach 1.58 m and 3.63 m over the same
#: window. Half a metre sits in the gap with room on both sides; at the 0.33 m/s
#: this chassis cannot go below, 25 seconds of real driving is eight metres of
#: path, and no route round furniture nets less than half a metre of it.
#:
#: 25 seconds rather than 15 so that Nav2's own progress checker, which is given
#: 15, always gets to go first.
STALL_PATIENCE_S = 25.0
STALL_MOVED_M = 0.5


class Stall(object):
    """Is this goal going anywhere, or is the rover turning on the spot for ever?

    **The fault this watches for is the repository's oldest open one, and Nav2
    cannot see it.** `MapGridCritic`'s flood runs through walls in the build
    installed here, so where a route bends round a corner the controller is aimed
    at a point behind the wall, every forward sample is refused by the obstacle
    critic, and pivoting is free -- see "The controller is aimed round the corner"
    in the README. What makes it invisible from inside Nav2 is the progress
    checker: this rover runs `PoseProgressChecker` precisely so that a legitimate
    pivot is not called stuck, and the cost of that is that a rover pivoting back
    and forth for ever is not called stuck either. Recorded on the rover on
    2026-09-01: fifty seconds, six centimetres, forty-three replans, and not one
    recovery attempted.

    **Why exploring gets to give up where `drive_to` does not.** A person who
    asked for one particular place wants every recovery Nav2 has before being
    told no. An explore has sixteen other frontiers and no opinion about which,
    so a goal that has gone nowhere for half a minute is worth abandoning: it
    costs one frontier and saves the budget.

    **A rover that Nav2 is actively recovering is left alone.** The ladder --
    clear the costmaps, spin, wait -- works against the thing it is for, which is
    a rover stuck against something. Any change in the recovery count moves the
    anchor, so this only fires where nothing is being attempted at all, which is
    exactly the aiming trap.
    """

    def __init__(self, patience_s=STALL_PATIENCE_S, moved_m=STALL_MOVED_M):
        self.patience_s = patience_s
        self.moved_m = moved_m
        self.anchor = None
        self.anchor_at = 0.0
        self.recoveries = 0

    def update(self, now, where, recoveries=0):
        """Called as the goal runs. Returns a sentence when it should be given up.

        `where` may be None -- the transform tree can go quiet for a moment -- and
        a position nobody can vouch for must not be read as a rover that has not
        moved. It holds the anchor where it is and waits for the next one.
        """
        if where is None:
            return None
        if self.anchor is None:
            self.anchor, self.anchor_at = where, now
            return None
        moved = math.hypot(where[0] - self.anchor[0], where[1] - self.anchor[1])
        if moved >= self.moved_m or recoveries != self.recoveries:
            self.anchor, self.anchor_at = where, now
            self.recoveries = recoveries
            return None
        if now - self.anchor_at < self.patience_s:
            return None
        return ("it has not got %.1f m further on in %.0f seconds and Nav2 is not "
                "recovering, so it is turning on the spot rather than going "
                "anywhere" % (self.moved_m, now - self.anchor_at))


class Explorer(object):
    """One exploring run's memory: what has been tried, and where to go next.

    A class rather than three locals in the bridge, and the reason is the same
    one that put `survey` in a module with no ROS in it. Two things run this
    policy -- `explore` in `nav_bridge.py` on the rover, and `explore_sim.py` at a
    desk against a room the rover has actually driven -- and if the simulation
    kept its own copy of the rules it would be a simulation of a policy nobody
    ships. The bridge supplies the map, the planner and the wheels; everything
    about *which* frontier and *whether there are any left* is here.

    **Why every frontier is written off as soon as it is driven to, and not only
    when it fails.** A failure is obvious. An arrival is the case that makes the
    loop terminate: the controller stops within its 22 cm tolerance, and if the
    lidar did not see round the corner from there the frontier is still on the
    map and still the nearest one -- so a loop that only wrote off failures would
    drive the same 30 cm for the rest of its budget. What it costs is a pocket
    left unexplored behind a corner the rover stood next to, and the way to pick
    that up is another run, because this memory lasts one call and no longer.
    """

    def __init__(self, min_frontier_m=MIN_FRONTIER_M, blacklist_m=BLACKLIST_M,
                 hysteresis_m=HYSTERESIS_M):
        self.min_frontier_m = min_frontier_m
        self.blacklist_m = blacklist_m
        self.hysteresis_m = hysteresis_m
        self.blacklist = []
        self.previous = None
        self.summary = {}

    def choose(self, grid, where):
        """Everywhere still worth driving to, best first, minus what has been tried."""
        found, self.summary = survey(
            grid, where, min_frontier_m=self.min_frontier_m,
            blacklist=self.blacklist, previous=self.previous,
            hysteresis_m=self.hysteresis_m, blacklist_m=self.blacklist_m)
        return found

    def wrote_off(self, x, y):
        """This one is not worth offering again this run.

        Used for a frontier the rover cannot fit at, one the planner will not
        route to, and one it has already been sent to. See the class docstring
        for why the third belongs in the same list as the first two.
        """
        self.blacklist.append((float(x), float(y)))

    def committed(self, x, y):
        """The rover is being sent here: remember it, and prefer its neighbours.

        Both halves matter and they pull in opposite directions, which is the
        point. `previous` gives whatever is near this goal a head start next
        round, so the rover finishes the room it is in rather than being pulled
        across the house by a frontier that is a few centimetres wider. The
        blacklist stops that head start from becoming a rover driving to the same
        place for ever.
        """
        self.previous = (float(x), float(y))
        self.wrote_off(x, y)

    def tried(self):
        return len(self.blacklist)


def unknown_share(summary):
    """How much of the map is still unknown, as a fraction, or None.

    Reported rather than acted on. It is the number a person watching wants --
    "still a third of it left" -- and it is a bad stopping rule, because a map
    that grows as the rover drives has a denominator that grows with it: a rover
    that has just found a corridor can drive for a minute and see the fraction go
    *up*. What stops the loop is running out of reachable frontiers.

    **Asked with `.get` rather than by subscript, and that is a fix rather than a
    style.** An `explore` that ends before it has looked at the map once -- out of
    budget, stopped, no map yet -- has no summary to report, and this used to
    raise `KeyError: 'free_cells'` while building the sentence that says so. On
    the rover the whole outcome was then lost and the caller was left holding an
    open socket with nothing coming down it, which reads as a bridge that has
    hung rather than a run that never started.
    """
    total = summary.get("free_cells", 0) + summary.get("unknown_cells", 0)
    if not total:
        return None
    return summary["unknown_cells"] / float(total)


# --- reading a map off disk, so this is runnable without a rover ---------------
#
# `map_score.py` writes the grid the mapper ended up with as a PGM, and
# `replay_bag.sh` writes one for every replayed drive. That makes a real map from
# a real drive a file, and a file is something the chooser can be argued with at
# a desk:
#
#     python3 frontier.py fixtures/kitchen-loop.pgm.gz --at 6.3,8.7
#
# The convention is the ROS map server's, which is the one `map_score.py` writes:
# 0 is an obstacle, 205 unknown, 254 free, and the rows run top-down where a ROS
# grid's run bottom-up. Getting that flip wrong produces a map that looks
# entirely plausible upside down, which is why it is done here once rather than
# at each call site.

PGM_UNKNOWN = 205
PGM_OCCUPIED_AT = 64


def read_pgm(path, resolution=0.05, origin=(0.0, 0.0)):
    """A `Grid` from a PGM written by `map_score.py`, gzipped or not.

    The origin defaults to (0, 0) because a PGM does not carry one -- the ROS
    map server keeps it in a separate YAML. That is fine for everything this is
    used for, which is asking where the frontiers in a saved map are relative to
    each other; it is not a substitute for the live grid, which knows.
    """
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rb") as fh:
        raw = fh.read()
    magic, dimensions, _depth, pixels = raw.split(b"\n", 3)
    if magic != b"P5":
        raise ValueError("%s is not a binary PGM" % (path,))
    width, height = (int(v) for v in dimensions.split())
    if len(pixels) != width * height:
        raise ValueError("%s: %d pixels for a %dx%d image"
                         % (path, len(pixels), width, height))
    data = [0] * (width * height)
    for row in range(height):
        src = row * width
        dst = (height - 1 - row) * width
        for col in range(width):
            value = pixels[src + col]
            data[dst + col] = (100 if value <= PGM_OCCUPIED_AT
                               else -1 if value == PGM_UNKNOWN else 0)
    return Grid(width, height, resolution, origin[0], origin[1], data)


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("map", help="a PGM written by map_score.py, .gz allowed")
    ap.add_argument("--at", help="where the rover is, as X,Y in metres; "
                                 "defaults to the middle of the known floor")
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--min-frontier", type=float, default=MIN_FRONTIER_M)
    args = ap.parse_args(argv)

    grid = read_pgm(args.map, args.resolution)
    free, _unknown = classify(grid)
    if args.at:
        where = tuple(float(v) for v in args.at.split(","))
    else:
        seen = [i for i, f in enumerate(free) if f]
        if not seen:
            print("nothing in %s is mapped free" % args.map)
            return 1
        where = grid.point_of(
            int(sum(i % grid.width for i in seen) / len(seen)),
            int(sum(i // grid.width for i in seen) / len(seen)))

    found, summary = survey(grid, where, min_frontier_m=args.min_frontier)
    share = unknown_share(summary)
    print("%s: %.1f x %.1f m at %.0f cm, rover at %.2f, %.2f"
          % (args.map, grid.width * grid.resolution,
             grid.height * grid.resolution, grid.resolution * 100,
             where[0], where[1]))
    print("  %d free cells, %d of them reachable, %d unknown (%.0f%% of the map)"
          % (summary["free_cells"], summary["reachable_cells"],
             summary["unknown_cells"], 100.0 * (share or 0.0)))
    print("  %d frontiers worth driving to, %d too small, %d blacklisted"
          % (summary["frontiers"], summary["rejected_small"],
             summary["rejected_blacklisted"]))
    for rank, candidate in enumerate(found, 1):
        print("  %2d. %6.2f m away, %5.2f m of boundary, facing %4.0f deg, "
              "at (%6.2f, %6.2f), cost %7.2f"
              % (rank, candidate["distance_m"], candidate["size_m"],
                 math.degrees(candidate["yaw"]), candidate["x"], candidate["y"],
                 candidate["cost"]))
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
