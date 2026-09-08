#!/usr/bin/env python3
"""Run the exploring policy round a room the rover has actually driven.

    python3 explore_sim.py fixtures/kitchen-loop.pgm.gz
    python3 explore_sim.py fixtures/kitchen-loop.pgm.gz --picture /tmp/run

**What this is for.** `explore` on the rover gives itself new work for ten
minutes at a time, which makes two questions worth answering before it is ever
let loose: does it stop, and does it get round the house? Neither is answerable
from the rover in less than an afternoon per attempt, and both are answerable
here in about a second.

**The room is a real one, and that is the whole point of it.** The floor plan is
the occupancy grid `slam_toolbox` produced from the recorded `kitchen-loop`
drive -- the same map `replay_bag.sh` writes and `map_score.py` scores. So the
doorways are real doorways at their real widths, the furniture is where the
furniture was, and the awkward diagonal wall that generates a hundred one-cell
frontiers is in it. A room invented for the test would have proved that the
policy works in an invented room, which is the mistake this repository has
already paid for twice: see "The simulation that could not fail" in the README.

**What is faithful, and what is not.** The choosing is not a model of the rover's
choosing -- it *is* the rover's choosing, `frontier.Explorer` imported from the
same file the bridge imports it from, blacklist rule and all. What is modelled,
and modelled crudely, is everything below it:

    the lidar     360 rays to 8 m, stopping at the first wall. The real one
                  misses thin chair legs and sees through the gap under a sofa.
    the mapper    a revealed cell is free or wall for ever. slam_toolbox
                  changes its mind, and its map moves under a loop closure.
    the driving   a straight walk along known-free cells. Nav2 plans with an
                  inflated body and will refuse gaps this walks through, which
                  is exactly what `route_to` in the bridge exists to catch --
                  and this has no equivalent of it.

So the coverage number below is an optimistic bound, not a prediction: it says
the policy is capable of getting round this house, not that the rover will.
**Termination is the result to trust**, because every reason a real run would
stop early is missing here, so a policy that fails to stop in this room would
certainly fail to stop in the real one.
"""

import argparse
import collections
import math
import struct
import sys
import zlib

import frontier

#: What the lidar can see, in metres. `lidar_node.py --range-max` and
#: slam_toolbox's `max_laser_range` are both 8.0, and it is the smaller of the
#: two that decides how far the map grows from one place.
SIGHT_M = 8.0

#: Rays per revolution. The real sensor bins to about 450 usable points; 360 is
#: one a degree, which at 8 m leaves 14 cm between neighbouring rays -- a gap
#: wider than a cell, so a few cells past 6 m are missed the way real ones are.
RAYS = 360

#: How far the rover gets before the mapper has another look. The real one maps
#: continuously; this reveals every few cells, which is the same thing sampled.
#: `minimum_travel_distance` in slam_toolbox.yaml is 0.2 m.
STEP_M = 0.2

#: The most goals a run may take before this gives up and says so. Not a budget
#: -- the point of the exercise is that the policy stops on its own -- but a
#: backstop, so that a policy that does not stop fails the test in a second
#: instead of hanging it.
MAX_GOALS = 200


class Room(object):
    """The floor plan the rover is being driven round: what is really there.

    Three states, and the third is the one worth explaining. `WALL` and `FLOOR`
    are what they sound like. `OUTSIDE` is a cell the recorded drive never saw,
    and it is treated as wall -- not because there is a wall there, but because
    this simulation cannot say what is there and a guess would be the invented
    geometry the whole exercise is avoiding. The consequence is that "explored
    the whole room" here means the part of the house the rover really did see.
    """

    WALL, FLOOR, OUTSIDE = 0, 1, 2

    def __init__(self, grid):
        self.width, self.height = grid.width, grid.height
        self.resolution = grid.resolution
        self.origin_x, self.origin_y = grid.origin_x, grid.origin_y
        self.truth = bytearray(self.width * self.height)
        for i, value in enumerate(grid.data):
            self.truth[i] = (self.OUTSIDE if value < 0
                             else self.FLOOR if value <= frontier.FREE_AT
                             else self.WALL)

    def solid(self, col, row):
        """Does a ray stop here? Off the plan counts as solid."""
        if not (0 <= col < self.width and 0 <= row < self.height):
            return True
        return self.truth[row * self.width + col] != self.FLOOR

    def floor_cells(self):
        return sum(1 for v in self.truth if v == self.FLOOR)


def blank_map(room):
    """The grid the rover starts with: everything unknown, nothing believed."""
    return frontier.Grid(room.width, room.height, room.resolution,
                         room.origin_x, room.origin_y,
                         [-1] * (room.width * room.height))


def reveal(room, seen, col, row):
    """Cast the lidar from one cell and write what it finds into the map.

    Every ray marches until it meets something solid, marks that cell as a wall
    and stops. Cells it passes through become free.

    Marching in cell steps rather than in metres, because the two disagree in a
    way that matters at this resolution: a ray stepped at 5 cm through a wall met
    at 45 degrees can step straight over the corner of the cell it should have
    stopped in, leaving a one-cell hole that becomes a frontier the rover then
    drives to and cannot see anything from. Half-cell steps close it.
    """
    reach = SIGHT_M / room.resolution
    for ray in range(RAYS):
        angle = 2.0 * math.pi * ray / RAYS
        dx, dy = math.cos(angle) * 0.5, math.sin(angle) * 0.5
        x, y = col + 0.5, row + 0.5
        for _ in range(int(reach * 2.0)):
            x += dx
            y += dy
            here_col, here_row = int(math.floor(x)), int(math.floor(y))
            if not (0 <= here_col < room.width and 0 <= here_row < room.height):
                break
            index = here_row * room.width + here_col
            if room.truth[index] == room.FLOOR:
                seen.data[index] = 0
            else:
                # A cell the drive never saw is drawn as a wall, because that is
                # what it behaves like here -- but it is not one, and the
                # difference shows up as a map with a hard edge where the real
                # one has an open door. Said once here rather than silently.
                seen.data[index] = 100
                break


def path_over(seen, from_cell, to_cell):
    """Cells from here to there over ground the map already calls free.

    The same breadth-first walk the chooser ranks frontiers with, run once more
    to get the route rather than the distance -- so a goal this cannot reach is
    a goal the chooser should not have offered, and a disagreement between the
    two is a bug in one of them rather than a quirk of the simulation.

    This is emphatically not Nav2. It walks as a point, so it goes through gaps
    the rover's inflated body would not fit; the bridge's `route_to` is what
    catches those on the rover, and there is no equivalent here.
    """
    free, _unknown = frontier.classify(seen)
    distance = frontier.reachable_from(seen, free, from_cell)
    width = seen.width
    end = to_cell[1] * width + to_cell[0]
    if not seen.inside(*to_cell) or distance[end] < 0:
        return None
    route = [end]
    here = end
    while distance[here] > 0:
        row, col = divmod(here, width)
        for other in (here - 1 if col > 0 else -1,
                      here + 1 if col + 1 < width else -1,
                      here - width if row > 0 else -1,
                      here + width if row + 1 < seen.height else -1):
            if other >= 0 and distance[other] == distance[here] - 1:
                here = other
                break
        else:
            return None
        route.append(here)
    route.reverse()
    return route


def run(room, start, min_frontier_m=frontier.MIN_FRONTIER_M, verbose=True):
    """Explore the room from `start`, and report what happened.

    The loop is `explore` in `nav_bridge.py` with the ROS taken out: choose,
    write off, drive, repeat. Every decision in it comes from the shared
    `frontier.Explorer`, so the two cannot disagree about the policy -- only
    about the rover.
    """
    seen = blank_map(room)
    explorer = frontier.Explorer(min_frontier_m=min_frontier_m)
    col, row = room_cell(room, start)
    reveal(room, seen, col, row)

    step_cells = max(1, int(round(STEP_M / room.resolution)))
    metres = 0.0
    goals = arrived = blocked = 0
    visited = [(col, row)]

    while goals < MAX_GOALS:
        where = seen.point_of(col, row)
        found = explorer.choose(seen, where)
        if not found:
            return {"reason": "finished", "goals": goals, "arrived": arrived,
                    "blocked": blocked, "metres": metres,
                    "seen": seen, "visited": visited,
                    "summary": explorer.summary}

        candidate = found[0]
        goals += 1
        explorer.committed(candidate["x"], candidate["y"])
        target = room_cell(room, (candidate["x"], candidate["y"]))
        route = path_over(seen, (col, row), target)
        if route is None:
            # The chooser offered a frontier its own walk cannot reach, which
            # would be a bug in `reachable_from`. Counted rather than asserted,
            # so that the run finishes and says how often it happened.
            blocked += 1
            continue

        for at in range(0, len(route), step_cells):
            here = route[at]
            col, row = here % seen.width, here // seen.width
            metres += step_cells * room.resolution
            reveal(room, seen, col, row)
            visited.append((col, row))
        col, row = route[-1] % seen.width, route[-1] // seen.width
        reveal(room, seen, col, row)
        visited.append((col, row))
        arrived += 1
        if verbose:
            free, _u = frontier.classify(seen)
            print("  goal %2d: %5.2f m away, %5.2f m of edge -> %5.1f m driven, "
                  "%d cells known free"
                  % (goals, candidate["distance_m"], candidate["size_m"],
                     metres, sum(free)))

    return {"reason": "gave up", "goals": goals, "arrived": arrived,
            "blocked": blocked, "metres": metres, "seen": seen,
            "visited": visited, "summary": explorer.summary}


def room_cell(room, point):
    return (int(math.floor((point[0] - room.origin_x) / room.resolution)),
            int(math.floor((point[1] - room.origin_y) / room.resolution)))


def coverage(room, seen):
    """How much of the real floor the rover ended up knowing about.

    Counted against the floor it could actually have reached, not against every
    floor cell on the plan: the recorded drive saw through a doorway into a
    corner it never entered, and a policy cannot be marked down for failing to
    reach somewhere there is no way to.
    """
    free, _unknown = frontier.classify(seen)
    truth_floor = reachable_floor(room)
    known = sum(1 for i in truth_floor if free[i])
    return known, len(truth_floor)


def reachable_floor(room):
    """Every floor cell with a way to it from the largest room, as indices."""
    width, height = room.width, room.height
    best = []
    unseen = bytearray(width * height)
    for start in range(width * height):
        if room.truth[start] != room.FLOOR or unseen[start]:
            continue
        group = [start]
        unseen[start] = 1
        queue = collections.deque((start,))
        while queue:
            here = queue.popleft()
            row, col = divmod(here, width)
            for other, ok in ((here - 1, col > 0), (here + 1, col + 1 < width),
                              (here - width, row > 0),
                              (here + width, row + 1 < height)):
                if ok and not unseen[other] and room.truth[other] == room.FLOOR:
                    unseen[other] = 1
                    group.append(other)
                    queue.append(other)
        if len(group) > len(best):
            best = group
    return best


def picture(room, result, path):
    """The run as a PNG: what was known at the end, and where the rover went."""
    seen = result["seen"]
    free, unknown = frontier.classify(seen)
    scale = 2
    width, height = seen.width * scale, seen.height * scale
    pixels = bytearray(width * height * 3)

    def put(col, row, rgb):
        for dy in range(scale):
            for dx in range(scale):
                x, y = col * scale + dx, (seen.height - 1 - row) * scale + dy
                if 0 <= x < width and 0 <= y < height:
                    at = (y * width + x) * 3
                    pixels[at:at + 3] = bytes(rgb)

    for row in range(seen.height):
        for col in range(seen.width):
            i = row * seen.width + col
            truth = room.truth[i]
            if free[i]:
                put(col, row, (248, 248, 244))
            elif unknown[i] and truth == room.FLOOR:
                put(col, row, (210, 150, 150))     # floor it never found
            elif unknown[i]:
                put(col, row, (105, 105, 115))
            else:
                put(col, row, (15, 15, 20))
    for col, row in result["visited"]:
        put(col, row, (40, 150, 255))

    rows = b"".join(b"\x00" + bytes(pixels[y * width * 3:(y + 1) * width * 3])
                    for y in range(height))

    def chunk(tag, body):
        block = tag + body
        return (struct.pack(">I", len(body)) + block
                + struct.pack(">I", zlib.crc32(block)))

    with open(path, "wb") as fh:
        fh.write(b"\x89PNG\r\n\x1a\n"
                 + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height,
                                              8, 2, 0, 0, 0))
                 + chunk(b"IDAT", zlib.compress(rows, 6))
                 + chunk(b"IEND", b""))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("map", help="the floor plan, a PGM from map_score.py")
    ap.add_argument("--at", help="where the rover starts, as X,Y in metres")
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--min-frontier", type=float,
                    default=frontier.MIN_FRONTIER_M)
    ap.add_argument("--picture", help="write PNGs to this path prefix")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    grid = frontier.read_pgm(args.map, args.resolution)
    room = Room(grid)
    floor = reachable_floor(room)
    if not floor:
        print("%s has no floor in it" % args.map)
        return 1
    if args.at:
        start = tuple(float(v) for v in args.at.split(","))
    else:
        # The middle of the biggest room, which is where a rover parked in a
        # house tends to be, and is at least somewhere it can drive out of.
        start = grid.point_of(
            int(sum(i % room.width for i in floor) / len(floor)),
            int(sum(i // room.width for i in floor) / len(floor)))
        if room.solid(*room_cell(room, start)):
            start = grid.point_of(floor[len(floor) // 2] % room.width,
                                  floor[len(floor) // 2] // room.width)

    print("%s: %.1f x %.1f m, %d cells of reachable floor, rover starts at "
          "%.2f, %.2f" % (args.map, room.width * room.resolution,
                          room.height * room.resolution, len(floor),
                          start[0], start[1]))
    result = run(room, start, args.min_frontier, verbose=not args.quiet)
    known, total = coverage(room, result["seen"])

    print("%s after %d goals: %d arrived, %d unreachable, %.1f m driven"
          % (result["reason"], result["goals"], result["arrived"],
             result["blocked"], result["metres"]))
    print("covered %d of %d reachable floor cells (%.1f%%), %d frontiers left"
          % (known, total, 100.0 * known / total,
             result["summary"].get("frontiers", 0)))

    if args.picture:
        picture(room, result, args.picture + ".png")
        print("wrote %s.png -- blue is where it drove, pink is floor it never "
              "found" % args.picture)
    return 0 if result["reason"] == "finished" else 1


if __name__ == "__main__":
    sys.exit(main())
