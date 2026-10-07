#!/usr/bin/env python3
"""Decide whether the rover's body fits where somebody asked it to stop.

Its own module, with no ROS in it, because two things need this arithmetic and
only one of them can import `rclpy`: the bridge that runs it on the rover, and
the selftest that runs on a workstation with no ROS at all. The same reason
`drive_mixer.py` is a module -- a copy of a *table* drifts visibly, a copy of a
geometry test drifts invisibly.

**The fault this exists to stop.** Nav2's global planner used to be NavFn, which
searches a cost grid as though the rover were a point, and its controller is DWB,
which checks the real rectangle. They disagree about exactly one thing, and that
thing is whether a spot five centimetres from a wall is somewhere the rover can
go.
Recorded on the rover: a goal was set at (4.34, -0.98), a cell whose cost was 216
-- traversable for a point, and with the body laid over it at every one of
twenty-four headings the footprint always overlapped the inscribed ring and at
one heading covered a lethal cell. NavFn returned a clean straight path to it.
DWB, which has no forward sample under 0.40 m/s and so no move shorter than
32 cm, could not land inside the arrival circle without ending up inside the
wall, so it stood there making small heading corrections until the allowance ran
out thirty seconds later. Nothing in the logs said "that goal is inside a wall",
because as far as either half of Nav2 was concerned nothing had gone wrong.

So the bridge tests the goal before it sends it, and moves it to the nearest
place the rover actually fits.

**What counts as fitting.** A costmap cell at 253 is one whose centre is within
the robot's inscribed radius of an obstacle, and 254 is the obstacle itself, so a
body covering either is a body in contact. 255 is *unknown*, and unknown is
deliberately allowed: the planner is configured with `allow_unknown`, this rover
maps as it drives, and a goal in a room it has not seen yet is a normal thing to
ask for rather than a mistake.
"""

import heapq
import json
import math

#: nav2_costmap_2d's own names for the three costs that are not a gradient.
INSCRIBED = 253
LETHAL = 254
UNKNOWN = 255

#: How far the goal may be moved before it stops being the goal somebody meant.
#: Half a metre is about two body lengths of slack and comfortably more than the
#: 0.45 m inflation radius, so anywhere inside the gradient has a way out of it.
REACH_M = 0.5

#: Headings tried at each candidate, fanning out from the one that was asked for.
#: 24 is every 15 degrees, which is finer than the 15-degree arrival tolerance,
#: so a heading that would have fitted is never missed by more than the goal
#: checker would have forgiven anyway.
HEADINGS = 24


class CostGrid(object):
    """A costmap as it arrives from `GetCostmap`, plus the arithmetic to index it.

    Deliberately not a numpy array. This runs on a four-core A53 next to a SLAM
    node and a controller already fighting over those cores, and importing numpy
    for a few hundred lookups costs more than the lookups do.
    """

    def __init__(self, width, height, resolution, origin_x, origin_y, data):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.data = data

    def cell_of(self, x, y):
        """The (column, row) that a point in the costmap's frame lands in."""
        return (int(math.floor((x - self.origin_x) / self.resolution)),
                int(math.floor((y - self.origin_y) / self.resolution)))

    def cost(self, col, row):
        """The cost at a cell, with everything off the edge reading as unknown.

        Off the edge is genuinely unknown rather than genuinely blocked -- the
        global costmap is only as big as the map so far -- and calling it 255
        keeps the one rule this module has, which is that unknown does not stop
        the rover.
        """
        if not (0 <= col < self.width and 0 <= row < self.height):
            return UNKNOWN
        return self.data[row * self.width + col]


def blocked(cost):
    """Is this a cost the body cannot be laid over?

    Only the two that mean contact. See the module docstring for why unknown is
    not one of them.
    """
    return INSCRIBED <= cost <= LETHAL


def corners(footprint, x, y, yaw):
    """The footprint's corners moved to a pose, still in metres."""
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return [(x + ax * cos_yaw - ay * sin_yaw, y + ax * sin_yaw + ay * cos_yaw)
            for ax, ay in footprint]


def covered(grid, footprint, x, y, yaw):
    """Every cell the body covers at a pose.

    The interior comes from a point-in-polygon test on the centre of each cell in
    the bounding box, and the outline from walking the edges. Both, because
    either alone has a hole in it: a footprint edge that passes through a cell
    without covering its centre would be missed by the interior test, and that is
    exactly how a body ends up straddling a wall the check said was clear.
    """
    exact = [((px - grid.origin_x) / grid.resolution,
              (py - grid.origin_y) / grid.resolution)
             for px, py in corners(footprint, x, y, yaw)]
    count = len(exact)
    cells = set()

    cols = [c for c, _ in exact]
    rows = [r for _, r in exact]
    for row in range(int(math.floor(min(rows))), int(math.floor(max(rows))) + 1):
        for col in range(int(math.floor(min(cols))),
                         int(math.floor(max(cols))) + 1):
            centre_x, centre_y = col + 0.5, row + 0.5
            inside = False
            for i in range(count):
                x0, y0 = exact[i]
                x1, y1 = exact[(i + 1) % count]
                if (y0 > centre_y) != (y1 > centre_y) and \
                        centre_x < x0 + (centre_y - y0) * (x1 - x0) / (y1 - y0):
                    inside = not inside
            if inside:
                cells.add((col, row))

    for i in range(count):
        x0, y0 = exact[i]
        x1, y1 = exact[(i + 1) % count]
        steps = int(max(abs(x1 - x0), abs(y1 - y0)) * 2.0) + 1
        for step in range(steps + 1):
            share = float(step) / steps
            cells.add((int(math.floor(x0 + (x1 - x0) * share)),
                       int(math.floor(y0 + (y1 - y0) * share))))
    return cells


def fits(grid, footprint, x, y, yaw, worst=INSCRIBED):
    """Can the rover stand here, facing this way, without being in something?

    `worst` is the lowest cost that counts as being in it: the inscribed ring
    by default, and `LETHAL` for the live costmap, where only a cell the scan
    hit is something the body would touch."""
    for col, row in covered(grid, footprint, x, y, yaw):
        if worst <= grid.cost(col, row) <= LETHAL:
            return False
    return True


def line_fits(grid, footprint, a, b, worst=INSCRIBED, skip_m=0.0):
    """Can the body drive straight from `a` to `b`, facing along the line?

    Sampled at the grid's own resolution, end to end, so nothing a cell wide is
    stepped over. The first `skip_m` is not checked: where the rover already
    stands is not a question about the way ahead."""
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    steps = max(1, int(math.ceil(length / grid.resolution)))
    for step in range(steps + 1):
        share = float(step) / steps
        if share * length < skip_m:
            continue
        if not fits(grid, footprint, a[0] + (b[0] - a[0]) * share,
                    a[1] + (b[1] - a[1]) * share, yaw, worst):
            return False
    return True


def straight_legs(grid, footprint, start, route, max_legs=4):
    """The fewest straight lines, along a planner's route, the body fits down.

    Each leg runs from where the last ended to the furthest point of `route` a
    straight drive reaches without the body touching anything the costmap
    holds. Returns the legs' end points, the last being the route's end, or
    None when a point cannot be reached straight or it takes more than
    `max_legs`: a route that needs more is not one to drive by turning on the
    spot between straight lines.

    This is how the rover goes round something near a goal: a turn, a straight
    line, a turn, as the rover drives naturally, instead of the curve the
    controller will not follow so close to where it is going (2026-10-07).
    """
    return _legs(start, route, max_legs,
                 lambda a, b: line_fits(grid, footprint, a, b))


def _legs(start, route, max_legs, straight):
    """`route` cut into the fewest lines `straight(a, b)` allows, or None."""
    points = list(route)
    if not points:
        return None
    legs, here, reached = [], tuple(start), -1
    while reached < len(points) - 1:
        onward = None
        for index in range(len(points) - 1, reached, -1):
            if straight(here, points[index]):
                onward = index
                break
        if onward is None or len(legs) >= max_legs:
            return None
        here, reached = tuple(points[onward]), onward
        legs.append(here)
    return legs


def room_round(grid, start, goal, clear_m, tight_m, relax_m):
    """Where the rover's centre may go on its way round something, one byte a
    cell: at least `clear_m` from anything the costmap holds as lethal, or,
    within `relax_m` of where it starts or is going, at least `tight_m`.

    For the live costmap, where a person standing in the way is a few lethal
    cells and nothing else: `clear_m` is the body's furthest corner and some to
    spare, so at whatever heading it passes, it does not brush them. Distances
    are from a lethal cell's edge, measured cell centre to cell centre.
    """
    res = grid.resolution
    width, height = grid.width, grid.height
    reach = int(math.ceil(clear_m / res)) + 1
    nearest = [(reach + 1) ** 2] * (width * height)
    disc = [(dc, dr, dc * dc + dr * dr)
            for dc in range(-reach, reach + 1) for dr in range(-reach, reach + 1)
            if dc * dc + dr * dr <= reach * reach]
    data = grid.data
    for index in range(width * height):
        if data[index] != LETHAL:
            continue
        col, row = index % width, index // width
        for dc, dr, d2 in disc:
            c, r = col + dc, row + dr
            if 0 <= c < width and 0 <= r < height and d2 < nearest[r * width + c]:
                nearest[r * width + c] = d2
    clear2 = (clear_m / res + 0.5) ** 2
    tight2 = (tight_m / res + 0.5) ** 2
    out = bytearray(width * height)
    for index in range(width * height):
        col, row = index % width, index // width
        x = grid.origin_x + (col + 0.5) * res
        y = grid.origin_y + (row + 0.5) * res
        need = clear2
        if (math.hypot(x - start[0], y - start[1]) <= relax_m
                or math.hypot(x - goal[0], y - goal[1]) <= relax_m):
            need = tight2
        out[index] = 1 if nearest[index] >= need else 0
    return out


#: How much wider than it has to be the way round is searched for first. The
#: shortest way hugs the edge of the room it is given, and a straight line
#: between two points on a curved edge cuts inside it: round one person, legs
#: at the edge took five, and with this much spare, two or three.
ROUND_SPARE_M = 0.10


def legs_round(grid, start, goal, clear_m, tight_m, relax_m, max_legs=4):
    """A way round whatever is between `start` and `goal`, as straight legs.

    The shortest way over `room_round`'s cells, eight-connected, cut into the
    fewest straight lines that stay on them. It is searched for
    `ROUND_SPARE_M` wider first, and as given if that finds nothing; the legs
    are held to `clear_m` either way. Returns the legs' end points, the last
    being `goal`, or None when there is no way or it takes more than
    `max_legs`. Unlike `straight_legs` it needs no planner's route: it is for
    the live costmap, which the global planner never sees (2026-10-07: M3
    session 13, the owner standing in front of a near goal the planner drew
    straight through them).
    """
    room = room_round(grid, start, goal, clear_m, tight_m, relax_m)
    width, height = grid.width, grid.height
    begin, end = grid.cell_of(*start), grid.cell_of(*goal)
    for col, row in (begin, end):
        if not (0 <= col < width and 0 <= row < height):
            return None
    if not room[end[1] * width + end[0]]:
        return None
    res = grid.resolution

    def stays(a, b):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(math.ceil(2.0 * length / res)))
        for i in range(steps + 1):
            col, row = grid.cell_of(a[0] + (b[0] - a[0]) * i / steps,
                                    a[1] + (b[1] - a[1]) * i / steps)
            if (col, row) == begin:
                continue
            if not (0 <= col < width and 0 <= row < height) \
                    or not room[row * width + col]:
                return False
        return True

    wider = room_round(grid, start, goal, clear_m + ROUND_SPARE_M, tight_m,
                       relax_m)
    for search in (wider, room):
        route = _shortest(grid, search, begin, end, goal)
        legs = _legs(start, route, max_legs, stays) if route else None
        if legs:
            return legs
    return None


def _shortest(grid, room, begin, end, goal):
    """The shortest eight-connected way over `room` from cell `begin` to cell
    `end`, as the cell centres after `begin` with `goal` itself last, or None.
    """
    width, height = grid.width, grid.height
    if not room[end[1] * width + end[0]]:
        return None
    root2 = math.sqrt(2.0)

    def guess(cell):
        dc, dr = abs(cell[0] - end[0]), abs(cell[1] - end[1])
        return max(dc, dr) + (root2 - 1.0) * min(dc, dr)

    came, cost, frontier = {begin: None}, {begin: 0.0}, [(guess(begin), begin)]
    while frontier:
        _, cell = heapq.heappop(frontier)
        if cell == end:
            break
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                step = (cell[0] + dc, cell[1] + dr)
                if step == cell or not (0 <= step[0] < width
                                        and 0 <= step[1] < height):
                    continue
                if not room[step[1] * width + step[0]]:
                    continue
                spent = cost[cell] + (root2 if dc and dr else 1.0)
                if spent < cost.get(step, float("inf")):
                    cost[step], came[step] = spent, cell
                    heapq.heappush(frontier, (spent + guess(step), step))
    if end not in came:
        return None
    cells, cell = [], end
    while cell is not None and cell != begin:
        cells.append(cell)
        cell = came[cell]
    cells.reverse()
    res = grid.resolution
    return [(grid.origin_x + (c + 0.5) * res, grid.origin_y + (r + 0.5) * res)
            for c, r in cells[:-1]] + [tuple(goal)]


def candidates(grid, x, y, reach_m):
    """Cells within `reach_m` of a point, nearest first.

    Sorted by true distance rather than walked as square rings, because the
    nearest place that fits is worth actually finding: the difference between a
    goal moved 11 cm and one moved 20 cm is the difference between arriving where
    somebody pointed and arriving somewhere else in the room.
    """
    span = int(math.ceil(reach_m / grid.resolution))
    out = []
    for drow in range(-span, span + 1):
        for dcol in range(-span, span + 1):
            offset_x = dcol * grid.resolution
            offset_y = drow * grid.resolution
            away = math.hypot(offset_x, offset_y)
            if away <= reach_m:
                out.append((away, x + offset_x, y + offset_y))
    out.sort(key=lambda item: item[0])
    return out


def headings_from(yaw, count=HEADINGS):
    """Every heading, ordered by how far it is from the one that was asked for."""
    step = 2.0 * math.pi / count
    order = [0]
    for i in range(1, count // 2 + 1):
        order.append(i)
        if i != count // 2:
            order.append(-i)
    return [yaw + i * step for i in order]


def fit(grid, footprint, x, y, yaw, reach_m=REACH_M):
    """Where the rover should actually be sent, given where it was asked to go.

    Returns the pose to use and how far it had to move to find it, or None when
    there is nowhere within `reach_m` that the body fits -- which is the honest
    answer when somebody points at a wall, and a great deal better than the
    thirty seconds of shuffling that used to follow.

    The pose asked for is tried first, at its own heading, so a goal in open
    floor -- which is nearly all of them -- costs one polygon test and comes back
    unchanged.
    """
    if fits(grid, footprint, x, y, yaw):
        return {"x": x, "y": y, "yaw": yaw, "moved_m": 0.0, "turned_deg": 0.0}
    for away, near_x, near_y in candidates(grid, x, y, reach_m):
        # The cheap rejection first: a centre already in contact cannot be
        # rescued by any heading, and near a wall that is most of the cells.
        if blocked(grid.cost(*grid.cell_of(near_x, near_y))):
            continue
        for heading in headings_from(yaw):
            if fits(grid, footprint, near_x, near_y, heading):
                turned = math.degrees(
                    (heading - yaw + math.pi) % (2.0 * math.pi) - math.pi)
                return {"x": near_x, "y": near_y, "yaw": heading,
                        "moved_m": away, "turned_deg": turned}
    return None


def polygon_from(footprint_text, robot_radius, sides=12):
    """The footprint as the costmap node is actually configured with it.

    Nav2 takes either a polygon or a radius and the polygon wins, which is the
    order tried here. A radius becomes a twelve-sided approximation rather than a
    square, because a square drawn round a circle is 27% too big at the corners
    and the whole point of this module is not to guess at the body.
    """
    text = (footprint_text or "").strip()
    if text and text not in ("[]", '""', "''"):
        try:
            points = json.loads(text)
        except ValueError:
            points = None
        if points and len(points) >= 3:
            return [(float(px), float(py)) for px, py in points]
    if robot_radius and robot_radius > 0.0:
        return [(robot_radius * math.cos(2.0 * math.pi * i / sides),
                 robot_radius * math.sin(2.0 * math.pi * i / sides))
                for i in range(sides)]
    return None


def inscribed_radius(footprint_text, robot_radius):
    """How far the costmap's 253 ring reaches out from an obstacle, in metres.

    The planner lays the rover's *centre* on the costmap and treats 253 as
    contact, so this is the clearance a route needs on each side: a gap is
    passable only if it is wider than twice this. A walk over the occupancy grid
    that is meant to agree with the planner has to keep this far from walls, and
    the autonomy executive's does (`autonomy/mapgrid.py`).

    Nav2's rule: a polygon's is the nearest its outline comes to the centre, and
    a bare radius is the radius -- not the twelve-sided stand-in above, whose
    edges come 3% closer. Measured on the rover's global costmap on 2026-10-05
    with `robot_radius: 0.200`: every 253 cell is within 0.200 m of a lethal one,
    and nothing below 253 is nearer than 0.206 m.
    """
    text = (footprint_text or "").strip()
    if text and text not in ("[]", '""', "''"):
        try:
            points = json.loads(text)
        except ValueError:
            points = None
        if points and len(points) >= 3:
            nearest = None
            for i, (x0, y0) in enumerate(points):
                x1, y1 = points[(i + 1) % len(points)]
                dx, dy = float(x1) - float(x0), float(y1) - float(y0)
                length2 = dx * dx + dy * dy
                share = 0.0 if length2 == 0.0 else max(0.0, min(1.0, (
                    -float(x0) * dx - float(y0) * dy) / length2))
                away = math.hypot(float(x0) + share * dx, float(y0) + share * dy)
                nearest = away if nearest is None else min(nearest, away)
            return nearest
    if robot_radius and robot_radius > 0.0:
        return float(robot_radius)
    return None
