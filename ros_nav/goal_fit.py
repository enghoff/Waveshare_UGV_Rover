#!/usr/bin/env python3
"""Decide whether the rover's body fits where somebody asked it to stop.

Its own module, with no ROS in it, because two things need this arithmetic and
only one of them can import `rclpy`: the bridge that runs it on the rover, and
the selftest that runs on a workstation with no ROS at all. The same reason
`drive_mixer.py` is a module -- a copy of a *table* drifts visibly, a copy of a
geometry test drifts invisibly.

**The fault this exists to stop.** Nav2's planner is NavFn, which searches a cost
grid as though the rover were a point, and its controller is DWB, which checks
the real rectangle. They disagree about exactly one thing, and that thing is
whether a spot five centimetres from a wall is somewhere the rover can go.
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


def worst_cost(grid, footprint, x, y, yaw):
    """The highest cost anywhere under the body at a pose."""
    return max(grid.cost(col, row)
               for col, row in covered(grid, footprint, x, y, yaw))


def fits(grid, footprint, x, y, yaw):
    """Can the rover stand here, facing this way, without being in something?"""
    for col, row in covered(grid, footprint, x, y, yaw):
        if blocked(grid.cost(col, row)):
            return False
    return True


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
