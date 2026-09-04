"""Where the rover would have to stand to look at a thing it has placed.

The store knows where a thing is. What it does not know, and what anybody who
wants a better look at one immediately needs, is **where the rover could stand to
see it** -- because the answer is almost never "where it is". A sofa's own
coordinates are inside the sofa; the wall it is against is solid; and half the
floor around it is grey, meaning the rover has never been there rather than that
it is empty. So a "go and look at that" button that drove at the placement would
be a button that drove into furniture, and Nav2 would refuse most of the presses
it did not.

This module answers the question the map can actually answer: of the points
around the thing that are **mapped floor the rover fits on**, and that have
**nothing solid between them and the thing**, which one is best to send it to.

**The choice is the shortest way there and back to the thing.** Every candidate
is scored by how far the rover must travel to it plus how far it would then be
from the thing, and the smallest wins. That single number does what two rules
would have argued about: it prefers the near side of an object to the far side,
it prefers standing close over standing at the edge of the band, and it will
spend a metre of driving to end up a metre closer. It is a straight line and not
a route -- there is a planner for that, and it is on the other side of a socket
-- so it is a preference, not a promise: what the rover ends up driving is
whatever Nav2 plans to the point this picks.

Nothing here reads the store, and nothing here drives. It takes a placement, an
occupancy grid and where the rover is standing, and returns a point; the daemon's
`world_state_viewpoint` supplies all three and the drive console does the driving.
That is what makes it testable at a desk against grids drawn by hand, which is
the only place the awkward cases -- a thing in a doorway, a thing against a wall,
a thing with mapped floor on the far side only -- can be set up on purpose.
"""
from __future__ import annotations

import math
from typing import Any, Iterator, NamedTuple

#: What an occupancy value has to reach before a cell counts as something solid.
#: The same threshold `ros_navigator.GRID_OCCUPIED_AT` renders the map with and
#: `rover_world.OCCUPIED_AT` bounds a sighting with -- one ROS convention read in
#: three places, and stated again here because nothing in this package may import
#: the daemon.
OCCUPIED_AT = 50
#: Floor the rover has never seen, which ROS publishes as -1 and the map draws
#: grey. It is not empty floor and must never be treated as such: there is no
#: route to plan across it, so a standing point on it would be refused by the
#: planner a minute later instead of here and now.
UNKNOWN = -1

#: How close the rover is willing to stand to a thing, in metres, before the
#: thing's own width is taken into account. The chassis is about 20 cm from its
#: centre to its bumper and the lens is 130 degrees across, so this is close
#: enough that a thing fills a useful part of the frame and far enough that the
#: rover is not touching it.
NEAR_M = 0.8
#: And how far away is still worth driving to. Beyond this the crop is small and
#: the placement's own doubt -- tens of centimetres on this rover -- is a large
#: part of the distance, so "closer" stops meaning much.
FAR_M = 2.5
#: Held clear of the thing's own edge on top of that, so that a wide object is
#: viewed from beside its edge rather than from a point measured to its middle.
#: `extent_m` on a placement is the width the crossing measured.
STANDOFF_M = 0.4
#: How much room around the standing point has to be free of anything solid.
#: `rover_nav.MAP_POINT_CLEAR_M`, which is what a click on the console's map is
#: held to, so that a point this offers and a point a person taps are judged by
#: the same rule.
CLEAR_M = 0.15
#: How close to the thing the sight line has to stay clear. Short of the thing
#: itself, deliberately: a sofa **is** an obstacle in the grid, so a line of
#: sight that had to reach the placement exactly would be blocked by the very
#: object it was drawn to.
SEE_UP_TO_M = 0.35
#: The gap between the rings of candidates, and between candidates along one.
#: Finer than this buys nothing -- the planner nudges a goal it cannot fit by
#: about this much anyway -- and coarser starts to miss a doorway-sized gap.
RING_M = 0.15
ARC_M = 0.15


class Grid(NamedTuple):
    """An occupancy grid, in the shape the navigator's `map` op hands it over.

    Built by the daemon straight from `_world_grid`, whose tuple is in this
    order. `cells` is indexed `cells[iy][ix]`, which is true of both the numpy
    array the daemon decodes and the lists of rows the tests draw by hand.
    """

    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    cells: Any


def value_at(grid: Grid, x_m: float, y_m: float) -> int | None:
    """What the map says about the point, or None if it is off the edge of it.

    **Floor, not truncation.** `int()` rounds toward zero, so a point five
    centimetres the wrong side of the origin would land in cell 0 and a walk off
    that edge of the map would never notice it had left. The same trap
    `_world_reach` documents, and the same fix.
    """
    ix = math.floor((float(x_m) - grid.origin_x_m) / grid.resolution_m)
    iy = math.floor((float(y_m) - grid.origin_y_m) / grid.resolution_m)
    if not (0 <= ix < grid.width and 0 <= iy < grid.height):
        return None
    return int(grid.cells[iy][ix])


def why_not_stand(grid: Grid, x_m: float, y_m: float,
                  clear_m: float = CLEAR_M) -> str:
    """Empty if the rover could stand there, otherwise which of the three reasons.

    The three are worth telling apart because they mean different things to the
    person who pressed the button: off the edge of the map and unmapped are "the
    rover has not been round there yet", and solid is "that is furniture".
    """
    here = value_at(grid, x_m, y_m)
    if here is None:
        return "off the map"
    if here == UNKNOWN:
        return "unmapped"
    margin = max(1, int(round(clear_m / grid.resolution_m)))
    for down in range(-margin, margin + 1):
        for across in range(-margin, margin + 1):
            near = value_at(grid, x_m + across * grid.resolution_m,
                            y_m + down * grid.resolution_m)
            # Off the map beside a cell that is on it is an edge, and an edge is
            # not a wall: the rover fits, it simply cannot see past it. Unknown
            # is the same -- it is refused under the standing point itself and
            # tolerated around it, or nothing within half a metre of the frontier
            # would ever be offered.
            if near is not None and near >= OCCUPIED_AT:
                return "solid"
    return ""


def can_see(grid: Grid, from_xy: tuple[float, float],
            to_xy: tuple[float, float], up_to_m: float = SEE_UP_TO_M) -> bool:
    """Whether nothing solid stands between the two points.

    **Unknown cells are see-through here, and that is the same choice
    `_world_reach` makes.** Grey is floor the rover has not driven past, not a
    wall; a room is mapped along the lines the lidar happened to take, so a rule
    that stopped at the first grey cell would refuse most of a half-mapped room
    and would do it in the cases where the rover is standing right next to the
    thing. What it costs is the wrong answer where a wall is genuinely unmapped,
    and what that costs in turn is a drive to a point from which the thing is not
    actually visible -- a wasted move rather than a collision, because the
    planner has its own costmap and the standing point itself is known floor.
    """
    fx, fy = float(from_xy[0]), float(from_xy[1])
    span = math.hypot(float(to_xy[0]) - fx, float(to_xy[1]) - fy)
    reach = span - float(up_to_m)
    if reach <= 0.0:
        return True
    step = grid.resolution_m / 2.0
    dx = (float(to_xy[0]) - fx) / span * step
    dy = (float(to_xy[1]) - fy) / span * step
    for count in range(1, int(reach / step) + 1):
        here = value_at(grid, fx + dx * count, fy + dy * count)
        if here is not None and here >= OCCUPIED_AT:
            return False
    return True


def band(place: dict[str, Any], near_m: float = NEAR_M,
         far_m: float = FAR_M) -> tuple[float, float]:
    """How far from the thing the rover should end up, as a range in metres.

    Its own width is part of the near end, because a placement names the middle
    of a thing and the rover has to stand outside it. A thing wider than the far
    end pushes that out with it rather than being refused: a five-metre wall unit
    is a thing to be looked at from four metres, not a thing with no viewpoint.
    """
    near = max(float(near_m),
               float(place.get("extent_m") or 0.0) / 2.0 + STANDOFF_M)
    return near, max(float(far_m), near + RING_M)


def _ring(place: dict[str, Any], from_xy: tuple[float, float], near_m: float,
          far_m: float) -> Iterator[tuple[float, float, float, float, float]]:
    """Candidate standing points as `(cost, range, travel, x, y)`, cheapest first.

    Cheapest first is what keeps this quick: the whole grid arithmetic below runs
    only until one candidate passes, so an ordinary press tests a handful of
    points rather than the couple of thousand this yields.

    **The cost ties along the whole line between the rover and the thing**, and
    that is not an awkward case, it is the ordinary one: driving straight at
    something and stopping is the same total distance wherever you stop. So the
    sum decides how far out of the way a viewpoint is worth going, and the tie it
    leaves is settled by standing closer -- which is what makes this a button for
    getting a better look at something rather than a button for creeping into
    view of it from across the room.
    """
    px, py = float(place["x_m"]), float(place["y_m"])
    fx, fy = float(from_xy[0]), float(from_xy[1])
    found = []
    rings = max(1, int(round((far_m - near_m) / RING_M)) + 1)
    for step in range(rings):
        range_m = near_m + step * RING_M
        if range_m > far_m + 1e-9:
            break
        around = max(8, int(round(2.0 * math.pi * range_m / ARC_M)))
        for turn in range(around):
            angle = 2.0 * math.pi * turn / around
            # **Rounded here and not on the way out.** The point that goes to the
            # navigator has to be the point that was tested, and it was not: the
            # answer used to be rounded to the millimetre as it was returned,
            # after the sight line had been walked from the unrounded one. Half a
            # millimetre is nothing until a line grazes the corner of a wall cell,
            # and against the rover's own map on 2026-09-04 three of sixty things
            # came back with a standing point that could no longer see them when
            # the check was repeated at the coordinates actually sent.
            x_m = round(px + range_m * math.cos(angle), 3)
            y_m = round(py + range_m * math.sin(angle), 3)
            travel = math.hypot(x_m - fx, y_m - fy)
            # **Scored to the centimetre, because the ties are the point.** Every
            # candidate on the line between the rover and the thing costs the
            # same to within the millimetre the points above are quantized to,
            # and comparing those raw makes the winner whichever ring's cosine
            # happened to land low -- which is to say, an arbitrary distance from
            # the thing. Rounded, a tie is a tie and the rule below decides it;
            # a detour worth a centimetre still outranks one that is not.
            found.append((round(travel + range_m, 2), round(range_m, 3),
                          travel, x_m, y_m))
    found.sort()
    return iter(found)


def viewpoint(place: dict[str, Any], grid: Grid, from_xy: tuple[float, float],
              near_m: float = NEAR_M, far_m: float = FAR_M,
              clear_m: float = CLEAR_M) -> dict[str, Any]:
    """The place to send the rover so that it ends up looking at this thing.

    `ok` is whether there is one. When there is, `x_m`/`y_m` is where to drive to
    and `heading_deg` is the way to be facing on arrival, which is what turns
    "somewhere it can be seen from" into "seen": the map's own convention, so it
    goes to the navigator as the goal's yaw unchanged.

    When there is not, `why` is a sentence for the console to show and the counts
    beside it are what it was worked out from -- how many candidates were off the
    map or unmapped, how many were solid, and how many were fine to stand on but
    had something between them and the thing. Those three are different faults
    with different answers: drive around a bit more, this thing is buried, and
    this thing is behind a wall from every side that has been mapped.
    """
    near, far = band(place, near_m, far_m)
    target = (float(place["x_m"]), float(place["y_m"]))
    counts = {"tried": 0, "unmapped": 0, "solid": 0, "blind": 0}
    for _cost, range_m, travel, x_m, y_m in _ring(place, from_xy, near, far):
        counts["tried"] += 1
        why = why_not_stand(grid, x_m, y_m, clear_m)
        if why:
            counts["solid" if why == "solid" else "unmapped"] += 1
            continue
        if not can_see(grid, (x_m, y_m), target):
            counts["blind"] += 1
            continue
        return {
            "ok": True,
            # As tested, to the millimetre, and not rounded again on the way out.
            "x_m": x_m,
            "y_m": y_m,
            # Facing the thing, which is the whole of what makes this a viewpoint
            # rather than a nearby patch of floor.
            "heading_deg": round(math.degrees(
                math.atan2(target[1] - y_m, target[0] - x_m)), 1),
            "range_m": round(range_m, 2),
            "travel_m": round(travel, 2),
            "near_m": round(near, 2),
            "far_m": round(far, 2),
            **counts,
        }
    return {"ok": False, "why": _nowhere(counts, near, far),
            "near_m": round(near, 2), "far_m": round(far, 2), **counts}


def _nowhere(counts: dict[str, int], near: float, far: float) -> str:
    """Why nothing within reach of the thing would do, in one sentence.

    Whichever of the three refusals dominated, because that is the one a person
    can act on -- and they are acted on differently. Written here rather than in
    the console so that a caller with no screen gets the same answer.
    """
    if not counts["tried"]:
        return "that thing has no position on this map"
    if counts["blind"] >= max(counts["solid"], counts["unmapped"]):
        return ("every mapped patch of floor within %.1f m of it has something "
                "solid in the way, so there is nowhere to see it from" % far)
    if counts["solid"] >= counts["unmapped"]:
        return ("it is up against something solid on every side, with no room "
                "for the rover between %.1f and %.1f m of it" % (near, far))
    return ("the floor within %.1f m of it has not been mapped, so there is "
            "nowhere the rover can be sent to look at it" % far)
