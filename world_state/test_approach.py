"""Where to stand to see a thing: the geometry, against rooms drawn by hand.

A grid is the one input here that a rover cannot be asked for at a desk, so the
rooms below are built a wall at a time. What is checked is the three ways the
obvious answer -- drive at the thing -- is wrong: the thing's own coordinates are
inside the thing, the floor around it is often grey rather than empty, and a wall
between two open patches of floor decides which of them is a viewpoint.
"""
from __future__ import annotations

import math

import test_fakes  # noqa: F401 -- puts the package's parent on sys.path
from test_harness import check
from world_state import approach


# --- rooms ------------------------------------------------------------------

def a_room(across_m: float = 8.0, up_m: float = 8.0,
           resolution_m: float = 0.05, fill: int = 0) -> approach.Grid:
    """Open floor with its origin at (0, 0), in the rover's own cell size.

    `fill` is what the room is made of: free floor by default, and
    `approach.UNKNOWN` for a room the rover has not been round yet.
    """
    width = int(round(across_m / resolution_m))
    height = int(round(up_m / resolution_m))
    return approach.Grid(width, height, resolution_m, 0.0, 0.0,
                         [[fill] * width for _ in range(height)])


def paint(grid: approach.Grid, x0: float, y0: float, x1: float, y1: float,
          value: int) -> approach.Grid:
    """A rectangle of the room, in metres, made solid or unknown or clear."""
    for iy in range(max(0, int(y0 / grid.resolution_m)),
                    min(grid.height, int(math.ceil(y1 / grid.resolution_m)))):
        for ix in range(max(0, int(x0 / grid.resolution_m)),
                        min(grid.width, int(math.ceil(x1 / grid.resolution_m)))):
            grid.cells[iy][ix] = value
    return grid


def _bearing(from_xy, to_xy) -> float:
    return math.degrees(math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0]))


def a_look(x_m: float, y_m: float, agrees: bool = True) -> dict:
    """One look of the thing, in the shape `view.rays` hands them over.

    Only where the rover was standing and whether the resolver still stands by
    the attachment are read here; the rest of a ray is for the map to draw.
    """
    return {"x_m": x_m, "y_m": y_m,
            "relation": {"agrees": agrees, "range_m": 1.0}}


# --- the ordinary case ------------------------------------------------------

def test_the_viewpoint_is_on_the_way_to_the_thing_and_faces_it() -> None:
    """An open room, the rover at one end and a thing at the other.

    The answer has to be the near side of it and not the far side, at the near
    end of the band and not the far end, and facing the thing rather than facing
    the way the rover happened to be travelling. All three come out of the one
    score -- how far to drive there, plus how far it would then be from the thing
    -- which is why this is the first thing checked.
    """
    grid = a_room()
    place = {"x_m": 6.0, "y_m": 4.0}
    found = approach.viewpoint(place, grid, (2.0, 4.0))
    check("open floor has a viewpoint", found["ok"], True)
    check("...at the near end of the band", found["range_m"], approach.NEAR_M)
    check("...on the rover's side of the thing", found["x_m"] < place["x_m"], True)
    check("...so the drive is the rest of the way",
          found["travel_m"], round(4.0 - approach.NEAR_M, 2))
    check("...and it arrives facing the thing",
          round(_bearing((found["x_m"], found["y_m"]), (6.0, 4.0))), 0.0)
    check("...having tested almost nothing to find it", found["tried"] <= 2, True)


def test_a_rover_already_in_front_of_it_barely_moves() -> None:
    """Standing a metre from the thing already, the drive is what is left of the
    band -- twenty centimetres and a turn. The console sends a turn on the spot
    rather than a goal that close, but that is its rule and not this one's: what
    matters here is that being nearly there is not mistaken for a reason to go
    somewhere else."""
    grid = a_room()
    found = approach.viewpoint({"x_m": 5.0, "y_m": 4.0}, grid, (4.0, 4.0))
    check("a rover already in front of a thing stays in front of it",
          found["travel_m"], round(1.0 - approach.NEAR_M, 2))
    check("...and is told which way to face", found["heading_deg"], 0.0)


def test_a_wide_thing_is_looked_at_from_further_out() -> None:
    """A placement names the middle of a thing, so the standoff is measured from
    its edge. A wall unit three metres across is a thing to be looked at from two
    metres, and nothing about the near bound alone would say so."""
    grid = a_room()
    narrow = approach.viewpoint({"x_m": 6.0, "y_m": 4.0}, grid, (2.0, 4.0))
    wide = approach.viewpoint({"x_m": 6.0, "y_m": 4.0, "extent_m": 3.0},
                              grid, (2.0, 4.0))
    check("a mug is looked at from the near bound",
          narrow["range_m"], approach.NEAR_M)
    check("...and a three-metre thing from its own edge plus the standoff",
          wide["range_m"], round(1.5 + approach.STANDOFF_M, 2))


# --- the three ways the room says no ----------------------------------------

def test_a_thing_against_a_wall_is_looked_at_from_the_open_side() -> None:
    """The case the whole module exists for. The placement is 30 cm off a wall,
    so the far side of it is solid and the near side is where the rover fits."""
    grid = paint(a_room(), 6.3, 0.0, 8.0, 8.0, 100)
    found = approach.viewpoint({"x_m": 6.0, "y_m": 4.0}, grid, (2.0, 4.0))
    check("a thing against a wall still has a viewpoint", found["ok"], True)
    check("...and it is out in the room rather than inside the wall",
          found["x_m"] < 6.3, True)
    check("...facing back at the wall", abs(found["heading_deg"]) < 90.0, True)


def test_a_wall_between_the_two_decides_which_open_floor_is_a_viewpoint() -> None:
    """Both sides of a partition are mapped floor the rover fits on. Only one of
    them can see the thing, and the difference is invisible to everything except
    the sight line: nothing about standing there is wrong."""
    grid = paint(a_room(), 5.4, 0.0, 5.6, 8.0, 100)     # a partition at x = 5.5
    place = {"x_m": 6.2, "y_m": 4.0}
    found = approach.viewpoint(place, grid, (2.0, 4.0))
    check("the thing behind a partition is still reachable", found["ok"], True)
    check("...from its own side of it", found["x_m"] > 5.6, True)
    check("...and the far side was tested and refused", found["blind"] > 0, True)
    check("standing on the wrong side really cannot see it",
          approach.can_see(grid, (5.0, 4.0), (6.2, 4.0)), False)
    check("...and standing on the right side can",
          approach.can_see(grid, (7.0, 4.0), (6.2, 4.0)), True)


def test_unmapped_floor_is_not_offered_and_says_so() -> None:
    """Grey is floor the rover has never seen rather than floor that is empty.
    A standing point on it would be refused by the planner a minute later, so it
    is refused here and in the map's own vocabulary."""
    grid = paint(a_room(fill=approach.UNKNOWN), 0.0, 0.0, 3.0, 8.0, 0)
    found = approach.viewpoint({"x_m": 6.0, "y_m": 4.0}, grid, (2.0, 4.0))
    check("a thing standing in unmapped space has no viewpoint",
          found["ok"], False)
    check("...and the sentence says which of the three it was",
          "has not been mapped" in found["why"], True)
    check("...having tried the whole ring rather than giving up early",
          found["tried"] > 100, True)


def test_a_buried_thing_says_it_is_buried() -> None:
    """Solid on every side within the band. Nothing to be done about it from a
    console, which is exactly why the sentence has to distinguish it from the
    grey case that a bit more exploring would fix."""
    grid = paint(a_room(), 3.0, 1.0, 9.0, 7.0, 100)
    found = approach.viewpoint({"x_m": 6.0, "y_m": 4.0}, grid, (2.0, 4.0))
    check("a thing with no room around it has no viewpoint", found["ok"], False)
    check("...and says it is up against something solid",
          "up against something solid" in found["why"], True)


def test_a_thing_boxed_in_by_a_wall_is_visible_from_nowhere() -> None:
    """Open floor all round, and a solid ring between it and the thing. Every
    candidate is a place the rover fits and none of them can see it."""
    grid = a_room()
    paint(grid, 5.4, 3.4, 6.6, 3.6, 100)
    paint(grid, 5.4, 4.4, 6.6, 4.6, 100)
    paint(grid, 5.4, 3.4, 5.6, 4.6, 100)
    paint(grid, 6.4, 3.4, 6.6, 4.6, 100)
    found = approach.viewpoint({"x_m": 6.0, "y_m": 4.0}, grid, (2.0, 4.0))
    check("a thing behind walls on every side has no viewpoint",
          found["ok"], False)
    check("...and says the way is blocked rather than that the floor is missing",
          "solid in the way" in found["why"], True)
    check("...counted as blind rather than as unmapped", found["blind"] > 0, True)


# --- the two rules underneath -----------------------------------------------

def test_the_rover_is_kept_its_own_width_from_anything_solid() -> None:
    """The same clearance a click on the console's map is held to, so that a
    point this offers and a point a person taps are judged by one rule. A single
    cell would let the rover be sent between the pixels of a thin wall."""
    grid = paint(a_room(), 4.0, 0.0, 8.0, 8.0, 100)
    check("a point inside the wall is solid",
          approach.why_not_stand(grid, 4.5, 4.0), "solid")
    check("...and so is one a few centimetres off it",
          approach.why_not_stand(grid, 3.93, 4.0), "solid")
    check("...while one the rover's own width away is not",
          approach.why_not_stand(grid, 3.7, 4.0), "")
    check("off the edge of the map is neither solid nor floor",
          approach.why_not_stand(grid, 9.0, 4.0), "off the map")
    check("and the edge itself is somewhere the rover fits",
          approach.why_not_stand(grid, 0.02, 4.0), "")


def test_a_sight_line_stops_short_of_the_thing_it_is_drawn_to() -> None:
    """A sofa is an obstacle in the grid, so a line of sight that had to reach
    the placement exactly would be blocked by the very object it was drawn to.
    Unknown cells are see-through, as they are everywhere else the world state
    walks this grid."""
    grid = paint(a_room(), 5.9, 3.9, 6.1, 4.1, 100)
    check("the thing's own cells do not blind the rover to it",
          approach.can_see(grid, (4.0, 4.0), (6.0, 4.0)), True)
    check("...but something solid short of it does",
          approach.can_see(grid, (4.0, 4.0), (7.0, 4.0)), False)
    grey = paint(a_room(), 5.0, 0.0, 5.2, 8.0, approach.UNKNOWN)
    check("grey is not a wall", approach.can_see(grey, (4.0, 4.0), (6.0, 4.0)),
          True)


# --- the lines the thing has been seen along --------------------------------

def test_a_known_sight_line_is_preferred_to_the_nearest_floor() -> None:
    """The whole of what `looks` changes. An open room, the thing in the middle
    of it and the rover to the east; every side is mapped floor and the map has no
    reason to prefer one. The one look of it was taken from the north, so that is
    where the rover is sent, even though the east side is a metre closer."""
    grid = a_room()
    place = {"x_m": 4.0, "y_m": 4.0}
    looks = [a_look(4.0, 6.5)]
    plain = approach.viewpoint(place, grid, (6.0, 4.0))
    known = approach.viewpoint(place, grid, (6.0, 4.0), looks)
    check("with no history the rover stops on the side it came from",
          plain["x_m"] > place["x_m"], True)
    check("...and says so", plain["along"],
          "the nearest floor it can be seen from")
    check("a thing seen from the north is approached from the north",
          known["y_m"] > place["y_m"], True)
    check("...on the line, not beside it", round(known["x_m"], 1), 4.0)
    check("...at the near end of the band", known["range_m"], approach.NEAR_M)
    check("...facing the thing", round(known["heading_deg"]), -90.0)
    check("...and says which line it chose", known["along"],
          "the line it has most often been seen along")
    check("...counting the looks that gave one", known["sight_lines"], 1)
    check("...and it really is the longer drive",
          known["travel_m"] > plain["travel_m"], True)


def test_the_median_line_is_one_the_rover_has_stood_on() -> None:
    """Seen twice from the north and once from the east, the median is north --
    not the north-east that an average of the three would name, which is a
    direction nothing was ever seen along."""
    check("the median of a cluster and an outlier is in the cluster",
          approach.middle_of([90.0, 88.0, 0.0]), 88.0)
    check("...and of two directions it is the lower of the two, not between them",
          approach.middle_of([90.0, 0.0]), 0.0)
    check("it wraps rather than averaging across the back",
          approach.middle_of([170.0, -170.0, 175.0]), 175.0)
    check("nothing seen has no median", approach.middle_of([]), None)


def test_a_blocked_sight_line_falls_through_to_the_ring() -> None:
    """A room the rover has seen the thing from the north of, with a partition
    since put across that side. The line is still where it was seen from and it is
    no longer a line to stand on, so the answer comes off the ring instead -- and
    says so, which is the difference between a rover that ignored the history and
    one whose history no longer holds."""
    grid = paint(a_room(), 0.0, 4.6, 8.0, 4.8, 100)
    place = {"x_m": 4.0, "y_m": 4.0}
    found = approach.viewpoint(place, grid, (6.0, 4.0), [a_look(4.0, 6.5)])
    check("a thing whose sight line is blocked still has a viewpoint",
          found["ok"], True)
    check("...off the ring rather than the line", found["along"],
          "the nearest floor it can be seen from")
    check("...which is the side the rover is on", found["x_m"] > place["x_m"],
          True)
    check("...and the line it could not use is still counted",
          found["sight_lines"], 1)


def test_the_second_line_is_tried_when_the_median_will_not_do() -> None:
    """Seen from the north twice and from the south once, with the north side
    walled off since. The south look is a line the rover has stood on and the ring
    is not, so it is preferred to the ring even though both would answer."""
    grid = paint(a_room(), 0.0, 4.6, 8.0, 4.8, 100)
    place = {"x_m": 4.0, "y_m": 4.0}
    found = approach.viewpoint(place, grid, (6.0, 4.0),
                               [a_look(4.0, 6.5), a_look(4.1, 6.4),
                                a_look(4.0, 1.5)])
    check("the other line it has been seen along is used", found["along"],
          "a line it has been seen along")
    check("...which is the south side", found["y_m"] < place["y_m"], True)
    check("...on the line", round(found["x_m"], 1), 4.0)


def test_a_look_the_resolver_disowns_is_not_a_sight_line() -> None:
    """A crop attached to the wrong thing points somewhere else in the room, so
    the place it was taken from is not a place this thing was seen from. The
    decision is the resolver's own, already made and carried on the ray."""
    place = {"x_m": 4.0, "y_m": 4.0}
    check("a look that agrees gives a direction",
          approach.seen_from(place, [a_look(4.0, 6.5)]), [90.0])
    check("...and one that does not gives none",
          approach.seen_from(place, [a_look(4.0, 6.5, agrees=False)]), [])
    check("a look with no pose is not a direction either",
          approach.seen_from(place, [{"relation": None}]), [])


def test_a_look_taken_from_on_top_of_the_thing_says_nothing() -> None:
    """Where a thing is, to within tens of centimetres, is what the placement
    already claims. A look from 20 cm away is inside that doubt, so the direction
    of that standing point from the thing is noise and is left out."""
    place = {"x_m": 4.0, "y_m": 4.0}
    check("a look from arm's length is not a bearing to stand on",
          approach.seen_from(place, [a_look(4.2, 4.0)]), [])
    check("...and one from beyond the baseline is",
          approach.seen_from(place, [a_look(4.0 + approach.SIGHT_BASELINE_M
                                            + 0.01, 4.0)]), [0.0])


def test_a_thing_never_placed_where_it_was_seen_is_still_approached() -> None:
    """No looks at all is the ordinary case for every caller that has none to
    hand -- a test drawing a room, and the console before this was kept. It is the
    behaviour the module had before sight lines existed, unchanged."""
    grid = a_room()
    found = approach.viewpoint({"x_m": 6.0, "y_m": 4.0}, grid, (2.0, 4.0), [])
    check("no history is not a refusal", found["ok"], True)
    check("...and it is the ring's answer", found["range_m"], approach.NEAR_M)
    check("...with nothing claimed about sight lines", found["sight_lines"], 0)


TESTS = (
    test_the_viewpoint_is_on_the_way_to_the_thing_and_faces_it,
    test_a_rover_already_in_front_of_it_barely_moves,
    test_a_wide_thing_is_looked_at_from_further_out,
    test_a_thing_against_a_wall_is_looked_at_from_the_open_side,
    test_a_wall_between_the_two_decides_which_open_floor_is_a_viewpoint,
    test_unmapped_floor_is_not_offered_and_says_so,
    test_a_buried_thing_says_it_is_buried,
    test_a_thing_boxed_in_by_a_wall_is_visible_from_nowhere,
    test_the_rover_is_kept_its_own_width_from_anything_solid,
    test_a_sight_line_stops_short_of_the_thing_it_is_drawn_to,
    test_a_known_sight_line_is_preferred_to_the_nearest_floor,
    test_the_median_line_is_one_the_rover_has_stood_on,
    test_a_blocked_sight_line_falls_through_to_the_ring,
    test_the_second_line_is_tried_when_the_median_will_not_do,
    test_a_look_the_resolver_disowns_is_not_a_sight_line,
    test_a_look_taken_from_on_top_of_the_thing_says_nothing,
    test_a_thing_never_placed_where_it_was_seen_is_still_approached,
)
