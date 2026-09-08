"""What the rover would consider doing, and whether the arithmetic behind it holds.

Most of this is about one claim: that a second look from the right place is worth
more than a second look from where the rover already stands. That is the whole
of `improve_geometry`, and a scorer built on a generator that got it wrong would
rank confidently and drive the rover in circles -- so the crossing model is
checked against the case it exists for, where two rays nearly in line buy nothing
however good each of them is.

The rooms are drawn rather than described, so that a reader can see the doorway
the frontier is in and the wall the viewpoint is behind.
"""
from __future__ import annotations

import os
import re

import goals
from situation import Situation
from test_fakes import a_situation, a_thing
from test_harness import check

#: A room with a wide doorway onto ground nobody has mapped. The rover is
#: standing in the middle of the room; the unknown half is below the doorway.
ROOM = [
    "##################################",
    "#................................#",
    "#................................#",
    "#................................#",
    "#...............R................#",
    "#................................#",
    "#................................#",
    "#................................#",
    "############..............########",
    "???????????................???????",
    "???????????................???????",
    "??????????????????????????????????",
    "??????????????????????????????????",
]

#: A room with a closet shut off from it. Nothing placed inside the closet can
#: be looked at from anywhere the rover can walk to, which is what a thing
#: placed inside a wall or under the sofa looks like on a real map.
CLOSET = [
    "####################",
    "#..................#",
    "#........R.........#",
    "#..................#",
    "#.#######..........#",
    "#.#.....#..........#",
    "#.#.....#..........#",
    "#.#######..........#",
    "####################",
]

#: A room that opens along its whole lower edge onto ground nobody has mapped:
#: four metres of frontier with a room's worth of unknown floor behind it, which
#: is the biggest single thing this rover could be offered. Built from its parts
#: rather than drawn out, because thirty identical rows of question marks say
#: less than the line that makes them.
HOUSE = [
    "#" * 44,
    *(["#" + "." * 42 + "#"] * 4),
    "#" + "." * 20 + "R" + "." * 21 + "#",
    *(["#" + "." * 42 + "#"] * 4),
    *(["?" * 44] * 26),
]

#: The same room with nothing unmapped anywhere: every cell is floor or wall.
FINISHED = [
    "##################################",
    "#................................#",
    "#................................#",
    "#...............R................#",
    "#................................#",
    "#................................#",
    "##################################",
]


def _situation(rows=ROOM, **kwargs) -> Situation:
    return Situation(a_situation(rows, **kwargs))


def test_it_cannot_reach_the_rover() -> None:
    """The generators are arithmetic over a situation, and nothing else.

    Checked by reading the imports rather than by trusting the shape of the
    code, for `replay.py`'s reason: this is the module somebody will reach into
    for one convenient live lookup, and the day they do the decision stops being
    reproducible without anything failing.
    """
    source = open(os.path.join(os.path.dirname(__file__), "goals.py"),
                  encoding="utf-8").read()
    imported = set(re.findall(r"^(?:from|import)\s+([A-Za-z_][\w.]*)",
                              source, re.M))
    check("it imports nothing that could talk to the rover",
          sorted(imported - {"__future__", "typing", "math"}),
          ["mapgrid", "refs", "situation"])


# --- going where the map stops ----------------------------------------------

def test_a_doorway_onto_unmapped_ground_is_worth_driving_to() -> None:
    found = goals.explore_frontier(_situation())
    check("there is somewhere to explore", bool(found), True)
    first = found[0]
    check("...and it is a frontier", first.type, "explore_frontier")
    check("...it says what it expects to find",
          "unknown ground" in first.expects, True)
    check("...it is measured in floor, not in wishes", first.gain_kind,
          "unknown_floor_m2")
    check("...the estimate is capped by the ground actually behind it",
          first.gain_value <= first.gain_detail["swept_m2"], True)
    check("...it is honest that this is the one thing the rover cannot see "
          "what it is driving into", first.risk,
          "drives_onto_unmapped_ground")
    check("...and it would drive there", first.action[0]["call"], "drive_to")


def test_a_finished_room_offers_nothing_to_explore() -> None:
    """Not an error and not an empty answer with a shrug: there is nothing."""
    check("a mapped room has no frontiers",
          goals.explore_frontier(_situation(FINISHED)), [])


def test_the_frontiers_are_the_rovers_own_and_not_a_second_opinion() -> None:
    """The candidate is `frontier.survey`'s answer, at its own coordinates."""
    import mapgrid

    here = _situation()
    found, _summary = mapgrid.frontiers(here.grid, here.where)
    candidates = goals.explore_frontier(here)
    check("one candidate per frontier the rover's own chooser offered",
          len(candidates), min(len(found), goals.FRONTIER_LIMIT))
    check("...at the place it offered",
          candidates[0].constraints["goal"],
          {"x_m": round(found[0]["x"], 3), "y_m": round(found[0]["y"], 3)})


# --- going where a thing would come out better -------------------------------

def _thing_and_room(major_deg: float, **kwargs):
    """A room with one badly placed thing in the middle of the floor."""
    fields = {"uncertainty_m": 0.60, "major_deg": major_deg, **kwargs}
    thing = a_thing("object:8", 1.6, 1.0, **fields)
    return _situation(entities=[thing]), thing


def test_a_look_across_the_uncertainty_beats_a_look_along_it() -> None:
    """The claim the whole goal type rests on.

    The thing's uncertainty is a long thin ellipse. A viewpoint whose ray runs
    down that long axis adds a constraint across it -- which is the direction
    that is already known -- and buys nothing. A viewpoint to the side cuts it.
    """
    placement = {"x_m": 1.6, "y_m": 1.0, "error_major_m": 0.60,
                 "error_minor_m": 0.05, "error_major_deg": 0.0,
                 "height_sigma_m": 0.05}
    thing = {"id": "object:8", "ranging": {"never_ranged": False}}
    along = goals._from_viewpoint(placement, thing, 0.6, 1.0)     # due -x
    across = goals._from_viewpoint(placement, thing, 1.6, 0.0)    # due -y
    check("looking along the long axis buys nothing", along["gain_m"], 0.0)
    check("...and looking across it buys most of the error",
          across["gain_m"] > 0.4, True)
    check("...which is what the sentence says",
          "crossing at 90 degrees" in across["why"], True)


def test_no_placement_is_ever_predicted_better_than_this_rover_manages() -> None:
    """Bearings alone would promise a centimetre from half a metre away."""
    placement = {"x_m": 1.6, "y_m": 1.0, "error_major_m": 0.60,
                 "error_minor_m": 0.05, "error_major_deg": 0.0,
                 "height_sigma_m": 0.05}
    got = goals._from_viewpoint(placement, {"id": "object:8"}, 1.6, 0.5)
    check("the prediction stops at what has actually been achieved",
          got["after_m"], goals.PLACEMENT_FLOOR_M)


def test_a_thing_nobody_has_measured_the_distance_to_wants_the_depth_camera() -> None:
    here, _thing = _thing_and_room(90.0, looks=6, ranged=0)
    found = goals.improve_geometry(here)
    check("it is worth a candidate", bool(found), True)
    check("...which says the distance is the point",
          "never had its distance measured" in found[0].why, True)
    check("...and needs the camera that measures it",
          found[0].constraints["needs_depth_camera"], True)
    check("...from inside the band that camera was accepted in",
          found[0].constraints["in_certified_band"], True)


def test_a_thing_already_placed_well_is_not_worth_going_to() -> None:
    here, _thing = _thing_and_room(90.0, uncertainty_m=0.08, ranged=4)
    check("nothing is offered for a thing already placed to 8 cm",
          goals.improve_geometry(here), [])


def test_a_thing_from_a_map_that_has_gone_is_not_driven_to() -> None:
    """Its coordinates were in a map that no longer exists; the world state
    keeps it to recognise, not to drive to."""
    thing = a_thing("object:8", 1.6, 1.0, uncertainty_m=0.60, map_session=3)
    here = _situation(entities=[thing], map_session=7)
    check("no candidate names a placement from an older map",
          goals.improve_geometry(here), [])


def test_a_thing_shut_in_a_room_of_its_own_is_refused_out_loud() -> None:
    """The refusal has to be visible: a thing with nowhere to look at it from
    is otherwise indistinguishable from a thing nothing wanted to look at."""
    thing = a_thing("object:8", 0.55, 0.30, uncertainty_m=0.60)
    found = goals.improve_geometry(_situation(CLOSET, entities=[thing]))
    check("it still appears", len(found), 1)
    check("...with no route", found[0].constraints["reachable_m"], None)
    check("...and the reason in its own words",
          "wall in the way" in found[0].why
          or "nowhere within the certified band" in found[0].why, True)


def test_two_viewpoints_are_offered_when_they_are_a_real_choice() -> None:
    """The good viewpoint across the room and the adequate one nearby are a
    trade, and the trade belongs to the scorer rather than to this file.

    The thing's uncertainty runs across the rover's line of sight here, so the
    nearest place to stand is not the best one and the two really differ.
    """
    here, _thing = _thing_and_room(90.0)
    found = [one for one in goals.improve_geometry(here)
             if one.target == "object:8"]
    check("more than one place to stand is offered", len(found) > 1, True)
    check("...at different places",
          len({one.constraints["goal"]["x_m"] for one in found}) > 1, True)
    check("...and they cost different amounts",
          len({round(one.travel_m, 2) for one in found}) > 1, True)


def test_the_height_it_cannot_predict_is_declared_rather_than_assumed() -> None:
    here, _thing = _thing_and_room(90.0, height_sigma_m=1.2)
    found = goals.improve_geometry(here)
    check("the tilt is admitted to be unknown",
          found[0].constraints["tilt_unknown"], True)


def test_the_same_situation_produces_the_same_list_twice() -> None:
    here = _situation(entities=[a_thing("object:8", 1.6, 1.0,
                                        uncertainty_m=0.6),
                                a_thing("object:9", 2.2, 1.2,
                                        uncertainty_m=0.5)])
    first = [one.id for one in goals.generate(here)]
    second = [one.id for one in goals.generate(Situation(here.as_dict()))]
    check("the candidates are the same, in the same order", first, second)
    check("...and there are some", bool(first), True)


TESTS = (
    test_it_cannot_reach_the_rover,
    test_a_doorway_onto_unmapped_ground_is_worth_driving_to,
    test_a_finished_room_offers_nothing_to_explore,
    test_the_frontiers_are_the_rovers_own_and_not_a_second_opinion,
    test_a_look_across_the_uncertainty_beats_a_look_along_it,
    test_no_placement_is_ever_predicted_better_than_this_rover_manages,
    test_a_thing_nobody_has_measured_the_distance_to_wants_the_depth_camera,
    test_a_thing_already_placed_well_is_not_worth_going_to,
    test_a_thing_from_a_map_that_has_gone_is_not_driven_to,
    test_a_thing_shut_in_a_room_of_its_own_is_refused_out_loud,
    test_two_viewpoints_are_offered_when_they_are_a_real_choice,
    test_the_height_it_cannot_predict_is_declared_rather_than_assumed,
    test_the_same_situation_produces_the_same_list_twice,
)
