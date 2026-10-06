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

import math
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


#: What the navigation bridge says about the body with every map since
#: 2026-10-05: the planner keeps the rover's centre 0.20 m from walls, and a goal
#: is moved up to half a metre onto floor where the body fits.
BODY = {"inscribed_radius_m": 0.20, "goal_fit_reach_m": 0.5}


def _with_body(body: dict, **said) -> Situation:
    body["map"] = {**body["map"], **(said or BODY)}
    return Situation(body)


def _pocket(gap_cells: int) -> list[str]:
    """A room, a wall with a gap in it, and a pocket open to unmapped ground.

    Drawn at 5 cm, the resolution the rover maps at, because the gap is the
    whole point and at 10 cm a cell is half the clearance being tested.
    """
    width = 40
    left = (width - gap_cells) // 2
    room = ["#" + "." * (width - 2) + "#"] * 20
    room = room[:10] + ["#" + "." * 18 + "R" + "." * 19 + "#"] + room[11:]
    return (["#" * width] + room
            + ["#" * left + "." * gap_cells + "#" * (width - left - gap_cells)]
            + ["#" + "." * (width - 2) + "#"] * 12
            + ["#" + "?" * (width - 2) + "#"] * 6)


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
          ["cooling", "mapgrid", "refs", "situation"])


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


def test_a_gap_the_body_does_not_fit_through_is_not_a_way_there() -> None:
    """A frontier reached through a gap narrower than the planner allows is refused.

    The planner refuses any cell within 0.20 m of a wall for the rover's centre,
    so a 30 cm gap is closed to it however open it looks to a point; a 50 cm one
    is not. Walked as a point, the first is a frontier a couple of metres away,
    which is the fault the rover had (see the next test).
    """
    import scoring

    narrow = a_situation(_pocket(6), resolution_m=0.05)
    as_point = goals.explore_frontier(Situation(narrow))
    check("walked as a point, the pocket behind a 30 cm gap is somewhere to go",
          [one.constraints["reachable_m"] is not None for one in as_point],
          [True])
    here = _with_body(a_situation(_pocket(6), resolution_m=0.05))
    refused = goals.explore_frontier(here)
    check("walked with the body, it is not", [one.constraints["reachable_m"]
                                              for one in refused], [None])
    check("...it is still offered, so the refusal is in the record",
          [one.type for one in refused], ["explore_frontier"])
    check("...and says why", "not for the rover's body" in refused[0].why, True)
    check("...the walk a point would have taken is kept beside it",
          refused[0].gain_detail["point_walk_m"],
          as_point[0].constraints["reachable_m"])
    check("...and the scorer refuses it as unreachable for the body",
          [veto["why"] for veto in scoring.vetoes(refused[0], here)
           if veto["veto"] == "unreachable"],
          ["there is no route to it over floor the map calls free that the "
           "rover's body fits through"])

    wide = _with_body(a_situation(_pocket(10), resolution_m=0.05))
    through = goals.explore_frontier(wide)
    check("a 50 cm gap is a way through for the body",
          [one.constraints["reachable_m"] is not None for one in through],
          [True])
    check("...and the body's walk to it is never shorter than a point's",
          through[0].constraints["reachable_m"] >= through[0].gain_detail[
              "point_walk_m"], True)
    # The room's left wall is the column of cells at x 0.00-0.05 m.
    check("a goal 10 cm from a wall is still reachable: the bridge moves it",
          wide.reach.reachable(0.125, 1.6) is not None, True)
    check("...but not without the allowance the bridge said it gives",
          _with_body(a_situation(_pocket(10), resolution_m=0.05),
                     inscribed_radius_m=0.20).reach.reachable(0.125, 1.6), None)


def test_the_pocket_by_the_charger_is_not_somewhere_to_drive() -> None:
    """The two recorded decisions that chose it, replayed on their own maps.

    **The fault, from the record.** Every frontier an autonomous run had chosen
    before 2026-10-05 -- seven, on 2026-10-03 and in M3 session 2 -- was in a
    pocket by the charger that the lidar sees into through a 30-40 cm gap. The
    executive's walk called each about five metres away; Nav2 had no way
    through the gap, found a 38 m way round twice, drove off along it, and gave
    up. Three in a row ended the run of 2026-10-03. Here: as recorded, the
    frontier it chose is reachable at that distance; with the body the bridge now
    describes, it is not, and a place the same run did drive to still is.
    """
    import gzip
    import json

    import scoring

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures",
                        "pocket-by-the-charger.json.gz")
    with gzip.open(path, "rt", encoding="utf-8") as handle:
        cases = json.load(handle)["cases"]
    for case in cases:
        name = "episode %d" % case["episode"]

        def recorded() -> dict:
            body = a_situation(FINISHED)
            body["map"] = dict(case["map"])
            body["nav"]["pose"] = dict(case["pose"])
            return body

        chose = case["chose"]
        as_was = {one.id: one for one in
                  goals.explore_frontier(Situation(recorded()))}
        walked = as_was[chose["id"]].constraints["reachable_m"]
        check(name + ": as recorded, the frontier it chose was about five "
              "metres' walk", walked is not None and 4.5 < walked < 6.0, True)
        here = _with_body(recorded())
        now = {one.id: one for one in goals.explore_frontier(here)}
        check(name + ": walked with the body it is unreachable",
              now[chose["id"]].constraints["reachable_m"], None)
        check(name + ": ...and refused",
              "unreachable" in [veto["veto"] for veto in
                                scoring.vetoes(now[chose["id"]], here)], True)
        check(name + ": ...while the place the run did drive to is still "
              "somewhere to go",
              here.reach.reachable(case["arrived"]["x_m"],
                                   case["arrived"]["y_m"]) is not None, True)
        check(name + ": ...and the rover is standing somewhere it can walk from",
              here.reach.standing is not None, True)


def _costmap_of(situation: Situation, inscribed_m: float):
    """The planner's costmap rebuilt from a map: the bridge's goal fit runs on it.

    Occupied cells lethal, the ring within the inscribed radius of them 253, and
    unknown 255 -- the static layer and its inflation, which is what the bridge
    reads when it decides whether the body fits.
    """
    import mapgrid

    grid = situation.grid
    goal_fit = mapgrid.goal_fit
    _free, unknown = mapgrid.frontier.classify(grid)
    clear = mapgrid._clear_of_walls(grid, bytearray(b"\1") * len(grid.data),
                                    inscribed_m)
    data = bytearray(len(grid.data))
    for here, value in enumerate(grid.data):
        if value >= mapgrid.frontier.OCCUPIED_AT:
            data[here] = goal_fit.LETHAL
        elif not clear[here]:
            data[here] = goal_fit.INSCRIBED
        elif unknown[here]:
            data[here] = goal_fit.UNKNOWN
    return goal_fit.CostGrid(grid.width, grid.height, grid.resolution,
                             grid.origin_x, grid.origin_y, bytes(data))


def test_a_goal_with_no_room_for_the_body_is_not_somewhere_to_drive() -> None:
    """Seven recorded drives the bridge refused in two seconds, on their own maps.

    **The fault, from the record.** The walk of 2026-10-05 kept the rover's
    centre 0.20 m from walls, which is the planner's test for a route, and let a
    goal count if it was within half a metre of that floor. The bridge's test
    for a goal is stricter: no part of the body may lie over the 253 ring, so the
    centre needs about twice the clearance. Seven drives in the record -- two of
    them in M3 session 3 -- were chosen on the first rule and refused on the
    second, "there is nowhere within half a metre of that spot where the rover's
    body fits". Here: the bridge's own `goal_fit.fit` on each recorded map
    refuses the goal, the walk now refuses it too, and the goal of the next drive
    that arrived is still somewhere to go.
    """
    import gzip
    import json

    import mapgrid

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures",
                        "no-room-for-the-body.json.gz")
    with gzip.open(path, "rt", encoding="utf-8") as handle:
        cases = json.load(handle)["cases"]
    check("the record holds the seven", len(cases), 7)
    goal_fit = mapgrid.goal_fit
    body = goal_fit.polygon_from("", BODY["inscribed_radius_m"])
    for case in cases:
        name = "episode %d" % case["episode"]
        recorded = a_situation(FINISHED)
        recorded["map"] = dict(case["map"])
        recorded["nav"]["pose"] = dict(case["pose"])
        here = _with_body(recorded)
        goal = case["goal"]
        x, y = float(goal["x_m"]), float(goal["y_m"])
        check(name + ": the bridge's own fit finds nowhere for the body",
              goal_fit.fit(_costmap_of(here, BODY["inscribed_radius_m"]), body,
                           x, y, math.radians(float(goal["heading_deg"]))),
              None)
        check(name + ": ...and the walk refuses the goal",
              here.reach.reachable(x, y), None)
        check(name + ": ...while the goal of the next drive that arrived is "
              "still somewhere to go",
              here.reach.reachable(case["arrived"]["x_m"],
                                   case["arrived"]["y_m"]) is not None, True)


def test_a_place_called_fit_is_one_the_bridge_accepts() -> None:
    """The walk's fitting floor never includes a cell the bridge would refuse.

    The walk stamps the body as every cell it could touch, so that it can be
    laid round every wall at once; the bridge lays its polygon down at one pose
    at a time. Checked on the cluttered room of a recorded map, at every cell the
    walk calls fit near the goal, and at the turns and millimetre offsets that
    change which edge cells the polygon touches.
    """
    import gzip
    import json

    import mapgrid

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures",
                        "no-room-for-the-body.json.gz")
    with gzip.open(path, "rt", encoding="utf-8") as handle:
        case = json.load(handle)["cases"][-1]
    recorded = a_situation(FINISHED)
    recorded["map"] = dict(case["map"])
    recorded["nav"]["pose"] = dict(case["pose"])
    here = _with_body(recorded)
    grid = here.grid
    goal_fit = mapgrid.goal_fit
    costmap = _costmap_of(here, BODY["inscribed_radius_m"])
    body = goal_fit.polygon_from("", BODY["inscribed_radius_m"])
    fits = mapgrid._where_the_body_fits(grid, BODY["inscribed_radius_m"])
    col, row = grid.cell_of(float(case["goal"]["x_m"]), float(case["goal"]["y_m"]))
    tried = refused = 0
    for r in range(row - 30, row + 31):
        for c in range(col - 30, col + 31):
            if not fits[r * grid.width + c]:
                continue
            px, py = grid.point_of(c, r)
            for dx, dy in ((0.0, 0.0), (0.0004, -0.0004), (-0.0004, 0.0004)):
                for turn in range(0, 30, 5):
                    tried += 1
                    if not goal_fit.fits(costmap, body, px + dx, py + dy,
                                         math.radians(turn)):
                        refused += 1
    check("the walk calls places fit near the goal", tried > 1000, True)
    check("...and the bridge refuses none of them, turned or shifted", refused, 0)


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


def test_a_viewpoint_faces_the_thing_it_is_for() -> None:
    """Found on 2026-10-03: runs 3 and 4 looked from each viewpoint facing
    wherever the drive left the rover, and in 16 of 19 the thing was more than
    30 degrees off the camera's axis, often behind it. The one goal that
    improved anything happened to face its thing."""
    here, _thing = _thing_and_room(90.0)
    found = [one for one in goals.improve_geometry(here)
             if one.target == "object:8"]
    check("there is a viewpoint", bool(found), True)
    for one in found:
        goal = one.constraints["goal"]
        toward = math.degrees(math.atan2(1.0 - goal["y_m"], 1.6 - goal["x_m"]))
        check(f"{one.id} is driven to facing the thing",
              round(goal.get("heading_deg", 999.0) - toward, 0), 0.0)
        check("...and the look is aimed at it",
              one.constraints.get("look_at"), {"x_m": 1.6, "y_m": 1.0})


def test_things_put_aside_do_not_crowd_out_the_rest_of_the_house() -> None:
    """Found on 2026-10-03: only the twelve worst-placed things were ever
    considered, so once those were put aside or had nowhere to stand, run 4 had
    nothing to do while 54 other placed things waited. A thing that is cooling,
    or that cannot be looked at, no longer takes up one of the twelve places."""
    at = 1757320800.0
    worst = [a_thing(f"object:{n}", 1.0 + 0.1 * n, 1.0, uncertainty_m=0.9)
             for n in range(goals.ENTITY_LIMIT)]
    shut = a_thing("object:90", 0.55, 0.30, uncertainty_m=0.95)
    far = a_thing("object:99", 2.8, 1.2, uncertainty_m=0.5)
    cooled = [{"target": one["id"], "since": at, "until": at + 900.0,
               "uncertainty_m": 0.9} for one in worst]
    here = _situation(CLOSET, entities=[*worst, shut, far], cooled=cooled, at=at)
    found = {one.target for one in goals.improve_geometry(here)}
    check("a thing outside the twelve worst is still considered",
          "object:99" in found, True)
    check("...and the ones put aside are still offered, to be refused aloud",
          {one["id"] for one in worst} <= found, True)


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
    test_a_gap_the_body_does_not_fit_through_is_not_a_way_there,
    test_the_pocket_by_the_charger_is_not_somewhere_to_drive,
    test_a_goal_with_no_room_for_the_body_is_not_somewhere_to_drive,
    test_a_place_called_fit_is_one_the_bridge_accepts,
    test_a_look_across_the_uncertainty_beats_a_look_along_it,
    test_no_placement_is_ever_predicted_better_than_this_rover_manages,
    test_a_thing_nobody_has_measured_the_distance_to_wants_the_depth_camera,
    test_a_thing_already_placed_well_is_not_worth_going_to,
    test_a_thing_from_a_map_that_has_gone_is_not_driven_to,
    test_a_thing_shut_in_a_room_of_its_own_is_refused_out_loud,
    test_two_viewpoints_are_offered_when_they_are_a_real_choice,
    test_a_viewpoint_faces_the_thing_it_is_for,
    test_things_put_aside_do_not_crowd_out_the_rest_of_the_house,
    test_the_height_it_cannot_predict_is_declared_rather_than_assumed,
    test_the_same_situation_produces_the_same_list_twice,
)
