"""What a decision is made from: that it is read once, whole, and replays.

The check that matters most here is the dullest one. An occupancy map travels as
bytes, unknown is -1, and a byte cannot hold -1 -- so a decode that forgets the
sign turns every unexplored cell into a wall, and a rover with a house left to
explore reports that there is nothing to explore anywhere. Nothing about that
looks broken from the outside, which is exactly why it is checked here.
"""
from __future__ import annotations

import tempfile

import client
import situation as situation_mod
from store import EpisodeStore
from test_fakes import FakeRover, a_situation, a_thing, rover_at
from test_harness import check

ROOM = [
    "########################",
    "#......................#",
    "#..........R...........#",
    "#......................#",
    "##########.....#########",
    "?????????.......????????",
    "????????????????????????",
]


def test_the_situation_is_read_through_the_door_that_only_opens_outwards() -> None:
    rover = FakeRover(room=ROOM, entities=[a_thing("object:1", 1.0, 1.0)])
    here = situation_mod.Situation.read(rover, at=1757320800.0)
    check("every call it made is one the client allows",
          sorted(set(rover.asked)) == sorted(set(rover.asked) & client.ALLOWED),
          True)
    check("...and it asked for the map", "nav_grid" in rover.asked, True)
    check("it read the world's generation", here.world_generation,
          rover.generation)
    check("...the battery", here.battery_v, 12.07)
    check("...and where the rover is standing, which is where the R is",
          (round(here.pose["x_m"], 2), round(here.pose["y_m"], 2)),
          tuple(round(one, 2) for one in rover_at(ROOM)))


def test_an_unknown_cell_does_not_read_as_a_wall() -> None:
    """The decode's one bug, and the only one that would be invisible."""
    here = situation_mod.Situation(a_situation(ROOM))
    grid = here.grid
    col, row = grid.cell_of(*rover_at(ROOM))
    check("the floor the rover stands on reads as free", grid.at(col, row), 0)
    # Bottom row of the drawing is the lowest y, and it is all unseen.
    check("ground nobody has seen reads as unknown", grid.at(1, 0), -1)
    check("...and a wall reads as a wall", grid.at(0, grid.height - 1), 100)
    check("the rover can walk to floor in its own room",
          here.reach.reachable(*rover_at(ROOM)), 0.0)


def test_a_rover_with_no_map_says_so_rather_than_reporting_an_empty_house() -> None:
    rover = FakeRover(room=None)
    here = situation_mod.Situation.read(rover, at=1757320800.0)
    check("there is no grid", here.grid, None)
    check("...and the reason is the mapper's own",
          "slam_toolbox" in here.grid_error, True)
    check("...which is a reason the rover is not fit to be asked to move",
          "no_map" in here.health(), True)


def test_the_things_that_would_stop_a_decision_are_named() -> None:
    check("a healthy rover has nothing wrong with it",
          situation_mod.Situation(a_situation(ROOM)).health(), {})
    latched = situation_mod.Situation(a_situation(ROOM, estop=True))
    check("a latched stop is named", "estop" in latched.health(), True)
    lost = situation_mod.Situation(a_situation(ROOM, position_trusted=False))
    check("not knowing where it is, is named", "pose" in lost.health(), True)
    unsettled = situation_mod.Situation(a_situation(ROOM, map_settled=False))
    check("a map that has not settled is named",
          "map" in unsettled.health(), True)
    parked = situation_mod.Situation(a_situation(
        ROOM, world={"last_at": 1757320800.0 - 300.0}))
    check("a parked rover looking every five minutes is not unfit",
          "looking" in parked.health(), False)
    stale = situation_mod.Situation(a_situation(
        ROOM, world={"last_at": 1757320800.0 - 1200.0}))
    check("...but a perception loop that has stopped is named",
          "looking" in stale.health(), True)
    failing = situation_mod.Situation(a_situation(
        ROOM, world={"last_status": "error", "last_detail": "the camera died"}))
    check("...and so is one whose last look failed",
          "world_state" in failing.health(), True)


def test_a_rover_off_the_floor_can_be_routed_from_nowhere() -> None:
    """A pose nowhere near mapped floor is a real state, not a bug here.

    It happens when the match has drifted or the map was replaced under the
    rover, and the honest answer is that no route can be planned from a place
    the walk cannot start at -- not a route ranked badly.
    """
    body = a_situation(ROOM)
    body["nav"]["pose"] = {"x_m": 5.0, "y_m": 5.0, "heading_deg": 0.0}
    here = situation_mod.Situation(body)
    check("standing inside a wall is reported",
          "off_the_floor" in here.health(), True)


def test_what_it_was_read_from_is_what_gets_written_down() -> None:
    """The snapshot and the object that decided must not be two things."""
    with tempfile.TemporaryDirectory() as directory:
        store = EpisodeStore(directory)
        here = situation_mod.Situation(a_situation(
            ROOM, entities=[a_thing("object:1", 1.0, 1.0)]))
        digest = store.snapshot("situation", here.as_dict())
        again = situation_mod.Situation.from_dict(store.snapshot_body(digest))
        check("it comes back the same", again.as_dict(), here.as_dict())
        check("...including the map", again.grid.width, here.grid.width)
        check("an unchanged reading costs one row",
              store.snapshot("situation", here.as_dict()), digest)
        store.close()


def test_a_situation_holds_no_handle_on_anything() -> None:
    """Read on the rover, reasoned about anywhere: the object is plain data."""
    import json

    here = situation_mod.Situation(a_situation(
        ROOM, entities=[a_thing("object:1", 1.0, 1.0)]))
    text = json.dumps(here.as_dict())
    check("it survives a round trip through JSON",
          situation_mod.Situation.from_json(text).where, here.where)


TESTS = (
    test_the_situation_is_read_through_the_door_that_only_opens_outwards,
    test_an_unknown_cell_does_not_read_as_a_wall,
    test_a_rover_with_no_map_says_so_rather_than_reporting_an_empty_house,
    test_the_things_that_would_stop_a_decision_are_named,
    test_a_rover_off_the_floor_can_be_routed_from_nowhere,
    test_what_it_was_read_from_is_what_gets_written_down,
    test_a_situation_holds_no_handle_on_anything,
)
