"""Semantic world-state checks: the store, the pose, and what a look may claim.

These are the daemon's half of `world_state/` -- the tool calls that reach the
store, and the two facts a sighting is only allowed to assert from: the pose the
rover actually held when the frame was taken, and how far it could have seen in
that direction according to its own map.
"""
from __future__ import annotations

import json
import os
import time

from test_fakes import FakeLink
from test_harness import SKIP, check

def test_the_world_state_calls_reach_the_store():
    """The daemon's semantic-world calls, end to end, with no camera and no model.

    What is proved here is the wiring rather than the model: that an inspection
    takes its picture through the path that already owns the camera, that the
    gimbal angles and the rover's pose arrive on the observation, and that every
    one of these calls answers a browser in a sentence rather than raising on a
    rover where the component is not installed.

    The deterministic fake stands in for the perception sidecar, which is exactly
    what it is for and exactly what it does not settle: whether the world state is
    any good is a question only the rover and the real encoders can answer.
    """
    import tempfile

    import rover_daemon
    import world_state.view

    with tempfile.TemporaryDirectory() as directory:
        was = (os.environ.get("UGV_WORLD_DIR"), os.environ.get("UGV_WORLD_FAKE"))
        os.environ["UGV_WORLD_DIR"] = directory
        os.environ["UGV_WORLD_FAKE"] = "1"
        try:
            rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/null")
            # The camera path is the daemon's own; only the device under it is
            # replaced, so what is exercised here is `_world_capture` reading the
            # same frame `camera_jpeg` would.
            jpeg = bytes.fromhex("ffd8ffe0") + bytes(40) + bytes.fromhex("ffd9")
            # `when=True` is what `_world_capture` asks, because a bearing is
            # only as good as the heading at the instant the shutter opened.
            rover._whole_jpeg = lambda when=False: (
                (jpeg, "", time.time()) if when else (jpeg, ""))
            rover.pan, rover.tilt = 25.0, -8.0

            empty = rover.call("world_state_summary", {})
            check("an empty world answers", empty["ok"], True)
            check("...with nothing in it", empty["summary"]["entities"], 0)
            check("...and says which model would answer an inspection",
                  "fake" in empty["backend"], True)

            # The fake answers nothing, which is a finding rather than a failure:
            # a picture with nothing worth remembering in it should leave the world
            # alone and still be written down.
            looked = rover.call("world_inspect", {})
            check("an inspection runs", looked["ok"], True)
            check("...and an empty answer stores nothing", looked["stored"], 0)
            check("...and is recorded where the popup shows it",
                  rover.call("world_state_summary", {})["summary"]["inspections"], 1)

            # And with something in the answer, so the provenance can be checked.
            # The encoders are what an inspection asks now, so this is scripted
            # into them rather than into the language model -- which the daemon
            # still holds, for the conversational `look`.
            rover._world_inspector_cache.eyes.looks.append(
                [world_state.Sighting(bbox=[0.1, 0.3, 0.5, 0.9],
                                      dino=b"", siglip=b"")])
            saw = rover.call("world_inspect", {})
            check("a second inspection records what was measured",
                  saw["stored"], 1)
            check("...and claims no identity for it, which is now the whole point",
                  saw["created"], 0)
            entities = rover.call("world_state_entities", {})
            check("...so the entity list is empty and the observation is not",
                  (entities["entities"], len(entities["recent"])), ([], 1))
            observation = entities["recent"][0]
            check("the gimbal angles it was taken at are on the observation",
                  (observation["observer_pan_deg"], observation["observer_tilt_deg"]),
                  (25.0, -8.0))
            check("...and the box it was measured in is on it too",
                  observation["bbox"], [0.1, 0.3, 0.5, 0.9])
            # No navigator on this rover, so there is no pose and no ray -- which
            # is the honest answer rather than a ray from the origin.
            check("no navigator means no rover pose is claimed",
                  observation["pose"], None)
            check("...and therefore nothing that could ever place it",
                  world_state.view.ray(observation, rover.camera_fov_deg), None)

            frame = rover.call("world_state_frame",
                               {"frame_id": observation["frame_id"]})
            check("the picture it was read from can be fetched back",
                  frame["bytes"], len(jpeg))
            check("a frame that does not exist is refused rather than raising",
                  rover.call("world_state_frame", {"frame_id": "nope"})["ok"], False)

            # **A handler that returns a dict is not the same as a call that
            # answers.** Every one of these replies travels as one line of JSON,
            # and the console's detail pane was dead for months because one of
            # them could not: `world_state_entity` handed back the entity row
            # with its exemplars still in it as raw float32, the daemon failed to
            # encode the reply and wrote nothing at all, and the page waited
            # forever and said "nothing selected" for every thing anyone clicked.
            # So the assertion is on the encoded answer, not on its `ok`.
            store = rover._world_store()
            placed_id = store.create_entity()
            store.place(placed_id, {"x_m": 1.0, "y_m": 2.0, "uncertainty_m": 0.3,
                                    "baseline_m": 3.0, "parallax_deg": 30.0},
                        store.map_session())
            store.add_exemplar(placed_id, b"\x00\x00\x80\x3f" * 8)
            for name, arguments in (("world_state_summary", {}),
                                    ("world_state_entities", {}),
                                    ("world_state_entity", {"id": placed_id}),
                                    ("world_state_observations", {})):
                answer = rover.call(name, arguments)
                check(f"{name} answers", answer["ok"], True)
                try:
                    json.dumps(answer)
                    encoded = True
                except TypeError as error:
                    encoded = str(error)
                check("...and the answer can go down the wire", encoded, True)

            one = rover.call("world_state_entity", {"id": placed_id})
            check("the entity comes back under the id that was asked for",
                  one["entity"]["id"], placed_id)
            check("...with its position decoded rather than as stored JSON",
                  (one["entity"].get("placement") or {}).get("x_m"), 1.0)
            check("...and no raw vector anywhere in it",
                  "exemplars" in one["entity"], False)
            check("...but a count of the ones it holds",
                  one["entity"]["exemplar_count"], 1)
            check("an entity that does not exist is refused in a sentence",
                  rover.call("world_state_entity", {"id": "object:404"})["ok"],
                  False)

            session = rover.call("world_map_session", {})
            check("clearing the map starts a new session", session["map_session"], 2)
            check("...and deletes nothing",
                  rover.call("world_state_summary", {})["summary"]["observations"],
                  1)

            # A store with more in it than one reply may carry, which is what
            # the console's observation stream walks back through. The page
            # starts below a row the caller names rather than at a count of rows
            # to skip, because the rover goes on recording while it is read.
            for _ in range(3):
                store.record([world_state.Sighting(bbox=[0.2, 0.2, 0.4, 0.4],
                                                   dino=b"", siglip=b"")],
                             capture={"frame_id": "f"})
            page = rover.call("world_state_observations", {"limit": 3})
            check("the history answers a page at a time",
                  len(page["observations"]), 3)
            check("...and says there is more under it", page["more"], True)
            oldest = page["observations"][-1]
            under = rover.call("world_state_observations",
                               {"limit": 3, "before_at": oldest["observed_at"],
                                "before_id": oldest["id"]})
            check("...the next page starting below the row it was given",
                  [row["id"] for row in under["observations"]], [oldest["id"] - 1])
            check("...and saying so when the store has run out",
                  under["more"], False)

            cleared = rover.call("world_state_clear", {})
            check("clearing the semantic world empties it", cleared["ok"], True)
            check("...of observations",
                  rover.call("world_state_summary", {})["summary"]["observations"],
                  0)
            check("...and of the pictures it kept", cleared["frames_removed"], 2)
            rover.close_world()
        finally:
            for name, value in zip(("UGV_WORLD_DIR", "UGV_WORLD_FAKE"), was):
                if value is None:
                    os.environ.pop(name, None)
                else:
                    os.environ[name] = value


def test_a_search_hands_back_the_whole_of_every_look_it_matched() -> None:
    """A match has to be an observation, not just the columns that ranked it.

    The console narrows its entity list, its map and its observation stream down
    to what a search answered, so a match is opened in that stream exactly as an
    ordinary look is. Ranked over the vector columns alone, those rows would come
    back without a pose, a source or the measurement behind them -- and the same
    look with the box emptied shows all three, which makes the filter look like
    it lost them.
    """
    import struct
    import tempfile

    try:
        import rover_daemon
        import world_state
    except ImportError as exc:
        SKIP.append(f"the world-state search ({type(exc).__name__})")
        return

    with tempfile.TemporaryDirectory() as directory:
        was = (os.environ.get("UGV_WORLD_DIR"), os.environ.get("UGV_WORLD_FAKE"))
        os.environ["UGV_WORLD_DIR"] = directory
        os.environ["UGV_WORLD_FAKE"] = "1"
        try:
            rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/null")
            eyes = rover._world_inspector().eyes
            # The fake turns a phrase into a vector of its own, so storing the
            # phrase's own vector is a region that matches it exactly. That
            # proves the wiring and nothing whatever about SigLIP2.
            wanted, error = eyes.embed(["a spray bottle"])
            check("the stand-in sidecar embeds a phrase", error, "")
            store = rover._world_store()
            store.record([world_state.Sighting(bbox=[0.1, 0.2, 0.3, 0.4],
                                               dino=b"", siglip=wanted[0]),
                          world_state.Sighting(bbox=[0.5, 0.5, 0.6, 0.6],
                                               dino=b"",
                                               siglip=struct.pack("<8f", *([0.0] * 7 + [1.0])))],
                         capture={"frame_id": "f1", "pan": 12.0, "tilt": -3.0,
                                  "pose": {"x_m": 1.0, "y_m": 2.0,
                                           "heading_deg": 90.0}},
                         source="perception", fov_deg=66.0)

            answer = rover.call("world_state_search", {"query": "a spray bottle"})
            check("a phrase is compared with what the rover saw", answer["ok"], True)
            check("...and the region that is it comes top",
                  answer["matches"][0]["bbox"], [0.1, 0.2, 0.3, 0.4])
            check("...believed, because the score clears the floor",
                  answer["confident"], True)
            check("...and the floor rides with the answer, so a page marking "
                  "rows against it need not hold its own copy",
                  answer["floor"], world_state.search.MATCHES)

            match = answer["matches"][0]
            check("a match carries the row identifier the stream keys on",
                  match["id"], match["observation_id"])
            check("...the pose the look was taken from",
                  (match["pose"]["x_m"], match["pose"]["heading_deg"]), (1.0, 90.0))
            check("...where the gimbal was pointing",
                  (match["observer_pan_deg"], match["observer_tilt_deg"]),
                  (12.0, -3.0))
            check("...what took it", match["source"], "perception")
            check("...and no raw vector, which would not go down the wire",
                  "siglip_blob" in match, False)
            try:
                json.dumps(answer)
                encoded = True
            except TypeError as bad:
                encoded = str(bad)
            check("...so the whole answer travels as one line of JSON",
                  encoded, True)

            check("a phrase nothing was stored against is refused rather than "
                  "answered", rover.call("world_state_search", {"query": " "})["ok"],
                  False)
            rover.close_world()
        finally:
            for name, value in zip(("UGV_WORLD_DIR", "UGV_WORLD_FAKE"), was):
                if value is None:
                    os.environ.pop(name, None)
                else:
                    os.environ[name] = value


def test_a_rover_without_a_camera_refuses_to_inspect():
    """Every one of these is reached from a page of live buttons, so a rover that
    cannot do it has to say so in a sentence."""
    import tempfile

    import rover_daemon

    with tempfile.TemporaryDirectory() as directory:
        was = os.environ.get("UGV_WORLD_DIR")
        os.environ["UGV_WORLD_DIR"] = directory
        os.environ["UGV_WORLD_FAKE"] = "1"
        try:
            rover = rover_daemon.Rover(FakeLink(), "unused", device=None)
            blind = rover.call("world_inspect", {})
            check("an inspection with no camera is refused", blind["ok"], False)
            check("...as a missing camera rather than as a missing model",
                  "no camera" in blind["error"], True)
            check("...and changes nothing",
                  rover.call("world_state_summary", {})["summary"]["observations"], 0)
            check("...but is written down",
                  rover.call("world_state_summary", {})["summary"]["last_status"],
                  "no_frame")
            rover.close_world()
        finally:
            os.environ.pop("UGV_WORLD_FAKE", None)
            if was is None:
                os.environ.pop("UGV_WORLD_DIR", None)
            else:
                os.environ["UGV_WORLD_DIR"] = was


def test_the_camera_is_asked_twice_before_an_inspection_is_lost():
    """An empty grab in front of a look is retried rather than recorded as a loss.

    The fault this was written for turned out to be two grabs overlapping rather
    than one following another closely, and `_snapshot` now holds the camera for
    the length of a grab so that cannot happen -- see rover_camera. The retry
    stays as the backstop for an empty grab from any other cause, and this is
    what checks it still asks twice and no more.
    """
    import tempfile

    import rover_daemon

    with tempfile.TemporaryDirectory() as directory:
        was = os.environ.get("UGV_WORLD_DIR")
        os.environ["UGV_WORLD_DIR"] = directory
        os.environ["UGV_WORLD_FAKE"] = "1"
        try:
            rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/null")
            jpeg = bytes.fromhex("ffd8ffe0") + bytes(40) + bytes.fromhex("ffd9")
            tries = []

            def flaky(when=False):
                tries.append(1)
                if len(tries) == 1:
                    return ((None, "the camera gave no whole picture", None)
                            if when else
                            (None, "the camera gave no whole picture"))
                return (jpeg, "", time.time()) if when else (jpeg, "")

            rover._whole_jpeg = flaky
            frame = rover._world_capture()
            check("the second attempt gets the picture", frame["ok"], True)
            check("...having asked exactly twice", len(tries), 2)

            tries.clear()
            def never(when=False):
                tries.append(1)
                return ((None, "the camera gave no whole picture", None)
                        if when else (None, "the camera gave no whole picture"))

            rover._whole_jpeg = never
            frame = rover._world_capture()
            check("a camera that is really gone still fails", frame["ok"], False)
            check("...saying that both attempts failed",
                  "and again" in frame["error"], True)
            check("...after two attempts and no more", len(tries), 2)

            # While tracking runs there is no second attempt to make: the loop owns
            # the camera and hands back whatever its newest frame is, so asking
            # again half a second later asks the same question of the same frame.
            tries.clear()
            rover._tracking.set()
            rover._world_capture()
            check("a tracking rover is asked once, because it is one frame",
                  len(tries), 1)
            rover._tracking.clear()
            rover.close_world()
        finally:
            os.environ.pop("UGV_WORLD_FAKE", None)
            if was is None:
                os.environ.pop("UGV_WORLD_DIR", None)
            else:
                os.environ["UGV_WORLD_DIR"] = was


def test_a_world_observation_takes_the_live_pose_and_no_other() -> None:
    """Where an inspection says the rover was standing.

    **The fault this reproduces was recorded on the rover.** `_world_pose` used to
    read `nav.slam.pose`, which is not where the rover is: it is the pose printed
    on the last map picture somebody asked for, and `nav.slam` starts life as a
    placeholder whose pose is the map origin. So a daemon that had just restarted
    with no console watching put twenty-two regions on bearings drawn from (0, 0),
    those crossed real bearings 4.8 m away, and six things were placed in a room
    that never held them.
    """
    import rover_world

    class Nav:
        """A navigator whose map cache and whose live position disagree, which is
        the ordinary case: the cache is as old as the last drawn map."""

        class slam:
            pose = (0.0, 0.0, 0.0)

        def __init__(self, trusted=True, pose=None):
            self._trusted = trusted
            self._pose = pose

        def status(self):
            return {"position_trusted": self._trusted, "pose": self._pose}

    class Asking:
        _world_pose = rover_world.RoverWorld._world_pose

    rover = Asking()
    rover.nav = Nav(True, {"x_m": 3.25, "y_m": -1.5, "heading_deg": 44.0})
    check("the pose is the live one, not the one on the last map drawn",
          rover._world_pose(),
          {"x_m": 3.25, "y_m": -1.5, "heading_deg": 44.0})

    rover.nav = Nav(False, {"x_m": 3.25, "y_m": -1.5, "heading_deg": 44.0})
    check("a position the navigator does not trust is no position at all",
          rover._world_pose(), None)

    rover.nav = Nav(True, None)
    check("...and neither is a navigator that has no pose to give",
          rover._world_pose(), None)

    class Broken:
        def status(self):
            raise OSError("the bridge is not answering")

    rover.nav = Broken()
    check("a bridge that is down leaves the observation without a bearing",
          rover._world_pose(), None)


def test_how_far_the_rover_could_see_comes_off_its_own_map() -> None:
    """The range bound the world state was missing, read out of the grid.

    The resolver cannot ask this for itself -- the map belongs to the navigator --
    so the daemon answers it, and the answer is what stops a thing being placed
    through a wall. Two things have to come out right: a wall stops the walk, and
    so does the edge of what has been mapped, because the rover has never seen
    anything out there either.
    """
    try:
        import numpy
    except ImportError as error:
        SKIP.append(f"reading reach off the occupancy grid ({error})")
        return

    import base64
    import zlib

    import rover_world

    # A 4 m x 4 m room at 10 cm cells with its origin at the corner, free
    # everywhere except a wall two metres along the +x axis.
    cells = numpy.zeros((40, 40), dtype=numpy.int8)
    cells[:, 20] = 100
    payload = {
        "ok": True, "width": 40, "height": 40, "resolution_m": 0.1,
        "origin_x_m": 0.0, "origin_y_m": 0.0,
        "data": base64.b64encode(zlib.compress(cells.tobytes())).decode(),
    }

    class Nav:
        asked = 0

        def ask(self, request, timeout_s):
            Nav.asked += 1
            return payload

    class Asking:
        _world_grid = rover_world.RoverWorld._world_grid
        _world_reach = rover_world.RoverWorld._world_reach

    rover = Asking()
    rover.nav = Nav()

    # Good to the grid's own resolution and never longer than the truth, which
    # is the direction that errs toward refusing a placement.
    def within_a_cell(got, want):
        return got is not None and want - 0.1 <= got <= want

    check("a wall two metres ahead bounds the sighting there",
          within_a_cell(rover._world_reach(0.5, 2.0, 0.0), 1.5), True)
    check("...and the map's own edge bounds it when there is no wall",
          within_a_cell(rover._world_reach(0.5, 2.0, 180.0), 0.5), True)
    check("a bearing along a clear line runs to the far edge",
          within_a_cell(rover._world_reach(0.5, 2.0, 90.0), 2.0), True)

    # The grid is fetched once and reused, because one resolve pass asks this for
    # every bearing in the pending pool.
    before = Nav.asked
    for _ in range(20):
        rover._world_reach(0.5, 2.0, 0.0)
    check("...and the map is not refetched for every bearing",
          Nav.asked, before)

    class Down:
        def ask(self, request, timeout_s):
            return {"ok": False, "error": "the bridge is not answering"}

    rover = Asking()
    rover.nav = Down()
    check("no map means the bearing is left unbounded, not refused",
          rover._world_reach(0.5, 2.0, 0.0), None)


def test_where_to_stand_to_look_at_a_thing_is_a_place_on_this_map() -> None:
    """The call behind the console's "go to" button, with the three refusals.

    The geometry itself is proved in `world_state/test_approach.py` against rooms
    drawn by hand. What is proved here is the daemon's half: that the answer is
    worked out from the live pose and the live map rather than from either of the
    caches beside them, that a thing with no position or a position measured in a
    map that has since been cleared is refused in a sentence instead of driving
    the rover to a coordinate that means nothing now, and that the answer goes
    down the wire -- which the numpy grid underneath it makes a real question.
    """
    try:
        import numpy
    except ImportError as error:
        SKIP.append(f"choosing where to stand to see a thing ({error})")
        return

    import base64
    import tempfile
    import zlib

    import rover_daemon
    import world_state

    # Six metres square at 10 cm cells, open floor, with the rover at one end and
    # a thing three metres away at the other.
    cells = numpy.zeros((60, 60), dtype=numpy.int8)
    payload = {
        "ok": True, "width": 60, "height": 60, "resolution_m": 0.1,
        "origin_x_m": 0.0, "origin_y_m": 0.0,
        "data": base64.b64encode(zlib.compress(cells.tobytes())).decode(),
    }

    class Nav:
        def __init__(self, pose):
            self._pose = pose

        def status(self):
            return {"position_trusted": self._pose is not None, "pose": self._pose}

        def ask(self, request, timeout_s):
            return payload

    with tempfile.TemporaryDirectory() as directory:
        was = (os.environ.get("UGV_WORLD_DIR"), os.environ.get("UGV_WORLD_FAKE"))
        os.environ["UGV_WORLD_DIR"] = directory
        os.environ["UGV_WORLD_FAKE"] = "1"
        try:
            rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/null")
            rover.nav = Nav({"x_m": 1.0, "y_m": 3.0, "heading_deg": 90.0})
            store = rover._world_store()
            thing = store.create_entity()

            check("a thing with no position has nowhere to be looked at from",
                  rover.call("world_state_viewpoint", {"id": thing})["ok"], False)
            check("...and an entity that does not exist is refused too",
                  rover.call("world_state_viewpoint", {"id": "object:404"})["ok"],
                  False)

            store.place(thing, {"x_m": 4.0, "y_m": 3.0, "uncertainty_m": 0.2},
                        store.map_session())
            found = rover.call("world_state_viewpoint", {"id": thing})
            check("a placed thing has a viewpoint", found["ok"], True)
            check("...on the rover's side of it, at the near bound",
                  (found["x_m"], found["y_m"]), (3.2, 3.0))
            check("...facing it", found["heading_deg"], 0.0)
            # The rover is pointed across the room, so the turn is the whole of
            # the difference -- and it is what the console sends when the drive
            # is too short to be worth planning.
            check("...and the turn is measured from where it is pointing now",
                  found["turn_deg"], -90.0)
            check("...with the drive that is left over", found["travel_m"], 2.2)
            try:
                json.dumps(found)
                encoded = True
            except TypeError as error:
                encoded = str(error)
            check("...and the answer goes down the wire as one line of JSON",
                  encoded, True)
            check("...off the ring, there being no look to read a line from",
                  found["along"], "the nearest floor it can be seen from")

            # And now the thing has been looked at, twice, from the north of it.
            # Both looks are attached to it and both agree with where it was
            # placed, so the north side is a line the rover has actually seen it
            # along -- which is what it is sent to, in preference to the near side
            # it was sent to a moment ago.
            for pan in (89.0, 91.0):
                store.record([world_state.Sighting(bbox=[0.45, 0.45, 0.55, 0.55],
                                                   dino=b"", siglip=b"")],
                             capture={"frame_id": "north", "pan": pan, "tilt": 0.0,
                                      "pose": {"x_m": 4.0, "y_m": 5.0,
                                               "heading_deg": 0.0}},
                             source="perception", fov_deg=66.0)
            store.attach(thing, [row["id"] for row
                                 in store.observations(limit=2)], why="the test")
            seen = rover.call("world_state_viewpoint", {"id": thing})
            check("a thing that has been seen from somewhere is approached from "
                  "there", seen["along"],
                  "the line it has most often been seen along")
            check("...which is the north side of it", seen["y_m"] > 3.0, True)
            check("...on the line rather than beside it", round(seen["x_m"], 1),
                  4.0)
            check("...facing it", round(seen["heading_deg"]), -90.0)
            check("...reading both of the looks behind it", seen["sight_lines"], 2)

            # Clearing the map does not delete the entity, and that is exactly
            # the trap: its coordinates are still there and they now name a place
            # in a map that has never existed.
            rover.call("world_map_session", {})
            stale = rover.call("world_state_viewpoint", {"id": thing})
            check("a position measured under a map that is gone is refused",
                  stale["ok"], False)
            check("...saying which map it was measured in",
                  "was placed under map 1" in stale["error"], True)

            # Placed again under the new map, the thing is approachable again --
            # and the looks behind it are not. Where the rover was standing is a
            # position in the map of the day, so a pose recorded before the clear
            # names a place in this map by coincidence, and the north side has to
            # be earned again rather than remembered.
            store.place(thing, {"x_m": 4.0, "y_m": 3.0, "uncertainty_m": 0.2},
                        store.map_session())
            afresh = rover.call("world_state_viewpoint", {"id": thing})
            check("a look taken under the old map is not a line on this one",
                  afresh["sight_lines"], 0)
            check("...so the ring answers instead", afresh["along"],
                  "the nearest floor it can be seen from")

            rover.nav = Nav(None)
            store.place(thing, {"x_m": 4.0, "y_m": 3.0}, store.map_session())
            lost = rover.call("world_state_viewpoint", {"id": thing})
            check("a rover that does not know where it is refuses to choose",
                  lost["ok"], False)
            check("...as a missing position rather than a missing map",
                  "publishing the rover's position" in lost["error"], True)
            rover.close_world()
        finally:
            for name, value in zip(("UGV_WORLD_DIR", "UGV_WORLD_FAKE"), was):
                if value is None:
                    os.environ.pop(name, None)
                else:
                    os.environ[name] = value


def test_the_rover_looks_when_there_is_something_new_to_see() -> None:
    """When building the world state by itself, what is worth a look.

    Not a timer on its own. Observations taken from a place the rover has already
    looked from cannot be triangulated against the ones already there -- they need
    a baseline and there is none -- and every one of them enlarges the pool the
    resolver reads on every later look. So a parked rover recording every fifteen
    seconds gets steadily slower at placing things and no better at it.
    """
    import rover_world

    class Standing:
        """Enough of a rover to answer the question, and nothing else."""

        def __init__(self, pose):
            self._world_build_at = 0.0
            self._world_build_from = None
            self._pose = pose
            self.pan = 0.0

        def _world_pose(self):
            return self._pose

        worth = rover_world.RoverWorld._world_worth_looking
        _world_camera_deg = rover_world.RoverWorld._world_camera_deg

    here = {"x_m": 1.0, "y_m": 2.0, "heading_deg": 30.0}
    rover = Standing(dict(here))
    check("a rover that has never looked, looks", rover.worth(1000.0), True)

    rover._world_build_from = dict(here, camera_deg=30.0)
    rover._world_build_at = 1000.0
    check("...and having just looked, does not look again",
          rover.worth(1000.0 + rover_world.LOOK_EVERY_S - 1), False)
    check("...nor a while later from the very same spot",
          rover.worth(1000.0 + rover_world.LOOK_EVERY_S + 5), False)

    later = 1000.0 + rover_world.LOOK_EVERY_S + 5
    rover._pose = dict(here, x_m=here["x_m"] + rover_world.MOVED_ENOUGH_M + 0.05)
    check("a step to somewhere new is worth a look", rover.worth(later), True)

    rover._pose = dict(here,
                       heading_deg=here["heading_deg"] + rover_world.TURNED_ENOUGH_DEG + 1)
    check("...and so is turning to face somewhere new", rover.worth(later), True)

    # **Where the camera points, not where the chassis does.** Swinging the gimbal
    # across the room from a standstill is the one way a rover that has not moved
    # can still see something new, and measured against the chassis alone it
    # counted as nothing -- which is what the rover actually did.
    rover._pose = dict(here)
    rover.pan = rover_world.TURNED_ENOUGH_DEG + 1
    check("turning only the gimbal is a new direction too",
          rover.worth(later), True)
    rover.pan = 0.0

    # The direction is an angle, so 359 degrees away is one degree away.
    rover._pose = dict(here, heading_deg=here["heading_deg"] - 2.0 + 360.0)
    check("...but two degrees the other side of north is not a new direction",
          rover.worth(later), False)

    rover._pose = dict(here)
    check("a rover that has stood still for a long time looks anyway",
          rover.worth(1000.0 + rover_world.LOOK_ANYWAY_S + 1), True)

    # A pose the navigator will not give is a bearing that cannot be measured --
    # but not a picture that cannot be taken, and it used to stop both. A rover
    # whose scan matcher has lost confidence then fell back to one look every five
    # minutes, while it was driving through the very part of the building that had
    # confused it. It keeps looking now, slower, and stores frames with no bearing.
    rover._pose = None
    check("with no pose it does not look on the ordinary clock",
          rover.worth(1000.0 + rover_world.LOOK_EVERY_S + 0.1), False)
    check("...but it does keep taking pictures, on a slower one",
          rover.worth(1000.0 + rover_world.LOOK_BLIND_S + 0.1), True)


def test_the_camera_the_world_is_built_through_is_chosen_in_code() -> None:
    """The rover has two cameras that see the room and they write into one world.

    **The switch is a constant in this file and nothing on the console**, which is
    the point of the test: two cameras writing bearings into one store through a
    switch a person could flip mid-drive would leave it holding two halves
    measured through different optics from different places. Which one took a look
    is recorded on the row, so the question stays answerable afterwards; what must
    not happen is the answer changing under a running experiment.
    """
    import rover_world
    import world_state

    check("the default is the camera every bearing so far was drawn through",
          rover_world.WORLD_CAMERA, world_state.oak.GIMBAL)

    class Standing:
        """Enough of a rover to be asked which way it is looking."""

        def __init__(self):
            self.pan = 40.0

        _world_camera_deg = rover_world.RoverWorld._world_camera_deg

    rover = Standing()
    pose = {"x_m": 0.0, "y_m": 0.0, "heading_deg": 10.0}
    check("through the gimbal, swinging the servo is a new direction",
          rover._world_camera_deg(pose), -30.0)

    # **Through the OAK it is not**, and reading the gimbal's pan there would have
    # a parked rover think it had found somewhere new to look every time the
    # tracking loop moved a servo the depth camera cannot see through.
    was = rover_world.WORLD_CAMERA
    rover_world.WORLD_CAMERA = world_state.oak.OAK
    try:
        check("through the OAK, only the rover turning is",
              round(rover._world_camera_deg(pose), 1),
              round(10.0 - world_state.oak.pan_deg(), 1))
    finally:
        rover_world.WORLD_CAMERA = was


def test_a_look_through_the_oak_needs_the_mount_measured() -> None:
    """A bearing through a camera nobody has located is a guess with a number on it.

    Refused rather than recorded without one, which is the opposite of what a
    missing pose earns -- and the difference is that a missing pose passes. *Every*
    look through this camera would be bearingless until somebody ran the bench, and
    a rover quietly filling its store with directionless pictures looks exactly
    like one that is working.
    """
    import rover_world
    import world_state

    class Blind:
        _world_capture_oak = rover_world.RoverWorld._world_capture_oak

        def _world_ranger(self):
            return world_state.FakeRanger()

    was_mount, was_measured = world_state.oak.MOUNT, world_state.oak.MEASURED
    world_state.oak.MEASURED = False
    try:
        answer = Blind()._world_capture_oak()
        check("an unmeasured mount refuses the look", answer["ok"], False)
        check("...and says which bench measures it",
              "bench_oak" in answer["error"], True)
    finally:
        world_state.oak.MOUNT = was_mount
        world_state.oak.MEASURED = was_measured

    class Nothing:
        _world_capture_oak = rover_world.RoverWorld._world_capture_oak

        def _world_ranger(self):
            return None

    check("a rover with no depth camera refuses it too, and says so",
          Nothing()._world_capture_oak()["ok"], False)

    # And with both in place, the picture comes back stamped with the mount as a
    # pan and a tilt -- which is what makes `view.ray` treat this camera as a
    # gimbal that never moves and need no new arithmetic at all.
    class Looking:
        _world_capture_oak = rover_world.RoverWorld._world_capture_oak

        def _world_ranger(self):
            return world_state.FakeRanger()

    world_state.oak.MOUNT = world_state.oak.Mount(yaw_deg=-0.7, pitch_deg=2.7)
    world_state.oak.MEASURED = True
    try:
        got = Looking()._world_capture_oak()
        check("a measured rover takes the picture", got["ok"], True)
        check("...and records the mount as the camera's own pan and tilt",
              (got["camera"], got["pan"], got["tilt"]), ("oak", -0.7, 2.7))
    finally:
        world_state.oak.MOUNT = was_mount
        world_state.oak.MEASURED = was_measured


TESTS = (
    test_a_search_hands_back_the_whole_of_every_look_it_matched,
    test_the_world_state_calls_reach_the_store,
    test_a_rover_without_a_camera_refuses_to_inspect,
    test_the_camera_is_asked_twice_before_an_inspection_is_lost,
    test_a_world_observation_takes_the_live_pose_and_no_other,
    test_how_far_the_rover_could_see_comes_off_its_own_map,
    test_where_to_stand_to_look_at_a_thing_is_a_place_on_this_map,
    test_the_rover_looks_when_there_is_something_new_to_see,
    test_the_camera_the_world_is_built_through_is_chosen_in_code,
    test_a_look_through_the_oak_needs_the_mount_measured,
)
