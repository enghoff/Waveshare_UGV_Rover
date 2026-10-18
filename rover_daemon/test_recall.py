"""What a voice model may ask of the room the rover has already looked at.

Two tools, and what is checked is the part that is easy to get silently wrong:
that a phrase reaches the same ranking the console uses, that the answer comes
back in metres and words with none of the console's own vocabulary in it, and
that "go to the desk" starts a drive and comes straight back rather than holding
the one connection a model has for the whole trip.

The position a found thing carries is the exception to that and is checked as
one: it is what lets two answers be compared, which is how a question about two
things gets answered now that no tool measures between them.

The deterministic fake sidecar stands in for SigLIP2, which is exactly what it is
for and exactly what it does not settle: whether a phrase really finds a bed is a
question only the rover and the real encoders can answer.
"""
from __future__ import annotations

import json
import math
import os
import struct
import tempfile

from test_fakes import FakeLink
from test_harness import SKIP, check


class Nav:
    """A navigator that knows where the rover is, hands over a map, and
    remembers where it was sent.

    The background machinery itself is `RosNavigator`'s and is checked against a
    bridge in test_ros_nav.py; what this stands in for is the shape of it --
    `driving`, `errand` and an errand that answers at once.
    """

    def __init__(self, pose, grid) -> None:
        self._pose = pose
        self._grid = grid
        self.driving = False
        self.errand: dict = {}
        self.away_for = 0.0
        self.ran_errand = None
        self.sent: list = []
        self.stops = 0

    def status(self):
        return {"position_trusted": self._pose is not None, "pose": self._pose,
                "map_id": "map-one", "map_settled": True}

    def ask(self, request, timeout_s):
        return self._grid

    def stop(self, latch: bool = False):
        self.stops += 1
        self.driving = False
        self.errand = {}
        return {"stopped": True, "latched": False}

    def drive_to_in_background(self, x_m, y_m, heading_deg=None, speed_ms=None,
                               for_what=None):
        self.sent.append({"x_m": x_m, "y_m": y_m, "heading_deg": heading_deg,
                          "for_what": dict(for_what or {})})
        self.errand = {**(for_what or {}), "x_m": x_m, "y_m": y_m}
        self.driving = True
        return {"started": True, "running": "errand", "running_s": 0.0}


def _open_room():
    """Six metres square of mapped floor at 10 cm cells, as the navigator sends
    it: zlib over the raw bytes, base64 over that."""
    import base64
    import zlib

    import numpy

    cells = numpy.zeros((60, 60), dtype=numpy.int8)
    return {"ok": True, "width": 60, "height": 60, "resolution_m": 0.1,
            "origin_x_m": 0.0, "origin_y_m": 0.0,
            "data": base64.b64encode(zlib.compress(cells.tobytes())).decode()}


def _orthogonal_to(blob: bytes) -> bytes:
    """A vector that scores zero against the given one.

    The fake sidecar hashes a phrase into eight non-negative numbers, so two
    unrelated phrases still score highly against each other and the floor cannot
    be exercised by choosing a different phrase. A stored vector is arbitrary
    bytes, though, so one can simply be built at right angles to the query -- which
    is what "the rover has never seen anything like that" looks like to the
    ranking.
    """
    query = struct.unpack("<8f", blob)
    scale = query[0] / sum(value * value for value in query)
    return struct.pack("<8f", *[(1.0 if index == 0 else 0.0) - scale * value
                                for index, value in enumerate(query)])


def _a_rover(directory, pose=(1.0, 3.0, 90.0)):
    """A daemon with a world state, a map and somewhere to stand."""
    import rover_daemon

    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/null")
    rover.nav = Nav(None if pose is None else
                    {"x_m": pose[0], "y_m": pose[1], "heading_deg": pose[2]},
                    _open_room())
    return rover


def _remember(rover, phrase, at=None, from_xy=(1.0, 1.0)):
    """One thing the rover has seen, optionally placed where it was told.

    The look carries the phrase's own vector, so that phrase ranks it first with
    certainty rather than with whatever two hashes happened to agree on. `at` is
    where the crossing would have put it; leaving it out is a thing seen from one
    place only, which is the ordinary state of most of the store.
    """
    import world_state

    eyes = rover._world_inspector().eyes
    vectors, error = eyes.embed([phrase])
    if error:
        raise RuntimeError(error)
    store = rover._world_store()
    store.record([world_state.Sighting(bbox=[0.45, 0.45, 0.55, 0.55], dino=b"",
                                       siglip=vectors[0])],
                 capture={"frame_id": "f", "pan": 0.0, "tilt": 0.0,
                          "pose": {"x_m": from_xy[0], "y_m": from_xy[1],
                                   "heading_deg": 0.0}},
                 source="perception", fov_deg=66.0)
    entity = store.create_entity()
    store.attach(entity, [store.observations(limit=1)[0]["id"]], why="the test")
    if at is not None:
        # Shaped as the resolver writes one: where, how well known, how wide the
        # crossing measured it (a half-width -- see `locate.extent_of`), and how
        # many separate places agreed.
        store.place(entity, {"x_m": at[0], "y_m": at[1], "uncertainty_m": 0.2,
                             "extent_m": 0.3, "viewpoints": 2,
                             "rays_agreeing": 4}, store.map_session())
    return entity


def _in_a_world(pose=(1.0, 3.0, 90.0)):
    """A rover with an empty world state, and the environment put back after."""
    directory = tempfile.TemporaryDirectory()
    was = (os.environ.get("UGV_WORLD_DIR"), os.environ.get("UGV_WORLD_FAKE"))
    os.environ["UGV_WORLD_DIR"] = directory.name
    os.environ["UGV_WORLD_FAKE"] = "1"

    def done(rover):
        rover.close_world()
        for name, value in zip(("UGV_WORLD_DIR", "UGV_WORLD_FAKE"), was):
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value
        try:
            directory.cleanup()
        except OSError:            # Windows holds the database file open a moment
            pass

    return _a_rover(directory, pose), done


# --- finding one thing ------------------------------------------------------

def test_finding_a_thing_the_rover_has_seen() -> None:
    """The four answers `find_thing` has, which are four different situations.

    Never seen it; seen it but never from two places, so it has no position; has
    a position but does not know its own; and knows both, which is the only one
    of the four that is a distance and a direction.
    """
    try:
        import numpy  # noqa: F401
        import rover_daemon  # noqa: F401
    except ImportError as error:
        SKIP.append(f"finding a thing by describing it ({error})")
        return

    rover, done = _in_a_world()
    try:
        empty = rover.call("find_thing", {"description": "a bed"})
        check("a rover that has looked at nothing has not found it",
              (empty["ok"], empty["found"]), (True, False))
        check("...and says so in a sentence rather than a score",
              "has not seen anything matching that" in empty["note"], True)

        # Seen, and only ever from where the rover is standing now, so nothing
        # has crossed the first bearing and it has no position.
        _remember(rover, "a bed", from_xy=(1.0, 3.0))
        seen = rover.call("find_thing", {"description": "a bed"})
        check("something seen once is found", seen["found"], True)
        check("...but has no position", seen["placed"], False)
        check("...and the answer says what would give it one",
              "from one place" in seen["note"], True)
        check("...with how often it has been seen", seen["seen_times"], 1)

        # And the same thing, crossed, two metres in front of the rover. The
        # rover is at (1, 3) facing north, the thing is at (3, 3), so it is two
        # metres away and off to its right.
        rover._world_store().place(
            _placed_id(rover), {"x_m": 3.0, "y_m": 3.0, "uncertainty_m": 0.2},
            rover._world_store().map_session())
        placed = rover.call("find_thing", {"description": "a bed"})
        check("a thing with a position is answered in metres",
              placed["distance_m"], 2.0)
        check("...and in words rather than in degrees", placed["direction"],
              "to your right")
        check("...and says when it was last seen, roughly",
              placed["last_seen"], "just now")
        check("nothing the model is shown is an identifier or a map coordinate",
              _leaks(placed), [])

        # A rover that does not know where it is cannot say how far away
        # anything is, and must not answer as though it could.
        rover.nav = Nav(None, _open_room())
        lost = rover.call("find_thing", {"description": "a bed"})
        check("a rover that is lost still knows it has seen the thing",
              (lost["found"], lost["placed"]), (True, True))
        check("...but offers no distance", "distance_m" in lost, False)
        check("...and says which of the two it does not know",
              "not where the rover itself is" in lost["note"], True)
    finally:
        done(rover)


def test_a_thing_the_rover_has_never_seen_is_not_invented() -> None:
    """The failure this whole feature is designed around: a list of scores always
    has a top, and a rover that answers "the bed is over there" when there is no
    bed in the building is worse than one that answers nothing.

    The bar is `world_state.search.MATCHES`, measured on the rover, and it is
    applied to the row this settles on rather than to the best row in the list --
    those are not the same row when the best look belongs to nothing yet.
    """
    try:
        import numpy  # noqa: F401
        import rover_daemon  # noqa: F401
        import world_state
    except ImportError as error:
        SKIP.append(f"refusing to invent a thing ({error})")
        return

    rover, done = _in_a_world()
    try:
        eyes = rover._world_inspector().eyes
        vectors, _error = eyes.embed(["a spray bottle"])
        store = rover._world_store()
        store.record([world_state.Sighting(bbox=[0.4, 0.4, 0.6, 0.6], dino=b"",
                                           siglip=_orthogonal_to(vectors[0]))],
                     capture={"frame_id": "f", "pan": 0.0, "tilt": 0.0,
                              "pose": {"x_m": 1.0, "y_m": 1.0,
                                       "heading_deg": 0.0}},
                     source="perception", fov_deg=66.0)
        entity = store.create_entity()
        store.attach(entity, [store.observations(limit=1)[0]["id"]], why="test")
        store.place(entity, {"x_m": 3.0, "y_m": 3.0, "uncertainty_m": 0.2},
                    store.map_session())

        answer = rover.call("find_thing", {"description": "a spray bottle"})
        check("a thing that scores below the floor is not found",
              answer["found"], False)
        check("...and no distance to the nearest thing is offered instead",
              "distance_m" in answer, False)
        going = rover.call("go_to_thing", {"description": "a spray bottle"})
        check("...and the rover is not sent anywhere", going["ok"], False)
        check("...saying it has not seen one", "has not seen" in going["error"],
              True)
        check("...and nothing was sent to the navigator", rover.nav.sent, [])
    finally:
        done(rover)


def test_a_known_thing_is_found_behind_looks_that_belong_to_nothing() -> None:
    """The fault of 2026-10-13, reproduced.

    The rover spent a morning on a map it had not confirmed, which records the
    picture and nothing else, so its newest looks belonged to no thing. The old
    search ranked looks and walked ten of them for one that belonged to a thing;
    when the ten best were all unclaimed it answered "not seen" while quoting a
    score above the bar, about a thing the store held. Things are ranked now, so
    looks that belong to nothing cannot stand in front of one.
    """
    try:
        import numpy  # noqa: F401
        import rover_daemon  # noqa: F401
        import world_state
    except ImportError as error:
        SKIP.append(f"finding a thing behind unclaimed looks ({error})")
        return

    rover, done = _in_a_world()
    try:
        lamp = _remember(rover, "a floor lamp", at=(3.0, 3.0), from_xy=(1.0, 3.0))
        eyes = rover._world_inspector().eyes
        vectors, _error = eyes.embed(["a floor lamp"])
        store = rover._world_store()
        for _ in range(12):
            store.record([world_state.Sighting(bbox=[0.45, 0.45, 0.55, 0.55],
                                               dino=b"", siglip=vectors[0])],
                         capture={"frame_id": "f", "pan": 0.0, "tilt": 0.0},
                         source="perception", fov_deg=66.0)
        unclaimed = [one for one in store.observations(limit=12)
                     if not one.get("entity_id")]
        check("the newest looks belong to nothing, as on the rover",
              len(unclaimed), 12)

        found = rover.call("find_thing", {"description": "a floor lamp"})
        check("the thing behind them is found", found["found"], True)
        check("...and placed, two metres off", found.get("distance_m"), 2.0)
        going = rover.call("go_to_thing", {"description": "a floor lamp"})
        check("...and can be gone to", going["ok"], True)
        check("...the trip being for that thing",
              rover.nav.sent[-1]["for_what"]["id"], lamp)
    finally:
        done(rover)


# --- going to one thing -----------------------------------------------------

def test_going_to_a_thing_sets_off_and_answers_at_once() -> None:
    """A trip lasts a minute and the model holds one connection, so this must
    start the drive and come back -- `explore`'s rule, for `explore`'s reason.

    What it sends is the viewpoint the world state chose, unchanged, with the
    bearing to face on arrival; the difference between parking in the right spot
    and parking in the right spot with its back to the thing.
    """
    try:
        import numpy  # noqa: F401
        import rover_daemon  # noqa: F401
    except ImportError as error:
        SKIP.append(f"driving to a thing by describing it ({error})")
        return

    rover, done = _in_a_world()
    try:
        _remember(rover, "the desk", at=(4.0, 3.0), from_xy=(1.0, 3.0))
        went = rover.call("go_to_thing", {"description": "the desk"})
        check("the rover sets off", (went["ok"], went["going"]), (True, True))
        check("...and says it has set off rather than arrived",
              "has set off" in went["note"], True)
        check("...to the viewpoint the world state chose",
              (rover.nav.sent[-1]["x_m"], rover.nav.sent[-1]["y_m"]),
              (3.2, 3.0))
        check("...facing the thing on arrival",
              rover.nav.sent[-1]["heading_deg"], 0.0)
        check("...carrying which thing the trip is for, so asking again knows",
              rover.nav.sent[-1]["for_what"]["said"], "the desk")
        check("nothing the model is shown is an identifier or a map coordinate",
              _leaks(went), [])

        # Asked again for the same thing, mid-trip. A model unsure whether its
        # call landed asks again, and starting over would be a rover that never
        # arrived.
        before = len(rover.nav.sent)
        again = rover.call("go_to_thing", {"description": "the desk"})
        check("asking again does not start a second trip",
              len(rover.nav.sent), before)
        check("...and reports the one in flight", again["ok"], True)
        check("...saying it is already on its way",
              "already on its way" in again["note"], True)
        check("...without stopping it", rover.nav.stops, 0)

        # Asked for something else, mid-trip. That is not an echo, it is somebody
        # changing their mind, and it is the drive console's own rule.
        _remember(rover, "the sofa", at=(4.0, 5.0), from_xy=(1.0, 3.0))
        elsewhere = rover.call("go_to_thing", {"description": "the sofa"})
        check("a different thing stops the trip that was running",
              rover.nav.stops, 1)
        check("...and sets off for the new one", elsewhere["ok"], True)
        check("...which is somewhere else", len(rover.nav.sent), before + 1)
    finally:
        done(rover)


def test_a_thing_with_nowhere_to_see_it_from_is_refused_in_words() -> None:
    """The three ways the room says no reach the model as sentences rather than
    as a rover that sets off for a placement inside a wall. What it must never do
    is answer `ok` and stay still: the model reads that and says it is on its way.
    """
    try:
        import numpy  # noqa: F401
        import rover_daemon  # noqa: F401
    except ImportError as error:
        SKIP.append(f"refusing a thing with no viewpoint ({error})")
        return

    rover, done = _in_a_world()
    try:
        _remember(rover, "the lamp", from_xy=(1.0, 3.0))
        nowhere = rover.call("go_to_thing", {"description": "the lamp"})
        check("a thing with no position is not driven to", nowhere["ok"], False)
        check("...because there is nowhere to be sent",
              "no position yet" in nowhere["error"], True)
        check("...and nothing reached the navigator", rover.nav.sent, [])

        # Off the edge of the mapped floor: the viewpoint refuses, and the
        # sentence is the map's rather than a stack trace.
        _remember(rover, "the shed", at=(20.0, 20.0), from_xy=(1.0, 3.0))
        outside = rover.call("go_to_thing", {"description": "the shed"})
        check("a thing outside the map is not driven to", outside["ok"], False)
        check("...saying the floor around it is not mapped",
              "has not been mapped" in outside["error"], True)
    finally:
        done(rover)


# --- what a found thing says about itself -----------------------------------

def test_a_found_thing_carries_what_was_measured_about_it() -> None:
    """The metadata beside the distance, and the one piece of it that is a
    coordinate.

    A position means nothing on its own and everything against another position,
    which is exactly what it is for: two of these are how "how far is the bed
    from the desk" is answered, no tool having measured it since. So what is
    checked is that the pair is the store's own, that two things come back in one
    frame, and that the distance between the pairs is the distance between the
    placements.
    """
    try:
        import numpy  # noqa: F401
        import rover_daemon  # noqa: F401
    except ImportError as error:
        SKIP.append(f"what a found thing says about itself ({error})")
        return

    rover, done = _in_a_world()
    try:
        _remember(rover, "the bed", at=(1.0, 1.0), from_xy=(1.0, 3.0))
        bed = rover.call("find_thing", {"description": "the bed"})
        check("a found thing says where it is on the map",
              (bed["map_x_m"], bed["map_y_m"]), (1.0, 1.0))
        check("...and how far out that may be", bed["known_to_m"], 0.2)
        check("...how wide it is, which is twice the half-width the store keeps",
              bed["width_m"], 0.6)
        check("...how many separate places agreed about it",
              bed["seen_from_places"], 2)
        check("...and when it was first seen as well as last",
              (bed["first_seen"], bed["last_seen"]), ("just now", "just now"))
        check("nothing it says is an identifier or the console's own vocabulary",
              _leaks(bed), [])

        # The point of the position: two of them, one frame, one subtraction.
        _remember(rover, "the desk", at=(4.0, 5.0), from_xy=(1.0, 3.0))
        desk = rover.call("find_thing", {"description": "the desk"})
        apart = math.hypot(desk["map_x_m"] - bed["map_x_m"],
                           desk["map_y_m"] - bed["map_y_m"])
        check("two things are in the same frame, so they can be compared",
              round(apart, 1), 5.0)

        # And a position measured under a map that has been replaced is not a
        # position in this one, so none of the above is offered for it. A restore
        # that failed is how this happens in the room: the button that clears the
        # map empties the store instead of stranding it.
        rover._world_store().follow_map("a-map-this-was-not-recorded-in")
        stale = rover.call("find_thing", {"description": "the bed"})
        check("a thing placed under a map that is gone has no position now",
              stale["placed"], False)
        check("...and offers no coordinates at all", "map_x_m" in stale, False)
        check("...but is still a thing the rover has seen", stale["found"], True)
        # The sentence the voice model reads out, and it used to be the wrong
        # one of the two: a thing whose map was replaced was measured properly,
        # and saying the rover had only glimpsed it once was a plain untruth
        # about a thing it had looked at from several places.
        check("...and it says the map changed rather than that it never looked",
              "map that has since been replaced" in stale["note"], True)
        check("...and says what would put it back on the map",
              "from two places" in stale["note"], True)
    finally:
        done(rover)


def test_which_way_something_is_in_words() -> None:
    """Degrees off the rover's nose, said the way a person says them. Positive is
    to the rover's left, which is the map's convention and the opposite of the
    gimbal's -- the sign error that would put every answer on the wrong side of
    the room without anything else going wrong."""
    try:
        import rover_recall
    except ImportError as error:
        SKIP.append(f"saying which way something is ({error})")
        return

    check("dead ahead", rover_recall.which_way(0.0), "straight ahead")
    check("a little off is still ahead", rover_recall.which_way(-15.0),
          "straight ahead")
    check("half a right angle to the left",
          rover_recall.which_way(45.0), "ahead and to your left")
    check("a right angle to the right",
          rover_recall.which_way(-90.0), "to your right")
    check("over its shoulder", rover_recall.which_way(140.0),
          "behind you and to your left")
    check("dead astern", rover_recall.which_way(175.0), "straight behind you")


# --- the world model's records ------------------------------------------------

def _model_records(rover, observation_ids, at, state="present", age_s=0.0,
                   gone_at=None, session=None, height_m=None, visits=3, looked_for=0):
    """The world model's job's output (world_state/visit_model/job.py), written
    where world_state/visit_things.py reads it: one record holding the given
    looks, standing at `at`."""
    import time

    store = rover._world_store()
    folder = os.path.join(store.dir, "visit_model")
    os.makedirs(folder, exist_ok=True)
    now = time.time()
    record = {"id": "v7", "x_m": at[0], "y_m": at[1], "spread_m": 0.15, "extent_m": 0.6,
              "height_m": height_m, "state": state, "seen_at": now - 60.0, "since": now - 600.0,
              "gone_at": gone_at, "visits": visits, "looked_for": looked_for,
              "path": [[at[0], at[1], round(now - 600.0)]],
              "observations": list(observation_ids)}
    payload = {"model": "test", "written_at": now - age_s, "generation": "0123456789abcdef",
               "filed_until": now - age_s - 90.0,
               "map_session": store.map_session() if session is None else session,
               "things": [record]}
    with open(os.path.join(folder, "things.json"), "w") as f:
        json.dump(payload, f)


def test_the_world_models_records_answer_first() -> None:
    """**What the owner chose on 2026-10-17**: `find_thing` ranks the world
    model's records (version 4, kept by world_state/visit_model/job.py) rather
    than the store's things, and answers from the store only when the records
    cannot -- none written, a quarter of an hour old, or on another map. The
    record here holds the store's one look of the bed and has a position, while
    the store's thing for that look has none, so which of the two answered is
    plain from the answer."""
    try:
        import numpy  # noqa: F401
        import rover_daemon  # noqa: F401
    except ImportError as error:
        SKIP.append(f"the world model's records answering first ({error})")
        return

    import time

    rover, done = _in_a_world()
    try:
        _remember(rover, "a bed", from_xy=(1.0, 3.0))
        look = rover._world_store().observations(limit=1)[0]["id"]
        _model_records(rover, [look], at=(3.0, 3.0))
        found = rover.call("find_thing", {"description": "a bed"})
        check("the world model's record is the thing found",
              (found["found"], found["placed"]), (True, True))
        check("...answered from its position, in metres and words",
              (found["distance_m"], found["direction"]), (2.0, "to your right"))
        check("...with how often it was seen", found["seen_times"], 1)
        check("...and nothing of the model's vocabulary in it", _leaks(found), [])
        check("...nor its record's name", "v7" in json.dumps(found), False)

        going = rover.call("go_to_thing", {"description": "a bed"})
        check("going to it sets off", going["ok"], True)
        sent = rover.nav.sent[-1]
        check("...for somewhere the record's position can be seen from",
              math.hypot(sent["x_m"] - 3.0, sent["y_m"] - 3.0) < 3.0, True)
        rover.nav.stop()

        _model_records(rover, [look], at=(3.0, 3.0), state="gone", gone_at=time.time() - 120.0)
        gone = rover.call("find_thing", {"description": "a bed"})
        check("a record seen gone says the thing is no longer where it was",
              "no longer there" in gone.get("no_longer_there", ""), True)

        _model_records(rover, [look], at=(3.0, 3.0), age_s=3600.0)
        stale = rover.call("find_thing", {"description": "a bed"})
        check("records an hour old leave the answer to the store, whose thing has no position",
              (stale["found"], stale["placed"]), (True, False))

        _model_records(rover, [look], at=(3.0, 3.0), session=-1)
        elsewhere = rover.call("find_thing", {"description": "a bed"})
        check("records on another map leave the answer to the store too",
              elsewhere["placed"], False)
    finally:
        done(rover)


def test_of_two_records_alike_the_present_and_recent_one_answers() -> None:
    """The model keeps a record of each place a thing stood, and several of one
    thing it could not join, which a phrase scores within about 0.01 of each
    other. "Where is the bed" means where it is now: among near-ties a record
    still where it was beats one seen gone, then the one seen most recently --
    by its newest look, not the record's own time, which a merge leaves old."""
    try:
        import numpy  # noqa: F401
        import rover_daemon  # noqa: F401
    except ImportError as error:
        SKIP.append(f"the present and recent record answering ({error})")
        return

    import time

    rover, done = _in_a_world()
    try:
        _remember(rover, "a bed", from_xy=(1.0, 3.0))
        time.sleep(0.05)
        _remember(rover, "a bed", from_xy=(1.0, 3.0))
        older, newer = sorted(o["id"] for o in rover._world_store().observations(limit=2))
        store = rover._world_store()
        folder = os.path.join(store.dir, "visit_model")
        os.makedirs(folder, exist_ok=True)

        def write(newer_state, newer_gone, newer_looked_for=0):
            now = time.time()
            base = {"spread_m": 0.15, "extent_m": 0.6, "height_m": None, "since": now - 600.0,
                    "visits": 2, "seen_at": now - 86400.0 * 7}
            things = [dict(base, id="v7", x_m=3.0, y_m=3.0, state="present", gone_at=None,
                           path=[[3.0, 3.0, 0]], observations=[older]),
                      dict(base, id="v8", x_m=1.0, y_m=5.0, state=newer_state, gone_at=newer_gone,
                           looked_for=newer_looked_for, path=[[1.0, 5.0, 0]],
                           observations=[newer])]
            with open(os.path.join(folder, "things.json.tmp"), "w") as f:
                json.dump({"model": "test", "written_at": now, "map_session": store.map_session(),
                           "things": things}, f)
            os.replace(os.path.join(folder, "things.json.tmp"), os.path.join(folder, "things.json"))

        write("present", None)
        recent = rover.call("find_thing", {"description": "a bed"})
        check("of two records alike, the one seen more recently answers",
              (recent["distance_m"], recent["direction"]), (2.0, "straight ahead"))
        check("...and when it was seen is its newest look's time, not the record's own",
              recent["last_seen"], "just now")

        write("gone", time.time())
        present = rover.call("find_thing", {"description": "a bed"})
        check("one seen gone gives way to one still where it was",
              (present["distance_m"], present["direction"]), (2.0, "to your right"))

        import world_state.visit_things as visit_things
        was = visit_things.SET_ASIDE_DAYS
        visit_things.SET_ASIDE_DAYS = 0.0   # these looks are seconds old, not a week
        try:
            write("present", None, newer_looked_for=3)
            aside = rover.call("find_thing", {"description": "a bed"})
            check("so does one set aside as not found again",
                  (aside["distance_m"], aside["direction"], "not_found_again" in aside),
                  (2.0, "to your right", False))
        finally:
            visit_things.SET_ASIDE_DAYS = was
    finally:
        done(rover)


def test_a_record_set_aside_says_it_was_not_found_again() -> None:
    """A record seen on one or two visits, not for a week, and not recognised
    on three close looks at its place since is set aside -- never called gone:
    found, it says the rover has not found it there again and it may not be
    there. Two looks short of that it answers as any other; a record seen on
    three visits is never set aside."""
    try:
        import numpy  # noqa: F401
        import rover_daemon  # noqa: F401
    except ImportError as error:
        SKIP.append(f"a record set aside as not found again ({error})")
        return

    import world_state.visit_things as visit_things

    rover, done = _in_a_world()
    was = visit_things.SET_ASIDE_DAYS
    visit_things.SET_ASIDE_DAYS = 0.0   # these looks are seconds old, not a week
    try:
        _remember(rover, "a bed", from_xy=(1.0, 3.0))
        look = rover._world_store().observations(limit=1)[0]["id"]
        _model_records(rover, [look], at=(3.0, 3.0), visits=1, looked_for=3)
        found = rover.call("find_thing", {"description": "a bed"})
        check("set aside, it says it was not found again, and is not called gone",
              ("not found it again" in found.get("not_found_again", ""), "no_longer_there" in found),
              (True, False))
        check("...and is still placed, where it was seen",
              (found["placed"], found["distance_m"]), (True, 2.0))
        listed = rover.call("world_state_entities", {"source": "model"})
        check("the console's list carries it, uncounted among things in place",
              (listed["entities"][0]["set_aside"], listed["summary"]["entities"]), (True, 0))
        _model_records(rover, [look], at=(3.0, 3.0), visits=1, looked_for=2)
        check("two looks for it are not enough",
              "not_found_again" in rover.call("find_thing", {"description": "a bed"}), False)
        _model_records(rover, [look], at=(3.0, 3.0), visits=3, looked_for=9)
        check("nor is a record seen on three visits ever set aside",
              "not_found_again" in rover.call("find_thing", {"description": "a bed"}), False)
    finally:
        visit_things.SET_ASIDE_DAYS = was
        done(rover)


def test_the_console_reads_the_world_models_records() -> None:
    """The console's popup asks for its things with `source: "model"` and gets
    the world model's records in the store entities' shape, each with `gone`;
    one of them by name, with its looks; and a search's looks named by the
    record that holds them. Records that may not answer leave all three to the
    store, and a caller that names no source reads the store either way."""
    try:
        import numpy  # noqa: F401
        import rover_daemon  # noqa: F401
    except ImportError as error:
        SKIP.append(f"the console reading the world model's records ({error})")
        return

    import time

    rover, done = _in_a_world()
    try:
        _remember(rover, "a bed", from_xy=(1.0, 3.0))
        look = rover._world_store().observations(limit=1)[0]["id"]
        _model_records(rover, [look], at=(3.0, 3.0), state="gone", gone_at=time.time() - 60.0)
        listed = rover.call("world_state_entities", {"source": "model"})
        check("the console's things are the model's records", listed.get("source"), "model")
        check("...one record, gone, at its last place",
              [(one["id"], one["gone"], one["placement"]["x_m"]) for one in listed["entities"]],
              [("v7", True, 3.0)])
        check("...with the rays of its own looks", len(listed["entities"][0]["rays"]), 1)
        check("...and a count of the records still present", listed["summary"]["entities"], 0)
        check("a list asked for without a source is still the store's",
              rover.call("world_state_entities", {}).get("source"), "store")

        one = rover.call("world_state_entity", {"id": "v7"})
        check("a record is read by its name, with its looks",
              (one["ok"], one["entity"]["kind"], len(one["observations"])), (True, "record", 1))

        found = rover.call("world_state_search", {"query": "a bed", "source": "model"})
        check("a search's look is named by the record that holds it",
              [m["entity_id"] for m in found["matches"]], ["v7"])

        _model_records(rover, [look], at=(3.0, 3.0), age_s=3600.0)
        stale = rover.call("world_state_entities", {"source": "model"})
        check("records an hour old leave the console to the store", stale.get("source"), "store")
    finally:
        done(rover)


def test_autonomy_reads_the_world_models_records_or_none() -> None:
    """Autonomy asks for the model's records with no fallback and no sight
    lines: a run carries the records' names from one step to the next, so the
    store's things, named differently, would be strangers to it. It gets each
    record's place with its height, what names the records, and up to when
    the model has filed; and records that may not answer are an error saying
    why, not the store's list."""
    try:
        import numpy  # noqa: F401
        import rover_daemon  # noqa: F401
    except ImportError as error:
        SKIP.append(f"autonomy reading the world model's records ({error})")
        return

    asked = {"source": "model", "fallback": False, "rays": False}
    rover, done = _in_a_world()
    try:
        _remember(rover, "a bed", from_xy=(1.0, 3.0))
        look = rover._world_store().observations(limit=1)[0]["id"]
        _model_records(rover, [look], at=(3.0, 3.0), height_m=0.3)
        listed = rover.call("world_state_entities", asked)
        check("autonomy's things are the model's records",
              (listed.get("source"), [one["id"] for one in listed["entities"]]), ("model", ["v7"]))
        entity = listed["entities"][0]
        check("...placed, with the height its tilt is worked out from",
              (entity["placement"]["x_m"], entity["placement"]["height_m"], entity["gone"]),
              (3.0, 0.3, False))
        check("...without sight lines or the newest looks",
              ("rays" in entity, "recent" in listed), (False, False))
        check("...named under the records' generation, with how far the model has filed",
              (listed["generation"], listed["filed_until"] is not None), ("0123456789abcdef", True))

        _model_records(rover, [look], at=(3.0, 3.0), age_s=3600.0)
        stale = rover.call("world_state_entities", asked)
        check("records an hour old are an error, not the store's list",
              (stale["ok"], stale.get("source"), "60 minutes old" in stale.get("error", "")),
              (False, "model", True))
    finally:
        done(rover)


# --- helpers ----------------------------------------------------------------

def _placed_id(rover) -> str:
    return rover._world_store().entities()[0]["id"]


def _leaks(answer: dict) -> list:
    """The console's own vocabulary, where a model-facing result should have none.

    An entity identifier, a raw cosine, a map session, the placement as the store
    writes it: all of them are things a model would either read out loud or reason
    about wrongly, and none of them says anything a distance and a direction do
    not. **`map_x_m` and `map_y_m` are deliberately not on this list.** A position
    coming back is what lets one thing be compared with another; what is refused
    is a position going *in*, which is `_tool_drive_to`'s argument and is enforced
    by there being no such parameter on any schema here.
    """
    text = json.dumps(answer)
    return [bad for bad in ("object:", "entity_id", "placement", "map_session",
                            "score", "uncertainty_m", "extent_m")
            if bad in text]


TESTS = (
    test_finding_a_thing_the_rover_has_seen,
    test_a_thing_the_rover_has_never_seen_is_not_invented,
    test_a_known_thing_is_found_behind_looks_that_belong_to_nothing,
    test_going_to_a_thing_sets_off_and_answers_at_once,
    test_a_thing_with_nowhere_to_see_it_from_is_refused_in_words,
    test_a_found_thing_carries_what_was_measured_about_it,
    test_the_world_models_records_answer_first,
    test_of_two_records_alike_the_present_and_recent_one_answers,
    test_the_console_reads_the_world_models_records,
    test_autonomy_reads_the_world_models_records_or_none,
    test_a_record_set_aside_says_it_was_not_found_again,
    test_which_way_something_is_in_words,
)
