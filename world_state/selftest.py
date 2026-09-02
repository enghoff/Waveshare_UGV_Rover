#!/usr/bin/env python3
"""Offline checks for the semantic world state. No rover, no GPU, no encoders.

    python world_state/selftest.py
    ssh orin 'cd ~/ugv/world_state && python3 selftest.py'

What is covered is the part where a bug is silent rather than loud. An inspection
that stores nothing says so out loud; an inspection that stores the *wrong* thing
looks exactly like one that worked. So: nothing becomes an identity that was not
measured, the provenance the rover measured really is on every row, a database
written by an older build still opens, and every failure path leaves the world
untouched.

Everything here runs against `FakeEyes` and a temporary directory. That is enough
to prove the store, the rules and the geometry, and nothing at all about what the
real encoders see.
"""
from __future__ import annotations

import math
import os
import sqlite3
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
# The package is imported as a package, so what goes on the path is its parent --
# ~/ugv on the rover, the checkout root here.
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from test_harness import FAIL, PASS, SKIP, check  # noqa: E402

from world_state import locate                     # noqa: E402
from world_state import resolve                    # noqa: E402
from world_state import search                     # noqa: E402
from world_state import view                       # noqa: E402
from world_state.inspector import Inspector        # noqa: E402
from world_state.store import WorldStore           # noqa: E402

#: A one-pixel JPEG. Nothing decodes it here -- the store keeps bytes and the fake
#: sidecar counts them -- but it should be a real picture, because the thing the
#: rover stores is a real picture.
JPEG = bytes.fromhex(
    "ffd8ffe000104a46494600010100000100010000ffdb004300ffffffffffffffffffffffff"
    "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
    "ffffffffffffffffffffffffffffffffffffffc00011080001000101011100ffc400140001"
    "00000000000000000000000000000009ffc4001401010000000000000000000000000000"
    "0000ffda000c03010002110311003f00b7ffd9")


def a_store(directory):
    return WorldStore(directory)


def a_capture(pan=0.0, tilt=0.0, ok=True, error="", live=False):
    def capture():
        if not ok:
            return {"ok": False, "error": error}
        return {"ok": True, "jpeg": JPEG, "pan": pan, "tilt": tilt, "live": live,
                "width": 640, "height": 480}
    return capture


def a_pose(x=1.0, y=2.0, heading=90.0):
    return lambda: {"x_m": x, "y_m": y, "heading_deg": heading}


# --- the store --------------------------------------------------------------
#
# The rows come from `a_sighting`, defined with the encoder tests further down,
# because a region is the only thing that reaches the store now: there is no
# second kind of observation and no model answer to validate on the way in.

def test_an_empty_database_is_an_ordinary_thing_to_open() -> None:
    """A rover that has never inspected anything, and every test below.

    Worth its own check because this is the state the rover is in after a new
    computer, and a store that needed a migration step run by hand before it would
    open would fail there rather than here.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        check("an empty world opens", store.summary()["entities"], 0)
        check("...with no observations", store.summary()["observations"], 0)
        check("...and a map session of its own", store.summary()["map_session"], 1)
        check("...and the frames directory exists",
              os.path.isdir(os.path.join(directory, "frames")), True)
        store.close()


def test_the_application_owns_the_identifiers() -> None:
    """Names are allocated here and nowhere else.

    Nothing calls this during an inspection any more -- an inspection decides no
    identity at all -- but the rule is the one thing about identity that survived
    both models failing at it, and the resolver that will create entities from
    triangulated positions comes in by this door. So it is checked directly.

    The identifier is counted in a table rather than derived from the rows, so an
    entity deleted by hand cannot hand its name to something else later.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        check("an identifier is the kind and a counter",
              [store.allocate("furniture"), store.allocate("opening")],
              ["furniture:1", "opening:1"])
        check("...counted per kind", store.allocate("furniture"), "furniture:2")
        store.close()
        reopened = a_store(directory)
        check("...and not restarted by a reopen",
              reopened.allocate("furniture"), "furniture:3")
        reopened.close()


def test_an_inspection_claims_no_identity_at_all() -> None:
    """The change this phase exists for.

    Two models were asked which lasting thing they were looking at, on this
    rover's own frames. One never recognises anything and fills the store with
    duplicates; the other recognises things that are not in the room and copies
    their descriptions out of the list it was shown. So nothing an inspection
    stores says which thing it is -- that waits for a position, measured from two
    places.

    Deliberately not covered here, because it must never start working: matching
    on a name. The same chair came back from a model as a black leather recliner
    and then a blue one on a byte-identical frame, which is why regions arrive
    here with no name at all.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        stored = store.record(
            [a_sighting(), a_sighting(bbox=[0.6, 0.2, 0.9, 0.7])],
            capture={"frame_id": "f1"})
        check("both observations are kept", stored["stored"], 2)
        check("...and no entity is created for either", stored["created"], 0)
        check("...nor claimed to be matched", stored["matched"], 0)
        check("...so the entity table stays empty", store.entities(), [])

        # The same two things again, from a second look. A store that matched on
        # anything cheap would now report two matches, and this is where it would
        # show up.
        store.record(
            [a_sighting(), a_sighting(bbox=[0.6, 0.2, 0.9, 0.7])],
            capture={"frame_id": "f2"})
        check("a second look at the same room still creates nothing",
              store.summary()["entities"], 0)
        check("...while every observation is kept",
              store.summary()["observations"], 4)
        check("...all of them with no entity, which is where the popup shows them",
              store.summary()["unmatched"], 4)
        check("...and the provenance to settle it later is on each one",
              all(row["observed_at"] and row["map_session"]
                  for row in store.observations()), True)
        store.close()


def test_the_world_survives_the_process_that_wrote_it() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        store.record([a_sighting()],
                     capture={"frame_id": "f1", "pan": 20.0,
                              "pose": {"x_m": 1.0, "y_m": 2.0,
                                       "heading_deg": 90.0}})
        store.close()

        reopened = a_store(directory)
        rows = reopened.observations()
        check("the observation is still there after a restart",
              [row["bbox"] for row in rows], [[0.1, 0.3, 0.5, 0.9]])
        check("...with the pose that will one day place it",
              rows[0]["pose"], {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0})
        check("...and the gimbal angle with it", rows[0]["observer_pan_deg"], 20.0)
        reopened.close()


def test_a_database_from_an_older_build_still_opens() -> None:
    """The rover's database was written before this change and outlives it.

    `CREATE TABLE IF NOT EXISTS` does nothing to a table that already exists, so
    a column added later has to be added by hand or every insert naming it fails
    on exactly the machine that matters. Columns are added and never removed, so
    what the old rows recorded -- including the answers from when the model was
    still asked which thing it was looking at -- is still there to be read.
    """
    with tempfile.TemporaryDirectory() as directory:
        # The table as the deployed build wrote it: no `stored` column, and a row
        # from an inspection that was shown four entities and made two.
        old = sqlite3.connect(os.path.join(directory, "world.db"))
        with old:
            old.execute("CREATE TABLE inferences (id INTEGER PRIMARY KEY"
                        " AUTOINCREMENT, started_at REAL NOT NULL, duration_s REAL,"
                        " status TEXT NOT NULL, detail TEXT, backend TEXT,"
                        " model_id TEXT, prompt_version TEXT, frame_id TEXT,"
                        " frame_live INTEGER, known_count INTEGER, returned INTEGER,"
                        " matched INTEGER, created INTEGER, rejected INTEGER,"
                        " map_session INTEGER, raw_json TEXT)")
            old.execute(
                "INSERT INTO inferences(started_at, status, known_count, created)"
                " VALUES(1.0, 'ok', 4, 2)")
        old.close()

        reopened = a_store(directory)
        reopened.record_inference(started_at=2.0, status="ok", stored=3)
        rows = reopened.inferences()
        check("a column the old build never had is added on open",
              rows[0]["stored"], 3)
        check("...leaving the older row readable", rows[1]["status"], "ok")
        check("...with what it recorded at the time still in it",
              (rows[1]["known_count"], rows[1]["created"]), (4, 2))
        check("...and nothing written into it after the fact",
              rows[1]["stored"], None)
        reopened.close()


def test_the_frame_is_kept_and_every_observation_points_at_it() -> None:
    """Without the picture there is no way to tell a hallucinated entity from a
    real one somebody had forgotten was in the room."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        frame_id = store.save_frame(JPEG, 640, 480)
        store.record([a_sighting(), a_sighting(bbox=[0.6, 0.2, 0.9, 0.7])],
                     capture={"frame_id": frame_id,
                              "frame_path": store.frame_path(frame_id),
                              "pan": 35.0, "tilt": -10.0,
                              "pose": {"x_m": 2.1, "y_m": 0.4,
                                       "heading_deg": 88.0}})
        check("the picture is on disk", store.frame(frame_id) == JPEG, True)
        rows = store.observations()
        check("both observations name the frame they came from",
              {row["frame_id"] for row in rows}, {frame_id})
        check("...and the path it is stored at",
              all(row["frame_path"] for row in rows), True)
        check("the gimbal angles it was taken at are kept",
              (rows[0]["observer_pan_deg"], rows[0]["observer_tilt_deg"]),
              (35.0, -10.0))
        check("...and the rover's own pose with them",
              rows[0]["pose"], {"x_m": 2.1, "y_m": 0.4, "heading_deg": 88.0})
        check("a frame that is not there answers None rather than raising",
              store.frame("nothing-like-this"), None)
        check("...and a name that is a path traversal is refused before it opens",
              store.frame("../../etc/passwd"), None)
        store.close()


def test_a_missing_pose_is_recorded_as_missing() -> None:
    """SLAM not having settled is not a reason to refuse to look at the room."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        store.record([a_sighting()],
                     capture={"frame_id": "f1", "pan": 12.0, "tilt": None,
                              "pose": None})
        row = store.observations()[0]
        check("no pose stores null rather than failing", row["pose"], None)
        check("...and the gimbal angle it does have is still there",
              row["observer_pan_deg"], 12.0)
        check("...and the observation counted", store.summary()["observations"], 1)
        store.close()


def test_clearing_the_semantic_world_takes_its_frames_with_it() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        frame_id = store.save_frame(JPEG)
        store.record([a_sighting()],
                     capture={"frame_id": frame_id})
        store.record_inference(started_at=time.time(), status="ok")
        store.allocate("furniture")
        cleared = store.clear()
        check("the observations go", store.summary()["observations"], 0)
        check("...and the entities with them", store.summary()["entities"], 0)
        check("...and the diagnostics log", store.inferences(), [])
        check("...and the stored pictures", cleared["frames_removed"], 1)
        check("...leaving nothing behind in the directory",
              os.listdir(store.frames_dir), [])
        check("the map session is not touched by a semantic clear",
              cleared["map_session"], 1)
        check("...and identifiers start again, which is what a repeatable "
              "experiment needs", store.allocate("furniture"), "furniture:1")
        store.close()


def test_clearing_the_map_keeps_the_semantic_world() -> None:
    """The two clears are different operations and neither may perform the other.

    Only one direction is obvious. Clearing semantic memory must not touch SLAM --
    it cannot from here, this process does not own the map -- and clearing the map
    must not delete a room's worth of observations that are still true.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        store.record([a_sighting()],
                     capture={"frame_id": "f1"})
        session = store.new_map_session()
        check("a map clear starts a new session", session, 2)
        check("...and deletes no observation", store.summary()["observations"], 1)
        check("...and the old one still says which map it belongs to",
              store.observations()[0]["map_session"], 1)
        store.record([a_sighting()], capture={"frame_id": "f2"})
        check("...while a new one is stamped with the new session",
              store.observations()[0]["map_session"], 2)
        check("...so the two are never compared as if they shared a map",
              sorted(row["map_session"] for row in store.observations()), [1, 2])
        store.close()


def test_unreadable_stored_json_cannot_take_the_viewer_down() -> None:
    """The popup exists to show what went wrong, so it must survive a row that is
    itself what went wrong."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        store.record([a_sighting()],
                     capture={"frame_id": "f1"})
        with store.db:
            store.db.execute("UPDATE observations SET raw_json = '{not json',"
                             " bbox_json = 'nonsense'")
        row = store.observations()[0]
        check("a corrupt raw answer comes back marked rather than raising",
              "unreadable" in (row["raw"] or {}), True)
        check("...and so does a corrupt box", "unreadable" in (row["bbox"] or {}),
              True)
        check("...and the observation list still renders",
              len(store.observations()), 1)
        store.close()


def test_a_reader_is_not_blocked_by_a_writer() -> None:
    """The console reads the world while the inspection writing it is still going.

    With the default journal that ordinary overlap is a locked database rather
    than a slightly stale read, which is the whole reason WAL is set.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        store.record([a_sighting()],
                     capture={"frame_id": "f1"})
        writer = sqlite3.connect(store.path, timeout=1.0)
        try:
            writer.execute("BEGIN IMMEDIATE")
            writer.execute("INSERT INTO entities(id, kind, label,"
                           " canonical_description, created_at, last_seen_at)"
                           " VALUES('object:99','object','x','x',1,1)")
            check("a read succeeds while a write is open",
                  len(store.observations()), 1)
            writer.rollback()
        except sqlite3.OperationalError as error:
            FAIL.append(f"a read succeeds while a write is open: {error}")
        finally:
            writer.close()
        check("the journal really is WAL",
              store.db.execute("PRAGMA journal_mode").fetchone()[0], "wal")
        store.close()


# --- what gets drawn --------------------------------------------------------

def test_an_observation_becomes_a_bearing_from_a_measured_pose() -> None:
    """The gimbal takes pan positive to the right; the map takes bearings positive
    to the left. That minus sign is the whole of the conversion, and getting it
    backwards draws a perfectly ordinary ray over the wrong half of the room."""
    observation = {"pose": {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0},
                   "observer_pan_deg": 30.0, "bbox": [0.4, 0.2, 0.6, 0.8]}
    drawn = view.ray(observation, fov_deg=130.0)
    check("the ray starts where the rover was", (drawn["x_m"], drawn["y_m"]),
          (1.0, 2.0))
    check("a gimbal turned right of the nose points right of the heading",
          drawn["bearing_deg"], 60.0)
    check("...and a box the width of a fifth of the picture is that wide",
          drawn["span_deg"], 26.0)

    left = dict(observation, bbox=[0.0, 0.2, 0.2, 0.8])
    check("something at the left of the picture is further to the left again",
          view.ray(left, fov_deg=130.0)["bearing_deg"], 112.0)

    check("no pose means no ray, rather than a ray from the origin",
          view.ray(dict(observation, pose=None), 130.0), None)
    check("no gimbal angle means no ray either",
          view.ray(dict(observation, observer_pan_deg=None), 130.0), None)
    check("a box that is missing still leaves the camera direction",
          view.ray(dict(observation, bbox=None), 130.0)["bearing_deg"], 60.0)


def test_the_rays_of_one_entity_are_bounded_and_oldest_first() -> None:
    observations = [{"pose": {"x_m": float(n), "y_m": 0.0, "heading_deg": 0.0},
                     "observer_pan_deg": 0.0, "bbox": None, "observed_at": n}
                    for n in range(10)]
    drawn = view.rays(observations, 130.0, limit=4)
    check("only the newest few are drawn", len(drawn), 4)
    check("...oldest of those first, so the newest is on top",
          [one["observed_at"] for one in drawn], [3, 2, 1, 0])


def _look(x_m, y_m, bearing_deg):
    return {"x_m": x_m, "y_m": y_m, "bearing_deg": bearing_deg}


def test_two_bearings_from_two_places_locate_a_thing() -> None:
    """The sofa is at (3, 3). The rover sees it from two corners."""
    # From the origin it is at 45 degrees; from (6, 0) it is at 135.
    found = locate.fix(_look(0.0, 0.0, 45.0), _look(6.0, 0.0, 135.0))
    check("two crossing bearings give a point", found is not None, True)
    check("...where the thing actually is, in x", round(found["x_m"]), 3)
    check("...and in y", round(found["y_m"]), 3)
    check("...with the baseline it was measured over",
          round(found["baseline_m"]), 6)
    check("...and an honest uncertainty attached",
          found["uncertainty_m"] > 0.0, True)


def test_turning_on_the_spot_locates_nothing() -> None:
    """The failure mode of the first experiment: every ray from one point."""
    check("rays from one place do not meet anywhere useful",
          locate.fix(_look(0.0, 0.0, 40.0), _look(0.0, 0.0, 50.0)), None)
    check("...nor do rays from a place barely different",
          locate.fix(_look(0.0, 0.0, 40.0), _look(0.1, 0.0, 50.0)), None)


def test_two_looks_along_the_same_line_are_one_look() -> None:
    check("no parallax, no fix",
          locate.fix(_look(0.0, 0.0, 45.0), _look(1.0, 1.0, 45.0)), None)
    check("...and a thing behind the rover was not seen",
          locate.fix(_look(0.0, 0.0, -135.0), _look(6.0, 0.0, -45.0)), None)


def test_uncertainty_grows_when_the_baseline_shrinks() -> None:
    """Why the rover has to drive rather than shuffle.

    The same thing at (3, 3) seen from the same first place, with the second look
    taken 1.5 m away and then 6 m away.
    """
    short = locate.fix(_look(0.0, 0.0, 45.0), _look(1.5, 0.0, 63.43))
    long = locate.fix(_look(0.0, 0.0, 45.0), _look(6.0, 0.0, 135.0))
    check("a short baseline still gives a fix", short is not None, True)
    check("...on the same thing", round(short["x_m"]), 3)
    check("...but a much less certain one",
          short["uncertainty_m"] > long["uncertainty_m"] * 2, True)


def test_two_identical_chairs_stay_two_things() -> None:
    """The case no model can answer from one picture, and geometry answers easily."""
    chair_a = locate.fix(_look(0.0, 0.0, 45.0), _look(6.0, 0.0, 135.0))
    # A second chair four metres away, seen from the same two places.
    chair_b = locate.fix(_look(0.0, 0.0, 8.5), _look(6.0, 0.0, 172.9))
    apart = ((chair_a["x_m"] - chair_b["x_m"]) ** 2
             + (chair_a["y_m"] - chair_b["y_m"]) ** 2) ** 0.5
    check("both chairs get a position", bool(chair_a and chair_b), True)
    check("...far enough apart to be told apart",
          apart > chair_a["uncertainty_m"] + chair_b["uncertainty_m"], True)
    check("a new look at the first chair agrees with the first chair",
          locate.agrees(chair_a, _look(0.0, 3.0, 0.0)), True)
    check("...and does not agree with the second",
          locate.agrees(chair_b, _look(0.0, 3.0, 0.0)), False)


def test_a_fix_on_top_of_the_camera_is_not_a_thing_in_the_room() -> None:
    """**The fault the validation drive of 2026-09-02 found, in its own numbers.**

    Driven between three places, the rover placed six things and every one of them
    landed between 0.13 and 0.59 m from the nearest camera that saw it. That is
    not a floor lamp; the rover would have driven into it, and no crop of anything
    thirteen centimetres from that lens survives the region filter.

    The cause is that two rays pointing *inward* from two nearby places cross in
    the gap between them, at a healthy parallax angle, so neither the baseline
    guard nor the parallax guard catches it -- and because a nudge of a degree and
    a half barely moves a point a quarter of a metre away, such a crossing reports
    a tiny uncertainty and wins the resolver's ranking against every real thing in
    the room.

    The four cases below are the rover's own recorded rays.
    """
    from_rover = [
        # what it called it, where it stood, the bearing, where it put it
        ("a ceiling light", (-0.134, 1.347), -23.2, (0.364, 1.346), -147.8),
        ("a wall", (-0.134, 1.347), -49.3, (0.364, 1.346), -104.0),
        ("a floor lamp", (0.304, 1.276), -174.8, (-0.812, 1.076), 134.6),
        ("a houseplant", (0.304, 1.276), -176.4, (-0.812, 1.076), 112.1),
    ]
    for label, first, first_deg, second, second_deg in from_rover:
        found = locate.fix(_look(first[0], first[1], first_deg),
                           _look(second[0], second[1], second_deg))
        check(f"{label!r} was not really that close to the camera", found, None)

    # And the guard must not cost anything real. The furthest apart the rover got
    # on that drive was 1.73 m, and from those two places a thing three metres out
    # in the room is still placed.
    here, there = (-0.856, 1.065), (0.857, 0.838)
    thing = (0.0, 4.0)
    real = locate.fix(
        _look(here[0], here[1],
              math.degrees(math.atan2(thing[1] - here[1], thing[0] - here[0]))),
        _look(there[0], there[1],
              math.degrees(math.atan2(thing[1] - there[1], thing[0] - there[0]))))
    check("a thing out in the room is still placed", real is not None, True)
    if real:
        check("...where it actually is",
              math.dist((real["x_m"], real["y_m"]), thing) < 0.1, True)
        check("...well clear of both cameras",
              min(math.dist((real["x_m"], real["y_m"]), here),
                  math.dist((real["x_m"], real["y_m"]), there))
              > locate.MIN_RANGE_M, True)


def test_a_bearing_at_the_edge_of_a_television_still_points_at_it() -> None:
    """An entity is stored as a point, but a television is a metre wide.

    Two looks from different sides of one centre on different parts of it, so a
    bearing that lands within the thing's own silhouette is pointing at it. With
    only the bearing error and the placement error, matching a television at two
    and a half metres allowed 0.115 m -- a tenth of the television -- and the
    looks that should have joined it made a second one instead.
    """
    telly = {"x_m": 0.0, "y_m": 2.5, "uncertainty_m": 0.05}
    # Standing at the origin, looking north. A twenty-degree region, which is
    # what the rover actually recorded for its televisions.
    edge = {"x_m": 0.0, "y_m": 0.0, "span_deg": 20.0,
            "bearing_deg": math.degrees(math.atan2(2.5, 0.3))}
    check("a bearing at the edge of it misses if it is treated as a point",
          locate.agrees(telly, edge), False)
    check("...but is pointing at it once its width counts",
          locate.agrees(telly, edge, locate.match_tolerance(telly, edge)), True)

    # And the width may not be used to swallow the room.
    across = {"x_m": 0.0, "y_m": 0.0, "span_deg": 20.0,
              "bearing_deg": math.degrees(math.atan2(2.5, 2.0))}
    check("something two metres to the side is still not the television",
          locate.agrees(telly, across, locate.match_tolerance(telly, across)),
          False)
    wall = {"x_m": 0.0, "y_m": 0.0, "span_deg": 90.0, "bearing_deg": 0.0}
    check("and a region filling the frame is capped rather than boundless",
          locate.match_tolerance(telly, wall)
          <= 2.5 * math.tan(math.radians(locate.BEARING_SIGMA_DEG)) + 0.05
          + locate.MAX_EXTENT_M + 1e-9, True)


def test_the_best_pair_places_the_thing() -> None:
    looks = [_look(0.0, 0.0, 45.0), _look(0.2, 0.0, 44.0),
             _look(6.0, 0.0, 135.0)]
    best = locate.best_fix(looks)
    check("a fix is found among several looks", best is not None, True)
    check("...using the pair with the longest useful baseline",
          round(best["baseline_m"]), 6)
    check("one look alone places nothing", locate.best_fix(looks[:1]), None)



# --- the two perception backends --------------------------------------------
#
# Nothing here loads a model. What is worth checking offline is the part that
# decides *which* backend runs and whether the two are really interchangeable,
# because the failure mode is silent: a rover that quietly drops to the CPU
# keeps answering, and its vectors stop comparing with the ones already stored.


def test_a_board_with_no_engines_falls_back_and_says_why() -> None:
    """The ordinary state of a freshly deployed rover, and it must not be fatal."""
    from world_state import engines
    from world_state.perceive import Perception

    with tempfile.TemporaryDirectory() as empty:
        ready, why = engines.available(empty)
        check("an empty directory offers no GPU path", ready, False)
        check("...and the reason names the installer",
              "install_perception.sh" in why or "TensorRT" in why, True)
        backend, missing = Perception(empty).chosen()
        check("...so the CPU backend is the one chosen", backend, "onnxruntime")
        check("...with the reason kept rather than swallowed", bool(missing), True)


def test_a_query_can_be_embedded_before_anything_has_been_looked_at() -> None:
    """The first thing a freshly booted sidecar is asked may be a search.

    Reproduces a fault seen on the rover: the query path reached for the numeric
    library the loader installs on the object *before* calling the loader, so the
    first search after a reboot died with an AttributeError instead of either
    answering or saying the models were missing.
    """
    from world_state.perceive import Perception, Unavailable

    with tempfile.TemporaryDirectory() as empty:
        perception = Perception(empty)
        check("a search before the first look has nothing loaded",
              perception._loaded, False)
        try:
            perception.embed_text(["a spray bottle"])
            raised = None
        except Exception as failure:                      # noqa: BLE001
            raised = failure
        check("...and asking it for a vector fails for a nameable reason",
              isinstance(raised, Unavailable), True)
        check("...not because the loader had not run yet",
              isinstance(raised, AttributeError), False)


def test_both_backends_answer_the_same_four_questions() -> None:
    """They are swapped at run time, so a method on one and not the other is a
    crash on whichever board has the wrong one. `release` is on the list because
    the search path calls it on whichever backend is in use, and on the CPU it
    has nothing to do."""
    from world_state.perceive import _CpuModels, _GpuModels

    wanted = {"regions", "appearance", "image_vectors", "text_vectors",
              "open", "release"}
    for backend in (_CpuModels, _GpuModels):
        have = {name for name in dir(backend) if not name.startswith("_")}
        check(f"{backend.name} answers all six",
              wanted - have, set())
    check("the two name themselves differently",
          _CpuModels.name != _GpuModels.name, True)


def test_the_installer_builds_exactly_the_engines_the_runtime_opens() -> None:
    """A renamed engine is a rover that silently runs on the CPU for ever."""
    from world_state import engines

    script = os.path.join(HERE, "install_perception.sh")
    with open(script, encoding="utf-8") as handle:
        built = {line.split()[1] for line in handle
                 if line.strip().startswith("build ") and ".plan" in line}
    check("every engine the runtime wants is built by the installer",
          set(engines.REQUIRED) - built, set())
    check("...and the installer builds nothing the runtime ignores",
          built - set(engines.REQUIRED), set())


def test_a_region_that_is_an_edge_is_not_a_thing() -> None:
    """The three-line filter, on the shapes it exists to reject."""
    from world_state.perceive import _worth_keeping

    check("a chair-sized box is kept", _worth_keeping([0.3, 0.3, 0.5, 0.6]), True)
    check("half the frame is the floor", _worth_keeping([0.0, 0.0, 0.9, 0.9]), False)
    check("a speck is a highlight", _worth_keeping([0.5, 0.5, 0.52, 0.52]), False)
    check("a skirting board is an edge",
          _worth_keeping([0.0, 0.5, 0.95, 0.56]), False)
    check("a box with no width is nothing",
          _worth_keeping([0.5, 0.5, 0.5, 0.7]), False)


def test_a_crop_with_no_picture_in_it_is_not_a_region() -> None:
    """**Reproduces the entity the rover built out of blown-out windows.**

    The filter above works on the shape of a box and nothing else, so a window
    the camera has burnt to white and a bare patch of wall both sail through it:
    right size, right aspect, nothing inside. On the drive of 2026-09-02, 58 of
    the 338 regions the rover stored -- 17% -- were one or the other, and they do
    real harm rather than merely wasting a slot, because two pictures of nothing
    look like each other. `object:14` was built almost entirely out of them and
    wandered four metres across the map.

    Two ways to have no picture, so two numbers: no contrast at all, and mostly
    burnt out. A window frame across a white sky has plenty of contrast and still
    says nothing about what is behind it, which is why the second test exists.
    """
    try:
        import numpy
    except ImportError as error:
        SKIP.append(f"rejecting a crop with no picture in it ({error})")
        return

    from world_state.perceive import _blank

    wall = numpy.full((40, 40, 3), 200.0, dtype="float32")
    check("a bare patch of wall is not a region", _blank(numpy, wall), True)

    window = numpy.zeros((40, 40, 3), dtype="float32")
    window[:, :34] = 255.0                      # burnt-out glass, frame at one edge
    check("...nor is a window the camera has blown out",
          _blank(numpy, window), True)

    chair = numpy.zeros((40, 40, 3), dtype="float32")
    chair[:20] = 40.0
    chair[20:] = 190.0                          # dark against a light floor
    check("but something with a picture in it is",
          _blank(numpy, chair), False)

    # A pale lampshade against a wall is the case this must not eat: bright, and
    # burnt out over part of itself, but with a shape in it. The line is drawn by
    # contrast and by how much of the crop is at full white, not by brightness.
    lamp = numpy.full((40, 40, 3), 210.0, dtype="float32")
    lamp[8:32, 8:32] = 255.0                    # half the crop, blown out
    lamp[32:] = 120.0                           # the table it stands on
    check("...and neither is a pale thing with a shape in it",
          _blank(numpy, lamp), False)


def test_a_cushion_inside_a_sofa_is_not_a_second_thing() -> None:
    """Reproduces what made the rover record one sofa several times over.

    FastSAM segments everything, parts included, so a sofa comes back as a sofa
    *and* as its arm, its back and each of its cushions. Ordinary suppression
    cannot see that: it divides the overlap by the union of the pair, and a
    cushion inside a sofa scores about 0.15 that way -- below any threshold
    worth setting -- so both survived, both got a bearing, and both became
    entities. On ten of the rover's own frames 57% of everything it embedded
    was a piece of something else it embedded from the same picture.

    Dividing by the smaller box instead is the whole fix, and these are the two
    cases that have to come out differently: a part inside a whole, and two
    genuinely separate things that merely touch.
    """
    try:
        import numpy
    except ImportError as error:
        SKIP.append(f"suppressing a part inside a whole ({error})")
        return

    from world_state.perceive import FASTSAM_OVERLAP, _suppress

    sofa = [0.10, 0.30, 0.80, 0.75]
    cushion = [0.30, 0.45, 0.55, 0.70]      # wholly inside it
    lamp = [0.78, 0.10, 0.95, 0.50]         # beside it, overlapping a corner
    boxes = numpy.array([sofa, cushion, lamp])
    scores = numpy.array([0.9, 0.8, 0.7])
    kept = sorted(int(index) for index in
                  _suppress(numpy, boxes, scores, FASTSAM_OVERLAP))
    check("the sofa and the lamp are two things", kept, [0, 2])

    # And the other way round, because suppression keeps the higher score and
    # the part is often the more confident box. The whole must win on the merit
    # of containing the other, not on having scored better.
    scores = numpy.array([0.6, 0.95, 0.7])
    kept = sorted(int(index) for index in
                  _suppress(numpy, boxes, scores, FASTSAM_OVERLAP))
    check("...whichever of the pair scored higher", len(kept), 2)

    # The case the old rule got right, which the new one must not break: two
    # rival guesses at the same object, near enough the same size.
    boxes = numpy.array([[0.1, 0.1, 0.5, 0.5], [0.12, 0.12, 0.52, 0.52]])
    kept = _suppress(numpy, boxes, numpy.array([0.9, 0.8]), FASTSAM_OVERLAP)
    check("two guesses at one object are still one", len(kept), 1)

    # Two things that touch are two things. A quarter of the smaller box lies
    # inside the larger here, which is well under the threshold.
    boxes = numpy.array([[0.1, 0.1, 0.5, 0.5], [0.4, 0.4, 0.8, 0.8]])
    kept = _suppress(numpy, boxes, numpy.array([0.9, 0.8]), FASTSAM_OVERLAP)
    check("...but two that only touch are two", len(kept), 2)



# --- an inspection through the encoders --------------------------------------
#
# The path the rover actually uses now. What is worth proving offline is that
# what was *measured* survives into the database unchanged and that what was not
# measured stays empty: a bearing invented from a missing pose would be the one
# failure this whole design exists to avoid.


def a_sighting(bbox=None, dino=None, siglip=None):
    """One measured region, and there is nothing on it saying what it is.

    That is the whole shape of what perception returns now: a box, a region
    score and two vectors. Anything downstream that wants to tell two of
    these apart has to do it from the vectors or from where they point.
    """
    from world_state.perception_client import Sighting

    return Sighting(bbox=bbox or [0.1, 0.3, 0.5, 0.9],
                    region_score=0.83, area=0.24,
                    dino=dino if dino is not None else a_vector(1.0, 0.0),
                    siglip=siglip if siglip is not None
                    else a_vector(0.5, 0.5))


def a_seeing_inspector(directory, looks=None, fail="", capture=None, pose=None,
                       fov_deg=100.0):
    from world_state.perception_client import FakeEyes

    store = a_store(directory)
    eyes = FakeEyes(looks or [], fail=fail)
    return store, eyes, Inspector(
        store, eyes, capture or a_capture(pan=20.0, tilt=-5.0),
        pose or a_pose(), fov_deg=fov_deg)


def test_an_inspection_through_the_encoders_keeps_what_it_measured() -> None:
    with tempfile.TemporaryDirectory() as directory:
        seen, matched = a_vector(0.6, 0.8), a_vector(0.0, 1.0)
        store, eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting(dino=seen, siglip=matched),
                         a_sighting(bbox=[0.6, 0.2, 0.9, 0.7])]])
        result = inspector.inspect()
        check("the inspection succeeded", result["ok"], True)
        check("...storing one observation per region", result["stored"], 2)
        check("...with a bearing for each", result["placed"], 2)
        check("...and claiming no identity for either", result["created"], 0)
        check("...having asked the encoders exactly once", len(eyes.calls), 1)

        rows = store.db.execute(
            "SELECT * FROM observations ORDER BY id").fetchall()
        first = dict(rows[0])
        check("the appearance vector is stored as the bytes it arrived as",
              first["dino_blob"], seen)
        check("...and so is the semantic one", first["siglip_blob"], matched)
        check("the backend that produced them travels with them",
              first["vectors_from"], "fake")
        check("what drew the box is recorded apart from what named it",
              first["region_source"], "fastsam")
        check("no identity was written", first["entity_id"], None)
        check("nothing was prompted, so no prompt version is claimed",
              first["prompt_version"], None)

        store.close()

def test_a_bearing_is_the_pose_the_gimbal_and_the_box_together() -> None:
    """The arithmetic `view.ray` already proves, checked where it is stored."""
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting(bbox=[0.4, 0.3, 0.6, 0.9])]],
            capture=a_capture(pan=20.0), pose=a_pose(heading=90.0), fov_deg=100.0)
        inspector.inspect()
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        # Heading 90 to the left, gimbal 20 to the right, and a box centred in
        # the picture: 90 - 20 + 0.
        check("the stored bearing is heading minus pan plus the box offset",
              row["bearing_deg"], 70.0)
        check("...and the box's width became a cone", row["span_deg"], 20.0)

        store.close()

def test_without_a_pose_nothing_pretends_to_know_a_direction() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()]], pose=lambda: None)
        result = inspector.inspect()
        check("the observation is still kept", result["stored"], 1)
        check("...but nothing was placed", result["placed"], 0)
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("the bearing is empty rather than guessed", row["bearing_deg"], None)
        check("and the popup is told why in words",
              "without a bearing" in result["detail"], True)

        store.close()

def test_a_sidecar_that_is_down_writes_one_row_and_no_observations() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, fail="the perception sidecar is not answering")
        result = inspector.inspect()
        check("the inspection failed rather than raising", result["ok"], False)
        check("...saying which sidecar", "perception" in result["error"], True)
        check("nothing was stored", store.summary()["observations"], 0)
        check("...and exactly one diagnostics row was written",
              len(store.inferences()), 1)
        store.close()


def test_every_failure_leaves_the_world_exactly_as_it_was() -> None:
    """The ways an inspection can fail, and one rule for all of them."""
    cases = [
        ("the sidecar is not running",
         dict(fail="nothing is listening on 8776"), "unavailable"),
        ("the camera gave nothing",
         dict(looks=[[a_sighting()]],
              capture=a_capture(ok=False, error="the camera gave nothing")),
         "no_frame"),
    ]
    for name, kwargs, status in cases:
        with tempfile.TemporaryDirectory() as directory:
            store, _eyes, inspector = a_seeing_inspector(directory, **kwargs)
            # Something already in the world, so "unchanged" is a claim with
            # content rather than "still empty".
            store.record([a_sighting()], capture={"frame_id": "f0"})
            before = store.summary()
            result = inspector.inspect()
            check(f"{name}: the inspection fails", result["ok"], False)
            check(f"{name}: ...saying which kind of failure", result["status"],
                  status)
            after = store.summary()
            check(f"{name}: ...and changes no entity",
                  after["entities"], before["entities"])
            check(f"{name}: ...and no observation",
                  after["observations"], before["observations"])
            check(f"{name}: ...but is written down where the popup shows it",
                  store.inferences()[0]["status"], status)
            store.close()


def test_only_one_inspection_at_a_time() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()]])
        inspector._lock.acquire()
        inspector.started_at = time.monotonic()
        try:
            result = inspector.inspect()
            check("a second request is refused rather than queued",
                  result["status"], "busy")
            check("...and nothing was stored for it", store.inferences(), [])
        finally:
            inspector._lock.release()
        check("...and the next one runs normally", inspector.inspect()["ok"], True)
        store.close()


def test_a_clear_is_refused_while_an_inspection_runs() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()]])
        inspector.inspect()
        inspector._lock.acquire()
        try:
            check("the inspector says it is busy", inspector.busy, True)
        finally:
            inspector._lock.release()
        check("...and is free again afterwards", inspector.busy, False)
        check("the clear then works", store.clear()["ok"], True)
        store.close()


def test_perception_writes_no_name_and_no_warning_about_one() -> None:
    """Nothing measures what a region is, so nothing pretends to.

    There used to be a name here and a warning beside it when the name said
    nothing in particular. Both are gone with the word list that produced
    them, and what has to be true now is that the row says so by holding
    nothing rather than by holding a plausible-looking guess -- the same rule
    the bearing columns follow when there was no pose.
    """
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()]])
        inspector.inspect()
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("nothing wrote a name, so the column is null rather than blank",
              row["label"], None)
        check("...and no leftover score for one", row["label_score"], None)
        check("...and no warning about a name that was never there",
              row["note"], None)
        check("what it does carry is the measurement",
              (row["bearing_deg"] is not None, bool(row["siglip_blob"])),
              (True, True))
        import json as _json
        raw = _json.loads(row["raw_json"])
        check("...and what the sidecar said has no name in it either",
              sorted(raw), ["area", "region_score"])
        store.close()



def test_a_bearing_never_runs_past_half_a_turn() -> None:
    """Measured on the rover before it was fixed: -205.9 degrees, stored.

    Three numbers are added to make a bearing -- the rover's heading, the
    gimbal's pan and the box's offset -- and each can be large. A bearing outside
    (-180, 180] points exactly where its wrapped twin does and compares with
    nothing, so the resolver would see two directions where there is one.
    """
    facing = {"pose": {"x_m": 0.0, "y_m": 0.0, "heading_deg": -150.4},
              "observer_pan_deg": 50.0, "bbox": [0.4, 0.3, 0.6, 0.9]}
    check("the sum wraps rather than running past half a turn",
          view.ray(facing, 100.0)["bearing_deg"], 159.6)
    other_way = dict(facing, observer_pan_deg=-200.0)
    check("...in the other direction too",
          -180.0 < view.ray(other_way, 100.0)["bearing_deg"] <= 180.0, True)



def test_the_pending_pool_holds_what_has_a_direction_but_no_home() -> None:
    """Phase 2's waiting room, and what is deliberately not in it."""
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting(), a_sighting(bbox=[0.6, 0.2, 0.9, 0.7])]])
        inspector.inspect()
        pending = store.unplaced()
        check("both observations are waiting to be placed", len(pending), 2)
        check("...each with a direction", all(row["bearing_deg"] is not None
                                              for row in pending), True)
        check("...and the vectors the resolver will need",
              all(row["dino_blob"] and row["siglip_blob"] for row in pending), True)
        check("...oldest first, so the pool cannot starve",
              pending[0]["id"] < pending[1]["id"], True)

        # An observation with no pose was never pending anything: no later look
        # can supply a direction that was never measured.
        store.record([a_sighting()], capture={"frame_id": "f9"})
        check("an observation with no bearing is not in the pool",
              len(store.unplaced()), 2)
        check("...but it is still in the history", store.summary()["observations"], 3)

        check("a bearing from another map is not comparable and is left out",
              len(store.unplaced(map_session=99)), 0)
        store.close()


def test_the_vectors_never_reach_the_wire_by_accident() -> None:
    """Raw bytes in a row that is about to be JSON is a crash, not a nuisance."""
    with tempfile.TemporaryDirectory() as directory:
        seen = a_vector(0.6, 0.8)
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting(dino=seen)]])
        inspector.inspect()
        shown = store.observations()[0]
        check("the console's copy carries no raw vector",
              ("dino_blob" in shown, "siglip_blob" in shown), (False, False))
        check("...but says they are there and how big",
              (shown["dino_bytes"], shown["siglip_bytes"]), (32, 32))
        import json as _json
        _json.dumps(shown)
        check("...so the row can be sent as JSON at all", True, True)
        check("the resolver's copy keeps them",
              store.unplaced()[0]["dino_blob"], seen)
        store.close()



# --- the resolver ------------------------------------------------------------
#
# The part the proof-of-concept failed at. Every test here is a case the rover
# actually has to survive rather than a check that the code runs: two identical
# chairs, a rover that only turned on the spot, and an appearance score high
# enough to be tempting and pointing at the wrong side of the room.


def a_vector(*values, width=8):
    """A float32 vector, padded, so appearance can be steered in a test."""
    import struct

    numbers = list(values) + [0.0] * (width - len(values))
    return struct.pack(f"<{width}f", *numbers)


def observe(store, x, y, bearing, vector=None, inference=None, fov_deg=100.0):
    """One look at something, from a place, along a bearing.

    The box is centred, so the bearing really is the pose's heading minus the
    gimbal's pan and nothing else -- which keeps these tests about the resolver
    rather than about `view.ray`, which has its own.
    """
    from world_state.perception_client import Sighting

    seen = [Sighting(bbox=[0.45, 0.3, 0.55, 0.9],
                     dino=vector if vector is not None else a_vector(1.0, 0.0),
                     siglip=a_vector(0.5, 0.5))]
    store.record(seen, capture={"frame_id": "f", "pan": 0.0,
                                "pose": {"x_m": x, "y_m": y,
                                         "heading_deg": bearing}},
                 fov_deg=fov_deg, region_source="fastsam", vectors_from="fake",
                 inference_id=inference)


def test_two_looks_from_two_places_make_one_lasting_thing() -> None:
    """The whole point, in its simplest form."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        observe(store, 0.0, 0.0, 45.0, inference=1)
        observe(store, 6.0, 0.0, 135.0, inference=2)
        result = resolve.resolve(store)
        check("one thing was created", result["created"], 1)
        check("...and nothing was left ambiguous", result["ambiguous"], 0)
        check("...and the pool is empty", len(store.unplaced()), 0)

        placed = store.placed()
        check("the thing has a position", len(placed), 1)
        check("...where the two bearings actually cross",
              (round(placed[0]["placement"]["x_m"]),
               round(placed[0]["placement"]["y_m"])), (3, 3))
        check("...with an uncertainty rather than a claim of precision",
              placed[0]["placement"]["uncertainty_m"] > 0, True)
        check("...and both looks attached to it",
              placed[0]["observation_count"], 2)
        check("the popup can be told which two looks placed it",
              "crossed at" in result["decisions"][0]["why"], True)
        store.close()


def test_a_rover_that_only_turned_on_the_spot_places_nothing() -> None:
    """Rays from one point meet nowhere useful, and saying so is the point."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        observe(store, 1.0, 1.0, 40.0, inference=1)
        observe(store, 1.0, 1.0, 50.0, inference=2)
        result = resolve.resolve(store)
        check("nothing was created", result["created"], 0)
        check("...and both observations are still waiting",
              len(store.unplaced()), 2)
        check("...which is reported rather than silent",
              result["still_waiting"], 2)
        store.close()


def test_two_identical_chairs_are_not_guessed_at_from_two_places() -> None:
    """**The test the whole design exists to pass.**

    Two chairs and two viewpoints give four rays and four valid crossings: the
    two real chairs and two phantoms where a ray to one chair crosses a ray to
    the other. All four are sound geometry, and appearance cannot break the tie
    -- measured on this rover, the twin chair scores *higher* than the same chair
    seen from a new angle. From two places the answer is not knowable, so the
    resolver must wait rather than invent two things in the wrong places.
    """
    near, far = (2.7, 0.4), (3.0, 3.0)

    def seen_from(x_m, y_m, chair, inference):
        """The bearing from a place to a chair, rather than a number typed in.

        Typed-in bearings were rounded to a tenth of a degree and one of them was
        a degree and a half out, which passed only because a bearing used to be
        believed to five degrees. What the test means is that the rover stood
        here and the chair is there.
        """
        bearing = math.degrees(math.atan2(chair[1] - y_m, chair[0] - x_m))
        observe(store, x_m, y_m, round(bearing, 2), inference=inference)

    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            seen_from(0.0, 0.0, far, 1)
            seen_from(0.0, 0.0, near, 1)
            seen_from(6.0, 0.0, far, 2)
            seen_from(6.0, 0.0, near, 2)
            result = resolve.resolve(store)
            check("nothing was placed from two viewpoints", result["created"], 0)
            check("...and all four looks are still waiting",
                  len(store.unplaced()), 4)

            # A third viewpoint separates them: a real chair is agreed by every
            # ray aimed at it, and a phantom by only the two that made it.
            seen_from(3.0, -3.0, far, 3)
            seen_from(3.0, -3.0, near, 3)
            result = resolve.resolve(store)
            check("a third look from somewhere else settles it",
                  result["created"], 2)
            places = sorted((round(one["placement"]["x_m"], 1),
                             round(one["placement"]["y_m"], 1))
                            for one in store.placed())
            check("...as two things in two places", len(places), 2)
            check("...far enough apart to be told apart",
                  abs(places[0][1] - places[1][1]) > 1.5, True)
            check("...and each within a handspan of the chair it is",
                  max(min(math.dist(place, chair) for chair in (near, far))
                      for place in places) < 0.5, True)
        finally:
            store.close()


def test_one_television_seen_six_times_is_one_television() -> None:
    """**The duplicate the validation drive of 2026-09-02 produced.**

    Driven round three places and inspected at six headings from each, the rover
    placed four televisions, two of them eight centimetres apart, and three people
    where there was one. Two entities that are really one thing is the failure
    this whole design exists to prevent.

    The cause is an ordering one. A new thing absorbs the rays that support it,
    but never two rays from the same frame -- two regions of one frame are two
    different things by construction -- so with six frames looking at one
    television, the first entity takes a few and the rest are still waiting. They
    then pair with each other into a second television, because the list of things
    already placed was read once before any of this began and the thing created a
    moment ago is not on it.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            # One television at (3, 3), seen from six places around it, each look
            # its own inspection the way a survey makes them.
            telly = (3.0, 3.0)
            from_where = [(0.0, 0.0), (0.6, -0.4), (6.0, 0.0),
                          (5.4, 0.5), (3.0, -3.0), (2.4, -2.6)]
            for index, (x_m, y_m) in enumerate(from_where, start=1):
                bearing = math.degrees(math.atan2(telly[1] - y_m, telly[0] - x_m))
                observe(store, x_m, y_m, round(bearing, 2), inference=index)
            result = resolve.resolve(store)
            check("six looks at one television make one television",
                  result["created"], 1)
            placed = store.placed()
            check("...and only one thing is placed", len(placed), 1)
            check("...where the television is",
                  math.dist((placed[0]["placement"]["x_m"],
                             placed[0]["placement"]["y_m"]), telly) < 0.2, True)
            check("...with the later looks joining it rather than being ignored",
                  placed[0]["observation_count"] >= 4, True)
        finally:
            store.close()


def test_a_thing_cannot_be_seen_through_a_wall() -> None:
    """**The fault that put two rooms inside one entity, with its own numbers.**

    A bearing carries no range, so two bearings cross *somewhere* whatever they
    are pointed at -- and two cameras aimed at two different things a couple of
    metres away in two different rooms give rays that meet ten metres off, at a
    healthy angle and off a healthy baseline. Every guard in `locate` accepted
    that, because none of them asked whether the rover could have seen that far
    in that direction at all.

    The rover could not, and the rover already knew: its own occupancy grid says
    where the first wall on a bearing is. On the run of 2026-09-02 it placed one
    thing at (9.87, 1.29) -- 4.7 m outside the edge of its own map -- from
    bearings whose first obstacle was 1.1 and 1.95 m away, and another 3.8 m out
    through a wall 55 cm in front of the rover. Those two are below.
    """
    # Two bearings from the rover's own record, and the crossing they made.
    first = {"x_m": 3.028, "y_m": 6.26, "bearing_deg": -36.0, "span_deg": 10.9}
    second = {"x_m": -0.724, "y_m": 0.081, "bearing_deg": 7.3, "span_deg": 10.5}
    unbounded = locate.fix(first, second)
    check("the two bearings cross, which is why this was ever placed",
          unbounded is not None, True)
    check("...ten and a half metres out, and confident about it",
          (round(math.dist((unbounded["x_m"], unbounded["y_m"]),
                           (second["x_m"], second["y_m"])), 1),
           unbounded["uncertainty_m"]), (10.5, 0.713))

    # What the map said at the time: a wall about a metre ahead of each of them.
    walled = locate.fix({**first, "reach_m": 1.95},
                        {**second, "reach_m": 1.10})
    check("...and with the map consulted, there is no such thing to place",
          walled, None)

    # The case that must keep working, and the reason the margin exists: a thing
    # a couple of metres away with the far wall of the room behind it.
    close = locate.fix({"x_m": 0.0, "y_m": 0.0, "bearing_deg": 45.0,
                        "reach_m": 6.0, "span_deg": 10.0},
                       {"x_m": 6.0, "y_m": 0.0, "bearing_deg": 135.0,
                        "reach_m": 6.0, "span_deg": 10.0})
    check("a thing in front of a far wall is still placed", close is not None,
          True)
    check("...and so is one standing right against the wall itself",
          locate.fix({"x_m": 0.0, "y_m": 0.0, "bearing_deg": 45.0,
                      "reach_m": 4.1, "span_deg": 10.0},
                     {"x_m": 6.0, "y_m": 0.0, "bearing_deg": 135.0,
                      "reach_m": 4.1, "span_deg": 10.0}) is not None, True)

    # And the other half of it: a bearing may not join a thing that sits behind
    # its own wall, however well the angle lines up.
    point = {"x_m": 1.66, "y_m": -2.93, "uncertainty_m": 0.56}
    aimed = {"x_m": -0.72, "y_m": 0.08, "bearing_deg": -50.2, "span_deg": 10.0}
    check("the bearing does point that way", locate.agrees(point, aimed), True)
    check("...but not through a wall 55 cm ahead of it",
          locate.agrees(point, {**aimed, "reach_m": 0.55}), False)
    check("a bearing the map cannot bound is left alone",
          locate.agrees(point, {**aimed, "reach_m": None}), True)


def test_a_thing_does_not_move_out_from_under_its_own_evidence() -> None:
    """**The wandering entity the drive of 2026-09-02 recorded, with its numbers.**

    An entity is re-placed from everything attached to it whenever a look joins,
    and it used to take the pair of bearings with the smallest uncertainty --
    which is a statement about two rays and about nothing else. So one lucky pair
    could move a thing with a dozen looks behind it clean out from under all of
    them: 13 of that drive's 151 re-placements moved more than half a metre, one
    of them 2.6 m in a single step, and afterwards 45% of every entity's own
    bearings missed its own stated position.

    The four bearings below are `object:14`'s, taken out of the rover's own
    database. The tightest pair among them lands somewhere two of the four
    disagree with; the pair all four agree with is two metres away and has a
    wider uncertainty, and it is the right answer. `world_state/replay.py` is
    what this was found with.
    """
    measured = [(-3.739, 2.906, -82.2), (-4.351, 4.421, -89.6),
                (-3.782, 2.911, -88.6), (0.544, 0.078, -119.1)]
    rays = [{"x_m": x, "y_m": y, "bearing_deg": bearing, "span_deg": 20.0}
            for x, y, bearing in measured]

    def tightest(candidates):
        """What `best_fix` used to do, kept here so the test is a comparison."""
        best = None
        for index, first in enumerate(candidates):
            for second in candidates[index + 1:]:
                found = locate.fix(first, second)
                if found is None:
                    continue
                if best is None or found["uncertainty_m"] < best["uncertainty_m"]:
                    best = found
        return best

    was = tightest(rays)
    now = locate.best_fix(rays)
    check("the tightest pair of these four is agreed by only half of them",
          sum(1 for ray in rays if locate.agrees(was, ray)), 2)
    check("...and the placement chosen instead is agreed by all four",
          sum(1 for ray in rays if locate.agrees(now, ray)), 4)
    check("...which is a different place, not a rounding",
          math.dist((was["x_m"], was["y_m"]), (now["x_m"], now["y_m"])) > 1.5,
          True)
    check("...and it is allowed to be the less certain of the two",
          now["uncertainty_m"] > was["uncertainty_m"], True)


def test_the_reason_survives_the_inspection_that_decided_it() -> None:
    """The question a person asks of an identity is why, not what.

    The resolver has always written a sentence about each decision, but it went
    back in the reply to whichever inspection happened to trigger the resolve and
    was gone by the time anybody opened the console to ask. It is kept on the
    observation now, which is where the rest of that decision's evidence is.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            observe(store, 0.0, 0.0, 45.0, inference=1)
            observe(store, 6.0, 0.0, 135.0, inference=2)
            result = resolve.resolve(store)
            check("the thing was placed", result["created"], 1)
            notes = [one["note"] for one in store.observations()
                     if one.get("entity_id")]
            check("...and both looks say why they belong to it",
                  len(notes), 2)
            check("...in the resolver's own words",
                  all(note and "crossed at" in note for note in notes), True)
        finally:
            store.close()


def test_a_third_look_joins_the_thing_it_points_at() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        observe(store, 0.0, 0.0, 45.0, inference=1)
        observe(store, 6.0, 0.0, 135.0, inference=2)
        resolve.resolve(store)
        entity_id = store.placed()[0]["id"]

        observe(store, 3.0, -1.0, 90.0, inference=3)
        result = resolve.resolve(store)
        check("the new look was matched rather than made into a second thing",
              (result["matched"], result["created"]), (1, 0))
        check("...to the thing that was already there",
              result["decisions"][0]["entity_id"], entity_id)
        check("...and the reason names the distance",
              "m away" in result["decisions"][0]["why"], True)
        check("the world still holds one thing", len(store.placed()), 1)
        check("...with three looks behind it",
              store.placed()[0]["observation_count"], 3)
        store.close()


def test_appearance_cannot_overrule_where_a_thing_is() -> None:
    """The redundant-furniture rule, stated as a test.

    A crop that looks *exactly* like the stored exemplar, on a bearing pointing
    at the other side of the room, must not match. This is the failure mode of
    every appearance-first design and the reason geometry is the key here.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        twin = a_vector(1.0, 0.0)
        observe(store, 0.0, 0.0, 45.0, vector=twin, inference=1)
        observe(store, 6.0, 0.0, 135.0, vector=twin, inference=2)
        resolve.resolve(store)
        check("something was placed", len(store.placed()), 1)

        # The same appearance entirely, four metres away across the room.
        observe(store, 0.0, 0.0, -60.0, vector=twin, inference=3)
        result = resolve.resolve(store)
        check("a perfect appearance match on the wrong bearing is not a match",
              result["matched"], 0)
        check("...and it is not quietly made into a new thing either",
              result["created"], 0)
        check("...it waits for a second bearing of its own",
              len(store.unplaced()), 1)
        store.close()


def test_two_placed_things_on_one_bearing_are_ambiguous_not_a_guess() -> None:
    """One chair directly behind another, from where the rover is standing.

    Both are placed, both are consistent with the new bearing, and appearance
    cannot separate them. Attaching the look to the nearer one would be a guess
    dressed as an answer, so it is attached to neither and the popup is told why.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        observe(store, 0.0, 0.0, 45.0, inference=1)      # a chair at (2, 2)
        observe(store, 4.0, 0.0, 135.0, inference=2)
        resolve.resolve(store)
        observe(store, 0.0, 4.0, 0.0, inference=3)       # another at (4, 4)
        observe(store, 4.0, 0.0, 90.0, inference=4)
        resolve.resolve(store)
        check("both chairs were placed", len(store.placed()), 2)

        # From the origin the two are in exactly the same direction.
        observe(store, 0.0, 0.0, 45.0, inference=5)
        result = resolve.resolve(store)
        check("a bearing consistent with both is left alone",
              result["ambiguous"], 1)
        check("...rather than attached to either", result["matched"], 0)
        check("...and the reason says so in words",
              "equally consistent" in result["decisions"][0]["why"], True)
        check("...and the look is still in the pool",
              len(store.unplaced()), 1)
        store.close()


def test_a_chair_and_a_ceiling_light_are_never_the_same_thing() -> None:
    """The gate that replaced the word list, doing the one job it may do.

    Two bearings that cross beautifully are still not two looks at one thing
    if the crops behind them look nothing like each other. This used to be
    decided by comparing two names against a hand-written list of synonyms;
    it is decided now by comparing the appearance vectors, which is the same
    question asked of something the rover actually measured.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            # A perfect crossing at (3, 3), and two crops with nothing in
            # common: on this rover a chair against a spray bottle is 0.122.
            observe(store, 0.0, 0.0, 45.0, vector=a_vector(1.0, 0.0),
                    inference=1)
            observe(store, 6.0, 0.0, 135.0, vector=a_vector(0.0, 1.0),
                    inference=2)
            result = resolve.resolve(store)
            check("two things that look nothing alike are not one thing",
                  result["created"], 0)
            check("...and both looks are left waiting rather than merged",
                  len(store.unplaced()), 2)

            # The identical geometry, with two crops that do look alike.
            store.clear()
            observe(store, 0.0, 0.0, 45.0, vector=a_vector(1.0, 0.05),
                    inference=1)
            observe(store, 6.0, 0.0, 135.0, vector=a_vector(1.0, 0.0),
                    inference=2)
            check("the same crossing between two views of one thing is placed",
                  resolve.resolve(store)["created"], 1)
        finally:
            store.close()


def test_the_right_place_is_not_enough_if_it_looks_wrong() -> None:
    """What the list of things that move used to buy, without the list.

    A bottle is a poor thing to identify by position, because it was moved;
    the old rule knew which things those were by name, and there are no names
    any more. What survives is the half that never needed one: a look on a
    bearing that points straight at a placed thing, but whose crop looks
    nothing like anything that thing has shown, is not that thing.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        bottle = a_vector(1.0, 0.0)
        observe(store, 0.0, 0.0, 45.0, vector=bottle, inference=1)
        observe(store, 6.0, 0.0, 135.0, vector=bottle, inference=2)
        resolve.resolve(store)
        check("two looks that agree place the thing", len(store.placed()), 1)

        # The right place, and nothing like it to look at.
        observe(store, 3.0, -1.0, 90.0, vector=a_vector(0.0, 1.0), inference=3)
        result = resolve.resolve(store)
        check("a look in the right place that looks wrong is not matched",
              result["matched"], 0)
        check("...nor quietly made into a second thing", result["created"], 0)
        check("...it waits for a second bearing of its own",
              len(store.unplaced()), 1)
        store.close()


def test_the_evidence_survives_the_decision() -> None:
    """An entity is an opinion; the observations behind it are history."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        observe(store, 0.0, 0.0, 45.0, inference=1)
        observe(store, 6.0, 0.0, 135.0, inference=2)
        resolve.resolve(store)
        entity_id = store.placed()[0]["id"]
        rows = store.observations(entity_id)
        check("both observations still hold the bearing they measured",
              [row["bearing_deg"] for row in rows], [135.0, 45.0])
        check("...and the pose behind it",
              all(row["pose"] for row in rows), True)
        check("the entity keeps an exemplar of what it looked like",
              len(store.exemplars(entity_id, width=32)), 2)
        store.close()


def test_one_entity_can_be_sent_to_a_console_like_the_list_can() -> None:
    """The console's detail pane said "nothing selected" for every entity.

    Not a rendering fault and not a race. `store.entity` handed back the row as
    SQLite produced it, exemplars and all, so the reply to `world_state_entity`
    held a raw float32 BLOB; the daemon could not turn it into JSON, wrote no
    reply at all, and the page waited forever for a payload that never came.
    Clicking a thing therefore did nothing, in a popup whose whole purpose is
    showing the looks behind a thing.

    Two properties, and the second is what made the first easy to miss: the row
    has to be serialisable, and it has to carry the same decoded placement the
    list carries -- a reply that got through without one would have shown a
    placed thing as having no position.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            entity_id = store.create_entity()
            store.place(entity_id, {"x_m": 1.0, "y_m": 2.0, "uncertainty_m": 0.3,
                                    "baseline_m": 3.0, "parallax_deg": 30.0}, 1)
            store.add_exemplar(entity_id, a_vector(1.0, 0.0))
            one = store.entity(entity_id)
            check("the row carries no raw vector", "exemplars" in one, False)
            check("...but says how many it has", one["exemplar_count"], 1)
            check("...and its position, decoded rather than as stored JSON",
                  (one["placement"] or {}).get("x_m"), 1.0)
            import json as _json
            _json.dumps(one)
            check("...so it can be sent to a console at all", True, True)
            check("an entity that is not there is still None rather than a crash",
                  store.entity("object:404"), None)
        finally:
            store.close()


def test_a_placement_belongs_to_the_map_it_was_measured_in() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        observe(store, 0.0, 0.0, 45.0, inference=1)
        observe(store, 6.0, 0.0, 135.0, inference=2)
        resolve.resolve(store)
        check("the thing is placed in this map", len(store.placed(1)), 1)
        store.new_map_session()
        check("...and in no other", len(store.placed(2)), 0)

        observe(store, 3.0, -1.0, 90.0, inference=3)
        result = resolve.resolve(store)
        check("a look in the new map cannot join a thing placed in the old one",
              result["matched"], 0)
        store.close()



def test_an_inspection_settles_identity_as_well_as_measuring() -> None:
    """The two halves joined: measure, then decide, in that order.

    The order is the safety property. Everything measured is written down before
    anything is decided about it, so a resolver that fails leaves a rover with
    twelve honest observations rather than with a failed inspection.
    """
    with tempfile.TemporaryDirectory() as directory:
        store, eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()], [a_sighting()]],
            capture=a_capture(pan=0.0), pose=a_pose(0.0, 0.0, 45.0))
        first = inspector.inspect()
        check("the first look measures and settles nothing",
              (first["stored"], first["created"]), (1, 0))
        check("...and says it is waiting for a look from elsewhere",
              "waiting for a look from elsewhere" in first["detail"], True)

        inspector.pose = a_pose(6.0, 0.0, 135.0)
        second = inspector.inspect()
        check("the second look from another place places the thing",
              second["created"], 1)
        check("...and the popup is told which two looks did it",
              "crossed at" in second["decisions"][0]["why"], True)
        check("the world now holds one placed thing", len(store.placed()), 1)
        store.close()



# --- finding a thing by describing it ----------------------------------------


def _packed(*values):
    import struct
    return struct.pack(f"<{len(values)}f", *values)


def test_a_query_that_matches_nothing_says_so() -> None:
    """The part that matters, and the part a ranking alone cannot do.

    A list of scores always has a top, so the question is whether that top means
    anything. Measured on the rover it is the raw score that answers this and not
    the separation, so a field of near misses is not a match however flat it is.
    """
    query = _packed(1.0, 0.0, 0.0)
    # Everything here scores about 0.05 against the query: the shape a room full
    # of things that are not what was asked for produces.
    near = [{"id": n, "siglip_blob": _packed(0.05 + n * 0.0002, 1.0, 0.0)}
            for n in range(40)]
    answer = search.rank(query, near)
    check("a field of near misses is ranked", len(answer["matches"]), 10)
    check("...but not believed", answer["confident"], False)
    check("...and the reason is in words a person can read",
          "nothing here matches" in answer["detail"], True)
    check("...which quotes the score and the bar it missed",
          f"{search.MATCHES:.2f}" in answer["detail"], True)

    real = near + [{"id": 99, "siglip_blob": _packed(1.0, 0.02, 0.0)}]
    answer = search.rank(query, real)
    check("a match that scores well is believed", answer["confident"], True)
    check("...and it is the right one", answer["matches"][0]["observation_id"], 99)
    check("...with the score behind the verdict shown",
          answer["best"] >= search.MATCHES, True)


def test_a_search_says_which_part_of_the_frame_it_found() -> None:
    """A picture of a room is not an answer.

    A stored frame holds a dozen things and the match is one of them, so without
    the box the person is left to guess which. It travels from the store already
    decoded, under the same name the rest of the codebase uses.
    """
    query = _packed(1.0, 0.0, 0.0)
    rows = [{"id": 1, "siglip_blob": _packed(1.0, 0.0, 0.0),
             "frame_id": "f1", "bbox": [0.1, 0.2, 0.3, 0.4]},
            {"id": 2, "siglip_blob": _packed(0.0, 1.0, 0.0),
             "frame_id": "f1", "bbox": None}]
    answer = search.rank(query, rows)
    check("the match carries the box it was found in",
          answer["matches"][0]["bbox"], [0.1, 0.2, 0.3, 0.4])
    check("...and an observation without one says so rather than failing",
          answer["matches"][1]["bbox"], None)

    # And the store hands it over decoded rather than as the JSON it is stored as.
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            observe(store, 0.0, 0.0, 45.0)
            rows = store.searchable()
            check("the store decodes the box for a search", len(rows), 1)
            check("...as four numbers, not a string",
                  isinstance(rows[0]["bbox"], list), True)
            check("...and does not leak the column name",
                  "bbox_json" in rows[0], False)
        finally:
            store.close()


def test_a_flat_field_is_not_what_decides_a_match() -> None:
    """The rule this replaced, kept as a test so it cannot come back by accident.

    Two searches with the same separation and different scores must get different
    answers, and two with the same score and different separations the same one.
    Measured on the rover the separation told present from absent no better than
    a coin, so it must not be able to overturn the score.
    """
    query = _packed(1.0, 0.0, 0.0)
    crowd = [{"id": n, "siglip_blob": _packed(0.02, 1.0, n * 0.01)}
             for n in range(40)]

    # A real match with nothing else near it, and the same match in a room where
    # several things score almost as well. The separation differs greatly.
    alone = search.rank(query, crowd + [
        {"id": 1, "siglip_blob": _packed(1.0, 0.05, 0.0)}])
    among = search.rank(query, crowd + [
        {"id": 1, "siglip_blob": _packed(1.0, 0.05, 0.0)},
        {"id": 2, "siglip_blob": _packed(1.0, 0.07, 0.0)},
        {"id": 3, "siglip_blob": _packed(1.0, 0.09, 0.0)}])
    check("a thing seen once is found", alone["confident"], True)
    check("...and seeing it three times does not unfind it",
          among["confident"], True)
    check("...even though the separation has collapsed",
          among["stands_clear"] < alone["stands_clear"], True)


def test_too_little_seen_is_not_a_match_either() -> None:
    """Three stored regions cannot rule anything out, but they can still find
    something. What changes below a dozen is what the answer says about itself,
    not whether it is believed."""
    query = _packed(1.0, 0.0, 0.0)
    wrong = [{"id": n, "siglip_blob": _packed(0.04, 1.0, 0.0)}
             for n in range(3)]
    answer = search.rank(query, wrong)
    check("three things that are not it is not a match", answer["confident"], False)
    check("...and it says the rover has barely looked, rather than "
          "blaming the query", "not have looked at it yet" in answer["detail"],
          True)

    right = wrong + [{"id": 9, "siglip_blob": _packed(1.0, 0.0, 0.0)}]
    answer = search.rank(query, right)
    check("but a real match among four is still a match",
          answer["confident"], True)
    check("...and does not pretend to a separation worth reading",
          "spreads above" in answer["detail"], False)


def test_vectors_from_the_other_backend_are_not_ranked() -> None:
    """Comparing across backends would rank noise, so it is refused."""
    query = _packed(1.0, 0.0, 0.0)
    rows = [{"id": 1, "siglip_blob": _packed(1.0, 0.0, 0.0),
             "vectors_from": "onnxruntime"},
            {"id": 2, "siglip_blob": _packed(0.2, 1.0, 0.0),
             "vectors_from": "tensorrt"}]
    answer = search.rank(query, rows, backend="tensorrt")
    check("the row from the other backend is counted out",
          (answer["considered"], answer["skipped"]), (1, 1))
    check("...and the one that can be compared is ranked",
          answer["matches"][0]["observation_id"], 2)


TESTS = (
    test_an_empty_database_is_an_ordinary_thing_to_open,
    test_the_application_owns_the_identifiers,
    test_an_inspection_claims_no_identity_at_all,
    test_the_world_survives_the_process_that_wrote_it,
    test_a_database_from_an_older_build_still_opens,
    test_the_frame_is_kept_and_every_observation_points_at_it,
    test_a_missing_pose_is_recorded_as_missing,
    test_clearing_the_semantic_world_takes_its_frames_with_it,
    test_clearing_the_map_keeps_the_semantic_world,
    test_unreadable_stored_json_cannot_take_the_viewer_down,
    test_a_reader_is_not_blocked_by_a_writer,
    test_an_observation_becomes_a_bearing_from_a_measured_pose,
    test_the_rays_of_one_entity_are_bounded_and_oldest_first,
    test_two_bearings_from_two_places_locate_a_thing,
    test_turning_on_the_spot_locates_nothing,
    test_two_looks_along_the_same_line_are_one_look,
    test_uncertainty_grows_when_the_baseline_shrinks,
    test_two_identical_chairs_stay_two_things,
    test_a_fix_on_top_of_the_camera_is_not_a_thing_in_the_room,
    test_a_bearing_at_the_edge_of_a_television_still_points_at_it,
    test_the_best_pair_places_the_thing,
    test_a_board_with_no_engines_falls_back_and_says_why,
    test_a_query_can_be_embedded_before_anything_has_been_looked_at,
    test_both_backends_answer_the_same_four_questions,
    test_the_installer_builds_exactly_the_engines_the_runtime_opens,
    test_a_region_that_is_an_edge_is_not_a_thing,
    test_a_crop_with_no_picture_in_it_is_not_a_region,
    test_a_cushion_inside_a_sofa_is_not_a_second_thing,
    test_an_inspection_through_the_encoders_keeps_what_it_measured,
    test_a_bearing_is_the_pose_the_gimbal_and_the_box_together,
    test_without_a_pose_nothing_pretends_to_know_a_direction,
    test_a_sidecar_that_is_down_writes_one_row_and_no_observations,
    test_every_failure_leaves_the_world_exactly_as_it_was,
    test_only_one_inspection_at_a_time,
    test_a_clear_is_refused_while_an_inspection_runs,
    test_perception_writes_no_name_and_no_warning_about_one,
    test_a_bearing_never_runs_past_half_a_turn,
    test_the_pending_pool_holds_what_has_a_direction_but_no_home,
    test_the_vectors_never_reach_the_wire_by_accident,
    test_two_looks_from_two_places_make_one_lasting_thing,
    test_a_rover_that_only_turned_on_the_spot_places_nothing,
    test_two_identical_chairs_are_not_guessed_at_from_two_places,
    test_one_television_seen_six_times_is_one_television,
    test_a_thing_cannot_be_seen_through_a_wall,
    test_a_thing_does_not_move_out_from_under_its_own_evidence,
    test_the_reason_survives_the_inspection_that_decided_it,
    test_a_third_look_joins_the_thing_it_points_at,
    test_appearance_cannot_overrule_where_a_thing_is,
    test_two_placed_things_on_one_bearing_are_ambiguous_not_a_guess,
    test_a_chair_and_a_ceiling_light_are_never_the_same_thing,
    test_the_right_place_is_not_enough_if_it_looks_wrong,
    test_the_evidence_survives_the_decision,
    test_one_entity_can_be_sent_to_a_console_like_the_list_can,
    test_a_placement_belongs_to_the_map_it_was_measured_in,
    test_an_inspection_settles_identity_as_well_as_measuring,
    test_a_query_that_matches_nothing_says_so,
    test_a_search_says_which_part_of_the_frame_it_found,
    test_a_flat_field_is_not_what_decides_a_match,
    test_too_little_seen_is_not_a_match_either,
    test_vectors_from_the_other_backend_are_not_ranked,
)


def main() -> int:
    for test in TESTS:
        try:
            test()
        except Exception as error:
            FAIL.append(f"{test.__name__} raised {type(error).__name__}: {error}")

    for name in PASS:
        print(f"  ok   {name}")
    for name in SKIP:
        print(f"  skip {name}")
    for name in FAIL:
        print(f"  FAIL {name}")
    print(f"\n{len(PASS)} passed, {len(FAIL)} failed, {len(SKIP)} skipped")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
