"""The database: identifiers, provenance, persistence and clearing.

An inspection that stores nothing says so out loud; one that stores the *wrong*
thing looks exactly like one that worked. So what is checked here is that only
the application names a row, that the provenance the rover measured is on it, and
that a database written by an older build still opens.
"""
from __future__ import annotations

import os
import sqlite3
import tempfile
import time

from test_harness import FAIL, check
from test_fakes import JPEG, a_sighting, a_store


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
)
