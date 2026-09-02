#!/usr/bin/env python3
"""Offline checks for the semantic world state. No rover, no GPU, no model.

    python world_state/selftest.py
    ssh orin 'cd ~/ugv/world_state && python3 selftest.py'

What is covered is the part where a bug is silent rather than loud. An inspection
that stores nothing says so out loud; an inspection that stores the *wrong* thing
looks exactly like one that worked. So: nothing the model says can become an
identity, the provenance the rover measured really is on every row, a database
written by an older build still opens, and every failure path leaves the world
untouched.

Everything here runs against `FakeReasoner` and a temporary directory. That is
enough to prove the store, the rules and the geometry, and nothing at all about
any real model.
"""
from __future__ import annotations

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
from world_state.contract import (                 # noqa: E402
    KINDS, extract_json, validate,
)
from world_state.inspector import Inspector        # noqa: E402
from world_state.reasoner import FakeReasoner      # noqa: E402
from world_state.store import WorldStore           # noqa: E402

#: A one-pixel JPEG. Nothing decodes it here -- the store keeps bytes and the fake
#: reasoner counts them -- but it should be a real picture, because the thing the
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


def sofa(label="grey sofa", kind="furniture"):
    return {"kind": kind, "label": label, "bbox_norm": [0.1, 0.3, 0.5, 0.9]}


def answer(*observations, scene="a living room"):
    return {"scene": scene, "observations": list(observations)}


# --- the store --------------------------------------------------------------

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
    on the label. The same chair came back as a black leather recliner and then a
    blue one on a byte-identical frame.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        stored = store.record(
            validate(answer(sofa(), sofa(label="doorway", kind="opening"))).seen,
            capture={"frame_id": "f1"})
        check("both observations are kept", stored["stored"], 2)
        check("...and no entity is created for either", stored["created"], 0)
        check("...nor claimed to be matched", stored["matched"], 0)
        check("...so the entity table stays empty", store.entities(), [])

        # The same two things again, from a second look. A label-matching store
        # would now report two matches and this is where it would show up.
        store.record(
            validate(answer(sofa(), sofa(label="doorway", kind="opening"))).seen,
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
        store.record(validate(answer(sofa())).seen,
                     capture={"frame_id": "f1", "pan": 20.0,
                              "pose": {"x_m": 1.0, "y_m": 2.0,
                                       "heading_deg": 90.0}})
        store.close()

        reopened = a_store(directory)
        rows = reopened.observations()
        check("the observation is still there after a restart",
              [row["label"] for row in rows], ["grey sofa"])
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
        store.record(validate(answer(sofa(), sofa(label="table"))).seen,
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
        store.record(validate(answer(sofa())).seen,
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
        store.record(validate(answer(sofa())).seen,
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
        store.record(validate(answer(sofa())).seen,
                     capture={"frame_id": "f1"})
        session = store.new_map_session()
        check("a map clear starts a new session", session, 2)
        check("...and deletes no observation", store.summary()["observations"], 1)
        check("...and the old one still says which map it belongs to",
              store.observations()[0]["map_session"], 1)
        store.record(validate(answer(sofa())).seen, capture={"frame_id": "f2"})
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
        store.record(validate(answer(sofa())).seen,
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
        store.record(validate(answer(sofa())).seen,
                     capture={"frame_id": "f1"})
        writer = sqlite3.connect(store.path, timeout=1.0)
        try:
            writer.execute("BEGIN IMMEDIATE")
            writer.execute("INSERT INTO entities(id, kind, label,"
                           " canonical_description, created_at, last_seen_at)"
                           " VALUES('object:99','object','x','x',1,1)")
            check("a read succeeds while a write is open",
                  [row["label"] for row in store.observations()], ["grey sofa"])
            writer.rollback()
        except sqlite3.OperationalError as error:
            FAIL.append(f"a read succeeds while a write is open: {error}")
        finally:
            writer.close()
        check("the journal really is WAL",
              store.db.execute("PRAGMA journal_mode").fetchone()[0], "wal")
        store.close()


# --- what the model is not allowed to claim ---------------------------------

def test_an_identity_the_model_volunteers_is_thrown_away() -> None:
    """Nothing asks for one, and a model that offers one anyway is not obeyed.

    The prompt no longer mentions identity and a grammar-constrained answer
    cannot contain it, but a backend that could not be constrained still can --
    and a stale build of this file, or a model with an opinion of its own, is
    exactly how a guess would creep back into being believed. So it is stripped
    at the boundary and reported, the same treatment the metres get.
    """
    volunteered = dict(sofa(), existing_entity="furniture:1", track_id=7)
    result = validate(answer(volunteered))
    check("the observation itself is still kept", len(result.seen), 1)
    check("...with no way to express an identity on it",
          hasattr(result.seen[0], "existing_entity"), False)
    check("...and the claim gone from what the model said",
          sorted(key for key in result.seen[0].raw
                 if key in ("existing_entity", "track_id")), [])
    check("...and reported rather than dropped quietly",
          "existing_entity" in result.detail(), True)


def test_the_prompt_says_nothing_about_what_has_been_seen_before() -> None:
    """Measured on the rover, twice, and it is the reason the list is gone.

    Cosmos 3 was shown a known list holding a grand piano and a fish tank,
    neither in the room, and matched the armchair to the piano and drew the fish
    tank into the scene with its description copied out of the list. The list did
    not merely fail to settle identity; it corrupted the detections. Cosmos
    Reason 2 detected *more* real things with the list gone.
    """
    from world_state.contract import build_prompt

    prompt = build_prompt()
    check("the prompt takes no world state at all",
          build_prompt.__code__.co_argcount, 0)
    # Not even a prohibition: naming the subject in order to forbid it is still
    # naming it, and what was measured is that raising previously-seen objects at
    # all changes what the model reports it can see.
    for word in ("existing_entity", "already", "seen", "recognise", "object:"):
        check(f"...and never mentions {word!r}", word in prompt, False)
    check("what it does say is to report only this picture",
          "carry anything over" in prompt, True)


def test_a_label_that_names_nothing_is_kept_and_marked() -> None:
    """"a thing" can never be matched to anything, so the row says so.

    Kept rather than refused: it is what the model said, and an observation that
    was thrown away is indistinguishable in the popup from one that was never
    made.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        result = validate(answer(sofa(label="thing"), sofa(label="red toolbox")))
        stored = store.record(result.seen, capture={"frame_id": "f1"})
        check("both are recorded as having been said", stored["stored"], 2)
        check("...and neither becomes an entity", stored["created"], 0)
        notes = {row["label"]: row["note"] for row in store.observations()}
        check("the vague one is marked as unmatchable",
              "names nothing in particular" in (notes["thing"] or ""), True)
        check("...and the concrete one carries no complaint",
              notes["red toolbox"], None)
        store.close()


# --- the model's answer -----------------------------------------------------

def test_a_good_answer_is_accepted_whole() -> None:
    result = validate(answer(sofa(), sofa(label="doorway", kind="opening")),)
    check("both observations survive", len(result.seen), 2)
    check("the scene sentence comes through", result.scene, "a living room")
    check("the kinds are the vocabulary's", [item.kind for item in result.seen],
          ["furniture", "opening"])
    check("the box is kept as fractions", result.seen[0].bbox,
          [0.1, 0.3, 0.5, 0.9])
    check("...and nothing was thrown away", result.rejected, [])


def test_the_schema_holds_the_model_to_a_length() -> None:
    """Measured on the rover: without a cap this model wrote three and a half
    thousand characters of essay into `scene` and ran out of tokens before it
    closed the object, so a perfectly good look at a room was thrown away as
    truncated. llama.cpp builds `maxLength` into the grammar, so the answer now
    ends and the object closes -- and the validator's own limits sit above the
    grammar's, so a backend that cannot constrain anything still cannot store an
    essay."""
    from world_state.contract import MAX_LABEL, MAX_SCENE, RESPONSE_SCHEMA

    properties = RESPONSE_SCHEMA["properties"]
    item = properties["observations"]["items"]["properties"]
    check("the scene sentence is capped in the grammar",
          properties["scene"]["maxLength"] <= MAX_SCENE, True)
    check("...and so is every label", item["label"]["maxLength"] <= MAX_LABEL, True)
    check("an observation is a kind, a name and a box, and nothing else",
          sorted(item), ["bbox_norm", "kind", "label"])
    check("...all three of them required",
          sorted(properties["observations"]["items"]["required"]),
          ["bbox_norm", "kind", "label"])

    essay = validate(answer(dict(sofa(), label="x" * 5000)))
    check("a backend that could not constrain it is still cut down here",
          len(essay.seen[0].label), MAX_LABEL)


def test_prose_and_fences_and_thinking_are_got_through() -> None:
    """Three things get in the way of json.loads in practice and all three are
    ordinary. The thinking is dropped here and never reaches the store."""
    payload, why = extract_json('```json\n{"scene": "x", "observations": []}\n```')
    check("a fenced block is read", payload, {"scene": "x", "observations": []})
    check("...without complaint", why, "")

    payload, _ = extract_json(
        '<think>the sofa is grey, or is it</think>{"scene":"y","observations":[]}')
    check("a reasoning block is dropped", payload,
          {"scene": "y", "observations": []})

    payload, _ = extract_json(
        'Here is what I see: {"scene":"z","observations":[]} -- hope that helps')
    check("a sentence either side is stepped over", payload,
          {"scene": "z", "observations": []})

    payload, why = extract_json("I can see a sofa and a table.")
    check("prose alone is refused", payload, None)
    check("...and says what happened",
          "no JSON object" in why, True)

    payload, why = extract_json('{"scene": "cut off", "observations": [{"lab')
    check("an answer that ran out of tokens is refused", payload, None)
    check("...and is named as that rather than as bad JSON",
          "cut off" in why, True)

    payload, why = extract_json("")
    check("an empty answer is refused", payload, None)


def test_the_answer_has_to_be_the_right_shape() -> None:
    check("a list where an object should be",
          validate([1, 2, 3]).error, "the model's answer was not an object")
    check("no observations key at all",
          validate({"scene": "x"}).error,
          "the model's answer had no observations list")
    check("observations that are not a list",
          validate({"observations": "a sofa"}).error,
          "the model's observations were not a list")
    empty = validate({"scene": "an empty room", "observations": []})
    check("nothing salient is a finding rather than a failure", empty.ok, True)
    check("...with nothing to store", empty.seen, [])

    from world_state.contract import MAX_OBSERVATIONS

    result = validate(answer(*[sofa(label=f"box {n}") for n in range(20)]))
    check("a model inventorying the room is cut off at the cap",
          len(result.seen), MAX_OBSERVATIONS)
    check("...and told on", "returned 20 observations" in result.detail(), True)


def test_a_box_is_clamped_or_dropped_but_never_believed() -> None:
    result = validate(answer(dict(sofa(), bbox_norm=[-0.4, 0.1, 1.9, 0.8])))
    check("a box hanging off the edge is pulled back onto the picture",
          result.seen[0].bbox, [0.0, 0.1, 1.0, 0.8])

    # Measured on the rover: asked for fractions, this model answers on the
    # thousand-unit grid its Qwen3-VL base was trained on about half the time.
    # Both are readable, because a picture is one unit across and a box in the
    # hundreds is therefore not a fraction.
    grid = validate(answer(dict(sofa(), bbox_norm=[200.0, 550.0, 250.0, 680.0])),)
    check("a box on the model's own thousand grid is divided down",
          grid.seen[0].bbox, [0.2, 0.55, 0.25, 0.68])
    check("...and the popup is told it happened", grid.rescaled, 1)
    check("...and said so in the diagnostics line",
          "0-1000 grid" in grid.detail(), True)

    off_grid = validate(answer(dict(sofa(), bbox_norm=[0.0, 0.0, 4096.0, 4096.0])),)
    check("a box on neither scale is dropped", off_grid.seen[0].bbox, None)

    inside_out = validate(answer(dict(sofa(), bbox_norm=[0.8, 0.1, 0.2, 0.8])),)
    check("an inside-out box is dropped", inside_out.seen[0].bbox, None)
    check("...and the observation kept, because the label is the finding",
          inside_out.seen[0].label, "grey sofa")

    for bad in ([0.1, 0.2], "left", [0.1, 0.2, "x", 0.4], [float("nan")] * 4):
        dropped = validate(answer(dict(sofa(), bbox_norm=bad)))
        check(f"a box given as {bad!r} is dropped", dropped.seen[0].bbox, None)

    # A complaint about a box costs nothing, and the counts have to say so. On the
    # rover this read as eight observations returned and two rejected where the
    # model had offered six and lost none, which is the sort of number the whole
    # experiment is later read off.
    complained = validate(answer(dict(sofa(), bbox_norm=[0.8, 0.1, 0.2, 0.8]),
                                 {"kind": "object", "bbox_norm": [0.1, 0.1, 0.2,
                                                                  0.2]}))
    check("a dropped box does not count as a refused observation",
          complained.refused, 1)
    check("...and the observation whose box went is still kept",
          len(complained.seen), 1)
    check("...while the one with no label at all is the one refused",
          "no label" in complained.detail(), True)


def test_the_model_is_not_allowed_to_measure_the_room() -> None:
    """Where the camera was is a reading the rover took; how far away the sofa is
    would be a guess from one photograph. The first is provenance, the second is
    never a property of anything here."""
    metric = dict(sofa(), distance_m=2.4, map_x=4.72, map_y=2.18, z=1.1)
    result = validate(answer(metric))
    check("the observation is still kept", len(result.seen), 1)
    check("...with the metres stripped out of the raw record",
          sorted(key for key in result.seen[0].raw
                 if key in ("distance_m", "map_x", "map_y", "z")), [])
    check("...and the stripping reported rather than done quietly",
          "distance_m" in result.detail(), True)


def test_an_unknown_kind_is_filed_rather_than_refused() -> None:
    result = validate(answer(dict(sofa(), kind="seating")))
    check("a kind outside the vocabulary becomes unknown",
          result.seen[0].kind, "unknown")
    check("...and the label, which is the content, survives",
          result.seen[0].label, "grey sofa")
    check("...and it is still stored somewhere sensible",
          result.seen[0].kind in KINDS, True)
    check("an observation with no label at all is refused",
          validate(answer({"kind": "object"})).seen, [])


# --- one inspection, end to end ---------------------------------------------

def an_inspector(directory, answers=None, fail="", capture=None, pose=None):
    store = a_store(directory)
    reasoner = FakeReasoner(answers or [], fail=fail)
    return store, reasoner, Inspector(store, reasoner,
                                      capture or a_capture(pan=20.0, tilt=-5.0),
                                      pose or a_pose())


def test_one_inspection_records_what_it_saw_and_where_it_stood() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store, reasoner, inspector = an_inspector(
            directory, [answer(sofa(), sofa(label="doorway", kind="opening"))])
        result = inspector.inspect()
        check("the inspection succeeded", result["ok"], True)
        check("...storing an observation for each thing seen", result["stored"], 2)
        check("...and claiming no identity for either", result["created"], 0)
        check("the model was given a picture and a prompt, and nothing else",
              sorted(reasoner.calls[0]), ["bytes", "prompt"])
        row = store.observations()[0]
        check("the frame it looked at was kept",
              store.frame(row["frame_id"]) == JPEG, True)
        check("...with the gimbal angles it was taken at",
              (row["observer_pan_deg"], row["observer_tilt_deg"]), (20.0, -5.0))
        check("...and the rover's pose",
              row["pose"], {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0})
        check("...and the model that answered", row["model_id"], "fake-reasoner")
        check("one line went into the diagnostics log",
              [row["status"] for row in store.inferences()], ["ok"])
        store.close()


def test_the_second_look_is_told_nothing_about_the_first() -> None:
    """The inverse of what this used to check, and the point of the change.

    The second inspection used to be handed the entities the first one created,
    so the model could say it was looking at them again. It is now asked the
    identical question, because the answer to "which thing is this" is not in the
    picture and asking for it made the picture worse.
    """
    with tempfile.TemporaryDirectory() as directory:
        store, reasoner, inspector = an_inspector(directory, [
            answer(sofa()),
            answer(sofa(label="grey three-seat sofa")),
        ])
        inspector.inspect()
        second = inspector.inspect()
        check("the second look is asked exactly what the first was",
              reasoner.calls[1]["prompt"], reasoner.calls[0]["prompt"])
        check("...and still claims nothing about identity", second["created"], 0)
        check("both looks are kept as observations",
              store.summary()["observations"], 2)
        check("...and the store has no opinion yet about whether they are one "
              "sofa or two", store.summary()["entities"], 0)
        store.close()


def test_every_failure_leaves_the_world_exactly_as_it_was() -> None:
    """Nine ways an inspection can fail, and one rule for all of them."""
    cases = [
        ("the sidecar is not running", dict(fail="nothing is listening on 8775"),
         "unavailable"),
        ("the camera gave nothing",
         dict(answers=[answer(sofa())],
              capture=a_capture(ok=False, error="the camera gave nothing")),
         "no_frame"),
        ("the model answered in prose",
         dict(answers=["I can see a sofa."]), "bad_json"),
        ("the model answered with the wrong shape",
         dict(answers=[{"scene": "x"}]), "invalid"),
        ("the model answered with nothing at all",
         dict(answers=[""]), "bad_json"),
    ]
    for name, kwargs, status in cases:
        with tempfile.TemporaryDirectory() as directory:
            store, _reasoner, inspector = an_inspector(directory, **kwargs)
            # Something already in the world, so "unchanged" is a claim with
            # content rather than "still empty".
            store.record(validate(answer(sofa(label="red toolbox"))).seen,
                         capture={"frame_id": "f0"})
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


def test_a_model_that_will_not_stop_talking_costs_one_inspection() -> None:
    """Runaway generation is the likeliest way this hangs, so the truncated answer
    has to come back as a failure with a name rather than as a parse error."""
    with tempfile.TemporaryDirectory() as directory:
        store, _reasoner, inspector = an_inspector(
            directory, ['{"scene": "a room", "observations": [{"label": "so'])
        result = inspector.inspect()
        check("a cut-off answer fails", result["ok"], False)
        check("...as a truncation rather than as nonsense",
              "cut off" in result["error"], True)
        check("...and the frame it was looking at is still there to look at",
              store.frame(store.inferences()[0]["frame_id"]) == JPEG, True)
        store.close()


def test_only_one_inspection_at_a_time() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store, _reasoner, inspector = an_inspector(directory, [answer(sofa())])
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


def test_nothing_salient_is_not_a_failure() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store, _reasoner, inspector = an_inspector(
            directory, [{"scene": "an empty corridor", "observations": []}])
        result = inspector.inspect()
        check("an empty answer succeeds", result["ok"], True)
        check("...with nothing stored", result["stored"], 0)
        check("...and the popup can tell it apart from a failure",
              store.inferences()[0]["status"], "ok")
        store.close()


def test_a_clear_is_refused_while_an_inspection_runs() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store, _reasoner, inspector = an_inspector(directory, [answer(sofa())])
        inspector.inspect()
        inspector._lock.acquire()
        try:
            check("the inspector says it is busy", inspector.busy, True)
        finally:
            inspector._lock.release()
        check("...and is free again afterwards", inspector.busy, False)
        check("the clear then works", store.clear()["ok"], True)
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


def test_the_vocabulary_is_phrases_and_not_comments() -> None:
    from world_state.perceive import read_vocabulary

    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, "words.txt")
        with open(path, "w", encoding="utf-8") as handle:
            handle.write("\n".join(["# a comment", "", "a chair",
                                    "  a wooden table  ", "", "# another", ""]))
        words = read_vocabulary(path)
        check("comments and blank lines are dropped", words,
              ["a chair", "a wooden table"])


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



# --- an inspection through the encoders --------------------------------------
#
# The path the rover actually uses now. What is worth proving offline is that
# what was *measured* survives into the database unchanged and that what was not
# measured stays empty: a bearing invented from a missing pose would be the one
# failure this whole design exists to avoid.


def a_sighting(label="a wooden chair", bbox=None, dino=None, siglip=None):
    from world_state.perception_client import Sighting

    return Sighting(bbox=bbox or [0.1, 0.3, 0.5, 0.9], label=label,
                    label_score=0.11, region_score=0.83, area=0.24,
                    dino=dino if dino is not None else b"\x01\x02\x03\x04",
                    siglip=siglip if siglip is not None else b"\x05\x06\x07\x08")


def a_seeing_inspector(directory, looks=None, fail="", capture=None, pose=None,
                       fov_deg=100.0):
    from world_state.perception_client import FakeEyes

    store = a_store(directory)
    eyes = FakeEyes(looks or [], fail=fail)
    return store, eyes, Inspector(
        store, FakeReasoner([]), capture or a_capture(pan=20.0, tilt=-5.0),
        pose or a_pose(), eyes=eyes, fov_deg=fov_deg)


def test_an_inspection_through_the_encoders_keeps_what_it_measured() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store, eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting(), a_sighting(label="a doorway")]])
        result = inspector.inspect()
        check("the inspection succeeded", result["ok"], True)
        check("...storing one observation per region", result["stored"], 2)
        check("...with a bearing for each", result["placed"], 2)
        check("...and claiming no identity for either", result["created"], 0)
        check("the language model was never asked", inspector.reasoner.calls, [])

        rows = store.db.execute(
            "SELECT * FROM observations ORDER BY id").fetchall()
        first = dict(rows[0])
        check("the appearance vector is stored as the bytes it arrived as",
              first["dino_blob"], b"\x01\x02\x03\x04")
        check("...and so is the semantic one", first["siglip_blob"],
              b"\x05\x06\x07\x08")
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


def test_a_vague_label_is_kept_and_marked_the_same_way_either_way() -> None:
    """A vocabulary can produce an unrecognisable name as easily as a model can."""
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting(label="a thing")]])
        inspector.inspect()
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("the row is kept", row["label"], "a thing")
        check("...and says it can never be matched",
              "never be matched" in (row["note"] or ""), True)
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
            directory, [[a_sighting(), a_sighting(label="a doorway")]])
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
        store.record(validate(answer(sofa())).seen, capture={"frame_id": "f9"})
        check("an observation with no bearing is not in the pool",
              len(store.unplaced()), 2)
        check("...but it is still in the history", store.summary()["observations"], 3)

        check("a bearing from another map is not comparable and is left out",
              len(store.unplaced(map_session=99)), 0)
        store.close()


def test_the_vectors_never_reach_the_wire_by_accident() -> None:
    """Raw bytes in a row that is about to be JSON is a crash, not a nuisance."""
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(directory, [[a_sighting()]])
        inspector.inspect()
        shown = store.observations()[0]
        check("the console's copy carries no raw vector",
              ("dino_blob" in shown, "siglip_blob" in shown), (False, False))
        check("...but says they are there and how big",
              (shown["dino_bytes"], shown["siglip_bytes"]), (4, 4))
        import json as _json
        _json.dumps(shown)
        check("...so the row can be sent as JSON at all", True, True)
        check("the resolver's copy keeps them",
              store.unplaced()[0]["dino_blob"], b"")
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


def observe(store, x, y, bearing, label="a wooden chair", vector=None,
            inference=None, fov_deg=100.0):
    """One look at something, from a place, along a bearing.

    The box is centred, so the bearing really is the pose's heading minus the
    gimbal's pan and nothing else -- which keeps these tests about the resolver
    rather than about `view.ray`, which has its own.
    """
    from world_state.perception_client import Sighting

    seen = [Sighting(bbox=[0.45, 0.3, 0.55, 0.9], label=label,
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
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        # One chair at (3, 3), another at about (2.7, 0.4).
        observe(store, 0.0, 0.0, 45.0, inference=1)
        observe(store, 0.0, 0.0, 8.5, inference=1)
        observe(store, 6.0, 0.0, 135.0, inference=2)
        observe(store, 6.0, 0.0, 172.9, inference=2)
        result = resolve.resolve(store)
        check("nothing was placed from two viewpoints", result["created"], 0)
        check("...and all four looks are still waiting",
              len(store.unplaced()), 4)

        # A third viewpoint separates them: a real chair is agreed by every ray
        # aimed at it, and a phantom by only the two that made it.
        observe(store, 3.0, -3.0, 90.0, inference=3)      # the chair at (3, 3)
        observe(store, 3.0, -3.0, 96.5, inference=3)      # the one at (2.7, 0.4)
        result = resolve.resolve(store)
        check("a third look from somewhere else settles it", result["created"], 2)
        places = sorted((round(one["placement"]["x_m"], 1),
                         round(one["placement"]["y_m"], 1))
                        for one in store.placed())
        check("...as two things in two places", len(places), 2)
        check("...far enough apart to be told apart",
              abs(places[0][1] - places[1][1]) > 1.5, True)
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
    """The cheapest gate, doing the one job it is allowed to do."""
    check("the same phrase is compatible with itself",
          resolve.compatible("a wooden chair", "a wooden chair"), True)
    check("...and so is another name for the same kind of thing",
          resolve.compatible("a wooden chair", "an office chair"), True)
    check("a chair is not a ceiling light",
          resolve.compatible("a wooden chair", "a ceiling light"), False)
    check("an empty label matches nothing", resolve.compatible("", "a chair"),
          False)


def test_a_thing_that_moves_is_not_matched_on_position_alone() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        bottle = a_vector(1.0, 0.0)
        observe(store, 0.0, 0.0, 45.0, label="a bottle", vector=bottle,
                inference=1)
        observe(store, 6.0, 0.0, 135.0, label="a bottle", vector=bottle,
                inference=2)
        resolve.resolve(store)
        check("a movable thing can still be placed when it looks the same",
              len(store.placed()), 1)

        # The right place, and nothing like it to look at.
        observe(store, 3.0, -1.0, 90.0, label="a bottle",
                vector=a_vector(0.0, 1.0), inference=3)
        result = resolve.resolve(store)
        check("a bottle in the right place that looks wrong is not matched",
              result["matched"], 0)
        check("...it is called ambiguous rather than guessed",
              result["ambiguous"], 1)
        check("...and the reason says the thing moves",
              "moves" in result["decisions"][0]["why"], True)
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
    near = [{"id": n, "siglip_blob": _packed(0.05 + n * 0.0002, 1.0, 0.0),
             "label": "a thing"} for n in range(40)]
    answer = search.rank(query, near)
    check("a field of near misses is ranked", len(answer["matches"]), 10)
    check("...but not believed", answer["confident"], False)
    check("...and the reason is in words a person can read",
          "nothing here matches" in answer["detail"], True)
    check("...which quotes the score and the bar it missed",
          f"{search.MATCHES:.2f}" in answer["detail"], True)

    real = near + [{"id": 99, "siglip_blob": _packed(1.0, 0.02, 0.0),
                    "label": "a spray bottle"}]
    answer = search.rank(query, real)
    check("a match that scores well is believed", answer["confident"], True)
    check("...and it is the right one", answer["matches"][0]["observation_id"], 99)
    check("...with the score behind the verdict shown",
          answer["best"] >= search.MATCHES, True)


def test_a_flat_field_is_not_what_decides_a_match() -> None:
    """The rule this replaced, kept as a test so it cannot come back by accident.

    Two searches with the same separation and different scores must get different
    answers, and two with the same score and different separations the same one.
    Measured on the rover the separation told present from absent no better than
    a coin, so it must not be able to overturn the score.
    """
    query = _packed(1.0, 0.0, 0.0)
    crowd = [{"id": n, "siglip_blob": _packed(0.02, 1.0, n * 0.01),
              "label": "a thing"} for n in range(40)]

    # A real match with nothing else near it, and the same match in a room where
    # several things score almost as well. The separation differs greatly.
    alone = search.rank(query, crowd + [
        {"id": 1, "siglip_blob": _packed(1.0, 0.05, 0.0), "label": "a bottle"}])
    among = search.rank(query, crowd + [
        {"id": 1, "siglip_blob": _packed(1.0, 0.05, 0.0), "label": "a bottle"},
        {"id": 2, "siglip_blob": _packed(1.0, 0.07, 0.0), "label": "a bottle"},
        {"id": 3, "siglip_blob": _packed(1.0, 0.09, 0.0), "label": "a bottle"}])
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
    wrong = [{"id": n, "siglip_blob": _packed(0.04, 1.0, 0.0), "label": "a chair"}
             for n in range(3)]
    answer = search.rank(query, wrong)
    check("three things that are not it is not a match", answer["confident"], False)
    check("...and it says the rover has barely looked, rather than "
          "blaming the query", "not have looked at it yet" in answer["detail"],
          True)

    right = wrong + [{"id": 9, "siglip_blob": _packed(1.0, 0.0, 0.0),
                      "label": "a bottle"}]
    answer = search.rank(query, right)
    check("but a real match among four is still a match",
          answer["confident"], True)
    check("...and does not pretend to a separation worth reading",
          "spreads above" in answer["detail"], False)


def test_vectors_from_the_other_backend_are_not_ranked() -> None:
    """Comparing across backends would rank noise, so it is refused."""
    query = _packed(1.0, 0.0, 0.0)
    rows = [{"id": 1, "siglip_blob": _packed(1.0, 0.0, 0.0),
             "vectors_from": "onnxruntime", "label": "a chair"},
            {"id": 2, "siglip_blob": _packed(0.2, 1.0, 0.0),
             "vectors_from": "tensorrt", "label": "a chair"}]
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
    test_an_identity_the_model_volunteers_is_thrown_away,
    test_the_prompt_says_nothing_about_what_has_been_seen_before,
    test_a_label_that_names_nothing_is_kept_and_marked,
    test_a_good_answer_is_accepted_whole,
    test_the_schema_holds_the_model_to_a_length,
    test_prose_and_fences_and_thinking_are_got_through,
    test_the_answer_has_to_be_the_right_shape,
    test_a_box_is_clamped_or_dropped_but_never_believed,
    test_the_model_is_not_allowed_to_measure_the_room,
    test_an_unknown_kind_is_filed_rather_than_refused,
    test_one_inspection_records_what_it_saw_and_where_it_stood,
    test_the_second_look_is_told_nothing_about_the_first,
    test_every_failure_leaves_the_world_exactly_as_it_was,
    test_a_model_that_will_not_stop_talking_costs_one_inspection,
    test_only_one_inspection_at_a_time,
    test_nothing_salient_is_not_a_failure,
    test_a_clear_is_refused_while_an_inspection_runs,
    test_an_observation_becomes_a_bearing_from_a_measured_pose,
    test_the_rays_of_one_entity_are_bounded_and_oldest_first,
    test_two_bearings_from_two_places_locate_a_thing,
    test_turning_on_the_spot_locates_nothing,
    test_two_looks_along_the_same_line_are_one_look,
    test_uncertainty_grows_when_the_baseline_shrinks,
    test_two_identical_chairs_stay_two_things,
    test_the_best_pair_places_the_thing,
    test_a_board_with_no_engines_falls_back_and_says_why,
    test_a_query_can_be_embedded_before_anything_has_been_looked_at,
    test_both_backends_answer_the_same_four_questions,
    test_the_installer_builds_exactly_the_engines_the_runtime_opens,
    test_the_vocabulary_is_phrases_and_not_comments,
    test_a_region_that_is_an_edge_is_not_a_thing,
    test_an_inspection_through_the_encoders_keeps_what_it_measured,
    test_a_bearing_is_the_pose_the_gimbal_and_the_box_together,
    test_without_a_pose_nothing_pretends_to_know_a_direction,
    test_a_sidecar_that_is_down_writes_one_row_and_no_observations,
    test_a_vague_label_is_kept_and_marked_the_same_way_either_way,
    test_a_bearing_never_runs_past_half_a_turn,
    test_the_pending_pool_holds_what_has_a_direction_but_no_home,
    test_the_vectors_never_reach_the_wire_by_accident,
    test_two_looks_from_two_places_make_one_lasting_thing,
    test_a_rover_that_only_turned_on_the_spot_places_nothing,
    test_two_identical_chairs_are_not_guessed_at_from_two_places,
    test_a_third_look_joins_the_thing_it_points_at,
    test_appearance_cannot_overrule_where_a_thing_is,
    test_two_placed_things_on_one_bearing_are_ambiguous_not_a_guess,
    test_a_chair_and_a_ceiling_light_are_never_the_same_thing,
    test_a_thing_that_moves_is_not_matched_on_position_alone,
    test_the_evidence_survives_the_decision,
    test_a_placement_belongs_to_the_map_it_was_measured_in,
    test_an_inspection_settles_identity_as_well_as_measuring,
    test_a_query_that_matches_nothing_says_so,
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
