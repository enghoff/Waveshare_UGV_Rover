#!/usr/bin/env python3
"""Offline checks for the semantic world state. No rover, no GPU, no model.

    python world_state/selftest.py
    ssh orin 'cd ~/ugv/world_state && python3 selftest.py'

What is covered is the part where a bug is silent rather than loud. An inspection
that stores nothing says so out loud; an inspection that stores the *wrong* thing
looks exactly like one that worked, and the experiment this component exists for
would then measure the association code rather than the model. So: identifiers are
the application's, an identifier the model was not shown is refused, history
survives a canonical change, and every failure path leaves the world untouched.

Everything here runs against `FakeReasoner` and a temporary directory. That is
enough to prove the store and the rules and nothing at all about Cosmos, which is
why the task this implements calls the fake development scaffolding rather than a
result.
"""
from __future__ import annotations

import json
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


def sofa(existing=None, label="grey sofa", kind="furniture"):
    return {"existing_entity": existing, "kind": kind, "label": label,
            "description": f"a {label}", "location_hint": "ahead-left",
            "bbox_norm": [0.1, 0.3, 0.5, 0.9]}


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
        check("...and nothing to show the model",
              store.known_entities(), [])
        check("...and the frames directory exists",
              os.path.isdir(os.path.join(directory, "frames")), True)
        store.close()


def test_the_application_owns_the_identifiers() -> None:
    """The model proposes; the store names.

    The identifier is allocated per kind and counted in a table rather than
    derived from the rows, so an entity deleted by hand cannot hand its name to
    something else later.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        result = validate(answer(sofa(), sofa(label="doorway", kind="opening")),
                          set())
        stored = store.record(result.seen, capture={"frame_id": "f1"})
        check("two new things become two new entities", stored["created"], 2)
        check("...named by kind and a counter the store keeps",
              stored["entities"], ["furniture:1", "opening:1"])
        again = store.record(validate(answer(sofa(label="chair")), set()).seen,
                             capture={"frame_id": "f2"})
        check("...and the counter does not restart", again["entities"],
              ["furniture:2"])
        store.close()


def test_history_survives_the_canonical_description_changing() -> None:
    """The point of separating observations from entities, in one check.

    The canonical description follows the newest observation, which is what makes
    the popup useful; every earlier one is still there, which is what makes a
    model that changed its mind visible rather than invisible.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        store.record(validate(answer(sofa()), set()).seen,
                     capture={"frame_id": "f1"})
        known = {entity["id"] for entity in store.known_entities()}
        store.record(
            validate(answer(sofa(existing="furniture:1", label="blue sofa")),
                     known).seen,
            capture={"frame_id": "f2"})
        entity = store.entity("furniture:1")
        check("the entity's current wording is the newest one",
              entity["canonical_description"], "a blue sofa")
        check("...and it has been seen twice", entity["observation_count"], 2)
        history = store.observations("furniture:1")
        check("...while the history keeps both",
              [row["label"] for row in history], ["blue sofa", "grey sofa"])
        check("...and the popup can see the wording moved",
              store.entities()[0]["distinct_labels"], 2)
        store.close()


def test_the_world_survives_the_process_that_wrote_it() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        store.record(validate(answer(sofa()), set()).seen,
                     capture={"frame_id": "f1"})
        store.close()

        reopened = a_store(directory)
        check("the entity is still there after a restart",
              [row["id"] for row in reopened.entities()], ["furniture:1"])
        check("...with its history", len(reopened.observations("furniture:1")), 1)
        check("...and the counter has not gone back to one",
              reopened.record(validate(answer(sofa(label="lamp")), set()).seen,
                              capture={"frame_id": "f2"})["entities"],
              ["furniture:2"])
        reopened.close()


def test_the_frame_is_kept_and_every_observation_points_at_it() -> None:
    """Without the picture there is no way to tell a hallucinated entity from a
    real one somebody had forgotten was in the room."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        frame_id = store.save_frame(JPEG, 640, 480)
        store.record(validate(answer(sofa(), sofa(label="table")), set()).seen,
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
        store.record(validate(answer(sofa()), set()).seen,
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
        store.record(validate(answer(sofa()), set()).seen,
                     capture={"frame_id": frame_id})
        store.record_inference(started_at=time.time(), status="ok")
        cleared = store.clear()
        check("the entities go", store.summary()["entities"], 0)
        check("...and the observations", store.summary()["observations"], 0)
        check("...and the diagnostics log", store.inferences(), [])
        check("...and the stored pictures", cleared["frames_removed"], 1)
        check("...leaving nothing behind in the directory",
              os.listdir(store.frames_dir), [])
        check("the map session is not touched by a semantic clear",
              cleared["map_session"], 1)
        check("...and identifiers start again, which is what a repeatable "
              "experiment needs",
              store.record(validate(answer(sofa()), set()).seen,
                           capture={"frame_id": "f2"})["entities"],
              ["furniture:1"])
        store.close()


def test_clearing_the_map_keeps_the_semantic_world() -> None:
    """The two clears are different operations and neither may perform the other.

    Only one direction is obvious. Clearing semantic memory must not touch SLAM --
    it cannot from here, this process does not own the map -- and clearing the map
    must not delete a room's worth of observations that are still true.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        store.record(validate(answer(sofa()), set()).seen,
                     capture={"frame_id": "f1"})
        session = store.new_map_session()
        check("a map clear starts a new session", session, 2)
        check("...and leaves the entities alone", store.summary()["entities"], 1)
        check("...and their history", len(store.observations("furniture:1")), 1)
        check("...and the old observation still says which map it belongs to",
              store.observations("furniture:1")[0]["map_session"], 1)
        known = {entity["id"] for entity in store.known_entities()}
        store.record(validate(answer(sofa(existing="furniture:1")), known).seen,
                     capture={"frame_id": "f2"})
        check("...while a new one is stamped with the new session",
              store.observations("furniture:1")[0]["map_session"], 2)
        check("...which is what the entity list reports",
              store.entities()[0]["last_map_session"], 2)
        store.close()


def test_unreadable_stored_json_cannot_take_the_viewer_down() -> None:
    """The popup exists to show what went wrong, so it must survive a row that is
    itself what went wrong."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        store.record(validate(answer(sofa()), set()).seen,
                     capture={"frame_id": "f1"})
        with store.db:
            store.db.execute("UPDATE observations SET raw_json = '{not json',"
                             " bbox_json = 'nonsense'")
        row = store.observations()[0]
        check("a corrupt raw answer comes back marked rather than raising",
              "unreadable" in (row["raw"] or {}), True)
        check("...and so does a corrupt box", "unreadable" in (row["bbox"] or {}),
              True)
        check("...and the entity list still renders",
              len(store.entities()), 1)
        store.close()


def test_a_reader_is_not_blocked_by_a_writer() -> None:
    """The console reads the world while the inspection writing it is still going.

    With the default journal that ordinary overlap is a locked database rather
    than a slightly stale read, which is the whole reason WAL is set.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        store.record(validate(answer(sofa()), set()).seen,
                     capture={"frame_id": "f1"})
        writer = sqlite3.connect(store.path, timeout=1.0)
        try:
            writer.execute("BEGIN IMMEDIATE")
            writer.execute("INSERT INTO entities(id, kind, label,"
                           " canonical_description, created_at, last_seen_at)"
                           " VALUES('object:99','object','x','x',1,1)")
            check("a read succeeds while a write is open",
                  [row["id"] for row in store.entities()], ["furniture:1"])
            writer.rollback()
        except sqlite3.OperationalError as error:
            FAIL.append(f"a read succeeds while a write is open: {error}")
        finally:
            writer.close()
        check("the journal really is WAL",
              store.db.execute("PRAGMA journal_mode").fetchone()[0], "wal")
        store.close()


# --- association ------------------------------------------------------------

def test_the_model_may_only_name_what_it_was_shown() -> None:
    """The rule the whole experiment rests on.

    An identifier the model was not given is invention or a stale memory, and
    creating an entity for it would let the model choose its own names by the back
    door -- after which "the sofa kept its identifier" would mean nothing.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        store.record(validate(answer(sofa()), set()).seen,
                     capture={"frame_id": "f1"})
        known = {"furniture:1"}

        result = validate(answer(sofa(existing="furniture:1")), known)
        check("a name it was shown is accepted", len(result.seen), 1)
        check("...as a reference rather than a new thing",
              result.seen[0].existing_entity, "furniture:1")

        invented = validate(answer(sofa(existing="furniture:77")), known)
        check("a name it was not shown is refused", invented.seen, [])
        check("...and said so, rather than silently dropped",
              "furniture:77" in invented.detail(), True)
        check("...and no entity is conjured for it",
              store.record(invented.seen, capture={"frame_id": "f2"})["created"], 0)
        check("...leaving the world exactly as it was",
              store.summary()["entities"], 1)
        store.close()


def test_a_stale_identifier_is_refused_at_the_database_too() -> None:
    """Validation checks the list; the store checks the rows. An entity deleted
    between the two is the same case as one that never existed."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        result = validate(answer(sofa(existing="furniture:1")), {"furniture:1"})
        stored = store.record(result.seen, capture={"frame_id": "f1"})
        check("an identifier with no row behind it matches nothing",
              stored["matched"], 0)
        check("...and creates nothing", stored["created"], 0)
        check("...and is counted as rejected", stored["rejected"], 1)
        check("...while the observation is still kept as history",
              store.summary()["observations"], 1)
        check("...attached to no entity, where the popup shows it",
              store.observations(unmatched=True)[0]["note"],
              "the model named an entity that is not in the store")
        store.close()


def test_two_new_observations_never_merge_two_existing_things() -> None:
    """Over-creating is the expected failure and is left visible; merging is the
    unacceptable one, because it destroys the evidence."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        store.record(validate(answer(sofa(), sofa(label="grey sofa")),
                              set()).seen, capture={"frame_id": "f1"})
        check("two identical descriptions become two entities, not one",
              [row["id"] for row in store.entities()],
              ["furniture:1", "furniture:2"])
        known = {entity["id"] for entity in store.known_entities()}
        store.record(validate(answer(sofa(), sofa(label="grey sofa")),
                              set()).seen, capture={"frame_id": "f2"})
        check("...and seeing them again with no reference makes two more",
              len(store.entities()), 4)
        check("...with the duplicates plain to see in the list",
              sorted(row["label"] for row in store.entities()),
              ["grey sofa"] * 4)
        check("...and nothing merged", len(known), 2)
        store.close()


def test_a_label_that_names_nothing_gets_no_entity() -> None:
    """"object:7 -- thing" is a row that can never be recognised again, so the
    observation is kept as history and no entity is made for it."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        result = validate(answer(sofa(label="thing"), sofa(label="red toolbox")),
                          set())
        stored = store.record(result.seen, capture={"frame_id": "f1"})
        check("the vague one creates nothing", stored["created"], 1)
        check("...and the concrete one does",
              [row["label"] for row in store.entities()], ["red toolbox"])
        check("...while both are recorded as having been said",
              store.summary()["observations"], 2)
        check("...and the popup is told why one of them has no entity",
              "concrete" in (store.observations(unmatched=True)[0]["note"] or ""),
              True)
        store.close()


# --- the model's answer -----------------------------------------------------

def test_a_good_answer_is_accepted_whole() -> None:
    result = validate(answer(sofa(), sofa(label="doorway", kind="opening")),
                      set())
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
    from world_state.contract import (
        MAX_DESCRIPTION, MAX_LABEL, MAX_SCENE, RESPONSE_SCHEMA,
    )

    properties = RESPONSE_SCHEMA["properties"]
    item = properties["observations"]["items"]["properties"]
    check("the scene sentence is capped in the grammar",
          properties["scene"]["maxLength"] <= MAX_SCENE, True)
    check("...and so is every label", item["label"]["maxLength"] <= MAX_LABEL, True)
    check("...and every description",
          item["description"]["maxLength"] <= MAX_DESCRIPTION, True)

    essay = validate(answer(dict(sofa(), description="x" * 5000)), set())
    check("a backend that could not constrain it is still cut down here",
          len(essay.seen[0].description), MAX_DESCRIPTION)


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
          validate([1, 2, 3], set()).error, "the model's answer was not an object")
    check("no observations key at all",
          validate({"scene": "x"}, set()).error,
          "the model's answer had no observations list")
    check("observations that are not a list",
          validate({"observations": "a sofa"}, set()).error,
          "the model's observations were not a list")
    empty = validate({"scene": "an empty room", "observations": []}, set())
    check("nothing salient is a finding rather than a failure", empty.ok, True)
    check("...with nothing to store", empty.seen, [])

    from world_state.contract import MAX_OBSERVATIONS

    result = validate(answer(*[sofa(label=f"box {n}") for n in range(20)]), set())
    check("a model inventorying the room is cut off at the cap",
          len(result.seen), MAX_OBSERVATIONS)
    check("...and told on", "returned 20 observations" in result.detail(), True)


def test_a_box_is_clamped_or_dropped_but_never_believed() -> None:
    result = validate(answer(dict(sofa(), bbox_norm=[-0.4, 0.1, 1.9, 0.8])), set())
    check("a box hanging off the edge is pulled back onto the picture",
          result.seen[0].bbox, [0.0, 0.1, 1.0, 0.8])

    # Measured on the rover: asked for fractions, this model answers on the
    # thousand-unit grid its Qwen3-VL base was trained on about half the time.
    # Both are readable, because a picture is one unit across and a box in the
    # hundreds is therefore not a fraction.
    grid = validate(answer(dict(sofa(), bbox_norm=[200.0, 550.0, 250.0, 680.0])),
                    set())
    check("a box on the model's own thousand grid is divided down",
          grid.seen[0].bbox, [0.2, 0.55, 0.25, 0.68])
    check("...and the popup is told it happened", grid.rescaled, 1)
    check("...and said so in the diagnostics line",
          "0-1000 grid" in grid.detail(), True)

    off_grid = validate(answer(dict(sofa(), bbox_norm=[0.0, 0.0, 4096.0, 4096.0])),
                        set())
    check("a box on neither scale is dropped", off_grid.seen[0].bbox, None)

    inside_out = validate(answer(dict(sofa(), bbox_norm=[0.8, 0.1, 0.2, 0.8])),
                          set())
    check("an inside-out box is dropped", inside_out.seen[0].bbox, None)
    check("...and the observation kept, because the label is the finding",
          inside_out.seen[0].label, "grey sofa")

    for bad in ([0.1, 0.2], "left", [0.1, 0.2, "x", 0.4], [float("nan")] * 4):
        dropped = validate(answer(dict(sofa(), bbox_norm=bad)), set())
        check(f"a box given as {bad!r} is dropped", dropped.seen[0].bbox, None)

    # A complaint about a box costs nothing, and the counts have to say so. On the
    # rover this read as eight observations returned and two rejected where the
    # model had offered six and lost none, which is the sort of number the whole
    # experiment is later read off.
    complained = validate(answer(dict(sofa(), bbox_norm=[0.8, 0.1, 0.2, 0.8]),
                                 dict(sofa(existing="furniture:99"))), set())
    check("a dropped box does not count as a refused observation",
          complained.refused, 1)
    check("...and the observation whose box went is still kept",
          len(complained.seen), 1)
    check("...while the invented identifier is the one that was refused",
          "furniture:99" in complained.detail(), True)


def test_the_model_is_not_allowed_to_measure_the_room() -> None:
    """Where the camera was is a reading the rover took; how far away the sofa is
    would be a guess from one photograph. The first is provenance, the second is
    never a property of anything here."""
    metric = dict(sofa(), distance_m=2.4, map_x=4.72, map_y=2.18, z=1.1)
    result = validate(answer(metric), set())
    check("the observation is still kept", len(result.seen), 1)
    check("...with the metres stripped out of the raw record",
          sorted(key for key in result.seen[0].raw
                 if key in ("distance_m", "map_x", "map_y", "z")), [])
    check("...and the stripping reported rather than done quietly",
          "distance_m" in result.detail(), True)


def test_an_unknown_kind_is_filed_rather_than_refused() -> None:
    result = validate(answer(dict(sofa(), kind="seating")), set())
    check("a kind outside the vocabulary becomes unknown",
          result.seen[0].kind, "unknown")
    check("...and the label, which is the content, survives",
          result.seen[0].label, "grey sofa")
    check("...and it is still stored somewhere sensible",
          result.seen[0].kind in KINDS, True)
    check("an observation with no label at all is refused",
          validate(answer({"kind": "object"}), set()).seen, [])


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
        check("...creating an entity for each new thing", result["created"], 2)
        check("...and matching nothing, there being nothing to match",
              result["matched"], 0)
        check("the model was shown no known entities on a first look",
              reasoner.calls[0]["known"], [])
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


def test_the_second_look_is_offered_the_first_look_s_entities() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store, reasoner, inspector = an_inspector(directory, [
            answer(sofa()),
            answer(sofa(existing="furniture:1", label="grey three-seat sofa")),
        ])
        inspector.inspect()
        second = inspector.inspect()
        check("the model was shown what it named last time",
              reasoner.calls[1]["known"], ["furniture:1"])
        check("...and its reference was accepted", second["matched"], 1)
        check("...without creating anything", second["created"], 0)
        check("the entity has two observations",
              store.entity("furniture:1")["observation_count"], 2)
        check("...and one identity", len(store.entities()), 1)
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
            store.record(validate(answer(sofa(label="red toolbox")), set()).seen,
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
        check("...with nothing created", result["created"], 0)
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


TESTS = (
    test_an_empty_database_is_an_ordinary_thing_to_open,
    test_the_application_owns_the_identifiers,
    test_history_survives_the_canonical_description_changing,
    test_the_world_survives_the_process_that_wrote_it,
    test_the_frame_is_kept_and_every_observation_points_at_it,
    test_a_missing_pose_is_recorded_as_missing,
    test_clearing_the_semantic_world_takes_its_frames_with_it,
    test_clearing_the_map_keeps_the_semantic_world,
    test_unreadable_stored_json_cannot_take_the_viewer_down,
    test_a_reader_is_not_blocked_by_a_writer,
    test_the_model_may_only_name_what_it_was_shown,
    test_a_stale_identifier_is_refused_at_the_database_too,
    test_two_new_observations_never_merge_two_existing_things,
    test_a_label_that_names_nothing_gets_no_entity,
    test_a_good_answer_is_accepted_whole,
    test_the_schema_holds_the_model_to_a_length,
    test_prose_and_fences_and_thinking_are_got_through,
    test_the_answer_has_to_be_the_right_shape,
    test_a_box_is_clamped_or_dropped_but_never_believed,
    test_the_model_is_not_allowed_to_measure_the_room,
    test_an_unknown_kind_is_filed_rather_than_refused,
    test_one_inspection_records_what_it_saw_and_where_it_stood,
    test_the_second_look_is_offered_the_first_look_s_entities,
    test_every_failure_leaves_the_world_exactly_as_it_was,
    test_a_model_that_will_not_stop_talking_costs_one_inspection,
    test_only_one_inspection_at_a_time,
    test_nothing_salient_is_not_a_failure,
    test_a_clear_is_refused_while_an_inspection_runs,
    test_an_observation_becomes_a_bearing_from_a_measured_pose,
    test_the_rays_of_one_entity_are_bounded_and_oldest_first,
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
