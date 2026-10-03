"""Consolidating a session by EM, and putting it back.

What matters most here is the second half. A consolidation rewrites which thing every
look belongs to, so it is only safe to try on the rover's own store if undoing it puts
back exactly what was there -- every thing row for row, every look where it was and
with the note it had -- while leaving alone what the rover has recorded since.
"""
from __future__ import annotations

import json
import math
import struct
import tempfile
import time

from test_harness import check
from test_fakes import a_store
from world_state import consolidate

WIDTH = 384


def _vector(axis, wobble=0.0):
    values = [0.0] * WIDTH
    values[axis] = 1.0
    values[(axis + 1) % WIDTH] = wobble
    return struct.pack(f"<{WIDTH}f", *values)


def _thing(store, entity_id, x_m, y_m, axis, born=None):
    placement = {"x_m": x_m, "y_m": y_m, "uncertainty_m": 0.2, "error_major_m": 0.2,
                 "error_minor_m": 0.1, "error_major_deg": 0.0, "extent_m": 0.3}
    born = time.time() - 100.0 if born is None else born
    store.db.execute(
        "INSERT INTO entities(id, kind, label, canonical_description, created_at,"
        " last_seen_at, observation_count, placement_json, placement_uncertainty_m,"
        " placement_map_session, placement_updated_at, exemplars)"
        " VALUES(?, 'object', '', '', ?, ?, 0, ?, 0.2, 1, ?, ?)",
        (entity_id, born, born, json.dumps(placement), born, _vector(axis)))


def _look(store, look, at, target, entity_id=None, axis=0, off_deg=0.0,
          observed_at=None):
    (px, py), (tx, ty) = at, target
    bearing = math.degrees(math.atan2(ty - py, tx - px)) + off_deg
    return store.db.execute(
        "INSERT INTO observations(entity_id, inference_id, observed_at, source,"
        " frame_id, bbox_json, observer_pose_json, map_session, bearing_deg, span_deg,"
        " dino_blob, vectors_from, note) VALUES(?,?,?,?,?,?,?,1,?,5.0,?,'tensorrt',?)",
        (entity_id, look, time.time() - 90.0 + look if observed_at is None
         else observed_at, "test", f"frame-{look}", "[0.4, 0.4, 0.6, 0.6]",
         json.dumps({"x_m": px, "y_m": py, "heading_deg": 0.0}), bearing,
         _vector(axis, 0.01 * look), f"note {look}")).lastrowid


DOOR = (3.0, 0.0)
SHELF = (0.0, 4.0)
PLACES = ((0.0, 0.0), (0.0, 1.5), (0.5, -1.5), (1.0, 2.0))


def _split_door(store):
    """The door as the rover had it: two things 0.4 m apart that never shared a look,
    and a shelf seen in two of the same pictures."""
    _thing(store, "object:1", 3.0, 0.2, 0)
    _thing(store, "object:2", 3.0, -0.2, 0)
    _thing(store, "object:3", *SHELF, 7)
    for look, at in enumerate(PLACES, start=1):
        _look(store, look, at, DOOR, "object:1" if look <= 2 else "object:2")
    _look(store, 2, PLACES[1], SHELF, "object:3", axis=7)
    _look(store, 4, PLACES[3], SHELF, "object:3", axis=7)
    store.db.execute("UPDATE entities SET observation_count = (SELECT COUNT(*) FROM"
                     " observations WHERE entity_id = entities.id)")
    store.db.commit()


def _picture(store):
    """Everything a rollback must restore, as comparable values."""
    things = sorted(tuple(row) for row in store.db.execute(
        "SELECT * FROM entities ORDER BY id"))
    looks = sorted(tuple(row) for row in store.db.execute(
        "SELECT id, entity_id, note FROM observations ORDER BY id"))
    return things, looks


def test_the_split_door_is_joined_and_put_back_exactly() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        _split_door(store)
        before = _picture(store)
        worked = consolidate.plan(store)
        told = worked.summary()
        check("a plan merges the door's two halves",
              (told["things_before"], told["things_after"], len(told["merges"])),
              (3, 2, 1))
        check("...and writes nothing", _picture(store), before)

        done = consolidate.apply(store, worked)
        door = [one for one in ("object:1", "object:2")
                if store.db.execute("SELECT 1 FROM entities WHERE id = ?",
                                    (one,)).fetchone()]
        check("applied, one thing holds all four looks at the door",
              (len(door), store.db.execute(
                  "SELECT observation_count FROM entities WHERE id = ?",
                  (door[0],)).fetchone()[0]), (1, 4))
        check("...and the shelf keeps its two",
              store.db.execute("SELECT COUNT(*) FROM observations WHERE entity_id ="
                               " 'object:3'").fetchone()[0], 2)
        moved = store.db.execute(
            "SELECT note FROM observations WHERE note LIKE 'moved from%'").fetchall()
        check("a moved look says it was moved, and by which run",
              (len(moved), f"consolidation {done['run']}" in moved[0][0]), (2, True))

        undone = consolidate.rollback(store)
        check("rolled back, every thing and look is as it was",
              (undone["ok"], _picture(store) == before), (True, True))
        check("...and it cannot be rolled back twice",
              consolidate.rollback(store)["ok"], False)
        store.close()


def test_a_look_recorded_since_stays_where_the_resolver_put_it() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        _split_door(store)
        consolidate.apply(store, consolidate.plan(store))
        kept = store.db.execute("SELECT id FROM entities WHERE id IN"
                                " ('object:1', 'object:2')").fetchone()[0]
        later = _look(store, 9, (0.2, 0.0), DOOR, kept, observed_at=time.time() + 1.0)
        store.db.commit()
        consolidate.rollback(store)
        check("the later look is still on the door it joined",
              store.db.execute("SELECT entity_id FROM observations WHERE id = ?",
                               (later,)).fetchone()[0], kept)
        check("...and that thing's count includes it",
              store.db.execute("SELECT observation_count FROM entities WHERE id = ?",
                               (kept,)).fetchone()[0], 3)
        store.close()


def test_a_thing_founded_since_from_a_let_go_look_goes() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        _split_door(store)
        # A look attached to the shelf that points well away from it.
        stray = _look(store, 5, PLACES[0], SHELF, "object:3", axis=7, off_deg=30.0)
        store.db.commit()
        consolidate.apply(store, consolidate.plan(store))
        check("the stray look is let go",
              store.db.execute("SELECT entity_id FROM observations WHERE id = ?",
                               (stray,)).fetchone()[0], None)
        # What the resolver would do next: found something from it.
        _thing(store, "object:9", -1.0, 4.0, 7, born=time.time() + 1.0)
        store.db.execute("UPDATE observations SET entity_id = 'object:9' WHERE id = ?",
                         (stray,))
        store.db.commit()
        undone = consolidate.rollback(store)
        check("rolled back, the look is the shelf's again and the newcomer is gone",
              (store.db.execute("SELECT entity_id FROM observations WHERE id = ?",
                                (stray,)).fetchone()[0],
               undone["founded_since_and_emptied"]), ("object:3", ["object:9"]))
        store.close()


def test_it_refuses_rather_than_guess() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        _split_door(store)
        check("nothing applied, nothing to roll back",
              consolidate.rollback(store)["ok"], False)
        worked = consolidate.plan(store)
        _thing(store, "object:4", 1.0, 1.0, 9)
        store.db.commit()
        check("a plan made before the things changed is not applied",
              (consolidate.apply(store, worked)["ok"],
               store.db.execute("SELECT COUNT(*) FROM consolidations").fetchone()[0]),
              (False, 0))
        consolidate.apply(store, consolidate.plan(store))
        store.clear()
        check("after a clear there is nothing to put back",
              consolidate.rollback(store)["ok"], False)
        store.close()


TESTS = (test_the_split_door_is_joined_and_put_back_exactly,
         test_a_look_recorded_since_stays_where_the_resolver_put_it,
         test_a_thing_founded_since_from_a_let_go_look_goes,
         test_it_refuses_rather_than_guess)
