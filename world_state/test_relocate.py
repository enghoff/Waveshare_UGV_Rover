"""A record that follows its thing when the thing is moved (relocate.py).

What matters most is the refusals and the undo. Moving a record rewrites which thing
the arrival's looks belong to, so it must refuse whenever another record could as
well be the thing that came, and putting a move back must leave exactly what was
there.
"""
from __future__ import annotations

import json
import math
import struct
import tempfile
import time

from test_harness import check
from test_fakes import a_store
from world_state import relocate, resolve

WIDTH = 384
NOW = time.time()


def section(title):
    print()
    print(title)


def _vector(axis, wobble=0.0):
    values = [0.0] * WIDTH
    values[axis] = 1.0
    values[(axis + 1) % WIDTH] = wobble
    return struct.pack(f"<{WIDTH}f", *values)


def _thing(store, entity_id, x_m, y_m, axis=0, born=NOW - 5000.0):
    placement = {"x_m": x_m, "y_m": y_m, "uncertainty_m": 0.2, "error_major_m": 0.2,
                 "error_minor_m": 0.1, "error_major_deg": 0.0, "extent_m": 0.5,
                 "height_m": 0.2, "viewpoints": 3}
    store.db.execute(
        "INSERT INTO entities(id, kind, label, canonical_description, created_at,"
        " last_seen_at, observation_count, placement_json, placement_uncertainty_m,"
        " placement_map_session, placement_updated_at, exemplars, exemplars_alone)"
        " VALUES(?, 'object', '', '', ?, ?, 0, ?, 0.2, 1, ?, ?, ?)",
        (entity_id, born, born, json.dumps(placement), born, _vector(axis), _vector(axis)))
    store.db.commit()


LOOKS = iter(range(1, 10000))


def _look(store, entity_id, at, target, axis=0, observed_at=NOW - 3000.0):
    (px, py), (tx, ty) = at, target
    look = next(LOOKS)
    bearing = math.degrees(math.atan2(ty - py, tx - px))
    oid = store.db.execute(
        "INSERT INTO observations(entity_id, inference_id, observed_at, source, frame_id,"
        " bbox_json, observer_pose_json, map_session, bearing_deg, span_deg, dino_blob,"
        " dino_alone_blob, note) VALUES(?,?,?,?,?,?,?,1,?,5.0,?,?,?)",
        (entity_id, look, observed_at, "test", f"frame-{look}", "[0.4, 0.4, 0.6, 0.6]",
         json.dumps({"x_m": px, "y_m": py, "heading_deg": 0.0}), bearing,
         _vector(axis, 0.01), _vector(axis, 0.02), f"note {look}")).lastrowid
    store.db.commit()
    return oid


def _count(store):
    store.db.execute("UPDATE entities SET observation_count = (SELECT COUNT(*) FROM"
                     " observations WHERE entity_id = entities.id)")
    store.db.commit()


def _armchair(store, arrival_on=None, arrival_axis=0):
    """An armchair at (0, 0), seen twice there, then moved to (2, 0) and seen
    twice there. Returns the moved event."""
    _thing(store, "object:1", 0.0, 0.0)
    old = [_look(store, "object:1", (0.0, -2.0), (0.0, 0.0)),
           _look(store, "object:1", (-2.0, 0.0), (0.0, 0.0))]
    new = [_look(store, arrival_on, (2.0, -2.0), (2.0, 0.0), axis=arrival_axis,
                 observed_at=NOW - 60.0),
           _look(store, arrival_on, (4.0, 0.0), (2.0, 0.0), axis=arrival_axis,
                 observed_at=NOW - 30.0)]
    _count(store)
    return {"from": [0.1, 0.0], "to": [2.0, 0.0], "regions": new, "remembered": old}


def _placed(store, entity_id):
    row = store.db.execute("SELECT placement_json FROM entities WHERE id = ?",
                           (entity_id,)).fetchone()
    return json.loads(row[0]) if row and row[0] else None


def test_moves_the_record():
    section("relocate: the record follows its thing")
    store = a_store(tempfile.mkdtemp())
    event = _armchair(store)
    got = relocate.relocate(store, event, 1, now=NOW)
    check("the record that held the old place moves", (got["done"], got["entity_id"]),
          ("moved", "object:1"))
    where = _placed(store, "object:1")
    check("...placed at the arrival", (where["x_m"], where["y_m"]), (2.0, 0.0))
    check("...keeping what the thing is", (where["extent_m"], where["height_m"]), (0.5, 0.2))
    check("...and where it came from", where["moved_from"], [0.1, 0.0])
    filed = [r[0] for r in store.db.execute(
        "SELECT entity_id FROM observations WHERE id IN (?, ?)", event["regions"])]
    check("the arrival's looks are filed under it", filed, ["object:1", "object:1"])
    check("its place is refitted from the arrival's first look on",
          store.moved_since("object:1", 1), NOW - 60.0)
    check("the move is journalled", store.db.execute(
        "SELECT COUNT(*) FROM move_looks WHERE move_id = ?", (got["move_id"],)).fetchone()[0], 2)


def test_refuses():
    section("relocate: when it refuses")
    store = a_store(tempfile.mkdtemp())
    event = _armchair(store)
    _thing(store, "object:2", 2.3, 0.0)
    _look(store, "object:2", (2.3, -2.0), (2.3, 0.0))
    _count(store)
    got = relocate.relocate(store, event, 1, now=NOW)
    check("another look-alike, filed before and nearer the arrival, is the likelier",
          (got["done"], "object:2" in got["why"]), ("abstained", True))
    check("...and nothing changed", (_placed(store, "object:1")["x_m"],
                                     store.moved_since("object:1", 1)), (0.0, None))

    store = a_store(tempfile.mkdtemp())
    event = _armchair(store)
    _thing(store, "object:2", 2.3, 0.0, axis=40)
    got = relocate.relocate(store, event, 1, now=NOW)
    check("a nearer thing that looks different is no rival", got["done"], "moved")

    store = a_store(tempfile.mkdtemp())
    event = _armchair(store)
    _thing(store, "object:2", 2.3, 0.0, born=NOW - 70.0)
    _look(store, "object:2", (2.3, -2.0), (2.3, 0.0), observed_at=NOW - 70.0)
    _count(store)
    got = relocate.relocate(store, event, 1, now=NOW)
    check("nor is one made in the moment of the move", got["done"], "moved")

    store = a_store(tempfile.mkdtemp())
    event = dict(_armchair(store), **{"from": [3.0, 3.0]})
    got = relocate.relocate(store, event, 1, now=NOW)
    check("a record placed away from the departure is not the thing that left",
          got["done"], "abstained")

    store = a_store(tempfile.mkdtemp())
    event = _armchair(store, arrival_axis=9)
    got = relocate.relocate(store, event, 1, now=NOW)
    check("an arrival that looks like something else is not it", got["done"], "abstained")

    store = a_store(tempfile.mkdtemp())
    event = _armchair(store)
    store.db.execute("UPDATE observations SET entity_id = NULL WHERE id IN (?, ?)",
                     event["remembered"])
    store.db.commit()
    got = relocate.relocate(store, event, 1, now=NOW)
    check("nor does an old place whose pictures are on no record", got["done"], "abstained")
    check("a malformed event is an error", relocate.relocate(store, {"from": [0, 0]}, 1)["ok"],
          False)


def test_folds_and_undoes():
    section("relocate: a record made of the arrival is folded in, and undo puts it all back")
    store = a_store(tempfile.mkdtemp())
    _thing(store, "object:9", 2.0, 0.0, born=NOW - 60.0)
    event = _armchair(store, arrival_on="object:9")
    before = {r[0]: r[1] for r in store.db.execute("SELECT id, entity_id FROM observations")}
    got = relocate.relocate(store, event, 1, now=NOW)
    check("the arrival's own record is folded into the moved one", got["folded"], ["object:9"])
    check("...and is gone", store.db.execute(
        "SELECT COUNT(*) FROM entities WHERE id = 'object:9'").fetchone()[0], 0)
    undone = relocate.undo(store, got["move_id"])
    check("undo answers", undone["ok"], True)
    after = {r[0]: r[1] for r in store.db.execute("SELECT id, entity_id FROM observations")}
    check("every look is back where it was", after, before)
    check("the folded record is back", store.db.execute(
        "SELECT observation_count FROM entities WHERE id = 'object:9'").fetchone()[0], 2)
    check("the record is back in its old place", _placed(store, "object:1")["x_m"], 0.0)
    check("its place is refitted from all its looks again", store.moved_since("object:1", 1),
          None)
    check("a move is put back once", relocate.undo(store, got["move_id"])["ok"], False)


def test_refit_since_the_move():
    section("resolve._replace_placement: a moved record is not pulled back by its old looks")
    store = a_store(tempfile.mkdtemp())
    event = _armchair(store)
    resolve._replace_placement(store, "object:1", 1)
    check("before the move its two crossing looks place it", round(
        _placed(store, "object:1")["x_m"], 1), 0.0)
    relocate.relocate(store, event, 1, now=NOW)
    resolve._replace_placement(store, "object:1", 1)
    where = _placed(store, "object:1")
    check("after it, a refit uses the arrival's looks only and lands at the arrival",
          (round(where["x_m"], 1), round(where["y_m"], 1)), (2.0, 0.0))


TESTS = (test_moves_the_record, test_refuses, test_folds_and_undoes, test_refit_since_the_move)

if __name__ == "__main__":
    for test in TESTS:
        test()
