"""Proposing, joining and putting back things the resolver split.

What matters most is the last part. Joining two things rewrites which thing their looks
belong to, so it is only safe to try on the rover's own store if undoing it puts back
exactly what was there -- both things row for row, every look where it was and with the
note it had -- while leaving alone what the rover has recorded since.
"""
from __future__ import annotations

import json
import math
import struct
import tempfile
import time

from test_harness import check
from test_fakes import a_store
from world_state import merging

PLAIN, SEMANTIC = 384, 768


def _vector(width, axis, wobble=0.0):
    values = [0.0] * width
    values[axis] = 1.0
    values[(axis + 1) % width] = wobble
    return struct.pack(f"<{width}f", *values)


def _thing(store, entity_id, x_m, y_m):
    placement = {"x_m": x_m, "y_m": y_m, "uncertainty_m": 0.2, "error_major_m": 0.2,
                 "error_minor_m": 0.1, "error_major_deg": 0.0, "extent_m": 0.3}
    born = time.time() - 100.0
    store.db.execute(
        "INSERT INTO entities(id, kind, label, canonical_description, created_at,"
        " last_seen_at, observation_count, placement_json, placement_uncertainty_m,"
        " placement_map_session, placement_updated_at, exemplars)"
        " VALUES(?, 'object', '', '', ?, ?, 0, ?, 0.2, 1, ?, ?)",
        (entity_id, born, born, json.dumps(placement), born, _vector(PLAIN, 0)))


def _look(store, look, at, target, entity_id, axis=0, backend="tensorrt",
          observed_at=None):
    (px, py), (tx, ty) = at, target
    bearing = math.degrees(math.atan2(ty - py, tx - px))
    return store.db.execute(
        "INSERT INTO observations(entity_id, inference_id, observed_at, source,"
        " frame_id, bbox_json, observer_pose_json, map_session, bearing_deg, span_deg,"
        " dino_blob, dino_alone_blob, siglip_blob, vectors_from, note)"
        " VALUES(?,?,?,?,?,?,?,1,?,5.0,?,?,?,?,?)",
        (entity_id, look, time.time() - 90.0 + look if observed_at is None
         else observed_at, "test", f"frame-{look}", "[0.4, 0.4, 0.6, 0.6]",
         json.dumps({"x_m": px, "y_m": py, "heading_deg": 0.0}), bearing,
         _vector(PLAIN, axis, 0.01 * look), _vector(PLAIN, axis, 0.02 * look),
         _vector(SEMANTIC, axis, 0.01 * look), backend, f"note {look}")).lastrowid


DOOR = (3.0, 0.0)
SHELF = (0.0, 4.0)
PLACES = ((0.0, 0.0), (0.0, 1.5), (0.5, -1.5), (1.0, 2.0))


def _split_door(store):
    """The door as the rover had it: two things 0.4 m apart that never shared a look,
    and a shelf seen in two of the same pictures."""
    _thing(store, "object:1", 3.0, 0.2)
    _thing(store, "object:2", 3.0, -0.2)
    _thing(store, "object:3", *SHELF)
    for look, at in enumerate(PLACES, start=1):
        _look(store, look, at, DOOR, "object:1" if look <= 2 else "object:2")
    _look(store, 2, PLACES[1], SHELF, "object:3", axis=7)
    _look(store, 4, PLACES[3], SHELF, "object:3", axis=7)
    _count(store)


def _count(store):
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


def test_the_split_door_is_proposed_joined_and_put_back_exactly() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        _split_door(store)
        before = _picture(store)
        asked = merging.propose(store)
        check("the door's two halves are proposed, and nothing else",
              [(one["keep"], one["gone"]) for one in asked["proposals"]],
              [("object:1", "object:2")])
        check("...with the looks to judge them by",
              sorted(asked["proposals"][0]["shown"]), ["object:1", "object:2"])
        check("...and asking writes nothing", _picture(store), before)

        done = merging.apply(store, [["object:1", "object:2"]])
        check("joined, one thing holds all four looks at the door",
              (done["ok"], done["run"], store.db.execute(
                  "SELECT observation_count FROM entities WHERE id = 'object:1'"
              ).fetchone()[0], store.db.execute(
                  "SELECT COUNT(*) FROM entities WHERE id = 'object:2'").fetchone()[0]),
              (True, 1, 4, 0))
        moved = store.db.execute(
            "SELECT note FROM observations WHERE note LIKE 'joined to%'").fetchall()
        check("a moved look says where it came from and which run moved it",
              (len(moved), "from object:2 by merge 1" in moved[0][0]), (2, True))
        check("...the shelf keeps its two",
              store.db.execute("SELECT COUNT(*) FROM observations WHERE entity_id ="
                               " 'object:3'").fetchone()[0], 2)
        check("...and the run is listed",
              [(run["id"], run["pairs"]) for run in merging.runs(store)],
              [(1, [["object:1", "object:2"]])])
        check("asked again, there is nothing left to propose",
              merging.propose(store)["proposals"], [])

        undone = merging.rollback(store)
        check("rolled back, every thing and look is as it was",
              (undone["ok"], undone["things_restored"], undone["looks_moved_back"],
               _picture(store) == before), (True, 2, 2, True))
        check("...and it cannot be rolled back twice",
              merging.rollback(store)["ok"], False)
        store.close()


def test_a_look_recorded_since_stays_where_the_resolver_put_it() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        _split_door(store)
        merging.apply(store, [["object:1", "object:2"]])
        later = _look(store, 9, (0.2, 0.0), DOOR, "object:1",
                      observed_at=time.time() + 1.0)
        store.db.commit()
        merging.rollback(store)
        check("the later look is still on the door it joined",
              store.db.execute("SELECT entity_id FROM observations WHERE id = ?",
                               (later,)).fetchone()[0], "object:1")
        check("...and that thing's count includes it",
              store.db.execute("SELECT observation_count FROM entities WHERE id ="
                               " 'object:1'").fetchone()[0], 3)
        check("...while the other half has its own two back",
              store.db.execute("SELECT observation_count FROM entities WHERE id ="
                               " 'object:2'").fetchone()[0], 2)
        store.close()


def test_it_refuses_rather_than_guess() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        _split_door(store)
        before = _picture(store)
        check("nothing applied, nothing to roll back",
              merging.rollback(store)["ok"], False)
        refused = merging.apply(store, [["object:1", "object:3"]])
        check("two things with a region in one picture are not joined",
              (refused["ok"], "two things" in refused["refused"][0]["why"],
               _picture(store) == before,
               store.db.execute("SELECT COUNT(*) FROM merge_runs").fetchone()[0]),
              (False, True, True, 0))
        check("a thing that is not there is not joined",
              merging.apply(store, [["object:1", "object:9"]])["ok"], False)
        check("no pairs is not an instruction",
              merging.apply(store, [])["ok"], False)
        partly = merging.apply(store, [["object:1", "object:2"], ["object:2", "object:3"]])
        check("a pair reusing a thing already joined in the run is refused, the rest done",
              (partly["ok"], len(partly["joined"]), len(partly["refused"])), (True, 1, 1))
        store.clear()
        check("after a clear there is nothing to put back",
              merging.rollback(store)["ok"], False)
        store.close()


def test_proposals_never_reuse_a_thing_or_mix_backends() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        _split_door(store)
        # A third piece of the door, and a fourth whose crops came from the other
        # perception backend and so cannot be compared with the rest.
        _thing(store, "object:4", 3.1, 0.0)
        _look(store, 5, (0.0, -1.0), DOOR, "object:4")
        _thing(store, "object:5", 2.9, 0.0)
        _look(store, 6, (0.5, 1.0), DOOR, "object:5", backend="cpu")
        _count(store)
        proposals = merging.propose(store)["proposals"]
        named = [name for one in proposals for name in (one["keep"], one["gone"])]
        check("no thing is in two proposals", len(named), len(set(named)))
        check("...and the other backend's thing is in none", "object:5" in named, False)
        store.close()


def _rug(store, shared_picture=False):
    """The rug of 2026-10-08 in miniature: two records 0.2 m apart whose looks
    point at the same place, the second's seen from the other side, so that its
    masked crop and its SigLIP vector look like something else."""
    _thing(store, "object:1", 3.0, 0.1)
    _thing(store, "object:2", 3.0, -0.1)
    for look, at in enumerate(PLACES[:2], start=1):
        _look(store, look, at, DOOR, "object:1")
    for look, at in enumerate(PLACES[2:], start=3):
        oid = _look(store, look, at, DOOR, "object:2")
        store.db.execute("UPDATE observations SET dino_alone_blob=?, siglip_blob=?"
                         " WHERE id=?", (_vector(PLAIN, 9), _vector(SEMANTIC, 9), oid))
    if shared_picture:
        _look(store, 1, PLACES[0], (3.0, 0.4), "object:2")
    _count(store)


def test_cofit_joins_what_appearance_between_records_refuses() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        _rug(store)
        a, b = merging._things(store, 1)
        check("record to record, the two sides of the rug look like two things",
              merging.appearance(a, b) < merging.LOOKS_ALIKE_ABOVE, True)
        check("...so the proposer offers nothing", merging.propose(store)["proposals"], [])
        pairs = merging.cofit_pairs(store)
        check("but each one's looks would have been filed to the other",
              [(one["keep"], one["gone"], one["cofit"]) for one in pairs],
              [("object:1", "object:2", 1.0)])
        store.close()


def test_cofit_never_joins_two_regions_of_one_picture() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        _rug(store, shared_picture=True)
        check("two records with a region each in one picture are two things",
              merging.cofit_pairs(store), [])
        store.close()


TESTS = (test_the_split_door_is_proposed_joined_and_put_back_exactly,
         test_a_look_recorded_since_stays_where_the_resolver_put_it,
         test_it_refuses_rather_than_guess,
         test_proposals_never_reuse_a_thing_or_mix_backends,
         test_cofit_joins_what_appearance_between_records_refuses,
         test_cofit_never_joins_two_regions_of_one_picture)
