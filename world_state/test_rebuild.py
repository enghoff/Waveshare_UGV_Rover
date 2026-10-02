"""The stored looks read again under their outlines, and the things built again from them.

`rebuild` is what the store is put through after the way a look is measured changes,
so what it must not do matters as much as what it does: a range the rover dropped for
turning stays dropped, nothing is written without being asked, and a rebuild that was
interrupted half way is picked up rather than leaving looks nobody can see.
"""
from __future__ import annotations

import tempfile

from test_harness import check
from test_fakes import a_capture, a_pose, a_sighting, a_store
from test_outline import BOX, _DepthOf, a_painting_behind_a_chair, the_painting_alone
from world_state import outline, rebuild
from world_state.depth_client import TURNING


def _np():
    import numpy
    return numpy


def _painted(np):
    sighting = a_sighting(bbox=list(BOX))
    sighting.outline = outline.encode(np, the_painting_alone(np), BOX)
    return sighting


def _looked(directory, np, looks=1):
    """A store holding `looks` looks at the painting behind the chair, read live."""
    from world_state.inspector import Inspector
    from world_state.perception_client import FakeEyes

    store = a_store(directory)
    ranger = _DepthOf(a_painting_behind_a_chair(np))
    for index in range(looks):
        sighting, other = _painted(np), a_sighting(bbox=[0.05, 0.2, 0.15, 0.4])
        Inspector(store, FakeEyes([[sighting, other]]), a_capture(),
                  a_pose(x=1.0 + 0.5 * index, heading=90.0), fov_deg=100.0,
                  ranger=ranger).inspect(fresh=True)
    return store


def test_an_old_look_is_read_again_under_its_outline() -> None:
    from world_state.perception_client import FakeEyes

    np = _np()
    with tempfile.TemporaryDirectory() as directory:
        store = _looked(directory, np)
        row_id = store.db.execute("SELECT id FROM observations WHERE range_from = 'outline'"
                                  ).fetchone()[0]
        # As every look before 2026-10-02 was written: the service's box, and no outline.
        store.db.execute("UPDATE observations SET range_m = 1.2, range_sigma_m = 0.05,"
                         " range_from = 'service', outline_blob = NULL WHERE id = ?", (row_id,))
        store.db.commit()
        dry = rebuild.rerange(np, store, eyes=FakeEyes([[_painted(np)]]))
        check("a dry run counts the change", (dry["changed"], dry["outlines_drawn"]), (1, 1))
        check("...and writes nothing",
              store.db.execute("SELECT range_m FROM observations WHERE id = ?",
                               (row_id,)).fetchone()[0], 1.2)
        rebuild.rerange(np, store, eyes=FakeEyes([[_painted(np)]]), write=True)
        new, method, kept = store.db.execute(
            "SELECT range_m, range_from, outline_blob FROM observations WHERE id = ?",
            (row_id,)).fetchone()
        check("written, the painting is read at the painting",
              (abs(new - 2.5) < 0.1, method, kept is not None), (True, "outline", True))

        store.db.execute("UPDATE observations SET range_m = NULL, range_sigma_m = NULL,"
                         " range_absent = ?, range_from = NULL WHERE id = ?", (TURNING, row_id))
        store.db.commit()
        rebuild.rerange(np, store, write=True)
        check("a range dropped for turning stays dropped",
              tuple(store.db.execute("SELECT range_m, range_absent FROM observations"
                                     " WHERE id = ?", (row_id,)).fetchone()), (None, TURNING))
        store.close()


def _things(store):
    return sorted((row[0], round(row[1], 6)) for row in store.db.execute(
        "SELECT id, placement_uncertainty_m FROM entities WHERE placement_json IS NOT NULL"))


def test_the_things_are_built_again_with_new_names() -> None:
    np = _np()
    with tempfile.TemporaryDirectory() as directory:
        store = _looked(directory, np, looks=3)
        before = _things(store)
        check("the looks made something to rebuild", bool(before), True)
        done = rebuild.rebuild(store)
        after = _things(store)
        check("as many things as before", (done["things_after"], len(after)),
              (len(before), len(before)))
        check("...placed as well as before",
              sorted(one[1] for one in after), sorted(one[1] for one in before))
        check("...under names never used before",
              {one[0] for one in after} & {one[0] for one in before}, set())
        check("and no look is left held back",
              store.db.execute("SELECT COUNT(*) FROM observations WHERE map_session < 0"
                               ).fetchone()[0], 0)

        # Interrupted half way: the things gone and the looks still held back.
        session = store.map_session()
        store.db.execute("DELETE FROM entities")
        store.db.execute("UPDATE observations SET entity_id = NULL, map_session = ?",
                         (-1 - session,))
        store.db.commit()
        again = rebuild.rebuild(store)
        check("an interrupted rebuild is picked up, not lost",
              (again["things_after"], again["looks"] > 0), (len(before), True))
        store.close()


def test_a_backup_is_a_whole_copy() -> None:
    import sqlite3

    np = _np()
    with tempfile.TemporaryDirectory() as directory:
        store = _looked(directory, np)
        path = rebuild.backup(store)
        copy = sqlite3.connect(path)
        check("the copy holds every look",
              copy.execute("SELECT COUNT(*) FROM observations").fetchone()[0],
              store.db.execute("SELECT COUNT(*) FROM observations").fetchone()[0])
        copy.close()
        store.close()


TESTS = (test_an_old_look_is_read_again_under_its_outline,
         test_the_things_are_built_again_with_new_names,
         test_a_backup_is_a_whole_copy)
