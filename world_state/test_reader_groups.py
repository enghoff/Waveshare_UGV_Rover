"""Grouping must be withdrawable and must never feed back into the resolver."""
import json
import tempfile
from unittest.mock import patch

from test_fakes import a_store
from test_harness import check
from test_merging import _rug, _split_door, _thing, _look, _count, DOOR
from world_state import merging, reader_groups


def test_preview_preserves_every_live_row_and_withdraws_conflicts():
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            _split_door(store)
            before = list(store.db.iterdump())
            grouped = reader_groups.preview(store)
            check("a preview joins the split door for review", grouped["groups"],
                  [{"representative": "object:1", "members": ["object:1", "object:2"]}])
            check("every live row, exemplar and journal is unchanged", list(store.db.iterdump()), before)
            check("the snapshot is current and converged",
                  (grouped["stale"], grouped["converged"], grouped["accepted"]), (False, True, False))
            _look(store, 1, (0.0, 0.0), DOOR, "object:2")
            _count(store)
            withdrawn = reader_groups.preview(store)
            check("new same-picture evidence withdraws the old grouping", withdrawn["groups"], [])
            check("the withdrawn grouping has a different evidence revision",
                  withdrawn["snapshot_revision"] != grouped["snapshot_revision"], True)
        finally:
            store.close()


def test_geometry_blocks_strong_appearance_before_ranking():
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            _thing(store, "object:1", 0.0, 0.0)
            _thing(store, "object:2", 1.0, 0.0)
            for row in store.db.execute("SELECT id,placement_json FROM entities").fetchall():
                placement = json.loads(row["placement_json"])
                placement.update(error_major_m=0.05, error_minor_m=0.05, extent_m=0.0)
                store.db.execute("UPDATE entities SET placement_json=? WHERE id=?",
                                 (json.dumps(placement), row["id"]))
            _look(store, 1, (-1.0, 1.0), (0.0, 0.0), "object:1")
            _look(store, 2, (-1.0, -1.0), (1.0, 0.0), "object:2")
            _count(store)
            a, b = merging._things(store, 1)
            check("the crops alone favour joining", merging.appearance(a, b) > 0, True)
            check("the ellipse gate refuses the pair", merging.geometry(a, b, 0.001), -float("inf"))
            check("no incompatible pair reaches a preview", reader_groups.preview(store)["groups"], [])
        finally:
            store.close()


def test_concurrent_preview_is_refused_and_later_retry_is_free():
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            reader_groups._preview_lock.acquire()
            try:
                check("a second preview is refused", reader_groups.preview(store)["ok"], False)
            finally:
                reader_groups._preview_lock.release()
            check("the next preview can run", reader_groups.preview(store)["ok"], True)
        finally:
            store.close()


def test_new_evidence_during_preview_marks_snapshot_stale():
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            _split_door(store)
            real_apply = merging.apply

            def concurrent_look(clone, pairs, reach=None):
                outcome = real_apply(clone, pairs, reach=reach)
                _look(store, 1, (0.0, 0.0), DOOR, "object:2")
                _count(store)
                return outcome

            with patch.object(merging, "apply", concurrent_look):
                result = reader_groups.preview(store)
            check("new evidence cannot be represented as a current preview", result["stale"], True)
            check("the concurrent real look stays in its original entity",
                  store.db.execute("SELECT entity_id FROM observations ORDER BY id DESC LIMIT 1").fetchone()[0],
                  "object:2")
        finally:
            store.close()


def test_shared_picture_constraint_survives_multi_round_groups():
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            _split_door(store)
            _thing(store, "object:4", 3.1, 0.0)
            _look(store, 1, (0.0, 0.0), DOOR, "object:4")
            _count(store)
            result = reader_groups.preview(store)
            check("chain evidence still produces a review candidate", bool(result["groups"]), True)
            check("a chain cannot join two original records sharing a picture",
                  any({"object:1", "object:4"} <= set(group["members"])
                      for group in result["groups"]), False)
        finally:
            store.close()


def test_old_recording_is_upgraded_only_in_the_disposable_copy():
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            _split_door(store)
            for table in ("merge_looks", "merge_entities", "merge_runs"):
                store.db.execute("DROP TABLE " + table)
            store.db.commit()
            before = list(store.db.iterdump())
            result = reader_groups.preview(store)
            check("an older recording can be previewed", (result["ok"], bool(result["groups"])),
                  (True, True))
            check("the older source schema remains untouched", list(store.db.iterdump()), before)
        finally:
            store.close()


def test_cofit_groups_what_the_proposer_leaves():
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            _rug(store)
            before = list(store.db.iterdump())
            grouped = reader_groups.preview(store)
            check("the rug's two records are one group, by co-fit",
                  (grouped["groups"], [e.get("rule") for e in grouped["rounds"]]),
                  ([{"representative": "object:1", "members": ["object:1", "object:2"]}],
                   ["cofit"]))
            check("...and the live store is untouched", list(store.db.iterdump()), before)
        finally:
            store.close()


TESTS = (test_cofit_groups_what_the_proposer_leaves,
         test_preview_preserves_every_live_row_and_withdraws_conflicts,
         test_geometry_blocks_strong_appearance_before_ranking,
         test_concurrent_preview_is_refused_and_later_retry_is_free,
         test_new_evidence_during_preview_marks_snapshot_stale,
         test_shared_picture_constraint_survives_multi_round_groups,
         test_old_recording_is_upgraded_only_in_the_disposable_copy)
