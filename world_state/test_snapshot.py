"""The store copied out to a snapshot and back: rows, pictures and names.

What a restore gets wrong without anybody noticing is the part checked here. A
picture deleted from the store after a snapshot must still be in the snapshot; a
picture the snapshot never had must not survive the restore; and a name handed
out after the snapshot must never be handed out again to something else, which
is the rule `autonomy/refs.py` rests on.
"""
from __future__ import annotations

import contextlib
import os
import sqlite3
import tempfile

from test_harness import check
from test_fakes import JPEG, a_sighting, a_store

from world_state import snapshot


def _look(store, frame_id):
    store.record([a_sighting()], capture={"frame_id": frame_id})


def test_a_snapshot_keeps_the_rows_and_shares_the_pictures() -> None:
    with tempfile.TemporaryDirectory() as root:
        store = a_store(os.path.join(root, "world"))
        kept = os.path.join(root, "kept")
        first = store.save_frame(JPEG)
        with open(store.depth_path(first), "wb") as handle:
            handle.write(b"depth")
        _look(store, first)
        thing = store.create_entity()
        store.place(thing, {"x_m": 1.0, "y_m": 2.0, "uncertainty_m": 0.2},
                    store.map_session())

        out = snapshot.copy_out(store, kept)
        check("a snapshot counts what it kept",
              (out["entities"], out["placed"], out["observations"], out["frames"]),
              (1, 1, 1, 2))
        check("...sharing the pictures rather than copying them",
              (out["linked"], out["copied"], out["lost"]), (2, 0, 0))
        check("...so a kept picture is the store's own file",
              os.path.samefile(store.frame_path(first),
                               os.path.join(kept, "frames", first + ".jpg")), True)
        with contextlib.closing(sqlite3.connect(os.path.join(kept, "world.db"))) as copy:
            mode = copy.execute("PRAGMA journal_mode").fetchone()[0]
        check("...and the copy is one file, not a write-ahead log", mode, "delete")
        check("...and it reads without anything written beside it",
              snapshot.counts(os.path.join(kept, "world.db"))["entities"], 1)

        # The store moves on: a clear deletes the pictures, and new looks arrive.
        store.clear()
        check("a clear of the store leaves the snapshot's pictures alone",
              os.path.exists(os.path.join(kept, "frames", first + ".jpg")), True)
        later = store.save_frame(JPEG)
        _look(store, later)

        back = snapshot.copy_in(store, kept)
        check("restoring puts the rows back",
              (back["entities"], back["observations"]), (1, 1))
        check("...and the pictures the snapshot had",
              os.path.exists(store.frame_path(first))
              and os.path.exists(store.depth_path(first)), True)
        check("...and none it did not", os.path.exists(store.frame_path(later)),
              False)
        check("...counting both", (back["linked"], back["removed"]), (2, 1))
        check("the store is still written ahead",
              store.db.execute("PRAGMA journal_mode").fetchone()[0], "wal")
        check("...and still takes a look",
              store.record([a_sighting()], capture={"frame_id": first})["stored"], 1)
        store.close()


def test_a_name_handed_out_after_a_snapshot_is_never_handed_out_again() -> None:
    """The same filling of the store keeps the higher counters; a cleared one
    gets a new generation, so no reference minted before resolves to a stranger."""
    with tempfile.TemporaryDirectory() as root:
        store = a_store(os.path.join(root, "world"))
        kept = os.path.join(root, "kept")
        store.allocate("object")
        generation = store.generation()
        snapshot.copy_out(store, kept)
        check("a name after the snapshot", store.allocate("object"), "object:2")

        back = snapshot.copy_in(store, kept)
        check("restoring the same filling keeps its generation",
              (back["new_generation"], store.generation()), (False, generation))
        check("...and never issues that name again", store.allocate("object"),
              "object:3")

        store.clear()
        back = snapshot.copy_in(store, kept)
        check("restoring over a cleared store mints a new generation",
              back["new_generation"], True)
        check("...that is neither the cleared one nor the snapshot's",
              store.generation() != generation, True)
        store.close()


def test_a_snapshot_with_no_database_changes_nothing() -> None:
    with tempfile.TemporaryDirectory() as root:
        store = a_store(os.path.join(root, "world"))
        _look(store, store.save_frame(JPEG))
        try:
            snapshot.copy_in(store, os.path.join(root, "nothing"))
            refused = False
        except FileNotFoundError:
            refused = True
        check("a snapshot with no database is refused", refused, True)
        check("...before anything is touched",
              store.summary()["observations"], 1)
        store.close()


TESTS = (
    test_a_snapshot_keeps_the_rows_and_shares_the_pictures,
    test_a_name_handed_out_after_a_snapshot_is_never_handed_out_again,
    test_a_snapshot_with_no_database_changes_nothing,
)
