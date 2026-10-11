"""The world store copied out to a snapshot, and copied back in from one.

A snapshot (rover_daemon/rover_snapshots.py) keeps the rover's state at one
moment so that it can be had back later. This is the semantic world's half --
the database and the pictures behind it. The map the positions in it were
measured in is kept beside it by the navigation stack (ros_nav/nav_snapshot.py),
and the two only mean anything together.

## The database

sqlite's own backup in both directions, never a file copy: the store keeps a
write-ahead log beside it, and the file alone can be hours stale. The copy is
switched out of WAL so that it is one self-contained file a reader can open
without writing beside it. Copying back in is the same backup with the ends
swapped, into the store's own connection, so everything already holding the
store -- the inspector, the resolver, the daemon's handlers -- goes on holding
it and simply finds other rows there. `_create` then runs over the result,
because a snapshot taken by an older build lacks whatever columns were added
since, and every insert naming one would fail.

## The pictures

Shared rather than copied. A frame is written once under a name nothing else
will ever have (`save_frame`: the second it was taken and six random hex
digits), the depth map beside it likewise, and nothing rewrites either in place
-- the console's frame cache relies on the same fact. So a snapshot holds a hard
link to each, which costs a directory entry instead of a hundred-odd kilobytes,
and taking one is a second rather than minutes of copying gigabytes. Deleting
from either side -- a clear of the live store, or deleting the snapshot -- leaves
the other's copy where it is. Where the filesystem will not link, it copies.

**If anything ever rewrites a frame in place, every snapshot sharing it changes
with it.** That is the price of the above, and write-once is what keeps it free.

## The names

Restoring puts back the identifier counters as they were when the snapshot was
taken, and a name is only safe while nothing hands it out twice -- an episode
may hold `object:140` from after the snapshot (autonomy/refs.py). So:

- if the live store is the same filling as the snapshot -- the same generation,
  no clear in between -- the counters keep whichever is higher, so a name issued
  after the snapshot is never issued again to something else, and a reference to
  a thing made after the snapshot finds nothing, which is the truth;
- otherwise the restored store gets a new generation, and every reference to the
  old one stops resolving -- those to things the snapshot does hold included --
  rather than risk resolving to a stranger, the trade `refs.py` already made.
"""
from __future__ import annotations

import contextlib
import os
import shutil
import sqlite3
import time
from typing import Any

from .store import new_generation

#: The database and the pictures inside a snapshot's directory.
DATABASE = "world.db"
FRAMES = "frames"
#: What counts as a picture in the store's frame directory: the frame, and the
#: depth map kept beside it.
PICTURES = (".jpg", ".depth.gz")


def _pictures(directory: str) -> set[str]:
    try:
        names = os.listdir(directory)
    except FileNotFoundError:
        return set()
    return {name for name in names if name.endswith(PICTURES)}


def _share(source: str, target: str, names) -> tuple[int, int, int]:
    """Link each picture from `source` into `target`, copying where linking fails.

    Answers `(linked, copied, lost)`: lost is a picture that went missing between
    being listed and being linked, which the caller's holding the looks off makes
    a fault rather than a race.
    """
    linked = copied = lost = 0
    for name in sorted(names):
        here, there = os.path.join(source, name), os.path.join(target, name)
        try:
            os.link(here, there)
            linked += 1
            continue
        except FileNotFoundError:
            lost += 1
            continue
        except OSError:
            pass
        try:
            shutil.copy2(here, there)
            copied += 1
        except OSError:
            lost += 1
    return linked, copied, lost


def counts(path: str) -> dict[str, Any]:
    """What a snapshot's database holds, read without writing anything."""
    uri = "file:" + os.path.abspath(path).replace("\\", "/") + "?mode=ro"
    with contextlib.closing(sqlite3.connect(uri, uri=True)) as db:
        meta = dict(db.execute("SELECT key, value FROM meta").fetchall())
        try:
            session = int(meta.get("map_session") or 1)
        except ValueError:
            session = 1

        def number(query: str, *args: Any) -> int:
            return int(db.execute(query, args).fetchone()[0])

        return {
            "entities": number("SELECT COUNT(*) FROM entities"),
            # Placed on the map the store was following, which is what the
            # console's map draws: the rest belong to maps that had gone already.
            "placed": number("SELECT COUNT(*) FROM entities WHERE placement_json "
                             "IS NOT NULL AND placement_map_session = ?", session),
            "observations": number("SELECT COUNT(*) FROM observations"),
            "map_session": session,
            "map_id": meta.get("map_id", ""),
            "generation": meta.get("generation", ""),
        }


def copy_out(store, directory: str) -> dict[str, Any]:
    """Copy the store into `directory`: the database, and every picture shared.

    The caller holds the rover's looks off (`Inspector.not_looking`), so that no
    look is half written while this runs and no picture listed here is deleted
    before it is linked. The rows and the list of pictures are both taken under
    the store's own lock, so they describe the same moment.
    """
    began = time.monotonic()
    frames = os.path.join(directory, FRAMES)
    os.makedirs(frames, exist_ok=True)
    path = os.path.join(directory, DATABASE)
    target = sqlite3.connect(path)
    try:
        with store._lock:
            store.db.backup(target)
            names = _pictures(store.frames_dir)
        target.execute("PRAGMA journal_mode=DELETE")
    finally:
        target.close()
    linked, copied, lost = _share(store.frames_dir, frames, names)
    return {**counts(path), "frames": linked + copied, "linked": linked,
            "copied": copied, "lost": lost, "bytes": os.path.getsize(path),
            "took_s": round(time.monotonic() - began, 2)}


def copy_in(store, directory: str) -> dict[str, Any]:
    """Make the store what `directory` holds: its rows, and exactly its pictures.

    Raises `FileNotFoundError` for a snapshot with no database, before anything
    is touched. The caller holds the rover's looks off for the reason
    `copy_out` gives. Pictures the snapshot does not have are deleted from the
    store's directory and the ones it has are linked back in; a picture already
    there under the same name is the same picture, so it is left alone.
    """
    began = time.monotonic()
    path = os.path.join(directory, DATABASE)
    if not os.path.exists(path):
        raise FileNotFoundError(f"the snapshot has no {DATABASE}")
    shared = os.path.join(directory, FRAMES)
    wanted = _pictures(shared)
    uri = "file:" + os.path.abspath(path).replace("\\", "/") + "?mode=ro"
    with store._lock:
        was = store.generation()
        issued = {str(row[0]): int(row[1]) for row in
                  store.db.execute("SELECT kind, next FROM counters").fetchall()}
        source = sqlite3.connect(uri, uri=True)
        try:
            source.backup(store.db)
        finally:
            source.close()
        store.db.execute("PRAGMA journal_mode=WAL")
        store._create()
        renamed = store.generation() != was
        with store.db:
            if renamed:
                store.db.execute("REPLACE INTO meta(key, value) "
                                 "VALUES('generation', ?)", (new_generation(),))
            else:
                for kind, after in issued.items():
                    row = store.db.execute("SELECT next FROM counters WHERE kind = ?",
                                           (kind,)).fetchone()
                    if row is None or int(row[0]) < after:
                        store.db.execute("REPLACE INTO counters(kind, next) "
                                         "VALUES(?, ?)", (kind, after))
        present = _pictures(store.frames_dir)
        removed = 0
        for name in present - wanted:
            try:
                os.remove(os.path.join(store.frames_dir, name))
                removed += 1
            except OSError:
                pass
        linked, copied, lost = _share(shared, store.frames_dir, wanted - present)
    summary = store.summary()
    return {"entities": summary["entities"],
            "observations": summary["observations"],
            "map_session": summary["map_session"],
            "frames": len(wanted) - lost, "linked": linked, "copied": copied,
            "lost": lost, "removed": removed,
            # Whether references minted against the store before this one stop
            # resolving: see "The names" above.
            "new_generation": renamed,
            "took_s": round(time.monotonic() - began, 2)}
