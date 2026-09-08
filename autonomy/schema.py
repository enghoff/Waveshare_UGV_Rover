"""SQLite schema for the episode record, and the one rule that shapes it.

**Nothing in this database is ever updated or deleted.** Every table is written
by INSERT and read by SELECT: there is no UPDATE, DELETE or REPLACE anywhere in
`store.py`, and `test_store.py` checks the module for one. That is a stronger
rule than "append first" and it is worth the small awkwardness it costs, because
the whole value of this component is that a record of what the rover decided
cannot be quietly improved after the fact.

Three places where the obvious design would have broken it, and what they do
instead:

**An episode does not carry its own outcome.** There is no `closed_at` column and
no `outcome` column, because filling one in later is an UPDATE. Closing an
episode appends a `closed` event like any other, and the outcome is read back off
that event. The row written when the episode opened still reads exactly as it
did.

**Deleting evidence does not touch the evidence row.** The bytes on disk go, and
a row goes into `deletions` saying when and why. "Is this picture still here" is
therefore a question with a history rather than a flag, and a replay can say *the
owner deleted this on the twelfth* instead of finding nothing and shrugging.

**There is no counters table.** The world state has one because a row deleted by
hand must not hand its name to a later thing; here nothing is ever deleted, so an
episode takes the number of its own row, chosen inside the transaction that
writes it. One less mutable row is worth more than the symmetry.

The schema grows the way the world state's does -- columns are added and never
removed or retyped -- so that a recording made by an older build still opens. See
`ADDED_COLUMNS`.
"""
from __future__ import annotations

SCHEMA_VERSION = 2

#: Columns added after the table they belong to was first created, applied every
#: time a database is opened. Adding to this is how the schema grows; nothing is
#: ever taken out of it.
#:
#: Still empty at version 2: the change from 1 to 2 added two whole tables --
#: `marks` and `pins` -- and a table arrives on an existing database by itself,
#: because `CREATE TABLE IF NOT EXISTS` runs on every open. No column has been
#: added to an existing table yet. This stays here rather than waiting for the
#: first one, because the migration that goes wrong is the one written in a hurry
#: against a rover that already holds a month of recordings; `test_store.py`
#: exercises the machinery directly for the same reason.
ADDED_COLUMNS: dict[str, dict[str, str]] = {}

SCHEMA = """
    CREATE TABLE IF NOT EXISTS meta (
        key   TEXT PRIMARY KEY,
        value TEXT NOT NULL
    );
    -- What was known when the episode opened, and nothing that happens
    -- afterwards. `world_generation` is the store its references belong to, and
    -- it is on the episode rather than only inside each reference so that a
    -- whole episode can be recognised as belonging to a world that has since
    -- been cleared without parsing anything.
    -- `id` is the episode number and appears inside `ref`; the store chooses it
    -- as one past the highest, inside the transaction that writes the row.
    CREATE TABLE IF NOT EXISTS episodes (
        id               INTEGER PRIMARY KEY,
        ref              TEXT NOT NULL UNIQUE,
        opened_at        REAL NOT NULL,
        trigger          TEXT NOT NULL,
        trigger_json     TEXT,
        world_generation TEXT NOT NULL,
        map_session      INTEGER,
        note             TEXT NOT NULL DEFAULT ''
    );
    CREATE INDEX IF NOT EXISTS episodes_by_time
        ON episodes(opened_at DESC, id DESC);
    CREATE INDEX IF NOT EXISTS episodes_by_generation
        ON episodes(world_generation);

    -- Everything that happened, in the order it happened. `seq` is allocated
    -- within the episode and is what replay walks; `id` is the order the
    -- database saw them, which is the same thing until two threads write at
    -- once and is not worth relying on.
    --
    -- `corrects` is how a record is put right without being rewritten: a later
    -- event names the earlier one it annotates, both are kept, and a reader is
    -- shown both. Null on nearly every row.
    CREATE TABLE IF NOT EXISTS events (
        id            INTEGER PRIMARY KEY AUTOINCREMENT,
        episode_id    INTEGER NOT NULL,
        seq           INTEGER NOT NULL,
        at            REAL NOT NULL,
        kind          TEXT NOT NULL,
        body_json     TEXT NOT NULL,
        refs_json     TEXT,
        evidence_json TEXT,
        corrects      INTEGER,
        UNIQUE(episode_id, seq)
    );
    CREATE INDEX IF NOT EXISTS events_by_episode
        ON events(episode_id, seq);
    CREATE INDEX IF NOT EXISTS events_by_kind
        ON events(kind, at DESC);

    -- The decision inputs, copied rather than referenced, and addressed by what
    -- they contain. Two episodes that saw the same world hold one row between
    -- them, which is what makes snapshotting every decision affordable.
    CREATE TABLE IF NOT EXISTS snapshots (
        digest    TEXT PRIMARY KEY,
        kind      TEXT NOT NULL,
        taken_at  REAL NOT NULL,
        bytes     INTEGER NOT NULL,
        body_json TEXT NOT NULL
    );

    -- Pictures and depth maps, copied out of the world state's frames directory
    -- before anything there can delete them. The file on disk is named by the
    -- digest, so the directory needs no index of its own and a half-written copy
    -- can never be mistaken for a good one.
    CREATE TABLE IF NOT EXISTS evidence (
        digest      TEXT PRIMARY KEY,
        kind        TEXT NOT NULL,
        bytes       INTEGER NOT NULL,
        stored_at   REAL NOT NULL,
        source_json TEXT
    );

    -- Evidence that has gone, and why. A row here is the only thing that
    -- distinguishes "the owner asked for this to be deleted" from "this
    -- recording is broken", and those must not read alike.
    CREATE TABLE IF NOT EXISTS deletions (
        id     INTEGER PRIMARY KEY AUTOINCREMENT,
        digest TEXT NOT NULL,
        at     REAL NOT NULL,
        why    TEXT NOT NULL,
        detail TEXT NOT NULL DEFAULT ''
    );
    CREATE INDEX IF NOT EXISTS deletions_by_digest
        ON deletions(digest);

    -- What the world state did to identity afterwards. A merge is one row; a
    -- thing taken apart into three is three rows from the same `from_ref`, which
    -- is the truthful shape -- "what is it now" has more than one answer and
    -- should say so rather than pick one.
    CREATE TABLE IF NOT EXISTS aliases (
        id       INTEGER PRIMARY KEY AUTOINCREMENT,
        at       REAL NOT NULL,
        kind     TEXT NOT NULL,
        from_ref TEXT NOT NULL,
        to_ref   TEXT NOT NULL,
        note     TEXT NOT NULL DEFAULT ''
    );
    CREATE INDEX IF NOT EXISTS aliases_by_from
        ON aliases(from_ref);

    -- Where something had got to. The recorder keeps the last look it recorded
    -- here, so that restarting it carries on rather than starting again or
    -- recording everything twice.
    --
    -- A table of appended rows rather than one row it edits, for the rule at
    -- the top of this file: the newest row of a kind is the current answer, and
    -- the ones under it are the history of where it had got to and when, which
    -- is exactly what somebody debugging a gap in a recording wants.
    CREATE TABLE IF NOT EXISTS marks (
        id    INTEGER PRIMARY KEY AUTOINCREMENT,
        at    REAL NOT NULL,
        kind  TEXT NOT NULL,
        value TEXT NOT NULL
    );
    CREATE INDEX IF NOT EXISTS marks_by_kind
        ON marks(kind, id DESC);

    -- Episodes retention must not touch. An acceptance recording is the case
    -- this exists for: the rover deleting the evidence behind the run somebody
    -- is arguing from would be the worst thing this component could do.
    --
    -- Pinning and unpinning are both rows, and the newest wins, so "it was
    -- pinned in September and released in October" is answerable.
    CREATE TABLE IF NOT EXISTS pins (
        id      INTEGER PRIMARY KEY AUTOINCREMENT,
        at      REAL NOT NULL,
        ref     TEXT NOT NULL,
        pinned  INTEGER NOT NULL,
        why     TEXT NOT NULL DEFAULT ''
    );
    CREATE INDEX IF NOT EXISTS pins_by_ref
        ON pins(ref, id DESC);
"""
