"""The episode record: SQLite plus a directory of evidence named by its contents.

Runtime data lives outside the deploy tree, at `~/.ugv/autonomy`, for the same
reason the world state's does -- a deploy replaces the code and must never be
able to replace the recording.

**Every method here either inserts a row or reads one.** There is no UPDATE, no
DELETE and no REPLACE in this file, and `test_store.py` reads the module to check
it. The only destructive act in the component is removing a file of evidence from
disk, and that leaves a row in `deletions` behind it.

The two ideas worth knowing before reading the methods:

**Snapshot, do not look up.** `snapshot` copies the decision inputs into the
database and hands back a digest. An episode records that digest; nothing about
replaying it ever asks the world state a question. That is what makes a
reconstruction independent of everything that happens to the world afterwards,
and it is why `events.decision` refuses anything but a digest for its inputs.

**Evidence is named by what is in it.** `keep_evidence` hashes the bytes, writes
them under that name and returns `sha256:...`. Twenty episodes that looked at the
same picture hold one copy between them, the name survives a world-state clear,
and a half-written copy can never be mistaken for a good one because the file is
put in place with a rename.
"""
from __future__ import annotations

import json
import os
import sqlite3
import tempfile
import threading
import time
from typing import Any, Iterable

import events as events_mod
import refs as refs_mod
from schema import ADDED_COLUMNS, SCHEMA, SCHEMA_VERSION

ENV_DIR = "UGV_AUTONOMY_DIR"

#: How evidence a replay wants may be missing, in the order a reader cares
#: about. `deleted` is the one that must never be confused with the others: it
#: means somebody asked, not that the recording is broken.
HELD, DELETED, ABSENT = "held", "deleted", "absent"


def autonomy_dir() -> str:
    return os.environ.get(ENV_DIR) or os.path.expanduser("~/.ugv/autonomy")


class EpisodeStore:
    """Episodes, the events in them, and the evidence they stand on.

    One connection guarded by one lock, and WAL set, for the reasons the world
    state's store gives: the daemon serves each client on its own thread, and the
    way results get looked at on a rover is the sqlite3 command line over ssh
    while the thing is still running.
    """

    def __init__(self, directory: str | None = None) -> None:
        self.dir = directory or autonomy_dir()
        self.evidence_dir = os.path.join(self.dir, "evidence")
        os.makedirs(self.evidence_dir, exist_ok=True)
        self.path = os.path.join(self.dir, "episodes.db")
        self._lock = threading.RLock()
        self.db = sqlite3.connect(self.path, check_same_thread=False, timeout=5.0)
        self.db.row_factory = sqlite3.Row
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA busy_timeout=5000")
        self._create()

    def close(self) -> None:
        with self._lock:
            self.db.close()

    # --- schema ---------------------------------------------------------------

    def _create(self) -> None:
        """Bring an empty or existing database up to the current schema.

        Opening a database that does not exist is the ordinary path rather than a
        special case: the rover's first episode after a new computer, and every
        test in `selftest.py`, both start here.
        """
        with self._lock, self.db:
            self.db.executescript(SCHEMA)
            for table, columns in ADDED_COLUMNS.items():
                self._add_columns(table, columns)
            self.db.execute(
                "INSERT OR IGNORE INTO meta(key, value) VALUES('schema_version', ?)",
                (str(SCHEMA_VERSION),))
            # This store's own generation, minted once and never again. An
            # episode reference that outlives its database -- in somebody's
            # notes, or in a progress entry -- should not come back pointing at
            # a different episode in a database that was started again.
            self.db.execute(
                "INSERT OR IGNORE INTO meta(key, value) VALUES('generation', ?)",
                (refs_mod.new_generation(),))
            self.db.execute(
                "INSERT OR IGNORE INTO meta(key, value) VALUES('created_at', ?)",
                (str(time.time()),))

    def _add_columns(self, table: str, columns: dict[str, str]) -> None:
        """Add columns a later version wants to a table an earlier one created.

        `CREATE TABLE IF NOT EXISTS` does nothing to a table that already exists,
        so a database written by an older build would go on without the new
        column and every insert naming it would fail. This is the migration, and
        it is deliberately the only kind there is: columns are added and never
        removed or retyped, so a rover's recorded episodes survive every change
        to what gets recorded next.
        """
        have = {row["name"] for row in self.db.execute(f"PRAGMA table_info({table})")}
        for name, declaration in columns.items():
            if name not in have:
                self.db.execute(f"ALTER TABLE {table} ADD COLUMN {name} "
                                f"{declaration}")

    # --- meta -----------------------------------------------------------------

    def _meta(self, key: str, default: str = "") -> str:
        with self._lock:
            row = self.db.execute("SELECT value FROM meta WHERE key = ?",
                                  (key,)).fetchone()
        return default if row is None else row["value"]

    def generation(self) -> str:
        return self._meta("generation", refs_mod.UNKNOWN)

    def schema_version(self) -> int:
        try:
            return int(self._meta("schema_version", "0"))
        except ValueError:
            return 0

    # --- episodes -------------------------------------------------------------

    def open_episode(self, trigger: str, *, world_generation: str | None = None,
                     map_session: int | None = None, detail: Any = None,
                     note: str = "", at: float | None = None) -> str:
        """Start an episode and return its durable reference.

        `world_generation` is the world store this episode's references will
        belong to, read off whatever the world state last reported with
        `refs.generation_of`. **A caller that does not know it should pass
        nothing**, which records `unknown` and marks every world reference in the
        episode permanently unresolvable. That is the honest state while the
        world store has no generation to report, and it is not a placeholder to
        be filled in later -- filling it in later would be a guess about which
        store the episode was talking to.
        """
        stamp = time.time() if at is None else at
        generation = world_generation or refs_mod.UNKNOWN
        if generation != refs_mod.UNKNOWN and not refs_mod.GENERATION.match(generation):
            raise ValueError(f"not a generation token: {generation!r}")
        with self._lock, self.db:
            row = self.db.execute(
                "SELECT IFNULL(MAX(id), 0) + 1 AS next FROM episodes").fetchone()
            number = int(row["next"])
            ref = refs_mod.episode(self.generation(), number)
            self.db.execute(
                "INSERT INTO episodes(id, ref, opened_at, trigger, trigger_json,"
                " world_generation, map_session, note)"
                " VALUES(?, ?, ?, ?, ?, ?, ?, ?)",
                (number, ref, stamp, trigger,
                 None if detail is None else _dump(detail),
                 generation, map_session, note))
        return ref

    def append(self, episode_ref: str, event: events_mod.Event) -> int:
        """Add one event to an episode and return its sequence number.

        Appending to a closed episode is allowed and is how an annotation
        arrives: something noticed a fortnight later is still worth recording,
        and refusing it would only push it somewhere that is not the record. What
        the reader is shown is that it came after the close -- see
        `replay.reconstruct`.
        """
        events_mod.validate(event.kind, event.body)
        with self._lock, self.db:
            episode_id = self._episode_id(episode_ref)
            if event.corrects is not None and not self._has_seq(episode_id,
                                                               event.corrects):
                raise ValueError(f"{episode_ref} has no event {event.corrects} "
                                 f"to correct")
            row = self.db.execute(
                "SELECT IFNULL(MAX(seq), 0) + 1 AS next FROM events"
                " WHERE episode_id = ?", (episode_id,)).fetchone()
            seq = int(row["next"])
            self.db.execute(
                "INSERT INTO events(episode_id, seq, at, kind, body_json,"
                " refs_json, evidence_json, corrects)"
                " VALUES(?, ?, ?, ?, ?, ?, ?, ?)",
                (episode_id, seq, time.time(), event.kind, _dump(event.body),
                 _dump(list(event.refs)) if event.refs else None,
                 _dump(list(event.evidence)) if event.evidence else None,
                 event.corrects))
        return seq

    def close_episode(self, episode_ref: str, outcome: str, *, detail: str = "",
                      success: bool | None = None) -> int:
        """End an episode by appending the event its outcome is read from.

        Raises if it is already closed. Two closes would mean two answers to
        "how did this go", and the code that produced them has a bug worth
        hearing about rather than a second record worth keeping.
        """
        with self._lock:
            if self.outcome(episode_ref) is not None:
                raise ValueError(f"{episode_ref} is already closed")
            body: dict[str, Any] = {"outcome": outcome}
            if detail:
                body["detail"] = detail
            if success is not None:
                body["success"] = success
            return self.append(episode_ref, events_mod.make(events_mod.CLOSED, body))

    def outcome(self, episode_ref: str) -> dict[str, Any] | None:
        """How the episode ended, or None while it is still open."""
        with self._lock:
            episode_id = self._episode_id(episode_ref)
            row = self.db.execute(
                "SELECT body_json FROM events WHERE episode_id = ? AND kind = ?"
                " ORDER BY seq LIMIT 1",
                (episode_id, events_mod.CLOSED)).fetchone()
        return None if row is None else _load(row["body_json"])

    def episode(self, episode_ref: str) -> dict[str, Any]:
        """One episode and every event in it, in the order they happened."""
        with self._lock:
            row = self.db.execute("SELECT * FROM episodes WHERE ref = ?",
                                  (episode_ref,)).fetchone()
            if row is None:
                raise KeyError(episode_ref)
            rows = self.db.execute(
                "SELECT * FROM events WHERE episode_id = ? ORDER BY seq",
                (row["id"],)).fetchall()
        return {**_episode_row(row), "events": [_event_row(one) for one in rows]}

    def episodes(self, *, limit: int = 50, before: float | None = None,
                 world_generation: str | None = None) -> list[dict[str, Any]]:
        """Episodes newest first, without their events."""
        where, params = [], []
        if before is not None:
            where.append("opened_at < ?")
            params.append(before)
        if world_generation is not None:
            where.append("world_generation = ?")
            params.append(world_generation)
        clause = (" WHERE " + " AND ".join(where)) if where else ""
        with self._lock:
            rows = self.db.execute(
                f"SELECT * FROM episodes{clause} ORDER BY opened_at DESC, id DESC"
                f" LIMIT ?", (*params, limit)).fetchall()
        return [_episode_row(one) for one in rows]

    # --- snapshots ------------------------------------------------------------

    def snapshot(self, kind: str, body: Any, *, at: float | None = None) -> str:
        """Copy what a decision was made from into the record, and name it.

        Addressed by content, so recording the same world twice costs one row.
        The digest covers the body only and not the time it was taken, which is
        the point: two decisions made from an unchanged world should agree that
        it was unchanged.
        """
        payload = _dump(body).encode()
        digest = refs_mod.digest(payload)
        with self._lock, self.db:
            self.db.execute(
                "INSERT OR IGNORE INTO snapshots(digest, kind, taken_at, bytes,"
                " body_json) VALUES(?, ?, ?, ?, ?)",
                (digest, kind, time.time() if at is None else at, len(payload),
                 payload.decode()))
        return digest

    def snapshot_body(self, digest: str) -> Any | None:
        with self._lock:
            row = self.db.execute(
                "SELECT body_json FROM snapshots WHERE digest = ?",
                (digest,)).fetchone()
        return None if row is None else _load(row["body_json"])

    # --- evidence -------------------------------------------------------------

    def keep_evidence(self, kind: str, data: bytes, *,
                      source: dict[str, Any] | None = None) -> str:
        """Take a copy of a picture or a depth map before anything can delete it.

        Called with the bytes rather than a path on purpose. The caller is
        usually reading a frame out of the world state's frames directory, which
        a clear can empty at any moment, and a store that took a path would be
        recording a promise instead of a picture.
        """
        digest = refs_mod.digest(data)
        target = self._evidence_path(digest)
        if not os.path.exists(target):
            os.makedirs(os.path.dirname(target), exist_ok=True)
            handle, temporary = tempfile.mkstemp(dir=os.path.dirname(target))
            try:
                with os.fdopen(handle, "wb") as out:
                    out.write(data)
                # Renamed into place rather than written in place, so that a
                # process killed halfway through leaves no file named after a
                # digest it does not have.
                os.replace(temporary, target)
            except BaseException:
                _remove(temporary)
                raise
        with self._lock, self.db:
            self.db.execute(
                "INSERT OR IGNORE INTO evidence(digest, kind, bytes, stored_at,"
                " source_json) VALUES(?, ?, ?, ?, ?)",
                (digest, kind, len(data), time.time(),
                 None if source is None else _dump(source)))
        return digest

    def evidence_bytes(self, digest: str) -> bytes | None:
        path = self._evidence_path(digest)
        try:
            with open(path, "rb") as handle:
                return handle.read()
        except OSError:
            return None

    def evidence_state(self, digest: str) -> dict[str, Any]:
        """Whether this evidence is still here, and if not, which kind of gone.

        Three answers rather than two, and the distinction is the requirement:
        evidence the owner deleted must never read like evidence that was never
        recorded, or like a recording that has come apart.

        Asking about something that is not a digest raises rather than answering
        `absent`, because the caller that passes a path has a bug and "no such
        evidence" would hide it.
        """
        self._evidence_path(digest)
        with self._lock:
            row = self.db.execute("SELECT * FROM evidence WHERE digest = ?",
                                  (digest,)).fetchone()
            gone = self.db.execute(
                "SELECT * FROM deletions WHERE digest = ? ORDER BY at DESC"
                " LIMIT 1", (digest,)).fetchone()
        if gone is not None:
            return {"digest": digest, "state": DELETED, "at": gone["at"],
                    "why": gone["why"], "detail": gone["detail"]}
        if row is None:
            return {"digest": digest, "state": ABSENT}
        if not os.path.exists(self._evidence_path(digest)):
            # Recorded, not deleted by anybody, and not on disk. Something took
            # it away behind the store's back, and saying so is more use than
            # calling it absent.
            return {"digest": digest, "state": ABSENT, "why": "file is missing",
                    "bytes": row["bytes"], "kind": row["kind"]}
        return {"digest": digest, "state": HELD, "bytes": row["bytes"],
                "kind": row["kind"], "stored_at": row["stored_at"]}

    def delete_evidence(self, digest: str, why: str, *, detail: str = "") -> dict:
        """Remove the bytes and write down that they were removed.

        The only destructive operation in the component. `why` is what a reader
        of an unreplayable episode will be shown, so it should say who asked --
        "the owner asked for this to be deleted" and "retention expired at
        thirty days" call for quite different reactions.
        """
        if not why:
            raise ValueError("deleting evidence needs a reason")
        _remove(self._evidence_path(digest))
        with self._lock, self.db:
            self.db.execute(
                "INSERT INTO deletions(digest, at, why, detail)"
                " VALUES(?, ?, ?, ?)", (digest, time.time(), why, detail))
        return self.evidence_state(digest)

    def _evidence_path(self, digest: str) -> str:
        parsed = refs_mod.parse(digest)
        if parsed is None or not parsed.is_digest:
            raise ValueError(f"not an evidence digest: {digest!r}")
        return os.path.join(self.evidence_dir, parsed.local[:2], parsed.local)

    # --- what the world state did to identity afterwards -----------------------

    def alias(self, kind: str, from_ref: str, to_ref: str, *,
              note: str = "", at: float | None = None) -> None:
        """Record that a thing became another thing, or several.

        A merge is one row. A thing taken apart into three is three rows sharing
        a `from_ref`, because "what is it now" genuinely has three answers and
        picking one would be inventing a fact.

        This never rewrites an episode. An episode said what it said, and what
        happened to identity afterwards is a separate record that a reader is
        shown alongside it.
        """
        if kind not in ("merge", "split"):
            raise ValueError(f"alias kind must be merge or split, not {kind!r}")
        for ref in (from_ref, to_ref):
            if refs_mod.parse(ref) is None:
                raise ValueError(f"not a reference: {ref!r}")
        with self._lock, self.db:
            self.db.execute(
                "INSERT INTO aliases(at, kind, from_ref, to_ref, note)"
                " VALUES(?, ?, ?, ?, ?)",
                (time.time() if at is None else at, kind, from_ref, to_ref, note))

    def now_called(self, ref: str, *, depth: int = 8) -> list[str]:
        """Follow the aliases from a reference to whatever it is called today.

        Returns the reference itself when nothing has happened to it, and more
        than one when it was taken apart. `depth` stops a cycle -- two things
        merged into each other by two different passes is a bug, and looping
        for ever is a worse way to find out about it than returning what was
        reached.
        """
        seen, frontier, out = {ref}, [ref], []
        for _ in range(depth):
            if not frontier:
                break
            following = []
            for one in frontier:
                with self._lock:
                    rows = self.db.execute(
                        "SELECT to_ref FROM aliases WHERE from_ref = ?"
                        " ORDER BY at, id", (one,)).fetchall()
                fresh = [row["to_ref"] for row in rows if row["to_ref"] not in seen]
                # Nowhere new to go is the end of the chain, whether that is
                # because nothing ever happened to this thing or because the
                # only place it leads is somewhere already visited. The second
                # is a cycle -- two passes that merged two things into each
                # other -- and returning where it closed is more use than
                # returning nothing.
                if not fresh:
                    out.append(one)
                    continue
                for ref in fresh:
                    seen.add(ref)
                    following.append(ref)
            frontier = following
        return out + frontier

    # --- reading --------------------------------------------------------------

    def summary(self) -> dict[str, Any]:
        with self._lock:
            counts = {
                name: self.db.execute(
                    f"SELECT COUNT(*) AS n FROM {name}").fetchone()["n"]
                for name in ("episodes", "events", "snapshots", "evidence",
                             "deletions", "aliases")
            }
            held = self.db.execute(
                "SELECT IFNULL(SUM(bytes), 0) AS n FROM evidence").fetchone()["n"]
            open_now = self.db.execute(
                "SELECT COUNT(*) AS n FROM episodes e WHERE NOT EXISTS"
                " (SELECT 1 FROM events v WHERE v.episode_id = e.id"
                "   AND v.kind = ?)", (events_mod.CLOSED,)).fetchone()["n"]
        return {**counts, "open": open_now, "evidence_bytes": held,
                "generation": self.generation(), "dir": self.dir,
                "schema_version": self.schema_version()}

    def _episode_id(self, episode_ref: str) -> int:
        row = self.db.execute("SELECT id FROM episodes WHERE ref = ?",
                              (episode_ref,)).fetchone()
        if row is None:
            raise KeyError(episode_ref)
        return int(row["id"])

    def _has_seq(self, episode_id: int, seq: int) -> bool:
        return self.db.execute(
            "SELECT 1 FROM events WHERE episode_id = ? AND seq = ?",
            (episode_id, seq)).fetchone() is not None


def _episode_row(row: sqlite3.Row) -> dict[str, Any]:
    return {"ref": row["ref"], "number": row["id"], "opened_at": row["opened_at"],
            "trigger": row["trigger"], "trigger_detail": _load(row["trigger_json"]),
            "world_generation": row["world_generation"],
            "map_session": row["map_session"], "note": row["note"]}


def _event_row(row: sqlite3.Row) -> dict[str, Any]:
    return {"seq": row["seq"], "at": row["at"], "kind": row["kind"],
            "body": _load(row["body_json"]) or {},
            "refs": _load(row["refs_json"]) or [],
            "evidence": _load(row["evidence_json"]) or [],
            "corrects": row["corrects"]}


def _dump(value: Any) -> str:
    """JSON with sorted keys, because a snapshot's digest has to be stable.

    Two identical worlds written by two dictionaries built in different orders
    must produce the same digest, or content addressing buys nothing.
    """
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      default=str)


def _load(text: str | None) -> Any:
    """Stored JSON, or None if it cannot be read.

    Unreadable rather than raising, for the reason the world state's viewer gives:
    one bad row should be reported next to the good ones rather than take the
    whole reading down.
    """
    if not text:
        return None
    try:
        return json.loads(text)
    except (ValueError, TypeError):
        return None


def _remove(path: str) -> None:
    try:
        os.remove(path)
    except OSError:
        pass
