"""The world state on disk: entities, their observation history, and the frames.

The one design decision here is that an observation and an entity are different
things. An observation is what the model said about one picture at one moment, and
it is never rewritten; an entity is the application's current opinion about a
lasting thing in the room, and it is derived from observations. Collapsing the two
-- letting an answer from the model update a row in place -- would destroy exactly
the evidence this experiment exists to collect, because "the sofa's description
changed" and "the model saw a different sofa" would leave the same record behind.

**Nothing creates an entity at the moment, and that is deliberate.** Entities used
to be allocated from whatever the model said it was looking at, which the rover
measured as worthless in both directions: one model never recognises anything and
the other recognises things that are not in the room. So `record` writes
observations and stops there, every one of them carrying the gimbal angles and the
rover pose behind it, and the `entities` table waits for the resolver that will
key identity off a triangulated map position rather than off a picture. What comes
back from an inspection now is an honest record of what was seen, from a measured
place, at a known time.

The database and the frames live outside the deploy tree, under ``~/.ugv/world``,
for the same reason the TLS keys do: a source deploy replaces ``~/ugv`` and would
otherwise take the experiment's results with it.
"""
from __future__ import annotations

import json
import os
import sqlite3
import threading
import time
import uuid
from typing import Any

from . import view

#: Where the database and the frames go. Overridable so the tests can run against
#: a temporary directory, which is the only thing that ever overrides it.
ENV_DIR = "UGV_WORLD_DIR"

SCHEMA_VERSION = 2
#: How many past inferences the diagnostics view keeps in front of the reader.
INFERENCE_LIMIT = 12
#: How many appearance vectors an entity keeps. Several, because the average of a
#: chair seen from the front and the same chair from the side is a picture of
#: neither; few, because every candidate's exemplars are read on every decision
#: and a thing seen two hundred times is not better identified by two hundred
#: vectors.
EXEMPLARS = 5


#: Where Linux publishes an identifier that is fixed for the life of one boot and
#: different on the next. Read in preference to the clock or the uptime because
#: this rover has no battery-backed clock: its wall time starts at whatever the
#: last write left behind and jumps again the moment it reaches the network, so
#: "was this row written before the machine came up" is not a question the
#: timestamps on the rows can answer. Absent off Linux, and the empty string that
#: comes back then means "which boot this is cannot be told" rather than any
#: particular boot.
BOOT_ID_PATH = "/proc/sys/kernel/random/boot_id"


def world_dir() -> str:
    return os.environ.get(ENV_DIR) or os.path.expanduser("~/.ugv/world")


def host_boot_id() -> str:
    try:
        with open(BOOT_ID_PATH) as handle:
            return handle.read().strip()
    except OSError:
        return ""


class WorldStore:
    """The semantic world, as SQLite plus a directory of JPEGs.

    One connection guarded by one lock rather than a connection per thread. The
    daemon serves each client on its own thread, so a console reading the entity
    list while an inspection writes one is the ordinary case rather than the rare
    one; the writes here are milliseconds because the model call happens well
    outside the transaction, so the lock costs nothing and removes "database is
    locked" from the failure list entirely.

    WAL is still set, and not for that: a reader outside this process -- the
    sqlite3 command line over ssh, which is how the experiment's results get
    looked at -- would otherwise be blocked by an inspection that is writing, and
    the default journal turns that ordinary overlap into an error.
    """

    def __init__(self, directory: str | None = None) -> None:
        self.dir = directory or world_dir()
        self.frames_dir = os.path.join(self.dir, "frames")
        os.makedirs(self.frames_dir, exist_ok=True)
        self.path = os.path.join(self.dir, "world.db")
        self._lock = threading.RLock()
        self.db = sqlite3.connect(self.path, check_same_thread=False, timeout=5.0)
        self.db.row_factory = sqlite3.Row
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA busy_timeout=5000")
        self._create()

    # --- schema ---------------------------------------------------------------

    def _create(self) -> None:
        """Bring an empty or existing database up to the current schema.

        Written so that opening a database that does not exist is the ordinary
        path rather than a special case: the rover's first inspection after a new
        computer, and every test in `selftest.py`, both start here.
        """
        with self._lock, self.db:
            self.db.executescript(SCHEMA)
            for table, columns in ADDED_COLUMNS.items():
                self._add_columns(table, columns)
            self.db.execute(
                "REPLACE INTO meta(key, value) VALUES('schema_version', ?)",
                (str(SCHEMA_VERSION),))
            # Session one exists from the moment the database does, so that the
            # very first observation is stamped with something rather than with
            # null. Null means "which map was live is not known", which is a real
            # state -- an observation taken with no navigator running -- and it
            # should not also be what a brand new database says.
            self.db.execute(
                "INSERT OR IGNORE INTO meta(key, value) VALUES('map_session', '1')")

    def _add_columns(self, table: str, columns: dict[str, str]) -> None:
        """Add columns a later version wants to a table an earlier one created.

        `CREATE TABLE IF NOT EXISTS` does nothing to a table that already exists,
        so a database written by an older build would silently go on without the
        new column and every insert naming it would fail. This is the migration,
        and it is deliberately the only kind there is: **columns are added and
        never removed or retyped**, so a rover's recorded history survives every
        change to what is recorded next.

        The rows already in the deployed database were written when the model was
        still being asked which lasting thing it was looking at. They keep their
        answers, in columns nothing writes any more, because throwing away the
        evidence for a negative result would leave nothing to point at.
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

    def map_session(self) -> int:
        try:
            return int(self._meta("map_session", "1"))
        except ValueError:
            return 1

    def new_map_session(self) -> int:
        """The SLAM map was cleared, so anything positional from before belongs to
        a map that no longer exists.

        Nothing here refuses anything on the strength of the number. It is stamped
        on every observation so that an entity last seen under an older map is
        visible as such, and so that a staleness rule -- whenever one arrives --
        finds a history to apply itself to rather than a migration to perform.

        Both directions matter and only one of them is obvious: this does not touch
        a single entity or observation, and `clear` does not touch the SLAM map.
        """
        with self._lock, self.db:
            session = self.map_session() + 1
            self.db.execute("REPLACE INTO meta(key, value) VALUES('map_session', ?)",
                            (str(session),))
        return session

    # --- frames ---------------------------------------------------------------

    def save_frame(self, jpeg: bytes, width: int | None = None,
                   height: int | None = None) -> str:
        """Keep the picture the model was shown, and answer with its name.

        Without the frame there is no way to separate a hallucinated entity from a
        real one the person reading the popup had forgotten was in the room, and
        telling those two apart is most of what this experiment is for. Kept for
        failed inferences too: a model that answered with nonsense is worth looking
        at beside what it was looking at.
        """
        frame_id = f"{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:6]}"
        path = self.frame_path(frame_id)
        with open(path, "wb") as handle:
            handle.write(jpeg)
        with self._lock, self.db:
            self.db.execute(
                "INSERT INTO frames(id, path, taken_at, bytes, width, height) "
                "VALUES(?, ?, ?, ?, ?, ?)",
                (frame_id, path, time.time(), len(jpeg), width, height))
        return frame_id

    def frame_path(self, frame_id: str) -> str:
        return os.path.join(self.frames_dir, f"{frame_id}.jpg")

    def frame(self, frame_id: str) -> bytes | None:
        """The stored JPEG, or None if it is not there any more.

        A missing frame is an ordinary answer rather than an error: the row that
        references it outlives the file if somebody empties the directory by hand,
        and a viewer that fell over on that would be hiding the observations it was
        opened to show. The name is checked rather than trusted because it arrives
        from a browser and is about to be turned into a path.
        """
        if not frame_id or not _plain_name(frame_id):
            return None
        try:
            with open(self.frame_path(frame_id), "rb") as handle:
                return handle.read()
        except OSError:
            return None

    # --- reading --------------------------------------------------------------

    def summary(self) -> dict[str, Any]:
        with self._lock:
            entities = self.db.execute(
                "SELECT COUNT(*) AS n FROM entities").fetchone()["n"]
            observations = self.db.execute(
                "SELECT COUNT(*) AS n FROM observations").fetchone()["n"]
            unmatched = self.db.execute(
                "SELECT COUNT(*) AS n FROM observations "
                "WHERE entity_id IS NULL").fetchone()["n"]
            inspections = self.db.execute(
                "SELECT COUNT(*) AS n FROM inferences").fetchone()["n"]
            last = self.db.execute(
                "SELECT * FROM inferences ORDER BY id DESC LIMIT 1").fetchone()
            last_ok = self.db.execute(
                "SELECT started_at FROM inferences WHERE status = 'ok' "
                "ORDER BY id DESC LIMIT 1").fetchone()
        return {
            "entities": entities,
            "observations": observations,
            "unmatched": unmatched,
            "inspections": inspections,
            "map_session": self.map_session(),
            "last_at": None if last is None else last["started_at"],
            "last_status": None if last is None else last["status"],
            "last_detail": None if last is None else last["detail"],
            "last_ok_at": None if last_ok is None else last_ok["started_at"],
        }

    def entities(self) -> list[dict[str, Any]]:
        """Every entity, most recently seen first.

        Two derived columns come with it, and both exist so the console can show
        an entity without asking a second question: the map session it was last
        seen under, so that one belonging to a map that no longer exists shows as
        such, and the newest frame it appeared in, which is the picture the list
        draws.
        """
        with self._lock:
            rows = self.db.execute("""
                SELECT e.*,
                       (SELECT o.map_session FROM observations o
                         WHERE o.entity_id = e.id
                         ORDER BY o.observed_at DESC, o.id DESC LIMIT 1)
                           AS last_map_session,
                       (SELECT o.frame_id FROM observations o
                         WHERE o.entity_id = e.id
                         ORDER BY o.observed_at DESC, o.id DESC LIMIT 1)
                           AS last_frame_id
                  FROM entities e
                 ORDER BY e.last_seen_at DESC, e.id
            """).fetchall()
        return [_shown(dict(row)) for row in rows]

    def entity(self, entity_id: str) -> dict[str, Any] | None:
        """One entity, shaped the same way the list shapes them.

        **Through `_shown` for the reason the list is, and this is not a
        tidy-up.** It used to hand back the row as it came out of SQLite, so
        the reply to `world_state_entity` carried a raw float32 BLOB of
        exemplars; the daemon could not serialise it, answered nothing at
        all, and the console's detail pane said "nothing selected" for every
        entity anyone clicked. It also left `placement_json` undecoded, so
        even a reply that got through would have shown a placed thing as
        having no position.
        """
        with self._lock:
            row = self.db.execute("SELECT * FROM entities WHERE id = ?",
                                  (entity_id,)).fetchone()
        return None if row is None else _shown(dict(row))

    def observations(self, entity_id: str | None = None, limit: int = 200,
                     unmatched: bool = False,
                     before: tuple[float, int] | None = None,
                     ids: list[int] | None = None) -> list[dict[str, Any]]:
        """Observation history, newest first.

        `entity_id` selects one entity's history; `unmatched` selects the
        observations no entity was allocated for, which is where everything
        starts and where a thing seen once from one place stays. The popup shows
        them so that a pool that is not draining is visible rather than lost.

        `before` is where a caller that already holds part of the history wants
        the next page to start: the `(observed_at, id)` of the oldest row it has,
        and the answer begins with the one below it. **A place in the history
        rather than a count of rows to skip**, because the rover goes on
        recording while somebody reads: every look taken during the reading
        pushes the whole history down by one, so an offset counted from the
        newest row would hand back rows already on the screen and step over
        others entirely. A place cannot move.

        `ids` names the rows wanted rather than describing a window of the
        history. It is what a search needs: the ranking runs over the vector
        columns alone, and the console draws the whole of every look it matched
        whether or not the stream on screen had reached back that far.
        """
        query = "SELECT * FROM observations"
        where: list[str] = []
        args: list[Any] = []
        if entity_id is not None:
            where.append("entity_id = ?")
            args.append(entity_id)
        elif unmatched:
            where.append("entity_id IS NULL")
        if before is not None:
            where.append("(observed_at < ? OR (observed_at = ? AND id < ?))")
            args += [float(before[0]), float(before[0]), int(before[1])]
        if ids is not None:
            if not ids:
                return []
            where.append("id IN (" + ",".join("?" * len(ids)) + ")")
            args += [int(one) for one in ids]
        if where:
            query += " WHERE " + " AND ".join(where)
        query += " ORDER BY observed_at DESC, id DESC LIMIT ?"
        args.append(int(limit))
        with self._lock:
            rows = self.db.execute(query, args).fetchall()
        return [_readable(dict(row)) for row in rows]

    def unplaced(self, map_session: int | None = None,
                 limit: int = 500) -> list[dict[str, Any]]:
        """The pending pool: observations with a direction and no home yet.

        **One bearing is a direction, not a position.** An observation belongs
        here from the moment it is taken until a second look from far enough away
        crosses it, which may be seconds later or may be never -- a thing seen
        once from one place is a thing the rover cannot honestly claim to have
        located. Nothing is discarded for waiting.

        Rows with no bearing are left out rather than kept waiting, because they
        are not pending anything: without a pose or a gimbal angle there is no
        direction to cross with a second one, and no later look can supply what
        was never measured.

        Oldest first within the window, because the first two looks at a thing
        are the pair most likely to have the longest baseline between them.

        **The window is the newest `limit`, and that is a correction rather than
        a preference.** It used to be the *oldest* `limit`, which is a jam and not
        a bound: once that many bearings had accumulated with nowhere to go -- and
        on the recording of 2026-09-03, 60 of 71 had nowhere to go -- the pool
        handed back the same unplaceable rows for ever and no observation taken
        afterwards was ever considered again. Every look after that point was
        wasted, silently, and the faster the rover looks the sooner it happens.

        What the correction costs is a bearing older than the window pairing with
        one taken now, which is a rover that saw something once, drove away and
        came back much later. That is the rarer case and it is the one the person
        at the console can see going by, because the pool's size is on the panel.
        """
        query = ("SELECT * FROM observations"
                 " WHERE entity_id IS NULL AND bearing_deg IS NOT NULL")
        args: list[Any] = []
        if map_session is not None:
            # A bearing measured in one map means nothing in the next, because
            # the coordinates it starts from moved.
            query += " AND map_session = ?"
            args.append(int(map_session))
        query += " ORDER BY observed_at DESC, id DESC LIMIT ?"
        args.append(int(limit))
        with self._lock:
            rows = self.db.execute(query, args).fetchall()
        return [_readable(dict(row), vectors=True) for row in reversed(rows)]

    def entities_in_frame(self, inference_id: Any) -> set:
        """Which lasting things this one look has already given a region to.

        **Two regions of one frame are two different things** -- the region
        finder's own overlap suppression saw to that -- so a look may give an
        entity one region and no more, ever.

        The resolver has always had that rule and kept it in a dictionary it
        built at the top of each pass, which is a memory that lasts exactly as
        long as the pass does. The pending pool does not: an observation with no
        partner waits indefinitely, by design. So a frame donated one more region
        every time round, and on the run of 2026-09-03 that is how a single
        entity came to hold four disjoint regions of one picture -- traced
        joining on three consecutive passes -- and twenty-six crops of at least
        six different objects. Asked of the store instead, the rule holds across
        passes and across restarts.
        """
        if inference_id is None:
            return set()
        with self._lock:
            rows = self.db.execute(
                "SELECT DISTINCT entity_id FROM observations"
                " WHERE inference_id = ? AND entity_id IS NOT NULL",
                (inference_id,)).fetchall()
        return {row["entity_id"] for row in rows}

    def placed(self, map_session: int | None = None) -> list[dict[str, Any]]:
        """Entities that have a position, in the map they were positioned in.

        A placement is meaningless outside its own map session: the coordinates
        it is written in came from a SLAM map that no longer exists once the map
        is cleared. So the session is part of the query rather than something a
        caller is trusted to check.
        """
        query = ("SELECT * FROM entities WHERE placement_json IS NOT NULL")
        args: list[Any] = []
        if map_session is not None:
            query += " AND placement_map_session = ?"
            args.append(int(map_session))
        with self._lock:
            rows = self.db.execute(query, args).fetchall()
        found = []
        for row in rows:
            entity = dict(row)
            try:
                entity["placement"] = json.loads(entity["placement_json"])
            except (ValueError, TypeError):
                continue
            entity.pop("exemplars", None)
            found.append(entity)
        return found

    def create_entity(self, kind: str = "object") -> str:
        """A lasting thing, identified by this application and nothing else.

        It gets an identifier and no name. Nothing measures what a thing is
        called any more -- the word list that used to supply one was measured to
        put "a computer monitor" on a sofa -- so the `label` column is written
        empty rather than filled with a guess, and what the console shows instead
        is the crops the thing was seen in.
        """
        now = time.time()
        entity_id = self.allocate(kind or "object")
        with self._lock, self.db:
            self.db.execute(
                "INSERT INTO entities(id, kind, label, canonical_description,"
                " created_at, last_seen_at, observation_count)"
                " VALUES(?,?,'','',?,?,0)",
                (entity_id, kind or "object", now, now))
        return entity_id

    def attach(self, entity_id: str, observation_ids: list[int],
               why: str = "") -> int:
        """Say which lasting thing these observations were of, and why.

        The observations are not rewritten in any other respect. What they
        recorded -- the box, the bearing, the pose behind it -- is the evidence
        for the decision and must survive the decision being made, and being
        made again differently later.

        `why` is the resolver's own sentence about this decision, kept because
        the question a person asks of an identity is not "what did it decide"
        but "why did it think that was the same chair", and an answer that only
        exists in the reply to the inspection that happened to trigger it is
        gone by the time anybody asks.
        """
        if not observation_ids:
            return 0
        marks = ",".join("?" * len(observation_ids))
        with self._lock, self.db:
            cursor = self.db.execute(
                f"UPDATE observations SET entity_id = ? WHERE id IN ({marks})",
                [entity_id, *[int(one) for one in observation_ids]])
            if why:
                self.db.execute(
                    f"UPDATE observations SET note = ? WHERE id IN ({marks})",
                    [why, *[int(one) for one in observation_ids]])
            row = self.db.execute(
                "SELECT COUNT(*) AS n, MAX(observed_at) AS last FROM observations"
                " WHERE entity_id = ?", (entity_id,)).fetchone()
            self.db.execute(
                "UPDATE entities SET observation_count = ?, last_seen_at = ?"
                " WHERE id = ?",
                (row["n"], row["last"] or time.time(), entity_id))
        return cursor.rowcount

    def place(self, entity_id: str, placement: dict[str, Any] | None,
              map_session: int) -> None:
        """Where this thing is, and how far out that might be.

        The placement replaces whatever was there; the observations that produced
        it do not. That asymmetry is the design: an estimate is the application's
        current opinion and may improve or be withdrawn, while the measurements
        behind it are history and are never touched.
        """
        with self._lock, self.db:
            if placement is None:
                self.db.execute(
                    "UPDATE entities SET placement_json = NULL,"
                    " placement_uncertainty_m = NULL, placement_map_session = NULL,"
                    " placement_updated_at = NULL WHERE id = ?", (entity_id,))
                return
            self.db.execute(
                "UPDATE entities SET placement_json = ?,"
                " placement_uncertainty_m = ?, placement_map_session = ?,"
                " placement_updated_at = ? WHERE id = ?",
                (json.dumps(placement), placement.get("uncertainty_m"),
                 int(map_session), time.time(), entity_id))

    def exemplars(self, entity_id: str, width: int = 0) -> list[bytes]:
        """The appearance vectors kept for this thing, as raw float32.

        Several rather than one averaged vector, because the average of a chair
        seen from the front and the same chair seen from the side is a picture of
        neither.
        """
        with self._lock:
            row = self.db.execute("SELECT exemplars FROM entities WHERE id = ?",
                                  (entity_id,)).fetchone()
        blob = None if row is None else row["exemplars"]
        if not blob or width <= 0:
            return []
        return [blob[start:start + width]
                for start in range(0, len(blob) - width + 1, width)]

    def add_exemplar(self, entity_id: str, vector: bytes,
                     keep: int = EXEMPLARS) -> int:
        """Keep one more appearance vector, dropping the oldest beyond `keep`.

        Bounded because this is evidence for a comparison rather than a history:
        an entity seen two hundred times does not become better identified by
        holding two hundred vectors, and the column is read on every candidate.
        """
        if not vector:
            return 0
        width = len(vector)
        with self._lock, self.db:
            row = self.db.execute("SELECT exemplars FROM entities WHERE id = ?",
                                  (entity_id,)).fetchone()
            blob = (row["exemplars"] if row is not None else None) or b""
            if len(blob) % width:
                # A vector of a different width means a different model produced
                # it, and mixing the two would compare numbers that mean
                # different things. The older ones go.
                blob = b""
            blob = (blob + vector)[-width * max(1, keep):]
            self.db.execute("UPDATE entities SET exemplars = ? WHERE id = ?",
                            (blob, entity_id))
        return len(blob) // width

    def searchable(self, map_session: int | None = None,
                   limit: int = 2000) -> list[dict[str, Any]]:
        """Every observation that carries a semantic vector, newest first.

        What a text search ranks. Brute force over a few hundred vectors is a
        dot product apiece, which is why the design refuses a vector database:
        the whole store fits in memory several times over and an index would be
        another thing to keep true.

        The backend that produced each vector comes along, because a query
        embedded on the GPU cannot be compared with a vector produced on the CPU
        and the caller has to be able to say so.
        """
        query = ("SELECT id, entity_id, frame_id, observed_at,"
                 " map_session, bearing_deg, bbox_json, siglip_blob,"
                 " vectors_from"
                 "  FROM observations WHERE siglip_blob IS NOT NULL")
        args: list[Any] = []
        if map_session is not None:
            query += " AND map_session = ?"
            args.append(int(map_session))
        query += " ORDER BY observed_at DESC, id DESC LIMIT ?"
        args.append(int(limit))
        with self._lock:
            rows = self.db.execute(query, args).fetchall()
        found = []
        for row in rows:
            one = dict(row)
            # The box comes out decoded, under the name the rest of the codebase
            # uses for it, so a caller that wants to draw the match on the frame
            # does not have to know the column is JSON.
            text = one.pop("bbox_json", None)
            try:
                one["bbox"] = json.loads(text) if text else None
            except (ValueError, TypeError):
                one["bbox"] = None
            found.append(one)
        return found

    def inferences(self, limit: int = INFERENCE_LIMIT) -> list[dict[str, Any]]:
        with self._lock:
            rows = self.db.execute(
                "SELECT * FROM inferences ORDER BY id DESC LIMIT ?",
                (int(limit),)).fetchall()
        return [dict(row) for row in rows]

    # --- writing --------------------------------------------------------------

    def record_inference(self, **fields: Any) -> int:
        """One line in the diagnostics log. Every inspection writes exactly one,
        including the ones that never reached the model."""
        values = [fields.get(name) for name in INFERENCE_COLUMNS]
        with self._lock, self.db:
            cursor = self.db.execute(
                f"INSERT INTO inferences({', '.join(INFERENCE_COLUMNS)}) "
                f"VALUES({', '.join('?' * len(INFERENCE_COLUMNS))})", values)
        return int(cursor.lastrowid)

    def update_inference(self, inference_id: int, **fields: Any) -> None:
        unknown = set(fields) - set(INFERENCE_COLUMNS)
        if unknown:
            raise ValueError(f"no such inference column: {sorted(unknown)}")
        if not fields:
            return
        assignments = ", ".join(f"{name} = ?" for name in fields)
        with self._lock, self.db:
            self.db.execute(f"UPDATE inferences SET {assignments} WHERE id = ?",
                            [*fields.values(), inference_id])

    def allocate(self, kind: str) -> str:
        """The next identifier of this kind. The application's, never a model's.

        Counted in a table rather than derived from the highest existing row, so
        that an entity deleted by hand cannot hand its name to a different thing
        later. `clear` resets the counters with everything else, which is what
        makes a fresh experiment start at one again.

        Public, and with no caller inside this file at the moment: the resolver
        that will create entities from triangulated positions lives outside the
        store, and this is the door it comes in by. The rule it enforces -- that
        names are allocated here and nowhere else -- is the one thing about
        identity that survived both models failing at it.
        """
        with self._lock, self.db:
            row = self.db.execute("SELECT next FROM counters WHERE kind = ?",
                                  (kind,)).fetchone()
            number = 1 if row is None else int(row["next"])
            self.db.execute("REPLACE INTO counters(kind, next) VALUES(?, ?)",
                            (kind, number + 1))
        return f"{kind}:{number}"

    def record(self, seen: list, *, capture: dict[str, Any],
               source: str = "perception", model_id: str = "",
               inference_id: int | None = None,
               fov_deg: float | None = None,
               region_source: str = "",
               vectors_from: str = "") -> dict[str, Any]:
        """Store one inspection's observations. No identity is decided here.

        Every row goes in with a null `entity_id`, and that is the honest state of
        the world rather than a gap waiting to be filled by something cheap. Which
        lasting thing an observation belongs to is a question about *where the
        thing is*, and one look from one place does not answer it: an object gets a
        position only once a second look arrives from somewhere far enough away for
        two bearings to cross. Until the resolver that does that arrives, an
        inspection records what was seen, from a measured pose, at a known time,
        and claims nothing further.

        What is deliberately **not** done meanwhile is a cheap stand-in -- matching
        on a name, say. The rover has already measured what that would be worth
        twice over: one language model called the same chair a black leather
        recliner and then a blue leather one on a byte-identical frame, and the
        word list that replaced it scored every phrase between 0.08 and 0.12
        whatever the crop held. Nothing measures what a thing is called any more,
        so nothing writes `label` at all and it comes back null.
        """
        now = time.time()
        session = self.map_session()
        pose = capture.get("pose")
        stored = 0
        placed = 0
        with self._lock, self.db:
            for item in seen:
                bbox = getattr(item, "bbox", None)
                # The bearing is worked out here, once, from what the rover
                # measured at the moment of the look: where it was standing,
                # where the gimbal was turned to, and where in the picture the
                # thing sat. Storing it rather than recomputing it later is the
                # point -- the camera's field of view is a property of the rover
                # at that moment, and a lens change should not silently rewrite
                # every bearing the rover ever measured.
                bearing = span = None
                elevation = elevation_span = None
                if fov_deg:
                    # The gimbal's tilt goes in with the pan, and it is not a
                    # decoration: the camera tilts about its own horizontal, so
                    # a ray's bearing cannot be read off the picture until the
                    # tilt is undone. It was recorded on every observation and
                    # dropped here, which put 184 of the 441 boxes of the drive
                    # of 2026-09-03 outside the accuracy `locate` is promised.
                    # The frame's size goes in because it chooses the lens, the
                    # capture mode being a window onto the sensor as well as a
                    # pixel count. See `view.azimuth_deg`.
                    drawn = view.ray({"pose": pose, "bbox": bbox,
                                      "observer_pan_deg": capture.get("pan"),
                                      "observer_tilt_deg": capture.get("tilt")},
                                     float(fov_deg),
                                     size=capture.get("frame_size"))
                    if drawn is not None:
                        bearing = drawn["bearing_deg"]
                        span = drawn["span_deg"]
                        # The vertical half of the same ray. It costs nothing
                        # further to take -- the projection returns a direction
                        # in three dimensions and the bearing uses two of them
                        # -- and until it was written down every fact this
                        # component held about the room was flat.
                        elevation = drawn["elevation_deg"]
                        elevation_span = drawn["elevation_span_deg"]
                        placed += 1
                # A row carries no name, no scene sentence, no prompt version and
                # no warning about any of them. Those four columns belonged to a
                # language model describing the picture in words; nothing writes
                # them now, and they stay in the table because the rows that hold
                # them are the record of that having been tried.
                self.db.execute(
                    "INSERT INTO observations(entity_id, inference_id, observed_at,"
                    " source, frame_id, frame_path,"
                    " bbox_json, observer_pan_deg,"
                    " observer_tilt_deg, observer_pose_json, map_session, model_id,"
                    " raw_json,"
                    " bearing_deg, span_deg, origin_sigma_m,"
                    " bearing_sigma_deg,"
                    " elevation_deg, elevation_span_deg,"
                    " region_source, region_score,"
                    " dino_blob, siglip_blob, vectors_from)"
                    " VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
                    (None, inference_id, now, source, capture.get("frame_id"),
                     capture.get("frame_path"),
                     None if bbox is None else json.dumps(bbox),
                     capture.get("pan"), capture.get("tilt"),
                     None if pose is None else json.dumps(pose), session, model_id,
                     json.dumps(item.raw, sort_keys=True),
                     bearing, span, capture.get("origin_sigma_m"),
                     capture.get("bearing_sigma_deg"),
                     elevation, elevation_span,
                     region_source or None,
                     getattr(item, "region_score", None) or None,
                     getattr(item, "dino", b"") or None,
                     getattr(item, "siglip", b"") or None,
                     vectors_from or None))
                stored += 1
        # `matched` and `created` are reported as zero rather than dropped, because
        # the console's diagnostics table and the deployed database's older rows
        # both still speak in them, and "nothing was matched and nothing created"
        # is the true answer for every inspection this build performs.
        return {"stored": stored, "matched": 0, "created": 0, "rejected": 0,
                "placed": placed, "entities": [], "map_session": session}

    def clear(self) -> dict[str, Any]:
        """Throw the semantic world away, and nothing else.

        The SLAM map is not touched and cannot be from here -- this process does
        not own it. That separation is the point: a repeatable experiment needs to
        start from an empty world without also throwing away the map the rover
        spent ten minutes building.

        The frames go with the rows that reference them. A directory of JPEGs that
        nothing points at is not evidence of anything, and on a rover it is the
        thing that quietly fills the disk.
        """
        removed = 0
        with self._lock, self.db:
            counts = {
                "entities": self.db.execute(
                    "SELECT COUNT(*) AS n FROM entities").fetchone()["n"],
                "observations": self.db.execute(
                    "SELECT COUNT(*) AS n FROM observations").fetchone()["n"],
            }
            self.db.execute("DELETE FROM observations")
            self.db.execute("DELETE FROM entities")
            self.db.execute("DELETE FROM inferences")
            self.db.execute("DELETE FROM frames")
            self.db.execute("DELETE FROM counters")
        # Everything in the directory rather than everything the table knew about,
        # so that a cleared world really is an empty directory: a frame stored for
        # an inference that then failed to write its row would otherwise stay
        # behind for ever, pointed at by nothing.
        for name in os.listdir(self.frames_dir):
            if name.endswith(".jpg"):
                try:
                    os.remove(os.path.join(self.frames_dir, name))
                    removed += 1
                except OSError:
                    pass
        return {"ok": True, "frames_removed": removed, **counts,
                "map_session": self.map_session()}

    def clear_if_rebooted(self, boot_id: str | None = None) -> dict[str, Any]:
        """Throw the world away when the host has rebooted since it was written.

        Everything in here is measured against the SLAM map: a bearing means
        something only from the pose it was taken at, and a position only in the
        frame the bearings crossed in. Nothing saves that map, so every boot
        starts an empty one -- and the store used to come back holding the old
        map's positions stamped with the *same* map session as the new map, which
        is precisely the comparison `new_map_session` exists to prevent. The
        console draws exactly those rows: a thing is on the map when its
        placement session matches the store's, so two chairs measured in a room
        the rover has since been carried out of appeared on the fresh map as
        though they had just been seen.

        Bumping the session and keeping the rows is the other answer, and it is
        the one the console's map button gave until it stopped: what survives
        that is a list of things with nowhere to be.

        An unknown boot deletes nothing. That is either a host with no `/proc` to
        read -- a desk, a replay -- or a database written before this was
        recorded, and "I cannot tell whether this machine has rebooted" must not
        be a reason to destroy an experiment somebody is halfway through. The
        identifier is remembered either way, so the next boot is knowable.

        `boot_id` is the host's own by default. Passing one names the boot
        instead, and passing the empty string is a host that cannot say which
        boot it is on -- which is a different thing from not asking, and the
        distinction is the whole of the caution above, so it is in the argument
        rather than in a falsy default that would quietly read `/proc`.
        """
        boot_id = host_boot_id() if boot_id is None else boot_id
        was = self._meta("boot_id")
        if not boot_id:
            return {"cleared": False, "boot_id": was,
                    "reason": "this host does not say which boot it is on"}
        if was == boot_id:
            return {"cleared": False, "boot_id": boot_id,
                    "reason": "the world was recorded under this boot"}
        with self._lock, self.db:
            self.db.execute("REPLACE INTO meta(key, value) VALUES('boot_id', ?)",
                            (boot_id,))
        if not was:
            return {"cleared": False, "boot_id": boot_id,
                    "reason": "the world does not say which boot recorded it"}
        gone = self.clear()
        # The session moves as well as the rows going, for the same reason the
        # console moves it: a clear that half failed must not leave old
        # coordinates comparable with new ones.
        return {"cleared": True, "boot_id": boot_id,
                "reason": f"recorded under boot {was}",
                "entities": gone["entities"],
                "observations": gone["observations"],
                "frames_removed": gone["frames_removed"],
                "map_session": self.new_map_session()}

    def close(self) -> None:
        with self._lock:
            self.db.close()


INFERENCE_COLUMNS = (
    "started_at", "duration_s", "status", "detail", "backend", "model_id",
    "prompt_version", "frame_id", "frame_live", "known_count", "returned",
    "stored", "matched", "created", "rejected", "map_session", "raw_json",
)

#: Columns added after the table they belong to was first created, applied by
#: `_add_columns` every time a database is opened. Adding to this is how the
#: schema grows; nothing is ever taken out of it, because a rover's recorded
#: history has to survive changes to what gets recorded next.
#:
#: `known_count` is the counterpart, and it stays in the table above without
#: appearing here: it was written when the model was still shown the entities it
#: had already named, nothing writes it now, and the old rows that hold it are the
#: record of that experiment.
ADDED_COLUMNS = {
    "inferences": {"stored": "INTEGER"},
    "observations": {
        # Where the thing was, from where the rover stood. This is the
        # measurement identity will be decided from, and it is stored per
        # observation rather than recomputed, because the camera's field of view
        # is a property of the rover at the moment of the look.
        "bearing_deg": "REAL",
        "span_deg": "REAL",
        # And how high it was, in degrees above the horizontal, with how tall it
        # looked. Measured off the same ray as the bearing and stored for the
        # same reason -- it is what the camera saw at that moment, not what
        # today's lens would say about that box. Null on every row written
        # before the vertical half of the ray was kept, and null there means
        # "not measured" rather than "level": see `locate.rise_m`, which
        # declines to answer at all rather than assume.
        "elevation_deg": "REAL",
        "elevation_span_deg": "REAL",
        # How well this particular bearing is known, in degrees, where the
        # constant in `locate` is what one is worth from a rover standing still.
        # Null on every row written before the inspection measured it, and null
        # means the constant -- see `locate.sigma_of`.
        "bearing_sigma_deg": "REAL",
        # How far out the point the bearing starts from may be, in metres: half
        # of whatever the rover covered while the shutter was open. Null on every
        # row written before this was measured, and null is right for them --
        # the gate that produced those rows refused any look taken on the move.
        # See `Inspector.MOVED_WHILE_LOOKING_M` and `locate.fix`.
        "origin_sigma_m": "REAL",
        # What drew the box, and how sure the region finder was of it.
        "region_source": "TEXT",
        "region_score": "REAL",
        # Nothing writes this now. It held how near a region's vector was to the
        # nearest phrase in a fixed word list, and the answer was between 0.08
        # and 0.12 whatever the crop held -- which is why there is no word list
        # any more. The column stays because the rows that carry it are the
        # record of that having been tried.
        "label_score": "REAL",
        # The two vectors, as raw float32. A BLOB and a numpy dot product is the
        # whole of the design's answer to "where is the vector database".
        "dino_blob": "BLOB",
        "siglip_blob": "BLOB",
        # **Which backend produced those two vectors, and it is load-bearing.**
        # The GPU engines and the CPU int8 graphs agree with full precision to
        # 1.000 and 0.86 respectively, which is far too wide a gap to compare
        # across. A resolver must never match a vector from one against a vector
        # from the other.
        "vectors_from": "TEXT",
    },
    "entities": {
        # Where this thing is, once two bearings from far enough apart have
        # crossed. Null until then, and null is the honest state: one bearing is
        # a direction and not a position.
        "placement_json": "TEXT",
        "placement_uncertainty_m": "REAL",
        # A placement means nothing outside the map it was measured in.
        "placement_map_session": "INTEGER",
        "placement_updated_at": "REAL",
        # Several appearance vectors rather than one averaged one, because an
        # average of two viewpoints of a chair is a picture of neither.
        "exemplars": "BLOB",
    },
}

# Three columns here are written by nothing in this build and are kept anyway,
# which is worth saying out loud so a later reader does not take them for an
# oversight. `observations.description` and `observations.location_hint` were the
# model's prose about each thing; `inferences.known_count` was how many entities it
# was shown before it answered. All three belonged to asking a model which lasting
# thing it was looking at, which the rover measured and which failed, and the rows
# holding them are the evidence for that. Columns are added and never dropped.
#
# The `entities` table is likewise empty on a fresh database and stays that way
# until the resolver arrives. It is not dead: it is the shape identity will take
# once it comes from a triangulated position rather than from a picture.
#
# No `state` column on entities, although the task's suggested shape lists one.
# Nothing would write anything but 'present' into it and nothing would read it, and
# "add fields only when they answer a real question" is the rule that decides it.
# Whether an entity has gone quiet is `last_seen_at` and whether it belongs to a
# map that no longer exists is `map_session`; both are answered from rows that
# something actually writes.
SCHEMA = """
    CREATE TABLE IF NOT EXISTS meta (
        key   TEXT PRIMARY KEY,
        value TEXT NOT NULL
    );
    CREATE TABLE IF NOT EXISTS counters (
        kind TEXT PRIMARY KEY,
        next INTEGER NOT NULL
    );
    CREATE TABLE IF NOT EXISTS entities (
        id                    TEXT PRIMARY KEY,
        kind                  TEXT NOT NULL,
        label                 TEXT NOT NULL,
        canonical_description TEXT NOT NULL DEFAULT '',
        created_at            REAL NOT NULL,
        last_seen_at          REAL NOT NULL,
        observation_count     INTEGER NOT NULL DEFAULT 0
    );
    CREATE TABLE IF NOT EXISTS observations (
        id                 INTEGER PRIMARY KEY AUTOINCREMENT,
        entity_id          TEXT,
        inference_id       INTEGER,
        observed_at        REAL NOT NULL,
        source             TEXT NOT NULL,
        frame_id           TEXT,
        frame_path         TEXT,
        scene_summary      TEXT,
        label              TEXT,
        description        TEXT,
        location_hint      TEXT,
        bbox_json          TEXT,
        observer_pan_deg   REAL,
        observer_tilt_deg  REAL,
        observer_pose_json TEXT,
        map_session        INTEGER,
        model_id           TEXT,
        prompt_version     TEXT,
        raw_json           TEXT,
        note               TEXT
    );
    CREATE INDEX IF NOT EXISTS observations_by_entity
        ON observations(entity_id, observed_at DESC);
    -- The order the console reads the history in, which is the whole of it
    -- newest first. Without this, every page of the stream sorts the entire
    -- table again, and the store grows by a look a second for as long as the
    -- rover is switched on.
    CREATE INDEX IF NOT EXISTS observations_by_time
        ON observations(observed_at DESC, id DESC);
    CREATE INDEX IF NOT EXISTS observations_by_inference
        ON observations(inference_id);
    CREATE TABLE IF NOT EXISTS inferences (
        id             INTEGER PRIMARY KEY AUTOINCREMENT,
        started_at     REAL NOT NULL,
        duration_s     REAL,
        status         TEXT NOT NULL,
        detail         TEXT,
        backend        TEXT,
        model_id       TEXT,
        prompt_version TEXT,
        frame_id       TEXT,
        frame_live     INTEGER,
        known_count    INTEGER,
        returned       INTEGER,
        stored         INTEGER,
        matched        INTEGER,
        created        INTEGER,
        rejected       INTEGER,
        map_session    INTEGER,
        raw_json       TEXT
    );
    CREATE TABLE IF NOT EXISTS frames (
        id       TEXT PRIMARY KEY,
        path     TEXT NOT NULL,
        taken_at REAL NOT NULL,
        bytes    INTEGER,
        width    INTEGER,
        height   INTEGER
    );
"""


def _plain_name(name: str) -> bool:
    """A frame identifier this store made: letters, digits and dashes, nothing else.

    It arrives from a browser and is about to become a path, which is the whole
    reason for checking rather than for trusting the caller.
    """
    return bool(name) and all(ch.isalnum() or ch == "-" for ch in name)


def _shown(entity: dict[str, Any]) -> dict[str, Any]:
    """An entity row as something that can be sent to a console.

    The counterpart of `_readable` for the other table, and it exists as one
    function for the reason that one does: every caller here is about to put
    the row on a socket as JSON, raw bytes cannot go, and a second copy of
    this shaping is a second copy that can drift. It already had -- see
    `entity`.

    The exemplars are raw float32 and how many there are is the useful part
    anyway; the placement is stored as JSON text and is wanted as an object.
    """
    blob = entity.pop("exemplars", None)
    entity["exemplar_count"] = 0 if not blob else max(1, len(blob) // 1536)
    text = entity.get("placement_json")
    try:
        entity["placement"] = json.loads(text) if text else None
    except (ValueError, TypeError):
        entity["placement"] = None
    return entity


def _readable(row: dict[str, Any], vectors: bool = False) -> dict[str, Any]:
    """Turn the JSON columns back into objects for a caller that is about to send
    the row somewhere as JSON anyway.

    Malformed stored JSON comes back as a marked string rather than raising. A row
    written by an older version, or one corrupted on disk, must not be able to take
    down the viewer that exists to show what went wrong.

    **The two vector columns come out by default**, replaced by their sizes.
    Almost every caller here is about to serialise the row as JSON and raw bytes
    cannot be serialised, so leaving them in would turn the console's entity list
    into a crash the first time an observation carried one. `vectors=True` is for
    the resolver, which is the one caller that wants the numbers rather than a
    picture of them.
    """
    for column, name in (("bbox_json", "bbox"), ("observer_pose_json", "pose"),
                         ("raw_json", "raw")):
        text = row.get(column)
        if text in (None, ""):
            row[name] = None
            continue
        try:
            row[name] = json.loads(text)
        except (ValueError, TypeError):
            row[name] = {"unreadable": str(text)[:400]}
    for column, name in (("dino_blob", "dino_bytes"), ("siglip_blob", "siglip_bytes")):
        blob = row.get(column)
        row[name] = 0 if blob is None else len(blob)
        if not vectors:
            row.pop(column, None)
    return row
