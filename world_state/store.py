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


def world_dir() -> str:
    return os.environ.get(ENV_DIR) or os.path.expanduser("~/.ugv/world")


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
        visible as such, and so that when the staleness model in
        [docs/cosmos-reason2-integration.md](../docs/cosmos-reason2-integration.md)
        arrives it finds a history to apply itself to rather than a migration to
        perform.

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

        Two derived columns come with it, and both exist to make a failure
        visible rather than to be pretty: the map session the entity was last seen
        under, so that one belonging to a map that no longer exists shows as such,
        and how many different labels its own history holds, which is what a
        canonical description drifting away from its observations looks like from
        the outside.
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
                           AS last_frame_id,
                       (SELECT COUNT(DISTINCT o.label) FROM observations o
                         WHERE o.entity_id = e.id) AS distinct_labels
                  FROM entities e
                 ORDER BY e.last_seen_at DESC, e.id
            """).fetchall()
        return [dict(row) for row in rows]

    def entity(self, entity_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self.db.execute("SELECT * FROM entities WHERE id = ?",
                                  (entity_id,)).fetchone()
        return None if row is None else dict(row)

    def observations(self, entity_id: str | None = None, limit: int = 200,
                     unmatched: bool = False) -> list[dict[str, Any]]:
        """Observation history, newest first.

        `entity_id` selects one entity's history; `unmatched` selects the
        observations no entity was allocated for, which is where a model that
        invented an identifier or answered with nothing concrete ends up, and which
        the popup shows so that those are visible rather than lost.
        """
        query = "SELECT * FROM observations"
        args: list[Any] = []
        if entity_id is not None:
            query += " WHERE entity_id = ?"
            args.append(entity_id)
        elif unmatched:
            query += " WHERE entity_id IS NULL"
        query += " ORDER BY observed_at DESC, id DESC LIMIT ?"
        args.append(int(limit))
        with self._lock:
            rows = self.db.execute(query, args).fetchall()
        return [_readable(dict(row)) for row in rows]

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

    def record(self, seen: list, *, capture: dict[str, Any], scene: str = "",
               source: str = "cosmos_visual", model_id: str = "",
               prompt_version: str = "",
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
        on the label, say. The rover has already measured what that would be worth:
        one model called the same chair a black leather recliner and then a blue
        leather one on a byte-identical frame, and the twin chair a couch. Keying
        identity off a name that drifts like that would fill the store with
        confident wrong answers, which is worse than an empty entity table.
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
                if fov_deg:
                    drawn = view.ray({"pose": pose, "bbox": bbox,
                                      "observer_pan_deg": capture.get("pan")},
                                     float(fov_deg))
                    if drawn is not None:
                        bearing = drawn["bearing_deg"]
                        span = drawn["span_deg"]
                        placed += 1
                # The one thing worth saying about a row beyond what it holds:
                # whether its label could ever be recognised again. "a thing" is
                # kept as history, because it is what the model said, but it will
                # never pass the resolver's first gate and the popup should say so
                # rather than leave a reader wondering.
                note = None if item.concrete else (
                    "the label names nothing in particular, so this observation "
                    "can never be matched to a lasting thing")
                self.db.execute(
                    "INSERT INTO observations(entity_id, inference_id, observed_at,"
                    " source, frame_id, frame_path, scene_summary, label,"
                    " bbox_json, observer_pan_deg,"
                    " observer_tilt_deg, observer_pose_json, map_session, model_id,"
                    " prompt_version, raw_json, note,"
                    " bearing_deg, span_deg, region_source, region_score,"
                    " label_score, dino_blob, siglip_blob, vectors_from)"
                    " VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
                    (None, inference_id, now, source, capture.get("frame_id"),
                     capture.get("frame_path"), scene, item.label,
                     None if bbox is None else json.dumps(bbox),
                     capture.get("pan"), capture.get("tilt"),
                     None if pose is None else json.dumps(pose), session, model_id,
                     prompt_version or None,
                     json.dumps(item.raw, sort_keys=True), note,
                     bearing, span, region_source or None,
                     getattr(item, "region_score", None) or None,
                     getattr(item, "label_score", None) or None,
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
        # What drew the box, which is not the same question as what named it.
        "region_source": "TEXT",
        "region_score": "REAL",
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


def _readable(row: dict[str, Any]) -> dict[str, Any]:
    """Turn the JSON columns back into objects for a caller that is about to send
    the row somewhere as JSON anyway.

    Malformed stored JSON comes back as a marked string rather than raising. A row
    written by an older version, or one corrupted on disk, must not be able to take
    down the viewer that exists to show what went wrong.
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
    return row
