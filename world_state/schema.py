"""SQLite schema and additive migrations; preserve historical observation columns."""

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
        # How far away the depth camera said this thing was, in metres, and what
        # that reading is worth. **The one measurement no bearing can make**: two
        # bearings pointed at two different chairs cross at a point that is on
        # neither of them, and only a range says so. Null on every row written
        # before the OAK was read, and on every row since whose box the OAK was
        # not pointing at -- the gimbal turns and this camera does not. Null means
        # the geometry gets no opinion about the distance rather than that the
        # distance was nothing; see `locate.stands_at_range`.
        "range_m": "REAL",
        "range_sigma_m": "REAL",
        # And which of the rover's two cameras took the picture, because a pixel
        # does not mean the same thing in both. The bearing is worked out once,
        # when the look is taken, through whichever lens that camera has -- so
        # nothing downstream needs this to read a bearing back. It is here because
        # a row that cannot say which camera it came from cannot be re-examined
        # when one of the two turns out to have been mounted crooked.
        "camera": "TEXT",
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
# until the resolver places observations from measured bearings.
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
