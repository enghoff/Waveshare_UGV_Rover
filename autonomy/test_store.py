"""The record: that it is only ever added to, and that it says when it cannot.

An episode nobody can check is worse than no episode, so the checks that matter
here are the ones about what the store refuses to do -- change a row it has
already written, hand an episode number out twice, or let a deleted picture read
like one that was never taken.
"""
from __future__ import annotations

import os
import sqlite3
import tempfile

import events
import refs
from store import ABSENT, DELETED, HELD, EpisodeStore
from test_fakes import PICTURE, WORLD, a_store, a_world, an_episode
from test_harness import check


def test_an_empty_database_is_an_ordinary_thing_to_open() -> None:
    """The state the rover is in after a new computer, and every check below."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        got = store.summary()
        check("no episodes", got["episodes"], 0)
        check("...no events", got["events"], 0)
        check("...no evidence", got["evidence"], 0)
        check("...a generation of its own",
              bool(refs.GENERATION.match(got["generation"])), True)
        check("...and an evidence directory",
              os.path.isdir(os.path.join(directory, "evidence")), True)
        store.close()


def test_nothing_in_the_record_is_ever_rewritten() -> None:
    """Watched as it runs, rather than argued for or grepped for.

    SQLite is asked to report every statement the store issues, and a full
    episode is recorded through it -- opened, appended to, closed, annotated
    afterwards, with evidence kept and then deleted and a merge recorded. Not one
    of those changes or removes a row.

    Stronger than reading the source, because it covers what the statements
    actually do rather than how they are spelled, and it is here because the
    append-only rule is the sort that quietly stops being true when somebody adds
    a convenient setter in six months.
    """
    ok = getattr(sqlite3, "SQLITE_OK", 0)
    update = getattr(sqlite3, "SQLITE_UPDATE", 23)
    delete = getattr(sqlite3, "SQLITE_DELETE", 9)
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        changed = []

        def watch(action, table, column, *_):
            if action in (update, delete):
                changed.append((action, table, column))
            return ok

        store.db.set_authorizer(watch)
        episode = an_episode(store)
        store.append(episode, events.note("noticed a fortnight later"))
        store.delete_evidence(refs.digest(PICTURE), "the owner asked")
        store.alias("merge", refs.world(WORLD, "object:8"),
                    refs.world(WORLD, "object:12"))
        store.db.set_authorizer(None)

        check("recording a whole episode changes no row", changed, [])
        store.close()


def test_an_episode_keeps_what_it_said_when_it_closed() -> None:
    """Closing appends; it does not fill anything in.

    Checked by reading the raw row rather than the store's own accessor, because
    what is being claimed is about the bytes in the table.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = store.open_episode("nothing_to_do", world_generation=WORLD)
        before = _raw_episode(store, episode)
        store.close_episode(episode, "abandoned", detail="no authority")
        after = _raw_episode(store, episode)
        check("the opened row is untouched by the close", after, before)
        check("...and the outcome is read off an event",
              store.outcome(episode)["outcome"], "abandoned")
        check("...with the detail it was given",
              store.outcome(episode)["detail"], "no authority")
        store.close()


def test_an_episode_cannot_be_closed_twice() -> None:
    """Two answers to "how did this go" is a bug worth hearing about."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = store.open_episode("nothing_to_do", world_generation=WORLD)
        store.close_episode(episode, "succeeded")
        try:
            store.close_episode(episode, "failed")
            check("a second close is refused", "allowed", "refused")
        except ValueError:
            check("a second close is refused", "refused", "refused")
        check("...and the first answer stands",
              store.outcome(episode)["outcome"], "succeeded")
        store.close()


def test_something_noticed_afterwards_is_still_recorded() -> None:
    """An annotation after the close is allowed on purpose.

    Refusing it would not stop somebody knowing the thing; it would only stop the
    record holding it. What the reader is told is that it came later, and
    `test_replay` checks that.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        seq = store.append(episode, events.note(
            "the owner says this was the painting, not the chair"))
        got = store.episode(episode)
        check("the annotation is in the episode",
              got["events"][-1]["seq"], seq)
        check("...and the close is still where it was",
              [one["kind"] for one in got["events"]].index("closed"),
              len(got["events"]) - 2)
        store.close()


def test_a_correction_points_at_what_it_corrects() -> None:
    """Both are kept. The wrong one is evidence of what was believed."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = store.open_episode("nothing_to_do", world_generation=WORLD)
        wrong = store.append(episode, events.note("ranged at 1.08 m"))
        store.append(episode, events.note("the range was the wall behind it",
                                          corrects=wrong))
        got = store.episode(episode)
        check("the original is still there and still says what it said",
              got["events"][0]["body"]["text"], "ranged at 1.08 m")
        check("...and the correction names it",
              got["events"][1]["corrects"], wrong)
        store.close()


def test_a_correction_must_name_an_event_that_exists() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = store.open_episode("nothing_to_do", world_generation=WORLD)
        try:
            store.append(episode, events.note("correcting thin air", corrects=99))
            check("correcting nothing is refused", "allowed", "refused")
        except ValueError:
            check("correcting nothing is refused", "refused", "refused")
        store.close()


def test_an_episode_number_is_never_handed_out_twice() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        first = [store.open_episode("t", world_generation=WORLD)
                 for _ in range(3)]
        store.close()
        store = a_store(directory)
        more = [store.open_episode("t", world_generation=WORLD)
                for _ in range(2)]
        check("five episodes, five names", len(set(first + more)), 5)
        check("...continuing where the last process left off",
              refs.parse(more[0]).local, "episode:4")
        store.close()


def test_the_record_survives_the_process_that_wrote_it() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        generation = store.generation()
        store.close()

        store = a_store(directory)
        check("the same generation, not a new one", store.generation(), generation)
        got = store.episode(episode)
        check("...and the episode is all there", len(got["events"]), 8)
        check("...with its evidence readable",
              store.evidence_bytes(refs.digest(PICTURE)), PICTURE)
        store.close()


def test_an_episode_says_which_world_it_was_talking_about() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        known = an_episode(store)
        unknown = an_episode(store, generation=None, keep_evidence=False)
        check("the one that knew", store.episode(known)["world_generation"],
              WORLD)
        check("the one that did not",
              store.episode(unknown)["world_generation"], refs.UNKNOWN)
        check("and they can be told apart without parsing anything",
              [one["ref"] for one in store.episodes(world_generation=WORLD)],
              [known])
        store.close()


def test_a_generation_that_is_not_one_is_refused() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            store.open_episode("t", world_generation="session-3")
            check("a bad generation is refused", "allowed", "refused")
        except ValueError:
            check("a bad generation is refused", "refused", "refused")
        store.close()


# --- the decision inputs -----------------------------------------------------

def test_a_snapshot_is_named_by_what_is_in_it() -> None:
    """Which is what makes snapshotting every decision affordable."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        first = store.snapshot("world_state", a_world())
        again = store.snapshot("world_state", a_world())
        check("the same world twice is one row", first, again)
        check("...and one row it is", store.summary()["snapshots"], 1)
        check("...and it reads back as it went in",
              store.snapshot_body(first), a_world())
        store.close()


def test_a_snapshot_digest_does_not_depend_on_dictionary_order() -> None:
    """Two identical worlds built in different orders are the same world."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        one = store.snapshot("world_state", {"a": 1, "b": [2, 3]})
        two = store.snapshot("world_state", {"b": [2, 3], "a": 1})
        check("same content, same name", one, two)
        store.close()


def test_a_decision_cannot_be_recorded_against_a_live_dictionary() -> None:
    """The mistake this whole component exists to prevent, refused at the door."""
    try:
        events.decision("look_at(object:8)", "because", a_world())
        check("a decision from a live dictionary is refused", "allowed",
              "refused")
    except ValueError:
        check("a decision from a live dictionary is refused", "refused",
              "refused")


# --- evidence ----------------------------------------------------------------

def test_evidence_is_kept_once_however_often_it_is_seen() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        first = store.keep_evidence("frame", PICTURE)
        again = store.keep_evidence("frame", PICTURE, source={"frame_id": "2"})
        check("the same picture twice is one name", first, again)
        check("...and one row", store.summary()["evidence"], 1)
        check("...and the bytes come back", store.evidence_bytes(first), PICTURE)
        check("...counted once", store.summary()["evidence_bytes"], len(PICTURE))
        store.close()


def test_a_deleted_picture_reads_as_deleted_and_says_who_asked() -> None:
    """Never as absent, and never quietly replaced. This is the requirement."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        digest = store.keep_evidence("frame", PICTURE)
        got = store.delete_evidence(digest, "the owner asked for it to go",
                                    detail="a person was in the picture")
        check("the state says deleted", got["state"], DELETED)
        check("...with the reason", got["why"], "the owner asked for it to go")
        check("...and the bytes really are gone",
              store.evidence_bytes(digest), None)
        check("...and it still reads as deleted after the fact",
              store.evidence_state(digest)["state"], DELETED)
        store.close()


def test_deleting_evidence_needs_a_reason() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        digest = store.keep_evidence("frame", PICTURE)
        try:
            store.delete_evidence(digest, "")
            check("a deletion without a reason is refused", "allowed", "refused")
        except ValueError:
            check("a deletion without a reason is refused", "refused", "refused")
        check("...and the picture is still there",
              store.evidence_state(digest)["state"], HELD)
        store.close()


def test_evidence_that_was_never_recorded_reads_as_absent() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        check("absent, not deleted",
              store.evidence_state(refs.digest(b"never seen"))["state"], ABSENT)
        store.close()


def test_evidence_taken_away_behind_the_store_s_back_says_so() -> None:
    """A recording that has come apart must not read like a deletion somebody
    asked for."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        digest = store.keep_evidence("frame", PICTURE)
        os.remove(store._evidence_path(digest))
        got = store.evidence_state(digest)
        check("absent rather than deleted", got["state"], ABSENT)
        check("...and it says the file went missing", got["why"],
              "file is missing")
        store.close()


def test_a_half_written_picture_never_takes_a_good_one_s_name() -> None:
    """The file is renamed into place, so the only file under a digest is one
    whose bytes hash to it."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        digest = store.keep_evidence("frame", PICTURE)
        path = store._evidence_path(digest)
        check("the file is named after its contents",
              refs.digest(open(path, "rb").read()), digest)
        check("...and nothing else is lying about in the directory",
              sorted(os.listdir(os.path.dirname(path))),
              [refs.parse(digest).local])
        store.close()


def test_a_digest_is_the_only_thing_evidence_can_be_asked_for_by() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            store.evidence_state("../../etc/passwd")
            check("a path is not a digest", "allowed", "refused")
        except ValueError:
            check("a path is not a digest", "refused", "refused")
        store.close()


# --- what happened to identity afterwards ------------------------------------

def test_a_merge_is_recorded_beside_the_episode_not_inside_it() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        eight = refs.world(WORLD, "object:8")
        store.alias("merge", eight, refs.world(WORLD, "object:12"),
                    note="the same painting from the other side")
        check("the thing is now called something else",
              store.now_called(eight), [refs.world(WORLD, "object:12")])
        check("...and the episode still says what it said",
              store.episode(episode)["events"][0]["refs"], [eight])
        store.close()


def test_a_thing_taken_apart_has_more_than_one_answer() -> None:
    """Which is the truthful shape. Three of the twenty-one most-looked-at things
    on the 2026-09-08 drive were two objects each, so this is not hypothetical.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        eight = refs.world(WORLD, "object:8")
        store.alias("split", eight, refs.world(WORLD, "object:106"),
                    note="the painting")
        store.alias("split", eight, refs.world(WORLD, "object:107"),
                    note="the dining chair in front of it")
        check("both answers, in the order they were recorded",
              store.now_called(eight),
              [refs.world(WORLD, "object:106"), refs.world(WORLD, "object:107")])
        store.close()


def test_following_a_chain_of_merges_ends_somewhere() -> None:
    """Even when two passes have merged two things into each other."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        one, two = refs.world(WORLD, "object:1"), refs.world(WORLD, "object:2")
        store.alias("merge", one, two)
        store.alias("merge", two, one)
        check("a cycle returns something rather than spinning",
              len(store.now_called(one)) > 0, True)
        store.close()


def test_a_name_that_is_not_one_cannot_be_aliased() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            store.alias("merge", "object:8", "object:12")
            check("a bare local name is refused", "allowed", "refused")
        except ValueError:
            check("a bare local name is refused", "refused", "refused")
        store.close()


# --- the schema over time ----------------------------------------------------

def test_a_recording_survives_a_column_arriving_under_it() -> None:
    """The first migration, run before there has been one.

    There is no older build to test against -- this component has shipped one
    schema -- so the test makes the situation instead: a database whose
    `episodes` table is today's minus `note`, a recording already in it, and
    `note` registered as a column a later build wants. That is exactly the shape
    the first real migration will have, and what it has to prove is that the row
    written by the older build is still readable afterwards.

    `ADDED_COLUMNS` is put back at the end, because the store holds the same
    dictionary and a leaked entry would follow every check after this one.
    """
    import schema

    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, "episodes.db")
        db = sqlite3.connect(path)
        db.executescript(f"""
            CREATE TABLE meta (key TEXT PRIMARY KEY, value TEXT NOT NULL);
            CREATE TABLE episodes (
                id               INTEGER PRIMARY KEY,
                ref              TEXT NOT NULL UNIQUE,
                opened_at        REAL NOT NULL,
                trigger          TEXT NOT NULL,
                trigger_json     TEXT,
                world_generation TEXT NOT NULL,
                map_session      INTEGER);
            INSERT INTO meta VALUES('generation', '{WORLD}');
            INSERT INTO episodes VALUES(1, 'au/{WORLD}/episode:1',
                1757320000.0, 'nothing_to_do', NULL, '{WORLD}', 7);
        """)
        db.commit()
        db.close()

        was = dict(schema.ADDED_COLUMNS)
        schema.ADDED_COLUMNS["episodes"] = {"note": "TEXT NOT NULL DEFAULT ''"}
        try:
            store = a_store(directory)
            got = store.episode(f"au/{WORLD}/episode:1")
            check("the recording made before the column existed still reads",
                  got["trigger"], "nothing_to_do")
            check("...and the new column is empty rather than absent",
                  got["note"], "")
            check("...and the store still knows which store it is",
                  store.generation(), WORLD)
            check("...and a new episode carries on from it",
                  refs.parse(store.open_episode("t",
                                                world_generation=WORLD)).local,
                  "episode:2")
            store.close()
        finally:
            schema.ADDED_COLUMNS.clear()
            schema.ADDED_COLUMNS.update(was)


def test_a_column_a_later_build_wants_is_added_to_a_table_that_exists() -> None:
    """The migration machinery itself, exercised directly.

    There has been no schema change yet, so `ADDED_COLUMNS` is empty and there is
    no real migration for a test to run through. This checks the mechanism that
    will carry the first one, which is the honest thing to claim.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        store._add_columns("episodes", {"weather": "TEXT"})
        have = {row["name"] for row in
                store.db.execute("PRAGMA table_info(episodes)")}
        check("the column arrives", "weather" in have, True)
        store._add_columns("episodes", {"weather": "TEXT"})
        check("...and adding it twice is not an error", "weather" in have, True)
        store.close()


def test_unreadable_stored_json_cannot_take_the_reading_down() -> None:
    """One bad row is reported beside the good ones."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = store.open_episode("nothing_to_do", world_generation=WORLD)
        store.append(episode, events.note("fine"))
        store.db.execute("INSERT INTO events(episode_id, seq, at, kind,"
                         " body_json) VALUES(1, 99, 1.0, 'note', '{oh dear')")
        store.db.commit()
        got = store.episode(episode)
        check("both rows come back", len(got["events"]), 2)
        check("...and the broken one is empty rather than fatal",
              got["events"][-1]["body"], {})
        store.close()


def test_an_event_that_could_not_be_read_back_is_refused() -> None:
    """At the moment somebody can still fix it, rather than at replay."""
    for kind, body in (("decision", {"chose": "x"}),
                       ("call", {"call": "look_at"}),
                       ("closed", {"outcome": "went well"}),
                       ("nonsense", {})):
        try:
            events.validate(kind, body)
            check(f"a bad {kind} is refused", "allowed", "refused")
        except ValueError:
            check(f"a bad {kind} is refused", "refused", "refused")


def test_a_measurement_the_rover_could_not_make_is_not_recorded_as_zero() -> None:
    """A null battery and a flat one must not read alike."""
    got = events.measured("the attempt", duration_s=2.4, battery_v=None,
                          travel_m=0.0)
    check("the reading that exists is kept", got.body["duration_s"], 2.4)
    check("...including a real zero", got.body["travel_m"], 0.0)
    check("...and the one that does not is absent",
          "battery_v" in got.body, False)



#: The schema exactly as version 1 shipped it, kept here so that the migration
#: to version 2 is tested against what the rover really has rather than against
#: today's schema with a table taken out. It is frozen: when version 3 arrives,
#: this stays as it is and a second frozen copy joins it.
SCHEMA_V1 = """
    CREATE TABLE IF NOT EXISTS meta (
        key   TEXT PRIMARY KEY,
        value TEXT NOT NULL
    );
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
    CREATE TABLE IF NOT EXISTS snapshots (
        digest    TEXT PRIMARY KEY,
        kind      TEXT NOT NULL,
        taken_at  REAL NOT NULL,
        bytes     INTEGER NOT NULL,
        body_json TEXT NOT NULL
    );
    CREATE TABLE IF NOT EXISTS evidence (
        digest      TEXT PRIMARY KEY,
        kind        TEXT NOT NULL,
        bytes       INTEGER NOT NULL,
        stored_at   REAL NOT NULL,
        source_json TEXT
    );
    CREATE TABLE IF NOT EXISTS deletions (
        id     INTEGER PRIMARY KEY AUTOINCREMENT,
        digest TEXT NOT NULL,
        at     REAL NOT NULL,
        why    TEXT NOT NULL,
        detail TEXT NOT NULL DEFAULT ''
    );
    CREATE TABLE IF NOT EXISTS aliases (
        id       INTEGER PRIMARY KEY AUTOINCREMENT,
        at       REAL NOT NULL,
        kind     TEXT NOT NULL,
        from_ref TEXT NOT NULL,
        to_ref   TEXT NOT NULL,
        note     TEXT NOT NULL DEFAULT ''
    );
"""


def test_a_version_one_recording_opens_and_carries_on() -> None:
    """The real migration, against the schema that really shipped.

    Version 1 went to the rover on 2026-09-08 and version 2 followed it the same
    day, adding `marks` and `pins`. What has to survive that is a recording made
    by the older build: its episodes still read, its evidence is still found, and
    the store can say it has been through both versions rather than only claiming
    to be the newer one.
    """
    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, "episodes.db")
        db = sqlite3.connect(path)
        db.executescript(SCHEMA_V1)
        db.execute("INSERT INTO meta VALUES('schema_version', '1')")
        db.execute("INSERT INTO meta VALUES('generation', ?)", (WORLD,))
        db.execute(
            "INSERT INTO episodes(id, ref, opened_at, trigger, world_generation,"
            " map_session, note) VALUES(1, ?, 1757320000.0, 'the rover looked',"
            " ?, 7, '')", (f"au/{WORLD}/episode:1", WORLD))
        db.execute(
            "INSERT INTO events(episode_id, seq, at, kind, body_json)"
            " VALUES(1, 1, 1757320000.0, 'note', '{\"text\": \"from version one\"}')")
        db.commit()
        db.close()

        store = a_store(directory)
        got = store.episode(f"au/{WORLD}/episode:1")
        check("the version one episode still reads", got["trigger"],
              "the rover looked")
        check("...and its event with it", got["events"][0]["body"]["text"],
              "from version one")
        check("...the store is now at version two", store.schema_version(), 2)
        check("...and still says which version made it",
              store.created_version(), 1)
        check("...with both versions in its history",
              [one["value"] for one in store.marks("schema_version")], ["2"])

        # The tables version 2 added are usable on the migrated database.
        store.mark("last_inference", 4242)
        store.pin(f"au/{WORLD}/episode:1", "kept for the write-up")
        check("...the new tables work", store.marked("last_inference"), "4242")
        check("...and pinning an old episode is allowed",
              store.pinned(), [f"au/{WORLD}/episode:1"])
        check("...and a new episode carries on from the old numbering",
              refs.parse(store.open_episode("t", world_generation=WORLD)).local,
              "episode:2")
        store.close()

def _raw_episode(store: EpisodeStore, ref: str) -> tuple:
    row = store.db.execute("SELECT * FROM episodes WHERE ref = ?",
                           (ref,)).fetchone()
    return tuple(row)


TESTS = (
    test_an_empty_database_is_an_ordinary_thing_to_open,
    test_nothing_in_the_record_is_ever_rewritten,
    test_an_episode_keeps_what_it_said_when_it_closed,
    test_an_episode_cannot_be_closed_twice,
    test_something_noticed_afterwards_is_still_recorded,
    test_a_correction_points_at_what_it_corrects,
    test_a_correction_must_name_an_event_that_exists,
    test_an_episode_number_is_never_handed_out_twice,
    test_the_record_survives_the_process_that_wrote_it,
    test_an_episode_says_which_world_it_was_talking_about,
    test_a_generation_that_is_not_one_is_refused,
    test_a_snapshot_is_named_by_what_is_in_it,
    test_a_snapshot_digest_does_not_depend_on_dictionary_order,
    test_a_decision_cannot_be_recorded_against_a_live_dictionary,
    test_evidence_is_kept_once_however_often_it_is_seen,
    test_a_deleted_picture_reads_as_deleted_and_says_who_asked,
    test_deleting_evidence_needs_a_reason,
    test_evidence_that_was_never_recorded_reads_as_absent,
    test_evidence_taken_away_behind_the_store_s_back_says_so,
    test_a_half_written_picture_never_takes_a_good_one_s_name,
    test_a_digest_is_the_only_thing_evidence_can_be_asked_for_by,
    test_a_merge_is_recorded_beside_the_episode_not_inside_it,
    test_a_thing_taken_apart_has_more_than_one_answer,
    test_following_a_chain_of_merges_ends_somewhere,
    test_a_name_that_is_not_one_cannot_be_aliased,
    test_a_recording_survives_a_column_arriving_under_it,
    test_a_version_one_recording_opens_and_carries_on,
    test_a_column_a_later_build_wants_is_added_to_a_table_that_exists,
    test_unreadable_stored_json_cannot_take_the_reading_down,
    test_an_event_that_could_not_be_read_back_is_refused,
    test_a_measurement_the_rover_could_not_make_is_not_recorded_as_zero,
)
