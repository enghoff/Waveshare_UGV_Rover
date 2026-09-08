"""Reading the record back: that it finds things, and admits what it cannot show.

A record nobody can read is worth less than one nobody keeps, so what is checked
here is that an episode can be found by the short name a person would read out
loud, that a picture retention has taken is reported rather than silently
skipped, and that nothing in this path touches the rover.
"""
from __future__ import annotations

import os
import re
import tempfile

import refs
import retention
import review
from test_fakes import (PICTURE, WORLD, FakeRover, a_look, a_store,
                        a_thing, an_episode)
from test_goals import ROOM
from test_harness import check


def _run(argv, directory):
    """Run the reader and give back everything it printed."""
    import io
    import contextlib

    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        code = review.main([*argv, "--dir", directory])
    return code, out.getvalue()


def test_reading_the_record_cannot_reach_the_rover() -> None:
    """The same structural check `replay` carries, for the same reason: this is
    the module somebody will reach into for one convenient live lookup."""
    source = open(os.path.join(os.path.dirname(__file__), "review.py"),
                  encoding="utf-8").read()
    imported = set(re.findall(r"^(?:from|import)\s+([A-Za-z_][\w.]*)",
                              source, re.M))
    check("it imports nothing that could talk to the rover",
          sorted(imported - {"__future__", "typing", "argparse", "os", "sys",
                             "time", "io", "contextlib"}),
          ["refs", "replay", "retention", "store", "summary"])


def test_the_listing_shows_what_happened() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE},
                          said=[{"seq": 1, "phase": "driving",
                                 "kind": "drive_to"},
                                {"seq": 2, "phase": "ended", "kind": "drive_to",
                                 "reason": "arrived"}])
        from recorder import Recorder
        Recorder(store, rover).poll()
        store.close()

        code, said = _run([], directory)
        check("it runs", code, 0)
        check("...listing both episodes", "2 episode(s) match" in said, True)
        check("...naming the move by its kind", "drive_to" in said, True)
        check("...and saying how it ended", "succeeded" in said, True)
        check("...and the look by what it found", "2 region(s)" in said, True)


def test_the_listing_can_be_narrowed() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE},
                          said=[{"seq": 1, "phase": "driving", "kind": "drive_to"}])
        from recorder import Recorder
        Recorder(store, rover).poll()
        store.close()

        _, moves = _run(["--moves"], directory)
        check("only the driving", "1 episode(s) match" in moves, True)
        check("...and it is the move", "drive_to" in moves, True)
        _, looks = _run(["--looks"], directory)
        check("only the looking", "1 episode(s) match" in looks, True)
        _, none = _run(["--world", "0000000000000000"], directory)
        check("a world nothing belongs to says so",
              "nothing in the record matches" in none, True)


def test_an_episode_is_found_by_the_name_a_person_reads_out() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        an_episode(store)
        store.close()
        code, said = _run(["episode:1"], directory)
        check("found", code, 0)
        check("...and shown in full",
              "chose look_at(object:8)" in said, True)
        check("...with its steps numbered", "step by step:" in said, True)
        check("...and the things it named",
              "object:8" in said, True)


def test_a_name_that_is_not_there_says_so_rather_than_failing_quietly() -> None:
    with tempfile.TemporaryDirectory() as directory:
        a_store(directory).close()
        code, said = _run(["episode:99"], directory)
        check("it refuses", code, 1)
        check("...and says how to find out what there is",
              "lists what there is" in said, True)


def test_a_picture_can_be_written_out_to_be_looked_at() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        an_episode(store)
        store.close()
        into = os.path.join(directory, "look")
        _, said = _run(["episode:1", "--save", into], directory)
        check("it wrote them", "wrote 2 picture(s)" in said, True)
        check("...and they are there", len(os.listdir(into)), 2)
        check("...named after the episode",
              all(one.startswith("episode-1-") for one in os.listdir(into)),
              True)


def test_a_picture_retention_took_is_reported_not_skipped() -> None:
    """The failure this avoids is a reader who asks for the pictures, gets two
    of three, and never learns that the third was deleted."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        an_episode(store)
        store.delete_evidence(refs.digest(PICTURE), "the owner asked")
        store.close()
        into = os.path.join(directory, "look")
        _, said = _run(["episode:1", "--save", into], directory)
        check("one written", "wrote 1 picture(s)" in said, True)
        check("...and the other accounted for",
              "cannot be written out" in said, True)
        check("...with the reason", "the owner asked" in said, True)


def test_an_episode_says_whether_retention_may_take_it() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        ref = an_episode(store)
        store.close()
        _, before = _run(["episode:1"], directory)
        check("not pinned, and it says so", "not pinned" in before, True)

        store = a_store(directory)
        store.pin(ref, "the acceptance recording")
        store.close()
        _, after = _run(["episode:1"], directory)
        check("pinned, and it says so", "pinned against retention" in after, True)


def test_the_stats_say_which_worlds_the_record_spans() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        an_episode(store)
        an_episode(store, generation="0011223344556677", keep_evidence=False)
        store.close()
        _, said = _run(["--stats"], directory)
        check("both worlds are listed", WORLD in said, True)
        check("...and the other", "0011223344556677" in said, True)
        check("...with the warning that spans them",
              "can only be looked up against its own" in said, True)
        check("...and what the record would cost to keep",
              retention.DEFAULT.describe() in said, True)


def test_an_unreadable_episode_is_marked_in_the_listing() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        an_episode(store)
        store.delete_evidence(refs.digest(PICTURE), "the owner asked")
        store.close()
        _, said = _run(["--broken"], directory)
        check("it can be listed on its own", "1 episode(s) match" in said, True)


def test_the_deliberations_can_be_listed_on_their_own() -> None:
    """The one kind of episode that contains a choice is worth finding."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(room=ROOM, rows=a_look(1, 100),
                          frames={"frame-1": PICTURE},
                          entities=[a_thing("object:8", 1.6, 1.0,
                                            uncertainty_m=0.60,
                                            major_deg=90.0)])
        from recorder import Recorder
        recorder = Recorder(store, rover)
        recorder.poll()
        recorder.consider()
        store.close()

        code, said = _run(["--decisions"], directory)
        check("it runs", code, 0)
        check("...listing the deliberation alone", "1 episode(s) match" in said,
              True)
        check("...under a word of its own", "decided" in said, True)
        check("...with how many goals it weighed", "candidate(s)" in said, True)


TESTS = (
    test_reading_the_record_cannot_reach_the_rover,
    test_the_listing_shows_what_happened,
    test_the_listing_can_be_narrowed,
    test_an_episode_is_found_by_the_name_a_person_reads_out,
    test_a_name_that_is_not_there_says_so_rather_than_failing_quietly,
    test_a_picture_can_be_written_out_to_be_looked_at,
    test_a_picture_retention_took_is_reported_not_skipped,
    test_an_episode_says_whether_retention_may_take_it,
    test_the_stats_say_which_worlds_the_record_spans,
    test_an_unreadable_episode_is_marked_in_the_listing,
    test_the_deliberations_can_be_listed_on_their_own,
)
