"""The few lines a person or a model reads: that they do not overclaim.

The check that matters is the negative one. An episode whose evidence has been
deleted, or whose references belong to a world that has been cleared, must say so
on the face of the summary -- because a summary is what somebody will read
instead of the record, and a confident account of something nobody can check any
more is the failure this component exists to prevent.
"""
from __future__ import annotations

import tempfile

import refs
import summary
from test_fakes import CLEARED, PICTURE, WORLD, a_store, an_episode
from test_harness import check


def test_a_summary_says_what_the_rover_did_and_why() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        got = summary.of(store, an_episode(store), live_world_generation=WORLD)
        for wanted in ("episode:1", "nothing_to_do", "considered 2 goals",
                       "chose look_at(object:8)",
                       "one look is a bearing and not a position",
                       "called look_at(entity=object:8, pan_deg=-20.0) -- ok",
                       "closed: abandoned -- shadow mode: no movement authority",
                       "2 pieces of evidence kept"):
            check(f"the summary says {wanted!r}", wanted in got, True)
        store.close()


def test_a_summary_of_a_cleared_world_says_nothing_can_be_looked_up() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        got = summary.of(store, an_episode(store),
                         live_world_generation=CLEARED)
        check("it says the reference cannot be looked up",
              "object:8 cannot be looked up" in got, True)
        check("...and why", "has since been cleared" in got, True)
        store.close()


def test_a_summary_of_an_episode_with_no_generation_says_that_too() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        got = summary.of(store, an_episode(store, generation=None,
                                           keep_evidence=False),
                         live_world_generation=WORLD)
        check("the world it ran in is not known",
              "world state not known" in got, True)
        store.close()


def test_a_summary_admits_a_deleted_picture() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        store.delete_evidence(refs.digest(PICTURE), "the owner asked")
        got = summary.of(store, episode)
        check("it says the evidence was deleted",
              "the evidence was deleted" in got, True)
        check("...with the reason", "the owner asked" in got, True)
        check("...and that the episode is no longer fully replayable",
              "no longer be replayed in full" in got, True)
        store.close()


def test_a_summary_says_what_a_thing_is_called_now() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        store.alias("merge", refs.world(WORLD, "object:8"),
                    refs.world(WORLD, "object:12"))
        got = summary.of(store, episode, live_world_generation=WORLD)
        check("it says the thing has become another",
              "object:8 is now object:12" in got, True)
        store.close()


def test_an_open_episode_is_not_reported_as_finished() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = store.open_episode("nothing_to_do", world_generation=WORLD)
        got = summary.of(store, episode)
        check("still open", "still open" in got, True)
        check("...and it decided nothing", "decided nothing" in got, True)
        store.close()


def test_the_recent_list_is_one_line_an_episode() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        an_episode(store)
        store.open_episode("a_person_asked", world_generation=WORLD)
        got = summary.recent(store).splitlines()
        check("two episodes, two lines", len(got), 2)
        check("newest first", "a_person_asked" in got[0], True)
        check("...and the open one says so", got[0].endswith("open"), True)
        check("...and the closed one says how it ended",
              got[1].endswith("abandoned"), True)
        store.close()


def test_an_empty_record_says_so_rather_than_printing_nothing() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        check("no episodes recorded", summary.recent(store),
              "no episodes recorded")
        store.close()


TESTS = (
    test_a_summary_says_what_the_rover_did_and_why,
    test_a_summary_of_a_cleared_world_says_nothing_can_be_looked_up,
    test_a_summary_of_an_episode_with_no_generation_says_that_too,
    test_a_summary_admits_a_deleted_picture,
    test_a_summary_says_what_a_thing_is_called_now,
    test_an_open_episode_is_not_reported_as_finished,
    test_the_recent_list_is_one_line_an_episode,
    test_an_empty_record_says_so_rather_than_printing_nothing,
)
