"""Rebuilding an episode: from the record alone, and honest about what is gone.

The four acceptance questions this answers are the ones about time passing. Can
the chosen action still be rebuilt after the world state has been cleared, after
the thing it chose has been merged into something else, after the pictures have
been deleted -- and does the account say so, rather than quietly finding
something newer with the same name.
"""
from __future__ import annotations

import os
import re
import tempfile

import events
import refs
import replay
from store import DELETED
from test_fakes import CLEARED, PICTURE, WORLD, a_store, a_world, an_episode
from test_harness import check


def test_the_chosen_action_is_rebuilt_from_stored_records_alone() -> None:
    """Criterion 2, asked directly: nothing but the database is open."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        got = replay.selected_action(store, episode)
        check("the action", got["action"], "look_at(object:8)")
        check("...its parameters",
              got["params"], {"entity": "object:8", "pan_deg": -20.0})
        check("...its result", got["result"], {"observations": 1,
                                               "ranged": False})
        check("...and how the episode ended", got["outcome"]["outcome"],
              "abandoned")
        store.close()


def test_a_replay_has_nothing_to_drive_the_rover_with() -> None:
    """Criterion 3, made structural.

    A text check on the module's imports. It proves that this build has no path
    to the hardware, which is a smaller claim than "replay is safe" and the one
    worth mechanising: the way this stops being true is somebody importing a
    client for one convenient lookup.
    """
    source = open(os.path.join(os.path.dirname(__file__), "replay.py"),
                  encoding="utf-8").read()
    imported = set(re.findall(r"^(?:from|import)\s+([A-Za-z_][\w.]*)",
                              source, re.M))
    check("replay imports nothing but the record and the standard library",
          sorted(imported - {"__future__", "typing"}),
          ["events", "refs", "store"])


def test_replaying_an_episode_twice_gives_the_same_answer() -> None:
    """Deterministic because it is a read, and it writes nothing that a later
    replay could see."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        first = replay.reconstruct(store, episode)
        counted = store.summary()
        second = replay.reconstruct(store, episode)
        check("the same reconstruction", first, second)
        check("...and replaying wrote nothing", store.summary(), counted)
        store.close()


def test_the_world_it_chose_from_is_the_world_it_saw() -> None:
    """Not today's. The snapshot is the whole of the difference."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        got = replay.reconstruct(store, episode)
        check("the decision inputs come back whole",
              got["decision"]["inputs"], a_world())
        check("...naming the thing that had one look",
              got["decision"]["inputs"]["things"][0]["looks"], 1)
        store.close()


def test_a_cleared_world_cannot_redirect_an_episode() -> None:
    """Criterion 7, and the fault it is really about.

    The rover's store was cleared and `object:8` was handed to something else.
    The episode still reads correctly, and its reference is reported as one that
    must not be looked up -- rather than resolving against a stranger.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        got = replay.reconstruct(store, episode, live_world_generation=CLEARED)
        check("the chosen action is unchanged",
              got["decision"]["chose"], "look_at(object:8)")
        eight = [one for one in got["references"]
                 if one["local"] == "object:8"][0]
        check("...and its reference is not resolvable", eight["resolvable"],
              False)
        check("...because the store it named has been cleared",
              "has since been cleared" in eight["why_not"], True)
        check("...while the episode is still fully replayable",
              got["replayable"], True)
        store.close()


def test_the_same_reference_resolves_while_its_world_is_still_live() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        got = replay.reconstruct(store, episode, live_world_generation=WORLD)
        eight = [one for one in got["references"]
                 if one["local"] == "object:8"][0]
        check("resolvable against its own store", eight["resolvable"], True)
        check("...with nothing to explain", eight["why_not"], "")
        store.close()


def test_an_episode_from_before_the_generation_existed_says_so() -> None:
    """Every episode recorded until `world_state` mints a generation is in this
    state, and it is reported rather than assumed away."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store, generation=None, keep_evidence=False)
        got = replay.reconstruct(store, episode, live_world_generation=WORLD)
        eight = [one for one in got["references"]
                 if one["local"] == "object:8"][0]
        check("not resolvable", eight["resolvable"], False)
        check("...and it says why",
              "before the world state could say" in eight["why_not"], True)
        check("...and the account of what happened is still complete",
              got["decision"]["chose"], "look_at(object:8)")
        store.close()


def test_a_merged_thing_is_reported_as_what_it_became() -> None:
    """The episode is not rewritten. The reader is told both."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        store.alias("merge", refs.world(WORLD, "object:8"),
                    refs.world(WORLD, "object:12"))
        got = replay.reconstruct(store, episode, live_world_generation=WORLD)
        eight = [one for one in got["references"]
                 if one["local"] == "object:8"][0]
        check("the episode still says object:8", eight["local"], "object:8")
        check("...and that it is now object:12",
              [refs.local_id(one) for one in eight["now_called"]], ["object:12"])
        store.close()


def test_a_deleted_picture_makes_the_episode_say_so() -> None:
    """Criterion 8. Reported as deleted, with the reason, and never replaced."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        store.delete_evidence(refs.digest(PICTURE),
                              "the owner asked for it to go")
        got = replay.reconstruct(store, episode)
        check("no longer fully replayable", got["replayable"], False)
        check("...and one thing is missing", len(got["missing"]), 1)
        check("...deleted, not absent", got["missing"][0]["state"], DELETED)
        check("...with the reason the owner gave",
              got["missing"][0]["why"], "the owner asked for it to go")
        check("...while what the rover decided is still readable",
              got["decision"]["chose"], "look_at(object:8)")
        store.close()


def test_a_missing_snapshot_makes_the_decision_uncheckable() -> None:
    """An episode whose inputs are gone is still an account of what happened; it
    is no longer an account anybody can check, and it says which."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = store.open_episode("nothing_to_do", world_generation=WORLD)
        store.append(episode, events.decision(
            "look_at(object:8)", "because", refs.digest(b"a world nobody kept")))
        store.close_episode(episode, "abandoned")
        got = replay.reconstruct(store, episode)
        check("not replayable", got["replayable"], False)
        check("...and it is the snapshot that is missing",
              got["missing"][0]["what"], "snapshot")
        check("...while the choice itself is still there",
              got["decision"]["chose"], "look_at(object:8)")
        store.close()


def test_hindsight_is_marked_as_hindsight() -> None:
    """An annotation added after the close must not read as something the rover
    knew at the time."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        store.append(episode, events.note("the owner says it was the painting"))
        got = replay.reconstruct(store, episode)
        check("one annotation", len(got["annotations"]), 1)
        check("...marked as after the close",
              got["annotations"][0]["after_close"], True)
        check("...and everything before it is not",
              any(step["after_close"] for step in got["steps"][:-1]), False)
        store.close()


def test_a_correction_is_shown_next_to_what_it_corrects() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = store.open_episode("nothing_to_do", world_generation=WORLD)
        wrong = store.append(episode, events.note("ranged at 1.08 m"))
        store.append(episode, events.note("that was the wall behind it",
                                          corrects=wrong))
        got = replay.reconstruct(store, episode)
        check("the original still says what it said",
              got["steps"][0]["body"]["text"], "ranged at 1.08 m")
        check("...and points at what corrects it",
              got["steps"][0]["corrected_by"], [2])
        store.close()


def test_a_shadow_run_that_chose_and_did_not_act_is_a_result() -> None:
    """Phase 1 has no movement authority, so this is the ordinary case and must
    not read as a gap in the record."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = store.open_episode("nothing_to_do", world_generation=WORLD)
        snapshot = store.snapshot("world_state", a_world())
        store.append(episode, events.decision(
            "drive_to(object:8)", "nothing else is worth looking at", snapshot))
        store.close_episode(episode, "abandoned",
                            detail="shadow mode: no movement authority")
        got = replay.selected_action(store, episode)
        check("the action it would have taken", got["action"],
              "drive_to(object:8)")
        check("...and that it never called anything", got["called"], False)
        check("...which is not a failure", got["outcome"]["outcome"],
              "abandoned")
        store.close()


def test_walking_an_episode_gives_its_steps_in_order() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        episode = an_episode(store)
        kinds = [step["kind"] for step in replay.play(store, episode)]
        check("the order it happened in", kinds,
              ["candidate", "candidate", "decision", "model", "call",
               "world_change", "measured", "closed"])
        store.close()


TESTS = (
    test_the_chosen_action_is_rebuilt_from_stored_records_alone,
    test_a_replay_has_nothing_to_drive_the_rover_with,
    test_replaying_an_episode_twice_gives_the_same_answer,
    test_the_world_it_chose_from_is_the_world_it_saw,
    test_a_cleared_world_cannot_redirect_an_episode,
    test_the_same_reference_resolves_while_its_world_is_still_live,
    test_an_episode_from_before_the_generation_existed_says_so,
    test_a_merged_thing_is_reported_as_what_it_became,
    test_a_deleted_picture_makes_the_episode_say_so,
    test_a_missing_snapshot_makes_the_decision_uncheckable,
    test_hindsight_is_marked_as_hindsight,
    test_a_correction_is_shown_next_to_what_it_corrects,
    test_a_shadow_run_that_chose_and_did_not_act_is_a_result,
    test_walking_an_episode_gives_its_steps_in_order,
)
