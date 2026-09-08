"""The shadow run: that it misses nothing, repeats nothing, and cannot drive.

Two failures would make a shadow run worthless, and neither is loud. A recorder
that silently skipped what happened between two polls would leave a record with
holes in it that read like a rover doing nothing; a recorder that recorded the
same look twice on every restart would leave one that reads like a rover doing
everything twice. Both are checked here, and so is the one that would be worse
than worthless: a recorder that could move the rover it is supposed to be
watching.
"""
from __future__ import annotations

import tempfile

import client
import refs
import replay
import summary
from recorder import LOOK_MARK, MOVE_MARK, Recorder
from test_fakes import (CLEARED, PICTURE, WORLD, FakeRover, a_look, a_store)
import retention
from test_harness import check


def test_a_recorder_cannot_move_the_rover() -> None:
    """The structural half of "no movement-capable path", asked by trying.

    Every call a shadow run could plausibly reach for, refused before a socket
    is opened -- so a recorder pointed at a daemon that would happily drive
    still cannot ask it to.
    """
    rover = FakeRover()
    for name in sorted(client.MOVES):
        try:
            rover.call(name, {})
            check(f"{name} is refused", "allowed", "refused")
        except client.Refused:
            check(f"{name} is refused", "refused", "refused")
    check("...and none of them reached the transport", rover.asked, [])
    check("...while a read goes through",
          rover.call("world_state_summary")["ok"], True)


def test_world_state_inspect_is_not_a_call_a_recorder_may_make() -> None:
    """It reads nothing -- it turns the gimbal and takes a picture. A recorder
    that triggered looks would be part of the work rather than watching it."""
    rover = FakeRover()
    for name in ("world_inspect", "world_state_clear", "clear_map",
                 "refit_pose", "set_vision"):
        try:
            rover.call(name, {})
            check(f"{name} is refused", "allowed", "refused")
        except client.Refused:
            check(f"{name} is refused", "refused", "refused")


def test_one_episode_per_look() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100) + a_look(2, 200),
                          frames={"frame-1": PICTURE, "frame-2": PICTURE})
        got = Recorder(store, rover).poll()
        check("two looks, two episodes", got["looks"], 2)
        check("...and that is what the store holds",
              store.summary()["episodes"], 2)
        rows = store.episodes()
        check("...both triggered by the rover looking",
              sorted({one["trigger"] for one in rows}), ["the rover looked"])
        check("...carrying which inspection they were",
              sorted(one["trigger_detail"]["inference_id"] for one in rows),
              [1, 2])
        store.close()


def test_a_look_is_never_recorded_twice() -> None:
    """The mark is what makes a restart continue rather than repeat."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE})
        Recorder(store, rover).poll()
        again = Recorder(store, rover).poll()
        check("a second recorder finds nothing new", again["looks"], 0)
        check("...and the store still holds one episode",
              store.summary()["episodes"], 1)
        check("...with the mark where the first left it",
              store.marked(LOOK_MARK), "101")
        store.close()


def test_a_recorder_that_was_stopped_catches_up() -> None:
    """Nothing between two polls is lost, because the history is numbered."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE})
        Recorder(store, rover).poll()

        # Three looks happened while nothing was watching.
        rover.rows += a_look(2, 200) + a_look(3, 300) + a_look(4, 400)
        rover.frames.update({"frame-2": PICTURE, "frame-3": PICTURE,
                             "frame-4": PICTURE})
        got = Recorder(store, rover).poll()
        check("all three are picked up", got["looks"], 3)
        check("...oldest first, so the mark only moves over what is written",
              [one["trigger_detail"]["inference_id"]
               for one in store.episodes()][::-1],
              [1, 2, 3, 4])
        store.close()


def test_the_walk_back_is_bounded() -> None:
    """A recorder started against a month of history records the recent past and
    does not spend an hour copying frames nobody asked for."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rows = []
        for n in range(1, 61):
            rows += a_look(n, n * 10, regions=1)
        rover = FakeRover(rows=rows)
        got = Recorder(store, rover, keep_frames=False, catch_up=10).poll()
        check("it stops at the bound", got["looks"], 10)
        check("...taking the newest ten", store.summary()["episodes"], 10)
        store.close()


def test_a_look_carries_its_picture() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE})
        Recorder(store, rover).poll()
        episode = store.episodes()[0]["ref"]
        got = replay.reconstruct(store, episode, live_world_generation=WORLD)
        check("one piece of evidence", len(got["evidence"]), 1)
        check("...which is the picture", got["evidence"][0]["digest"],
              refs.digest(PICTURE))
        check("...and it is held", got["evidence"][0]["state"], "held")
        check("...so the episode is replayable", got["replayable"], True)
        store.close()


def test_a_look_can_be_recorded_without_its_picture() -> None:
    """Which is how the cost of keeping them gets measured."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE})
        Recorder(store, rover, keep_frames=False).poll()
        check("the episode is there", store.summary()["episodes"], 1)
        check("...and nothing was copied", store.summary()["evidence"], 0)
        check("...and the frame was never even fetched",
              "world_state_frame" in rover.asked, False)
        store.close()


def test_what_a_look_saw_is_named_durably() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE})
        Recorder(store, rover).poll()
        episode = store.episodes()[0]["ref"]
        got = replay.reconstruct(store, episode, live_world_generation=WORLD)
        check("both things are named",
              sorted(one["local"] for one in got["references"]),
              ["object:10", "object:11"])
        check("...and they resolve against the store that was live",
              all(one["resolvable"] for one in got["references"]), True)
        after = replay.reconstruct(store, episode,
                                   live_world_generation=CLEARED)
        check("...and against a cleared one, none of them do",
              any(one["resolvable"] for one in after["references"]), False)
        store.close()


def test_a_rover_that_cannot_say_which_world_it_is_still_gets_recorded() -> None:
    """A look is worth recording even when its names will never resolve."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(generation=None, rows=a_look(1, 100),
                          frames={"frame-1": PICTURE})
        got = Recorder(store, rover).poll()
        check("it is recorded", got["looks"], 1)
        episode = store.episodes()[0]
        check("...marked as belonging to no known world",
              episode["world_generation"], refs.UNKNOWN)
        check("...and it says so when read",
              "world state not known" in summary.of(store, episode["ref"]), True)
        store.close()


def test_a_look_that_attached_to_nothing_is_not_a_failure() -> None:
    """The ordinary state until two bearings cross."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100, attached=False),
                          frames={"frame-1": PICTURE})
        Recorder(store, rover).poll()
        episode = store.episodes()[0]["ref"]
        check("closed abandoned rather than failed",
              store.outcome(episode)["outcome"], "abandoned")
        check("...saying why",
              "still unattached" in store.outcome(episode)["detail"], True)
        store.close()


def test_a_summary_of_a_shadow_episode_says_what_the_look_found() -> None:
    """An episode with no decision in it has one piece of content, and a summary
    that omitted it would be four lines saying nothing happened."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE})
        Recorder(store, rover).poll()
        said = summary.of(store, store.episodes()[0]["ref"],
                          live_world_generation=WORLD)
        check("it says how many regions the look found",
              "a look found 2 regions" in said, True)
        check("...how many attached to something known",
              "2 of them attached to a thing the rover already knows" in said,
              True)
        check("...and how many had a distance",
              "1 with a measured distance" in said, True)
        store.close()


def test_a_shadow_episode_decides_nothing_and_says_so() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE})
        Recorder(store, rover).poll()
        episode = store.episodes()[0]["ref"]
        got = replay.reconstruct(store, episode)
        check("no decision", got["decision"], None)
        check("...no calls made on the rover's behalf", got["calls"], [])
        check("...and the summary is honest about it",
              "decided nothing" in summary.of(store, episode), True)
        check("...and there is no action to reconstruct",
              replay.selected_action(store, episode), None)
        store.close()


# --- moves -------------------------------------------------------------------

def test_a_move_is_one_episode_however_many_things_it_says() -> None:
    """The driving loop says something each time a move turns a corner, and all
    of them belong to the one move."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(said=[
            {"seq": 1, "phase": "choosing", "kind": "drive_to",
             "asked": {"x_m": 1.0}, "why": "somebody asked"},
            {"seq": 2, "phase": "turning", "kind": "drive_to",
             "asked": {"x_m": 1.0}},
            {"seq": 3, "phase": "driving", "kind": "drive_to",
             "asked": {"x_m": 1.0}, "route_m": 3.2},
        ])
        recorder = Recorder(store, rover)
        recorder.poll()
        check("one episode, not three", store.summary()["episodes"], 1)
        check("...counted once", recorder.recorded["moves"], 1)
        episode = store.episode(store.episodes()[0]["ref"])
        check("...with a step for each thing it said",
              [one["body"]["phase"] for one in episode["events"]],
              ["choosing", "turning", "driving"])
        check("...and still open, because it has not ended",
              store.outcome(store.episodes()[0]["ref"]), None)
        store.close()


def test_a_move_closes_with_the_navigators_own_word_for_how_it_went() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(said=[{"seq": 1, "phase": "driving",
                                 "kind": "drive_to"}])
        recorder = Recorder(store, rover)
        recorder.poll()
        rover.said.append({"seq": 2, "phase": "ended", "kind": "drive_to",
                           "reason": "arrived", "why": "it got there"})
        recorder.poll()
        episode = store.episodes()[0]["ref"]
        check("closed", store.outcome(episode)["outcome"], "succeeded")
        check("...carrying the reason the navigator gave",
              store.outcome(episode)["detail"], "arrived: it got there")
        store.close()


def test_a_move_that_was_blocked_is_not_recorded_as_fine() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(said=[
            {"seq": 1, "phase": "driving", "kind": "explore"},
            {"seq": 2, "phase": "ended", "kind": "explore", "reason": "blocked",
             "why": "the ROS stack is not answering"}])
        Recorder(store, rover).poll()
        check("failed", store.outcome(store.episodes()[0]["ref"])["outcome"],
              "failed")
        store.close()


def test_an_ending_nothing_recognises_does_not_close_as_fine() -> None:
    """A move that ended for a reason this build has never heard of is not one
    to record as having gone well."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(said=[
            {"seq": 1, "phase": "driving", "kind": "drive_to"},
            {"seq": 2, "phase": "ended", "kind": "drive_to",
             "reason": "swallowed_by_a_hole"}])
        Recorder(store, rover).poll()
        check("failed rather than succeeded",
              store.outcome(store.episodes()[0]["ref"])["outcome"], "failed")
        store.close()


def test_a_phase_shorter_than_the_poll_is_still_recorded() -> None:
    """A replan lasts about a fifth of a second and is the one phase of a move
    worth knowing about. The loop keeps it and hands it back."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(said=[{"seq": 1, "phase": "driving",
                                 "kind": "drive_to"}])
        recorder = Recorder(store, rover)
        recorder.poll()
        # Between the two polls: a replan nobody polling could have seen.
        rover.said += [
            {"seq": 2, "phase": "replanning", "kind": "drive_to",
             "why": "something moved into the path"},
            {"seq": 3, "phase": "driving", "kind": "drive_to", "replans": 1},
        ]
        recorder.poll()
        episode = store.episode(store.episodes()[0]["ref"])
        check("the replan is in the record",
              [one["body"]["phase"] for one in episode["events"]],
              ["driving", "replanning", "driving"])
        check("...with what provoked it",
              episode["events"][1]["body"]["why"],
              "something moved into the path")
        check("...and nothing counted as missed",
              recorder.recorded["missed_moves"], 0)
        store.close()


def test_a_gap_longer_than_the_loop_remembers_is_counted() -> None:
    """The loop keeps thirty-two sentences. A recorder away for longer really
    has lost some, and the number is reported rather than the gap hidden."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(said=[{"seq": 1, "phase": "driving",
                                 "kind": "drive_to"}])
        recorder = Recorder(store, rover)
        recorder.poll()
        # A long silence: the loop is now on sentence 90 and remembers none of
        # the ones between.
        rover.said = [{"seq": 90, "phase": "driving", "kind": "explore"}]
        recorder.poll()
        check("eighty-eight sentences lost, and said so",
              recorder.recorded["missed_moves"], 88)
        store.close()


def test_a_new_move_closes_one_that_was_never_seen_to_end() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(said=[{"seq": 1, "phase": "driving",
                                 "kind": "drive_to", "asked": {"x_m": 1.0}}])
        recorder = Recorder(store, rover)
        recorder.poll()
        rover.said.append({"seq": 2, "phase": "driving", "kind": "explore",
                           "asked": None})
        recorder.poll()
        check("two episodes", store.summary()["episodes"], 2)
        first = [one for one in store.episodes()
                 if one["trigger_detail"]["kind"] == "drive_to"][0]["ref"]
        check("...and the first is closed honestly",
              store.outcome(first)["outcome"], "interrupted")
        check("...saying why it could not say more",
              "before this one was seen to end" in store.outcome(first)["detail"],
              True)
        store.close()


def test_the_loop_going_idle_closes_a_move_that_never_ended() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(said=[{"seq": 1, "phase": "driving",
                                 "kind": "drive_to"}])
        recorder = Recorder(store, rover)
        recorder.poll()
        rover.said.append({"seq": 2, "phase": "idle"})
        recorder.poll()
        got = store.outcome(store.episodes()[0]["ref"])
        check("closed", got["outcome"], "interrupted")
        check("...saying the loop simply went quiet",
              "went idle" in got["detail"], True)
        store.close()


def test_an_idle_rover_produces_no_move_episodes() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(said=[{"seq": 0, "phase": "idle"}])
        got = Recorder(store, rover).poll()
        check("nothing to record", got["moves"], 0)
        check("...and no episode invented", store.summary()["episodes"], 0)
        store.close()


# --- the daemon going away ---------------------------------------------------

def test_a_daemon_that_stops_answering_loses_nothing() -> None:
    """A deploy restarts the daemon under a running recorder. The next poll
    carries on from the mark."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE})
        recorder = Recorder(store, rover)
        recorder.poll()

        rover.down = True
        got = recorder.poll()
        check("the poll reports it rather than raising", got["ok"], False)
        check("...and counts it", recorder.recorded["unreachable"], 1)

        rover.down = False
        rover.rows += a_look(2, 200)
        rover.frames["frame-2"] = PICTURE
        after = recorder.poll()
        check("...and the next poll carries on", after["looks"], 1)
        check("...with nothing recorded twice", store.summary()["episodes"], 2)
        store.close()


def test_a_missing_picture_does_not_lose_the_look() -> None:
    """The world state may have cleared the frame between the row and the fetch."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={})
        got = Recorder(store, rover).poll()
        check("the look is still recorded", got["looks"], 1)
        check("...with no evidence behind it", store.summary()["evidence"], 0)
        check("...and it is honest that nothing is missing, because nothing "
              "was ever kept",
              replay.reconstruct(store, store.episodes()[0]["ref"])["replayable"],
              True)
        store.close()


def test_the_world_is_snapshotted_beside_the_looks() -> None:
    """So that what the rover held at the time survives the store moving on."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE},
                          entities=[{"id": "object:10", "looks": 4,
                                     "placement": {"viewpoints": 1,
                                                   "rays_agreeing": 1}}])
        Recorder(store, rover).poll()
        check("one snapshot", store.summary()["snapshots"], 1)
        episode = store.episode(store.episodes()[0]["ref"])
        digest = [one for one in episode["events"]
                  if one["kind"] == "measured"][0]["body"]["world_at"]
        held = store.snapshot_body(digest)
        check("...holding the things the rover had",
              held["entities"][0]["id"], "object:10")
        check("...including that this one was placed from a single look",
              held["entities"][0]["placement"]["viewpoints"], 1)
        store.close()



# --- keeping itself off the disk ---------------------------------------------

def test_a_run_prunes_the_record_as_it_goes() -> None:
    """Otherwise the honest claim is not "the record cannot fill the disk" but
    "it cannot, as long as somebody remembers to run the other program"."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE})
        recorder = Recorder(store, rover,
                            policy=retention.Policy(keep_days=0.0,
                                                    max_bytes=10 ** 9))
        recorder.poll()
        check("the look is recorded", store.summary()["episodes"], 1)
        recorder.retain()
        check("...and its picture pruned, being past the age limit",
              store.evidence_state(refs.digest(PICTURE))["state"], "deleted")
        check("...counted in the run's own report",
              recorder.recorded["evidence_removed"], 1)
        check("...and the episode says it can no longer be replayed in full",
              replay.reconstruct(store,
                                 store.episodes()[0]["ref"])["replayable"],
              False)
        store.close()


def test_a_run_can_be_told_not_to_prune() -> None:
    """Which is how the unpruned cost of a run gets measured."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE})
        recorder = Recorder(store, rover, policy=None)
        recorder.poll()
        got = recorder.run(seconds=0.0)
        check("nothing was pruned", got["evidence_removed"], 0)
        check("...and the picture is still there",
              store.evidence_state(refs.digest(PICTURE))["state"], "held")
        store.close()


def test_pruning_never_takes_a_pinned_run() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE})
        recorder = Recorder(store, rover,
                            policy=retention.Policy(keep_days=0.0, max_bytes=0))
        recorder.poll()
        store.pin(store.episodes()[0]["ref"], "the acceptance run")
        recorder.retain()
        check("nothing removed", recorder.recorded["evidence_removed"], 0)
        check("...and the picture is still there",
              store.evidence_state(refs.digest(PICTURE))["state"], "held")
        store.close()


def test_only_the_sentence_that_was_watched_carries_a_pose() -> None:
    """The pose comes from one reading of the rover taken now. Attaching it to a
    sentence the loop said before we looked would record a measurement nobody
    made -- seen on the drive of 2026-09-08, where six phases of one turn all
    carried the same heading because they arrived in a single poll."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(said=[
            {"seq": 1, "phase": "turning", "kind": "turn_in_place"},
            {"seq": 2, "phase": "planning", "kind": "turn_in_place"},
            {"seq": 3, "phase": "turning", "kind": "turn_in_place"},
        ])
        Recorder(store, rover).poll()
        steps = store.episode(store.episodes()[0]["ref"])["events"]
        check("three sentences", len(steps), 3)
        check("...and only the last one has a pose",
              [bool(one["body"].get("pose")) for one in steps],
              [False, False, True])
        check("...which is also the only one marked as watched",
              [one["body"].get("watched") for one in steps],
              [None, None, True])
        check("...while every one keeps its own phase",
              [one["body"]["phase"] for one in steps],
              ["turning", "planning", "turning"])
        store.close()


def test_a_moves_summary_shows_the_move() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(said=[
            {"seq": 1, "phase": "choosing", "kind": "drive_to",
             "asked": {"x_m": 1.5}},
            {"seq": 2, "phase": "driving", "kind": "drive_to",
             "route_m": 3.24, "waypoints": 7},
            {"seq": 3, "phase": "driving", "kind": "drive_to", "route_m": 3.24},
            {"seq": 4, "phase": "replanning", "kind": "drive_to",
             "why": "something moved into the path", "replans": 1},
            {"seq": 5, "phase": "ended", "kind": "drive_to", "reason": "arrived"},
        ])
        Recorder(store, rover).poll()
        said = summary.of(store, store.episodes()[0]["ref"],
                          live_world_generation=WORLD)
        check("what was asked for",
              "drive_to(x_m=1.5), and it went" in said, True)
        check("...the phases, with repeats collapsed",
              "choosing -> driving x2 -> replanning -> ended" in said, True)
        check("...what provoked the replan",
              "replanning: something moved into the path" in said, True)
        check("...the route and the replan count",
              "3.24 m of route, 7 waypoints, 1 replan" in said, True)
        check("...and how much of it was actually watched",
              "1 of 5 steps seen as they happened" in said, True)
        check("...and how it ended", "closed: succeeded -- arrived" in said, True)
        store.close()

TESTS = (
    test_a_recorder_cannot_move_the_rover,
    test_world_state_inspect_is_not_a_call_a_recorder_may_make,
    test_one_episode_per_look,
    test_a_look_is_never_recorded_twice,
    test_a_recorder_that_was_stopped_catches_up,
    test_the_walk_back_is_bounded,
    test_a_look_carries_its_picture,
    test_a_look_can_be_recorded_without_its_picture,
    test_what_a_look_saw_is_named_durably,
    test_a_rover_that_cannot_say_which_world_it_is_still_gets_recorded,
    test_a_look_that_attached_to_nothing_is_not_a_failure,
    test_a_summary_of_a_shadow_episode_says_what_the_look_found,
    test_a_shadow_episode_decides_nothing_and_says_so,
    test_a_move_is_one_episode_however_many_things_it_says,
    test_only_the_sentence_that_was_watched_carries_a_pose,
    test_a_moves_summary_shows_the_move,
    test_a_move_closes_with_the_navigators_own_word_for_how_it_went,
    test_a_move_that_was_blocked_is_not_recorded_as_fine,
    test_an_ending_nothing_recognises_does_not_close_as_fine,
    test_a_phase_shorter_than_the_poll_is_still_recorded,
    test_a_gap_longer_than_the_loop_remembers_is_counted,
    test_a_new_move_closes_one_that_was_never_seen_to_end,
    test_the_loop_going_idle_closes_a_move_that_never_ended,
    test_an_idle_rover_produces_no_move_episodes,
    test_a_daemon_that_stops_answering_loses_nothing,
    test_a_missing_picture_does_not_lose_the_look,
    test_the_world_is_snapshotted_beside_the_looks,
    test_a_run_prunes_the_record_as_it_goes,
    test_a_run_can_be_told_not_to_prune,
    test_pruning_never_takes_a_pinned_run,
)
