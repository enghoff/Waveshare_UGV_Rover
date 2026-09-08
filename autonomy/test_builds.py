"""Which build produced an episode's evidence, and noticing one land mid-run.

The shadow run of 2026-09-08 is what these are about. A deploy landed in the
middle of it and changed the rules the resolver decides identity by, so the looks
recorded before it and the ones after it were decided differently -- and nothing
in the record said so. Anybody comparing the two halves would have been comparing
two experiments while believing they had one.
"""
from __future__ import annotations

import json
import os
import tempfile

import builds
from recorder import BUILD_MARK, WORLD_MARK, Recorder
from test_fakes import PICTURE, FakeRover, a_look, a_store
from test_harness import check

ONE = "d1aeef92cbf54001facf5c6c0f44bf6c17715638"
TWO = "ef0f7c4d778a17e14c55b1735d872bc0baaaef00"


def _state(directory: str, **components) -> str:
    path = os.path.join(directory, "deploy-state.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump({"version": 1, "components": components}, handle)
    return path


def test_the_build_of_each_watched_component_is_read() -> None:
    with tempfile.TemporaryDirectory() as directory:
        path = _state(directory, world_state=ONE, rover_daemon=TWO,
                      ros_nav=ONE, netwatch=ONE)
        got = builds.builds(path)
        check("the three that change what a look means",
              sorted(got), ["ros_nav", "rover_daemon", "world_state"])
        check("...shortened to something a person can compare",
              got["world_state"], "d1aeef92cbf5")
        check("...and nothing else is carried", "netwatch" in got, False)


def test_a_machine_that_was_never_deployed_to_says_nothing() -> None:
    """A developer's desk, or a test. Record what you can and claim nothing."""
    with tempfile.TemporaryDirectory() as directory:
        check("no file, no answer",
              builds.builds(os.path.join(directory, "nothing.json")), {})
        broken = os.path.join(directory, "broken.json")
        open(broken, "w", encoding="utf-8").write("{oh dear")
        check("an unreadable file, no answer", builds.builds(broken), {})
        check("...and it reads as not known rather than as nothing running",
              builds.describe({}), "which build was running is not known")


def test_a_redeploy_between_two_readings_is_named() -> None:
    check("the one that moved",
          builds.changed({"world_state": ONE, "ros_nav": ONE},
                         {"world_state": TWO, "ros_nav": ONE}),
          ["world_state"])
    check("...and nothing when nothing moved",
          builds.changed({"world_state": ONE}, {"world_state": ONE}), [])


def test_every_episode_says_which_build_produced_it() -> None:
    with tempfile.TemporaryDirectory() as directory:
        path = _state(directory, world_state=ONE, rover_daemon=ONE, ros_nav=ONE)
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE})
        recorder = Recorder(store, rover)
        recorder._builds = builds.builds(path)
        recorder.poll()
        got = store.episodes()[0]["trigger_detail"]["builds"]
        check("the world state that decided this look's identity",
              got["world_state"], "d1aeef92cbf5")
        store.close()


def test_a_deploy_landing_under_a_run_is_written_down() -> None:
    """It is not enough that the recording survives a restart. The rules that
    decide what a look means can change with it, and the record has to say so."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE})
        recorder = Recorder(store, rover)
        recorder._builds = {"world_state": "d1aeef92cbf5"}

        path = _state(directory, world_state=TWO, rover_daemon=ONE, ros_nav=ONE)
        builds.STATE, was = path, builds.STATE
        try:
            got = recorder.poll()
        finally:
            builds.STATE = was
        check("the change is reported by the poll", got["redeployed"],
              ["ros_nav", "rover_daemon", "world_state"])
        check("...counted", recorder.recorded["redeploys"], 1)
        check("...and marked, so the moment can be found afterwards",
              "ef0f7c4d778a" in store.marked(BUILD_MARK), True)
        check("...while the look recorded after it carries the new build",
              store.episodes()[0]["trigger_detail"]["builds"]["world_state"],
              "ef0f7c4d778a")
        store.close()


def test_an_unchanged_build_is_not_reported_every_poll() -> None:
    with tempfile.TemporaryDirectory() as directory:
        path = _state(directory, world_state=ONE, rover_daemon=ONE, ros_nav=ONE)
        store = a_store(directory)
        rover = FakeRover()
        recorder = Recorder(store, rover)
        builds.STATE, was = path, builds.STATE
        try:
            recorder._builds = builds.builds(path)
            recorder.poll()
            recorder.poll()
        finally:
            builds.STATE = was
        check("nothing to report", recorder.recorded["redeploys"], 0)
        check("...and no marks written", store.marks(BUILD_MARK), [])
        store.close()


def test_the_world_being_emptied_under_a_run_is_written_down() -> None:
    """The same kind of event as a deploy: every identifier the episodes before
    it hold has been handed to something else."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        rover = FakeRover(rows=a_look(1, 100), frames={"frame-1": PICTURE})
        recorder = Recorder(store, rover)
        first = recorder.poll()
        check("the first sight of a world is not a clear", first["cleared"], "")

        rover.generation = "4d2fda72621ef169"
        rover.rows += a_look(2, 200)
        rover.frames["frame-2"] = PICTURE
        got = recorder.poll()
        check("the clear is reported", got["cleared"], "4d2fda72621ef169")
        check("...counted", recorder.recorded["world_cleared"], 1)
        check("...and marked", store.marked(WORLD_MARK), "4d2fda72621ef169")

        both = {one["ref"]: one["world_generation"] for one in store.episodes()}
        check("...and the two looks belong to different worlds",
              len(set(both.values())), 2)
        store.close()


TESTS = (
    test_the_build_of_each_watched_component_is_read,
    test_a_machine_that_was_never_deployed_to_says_nothing,
    test_a_redeploy_between_two_readings_is_named,
    test_every_episode_says_which_build_produced_it,
    test_a_deploy_landing_under_a_run_is_written_down,
    test_an_unchanged_build_is_not_reported_every_poll,
    test_the_world_being_emptied_under_a_run_is_written_down,
)
