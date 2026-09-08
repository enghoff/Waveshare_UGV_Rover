"""Checks for the loop that carries out what the deliberation chose.

One room, drawn on paper, with a thing in it worth going to look at; a fake
rover holding the real permission rules; and a clock the checks wind by hand so
that a lease expires, an action times out and a run spends its budget without
anything waiting for any of it.

**What is being checked is mostly the ways a turn does not finish.** That a good
turn drives, looks and records is one check. The other twelve are a daemon
refusing, a person stopping the rover mid-drive, the permission lapsing, the
world going quiet, the battery falling, the map being replaced, a look failing,
a goal timing out -- because those are what a supervised trial on the rover is
for, and a fault first met there costs an afternoon and a person's attention.

The list is the one [M3](../docs/plans/autonomous-curiosity.md) asks for:
success, daemon refusal, timeout, service loss, low-battery abort, manual stop
and no-candidate idle, plus a stop in each state of the machine.
"""
from __future__ import annotations

import os
import tempfile
from typing import Any

import client
import executive as executive_mod
import permission
import scoring
import scenarios
import store as store_mod
import test_fakes
from test_harness import check

#: A room with somewhere to go and something to look at. The rover stands in
#: the middle of mapped floor with unmapped ground through the gap, which is
#: what makes a frontier goal available; the thing at `T` is placed badly
#: enough to be worth a second viewpoint.
ROOM = [
    "####################",
    "#..................#",
    "#..................#",
    "#........R.........#",
    "#..................#",
    "#..................#",
    "####.........#######",
    "???????????????????#",
]


def a_thing(**fields: Any) -> dict[str, Any]:
    """A thing the rover has seen and placed badly, which is what makes a
    second viewpoint worth driving to. `scenarios.thing`'s shape rather than a
    hand-written one, so that a change to what an entity looks like reaches
    these checks the way it reaches the curated rooms."""
    return scenarios.thing("object:19", 1.2, -0.4, uncertainty_m=1.4,
                           looks=5, ranged=0, **fields)


def a_rover(**fields: Any) -> test_fakes.ActingRover:
    rover = test_fakes.ActingRover(room=ROOM, entities=[a_thing()])
    for name, value in fields.items():
        setattr(rover, name, value)
    return rover


class Session:
    """A store, a rover and an executive, with the clock in the test's hand."""

    def __init__(self, rover=None, **budget) -> None:
        self.dir = tempfile.mkdtemp(prefix="ugv-executive-")
        self.store = store_mod.EpisodeStore(self.dir)
        self.rover = rover if rover is not None else a_rover()
        self.slept: list[float] = []
        self.said: list[str] = []
        self.run = self.rover.enable(**budget)
        self.executive = executive_mod.Executive(
            self.store, self.rover, scoring.DEFAULT,
            sleep=self._sleep, now=self.rover.clock, log=self.said.append)
        self.executive.attach()

    def _sleep(self, seconds: float) -> None:
        """Winding the clock rather than waiting on it.

        The permission's expiry, the action timeout and the idle wait are all
        measured on this same clock, so a check that a lease runs out is written
        by letting the loop do its own sleeping.
        """
        self.slept.append(seconds)
        self.rover.clock.tick(seconds)
        # And the daemon's watchdog looks, because on the rover it does: the
        # thing these checks are mostly about is what happens to a run while
        # this loop is not asking about it.
        self.rover.watchdog()

    def close(self) -> None:
        self.store.close()

    def episodes(self) -> list[dict[str, Any]]:
        return self.store.episodes(limit=20)

    def events(self, episode: str) -> list[dict[str, Any]]:
        return self.store.episode(episode)["events"]

    def calls(self, episode: str) -> list[dict[str, Any]]:
        return [one["body"] for one in self.events(episode)
                if one["kind"] == "call"]


def _arriving(session: Session, at=None) -> None:
    """Let the fake rover's move end the next time the loop looks at it.

    It lands where it was sent unless the check says otherwise, because a rover
    that arrived somewhere else is a different test -- and because the distance
    it covered is what the run's travel budget is spent from.
    """
    original = session.rover._autonomy_status

    def once(arguments):
        rover = session.rover
        if rover.driving:
            asked = rover.moves[-1] if rover.moves else {}
            rover.arrive(at=at or (asked.get("x_m", 0.0), asked.get("y_m", 0.0)))
        return original(arguments)

    session.rover._autonomy_status = once


# --- a turn that works --------------------------------------------------------

def test_one_turn_drives_looks_and_writes_down_what_changed():
    session = Session()
    _arriving(session)
    got = session.executive.once()
    check("the turn acted", got["acted"], True)
    check("...and finished", got["outcome"], "succeeded")

    check("the rover was sent somewhere", len(session.rover.moves), 1)
    check("...and took a look when it got there", session.rover.looks, 1)

    episode = got["episode"]
    calls = session.calls(episode)
    check("both actions are in the episode",
          [one["call"] for one in calls], ["drive_to", "world_inspect"])
    check("...with what came back", all(one["ok"] for one in calls), True)
    kinds = [one["kind"] for one in session.events(episode)]
    check("...beside the decision that chose them",
          ("decision" in kinds, "candidate" in kinds), (True, True))
    check("...and the measurement of what it changed",
          kinds.count("measured"), 2)
    check("the episode closed as a success",
          session.store.outcome(episode)["outcome"], "succeeded")
    session.close()


def test_every_movement_names_the_episode_and_the_action_that_asked_for_it():
    session = Session()
    _arriving(session)
    got = session.executive.once()
    dispatched = session.rover.permission.actions
    check("every action carries an identifier", len(dispatched), 2)
    for action_id, record in dispatched.items():
        check(f"{action_id} names its episode", record["episode"],
              got["episode"])
        check(f"...and its identifier begins with it",
              action_id.startswith(got["episode"]), True)
    session.close()


def test_a_turn_with_nothing_worth_doing_idles_without_acting():
    # No things to look at and no unmapped floor: nothing to propose.
    rover = test_fakes.ActingRover(room=["#####",
                                         "#...#",
                                         "#.R.#",
                                         "#####"], entities=[])
    session = Session(rover=rover)
    got = session.executive.once()
    check("nothing was acted on", got["acted"], False)
    check("...and nothing was sent to the rover", rover.moves, [])
    check("...but the turn is still an episode",
          session.store.outcome(got["episode"])["outcome"], "abandoned")
    check("...and it waited rather than spinning",
          round(sum(session.slept), 1), executive_mod.IDLE_S)
    check("...in naps short enough to renew through",
          max(session.slept) <= executive_mod.IDLE_NAP_S, True)
    check("...so the run is still alive at the end of the wait",
          rover.permission.status()["enabled"], True)
    session.close()


def test_standing_still_is_not_mistaken_for_a_dead_executive():
    """The fault the rover found on the first turn it was ever asked for.

    A parked rover with nothing worth doing is the ordinary case, and the wait
    that follows is twice the length of the permission's lease. A loop that
    slept through it in one go stopped renewing, and the daemon -- correctly, by
    its own rules -- took the wheels back from an executive that was merely
    waiting for the room to change.
    """
    rover = test_fakes.ActingRover(room=["#####",
                                         "#...#",
                                         "#.R.#",
                                         "#####"], entities=[])
    session = Session(rover=rover)
    check("the wait is longer than the lease",
          executive_mod.IDLE_S > permission.PERMIT_TTL_S, True)
    session.executive.once()
    check("and the run survives it", rover.permission.status()["enabled"], True)
    check("...having been renewed several times",
          (rover.permission.permit or {}).get("renewals", 0) >= 2, True)
    session.close()


# --- the ways a turn is cut short --------------------------------------------

def test_a_person_stopping_the_rover_ends_the_turn_and_the_loop():
    session = Session()
    stopped = {}

    def stop_when_driving(arguments):
        if session.rover.driving and not stopped:
            stopped.update(session.rover.permission.stop(
                by="a person", why="somebody pressed stop"))
        return {"ok": True, **session.rover.permission.status()}

    session.rover._autonomy_status = stop_when_driving
    got = session.executive.once()
    check("the turn was abandoned", got["outcome"], "interrupted")
    check("...saying who ended it", "somebody pressed stop" in got["why"], True)
    check("...and it is in the episode",
          session.store.outcome(got["episode"])["outcome"], "interrupted")

    # And the loop does not start another turn: only a person can re-enable.
    summary = session.executive.loop(turns=3)
    check("the loop does not begin again after a stop", summary["turns"], 0)
    check("...and says why it stopped",
          "somebody pressed stop" in summary["ended"], True)
    session.close()


def test_a_stop_in_any_state_of_the_machine_aborts_the_turn():
    """Stopped while deciding, while dispatching and while driving.

    Each one is a different place in the loop and a different thing has to
    notice: the gate before the choice, the daemon's refusal at dispatch, and
    the poll that watches a move nobody is waiting for.
    """
    for when in ("SELECT", "EXECUTE", "WAIT"):
        session = Session()
        rover = session.rover
        if when == "SELECT":
            rover.permission.stop(by="a person", why="stopped before deciding")
            got = session.executive.once()
            check("stopped while deciding: nothing was chosen",
                  got["acted"], False)
            check("...and the gate says who stopped it",
                  "stopped before deciding" in (got["why"] or ""), True)
        elif when == "EXECUTE":
            original = rover._autonomy_act

            def refuse(arguments, _original=original):
                rover.permission.stop(by="a person", why="stopped mid-dispatch")
                return _original(arguments)

            rover._autonomy_act = refuse
            got = session.executive.once()
            check("stopped while dispatching: the turn is interrupted",
                  got["outcome"], "interrupted")
            check("...and the rover was never sent anywhere", rover.moves, [])
        else:
            def stop_soon(arguments):
                if rover.driving:
                    rover.permission.stop(by="a person",
                                          why="stopped while driving")
                return {"ok": True, **rover.permission.status()}

            rover._autonomy_status = stop_soon
            got = session.executive.once()
            check("stopped while driving: the turn is interrupted",
                  got["outcome"], "interrupted")
            check("...naming the stop",
                  "stopped while driving" in got["why"], True)
        check(f"{when}: the latch stands afterwards",
              rover.permission.status()["latched"], True)
        session.close()


def test_permission_running_out_mid_drive_ends_the_turn():
    session = Session()
    rover = session.rover

    def never_renew(_arguments):
        """An executive that cannot renew: the daemon has stopped granting.

        This is the polite half of a hung executive. The other half -- nothing
        asking at all -- is the daemon's own watchdog, and it is checked in
        rover_daemon/test_autonomy.py, because it happens on the daemon.
        """
        return {"ok": False, "error": "the run ended: out of minutes"}

    def expire(arguments):
        if rover.driving:
            rover.permission.end_run("out of minutes")
        return {"ok": True, **rover.permission.status()}

    rover._autonomy_status = expire
    rover._autonomy_permit = never_renew
    got = session.executive.once()
    check("a run that ended under a move interrupts the turn",
          got["outcome"], "interrupted")
    check("...saying so", "out of minutes" in got["why"], True)
    check("...and it is not recorded as a stop",
          rover.permission.status()["latched"], False)
    session.close()


def test_a_goal_that_never_arrives_times_out():
    session = Session()
    got = session.executive.once()          # the fake never arrives on its own
    check("a move that never ends is given up on", got["outcome"], "interrupted")
    check("...after the declared time",
          f"{executive_mod.ACTION_TIMEOUT_S:.0f} s" in got["why"], True)
    check("...and the rover was told to stop", session.rover.driving, False)
    session.close()


def test_the_daemon_refusing_an_action_ends_the_turn_without_moving():
    session = Session()
    rover = session.rover
    original = rover._autonomy_act

    def redraw_the_map_first(arguments):
        """The map is replaced between the choice and the dispatch.

        Which is not a contrived order: a loop closure lands whenever
        slam_toolbox finds one, and the goal in flight was named in the frame
        that has just stopped existing.
        """
        rover.map_id = "m2"
        return original(arguments)

    rover._autonomy_act = redraw_the_map_first
    got = session.executive.once()
    check("a stale goal is refused", got["outcome"], "interrupted")
    check("...by the daemon rather than by the loop",
          "stale map" in got["why"] or "map" in got["why"], True)
    check("...and nothing moved", session.rover.moves, [])
    calls = session.calls(got["episode"])
    check("...with the refusal in the record", calls[-1]["ok"], False)
    session.close()


def test_a_low_battery_refuses_before_the_wheels_turn():
    session = Session(rover=a_rover(volts=10.9))
    got = session.executive.once()
    check("a flat battery stops the turn", got["acted"], False)
    check("...at the gate, before anything was planned",
          "11.2 V" in (got["why"] or ""), True)
    check("...and nothing was sent to the rover", session.rover.moves, [])
    session.close()


def test_losing_the_daemon_ends_the_loop_rather_than_the_rover():
    session = Session()
    session.rover.down = True
    summary = session.executive.loop(turns=2)
    check("the loop ends when the daemon goes", summary["turns"], 0)
    check("...saying what happened",
          "stopped answering" in summary["ended"], True)
    session.close()


def test_a_look_that_fails_ends_the_turn_after_the_drive():
    session = Session()
    _arriving(session)
    session.rover.inspection = {"ok": False, "error": "the camera did not open"}
    got = session.executive.once()
    check("the drive happened", len(session.rover.moves), 1)
    check("...the look failed", got["outcome"], "interrupted")
    check("...and the reason is the rover's own",
          "the camera did not open" in got["why"], True)
    check("...recorded as a failed call",
          session.calls(got["episode"])[-1]["ok"], False)
    session.close()


# --- what the loop cannot do -------------------------------------------------

def test_the_executive_cannot_move_the_rover_except_through_a_permit():
    rover = test_fakes.ActingRover()
    for name in sorted(client.MOVES):
        try:
            rover.call(name, {})
            check(f"{name} is refused to the executive", "not refused", name)
        except client.Refused as refused:
            check(f"{name} is refused to the executive",
                  "may make" in str(refused), True)
    check("...and driving is not on its list", "drive_to" in rover.allowed,
          False)
    check("...while acting under permission is",
          sorted(client.ACTING - rover.allowed), [])


def test_the_executive_cannot_give_itself_back_the_authority_a_person_took():
    rover = test_fakes.ActingRover()
    run = rover.enable()
    rover.permission.stop(by="a person", why="that is enough for today")
    for name in sorted(client.HUMAN):
        try:
            rover.call(name, {"by": "the executive"})
            check(f"{name} is refused to the executive", "not refused", name)
        except client.Refused as refused:
            check(f"{name} is refused to the executive",
                  "person's act" in str(refused), True)
    check("so the latch stands", rover.permission.status()["latched"], True)
    check("...and permission cannot be had",
          rover.call("autonomy_permit", {"run": run})["ok"], False)


def test_nothing_in_a_turn_asks_a_model_anything():
    """The model-independence check, made against the record rather than the code.

    A turn that consulted a model would have to record it -- `events.model` is
    the only way an episode may say a model answered -- so an episode with no
    model event in it is an attempt that did not depend on one. The fake rover
    offers no model at all, which is the other half: the loop completes a whole
    goal on a rover where nothing could have answered.
    """
    session = Session()
    _arriving(session)
    got = session.executive.once()
    kinds = [one["kind"] for one in session.events(got["episode"])]
    check("no model was asked anything", kinds.count("model"), 0)
    check("...and the turn succeeded anyway", got["outcome"], "succeeded")
    session.close()


def test_an_action_the_rover_would_not_admit_is_refused_before_anything_runs():
    """A plan is checked whole, before its first step is dispatched.

    Taking `world_inspect` off the admitted list is how a future goal type that
    wants an operation this rover does not offer would look. The check that
    matters is the second one: the drive that *is* admitted, and which comes
    first in the plan, never happens either.
    """
    session = Session()
    _arriving(session)
    was = dict(permission.ACTIONS)
    permission.ACTIONS.pop("world_inspect")
    try:
        got = session.executive.once()
    finally:
        permission.ACTIONS.clear()
        permission.ACTIONS.update(was)
    check("the plan is refused whole", got["outcome"], "interrupted")
    check("...naming the operation", "world_inspect" in got["why"], True)
    check("...before any of its steps ran", session.rover.moves, [])
    session.close()


def test_a_repeat_of_an_action_is_refused_rather_than_done_twice():
    """The dispatch identifier, from the executive's side.

    A connection that dropped after the daemon had already started the move is
    the case: the executive would ask again with the same identifier, and the
    answer has to be what happened the first time rather than a second drive.
    """
    session = Session()
    rover = session.rover
    run, permit = session.run, session.executive.renew()
    first = rover.call("autonomy_act",
                       {"permit": permit, "action": "drive_to",
                        "action_id": "au/x/episode:1#1", "episode": "au/x/episode:1",
                        "params": {"x_m": 1.0, "y_m": 1.0}})
    again = rover.call("autonomy_act",
                       {"permit": permit, "action": "drive_to",
                        "action_id": "au/x/episode:1#1", "episode": "au/x/episode:1",
                        "params": {"x_m": 1.0, "y_m": 1.0}})
    check("the first request drove", first["ok"], True)
    check("the second did not", again["ok"], False)
    check("...and says what the first one did",
          again["already"]["action"], "drive_to")
    check("...having driven once", len(rover.moves), 1)
    session.close()


TESTS = (
    test_one_turn_drives_looks_and_writes_down_what_changed,
    test_every_movement_names_the_episode_and_the_action_that_asked_for_it,
    test_a_turn_with_nothing_worth_doing_idles_without_acting,
    test_standing_still_is_not_mistaken_for_a_dead_executive,
    test_a_person_stopping_the_rover_ends_the_turn_and_the_loop,
    test_a_stop_in_any_state_of_the_machine_aborts_the_turn,
    test_permission_running_out_mid_drive_ends_the_turn,
    test_a_goal_that_never_arrives_times_out,
    test_the_daemon_refusing_an_action_ends_the_turn_without_moving,
    test_a_low_battery_refuses_before_the_wheels_turn,
    test_losing_the_daemon_ends_the_loop_rather_than_the_rover,
    test_a_look_that_fails_ends_the_turn_after_the_drive,
    test_the_executive_cannot_move_the_rover_except_through_a_permit,
    test_the_executive_cannot_give_itself_back_the_authority_a_person_took,
    test_nothing_in_a_turn_asks_a_model_anything,
    test_an_action_the_rover_would_not_admit_is_refused_before_anything_runs,
    test_a_repeat_of_an_action_is_refused_rather_than_done_twice,
)
