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

import contextlib
import tempfile
from typing import Any

import math

import client
import cooling
import decide as decide_mod
import executive as executive_mod
import goals
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


@contextlib.contextmanager
def nothing_in_reach():
    """For a trial's own mechanics. Since 2026-10-09 an M4 trial asks only of
    things out of reach of where the rover stands, and this room is two metres
    across, so everything in it is in reach; the reach rule has its own checks
    in test_goals.py."""
    was = goals.REACH_FAR_M
    goals.REACH_FAR_M = 0.0
    try:
        yield
    finally:
        goals.REACH_FAR_M = was


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


def test_a_geometry_goal_faces_its_thing_and_aims_the_look_at_it():
    """Found on 2026-10-03: the drive carried no heading and the look no aim, so
    most looks in runs 3 and 4 were taken with the thing off the picture."""
    session = Session()
    _arriving(session)
    got = session.executive.once()
    drive, look = session.calls(got["episode"])
    check("the drive is told which way to face",
          drive["params"].get("heading_deg") is not None, True)
    check("...and the look is aimed at the thing",
          look["params"].get("aim_at"), {"x_m": 1.2, "y_m": -0.4})
    check("...and names it, so its region is filed to it (agreed 2026-10-08)",
          str(look["params"].get("target") or "").startswith("object:"), True)
    session.close()
    import executive as executive_mod
    executive_mod.NAME_THE_TARGET = False
    try:
        session = Session()
        _arriving(session)
        got = session.executive.once()
        _drive, look = session.calls(got["episode"])
        check("...and names nothing with the case switched off",
              "target" in look["params"], False)
        session.close()
    finally:
        executive_mod.NAME_THE_TARGET = True


def test_a_geometry_look_tilts_level_only_when_the_thing_needs_it():
    """goals.LOOK_TILTS_DEG: a thing on the floor is seen by the depth camera
    only with the gimbal level; anything else is looked at from rest."""
    session = Session()
    candidate = {"id": "improve_geometry:object:9@0.50,0.00", "type": "improve_geometry",
                 "target": "object:9", "expects": "",
                 "constraints": {"goal": {"x_m": 0.5, "y_m": 0.0, "heading_deg": 0.0},
                                 "look_at": {"x_m": 2.0, "y_m": 0.0},
                                 "look_tilt_deg": 0.0}}
    look = session.executive.plan(candidate)[-1]["params"]
    check("a look at a rug below the camera asks for the level tilt",
          look.get("tilt_deg"), 0.0)
    candidate["constraints"]["look_tilt_deg"] = 20.0
    look = session.executive.plan(candidate)[-1]["params"]
    check("...and a look at the resting tilt does not move the gimbal",
          "tilt_deg" in look, False)
    session.close()


def test_an_m4_trial_copies_the_store_and_looks_again_from_where_it_stands():
    """M4 (agreed 2026-10-09): each attempt is paired with a re-look from where
    the rover stood when it chose, both scored against a copy of the store taken
    before either. The copy comes first, then the re-look -- a turn on the spot
    and an aimed look -- then the drive and the chosen viewpoint's look."""
    with nothing_in_reach():
        rover = a_rover()
        rover.trial = {"targets": ["object:19"], "relook": True, "snapshot": True}
        session = Session(rover)
        _arriving(session)
        here = session.rover._nav_status({})["pose"]
        got = session.executive.once()
        check("the attempt finished", got["outcome"], "succeeded")
        episode = got["episode"]
        calls = session.calls(episode)
        check("a copy of the store, then a turn and a look, then the drive and the "
              "look", [(one["call"], (one.get("result") or {}).get("role"))
                       for one in calls],
              [("world_snapshot", None), ("drive_to", "relook"),
               ("world_inspect", "relook"), ("drive_to", None),
               ("world_inspect", None)])
        check("...the copy named after the episode", session.rover.snapshots,
              [episode])
        turn, relook = calls[1]["params"], calls[2]["params"]
        check("the re-look's turn is to the spot the rover is on",
              (turn["x_m"], turn["y_m"]), (round(here["x_m"], 3), round(here["y_m"], 3)))
        facing = math.degrees(math.atan2(-0.4 - here["y_m"], 1.2 - here["x_m"]))
        check("...facing the thing", abs(turn["heading_deg"] - facing) < 0.1, True)
        check("...and its look is aimed at the thing and names it",
              (relook.get("aim_at"), relook.get("target")),
              ({"x_m": 1.2, "y_m": -0.4}, "object:19"))
        measured = [one["body"] for one in session.events(episode)
                    if one["kind"] == "measured" and one["body"].get("with_relook")]
        check("...and the measurement says it holds both looks' change",
              len(measured), 1)
        session.close()


def test_an_m4_trial_looks_only_at_its_targets():
    """A trial's attempts are predeclared: a run given other records does not
    wander off to this one, or to a frontier."""
    rover = a_rover()
    rover.trial = {"targets": ["object:99"], "relook": True, "snapshot": True}
    session = Session(rover)
    _arriving(session)
    got = session.executive.once()
    check("nothing outside the trial is attempted", got["acted"], False)
    check("...no look was taken and no copy made",
          (session.rover.looks, session.rover.snapshots), (0, []))
    decided = [one["body"] for one in session.events(got["episode"])
               if one["kind"] == "candidate"]
    check("...and every candidate says why",
          all("not a trial target" in str(one) for one in decided), True)
    session.close()


def test_a_relook_that_fails_does_not_end_the_attempt():
    """The re-look is the baseline. One that saw nothing gained nothing, and the
    chosen viewpoint is still worth its drive."""
    with nothing_in_reach():
        rover = a_rover()
        rover.trial = {"targets": ["object:19"], "relook": True, "snapshot": True}
        original = rover._perform

        def perform(action, params):
            if action == "world_inspect" and rover.looks == 0:
                rover.looks += 1
                return {"ok": False, "error": "the camera was busy"}
            return original(action, params)

        rover._perform = perform
        session = Session(rover)
        _arriving(session)
        got = session.executive.once()
        check("the attempt went on to its chosen viewpoint and finished",
              (got["outcome"], session.rover.looks, len(session.rover.moves)),
              ("succeeded", 2, 2))
        failed = [one for one in session.calls(got["episode"]) if not one["ok"]]
        check("...with the re-look that failed recorded as such",
              [(one["call"], one["result"].get("role")) for one in failed],
              [("world_inspect", "relook")])
        session.close()


def test_an_ordinary_run_neither_copies_the_store_nor_looks_again():
    session = Session()
    _arriving(session)
    got = session.executive.once()
    check("two calls, as before",
          [one["call"] for one in session.calls(got["episode"])],
          ["drive_to", "world_inspect"])
    check("...and no copy of the store", "world_snapshot" in session.rover.asked,
          False)
    session.close()


def test_a_relook_is_tilted_for_the_things_elevation_from_where_it_stands():
    """A thing low enough to need the level tilt from its viewpoint may need the
    resting tilt from further away, and the other way about."""
    import situation as situation_mod
    session = Session()
    session.executive.trial = {"relook": True}
    # The floor rug of goals.LOOK_TILTS_DEG, seen 1 m away at -13 degrees.
    candidate = {"id": "improve_geometry:object:9@1.00,0.00",
                 "type": "improve_geometry", "target": "object:9", "expects": "",
                 "constraints": {"goal": {"x_m": 1.0, "y_m": 0.0, "heading_deg": 0.0},
                                 "look_at": {"x_m": 2.0, "y_m": 0.0},
                                 "elevation_deg": -13.0, "range_m": 1.0,
                                 "look_tilt_deg": 0.0}}
    near = situation_mod.Situation({"nav": {"pose": {"x_m": 1.0, "y_m": 0.0,
                                                    "heading_deg": 90.0}}})
    steps = session.executive.plan(candidate, near)
    check("from the same distance, the re-look tilts level as the chosen look does",
          (steps[1]["params"].get("tilt_deg"), steps[3]["params"].get("tilt_deg")),
          (0.0, 0.0))
    check("...and its turn faces the thing", steps[0]["params"]["heading_deg"], 0.0)
    session.close()


def test_a_look_that_could_see_the_place_and_found_nothing_sets_it_aside_longer():
    """A record whose aimed look, taken with its place in the depth camera's
    view, filed nothing is probably not where it claims (2026-10-08)."""
    thing = a_thing()
    thing["placement"]["height_m"] = 0.3
    rover = test_fakes.ActingRover(room=ROOM, entities=[thing])
    rover.inspection = {**dict(rover.inspection), "aimed_filing": {
        "filed": None, "why": "no region of the look points at it"}}
    session = Session(rover)
    _arriving(session)
    got = session.executive.once()
    attempt = [one["body"] for one in session.events(got["episode"])
               if one["kind"] == "measured" and one["body"].get("what") == "the attempt"]
    check("the attempt says the place was seen empty",
          attempt[-1].get("seen_empty"), True)
    cooled = decide_mod._loads(session.store.marked(decide_mod.COOLED_MARK))
    entry = [one for one in cooled if one["target"] == "object:19"][0]
    check("...and the record is aside for two hours",
          entry["until"] - entry["since"], cooling.EMPTY_COOLDOWN_S)
    session.close()


def test_a_geometry_look_records_and_leaves_settling_to_the_rover():
    """M3 session 6, 2026-10-06: with 1,999 bearings pending, six of 39 looks
    failed waiting for the lock a settling pass holds, or timed out at 10 s
    inside one. A run's geometry look now records without settling, and the
    executive gives that one call longer than the others."""
    import executive as executive_mod

    session = Session()
    _arriving(session)
    waited: list[tuple[str, float]] = []
    original = session.rover._ask

    def timed(name, arguments):
        if name == "autonomy_act":
            waited.append((arguments.get("action"), session.rover.timeout))
        return original(name, arguments)

    session.rover._ask = timed
    got = session.executive.once()
    _drive, look = session.calls(got["episode"])
    check("the look does not settle", look["params"].get("settle"), False)
    check("...and is given longer to answer than a drive",
          waited, [("drive_to", 10.0),
                   ("world_inspect", executive_mod.LOOK_CALL_TIMEOUT_S)])
    check("...which still ends inside the permission renewed before it",
          executive_mod.LOOK_CALL_TIMEOUT_S < 15.0, True)
    check("...and the client's own timeout is put back afterwards",
          session.rover.timeout, 10.0)
    session.close()


def test_a_look_refused_while_the_rovers_own_runs_is_asked_again():
    """M3 session 11, 2026-10-07: the rover's own look ran past the 9 s the
    daemon waits for it, and the run's look was refused and counted as a
    failed goal. It is now asked again, as a new action, a few seconds on."""
    session = Session()
    _arriving(session)
    original = session.rover._ask
    refusals = [1]

    def busy_once(name, arguments):
        if (name == "autonomy_act" and arguments.get("action") == "world_inspect"
                and refusals[0]):
            refusals[0] -= 1
            return {"ok": False, "refused": "refused",
                    "error": "an inspection has been running for 11 s; this one "
                             "was not started"}
        return original(name, arguments)

    session.rover._ask = busy_once
    got = session.executive.once()
    looks = [c for c in session.calls(got["episode"]) if c["call"] == "world_inspect"]
    check("the goal succeeds although the rover was busy with its own look",
          got.get("outcome"), "succeeded")
    check("...the refused look and the one asked again are both recorded",
          [bool(c.get("ok")) for c in looks], [False, True])
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
        check("...and its identifier begins with it",
              action_id.startswith(got["episode"]), True)
    session.close()


#: A room with nothing in it to look at and no unmapped floor: a run here has
#: nothing worth doing from the start.
EMPTY = ["##########",
         "#........#",
         "#..R.....#",
         "#........#",
         "##########"]


def test_a_run_with_nothing_left_worth_doing_goes_back_and_ends():
    """The owner's word on 2026-10-03: not fifteen minutes of standing about on
    a draining battery, but back to where the run started, and the run over."""
    rover = test_fakes.ActingRover(room=EMPTY, entities=[])
    session = Session(rover=rover)
    start = rover.permission.run.start
    rover.at = (start["x_m"] + 0.5, start["y_m"])
    _arriving(session)
    summary = session.executive.loop()
    check("it drove back to where the run started",
          {k: rover.moves[-1].get(k) for k in ("x_m", "y_m", "heading_deg")},
          {k: start[k] for k in ("x_m", "y_m", "heading_deg")})
    check("...on the map the run started on", rover.moves[-1]["map_id"],
          start["map_id"])
    check("...and the run is over", rover.permission.status()["enabled"], False)
    check("...for that reason, not a stop",
          (rover.permission.status()["latched"],
           "nothing left worth doing" in summary["ended"]), (False, True))
    check("...without waiting first", sum(session.slept) < executive_mod.IDLE_S,
          True)
    home = session.episodes()[0]
    events = session.events(home["ref"])
    decision = next(one for one in events if one["kind"] == "decision")
    check("the trip is an episode with a decision of its own",
          decision["body"]["chose"], executive_mod.RETURN_GOAL)
    check("...and the drive in it",
          [one["call"] for one in session.calls(home["ref"])], ["drive_to"])
    session.close()


def test_a_run_already_where_it_started_ends_without_driving():
    rover = test_fakes.ActingRover(room=EMPTY, entities=[])
    session = Session(rover=rover)
    summary = session.executive.loop()
    check("nothing was sent to the rover", rover.moves, [])
    check("...and the run is over", rover.permission.status()["enabled"], False)
    check("...saying it was already there",
          "already" in summary["ended"], True)
    session.close()


def test_a_run_that_cannot_act_waits_a_while_then_goes_back():
    """A camera that has failed is a gate, not nothing to do: it may clear, so
    the run waits -- but not for ever, which is the battery again."""
    rover = test_fakes.ActingRover(room=EMPTY, entities=[])
    session = Session(rover=rover)
    start = rover.permission.run.start
    rover.at = (start["x_m"] + 0.5, start["y_m"])
    rover.world_status = "error"
    _arriving(session)
    summary = session.executive.loop()
    check("it waited before giving up",
          sum(session.slept) >= executive_mod.GATED_GIVE_UP_S, True)
    check("...in naps short enough to renew through",
          max(session.slept) <= executive_mod.IDLE_NAP_S, True)
    check("...then went back", (rover.moves[-1]["x_m"], rover.moves[-1]["y_m"]),
          (start["x_m"], start["y_m"]))
    check("...and ended the run saying why",
          ("could not act" in summary["ended"],
           rover.permission.status()["enabled"]), (True, False))
    session.close()


def test_standing_still_is_not_mistaken_for_a_dead_executive():
    """The fault the rover found on the first turn it was ever asked for.

    A rover that cannot act just now waits, and the wait is twice the length of
    the permission's lease. A loop that slept through it in one go stopped
    renewing, and the daemon -- correctly, by its own rules -- took the wheels
    back from an executive that was merely waiting for the room to change.
    """
    rover = test_fakes.ActingRover(room=EMPTY, entities=[])
    rover.world_status = "error"
    session = Session(rover=rover)
    check("the wait is longer than the lease",
          executive_mod.IDLE_S > permission.PERMIT_TTL_S, True)
    got = session.executive.once()
    check("nothing was acted on", got["acted"], False)
    check("...but the turn is still an episode",
          session.store.outcome(got["episode"])["outcome"], "abandoned")
    check("and the run survives the wait",
          rover.permission.status()["enabled"], True)
    check("...having been renewed several times",
          (rover.permission.permit or {}).get("renewals", 0) >= 2, True)
    session.close()


# --- the ways a turn is cut short --------------------------------------------

def test_a_loop_that_did_what_it_was_asked_says_that_rather_than_a_riddle():
    """The run is still open when `--turns` runs out, and the report has to say
    so in those words. Reading the rover's own answer regardless produced "the
    run ended: a run is open", which is the sort of sentence that makes a person
    distrust the rest of the record."""
    session = Session()
    _arriving(session)
    summary = session.executive.loop(turns=1)
    check("it did the turn it was asked for", summary["turns"], 1)
    check("...and says that is why it stopped", summary["ended"],
          "the executive finished the turns it was asked for")
    check("...with the run still open for the next one",
          session.rover.permission.status()["enabled"], True)
    session.executive.release()
    check("...until it is handed back",
          session.rover.permission.status()["enabled"], False)
    session.close()


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


def test_a_low_battery_does_not_stop_the_turn():
    """No battery floor since 2026-10-02: an autonomous run is conditioned on
    the battery as every other drive is."""
    session = Session(rover=a_rover(volts=10.4))
    got = session.executive.once()
    check("a low pack is no reason to stop",
          "battery" in (got["why"] or "").lower(), False)
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
    permit = session.executive.renew()
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


def _chose(session: Session, episode: str) -> str:
    decisions = [one["body"] for one in session.events(episode)
                 if one["kind"] == "decision"]
    return str(decisions[-1].get("chose") or "") if decisions else ""


def _blocked(session: Session) -> None:
    """Every drive fails the way Nav2 failed on 2026-10-03."""
    original = session.rover._autonomy_status

    def once(arguments):
        rover = session.rover
        if rover.driving:
            rover.arrive("blocked: there is no route to there that the rover "
                         "fits through")
        return original(arguments)

    session.rover._autonomy_status = once


def _place(chose: str) -> tuple[float, float]:
    x, y = chose.rsplit("@", 1)[1].split(",")
    return float(x), float(y)


def test_a_place_navigation_could_not_reach_is_not_driven_to_again():
    """Found on 2026-10-03: the same frontier, one the rover could not fit
    through to, was driven at four times, forty seconds of recoveries each,
    until three failures in a row ended the run."""
    session = Session()
    _blocked(session)
    first = session.executive.once()
    check("the drive failed", first["outcome"], "interrupted")
    was = _place(_chose(session, first["episode"]))
    second = session.executive.once()
    chose = _chose(session, second["episode"])
    check("the next turn goes somewhere else, or nowhere",
          "@" not in chose or math.dist(was, _place(chose)) > cooling.UNREACHABLE_M,
          True)
    session.close()


def test_a_goal_that_got_nowhere_is_not_chosen_again():
    """The fault of 2026-10-03, run/e3efe1d1/2: the same look at object:7 was
    chosen twenty-two times in a row, because the look left it no better and
    the cooling that should have put it aside counted looks the world state
    never recorded. Here the fake rover's look changes nothing either."""
    session = Session()
    _arriving(session)
    first = session.executive.once()
    chosen = _chose(session, first["episode"])
    check("the first turn chose a look at something",
          chosen.startswith("improve_geometry:"), True)
    check("...which left it no better and put it aside",
          first["measured"].get("put_aside"), True)
    second = session.executive.once()
    target = chosen.split(":", 1)[1].split("@", 1)[0]
    check("the next turn does not choose the same thing again",
          f":{target}@" in _chose(session, second["episode"]), False)
    session.close()


TESTS = (test_an_m4_trial_copies_the_store_and_looks_again_from_where_it_stands,
         test_an_m4_trial_looks_only_at_its_targets,
         test_a_relook_that_fails_does_not_end_the_attempt,
         test_an_ordinary_run_neither_copies_the_store_nor_looks_again,
         test_a_relook_is_tilted_for_the_things_elevation_from_where_it_stands,
         test_a_look_that_could_see_the_place_and_found_nothing_sets_it_aside_longer,
         test_a_geometry_look_tilts_level_only_when_the_thing_needs_it,
         
    test_a_place_navigation_could_not_reach_is_not_driven_to_again,
    test_a_goal_that_got_nowhere_is_not_chosen_again,
    test_one_turn_drives_looks_and_writes_down_what_changed,
    test_a_geometry_goal_faces_its_thing_and_aims_the_look_at_it,
    test_a_geometry_look_records_and_leaves_settling_to_the_rover,
    test_a_look_refused_while_the_rovers_own_runs_is_asked_again,
    test_every_movement_names_the_episode_and_the_action_that_asked_for_it,
    test_a_run_with_nothing_left_worth_doing_goes_back_and_ends,
    test_a_run_already_where_it_started_ends_without_driving,
    test_a_run_that_cannot_act_waits_a_while_then_goes_back,
    test_standing_still_is_not_mistaken_for_a_dead_executive,
    test_a_loop_that_did_what_it_was_asked_says_that_rather_than_a_riddle,
    test_a_person_stopping_the_rover_ends_the_turn_and_the_loop,
    test_a_stop_in_any_state_of_the_machine_aborts_the_turn,
    test_permission_running_out_mid_drive_ends_the_turn,
    test_a_goal_that_never_arrives_times_out,
    test_the_daemon_refusing_an_action_ends_the_turn_without_moving,
    test_a_low_battery_does_not_stop_the_turn,
    test_losing_the_daemon_ends_the_loop_rather_than_the_rover,
    test_a_look_that_fails_ends_the_turn_after_the_drive,
    test_the_executive_cannot_move_the_rover_except_through_a_permit,
    test_the_executive_cannot_give_itself_back_the_authority_a_person_took,
    test_nothing_in_a_turn_asks_a_model_anything,
    test_an_action_the_rover_would_not_admit_is_refused_before_anything_runs,
    test_a_repeat_of_an_action_is_refused_rather_than_done_twice,
)
