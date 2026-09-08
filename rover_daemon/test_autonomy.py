"""Offline checks for the permission that lets the rover move by itself.

Everything the plan calls a daemon-enforced permission is checked here, with no
ROS, no board and no executive: a fake navigator that remembers what it was told
to do, a driver board that reports a voltage, and a clock the test moves by hand
so that expiry happens in a microsecond rather than in fifteen seconds.

**What is worth the closest reading is the watchdog.** Every other check here
asks whether a refusal happens when the executive asks politely, which is the
easy half. The watchdog is what happens when the executive does not ask at all
-- killed, hung, or on the other end of a connection that has gone -- and Nav2
is perfectly healthy and would go on driving. Those checks move the clock past
the permission's expiry and then assert that the fake navigator was told to
stop.

The hardware half of this cannot be checked here and is not pretended at: that a
stop reaches the wheels in a measured distance is
[M3](../docs/plans/autonomous-curiosity.md)'s supervised trial, on the rover.
"""
from __future__ import annotations

from typing import Any

from test_fakes import FakeLink              # noqa: F401 -- sets sys.path up

import permission as permission_mod
import rover as rover_mod
from test_harness import check


class Clock:
    """A clock the test winds forward. Both hands, so that a status a person
    would read carries a stamp as well."""

    def __init__(self, at: float = 1000.0) -> None:
        self.at = float(at)

    def __call__(self) -> float:
        return self.at

    def tick(self, seconds: float) -> float:
        self.at += float(seconds)
        return self.at


class FakeNav:
    """A navigator that remembers what it was told, and says where it is.

    Deliberately not a stand-in for Nav2: nothing here plans, drives or arrives.
    What the permission rules care about is exactly what this can answer -- is
    the rover driving, where does it think it is, which map is that, and was it
    told to stop -- and a fake that did more would be a fake with opinions.
    """

    def __init__(self, *, where=(0.0, 0.0), map_id="map-1", trusted=True,
                 settled=True) -> None:
        self.where = where
        self.map_id = map_id
        self.trusted = trusted
        self.settled = settled
        self.driving = False
        self.stops = 0
        self.sent: list[dict[str, Any]] = []
        self.told = None
        self.started = True

    def status(self, since_seq=None) -> dict[str, Any]:
        pose = (None if self.where is None else
                {"x_m": self.where[0], "y_m": self.where[1],
                 "heading_deg": 0.0})
        return {"driving": self.driving, "exploring": False, "estop": False,
                "pose": pose, "map_id": self.map_id,
                "map_settled": self.settled, "map_kept": True,
                "position_trusted": self.trusted, "match_score": 0.9}

    def stop(self) -> dict[str, Any]:
        self.stops += 1
        self.driving = False
        return {"stopped": True, "latched": False}

    def drive_to_in_background(self, x_m, y_m, heading_deg=None,
                               speed_ms=None, for_what=None) -> dict[str, Any]:
        self.sent.append({"x_m": x_m, "y_m": y_m, "heading_deg": heading_deg,
                          "for_what": dict(for_what or {})})
        if not self.started:
            return {"started": False, "running": "errand"}
        self.driving = True
        return {"started": True, "running": "errand"}

    def close(self) -> None:
        pass


class Ended:
    """What a move hands back when it is over. `Outcome`'s two read fields."""

    def __init__(self, reason: str, detail: str = "") -> None:
        self.reason, self.detail = reason, detail


def a_rover(clock: Clock, *, volts: int = 1207, nav: FakeNav | None = None):
    """A daemon's Rover with a fake board and a fake navigator under it."""
    rover = rover_mod.Rover(FakeLink(volts=volts), "unused", device=None)
    rover.permission = permission_mod.Permission(clock=clock, wall=clock,
                                                 boot="aabbccdd")
    rover.nav = nav if nav is not None else FakeNav()
    return rover


def enabled(rover, **budget) -> str:
    """Open a run the way a person does, and hand back its identifier."""
    answer = rover.call("autonomy_enable",
                        {"by": "the owner", "why": "a supervised trial",
                         "budget": budget})
    return answer["run"]["id"]


def permitted(rover, run: str) -> str:
    return rover.call("autonomy_permit", {"run": run})["permit"]


def act(rover, permit: str, action: str, action_id: str, **params):
    return rover.call("autonomy_act",
                      {"permit": permit, "action": action,
                       "action_id": action_id, "episode": "au/1234/episode:1",
                       "params": params})


# --- nothing may move until somebody says so ---------------------------------

def test_a_fresh_daemon_has_no_authority():
    rover = a_rover(Clock())
    status = rover.call("autonomy_status", {})
    check("a daemon starts with autonomy off", status["enabled"], False)
    check("...and says why", status["why"],
          "autonomy has not been enabled since this daemon started")
    check("...with no latch, because nobody stopped anything",
          status["latched"], False)

    asked = rover.call("autonomy_permit", {"run": "run/whatever/1"})
    check("permission cannot be had without a run", asked["ok"], False)
    check("...and says so plainly", asked["error"], "autonomy is not enabled")

    moved = act(rover, "permit/whatever/1", "drive_to", "a#1", x_m=1.0, y_m=2.0)
    check("and an action is refused outright", moved["ok"], False)
    check("...for the reason a person would give", moved["refused"], "no run")
    check("...and nothing was sent to the navigator", rover.nav.sent, [])


def test_a_run_is_opened_by_a_person_and_bounded_before_it_starts():
    rover = a_rover(Clock())
    unnamed = rover.call("autonomy_enable", {})
    check("a run nobody is named for is refused", unnamed["ok"], False)

    run = rover.call("autonomy_enable",
                     {"by": "the owner", "why": "trial 1",
                      "budget": {"seconds": 120.0, "travel_m": 10.0}})
    check("a person opens one", run["ok"], True)
    check("...with the budget they asked for",
          (run["run"]["budget"]["seconds"], run["run"]["budget"]["travel_m"]),
          (120.0, 10.0))
    check("...and the standing limits for what they did not",
          run["run"]["budget"]["actions"], permission_mod.RUN_MAX_ACTIONS)
    check("...recorded against whoever opened it", run["run"]["by"],
          "the owner")

    again = rover.call("autonomy_enable", {"by": "the owner"})
    check("a second press does not open a second run", again["ok"], False)

    rover.call("autonomy_stop", {"by": "the owner"})
    wide = rover.call("autonomy_enable",
                      {"by": "the owner",
                       "budget": {"seconds": permission_mod.RUN_MAX_S * 2}})
    check("and the standing limits cannot be widened by asking",
          wide["ok"], False)
    check("...naming the limit", wide["error"],
          f"seconds may be at most {permission_mod.RUN_MAX_S}")


def test_only_the_three_admitted_operations_are_dispatched():
    clock = Clock()
    rover = a_rover(clock)
    permit = permitted(rover, enabled(rover))

    for name in ("run_script", "start_script", "explore", "clear_map",
                 "turn_in_place"):
        refused = act(rover, permit, name, f"a#{name}")
        check(f"{name} is not an autonomy action", refused["refused"],
              "not an autonomy action")
    check("nothing reached the navigator", rover.nav.sent, [])

    short = rover.call("autonomy_act",
                       {"permit": permit, "action": "drive_to",
                        "action_id": "", "episode": "au/1234/episode:1",
                        "params": {"x_m": 1.0, "y_m": 1.0}})
    check("an action with no identifier is refused", short["refused"],
          "unattributable")
    nowhere = act(rover, permit, "drive_to", "a#2", y_m=1.0)
    check("...and a drive with half a place is refused", nowhere["refused"],
          "incomplete")


# --- the permission is a lease ------------------------------------------------

def test_a_permit_that_ran_out_refuses_at_dispatch():
    clock = Clock()
    rover = a_rover(clock)
    run = enabled(rover)
    permit = permitted(rover, run)

    clock.tick(permission_mod.PERMIT_TTL_S + 0.1)
    refused = act(rover, permit, "drive_to", "a#1", x_m=1.0, y_m=1.0)
    check("an expired permission refuses the action", refused["refused"],
          "permit expired")
    check("...and nothing was sent", rover.nav.sent, [])

    renewed = rover.call("autonomy_permit", {"run": run})
    check("renewing brings it back", renewed["ok"], True)
    check("...as the same permission rather than a new one",
          renewed["permit"], permit)
    check("...counted", renewed["renewals"], 1)
    moved = act(rover, permit, "drive_to", "a#1", x_m=1.0, y_m=1.0)
    check("and now the drive goes", moved["ok"], True)
    check("...to the navigator, as a place",
          (rover.nav.sent[-1]["x_m"], rover.nav.sent[-1]["y_m"]), (1.0, 1.0))


def test_an_action_is_dispatched_once_and_repeats_are_answered():
    rover = a_rover(Clock())
    permit = permitted(rover, enabled(rover))
    first = act(rover, permit, "drive_to", "a#1", x_m=2.0, y_m=0.0)
    check("the first request drives", first["ok"], True)

    again = act(rover, permit, "drive_to", "a#1", x_m=2.0, y_m=0.0)
    check("the same request again does not drive again", again["ok"], False)
    check("...it is refused as already done", again["refused"], "already done")
    check("...and says what happened the first time",
          again["already"]["action"], "drive_to")
    check("...having sent the navigator exactly one goal",
          len(rover.nav.sent), 1)


def test_a_goal_chosen_on_another_map_is_refused():
    rover = a_rover(Clock())
    permit = permitted(rover, enabled(rover))
    stale = act(rover, permit, "drive_to", "a#1", x_m=1.0, y_m=1.0,
                map_id="map-0")
    check("a goal from a map that has been replaced is refused",
          stale["refused"], "stale map")
    fresh = act(rover, permit, "drive_to", "a#2", x_m=1.0, y_m=1.0,
                map_id="map-1")
    check("...and the same goal on the live map is not", fresh["ok"], True)


def test_a_pose_nobody_trusts_stops_a_drive_and_not_a_look():
    rover = a_rover(Clock(), nav=FakeNav(trusted=False))
    rover._tool_world_inspect = lambda arguments: {"ok": True, "regions": 3}
    permit = permitted(rover, enabled(rover))

    refused = act(rover, permit, "drive_to", "a#1", x_m=1.0, y_m=1.0)
    check("an untrusted pose refuses a drive", refused["refused"], "pose")
    looked = act(rover, permit, "world_inspect", "a#2")
    check("...and does not refuse a look", looked["ok"], True)


def test_a_goal_outside_the_safe_area_is_refused():
    rover = a_rover(Clock())
    permit = permitted(rover, enabled(
        rover, geofence={"x_m": 0.0, "y_m": 0.0, "radius_m": 3.0}))
    out = act(rover, permit, "drive_to", "a#1", x_m=5.0, y_m=0.0)
    check("a goal outside the safe area is refused", out["refused"],
          "outside the safe area")
    inside = act(rover, permit, "drive_to", "a#2", x_m=1.0, y_m=1.0)
    check("...and one inside it is not", inside["ok"], True)


# --- the watchdog: what happens when nobody asks ------------------------------

def test_the_wheels_are_taken_back_when_the_permission_runs_out():
    clock = Clock()
    nav = FakeNav()
    rover = a_rover(clock, nav=nav)
    permit = permitted(rover, enabled(rover))
    act(rover, permit, "drive_to", "a#1", x_m=1.0, y_m=0.0)
    check("the rover is driving", nav.driving, True)

    rover.autonomy_tick()
    check("a tick inside the permission changes nothing", nav.stops, 0)

    # The executive is gone: nothing renews, and Nav2 is perfectly healthy.
    clock.tick(permission_mod.PERMIT_TTL_S + 1.0)
    why = rover.autonomy_tick()
    check("an executive that stopped renewing loses the wheels", nav.stops, 1)
    check("...and the run is closed", rover.permission.run.ended, why)
    check("...for the reason a person would want", "permission ran out" in why,
          True)
    check("...leaving autonomy off", rover.call("autonomy_status", {})["enabled"],
          False)
    check("...and not latched, because nobody stopped anything",
          rover.call("autonomy_status", {})["latched"], False)

    dead = act(rover, permit, "drive_to", "a#2", x_m=2.0, y_m=0.0)
    check("and the old permission is worth nothing afterwards",
          dead["refused"], "no run")


def test_travel_is_spent_by_where_the_rover_actually_gets_to():
    clock = Clock()
    nav = FakeNav(where=(0.0, 0.0))
    rover = a_rover(clock, nav=nav)
    run = enabled(rover, travel_m=1.0)
    permit = permitted(rover, run)
    act(rover, permit, "drive_to", "a#1", x_m=9.0, y_m=0.0)

    # A move nobody is waiting for, ticking along under it. Each step is well
    # inside what the chassis can drive in the time, so none of them is a jump,
    # and the first is measured from where the rover stood when it set off.
    for step in (0.3, 0.6, 0.9):
        clock.tick(permission_mod.TICK_S)
        rover.call("autonomy_permit", {"run": run})
        nav.where = (step, 0.0)
        rover.autonomy_tick()
    check("the run has spent what it drove",
          round(rover.permission.run.travel_m, 2), 0.9)
    check("...and is still going", nav.stops, 0)

    clock.tick(permission_mod.TICK_S)
    rover.call("autonomy_permit", {"run": run})
    nav.where = (1.05, 0.0)
    why = rover.autonomy_tick()
    check("and the distance budget takes the wheels back", nav.stops, 1)
    check("...saying which budget it was", "driven" in why, True)


def test_a_pose_that_jumps_ends_the_run():
    clock = Clock()
    nav = FakeNav(where=(0.0, 0.0))
    rover = a_rover(clock, nav=nav)
    permit = permitted(rover, enabled(rover))
    act(rover, permit, "drive_to", "a#1", x_m=1.0, y_m=0.0)
    clock.tick(permission_mod.TICK_S)
    rover.autonomy_tick()

    clock.tick(permission_mod.TICK_S)
    nav.where = (4.0, 0.0)                       # a refit, not eight seconds
    why = rover.autonomy_tick()
    check("a pose that jumps ends the run", nav.stops, 1)
    check("...saying what it saw", "jumped" in why, True)
    check("...and not charging the jump as travel",
          round(rover.permission.run.travel_m, 2), 0.0)


def test_the_map_changing_under_a_run_ends_it():
    clock = Clock()
    nav = FakeNav()
    rover = a_rover(clock, nav=nav)
    permit = permitted(rover, enabled(rover))
    act(rover, permit, "drive_to", "a#1", x_m=1.0, y_m=0.0)
    clock.tick(permission_mod.TICK_S)
    rover.autonomy_tick()

    nav.map_id = "map-2"
    clock.tick(permission_mod.TICK_S)
    why = rover.autonomy_tick()
    check("a new map ends the run", nav.stops, 1)
    check("...naming both", "map-1" in why and "map-2" in why, True)


def test_a_flat_battery_ends_the_run_and_refuses_a_drive():
    clock = Clock()
    rover = a_rover(clock, volts=1100)
    permit = permitted(rover, enabled(rover))
    refused = act(rover, permit, "drive_to", "a#1", x_m=1.0, y_m=0.0)
    check("a drive is refused under the battery floor", refused["refused"],
          "battery low")
    why = rover.autonomy_tick()
    check("...and the run does not wait to be asked again",
          rover.permission.run.ended, why)
    check("...saying what the pack reads", "11.00 V" in why, True)


def test_failures_in_a_row_end_the_run():
    clock = Clock()
    nav = FakeNav()
    nav.started = False                      # every goal is refused by the nav
    rover = a_rover(clock, nav=nav)
    run = enabled(rover)
    permit = permitted(rover, run)
    for n in range(permission_mod.RUN_MAX_FAILURES):
        answer = act(rover, permit, "drive_to", f"a#{n}", x_m=1.0, y_m=0.0)
        check(f"goal {n} did not start", answer["ok"], False)
    check("the failures are counted", rover.permission.run.failures,
          permission_mod.RUN_MAX_FAILURES)
    why = rover.autonomy_tick()
    check("and enough of them in a row ends the run",
          rover.permission.run.ended, why)
    check("...saying so", "in a row failed" in why, True)


# --- a person always wins -----------------------------------------------------

def test_stopping_the_rover_latches_autonomy_off():
    clock = Clock()
    rover = a_rover(clock)
    run = enabled(rover)
    permit = permitted(rover, run)

    rover.call("stop_driving", {})
    status = rover.call("autonomy_status", {})
    check("a person's stop latches autonomy off", status["latched"], True)
    check("...and ends the run", status["enabled"], False)

    refused = rover.call("autonomy_permit", {"run": run})
    check("permission cannot be renewed after a stop", refused["ok"], False)
    check("...and says the latch is why", refused.get("latched"), True)
    moved = act(rover, permit, "drive_to", "a#1", x_m=1.0, y_m=0.0)
    check("...and no action gets through", moved["refused"], "latched")

    back = rover.call("autonomy_enable", {"by": "the owner", "why": "resuming"})
    check("only a person re-enabling clears it", back["ok"], True)
    check("...as a new run rather than the old one",
          back["run"]["id"] != run, True)
    check("...with the latch gone",
          rover.call("autonomy_status", {})["latched"], False)


def test_driving_by_hand_takes_the_rover_back():
    clock = Clock()
    nav = FakeNav()
    rover = a_rover(clock, nav=nav)
    run = enabled(rover)
    permit = permitted(rover, run)
    act(rover, permit, "drive_to", "a#1", x_m=1.0, y_m=0.0)

    # Whatever the console's move does next does not matter here; that it
    # arrived does. FakeNav has no `drive_to`, so the tool answers with a
    # failure -- and the takeover has already happened by then.
    rover.call("drive_to", {"x_m": 0.0, "y_m": 0.0})
    status = rover.call("autonomy_status", {})
    check("a manual move while a run is open ends it", status["enabled"], False)
    check("...and latches, because somebody else is driving now",
          status["latched"], True)
    check("...naming what happened", "by hand" in status["latch"]["why"], True)


def test_driving_by_hand_with_no_run_open_is_just_driving():
    rover = a_rover(Clock())
    rover.call("drive_to", {"x_m": 1.0, "y_m": 1.0})
    check("manual driving on its own latches nothing",
          rover.call("autonomy_status", {})["latched"], False)


def test_autonomys_own_stop_does_not_latch_it_off():
    clock = Clock()
    nav = FakeNav()
    rover = a_rover(clock, nav=nav)
    run = enabled(rover)
    permit = permitted(rover, run)
    act(rover, permit, "drive_to", "a#1", x_m=1.0, y_m=0.0)
    stopped = act(rover, permit, "stop", "a#2")
    check("autonomy can stop its own move", stopped["ok"], True)
    check("...which reaches the navigator", nav.stops, 1)
    status = rover.call("autonomy_status", {})
    check("...without latching itself off", status["latched"], False)
    check("...and the run carries on", status["enabled"], True)


def test_a_restart_invalidates_every_permission_that_was_given():
    clock = Clock()
    rover = a_rover(clock)
    run = enabled(rover)
    permit = permitted(rover, run)

    # What a restart is, from here: a new Permission with a new boot token and
    # nothing carried across. Nothing is read from disk because nothing is
    # written to disk, which is the property being checked.
    rover.permission = permission_mod.Permission(clock=clock, wall=clock,
                                                 boot="eeff0011")
    check("a restarted daemon has no run",
          rover.call("autonomy_status", {})["enabled"], False)
    refused = rover.call("autonomy_permit", {"run": run})
    check("...and will not renew the old run", refused["ok"], False)
    dead = act(rover, permit, "drive_to", "a#1", x_m=1.0, y_m=0.0)
    check("...nor honour the old permission", dead["refused"], "no run")


def test_a_move_that_ends_is_recorded_against_the_action_that_asked_for_it():
    clock = Clock()
    nav = FakeNav()
    rover = a_rover(clock, nav=nav)
    permit = permitted(rover, enabled(rover))
    act(rover, permit, "drive_to", "a#1", x_m=1.0, y_m=0.0)
    check("the goal carries its action into the navigator",
          nav.sent[-1]["for_what"]["autonomy_action"], "a#1")
    check("...and its episode",
          nav.sent[-1]["for_what"]["episode"], "au/1234/episode:1")

    nav.driving = False                          # the move is over
    rover._trip_ended("errand", nav.sent[-1]["for_what"], Ended("arrived"))
    check("an arrival is recorded against the action",
          rover.permission.actions["a#1"]["ok"], True)
    check("...and clears the failure count", rover.permission.run.failures, 0)

    act(rover, permit, "drive_to", "a#2", x_m=2.0, y_m=0.0)
    nav.driving = False
    rover._trip_ended("errand", nav.sent[-1]["for_what"], Ended("blocked",
                                                               "a wall"))
    check("and a move that did not arrive counts as a failure",
          rover.permission.run.failures, 1)
    check("...with the navigator's own word for it",
          rover.permission.actions["a#2"]["detail"], "blocked: a wall")


TESTS = (
    test_a_fresh_daemon_has_no_authority,
    test_a_run_is_opened_by_a_person_and_bounded_before_it_starts,
    test_only_the_three_admitted_operations_are_dispatched,
    test_a_permit_that_ran_out_refuses_at_dispatch,
    test_an_action_is_dispatched_once_and_repeats_are_answered,
    test_a_goal_chosen_on_another_map_is_refused,
    test_a_pose_nobody_trusts_stops_a_drive_and_not_a_look,
    test_a_goal_outside_the_safe_area_is_refused,
    test_the_wheels_are_taken_back_when_the_permission_runs_out,
    test_travel_is_spent_by_where_the_rover_actually_gets_to,
    test_a_pose_that_jumps_ends_the_run,
    test_the_map_changing_under_a_run_ends_it,
    test_a_flat_battery_ends_the_run_and_refuses_a_drive,
    test_failures_in_a_row_end_the_run,
    test_stopping_the_rover_latches_autonomy_off,
    test_driving_by_hand_takes_the_rover_back,
    test_driving_by_hand_with_no_run_open_is_just_driving,
    test_autonomys_own_stop_does_not_latch_it_off,
    test_a_restart_invalidates_every_permission_that_was_given,
    test_a_move_that_ends_is_recorded_against_the_action_that_asked_for_it,
)
