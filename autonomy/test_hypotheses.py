"""Checks for M0a's hypothesis inspections (R-AUT-12), from the goal to the record.

Two halves. The first is the generator and the scorer: which claims become
inspections, where they are looked at from, and when a place is not asked
again. The second drives whole attempts through the executive against the fake
rover holding the real permission rules, and is the list M0a's replay criterion
names: a bounded attempt, a refusal, a stale map, repeated goal generation, a
budget running out, a look that cannot be pointed -- and nothing afterwards
that acts on which thing it was.
"""
from __future__ import annotations

import json
import math
import tempfile
from typing import Any

import decide
import events
import executive as executive_mod
import hypotheses
import scenarios
import scoring
import situation as situation_mod
import store as store_mod
import test_fakes
from test_harness import check

#: Four metres by three of mapped floor, at a tenth of a metre a cell, with the
#: rover near one corner: room for a viewpoint a metre or two from anything.
ROOM = (["#" * 40]
        + ["#" + "." * 38 + "#" for _ in range(24)]
        + ["#" + "." * 4 + "R" + "." * 33 + "#"]
        + ["#" + "." * 38 + "#" for _ in range(3)]
        + ["#" * 40])

M0A = scoring.Weights(m0a_protocol=True)


def a_claim(entity_id: str = "object:19", x: float = 2.5, y: float = 1.5, *,
            height_m: float | None = 0.3, uncertainty_m: float = 0.25,
            **fields: Any) -> dict[str, Any]:
    thing = scenarios.thing(entity_id, x, y, uncertainty_m=uncertainty_m,
                            **fields)
    if height_m is not None and thing["placement"]:
        thing["placement"]["height_m"] = height_m
    return thing


def a_situation(entities, **extra) -> situation_mod.Situation:
    return situation_mod.Situation(scenarios.situation(ROOM, entities=entities,
                                                       **extra))


def inspections_of(found) -> list:
    return [one for one in found if one.type == hypotheses.GOAL_TYPE]


# --- what is proposed, and from where ----------------------------------------

def test_inspections_are_proposed_only_under_the_m0a_protocol():
    here = a_situation([a_claim()])
    off = scoring.consider(here, scoring.DEFAULT, authority=True)
    check("outside the protocol no inspection is even considered",
          [one["candidate"]["type"] for one in off["considered"]
           if one["candidate"]["type"] == hypotheses.GOAL_TYPE], [])
    on = scoring.consider(here, M0A, authority=True)
    kinds = {one["candidate"]["type"]: one["vetoes"] for one in on["considered"]}
    check("under it the claim is considered", hypotheses.GOAL_TYPE in kinds, True)
    check("...and every other kind of goal is refused for the protocol",
          all(any(v["veto"] == "the M0a protocol" for v in vetoes)
              for kind, vetoes in kinds.items()
              if kind != hypotheses.GOAL_TYPE), True)


def test_the_viewpoint_is_chosen_on_the_map_and_never_on_the_claim():
    here = a_situation([a_claim(uncertainty_m=0.45)])
    found = inspections_of(hypotheses.generate(here))
    check("one inspection for the one claim", len(found), 1)
    one = found[0]
    goal = one.constraints["goal"]
    gap = math.hypot(goal["x_m"] - 2.5, goal["y_m"] - 1.5)
    check("the rover is sent no nearer than the claim's uncertainty and its "
          "own footprint", gap >= 0.45 + hypotheses.STANDOFF_M - 0.05, True)
    check("...and no further than the band", gap <= hypotheses.FAR_M + 0.05,
          True)
    check("...on floor the map calls free", one.constraints["on_free_floor"],
          True)
    facing = math.degrees(math.atan2(1.5 - goal["y_m"], 2.5 - goal["x_m"]))
    check("...facing the place", round(goal["heading_deg"] - facing, 0), 0.0)
    limits = one.constraints["inspection"]["limits"]
    check("the request carries finite limits under the daemon's ceilings",
          (0 < limits["travel_m"] <= 8.0, 0 < limits["seconds"] <= 180.0,
           limits["attempts"]), (True, True, hypotheses.ATTEMPTS))
    detail = one.gain_detail
    check("...and its claim, its alternatives and its question",
          (detail["claim"]["x_m"], len(detail["alternatives"]) >= 2,
           bool(detail["question"])), (2.5, True, True))


def test_the_tilt_follows_where_the_place_is():
    low = inspections_of(hypotheses.generate(
        a_situation([a_claim(height_m=-0.2)])))[0]
    high = inspections_of(hypotheses.generate(
        a_situation([a_claim(height_m=0.9)])))[0]
    check("a place below the camera is looked at level",
          low.constraints["tilt_deg"], 0.0)
    check("...one above it from rest, twenty up", high.constraints["tilt_deg"],
          20.0)

    # Found on the rover on 2026-10-02: a dining-chair seat 0.19 m above the
    # camera, looked at from 1.7 m. Neither tilt fits the whole patch, and the
    # first tilt in the list was taken -- twenty up, which put the seat 14
    # degrees below the middle of the picture and its patch off the bottom
    # edge, where level would have had it 6 degrees above the middle. Three
    # checks at removed chairs came back "outside the depth camera's view".
    seat = hypotheses._fits({"uncertainty_m": 0.147, "height_m": 0.19,
                              "height_sigma_m": 0.079}, 1.7)
    check("when no tilt fits the whole patch, the one nearer the middle is taken",
          (seat["tilt_deg"], seat["patch_fits"]), (0.0, False))


def test_what_one_look_cannot_test_is_not_proposed():
    check("a claim placed looser than one look can test",
          inspections_of(hypotheses.generate(
              a_situation([a_claim(uncertainty_m=0.6)]))), [])
    check("a thing placed on another map session",
          inspections_of(hypotheses.generate(
              a_situation([a_claim(map_session=6)]))), [])
    check("a thing never placed",
          inspections_of(hypotheses.generate(
              a_situation([a_claim(placed=False)]))), [])
    # Placed from one look, it matches to 0.05 m but claims what one look has
    # been measured to be worth (world_state/locate.py, STATED_SINGLE_LOOK_M).
    one_look = a_claim(uncertainty_m=0.05)
    one_look["placement"]["stated_uncertainty_m"] = 1.0
    check("a thing placed from one look, by what it claims rather than what "
          "it matches with", inspections_of(hypotheses.generate(
              a_situation([one_look]))), [])


def test_a_place_already_spent_or_answered_is_refused_from_the_record():
    def ledger(*outcomes, at=(2.5, 1.5)):
        return [{"case": "c", "target": {"x_m": at[0], "y_m": at[1]},
                 "map_session": 7, "outcome": one, "at": 10.0 + n}
                for n, one in enumerate(outcomes)]

    def vetoes(entities, inspections):
        got = scoring.consider(a_situation(entities, inspections=inspections),
                               M0A, authority=True)
        return [v["veto"] for one in got["considered"]
                if one["candidate"]["type"] == hypotheses.GOAL_TYPE
                for v in one["vetoes"]]

    once = inspections_of(hypotheses.generate(a_situation(
        [a_claim()], inspections=ledger("unresolved"))))[0]
    check("one unresolved attempt leaves half the doubt",
          (once.gain_detail["doubt"], once.constraints["case_attempts"]),
          (0.5, 1))
    check("two attempts and the place is refused",
          "attempts spent" in vetoes([a_claim()],
                                     ledger("unresolved", "unresolved")), True)
    check("an answered place is refused",
          "already answered" in vetoes([a_claim()], ledger("contradicted")),
          True)
    check("...under any name: a thing renamed by a merge, 0.3 m along",
          "already answered" in vetoes([a_claim("object:77", 2.7, 1.3)],
                                       ledger("supported")), True)
    check("...while a place a metre away is its own case",
          vetoes([a_claim("object:78", 2.5, 0.5)], ledger("supported")), [])

    # Found on the rover on 2026-10-02: the twelve claims nearest the rover
    # were all spent, so a fresh place 1.6 m away was never proposed and five
    # turns came back "nothing to do". Spent places no longer take the slots.
    spent_near = [a_claim(f"object:{n}", 2.0 + 0.05 * n, 1.5)
                  for n in range(hypotheses.CLAIM_LIMIT)]
    spent_record = [entry for n in range(hypotheses.CLAIM_LIMIT)
                    for entry in ledger("unresolved", "unresolved",
                                        at=(2.0 + 0.05 * n, 1.5))]
    fresh = a_claim("object:99", 2.5, 2.6)    # 2.9 m from the rover, behind them all
    got = inspections_of(hypotheses.generate(a_situation(
        spent_near + [fresh], inspections=spent_record)))
    check("a fresh place behind twelve spent ones is still proposed",
          "object:99" in [one.target for one in got], True)


# --- whole attempts, against the fake rover -----------------------------------

class Attempt:
    """A store, a rover holding the real permission rules, and an executive
    running the M0a protocol, with the clock in the test's hand."""

    def __init__(self, entities=None, **budget) -> None:
        self.dir = tempfile.mkdtemp(prefix="ugv-m0a-")
        self.store = store_mod.EpisodeStore(self.dir)
        self.rover = test_fakes.ActingRover(room=ROOM,
                                            entities=entities or [a_claim()])
        self.run = self.rover.enable(**budget)
        self.executive = executive_mod.Executive(
            self.store, self.rover, M0A, sleep=self._sleep,
            now=self.rover.clock, log=lambda _line: None)
        self.executive.attach()
        self.arrive = True

    def _sleep(self, seconds: float) -> None:
        self.rover.clock.tick(seconds)
        if self.arrive and self.rover.driving:
            asked = self.rover.moves[-1]
            self.rover.arrive(at=(asked["x_m"], asked["y_m"]))
        self.rover.watchdog()

    def turn(self) -> dict[str, Any]:
        # The executive reads the ledger the way a shadow deliberation does.
        got = self.executive.once()
        return got

    def events(self, episode: str) -> list[dict[str, Any]]:
        return self.store.episode(episode)["events"]

    def inspections(self, episode: str) -> list[dict[str, Any]]:
        return [one["body"] for one in self.events(episode)
                if one["kind"] == "inspection"]

    def ledger(self) -> list[dict[str, Any]]:
        return decide.inspections(self.store)


def test_an_attempt_is_frozen_dispatched_checked_and_recorded():
    attempt = Attempt()
    got = attempt.turn()
    episode = got["episode"]
    kinds = [one["kind"] for one in attempt.events(episode)]
    check("the request is written down before anything is dispatched",
          kinds.index("inspection") < kinds.index("dispatch"), True)
    request, result = attempt.inspections(episode)
    check("...with its claim, question, alternatives, viewpoint and limits",
          all(request.get(k) for k in ("claim", "question", "alternatives",
                                        "viewpoint", "limits")), True)
    check("...and the source looks frozen from the world state",
          request["source"][:2], [100, 101])
    sent = attempt.rover.moves[-1]
    check("the drive carries the case and its limits to the daemon",
          (sent["inspection"]["case"], "heading_deg" in sent),
          (request["case"], True))
    check("the look was a fresh one that keeps its depth, at the chosen tilt",
          attempt.rover.permission.actions[f"{episode}#2"]["params"]
          .get("keep_depth"), True)
    asked = attempt.rover.checked[-1]
    check("the check is asked about the frozen claim and its own looks",
          (asked["claim"], asked["source"]), (request["claim"],
                                              request["source"]))
    check("the result is recorded with its outcome",
          (result["stage"], result["outcome"]), ("result", "supported"))
    check("...and spent against the place in the ledger",
          [(one["outcome"], one["attempt"]) for one in attempt.ledger()],
          [("supported", 1)])
    check("the attempt closes as a completed turn",
          attempt.store.outcome(episode)["outcome"], "succeeded")


def test_nothing_after_the_answer_acts_on_which_thing_it_was():
    attempt = Attempt()
    attempt.turn()
    after = attempt.rover.asked[attempt.rover.asked.index("world_state_check")
                                + 1:]
    check("after the check the executive only reads",
          [name for name in after if name not in
           ("autonomy_status", "autonomy_permit", "nav_status", "nav_grid",
            "battery", "world_state_summary", "world_state_entities",
            "world_building")], [])
    check("...and moves nothing", len(attempt.rover.moves), 1)
    got = attempt.turn()
    # With nothing else to do the run then drives back to where it started,
    # which is not an inspection: only drives that carry one are counted.
    check("the next turn does not inspect the answered place again",
          len([one for one in attempt.rover.moves if one.get("inspection")]), 1)
    check("...because it is refused as answered",
          "already answered" in json.dumps(
              attempt.store.episode(got["episode"])["events"]), True)


def test_a_refused_drive_is_an_attempt_with_an_unresolved_answer():
    attempt = Attempt()
    original = attempt.executive._read

    def read_then_lose_the_pose():
        here = original()
        # Trusted when the decision was made, not when the drive is dispatched:
        # the daemon's pose gate is what refuses it.
        attempt.rover.trusted = False
        return here

    attempt.executive._read = read_then_lose_the_pose
    got = attempt.turn()
    check("the turn was interrupted", got["outcome"], "interrupted")
    result = attempt.inspections(got["episode"])[-1]
    check("...and the attempt still has a result", (result["stage"],
          result["outcome"]), ("result", "unresolved"))
    check("...saying the daemon refused it", "refused" in result["why"], True)
    check("...and it is spent against the place",
          [one["outcome"] for one in attempt.ledger()], ["unresolved"])


def test_a_map_replaced_under_the_decision_refuses_the_drive():
    attempt = Attempt()
    original = attempt.executive._read

    def read_then_replace():
        here = original()
        attempt.rover.map_id = "m2"           # the map changes after the decision
        return here

    attempt.executive._read = read_then_replace
    got = attempt.turn()
    result = attempt.inspections(got["episode"])[-1]
    check("a goal chosen on a map the rover is no longer on is refused",
          (result["outcome"], "stale map" in result["code"]),
          ("unresolved", True))


def test_a_drive_that_runs_past_its_limit_is_stopped_and_recorded():
    attempt = Attempt()
    attempt.arrive = False
    rover = attempt.rover
    # The route the rover is driving goes round, so the attempt's metres run
    # out long before the viewpoint is reached.
    original = attempt._sleep

    def wander(seconds):
        if rover.driving:
            x, y = rover.at or scenarios.rover_in(ROOM)
            rover.at = (x, y + 0.5)
            rover.permission.moved(rover.at, rover.map_id, elapsed_s=10.0)
        original(seconds)

    attempt.executive.sleep = wander
    got = attempt.turn()
    result = attempt.inspections(got["episode"])[-1]
    check("the attempt ends when its travel is spent",
          (result["outcome"], "drove its" in result["why"]),
          ("unresolved", True))
    check("...with the rover stopped, and the run still open",
          (rover.driving, rover.permission.status()["enabled"]), (False, True))


def test_a_look_that_cannot_be_pointed_answers_unresolved():
    attempt = Attempt()
    attempt.rover.look_has_pose = False
    got = attempt.turn()
    result = attempt.inspections(got["episode"])[-1]
    check("a look whose direction was withheld is an unresolved answer",
          (result["outcome"], result["code"]), ("unresolved", "direction"))
    check("...not a failed turn", got["outcome"], "succeeded")


def test_regenerating_the_goal_does_not_restart_its_budget():
    attempt = Attempt()
    attempt.rover.check_answer = {"ok": True, "outcome": "unresolved",
                                  "code": "occluded", "why": "in the way",
                                  "evidence": {}}
    attempt.turn()
    # The resolver renames the thing between attempts: a merge.
    attempt.rover.entities = [a_claim("object:77", 2.6, 1.6)]
    attempt.turn()
    got = attempt.turn()
    check("two attempts were made on the place",
          len([one for one in attempt.rover.moves if one.get("inspection")]), 2)
    check("...and a third under the new name is refused",
          "attempts spent" in json.dumps(
              attempt.store.episode(got["episode"])["events"]), True)
    check("...with both attempts in the ledger against the one place",
          [one["claimed_by"] for one in attempt.ledger()],
          ["object:19", "object:77"])


def test_an_inspection_event_must_say_what_it_answered():
    try:
        events.inspection("case", "result", outcome="probably")
        refused = False
    except ValueError:
        refused = True
    check("a result that is not supported, contradicted or unresolved is "
          "refused where it is recorded", refused, True)


TESTS = (
    test_inspections_are_proposed_only_under_the_m0a_protocol,
    test_the_viewpoint_is_chosen_on_the_map_and_never_on_the_claim,
    test_the_tilt_follows_where_the_place_is,
    test_what_one_look_cannot_test_is_not_proposed,
    test_a_place_already_spent_or_answered_is_refused_from_the_record,
    test_an_attempt_is_frozen_dispatched_checked_and_recorded,
    test_nothing_after_the_answer_acts_on_which_thing_it_was,
    test_a_refused_drive_is_an_attempt_with_an_unresolved_answer,
    test_a_map_replaced_under_the_decision_refuses_the_drive,
    test_a_drive_that_runs_past_its_limit_is_stopped_and_recorded,
    test_a_look_that_cannot_be_pointed_answers_unresolved,
    test_regenerating_the_goal_does_not_restart_its_budget,
    test_an_inspection_event_must_say_what_it_answered,
)
