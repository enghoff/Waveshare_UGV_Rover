"""What a candidate is worth, and the refusals that no score can buy past.

The important test in this file is the third one. A hard constraint that can be
outranked by a large enough number is not a constraint, and the way that failure
arrives is never a deliberate decision -- it is a weight somebody raised to make
the rover keener, in a scorer where the refusals were terms rather than gates.
So the check is not that the veto is applied but that it survives a purpose
weight of a thousand.
"""
from __future__ import annotations

import json
import os
import tempfile

import goals
import scoring
from situation import Situation
from test_fakes import a_situation, a_thing
from test_harness import check
from test_goals import CLOSET, FINISHED, HOUSE, ROOM


def _situation(rows=ROOM, **kwargs) -> Situation:
    return Situation(a_situation(rows, **kwargs))


def _a_thing_worth_looking_at(**kwargs):
    fields = {"uncertainty_m": 0.60, "major_deg": 90.0, **kwargs}
    return a_thing("object:8", 1.6, 1.0, **fields)


def test_every_term_of_the_score_is_written_down() -> None:
    here = _situation(entities=[_a_thing_worth_looking_at()])
    candidate = goals.improve_geometry(here)[0]
    terms = scoring.score(candidate, here)
    for name in ("gain", "purpose_relevance", "time_cost", "travel_cost",
                 "energy_cost", "switching_cost", "utility", "weights_version"):
        check(f"the decomposition carries {name}", name in terms, True)
    check("...and the total is the terms, to the rounding they are shown at",
          abs(terms["utility"]
              - (terms["purpose_relevance"] * terms["gain"]
                 - scoring.DEFAULT.w_time * terms["time_cost"]
                 - scoring.DEFAULT.w_travel * terms["travel_cost"]
                 - scoring.DEFAULT.w_energy * terms["energy_cost"]
                 - terms["switching_cost"])) < 0.001, True)
    check("...and says which weights produced it", terms["weights_version"],
          scoring.DEFAULT.version)


def test_a_bigger_gain_never_buys_past_a_veto() -> None:
    """The check this file exists for."""
    here = _situation(CLOSET, entities=[a_thing("object:8", 0.55, 0.30,
                                                uncertainty_m=0.60)])
    keen = scoring.Weights(purpose={"default": 1000.0}, min_gain=0.0)
    got = scoring.consider(here, keen)
    check("the candidate is still considered", len(got["considered"]), 1)
    check("...it would have scored enormously",
          got["considered"][0]["score"]["utility"] > 100.0, True)
    check("...it is still refused", bool(got["considered"][0]["vetoes"]), True)
    check("...and nothing was chosen", got["preferred"], None)


def test_a_flat_battery_stops_everything_whatever_is_on_offer() -> None:
    here = _situation(entities=[_a_thing_worth_looking_at()], battery_v=10.9)
    got = scoring.consider(here, authority=True)
    check("there was something worth doing", bool(got["considered"]), True)
    check("...and it may not be done", got["chose"], None)
    check("...for the reason a person would give",
          any("battery" in one["gate"] for one in got["gate"]), True)
    check("...which names the reading",
          any("10.9" in one["why"] for one in got["gate"]), True)


def test_the_gate_names_each_thing_that_is_wrong() -> None:
    for field, expected in (({"estop": True}, "estop"),
                            ({"position_trusted": False}, "pose"),
                            ({"map_settled": False}, "map")):
        here = _situation(entities=[_a_thing_worth_looking_at()], **field)
        got = scoring.consider(here, authority=True)
        check(f"{expected} shuts the gate",
              any(one["gate"] == expected for one in got["gate"]), True)


def test_with_no_authority_it_still_says_what_it_would_do() -> None:
    """Which is the whole of a shadow decision: a want, and no way to act."""
    here = _situation(entities=[_a_thing_worth_looking_at()])
    got = scoring.consider(here)
    check("it wants something", got["preferred"] is not None, True)
    check("...and chose nothing", got["chose"], None)
    check("...because it may not move anything",
          any("authority" in one["gate"] for one in got["gate"]), True)
    check("...which is said in one sentence",
          got["why_nothing"].startswith("it would improve_geometry"), True)


def test_being_reachable_is_not_a_reason_to_drive() -> None:
    """A finished room with one well-placed thing in it: nothing is worth doing."""
    here = _situation(FINISHED, entities=[a_thing("object:8", 1.6, 0.4,
                                                  uncertainty_m=0.10,
                                                  ranged=4)])
    got = scoring.consider(here, authority=True)
    check("nothing is chosen", got["preferred"], None)
    check("...and the reason is not a failure",
          "nothing" in got["why_nothing"], True)


def test_a_candidate_that_costs_more_than_it_is_worth_loses_to_standing_still() -> None:
    here = _situation(entities=[_a_thing_worth_looking_at()])
    candidate = goals.improve_geometry(here)[0]
    expensive = scoring.Weights(w_travel=50.0, w_time=50.0)
    terms = scoring.score(candidate, here, expensive)
    check("its utility is negative", terms["utility"] < 0.0, True)
    got = scoring.consider(here, expensive, authority=True)
    check("...so nothing is chosen", got["preferred"], None)
    check("...and it says why",
          "more than it is worth" in got["why_nothing"], True)


def test_changing_its_mind_costs_something_and_sticking_does_not() -> None:
    body = a_situation(ROOM, entities=[_a_thing_worth_looking_at()])
    plain = Situation(body)
    candidate = goals.improve_geometry(plain)[0]
    check("with nothing under way there is no switching cost",
          scoring.score(candidate, plain)["switching_cost"], 0.0)

    sticking = Situation({**body, "previous_goal": candidate.id})
    check("...sticking with the last choice is free",
          scoring.score(candidate, sticking)["switching_cost"], 0.0)

    changing = Situation({**body, "previous_goal": "explore_frontier@9,9"})
    check("...changing to something else costs",
          scoring.score(candidate, changing)["switching_cost"],
          scoring.DEFAULT.switching_cost)

    interrupting = Situation(a_situation(
        ROOM, entities=[_a_thing_worth_looking_at()], driving=True))
    check("...and so does interrupting a move already under way",
          scoring.score(candidate, interrupting)["switching_cost"],
          scoring.DEFAULT.switching_cost)


def test_a_thing_that_has_been_put_aside_is_refused_with_its_reason() -> None:
    body = a_situation(ROOM, entities=[_a_thing_worth_looking_at()])
    body["cooled"] = [{"target": "object:8", "since": body["at"] - 10.0,
                       "until": body["at"] + 600.0, "uncertainty_m": 0.6,
                       "why": "three more looks left it no better placed"}]
    got = scoring.consider(Situation(body), authority=True)
    check("it is refused", bool(got["considered"][0]["vetoes"]), True)
    check("...as cooling off", got["considered"][0]["vetoes"][0]["veto"],
          "cooling off")
    check("...with the reason it was put aside",
          "no better placed" in got["considered"][0]["vetoes"][0]["why"], True)
    check("...and a cooling that has expired refuses nothing",
          bool(scoring.vetoes(
              goals.improve_geometry(Situation(body))[0],
              Situation({**body, "cooled": [
                  {**body["cooled"][0], "until": body["at"] - 1.0}]}))),
          False)


def test_a_safe_area_refuses_what_is_outside_it() -> None:
    here = _situation(entities=[_a_thing_worth_looking_at()])
    candidate = goals.improve_geometry(here)[0]
    check("inside a circle round the rover, nothing is refused",
          scoring.vetoes(candidate, here, scoring.Weights(
              geofence={"x_m": here.where[0], "y_m": here.where[1],
                        "radius_m": 5.0})), [])
    outside = scoring.vetoes(candidate, here, scoring.Weights(
        geofence={"x_m": 20.0, "y_m": 20.0, "radius_m": 1.0}))
    check("...outside it, the goal is", [one["veto"] for one in outside],
          ["outside the safe area"])
    boxed = scoring.vetoes(candidate, here, scoring.Weights(
        geofence={"min_x_m": 0.0, "max_x_m": 0.2}))
    check("...and a box works the same way",
          [one["veto"] for one in boxed], ["outside the safe area"])


def test_two_equal_candidates_break_the_same_way_twice() -> None:
    here = _situation(entities=[_a_thing_worth_looking_at(),
                                a_thing("object:9", 2.2, 1.0,
                                        uncertainty_m=0.60, major_deg=90.0)])
    first = [one["candidate"]["id"] for one in
             scoring.consider(here)["considered"]]
    again = [one["candidate"]["id"] for one in
             scoring.consider(Situation(here.as_dict()))["considered"]]
    check("the ranking is the same", first, again)
    check("...and there was something to rank", len(first) > 1, True)


def test_the_weights_can_be_changed_without_changing_the_code() -> None:
    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, scoring.CONFIG_NAME)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump({"version": "a-test", "w_travel": 9.0,
                       "purpose": {"explore_frontier": 3.0}}, handle)
        weights = scoring.Weights.load(directory)
        check("the version is the file's", weights.version, "a-test")
        check("...the weight is the file's", weights.w_travel, 9.0)
        check("...what is not in the file keeps its built-in value",
              weights.w_time, scoring.DEFAULTS["w_time"])
        check("...the purpose is the owner's",
              weights.relevance("explore_frontier"), 3.0)
        check("...and it says where it came from", weights.source, path)


def test_the_decision_carries_the_whole_configuration_not_a_name_for_it() -> None:
    got = scoring.consider(_situation(entities=[_a_thing_worth_looking_at()]))
    check("the weights are in the decision",
          got["weights"]["w_travel"], scoring.DEFAULT.w_travel)
    check("...all of them", sorted(got["weights"]) == sorted(
        [*scoring.DEFAULTS.keys(), "source"]), True)


def test_a_purpose_makes_one_kind_of_goal_matter_more() -> None:
    """The owner's policy, which is the one term nothing here may invent."""
    here = _situation(HOUSE, entities=[_a_thing_worth_looking_at()])
    plain = scoring.consider(here)
    exploring = scoring.consider(here, scoring.Weights(
        purpose={"default": 1.0, "explore_frontier": 3.0}))
    check("with no purpose declared, the thing wins",
          plain["preferred"]["candidate"]["type"], "improve_geometry")
    check("...and told the map matters three times as much, the frontier does",
          exploring["preferred"]["candidate"]["type"], "explore_frontier")


TESTS = (
    test_every_term_of_the_score_is_written_down,
    test_a_bigger_gain_never_buys_past_a_veto,
    test_a_flat_battery_stops_everything_whatever_is_on_offer,
    test_the_gate_names_each_thing_that_is_wrong,
    test_with_no_authority_it_still_says_what_it_would_do,
    test_being_reachable_is_not_a_reason_to_drive,
    test_a_candidate_that_costs_more_than_it_is_worth_loses_to_standing_still,
    test_changing_its_mind_costs_something_and_sticking_does_not,
    test_a_thing_that_has_been_put_aside_is_refused_with_its_reason,
    test_a_safe_area_refuses_what_is_outside_it,
    test_two_equal_candidates_break_the_same_way_twice,
    test_the_weights_can_be_changed_without_changing_the_code,
    test_the_decision_carries_the_whole_configuration_not_a_name_for_it,
    test_a_purpose_makes_one_kind_of_goal_matter_more,
)
