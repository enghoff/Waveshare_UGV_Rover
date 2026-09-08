"""The curated set, run as part of the ordinary checks.

Two different things are checked here and they are worth separating. The first
is that the library itself is sound -- every scenario names a map that exists,
every expectation is one the harness understands, and the whole set is big
enough to mean something. The second is the acceptance number the milestone
asks for: the expected ordering right in at least ninety-five per cent of the
cases.

**A harness that cannot fail is worth nothing**, so the last check here breaks
the scorer on purpose and makes sure the set notices.
"""
from __future__ import annotations

import scenarios
import scoring
from test_harness import check


def test_the_library_is_the_size_the_milestone_asks_for() -> None:
    got = scenarios.run()
    check("there are at least forty curated cases",
          got["cases"] >= scenarios.REQUIRED_CASES, True)
    check("...and every one of them says what it is about",
          [one["name"] for one in got["results"] if not one["why"]], [])


def test_the_scorer_agrees_with_the_curated_expectations() -> None:
    got = scenarios.run()
    wrong = [one["name"] for one in got["results"] if not one["passed"]]
    check(f"{got['passed']} of {got['cases']} scenarios come out as expected",
          got["rate"] >= scenarios.REQUIRED_RATE, True)
    check("...and these are the ones that did not", wrong, [])


def test_every_scenario_is_reproducible() -> None:
    """Run twice, and the same cases pass. A scenario that depends on the
    order a dictionary was built in would show up here and nowhere else."""
    first = scenarios.run()
    second = scenarios.run()
    check("the same scenarios pass both times",
          [one["passed"] for one in first["results"]],
          [one["passed"] for one in second["results"]])
    check("...with the same choices",
          [_chose(one) for one in first["results"]],
          [_chose(one) for one in second["results"]])


def test_the_set_notices_a_scorer_that_has_gone_wrong() -> None:
    """A check that cannot fail is not a check.

    Every cost is set to nothing: driving is free, time is worthless, changing
    its mind costs nothing and no gain is too small to be worth a drive. That is
    a rover that will cross the house for a centimetre, and six of the curated
    cases are about exactly that, so the set drops to 88% and the acceptance
    number refuses it.

    **Two of them would not be enough**, and that is worth knowing about the
    threshold rather than about this test: forty-eight cases at ninety-five per
    cent tolerates two disagreements, so a behaviour with only one scenario
    behind it is a behaviour the set cannot really defend. Where a rule matters,
    it is written more than once.
    """
    was = {name: scoring.DEFAULTS[name] for name in
           ("w_travel", "w_time", "switching_cost", "min_gain")}
    try:
        scoring.DEFAULTS.update({"w_travel": 0.0, "w_time": 0.0,
                                 "switching_cost": 0.0, "min_gain": 0.0})
        got = scenarios.run()
    finally:
        scoring.DEFAULTS.update(was)
    check("a scorer that minds nothing fails the set",
          got["rate"] < scenarios.REQUIRED_RATE, True)
    check("...and the ordinary one passes it again",
          scenarios.run()["rate"] >= scenarios.REQUIRED_RATE, True)


def _chose(result: dict) -> str:
    preferred = result["decision"]["preferred"]
    return "" if preferred is None else preferred["candidate"]["id"]


TESTS = (
    test_the_library_is_the_size_the_milestone_asks_for,
    test_the_scorer_agrees_with_the_curated_expectations,
    test_every_scenario_is_reproducible,
    test_the_set_notices_a_scorer_that_has_gone_wrong,
)
