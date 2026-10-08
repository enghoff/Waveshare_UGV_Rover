"""Putting a thing aside, and the two ways that lapses.

What is checked here is mostly that cooling cannot become a way of forgetting
something permanently. A rover that quietly stops considering half the things it
knows about, and never says so, is worse than one that keeps trying: the trying
is visible.
"""
from __future__ import annotations

import cooling
from test_fakes import a_situation, a_thing
from test_harness import check
from test_goals import ROOM

NOW = 1757320800.0


def _reading(looks: int, uncertainty_m: float | None = 0.60, *,
             entity_id: str = "object:8") -> dict:
    thing = a_thing(entity_id, 1.6, 1.0,
                    uncertainty_m=0.60 if uncertainty_m is None else uncertainty_m,
                    looks=looks)
    if uncertainty_m is None:
        thing["placement"] = None
        thing["placement_uncertainty_m"] = None
    return a_situation(ROOM, entities=[thing], at=NOW)


def test_looking_again_and_learning_nothing_puts_a_thing_aside() -> None:
    before = _reading(6)
    after = _reading(12)
    cooled = cooling.update(before, after, [], now=NOW)
    check("it is put aside", [one["target"] for one in cooled], ["object:8"])
    check("...for a stated while", cooled[0]["until"] - NOW, cooling.COOLDOWN_S)
    check("...with the reason in plain words",
          "no better than before" in cooled[0]["why"], True)


def test_one_unhelpful_look_is_not_enough() -> None:
    """Most looks cross with nothing. Cooling on one would cool everything."""
    check("a single look changes nothing",
          cooling.update(_reading(6), _reading(7), [], now=NOW), [])


def test_a_thing_that_actually_improved_is_left_alone() -> None:
    check("looks that helped do not cool it",
          cooling.update(_reading(6), _reading(20, 0.30), [], now=NOW), [])


def test_cooling_lapses_when_the_thing_finally_comes_out_better() -> None:
    cooled = cooling.update(_reading(6), _reading(12), [], now=NOW)
    check("it was cooling", len(cooled), 1)
    later = cooling.update(_reading(12), _reading(18, 0.20), cooled,
                           now=NOW + 60.0)
    check("...and a better placement ends it, without waiting", later, [])


def test_cooling_lapses_when_its_time_is_up() -> None:
    cooled = cooling.update(_reading(6), _reading(12), [], now=NOW)
    check("nothing is cooling once the time has passed",
          cooling.update(_reading(12), _reading(13), cooled,
                         now=NOW + cooling.COOLDOWN_S + 1.0), [])


def test_a_thing_that_has_gone_takes_its_cooling_with_it() -> None:
    """A merge or a clear hands the identifier to something else, and the new
    thing must not inherit a refusal earned by the old one."""
    cooled = cooling.update(_reading(6), _reading(12), [], now=NOW)
    empty = a_situation(ROOM, entities=[], at=NOW)
    check("the entry goes with the thing",
          cooling.update(_reading(12), empty, cooled, now=NOW + 60.0), [])


def test_a_thing_that_has_just_been_placed_counts_as_having_improved() -> None:
    check("gaining a position at all is learning something",
          cooling.update(_reading(6, None), _reading(12), [], now=NOW), [])


def test_it_reads_only_what_is_written_down() -> None:
    """No memory of its own: two calls with the same arguments agree, so a
    recorder that was stopped and restarted resumes where the record says."""
    before, after = _reading(6), _reading(12)
    first = cooling.update(before, after, [], now=NOW)
    second = cooling.update(before, after, [], now=NOW)
    check("the same inputs give the same answer", first, second)
    check("...and the first deliberation of all cools nothing",
          cooling.update(None, after, [], now=NOW), [])


def test_a_goal_that_left_its_thing_no_better_puts_it_aside_at_once() -> None:
    """The case counting looks cannot see: a look from where the rover already
    looked is the same picture, which is not recorded, so the count never moves.
    On 2026-10-03 one goal was chosen twenty-two times in a row that way."""
    cooled = cooling.after_attempt([], "object:8", _reading(6), _reading(6),
                                   now=NOW)
    check("one fruitless goal puts it aside", [one["target"] for one in cooled],
          ["object:8"])
    check("...for the stated while", cooled[0]["until"] - NOW, cooling.COOLDOWN_S)
    check("...saying why", "no better than before" in cooled[0]["why"], True)
    check("...and the next deliberation keeps it aside",
          [one["target"] for one in cooling.update(
              _reading(6), _reading(6), cooled, now=NOW + 30.0)], ["object:8"])
    check("...until it does come out better",
          cooling.update(_reading(6), _reading(7, 0.30), cooled, now=NOW + 60.0),
          [])


def test_a_goal_that_helped_or_lost_its_thing_puts_nothing_aside() -> None:
    check("a goal that improved the placement cools nothing",
          cooling.after_attempt([], "object:8", _reading(6), _reading(7, 0.30),
                                now=NOW), [])
    empty = a_situation(ROOM, entities=[], at=NOW)
    check("...nor one whose thing has gone",
          cooling.after_attempt([], "object:8", _reading(6), empty, now=NOW), [])
    other = [{"target": "object:3", "until": NOW + 100.0, "uncertainty_m": 0.5}]
    check("...and what was already aside stays aside",
          cooling.after_attempt(other, "object:8", _reading(6), _reading(7, 0.30),
                                now=NOW), other)


def test_a_look_at_one_record_puts_its_same_object_records_aside() -> None:
    """The owner's question on 2026-10-08: with a look's region filed to the
    record it was aimed at, could the rover go back to the same object by way
    of another record of it? Not straight away: the records the aimed region
    also fitted are put aside with it, whether or not the look helped."""
    def reading(uncertainty_m):
        here = a_thing("object:8", 1.6, 1.0, uncertainty_m=uncertainty_m, looks=7)
        here["same_object_suspects"] = ["object:9"]
        twin = a_thing("object:9", 1.7, 1.1, uncertainty_m=0.6, looks=4)
        twin["same_object_suspects"] = ["object:8"]
        return a_situation(ROOM, entities=[here, twin], at=NOW)

    cooled = cooling.after_attempt([], "object:8", reading(0.6), reading(0.3), now=NOW)
    check("a look that helped its thing puts the thing's other record aside",
          [one["target"] for one in cooled], ["object:9"])
    check("...saying why", "probably the same object as object:8" in cooled[0]["why"], True)
    cooled = cooling.after_attempt([], "object:8", reading(0.6), reading(0.6), now=NOW)
    check("...and one that did not help puts both aside",
          sorted(one["target"] for one in cooled), ["object:8", "object:9"])
    check("a thing with no other record puts nothing more aside",
          cooling.after_attempt([], "object:8", _reading(6), _reading(7, 0.30), now=NOW), [])


def test_a_placement_is_judged_by_what_it_claims() -> None:
    """2026-10-08: goals measured gain on the tolerance the resolver matches
    with, which never falls below its best crossing, rather than on what the
    placement claims."""
    import situation
    thing = a_thing("object:8", 1.6, 1.0, uncertainty_m=0.5, looks=7)
    thing["placement"] = dict(thing["placement"], stated_uncertainty_m=0.12)
    check("the claim is read where there is one", situation.claimed_m(thing), 0.12)
    del thing["placement"]["stated_uncertainty_m"]
    check("...and the matching figure where there is not",
          situation.claimed_m(thing), 0.5)


def test_a_place_navigation_could_not_reach_is_set_aside_for_a_while() -> None:
    """Found on 2026-10-03: one frontier the rover could not fit through to was
    driven at four times, forty seconds of recoveries each, until three
    failures in a row ended the run."""
    places = cooling.after_failed_drive([], {"x_m": -18.0, "y_m": -19.36},
                                        "blocked: no route it fits through",
                                        now=NOW)
    check("the place is written down", len(places), 1)
    check("...a goal within half a metre of it is refused",
          bool(cooling.unreachable_near(places, -17.8, -19.41, now=NOW + 60.0)),
          True)
    check("...one further off is not",
          cooling.unreachable_near(places, -17.0, -19.36, now=NOW + 60.0), None)
    check("...and it lapses",
          cooling.unreachable_near(places, -18.0, -19.36,
                                   now=NOW + cooling.UNREACHABLE_S + 1.0), None)
    check("a goal with no place to it writes nothing down",
          cooling.after_failed_drive(places, {}, "why", now=NOW), places)


TESTS = (
    test_a_look_at_one_record_puts_its_same_object_records_aside,
    test_a_placement_is_judged_by_what_it_claims,
    test_a_place_navigation_could_not_reach_is_set_aside_for_a_while,
    test_a_goal_that_left_its_thing_no_better_puts_it_aside_at_once,
    test_a_goal_that_helped_or_lost_its_thing_puts_nothing_aside,
    test_looking_again_and_learning_nothing_puts_a_thing_aside,
    test_one_unhelpful_look_is_not_enough,
    test_a_thing_that_actually_improved_is_left_alone,
    test_cooling_lapses_when_the_thing_finally_comes_out_better,
    test_cooling_lapses_when_its_time_is_up,
    test_a_thing_that_has_gone_takes_its_cooling_with_it,
    test_a_thing_that_has_just_been_placed_counts_as_having_improved,
    test_it_reads_only_what_is_written_down,
)
