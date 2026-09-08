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


TESTS = (
    test_looking_again_and_learning_nothing_puts_a_thing_aside,
    test_one_unhelpful_look_is_not_enough,
    test_a_thing_that_actually_improved_is_left_alone,
    test_cooling_lapses_when_the_thing_finally_comes_out_better,
    test_cooling_lapses_when_its_time_is_up,
    test_a_thing_that_has_gone_takes_its_cooling_with_it,
    test_a_thing_that_has_just_been_placed_counts_as_having_improved,
    test_it_reads_only_what_is_written_down,
)
