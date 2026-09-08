"""Writing a decision down so that it can be argued with a month later.

The test that carries the milestone is `test_the_ranking_can_be_recomputed_from
_the_record_alone`: it takes the snapshot the episode names, runs the scorer over
it again, and checks it produces the same ordering with the same numbers. That is
what "deterministic under replay" has to mean -- not that the code looks pure,
but that the record contains enough to redo the work and get the same answer.

Everything else here is about the reading a person gets: that the losing
candidates are kept, that a refusal which scored higher than the winner is
reported rather than buried, and that an episode nothing acted on closes
`abandoned` rather than as a failure.
"""
from __future__ import annotations

import tempfile

import decide
import replay
import scoring
import situation as situation_mod
import summary
from situation import Situation
from store import EpisodeStore
from test_fakes import a_situation, a_thing
from test_harness import check
from test_goals import CLOSET, FINISHED, HOUSE, ROOM


def _here(rows=ROOM, **kwargs) -> Situation:
    return Situation(a_situation(rows, **kwargs))


def _a_thing_worth_looking_at(**kwargs):
    fields = {"uncertainty_m": 0.60, "major_deg": 90.0, **kwargs}
    return a_thing("object:8", 1.6, 1.0, **fields)


def test_a_deliberation_is_recorded_whole() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = EpisodeStore(directory)
        here = _here(entities=[_a_thing_worth_looking_at()])
        got = decide.deliberate(store, here)
        rebuilt = replay.reconstruct(store, got["episode"])

        check("the losing candidates are kept too",
              len(rebuilt["candidates"]), len(got["decision"]["considered"]))
        check("...each with what it would have scored",
              all("score" in one.get("params", {})
                  for one in rebuilt["candidates"]), True)
        check("the decision names the snapshot it was made from",
              rebuilt["decision"]["inputs_digest"], got["inputs"])
        check("...and the snapshot is the world it saw, not today's",
              rebuilt["decision"]["inputs"]["at"], here.at)
        check("...and it chose nothing, because it may not act",
              rebuilt["decision"]["chose"], "nothing")
        check("...which is not a failure",
              rebuilt["outcome"]["outcome"], "abandoned")
        check("...and the summary says what it would have done instead",
              "it would have chosen this" in rebuilt["decision"]["why"], True)
        store.close()


def test_the_ranking_can_be_recomputed_from_the_record_alone() -> None:
    """The milestone's first criterion, checked rather than asserted."""
    with tempfile.TemporaryDirectory() as directory:
        store = EpisodeStore(directory)
        here = _here(HOUSE, entities=[_a_thing_worth_looking_at(),
                                      a_thing("object:9", 2.6, 1.0,
                                              uncertainty_m=0.4,
                                              major_deg=90.0)])
        got = decide.deliberate(store, here)
        recorded = [(one["candidate"]["id"], one["score"]["utility"])
                    for one in got["decision"]["considered"]]

        # Everything the scorer is allowed to look at, read back out of the
        # database and given to it again.
        body = store.snapshot_body(got["inputs"])
        weights = scoring.Weights.from_dict(got["decision"]["weights"])
        again = [(one["candidate"]["id"], one["score"]["utility"])
                 for one in scoring.consider(Situation(body), weights)[
                     "considered"]]

        check("the same candidates come back, in the same order",
              [one[0] for one in again], [one[0] for one in recorded])
        check("...with the same scores",
              [one[1] for one in again], [one[1] for one in recorded])
        check("...and there was something to rank", len(again) > 2, True)
        store.close()


def test_a_refusal_that_would_have_won_is_reported() -> None:
    """Criterion 7's hard half: why the better-looking option was not taken."""
    with tempfile.TemporaryDirectory() as directory:
        store = EpisodeStore(directory)
        def room(looks: int) -> Situation:
            return _here(entities=[
                _a_thing_worth_looking_at(),
                a_thing("object:9", 2.0, 1.0, uncertainty_m=0.9,
                        major_deg=90.0, looks=looks)])

        # The better prospect takes eight more looks and comes out no better
        # placed, so by the second deliberation it is cooling -- and the rover
        # has to explain why it is doing something else instead of simply
        # taking the next one down.
        decide.deliberate(store, room(6))
        got = decide.deliberate(store, room(14))
        refused = got["decision"]["refused_above_it"]
        check("the better option is named", bool(refused), True)
        check("...with its score", refused[0]["utility"]
              > got["decision"]["preferred"]["score"]["utility"], True)
        check("...and the reason", "cooling off" in refused[0]["why"], True)
        check("the printed record carries it",
              "was refused" in decide.render(got), True)
        store.close()


def test_the_record_says_what_it_wanted_what_it_would_cost_and_why_not() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = EpisodeStore(directory)
        got = decide.deliberate(store, _here(
            entities=[_a_thing_worth_looking_at()]))
        said = decide.render(got)
        check("it says what it would do", said.startswith("would "), True)
        check("...what it expects to get", "expects" in said, True)
        check("...what it would cost", " m and about " in said, True)
        check("...how the score was arrived at", "score " in said, True)
        check("...and that it cannot act", "cannot act" in said, True)
        store.close()


def test_a_rover_with_nothing_to_do_says_so_rather_than_inventing_an_errand() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = EpisodeStore(directory)
        got = decide.deliberate(store, _here(
            FINISHED, entities=[a_thing("object:8", 1.6, 0.4,
                                        uncertainty_m=0.10, ranged=4)]))
        check("nothing was preferred", got["decision"]["preferred"], None)
        check("...and the episode says why",
              "would do nothing" in decide.render(got), True)
        check("...and it is still an episode somebody can read",
              "considered" in summary.of(store, got["episode"])
              or "decided" in summary.of(store, got["episode"]), True)
        store.close()


def test_one_deliberation_tells_the_next_what_it_wanted() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = EpisodeStore(directory)
        here = _here(entities=[_a_thing_worth_looking_at()])
        first = decide.deliberate(store, here)
        wanted = first["decision"]["preferred"]["candidate"]["id"]
        check("what it wanted is left for the next one",
              wanted in store.marked(decide.GOAL_MARK), True)

        second = decide.prepare(store, _here(
            entities=[_a_thing_worth_looking_at()]))
        check("...and the next one knows it", second.previous_goal["id"],
              wanted)
        check("...by what it was about, not only by its name",
              second.previous_goal["target"], "object:8")
        store.close()


def test_a_thing_that_took_looks_and_got_no_better_is_put_aside_by_the_next_one() -> None:
    """The cooling arrives through the record rather than through memory."""
    with tempfile.TemporaryDirectory() as directory:
        store = EpisodeStore(directory)
        decide.deliberate(store, _here(
            entities=[_a_thing_worth_looking_at(looks=6)]))
        later = decide.prepare(store, _here(
            entities=[_a_thing_worth_looking_at(looks=14)]))
        check("the thing is cooling", [one["target"] for one in later.cooled],
              ["object:8"])
        got = decide.deliberate(store, later)
        check("...so it is refused rather than chosen",
              got["decision"]["preferred"], None)
        check("...and the cooling is part of what the decision was made from",
              bool(store.snapshot_body(got["inputs"])["cooled"]), True)
        store.close()


def test_reading_the_rover_and_deciding_are_separate_things() -> None:
    """One read, then arithmetic: the decision cannot go back for more.

    It matters because a scorer that read the rover between candidates would
    rank two of them against different worlds, and no snapshot could ever
    reproduce that.
    """
    from test_fakes import FakeRover

    rover = FakeRover(room=ROOM, entities=[_a_thing_worth_looking_at()])
    here = situation_mod.Situation.read(rover, at=1757320800.0)
    asked = len(rover.asked)
    scoring.consider(here)
    check("nothing was asked of the rover while it decided",
          len(rover.asked), asked)


def test_a_family_of_refusals_is_reported_once_with_its_members() -> None:
    """On the real map, things placed where the rover cannot stand come in
    families, and six copies of the same two-sentence refusal bury the one line
    a reader wants."""
    with tempfile.TemporaryDirectory() as directory:
        store = EpisodeStore(directory)
        shut_in = [a_thing(f"object:{n}", 0.45 + 0.05 * n, 0.30,
                           uncertainty_m=1.2, major_deg=90.0)
                   for n in (1, 2, 3)]
        reachable = a_thing("object:9", 1.35, 0.55, uncertainty_m=0.5,
                            major_deg=90.0)
        got = decide.deliberate(store, _here(CLOSET,
                                             entities=[*shut_in, reachable]))
        said = decide.render(got)
        check("it still chooses the one it can reach",
              got["decision"]["preferred"]["candidate"]["target"], "object:9")
        check("...and the three it cannot are reported as one family",
              "3 scored higher and were refused for the same reason" in said,
              True)
        check("...with the things named",
              "they are object:1, object:2, object:3" in said, True)
        check("...and the reason given once",
              said.count("there is no route to it"), 1)
        store.close()


TESTS = (
    test_a_deliberation_is_recorded_whole,
    test_the_ranking_can_be_recomputed_from_the_record_alone,
    test_a_refusal_that_would_have_won_is_reported,
    test_the_record_says_what_it_wanted_what_it_would_cost_and_why_not,
    test_a_rover_with_nothing_to_do_says_so_rather_than_inventing_an_errand,
    test_one_deliberation_tells_the_next_what_it_wanted,
    test_a_thing_that_took_looks_and_got_no_better_is_put_aside_by_the_next_one,
    test_reading_the_rover_and_deciding_are_separate_things,
    test_a_family_of_refusals_is_reported_once_with_its_members,
)
