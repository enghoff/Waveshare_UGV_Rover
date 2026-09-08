"""Scoring a candidate remedy against the things a person gave a verdict to.

The failure this guards against is silent and expensive: a remedy is judged
against the labelled drive, the verdicts are attached to the wrong things, and
the number that comes out is meaningless in a way nothing prints. Every entity
identifier in the room changes when the resolver changes, which is the only
occasion this program is ever run, so the join has to survive a rename.
"""
from __future__ import annotations

import bench_identity as bench
from test_harness import check


def _labels(**things) -> dict:
    return {"things": {name: dict(row, what=row.get("what", "a thing"))
                       for name, row in things.items()}}


def _verdict(verdict: str, looks: list[int], **extra) -> dict:
    row = {"verdict": verdict, "looks": looks, "uncertainty_m": 0.4,
           "viewpoints": 3, "rays_agreeing": 3, "from_range_m": None}
    row.update(extra)
    return row


# --- attaching a verdict to a thing that has been renamed --------------------

def test_a_verdict_follows_its_looks_through_a_rename() -> None:
    """The whole point of the join. `object:8` in the review and `object:41` in
    a later rebuild are the same thing if they hold the same looks, and nothing
    but the looks says so."""
    labels = _labels(**{"object:8": _verdict("mixed", [11, 12, 13])})
    rebuilt = {"object:41": {"looks": [11, 12, 13], "uncertainty_m": 0.9}}
    matched, lost = bench.join(labels, rebuilt)
    check("the verdict lands on the rebuilt thing", list(matched), ["object:41"])
    check("...carrying what it was called when it was reviewed",
          matched["object:41"]["labelled_as"], "object:8")
    check("...and the verdict itself", matched["object:41"]["verdict"], "mixed")
    check("...beside the rebuild's own numbers, not the review's",
          matched["object:41"]["uncertainty_m"], 0.9)
    check("nothing was lost", lost, [])


def test_a_thing_the_remedy_split_is_judged_on_its_larger_half() -> None:
    """A remedy that refuses an attachment splits a thing in two, which is the
    direction acceptance wants. The verdict goes to the half that kept most of
    the looks, because that is the thing the reviewer was looking at."""
    labels = _labels(**{"object:8": _verdict("mixed", [1, 2, 3, 4, 5])})
    rebuilt = {"object:2": {"looks": [1, 2, 3]}, "object:3": {"looks": [4, 5]}}
    matched, lost = bench.join(labels, rebuilt)
    check("one verdict, on the larger half", list(matched), ["object:2"])
    check("...and it says how much of the thing that half is",
          (matched["object:2"]["looks_kept"],
           matched["object:2"]["looks_labelled"]), (3, 5))
    check("nothing was lost", lost, [])


def test_two_verdicts_landing_on_one_thing_both_survive() -> None:
    """A remedy that merges two reviewed things into one is a fault it caused,
    and the count must show two verdicts rather than one overwriting the other."""
    labels = _labels(**{"object:8": _verdict("object", [1, 2]),
                        "object:9": _verdict("object", [3, 4])})
    matched, lost = bench.join(labels, {"object:1": {"looks": [1, 2, 3, 4]}})
    check("both verdicts are kept", len(matched), 2)
    check("...naming the two things that were reviewed",
          sorted(row["labelled_as"] for row in matched.values()),
          ["object:8", "object:9"])
    check("nothing was lost", lost, [])


def test_a_thing_the_remedy_stopped_building_is_reported_lost() -> None:
    """A verdict whose looks no longer belong to anything is not a pass and not
    a fail; it is a verdict this rebuild cannot be scored on, and saying so is
    the difference between 76 things and however many the rebuild kept."""
    labels = _labels(**{"object:8": _verdict("mixed", [1, 2]),
                        "object:9": _verdict("object", [3])})
    matched, lost = bench.join(labels, {"object:1": {"looks": [3]}})
    check("only the thing that still exists is scored", len(matched), 1)
    check("...and the other is named", lost, ["object:8"])


# --- what a signal is worth --------------------------------------------------

def test_a_gate_that_admits_nothing_is_not_a_gate() -> None:
    """The trap this whole program exists to avoid. A signal can always exclude
    every merge by excluding everything, so the tightest safe threshold is
    reported with the number of real objects it leaves eligible."""
    rows = [{"verdict": "mixed", "uncertainty_m": 0.10, "looks": [1]},
            {"verdict": "object", "uncertainty_m": 0.20, "looks": [2]},
            {"verdict": "object", "uncertainty_m": 0.30, "looks": [3]}]
    got = bench.gate(rows, bench.SIGNALS["uncertainty_m"])
    check("the threshold is the lowest merge", got["threshold"], 0.10)
    check("and it keeps nothing at all", got["clean_kept"], 0)
    check("out of two real objects", got["clean_total"], 2)


def test_a_signal_that_separates_them_scores_one() -> None:
    """The scale, at both ends: a signal that puts every merge above every real
    object scores 1.0, and reversing the labels scores 0.0 rather than 1.0."""
    rows = [{"verdict": "mixed", "uncertainty_m": 0.9, "looks": [1]},
            {"verdict": "mixed", "uncertainty_m": 0.8, "looks": [2]},
            {"verdict": "object", "uncertainty_m": 0.2, "looks": [3]},
            {"verdict": "object", "uncertainty_m": 0.1, "looks": [4]}]
    signal = bench.SIGNALS["uncertainty_m"]
    check("a perfect separation", bench.auc(rows, signal), 1.0)
    check("...and the same signal read backwards",
          bench.auc(rows, signal, positive="object", negative="mixed"), 0.0)
    flat = [dict(row, uncertainty_m=0.5) for row in rows]
    check("a signal that says nothing sits at chance",
          bench.auc(flat, signal), 0.5)


def test_the_labelled_drive_is_still_the_one_that_was_reviewed() -> None:
    """The checked-in set is the instrument, so a change to it should be a
    deliberate act that fails here first."""
    labels = bench.load()
    counts = {}
    for row in labels["things"].values():
        counts[row["verdict"]] = counts.get(row["verdict"], 0) + 1
    check("76 things carry a verdict", len(labels["things"]), 76)
    check("...of which the merges are what M0 fails on", counts.get("mixed"), 17)
    check("...beside the real objects", counts.get("object"), 52)
    check("...and the bare surfaces", counts.get("surface"), 7)
    check("every verdict can be joined by its looks",
          all(row["looks"] for row in labels["things"].values()), True)


TESTS = (
    test_a_verdict_follows_its_looks_through_a_rename,
    test_a_thing_the_remedy_split_is_judged_on_its_larger_half,
    test_two_verdicts_landing_on_one_thing_both_survive,
    test_a_thing_the_remedy_stopped_building_is_reported_lost,
    test_a_gate_that_admits_nothing_is_not_a_gate,
    test_a_signal_that_separates_them_scores_one,
    test_the_labelled_drive_is_still_the_one_that_was_reviewed,
)
