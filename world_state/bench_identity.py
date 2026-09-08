#!/usr/bin/env python3
"""Score a signal against the 76 things a person gave a verdict to.

    python world_state/bench_identity.py
    python world_state/bench_identity.py --things captures/run/things.json

**This exists because identity has been argued about on three faults spotted by
eye.** Two remedies were judged against them and one of the two fixed a merge
and caused another, which is indistinguishable from noise at that sample size.
The drive of 2026-09-08 was then labelled in full -- every thing holding four
looks or more, one written verdict each, in
[labels/m0-2026-09-08.json](labels/m0-2026-09-08.json) -- and this is the
program that scores against it, so that the next candidate is measured on 76
things rather than three.

A signal here is any number the component already computes about a thing. What
the acceptance criterion needs from one is not a correlation but a **gate**:
[R-WS-13](../docs/requirements/world-state.md#r-ws-13) asks for zero known
incorrect merges among the associations eligible to move the rover, so a signal
earns its place only if some threshold admits no merge at all while leaving a
useful part of the room eligible. Both halves are printed, because a gate that
admits nothing passes the first half trivially.

## What it measured, on the drive of 2026-09-08

The lead the labelling turned up was placement uncertainty: a thing holding two
objects is placed between them, so its rays agree less and it says so in a
number the resolver already reports. It is real and it is far too weak to gate.

| | merges | real objects |
|---|---:|---:|
| median placement uncertainty | 0.60 m | 0.36 m |

Ranking by it separates the two at AUC 0.68, whose 95% bootstrap interval
(0.52-0.83) only just clears chance. **The tightest gate that admits no merge
sits at 0.135 m and keeps one of the fifty-two real objects**, because the
merges reach down into the well-placed population: a blue bin with floor and
glare in it is placed to 0.135 m, and a sofa with a seated person among its
looks to 0.339 m. As a gate it is finished.

As a *ranking* hint it works, and the rover is already using it: the twelve
worst-placed things -- exactly what `autonomy/goals.py` offers as candidates to
go and look at again -- are 42% merges against a base rate of 22%, and the worst
five are 60%. Re-looking at a thing the geometry doubts is therefore twice as
likely to be re-looking at a mistake as picking one at random, which is a reason
to keep the ordering rather than a reason to trust the number.
"""
from __future__ import annotations

import argparse
import json
import os
import random
import statistics
import sys
from typing import Any, Callable

HERE = os.path.dirname(os.path.abspath(__file__))
LABELS = os.path.join(HERE, "labels", "m0-2026-09-08.json")

#: What the twelve worst-placed things are: the number of entities
#: `autonomy/goals.py` will consider re-looking at in one deliberation. The
#: enrichment line is about that list and not about an arbitrary top slice.
CONSIDERED = 12

#: Verdicts, as the labelled set writes them.
MERGE = "mixed"
CLEAN = "object"
SURFACE = "surface"


# --- the labelled set ---------------------------------------------------------

def load(path: str = LABELS) -> dict[str, Any]:
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def join(labels: dict[str, Any], things: dict[str, Any]
         ) -> tuple[dict[str, dict], list[str]]:
    """Attach each labelled verdict to a thing in a later rebuild.

    **By which looks a thing holds, never by its name.** An entity identifier is
    minted in the order things are discovered, so any change to the resolver --
    which is the only reason to run this program twice -- renames every thing in
    the room. The looks are observation ids in the recording and they do not
    move.

    A labelled thing matches the rebuilt thing holding most of its looks. Where
    a remedy has split one labelled thing in two, both halves point at the same
    verdict and the larger half wins it; the leftovers are returned by name so
    the caller can say how many verdicts the rebuild lost track of.
    """
    owner: dict[int, str] = {}
    for name, row in things.items():
        for look in row.get("looks", ()):
            owner[int(look)] = name

    matched: dict[str, dict] = {}
    lost: list[str] = []
    for name, row in labels["things"].items():
        votes: dict[str, int] = {}
        for look in row["looks"]:
            held = owner.get(int(look))
            if held is not None:
                votes[held] = votes.get(held, 0) + 1
        if not votes:
            lost.append(name)
            continue
        best = max(votes, key=lambda held: (votes[held], held))
        merged = dict(things[best])
        merged.update({"verdict": row["verdict"], "what": row["what"],
                       "labelled_as": name,
                       "looks_kept": votes[best],
                       "looks_labelled": len(row["looks"])})
        # Two labelled things landing on one rebuilt thing is a merge the remedy
        # introduced, and it must not silently overwrite the first verdict.
        while best in matched:
            best += "+"
        matched[best] = merged
    return matched, lost


def rows_of(labels: dict[str, Any]) -> dict[str, dict]:
    """The labelled set scored as it stands, with no later rebuild."""
    return {name: dict(row, labelled_as=name)
            for name, row in labels["things"].items()}


# --- scoring ------------------------------------------------------------------

def auc(rows: list[dict], signal: Callable[[dict], float],
        positive: str = MERGE, negative: str = CLEAN) -> float | None:
    """How often a merge outranks a real object, ties counted as half.

    Chance is 0.5 and a perfect separation is 1.0. This is the whole of what a
    ranking signal is worth and it deliberately says nothing about a threshold,
    which is the next function's business.
    """
    up = [signal(row) for row in rows if row["verdict"] == positive]
    down = [signal(row) for row in rows if row["verdict"] == negative]
    if not up or not down:
        return None
    wins = sum((a > b) + 0.5 * (a == b) for a in up for b in down)
    return wins / (len(up) * len(down))


def spread(rows: list[dict], signal: Callable[[dict], float],
           draws: int = 4000, seed: int = 7) -> tuple[float, float, float]:
    """A 95% interval on that number, and how often it comes out at chance.

    Seventeen merges is a small sample and an AUC computed from it deserves to
    be shown with its width rather than to three decimal places on its own. The
    third number is the share of resamples at or below 0.5, which is the plainest
    way to say how close a signal is to having nothing in it.
    """
    up = [row for row in rows if row["verdict"] == MERGE]
    down = [row for row in rows if row["verdict"] == CLEAN]
    rnd = random.Random(seed)
    got = []
    for _ in range(draws):
        sample = ([rnd.choice(up) for _ in up]
                  + [rnd.choice(down) for _ in down])
        value = auc(sample, signal)
        if value is not None:
            got.append(value)
    got.sort()
    if not got:
        return (float("nan"), float("nan"), float("nan"))
    return (got[int(0.025 * len(got))], got[int(0.975 * len(got)) - 1],
            sum(1 for one in got if one <= 0.5) / len(got))


def gate(rows: list[dict], signal: Callable[[dict], float]
         ) -> dict[str, Any]:
    """The tightest threshold admitting no merge, and what it costs.

    Eligible means below the threshold, because every signal here is a measure
    of doubt. The answer that matters is `clean_kept`: a gate that admits no
    merge by admitting nothing is not a remedy, and this is where that shows.
    """
    merges = [signal(row) for row in rows if row["verdict"] == MERGE]
    clean = [signal(row) for row in rows if row["verdict"] == CLEAN]
    if not merges:
        return {"threshold": None, "why": "no merge in this set"}
    lowest = min(merges)
    kept = sum(1 for value in clean if value < lowest)
    return {"threshold": lowest, "clean_kept": kept, "clean_total": len(clean),
            "share": kept / len(clean) if clean else 0.0}


def table(rows: list[dict], signal: Callable[[dict], float],
          thresholds: list[float]) -> list[dict]:
    """What each threshold would admit, merge and clean side by side."""
    merges = [signal(row) for row in rows if row["verdict"] == MERGE]
    clean = [signal(row) for row in rows if row["verdict"] == CLEAN]
    out = []
    for edge in thresholds:
        got_merge = sum(1 for value in merges if value < edge)
        got_clean = sum(1 for value in clean if value < edge)
        out.append({"threshold": edge, "merges_in": got_merge,
                    "clean_in": got_clean,
                    "clean_lost": len(clean) - got_clean})
    return out


def enrichment(rows: list[dict], signal: Callable[[dict], float],
               top: int = CONSIDERED) -> dict[str, Any]:
    """How many of the worst-ranked things are merges, against the base rate."""
    ranked = sorted(rows, key=signal, reverse=True)[:top]
    merges = sum(1 for row in ranked if row["verdict"] == MERGE)
    base = sum(1 for row in rows if row["verdict"] == MERGE) / len(rows)
    return {"top": top, "merges": merges, "rate": merges / len(ranked),
            "base_rate": base}


# --- signals ------------------------------------------------------------------

def _uncertainty(row: dict) -> float:
    return float(row["uncertainty_m"])


def _looks(row: dict) -> float:
    """How many looks the thing holds. Here as the control, not as a candidate.

    A merge is made of two objects' looks, so it should hold more of them --
    and it does, slightly, which is exactly why a candidate signal has to be
    shown beating this rather than merely beating chance.
    """
    return float(len(row["looks"]))


#: The labelled set also carries `viewpoints` and `rays_agreeing` as the review
#: reported them, and neither is scored: the script that wrote them is gone and
#: their definitions did not survive it -- `rays_agreeing` exceeds the look
#: count on four things, so whatever it counts, it is not looks. See
#: `as_reported` in the labels.
SIGNALS: dict[str, Callable[[dict], float]] = {
    "uncertainty_m": _uncertainty,
    "looks": _looks,
}

#: How many rows the operating table gets. The thresholds themselves are read
#: off the data rather than written down here, because a list of round numbers
#: chosen by the person reporting the result is a place to flatter it -- and
#: because two of the signals are not measured in metres at all.
STEPS = 9


def steps(rows: list[dict], signal: Callable[[dict], float],
          count: int = STEPS) -> list[float]:
    """Where to sample the operating table: evenly spaced through the ranking."""
    got = sorted({signal(row) for row in rows
                  if row["verdict"] in (MERGE, CLEAN)})
    if len(got) <= count:
        return got
    return [got[round(i * (len(got) - 1) / (count - 1))] for i in range(count)]


# --- saying it ----------------------------------------------------------------

def report(rows: list[dict], names: list[str]) -> str:
    lines = []
    counts: dict[str, int] = {}
    for row in rows:
        counts[row["verdict"]] = counts.get(row["verdict"], 0) + 1
    lines.append(f"{len(rows)} labelled things: "
                 + ", ".join(f"{counts.get(kind, 0)} {kind}"
                             for kind in (CLEAN, MERGE, SURFACE)))

    for name in names:
        signal = SIGNALS[name]
        lines.append("")
        lines.append(f"--- {name}")
        for kind in (CLEAN, MERGE, SURFACE):
            got = [signal(row) for row in rows if row["verdict"] == kind]
            if got:
                lines.append(f"  {kind:8} median {statistics.median(got):7.3f}"
                             f"   mean {statistics.mean(got):7.3f}"
                             f"   n {len(got)}")
        value = auc(rows, signal)
        if value is None:
            lines.append("  not enough of both kinds to rank")
            continue
        low, high, nothing = spread(rows, signal)
        lines.append(f"  a merge outranks a real object {value:.3f} of the time"
                     f" (95% {low:.3f}-{high:.3f}; chance is 0.500,"
                     f" and {nothing:.1%} of resamples land at or below it)")

        got = gate(rows, signal)
        if got.get("threshold") is None:
            lines.append(f"  no gate: {got['why']}")
        else:
            lines.append(
                f"  the tightest gate admitting no merge is {got['threshold']:.3f}"
                f", and it keeps {got['clean_kept']} of {got['clean_total']}"
                f" real objects ({got['share']:.0%})")

        lines.append(f"  {'admit below':>12} {'merges in':>10} {'clean in':>9}"
                     f" {'clean lost':>11}")
        for line in table(rows, signal, steps(rows, signal)):
            lines.append(f"  {line['threshold']:12.3f} {line['merges_in']:10d}"
                         f" {line['clean_in']:9d} {line['clean_lost']:11d}")

        rich = enrichment(rows, signal)
        lines.append(f"  the worst {rich['top']} by this signal are"
                     f" {rich['merges']} merges ({rich['rate']:.0%}),"
                     f" against a base rate of {rich['base_rate']:.0%}")
    return "\n".join(lines)


def worst(rows: list[dict], signal: Callable[[dict], float],
          top: int = CONSIDERED) -> str:
    lines = [f"the worst {top}, which is what the rover would re-look at first:"]
    for row in sorted(rows, key=signal, reverse=True)[:top]:
        lines.append(f"  {signal(row):6.2f}  {row['labelled_as']:12}"
                     f" {row['verdict']:8} {row['what']}")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Score a signal against labelled things.")
    parser.add_argument("--labels", default=LABELS)
    parser.add_argument("--things", help="a later rebuild's things.json, "
                        "joined to the verdicts by which looks each thing holds")
    parser.add_argument("--signal", action="append", choices=sorted(SIGNALS),
                        help="repeatable; every signal by default")
    parser.add_argument("--list", action="store_true",
                        help="name the worst things rather than scoring")
    args = parser.parse_args()

    labels = load(args.labels)
    if args.things:
        with open(args.things, encoding="utf-8") as handle:
            things = json.load(handle)
        matched, lost = join(labels, things)
        rows = list(matched.values())
        print(f"{len(rows)} of {len(labels['things'])} verdicts found a thing "
              f"in {os.path.basename(args.things)}")
        if lost:
            print(f"  {len(lost)} lost every look they were labelled on: "
                  + ", ".join(lost[:8]) + (" and others" if len(lost) > 8 else ""))
    else:
        rows = list(rows_of(labels).values())

    if args.list:
        print(worst(rows, SIGNALS["uncertainty_m"]))
        return 0

    print(report(rows, args.signal or sorted(SIGNALS)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
