#!/usr/bin/env python3
"""Put the things an old map left behind back on the map the rover is on.

    python world_state/reanchor.py                 # what it would do
    python world_state/reanchor.py --apply         # do it

**What this is for.** A position is a position in the map it was measured in, so
when the SLAM map is replaced every coordinate recorded under the old one stops
being a place -- and on this rover the map has been replaced sixty-odd times, so
by 2026-09-06 four fifths of everything it knew stood in maps that no longer
existed. `resolve._adopt` stops that happening again, by recognising a thing when
a fresh crossing plainly looks like it, but it only reaches what the rover
happens to look at twice more, and it refuses wherever two remembered things look
alike -- which on a pool full of duplicates is most of them. It does nothing at
all for the backlog.

**What makes the backlog recoverable is that the room did not move.** Two maps of
one room differ by a rotation and a shift and nothing else, so if that transform
can be found, every stranded position can be carried across at once -- including
the things in rooms the rover has not been back to, which is exactly what
recognition can never reach. The transform is found from the things that *are* in
both maps: pair them by appearance, then let those pairs vote. A pair that
disagrees with the consensus is evidence against itself rather than something
that drags the answer, which is why this is RANSAC and not a least-squares fit
over everything.

Measured on the rover's own store, 2026-09-06: map 62 gave 119 candidate pairs of
which 45 agreed on +76.6 degrees to within 0.75 m, and map 59 gave 66 of which 24
agreed on +82.0 degrees. Both estimates held to a degree across every inlier
tolerance from 0.5 m to 2 m, which is the thing worth checking -- a fit made of
coincidences wanders as the tolerance moves, and these do not.

**Landing on top of something is not a success.** Two thirds of what the old maps
hold was found again in the new one and is already standing there, so carrying
those across would leave the room holding two entities per object, half a metre
apart and looking alike -- and the resolver would then call every bearing at
either of them ambiguous and attach nothing, for ever. So a thing that lands on
its own double is merged into it instead of being moved, which is also what puts
the older row's history back where somebody asking about it will find it.

The evidence for a merge is deliberately not appearance alone: it is appearance
*and* landing within `SAME_THING_M` under a transform fitted from other pairs
entirely, which is an independent measurement rather than a second opinion from
the same one. Where two remembered things look alike and both land close, nothing
is merged and nothing is moved -- the same refusal `resolve._adopt` makes, for the
same reason, because two rows wrongly made one cannot be separated afterwards.
"""
from __future__ import annotations

import argparse
import itertools
import math
import os
import sys
from typing import Any

if __package__ in (None, ""):                       # run as a script on the rover
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from world_state.appearance import any_of
from world_state.resolve import RECOGNISED
from world_state.store import WorldStore

#: How close a carried position has to land to a pair's real position for that
#: pair to be counted as agreeing with the transform. Wider than the placements'
#: own uncertainty on purpose: what is being separated here is a pair that is the
#: same object from a pair that is two different chairs, and the first of those
#: misses by tens of centimetres while the second misses by metres.
FIT_TOLERANCE_M = 0.75
#: How many pairs have to agree before a transform is worth trusting with every
#: position in a map. Three fix a rotation and a shift with nothing left over to
#: check them; this wants a consensus.
MIN_AGREEING = 8
#: Landing this close to something that also looks like it means it *is* it.
SAME_THING_M = 1.0
#: ...unless a second look-alike is as close or nearly so, in which case which
#: one it is cannot be told and nothing is done. Widened to the transform's own
#: residual where that is worse. See the module docstring.
SECOND_PLACE_M = 0.5


# --- finding the transform ---------------------------------------------------

def vector_width(store) -> int:
    """How many bytes one appearance vector is, in this store.

    Asked of the data rather than assumed, because the exemplars are a
    concatenation and splitting them needs the width. It is 1536 for the DINOv2
    the rover runs, but a store written by another backend -- or by a test -- has
    its own, and a wrong width turns every comparison into a comparison of
    misaligned bytes, which scores near zero and quietly finds nothing.
    """
    with store._lock:
        row = store.db.execute(
            "SELECT LENGTH(dino_blob) AS width FROM observations"
            " WHERE dino_blob IS NOT NULL LIMIT 1").fetchone()
    return int(row["width"]) if row and row["width"] else 1536


def looks_like(store, one: dict[str, Any], other: dict[str, Any],
               width: int) -> float:
    """How much one remembered thing looks like another, on the exemplars.

    `appearance.any_of` asked the other way round from usual: the crops are the
    first thing's exemplars rather than one fresh observation, and the answer is
    still the middle of the second thing's, so a single odd exemplar on either
    side moves it one place along and no further.
    """
    mine = store.exemplars(one["id"], width=width)
    if not mine:
        return 0.0
    got = any_of(store, other["id"], mine)
    return 0.0 if got is None else got


def pairs_between(store, theirs: list[dict], here: list[dict],
                  width: int) -> list[tuple]:
    """Every stranded thing beside the thing in this map that most looks like it.

    Only the best, and only above `RECOGNISED`: this is the same bar a fresh
    crossing has to clear to take back an identity, and it is here for the same
    reason -- below it the two are not the same object and the pair would be
    voting on a transform between unrelated points.
    """
    pairs = []
    for one in theirs:
        best, who = 0.0, None
        for other in here:
            got = looks_like(store, one, other, width)
            if got > best:
                best, who = got, other
        if who is not None and best >= RECOGNISED:
            pairs.append((one, who, best))
    return pairs


def fit(pairs: list[tuple]) -> tuple[float, tuple[float, float]]:
    """The rotation and shift that best carry the old positions onto the new.

    The closed-form least-squares answer for a rigid 2D transform: the rotation
    is the angle of the summed cross and dot products about the two centroids,
    and the shift is whatever is then needed to bring the centroids together.
    No scaling term, deliberately -- two maps of one room are the same size, and
    a scale free to move would absorb exactly the drift this is meant to expose.
    """
    old = [one["placement"] for one, _, _ in pairs]
    new = [other["placement"] for _, other, _ in pairs]
    ox = sum(p["x_m"] for p in old) / len(old)
    oy = sum(p["y_m"] for p in old) / len(old)
    nx = sum(p["x_m"] for p in new) / len(new)
    ny = sum(p["y_m"] for p in new) / len(new)
    across = along = 0.0
    for before, after in zip(old, new):
        ax, ay = before["x_m"] - ox, before["y_m"] - oy
        bx, by = after["x_m"] - nx, after["y_m"] - ny
        across += ax * by - ay * bx
        along += ax * bx + ay * by
    turn = math.atan2(across, along)
    cos, sin = math.cos(turn), math.sin(turn)
    return turn, (nx - (cos * ox - sin * oy), ny - (sin * ox + cos * oy))


def carried(x_m: float, y_m: float, turn: float,
            shift: tuple[float, float]) -> tuple[float, float]:
    """One point of the old map, as a point of this one."""
    cos, sin = math.cos(turn), math.sin(turn)
    return (cos * x_m - sin * y_m + shift[0], sin * x_m + cos * y_m + shift[1])


def _miss(pair: tuple, turn: float, shift: tuple[float, float]) -> float:
    one, other, _ = pair
    x, y = carried(one["placement"]["x_m"], one["placement"]["y_m"], turn, shift)
    return math.hypot(x - other["placement"]["x_m"],
                      y - other["placement"]["y_m"])


def align(pairs: list[tuple],
          tolerance: float = FIT_TOLERANCE_M) -> dict[str, Any] | None:
    """The transform the most pairs agree on, refitted on those that agree.

    Every pair of pairs is tried as a hypothesis, which is exhaustive rather than
    sampled: two correspondences determine a rigid transform exactly, and with a
    couple of hundred candidates there are only tens of thousands of hypotheses
    -- a second of arithmetic, against a random search that would have to justify
    how many draws were enough.
    """
    if len(pairs) < 2:
        return None
    best = (0, 0.0, (0.0, 0.0))
    for first, second in itertools.combinations(range(len(pairs)), 2):
        turn, shift = fit([pairs[first], pairs[second]])
        agreed = sum(1 for one in pairs if _miss(one, turn, shift) <= tolerance)
        if agreed > best[0]:
            best = (agreed, turn, shift)
    if best[0] < 2:
        return None
    # Refit on the consensus rather than keeping the two that found it, then take
    # the consensus again: the hypothesis pair is two measurements and the refit
    # is all of them, so a pair sitting just outside the tolerance of a fit made
    # from two points is usually inside the one made from thirty.
    turn, shift = best[1], best[2]
    for _ in range(2):
        agreeing = [one for one in pairs if _miss(one, turn, shift) <= tolerance]
        if len(agreeing) < 2:
            return None
        turn, shift = fit(agreeing)
    agreeing = [one for one in pairs if _miss(one, turn, shift) <= tolerance]
    misses = sorted(_miss(one, turn, shift) for one in agreeing)
    return {"turn_deg": math.degrees(turn), "turn": turn, "shift": shift,
            "agreeing": len(agreeing), "candidates": len(pairs),
            "residual_m": misses[len(misses) // 2] if misses else 0.0,
            "worst_m": misses[-1] if misses else 0.0,
            "pairs": agreeing}


def steady(pairs: list[tuple]) -> str:
    """Whether the answer holds still as the tolerance moves, said in words.

    **The check that separates a transform from a coincidence**, and it is worth
    more than the inlier count. Correspondences drawn by appearance alone are
    noisy -- this rover's rooms are full of things that look like each other --
    so a modest fraction agreeing proves little on its own. What does prove
    something is that widening or tightening what counts as agreement moves the
    answer by a degree rather than by tens of them.
    """
    seen = []
    for tolerance in (0.5, 0.75, 1.0, 1.5, 2.0):
        got = align(pairs, tolerance)
        if got:
            seen.append((tolerance, got["turn_deg"], got["agreeing"]))
    if len(seen) < 3:
        return "too few agreeing pairs to say"
    spread = max(one[1] for one in seen) - min(one[1] for one in seen)
    detail = ", ".join(f"{one[0]:.2f} m: {one[2]} at {one[1]:+.1f} deg"
                       for one in seen)
    return (f"holds to {spread:.1f} deg across the tolerances ({detail})"
            if spread <= 3.0 else
            f"WANDERS by {spread:.1f} deg across the tolerances ({detail})")


# --- what to do about each stranded thing ------------------------------------

def moved(placement: dict[str, Any], turn: float, shift: tuple[float, float],
          residual_m: float, from_map: Any) -> dict[str, Any]:
    """A placement of the old map written as one of this map.

    The position turns and shifts; so does the direction of its error ellipse,
    which is an angle in the same frame and would otherwise be left pointing at
    the old map's north. The size of the error grows by the transform's own
    residual, added in quadrature because the two are independent -- what the
    rover measured about the thing, and how well the two maps are known to line
    up. Nothing else is touched: how many bearings agreed and how far apart they
    were are facts about looks that were taken, and carrying the answer to a new
    frame does not make them different facts.
    """
    x_m, y_m = carried(placement["x_m"], placement["y_m"], turn, shift)
    grown = dict(placement, x_m=round(x_m, 3), y_m=round(y_m, 3))
    for name in ("uncertainty_m", "error_major_m", "error_minor_m"):
        if placement.get(name) is not None:
            grown[name] = round(math.hypot(float(placement[name]), residual_m), 3)
    if placement.get("error_major_deg") is not None:
        grown["error_major_deg"] = round(
            (float(placement["error_major_deg"]) + math.degrees(turn) + 180.0)
            % 360.0 - 180.0, 1)
    grown["carried_from_map"] = from_map
    return grown


def plan(store, min_agreeing: int = MIN_AGREEING) -> dict[str, Any]:
    """What re-anchoring would do to this store, without doing any of it.

    `min_agreeing` is how much consensus a transform needs before a whole map's
    positions are carried on its word. It is an argument only so that the checks
    can exercise the mechanism in a room with six things in it rather than a
    house with three hundred; nothing that runs on the rover passes it.
    """
    now = store.map_session()
    width = vector_width(store)
    here = store.placed(now)
    stranded = store.placed_elsewhere(now)
    by_map: dict[Any, list] = {}
    for one in stranded:
        by_map.setdefault(one.get("placement_map_session"), []).append(one)

    maps = []
    for old_map in sorted(by_map, key=lambda one: (one is None, one)):
        theirs = by_map[old_map]
        pairs = pairs_between(store, theirs, here, width)
        found = align(pairs)
        note = steady(pairs) if pairs else "nothing in this map was found again"
        entry = {"map": old_map, "things": len(theirs), "candidates": len(pairs),
                 "steady": note, "fit": found, "move": [], "merge": [],
                 "left": []}
        if not found or found["agreeing"] < min_agreeing:
            entry["left"] = [one["id"] for one in theirs]
            entry["why"] = (
                f"only {0 if not found else found['agreeing']} pairs agree, and "
                f"{min_agreeing} are wanted before every position in a map is "
                f"carried on their word")
            maps.append(entry)
            continue
        turn, shift = found["turn"], found["shift"]
        # One thing in this map may absorb at most one thing from any one old
        # map. Two rows of the same old map are two different things unless
        # that map was itself duplicating -- and where they both land on one
        # double, which of them it is cannot be told apart from two chairs side
        # by side. Across *different* old maps there is no such rule: the same
        # sofa recorded in map 59 and again in map 62 is one sofa, and both
        # belong to the row standing here now.
        spoken_for: dict[str, tuple[float, dict]] = {}
        for one in theirs:
            x_m, y_m = carried(one["placement"]["x_m"], one["placement"]["y_m"],
                               turn, shift)
            near = []
            for other in here:
                gap = math.hypot(other["placement"]["x_m"] - x_m,
                                 other["placement"]["y_m"] - y_m)
                if gap <= SAME_THING_M + SECOND_PLACE_M:
                    if looks_like(store, one, other, width) >= RECOGNISED:
                        near.append((gap, other["id"]))
            near.sort()
            if near and near[0][0] <= SAME_THING_M:
                # A tie is refused rather than broken, so the comparison is
                # "not clearly further" and not "closer"; and what counts as
                # clearly is never finer than the transform's own residual,
                # because two candidates a worse fit apart are two candidates
                # this cannot tell between however the arithmetic falls.
                margin = max(SECOND_PLACE_M, found["residual_m"])
                if len(near) > 1 and near[1][0] - near[0][0] <= margin:
                    entry["left"].append(one["id"])
                    continue
                gap, into = near[0]
                held = spoken_for.get(into)
                if held is not None:
                    loser = one if held[0] <= gap else held[1]
                    entry["left"].append(loser["id"])
                    if loser is one:
                        continue
                    entry["merge"] = [row for row in entry["merge"]
                                      if row["gone"] != loser["id"]]
                spoken_for[into] = (gap, one)
                entry["merge"].append({"gone": one["id"], "into": into,
                                       "gap_m": round(gap, 2),
                                       "seen": one.get("observation_count") or 0})
            else:
                entry["move"].append({
                    "id": one["id"], "to": (round(x_m, 2), round(y_m, 2)),
                    "seen": one.get("observation_count") or 0,
                    "placement": moved(one["placement"], turn, shift,
                                       found["residual_m"], old_map)})
        maps.append(entry)
    return {"map_session": now, "here": len(here), "stranded": len(stranded),
            "maps": maps}


def apply(store, decided: dict[str, Any]) -> dict[str, Any]:
    """Carry out a plan.

    Through `WorldStore.place` one thing at a time rather than as one statement
    over the table, because a carried position is a placement like any other and
    the store is what knows what writing one means -- the uncertainty column
    beside the JSON, and the timestamp that says when the rover last changed its
    mind about where this is.

    Not atomic, and it does not need to be: the daemon goes on looking while this
    runs, but a run that died halfway would leave some things carried and the
    rest still stranded, which is the state this started in for those rows and is
    fixed by running it again.
    """
    now = decided["map_session"]
    done = {"moved": 0, "merged": 0, "refused": []}
    for entry in decided["maps"]:
        for one in entry["move"]:
            store.place(one["id"], one["placement"], now)
            done["moved"] += 1
    # Merges last, and each on its own: every one is its own consistency check
    # and its own refusal, and a shared frame between one pair must not stop the
    # rest.
    for entry in decided["maps"]:
        for one in entry["merge"]:
            got = store.merge(one["into"], one["gone"])
            if got.get("ok"):
                done["merged"] += 1
            else:
                done["refused"].append(got.get("why"))
    return done


def report(decided: dict[str, Any]) -> None:
    print(f"map session {decided['map_session']}: {decided['here']} things "
          f"standing in it, {decided['stranded']} stranded in older maps")
    for entry in decided["maps"]:
        found = entry["fit"]
        print(f"\nmap {entry['map']}: {entry['things']} things, "
              f"{entry['candidates']} of them found again in this map")
        print(f"  {entry['steady']}")
        if not found or entry.get("why"):
            print(f"  LEFT ALONE: {entry.get('why', 'no transform')}")
            continue
        print(f"  turn {found['turn_deg']:+.1f} deg, shift "
              f"({found['shift'][0]:+.2f}, {found['shift'][1]:+.2f}) m, from "
              f"{found['agreeing']} agreeing pairs, median miss "
              f"{found['residual_m']:.2f} m, worst {found['worst_m']:.2f} m")
        print(f"  {len(entry['move'])} carried onto this map, "
              f"{len(entry['merge'])} folded into the thing already standing "
              f"there, {len(entry['left'])} left alone")
        if entry["move"]:
            far = sorted(math.hypot(*one["to"]) for one in entry["move"])
            print(f"  the carried ones land {far[0]:.1f} m to {far[-1]:.1f} m "
                  f"from the map's origin, median {far[len(far)//2]:.1f} m")
        for one in sorted(entry["merge"], key=lambda one: -one["seen"])[:5]:
            print(f"    {one['gone']} ({one['seen']} looks) -> {one['into']}, "
                  f"{one['gap_m']} m apart")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--dir", default=os.path.expanduser("~/.ugv/world"),
                        help="where world.db lives")
    parser.add_argument("--apply", action="store_true",
                        help="carry it out; without this nothing is written")
    args = parser.parse_args()

    store = WorldStore(args.dir)
    try:
        decided = plan(store)
        report(decided)
        if not args.apply:
            print("\nnothing was written -- pass --apply to carry this out")
            return 0
        done = apply(store, decided)
        print(f"\ncarried {done['moved']} onto map {decided['map_session']}, "
              f"folded {done['merged']} into what was already there")
        for why in done["refused"]:
            print(f"  refused: {why}")
        left = store.placed_elsewhere(store.map_session())
        print(f"{len(left)} things are still stranded")
        return 0
    finally:
        store.close()


if __name__ == "__main__":
    sys.exit(main())
