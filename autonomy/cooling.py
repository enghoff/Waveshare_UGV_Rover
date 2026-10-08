"""Which things are not worth looking at again just now, and when that lapses.

**The failure this exists to prevent is a rover that mistakes repetition for
progress.** Some ambiguities do not resolve from anywhere the rover can stand --
a thing whose two bearings are nearly parallel, a crop that is half a sofa and
half the wall behind it -- and a scorer with no memory will keep offering the
same inspection, scoring it well every time, because the gap it would close is
still open. That is a rover busy all afternoon with nothing to show, and it
looks exactly like a rover working.

So a target that has taken more looks without coming out any better is put aside
for a while. Two things end that:

- **Time.** The room changes, the rover ends up somewhere else, and a viewpoint
  that was not available before may be now.
- **Evidence.** If the thing's placement does actually improve, the reason for
  cooling it has gone, and it resumes immediately rather than serving out a
  sentence.

Both are decided from what is written in the situation rather than from
anything held in memory here, which is what keeps a replay honest: the cooling
that applied to a decision is part of the inputs that decision was made from,
and is snapshotted with them.
"""
from __future__ import annotations

from typing import Any

import situation as situation_mod

#: How long a target stays cool. Fifteen minutes is long enough that the rover
#: gets on with something else and short enough that a thing which only needed a
#: different viewpoint is not written off for the afternoon.
COOLDOWN_S = 900.0

#: How long a thing stays aside when a look aimed at it, taken from where the
#: depth camera could see its place, filed nothing to it: the record is probably
#: not where it claims to be. Two hours, about a supervised session. On
#: 2026-10-08 a look at a record that had already come up empty filed 1 time in
#: 15 (docs/progress/2026-10-08-what-predicts-a-filing.md); an improvement to
#: its placement still brings it back at once.
EMPTY_COOLDOWN_S = 7200.0

#: How many more looks a thing may take between two deliberations while getting
#: no better placed, before it is put aside. Three rather than one: a single
#: look that does not help is ordinary -- most looks do not cross with anything
#: -- and cooling on one would cool nearly everything.
COOLDOWN_LOOKS = 3

#: What counts as the placement actually improving, in metres. Below this the
#: number moved but nothing was learned; the resolver refines a placement by
#: millimetres every time it revisits the same rays.
IMPROVED_M = 0.02


def update(previous: dict[str, Any] | None, current: dict[str, Any],
           cooled: list[dict[str, Any]] | None, *, now: float
           ) -> list[dict[str, Any]]:
    """The cooling list for this deliberation, from the last one and this.

    `previous` is the situation the previous deliberation was made from, or None
    on the first. Everything is read off the two readings rather than counted
    here, so a recorder that was stopped and started again resumes with whatever
    the record says rather than with an empty memory.
    """
    now = float(now)
    was = _by_id(previous)
    is_now = _by_id(current)

    kept: list[dict[str, Any]] = []
    for entry in cooled or []:
        target = str(entry.get("target") or "")
        if float(entry.get("until") or 0.0) <= now:
            continue
        here = is_now.get(target)
        if here is None:
            # The thing has gone -- merged into another, or taken away by a
            # clear. Nothing to cool, and keeping the entry would cool whatever
            # inherits the identifier.
            continue
        if _improved(entry.get("uncertainty_m"), here["uncertainty_m"]):
            continue
        kept.append(dict(entry))

    already = {str(one.get("target") or "") for one in kept}
    for target, here in sorted(is_now.items()):
        if target in already:
            continue
        before = was.get(target)
        if before is None:
            continue
        gained = here["looks"] - before["looks"]
        if gained < COOLDOWN_LOOKS:
            continue
        if _improved(before["uncertainty_m"], here["uncertainty_m"]):
            continue
        kept.append({
            "target": target,
            "since": now,
            "until": now + COOLDOWN_S,
            "looks": here["looks"],
            "uncertainty_m": here["uncertainty_m"],
            "why": (f"{gained} more looks at {target} left it placed to "
                    f"{_metres(here['uncertainty_m'])} -- no better than "
                    f"before, so it is put aside for "
                    f"{int(COOLDOWN_S / 60)} minutes or until something "
                    f"changes")})
    kept.sort(key=lambda one: str(one.get("target") or ""))
    return kept


def after_attempt(cooled: list[dict[str, Any]] | None, target: str,
                  before: dict[str, Any] | None, after: dict[str, Any], *,
                  now: float, seen_empty: bool = False) -> list[dict[str, Any]]:
    """The cooling list once a goal at `target` has been carried out.

    **A goal that went where it was sent, looked, and left the thing no better
    puts it aside at once.** Counting looks, as `update` does, cannot see this
    case: a look from a spot the rover has already looked from is the same
    picture, which the world state does not record at all, so the count never
    moves however often the goal is repeated. On 2026-10-03 a run chose the same
    look at object:7 twenty-two times in a row that way, from a viewpoint the
    rover could not stand at and was moved back off. One attempt is enough here,
    unlike one look: the goal was a deliberate viewpoint, and the scorer would
    choose the same one again.

    `before` and `after` are the situations either side of the attempt, read
    with the same measure `update` lapses an entry by. `seen_empty` is a look
    that could see the thing's place and found nothing to file to it, which
    sets the thing aside for `EMPTY_COOLDOWN_S` rather than `COOLDOWN_S`.
    """
    now = float(now)
    was = _by_id(before).get(target)
    here = _by_id(after).get(target)
    kept = [dict(one) for one in cooled or []]
    # **The other records of the same object are put aside whatever came of
    # it.** A look aimed at a thing files its region there, and where that
    # region also fitted other records, they are probably the same object
    # (world_state/aimed.py); so are the records the latest grouping joined
    # with it (world_state/reader_groups.py). Going to one of them next would be
    # going back to the thing just looked at, which is what the owner asked
    # about on 2026-10-08. Their own placement is left to lapse them as
    # `update` does.
    if here is not None:
        for other in here["same_as"]:
            if other == target or any(str(one.get("target") or "") == other
                                      for one in kept):
                continue
            again = _by_id(after).get(other) or {}
            kept.append({
                "target": other, "since": now, "until": now + COOLDOWN_S,
                "looks": again.get("looks", 0),
                "uncertainty_m": again.get("uncertainty_m"),
                "why": (f"{other} is probably the same object as {target}, "
                        f"which was looked at just now, so it is put aside for "
                        f"{int(COOLDOWN_S / 60)} minutes or until something changes")})
    # Gone, or helped: nothing more to add, and `update` decides the rest.
    if here is None or (was is not None
                        and _improved(was["uncertainty_m"], here["uncertainty_m"])):
        kept.sort(key=lambda one: str(one.get("target") or ""))
        return kept
    kept = [one for one in kept if str(one.get("target") or "") != target]
    aside = EMPTY_COOLDOWN_S if seen_empty else COOLDOWN_S
    kept.append({
        "target": target,
        "since": now,
        "until": now + aside,
        "looks": here["looks"],
        "uncertainty_m": here["uncertainty_m"],
        "why": ((f"a look at {target} from where the depth camera could see its "
                 f"place found nothing to file to it, so it is probably not where "
                 f"it is placed; put aside for {int(aside / 3600)} hours or until "
                 f"something changes") if seen_empty else
                (f"a goal at {target} left it placed to "
                 f"{_metres(here['uncertainty_m'])} -- no better than before, so "
                 f"it is put aside for {int(aside / 60)} minutes or until "
                 f"something changes"))})
    kept.sort(key=lambda one: str(one.get("target") or ""))
    return kept


#: How near a place navigation could not reach a goal has to be to count as the
#: same place, and how long it is left alone. Half a metre is `frontier.py`'s
#: blacklist radius -- the rover's width and a little, so the cell next door to
#: a doorway it could not get through is the same doorway. Half an hour because
#: the rover's own exploring writes such a place off for the rest of its run,
#: and a run from the console can last much longer than one exploration.
UNREACHABLE_M = 0.5
UNREACHABLE_S = 1800.0


def after_failed_drive(places: list[dict[str, Any]] | None,
                       goal: dict[str, Any] | None, why: str, *,
                       now: float) -> list[dict[str, Any]]:
    """The places not to drive to, once a drive to `goal` has failed.

    **The run's walk over the map is not navigation.** It counts free cells;
    Nav2 plans with the rover's whole body, so a frontier the walk puts five
    metres away can have no route the rover fits through. On 2026-10-03 one was
    driven at four times, forty seconds of recoveries each, until three failures
    in a row ended the run. The rover's own exploring keeps a blacklist for
    exactly this (`frontier.py`); this is the run's.
    """
    kept = [dict(one) for one in places or []
            if float(one.get("until") or 0.0) > float(now)]
    if not goal or goal.get("x_m") is None or goal.get("y_m") is None:
        return kept
    kept.append({"x_m": round(float(goal["x_m"]), 3),
                 "y_m": round(float(goal["y_m"]), 3),
                 "since": float(now), "until": float(now) + UNREACHABLE_S,
                 "why": str(why)[:300]})
    return kept


def unreachable_near(places: list[dict[str, Any]] | None, x: float, y: float,
                     *, now: float) -> dict[str, Any] | None:
    """The place navigation could not reach that a goal at (x, y) is, if any."""
    for one in places or []:
        if float(one.get("until") or 0.0) <= float(now):
            continue
        if (abs(float(one["x_m"]) - x) <= UNREACHABLE_M
                and abs(float(one["y_m"]) - y) <= UNREACHABLE_M
                and ((float(one["x_m"]) - x) ** 2 + (float(one["y_m"]) - y) ** 2
                     <= UNREACHABLE_M ** 2)):
            return one
    return None


def cooling(cooled: list[dict[str, Any]] | None, target: str, *,
            now: float) -> dict[str, Any] | None:
    """The entry cooling this target, if one is in force."""
    for entry in cooled or []:
        if str(entry.get("target") or "") != target:
            continue
        if float(entry.get("until") or 0.0) > float(now):
            return entry
    return None


def _by_id(body: dict[str, Any] | None) -> dict[str, dict[str, Any]]:
    """How each thing stood, keyed by identifier, out of a situation."""
    out: dict[str, dict[str, Any]] = {}
    for entity in (body or {}).get("entities") or []:
        target = str(entity.get("id") or "")
        if not target:
            continue
        uncertainty = situation_mod.claimed_m(entity)
        out[target] = {
            "looks": int(entity.get("observation_count") or 0),
            "uncertainty_m": uncertainty,
            # A look's same-object suspects, and the records the latest
            # grouping found to be one object with it (the daemon's
            # `_world_groups`): both mean that going to them next is going
            # back to the thing just looked at.
            "same_as": sorted({str(one) for one in [
                *(entity.get("same_object_suspects") or []),
                *(entity.get("group_mates") or [])]})}
    return out


def _improved(before: Any, after: Any) -> bool:
    """Did the placement actually get better, rather than merely move?"""
    if before is None or after is None:
        # A thing that had no placement and has one now has learned the most
        # there is to learn, so this counts as improvement; the other way round
        # is a placement that was withdrawn, which is a change worth resuming
        # for too.
        return before is not after
    return float(before) - float(after) >= IMPROVED_M


def _metres(value: Any) -> str:
    return "no position at all" if value is None else f"{float(value):.2f} m"
