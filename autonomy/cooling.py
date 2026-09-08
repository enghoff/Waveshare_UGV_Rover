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

#: How long a target stays cool. Fifteen minutes is long enough that the rover
#: gets on with something else and short enough that a thing which only needed a
#: different viewpoint is not written off for the afternoon.
COOLDOWN_S = 900.0

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
        placement = entity.get("placement") or {}
        uncertainty = placement.get("error_major_m")
        if uncertainty is None:
            uncertainty = entity.get("placement_uncertainty_m")
        out[target] = {
            "looks": int(entity.get("observation_count") or 0),
            "uncertainty_m": (None if uncertainty is None
                              else float(uncertainty))}
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
