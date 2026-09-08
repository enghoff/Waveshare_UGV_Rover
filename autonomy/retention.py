"""Keeping the record from filling the rover's disk, and saying what it removed.

A look costs a copied frame, and a rover left switched on takes looks all day.
Something has to remove them, and the whole difficulty is that removing evidence
is the one destructive act this component can perform -- so it is done through
`store.delete_evidence`, which leaves a row saying when and why, and a replay of
an affected episode afterwards reports it as no longer fully replayable rather
than quietly summarising an episode nobody can check.

Three rules, in the order they are applied:

**A pinned episode's evidence is never touched.** An acceptance recording is what
this is for. It is applied first and absolutely, so a pinned episode survives
even a store that is far over its limit -- the honest failure there is a full
disk with a loud reason, not a quietly deleted recording somebody was arguing
from.

**Evidence older than the age limit goes.** The reason for an age limit as well
as a size one is that a rover switched off for a month should not come back and
delete a week of recent looks because the total is over.

**Then oldest-first until the store is under its size limit.** Oldest first
because the value of a look decays and the newest looks are the ones somebody is
about to ask about.

Nothing here removes an episode. The account of what the rover did is small --
a few kilobytes of rows -- and it stays for ever; what goes is the pictures
behind it. An episode whose pictures have gone still says what happened, and
says that it can no longer show you.
"""
from __future__ import annotations

import time
from typing import Any, NamedTuple

import store as store_mod


class Policy(NamedTuple):
    """What the record is allowed to occupy.

    The numbers live here rather than in a document, because a document that
    carries a value creates a second place for it to go stale. The rover's own
    figure is whatever `DEFAULT` says at the commit it was deployed at.
    """

    keep_days: float
    max_bytes: int

    def describe(self) -> str:
        return (f"evidence is kept for {self.keep_days:g} days and up to "
                f"{self.max_bytes / (1024 ** 3):.1f} GB, oldest removed first, "
                f"pinned episodes never")


#: Chosen against the Orin's disk and the measured cost of a look. Deliberately
#: modest: the record is not the reason the rover has storage, and the number
#: that matters is that it cannot grow without bound.
DEFAULT = Policy(keep_days=14.0, max_bytes=2 * 1024 ** 3)


def apply(store: store_mod.EpisodeStore, policy: Policy = DEFAULT, *,
          now: float | None = None, dry_run: bool = False) -> dict[str, Any]:
    """Bring the record within the policy, and say exactly what was removed.

    `dry_run` answers "what would this take" without taking it, which is the
    form to use before turning retention on against a store somebody cares
    about.
    """
    stamp = time.time() if now is None else now
    protected = pinned_evidence(store)
    held = store.evidence_held(limit=100000)

    total = sum(one["bytes"] for one in held)
    oldest_allowed = stamp - policy.keep_days * 86400.0

    removed: list[dict[str, Any]] = []
    kept_pinned = 0
    freed = 0

    for one in held:
        if one["digest"] in protected:
            kept_pinned += 1
            continue
        too_old = one["stored_at"] < oldest_allowed
        too_big = total - freed > policy.max_bytes
        if not (too_old or too_big):
            continue
        why = ("retention: older than %g days" % policy.keep_days if too_old
               else "retention: the record was over %.1f GB"
                    % (policy.max_bytes / (1024 ** 3)))
        if not dry_run:
            store.delete_evidence(one["digest"], why,
                                  detail=f"kept since {_when(one['stored_at'])}")
        freed += one["bytes"]
        removed.append({"digest": one["digest"], "bytes": one["bytes"],
                        "stored_at": one["stored_at"], "why": why})

    return {
        "policy": policy.describe(),
        "held_before": total,
        "held_after": total - freed,
        "freed": freed,
        "removed": len(removed),
        "pinned_kept": kept_pinned,
        "over_limit_still": (total - freed) > policy.max_bytes,
        "dry_run": dry_run,
        "detail": removed[:20],
    }


def pinned_evidence(store: store_mod.EpisodeStore) -> set[str]:
    """Every digest a pinned episode stands on.

    Built fresh each pass rather than cached, because pinning is a thing a
    person does between passes and a stale set is the one way this could delete
    the recording it was told to keep.
    """
    out: set[str] = set()
    for ref in store.pinned():
        out.update(store.evidence_of(ref))
    return out


#: The shortest span a rate may be extrapolated from. Not a round number for
#: its own sake: a recorder starting against a rover with a month of history
#: copies hundreds of frames in a few seconds, and a rate taken across that span
#: says the disk fills in minutes. Refusing to answer is the only honest
#: response to it.
MIN_SPAN_H = 0.1


def would_fill(store: store_mod.EpisodeStore, *, hours: float = 0.0,
               policy: Policy = DEFAULT) -> dict[str, Any]:
    """How long the record has at the rate it has been growing, or why not.

    Measured from what is actually in the store rather than from an assumed
    frame size: the span between the oldest and newest evidence, and the bytes
    between them. `hours` overrides that span, for the caller who knows how long
    the recording really ran -- which is the answer to want after a catch-up,
    because the frames a catch-up copies are stamped with when they were
    *copied* and not with when the rover took them.

    **It refuses rather than guesses.** A rate from a span of a few seconds is
    the shape of a number somebody quotes in a report, and it would be wrong by
    three orders of magnitude.
    """
    held = store.evidence_held(limit=100000)
    if len(held) < 2:
        return {"known": False,
                "why": "not enough evidence to measure a rate from"}
    span_s = held[-1]["stored_at"] - held[0]["stored_at"]
    span_h = hours if hours > 0 else span_s / 3600.0
    if span_h < MIN_SPAN_H:
        return {"known": False, "frames": len(held),
                "why": (f"the evidence in this store spans {span_h * 3600:.0f} "
                        f"seconds, which is too short to extrapolate from; if "
                        f"a catch-up copied it, pass the hours the recording "
                        f"really ran for")}
    total = sum(one["bytes"] for one in held)
    rate = total / span_h
    return {
        "known": True,
        "bytes_per_hour": round(rate),
        "megabytes_per_hour": round(rate / (1024 ** 2), 1),
        "frames": len(held),
        "hours_measured": round(span_h, 3),
        "hours_to_limit": (round(policy.max_bytes / rate, 1) if rate else None),
        "days_to_limit": (round(policy.max_bytes / rate / 24.0, 1)
                          if rate else None),
    }


def _when(stamp: float) -> str:
    return time.strftime("%Y-%m-%d %H:%M", time.localtime(stamp))
