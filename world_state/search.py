"""Find me the thing I described.

A phrase is embedded by SigLIP2's text tower, whose image tower produced every
stored region vector, so the two land in the same space and the comparison is a
dot product. A few hundred vectors is a few hundred multiplications, which is why
there is no vector database anywhere in this design and should not be one.

**This is the only thing that turns a picture into words now, and deliberately
so.** Regions used to be named by the nearest phrase in a fixed word list to the
same vector, and that answer was worthless -- 0.08 to 0.12 whatever the crop
held. Asked the other way round, against a phrase somebody actually typed, the
identical vectors separate present from absent almost perfectly. The question is
what was wrong, not the model.

**The hard part is not the ranking, it is saying "nothing matches".** A list of
scores always has a top, so a rover that answers "the spray bottle is over there"
when there is no spray bottle in the building is the failure to design against.

This was first built on the argument that the raw cosines are uncalibrated and
that the honest test is therefore relative: whether the best score stands clear
of the field. **Measured on the rover, that argument is wrong.** Forty queries
against its own stored regions -- twenty-four describing things it had seen, and
sixteen describing things that are not in the building -- separate almost
perfectly by raw score and not at all by separation:

    best score      present 0.065 to 0.140, absent 0.040 to 0.098
                    a cut at 0.09 gets 4 of the 40 wrong
    stands clear    present 1.58 to 4.45, absent 1.56 to 3.07
                    the best cut any threshold could make gets 14 of the 40 wrong

So the verdict is an absolute floor after all. The separation is still computed
and still reported, because it is a useful thing to look at when a search
surprises you, but it decides nothing.

The floor is a measurement and not a constant of nature: it was taken with the
full-precision TensorRT engines against thirty-one regions from a single room,
and vectors from the CPU backend agree with those only to 0.86, which is why
rows from another backend are counted out rather than scored.
"""
from __future__ import annotations

import math
import struct
from typing import Any

#: What a region has to score against the phrase before the rover will say it has
#: found it. Measured, not chosen: see the module docstring for the forty queries
#: it comes from. It sits above every one of the sixteen absent queries but one --
#: "a laptop computer", which found a television and is a near miss rather than an
#: invention -- and below all but three of the twenty-four present ones, those
#: three being small things the rover had seen exactly once.
#:
#: It errs towards saying nothing was found, which is the direction to err in.
MATCHES = 0.09
#: Not a threshold. Below this many stored vectors the spread of the field is not
#: worth reading, so the answer says how little has been seen rather than quoting
#: a separation computed from four numbers.
ENOUGH_TO_JUDGE = 12


def unpack(blob: bytes) -> tuple[float, ...]:
    return struct.unpack(f"<{len(blob) // 4}f", blob) if blob else ()


def cosine(left: tuple[float, ...], right: tuple[float, ...]) -> float:
    if not left or len(left) != len(right):
        return 0.0
    dot = sum(a * b for a, b in zip(left, right))
    na = math.sqrt(sum(a * a for a in left))
    nb = math.sqrt(sum(b * b for b in right))
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    return dot / (na * nb)


def rank(query: bytes, rows: list[dict[str, Any]], limit: int = 10,
         backend: str = "") -> dict[str, Any]:
    """Score every stored vector against the query and say whether any of it means
    anything.

    Rows whose vectors came from a different backend are counted out rather than
    scored, because the GPU engines and the CPU graphs agree with full precision
    to 1.000 and 0.86 and comparing across them would rank noise.
    """
    wanted = unpack(query)
    if not wanted:
        return {"ok": False, "error": "the query has no vector", "matches": []}

    scored, skipped = [], 0
    for row in rows:
        if backend and row.get("vectors_from") and row["vectors_from"] != backend:
            skipped += 1
            continue
        stored = unpack(row.get("siglip_blob") or b"")
        if len(stored) != len(wanted):
            skipped += 1
            continue
        scored.append((cosine(wanted, stored), row))
    if not scored:
        return {"ok": True, "matches": [], "considered": 0, "skipped": skipped,
                "confident": False,
                "detail": ("nothing stored can be compared with this query"
                           if skipped else "nothing has been seen yet")}

    scored.sort(key=lambda pair: -pair[0])
    values = [value for value, _row in scored]
    middle = _median(values)
    spread = _spread(values, middle)
    best = values[0]
    stands = (best - middle) / spread if spread > 1e-9 else 0.0
    confident = best >= MATCHES

    matches = []
    for value, row in scored[:limit]:
        matches.append({
            "score": round(value, 4),
            "stands_clear": round((value - middle) / spread, 2)
            if spread > 1e-9 else 0.0,
            "observation_id": row.get("id"),
            "entity_id": row.get("entity_id"),
            "frame_id": row.get("frame_id"),
            # Which part of that frame this actually is. Without it a search
            # answers with a picture of a room and leaves the person to guess
            # which of the twelve things in it was the match.
            "bbox": row.get("bbox"),
            "observed_at": row.get("observed_at"),
            "bearing_deg": row.get("bearing_deg"),
            "map_session": row.get("map_session"),
        })
    return {
        "ok": True,
        "matches": matches,
        "considered": len(scored),
        "skipped": skipped,
        "confident": confident,
        # The bar itself, so a caller showing the ranked list beside the verdict
        # can mark which of those rows actually cleared it. Sent rather than
        # copied into the console, because a second copy of a measured constant
        # is a copy that can be left behind when the measurement is retaken.
        "floor": MATCHES,
        # The numbers behind the verdict, because "is that really the spray
        # bottle" is the question a person will ask of this and the answer has to
        # be checkable without opening the database.
        "best": round(best, 4),
        "median": round(middle, 4),
        "spread": round(spread, 4),
        "stands_clear": round(stands, 2),
        "detail": _detail(confident, best, stands, len(values)),
    }


def _detail(confident: bool, best: float, stands: float, count: int) -> str:
    where = (f"and it stands {stands:.1f} spreads above the middle of the field"
             if count >= ENOUGH_TO_JUDGE else
             f"out of only {count} things seen so far")
    if confident:
        return (f"the best match scores {best:.3f} against that description, "
                f"above the {MATCHES:.2f} a real match takes, {where}")
    if count < ENOUGH_TO_JUDGE:
        return (f"the best of the {count} things seen so far scores only "
                f"{best:.3f} against that description, below the {MATCHES:.2f} a "
                f"real match takes -- though {count} is little enough that the "
                f"rover may simply not have looked at it yet")
    return (f"the best match scores only {best:.3f} against that description, "
            f"below the {MATCHES:.2f} a real match takes; nothing here matches "
            f"it, {where}")


def _median(values: list[float]) -> float:
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def _spread(values: list[float], middle: float) -> float:
    """Standard deviation about the median rather than the mean.

    About the median because the thing being measured is how far the *top* of the
    list sits above the body of it, and a mean that the top scores have already
    pulled upwards would hide exactly the separation this is looking for.
    """
    if len(values) < 2:
        return 0.0
    return math.sqrt(sum((value - middle) ** 2 for value in values)
                     / (len(values) - 1))
