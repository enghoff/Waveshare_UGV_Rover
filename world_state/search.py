"""Find me the thing I described.

A phrase is embedded by the same text tower that embedded the vocabulary, so it
lands in the same space as every stored region vector and the comparison is a dot
product. A few hundred vectors is a few hundred multiplications, which is why
there is no vector database anywhere in this design and should not be one.

**The hard part is not the ranking, it is saying "nothing matches".** SigLIP's
raw cosines are uncalibrated and sit in a narrow band whatever is in the picture
-- measured on this rover, region-against-phrase scores fall between about 0.08
and 0.12 whether the phrase describes the region or not. An absolute floor drawn
across that band would be arbitrary, and a rover that answers "the spray bottle
is over there" when there is no spray bottle in the building is worse than one
that says it cannot find it.

So the test is relative and is about *separation*: the best match has to stand
clear of the field. A query that describes something in the room produces one or
two scores well above the rest; a query that describes nothing produces a flat
list where the top score is indistinguishable from the median. That is a
property of the shape of the answer rather than of its magnitude, and it does not
need a calibrated threshold to read.
"""
from __future__ import annotations

import math
import struct
from typing import Any

#: How far above the middle of the field the best score has to stand before it is
#: called a match, as a multiple of the spread of the field itself. Two and a half
#: standard deviations: a flat list of a few hundred scores throws up a top score
#: about two above the median by chance alone, so this asks for meaningfully more
#: than chance without demanding a separation that only a perfect query produces.
STANDS_CLEAR = 2.5
#: Below this many stored vectors the shape of the field means nothing and the
#: question cannot be answered honestly at all.
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
    confident = len(values) >= ENOUGH_TO_JUDGE and stands >= STANDS_CLEAR

    matches = []
    for value, row in scored[:limit]:
        matches.append({
            "score": round(value, 4),
            "stands_clear": round((value - middle) / spread, 2)
            if spread > 1e-9 else 0.0,
            "observation_id": row.get("id"),
            "entity_id": row.get("entity_id"),
            "label": row.get("label"),
            "frame_id": row.get("frame_id"),
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
        # The numbers behind the verdict, because "is that really the spray
        # bottle" is the question a person will ask of this and the answer has to
        # be checkable without opening the database.
        "best": round(best, 4),
        "median": round(middle, 4),
        "spread": round(spread, 4),
        "stands_clear": round(stands, 2),
        "detail": _detail(confident, stands, len(values)),
    }


def _detail(confident: bool, stands: float, count: int) -> str:
    if count < ENOUGH_TO_JUDGE:
        return (f"only {count} things have been seen, which is too few to say "
                f"whether the best of them means anything")
    if confident:
        return (f"the best match stands {stands:.1f} spreads above the middle of "
                f"the field, which is a real answer")
    return (f"the best match stands only {stands:.1f} spreads above the middle of "
            f"the field; nothing here matches that description")


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
