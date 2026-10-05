"""Find me the thing I described.

A phrase is embedded by SigLIP2's text tower, whose image tower produced every
stored region vector, so the two land in the same space and the comparison is a
dot product. A thousand vectors is one matrix multiply, which is why there is no
vector database anywhere in this design and should not be one: the whole store
fits in memory several times over, scoring all of it costs about three
milliseconds, and an index would be another thing to keep true.

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

**Measured again on 2026-10-05, that floor had become far too low**, because the
best of many chance scores is higher than the best of a few: it was taken against
31 regions, and a search by then ranked the newest 2,000 looks. Sixty phrases for
things that cannot be in the flat and sixty for things that are, judged the way
`find_thing` judges them: at 0.09, 23 of the 60 absent ones were found and 1 present
one was missed. About thirty other rules were scored on the same looks --
separation from the field, a correction for looks that score high against any
phrase, a margin over background phrases, a household vocabulary as a veto,
several looks of one thing agreeing -- and none held out better than about one
wrong in eight. So the answer is still a raw floor, set higher, with one kind of
region counted out:

    a floor of 0.105, slivers counted out
                    absent found 6 of 60, present missed 8 of 60

**A sliver is a region the edge of the frame has cut down to a strip**, and
SigLIP, handed one enlarged to a square, will call it anything: a few pixels of
window frame matched "a traffic light". What still gets through is genuine
resemblance -- the rug as a canoe, a dining chair seen edge-on as a harp, the
black cabinet as a grand piano -- which no bar on a score can tell from a real
match. What the higher floor misses is small things seen rarely: the tissue box,
a sneaker, the spray bottle.

**The floor holds because the count is capped.** `Store.searchable` hands a
search the newest 2,000 looks and the floor is a measurement at that count;
ranking more would need it measured again.
"""
from __future__ import annotations

import math
import struct
from typing import Any

#: What a region has to score against the phrase before the rover will say it has
#: found it. Measured, not chosen: see the module docstring for the hundred and
#: twenty phrases it comes from, scored against the newest 2,000 looks. It was
#: 0.09 until 2026-10-05, measured against 31 regions, and by then let 23 of 60
#: absent phrases through.
#:
#: It errs towards saying nothing was found, which is the direction to err in.
MATCHES = 0.105
#: A region touching the edge of the frame and narrower than this -- 40 pixels of
#: a 640 x 480 frame, either way -- is a sliver, and is counted out rather than
#: ranked. Measured with the floor above; see the module docstring.
EDGE = 0.01
SLIVER_WIDTH = 40.0 / 640.0
SLIVER_HEIGHT = 40.0 / 480.0
#: Not a threshold. Below this many stored vectors the spread of the field is not
#: worth reading, so the answer says how little has been seen rather than quoting
#: a separation computed from four numbers.
ENOUGH_TO_JUDGE = 12


def unpack(blob: bytes) -> tuple[float, ...]:
    return struct.unpack(f"<{len(blob) // 4}f", blob) if blob else ()


def _numpy():
    """numpy, or None on a host without it.

    The scoring is one matrix multiply where numpy is present and the same
    arithmetic in a Python loop where it is not. It is worth the branch: at the
    thousand vectors the rover is now carrying the loop costs 0.29 s of the call
    and the multiply costs about three milliseconds, and the loop is still the
    thing that has to work on a machine that has only the standard library.
    """
    try:
        import numpy
    except ImportError:
        return None
    return numpy


def sliver(bbox: Any) -> bool:
    """Whether a region is a strip the edge of the frame has cut it down to.

    Both halves are needed. A small region in the middle of the frame is a small
    thing seen whole, and a narrow one touching the edge is a strip of something
    nobody can see the rest of; it is only the second that was measured matching
    whatever was asked for. A look with no box is not one.
    """
    try:
        left, top, right, bottom = (float(value) for value in bbox)
    except (TypeError, ValueError):
        return False
    at_edge = left < EDGE or top < EDGE or right > 1.0 - EDGE or bottom > 1.0 - EDGE
    return at_edge and (right - left < SLIVER_WIDTH or bottom - top < SLIVER_HEIGHT)


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
    to 1.000 and 0.86 and comparing across them would rank noise. Slivers are
    counted out too, separately, because they could be compared and were
    measured to match anything.
    """
    wanted = unpack(query)
    if not wanted:
        return {"ok": False, "error": "the query has no vector", "matches": []}

    scored, skipped, slivers = _scored(query, wanted, rows, backend)
    if not scored:
        return {"ok": True, "matches": [], "considered": 0, "skipped": skipped,
                "slivers": slivers, "confident": False,
                "detail": ("nothing stored can be compared with this query"
                           if skipped or slivers else "nothing has been seen yet")}

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
        "slivers": slivers,
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


def _scored(query: bytes, wanted: tuple[float, ...],
            rows: list[dict[str, Any]],
            backend: str) -> tuple[list, int, int]:
    """Every comparable row with its cosine, and counts of the rest.

    A row is out either because its vectors came from the other backend or
    because it does not have a vector of the same length, and both of those are
    counted rather than scored so the answer can say how much of the store it
    could not look at. A sliver is out for a different reason and counted apart.
    """
    keep, skipped, slivers = [], 0, 0
    width = len(query)
    for row in rows:
        if backend and row.get("vectors_from") and row["vectors_from"] != backend:
            skipped += 1
            continue
        blob = row.get("siglip_blob") or b""
        if len(blob) != width:
            skipped += 1
            continue
        if sliver(row.get("bbox")):
            slivers += 1
            continue
        keep.append(row)
    if not keep:
        return [], skipped, slivers

    np = _numpy()
    if np is None:
        return [(cosine(wanted, unpack(row["siglip_blob"])), row)
                for row in keep], skipped, slivers

    # One buffer out of the blobs and one multiply over the lot. Double
    # precision because the loop above is the reference for what a score means
    # and it accumulates in double: the two answers then agree to the last digit
    # either of them prints, which is what the test beside this checks. It costs
    # 7 MB of working memory at the size the store has reached, and the multiply
    # is three milliseconds either way.
    stored = np.frombuffer(
        b"".join(bytes(row["siglip_blob"]) for row in keep),
        dtype="<f4").reshape(len(keep), width // 4).astype(np.float64)
    asked = np.frombuffer(query, dtype="<f4").astype(np.float64)
    lengths = np.sqrt((stored * stored).sum(axis=1))
    asked_length = float(np.sqrt(asked @ asked))
    if asked_length < 1e-9:
        return [(0.0, row) for row in keep], skipped, slivers
    # A stored vector of no length scores nothing rather than dividing by nought,
    # which is what the loop this replaces did with it.
    safe = np.where(lengths < 1e-9, 1.0, lengths)
    values = np.where(lengths < 1e-9, 0.0,
                      (stored @ asked) / (safe * asked_length))
    return list(zip(values.tolist(), keep)), skipped, slivers


def _detail(confident: bool, best: float, stands: float, count: int) -> str:
    where = (f"and it stands {stands:.1f} spreads above the middle of the field"
             if count >= ENOUGH_TO_JUDGE else
             f"out of only {count} things seen so far")
    if confident:
        return (f"the best match scores {best:.3f} against that description, "
                f"above the {MATCHES:.3f} a real match takes, {where}")
    if count < ENOUGH_TO_JUDGE:
        return (f"the best of the {count} things seen so far scores only "
                f"{best:.3f} against that description, below the {MATCHES:.3f} a "
                f"real match takes -- though {count} is little enough that the "
                f"rover may simply not have looked at it yet")
    return (f"the best match scores only {best:.3f} against that description, "
            f"below the {MATCHES:.3f} a real match takes; nothing here matches "
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
