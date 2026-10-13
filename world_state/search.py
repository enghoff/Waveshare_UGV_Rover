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

**What the voice model asks is ranked by thing, not by look** (2026-10-13). A
search over the newest 2,000 looks answers "which recent picture is most like
this", and `find_thing` wants "which thing". By 10-13 the two had come apart: the
store held 41,554 looks, the newest 2,000 reached only 198 of its 686 things, and
two thirds of them belonged to no thing at all -- 873 had been taken that morning
on a map the rover had not yet confirmed, which records the picture and nothing
else. So a phrase whose best looks were all unclaimed was "not seen" while
quoting a score above the bar, and the floor lamp and the sneakers, both in the
store, could not be found. `ThingIndex` scores each thing by the average of all
its looks instead -- one vector per thing, normalised looks summed and
normalised again -- and `rank_things` ranks those. Judged by eye on 65 phrases
for things in the flat, it named the wrong thing for 8 of them where the look
search named the wrong one for 17 and missed 8 more; the 8 are things the store
barely holds (a keyboard, a tissue box, a spray bottle, a mirror, a window). Its
bar is `THING_MATCHES`, a measurement of its own: see that constant.
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
#: What a *thing's* average look has to score against the phrase before the rover
#: will say it has found it. Not `MATCHES`: an average of many looks scores lower
#: against anything than its best look does, and there are a few hundred things
#: rather than two thousand looks, so the best chance score is lower too. Chosen on
#: bench_search's first phrase set over 683 things on 2026-10-13 -- 2 of 30 absent
#: phrases found and 2 of 34 present ones missed anywhere from 0.0975 to 0.105,
#: and the upper half of that taken, to err towards not found -- then checked on
#: the second set: 1 of 30 absent found, 2 of 26 present missed, and both of
#: those were things it named wrongly when it found them.
#:
#: Like `MATCHES` it depends on how many things are ranked; a store that grows
#: well past a thousand things wants it measured again with `bench_search.py`.
THING_MATCHES = 0.10
#: How many things may have changed before `ThingIndex` reads every look in one
#: pass rather than asking for each thing's looks on their own.
REBUILD_ALL = 64


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


class ThingIndex:
    """Every thing's average look, kept between searches and brought up to date
    at the start of each one.

    The average is over all of a thing's looks rather than its newest few,
    because which looks are newest says where the rover happened to be parked,
    not what the thing looks like. Building every one reads every look the store
    has -- 31,000 of them, 1.3 s on the rover -- so it is done once and then
    only for the things `Store.thing_prints` says have changed, which between
    two questions is a handful.

    Kept per backend as well as per thing (R-WS-4): the GPU and CPU vectors are
    not comparable, so a thing seen by both has two averages and a search ranks
    only those from the backend that took the newest look, which is the one the
    sidecar is running and so the one that embedded the phrase.
    """

    def __init__(self, store) -> None:
        self.store = store
        self.generation: str | None = None
        self.prints: dict[tuple[str, str], tuple[int, int, int]] = {}
        self.sums: dict[tuple[str, str], tuple[list | Any, int]] = {}
        self.backend = ""

    def refresh(self) -> int:
        """Bring every average up to date, and say how many were built again."""
        generation = self.store.generation()
        if generation != self.generation:
            # A cleared store issues its identifiers again from the start, so
            # nothing kept under the old ones can be trusted to mean the same.
            self.generation, self.prints, self.sums = generation, {}, {}
        prints = self.store.thing_prints()
        stale = {key for key, value in prints.items() if self.prints.get(key) != value}
        for gone in set(self.sums) - set(prints):
            del self.sums[gone]
        if stale:
            things = {entity_id for entity_id, _backend in stale}
            fresh = _summed(self.store.thing_looks(
                None if len(things) > REBUILD_ALL else sorted(things)))
            for key in [key for key in self.sums if key[0] in things]:
                del self.sums[key]
            self.sums.update({key: value for key, value in fresh.items()
                              if key[0] in things})
        self.prints = prints
        if prints:
            self.backend = max(prints.items(), key=lambda kv: kv[1][1])[0][1]
        return len(stale)

    def averages(self) -> list[tuple[str, int, list | Any]]:
        """(entity_id, looks, unit vector) for the backend being searched."""
        out = []
        for (entity_id, backend), (total, count) in self.sums.items():
            if backend != self.backend:
                continue
            length = math.sqrt(_dot(total, total))
            if length > 1e-9:
                out.append((entity_id, count, _scaled(total, 1.0 / length)))
        return out


def rank_things(query: bytes, index: ThingIndex, limit: int = 10) -> dict[str, Any]:
    """Score every thing's average look against the query, best first, and say
    whether the best of them means anything -- `rank`'s answer, about things."""
    wanted = unpack(query)
    if not wanted:
        return {"ok": False, "error": "the query has no vector", "matches": []}
    index.refresh()
    things = [one for one in index.averages() if len(one[2]) == len(wanted)]
    if not things:
        return {"ok": True, "matches": [], "considered": 0, "confident": False,
                "floor": THING_MATCHES,
                "detail": "the rover has not made a thing of anything it has seen yet"}
    np = _numpy()
    asked = (np.asarray(wanted, dtype=np.float64) if np is not None
             else list(wanted))
    length = math.sqrt(_dot(asked, asked))
    if length > 1e-9:
        asked = _scaled(asked, 1.0 / length)
    scored = sorted(((_dot(vector, asked), entity_id, count)
                     for entity_id, count, vector in things),
                    key=lambda one: -one[0])
    values = [value for value, _entity, _count in scored]
    middle = _median(values)
    spread = _spread(values, middle)
    best = values[0]
    stands = (best - middle) / spread if spread > 1e-9 else 0.0
    confident = best >= THING_MATCHES
    return {
        "ok": True,
        "matches": [{"entity_id": entity_id, "score": round(value, 4), "looks": count,
                     "stands_clear": round((value - middle) / spread, 2)
                     if spread > 1e-9 else 0.0}
                    for value, entity_id, count in scored[:limit]],
        "considered": len(scored),
        # Whether there are enough things for "none of them is it" to mean
        # anything, rather than "the rover has barely looked yet".
        "enough": len(scored) >= ENOUGH_TO_JUDGE,
        "confident": confident,
        "floor": THING_MATCHES,
        "best": round(best, 4),
        "median": round(middle, 4),
        "spread": round(spread, 4),
        "stands_clear": round(stands, 2),
        "detail": _detail(confident, best, stands, len(values), THING_MATCHES),
    }


def _summed(looks) -> dict[tuple[str, str], tuple[list | Any, int]]:
    """Each thing's looks as unit vectors added up, per backend, and how many.

    Slivers are left out, for the reason `rank` counts them out: they were
    measured matching whatever was asked, and an average is not immune to that.
    """
    np = _numpy()
    sums: dict[tuple[str, str], tuple[list | Any, int]] = {}
    for entity_id, backend, blob, box in looks:
        if sliver(box) or not blob:
            continue
        if np is not None:
            vector = np.frombuffer(bytes(blob), dtype="<f4").astype(np.float64)
            length = float(np.sqrt(vector @ vector))
        else:
            vector = list(unpack(bytes(blob)))
            length = math.sqrt(_dot(vector, vector))
        if length < 1e-9:
            continue
        vector = vector / length if np is not None else _scaled(vector, 1.0 / length)
        key = (entity_id, backend)
        if key in sums and len(sums[key][0]) == len(vector):
            total, count = sums[key]
            sums[key] = (total + vector if np is not None else
                         [a + b for a, b in zip(total, vector)], count + 1)
        elif key not in sums:
            sums[key] = (vector, 1)
    return sums


def _dot(left, right) -> float:
    try:
        return float(left @ right)
    except TypeError:
        return sum(a * b for a, b in zip(left, right))


def _scaled(vector, factor: float):
    try:
        return vector * factor
    except TypeError:
        return [value * factor for value in vector]


def _detail(confident: bool, best: float, stands: float, count: int,
            floor: float = MATCHES) -> str:
    where = (f"and it stands {stands:.1f} spreads above the middle of the field"
             if count >= ENOUGH_TO_JUDGE else
             f"out of only {count} things seen so far")
    if confident:
        return (f"the best match scores {best:.3f} against that description, "
                f"above the {floor:.3f} a real match takes, {where}")
    if count < ENOUGH_TO_JUDGE:
        return (f"the best of the {count} things seen so far scores only "
                f"{best:.3f} against that description, below the {floor:.3f} a "
                f"real match takes -- though {count} is little enough that the "
                f"rover may simply not have looked at it yet")
    return (f"the best match scores only {best:.3f} against that description, "
            f"below the {floor:.3f} a real match takes; nothing here matches "
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
