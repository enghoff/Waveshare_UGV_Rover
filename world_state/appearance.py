"""Normalized appearance vectors and exemplar comparison for identity resolution."""
from __future__ import annotations

import math

# Cleared at each resolver pass; blobs from different backends are never compared.
_UNIT: dict[bytes, tuple[float, ...]] = {}

def unit(blob: bytes) -> tuple[float, ...]:
    """A stored vector as floats scaled to unit length, or empty if it has none.

    Separate from `similarity` because the same blob is compared against many
    others in one pass, and unpacking 384 floats out of it for each of those
    comparisons was 96% of the resolver's running time -- measured on the rover
    at a 500-observation pool, where one pass took 14 s and 123,659 of those 14
    seconds' worth of arithmetic was this. Scaling here rather than dividing
    later means a comparison is one dot product.

    The cache is keyed on the bytes themselves, so it is shared by every caller
    within a pass and is bounded by how many distinct vectors exist. Cleared
    between passes by `resolve`, because a rover that ran for an hour would
    otherwise hold every vector it had ever compared.
    """
    got = _UNIT.get(blob)
    if got is None:
        import struct

        count = len(blob) // 4
        values = struct.unpack(f"<{count}f", blob) if count else ()
        length = math.sqrt(sum(value * value for value in values))
        got = tuple(value / length for value in values) if length > 1e-9 else ()
        _UNIT[blob] = got
    return got

def similarity(left: bytes, right: bytes) -> float:
    """Cosine between two float32 vectors, written out rather than in numpy.

    **Not because the daemon may not import one.** It said so here and that was
    already wrong: `python3-numpy` and `python3-scipy` are apt packages on this
    rover, in `/usr/lib/python3/dist-packages`, so they are neither vendored nor
    liable to be missing and a source deploy cannot remove them. The rule this
    docstring used to state was written for the Pi and has no force now.

    What is left is a measurement rather than a prohibition, and it points the
    other way as the pool grows. Timed on the rover, one thread, over 500 of its
    own stored vectors: comparing every pair takes **5.5 s this way and 73 ms as
    one matrix multiply**, which is 76 times. Three of the resolver's shapes are
    workarounds for that cost -- geometry runs before appearance, `MAX_NEW_PER_PASS`
    is 2, and the pending pool is capped at the newest 500 -- and all three stop
    being necessary once the arithmetic moves. What holds the change back is that
    it is the working part of a working resolver, not the import.

    If it does move, pin `OMP_NUM_THREADS=1` with it. The lidar scan matcher is
    this rover's only odometer, and what a spinning thread pool costs it has been
    measured here before: turning off onnxruntime's was worth 3.3x.
    """
    if not left or not right or len(left) != len(right):
        return 0.0
    a, b = unit(left), unit(right)
    if not a or not b:
        return 0.0
    return sum(x * y for x, y in zip(a, b))

def appearance(store, entity_id: str, vector: bytes) -> float | None:
    """How much this crop looks like what the entity has typically shown.

    The middle of the exemplars rather than the best of them, and **that is the
    fix for a gate that got looser every time it was wrong**. Scoring against the
    best is a ratchet: every crop that joins an entity becomes an exemplar, so an
    entity that has swallowed something it should not have will accept the next
    thing more readily for it. Measured on the run of 2026-09-03, an entity
    holding one exemplar admitted 10% of the pending pool at `DIFFERENT_THING`
    and the same entity holding twenty-one admitted 64%, monotonically the whole
    way. The exemplar window slides as well, so the crops the thing was founded
    on had been evicted and what it was being compared against was the last five
    things it took by mistake.

    The middle rather than the average for the reason the exemplars are kept
    apart at all: a chair seen from the front and the same chair from the side
    average to a picture of neither, and a single odd exemplar should move the
    answer no further than one place along.

    **None means the question could not be asked**, and that is not the same as a
    low score: an observation whose look produced no appearance vector, or an
    entity that has never stored an exemplar, has said nothing about whether it
    is the same thing. Since appearance is now the only gate left that can throw
    out a candidate on what it looks like, reporting silence as 0.0 would reject
    every candidate on a rover whose vectors had not arrived.
    """
    if not vector:
        return None
    seen = sorted(similarity(exemplar, vector)
                  for exemplar in store.exemplars(entity_id, width=len(vector)))
    if not seen:
        return None
    return seen[len(seen) // 2]
