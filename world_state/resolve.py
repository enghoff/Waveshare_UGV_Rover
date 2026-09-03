"""Which lasting thing an observation was of, decided from where it is.

This is the part the proof-of-concept failed at, and the reason it failed is
worth keeping in front of the reader: asking a vision-language model "is this
the sofa you saw before?" fails in both directions -- Cosmos Reason 2 never
recognises anything and Cosmos 3 recognises things that are not in the room --
and **no model can do better from one picture**, because two identical chairs at
opposite ends of a room are identical in the picture. What separates them is
where they are, which the rover already measures.

So identity here is a geometry problem with two supporting witnesses. The gates
run cheapest first and each one can only *remove* a candidate:

    1. map session   coordinates from a map that no longer exists mean nothing.
    2. spatial       the bearing has to point at where the thing already is.
                     **For furniture this is a hard gate**, and no amount of
                     appearance may overrule it -- that is the redundant-
                     furniture test the whole design exists to pass.
    3. appearance    DINOv2 against several stored exemplars, twice over: to
                     throw out a candidate that plainly is not the same object,
                     and then to choose between the ones geometry accepted.
    4. history       how recently and how often, as a tiebreak of last resort.

**There used to be a semantic gate in front of all of these** -- a chair is not a
ceiling light -- and it is gone with the word list that fed it. Nothing measures
what a region is called any more, because the nearest phrase in that list scored
between 0.08 and 0.12 whatever the crop held and put "a computer monitor" on a
sofa. What took its place is `DIFFERENT_THING`, which asks the same question of
the appearance vector and answers it from two numbers this rover measured rather
than from a hand-written list of synonyms.

Three outcomes, and the third is not optional:

    MATCH       exactly one candidate survives every gate
    NEW         none does, and two bearings cross well enough to place a thing
    AMBIGUOUS   more than one survives, or nothing crosses well enough yet

**AMBIGUOUS is a real answer and the pool is not a queue that must be drained.**
An observation with one bearing and no partner stays where it is indefinitely;
a thing seen once from one place is a thing the rover cannot honestly claim to
have located, and inventing a position for it is exactly the failure this
replaced.

Nothing here is a probability. The pieces stay separate and inspectable, because
"appearance 0.98 but 4.2 m away" and "appearance 0.97 and 0.16 m away" is a
sentence a person can check, and a fused score of 0.61 is not.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from . import cluster, locate

MATCH = "match"
NEW = "new"
AMBIGUOUS = "ambiguous"

#: Below this, two crops are not two views of one object, and the candidate is
#: removed. **A removal-only gate and never a confirmation**, which is the role
#: the word list's synonym families used to have and the reason this number is
#: nowhere near a matching threshold. It sits between the two things this rover
#: measured with DINOv2: a chair against a spray bottle scores 0.122, and the
#: same chair across a real change of viewpoint -- the worst honest case -- still
#: scores 0.696. Anything in between only ever removes what is plainly unrelated,
#: and geometry decides everything else.
#:
#: It applies only where there is something to compare. An entity with no stored
#: exemplar, or an observation whose look produced no appearance vector, is
#: passed through to the spatial gate rather than rejected for silence.
#:
#: **Both of those numbers were measured on the CPU int8 graphs and neither
#: describes what the rover now produces, so read this as a floor that removes
#: very little rather than as a calibrated gate.** Re-measured on the TensorRT
#: engines over both drives of 2026-09-03 -- 3,741 pairs of regions taken from one
#: frame, which are different objects by construction -- the noise band is stable
#: between runs and tighter than the 157-crop sample it used to be quoted from:
#:
#:     morning    1,138 pairs   median 0.349   p90 0.623   p95 0.715
#:     afternoon  2,603 pairs   median 0.336   p90 0.594   p95 0.677
#:
#: **0.5 let a fifth of those through, and it showed.** With the pairing pass no
#: longer stopping at the first standoff, the afternoon run placed five things of
#: which three were plainly two objects each -- a door with a blown-out ceiling
#: panel at 0.542, a framed picture with a bright doorway at 0.525, a landscape
#: with a doorway at 0.538. At 0.55 all three go and both of the run's genuine
#: entities stay, while the morning run replays unchanged at four entities with
#: nothing mixed. It removes about a third of the pairs 0.5 admitted: 13-15% of
#: known-different pairs still pass.
#:
#: **It is a better place for a floor and it is still not a separation, and the
#: margin is thin enough to say out loud.** The chair the afternoon run genuinely
#: saw twice scores 0.648; the pair of looks that founds its framed picture scores
#: **0.557**, seven thousandths above this line. Geometry carries identity here.
#: **Do not try to fix identity by moving this number** -- what it can do is stop
#: the plainly unrelated founding a thing, and that is all this move claims.
DIFFERENT_THING = 0.55

#: How alike two DINOv2 vectors have to be before appearance is allowed to break
#: a tie. **Deliberately not a matching threshold.** Measured on this rover: the
#: same chair across a real change of viewpoint scores 0.696, and the *twin*
#: chair across the room scores 0.735 -- higher. Appearance answers "does this
#: look like that picture", not "is this the same object", so it is used only to
#: choose between candidates the geometry has already accepted, and only when
#: one is clearly ahead.
APPEARANCE_LEAD = 0.05

#: How much worse a rival crossing may be and still count as a rival. A pair of
#: rays that locates something to within half a metre is not thrown into doubt by
#: another pair that locates it to within a metre and a half: the second is a
#: poorer view of possibly the same thing, not a competing answer. Only crossings
#: of comparable quality can make each other ambiguous.
RIVAL_FACTOR = 2.0
#: How far apart two answers have to be before they are answers to different
#: questions, in metres. This is a statement about rooms rather than a
#: measurement: the resolver's question is which *thing* this is, and two
#: positions a handspan apart name the same chair whichever of them is right, so
#: refusing to place anything because they disagree by that much helps nobody.
#:
#: **It exists because the alternative gets worse as the rover gets better.** The
#: rival test asks whether two crossings are further apart than their own
#: uncertainty allows, and when bearings improved from five degrees to one and a
#: half those uncertainties shrank with them -- so a pair of crossings thirty
#: centimetres apart, which had comfortably overlapped, became a standoff and the
#: resolver stopped placing chairs it had been placing. Accuracy should not cost
#: the rover answers.
SAME_PLACE_M = 0.5

#: How much better one arrangement of a look's regions has to be than the next
#: before the difference is a decision rather than a rounding, as a fraction of
#: one bearing's whole allowance.
#:
#: **The geometric twin of `APPEARANCE_LEAD`, and it is asked of the look rather
#: than of the region.** Two regions of one picture and two things placed a
#: handspan apart have two arrangements between them, and where the total miss is
#: the same either way the geometry has said nothing about which region is which
#: -- so appearance is asked, and if that cannot separate them either, neither is
#: assigned. That is the case `object:12` and `object:15` of the drive of
#: 2026-09-03 are: over the four looks that saw both, the arrangement the rover
#: chose explained 2.12 m of miss where swapping the two regions explained 0.76,
#: so it had committed, look by look, to whichever it happened to consider first.
SAME_ANSWER = 0.05

#: How many new things one pairing pass may place before it stops and leaves the
#: rest for the next one.
#:
#: **A cost bound, and it became necessary the moment looks taken on the move
#: kept their bearings.** Searching for a crossing compares every pair and then
#: asks every ray whether it agrees with each survivor, which is one second at a
#: pool of 500 on the Orin -- and the search runs again after each placement,
#: because placing a thing takes rays out of the pool and changes what the rest
#: support. Measured there with a 500-ray pool, a pass that placed 107 things took
#: 55 seconds, against a `SETTLE_EVERY_S` of 10. The old build never met that
#: because it had a hundred bearings to work with, and stopped at the first
#: standoff besides.
#:
#: The cost is linear in this at a given pool size -- measured on the Orin over a
#: 500-ray pool, one placement is 2.7 s, two 5.3, three 7.7 and four 10.0. Two
#: keeps the worst pass comfortably inside `SETTLE_EVERY_S`, and two every ten
#: seconds is twelve a minute against the four things the rover's best drive so
#: far has placed in thirteen minutes.
#:
#: **Only new things are capped.** Joining an observation to something already
#: placed is the cheap path and runs over the whole pool every pass, which is the
#: right priority: an entity the rover already knows about should collect its
#: evidence immediately, and only inventing one is worth rationing.
#:
#: Nothing is lost by stopping: the pool persists, the next pass carries on from
#: it, and identity was never a queue that had to be drained in one go. What is
#: gained is that settling can never eat the looking.
MAX_NEW_PER_PASS = 2

@dataclass
class Decision:
    """What was decided about one observation, and why, in words.

    `why` is written for the popup rather than for a log: the question a person
    asks of this system is "why did it think that was the same chair", and the
    answer has to be readable without opening the database.
    """

    observation_id: int
    outcome: str
    entity_id: str | None = None
    why: str = ""
    candidates: list[dict[str, Any]] = field(default_factory=list)

    def line(self) -> str:
        return f"observation {self.observation_id}: {self.outcome} -- {self.why}"


def ray_of(observation: dict[str, Any],
           reach=None) -> dict[str, Any] | None:
    """The bearing already stored on an observation, as `locate` wants it.

    Recomputed from nothing: the bearing was worked out when the look was taken,
    from the field of view the camera had at that moment, and it is a
    measurement rather than a derivation.

    `reach` is the exception, and it is deliberately *not* a measurement of the
    observation. It answers "how far could the rover see in that direction",
    which is a question about the map, and the map grows as the rover explores --
    so a bearing that could not be bounded on one pass can be bounded on the
    next. It is asked here, once per ray, because the alternative is asking it
    inside the pair loops, which is the same answer computed a few hundred times.
    See `locate.beyond_reach` for what it stops.
    """
    pose = observation.get("pose")
    bearing = observation.get("bearing_deg")
    if not isinstance(pose, dict) or bearing is None:
        return None
    try:
        built = {"x_m": float(pose["x_m"]), "y_m": float(pose["y_m"]),
                 "bearing_deg": float(bearing),
                 "span_deg": float(observation.get("span_deg") or 0.0),
                 # How far out this ray's own starting point is, which the
                 # inspection measured and `locate` charges to every answer the
                 # ray takes part in. Absent on a row written before the rover
                 # measured it, and absent means nothing was moving.
                 "origin_sigma_m": float(observation.get("origin_sigma_m") or 0.0),
                 # And how well the bearing itself is known, which is not the
                 # same question: the origin says where the ray started, this
                 # says which way it pointed. A look taken while the rover was
                 # turning is worth less than one taken standing still, and
                 # since the frame carries its own timestamp the difference is
                 # measured rather than assumed. Absent means the constant --
                 # see `locate.sigma_of`.
                 "bearing_sigma_deg": observation.get("bearing_sigma_deg"),
                 "observation_id": observation.get("id"),
                 # Which look this ray came out of. Two rays from one look are
                 # two regions of one picture taken from one place, so they are
                 # one viewpoint however much they agree -- which is the
                 # difference between a thing seen from all round and a thing
                 # photographed twice from the doorway.
                 "inference_id": observation.get("inference_id")}
    except (KeyError, TypeError, ValueError):
        return None
    if reach is not None:
        try:
            far = reach(built["x_m"], built["y_m"], built["bearing_deg"])
        except Exception:
            # A map that cannot be read leaves the bearing unbounded, which is
            # what this did before there was a map to ask. It must never turn an
            # inspection into a failure.
            far = None
        if far is not None:
            built["reach_m"] = float(far)
    return built


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


#: Unit vectors by the blob they came from, for the length of one resolve pass.
#: Shared and unlocked, which is safe for the reason it is keyed on the bytes: an
#: entry is a pure function of its key, so a reader that finds one left behind by
#: a pass that has not cleared yet gets the right answer. The daemon serialises
#: its passes behind the inspector's lock in any case.
_UNIT: dict[bytes, tuple[float, ...]] = {}


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


def resolve(store, *, map_session: int | None = None,
            limit: int = 500, reach=None) -> dict[str, Any]:
    """One pass over the pending pool. Decides, records, and explains.

    Two passes internally, and the order is the whole algorithm. First every
    pending observation is offered to the things already placed, because joining
    a known thing is cheaper and safer than inventing one. Only what is left over
    is considered for pairing into something new, and only where two bearings
    genuinely cross.

    `reach(x_m, y_m, bearing_deg) -> metres | None` is how far the rover could
    see from there in that direction, which only the owner of the occupancy grid
    can answer -- so it arrives as a callable rather than being reached for, the
    way the camera and the pose do. **It is the strongest gate in here**, and
    what it stops is written up in `locate.beyond_reach`. Without it every
    bearing is a ray of unbounded length, which is what this was before, and two
    of them pointed at two different things in two different rooms cross in a
    third room.
    """
    session = store.map_session() if map_session is None else int(map_session)
    pending = store.unplaced(map_session=session, limit=limit)
    entities = store.placed(map_session=session)
    decisions: list[Decision] = []
    # One pass's worth of unpacked vectors and no more. Held for the pass because
    # every vector in the pool is compared against many others; dropped after it
    # because the pool is different next time and a daemon that ran all day would
    # otherwise keep every vector it had ever seen.
    _UNIT.clear()

    # Which entities each frame has already accounted for. Two regions in one
    # frame are two different things -- the region finder's own suppression saw
    # to that -- so once a frame has matched an entity, its other regions may
    # not match the same one however well they line up.
    #
    # **A cache of what the store says, and not the record itself.** This
    # dictionary lives for one pass and the pending pool lives indefinitely, so
    # when it was the record a frame gave an entity one more region every pass;
    # see `WorldStore.entities_in_frame` for what that cost on 2026-09-03.
    taken_in: dict[Any, set] = {}
    # Grouped by the look that took them, because a look is the unit the first
    # pass decides: its regions are two different things by construction, so
    # which of them is which thing is one arrangement rather than several
    # independent choices. Order is preserved -- oldest look first, which is the
    # order the pool came in.
    looks: dict[Any, list] = {}
    for observation in pending:
        looks.setdefault(observation.get("inference_id"), []).append(observation)
    leftover = []
    for group in looks.values():
        settled = _by_look(store, group, entities, session, taken_in, reach)
        spoken_for = {decision.observation_id for decision in settled}
        decisions.extend(settled)
        leftover.extend(one for one in group if one["id"] not in spoken_for)

    decisions.extend(DISCOVERY(store, leftover, session, entities, taken_in,
                               reach))

    _UNIT.clear()
    counted = {MATCH: 0, NEW: 0, AMBIGUOUS: 0}
    for decision in decisions:
        counted[decision.outcome] = counted.get(decision.outcome, 0) + 1
    return {
        "map_session": session,
        "considered": len(pending),
        "matched": counted[MATCH],
        "created": counted[NEW],
        "ambiguous": counted[AMBIGUOUS],
        "still_waiting": len(pending) - len(decisions),
        "decisions": [
            {"observation_id": one.observation_id, "outcome": one.outcome,
             "entity_id": one.entity_id, "why": one.why,
             "candidates": one.candidates}
            for one in decisions],
    }


def _solver():
    """`scipy.optimize.linear_sum_assignment`, or None if it cannot be reached.

    **The daemon may import a third-party package**, which was not true of the
    Pi this component was first written on and is the only reason the arithmetic
    here was ever hand-rolled: `python3-numpy` and `python3-scipy` are apt
    packages on this rover, in `/usr/lib/python3/dist-packages`, where no source
    deploy can touch them.

    None is answered rather than raised, and the caller then decides the look one
    region at a time in pool order, which is what this did for its first month.
    That is a real behaviour with its own tests rather than a second
    implementation of an assignment, and the decision says which way it was
    taken -- a rover that quietly stopped placing things because an import moved
    would be a worse failure than a rover that goes back to being greedy.
    """
    global _SOLVER
    if _SOLVER is None:
        try:
            from scipy.optimize import linear_sum_assignment  # noqa: PLC0415

            _SOLVER = linear_sum_assignment
        except Exception:                                     # noqa: BLE001
            _SOLVER = False
    return _SOLVER or None


_SOLVER: Any = None

#: What a forbidden pairing costs. Anything above the whole of one bearing's
#: allowance would do; this is far above it, so that the solver prefers every
#: feasible pairing it can make to any forbidden one and therefore arranges as
#: many regions as the gates allow before it minimises the miss.
_FORBIDDEN = 1e6


def _allowance_used(placement: dict[str, Any],
                    ray: dict[str, Any]) -> float | None:
    """How much of what this bearing is allowed to be off by it actually uses,
    or None if it is not pointing at this thing at all.

    A ratio rather than metres, because the costs of different pairings have to
    be comparable and metres are not: half a metre of miss is nothing on a
    sideboard five metres away and hopeless on a light fitting one metre off. The
    denominator is `locate.match_tolerance`, which is the same number
    `_against_known` has always compared against, so a ratio of 1.0 is exactly
    the edge of what would attach -- this reorders candidates the gate already
    admitted and never admits one it did not.
    """
    tolerance_m = locate.match_tolerance(placement, ray)
    if not locate.agrees(placement, ray, tolerance_m) or tolerance_m <= 0.0:
        return None
    miss_m = locate.cross_track_of(float(placement["x_m"]),
                                   float(placement["y_m"]), ray)
    return miss_m / tolerance_m


def _arrange(costs: list[list[float]], solve) -> tuple[list[int], float]:
    """Which entity each region should go to, and what the whole thing costs.

    A list as long as the regions, holding an index into the entities or -1, and
    the total of the pairings that were actually made. `solve` is
    `linear_sum_assignment`: it takes the rectangular matrix whole and returns
    the arrangement with the smallest total, which is the point of asking it
    rather than taking each region's own best in turn.
    """
    import numpy as np                                         # noqa: PLC0415

    matrix = np.array(costs, dtype="float64")
    rows, columns = solve(matrix)
    chosen = [-1] * len(costs)
    total = 0.0
    for row, column in zip(rows.tolist(), columns.tolist()):
        if matrix[row][column] >= _FORBIDDEN:
            continue
        chosen[row] = column
        total += float(matrix[row][column])
    return chosen, total


def _by_look(store, group, entities, session, taken_in,
             reach=None) -> list[Decision]:
    """Offer one look's regions to the things already placed, all at once.

    **The unit of decision is the look and not the region, and that is the fix
    for two adjacent objects being cut down the wrong seam.** Two regions of one
    picture are two different things -- the region finder's own suppression saw
    to that -- so a look may give an entity one region and no more. That rule was
    always here and was enforced first-come: whichever region was considered
    first claimed the entity, the second was pushed out to the pairing pass, and
    a twin was founded. It is a constraint on an arrangement, so it is solved as
    one now.

    What that buys is measured. On the drive of 2026-09-03, `object:12` and
    `object:15` sat 0.41 m apart, each holding some crops of a blue-topped bench
    and some of the dark cabinet beside it, with which entity got which flipping
    from look to look; all four of the later entity's looks would have joined the
    earlier one on geometry and appearance both, and all four were refused
    because their frame had already given it a region.

    Placements are read as they stood when the look began and rewritten once
    afterwards, rather than moving under the regions still being decided.
    """
    solve = _solver()
    rays: list[tuple[dict[str, Any], Any]] = []
    for observation in group:
        ray = ray_of(observation, reach)
        if ray is not None:
            rays.append((ray, observation))
    if not rays:
        return []
    frame = group[0].get("inference_id")
    if frame not in taken_in:
        taken_in[frame] = store.entities_in_frame(frame)
    already = taken_in[frame]
    open_to = [entity for entity in entities if entity["id"] not in already]
    if not open_to or solve is None:
        return [decision for decision in
                (_against_known(store, observation, entities, session, taken_in,
                                reach) for _ray, observation in rays)
                if decision is not None]

    # What each region would cost each thing, and what it looks like. Both are
    # wanted for every admissible pair: the geometry arranges the look and
    # appearance is asked only where the arrangement turns out not to care.
    costs: list[list[float]] = []
    looks: list[list[float | None]] = []
    for ray, observation in rays:
        vector = observation.get("dino_blob") or b""
        row_costs, row_looks = [], []
        for entity in open_to:
            placement = entity.get("placement") or {}
            used = _allowance_used(placement, ray)
            seen = None if used is None else appearance(store, entity["id"],
                                                        vector)
            if used is None or (seen is not None and seen < DIFFERENT_THING):
                row_costs.append(_FORBIDDEN)
                row_looks.append(None)
                continue
            row_costs.append(used)
            row_looks.append(seen)
        costs.append(row_costs)
        looks.append(row_looks)

    chosen, total = _arrange(costs, solve)
    decisions: list[Decision] = []
    touched: list[str] = []
    for index, (ray, observation) in enumerate(rays):
        column = chosen[index]
        if column < 0:
            continue
        # Would the look be arranged as well with this region somewhere else? If
        # so the geometry has not chosen, and the old per-region question is
        # asked of the alternatives it is indifferent between.
        spare = [row[:] for row in costs]
        spare[index][column] = _FORBIDDEN
        _other, without = _arrange(spare, solve)
        rivals = [other for other in range(len(open_to))
                  if other != column and costs[index][other] < _FORBIDDEN]
        if rivals and without - total <= SAME_ANSWER:
            settled = _by_appearance(index, column, rivals, looks, open_to)
            if settled is None:
                decisions.append(Decision(
                    observation["id"], AMBIGUOUS, None,
                    why=(f"{len(rivals) + 1} placed things are equally "
                         f"consistent with this look however its {len(rays)} "
                         f"regions are shared out, and appearance cannot "
                         f"separate them ({_reads(looks[index][column])} against "
                         f"{_reads(max((looks[index][one] or 0.0) for one in rivals))}); "
                         f"left unassigned rather than guessed"),
                    candidates=_shortlist(index, [column, *rivals], costs,
                                          looks, open_to)))
                continue
            column = settled
        entity_id = open_to[column]["id"]
        already.add(entity_id)
        placement = open_to[column].get("placement") or {}
        away_m = math.hypot(float(placement.get("x_m", 0.0)) - ray["x_m"],
                            float(placement.get("y_m", 0.0)) - ray["y_m"])
        why = (f"the bearing points at {entity_id} {away_m:.2f} m away, "
               f"appearance {_reads(looks[index][column])}, and of the "
               f"{len(rays)} regions in this look it is the one that fits it "
               f"best, using {costs[index][column]:.0%} of what its bearing is "
               f"allowed to be off by")
        store.attach(entity_id, [observation["id"]], why)
        if observation.get("dino_blob"):
            store.add_exemplar(entity_id, observation["dino_blob"])
        if entity_id not in touched:
            touched.append(entity_id)
        decisions.append(Decision(observation["id"], MATCH, entity_id, why=why,
                                  candidates=_shortlist(index, [column], costs,
                                                        looks, open_to)))
    for entity_id in touched:
        _replace_placement(store, entity_id, session, reach)
    return decisions


def _by_appearance(index: int, column: int, rivals: list[int],
                   looks: list[list[float | None]],
                   open_to: list[dict[str, Any]]) -> int | None:
    """Which of the things the geometry is indifferent between this crop looks
    most like, or None if appearance cannot separate them either.

    The rule `_against_known` has always applied, asked at the point it is now
    reached: appearance chooses only among candidates geometry has accepted, and
    only when one is clearly ahead by `APPEARANCE_LEAD`. Silence sorts last, so a
    candidate nothing could be compared against never wins on nothing.
    """
    ranked = sorted([column, *rivals],
                    key=lambda one: (-(looks[index][one] or 0.0),
                                     -(open_to[one].get("observation_count") or 0)))
    best, next_best = ranked[0], ranked[1]
    lead = (looks[index][best] or 0.0) - (looks[index][next_best] or 0.0)
    return best if lead >= APPEARANCE_LEAD else None


def _reads(value: float | None) -> str:
    """An appearance score for a person to read, or the fact that there is none."""
    return "not comparable" if value is None else f"{float(value):.2f}"


def _shortlist(index: int, columns: list[int], costs, looks,
               open_to) -> list[dict[str, Any]]:
    """The candidates behind one decision, for the console to show."""
    return [{"entity_id": open_to[one]["id"],
             "allowance_used": round(costs[index][one], 3),
             "appearance": (None if looks[index][one] is None
                            else round(looks[index][one], 3)),
             "seen": open_to[one].get("observation_count", 0)}
            for one in columns if costs[index][one] < _FORBIDDEN]


def _against_known(store, observation, entities, session,
                   taken_in, reach=None) -> Decision | None:
    """Offer one observation to the things already placed.

    None means "no candidate survived the gates", which is not a decision: the
    observation goes on to the pairing pass, where it may help place something
    new. A decision means it was matched, or that it was ambiguous and is being
    left alone deliberately.
    """
    ray = ray_of(observation, reach)
    if ray is None:
        return None
    vector = observation.get("dino_blob") or b""

    frame = observation.get("inference_id")
    if frame not in taken_in:
        taken_in[frame] = store.entities_in_frame(frame)
    already = taken_in[frame]
    surviving = []
    for entity in entities:
        if entity["id"] in already:
            continue
        placement = entity.get("placement") or {}
        # Wide enough to cover the thing itself, not just the bearing: see
        # `locate.match_tolerance`. Asking whether a bearing points at a
        # television is a different question from asking whether two bearings
        # converge, and the two want different tolerances.
        if not locate.agrees(placement, ray,
                             locate.match_tolerance(placement, ray)):
            continue
        looks = appearance(store, entity["id"], vector)
        # The only gate left that can rule a candidate out on what it is rather
        # than on where it is, and it removes only the plainly unrelated: this
        # rover measured a chair against a spray bottle at 0.122 and the same
        # chair across a change of viewpoint at 0.696.
        if looks is not None and looks < DIFFERENT_THING:
            continue
        surviving.append({
            "entity_id": entity["id"],
            "distance_m": round(math.hypot(
                float(placement.get("x_m", 0.0)) - ray["x_m"],
                float(placement.get("y_m", 0.0)) - ray["y_m"]), 2),
            "appearance": None if looks is None else round(looks, 3),
            "seen": entity.get("observation_count", 0),
        })

    if not surviving:
        return None
    if len(surviving) > 1:
        surviving.sort(key=lambda one: (-_looks(one), -one["seen"]))
        lead = _looks(surviving[0]) - _looks(surviving[1])
        if lead < APPEARANCE_LEAD:
            return Decision(
                observation["id"], AMBIGUOUS, None,
                why=(f"{len(surviving)} placed things are equally consistent with "
                     f"this bearing and appearance cannot separate them "
                     f"({_says(surviving[0])} against {_says(surviving[1])}); "
                     f"left unassigned rather than guessed"),
                candidates=surviving)
        # Appearance is allowed to choose only among candidates the geometry has
        # already accepted, and only when one is clearly ahead. It may never
        # bring a candidate back that the spatial gate rejected.

    chosen = surviving[0]
    already.add(chosen["entity_id"])
    why = (f"the bearing points at {chosen['entity_id']} "
           f"{chosen['distance_m']} m away, appearance {_says(chosen)}")
    store.attach(chosen["entity_id"], [observation["id"]], why)
    if vector:
        store.add_exemplar(chosen["entity_id"], vector)
    _replace_placement(store, chosen["entity_id"], session, reach)
    return Decision(observation["id"], MATCH, chosen["entity_id"], why=why,
                    candidates=surviving)


def _looks(candidate: dict[str, Any]) -> float:
    """A candidate's appearance score as a number to sort by.

    Silence sorts last rather than first. A candidate nothing could be compared
    against has not earned the lead, and treating it as 0.0 here only decides an
    ordering -- it never removes anything, which `DIFFERENT_THING` does and this
    must not.
    """
    value = candidate.get("appearance")
    return 0.0 if value is None else float(value)


def _says(candidate: dict[str, Any]) -> str:
    """The same number for a person to read, or the fact that there is none."""
    value = candidate.get("appearance")
    return "not comparable" if value is None else f"{float(value):.2f}"


def _pair_up(store, leftover, session, entities, taken_in,
             reach=None) -> list[Decision]:
    """Make new things out of pairs of bearings that actually cross.

    Every pair is tried. There used to be a cheap grouping by compatible name in
    front of this, and it went with the word list: what stops a ray at a chair
    pairing with a ray at a bottle now is that the two crops have to look like
    each other, which `DIFFERENT_THING` asks of the appearance vector directly.
    What comes out is the pair with the smallest uncertainty, because a
    least-squares fit over rays whose error is dominated by one bad box is worse
    than the best honest pair -- and because the popup has to be able to name the
    two looks that placed the thing.

    **A thing created here is a thing already placed for everything still
    waiting.** The list of known things is otherwise read once, before any of this
    runs, so without offering the remainder to each new thing as it appears, the
    rays that did not fit into the first television pair up into a second one.
    That is what the rover did on 2026-09-02: four televisions, two of them eight
    centimetres apart, and three people where there was one.
    """
    decisions: list[Decision] = []
    used: set[int] = set()
    while True:
        available = [one for one in leftover if one["id"] not in used]
        if len(available) < 2:
            break
        if sum(1 for one in decisions if one.outcome == NEW) >= MAX_NEW_PER_PASS:
            # Enough for one pass. See `MAX_NEW_PER_PASS`: the search behind each
            # placement is a second at a full pool, and the rest of the pool is
            # still there next time.
            break
        placed = _place_one(store, available, session, reach)
        if placed is None:
            break
        decision, taken = placed
        used.update(taken)
        decisions.append(decision)
        # A thing this pass has just made is a thing its founding frames have
        # already given a region to, and the rest of the pass has to know that
        # before it offers them another. Recorded here rather than re-read from
        # the store on every candidate, which is the same answer for the price
        # of one dictionary update.
        frame_of = {one["id"]: one.get("inference_id") for one in available}
        for observation_id in taken:
            frame = frame_of.get(observation_id)
            if frame in taken_in:
                taken_in[frame].add(decision.entity_id)
        # Re-read rather than appended to, so the new thing arrives in the
        # same shape as every other and carries the placement the store
        # actually holds.
        entities[:] = store.placed(map_session=session)
        for waiting in leftover:
            if waiting["id"] in used:
                continue
            joined = _against_known(store, waiting, entities, session,
                                    taken_in, reach)
            if joined is not None:
                decisions.append(joined)
                used.add(waiting["id"])
    return decisions


def _cluster_up(store, leftover, session, entities, taken_in,
                reach=None) -> list[Decision]:
    """Make new things by fitting all the leftover bearings at once.

    The same slot `_pair_up` fills and the same contract -- create what the
    evidence supports, then offer everything still waiting to whatever was
    created -- with the discovery itself handed to [cluster.py](cluster.py)
    instead of to a search over pairs. What changes is that a thing is placed
    from every ray that believes in it rather than from the best two, and that
    two rays which cross in two defensible ways no longer refuse each other.

    `MAX_NEW_PER_PASS` still applies. A pass that invents everything it can see
    leaves the look that follows it nothing to check, and the rest of the pool is
    still there next time.
    """
    rays = []
    blobs: dict[Any, bytes] = {}
    for observation in leftover:
        ray = ray_of(observation, reach)
        if ray is None:
            continue
        rays.append(ray)
        blobs[observation["id"]] = observation.get("dino_blob") or b""
    if len(rays) < 2:
        return []

    def looks_like(ray, others) -> bool:
        """Could this ray be the same thing as any of these? Removal only.

        The same question `_could_be_one` asks of a pair and `appearance` asks
        of an entity's exemplars, put to `cluster` as a veto. Best-of rather
        than all-of, because one object photographed from two sides scores 0.70
        against a good exemplar and much less against a bad one, and requiring
        every exemplar to agree would shrink an entity as it grew.
        """
        mine = blobs.get(ray.get("observation_id")) or b""
        if not mine:
            return True
        best = None
        for other in others:
            theirs = blobs.get(other.get("observation_id")) or b""
            if not theirs:
                continue
            got = similarity(mine, theirs)
            best = got if best is None else max(best, got)
        return best is None or best >= DIFFERENT_THING

    found = cluster.discover(rays, looks_like=looks_like,
                             limit=MAX_NEW_PER_PASS)
    decisions: list[Decision] = []
    used: set[int] = set()
    frame_of = {one["id"]: one.get("inference_id") for one in leftover}
    by_id = {one["id"]: one for one in leftover}

    for placement in found:
        members = [one["observation_id"] for one in placement["members"]
                   if one["observation_id"] not in used]
        if len(members) < 2:
            continue
        # A thing may take at most one region from any one picture. `cluster`
        # already enforces that inside a look, and it is asserted again here
        # because the pool spans many looks and the store's own record of what a
        # frame has accounted for has to agree with what is about to be written.
        claimed: set = set()
        kept = []
        for observation_id in members:
            frame = frame_of.get(observation_id)
            if frame in claimed:
                continue
            claimed.add(frame)
            kept.append(observation_id)
        if len(kept) < 2:
            continue
        entity_id = store.create_entity()
        store.place(entity_id, placement, session)
        why = (f"{placement['rays_agreeing']} bearings from "
               f"{placement['viewpoints']} places fitted {entity_id} to within "
               f"{placement['uncertainty_m']} m")
        store.attach(entity_id, kept, why)
        for observation_id in kept:
            vector = (by_id.get(observation_id) or {}).get("dino_blob") or b""
            if vector:
                store.add_exemplar(entity_id, vector)
            frame = frame_of.get(observation_id)
            if frame in taken_in:
                taken_in[frame].add(entity_id)
        used.update(kept)
        decisions.append(Decision(
            kept[0], NEW, entity_id, why=why,
            candidates=[{"entity_id": entity_id, "from_observations": kept,
                         "uncertainty_m": placement["uncertainty_m"]}]))

    if not decisions:
        return []
    # Everything still waiting is offered to what was just made, for the reason
    # `_pair_up` gives: the list of known things was read before any of this ran,
    # so without this the rays that did not make it into the first television
    # pair up into a second one next pass.
    entities[:] = store.placed(map_session=session)
    for waiting in leftover:
        if waiting["id"] in used:
            continue
        joined = _against_known(store, waiting, entities, session, taken_in,
                                reach)
        if joined is not None:
            decisions.append(joined)
            used.add(waiting["id"])
    return decisions


def _place_one(store, available, session, reach=None):
    """The best-supported crossing among these observations that nothing
    contradicts, or None.

    **Two bearings crossing is not enough on its own, and this is the phantom
    problem rather than a refinement.** Two identical chairs seen from two places
    produce four rays and *four* valid crossings: the two real chairs, and two
    phantoms where a ray to one chair happens to cross a ray to the other. All
    four are geometrically sound, and on this rover appearance cannot break the
    tie either -- the twin chair scored 0.735 against the same chair's 0.696
    across a change of viewpoint. From two viewpoints the answer is genuinely not
    knowable, and the honest outcome is to wait rather than to guess.

    What separates them is a third look. A real chair is agreed by every ray that
    was pointed at it; a phantom is agreed by exactly the two rays that made it.
    So a crossing is chosen by **how many rays support it**, and a crossing that
    ties with a conflicting one built from one of the same rays is passed over --
    see `_contested`, and note that it is passed over rather than ending the
    search, which is what it used to do.

    None means no crossing here survived that, and the whole group stays pending.
    That is the right answer for a rover that has looked at something from one
    place only.
    """
    rays = []
    for observation in available:
        ray = ray_of(observation, reach)
        if ray is not None:
            rays.append((ray, observation))
    if len(rays) < 2:
        return None

    found_fixes = []
    for index, (first, first_observation) in enumerate(rays):
        for second, second_observation in rays[index + 1:]:
            # Two regions in one frame are two different things: the region
            # finder's own suppression already made sure of that, so a pair from
            # one inspection can never be two looks at one object.
            if (first_observation.get("inference_id") is not None
                    and first_observation.get("inference_id")
                    == second_observation.get("inference_id")):
                continue
            # **Geometry first, and that is the module's own rule rather than a
            # preference**: the gates run cheapest first, and `fix` is a dozen
            # multiplications where the appearance gate below is a dot product
            # over 384 of them. Measured on the recording of 2026-09-03, 97 of
            # the 123 pairs that reach here have no usable crossing at all, so
            # asking what they look like first spent 96% of the resolver's time
            # on pairs that geometry was about to throw out anyway.
            crossing = locate.fix(first, second)
            if crossing is None:
                continue
            # Two crops that do not look like each other are not two looks at
            # one thing, however well their bearings cross. This is what stops a
            # ray at a chair pairing with a ray at a bottle now that nothing
            # names either of them, and it is the same removal-only gate
            # `_against_known` uses.
            if not _could_be_one(first_observation, second_observation):
                continue
            support = [observation for ray, observation in rays
                       if locate.agrees(crossing, ray)]
            # **Counted in viewpoints, not in rays.** Two regions of one frame
            # cannot both be the same object, so a crossing that a second region
            # of an already-counted frame happens to point near is not better
            # supported for it -- and that is exactly how a phantom wins:
            # measured here, a phantom at 0.67 m collected three rays from two
            # frames while the real chair at (3, 3) collected two, because close
            # to the rover every bearing agrees with everything.
            strength = len({one.get("inference_id") for one in support})
            found_fixes.append((crossing, first_observation, second_observation,
                                support, strength))
    if not found_fixes:
        return None

    found_fixes.sort(key=lambda one: (-one[4], one[0]["uncertainty_m"]))
    # **A contested crossing is one crossing being refused, not the end of the
    # group, and running the two together cost the run of 2026-09-03 most of what
    # it could have placed.** `_pair_up` stops the moment this answers None, so a
    # single standoff between two rays threw away every other crossing in the
    # pool -- 65 of that run's 181 pairing passes ended that way, with a median of
    # four crossings still on the table. What is genuinely unknowable is which of
    # two answers built from the same ray is right; the chair on the other side of
    # the room is not in doubt for it.
    chosen = None
    for index in range(len(found_fixes)):
        if not _contested(found_fixes, index):
            chosen = found_fixes[index]
            break
    if chosen is None:
        return None
    placement, first_observation, second_observation, support, strength = chosen
    # How much stands behind it, recorded with it. A thing founded on two looks
    # from two places and a thing agreed by eight are both "placed", and until
    # this travelled with the placement nothing downstream could tell them apart.
    placement = dict(placement, rays_agreeing=len(support),
                     viewpoints=locate.standing_places(
                         [ray for ray, one in rays
                          if one["id"] in {o["id"] for o in support}]))

    entity_id = store.create_entity()
    store.place(entity_id, placement, session)
    taken = [first_observation["id"], second_observation["id"]]
    why = (f"two looks {placement['baseline_m']} m apart crossed at "
           f"{placement['parallax_deg']} degrees, placing {entity_id} to "
           f"within {placement['uncertainty_m']} m")
    store.attach(entity_id, taken, why)
    for observation in (first_observation, second_observation):
        vector = observation.get("dino_blob") or b""
        if vector:
            store.add_exemplar(entity_id, vector)

    # Anything else in the group that also points at the new position joins it
    # now rather than waiting for the next pass -- except another region from a
    # frame that already contributed one, which is a different thing by
    # construction.
    claimed = {one.get("inference_id")
               for one in (first_observation, second_observation)}
    # **Not the same set that counted support.** Support asks whether rays
    # converge, and must stay tight or a phantom collects agreement from half the
    # room. This asks whether a ray points at the thing now placed, which is a
    # question about the thing's silhouette -- so it uses the wider tolerance,
    # and it is what stops the rays left over from making a second television.
    supporting = {observation["id"] for ray, observation in rays
                  if locate.agrees(placement, ray,
                                   locate.match_tolerance(placement, ray))}
    for _ray, observation in rays:
        if observation["id"] in taken or observation["id"] not in supporting:
            continue
        if observation.get("inference_id") in claimed:
            continue
        vector = observation.get("dino_blob") or b""
        # **The same appearance gate every other way in is behind, and it was
        # missing here.** This is the one path that attached a crop on geometry
        # alone: a thing has just been placed, everything else in the group that
        # points near it joins, and nothing asked whether any of them looked like
        # it. On the run of 2026-09-03 that is exactly what put a lit doorway
        # into an entity founded on a dark cabinet and a sofa, at 0.28 and 0.26
        # against its two exemplars, and the pole of a floor lamp into an entity
        # of framed pictures at 0.09 -- both far below what the founding pair
        # itself had to clear. The tolerance this loop uses is deliberately the
        # wide one, because it asks whether a bearing lands inside a thing's
        # silhouette; a wide gate on where it is wants the same gate on what it
        # looks like as everything else.
        looks = appearance(store, entity_id, vector)
        if looks is not None and looks < DIFFERENT_THING:
            continue
        claimed.add(observation.get("inference_id"))
        taken.append(observation["id"])
        store.attach(entity_id, [observation["id"]],
                     f"points at {entity_id} as well, from the same group"
                     + ("" if looks is None else f", appearance {looks:.2f}"))
        if vector:
            store.add_exemplar(entity_id, vector)

    return Decision(
        first_observation["id"], NEW, entity_id,
        why=why,
        candidates=[{"entity_id": entity_id,
                     "from_observations": taken,
                     "uncertainty_m": placement["uncertainty_m"]}]), taken


def _contested(found_fixes: list, index: int) -> bool:
    """Whether another crossing built from one of these same rays disagrees.

    **Only a crossing that shares a ray is a rival.** A ray points at one thing,
    so two comparably supported answers built from the same ray cannot both be
    right and nothing here can say which; a third look from somewhere else
    settles it. Two crossings built from entirely different rays are simply two
    different objects, and refusing those would mean a room could only ever hold
    one chair.

    Asked of every candidate in turn rather than only of the best one, which is
    what lets the caller pass over a standoff and place what is not in doubt.
    Better-supported crossings count as rivals to a worse-supported one, so a
    candidate that shares a ray with a standoff is refused along with it -- the
    ray is spoken for either way.
    """
    placement, first, second, _support, strength = found_fixes[index]
    rays = {first["id"], second["id"]}
    for other, other_first, other_second, _rest, other_strength in found_fixes:
        if other is placement:
            continue
        if other_strength < strength:
            # Sorted by support first, so nothing further down can be a rival.
            break
        if {other_first["id"], other_second["id"]}.isdisjoint(rays):
            continue
        if other["uncertainty_m"] > placement["uncertainty_m"] * RIVAL_FACTOR:
            continue
        apart = math.hypot(other["x_m"] - placement["x_m"],
                           other["y_m"] - placement["y_m"])
        if apart > max(SAME_PLACE_M,
                       other["uncertainty_m"] + placement["uncertainty_m"]):
            return True
    return False


def _could_be_one(first: dict[str, Any], second: dict[str, Any]) -> bool:
    """Could these two crops be two views of one object?

    A removal-only test, and it removes only the plainly unrelated -- see
    `DIFFERENT_THING` for the two measurements the number sits between. Two
    observations with no appearance vector between them say nothing either way,
    and saying nothing must not stop a placement: before perception carried
    vectors at all, every look was in exactly that position.
    """
    left = first.get("dino_blob") or b""
    right = second.get("dino_blob") or b""
    if not left or not right:
        return True
    return similarity(left, right) >= DIFFERENT_THING


def _replace_placement(store, entity_id: str, session: int,
                       reach=None) -> None:
    """Work the placement out again from everything now attached.

    Every observation-level measurement is kept when this happens: what changes
    is the application's opinion, and the evidence it was formed from is history.
    """
    observations = store.observations(entity_id, limit=24)
    rays = [ray for ray in (ray_of(one, reach) for one in observations) if ray]
    best = locate.best_fix(rays)
    if best is not None:
        # The pair chooses the answer; every ray that agrees with it then says
        # where exactly. See `locate.refine` -- this is what makes a look taken
        # to confirm a thing worth taking, because until it existed a third
        # agreeing bearing changed nothing at all.
        store.place(entity_id, locate.refine(best, rays), session)


#: Which pass makes new things out of bearings that nothing already placed
#: accounts for.
#:
#: `_pair_up` searches over pairs and commits to the best-supported crossing;
#: `_cluster_up` fits every leftover bearing at once and lets the association
#: settle itself. Both fill the same slot, take the same arguments and keep the
#: same gates, so which one runs is one name -- which is what makes them
#: comparable on a recording instead of on an argument.
#:
#: **It is `_pair_up`, and that is a measurement rather than caution.** Replayed
#: on the recording of 2026-09-03 by [bench_cluster.py](bench_cluster.py), the
#: greedy pass places 15 things with none of them mixed; the fitted pass places
#: 8 with soft weights or 12 with hard, and loosening its own gate as far as it
#: will go still reaches only 11. The reason is not the arithmetic -- the same
#: fit is what `locate.refine` now uses and it is better at *placing* than
#: anything here has been -- it is that discovery on this rover is
#: **incremental**. The greedy pass sees the pool again after every look and
#: offers every waiting ray to everything already placed, through the wide
#: `locate.match_tolerance` gate; the fitted pass has to find things from
#: crossings inside one pass's leftovers. With 35 usable looks in the whole
#: recording, and 275 of its 406 regions carrying no pose at all, no one pass
#: holds enough for the second to win.
#:
#: So this stays what it is until there is a recording where it does not. The
#: thing that would produce one is the shutter fix: looks taken while the rover
#: was turning now keep their bearings, which is where the pool gets several
#: times denser per pass. Flip this name and re-run the bench.
DISCOVERY = _pair_up
