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

    1. semantic     a chair is not a ceiling light. A hint, never a key.
    2. map session   coordinates from a map that no longer exists mean nothing.
    3. spatial       the bearing has to point at where the thing already is.
                     **For furniture this is a hard gate**, and no amount of
                     appearance may overrule it -- that is the redundant-
                     furniture test the whole design exists to pass.
    4. appearance    DINOv2 against several stored exemplars.
    5. history       how recently and how often, as a tiebreak of last resort.

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

import json
import math
from dataclasses import dataclass, field
from typing import Any

from . import locate

MATCH = "match"
NEW = "new"
AMBIGUOUS = "ambiguous"

#: Families of vocabulary phrases that may be the same thing under a different
#: name. **A permissive gate on purpose.** Too strict and one thing becomes two
#: entities, which is a duplicate nobody notices; too loose and the spatial gate
#: still has to reject it, which is visible in the popup as an ambiguity. The
#: second failure is the cheaper one, so this errs towards letting things in.
#:
#: Labels come from `vocabulary.txt`, so anything not named here simply has to
#: match exactly, which is the safe default rather than an omission.
SYNONYMS = (
    {"a wooden chair", "an office chair", "an armchair", "a chair", "a stool"},
    {"a sofa", "a couch", "an armchair"},
    {"a doorway", "a door", "an open door"},
    {"a cupboard", "a cabinet", "a wardrobe", "a bookcase", "a shelf"},
    {"a table", "a desk", "a coffee table", "a side table"},
    {"a television", "a computer monitor", "a screen"},
    {"a lamp", "a floor lamp", "a ceiling light", "a light"},
    {"a rug", "a carpet", "a mat"},
    {"a picture", "a painting", "a framed picture", "a mirror"},
)

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

#: Things that move. Placement is a strong identity rule for a bookcase and a
#: poor one for a bottle, so a movable thing is never matched on position alone
#: and never placed from a single crossing without appearance agreeing too.
MOVABLE = {
    "a bottle", "a spray bottle", "a cup", "a mug", "a glass", "a bag",
    "a cable", "a power cable", "a book", "a remote control", "a phone",
    "a box", "a cardboard box", "a cushion", "a plate", "a bowl", "a toy",
}


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


def compatible(first: str, second: str) -> bool:
    """Could these two names be the same thing?

    The first gate, and the weakest. Labels drift: the same scene named a chair
    "a wooden chair" from one angle and "an office chair" from another, which is
    exactly why this may only remove candidates and may never confirm one.
    """
    left, right = (first or "").strip().lower(), (second or "").strip().lower()
    if not left or not right:
        return False
    if left == right:
        return True
    return any(left in family and right in family for family in SYNONYMS)


def movable(label: str) -> bool:
    return (label or "").strip().lower() in MOVABLE


def ray_of(observation: dict[str, Any]) -> dict[str, Any] | None:
    """The bearing already stored on an observation, as `locate` wants it.

    Recomputed from nothing: the bearing was worked out when the look was taken,
    from the field of view the camera had at that moment, and it is a
    measurement rather than a derivation.
    """
    pose = observation.get("pose")
    bearing = observation.get("bearing_deg")
    if not isinstance(pose, dict) or bearing is None:
        return None
    try:
        return {"x_m": float(pose["x_m"]), "y_m": float(pose["y_m"]),
                "bearing_deg": float(bearing),
                "span_deg": float(observation.get("span_deg") or 0.0),
                "observation_id": observation.get("id")}
    except (KeyError, TypeError, ValueError):
        return None


def similarity(left: bytes, right: bytes) -> float:
    """Cosine between two float32 vectors, without numpy.

    Written out because this runs inside the daemon, which imports no third-party
    package, and because a few hundred multiplications is not worth an import
    that would have to be vendored and could be missing.
    """
    if not left or not right or len(left) != len(right):
        return 0.0
    import struct

    count = len(left) // 4
    a = struct.unpack(f"<{count}f", left)
    b = struct.unpack(f"<{count}f", right)
    dot = sum(x * y for x, y in zip(a, b))
    na = math.sqrt(sum(x * x for x in a))
    nb = math.sqrt(sum(y * y for y in b))
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    return dot / (na * nb)


def best_appearance(store, entity_id: str, vector: bytes) -> float:
    """How much this crop looks like the best of what the entity has shown.

    The best rather than the average, for the reason the exemplars are kept
    separately at all: a chair seen from the front and the same chair from the
    side average to a picture of neither.
    """
    if not vector:
        return 0.0
    best = 0.0
    for exemplar in store.exemplars(entity_id, width=len(vector)):
        best = max(best, similarity(exemplar, vector))
    return best


def resolve(store, *, map_session: int | None = None,
            limit: int = 500) -> dict[str, Any]:
    """One pass over the pending pool. Decides, records, and explains.

    Two passes internally, and the order is the whole algorithm. First every
    pending observation is offered to the things already placed, because joining
    a known thing is cheaper and safer than inventing one. Only what is left over
    is considered for pairing into something new, and only where two bearings
    genuinely cross.
    """
    session = store.map_session() if map_session is None else int(map_session)
    pending = store.unplaced(map_session=session, limit=limit)
    entities = store.placed(map_session=session)
    decisions: list[Decision] = []

    # Which entities each frame has already accounted for. Two regions in one
    # frame are two different things -- the region finder's own suppression saw
    # to that -- so once a frame has matched an entity, its other regions may
    # not match the same one however well they line up.
    taken_in: dict[Any, set] = {}
    leftover = []
    for observation in pending:
        decision = _against_known(store, observation, entities, session, taken_in)
        if decision is None:
            leftover.append(observation)
            continue
        decisions.append(decision)

    decisions.extend(_pair_up(store, leftover, session))

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


def _against_known(store, observation, entities, session,
                   taken_in) -> Decision | None:
    """Offer one observation to the things already placed.

    None means "no candidate survived the gates", which is not a decision: the
    observation goes on to the pairing pass, where it may help place something
    new. A decision means it was matched, or that it was ambiguous and is being
    left alone deliberately.
    """
    ray = ray_of(observation)
    if ray is None:
        return None
    label = observation.get("label") or ""
    vector = observation.get("dino_blob") or b""

    frame = observation.get("inference_id")
    already = taken_in.setdefault(frame, set())
    surviving = []
    for entity in entities:
        if entity["id"] in already:
            continue
        if not compatible(entity.get("label", ""), label):
            continue
        placement = entity.get("placement") or {}
        if not locate.agrees(placement, ray):
            continue
        surviving.append({
            "entity_id": entity["id"],
            "label": entity.get("label", ""),
            "distance_m": round(math.hypot(
                float(placement.get("x_m", 0.0)) - ray["x_m"],
                float(placement.get("y_m", 0.0)) - ray["y_m"]), 2),
            "appearance": round(best_appearance(store, entity["id"], vector), 3),
            "seen": entity.get("observation_count", 0),
        })

    if not surviving:
        return None
    if len(surviving) > 1:
        surviving.sort(key=lambda one: (-one["appearance"], -one["seen"]))
        lead = surviving[0]["appearance"] - surviving[1]["appearance"]
        if lead < APPEARANCE_LEAD:
            return Decision(
                observation["id"], AMBIGUOUS, None,
                why=(f"{len(surviving)} placed things are equally consistent with "
                     f"this bearing and appearance cannot separate them "
                     f"({surviving[0]['appearance']:.2f} against "
                     f"{surviving[1]['appearance']:.2f}); left unassigned "
                     f"rather than guessed"),
                candidates=surviving)
        # Appearance is allowed to choose only among candidates the geometry has
        # already accepted, and only when one is clearly ahead. It may never
        # bring a candidate back that the spatial gate rejected.

    chosen = surviving[0]
    if movable(label) and chosen["appearance"] < APPEARANCE_LEAD:
        return Decision(
            observation["id"], AMBIGUOUS, None,
            why=(f"{label} is a thing that moves, so its position is not enough "
                 f"on its own, and it does not look like what "
                 f"{chosen['entity_id']} has shown before"),
            candidates=surviving)

    already.add(chosen["entity_id"])
    store.attach(chosen["entity_id"], [observation["id"]])
    if vector:
        store.add_exemplar(chosen["entity_id"], vector)
    _replace_placement(store, chosen["entity_id"], session)
    return Decision(
        observation["id"], MATCH, chosen["entity_id"],
        why=(f"the bearing points at {chosen['entity_id']} "
             f"{chosen['distance_m']} m away, appearance "
             f"{chosen['appearance']:.2f}"),
        candidates=surviving)


def _pair_up(store, leftover, session) -> list[Decision]:
    """Make new things out of pairs of bearings that actually cross.

    Grouped by compatible label first, which is cheap, and then every pair inside
    a group is tried. What comes out is the pair with the smallest uncertainty,
    because a least-squares fit over rays whose error is dominated by one bad box
    is worse than the best honest pair -- and because the popup has to be able to
    name the two looks that placed the thing.
    """
    decisions: list[Decision] = []
    used: set[int] = set()
    for group in _by_label(leftover):
        while True:
            available = [one for one in group if one["id"] not in used]
            if len(available) < 2:
                break
            placed = _place_one(store, available, session)
            if placed is None:
                break
            decision, taken = placed
            used.update(taken)
            decisions.append(decision)
    return decisions


def _by_label(observations) -> list[list[dict[str, Any]]]:
    groups: list[list[dict[str, Any]]] = []
    for observation in observations:
        for group in groups:
            if compatible(group[0].get("label", ""), observation.get("label", "")):
                group.append(observation)
                break
        else:
            groups.append([observation])
    return groups


def _place_one(store, available, session):
    """The best-supported crossing among these observations, or None.

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
    ties with a conflicting one somewhere else is refused.

    None ends the group, and the whole group stays pending. That is the right
    answer for a rover that has looked at something from one place only.
    """
    rays = []
    for observation in available:
        ray = ray_of(observation)
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
            crossing = locate.fix(first, second)
            if crossing is None:
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
    placement, first_observation, second_observation, support, strength =         found_fixes[0]
    chosen_rays = {first_observation["id"], second_observation["id"]}
    for other, other_first, other_second, _support, other_strength in found_fixes[1:]:
        if other_strength < strength:
            break
        # **Only a crossing that shares a ray is a rival.** A ray points at one
        # thing, so two equally supported answers built from the same ray cannot
        # both be right and nothing here can say which. Two crossings built from
        # entirely different rays are simply two different objects, and refusing
        # those would mean a room could only ever hold one chair.
        if {other_first["id"], other_second["id"]}.isdisjoint(chosen_rays):
            continue
        if other["uncertainty_m"] > placement["uncertainty_m"] * RIVAL_FACTOR:
            continue
        apart = math.hypot(other["x_m"] - placement["x_m"],
                           other["y_m"] - placement["y_m"])
        if apart > other["uncertainty_m"] + placement["uncertainty_m"]:
            # A third look from somewhere else settles it; nothing here can.
            return None

    label = first_observation.get("label") or "a thing"
    if movable(label):
        # A movable thing placed from one crossing is a guess about where it was
        # a moment ago. Appearance has to agree as well before it becomes a
        # lasting thing with a position.
        seen = similarity(first_observation.get("dino_blob") or b"",
                          second_observation.get("dino_blob") or b"")
        if seen < 0.5:
            return None

    entity_id = store.create_entity(_kind_of(label), label)
    store.place(entity_id, placement, session)
    taken = [first_observation["id"], second_observation["id"]]
    store.attach(entity_id, taken)
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
    supporting = {one["id"] for one in support}
    for _ray, observation in rays:
        if observation["id"] in taken or observation["id"] not in supporting:
            continue
        if observation.get("inference_id") in claimed:
            continue
        claimed.add(observation.get("inference_id"))
        taken.append(observation["id"])
        store.attach(entity_id, [observation["id"]])
        vector = observation.get("dino_blob") or b""
        if vector:
            store.add_exemplar(entity_id, vector)

    return Decision(
        first_observation["id"], NEW, entity_id,
        why=(f"two looks {placement['baseline_m']} m apart crossed at "
             f"{placement['parallax_deg']} degrees, placing {entity_id} to "
             f"within {placement['uncertainty_m']} m"),
        candidates=[{"entity_id": entity_id, "label": label,
                     "from_observations": taken,
                     "uncertainty_m": placement["uncertainty_m"]}]), taken


def _kind_of(label: str) -> str:
    """The coarse bucket an identifier is counted in.

    Only three, and only because the identifier reads better as `furniture:3`
    than as `object:47`. Nothing decides anything on it.
    """
    words = (label or "").lower()
    if any(word in words for word in ("door", "window", "doorway", "staircase")):
        return "opening"
    if any(word in words for word in ("chair", "sofa", "couch", "table", "desk",
                                      "bookcase", "cupboard", "cabinet", "bed",
                                      "shelf", "wardrobe", "stool")):
        return "furniture"
    return "object"


def _replace_placement(store, entity_id: str, session: int) -> None:
    """Work the placement out again from everything now attached.

    Every observation-level measurement is kept when this happens: what changes
    is the application's opinion, and the evidence it was formed from is history.
    """
    observations = store.observations(entity_id, limit=24)
    rays = [ray for ray in (ray_of(one) for one in observations) if ray]
    best = locate.best_fix(rays)
    if best is not None:
        store.place(entity_id, best, session)
