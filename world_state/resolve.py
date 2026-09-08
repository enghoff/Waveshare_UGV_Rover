"""Resolve observations into entities using map geometry and appearance.

Ambiguous observations remain unplaced. Discovery uses pair crossings; the
cluster alternative remains available for comparison against recorded drives.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from . import cluster, locate, view
from .appearance import _UNIT, alone, any_of, appearance, similarity

MATCH = "match"
NEW = "new"
AMBIGUOUS = "ambiguous"

# Appearance rejects candidates below 0.55; a winner needs a 0.05 lead.
# These gates were measured on room recordings, not calibrated probabilities.
DIFFERENT_THING = 0.55
APPEARANCE_LEAD = 0.05
#: How far a look's appearance score may fall when everything but the thing
#: itself is blanked out of its crop, before the match is refused as having been
#: made on the intruder rather than on the thing. See `collapsed`.
#:
#: **Measured on the acceptance recording of 2026-09-07, and a development
#: candidate rather than a settled number.** Its review found four attachments
#: that were plainly a framed picture joined to an entity of dining chairs, each
#: because the box round the picture had a chair inside it. Those four scored
#: 0.561 to 0.698 on the plain crop and 0.203 to 0.469 on the masked one, while
#: the 360 correct attachments fell by a median of 0.061 and a 95th percentile
#: of 0.222. At 0.20 this catches four of four and refuses 25 of the 360.
#:
#: It was chosen after seeing those four, which is the thing the acceptance plan
#: forbids counting as independent evidence, so it is frozen here and owed a
#: held-out recording. Replacing the plain vector with the masked one instead was
#: measured and is worse: it loses 73 of the 394 to catch the same four.
COLLAPSED_ALONE = 0.20
#: What a new position's own crops have to score against a thing the rover knows
#: but cannot place here, for the two to be called the same thing. See `_adopt`.
#:
#: **Higher than `DIFFERENT_THING` because it is doing a different job.** That
#: gate only removes the plainly unrelated from candidates the geometry has
#: already accepted; this one has no geometry behind it at all, because the whole
#: situation is that the old coordinates are in a map that is gone. So it is set
#: where this rover's own measurements put the line: two regions of one frame,
#: which are different things by construction, score 0.32 in the median and 0.69
#: at the 95th, and one object across a real change of viewpoint scores 0.70.
RECOGNISED = 0.70
#: How far ahead some *other* thing in the room may be before this match is
#: refused as the wrong home for the crop. See `_outclassed`.
#:
#: **This is Lowe's ratio test, in the additive form these scores want.** SIFT
#: threw away a correspondence whose best match was not much better than its
#: second best, and the same idea does the job the gates here could not: every
#: other gate is a threshold a candidate passes on its own, so a crop that
#: resembles a painting at 0.55 is admitted without anyone asking that it
#: resembles a chair at 0.81. `_by_appearance` already compares rivals, but only
#: among the things the geometry accepted, and the thing a wrongly-attached crop
#: really belongs to is usually somewhere else in the room entirely.
#:
#: **Measured on the drive of 2026-09-08 and owed a held-out recording.** Over
#: 923 attachments the median crop resembles its own thing as well as any other
#: (a lead of -0.002); the 95th percentile is +0.146. The three merges found by
#: eye sit at +0.265, +0.172 and +0.272, which is ranks 8, 31 and 7. At 0.15 all
#: three are refused along with 41 other attachments, of which the worst are
#: plainly right -- a hanging lamp inside a doorway, a framed picture inside the
#: cabinet -- and the ones at the threshold are coin flips.
OUTCLASSED_LEAD = 0.15
RIVAL_FACTOR = 2.0
SAME_PLACE_M = 0.5
SAME_ANSWER = 0.05
# Bound discovery work so repeated inspection stays responsive as the pool grows.
MAX_NEW_PER_PASS = 2

#: One pass's worth of "what else does this crop look like", keyed by
#: observation. Cleared by `resolve` with `_UNIT`, and for the same reason.
_ELSEWHERE: dict = {}


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
                 # How high the thing sat, in degrees above the
                 # horizontal, with how tall it looked. Absent on every row
                 # written before the vertical half of the ray was kept, and
                 # absent means the geometry does not get a vertical opinion
                 # about this look rather than that the look was level. See
                 # `locate.rise_m`.
                 "elevation_deg": observation.get("elevation_deg"),
                 "elevation_span_deg": observation.get("elevation_span_deg"),
                 # And how far away the depth camera said it was, with what that
                 # reading is worth. Absent on every look the rover took before
                 # it read the depth camera, and on every look since taken
                 # somewhere the depth camera's picture does not cover -- absent
                 # means the geometry gets no opinion about the distance rather
                 # than that the distance was nothing. See `locate.stands_at_range`.
                 "range_m": observation.get("range_m"),
                 "range_sigma_m": observation.get("range_sigma_m"),
                 # Whether the frame cut the top or the bottom off the box,
                 # which is worked out from the box rather than stored: it is a
                 # property of where the region landed in the picture and no
                 # later change of lens can alter it. See
                 # `view.clipped_vertically` for why it matters more vertically
                 # than horizontally.
                 "elevation_clipped": view.clipped_vertically(
                     observation.get("bbox")),
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


#: Unit vectors by the blob they came from, for the length of one resolve pass.
#: Shared and unlocked, which is safe for the reason it is keyed on the bytes: an
#: entry is a pure function of its key, so a reader that finds one left behind by
#: a pass that has not cleared yet gets the right answer. The daemon serialises
#: its passes behind the inspector's lock in any case.


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
    _ELSEWHERE.clear()

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
    _ELSEWHERE.clear()
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
            fell = (None if used is None else
                    collapsed(store, entity["id"], observation, seen))
            if (used is None or (seen is not None and seen < DIFFERENT_THING)
                    or (fell is not None and fell >= COLLAPSED_ALONE)):
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
            store.add_exemplar(entity_id, observation["dino_blob"],
                               alone=observation.get("dino_alone_blob") or b"")
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
        # And whether that resemblance survives having the rest of the picture
        # taken away, which is what tells a chair from a picture with a chair in
        # front of it. See `collapsed`.
        fell = collapsed(store, entity["id"], observation, looks)
        if fell is not None and fell >= COLLAPSED_ALONE:
            continue
        # And whether something else in the room explains the crop far better,
        # which is the one question a threshold cannot ask. See `_outclassed`.
        if _outclassed(store, entity["id"], observation, looks, entities,
                       _ELSEWHERE):
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
    # **Learnt from only when the match was not in doubt.** A crop that joined
    # on a middling score becoming an exemplar is how a thing's template drifts
    # onto whatever it swallowed -- the model-drift problem visual trackers
    # solve by updating conservatively, and the mechanism behind `object:8` on
    # 2026-09-08: the chair joined the painting and was immediately part of what
    # the painting looked like. `RECOGNISED` is already this component's word
    # for "that is the same thing" rather than "that is not a different one".
    if vector and (chosen.get("appearance") or 0.0) >= RECOGNISED:
        store.add_exemplar(chosen["entity_id"], vector,
                           alone=observation.get("dino_alone_blob") or b"")
    _replace_placement(store, chosen["entity_id"], session, reach)
    return Decision(observation["id"], MATCH, chosen["entity_id"], why=why,
                    candidates=surviving)


def collapsed(store, entity_id: str, observation: dict[str, Any],
              seen: float | None) -> float | None:
    """How far this look's resemblance falls when only the thing itself is left.

    **The one gate that can tell "this is that chair" from "this has that chair
    in it".** A box drawn round a picture on the wall can contain the chair
    standing in front of it, and the crop then resembles an entity of chairs
    partly because it holds one. Asking the same question of the crop with
    everything but the region blanked out separates the two: a look that was
    matching on the thing barely moves, and one that was matching on the
    intruder collapses.

    `seen` is the score already computed from the plain crop, passed in rather
    than recomputed because both callers already have it.

    **None means the question could not be asked, and is not a low score.** A
    look whose backend returned no masks, an entity holding no masked exemplar,
    or a candidate nothing could be compared against on the plain crop either --
    all of them say nothing about whether this is the same thing, and refusing
    them would empty the world on any rover whose masks had not arrived.
    """
    if seen is None:
        return None
    apart = alone(store, entity_id, observation.get("dino_alone_blob") or b"")
    if apart is None:
        return None
    return round(seen - apart, 3)


def _outclassed(store, entity_id: str, observation: dict[str, Any],
                seen: float | None, entities: list[dict],
                found: dict) -> str | None:
    """The thing this crop resembles much more than the one it is joining.

    **The question none of the other gates asks.** Every gate in here gives a
    candidate a threshold to clear on its own: is it not plainly unrelated, does
    it survive masking, is it ahead of the other candidates the geometry
    accepted. None of them asks the question a person asks immediately on seeing
    the mistake -- that is not the painting, that is one of the chairs -- because
    the chair is across the room and was never a candidate.

    Returns the rival's identifier, or None when nothing is far enough ahead.
    `found` caches the scores for one observation across one pass, because the
    same crop is asked about once per candidate and the answer does not change.

    **Silence is not a low score**, as everywhere else here: a crop with no
    vector, or a room whose things hold no exemplars, says nothing about where
    this belongs and must not refuse anything.
    """
    if seen is None:
        return None
    vector = observation.get("dino_blob") or b""
    if not vector:
        return None
    key = observation.get("id")
    scores = found.get(key)
    if scores is None:
        scores = {}
        for entity in entities:
            got = appearance(store, entity["id"], vector)
            if got is not None:
                scores[entity["id"]] = got
        found[key] = scores
    for other, got in scores.items():
        if other != entity_id and got - seen >= OUTCLASSED_LEAD:
            return other
    return None


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
        placed = _place_one(store, available, session, entities, reach)
        if placed is None:
            # **Only when no crossing is left, and that ordering is the whole of
            # why this is safe to have at all.** A thing agreed by two viewpoints
            # is better evidence than a thing asserted by one, so the pool is
            # emptied of crossings first and what follows is the leftovers --
            # the looks at something the rover only ever saw from one place.
            placed = _place_from_range(store, available, session, entities,
                                       reach)
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
        known = _adopt(store, session,
                       [by_id[one] for one in kept if one in by_id])
        entity_id, again = known if known else (store.create_entity(), "")
        store.place(entity_id, placement, session)
        why = (f"{placement['rays_agreeing']} bearings from "
               f"{placement['viewpoints']} places fitted {entity_id} to within "
               f"{placement['uncertainty_m']} m{again}")
        store.attach(entity_id, kept, why)
        for observation_id in kept:
            one = by_id.get(observation_id) or {}
            vector = one.get("dino_blob") or b""
            if vector:
                store.add_exemplar(entity_id, vector,
                                   alone=one.get("dino_alone_blob") or b"")
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


def _place_one(store, available, session, entities, reach=None):
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
            # And neither crop may belong obviously somewhere else. A pair that
            # founds a thing is the one way in that no later gate can review,
            # because from the next pass onward the pair *is* what the thing
            # looks like. See `_outclassed`.
            left = first_observation.get("dino_blob") or b""
            right = second_observation.get("dino_blob") or b""
            # Both vectors or neither: `similarity` answers 0.0 for a missing
            # one, and 0.0 read as a score rather than as silence would refuse
            # every pair the moment anything else in the room was comparable.
            together = similarity(left, right) if left and right else None
            if together is not None and any(
                    _outclassed(store, None, one, together, entities,
                                _ELSEWHERE)
                    for one in (first_observation, second_observation)):
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

    taken = [first_observation["id"], second_observation["id"]]
    known = _adopt(store, session, [first_observation, second_observation])
    entity_id, again = known if known else (store.create_entity(), "")
    store.place(entity_id, placement, session)
    why = (f"two looks {placement['baseline_m']} m apart crossed at "
           f"{placement['parallax_deg']} degrees, placing {entity_id} to "
           f"within {placement['uncertainty_m']} m{again}")
    store.attach(entity_id, taken, why)
    for observation in (first_observation, second_observation):
        vector = observation.get("dino_blob") or b""
        if vector:
            store.add_exemplar(entity_id, vector,
                               alone=observation.get("dino_alone_blob") or b"")

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
        # The same comparative question the join path asks. This loop admits
        # whatever points at a thing placed a moment ago, so it is the easiest
        # way into a thing and wants the same refusal.
        if _outclassed(store, entity_id, observation, looks, entities,
                       _ELSEWHERE):
            continue
        claimed.add(observation.get("inference_id"))
        taken.append(observation["id"])
        store.attach(entity_id, [observation["id"]],
                     f"points at {entity_id} as well, from the same group"
                     + ("" if looks is None else f", appearance {looks:.2f}"))
        if vector and (looks or 0.0) >= RECOGNISED:
            store.add_exemplar(entity_id, vector,
                               alone=observation.get("dino_alone_blob") or b"")

    return Decision(
        first_observation["id"], NEW, entity_id,
        why=why,
        candidates=[{"entity_id": entity_id,
                     "from_observations": taken,
                     "uncertainty_m": placement["uncertainty_m"]}]), taken


def _already_there(placement: dict[str, Any], entities: list[dict]) -> bool:
    """Is there a thing here already, as far as either of them can tell?

    The two uncertainties added, because the question is whether the two
    positions can be told apart at all, and each of them is only as sharp as its
    own worst axis. A thing agreed by six rays to within 0.2 m and a point
    asserted by one look to within 0.2 m are the same place unless they are more
    than 0.4 m apart.
    """
    for entity in entities:
        known = entity.get("placement") or {}
        if "x_m" not in known or "y_m" not in known:
            continue
        apart = math.hypot(float(known["x_m"]) - placement["x_m"],
                           float(known["y_m"]) - placement["y_m"])
        if apart <= (float(known.get("uncertainty_m") or 0.0)
                     + placement["uncertainty_m"]):
            return True
    return False


def _place_from_range(store, available, session, entities, reach=None):
    """The best-measured single look that can stand a thing up on its own.

    `_place_one`'s neighbour, and deliberately the plainer of the two: one ray
    that measured its distance, placed where it says, with no support to count
    and nothing to contest it. What it does share is the tail -- a thing the
    rover already knows may adopt the position, the exemplars are stored, and
    the caller offers everything still waiting to the result -- so a thing born
    this way is an ordinary thing from the moment it exists.

    **It records that one look made it**, in `rays_agreeing` and `viewpoints`,
    because that is the difference a reader has to be able to see. The second
    look at the same object joins it through `_against_known` like any other and
    takes those numbers up with it.

    The best is the smallest uncertainty, which on one ray is dominated by how
    far away the thing is: a bearing worth 1.5 degrees is worth 3 cm at a metre
    and 16 cm at six. So this reaches for the near things first, which are also
    the ones the depth camera measures best.
    """
    offered = []
    for observation in available:
        ray = ray_of(observation, reach)
        if ray is None:
            continue
        placement = locate.at_range(ray)
        if placement is None:
            continue
        # **Not where something already is.** This look has already been offered
        # to every thing the rover knows and was not taken by any of them, so
        # standing a second thing up inside the first one's uncertainty invents
        # a duplicate rather than a discovery -- and a duplicate is worse than
        # the silence it replaces, because it looks like knowledge. Measured on
        # the drive of 2026-09-08 without this: 57 of 100 things placed this way
        # sat within half a metre of one placed by a crossing.
        if _already_there(placement, entities):
            continue
        offered.append((placement["uncertainty_m"], placement, observation))
    if not offered:
        return None
    offered.sort(key=lambda one: one[0])
    _uncertainty, placement, observation = offered[0]
    placement = dict(placement, rays_agreeing=1, viewpoints=1)

    known = _adopt(store, session, [observation])
    entity_id, again = known if known else (store.create_entity(), "")
    store.place(entity_id, placement, session)
    why = (f"one look measured {placement['from_range_m']} m to it, placing "
           f"{entity_id} to within {placement['uncertainty_m']} m from a single "
           f"viewpoint{again}")
    store.attach(entity_id, [observation["id"]], why)
    vector = observation.get("dino_blob") or b""
    if vector:
        store.add_exemplar(entity_id, vector,
                           alone=observation.get("dino_alone_blob") or b"")
    return Decision(
        observation["id"], NEW, entity_id,
        why=why,
        candidates=[{"entity_id": entity_id,
                     "from_observations": [observation["id"]],
                     "uncertainty_m": placement["uncertainty_m"]}]), [
        observation["id"]]


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


def _adopt(store, session: int, observations: list[dict[str, Any]]
           ) -> tuple[str, str] | None:
    """The thing the rover already knows that this new position belongs to.

    **A map change must not cost the rover what it knows.** Everything located
    here is located in the map of the day, and when the SLAM map is replaced --
    cleared, or rebuilt because a saved one would not load -- every coordinate
    recorded under the old one stops meaning anything. What does not stop meaning
    anything is the *appearance*: the crops are still crops of the same chair,
    and the rover has been keeping them as exemplars all along. So the moment a
    fresh crossing establishes a position in the new map, the thing standing
    there is offered to everything the rover owns but cannot currently place, and
    takes back its own identity where one of them is plainly it.

    Without this a map change is quietly destructive in a way nothing reports:
    the old entities keep their history and their looks and can never be placed
    again -- nothing re-places a thing the resolver will not consider, and it
    considers only what is placed in the map it is working in -- while the same
    furniture is discovered all over again as strangers. Replayed across the map
    change of 2026-09-06, that is 272 things of which 99 could be driven to and
    **none at all** were still what the rover had spent the previous day
    learning.

    Two rules, and the second is the one that stops this being dangerous.
    `RECOGNISED` is where a crop stops being a coincidence, and it is set from
    what this rover has measured rather than chosen to be safe. The lead is the
    identical-chairs rule the placed candidates already live under: where two
    things the rover knows look equally like the thing now standing here,
    appearance cannot say which, and inventing a new thing is the answer that
    can be corrected later. Merging two histories cannot be.

    Answers the entity to adopt and the sentence to say so, or None for a thing
    the rover has genuinely not seen before.
    """
    vectors = [one.get("dino_blob") or b"" for one in observations]
    vectors = [vector for vector in vectors if vector]
    if not vectors:
        return None
    ranked = []
    for entity in store.placed_elsewhere(session):
        # The best of the founding crops rather than their average, and each
        # score itself the middle of the entity's exemplars -- see
        # `appearance.any_of`, which is where both of those are written down and
        # where the ratchet this could otherwise become is held shut.
        looks = any_of(store, entity["id"], vectors)
        if looks is not None:
            ranked.append((looks, entity))
    if not ranked:
        return None
    ranked.sort(key=lambda one: (-one[0], one[1]["id"]))
    best, entity = ranked[0]
    if best < RECOGNISED:
        return None
    if len(ranked) > 1 and best - ranked[1][0] < APPEARANCE_LEAD:
        return None
    was = entity.get("placement_map_session")
    return entity["id"], (f"; recognised as {entity['id']}, which the rover last "
                          f"placed in map {was} and which these looks match at "
                          f"{best:.2f}")


def _replace_placement(store, entity_id: str, session: int,
                       reach=None) -> None:
    """Work the placement out again from everything now attached.

    Every observation-level measurement is kept when this happens: what changes
    is the application's opinion, and the evidence it was formed from is history.

    **Only the looks taken under this map, which matters from the moment a thing
    can outlive one.** A ray is a bearing *from a place*, and the place is a
    position in the map of the day; a look recorded before the map changed names
    a point in this one only by coincidence. An entity that has just been
    recognised across a map change carries a history of those, and fitting them
    together with the looks that recognised it would put the thing at the
    average of two rooms. The same rule, for the same reason, as the one
    `_world_sight_lines` applies when it works out where to stand.
    """
    observations = [one for one in store.observations(entity_id, limit=24)
                    if one.get("map_session") == session]
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
