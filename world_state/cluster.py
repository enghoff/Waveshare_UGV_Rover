"""Where several things are, from all the bearings at once.

[locate.py](locate.py) crosses **two** bearings and is right to: two rays from
two places meet at a point, and that is the only honest position a camera with no
range sensor can give. What it cannot do is decide *which* rays belong together,
and on a rover that looks at a room full of similar furniture that is the harder
half of the problem. `resolve._place_one` answers it greedily -- pick the
best-supported crossing, create the thing, attach whatever agrees, never look
again -- and every fault that pass has had is a fault of committing early.

This module answers both halves at once. It is the standard formulation for
bearings-only measurements with unknown data association: **the positions are
continuous unknowns, which rays belong to which thing is a latent variable, and
the two are estimated together by expectation-maximisation.** In the robotics
literature it is Bowman's probabilistic data association SLAM (ICRA 2017); in the
tracking literature the same machinery with a recursive filter instead of a batch
fit is joint probabilistic data association. Nothing here is novel and that is
the point -- the alternative was another hand-rolled heuristic.

## What one round does

    E   for each look, how likely is it that this region is this thing?
        Not a decision -- a weight, and the weights over one look are
        constrained so that a thing may claim at most one region of a picture.

    M   for each thing, where must it be for all the rays that partly claim it?
        A weighted least-squares fit over its own bearings, two unknowns,
        solved exactly. The fit reports a real covariance, which is the
        first honest error *shape* this component has had.

Repeat until the positions stop moving. Then throw away the things nothing
believes in.

## Why this and not a better crossing

Three of the faults written up in [README.md](README.md) are the same fault, and
this is the shape of the answer to all three.

* **A placement moving out from under its own evidence.** `best_fix` takes the
  pair the other rays agree with, which is a repair rather than a fix: the answer
  is still two rays and the rest only get a vote. Here every ray that believes in
  a thing pulls on it, in proportion to how much it believes.
* **A contested crossing being refused outright.** Two rays that cross in two
  defensible ways are a data-association ambiguity, and refusing both is the only
  safe answer *if a decision has to be made now*. It does not: a weight of a half
  each is a perfectly good intermediate state, and the third look that settles it
  moves the weights rather than arriving too late.
* **A phantom.** Two rays at two identical chairs cross where no chair is. That
  crossing is a candidate here too -- and it dies, because the rays that made it
  are also claimed by the real chairs, and a thing whose whole support is borrowed
  loses it as soon as the real things fit better.

## What it does not fix

**Parallax.** Four rays 3 degrees apart do not locate anything, and no estimator
makes them. What changes is the failure mode: the fit answers with a covariance
whose long axis runs off down the line of sight, and it is refused for *that*
rather than refused by a threshold on the angle between two rays. The refusal is
still a refusal.

## Adding a range measurement

**This is the reason this formulation and not a filter.** A bearing enters the
fit as one residual -- which way the thing lies, against which way the ray
pointed. A range from the depth camera is another residual on the same unknown:
how far the thing is, against how far the ray said. Nothing else changes; the
weights, the fit and the covariance all take it as read. `locate.residuals` is where
both live, and `RANGE_SIGMA_M` is the one constant a range needs. The OAK-D-Lite
on the front of this rover already serves stereo depth and nothing reads it; when
something does, a ray gains a `range_m` and this module needs no structural
change. See [oak_depth/README.md](../oak_depth/README.md) for what has to be
settled first, which is extrinsics rather than arithmetic.
"""
from __future__ import annotations

import math
from typing import Any

from . import locate

#: How many rounds of expectation-maximisation before giving up on convergence.
#: Measured on the recording of 2026-09-03: the positions stop moving after 4 to
#: 7 rounds and the weights after 3, so this is a guard against a cycle rather
#: than a working limit. A cycle is possible -- two things swapping the same ray
#: back and forth -- and the answer to it is the best round seen, not the last.
MAX_ROUNDS = 24

#: How far the furthest thing may move in a round for the answer to be settled,
#: in metres. A centimetre, which is a fifth of the occupancy grid's own cell and
#: far below anything the rest of the component can tell apart.
SETTLED_M = 0.01

#: What a region is worth if it is not any of the things on offer, as a
#: likelihood density per degree of bearing.
#:
#: **Most regions really are nothing**, and a formulation with no way to say so
#: forces every wall, floor and blank patch onto the nearest thing. This is the
#: alternative hypothesis, and it is deliberately generous: one over the width of
#: the camera's own view, which says a region is a priori as likely to point
#: anywhere the camera can see as at anything in particular.
#:
#: For scale, a bearing landing one standard deviation from a thing scores about
#: 27 times this, and one landing three away scores about a third of it. So the
#: crossover sits between two and three sigma, which is where the rest of this
#: component already draws the line between "points at it" and "does not".
CLUTTER_PER_DEG = 1.0 / 100.0

#: And the same thing for a range, as a density per metre: one over the depth of
#: the room the geometry is willing to believe in. Used only where a ray carries
#: a `range_m`, so that a region's own alternative hypothesis is measured in the
#: same units as the things it is being compared against. Without it a ranged ray
#: would score a density per degree per metre against a clutter density per
#: degree, and every ray would beat clutter by the width of a room.
CLUTTER_PER_M = 1.0 / locate.MAX_RANGE_M

#: How much more likely a region is to be nothing than to be any particular
#: thing, before any bearing is looked at.
#:
#: **Measured rather than chosen.** On the recording of 2026-09-03 the resolver
#: placed 15 things from 406 regions and left 340 waiting, and the great majority
#: of those really are walls, doors, floor and ceiling rather than things it
#: missed. A prior of 4 says a region is four times more likely to be scenery
#: than to be the thing being fitted, which is conservative in the direction that
#: matters: too high leaves a real thing unplaced and the observation waiting,
#: too low writes furniture into the world that was never there.
CLUTTER_PRIOR = 4.0

#: How far ahead of the runner-up a thing must be to claim a ray, as a fraction.
#:
#: **A ray goes to the best candidate for it provided the best is clearly better,
#: and not to whichever candidate passes a fixed bar.** A fixed bar was the
#: obvious way to write it and it cannot work, because the quantity it tests is
#: divided among however many candidates happen to exist: measured on three
#: objects seen from three places, nine rays generated fourteen candidates and no
#: ray gave any one of them as much as a third, so nothing was ever claimed and
#: nothing was ever placed.
#:
#: Half again, so the best must be one and a half times the second. It is the
#: same idiom `resolve.SAME_ANSWER` and `resolve.APPEARANCE_LEAD` use for the
#: same reason -- an answer is only an answer if the next one down is worse -- and
#: like them it is scale-free, which is the property that was missing.
#:
#: A ray with two equally good explanations claims neither, which is the right
#: answer and is what stops a phantom: two rays crossing at a place no object is
#: fit that place exactly as well as they fit the two real objects either side,
#: so the tie is real and refusing to break it is honest.
CLAIM_LEAD = 0.5

#: How small a thing's share of the evidence may get before it is not a thing.
#:
#: **This is what kills a phantom, and it is the mechanism rather than a
#: threshold.** Every candidate carries a share -- how much of the pool it
#: accounts for, relative to the average candidate -- and it is re-estimated
#: every round from the responsibilities it actually won. A crossing between two
#: different chairs explains no ray better than the chairs themselves do, so it
#: loses a little share each round, which lowers its likelihood, which loses it
#: more: it decays to nothing on its own. That is the ordinary way a mixture with
#: an unknown number of components sheds the ones it does not need, and it
#: replaced pruning the least-believed candidate by hand -- which cannot work,
#: because a real thing whose rays a phantom is holding *is* the least-believed
#: candidate at that moment.
MIN_SHARE = 0.05

#: How much evidence every candidate is credited with before any is counted, in
#: rays. One, so that a thing needs to be genuinely unexplained rather than
#: merely the least of several to lose its place. See `_share` for what happens
#: without it.
SHARE_PRIOR = 1.0

#: How many rays must claim a thing for it to be a thing. Two, which is the same
#: evidence `locate.fix` demands and no more.
#:
#: **Counted in rays that clear `CLAIMED`, and not as a total of the weights.**
#: A total was the obvious way to write it and it is unreachable: a bearing
#: landing exactly on a thing still only claims
#: `peak / (peak + CLUTTER_PER_DEG * CLUTTER_PRIOR)` of it, which at a degree and
#: a half of noise is 0.87 and never 1. Asking for a total of two therefore asked
#: for two and a third perfect bearings, so nothing with only two looks behind it
#: could ever be placed -- which is the commonest case there is.
MIN_CLAIMING = 2

#: How uncertain a position may be, in metres, before it is not a position.
#:
#: **This is the parallax floor restated as what it was always about.**
#: `MIN_PARALLAX_DEG` refuses two rays closer than 12 degrees because the
#: crossing runs away down the line of sight; what actually matters is how far it
#: ran, and a fit over many rays can be well conditioned where no single pair
#: clears 12 degrees. So the fit is allowed to try and is judged on its answer.
#: 0.6 m is what 12 degrees of parallax buys at this rover's median range of
#: about 2.5 m, so a thing refused here would have been refused there.
MAX_UNCERTAINTY_M = 0.6

#: Two crossings closer than this are the same crossing found twice, in metres.
#: Three occupancy-grid cells, which is far below anything this component claims
#: to tell apart, and deliberately far below `SAME_PLACE_M`: that one decides
#: whether two *converged fits* are one thing, and using it here instead cost the
#: neighbours the whole exercise is about -- a chair and its rug are 31 cm apart.
SAME_SEED_M = 0.15

#: Two fitted positions closer than this are one thing, in metres. The same
#: figure `resolve.SAME_PLACE_M` uses for the same judgement, and it is applied
#: here because two candidates started from two crossings of the same rays
#: routinely converge onto one answer -- which is the fit agreeing with itself
#: and must not be reported as two things.
SAME_PLACE_M = 0.5

#: The most feasible arrangements of one look worth enumerating exactly. Beyond
#: this the exact marginals are abandoned for the single best arrangement, which
#: is the same answer `resolve._by_look` computes and a strictly worse E-step.
#: 4096 covers every look in the recording of 2026-09-03 -- the worst had 6
#: regions against 5 candidates -- and the fallback has never been reached on real
#: data. It exists so that a room full of furniture degrades instead of hanging.
MAX_ARRANGEMENTS = 4096


def _wrap(degrees: float) -> float:
    return (degrees + 180.0) % 360.0 - 180.0


def _bearing_to(x_m: float, y_m: float, ray: dict[str, Any]) -> float:
    return math.degrees(math.atan2(y_m - float(ray["y_m"]),
                                   x_m - float(ray["x_m"])))


def _range_to(x_m: float, y_m: float, ray: dict[str, Any]) -> float:
    return math.hypot(x_m - float(ray["x_m"]), y_m - float(ray["y_m"]))


def _likelihood(place: dict[str, Any], ray: dict[str, Any], extent_m: float
                ) -> float:
    """How likely this ray is, if it is pointing at this thing.

    A Gaussian on however much the bearing misses the thing's *silhouette* by,
    scaled by the measurement noise alone, as a density per degree so that it is
    comparable with `CLUTTER_PER_DEG`. A bearing landing anywhere on the thing
    scores the peak; one landing off it falls away at the rate the bearing is
    known to. At the sigmas involved -- a degree and a half to a few degrees --
    wrapping makes no measurable difference and the plain Gaussian is used.
    """
    sigma = locate.noise_deg(place["x_m"], place["y_m"], ray)
    half = locate.silhouette_deg(place["x_m"], place["y_m"], ray, extent_m)
    off = abs(_wrap(_bearing_to(place["x_m"], place["y_m"], ray)
                    - float(ray["bearing_deg"])))
    missed = max(0.0, off - half)
    got = (math.exp(-0.5 * (missed / sigma) ** 2)
           / (sigma * math.sqrt(2.0 * math.pi)))

    # **And how far away it is, where the ray says.** This is the half of a range
    # measurement that matters most here and it is easy to leave out: a range in
    # the *fit* only refines a thing already associated, while a range in the
    # *association* is what destroys a phantom. Two bearings crossing where no
    # object is fit that crossing exactly as well as they fit the real objects
    # either side of it -- there is nothing in the angles to separate them -- but
    # a ray that also says how far along itself the thing sits agrees with one
    # place and not the other. Measured on three objects seen from three places,
    # bearings alone place none of them and bearings with a range place all
    # three.
    measured = ray.get("range_m")
    if measured is None:
        return got
    range_m = _range_to(place["x_m"], place["y_m"], ray)
    sigma_m = float(ray.get("range_sigma_m") or locate.RANGE_SIGMA_M)
    # Forgiven the thing's own depth for the same reason the bearing is forgiven
    # its width: a stereo camera ranges the near face of a wardrobe and the
    # wardrobe is stored as a point.
    short = max(0.0, abs(range_m - float(measured)) - max(0.0, extent_m) / 2.0)
    return got * (math.exp(-0.5 * (short / sigma_m) ** 2)
                  / (sigma_m * math.sqrt(2.0 * math.pi)))


def _arrangements(scores: list[list[float | None]], clutter: list[float],
                  limit: int = MAX_ARRANGEMENTS
                  ) -> list[tuple[float, tuple[int, ...]]] | None:
    """Every way this look could be shared out, with what each way is worth.

    `scores[i][k]` is what region *i* is worth as thing *k*, or None where the
    pairing is impossible; `clutter[i]` is what it is worth as nothing. An
    arrangement gives each region one thing or nothing, and **no thing twice** --
    two regions of one picture are two different objects, which is the constraint
    `resolve._by_look` enforces with an assignment solver and this enumerates.

    None means there are more arrangements than `limit`, and the caller falls
    back to the single best one.
    """
    regions = len(scores)
    out: list[tuple[float, tuple[int, ...]]] = []
    order: list[int] = [-1] * regions

    def walk(index: int, weight: float, used: frozenset) -> bool:
        if len(out) > limit:
            return False
        if index == regions:
            out.append((weight, tuple(order)))
            return True
        order[index] = -1
        if not walk(index + 1, weight * clutter[index], used):
            return False
        for column, score in enumerate(scores[index]):
            if score is None or column in used:
                continue
            order[index] = column
            if not walk(index + 1, weight * score, used | {column}):
                return False
        order[index] = -1
        return True

    return out if walk(0, 1.0, frozenset()) else None


def _best_arrangement(scores: list[list[float | None]],
                      clutter: list[float]) -> tuple[int, ...]:
    """The single most likely way to share this look out.

    The fallback for a look with too many arrangements to enumerate, and it is
    the same problem `resolve._by_look` solves -- minimise a total cost subject to
    one thing per region -- so it is solved the same way, with
    `scipy.optimize.linear_sum_assignment` where that can be reached and greedily
    where it cannot. Costs are negative log likelihoods, and a region is left as
    nothing whenever no thing beats its own clutter score.
    """
    regions, things = len(scores), (len(scores[0]) if scores else 0)
    order = [-1] * regions
    cells = []
    for index in range(regions):
        for column in range(things):
            score = scores[index][column]
            if score is not None and score > clutter[index]:
                cells.append((math.log(score) - math.log(clutter[index]),
                              index, column))
    if not cells:
        return tuple(order)
    solve = _solver()
    if solve is not None:
        big = max(gain for gain, _i, _k in cells) + 1.0
        costs = [[big for _ in range(things)] for _ in range(regions)]
        for gain, index, column in cells:
            costs[index][column] = big - gain
        rows, columns = solve(costs)
        for index, column in zip(rows, columns):
            if costs[index][column] < big:
                order[index] = int(column)
        return tuple(order)
    taken: set = set()
    for gain, index, column in sorted(cells, reverse=True):
        if order[index] == -1 and column not in taken:
            order[index] = column
            taken.add(column)
    return tuple(order)


_SOLVER: Any = None


def _solver():
    """`scipy.optimize.linear_sum_assignment`, or None if it cannot be reached.

    The same accessor `resolve._solver` is, for the same reason and with the same
    degradation: an import that has moved must make the rover greedy rather than
    silent. Duplicated rather than shared because the two modules are imported
    independently and a rover that placed nothing because of an import cycle
    would be a worse failure than two five-line functions.
    """
    global _SOLVER
    if _SOLVER is None:
        try:
            from scipy.optimize import linear_sum_assignment  # noqa: PLC0415

            _SOLVER = linear_sum_assignment
        except Exception:                                      # noqa: BLE001
            _SOLVER = False
    return _SOLVER or None


def _seeds(rays: list[dict[str, Any]], looks_like=None) -> list[dict[str, Any]]:
    """Where to start looking for things: every crossing that could be one.

    **`locate.fix` is the candidate generator**, which keeps every hard-won gate
    in it -- baseline, parallax, range at both ends, and the wall check -- as the
    test of whether a crossing is worth *starting* from. That is a deliberately
    different job from deciding whether a fitted thing is real, which is what the
    pruning at the end does, and it means this module inherits rather than
    reimplements the geometry the component already trusts.

    Crossings within `SAME_SEED_M` of one another are one seed, which is a much
    tighter test than the one that decides whether two *fitted* things are one.
    **The looser figure was a fault**: a chair and the rug it stands on are 31 cm
    apart on this rover's own map, and deduplicating seeds at half a metre gave
    them one starting point between them and therefore one entity. A seed is
    cheap and a missing seed is unrecoverable, so this only removes the same
    crossing found twice.
    """
    seeds: list[dict[str, Any]] = []
    for index, first in enumerate(rays):
        for second in rays[index + 1:]:
            if (first.get("inference_id") is not None
                    and first.get("inference_id") == second.get("inference_id")):
                continue
            if looks_like is not None and not looks_like(first, [second]):
                continue
            crossing = locate.fix(first, second)
            if crossing is None:
                continue
            if any(math.hypot(crossing["x_m"] - one["x_m"],
                              crossing["y_m"] - one["y_m"]) < SAME_SEED_M
                   for one in seeds):
                continue
            seeds.append({"x_m": crossing["x_m"], "y_m": crossing["y_m"],
                          "extent_m": crossing.get("extent_m") or 0.0,
                          "error_major_m": crossing["error_major_m"],
                          "error_minor_m": crossing["error_minor_m"],
                          "error_major_deg": crossing["error_major_deg"],
                          # What it is known to look like, which grows as rays
                          # claim it -- the same accumulation `store.add_exemplar`
                          # does for a real entity, and asked the same way: a ray
                          # is refused only if it looks like *none* of them.
                          "exemplars": [first, second]})
    return seeds


def _weigh(rays: list[dict[str, Any]], places: list[dict[str, Any]],
           looks_like=None, soft: bool = True) -> list[list[float]]:
    """How much each ray believes in each thing. The expectation step.

    Done a look at a time, because the constraint that makes this more than a
    mixture model applies to a look: **a thing may claim at most one region of
    one picture.** Within a look every feasible arrangement is enumerated and
    weighted, and a ray's belief in a thing is the share of the total weight
    carried by arrangements that pair them -- which is the exact marginal rather
    than an approximation to it.

    `soft=False` puts all of a look's weight on its single best arrangement,
    which is the max-mixture approximation and is what `bench_cluster.py`
    compares against. It is a strictly worse E-step and it is here to be measured
    rather than believed.
    """
    weights = [[0.0] * len(places) for _ in rays]
    looks: dict[Any, list[int]] = {}
    for index, ray in enumerate(rays):
        looks.setdefault(ray.get("inference_id"), []).append(index)

    for members in looks.values():
        scores: list[list[float | None]] = []
        clutter: list[float] = []
        for index in members:
            ray = rays[index]
            row: list[float | None] = []
            for place in places:
                if (looks_like is not None
                        and not looks_like(ray, place.get("exemplars") or [])):
                    row.append(None)
                    continue
                got = (_likelihood(place, ray, place.get("extent_m") or 0.0)
                       * (place.get("share") or 1.0))
                row.append(got if got > 0.0 else None)
            scores.append(row)
            # In whatever units the scores came out in, which depends on whether
            # this ray carries a range. See `CLUTTER_PER_M`.
            nothing = CLUTTER_PER_DEG * CLUTTER_PRIOR
            if ray.get("range_m") is not None:
                nothing *= CLUTTER_PER_M
            clutter.append(nothing)

        events = _arrangements(scores, clutter) if soft else None
        if events is None:
            order = _best_arrangement(scores, clutter)
            for slot, index in enumerate(members):
                if order[slot] >= 0:
                    weights[index][order[slot]] = 1.0
            continue
        total = sum(weight for weight, _order in events)
        if total <= 0.0:
            continue
        for weight, order in events:
            share = weight / total
            for slot, column in enumerate(order):
                if column >= 0:
                    weights[members[slot]][column] += share
    return weights


def _claims(weights: list[list[float]]) -> list[list[float]]:
    """Which one thing each ray is for, as weights with the runners-up removed.

    The soft weights are what the fit wants -- a ray that half believes in a
    thing should pull on it half as hard -- and what attachment wants is a
    decision. This is the decision: the best candidate for each ray, kept at its
    own weight, provided it is `CLAIM_LEAD` ahead of the next best. Everything
    else in the row goes to zero.
    """
    claimed = []
    for row in weights:
        best = second = 0.0
        winner = -1
        for column, weight in enumerate(row):
            if weight > best:
                best, second, winner = weight, best, column
            elif weight > second:
                second = weight
        keep = [0.0] * len(row)
        if winner >= 0 and best >= (1.0 + CLAIM_LEAD) * second:
            keep[winner] = best
        claimed.append(keep)
    return claimed


def _share(places: list[dict[str, Any]], weights: list[list[float]]) -> None:
    """Re-estimate how much of the pool each thing accounts for. In place.

    Scaled so the average candidate has a share of one rather than so that they
    sum to one, which is the same estimator written differently and keeps the
    numbers comparable with `CLUTTER_PER_DEG`. Normalised the usual way, adding a
    candidate would shrink every share and the hypothesis that a region is
    nothing would win by arithmetic rather than by evidence.

    **`SHARE_PRIOR` is what stops this eating everything, and without it it
    does.** The share feeds the likelihood that produced it, so a candidate a
    little below average shrinks, which shrinks its responsibilities, which
    shrinks it again -- and unchecked that is winner-take-all: measured on three
    things in a row seen from two places, all three decayed away and the answer
    was nothing at all. A pseudo-count on every candidate bounds the ratio, so a
    thing has to be genuinely unsupported to die rather than merely third best.
    It is a symmetric Dirichlet prior on the mixing weights, which is the
    ordinary way this is damped.
    """
    totals = [sum(row[column] for row in weights)
              for column in range(len(places))]
    average = (sum(totals) / len(totals)) if totals else 0.0
    for place, total in zip(places, totals):
        place["share"] = ((total + SHARE_PRIOR)
                          / (average + SHARE_PRIOR) if average > 0.0 else 1.0)


def _extent(place: dict[str, Any], rays: list[dict[str, Any]],
            weights: list[float]) -> float:
    """How wide the thing is, from the rays that claim it.

    `locate.extent_of` measures this from two observers; this is the same idea
    over however many claim it, capped the same way. It matters because it is the
    dominant term in `_spread_deg` at close range, so a wardrobe that is allowed
    to be a metre wide keeps the bearings that land on either end of it.
    """
    widest = 0.0
    for ray, weight in zip(rays, weights):
        if weight <= 0.0:
            continue
        range_m = _range_to(place["x_m"], place["y_m"], ray)
        span = float(ray.get("span_deg") or 0.0)
        widest = max(widest, 2.0 * range_m
                     * math.tan(math.radians(min(span, 90.0) / 2.0)))
    return min(locate.MAX_EXTENT_M, widest)


def _survives(place: dict[str, Any], rays: list[dict[str, Any]],
              weights: list[float]) -> str | None:
    """Why this fitted thing is not a thing, or None if it is one.

    **Every gate here is one `locate` already applies to a crossing**, asked of
    the fit's answer instead of of a pair of rays. That is the substance of the
    change and the reason it is safe: nothing has been relaxed, and the evidence
    a thing needs is still two independent places agreeing, a position inside the
    room, and a line of sight that does not pass through a wall.
    """
    claiming = [ray for ray, weight in zip(rays, weights) if weight > 0.0]
    if len(claiming) < MIN_CLAIMING:
        return (f"only {len(claiming)} bearing claims it, and a thing needs "
                f"{MIN_CLAIMING}")
    if locate.standing_places(claiming) < 2:
        return ("every ray that claims it was taken from one place, so its "
                "range was never tested")
    if place["error_major_m"] > MAX_UNCERTAINTY_M:
        return (f"the fit is good to {place['error_major_m']:.1f} m along its "
                f"own line of sight, which is not a position")
    point = (place["x_m"], place["y_m"])
    for ray in claiming:
        range_m = _range_to(place["x_m"], place["y_m"], ray)
        if range_m < locate.MIN_RANGE_M:
            return f"it sits {range_m:.2f} m from a camera that saw it"
        if range_m > locate.MAX_RANGE_M:
            return f"it sits {range_m:.1f} m away, which is outside a room"
        if locate.beyond_reach(ray, point):
            return "a camera that saw it would have been looking through a wall"
    return None


def discover(rays: list[dict[str, Any]], *, looks_like=None,
             soft: bool = True, limit: int | None = None
             ) -> list[dict[str, Any]]:
    """Everything these bearings place, fitted together.

    `rays` are what `resolve.ray_of` builds -- a pose, a bearing, what each is
    worth, and which look it came from. `looks_like(ray, others)` is asked
    whether a ray could be the same object as any of a candidate's known crops;
    it is the appearance gate, and it is a veto rather than a preference for the
    reason
    [README.md](README.md) gives at length: on this rover a twin chair scores
    *higher* than the same chair from a new angle, so appearance can say "not
    that" and can never say "that one rather than this one".

    Each answer is a placement in the shape `store.place` and
    `locate.cross_track` already read, plus `members`: the observation ids that
    claim it and how much. `why_not` on a rejected candidate is kept out of the
    answer entirely -- the caller gets what survived, and `bench_cluster.py` is
    where the rejections are looked at.

    `limit` caps how many are returned, best-evidenced first, which is the same
    restraint `resolve.MAX_NEW_PER_PASS` applies: a pass that invents fifteen
    things at once has no way to be checked by the look that follows it.
    """
    usable = [ray for ray in rays if ray.get("bearing_deg") is not None]
    if len(usable) < 2:
        return []
    places = _seeds(usable, looks_like)
    if not places:
        return []

    weights: list[list[float]] = []
    for _round in range(MAX_ROUNDS):
        weights = _weigh(usable, places, looks_like, soft)
        _share(places, weights)
        # The share is estimated from the soft responsibilities, because that is
        # what it is: how much of the pool this thing accounts for, contested
        # rays included. What the *fit* is given is the claims, so that a ray
        # with two equally good explanations moves neither of them.
        places = [place for place in places
                  if (place.get("share") or 0.0) >= MIN_SHARE]
        if not places:
            return []
        # Re-weighed against what is left, because a candidate that has just
        # gone was holding responsibility that belongs to its neighbours.
        #
        # **Soft here and claims later, which is the distinction the whole
        # module turns on.** What the fit is given is the responsibilities, so a
        # ray with two explanations pulls a little on each and both stay where
        # their own evidence puts them. Giving the fit the claims instead means
        # that while a phantom is still on the table every ray is contested,
        # nothing is claimed, nothing gets fitted at all and the answer is
        # nothing -- which is what happened.
        weights = _weigh(usable, places, looks_like, soft)
        moved = 0.0
        fitted = []
        for column, place in enumerate(places):
            column_weights = [row[column] for row in weights]
            if sum(column_weights) <= 0.0:
                continue
            extent_m = _extent(place, usable, column_weights)
            got = locate.fit_over(usable, column_weights, (place["x_m"], place["y_m"]),
                       extent_m)
            if got is None:
                continue
            moved = max(moved, math.hypot(got["x_m"] - place["x_m"],
                                          got["y_m"] - place["y_m"]))
            got["extent_m"] = extent_m
            # What it looks like travels with it, and gains whatever now claims
            # it. Without this the appearance veto would apply to the pair that
            # seeded a thing and to nothing that joined it afterwards, which is
            # the hole `_place_one` had before the run of 2026-09-03 closed it.
            got["share"] = place.get("share") or 1.0
            got["exemplars"] = list(place.get("exemplars") or [])
            for ray, weight in zip(usable, column_weights):
                if weight > 0.0 and ray not in got["exemplars"]:
                    got["exemplars"].append(ray)
            fitted.append(got)
        if not fitted:
            return []
        # Two candidates that have converged on one answer are one thing. Left
        # alone they split their own evidence between them and both then fail
        # `MIN_SUPPORT`, which is a real fault and not a tidy-up: it is how a
        # well-supported thing seeded twice disappears.
        places = _merge(fitted)
        if moved < SETTLED_M:
            break

    # **What is left is thinned one candidate at a time, and one sweep will not
    # do it.** A crossing where no object is lies exactly on rays that belong to
    # the real objects either side of it, so it explains them exactly as well as
    # they do and takes half of each: on two chairs seen from two places, the two
    # chairs and the phantom between them ended up with 1.3, 1.3 and 0.9 of a ray
    # and *none* of the three could claim two bearings. Remove the phantom and
    # each chair's share comes straight back.
    #
    # So: drop the least-supported candidate that cannot stand, recompute
    # everything, and repeat until what remains all stands. Least-supported is
    # measured by the share, which is the smoothed estimate rather than the raw
    # total. This is stepwise backward selection over the number of things, which
    # is the ordinary answer for a mixture whose component count is unknown, and
    # it costs one expectation step per candidate dropped.
    while places:
        weights = _weigh(usable, places, looks_like, soft)
        _share(places, weights)
        claimed = _claims(weights)
        doomed = None
        for column, place in enumerate(places):
            if _survives(place, usable,
                         [row[column] for row in claimed]) is None:
                continue
            share = place.get("share") or 0.0
            if doomed is None or share < doomed[1]:
                doomed = (column, share)
        if doomed is None:
            break
        places = [place for column, place in enumerate(places)
                  if column != doomed[0]]
    if not places:
        return []

    weights = _claims(_weigh(usable, places, looks_like, soft))
    out = []
    for column, place in enumerate(places):
        column_weights = [row[column] for row in weights]
        if _survives(place, usable, column_weights) is not None:
            continue
        out.append(_placement(place, usable, column_weights))
    out.sort(key=lambda one: (-one["viewpoints"], -one["rays_agreeing"],
                              one["uncertainty_m"]))
    return out if limit is None else out[:limit]


def _merge(places: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Fitted positions closer together than `SAME_PLACE_M`, as one each."""
    kept: list[dict[str, Any]] = []
    for place in sorted(places, key=lambda one: one["error_major_m"]):
        if any(math.hypot(place["x_m"] - one["x_m"],
                          place["y_m"] - one["y_m"]) < SAME_PLACE_M
               for one in kept):
            continue
        kept.append(place)
    return kept


def _placement(place: dict[str, Any], rays: list[dict[str, Any]],
               weights: list[float]) -> dict[str, Any]:
    """A fitted thing in the shape the store and the console already read.

    `uncertainty_m` is the long axis of the error ellipse, which is what the
    figure has always meant -- "to within" -- and is now a covariance rather than
    the furthest of four nudged copies. The two are close where a crossing was
    well conditioned and the ellipse is much the more honest where it was not.

    `baseline_m` and `parallax_deg` describe the widest-apart pair among the rays
    that claim it. They are no longer how the position was worked out, and they
    are kept because they are what a person means by "how well was this seen",
    and because the message the console shows is built from them.
    """
    claiming = [(ray, weight) for ray, weight in zip(rays, weights)
                if weight > 0.0]
    best = (0.0, 0.0)
    for index, (first, _w) in enumerate(claiming):
        for second, _w2 in claiming[index + 1:]:
            pair = (locate.baseline_m(first, second),
                    locate.parallax_deg(first, second))
            if pair[1] > best[1]:
                best = pair
    return {
        "x_m": round(place["x_m"], 3),
        "y_m": round(place["y_m"], 3),
        "uncertainty_m": round(place["error_major_m"], 3),
        "error_major_m": round(place["error_major_m"], 3),
        "error_minor_m": round(place["error_minor_m"], 3),
        "error_major_deg": round(place["error_major_deg"], 1),
        "extent_m": round(place.get("extent_m") or 0.0, 3),
        "baseline_m": round(best[0], 3),
        "parallax_deg": round(best[1], 1),
        "rays_agreeing": len(claiming),
        "viewpoints": locate.standing_places([ray for ray, _w in claiming]),
        "fitted_from": len([one for one in weights if one > 0.0]),
        "chi_square": round(place.get("chi_square") or 0.0, 2),
        "members": [{"observation_id": ray.get("observation_id"),
                     "inference_id": ray.get("inference_id"),
                     "weight": round(weight, 3)}
                    for ray, weight in zip(rays, weights) if weight > 0.0],
    }
