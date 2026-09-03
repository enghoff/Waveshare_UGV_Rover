"""Fitting several things out of many bearings at once.

Two halves, and they are checked separately because they fail differently. The
*estimator* -- where a thing must be for a set of rays -- is arithmetic, and what
can go wrong with it is that one bad bounding box drags the answer, or that a
bearing worth six degrees is treated as though it were worth one and a half. The
*association* -- which rays are the same thing -- is a judgement, and what can go
wrong with it is that two chairs become one, or that a crossing between two
different chairs becomes a third.

Nothing here needs a rover, a camera or a map. The rays are built by pointing
them at a known answer, so a test that fails says the arithmetic moved rather
than that the room did.
"""
from __future__ import annotations

import math

from test_fakes import a_ray
from test_harness import check
from world_state import cluster
from world_state import locate


# --- the estimator ----------------------------------------------------------

def test_clean_bearings_recover_the_point_they_were_aimed_at() -> None:
    """The base case, and it has to be exact rather than close: three rays built
    by pointing them at a place must fit that place back with nothing left over.
    A residual here would mean the bearing convention or the Jacobian is wrong,
    and every other number in this file would then be wrong by the same amount
    without looking it."""
    target = (3.0, 0.0)
    rays = [a_ray(0.0, 0.0, target), a_ray(0.0, 2.0, target),
            a_ray(0.0, -2.0, target)]
    got = locate.fit_over(rays, [1.0] * 3, (2.5, 0.3), 0.0)
    check("three bearings fit the place they were aimed at",
          (round(got["x_m"], 3), round(got["y_m"], 3)), (3.0, 0.0))
    check("...with nothing left over", round(got["chi_square"], 6), 0.0)
    check("...and the fit says how many terms it used", got["terms"], 3)

    check("a single bearing locates nothing, and says so rather than guessing",
          locate.fit_over(rays[:1], [1.0], (2.5, 0.3), 0.0), None)
    check("nor does a ray nothing believes in",
          locate.fit_over(rays, [0.0] * 3, (2.5, 0.3), 0.0), None)


def test_one_badly_drawn_box_does_not_drag_the_answer() -> None:
    """**This is the fault the fit was brought in to fix**, and the figure in
    `locate.refine` -- a worst bearing missing its own entity by 48.9 degrees
    before and 15.0 after -- is this behaviour measured on a real recording.

    A plain least-squares fit has no defence: every ray pulls with its whole
    weight, so a region drawn round a doorframe instead of the cabinet inside it
    moves the thing. The Huber loss gives such a ray a share of its pull
    proportional to how far it missed."""
    target = (3.0, 0.0)
    clean = [a_ray(0.0, 0.0, target), a_ray(0.0, 2.0, target),
             a_ray(0.0, -2.0, target)]
    got = locate.fit_over(clean, [1.0] * 3, (2.5, 0.3), 0.0)

    spoilt = clean + [a_ray(0.0, -2.0, target, off=20.0)]
    with_bad = locate.fit_over(spoilt, [1.0] * 4, (2.5, 0.3), 0.0)
    moved = math.hypot(with_bad["x_m"] - got["x_m"],
                       with_bad["y_m"] - got["y_m"])
    check("a bearing 20 degrees wrong moves the answer less than 20 cm",
          moved < 0.2, True)
    check("...because it keeps only a sixth of its pull",
          round(locate.robust_weight(with_bad["x_m"], with_bad["y_m"],
                                     spoilt[-1], 0.0), 2), 0.17)
    check("...while a bearing that agrees keeps all of it",
          locate.robust_weight(with_bad["x_m"], with_bad["y_m"],
                               spoilt[0], 0.0), 1.0)


def test_a_bearing_taken_while_turning_gets_less_of_a_say() -> None:
    """A look taken while the rover was swinging is worth what it says it is
    worth, and `sigma_of` is where that is recorded. The fit that came before
    this one weighted every ray alike, which since the shutter fix means a
    bearing good to six degrees pulled as hard as one good to one and a half."""
    target, wrong = (3.0, 0.0), (3.0, 0.6)
    good = [a_ray(0.0, 0.0, target), a_ray(0.0, 2.0, target)]

    trusted = locate.fit_over(good + [a_ray(0.0, -2.0, wrong)],
                              [1.0] * 3, (3.0, 0.0), 0.0)
    doubted = locate.fit_over(good + [a_ray(0.0, -2.0, wrong, sigma=6.0)],
                              [1.0] * 3, (3.0, 0.0), 0.0)
    check("a disagreeing ray worth 1.5 degrees pulls the answer 12 cm off",
          round(trusted["y_m"], 2), 0.12)
    check("...and the same ray worth 6.0 degrees pulls it 3 cm",
          round(doubted["y_m"], 2), 0.03)


def test_bearings_along_one_line_answer_with_a_cigar_and_not_a_point() -> None:
    """**The parallax floor, restated as what it was always about.**
    `MIN_PARALLAX_DEG` refuses two rays closer than 12 degrees because the
    crossing runs away down the line of sight. The fit is allowed to try, and
    what it reports is the running away: an error a metre long and centimetres
    wide, pointing along the bearing. Refusing it for that is
    `cluster.MAX_UNCERTAINTY_M`, and the point of doing it this way round is that
    a fit over many rays can be well conditioned where no single pair is."""
    target = (3.0, 0.0)
    along = [a_ray(0.0, 0.0, target), a_ray(0.0, 0.1, target)]
    got = locate.fit_over(along, [1.0] * 2, (2.5, 0.0), 0.0)
    check("two bearings two degrees apart are uncertain metres lengthways",
          got["error_major_m"] > 3.0, True)
    check("...and centimetres across", got["error_minor_m"] < 0.1, True)
    check("...running along the line of sight",
          abs(got["error_major_deg"]) <= 2.0, True)
    check("...which is not a position",
          got["error_major_m"] > cluster.MAX_UNCERTAINTY_M, True)

    across = [a_ray(0.0, 0.0, target), a_ray(0.0, 2.0, target),
              a_ray(0.0, -2.0, target)]
    fine = locate.fit_over(across, [1.0] * 3, (3.0, 0.0), 0.0)
    check("bearings from either side are uncertain neither way",
          fine["error_major_m"] < cluster.MAX_UNCERTAINTY_M, True)


def test_a_bearing_landing_on_a_wide_thing_is_not_a_bad_bearing() -> None:
    """A wardrobe is a metre wide and is stored as a point, so two looks from two
    sides of it centre on two different parts of it and neither is wrong. The
    allowance is the thing's own silhouette, and it is subtracted from the miss
    rather than added to the noise -- added, it would also divide the likelihood,
    and a wide thing close to the camera would then score worse than the
    hypothesis that the region is nothing at all."""
    at = (3.0, 0.0)
    ray = a_ray(0.0, 0.0, at, span=20.0, off=6.0)
    check("a three-quarter-metre thing at three metres is seven degrees wide",
          round(locate.silhouette_deg(3.0, 0.0, ray, 0.75), 1), 7.1)
    check("the bearing itself is still worth a degree and a half",
          round(locate.noise_deg(3.0, 0.0, ray), 2), 1.5)
    check("a bearing six degrees off that thing is pointing at it",
          locate.robust_weight(3.0, 0.0, ray, 0.75), 1.0)
    check("...where against a point it would be half suspect",
          round(locate.robust_weight(3.0, 0.0, ray, 0.0), 2), 0.5)


def test_a_range_would_enter_the_fit_as_one_more_residual() -> None:
    """**Nothing on this rover writes a range yet**, and this is here because the
    whole argument for fitting positions rather than crossing pairs is that a
    range costs one residual and no restructuring. An argument like that should
    be checked against the code rather than believed.

    The depth camera on the front of this rover serves stereo depth on loopback
    8770 and nothing reads it; when something does, a ray gains `range_m` and
    this is the path it takes."""
    at = (3.0, 0.0)
    bearing_only = locate.residuals(3.0, 0.0, a_ray(0.0, 0.0, at))
    check("a bearing is one residual", len(bearing_only), 1)

    with_range = dict(a_ray(0.0, 0.0, at), range_m=3.0)
    check("a bearing and a range are two", len(locate.residuals(3.0, 0.0,
                                                                with_range)), 2)
    check("...and a range that agrees leaves nothing over",
          round(locate.residuals(3.0, 0.0, with_range)[1][0], 6), 0.0)

    short = dict(a_ray(0.0, 0.0, at), range_m=2.85)
    off = locate.residuals(3.0, 0.0, short)[1][0]
    check("a range 15 cm short is one standard deviation of it",
          round(off, 2), round(0.15 / locate.RANGE_SIGMA_M, 2))

    # And it pulls: two bearings that cross badly plus a range that does not.
    along = [a_ray(0.0, 0.0, at), a_ray(0.0, 0.1, at)]
    ranged = [dict(along[0], range_m=2.0), along[1]]
    got = locate.fit_over(ranged, [1.0] * 2, (3.0, 0.0), 0.0)
    check("a range drags a lengthways-uncertain fix down its own line of sight",
          got["x_m"] < 2.6, True)
    check("...and the fix is no longer uncertain lengthways",
          got["error_major_m"] < 1.0, True)


# --- the association --------------------------------------------------------

def test_two_things_seen_from_two_places_come_back_as_two() -> None:
    """The base case for the whole module. Two objects, two looks from two places
    far enough apart, one region each per look -- and the answer has to be two
    things where they actually are, not one thing between them and not four."""
    first, second = (3.0, 1.0), (3.0, -1.0)
    rays = [a_ray(0.0, 0.0, first, look=1, observation=1),
            a_ray(0.0, 0.0, second, look=1, observation=2),
            a_ray(0.0, 2.5, first, look=2, observation=3),
            a_ray(0.0, 2.5, second, look=2, observation=4)]
    got = cluster.discover(rays)
    check("two objects come back as two", len(got), 2)
    places = sorted((one["x_m"], one["y_m"]) for one in got)
    check("...at the places they were put",
          [(round(x, 1), round(y, 1)) for x, y in places],
          [(3.0, -1.0), (3.0, 1.0)])
    check("...each agreed by two bearings",
          sorted(one["rays_agreeing"] for one in got), [2, 2])
    check("...from two places", sorted(one["viewpoints"] for one in got), [2, 2])


def test_a_thing_may_not_take_two_regions_of_one_picture() -> None:
    """The region finder's own suppression means two regions of one frame are two
    different objects, so an arrangement that gives one thing both of them is not
    merely unlikely, it is impossible. `_arrangements` enumerates only the ones
    that respect it."""
    scores = [[1.0, 0.5], [1.0, 0.5]]
    clutter = [0.01, 0.01]
    events = cluster._arrangements(scores, clutter)
    check("no arrangement gives one thing both regions",
          any(order[0] == order[1] != -1 for _w, order in events), False)
    check("both regions being nothing is still an arrangement",
          any(order == (-1, -1) for _w, order in events), True)
    check("...as is one thing each, either way round",
          sorted(order for _w, order in events if -1 not in order),
          [(0, 1), (1, 0)])

    check("a look with more arrangements than the cap says so rather than hangs",
          cluster._arrangements(scores, clutter, limit=1), None)


def test_the_crossing_between_two_different_chairs_is_not_a_third_chair() -> None:
    """**The phantom, and it is the reason support is counted rather than
    assumed.** Two identical chairs seen from two places give four rays and four
    geometrically sound crossings: the two chairs, and two places where a ray to
    one chair happens to cross a ray to the other. Appearance cannot break the
    tie on this rover -- a twin chair scores higher than the same chair from a new
    angle -- so what has to break it is that a phantom's whole support is
    borrowed from rays the real chairs fit better."""
    first, second = (3.0, 1.2), (3.0, -1.2)
    rays = [a_ray(0.0, 0.0, first, look=1, observation=1),
            a_ray(0.0, 0.0, second, look=1, observation=2),
            a_ray(0.0, 3.0, first, look=2, observation=3),
            a_ray(0.0, 3.0, second, look=2, observation=4),
            a_ray(1.0, -2.0, first, look=3, observation=5),
            a_ray(1.0, -2.0, second, look=3, observation=6)]
    got = cluster.discover(rays)
    check("three looks at two chairs place two things", len(got), 2)
    for one in got:
        check("...and each is agreed from three places", one["viewpoints"], 3)
    check("no thing is placed between them",
          any(abs(one["y_m"]) < 0.6 for one in got), False)


def test_a_thing_nothing_looked_at_twice_is_not_placed() -> None:
    """Rays from one standstill share an origin and cross nowhere, so however
    many of them agree, the range was never tested. `standing_places` is the
    count that matters and `locate.MIN_BASELINE_M` is where it draws the line --
    the same line `locate.fix` draws when it refuses to cross two such rays."""
    at = (3.0, 0.0)
    parked = [a_ray(0.0, 0.0, at, look=look, observation=look)
              for look in range(1, 5)]
    check("four looks from one place place nothing",
          cluster.discover(parked), [])

    shuffled = [a_ray(0.0, 0.0, at, look=1, observation=1),
                a_ray(0.0, 0.2, at, look=2, observation=2),
                a_ray(0.0, 0.35, at, look=3, observation=3)]
    check("...and nor do three from within a handspan of each other",
          cluster.discover(shuffled), [])


def test_appearance_can_refuse_a_ray_and_can_never_prefer_one() -> None:
    """On this rover appearance says "not that" and cannot say "that one rather
    than this one": measured, one chair across a change of viewpoint scores 0.696
    and its twin scores 0.735. So the gate is a veto -- it removes a pairing the
    geometry would have made, and it never chooses between two the geometry
    allows."""
    first, second = (3.0, 1.0), (3.0, -1.0)
    rays = [a_ray(0.0, 0.0, first, look=1, observation=1),
            a_ray(0.0, 0.0, second, look=1, observation=2),
            a_ray(0.0, 2.5, first, look=2, observation=3),
            a_ray(0.0, 2.5, second, look=2, observation=4)]
    check("with nothing refused, both things are placed",
          len(cluster.discover(rays)), 2)

    def nothing_matches(_ray, others):
        return not others

    check("a veto on every pairing places nothing at all",
          cluster.discover(rays, looks_like=nothing_matches), [])

    refused = {1, 3}

    def not_the_first_thing(ray, others):
        mine = ray.get("observation_id") in refused
        return all((one.get("observation_id") in refused) == mine
                   for one in others)

    got = cluster.discover(rays, looks_like=not_the_first_thing)
    check("a veto that splits the pool the way the geometry does changes nothing",
          len(got), 2)


def test_a_region_may_be_nothing_at_all() -> None:
    """Most regions in a real room are wall, floor, ceiling or doorway, and a
    formulation with no way to say so has to put every one of them on the nearest
    thing. `CLUTTER_PER_DEG` is the alternative hypothesis and this is the check
    that it wins where it should."""
    at = (3.0, 0.0)
    rays = [a_ray(0.0, 0.0, at, look=1, observation=1),
            a_ray(0.0, 2.5, at, look=2, observation=2),
            # pointing off into the room, at nothing anything else saw
            a_ray(0.0, 2.5, (0.5, 6.0), look=2, observation=3)]
    got = cluster.discover(rays)
    check("the thing two bearings agree on is placed", len(got), 1)
    check("...from those two bearings and not from three",
          got[0]["rays_agreeing"], 2)
    joined = {one["observation_id"] for one in got[0]["members"]}
    check("...and the bearing that was aimed at nothing joins nothing",
          3 in joined, False)


def test_soft_weights_and_a_single_arrangement_are_both_available() -> None:
    """The two E-steps, because which of them is better is a measurement and not
    an opinion. Soft weights are the exact marginals over every feasible
    arrangement; the hard version puts everything on the most likely one, which
    is the max-mixture approximation and is what `resolve._by_look` computes.
    On clean data they agree, and that agreement is what makes the comparison on
    a recording meaningful."""
    first, second = (3.0, 1.0), (3.0, -1.0)
    rays = [a_ray(0.0, 0.0, first, look=1, observation=1),
            a_ray(0.0, 0.0, second, look=1, observation=2),
            a_ray(0.0, 2.5, first, look=2, observation=3),
            a_ray(0.0, 2.5, second, look=2, observation=4)]
    soft = cluster.discover(rays, soft=True)
    hard = cluster.discover(rays, soft=False)
    check("both ways of weighting place the same two things",
          len(soft), len(hard))
    check("...in the same places",
          [(round(one["x_m"], 1), round(one["y_m"], 1)) for one in soft],
          [(round(one["x_m"], 1), round(one["y_m"], 1)) for one in hard])


def test_three_things_in_a_row_need_a_third_place_to_look_from() -> None:
    """**The limit of bearings-only association, and it is a real one rather than
    a shortcoming of the arithmetic.** Three objects in a line seen from two
    places have two globally consistent explanations, because a crossing between
    the ray to one object and the ray to another lies exactly on both -- there is
    nothing in the angles to prefer the true arrangement. So from two places this
    places a thing where no thing is, which is the honest behaviour of the model
    and is written down here rather than left to be discovered.

    A third place to look from settles it, and `resolve._place_one` says the same
    of the same situation: "from two viewpoints the answer is genuinely not
    knowable, and the honest outcome is to wait rather than to guess"."""
    row = ((3.0, 2.0), (3.0, 0.0), (3.0, -2.0))

    def looks_from(*origins):
        rays = []
        for index, at in enumerate(row):
            for which, (x_m, y_m) in enumerate(origins):
                rays.append(a_ray(x_m, y_m, at, look=which + 1,
                                  observation=index * 10 + which))
        return rays

    from_two = cluster.discover(looks_from((0.0, 0.0), (0.0, 3.0)))
    check("from two places, three things in a row are not three things",
          len(from_two), 1)
    check("...and what it does place is not one of them",
          [(round(one["x_m"], 1), round(one["y_m"], 1)) for one in from_two],
          [(1.8, 1.2)])

    from_three = cluster.discover(
        looks_from((0.0, 0.0), (0.0, 3.0), (1.5, -2.5)))
    check("a third place to look from settles it",
          sorted((round(one["x_m"], 1), round(one["y_m"], 1))
                 for one in from_three),
          [(3.0, -2.0), (3.0, 0.0), (3.0, 2.0)])


def test_a_range_on_each_ray_settles_what_two_places_cannot() -> None:
    """**The measurement the whole choice of formulation rests on.** The reason
    for fitting positions with soft associations rather than searching over pairs
    is that a range costs one residual and no restructuring -- and the payoff is
    not a better position, it is that the association stops being ambiguous. Two
    bearings crossing where no object is fit that crossing exactly as well as
    they fit the real objects either side of it; a ray that also says how far
    along itself the thing sits agrees with one and not the other.

    The depth camera on the front of this rover serves stereo depth on loopback
    8770 and nothing reads it. This is what reading it would buy."""
    row = ((3.0, 2.0), (3.0, 0.0), (3.0, -2.0))
    rays = []
    for index, at in enumerate(row):
        for which, (x_m, y_m) in enumerate(((0.0, 0.0), (0.0, 3.0))):
            ray = a_ray(x_m, y_m, at, look=which + 1,
                        observation=index * 10 + which)
            ray["range_m"] = math.hypot(at[0] - x_m, at[1] - y_m)
            rays.append(ray)
    got = cluster.discover(rays)
    check("with a range on every ray, two places are enough for three things",
          sorted((round(one["x_m"], 1), round(one["y_m"], 1)) for one in got),
          [(3.0, -2.0), (3.0, 0.0), (3.0, 2.0)])
    check("...and no phantom survives beside them", len(got), 3)


def test_a_pass_may_not_invent_everything_it_can_see() -> None:
    """`limit` is the same restraint `resolve.MAX_NEW_PER_PASS` applies: a pass
    that places everything leaves the look that follows it nothing to check.
    Best-evidenced first, so what waits is what was least sure."""
    rays = []
    for index, at in enumerate(((3.0, 2.0), (3.0, 0.0), (3.0, -2.0))):
        for which, (x_m, y_m) in enumerate(((0.0, 0.0), (0.0, 3.0),
                                            (1.5, -2.5))):
            rays.append(a_ray(x_m, y_m, at, look=which + 1,
                              observation=index * 10 + which))
    check("three things are there to be found", len(cluster.discover(rays)), 3)
    got = cluster.discover(rays, limit=2)
    check("...and a capped pass takes two of them", len(got), 2)
    check("...the best evidenced first",
          got[0]["viewpoints"] >= got[1]["viewpoints"], True)


TESTS = (
    test_clean_bearings_recover_the_point_they_were_aimed_at,
    test_one_badly_drawn_box_does_not_drag_the_answer,
    test_a_bearing_taken_while_turning_gets_less_of_a_say,
    test_bearings_along_one_line_answer_with_a_cigar_and_not_a_point,
    test_a_bearing_landing_on_a_wide_thing_is_not_a_bad_bearing,
    test_a_range_would_enter_the_fit_as_one_more_residual,
    test_two_things_seen_from_two_places_come_back_as_two,
    test_a_thing_may_not_take_two_regions_of_one_picture,
    test_the_crossing_between_two_different_chairs_is_not_a_third_chair,
    test_a_thing_nothing_looked_at_twice_is_not_placed,
    test_appearance_can_refuse_a_ray_and_can_never_prefer_one,
    test_a_region_may_be_nothing_at_all,
    test_soft_weights_and_a_single_arrangement_are_both_available,
    test_three_things_in_a_row_need_a_third_place_to_look_from,
    test_a_range_on_each_ray_settles_what_two_places_cannot,
    test_a_pass_may_not_invent_everything_it_can_see,
)
