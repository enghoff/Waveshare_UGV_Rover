"""Geometry: turning observations into bearings, and bearings into a place.

A bearing carries no range, so two of them cross somewhere whatever they were
aimed at. What is checked is that a crossing is only believed when the baseline
earns it, and that a rover which only turned on the spot places nothing.
"""
from __future__ import annotations

import math

from test_fakes import a_box
from test_harness import check
from world_state import locate
from world_state import view


# --- what gets drawn --------------------------------------------------------

def test_an_observation_becomes_a_bearing_from_a_measured_pose() -> None:
    """The gimbal takes pan positive to the right; the map takes bearings positive
    to the left. That minus sign is the whole of the conversion, and getting it
    backwards draws a perfectly ordinary ray over the wrong half of the room."""
    observation = {"pose": {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0},
                   "observer_pan_deg": 30.0, "bbox": [0.4, 0.2, 0.6, 0.8]}
    drawn = view.ray(observation, fov_deg=130.0)
    check("the ray starts where the rover was", (drawn["x_m"], drawn["y_m"]),
          (1.0, 2.0))
    check("a gimbal turned right of the nose points right of the heading",
          drawn["bearing_deg"], 59.2)
    check("...and a box the width of a fifth of the picture is that wide",
          drawn["span_deg"], 25.3)

    left = dict(observation, bbox=[0.0, 0.2, 0.2, 0.8])
    check("something at the left of the picture is further to the left again",
          view.ray(left, fov_deg=130.0)["bearing_deg"], 110.6)

    check("no pose means no ray, rather than a ray from the origin",
          view.ray(dict(observation, pose=None), 130.0), None)
    check("no gimbal angle means no ray either",
          view.ray(dict(observation, observer_pan_deg=None), 130.0), None)
    check("a box that is missing still leaves the camera direction",
          view.ray(dict(observation, bbox=None), 130.0)["bearing_deg"], 60.0)


def test_the_bearing_comes_through_the_lens_the_gimbal_is_aimed_with() -> None:
    """**The fault this replaced put 184 of one drive's 441 boxes outside the
    accuracy `locate` is promised.** A box's horizontal position times a field of
    view is only the right angle along the two centre lines of a 130-degree
    fisheye, and the tilt the gimbal was holding -- recorded on every observation
    -- was thrown away entirely. Both are wrong in the same place: high in the
    frame, which is where things on walls are."""
    check("a point on the lens axis is straight ahead whatever the tilt",
          [round(view.azimuth_deg(315.9 / 640, 227.4 / 480, tilt), 3)
           for tilt in (0.0, 10.0, 30.0)], [0.0, 0.0, 0.0])

    # Down one column near the left edge, tilting the camera up by 30 degrees.
    # The old multiplication said all three of these were the same angle and that
    # the tilt did not enter into it.
    moved = [round(view.azimuth_deg(0.1, cy, 30.0)
                   - view.azimuth_deg(0.1, cy, 0.0), 1)
             for cy in (0.1, 227.4 / 480, 0.9)]
    check("tilting the camera up swings a bearing high in the picture wide",
          moved[0], 21.4)
    check("...swings one low in the picture the other way",
          moved[2], -12.2)
    check("...and moves one on the axis row least of the three",
          min(abs(one) for one in moved) == abs(moved[1]), True)

    corner = {"pose": {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0},
              "observer_pan_deg": 0.0, "observer_tilt_deg": 30.0,
              "bbox": [0.05, 0.05, 0.15, 0.15]}
    check("and the tilt reaches the bearing the store writes",
          view.ray(corner, 130.0)["bearing_deg"],
          view.ray(dict(corner, observer_tilt_deg=0.0), 130.0)["bearing_deg"]
          + 21.4)


def test_a_bearing_the_rover_measured_is_not_recomputed() -> None:
    """A lens refitted today must not rewrite what the rover measured a month
    ago, and the console must draw the same sight line the resolver matches on:
    until this held, the page redrew old looks through the new model while
    `resolve.ray_of` went on reading the old number off the row."""
    observation = {"pose": {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0},
                   "observer_pan_deg": 0.0, "bbox": [0.4, 0.2, 0.6, 0.8],
                   "bearing_deg": -175.0, "span_deg": 12.0}
    drawn = view.ray(observation, fov_deg=130.0)
    check("the stored bearing is what comes back", drawn["bearing_deg"], -175.0)
    check("...and its span with it", drawn["span_deg"], 12.0)
    check("a row with no stored bearing is worked out from the box",
          view.ray({k: v for k, v in observation.items()
                    if k not in ("bearing_deg", "span_deg")},
                   130.0)["bearing_deg"], -0.8)


def test_the_rays_of_one_entity_are_bounded_and_oldest_first() -> None:
    observations = [{"pose": {"x_m": float(n), "y_m": 0.0, "heading_deg": 0.0},
                     "observer_pan_deg": 0.0, "bbox": None, "observed_at": n}
                    for n in range(10)]
    drawn = view.rays(observations, 130.0, limit=4)
    check("only the newest few are drawn", len(drawn), 4)
    check("...oldest of those first, so the newest is on top",
          [one["observed_at"] for one in drawn], [3, 2, 1, 0])


def test_a_look_is_related_to_the_position_the_thing_settled_on() -> None:
    """What the map draws once a thing has a position: not six stubs of the same
    length, but how each look stands to the one place it was settled at."""
    observation = {"id": 4, "pose": {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0},
                   "observer_pan_deg": 0.0, "bbox": a_box(0.10, 0.20)}
    drawn = view.ray(observation, fov_deg=130.0)
    check("the rover's own facing rides along, not only the camera's",
          (drawn["heading_deg"], drawn["pan_deg"]), (0.0, 0.0))

    place = {"x_m": 3.0, "y_m": 0.0, "uncertainty_m": 0.1,
             "error_major_m": 0.1, "error_minor_m": 0.05,
             "error_major_deg": 0.0, "extent_m": 0.2}
    dead_on = view.relate(place, drawn)
    check("a bearing straight at it misses by nothing",
          (dead_on["range_m"], dead_on["off_deg"], dead_on["miss_m"]),
          (3.0, 0.0, 0.0))
    check("...and agrees", dead_on["agrees"], True)

    # The same look, with the thing settled well off the bearing it measured.
    away = view.relate({**place, "y_m": 3.0}, drawn)
    check("a thing 45 degrees off this bearing is reported as that far off",
          away["off_deg"], -45.0)
    check("...and the miss is measured across the line of sight, in metres",
          away["miss_m"], 4.24)
    check("...which is far outside what the resolver allows", away["agrees"],
          False)

    check("an entity with no position has nothing to relate to",
          view.relate(None, drawn), None)


def test_the_rays_of_a_placed_thing_carry_the_relation() -> None:
    """The page must not work this out for itself: whether a look points at the
    thing is the resolver's decision, and a second copy of it can disagree."""
    observations = [{"id": n, "pose": {"x_m": 0.0, "y_m": float(n),
                                       "heading_deg": 0.0},
                     "observer_pan_deg": 0.0, "bbox": None, "observed_at": n}
                    for n in range(3)]
    place = {"x_m": 4.0, "y_m": 0.0, "uncertainty_m": 0.1,
             "error_major_m": 0.1, "error_minor_m": 0.1,
             "error_major_deg": 0.0, "extent_m": 0.2}
    plain = view.rays(observations, 130.0, limit=3)
    check("without a placement a ray is a direction and nothing more",
          [one["relation"] for one in plain], [None, None, None])
    related = view.rays(observations, 130.0, limit=3, placement=place)
    check("the look taken from where the thing lies agrees",
          related[-1]["relation"]["agrees"], True)
    check("...and the ones taken from further along the wall do not",
          [one["relation"]["agrees"] for one in related[:-1]], [False, False])
    check("each carries its observation, so a row can be joined to its sighting",
          [one["id"] for one in related], [2, 1, 0])


def _look(x_m, y_m, bearing_deg):
    return {"x_m": x_m, "y_m": y_m, "bearing_deg": bearing_deg}


def test_two_bearings_from_two_places_locate_a_thing() -> None:
    """The sofa is at (3, 3). The rover sees it from two corners."""
    # From the origin it is at 45 degrees; from (6, 0) it is at 135.
    found = locate.fix(_look(0.0, 0.0, 45.0), _look(6.0, 0.0, 135.0))
    check("two crossing bearings give a point", found is not None, True)
    check("...where the thing actually is, in x", round(found["x_m"]), 3)
    check("...and in y", round(found["y_m"]), 3)
    check("...with the baseline it was measured over",
          round(found["baseline_m"]), 6)
    check("...and an honest uncertainty attached",
          found["uncertainty_m"] > 0.0, True)


def test_turning_on_the_spot_locates_nothing() -> None:
    """The failure mode of the first experiment: every ray from one point."""
    check("rays from one place do not meet anywhere useful",
          locate.fix(_look(0.0, 0.0, 40.0), _look(0.0, 0.0, 50.0)), None)
    check("...nor do rays from a place barely different",
          locate.fix(_look(0.0, 0.0, 40.0), _look(0.1, 0.0, 50.0)), None)


def test_two_looks_along_the_same_line_are_one_look() -> None:
    check("no parallax, no fix",
          locate.fix(_look(0.0, 0.0, 45.0), _look(1.0, 1.0, 45.0)), None)
    check("...and a thing behind the rover was not seen",
          locate.fix(_look(0.0, 0.0, -135.0), _look(6.0, 0.0, -45.0)), None)


def test_uncertainty_grows_when_the_baseline_shrinks() -> None:
    """Why the rover has to drive rather than shuffle.

    The same thing at (3, 3) seen from the same first place, with the second look
    taken 1.5 m away and then 6 m away.
    """
    short = locate.fix(_look(0.0, 0.0, 45.0), _look(1.5, 0.0, 63.43))
    long = locate.fix(_look(0.0, 0.0, 45.0), _look(6.0, 0.0, 135.0))
    check("a short baseline still gives a fix", short is not None, True)
    check("...on the same thing", round(short["x_m"]), 3)
    check("...but a much less certain one",
          short["uncertainty_m"] > long["uncertainty_m"] * 2, True)


def test_two_identical_chairs_stay_two_things() -> None:
    """The case no model can answer from one picture, and geometry answers easily."""
    chair_a = locate.fix(_look(0.0, 0.0, 45.0), _look(6.0, 0.0, 135.0))
    # A second chair four metres away, seen from the same two places.
    chair_b = locate.fix(_look(0.0, 0.0, 8.5), _look(6.0, 0.0, 172.9))
    apart = ((chair_a["x_m"] - chair_b["x_m"]) ** 2
             + (chair_a["y_m"] - chair_b["y_m"]) ** 2) ** 0.5
    check("both chairs get a position", bool(chair_a and chair_b), True)
    check("...far enough apart to be told apart",
          apart > chair_a["uncertainty_m"] + chair_b["uncertainty_m"], True)
    check("a new look at the first chair agrees with the first chair",
          locate.agrees(chair_a, _look(0.0, 3.0, 0.0)), True)
    check("...and does not agree with the second",
          locate.agrees(chair_b, _look(0.0, 3.0, 0.0)), False)


def test_a_fix_on_top_of_the_camera_is_not_a_thing_in_the_room() -> None:
    """**The fault the validation drive of 2026-09-02 found, in its own numbers.**

    Driven between three places, the rover placed six things and every one of them
    landed between 0.13 and 0.59 m from the nearest camera that saw it. That is
    not a floor lamp; the rover would have driven into it, and no crop of anything
    thirteen centimetres from that lens survives the region filter.

    The cause is that two rays pointing *inward* from two nearby places cross in
    the gap between them, at a healthy parallax angle, so neither the baseline
    guard nor the parallax guard catches it -- and because a nudge of a degree and
    a half barely moves a point a quarter of a metre away, such a crossing reports
    a tiny uncertainty and wins the resolver's ranking against every real thing in
    the room.

    The four cases below are the rover's own recorded rays.
    """
    from_rover = [
        # what it called it, where it stood, the bearing, where it put it
        ("a ceiling light", (-0.134, 1.347), -23.2, (0.364, 1.346), -147.8),
        ("a wall", (-0.134, 1.347), -49.3, (0.364, 1.346), -104.0),
        ("a floor lamp", (0.304, 1.276), -174.8, (-0.812, 1.076), 134.6),
        ("a houseplant", (0.304, 1.276), -176.4, (-0.812, 1.076), 112.1),
    ]
    for label, first, first_deg, second, second_deg in from_rover:
        found = locate.fix(_look(first[0], first[1], first_deg),
                           _look(second[0], second[1], second_deg))
        check(f"{label!r} was not really that close to the camera", found, None)

    # And the guard must not cost anything real. The furthest apart the rover got
    # on that drive was 1.73 m, and from those two places a thing three metres out
    # in the room is still placed.
    here, there = (-0.856, 1.065), (0.857, 0.838)
    thing = (0.0, 4.0)
    real = locate.fix(
        _look(here[0], here[1],
              math.degrees(math.atan2(thing[1] - here[1], thing[0] - here[0]))),
        _look(there[0], there[1],
              math.degrees(math.atan2(thing[1] - there[1], thing[0] - there[0]))))
    check("a thing out in the room is still placed", real is not None, True)
    if real:
        check("...where it actually is",
              math.dist((real["x_m"], real["y_m"]), thing) < 0.1, True)
        check("...well clear of both cameras",
              min(math.dist((real["x_m"], real["y_m"]), here),
                  math.dist((real["x_m"], real["y_m"]), there))
              > locate.MIN_RANGE_M, True)


def test_a_bearing_at_the_edge_of_a_television_still_points_at_it() -> None:
    """An entity is stored as a point, but a television is a metre wide.

    Two looks from different sides of one centre on different parts of it, so a
    bearing that lands within the thing's own silhouette is pointing at it. With
    only the bearing error and the placement error, matching a television at two
    and a half metres allowed 0.115 m -- a tenth of the television -- and the
    looks that should have joined it made a second one instead.
    """
    telly = {"x_m": 0.0, "y_m": 2.5, "uncertainty_m": 0.05}
    # Standing at the origin, looking north. A twenty-degree region, which is
    # what the rover actually recorded for its televisions.
    edge = {"x_m": 0.0, "y_m": 0.0, "span_deg": 20.0,
            "bearing_deg": math.degrees(math.atan2(2.5, 0.3))}
    check("a bearing at the edge of it misses if it is treated as a point",
          locate.agrees(telly, edge), False)
    check("...but is pointing at it once its width counts",
          locate.agrees(telly, edge, locate.match_tolerance(telly, edge)), True)

    # And the width may not be used to swallow the room.
    across = {"x_m": 0.0, "y_m": 0.0, "span_deg": 20.0,
              "bearing_deg": math.degrees(math.atan2(2.5, 2.0))}
    check("something two metres to the side is still not the television",
          locate.agrees(telly, across, locate.match_tolerance(telly, across)),
          False)
    wall = {"x_m": 0.0, "y_m": 0.0, "span_deg": 90.0, "bearing_deg": 0.0}
    check("and a region filling the frame is capped rather than boundless",
          locate.match_tolerance(telly, wall)
          <= 2.5 * math.tan(math.radians(locate.BEARING_SIGMA_DEG)) + 0.05
          + locate.MAX_EXTENT_M + 1e-9, True)


def test_the_best_pair_places_the_thing() -> None:
    looks = [_look(0.0, 0.0, 45.0), _look(0.2, 0.0, 44.0),
             _look(6.0, 0.0, 135.0)]
    best = locate.best_fix(looks)
    check("a fix is found among several looks", best is not None, True)
    check("...using the pair with the longest useful baseline",
          round(best["baseline_m"]), 6)
    check("one look alone places nothing", locate.best_fix(looks[:1]), None)


def test_a_shallow_crossing_is_uncertain_lengthways_and_not_sideways() -> None:
    """**The error is a cigar, and it used to be charged as a disc.**

    Two bearings meeting at a shallow angle put a thing somewhere along their
    common line of sight and pin it well across that line. `uncertainty_m` is the
    length of the cigar, which is the right thing to tell a person and the wrong
    thing to add to a tolerance measured *across* a later bearing: a ray looking
    down the length of the error was being charged for all of it.
    """
    # Both looking at (6, 5), from two places 2.2 m apart along one wall.
    first = {"x_m": 0.0, "y_m": 0.0, "span_deg": 10.0,
             "bearing_deg": math.degrees(math.atan2(5.0, 6.0))}
    second = {"x_m": 2.2, "y_m": 0.0, "span_deg": 10.0,
              "bearing_deg": math.degrees(math.atan2(5.0, 3.8))}
    found = locate.fix(first, second)
    check("a shallow pair still places something", found is not None, True)
    check("...at a parallax barely over the floor",
          found["parallax_deg"] < locate.MIN_PARALLAX_DEG + 3.0, True)
    check("...and the error is far longer than it is wide",
          found["error_major_m"] > found["error_minor_m"] * 3.0, True)
    check("...with the long axis reported as a direction",
          -180.0 < found["error_major_deg"] <= 180.0, True)

    # A ray arriving from where the two founders stood looks along the length of
    # the error; one from the side looks across it.
    lengthways = {"x_m": 1.1, "y_m": 0.0,
                  "bearing_deg": math.degrees(math.atan2(5.0, 4.9))}
    sideways = {"x_m": found["x_m"] + 4.0, "y_m": found["y_m"] - 4.0,
                "bearing_deg": 135.0}
    check("looking along the error, only its width counts",
          locate.cross_track(found, lengthways) < found["uncertainty_m"] / 5.0,
          True)
    check("...and looking across it, close to all of it",
          locate.cross_track(found, sideways)
          > found["uncertainty_m"] * 0.9, True)

    check("a placement written before the shape was recorded keeps the radius",
          locate.cross_track({"x_m": 0.0, "y_m": 0.0, "uncertainty_m": 0.4},
                             {"x_m": 1.0, "y_m": 0.0}), 0.4)


def test_a_thing_may_not_claim_a_cone_the_geometry_never_earned() -> None:
    """**The fault of 2026-09-03, in the one number that caused it.**

    `object:1` was founded on a crossing at 13.9 degrees of parallax and carried
    0.464 m of uncertainty. Charged whole, that let a bearing two metres away sit
    22 degrees off and still count as pointing at it -- a cone 46 degrees wide
    against a bearing the geometry believes to a degree and a half. Twenty-six
    crops of a cabinet, two framed pictures, a doorway, a table and a person's
    head went into one thing through it.
    """
    placed = locate.fix({"x_m": 0.733, "y_m": -0.113, "bearing_deg": 46.1,
                         "span_deg": 24.0},
                        {"x_m": 0.060, "y_m": -0.096, "bearing_deg": 34.0,
                         "span_deg": 18.2})
    check("the rover's own founding pair still places the thing",
          placed is not None, True)
    check("...at the shallow parallax it was taken at",
          placed["parallax_deg"] < 15.0, True)

    ray = {"x_m": 0.075, "y_m": -0.123, "bearing_deg": 20.0, "span_deg": 8.0}
    range_m = math.dist((placed["x_m"], placed["y_m"]),
                        (ray["x_m"], ray["y_m"]))
    cone_deg = math.degrees(math.atan(
        locate.match_tolerance(placed, ray) / range_m))
    check("the cone a later bearing is allowed is not tens of degrees",
          cone_deg < 15.0, True)

    whole = (range_m * math.tan(math.radians(locate.BEARING_SIGMA_DEG))
             + placed["uncertainty_m"]
             + min(locate.MAX_EXTENT_M, placed["extent_m"]))
    was_deg = math.degrees(math.atan(whole / range_m))
    check("...where charging the whole radius allowed more than twice that",
          was_deg > cone_deg * 2.0, True)


def test_a_ray_that_started_somewhere_uncertain_says_so() -> None:
    """A look taken while the rover was driving. The bearing is as good as ever;
    where it started from is not, and the answer has to carry that rather than
    the look being thrown away for it."""
    still = locate.fix(_look(0.0, 0.0, 45.0), _look(6.0, 0.0, 135.0))
    moving = locate.fix(dict(_look(0.0, 0.0, 45.0), origin_sigma_m=0.09),
                        dict(_look(6.0, 0.0, 135.0), origin_sigma_m=0.06))
    check("the crossing is in the same place", 
          (round(moving["x_m"]), round(moving["y_m"])),
          (round(still["x_m"]), round(still["y_m"])))
    check("...and both rays' doubt about where they began is charged to it",
          round(moving["uncertainty_m"] - still["uncertainty_m"], 3), 0.15)
    check("...across the answer as well as along it, because a shifted ray "
          "moves the crossing in no particular direction",
          round(moving["error_minor_m"] - still["error_minor_m"], 3), 0.15)
    check("a ray that says nothing about it is a ray that was not moving",
          locate.fix(_look(0.0, 0.0, 45.0),
                     dict(_look(6.0, 0.0, 135.0), origin_sigma_m=None)),
          still)

    placed = dict(still, extent_m=0.0)
    ray = _look(0.0, -1.0, 76.0)
    check("and a bearing may miss by as much as its own start is unknown by",
          round(locate.match_tolerance(placed, dict(ray, origin_sigma_m=0.12))
                - locate.match_tolerance(placed, ray), 3), 0.12)


def test_a_placement_says_how_much_stands_behind_it() -> None:
    """Ten rays from one doorway and two from opposite sides of a room are not
    the same evidence, and a count of observations cannot tell them apart."""
    thing = (3.0, 3.0)

    def toward(x_m, y_m, look):
        return {"x_m": x_m, "y_m": y_m, "inference_id": look,
                "bearing_deg": round(math.degrees(
                    math.atan2(thing[1] - y_m, thing[0] - x_m)), 3)}

    # Three looks from one standstill and one from across the room.
    rays = [toward(0.0, 0.0, look) for look in (1, 2, 3)] + [toward(6.0, 0.0, 4)]
    found = locate.best_fix(rays)
    check("every ray that agrees is counted", found["rays_agreeing"], 4)
    check("...but standing still and looking again is not a second place to "
          "have looked from", found["viewpoints"], 2)

    # Now the rover walks, and each look is somewhere else.
    walked = [toward(x, 0.0, look) for look, x in enumerate((0.0, 2.0, 4.0, 6.0))]
    found = locate.best_fix(walked)
    check("driving between the looks is what makes them separate places",
          (found["rays_agreeing"], found["viewpoints"]), (4, 4))
    check("a shuffle of the same place is still one place",
          locate.standing_places([toward(0.0, 0.0, 1), toward(0.2, 0.1, 2),
                                  toward(0.1, 0.2, 3)]), 1)


def test_the_looks_that_agree_say_where_exactly() -> None:
    """A pair chooses the answer and everything agreeing with it refines where it
    landed -- which is what makes a look taken to confirm a thing worth taking."""
    thing = (3.0, 3.0)

    def toward(x_m, y_m, look, off_deg=0.0):
        return {"x_m": x_m, "y_m": y_m, "inference_id": look,
                "bearing_deg": round(math.degrees(
                    math.atan2(thing[1] - y_m, thing[0] - x_m)) + off_deg, 3)}

    pair = [toward(0.0, 0.0, 1, +1.0), toward(6.0, 0.0, 2, -1.0)]
    crossed = locate.fix(*pair)
    check("two bearings a degree out put the thing off its real place",
          round(math.hypot(crossed["x_m"] - 3.0, crossed["y_m"] - 3.0), 2) > 0.05,
          True)
    check("two rays are their own crossing and are not moved by this",
          locate.refine(crossed, pair), crossed)

    # Three more looks from elsewhere, each a degree out the other way.
    more = pair + [toward(1.0, -2.0, 3, -1.0), toward(5.0, -2.0, 4, +1.0),
                   toward(6.0, 3.0, 5, -1.0)]
    better = locate.refine(crossed, more)
    was = math.hypot(crossed["x_m"] - 3.0, crossed["y_m"] - 3.0)
    now = math.hypot(better["x_m"] - 3.0, better["y_m"] - 3.0)
    check("the looks that agree move it nearer where the thing really is",
          now < was, True)
    check("...and it says how many of them it used", better["refined_from"], 5)
    check("...and reports the spread it measured rather than a narrower promise",
          better["uncertainty_m"] >= crossed["uncertainty_m"], True)

    far = dict(crossed, x_m=crossed["x_m"] + 4.0)
    check("a fit that lands somewhere else entirely is a different answer, "
          "not a better one", locate.refine(far, more), far)


def test_a_far_look_no_longer_outvotes_a_near_one() -> None:
    """**What `refine` changed on 2026-09-03, and why it was wrong before.**

    It used to minimise how far the answer sat to the *side* of each ray, in
    metres. A degree of error is 1.7 cm across at one metre and 8.7 cm at five, so
    a look taken from across the room counted for five times as much as one taken
    from beside the thing -- when the error being minimised is an angle and is the
    same size at both. It fits the angle now, so a ray's say depends on what its
    bearing is worth and not on how far away it happened to be standing.
    """
    thing = (3.0, 0.0)

    def toward(x_m, y_m, look, off_deg=0.0):
        return {"x_m": x_m, "y_m": y_m, "inference_id": look,
                "bearing_deg": round(math.degrees(
                    math.atan2(thing[1] - y_m, thing[0] - x_m)) + off_deg, 6)}

    # Two honest bearings from close by, and one just as wrong as the others are
    # right -- but taken from four times further off.
    near = [toward(2.0, -1.0, 1), toward(2.0, 1.0, 2)]
    crossed = locate.fix(*near)
    far = toward(3.0, -8.0, 3, +2.0)
    got = locate.refine(crossed, near + [far])
    check("a wrong bearing from far away no longer drags the answer to it",
          round(abs(got["y_m"]), 2) <= 0.1, True)

    check("a bearing that is simply wrong keeps only part of its pull",
          locate.robust_weight(3.0, 0.0, dict(far, bearing_deg=far["bearing_deg"]
                                              + 8.0), 0.0) < 0.5, True)


def test_a_refined_thing_says_which_way_its_error_runs() -> None:
    """The founding pair's error shape is measured by nudging its two bearings,
    which describes the crossing and stops describing anything the moment more
    looks move the point off it. The fit reports a covariance, so the shape comes
    from all the rays that agree -- and `cross_track` reads that shape, which is
    the term that used to be the largest in every match decision."""
    thing = (3.0, 0.0)

    def toward(x_m, y_m, look):
        return {"x_m": x_m, "y_m": y_m, "inference_id": look,
                "bearing_deg": round(math.degrees(
                    math.atan2(thing[1] - y_m, thing[0] - x_m)), 6)}

    pair = [toward(0.0, -1.2, 1), toward(0.0, 1.2, 2)]
    crossed = locate.fix(*pair)
    check("two bearings from one side are uncertain along their line of sight",
          round(crossed["error_major_deg"]), 0)
    flat = crossed["error_minor_m"] / crossed["error_major_m"]

    # Two more looks from off to one side, which is exactly what the pair could
    # not see: they pin the range the pair was vague about and leave the
    # remaining doubt running the other way.
    more = pair + [toward(3.0, -3.0, 3), toward(3.5, -2.5, 4)]
    got = locate.refine(crossed, more)
    check("looks from a new direction leave the error running a different way",
          abs(got["error_major_deg"] - crossed["error_major_deg"]) > 45.0, True)
    check("...and rounder than the pair could know",
          got["error_minor_m"] / got["error_major_m"] > flat, True)
    check("the size is still the measured spread and never a narrower promise",
          got["uncertainty_m"] >= crossed["uncertainty_m"], True)



# --- how high a thing is ----------------------------------------------------

def _high(x_m, y_m, bearing_deg, elevation_deg=None, span_deg=0.0,
          clipped=False):
    """A bearing that also says how high above the horizontal it pointed.

    `elevation_deg=None` is a look taken before the vertical half of the ray was
    kept, which is every row in the deployed database, and the whole point of
    the tests below is that such a look behaves exactly as it always did.
    """
    got = {"x_m": x_m, "y_m": y_m, "bearing_deg": bearing_deg, "span_deg": 0.0}
    if elevation_deg is not None:
        got.update({"elevation_deg": elevation_deg,
                    "elevation_span_deg": span_deg,
                    "elevation_clipped": clipped})
    return got


#: The sofa is at (3, 3), seen from (0, 0) and from (6, 0): 4.24 m from each. A
#: thing a metre above the camera is 13.26 degrees up from either of them, and
#: one two and a half metres up is 30.51.
UP_1_M, UP_2_5_M = 13.26, 30.51


def test_the_vertical_half_of_the_ray_is_measured_the_same_way() -> None:
    """**The projection returns a direction in three dimensions and the bearing
    uses two of them.** The third was computed and dropped on every box this
    rover has ever drawn, which is why everything the component knows about the
    room is flat."""
    check("a point on the lens axis is exactly as high as the gimbal is tilted",
          [round(view.elevation_deg(315.9 / 640, 227.4 / 480, tilt), 3)
           for tilt in (0.0, 10.0, 30.0)], [0.0, 10.0, 30.0])
    check("higher in the picture is higher in the room",
          view.elevation_deg(0.5, 0.2, 0.0) > view.elevation_deg(0.5, 0.8, 0.0),
          True)
    check("and the middle of the frame is not the axis, by the same 13 pixels "
          "the bearing already allows for",
          round(view.elevation_deg(0.5, 0.5, 10.0), 2), 7.52)

    observation = {"pose": {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0},
                   "observer_pan_deg": 0.0, "observer_tilt_deg": 10.0,
                   "bbox": a_box()}
    drawn = view.ray(observation, fov_deg=130.0)
    check("a look carries how high the thing was", drawn["elevation_deg"], 10.0)
    check("...and how tall it looked", drawn["elevation_span_deg"] > 0.0, True)
    check("...and whether the frame cut it off", drawn["elevation_clipped"],
          False)
    check("a box running off the top of the picture says so",
          view.ray(dict(observation, bbox=[0.4, 0.0, 0.6, 0.5]),
                   130.0)["elevation_clipped"], True)


def test_a_height_needs_a_range_and_a_bearing_has_none() -> None:
    """Which is why the elevation is spent after a thing is placed rather than
    being a second way of placing it."""
    point = {"x_m": 3.0, "y_m": 3.0, "uncertainty_m": 0.2}
    check("once the crossing says how far, the angle says how high",
          round(locate.rise_m(point, _high(0.0, 0.0, 45.0, UP_1_M)), 2), 1.0)
    check("a look that measured no elevation says nothing rather than level",
          locate.rise_m(point, _high(0.0, 0.0, 45.0)), None)
    check("and a ray pointing nearly at the ceiling says nothing either",
          locate.rise_m(point, _high(0.0, 0.0, 45.0, 85.0)), None)
    check("the height is above the camera, and stays there until somebody "
          "measures how high that is",
          (locate.CAMERA_HEIGHT_M, locate.above_floor_m(1.0)), (None, None))


def test_two_rays_must_agree_about_the_height_as_well_as_the_place() -> None:
    """**The test a plan view cannot make.** A bearing at a picture on the wall
    crosses a bearing at the sideboard beneath it exactly as convincingly as two
    bearings at one thing, and until this existed nothing in the resolver
    looked."""
    together = locate.fix(_high(0.0, 0.0, 45.0, UP_1_M),
                          _high(6.0, 0.0, 135.0, UP_1_M))
    check("two rays that agree about the height place the thing",
          together is not None, True)
    check("...and it comes out a metre above the camera",
          round(together["height_m"], 2), 1.0)
    check("...knowing that to a handspan", together["height_sigma_m"] < 0.2,
          True)

    check("two rays a metre and a half apart vertically place nothing",
          locate.fix(_high(0.0, 0.0, 45.0, UP_1_M),
                     _high(6.0, 0.0, 135.0, UP_2_5_M)), None)

    check("the camera's own height is not needed for that and is not used",
          locate.rise_disagreement(
              {"x_m": 3.0, "y_m": 3.0, "uncertainty_m": 0.2},
              _high(0.0, 0.0, 45.0, UP_1_M),
              _high(6.0, 0.0, 135.0, UP_2_5_M))[0] > 1.4, True)

    # The gate is removal-only, which is what lets a rover keep working through
    # the change: every look and every entity it already holds measured no
    # elevation at all.
    check("a pair with no elevations is placed exactly as it always was",
          locate.fix(_high(0.0, 0.0, 45.0),
                     _high(6.0, 0.0, 135.0)) is not None, True)
    check("...and claims no height it did not measure",
          "height_m" in locate.fix(_high(0.0, 0.0, 45.0),
                                   _high(6.0, 0.0, 135.0)), False)
    check("one look measuring a height and the other not is not an answer",
          locate.rise_disagreement({"x_m": 3.0, "y_m": 3.0},
                                   _high(0.0, 0.0, 45.0, UP_1_M),
                                   _high(6.0, 0.0, 135.0)), None)


def test_a_look_joins_a_thing_only_at_the_height_it_stands() -> None:
    """**Where the elevation actually earns its keep.** Measured on the drive of
    2026-09-03, an entity is not usually built wrong -- it is joined wrong
    afterwards, and every look that joins one comes through `agrees`."""
    placed = {"x_m": 3.0, "y_m": 3.0, "uncertainty_m": 0.2,
              "height_m": 1.0, "height_sigma_m": 0.15}
    check("a bearing at the right height joins",
          locate.agrees(placed, _high(0.0, 0.0, 45.0, UP_1_M)), True)
    check("...and the same bearing at something a metre and a half higher "
          "does not",
          locate.agrees(placed, _high(0.0, 0.0, 45.0, UP_2_5_M)), False)
    check("a thing with no height asks nothing of a look",
          locate.agrees({"x_m": 3.0, "y_m": 3.0, "uncertainty_m": 0.2},
                        _high(0.0, 0.0, 45.0, UP_2_5_M)), True)
    check("and neither does a look with no elevation",
          locate.agrees(placed, _high(0.0, 0.0, 45.0)), True)


def test_a_thing_is_forgiven_its_own_height_once_and_not_twice() -> None:
    """A doorway is two metres tall, so two looks at it centre their boxes a
    metre apart and both are pointing at the doorway. That allowance belongs to
    the ray asking to join, and a placement's own doubt must not carry a copy of
    it -- with both, anything founded on a box the frame had cut claimed a metre
    of slack and was then offered another metre by every ray that came near."""
    point = {"x_m": 3.0, "y_m": 3.0, "uncertainty_m": 0.2}
    tight = _high(0.0, 0.0, 45.0, UP_1_M)
    tall = _high(0.0, 0.0, 45.0, UP_1_M, span_deg=20.0)
    cut = _high(0.0, 0.0, 45.0, UP_1_M, clipped=True)
    check("a small crop forgives nothing for its own size",
          round(locate.rise_extent_m(point, tight), 3), 0.0)
    check("a crop of something tall forgives half of how tall it looked",
          round(locate.rise_extent_m(point, tall), 2), 0.75)
    check("and a crop the frame cut is forgiven the whole allowance, because "
          "its middle is wherever the frame happened to cut",
          locate.rise_extent_m(point, cut), locate.MAX_RISE_EXTENT_M)
    check("what a placement's own height is known to is measurement error and "
          "not any of that",
          locate.height_over(point, [tall, cut])[1],
          locate.rise_noise_m(point, tall))
    # ...with one exception, and the rover found it rather than the replay: a
    # height taken off a cut box has nothing later to forgive where its middle
    # sat, because `rise_extent_m` allows for the crop that is joining.
    check("a height with nothing but a cut box behind it says so",
          locate.height_over(point, [cut])[1]
          > locate.height_over(point, [tight])[1] + 0.9, True)
    check("...and one uncut look in the entity is enough to settle it",
          locate.height_over(point, [cut, tight])[1],
          locate.rise_noise_m(point, tight))


def test_the_height_improves_as_the_looks_do() -> None:
    """Left at whatever the founding pair said, a height would be a claim from
    two looks that every later one was then measured against -- and since
    `stands_as_high` refuses a look that disagrees with it, a pair that centred
    its boxes low would go on refusing every honest look at the top of the thing
    for ever."""
    rays = [_high(0.0, 0.0, 45.0, UP_1_M), _high(6.0, 0.0, 135.0, UP_1_M),
            _high(0.0, 6.0, -45.0, UP_1_M), _high(6.0, 6.0, -135.0, UP_1_M)]
    crossed = locate.fix(rays[0], rays[1])
    refined = locate.refine(crossed, rays)
    check("a refined placement still says how high the thing is",
          round(refined["height_m"], 2), 1.0)
    check("...taken from every ray that agrees, not from the founding two",
          refined["refined_from"], 4)

    # One badly cut box among four must not drag the answer, which is why the
    # middle is taken rather than the mean.
    astray = rays[:3] + [_high(6.0, 6.0, -135.0, UP_2_5_M, clipped=True)]
    check("one look at the wrong height does not move it",
          round(locate.height_over({"x_m": 3.0, "y_m": 3.0}, astray)[0], 2),
          1.0)


TESTS = (
    test_an_observation_becomes_a_bearing_from_a_measured_pose,
    test_the_bearing_comes_through_the_lens_the_gimbal_is_aimed_with,
    test_a_bearing_the_rover_measured_is_not_recomputed,
    test_the_rays_of_one_entity_are_bounded_and_oldest_first,
    test_a_look_is_related_to_the_position_the_thing_settled_on,
    test_the_rays_of_a_placed_thing_carry_the_relation,
    test_two_bearings_from_two_places_locate_a_thing,
    test_turning_on_the_spot_locates_nothing,
    test_two_looks_along_the_same_line_are_one_look,
    test_uncertainty_grows_when_the_baseline_shrinks,
    test_two_identical_chairs_stay_two_things,
    test_a_fix_on_top_of_the_camera_is_not_a_thing_in_the_room,
    test_a_bearing_at_the_edge_of_a_television_still_points_at_it,
    test_the_best_pair_places_the_thing,
    test_a_shallow_crossing_is_uncertain_lengthways_and_not_sideways,
    test_a_thing_may_not_claim_a_cone_the_geometry_never_earned,
    test_a_ray_that_started_somewhere_uncertain_says_so,
    test_a_placement_says_how_much_stands_behind_it,
    test_the_looks_that_agree_say_where_exactly,
    test_a_far_look_no_longer_outvotes_a_near_one,
    test_a_refined_thing_says_which_way_its_error_runs,
    test_the_vertical_half_of_the_ray_is_measured_the_same_way,
    test_a_height_needs_a_range_and_a_bearing_has_none,
    test_two_rays_must_agree_about_the_height_as_well_as_the_place,
    test_a_look_joins_a_thing_only_at_the_height_it_stands,
    test_a_thing_is_forgiven_its_own_height_once_and_not_twice,
    test_the_height_improves_as_the_looks_do,
)
