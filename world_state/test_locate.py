"""Geometry: turning observations into bearings, and bearings into a place.

A bearing carries no range, so two of them cross somewhere whatever they were
aimed at. What is checked is that a crossing is only believed when the baseline
earns it, and that a rover which only turned on the spot places nothing.
"""
from __future__ import annotations

import math

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
          drawn["bearing_deg"], 60.0)
    check("...and a box the width of a fifth of the picture is that wide",
          drawn["span_deg"], 26.0)

    left = dict(observation, bbox=[0.0, 0.2, 0.2, 0.8])
    check("something at the left of the picture is further to the left again",
          view.ray(left, fov_deg=130.0)["bearing_deg"], 112.0)

    check("no pose means no ray, rather than a ray from the origin",
          view.ray(dict(observation, pose=None), 130.0), None)
    check("no gimbal angle means no ray either",
          view.ray(dict(observation, observer_pan_deg=None), 130.0), None)
    check("a box that is missing still leaves the camera direction",
          view.ray(dict(observation, bbox=None), 130.0)["bearing_deg"], 60.0)


def test_the_rays_of_one_entity_are_bounded_and_oldest_first() -> None:
    observations = [{"pose": {"x_m": float(n), "y_m": 0.0, "heading_deg": 0.0},
                     "observer_pan_deg": 0.0, "bbox": None, "observed_at": n}
                    for n in range(10)]
    drawn = view.rays(observations, 130.0, limit=4)
    check("only the newest few are drawn", len(drawn), 4)
    check("...oldest of those first, so the newest is on top",
          [one["observed_at"] for one in drawn], [3, 2, 1, 0])


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


TESTS = (
    test_an_observation_becomes_a_bearing_from_a_measured_pose,
    test_the_rays_of_one_entity_are_bounded_and_oldest_first,
    test_two_bearings_from_two_places_locate_a_thing,
    test_turning_on_the_spot_locates_nothing,
    test_two_looks_along_the_same_line_are_one_look,
    test_uncertainty_grows_when_the_baseline_shrinks,
    test_two_identical_chairs_stay_two_things,
    test_a_fix_on_top_of_the_camera_is_not_a_thing_in_the_room,
    test_a_bearing_at_the_edge_of_a_television_still_points_at_it,
    test_the_best_pair_places_the_thing,
)
