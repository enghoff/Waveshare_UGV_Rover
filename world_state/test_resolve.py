"""Identity: when two looks are one thing, and when they are not.

This is where the faults found by replaying real runs live. A thing cannot be
seen through a wall, two identical chairs are not guessed at from two places, and
appearance never overrules where a thing is.
"""
from __future__ import annotations

import math
import tempfile

from test_harness import check
from test_fakes import (a_capture, a_look, a_pose, a_seeing_inspector,
                        a_sighting, a_store, a_vector, observe)
from world_state import locate
from world_state import resolve


def test_two_looks_from_two_places_make_one_lasting_thing() -> None:
    """The whole point, in its simplest form."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        observe(store, 0.0, 0.0, 45.0, inference=1)
        observe(store, 6.0, 0.0, 135.0, inference=2)
        result = resolve.resolve(store)
        check("one thing was created", result["created"], 1)
        check("...and nothing was left ambiguous", result["ambiguous"], 0)
        check("...and the pool is empty", len(store.unplaced()), 0)

        placed = store.placed()
        check("the thing has a position", len(placed), 1)
        check("...where the two bearings actually cross",
              (round(placed[0]["placement"]["x_m"]),
               round(placed[0]["placement"]["y_m"])), (3, 3))
        check("...with an uncertainty rather than a claim of precision",
              placed[0]["placement"]["uncertainty_m"] > 0, True)
        check("...and both looks attached to it",
              placed[0]["observation_count"], 2)
        check("the popup can be told which two looks placed it",
              "crossed at" in result["decisions"][0]["why"], True)
        store.close()


def test_a_rover_that_only_turned_on_the_spot_places_nothing() -> None:
    """Rays from one point meet nowhere useful, and saying so is the point."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        observe(store, 1.0, 1.0, 40.0, inference=1)
        observe(store, 1.0, 1.0, 50.0, inference=2)
        result = resolve.resolve(store)
        check("nothing was created", result["created"], 0)
        check("...and both observations are still waiting",
              len(store.unplaced()), 2)
        check("...which is reported rather than silent",
              result["still_waiting"], 2)
        store.close()


def test_two_identical_chairs_are_not_guessed_at_from_two_places() -> None:
    """**The test the whole design exists to pass.**

    Two chairs and two viewpoints give four rays and four valid crossings: the
    two real chairs and two phantoms where a ray to one chair crosses a ray to
    the other. All four are sound geometry, and appearance cannot break the tie
    -- measured on this rover, the twin chair scores *higher* than the same chair
    seen from a new angle. From two places the answer is not knowable, so the
    resolver must wait rather than invent two things in the wrong places.
    """
    near, far = (2.7, 0.4), (3.0, 3.0)

    def seen_from(x_m, y_m, chair, inference):
        """The bearing from a place to a chair, rather than a number typed in.

        Typed-in bearings were rounded to a tenth of a degree and one of them was
        a degree and a half out, which passed only because a bearing used to be
        believed to five degrees. What the test means is that the rover stood
        here and the chair is there.
        """
        bearing = math.degrees(math.atan2(chair[1] - y_m, chair[0] - x_m))
        observe(store, x_m, y_m, round(bearing, 2), inference=inference)

    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            seen_from(0.0, 0.0, far, 1)
            seen_from(0.0, 0.0, near, 1)
            seen_from(6.0, 0.0, far, 2)
            seen_from(6.0, 0.0, near, 2)
            result = resolve.resolve(store)
            check("nothing was placed from two viewpoints", result["created"], 0)
            check("...and all four looks are still waiting",
                  len(store.unplaced()), 4)

            # A third viewpoint separates them: a real chair is agreed by every
            # ray aimed at it, and a phantom by only the two that made it.
            seen_from(3.0, -3.0, far, 3)
            seen_from(3.0, -3.0, near, 3)
            result = resolve.resolve(store)
            check("a third look from somewhere else settles it",
                  result["created"], 2)
            places = sorted((round(one["placement"]["x_m"], 1),
                             round(one["placement"]["y_m"], 1))
                            for one in store.placed())
            check("...as two things in two places", len(places), 2)
            check("...far enough apart to be told apart",
                  abs(places[0][1] - places[1][1]) > 1.5, True)
            check("...and each within a handspan of the chair it is",
                  max(min(math.dist(place, chair) for chair in (near, far))
                      for place in places) < 0.5, True)
        finally:
            store.close()


def test_one_television_seen_six_times_is_one_television() -> None:
    """**The duplicate the validation drive of 2026-09-02 produced.**

    Driven round three places and inspected at six headings from each, the rover
    placed four televisions, two of them eight centimetres apart, and three people
    where there was one. Two entities that are really one thing is the failure
    this whole design exists to prevent.

    The cause is an ordering one. A new thing absorbs the rays that support it,
    but never two rays from the same frame -- two regions of one frame are two
    different things by construction -- so with six frames looking at one
    television, the first entity takes a few and the rest are still waiting. They
    then pair with each other into a second television, because the list of things
    already placed was read once before any of this began and the thing created a
    moment ago is not on it.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            # One television at (3, 3), seen from six places around it, each look
            # its own inspection the way a survey makes them.
            telly = (3.0, 3.0)
            from_where = [(0.0, 0.0), (0.6, -0.4), (6.0, 0.0),
                          (5.4, 0.5), (3.0, -3.0), (2.4, -2.6)]
            for index, (x_m, y_m) in enumerate(from_where, start=1):
                bearing = math.degrees(math.atan2(telly[1] - y_m, telly[0] - x_m))
                observe(store, x_m, y_m, round(bearing, 2), inference=index)
            result = resolve.resolve(store)
            check("six looks at one television make one television",
                  result["created"], 1)
            placed = store.placed()
            check("...and only one thing is placed", len(placed), 1)
            check("...where the television is",
                  math.dist((placed[0]["placement"]["x_m"],
                             placed[0]["placement"]["y_m"]), telly) < 0.2, True)
            check("...with the later looks joining it rather than being ignored",
                  placed[0]["observation_count"] >= 4, True)
        finally:
            store.close()


def test_one_thing_cannot_swallow_the_wall_behind_it() -> None:
    """**The entity that ate fifty-three degrees of hallway, 2026-09-02.**

    The slack a bearing is allowed used to come from whichever crop was asking to
    join, capped at `MAX_EXTENT_M` -- and the cap saturated, so most match
    decisions were made with three quarters of a metre of room whatever the thing
    was. At a metre and a half that is a cone twenty-seven degrees either side of
    a bearing measured to one and a half, and a rover parked in a hallway
    collected thirteen bearings spanning fifty-three degrees into one entity: a
    ceiling corner, a dark doorway, a framed picture and a wall panel.

    The width is the thing's own now, measured when it is placed.
    """
    # A small thing -- a framed picture, a hand's breadth across -- placed 1.44 m
    # from where the rover stands, which is the geometry of that hallway.
    picture = {"x_m": -2.63, "y_m": 5.14, "uncertainty_m": 0.16,
               "extent_m": 0.12}
    at = (-2.25, 3.75)
    straight = math.degrees(math.atan2(picture["y_m"] - at[1],
                                       picture["x_m"] - at[0]))

    def off_by(degrees, span_deg, own_width):
        ray = {"x_m": at[0], "y_m": at[1],
               "bearing_deg": straight + degrees, "span_deg": span_deg}
        point = dict(picture)
        if own_width is None:
            point.pop("extent_m")               # what the old rule saw
        return locate.agrees(point, ray, locate.match_tolerance(point, ray))

    check("a bearing straight at it matches", off_by(0.0, 18.0, 0.12), True)
    check("...and so does one a few degrees off, which is the bearing's own error",
          off_by(8.0, 18.0, 0.12), True)

    # Observation 2632 of that run, with its own numbers: a region spanning
    # seventy-nine degrees -- most of the hallway -- pointed twenty-five degrees
    # away from the picture, which at this range is two thirds of a metre off.
    check("but a region spanning most of the frame cannot claim it from 25 degrees away",
          off_by(25.0, 79.5, 0.12), False)
    check("...which is exactly what the old rule allowed, because the slack came "
          "from that region rather than from the picture",
          off_by(25.0, 79.5, None), True)

    # A thing that really is wide keeps its room: a television a metre across,
    # seen from two and a half metres.
    telly = {"x_m": 2.5, "y_m": 0.0, "uncertainty_m": 0.1, "extent_m": 0.5}
    wide = {"x_m": 0.0, "y_m": 0.0, "bearing_deg": 11.0, "span_deg": 5.0}
    check("a television is still matched across its own width",
          locate.agrees(telly, wide, locate.match_tolerance(telly, wide)), True)


def test_a_thing_cannot_be_seen_through_a_wall() -> None:
    """**The fault that put two rooms inside one entity, with its own numbers.**

    A bearing carries no range, so two bearings cross *somewhere* whatever they
    are pointed at -- and two cameras aimed at two different things a couple of
    metres away in two different rooms give rays that meet ten metres off, at a
    healthy angle and off a healthy baseline. Every guard in `locate` accepted
    that, because none of them asked whether the rover could have seen that far
    in that direction at all.

    The rover could not, and the rover already knew: its own occupancy grid says
    where the first wall on a bearing is. On the run of 2026-09-02 it placed one
    thing at (9.87, 1.29) -- 4.7 m outside the edge of its own map -- from
    bearings whose first obstacle was 1.1 and 1.95 m away, and another 3.8 m out
    through a wall 55 cm in front of the rover. Those two are below.
    """
    # Two bearings from the rover's own record, and the crossing they made.
    first = {"x_m": 3.028, "y_m": 6.26, "bearing_deg": -36.0, "span_deg": 10.9}
    second = {"x_m": -0.724, "y_m": 0.081, "bearing_deg": 7.3, "span_deg": 10.5}
    unbounded = locate.fix(first, second)
    check("the two bearings cross, which is why this was ever placed",
          unbounded is not None, True)
    check("...ten and a half metres out, and confident about it",
          (round(math.dist((unbounded["x_m"], unbounded["y_m"]),
                           (second["x_m"], second["y_m"])), 1),
           unbounded["uncertainty_m"]), (10.5, 0.713))

    # What the map said at the time: a wall about a metre ahead of each of them.
    walled = locate.fix({**first, "reach_m": 1.95},
                        {**second, "reach_m": 1.10})
    check("...and with the map consulted, there is no such thing to place",
          walled, None)

    # The case that must keep working, and the reason the margin exists: a thing
    # a couple of metres away with the far wall of the room behind it.
    close = locate.fix({"x_m": 0.0, "y_m": 0.0, "bearing_deg": 45.0,
                        "reach_m": 6.0, "span_deg": 10.0},
                       {"x_m": 6.0, "y_m": 0.0, "bearing_deg": 135.0,
                        "reach_m": 6.0, "span_deg": 10.0})
    check("a thing in front of a far wall is still placed", close is not None,
          True)
    check("...and so is one standing right against the wall itself",
          locate.fix({"x_m": 0.0, "y_m": 0.0, "bearing_deg": 45.0,
                      "reach_m": 4.1, "span_deg": 10.0},
                     {"x_m": 6.0, "y_m": 0.0, "bearing_deg": 135.0,
                      "reach_m": 4.1, "span_deg": 10.0}) is not None, True)

    # And the other half of it: a bearing may not join a thing that sits behind
    # its own wall, however well the angle lines up.
    point = {"x_m": 1.66, "y_m": -2.93, "uncertainty_m": 0.56}
    aimed = {"x_m": -0.72, "y_m": 0.08, "bearing_deg": -50.2, "span_deg": 10.0}
    check("the bearing does point that way", locate.agrees(point, aimed), True)
    check("...but not through a wall 55 cm ahead of it",
          locate.agrees(point, {**aimed, "reach_m": 0.55}), False)
    check("a bearing the map cannot bound is left alone",
          locate.agrees(point, {**aimed, "reach_m": None}), True)


def test_a_thing_does_not_move_out_from_under_its_own_evidence() -> None:
    """**The wandering entity the drive of 2026-09-02 recorded, with its numbers.**

    An entity is re-placed from everything attached to it whenever a look joins,
    and it used to take the pair of bearings with the smallest uncertainty --
    which is a statement about two rays and about nothing else. So one lucky pair
    could move a thing with a dozen looks behind it clean out from under all of
    them: 13 of that drive's 151 re-placements moved more than half a metre, one
    of them 2.6 m in a single step, and afterwards 45% of every entity's own
    bearings missed its own stated position.

    The four bearings below are `object:14`'s, taken out of the rover's own
    database. The tightest pair among them lands somewhere two of the four
    disagree with; the pair all four agree with is two metres away and has a
    wider uncertainty, and it is the right answer. `world_state/replay.py` is
    what this was found with.
    """
    measured = [(-3.739, 2.906, -82.2), (-4.351, 4.421, -89.6),
                (-3.782, 2.911, -88.6), (0.544, 0.078, -119.1)]
    rays = [{"x_m": x, "y_m": y, "bearing_deg": bearing, "span_deg": 20.0}
            for x, y, bearing in measured]

    def tightest(candidates):
        """What `best_fix` used to do, kept here so the test is a comparison."""
        best = None
        for index, first in enumerate(candidates):
            for second in candidates[index + 1:]:
                found = locate.fix(first, second)
                if found is None:
                    continue
                if best is None or found["uncertainty_m"] < best["uncertainty_m"]:
                    best = found
        return best

    was = tightest(rays)
    now = locate.best_fix(rays)
    check("the tightest pair of these four is agreed by only half of them",
          sum(1 for ray in rays if locate.agrees(was, ray)), 2)
    check("...and the placement chosen instead is agreed by all four",
          sum(1 for ray in rays if locate.agrees(now, ray)), 4)
    check("...which is a different place, not a rounding",
          math.dist((was["x_m"], was["y_m"]), (now["x_m"], now["y_m"])) > 1.5,
          True)
    check("...and it is allowed to be the less certain of the two",
          now["uncertainty_m"] > was["uncertainty_m"], True)


def test_the_reason_survives_the_inspection_that_decided_it() -> None:
    """The question a person asks of an identity is why, not what.

    The resolver has always written a sentence about each decision, but it went
    back in the reply to whichever inspection happened to trigger the resolve and
    was gone by the time anybody opened the console to ask. It is kept on the
    observation now, which is where the rest of that decision's evidence is.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            observe(store, 0.0, 0.0, 45.0, inference=1)
            observe(store, 6.0, 0.0, 135.0, inference=2)
            result = resolve.resolve(store)
            check("the thing was placed", result["created"], 1)
            notes = [one["note"] for one in store.observations()
                     if one.get("entity_id")]
            check("...and both looks say why they belong to it",
                  len(notes), 2)
            check("...in the resolver's own words",
                  all(note and "crossed at" in note for note in notes), True)
        finally:
            store.close()


def test_a_third_look_joins_the_thing_it_points_at() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        observe(store, 0.0, 0.0, 45.0, inference=1)
        observe(store, 6.0, 0.0, 135.0, inference=2)
        resolve.resolve(store)
        entity_id = store.placed()[0]["id"]

        observe(store, 3.0, -1.0, 90.0, inference=3)
        result = resolve.resolve(store)
        check("the new look was matched rather than made into a second thing",
              (result["matched"], result["created"]), (1, 0))
        check("...to the thing that was already there",
              result["decisions"][0]["entity_id"], entity_id)
        check("...and the reason names the distance",
              "m away" in result["decisions"][0]["why"], True)
        check("the world still holds one thing", len(store.placed()), 1)
        check("...with three looks behind it",
              store.placed()[0]["observation_count"], 3)
        store.close()


def test_appearance_cannot_overrule_where_a_thing_is() -> None:
    """The redundant-furniture rule, stated as a test.

    A crop that looks *exactly* like the stored exemplar, on a bearing pointing
    at the other side of the room, must not match. This is the failure mode of
    every appearance-first design and the reason geometry is the key here.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        twin = a_vector(1.0, 0.0)
        observe(store, 0.0, 0.0, 45.0, vector=twin, inference=1)
        observe(store, 6.0, 0.0, 135.0, vector=twin, inference=2)
        resolve.resolve(store)
        check("something was placed", len(store.placed()), 1)

        # The same appearance entirely, four metres away across the room.
        observe(store, 0.0, 0.0, -60.0, vector=twin, inference=3)
        result = resolve.resolve(store)
        check("a perfect appearance match on the wrong bearing is not a match",
              result["matched"], 0)
        check("...and it is not quietly made into a new thing either",
              result["created"], 0)
        check("...it waits for a second bearing of its own",
              len(store.unplaced()), 1)
        store.close()


def test_two_placed_things_on_one_bearing_are_ambiguous_not_a_guess() -> None:
    """One chair directly behind another, from where the rover is standing.

    Both are placed, both are consistent with the new bearing, and appearance
    cannot separate them. Attaching the look to the nearer one would be a guess
    dressed as an answer, so it is attached to neither and the popup is told why.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        observe(store, 0.0, 0.0, 45.0, inference=1)      # a chair at (2, 2)
        observe(store, 4.0, 0.0, 135.0, inference=2)
        resolve.resolve(store)
        observe(store, 0.0, 4.0, 0.0, inference=3)       # another at (4, 4)
        observe(store, 4.0, 0.0, 90.0, inference=4)
        resolve.resolve(store)
        check("both chairs were placed", len(store.placed()), 2)

        # From the origin the two are in exactly the same direction.
        observe(store, 0.0, 0.0, 45.0, inference=5)
        result = resolve.resolve(store)
        check("a bearing consistent with both is left alone",
              result["ambiguous"], 1)
        check("...rather than attached to either", result["matched"], 0)
        check("...and the reason says so in words",
              "equally consistent" in result["decisions"][0]["why"], True)
        check("...and the look is still in the pool",
              len(store.unplaced()), 1)
        store.close()


def test_a_chair_and_a_ceiling_light_are_never_the_same_thing() -> None:
    """The gate that replaced the word list, doing the one job it may do.

    Two bearings that cross beautifully are still not two looks at one thing
    if the crops behind them look nothing like each other. This used to be
    decided by comparing two names against a hand-written list of synonyms;
    it is decided now by comparing the appearance vectors, which is the same
    question asked of something the rover actually measured.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            # A perfect crossing at (3, 3), and two crops with nothing in
            # common: on this rover a chair against a spray bottle is 0.122.
            observe(store, 0.0, 0.0, 45.0, vector=a_vector(1.0, 0.0),
                    inference=1)
            observe(store, 6.0, 0.0, 135.0, vector=a_vector(0.0, 1.0),
                    inference=2)
            result = resolve.resolve(store)
            check("two things that look nothing alike are not one thing",
                  result["created"], 0)
            check("...and both looks are left waiting rather than merged",
                  len(store.unplaced()), 2)

            # The identical geometry, with two crops that do look alike.
            store.clear()
            observe(store, 0.0, 0.0, 45.0, vector=a_vector(1.0, 0.05),
                    inference=1)
            observe(store, 6.0, 0.0, 135.0, vector=a_vector(1.0, 0.0),
                    inference=2)
            check("the same crossing between two views of one thing is placed",
                  resolve.resolve(store)["created"], 1)
        finally:
            store.close()


def test_the_right_place_is_not_enough_if_it_looks_wrong() -> None:
    """What the list of things that move used to buy, without the list.

    A bottle is a poor thing to identify by position, because it was moved;
    the old rule knew which things those were by name, and there are no names
    any more. What survives is the half that never needed one: a look on a
    bearing that points straight at a placed thing, but whose crop looks
    nothing like anything that thing has shown, is not that thing.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        bottle = a_vector(1.0, 0.0)
        observe(store, 0.0, 0.0, 45.0, vector=bottle, inference=1)
        observe(store, 6.0, 0.0, 135.0, vector=bottle, inference=2)
        resolve.resolve(store)
        check("two looks that agree place the thing", len(store.placed()), 1)

        # The right place, and nothing like it to look at.
        observe(store, 3.0, -1.0, 90.0, vector=a_vector(0.0, 1.0), inference=3)
        result = resolve.resolve(store)
        check("a look in the right place that looks wrong is not matched",
              result["matched"], 0)
        check("...nor quietly made into a second thing", result["created"], 0)
        check("...it waits for a second bearing of its own",
              len(store.unplaced()), 1)
        store.close()


def test_the_evidence_survives_the_decision() -> None:
    """An entity is an opinion; the observations behind it are history."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        observe(store, 0.0, 0.0, 45.0, inference=1)
        observe(store, 6.0, 0.0, 135.0, inference=2)
        resolve.resolve(store)
        entity_id = store.placed()[0]["id"]
        rows = store.observations(entity_id)
        check("both observations still hold the bearing they measured",
              [row["bearing_deg"] for row in rows], [135.0, 45.0])
        check("...and the pose behind it",
              all(row["pose"] for row in rows), True)
        check("the entity keeps an exemplar of what it looked like",
              len(store.exemplars(entity_id, width=32)), 2)
        store.close()


def test_one_entity_can_be_sent_to_a_console_like_the_list_can() -> None:
    """The console's detail pane said "nothing selected" for every entity.

    Not a rendering fault and not a race. `store.entity` handed back the row as
    SQLite produced it, exemplars and all, so the reply to `world_state_entity`
    held a raw float32 BLOB; the daemon could not turn it into JSON, wrote no
    reply at all, and the page waited forever for a payload that never came.
    Clicking a thing therefore did nothing, in a popup whose whole purpose is
    showing the looks behind a thing.

    Two properties, and the second is what made the first easy to miss: the row
    has to be serialisable, and it has to carry the same decoded placement the
    list carries -- a reply that got through without one would have shown a
    placed thing as having no position.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            entity_id = store.create_entity()
            store.place(entity_id, {"x_m": 1.0, "y_m": 2.0, "uncertainty_m": 0.3,
                                    "baseline_m": 3.0, "parallax_deg": 30.0}, 1)
            store.add_exemplar(entity_id, a_vector(1.0, 0.0))
            one = store.entity(entity_id)
            check("the row carries no raw vector", "exemplars" in one, False)
            check("...but says how many it has", one["exemplar_count"], 1)
            check("...and its position, decoded rather than as stored JSON",
                  (one["placement"] or {}).get("x_m"), 1.0)
            import json as _json
            _json.dumps(one)
            check("...so it can be sent to a console at all", True, True)
            check("an entity that is not there is still None rather than a crash",
                  store.entity("object:404"), None)
        finally:
            store.close()


def test_a_placement_belongs_to_the_map_it_was_measured_in() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        observe(store, 0.0, 0.0, 45.0, inference=1)
        observe(store, 6.0, 0.0, 135.0, inference=2)
        resolve.resolve(store)
        check("the thing is placed in this map", len(store.placed(1)), 1)
        store.new_map_session()
        check("...and in no other", len(store.placed(2)), 0)

        observe(store, 3.0, -1.0, 90.0, inference=3)
        result = resolve.resolve(store)
        check("a look in the new map cannot join a thing placed in the old one",
              result["matched"], 0)
        store.close()


def test_an_inspection_settles_identity_as_well_as_measuring() -> None:
    """The two halves joined: measure, then decide, in that order.

    The order is the safety property. Everything measured is written down before
    anything is decided about it, so a resolver that fails leaves a rover with
    twelve honest observations rather than with a failed inspection.
    """
    with tempfile.TemporaryDirectory() as directory:
        store, eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()], [a_sighting()]],
            capture=a_capture(pan=0.0), pose=a_pose(0.0, 0.0, 45.0))
        first = inspector.inspect()
        check("the first look measures and settles nothing",
              (first["stored"], first["created"]), (1, 0))
        check("...and says it is waiting for a look from elsewhere",
              "waiting for a look from elsewhere" in first["detail"], True)

        inspector.pose = a_pose(6.0, 0.0, 135.0)
        second = inspector.inspect()
        check("the second look from another place places the thing",
              second["created"], 1)
        check("...and the popup is told which two looks did it",
              "crossed at" in second["decisions"][0]["why"], True)
        check("the world now holds one placed thing", len(store.placed()), 1)
        store.close()


def test_one_look_gives_a_thing_one_region_however_many_passes_it_takes() -> None:
    """**The fault of 2026-09-03, and it is a fault of memory rather than of rule.**

    Two regions of one frame are two different things -- the region finder's
    overlap suppression saw to that -- and the resolver has always refused the
    second. It refused it *within a pass*, out of a dictionary rebuilt every time
    `resolve` is called, while an observation with no partner waits in the
    pending pool indefinitely by design. So the frame simply came back next pass
    and gave another.

    On the rover this was not subtle. One entity took four disjoint regions of a
    single picture -- traced joining on three consecutive passes -- and finished
    holding twenty-six crops of a cabinet, two framed pictures, a doorway, a
    table and a person's head.

    Here one look sees three things within a few degrees of each other, a second
    look from across the room places one of them, and the pool is then resolved
    several times over. The other two point straight at the thing that was
    placed -- that is the whole difficulty, and refusing them is the rule -- so
    what is checked is that they are still being refused on the fourth pass.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            # Three regions of one standstill, close enough together that every
            # one of them points at where the first is placed.
            for bearing in (43.0, 45.0, 47.0):
                observe(store, 0.0, 0.0, bearing, inference=1)
            observe(store, 6.0, 0.0, 135.0, inference=2)
            resolve.resolve(store)
            placed = store.placed()
            check("the crossing places one thing", len(placed), 1)
            check("...taking exactly one region of the look that saw three",
                  len(store.entities_in_frame(1)), 1)
            counted = placed[0]["observation_count"]
            check("...which is that region and the one from across the room",
                  counted, 2)

            # The other two regions of frame 1 do point at it: this is the
            # refusal being tested, not a bearing that happens to miss.
            entity = placed[0]
            waiting = store.unplaced()
            check("...while the two left over aim at it well within tolerance",
                  all(locate.agrees(entity["placement"],
                                    resolve.ray_of(one),
                                    locate.match_tolerance(entity["placement"],
                                                           resolve.ray_of(one)))
                      for one in waiting), True)

            for _ in range(3):
                resolve.resolve(store)
            check("...and no later pass gives it another",
                  store.placed()[0]["observation_count"], counted)
            check("...so the leftovers are still waiting rather than swallowed",
                  len(store.unplaced()), 2)
        finally:
            store.close()


def test_a_wrong_exemplar_does_not_make_the_next_one_easier() -> None:
    """**The appearance gate got looser every time it was wrong.**

    A crop that joins an entity becomes one of its exemplars, and the score was
    the best of them -- so a thing that had swallowed something unrelated would
    accept the next unrelated thing more readily for it. Measured on the run of
    2026-09-03: one exemplar admitted 10% of the pending pool, twenty-one
    admitted 64%, monotonically the whole way.

    Here an entity holds four exemplars of one thing and one of something else.
    A crop of that something else must not clear the gate on the strength of the
    single wrong exemplar.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            entity = store.create_entity()
            looks_like = a_vector(1.0, 0.0)
            unrelated = a_vector(0.0, 1.0)
            for _ in range(4):
                store.add_exemplar(entity, looks_like)
            store.add_exemplar(entity, unrelated)

            check("a crop of what the thing mostly is still scores well",
                  resolve.appearance(store, entity, looks_like) > 0.9, True)
            # Scored against the best of the exemplars this is 1.0 -- a perfect
            # match to the one crop that should never have been there.
            check("...and one of the odd exemplar does not inherit its score",
                  resolve.appearance(store, entity, unrelated)
                  < resolve.DIFFERENT_THING, True)
            check("the question cannot be asked of a thing with no exemplars",
                  resolve.appearance(store, store.create_entity(), looks_like),
                  None)
            check("...nor of a look that produced no vector",
                  resolve.appearance(store, entity, b""), None)
        finally:
            store.close()


def test_a_standoff_in_one_corner_does_not_stop_the_other_corner() -> None:
    """Two identical chairs from two places is a standoff nothing can settle, and
    it used to end the whole pass -- so the lamp across the room, which no ray
    disagrees about at all, went unplaced with it. On the run of 2026-09-03 that
    happened in 65 of 181 pairing passes, with a median of four crossings still on
    the table each time.

    What is unknowable is which of two answers built from *the same ray* is right.
    That says nothing about a crossing built from two other rays entirely.
    """
    near, far = (2.7, 0.4), (3.0, 3.0)
    lamp = (0.0, 8.0)

    def seen_from(x_m, y_m, thing, inference, vector):
        bearing = math.degrees(math.atan2(thing[1] - y_m, thing[0] - x_m))
        observe(store, x_m, y_m, round(bearing, 2), vector=vector,
                inference=inference)

    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            chair, brass = a_vector(1.0, 0.0), a_vector(0.0, 1.0)
            seen_from(0.0, 0.0, far, 1, chair)
            seen_from(0.0, 0.0, near, 1, chair)
            seen_from(6.0, 0.0, far, 2, chair)
            seen_from(6.0, 0.0, near, 2, chair)
            seen_from(-2.0, 5.0, lamp, 3, brass)
            seen_from(2.0, 5.0, lamp, 4, brass)

            result = resolve.resolve(store)
            check("the lamp nothing argues about is placed", result["created"], 1)
            placed = store.placed()
            check("...and the two chairs are still a standoff, not two guesses",
                  len(placed), 1)
            check("...where it actually is",
                  [(round(one["placement"]["x_m"]),
                    round(one["placement"]["y_m"])) for one in placed],
                  [(0, 8)])
            check("...with all four of their looks still waiting",
                  len(store.unplaced()), 4)
        finally:
            store.close()


def test_one_pass_places_a_few_things_and_leaves_the_rest_for_the_next() -> None:
    """Searching for a crossing is a second at a full pool and it runs again after
    every placement, so a pass that placed everything it could took 55 s on the
    rover against a settle cadence of 10. The pool is not a queue that has to be
    drained in one go, so it is not drained in one go."""
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            # Six lamps in a row, each seen from two places well apart, so every
            # one of them is placeable and none of them argues with any other.
            for n in range(6):
                lamp = (float(n) * 3.0, 9.0)
                # One dimension each, so no lamp can be mistaken for another and
                # what is being counted is the pass and not the appearance gate.
                mark = a_vector(*[1.0 if axis == n else 0.0 for axis in range(6)])
                for index, (x_m, y_m) in enumerate(((lamp[0] - 2.0, 0.0),
                                                    (lamp[0] + 2.0, 0.0))):
                    bearing = math.degrees(math.atan2(lamp[1] - y_m, lamp[0] - x_m))
                    observe(store, x_m, y_m, round(bearing, 2), vector=mark,
                            inference=n * 2 + index)

            first = resolve.resolve(store)
            check("one pass places what it is allowed to and stops",
                  first["created"], resolve.MAX_NEW_PER_PASS)
            check("...leaving the rest of the pool exactly where it was",
                  len(store.unplaced()), (6 - resolve.MAX_NEW_PER_PASS) * 2)
            check("...and the next pass carries on from it",
                  resolve.resolve(store)["created"], resolve.MAX_NEW_PER_PASS)
            for _again in range(6):                # bounded: six lamps, at worst
                if not resolve.resolve(store)["created"]:
                    break
            check("...until every lamp is placed", len(store.placed()), 6)
            check("...and nothing is left waiting", len(store.unplaced()), 0)
        finally:
            store.close()


def test_two_things_side_by_side_are_not_cut_down_the_wrong_seam() -> None:
    """**The fault the drive of 2026-09-03 left behind, and it was throwing the
    evidence away rather than filing it wrongly.** A blue-topped bench and the
    dark wardrobe beside it, 41 cm apart on the map, came back as two things
    each holding some crops of each, with which one got the bench flipping from
    look to look.

    Asked one region at a time the question really is unanswerable: each of the
    two bearings is consistent with both things, and appearance cannot separate
    a dark wardrobe from a bench in shadow -- so both regions are declared
    ambiguous and neither is used, every look, for ever. Asked of the look as a
    whole it *is* answerable, because there are two regions and two things and
    one way of sharing them out is plainly better than the other.

    Both orders are checked, because the order the pool happened to be in is
    what used to decide.
    """
    for far_first in (True, False):
        with tempfile.TemporaryDirectory() as directory:
            store = a_store(directory)
            bench, wardrobe = (3.0, 3.0), (3.4, 2.75)

            def bearing(frm, to):
                return round(math.degrees(math.atan2(to[1] - frm[1],
                                                     to[0] - frm[0])), 2)

            # Two looks from two places, each seeing both things. Neither look
            # may give one thing two regions, so this is how the rover comes to
            # hold two things a handspan apart in the first place.
            for step, place in ((1, (0.0, 0.0)), (2, (6.0, 0.0))):
                a_look(store, place[0], place[1],
                       [bearing(place, bench), bearing(place, wardrobe)],
                       inference=step)
            for _ in range(3):
                resolve.resolve(store)
            placed = sorted(store.placed(),
                            key=lambda one: one["placement"]["y_m"])
            check("two things stand a handspan apart", len(placed), 2)
            check("...and that is how far apart they are",
                  round(math.hypot(
                      placed[0]["placement"]["x_m"] - placed[1]["placement"]["x_m"],
                      placed[0]["placement"]["y_m"] - placed[1]["placement"]["y_m"]),
                      2), 0.46)

            # A third look, from a third place, with a region on each of them.
            third = (3.0, -2.0)
            offered = [bearing(third, bench), bearing(third, wardrobe)]
            if not far_first:
                offered.reverse()
            a_look(store, third[0], third[1], offered, inference=9)
            result = resolve.resolve(store)

            check("both regions of the look found a home", result["matched"], 2)
            check("...none was left ambiguous", result["ambiguous"], 0)
            check("...and no third copy was invented", len(store.placed()), 2)
            went = {round(row["bearing_deg"], 2): row["entity_id"]
                    for row in store.observations(limit=80)
                    if row["entity_id"] and row["inference_id"] == 9}
            nearer = placed[0]["id"] if placed[0]["placement"]["y_m"] < \
                placed[1]["placement"]["y_m"] else placed[1]["id"]
            further = [one["id"] for one in placed if one["id"] != nearer][0]
            check("the bearing at the wardrobe went to the wardrobe",
                  went.get(bearing(third, wardrobe)), nearer)
            check("...and the bearing at the bench to the bench",
                  went.get(bearing(third, bench)), further)
            check("...and the reason says the whole look decided it",
                  "regions in this look" in
                  (store.observations(nearer, limit=1)[0]["note"] or ""), True)
            store.close()


TESTS = (
    test_two_looks_from_two_places_make_one_lasting_thing,
    test_two_things_side_by_side_are_not_cut_down_the_wrong_seam,
    test_a_rover_that_only_turned_on_the_spot_places_nothing,
    test_two_identical_chairs_are_not_guessed_at_from_two_places,
    test_one_television_seen_six_times_is_one_television,
    test_one_thing_cannot_swallow_the_wall_behind_it,
    test_a_thing_cannot_be_seen_through_a_wall,
    test_a_thing_does_not_move_out_from_under_its_own_evidence,
    test_the_reason_survives_the_inspection_that_decided_it,
    test_a_third_look_joins_the_thing_it_points_at,
    test_appearance_cannot_overrule_where_a_thing_is,
    test_two_placed_things_on_one_bearing_are_ambiguous_not_a_guess,
    test_a_chair_and_a_ceiling_light_are_never_the_same_thing,
    test_the_right_place_is_not_enough_if_it_looks_wrong,
    test_the_evidence_survives_the_decision,
    test_one_entity_can_be_sent_to_a_console_like_the_list_can,
    test_a_placement_belongs_to_the_map_it_was_measured_in,
    test_an_inspection_settles_identity_as_well_as_measuring,
    test_one_look_gives_a_thing_one_region_however_many_passes_it_takes,
    test_a_wrong_exemplar_does_not_make_the_next_one_easier,
    test_a_standoff_in_one_corner_does_not_stop_the_other_corner,
    test_one_pass_places_a_few_things_and_leaves_the_rest_for_the_next,
)
