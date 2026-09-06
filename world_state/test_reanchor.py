"""Putting a map's worth of stranded things back on the map the rover is on.

The thing being tested is a claim about a room rather than about arithmetic: two
maps of one room differ by a turn and a shift, so if the turn and the shift can
be recovered from the things that appear in both, everything else the old map
held can be carried across on their word. What must not happen is a transform
invented out of coincidences, or two different objects quietly becoming one row.
"""
from __future__ import annotations

import math
import tempfile

from test_harness import check
from test_fakes import a_look, a_store, a_vector, observe
from world_state import reanchor, resolve


def its_own(index: int, width: int = 8):
    """An appearance nothing else in a fixture shares: one axis, to itself.

    Orthogonal rather than merely different, because `DIFFERENT_THING` only
    throws out what is plainly unrelated: two vectors a quarter turn apart still
    score 0.7 and a bearing at one thing will happily join another. What these
    tests are about is which map a thing is in, so appearance has to be out of
    the way rather than realistic.
    """
    return a_vector(*[1.0 if one == index else 0.0 for one in range(width)])


def much_like(index: int, width: int = 8):
    """An appearance that scores about 0.89 against `its_own(index)`."""
    return a_vector(*[1.0 if one == index else (0.5 if one == width - 1 else 0.0)
                      for one in range(width)])


def a_room(store, things):
    """Place each of `things` by crossing two looks at it, and answer their ids.

    Two viewpoints two metres either side of the thing, which is parallax enough
    for a crossing and is the shape every other resolver test uses. Resolved
    until the pool stops draining, because one pass places `MAX_NEW_PER_PASS`
    and a room has more things in it than that.

    **Recognition is held off while the fixture is built**, which is the point of
    the fixture rather than a convenience: what `reanchor` is for is the backlog
    left by the map changes that happened before `resolve._adopt` existed, and
    with it running these rooms would recognise each other as they were laid down
    and there would be nothing stranded to test against.
    """
    was = resolve._adopt
    resolve._adopt = lambda *a, **k: None
    try:
        for index, ((x_m, y_m), vector) in enumerate(things):
            for spot in ((x_m - 2.0, y_m - 2.0), (x_m + 2.0, y_m - 2.0)):
                bearing = math.degrees(math.atan2(y_m - spot[1], x_m - spot[0]))
                observe(store, spot[0], spot[1], round(bearing, 2), vector=vector,
                        inference=f"{store.map_session()}-{index}-{spot[0]}")
        for _ in range(len(things) + 2):
            if not resolve.resolve(store)["created"]:
                break
    finally:
        resolve._adopt = was
    return [one["id"] for one in store.placed(store.map_session())]


def standing(store, spot, vector, uncertainty_m=0.2):
    """A thing already placed here, put in directly rather than resolved into.

    Two things that look alike and stand a metre apart cannot be built by
    crossing bearings at them -- the resolver refuses to invent look-alikes that
    close together, which is the phantom rule doing its job. On the rover they
    arise anyway, over many passes and across map changes, and there were
    clusters of three at 0.90, 0.85 and 0.81 in its store when this was written.
    So the state is written down directly, because it is `reanchor` being tested
    here and not the resolver that would never have produced it in one go.
    """
    entity_id = store.create_entity()
    store.place(entity_id, {"x_m": spot[0], "y_m": spot[1],
                            "uncertainty_m": uncertainty_m}, store.map_session())
    store.add_exemplar(entity_id, vector)
    return entity_id


def turned(point, degrees, shift):
    """A point of one map as a point of another, for building the fixture."""
    turn = math.radians(degrees)
    cos, sin = math.cos(turn), math.sin(turn)
    return (cos * point[0] - sin * point[1] + shift[0],
            sin * point[0] + cos * point[1] + shift[1])


WHERE = [(2.0, 6.0), (-3.0, 5.0), (4.0, 8.0), (-2.0, 9.0), (5.0, 4.0), (0.0, 11.0)]
TURN, SHIFT = 40.0, (1.5, -2.5)


def test_a_room_seen_twice_gives_up_the_transform_between_its_two_maps() -> None:
    """**The whole claim, and the reason the backlog is recoverable at all.**

    Six things in a room, recorded under one map; the map is replaced and four of
    them are found again, in coordinates that are the same room turned 40 degrees
    and shifted. Nothing tells the rover that -- it has two sets of coordinates
    with nothing in common but what the things look like. Recovering the turn
    from that is what lets the two it did *not* see again be carried across as
    well, which is the half no amount of looking can ever reach.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            a_room(store, [(spot, its_own(index))
                           for index, spot in enumerate(WHERE)])
            check("the rover knows six things", len(store.placed()), 6)

            store.new_map_session()
            # The same room, turned and shifted: four of the six are seen again.
            a_room(store, [(turned(WHERE[index], TURN, SHIFT), its_own(index))
                           for index in range(4)])
            check("four of them stand in the new map",
                  len(store.placed(store.map_session())), 4)
            check("...and six are stranded in the old one",
                  len(store.placed_elsewhere(store.map_session())), 6)

            decided = reanchor.plan(store, min_agreeing=3)
            entry = decided["maps"][0]
            check("the turn between the two maps is recovered",
                  round(entry["fit"]["turn_deg"]), 40)
            check("...from the things that appear in both",
                  entry["fit"]["agreeing"], 4)
            check("...and it holds as the tolerance moves",
                  entry["steady"].startswith("holds to"), True)
            check("the four seen again are folded into what stands here",
                  len(entry["merge"]), 4)
            check("...and the two never seen again are carried across",
                  len(entry["move"]), 2)

            reanchor.apply(store, decided)
            check("nothing is stranded any more",
                  len(store.placed_elsewhere(store.map_session())), 0)
            check("...and the room is six things again, not ten",
                  len(store.placed(store.map_session())), 6)
            # The fifth thing was never seen in the new map, so where it is now
            # rests entirely on the transform the other four fixed.
            wanted = turned(WHERE[4], TURN, SHIFT)
            got = [one["placement"] for one in store.placed()
                   if math.dist((one["placement"]["x_m"],
                                 one["placement"]["y_m"]), wanted) < 0.4]
            check("the thing the rover never saw again is where the room says",
                  len(got), 1)
            check("...and says how far out that is",
                  got[0]["uncertainty_m"] > 0, True)
            check("...and that it was carried rather than measured here",
                  got[0]["carried_from_map"], 1)
        finally:
            store.close()


def test_a_thing_that_was_seen_again_keeps_the_history_of_both() -> None:
    """A fold is the point of a fold: one row, all the looks, the good position.

    The row that survives is the one standing in this map, because its
    coordinates were measured here rather than carried; what it gains is
    everything the older row was ever seen in, which is what somebody asking
    "how long have you known about that" is asking for.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            a_room(store, [(WHERE[0], its_own(0)), (WHERE[1], its_own(1))])
            check("two things, two looks each",
                  sorted(one["observation_count"] for one in store.placed()),
                  [2, 2])

            store.new_map_session()
            a_room(store, [(turned(WHERE[0], TURN, SHIFT), its_own(0)),
                           (turned(WHERE[1], TURN, SHIFT), its_own(1))])
            here = {one["id"] for one in store.placed(store.map_session())}
            reanchor.apply(store, reanchor.plan(store, min_agreeing=2))

            left = store.placed()
            check("two rows, not four", len(left), 2)
            check("...and the rows kept are the ones measured in this map",
                  {one["id"] for one in left}, here)
            check("...each holding every look at it",
                  sorted(one["observation_count"] for one in left), [4, 4])
            check("...and none of the observations was lost",
                  len(store.observations(limit=99)), 8)
            check("...and nothing is left standing in the old map",
                  len(store.placed_elsewhere(store.map_session())), 0)
        finally:
            store.close()


def test_three_old_maps_do_not_each_drop_their_own_copy_of_the_sofa() -> None:
    """**The fault the rover's own store showed on the day this was written.**

    By then the room had been mapped four times over, so the same sofa had a row
    in three maps that were gone and one in the map the rover was on. Planning
    all three old maps against that one at once carries three sofas across, each
    landing beside the others and none of them folded into anything -- because
    what each was measured against was the handful of things standing here
    *before* any of them arrived. The result is the ambiguity the folding exists
    to prevent, arrived at three times over, and from then on every bearing at
    the sofa would be refused as unassignable.

    So they are carried one map at a time, best-supported first, and the store is
    read again in between.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            room = [(spot, its_own(index))
                    for index, spot in enumerate(WHERE[:4])]
            # The same four things, mapped three times, each map its own frame.
            a_room(store, room)
            for turn, shift in ((25.0, (1.0, -1.0)), (-15.0, (-2.0, 3.0))):
                store.new_map_session()
                a_room(store, [(turned(spot, turn, shift), vector)
                               for spot, vector in room])
            # ...and a fourth map, which is the one the rover is on.
            store.new_map_session()
            a_room(store, [(turned(spot, 40.0, SHIFT), vector)
                           for spot, vector in room])
            check("four things standing here, twelve stranded in three maps",
                  (len(store.placed(store.map_session())),
                   len(store.placed_elsewhere(store.map_session()))), (4, 12))

            reanchor.carry_all(store, min_agreeing=3, write=True,
                               say=lambda *a: None)
            check("the room is four things again", len(store.placed()), 4)
            check("...with nothing left stranded",
                  len(store.placed_elsewhere(store.map_session())), 0)
            check("...and each holding every look from all four maps",
                  sorted(one["observation_count"] for one in store.placed()),
                  [8, 8, 8, 8])
        finally:
            store.close()


def test_a_transform_nothing_agrees_on_moves_nothing() -> None:
    """Too little in common is an answer, and the answer is to leave it alone.

    A map the rover barely overlaps with cannot be lined up, and a rotation
    fitted through two coincidences would carry a whole map's worth of positions
    into the wrong room -- which is worse than leaving them where anybody asking
    is told plainly that the map changed.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            a_room(store, [(spot, its_own(index))
                           for index, spot in enumerate(WHERE[:4])])
            store.new_map_session()
            # One thing in the new map, and it is not any of them.
            a_room(store, [((0.0, 4.0), its_own(7))])

            decided = reanchor.plan(store, min_agreeing=3)
            entry = decided["maps"][0]
            check("nothing is carried", len(entry["move"]), 0)
            check("...nothing is folded", len(entry["merge"]), 0)
            check("...every stranded thing is left where it is",
                  len(entry["left"]), 4)
            check("...and the reason says how many agreed",
                  "agree" in (entry.get("why") or ""), True)
            reanchor.apply(store, decided)
            check("the store is untouched",
                  len(store.placed_elsewhere(store.map_session())), 4)
        finally:
            store.close()


def test_two_things_that_land_on_one_double_are_left_alone() -> None:
    """Two chairs side by side must not become one chair.

    What makes a fold safe is that the thing lands on its own double and on
    nothing else. Where a second look-alike is nearly as close, which of them it
    is cannot be told -- the resolver's own rule, and the reason is the same: a
    row wrongly folded into another cannot be taken apart again.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            anchors = [(WHERE[index], its_own(index)) for index in range(3)]
            a_room(store, anchors + [(WHERE[3], its_own(3))])
            chair = store.placed()[-1]["id"]

            store.new_map_session()
            a_room(store, [(turned(spot, TURN, SHIFT), vector)
                           for spot, vector in anchors])
            # ...and where the one chair stood there are now two things that
            # look like it, a metre and a bit apart, with the place the chair
            # maps to squarely between them.
            middle = turned(WHERE[3], TURN, SHIFT)
            standing(store, (middle[0] - 0.6, middle[1]), its_own(3))
            standing(store, (middle[0] + 0.6, middle[1]), much_like(3))

            decided = reanchor.plan(store, min_agreeing=3)
            entry = decided["maps"][0]
            check("the anchors still fix the transform",
                  round(entry["fit"]["turn_deg"]), 40)
            check("...and are folded into their own doubles",
                  len(entry["merge"]), 3)
            check("the chair is left alone rather than guessed at",
                  entry["left"], [chair])
        finally:
            store.close()


def test_a_fold_is_refused_where_one_look_saw_both() -> None:
    """Two regions of one picture are two things, and that outranks everything.

    The store enforces it rather than trusting the caller, because it is the rule
    the resolver's own bookkeeping rests on: a look may give a thing one region
    and no more, so two rows sharing a look cannot be one row however alike they
    look and however close they land.
    """
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        try:
            two = [its_own(0), its_own(1)]
            spots = [(0.0, 5.0), (2.5, 5.0)]
            for index, standing in enumerate([(0.0, 0.0), (6.0, 0.0), (3.0, -4.0)],
                                             start=1):
                a_look(store, standing[0], standing[1],
                       [round(math.degrees(math.atan2(spot[1] - standing[1],
                                                      spot[0] - standing[0])), 2)
                        for spot in spots],
                       vectors=two, inference=index)
            for _ in range(4):
                if not resolve.resolve(store)["created"]:
                    break
            placed = store.placed()
            check("one look put a region on each of two things", len(placed), 2)

            got = store.merge(placed[0]["id"], placed[1]["id"])
            check("the merge is refused", got["ok"], False)
            check("...saying which look saw both",
                  "region in look" in got["why"], True)
            check("...and both things are still there",
                  len(store.entities()), 2)
        finally:
            store.close()


TESTS = (
    test_a_room_seen_twice_gives_up_the_transform_between_its_two_maps,
    test_a_thing_that_was_seen_again_keeps_the_history_of_both,
    test_three_old_maps_do_not_each_drop_their_own_copy_of_the_sofa,
    test_a_transform_nothing_agrees_on_moves_nothing,
    test_two_things_that_land_on_one_double_are_left_alone,
    test_a_fold_is_refused_where_one_look_saw_both,
)
