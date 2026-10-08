"""A look aimed at a thing gives that thing the region it was aimed at.

The case the rule exists for is the painting of 2026-10-07: eight records of one
object, so that a new look's region was equally good for several of them and the
resolver, rightly, filed it under none. Aimed at one of them, the region goes to
that one, the others it fitted are remembered as probably the same object, and
where the look ranged the thing, the range becomes the target's position and
what it claims.
"""
from __future__ import annotations

import json
import math
import struct
import tempfile
import time

from test_harness import check
from test_fakes import a_store
from world_state import aimed, resolve

PLAIN = 384


def _vector(axis, wobble=0.0):
    values = [0.0] * PLAIN
    values[axis] = 1.0
    values[(axis + 1) % PLAIN] = wobble
    return struct.pack(f"<{PLAIN}f", *values)


def _thing(store, entity_id, x_m, y_m, axis=0, uncertainty=0.4):
    placement = {"x_m": x_m, "y_m": y_m, "uncertainty_m": uncertainty,
                 "error_major_m": uncertainty, "error_minor_m": uncertainty / 2,
                 "error_major_deg": 0.0, "extent_m": 0.3, "viewpoints": 2,
                 "baseline_m": 1.0}
    born = time.time() - 100.0
    store.db.execute(
        "INSERT INTO entities(id, kind, label, canonical_description, created_at,"
        " last_seen_at, observation_count, placement_json, placement_uncertainty_m,"
        " placement_map_session, placement_updated_at, exemplars)"
        " VALUES(?, 'object', '', '', ?, ?, 0, ?, ?, 1, ?, ?)",
        (entity_id, born, born, json.dumps(placement), uncertainty, born,
         _vector(axis) * 3))
    store.db.commit()


def _region(store, look, at, target, entity_id=None, axis=0, range_m=None,
            off_deg=0.0):
    (px, py), (tx, ty) = at, target
    bearing = math.degrees(math.atan2(ty - py, tx - px)) + off_deg
    oid = store.db.execute(
        "INSERT INTO observations(entity_id, inference_id, observed_at, source,"
        " frame_id, bbox_json, observer_pose_json, map_session, bearing_deg, span_deg,"
        " dino_blob, range_m, range_sigma_m)"
        " VALUES(?,?,?,?,?,?,?,1,?,5.0,?,?,?)",
        (entity_id, look, time.time() - 90.0 + look, "test", f"frame-{look}",
         "[0.4, 0.4, 0.6, 0.6]",
         json.dumps({"x_m": px, "y_m": py, "heading_deg": 0.0}), bearing,
         _vector(axis), range_m, None if range_m is None else 0.05)).lastrowid
    store.db.commit()
    return oid


PAINTING = (3.0, 0.0)
SHELF = (0.0, 4.0)


def _fresh():
    store = a_store(tempfile.mkdtemp(prefix="world-aimed-"))
    store.db.execute("REPLACE INTO meta(key, value) VALUES('map_session', '1')")
    store.db.commit()
    return store


def test_the_region_at_the_aim_goes_to_the_target():
    store = _fresh()
    # Two records of one painting, a hand's breadth apart, and a shelf.
    _thing(store, "object:1", 3.0, 0.0)
    _thing(store, "object:2", 3.1, 0.05)
    _thing(store, "object:3", 0.0, 4.0, axis=5)
    painting = _region(store, 7, (0.0, 0.0), PAINTING)
    shelf = _region(store, 7, (0.0, 0.0), SHELF, axis=5)
    got = aimed.file_by_aim(store, "object:1", "frame-7")
    owner = dict(store.db.execute("SELECT id, entity_id FROM observations"))
    check("aimed at one record of the painting, its region goes to that record",
          (got["filed"], owner[painting]), (painting, "object:1"))
    check("...and the shelf's region is left for the resolver", owner[shelf], None)
    check("...and the other record it fitted is kept as probably the same object",
          got["same_object_suspects"], ["object:2"])
    check("...both ways round",
          store.same_object_suspects(1), {"object:1": ["object:2"], "object:2": ["object:1"]})
    exemplars = store.db.execute(
        "SELECT LENGTH(exemplars) FROM entities WHERE id='object:1'").fetchone()[0]
    check("...and it is not made one of the target's exemplars", exemplars, 3 * PLAIN * 4)


def test_a_record_the_region_could_have_been_filed_to_is_a_suspect():
    """The rug of 2026-10-08: seven records, whose filed regions looked 0.57 to
    0.68 like each other -- under `RECOGNISED`, so none was named and the run went
    back to the rug by a second record."""
    store = _fresh()
    _thing(store, "object:1", 3.0, 0.0)
    _thing(store, "object:2", 3.1, 0.05)
    _thing(store, "object:3", 3.0, -0.05)
    # Cosines of about 0.62 and 0.45 against the region: one passes filing's
    # own bar, the other does not.
    store.db.execute("UPDATE entities SET exemplars=? WHERE id='object:2'",
                     (_vector(0, 1.265) * 3,))
    store.db.execute("UPDATE entities SET exemplars=? WHERE id='object:3'",
                     (_vector(0, 2.0) * 3,))
    store.db.commit()
    _region(store, 13, (0.0, 0.0), PAINTING)
    got = aimed.file_by_aim(store, "object:1", "frame-13")
    check("a record it fits and looks 0.62 like is a suspect; one at 0.45 is not",
          got["same_object_suspects"], ["object:2"])


def test_a_look_that_does_not_point_at_the_target_files_nothing():
    store = _fresh()
    _thing(store, "object:1", 3.0, 0.0)
    shelf = _region(store, 8, (0.0, 0.0), SHELF)
    got = aimed.file_by_aim(store, "object:1", "frame-8")
    check("nothing in the picture points at the target: nothing is filed",
          (got["filed"], store.db.execute(
              "SELECT entity_id FROM observations WHERE id=?", (shelf,)).fetchone()[0]),
          (None, None))
    got = aimed.file_by_aim(store, "object:9", "frame-8")
    check("...and a target that is not placed says so", got["filed"], None)


def test_two_regions_alike_at_the_aim_are_refused():
    store = _fresh()
    _thing(store, "object:1", 3.0, 0.0)
    _region(store, 9, (0.0, 0.0), PAINTING, off_deg=0.2)
    _region(store, 9, (0.0, 0.0), PAINTING, off_deg=-0.2)
    got = aimed.file_by_aim(store, "object:1", "frame-9")
    check("two regions point at it and look alike: neither is guessed",
          (got["filed"], got["why"]), (None, "two regions point at it and look alike"))


def test_an_aimed_range_is_the_position_and_the_claim():
    store = _fresh()
    # Placed from bearings 0.4 m off where the painting really is.
    _thing(store, "object:1", 3.3, 0.3, uncertainty=0.5)
    # Bearings a few degrees out, as the rover's are (2026-10-07).
    for look, at, off in ((1, (0.0, 0.0), 3.0), (2, (0.0, 1.5), -4.0),
                          (3, (1.0, -1.5), 2.5)):
        _region(store, look, at, (3.3, 0.3), entity_id="object:1", off_deg=off)
    resolve._replace_placement(store, "object:1", 1)
    before = json.loads(store.db.execute(
        "SELECT placement_json FROM entities WHERE id='object:1'").fetchone()[0])
    # Aimed from (0.5, 0.3): pointed at the painting where it really is, and
    # ranged to it.
    true = PAINTING
    at = (0.5, 0.3)
    _region(store, 10, at, true, range_m=math.hypot(true[0] - at[0], true[1] - at[1]))
    got = aimed.file_by_aim(store, "object:1", "frame-10")
    after = json.loads(store.db.execute(
        "SELECT placement_json FROM entities WHERE id='object:1'").fetchone()[0])
    check("an aimed look that ranged it is filed", got["ranged"], True)
    check("...and its range moves the position to where the painting is",
          (round(math.hypot(before["x_m"] - true[0], before["y_m"] - true[1]), 1),
           round(math.hypot(after["x_m"] - true[0], after["y_m"] - true[1]), 1)),
          (0.3, 0.0))
    check("...and the claim is the range's, smaller than the bearings'",
          after["stated_uncertainty_m"] < before["stated_uncertainty_m"], True)
    check("...but never below what a single range has been measured to be worth",
          after["stated_uncertainty_m"] >= aimed.CLAIM_FLOOR_M, True)
    from world_state import locate
    rays = [resolve.ray_of(one) for one in store.observations("object:1")]
    bearings_only = locate.refine(locate.best_fix(rays), rays)
    check("...while the tolerance it is matched with stays the bearings' own",
          after["uncertainty_m"], bearings_only["uncertainty_m"])


def test_an_aimed_range_that_disagrees_widens_the_claim():
    """A kitchen cabinet on the rover on 2026-10-08: two aimed ranges 1.25 m
    apart, the one that no longer pointed at the placement dropped, and the
    other claiming 0.20 m."""
    store = _fresh()
    _thing(store, "object:1", 3.0, 0.0)
    agrees = _region(store, 11, (0.0, 0.0), PAINTING, entity_id="object:1", range_m=3.0)
    # From further along, ranged to something two metres to the side.
    off = _region(store, 12, (0.0, 2.0), (3.0, 2.0), entity_id="object:1", range_m=3.0)
    placement = json.loads(store.db.execute(
        "SELECT placement_json FROM entities WHERE id='object:1'").fetchone()[0])
    rays = [resolve.ray_of(one) for one in store.observations("object:1")]
    alone = aimed.aimed_claim(rays, {agrees}, placement)
    both = aimed.aimed_claim(rays, {agrees, off}, placement)
    check("one aimed range on its own claims the floor",
          alone["aimed_uncertainty_m"], aimed.CLAIM_FLOOR_M)
    check("...a second that disagrees gives no position",
          (round(both["x_m"], 1), round(both["y_m"], 1), both["aimed_ranges"]),
          (3.0, 0.0, 1))
    check("...but widens the claim to the disagreement rather than vanishing",
          (both["aimed_uncertainty_m"] > 1.0, both["aimed_ranges_disagreeing"]), (True, 1))


def test_an_inspection_aimed_at_a_thing_says_what_it_filed():
    from test_fakes import a_seeing_inspector, a_sighting
    directory = tempfile.mkdtemp(prefix="world-aimed-inspect-")
    store, _eyes, inspector = a_seeing_inspector(
        directory, looks=[[a_sighting(bbox=[0.45, 0.4, 0.55, 0.6])]])
    plain = inspector.inspect(settle=False)
    check("a look with no target carries no filing", "aimed_filing" in plain, False)
    store2, _eyes2, inspector2 = a_seeing_inspector(
        tempfile.mkdtemp(prefix="world-aimed-inspect-"),
        looks=[[a_sighting(bbox=[0.45, 0.4, 0.55, 0.6])]])
    aimed_at = inspector2.inspect(settle=False, target="object:404")
    check("a look aimed at a thing says what came of filing it",
          aimed_at.get("aimed_filing", {}).get("filed"), None)
    check("...and why, here that the thing is not placed",
          "not placed" in aimed_at.get("aimed_filing", {}).get("why", ""), True)


def test_an_aimed_look_takes_its_own_picture():
    """On the rover on 2026-10-08 five aimed looks in eight matched the picture
    taken a second earlier on arrival, recorded nothing and filed nothing."""
    from test_fakes import a_camera_showing, a_seeing_inspector, a_sighting
    from test_harness import SKIP
    from test_inspect import _room
    same = _room()
    if same is None:
        SKIP.append("an aimed look after the same picture (no numpy or OpenCV)")
        return
    looks = [[a_sighting(bbox=[0.45, 0.4, 0.55, 0.6])] for _ in range(2)]
    results = {}
    for name, target in (("plain", None), ("aimed", "object:404")):
        _store, _eyes, inspector = a_seeing_inspector(
            tempfile.mkdtemp(prefix="world-aimed-again-"), [list(l) for l in looks],
            capture=a_camera_showing([same]))
        inspector.inspect(settle=False)
        results[name] = inspector.inspect(settle=False, target=target)
    check("the same picture twice: an ordinary look records nothing the second time",
          results["plain"].get("status"), "unchanged")
    check("...but a look aimed at a thing records it and says what it filed",
          (results["aimed"].get("status"), "aimed_filing" in results["aimed"]),
          ("ok", True))


TESTS = (
    test_the_region_at_the_aim_goes_to_the_target,
    test_a_record_the_region_could_have_been_filed_to_is_a_suspect,
    test_a_look_that_does_not_point_at_the_target_files_nothing,
    test_two_regions_alike_at_the_aim_are_refused,
    test_an_aimed_range_is_the_position_and_the_claim,
    test_an_aimed_range_that_disagrees_widens_the_claim,
    test_an_inspection_aimed_at_a_thing_says_what_it_filed,
    test_an_aimed_look_takes_its_own_picture,
)
