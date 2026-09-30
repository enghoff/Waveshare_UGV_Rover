"""Redrawing the store through a refitted lens: what reproduces, and only that.

The migration touches the one thing this component otherwise promises never to
touch -- a direction the rover measured -- so what is checked is its refusal as
much as its arithmetic: a look the old lens does not reproduce is left exactly
as it was, a range is replayed from its own depth map through the geometry the
rover had then before anything is rewritten, and a thing moves only as far as
its own corrected looks say.

    python3 test_relens.py
"""
from __future__ import annotations

import gzip
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from test_fakes import a_store, a_vector                          # noqa: E402
from test_harness import check                                    # noqa: E402
from world_state import locate, oak, relens                       # noqa: E402
from world_state.perception_client import Sighting               # noqa: E402

#: Well off the lens axis, where the old lens and the refitted one disagree most.
OFF_AXIS = [0.10, 0.30, 0.20, 0.50]


def _record(store, x, y, heading, bbox=OFF_AXIS, frame_id="f", ranges=None):
    seen = [Sighting(bbox=bbox, dino=a_vector(1.0, 0.0),
                     siglip=a_vector(0.5, 0.5))]
    store.record(seen, capture={"frame_id": frame_id, "pan": 0.0, "tilt": 10.0,
                                "frame_size": (640, 480),
                                "pose": {"x_m": x, "y_m": y,
                                         "heading_deg": heading}},
                 fov_deg=130.0, region_source="yoloe", vectors_from="fake",
                 ranges=ranges)
    return int(store.db.execute("SELECT MAX(id) FROM observations").fetchone()[0])


def _through_old(action):
    relens.through(relens.PREVIOUS_LENS)
    try:
        return action()
    finally:
        relens.through(None)


def test_only_a_look_the_old_lens_reproduces_is_redrawn() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        old = _through_old(lambda: _record(store, 0.0, 0.0, 0.0))
        new = _record(store, 1.0, 0.0, 0.0)            # already the new lens
        result = relens.plan(store, store.frames_dir, relens.CURRENT_OAK_LENS)
        check("a look taken through the old lens reproduces and is redrawn",
              sorted(result["changes"]), [old])
        row, change = result["changes"][old]
        stored_new = store.db.execute(
            "SELECT bearing_deg FROM observations WHERE id = ?", (new,)
        ).fetchone()[0]
        check("...to exactly what today's lens draws for it",
              change["bearing_deg"], stored_new)
        check("...which is further round than the old lens had it",
              abs(change["bearing_deg"]) > abs(row["bearing_deg"]), True)
        check("a look the old lens does not reproduce is not in the plan",
              new in result["changes"], False)
        store.close()


def test_a_range_is_replayed_from_its_depth_map_before_it_is_redone() -> None:
    """A flat wall three metres out, seen through the chassis bracket."""
    import numpy

    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        box = [0.45, 0.40, 0.55, 0.50]
        observation = _through_old(lambda: _record(store, 0.0, 0.0, 0.0, box,
                                                   frame_id="wall"))
        store.db.execute("UPDATE observations SET observed_at = ? WHERE id = ?",
                         (oak.RAIL_SINCE - 100.0, observation))
        store.db.commit()
        depth = numpy.full((180, 320), 3000, dtype="<u2")
        header = json.dumps({"width": 320, "height": 180, "dtype": "uint16",
                             "unit": "mm", "age_s": 0.1, "apart_s": 0.0}).encode()
        with gzip.open(os.path.join(store.frames_dir, "wall.depth.gz"), "wb") as f:
            f.write(len(header).to_bytes(4, "little") + header + depth.tobytes())

        # What the rover stored: the old pipeline, run as it ran then.
        row = dict(store.db.execute("SELECT * FROM observations WHERE id = ?",
                                    (observation,)).fetchone())
        row["bbox"] = json.loads(row["bbox_json"])
        service = relens._depth_service()
        then = _through_old(lambda: relens.replay_range(
            row, depth, relens.PREVIOUS_OAK_LENS, relens.PREVIOUS_CHASSIS_MOUNT,
            relens.extractor(service, relens.PREVIOUS_OAK_LENS)))
        check("the old pipeline ranges a wall in the middle of the picture",
              then[0] is not None, True)
        store.db.execute("UPDATE observations SET range_m = ?, range_sigma_m = ?"
                         " WHERE id = ?", (then[0], then[1], observation))
        store.db.commit()

        result = relens.plan(store, store.frames_dir, relens.CURRENT_OAK_LENS)
        check("the replay gives back what was stored, so it is redone",
              result["range_kept"], 1)
        _row, change = result["changes"][observation]
        check("...through the corrected geometry, still on the wall",
              change["range_m"] is not None and abs(change["range_m"] - 3.0) < 0.2,
              True)
        store.db.execute("UPDATE observations SET range_m = ? WHERE id = ?",
                         (then[0] + 0.5, observation))
        store.db.commit()
        refused = relens.plan(store, store.frames_dir, relens.CURRENT_OAK_LENS)
        check("a stored range the replay does not give back is left alone",
              (refused["range_kept"], refused["range_refused"],
               "range_m" in refused["changes"][observation][1]), (0, 1, False))
        store.close()


def test_apply_keeps_a_copy_and_moves_a_thing_to_where_its_looks_cross() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store = a_store(directory)
        # Two looks from 1.5 m apart, each with the thing well off the lens axis,
        # so the old lens and the new one cross them in different places.
        looks = [_through_old(lambda: _record(store, 0.0, 0.0, 30.0)),
                 _through_old(lambda: _record(store, 1.5, 0.0, 60.0))]
        session = store.map_session()
        entity = store.create_entity()
        store.attach(entity, looks, "test")
        placed = relens.placement_for(store, {"id": entity,
                                              "placement_map_session": session},
                                      None)
        store.place(entity, placed, session)
        result = relens.plan(store, store.frames_dir, relens.CURRENT_OAK_LENS)
        done = relens.apply(store, store.path, result, None, say=lambda _line: None)
        check("the database was copied before anything was written",
              os.path.exists(done["backup"]), True)
        after = store.entity(entity)["placement"]
        rays = [ray for ray in (relens.resolve.ray_of(one, None) for one in
                                store.observations(entity)) if ray]
        crossing = locate.fix(rays[0], rays[1])
        check("the thing now stands where its redrawn looks cross",
              (round(after["x_m"], 2), round(after["y_m"], 2)),
              (round(crossing["x_m"], 2), round(crossing["y_m"], 2)))
        check("...which is not where the old lens had put it",
              (round(placed["x_m"], 2), round(placed["y_m"], 2))
              != (round(after["x_m"], 2), round(after["y_m"], 2)), True)
        check("and it is still the same thing", store.entity(entity)["id"], entity)
        store.close()


def test_a_place_the_resolver_built_up_moves_only_by_what_the_lens_changed() -> None:
    """A thing's stored place is history, built from whichever looks the resolver
    had at the time, and need not be where a fresh fit of its looks lands. The
    redraw must move it by what the lens changed and not snap it to the fresh
    fit, which would be the method's doing rather than the lens's."""
    stored = {"x_m": 2.30, "y_m": 1.00, "height_m": 0.40, "uncertainty_m": 0.2}
    old_fit = {"x_m": 2.00, "y_m": 1.00, "height_m": 0.30}
    new_fit = {"x_m": 2.10, "y_m": 0.90, "height_m": 0.35}
    moved = relens.moved_placement(stored, old_fit, new_fit)
    check("a place the fresh fit does not reproduce is shifted by the lens's change",
          (moved["x_m"], moved["y_m"], moved["height_m"]), (2.4, 0.9, 0.45))
    check("...keeping everything else it held",
          moved["uncertainty_m"], 0.2)
    check("a place the fresh fit reproduces becomes the new fit",
          relens.moved_placement(dict(old_fit), old_fit, new_fit), new_fit)
    check("one whose redrawn looks agree on nowhere is unplaced",
          relens.moved_placement(stored, old_fit, None), None)
    check("and one its old looks never placed either is left as it was",
          relens.moved_placement(stored, None, new_fit), stored)


TESTS = (
    test_a_place_the_resolver_built_up_moves_only_by_what_the_lens_changed,
    test_only_a_look_the_old_lens_reproduces_is_redrawn,
    test_a_range_is_replayed_from_its_depth_map_before_it_is_redone,
    test_apply_keeps_a_copy_and_moves_a_thing_to_where_its_looks_cross,
)


def main() -> int:
    from test_harness import FAIL, PASS

    for one in TESTS:
        try:
            one()
        except Exception as error:                                # noqa: BLE001
            FAIL.append(f"{one.__name__} raised {type(error).__name__}: {error}")
    for line in FAIL:
        print("FAIL " + line)
    print(f"{len(PASS)} passed, {len(FAIL)} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
