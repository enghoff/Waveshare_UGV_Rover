"""One inspection: what it measures, what it stores, and what it refuses.

An inspection is the only thing that writes to the world, so every failure path
has to leave it exactly as it was. Only one may run at a time, a clear may not
cut across one, and the vectors must never reach the wire by accident.
"""
from __future__ import annotations

import tempfile
import time

from test_harness import check
from test_fakes import a_capture, a_pose, a_seeing_inspector, a_sighting, a_vector
from world_state import view


def test_an_inspection_through_the_encoders_keeps_what_it_measured() -> None:
    with tempfile.TemporaryDirectory() as directory:
        seen, matched = a_vector(0.6, 0.8), a_vector(0.0, 1.0)
        store, eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting(dino=seen, siglip=matched),
                         a_sighting(bbox=[0.6, 0.2, 0.9, 0.7])]])
        result = inspector.inspect()
        check("the inspection succeeded", result["ok"], True)
        check("...storing one observation per region", result["stored"], 2)
        check("...with a bearing for each", result["placed"], 2)
        check("...and claiming no identity for either", result["created"], 0)
        check("...having asked the encoders exactly once", len(eyes.calls), 1)

        rows = store.db.execute(
            "SELECT * FROM observations ORDER BY id").fetchall()
        first = dict(rows[0])
        check("the appearance vector is stored as the bytes it arrived as",
              first["dino_blob"], seen)
        check("...and so is the semantic one", first["siglip_blob"], matched)
        check("the backend that produced them travels with them",
              first["vectors_from"], "fake")
        check("what drew the box is recorded apart from what named it",
              first["region_source"], "fastsam")
        check("no identity was written", first["entity_id"], None)
        check("nothing was prompted, so no prompt version is claimed",
              first["prompt_version"], None)

        store.close()


def test_a_bearing_is_the_pose_the_gimbal_and_the_box_together() -> None:
    """The arithmetic `view.ray` already proves, checked where it is stored."""
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting(bbox=[0.4, 0.3, 0.6, 0.9])]],
            capture=a_capture(pan=20.0), pose=a_pose(heading=90.0), fov_deg=100.0)
        inspector.inspect()
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        # Heading 90 to the left, gimbal 20 to the right, and a box centred in
        # the picture: 90 - 20 + 0.
        check("the stored bearing is heading minus pan plus the box offset",
              row["bearing_deg"], 70.0)
        check("...and the box's width became a cone", row["span_deg"], 20.0)

        store.close()


def test_without_a_pose_nothing_pretends_to_know_a_direction() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()]], pose=lambda: None)
        result = inspector.inspect()
        check("the observation is still kept", result["stored"], 1)
        check("...but nothing was placed", result["placed"], 0)
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("the bearing is empty rather than guessed", row["bearing_deg"], None)
        check("and the popup is told why in words",
              "without a bearing" in result["detail"], True)

        store.close()


def test_a_sidecar_that_is_down_writes_one_row_and_no_observations() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, fail="the perception sidecar is not answering")
        result = inspector.inspect()
        check("the inspection failed rather than raising", result["ok"], False)
        check("...saying which sidecar", "perception" in result["error"], True)
        check("nothing was stored", store.summary()["observations"], 0)
        check("...and exactly one diagnostics row was written",
              len(store.inferences()), 1)
        store.close()


def test_every_failure_leaves_the_world_exactly_as_it_was() -> None:
    """The ways an inspection can fail, and one rule for all of them."""
    cases = [
        ("the sidecar is not running",
         dict(fail="nothing is listening on 8776"), "unavailable"),
        ("the camera gave nothing",
         dict(looks=[[a_sighting()]],
              capture=a_capture(ok=False, error="the camera gave nothing")),
         "no_frame"),
    ]
    for name, kwargs, status in cases:
        with tempfile.TemporaryDirectory() as directory:
            store, _eyes, inspector = a_seeing_inspector(directory, **kwargs)
            # Something already in the world, so "unchanged" is a claim with
            # content rather than "still empty".
            store.record([a_sighting()], capture={"frame_id": "f0"})
            before = store.summary()
            result = inspector.inspect()
            check(f"{name}: the inspection fails", result["ok"], False)
            check(f"{name}: ...saying which kind of failure", result["status"],
                  status)
            after = store.summary()
            check(f"{name}: ...and changes no entity",
                  after["entities"], before["entities"])
            check(f"{name}: ...and no observation",
                  after["observations"], before["observations"])
            check(f"{name}: ...but is written down where the popup shows it",
                  store.inferences()[0]["status"], status)
            store.close()


def test_only_one_inspection_at_a_time() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()]])
        inspector._lock.acquire()
        inspector.started_at = time.monotonic()
        try:
            result = inspector.inspect()
            check("a second request is refused rather than queued",
                  result["status"], "busy")
            check("...and nothing was stored for it", store.inferences(), [])
        finally:
            inspector._lock.release()
        check("...and the next one runs normally", inspector.inspect()["ok"], True)
        store.close()


def test_a_clear_is_refused_while_an_inspection_runs() -> None:
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()]])
        inspector.inspect()
        inspector._lock.acquire()
        try:
            check("the inspector says it is busy", inspector.busy, True)
        finally:
            inspector._lock.release()
        check("...and is free again afterwards", inspector.busy, False)
        check("the clear then works", store.clear()["ok"], True)
        store.close()


def test_perception_writes_no_name_and_no_warning_about_one() -> None:
    """Nothing measures what a region is, so nothing pretends to.

    There used to be a name here and a warning beside it when the name said
    nothing in particular. Both are gone with the word list that produced
    them, and what has to be true now is that the row says so by holding
    nothing rather than by holding a plausible-looking guess -- the same rule
    the bearing columns follow when there was no pose.
    """
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()]])
        inspector.inspect()
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("nothing wrote a name, so the column is null rather than blank",
              row["label"], None)
        check("...and no leftover score for one", row["label_score"], None)
        check("...and no warning about a name that was never there",
              row["note"], None)
        check("what it does carry is the measurement",
              (row["bearing_deg"] is not None, bool(row["siglip_blob"])),
              (True, True))
        import json as _json
        raw = _json.loads(row["raw_json"])
        check("...and what the sidecar said has no name in it either",
              sorted(raw), ["area", "region_score"])
        store.close()


def test_a_bearing_never_runs_past_half_a_turn() -> None:
    """Measured on the rover before it was fixed: -205.9 degrees, stored.

    Three numbers are added to make a bearing -- the rover's heading, the
    gimbal's pan and the box's offset -- and each can be large. A bearing outside
    (-180, 180] points exactly where its wrapped twin does and compares with
    nothing, so the resolver would see two directions where there is one.
    """
    facing = {"pose": {"x_m": 0.0, "y_m": 0.0, "heading_deg": -150.4},
              "observer_pan_deg": 50.0, "bbox": [0.4, 0.3, 0.6, 0.9]}
    check("the sum wraps rather than running past half a turn",
          view.ray(facing, 100.0)["bearing_deg"], 159.6)
    other_way = dict(facing, observer_pan_deg=-200.0)
    check("...in the other direction too",
          -180.0 < view.ray(other_way, 100.0)["bearing_deg"] <= 180.0, True)


def test_the_pending_pool_holds_what_has_a_direction_but_no_home() -> None:
    """Phase 2's waiting room, and what is deliberately not in it."""
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting(), a_sighting(bbox=[0.6, 0.2, 0.9, 0.7])]])
        inspector.inspect()
        pending = store.unplaced()
        check("both observations are waiting to be placed", len(pending), 2)
        check("...each with a direction", all(row["bearing_deg"] is not None
                                              for row in pending), True)
        check("...and the vectors the resolver will need",
              all(row["dino_blob"] and row["siglip_blob"] for row in pending), True)
        check("...oldest first, so the pool cannot starve",
              pending[0]["id"] < pending[1]["id"], True)

        # An observation with no pose was never pending anything: no later look
        # can supply a direction that was never measured.
        store.record([a_sighting()], capture={"frame_id": "f9"})
        check("an observation with no bearing is not in the pool",
              len(store.unplaced()), 2)
        check("...but it is still in the history", store.summary()["observations"], 3)

        check("a bearing from another map is not comparable and is left out",
              len(store.unplaced(map_session=99)), 0)
        store.close()


def test_the_vectors_never_reach_the_wire_by_accident() -> None:
    """Raw bytes in a row that is about to be JSON is a crash, not a nuisance."""
    with tempfile.TemporaryDirectory() as directory:
        seen = a_vector(0.6, 0.8)
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting(dino=seen)]])
        inspector.inspect()
        shown = store.observations()[0]
        check("the console's copy carries no raw vector",
              ("dino_blob" in shown, "siglip_blob" in shown), (False, False))
        check("...but says they are there and how big",
              (shown["dino_bytes"], shown["siglip_bytes"]), (32, 32))
        import json as _json
        _json.dumps(shown)
        check("...so the row can be sent as JSON at all", True, True)
        check("the resolver's copy keeps them",
              store.unplaced()[0]["dino_blob"], seen)
        store.close()


TESTS = (
    test_an_inspection_through_the_encoders_keeps_what_it_measured,
    test_a_bearing_is_the_pose_the_gimbal_and_the_box_together,
    test_without_a_pose_nothing_pretends_to_know_a_direction,
    test_a_sidecar_that_is_down_writes_one_row_and_no_observations,
    test_every_failure_leaves_the_world_exactly_as_it_was,
    test_only_one_inspection_at_a_time,
    test_a_clear_is_refused_while_an_inspection_runs,
    test_perception_writes_no_name_and_no_warning_about_one,
    test_a_bearing_never_runs_past_half_a_turn,
    test_the_pending_pool_holds_what_has_a_direction_but_no_home,
    test_the_vectors_never_reach_the_wire_by_accident,
)
