"""One inspection: what it measures, what it stores, and what it refuses.

An inspection is the only thing that writes to the world, so every failure path
has to leave it exactly as it was. Only one may run at a time, a clear may not
cut across one, and the vectors must never reach the wire by accident.
"""
from __future__ import annotations

import tempfile
import time

from test_harness import check
from test_fakes import (a_capture, a_pose, a_seeing_inspector, a_sighting,
                        a_turning_pose, a_vector)
from world_state import locate
from world_state import view
from world_state.inspector import FRAME_TIME_SIGMA_S, Inspector


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
              first["region_source"], "yoloe")
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
        # the picture: 90 - 20, less the 0.8 degrees by which the middle of the
        # frame is not the lens axis. `fov_deg` is passed and no longer decides
        # anything -- the angle comes through the swept lens now, chosen by the
        # frame's size -- and it is left here because it is still the switch
        # that says the caller knows what the camera saw.
        check("the stored bearing is heading minus pan plus the box offset",
              row["bearing_deg"], 69.2)
        check("...and the box's width became a cone", row["span_deg"], 25.6)

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
          view.ray(facing, 100.0)["bearing_deg"], 158.8)
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


def test_a_look_taken_on_the_move_keeps_the_picture_and_drops_the_bearing() -> None:
    """The pose is read either side of the shutter, and the gap is the verdict.

    A bounded grab is 0.36 s on the rover and the rover may be driving through
    all of it, so a bearing drawn from one reading taken afterwards is drawn from
    where the rover ended up. **Travelling and turning are answered differently
    and that is the point**: travel shifts where the ray starts, which is a
    residual the geometry can be told about, and turning swings where it points,
    which is a crossing in the wrong place. So an ordinary drive keeps its
    bearing and says how good it is, and a turn still costs the look its
    direction -- while the picture, the regions and the vectors are kept either
    way, because a rover recording once a second is recording pictures.
    """
    with tempfile.TemporaryDirectory() as directory:
        driving = iter([{"x_m": 0.0, "y_m": 0.0, "heading_deg": 90.0},
                        {"x_m": 0.0, "y_m": 0.17, "heading_deg": 90.0}])
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()]], pose=lambda: next(driving))
        result = inspector.inspect()
        check("an ordinary look taken while driving straight keeps its bearing",
              result["placed"], 1)
        row = dict(store.db.execute(
            "SELECT bearing_deg, origin_sigma_m FROM observations").fetchone())
        check("...and records how far out its own starting point may be",
              row["origin_sigma_m"], 0.085)
        check("...which is half of what the rover covered, not all of it",
              round(result["moved_m"] / 2.0, 3), row["origin_sigma_m"])
        store.close()

    with tempfile.TemporaryDirectory() as directory:
        walking = iter([{"x_m": 0.0, "y_m": 0.0, "heading_deg": 90.0},
                        {"x_m": 0.0, "y_m": 0.6, "heading_deg": 90.0}])
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()]], pose=lambda: next(walking))
        result = inspector.inspect()
        check("the look was recorded", result["stored"], 1)
        check("...with no bearing, because the rover covered too much ground",
              result["placed"], 0)
        check("...and says how far it moved for it", result["moved_m"], 0.6)
        check("...in a sentence rather than a silence",
              "while the shutter was open" in result["detail"], True)
        row = store.db.execute("SELECT frame_id, bearing_deg, dino_blob"
                               " FROM observations").fetchone()
        check("...the picture is kept", bool(row["frame_id"]), True)
        check("...and the vector with it", len(row["dino_blob"]), 32)
        check("...and the direction is absent rather than guessed",
              row["bearing_deg"], None)
        store.close()

    with tempfile.TemporaryDirectory() as directory:
        swinging = iter([{"x_m": 0.0, "y_m": 0.0, "heading_deg": 90.0},
                         {"x_m": 0.0, "y_m": 0.02, "heading_deg": 104.0}])
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()]], pose=lambda: next(swinging))
        result = inspector.inspect()
        check("a look taken mid-turn still loses its bearing, however little "
              "ground it covered", result["placed"], 0)
        check("...and the sentence blames the turn rather than the travel",
              "swings the bearing" in result["detail"], True)
        store.close()

    with tempfile.TemporaryDirectory() as directory:
        creeping = iter([{"x_m": 0.0, "y_m": 0.0, "heading_deg": 90.0},
                         {"x_m": 0.0, "y_m": 0.08, "heading_deg": 91.0}])
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()]], pose=lambda: next(creeping))
        result = inspector.inspect()
        check("a look taken at a walk still gets its bearing",
              result["placed"], 1)
        check("...from halfway through the shutter, not from either end",
              store.unplaced()[0]["pose"]["y_m"], 0.04)
        store.close()


def test_looking_and_settling_are_separately_paced() -> None:
    """A rover that looks once a second cannot settle once a second.

    One resolver pass over a pool of 500 bearings that can place things is 55 s on
    the rover, because it compares every pair and then asks every ray whether it
    agrees with each crossing that survived; a look is a near-constant 0.45 s. So
    the caller decides, and a look that did not settle has to say so rather than
    reporting nothing found.
    """
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()], [a_sighting(bbox=[0.6, 0.2, 0.9, 0.7])]])
        quick = inspector.inspect(settle=False)
        check("a look that did not settle stored its regions anyway",
              quick["stored"], 1)
        check("...and says identity is not settled", quick["settled"], False)
        check("...in the line the popup reads",
              "identity not settled yet" in quick["detail"], True)
        check("...leaving the bearing pending", len(store.unplaced()), 1)
        inspector.inspect(settle=False)
        settled = inspector.settle()
        check("settling on its own is an ordinary call", settled["ok"], True)
        check("...and considers everything that has piled up",
              settled["considered"], 2)
        store.close()


def test_the_heading_is_taken_at_the_shutter_rather_than_averaged() -> None:
    """**The fault that cost the drive of 2026-09-03 two thirds of its bearings.**
    71 of its 108 looks stored no direction for anything they saw, every one
    because the rover was turning while the shutter was open. The reason was
    never that a turn makes the bearing unknowable: it was that a bracket of two
    pose readings cannot say where in itself the picture was taken. The camera
    always knew -- every frame it hands back carries the moment -- and both paths
    through it were dropping it.
    """
    inspector = Inspector.__new__(Inspector)
    at_shutter = inspector._at_the_shutter

    check("a picture taken at the start of the bracket is the first reading",
          at_shutter(100.0, 100.4, 100.0, 20.0)[0], 0.0)
    check("...at the end of it, the second", at_shutter(100.0, 100.4, 100.4,
                                                        20.0)[0], 1.0)
    check("...and halfway through, the midpoint the old code assumed",
          at_shutter(100.0, 100.4, 100.2, 20.0)[0], 0.5)

    # What is left over is the turn rate times how well the instant is known.
    # 20 degrees over 0.4 s is 50 deg/s, and 30 ms of that is 1.5 degrees.
    _share, sigma = at_shutter(100.0, 100.4, 100.2, 20.0)
    check("what is left over is the turn rate times the timing error",
          round(sigma, 2), round(50.0 * FRAME_TIME_SIGMA_S, 2))
    check("a rover standing still leaves nothing over",
          at_shutter(100.0, 100.4, 100.2, 0.0)[1], 0.0)

    check("no timestamp cannot be interpolated to",
          at_shutter(100.0, 100.4, None, 20.0), (None, None))
    check("...nor an instant before the bracket",
          at_shutter(100.0, 100.4, 99.9, 20.0), (None, None))
    check("...nor one after it, which is the tracking loop's stale frame",
          at_shutter(100.0, 100.4, 100.5, 20.0), (None, None))
    check("...nor a bracket of no length at all",
          at_shutter(100.0, 100.0, 100.0, 20.0), (None, None))


def test_a_turning_look_keeps_a_wide_bearing_instead_of_none() -> None:
    """Travel already bought a wider answer rather than no answer; turning does
    now too, and by the same means -- the residual is carried on the observation
    instead of being the reason to throw the look away."""
    inspector = Inspector.__new__(Inspector)
    before = {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0}
    after = {"x_m": 0.0, "y_m": 0.0, "heading_deg": 20.0}

    where, moved, turned, sigma = inspector._where(
        before, after, before_at=100.0, after_at=100.4, taken_at=100.1)
    check("a look taken while turning keeps its bearing", where is not None, True)
    check("...aimed a quarter of the way through the turn, not half",
          where["heading_deg"], 5.0)
    check("...and says how much the turn cost it", sigma, 1.5)
    check("...while still reporting the turn itself", turned, 20.0)

    check("the same look with no timestamp is refused, as it always was",
          inspector._where(before, after, before_at=100.0, after_at=100.4,
                           taken_at=None)[0], None)

    # A rover spinning on the spot: 120 degrees in 0.4 s is 300 a second, which
    # leaves 9 degrees of cone at 30 ms and is past MAX_BEARING_SIGMA_DEG. A
    # driving rover does not reach this -- the drive of 2026-09-03 peaked at 100
    # degrees a second -- and that is the point of where the limit sits.
    fast = inspector._where({"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0},
                            {"x_m": 0.0, "y_m": 0.0, "heading_deg": 120.0},
                            before_at=100.0, after_at=100.4, taken_at=100.2)
    check("a bearing too wide to cross another one is dropped, not kept",
          fast[0], None)
    check("...and the turn that cost it is still reported", fast[2], 120.0)

    still = inspector._where(before, {**before}, before_at=100.0,
                             after_at=100.4, taken_at=100.2)
    check("a rover standing still claims no extra error", still[3], 0.0)


def test_what_a_bearing_is_worth_reaches_the_row_and_the_geometry() -> None:
    """The residual is no use on the observation alone: `locate` has to spend it,
    the way it already spends `origin_sigma_m`."""
    with tempfile.TemporaryDirectory() as directory:
        store, _eyes, inspector = a_seeing_inspector(
            directory, [[a_sighting()]],
            capture=a_capture(pan=0.0, tilt=0.0,
                              taken_at=lambda: time.time(), delay_s=0.20),
            pose=a_turning_pose([0.0, 12.0]))
        answer = inspector.inspect()
        check("the look was stored", answer["ok"], True)
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("it kept its bearing despite the turn",
              row["bearing_deg"] is not None, True)
        check("...and the row says what that bearing is worth",
              row["bearing_sigma_deg"] is not None and
              row["bearing_sigma_deg"] > 0, True)
        check("...which the status line reports rather than hiding",
              "leaving the bearing good to" in (answer["detail"] or ""), True)
        check("...as the figure the geometry will spend, not the raw residual",
              f"{locate.BEARING_SIGMA_DEG:.1f} deg" in (answer["detail"] or "")
              or float(row["bearing_sigma_deg"]) > locate.BEARING_SIGMA_DEG,
              True)
        store.close()

    # And the geometry spends it. Same two rays, one of them measured while the
    # rover was turning: the crossing has to come out wider.
    left = {"x_m": 0.0, "y_m": 0.0, "bearing_deg": 45.0}
    right = {"x_m": 6.0, "y_m": 0.0, "bearing_deg": 135.0}
    steady = locate.fix(left, right)
    turning = locate.fix({**left, "bearing_sigma_deg": 6.0}, right)
    check("a crossing made with a turning look is the wider answer",
          turning["uncertainty_m"] > steady["uncertainty_m"] * 2, True)
    check("...and a bearing may only ever widen, never narrow",
          locate.sigma_of({**left, "bearing_sigma_deg": 0.1}),
          locate.BEARING_SIGMA_DEG)
    check("...with silence meaning the constant, as every older row has",
          locate.sigma_of(left), locate.BEARING_SIGMA_DEG)
    check("a wider bearing is allowed to miss by more, at the same range",
          locate.match_tolerance({"x_m": 3.0, "y_m": 3.0, "uncertainty_m": 0.1,
                                  "error_major_m": 0.1, "error_minor_m": 0.1,
                                  "error_major_deg": 0.0, "extent_m": 0.1},
                                 {**left, "bearing_sigma_deg": 6.0})
          > locate.match_tolerance({"x_m": 3.0, "y_m": 3.0,
                                    "uncertainty_m": 0.1,
                                    "error_major_m": 0.1, "error_minor_m": 0.1,
                                    "error_major_deg": 0.0, "extent_m": 0.1},
                                   left), True)


TESTS = (
    test_the_heading_is_taken_at_the_shutter_rather_than_averaged,
    test_a_turning_look_keeps_a_wide_bearing_instead_of_none,
    test_what_a_bearing_is_worth_reaches_the_row_and_the_geometry,
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
    test_a_look_taken_on_the_move_keeps_the_picture_and_drops_the_bearing,
    test_looking_and_settling_are_separately_paced,
)
