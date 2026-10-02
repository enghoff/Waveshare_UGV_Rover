"""A look's heading, checked against the map and never by moving the rover.

`headingcheck.HeadingCheck` decides, for each look, whether its pose can be
believed as read, should be corrected by what one scan found, or gives no
direction at all. The measurements behind every number are the tape runs of
2026-10-01: turning on the spot leaves the heading 7% of every turn out, looks
taken while turning missed by a median 4.7 degrees against 2.6 still, and one scan
matched against the map in a narrow window finds the truth to within 2. The
driven run of 2026-10-02 is why only a check that searched and fitted nowhere
withholds: withholding every look no check vouched for left 17 of 906 regions
with a direction.
"""
from __future__ import annotations

import tempfile

from test_harness import check
from test_fakes import a_capture, a_pose, a_sighting, a_store, a_turning_pose
from world_state import headingcheck
from world_state.inspector import Inspector

HERE = {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0}


def agrees(**extra):
    return {"trusted": True, "settled": True, "turned_deg": 0.4, "moved_m": 0.02,
            "score": 0.96, "x_m": 1.0, "y_m": 2.0, "heading_deg": 90.4,
            "was": dict(HERE), **extra}


def disagrees(by_deg=-6.5):
    return {"trusted": True, "settled": False, "turned_deg": by_deg,
            "moved_m": 0.04, "score": 0.95, "x_m": 1.03, "y_m": 2.0,
            "heading_deg": 90.0 + by_deg, "was": dict(HERE)}


def misfit():
    """A search that ran and fitted nowhere near the believed pose: a carried
    rover, measured on 2026-10-02 at 61% of the scan on a wall against 90%."""
    return {"trusted": False, "settled": False, "score": 0.61, "rival": 0.55,
            "guess_score": 0.3, "x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0,
            "why": "the scan does not fit the map well enough", "was": dict(HERE)}


def refused():
    """No search at all, as `nav_map.measure_pose` answers mid-move."""
    return {"trusted": False, "why": "the rover is moving, and a scan taken on "
                                     "the move does not describe one place"}


def test_a_still_look_is_checked_and_kept_corrected_or_withheld() -> None:
    answers = [agrees(), disagrees(-6.5), misfit()]
    asked = []
    check_ = headingcheck.HeadingCheck(
        lambda offset: asked.append(offset) or answers.pop(0))

    pose, note = check_.judge(dict(HERE), 0.0, 0.0)
    check("a still look the scan agrees with keeps its pose",
          (pose["heading_deg"], note), (90.0, None))
    check("...and carries what the check found",
          pose["checked"]["off_deg"], 0.4)

    pose, note = check_.judge(dict(HERE), 0.0, 0.0)
    check("a still look the scan confidently disagrees with takes the scan's "
          "heading", pose["heading_deg"], 83.5)
    check("...and position", pose["x_m"], 1.03)
    check("...and says it was corrected",
          (pose["checked"]["corrected"], "corrected by -6.5" in note), (True, True))

    pose, note = check_.judge(dict(HERE), 0.0, 0.0)
    check("the next search starts from the last correction found",
          tuple(round(v, 3) for v in asked[-1]), (0.03, 0.0, -6.5))
    check("a still look the scan cannot place gets no direction", pose, None)
    check("...and the reason is the scan's", "fit the map" in note, True)


def test_a_moving_look_takes_a_fresh_correction_or_the_navigators_heading() -> None:
    answers = [agrees(), disagrees(), agrees()]
    check_ = headingcheck.HeadingCheck(lambda offset: answers.pop(0))
    pose, note = check_.judge(dict(HERE), 0.5, 0.0)
    check("before any check, a moving look takes the heading the navigator "
          "believes", (pose, note), (HERE, None))

    check_.judge(dict(HERE), 0.0, 0.0)            # a still look the scan agrees with
    check_.saw({"heading_deg": 90.0})
    check_.saw({"heading_deg": 95.0})
    pose, _ = check_.judge(dict(HERE), 0.5, 2.0)
    check("driving straight after a good check keeps its bearing, vouched for",
          pose["checked"]["from_earlier_check"], True)

    check_.saw({"heading_deg": 115.0})
    pose, note = check_.judge(dict(HERE), 0.2, 8.0)
    check("after 25 degrees of turning, a moving look goes back to the "
          "navigator's heading", (pose, note), (HERE, None))

    check_.judge(dict(HERE), 0.0, 0.0)            # still, but the scan disagrees
    pose, _ = check_.judge(dict(HERE), 0.5, 0.0)
    check("a moving look just after a correcting check takes the same correction",
          (pose["heading_deg"], pose["checked"]["from_earlier_check"]), (83.5, True))
    check_.saw({"x_m": 1.0, "y_m": 2.0, "heading_deg": 115.0})
    check_.saw({"x_m": 1.8, "y_m": 2.0, "heading_deg": 115.0})
    pose, _ = check_.judge(dict(HERE), 0.5, 0.0)
    check("...but not once the rover has driven on: the map may have caught up",
          pose, HERE)

    check_.judge(dict(HERE), 0.0, 0.0)            # still, and the scan agrees
    check_.saw({"x_m": 3.0, "y_m": 2.0, "heading_deg": 115.0})
    pose, _ = check_.judge(dict(HERE), 0.5, 0.0)
    check("a check that found the heading right lets straight driving keep its "
          "bearings however far", "checked" in pose, True)


def test_a_driven_run_keeps_its_directions() -> None:
    """The run of 2026-10-02, in miniature: the rover is partway through a move
    for all of it, so every check is refused and it turns 90 degrees between
    stops. Withholding everything no check vouched for gave 171 of its 216
    looks no direction."""
    check_ = headingcheck.HeadingCheck(lambda offset: refused())
    given = 0
    for step in range(20):
        heading = 90.0 + 4.5 * step
        check_.saw({"x_m": 1.0 + 0.1 * step, "y_m": 2.0, "heading_deg": heading})
        where = {"x_m": 1.0 + 0.1 * step, "y_m": 2.0, "heading_deg": heading}
        still = step % 5 == 0           # a pause inside the move, still refused
        pose, _ = check_.judge(where, 0.0 if still else 0.2,
                               0.0 if still else 4.5)
        given += pose is not None
    check("every look of a run no check could speak for keeps its direction",
          given, 20)
    check("...and nothing about a refusal counts against the rover",
          check_.misplaced, None)


def test_a_rover_the_scan_cannot_place_stays_withheld() -> None:
    """R-WS-16: once a search has fitted nowhere, nothing gets a direction,
    moving or still, refused or not, until a search fits again."""
    answers = [misfit(), refused(), agrees()]
    check_ = headingcheck.HeadingCheck(lambda offset: answers.pop(0))
    pose, _ = check_.judge(dict(HERE), 0.0, 0.0)
    check("a still look the scan fits nowhere gets no direction", pose, None)
    pose, note = check_.judge(dict(HERE), 0.5, 3.0)
    check("...nor does the moving look after it",
          (pose, "could not place the rover" in note), (None, True))
    pose, _ = check_.judge(dict(HERE), 0.0, 0.0)
    check("...nor a still look whose check was refused", pose, None)
    pose, _ = check_.judge(dict(HERE), 0.0, 0.0)
    check("a search that fits gives directions back", pose["heading_deg"], 90.0)
    pose, _ = check_.judge(dict(HERE), 0.5, 3.0)
    check("...to moving looks too", pose is not None, True)


def test_turning_is_counted_across_the_half_circle() -> None:
    check_ = headingcheck.HeadingCheck(lambda offset: None)
    check_.saw({"heading_deg": 170.0})
    check_.saw({"heading_deg": -170.0})
    check("20 degrees across the wrap, not 340", check_.turned_deg, 20.0)
    check_.saw(None)
    check_.saw({"x_m": 1.0})
    check("a missing pose or heading counts as no turn", check_.turned_deg, 20.0)


def test_a_check_that_cannot_run_never_raises_and_withholds_nothing() -> None:
    def broken(offset):
        raise OSError("the bridge is down")
    check_ = headingcheck.HeadingCheck(broken)
    pose, note = check_.judge(dict(HERE), 0.0, 0.0)
    check("a measurement that raised leaves the navigator's heading",
          (pose, note), (HERE, None))
    check("...and the reason is kept for whoever asks",
          "bridge is down" in check_.last["why"], True)
    pose, _ = headingcheck.HeadingCheck(lambda o: None).judge(dict(HERE), 0.0, 0.0)
    check("one that answered nothing leaves it too", pose, HERE)
    pose, _ = headingcheck.HeadingCheck(lambda o: {"trusted": True,
                                                   "settled": False}
                                        ).judge(dict(HERE), 0.0, 0.0)
    check("a disagreement with no pose to correct from gives none", pose, None)


def test_the_inspector_takes_bearings_from_the_checked_pose() -> None:
    """End to end through one look: the stored bearing moves with the correction,
    and a refused check keeps the picture and drops the direction."""
    def inspector_with(directory, measure, pose=None):
        from world_state.perception_client import FakeEyes

        store = a_store(directory)
        eyes = FakeEyes([[a_sighting(bbox=[0.4, 0.3, 0.6, 0.9])]])
        return store, Inspector(store, eyes, a_capture(pan=20.0),
                                pose or a_pose(heading=90.0), fov_deg=100.0,
                                measure=measure)

    with tempfile.TemporaryDirectory() as directory:
        store, inspector = inspector_with(directory, lambda o: agrees())
        inspector.inspect()
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("a look the scan agrees with keeps the bearing it always had",
              row["bearing_deg"], 70.7)
        check("...and the check is written beside the pose",
              '"checked"' in row["observer_pose_json"], True)
        store.close()

    with tempfile.TemporaryDirectory() as directory:
        store, inspector = inspector_with(directory, lambda o: disagrees(-6.5))
        answer = inspector.inspect()
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("a corrected heading moves the stored bearing by the same amount",
              row["bearing_deg"], 64.2)
        detail = store.db.execute(
            "SELECT detail FROM inferences WHERE id = ?",
            (answer["inference_id"],)).fetchone()[0]
        check("...and the look's line says it was corrected",
              "heading corrected by -6.5 deg" in detail, True)
        store.close()

    with tempfile.TemporaryDirectory() as directory:
        store, inspector = inspector_with(directory, lambda o: misfit())
        answer = inspector.inspect()
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("a check that fits nowhere keeps the picture",
              row["frame_id"] is not None, True)
        check("...and records no direction", row["bearing_deg"], None)
        detail = store.db.execute(
            "SELECT detail FROM inferences WHERE id = ?",
            (answer["inference_id"],)).fetchone()[0]
        check("...and says why in the look's line",
              "could not be checked against the map" in detail, True)
        store.close()

    with tempfile.TemporaryDirectory() as directory:
        store, inspector = inspector_with(directory, lambda o: refused())
        inspector.inspect()
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("a check refused mid-move leaves the bearing the navigator's "
              "heading gives", row["bearing_deg"], 70.7)
        store.close()

    def turning_bearing(measure):
        with tempfile.TemporaryDirectory() as directory:
            from world_state.perception_client import FakeEyes

            store = a_store(directory)
            eyes = FakeEyes([[a_sighting(bbox=[0.4, 0.3, 0.6, 0.9])]])
            Inspector(store, eyes, a_capture(pan=20.0),
                      a_turning_pose([90.0, 110.0]), fov_deg=100.0,
                      measure=measure).inspect()
            row = dict(store.db.execute("SELECT * FROM observations").fetchone())
            store.close()
            return row["bearing_deg"]
    check("a look taken mid-turn, before any check, is judged by its own "
          "shutter as it was before the check existed",
          turning_bearing(lambda o: agrees()), turning_bearing(None))

    with tempfile.TemporaryDirectory() as directory:
        from world_state.perception_client import FakeEyes

        store = a_store(directory)
        eyes = FakeEyes([[a_sighting(bbox=[0.4, 0.3, 0.6, 0.9])]])
        Inspector(store, eyes, a_capture(pan=20.0), a_pose(heading=90.0),
                  fov_deg=100.0).inspect()
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("with nothing to measure, a look's pose is taken as read",
              row["bearing_deg"], 70.7)
        store.close()


TESTS = (test_a_still_look_is_checked_and_kept_corrected_or_withheld,
         test_a_moving_look_takes_a_fresh_correction_or_the_navigators_heading,
         test_a_driven_run_keeps_its_directions,
         test_a_rover_the_scan_cannot_place_stays_withheld,
         test_turning_is_counted_across_the_half_circle,
         test_a_check_that_cannot_run_never_raises_and_withholds_nothing,
         test_the_inspector_takes_bearings_from_the_checked_pose)
