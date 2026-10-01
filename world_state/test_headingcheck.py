"""A look's heading, checked against the map and never by moving the rover.

`headingcheck.HeadingCheck` decides, for each look, whether its pose can be
believed as read, should be corrected by what one scan found, or gives no
direction at all. The measurements behind every number are the tape runs of
2026-10-01: turning on the spot leaves the heading 7% of every turn out, looks
taken while turning missed by a median 4.7 degrees against 2.6 still, and one scan
matched against the map in a narrow window finds the truth to within 2.
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


def refused():
    return {"trusted": False, "why": "the scan does not fit the map well enough"}


def test_a_still_look_is_checked_and_kept_corrected_or_withheld() -> None:
    answers = [agrees(), disagrees(-6.5), refused()]
    check_ = headingcheck.HeadingCheck(lambda: answers.pop(0))

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
    check("a still look the scan cannot place gets no direction", pose, None)
    check("...and the reason is the scan's", "fit the map" in note, True)


def test_a_moving_look_needs_a_recent_good_check() -> None:
    answers = [agrees(), disagrees(), agrees()]
    check_ = headingcheck.HeadingCheck(lambda: answers.pop(0))
    pose, _ = check_.judge(dict(HERE), 0.5, 0.0)
    check("before any check, a moving look gets no direction", pose, None)

    check_.judge(dict(HERE), 0.0, 0.0)            # a still look the scan agrees with
    check_.saw({"heading_deg": 90.0})
    check_.saw({"heading_deg": 95.0})
    pose, _ = check_.judge(dict(HERE), 0.5, 2.0)
    check("driving straight after a good check keeps its bearing",
          pose is not None, True)

    check_.saw({"heading_deg": 115.0})
    pose, note = check_.judge(dict(HERE), 0.2, 8.0)
    check("after 25 degrees of turning, a moving look gets none", pose, None)
    check("...and says the rover has turned since it was checked",
          "turned 25 deg" in note, True)

    check_.judge(dict(HERE), 0.0, 0.0)            # still, but the scan disagrees
    pose, _ = check_.judge(dict(HERE), 0.5, 0.0)
    check("a check that had to correct leaves moving looks waiting", pose, None)

    check_.judge(dict(HERE), 0.0, 0.0)            # still, and the scan agrees
    pose, _ = check_.judge(dict(HERE), 0.5, 0.0)
    check("...until a check finds the heading right again", pose is not None, True)


def test_turning_is_counted_across_the_half_circle() -> None:
    check_ = headingcheck.HeadingCheck(lambda: None)
    check_.saw({"heading_deg": 170.0})
    check_.saw({"heading_deg": -170.0})
    check("20 degrees across the wrap, not 340", check_.turned_deg, 20.0)
    check_.saw(None)
    check_.saw({"x_m": 1.0})
    check("a missing pose or heading counts as no turn", check_.turned_deg, 20.0)


def test_a_check_that_fails_outright_withholds_and_never_raises() -> None:
    def broken():
        raise OSError("the bridge is down")
    pose, note = headingcheck.HeadingCheck(broken).judge(dict(HERE), 0.0, 0.0)
    check("a measurement that raised gives no direction", pose, None)
    check("...and names why", "bridge is down" in note, True)
    pose, _ = headingcheck.HeadingCheck(lambda: None).judge(dict(HERE), 0.0, 0.0)
    check("one that answered nothing gives none either", pose, None)
    pose, _ = headingcheck.HeadingCheck(lambda: {"trusted": True, "settled": False}
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
        store, inspector = inspector_with(directory, lambda: agrees())
        inspector.inspect()
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("a look the scan agrees with keeps the bearing it always had",
              row["bearing_deg"], 70.7)
        check("...and the check is written beside the pose",
              '"checked"' in row["observer_pose_json"], True)
        store.close()

    with tempfile.TemporaryDirectory() as directory:
        store, inspector = inspector_with(directory, lambda: disagrees(-6.5))
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
        store, inspector = inspector_with(directory, refused)
        answer = inspector.inspect()
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("a refused check keeps the picture", row["frame_id"] is not None, True)
        check("...and records no direction", row["bearing_deg"], None)
        detail = store.db.execute(
            "SELECT detail FROM inferences WHERE id = ?",
            (answer["inference_id"],)).fetchone()[0]
        check("...and says why in the look's line",
              "could not be checked against the map" in detail, True)
        store.close()

    with tempfile.TemporaryDirectory() as directory:
        store, inspector = inspector_with(
            directory, lambda: agrees(),
            pose=a_turning_pose([90.0, 110.0]))
        inspector.inspect()
        row = dict(store.db.execute("SELECT * FROM observations").fetchone())
        check("a look taken mid-turn, before any check, gets no direction",
              row["bearing_deg"], None)
        store.close()

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
         test_a_moving_look_needs_a_recent_good_check,
         test_turning_is_counted_across_the_half_circle,
         test_a_check_that_fails_outright_withholds_and_never_raises,
         test_the_inspector_takes_bearings_from_the_checked_pose)
