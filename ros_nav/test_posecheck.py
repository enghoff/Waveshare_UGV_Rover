"""The bookkeeping behind "has the pose been checked since the rover turned".

`posecheck.PoseWatch` decides when the world state may take a bearing again and
when the keeper should search, and all of that is arithmetic over odometry
samples and a clock. What `nav_map.check_pose` does with its answers is tested
here through a stand-in node, because the module needs slam_toolbox to import
and the decision it makes needs no radio.
"""
import types

from test_harness import check, section

import posecheck


def test_turning_puts_the_pose_in_doubt():
    section("turning on the spot puts the pose in doubt (2026-10-01)")
    w = posecheck.PoseWatch()
    check("a watch that has seen nothing has nothing in doubt", w.checked())
    w.feed((0.0, 0.0, 0.0), 0.0)
    w.feed((0.0, 0.0, 10.0), 0.1)
    check("ten degrees is inside the allowance", w.checked())
    w.feed((0.0, 0.0, 20.0), 0.2)
    check("twenty is not: a turn at 7% puts more than a degree in",
          w.checked(), False)
    check("...and is counted as turned", w.status()["turned_since_check_deg"],
          20.0, tolerance=1e-6)

    w = posecheck.PoseWatch()
    w.feed((0.0, 0.0, 170.0), 0.0)
    w.feed((0.0, 0.0, -170.0), 0.1)
    check("a turn across the half-circle counts its 20 degrees, not 340",
          w.status()["turned_since_check_deg"], 20.0, tolerance=1e-6)

    w = posecheck.PoseWatch()
    w.feed((0.0, 0.0, 0.0), 0.0)
    for i in range(1, 9):
        w.feed((0.0, 0.0, 10.0 * (i % 2)), 0.1 * i)
    check("back and forth sums both ways: the two directions were not equal",
          w.status()["turned_since_check_deg"], 80.0, tolerance=1e-6)

    w = posecheck.PoseWatch()
    w.feed((0.0, 0.0, 0.0), 0.0)
    w.feed((1.0, 0.0, 0.0), 1.0)
    check("driving straight is not turning", w.checked())


def test_when_the_keeper_checks_on_its_own():
    section("the keeper checks only a still rover, and spaces out refusals")
    w = posecheck.PoseWatch()
    w.feed((0.0, 0.0, 0.0), 0.0)
    w.feed((0.0, 0.0, 90.0), 1.0)
    check("not while it has only just stopped turning", w.due(1.5), False)
    check("once it has stood still", w.due(1.0 + posecheck.STILL_S + 0.1))
    w.outcome(3.0, {"trusted": False}, {"trusted": False,
                                        "why": "the room fits two ways"})
    check("a refused check leaves the pose in doubt", w.checked(), False)
    check("...and is not retried at once", w.due(4.0), False)
    check("...but is after a while", w.due(3.0 + posecheck.RETRY_S + 0.1))
    w.feed((0.0, 0.0, 95.0), 5.0)
    check("new motion makes a retry worth having straight away",
          w.due(5.0 + posecheck.STILL_S + 0.1))


def test_a_check_confirms_only_what_it_measured():
    section("a check confirms the pose only when a second look agrees")
    w = posecheck.PoseWatch()
    w.feed((0.0, 0.0, 0.0), 0.0)
    w.feed((0.0, 0.0, 120.0), 1.0)
    report = w.outcome(3.0, {"trusted": True, "fitted": True, "turned_deg": -8.4,
                             "moved_m": 0.03, "why": "moved onto the map"},
                       {"trusted": True, "settled": True, "turned_deg": 0.2,
                        "moved_m": 0.01, "why": "where it thinks it is"})
    check("a fit that moved the rover and a look that agrees confirm it",
          (report["confirmed"], w.checked()), (True, True))
    check("...and the turning that made it due is reported and then cleared",
          (report["turned_deg"], w.status()["turned_since_check_deg"]),
          (120.0, 0.0))
    check("...with what it found and what was left",
          (report["found_deg"], report["left_deg"]), (-8.4, 0.2))

    w.feed((0.0, 0.0, 150.0), 4.0)
    report = w.outcome(6.0, {"trusted": True, "fitted": False, "turned_deg": 2.0},
                       {"trusted": True, "settled": False, "turned_deg": 2.0,
                        "why": "two degrees from here"})
    check("a correction the mapper did not keep confirms nothing",
          (report["confirmed"], w.checked()), (False, False))


def test_the_drift_check_raises_doubt():
    section("the drift check's verdict puts the pose in doubt (R-WS-16)")
    w = posecheck.PoseWatch()
    w.drifted({"trusted": True, "agrees": True, "here_score": 0.93})
    check("agreement is not doubt", w.checked())
    w.drifted({"trusted": False, "agrees": False, "here_score": 0.94,
               "why": "the room fits two ways"})
    check("'cannot say' with the scan on the walls is not doubt either",
          w.checked())
    w.drifted({"trusted": False, "agrees": False, "here_score": 0.259,
               "why": "nowhere near here"})
    check("the carried rover of 2026-10-01 is: 26% on the walls",
          (w.checked(), w.status()["pose_doubt"]), (False, "nowhere near here"))
    w = posecheck.PoseWatch()
    w.drifted({"trusted": True, "agrees": False, "here_score": 0.6,
               "why": "47 degrees from where the scan fits"})
    check("so is a confident fit somewhere else", w.checked(), False)
    w.outcome(1.0, {"trusted": True, "fitted": True},
              {"trusted": True, "settled": True})
    check("a check that confirms the pose clears the doubt", w.checked())


def test_check_pose_on_the_keeper():
    """`nav_map.check_pose` itself, with the search stood in for."""
    section("check_pose: narrow, never on an unconfirmed restore, and verified")
    import test_maprestore            # stubs slam_toolbox before nav_map loads
    import nav_map

    class Node(object):
        def __init__(self, trustworthy=True, fits=None, measures=None):
            self.pose_watch = posecheck.PoseWatch()
            self.map_lock = test_maprestore.threading.RLock()
            self._trustworthy = trustworthy
            self.fits = fits or []
            self.measures = measures or []
            self.asked = []
            self.logged = []

        def map_trustworthy(self):
            return self._trustworthy

        def map_fit_now(self, window_m=None, window_deg=None, min_score=None,
                        around=None):
            self.asked.append(("fit", window_m, window_deg, around))
            return self.fits.pop(0)

        def map_measure(self, window_m=None, window_deg=None, min_score=None,
                        around=None):
            self.asked.append(("measure", window_m, window_deg, around))
            return self.measures.pop(0), None, None

        def get_logger(self):
            return types.SimpleNamespace(info=self.logged.append,
                                         warn=self.logged.append)

    node = Node()
    check("nothing in doubt, nothing searched",
          (nav_map.NavMap.check_pose(node), node.asked), (None, []))

    node = Node(fits=[{"trusted": True, "fitted": True, "turned_deg": -21.7}],
                measures=[{"trusted": True, "settled": True, "turned_deg": 0.1}])
    node.pose_watch.feed((0.0, 0.0, 0.0), 0.0)
    node.pose_watch.feed((0.0, 0.0, 180.0), 1.0)
    report = nav_map.NavMap.check_pose(node)
    check("after a half turn it searches the narrow window, then looks again",
          node.asked, [("fit", posecheck.WINDOW_M, posecheck.WINDOW_DEG, None),
                       ("measure", posecheck.WINDOW_M, posecheck.WINDOW_DEG,
                        None)])
    check("...and the pose is checked again", (report["confirmed"],
                                               node.pose_watch.checked()),
          (True, True))
    check("...and the check is written in the log", len(node.logged), 1)

    node = Node(trustworthy=False)
    node.pose_watch.feed((0.0, 0.0, 0.0), 0.0)
    node.pose_watch.feed((0.0, 0.0, 90.0), 1.0)
    report = nav_map.NavMap.check_pose(node)
    check("an unconfirmed restore is never searched unasked",
          (node.asked, report["confirmed"], node.pose_watch.checked()),
          ([], False, False))


TESTS = (test_turning_puts_the_pose_in_doubt, test_when_the_keeper_checks_on_its_own,
         test_a_check_confirms_only_what_it_measured, test_the_drift_check_raises_doubt,
         test_check_pose_on_the_keeper)
