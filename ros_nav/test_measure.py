"""The read-only measurement a look's heading is checked with.

`nav_map.measure_pose` must never hold up a move or the stop that ends one: it
takes no move mutex and gives up on a busy keeper rather than waiting behind a
graph write. An in-move refit that did neither held the wheels for fifteen seconds
on 2026-10-01 and was reverted. On the move it measures from where the rover was
half way through the scan's sweep, and refuses a sweep turned too fast to match.
"""
import math
import threading
import time

from test_harness import check, section

import test_maprestore            # stubs slam_toolbox before nav_map loads
import nav_map


class Fit(object):
    def __init__(self, ok):
        self.ok = ok


class Scan(object):
    """A sweep stamped at 100 s that took a tenth of a second."""
    def __init__(self):
        self.header = type("Header", (), {"stamp": 100.0})()
        self.scan_time = 0.1
        self.time_increment = 0.0
        self.ranges = [1.0] * 360


class Node(object):
    scan_moment = nav_map.NavMap.scan_moment

    def __init__(self, driving=False, turning_dps=10.0, reaches=True):
        self._lock = threading.Lock()
        self.map_lock = threading.RLock()
        self.move_mutex = threading.Lock()
        self.driving = driving
        self.scan_msg = Scan()
        self.turning_dps = turning_dps
        self.reaches = reaches
        self.asked = []
        self.fits = []

    def pose_at(self, stamp, after_s=0.0):
        if not self.reaches:
            return None
        # Driving forward at half a metre a second from (3, 4) facing east, turning.
        return (3.0 + 0.5 * after_s, 4.0,
                math.radians(self.turning_dps * after_s))

    def map_measure(self, window_m=None, window_deg=None, min_score=None,
                    around=None, where=None, scan=None):
        self.asked.append((window_m, window_deg, around))
        self.measured_from = where
        fit = Fit(self.fits.pop(0) if self.fits else True)
        return ({"trusted": fit.ok, "settled": False, "turned_deg": -6.5},
                fit, None)

    def pose_deg(self):
        return (1.0, 2.0, 90.0)


def test_measure_pose_is_read_only_and_never_waits():
    section("measure_pose: narrow, read-only, never behind a move or a save")
    node = Node()
    answer = nav_map.NavMap.measure_pose(node)
    check("it searches the narrow window and passes the answer back",
          (node.asked, answer["turned_deg"]),
          ([(nav_map.MEASURE_WINDOW_M, nav_map.MEASURE_WINDOW_DEG, None)], -6.5))

    node = Node()
    nav_map.NavMap.measure_pose(node, (0.1, 0.0, -40.0))
    check("handed the last correction, it searches around it",
          node.asked[0][2], (1.1, 2.0, 50.0))
    node = Node()
    node.fits = [False, True]
    answer = nav_map.NavMap.measure_pose(node, (0.1, 0.0, -40.0))
    check("...and around the rover's own pose when nothing fits there",
          (len(node.asked), node.asked[1][2], answer["trusted"]),
          (2, None, True))

    node = Node()
    node.move_mutex.acquire()
    answer = nav_map.NavMap.measure_pose(node)
    check("a move holding the mutex does not stop it measuring",
          answer.get("trusted"), True)

    node = Node(driving=True)
    answer = nav_map.NavMap.measure_pose(node)
    check("a moving rover is measured",
          (answer["trusted"], answer["moving"], len(node.asked)), (True, True, 1))
    check("...from where it was half way through the scan's sweep",
          tuple(round(v, 3) for v in node.measured_from), (3.025, 4.0, 0.5))
    check("...and says how fast it was turning", answer["turn_dps"], 10.0)
    node = Node(driving=True)
    nav_map.NavMap.measure_pose(node, (0.1, 0.0, -40.0))
    check("handed the last correction on the move, it searches around that moment",
          tuple(round(v, 3) for v in node.asked[0][2]), (3.125, 4.0, -39.5))

    node = Node(driving=True, turning_dps=nav_map.MEASURE_MAX_TURN_DPS * 2)
    answer = nav_map.NavMap.measure_pose(node)
    check("a sweep turned faster than it can be matched is refused, unsearched",
          (answer["trusted"], node.asked, "score" in answer), (False, [], False))
    node = Node(driving=True, reaches=False)
    answer = nav_map.NavMap.measure_pose(node)
    check("so is one whose moment the transform tree no longer reaches",
          (answer["trusted"], "transform tree" in answer["why"]), (False, True))

    node = Node()
    held = threading.Event()
    release = threading.Event()

    def keeper():
        with node.map_lock:
            held.set()
            release.wait(5.0)
    threading.Thread(target=keeper, daemon=True).start()
    held.wait(1.0)
    began = time.monotonic()
    answer = nav_map.NavMap.measure_pose(node)
    waited = time.monotonic() - began
    release.set()
    check("a keeper writing the graph gets 'not now', not a wait",
          (answer["trusted"], waited < nav_map.MEASURE_WAIT_S + 0.2), (False, True))


TESTS = (test_measure_pose_is_read_only_and_never_waits,)
