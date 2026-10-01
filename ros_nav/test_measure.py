"""The read-only measurement a look's heading is checked with.

`nav_map.measure_pose` must never hold up a move or the stop that ends one: it
takes no move mutex, refuses a moving rover, and gives up on a busy keeper rather
than waiting behind a graph write. An in-move refit that did none of these held
the wheels for fifteen seconds on 2026-10-01 and was reverted.
"""
import threading
import time

from test_harness import check, section

import test_maprestore            # stubs slam_toolbox before nav_map loads
import nav_map


class Fit(object):
    def __init__(self, ok):
        self.ok = ok


class Node(object):
    def __init__(self, driving=False):
        self._lock = threading.Lock()
        self.map_lock = threading.RLock()
        self.move_mutex = threading.Lock()
        self.driving = driving
        self.asked = []
        self.fits = []

    def map_measure(self, window_m=None, window_deg=None, min_score=None,
                    around=None):
        self.asked.append((window_m, window_deg, around))
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
    check("a moving rover is not measured",
          (answer["trusted"], node.asked), (False, []))

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
