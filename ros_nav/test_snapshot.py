"""The map's half of a snapshot: copied out, and a kept one put back.

`nav_snapshot` needs slam_toolbox to import, so what runs here is the real code
over a stand-in node and a real `SavedMap` in a temporary directory. The two
things worth proving are where the rover is put -- kept where it is on the same
map, put where the snapshot stood on another -- and that the saved map on disk
becomes the snapshot's only once the mapper has really loaded it.
"""
import json
import os
import tempfile
import threading
import types

from test_harness import check, section
import test_maprestore  # noqa: F401 -- stubs slam_toolbox's service types

import mapstore                                             # noqa: E402
import nav_snapshot                                         # noqa: E402

#: Where the snapshot says the rover stood, and where the rover stands now.
STOOD = (-3.0, 2.0, 90.0)
LIVE = (1.0, 1.0, 10.0)


class Result:
    RESULT_SUCCESS = 0
    result = 0


class Node(nav_snapshot.NavSnapshot):
    """As much of `NavBridge` as `NavSnapshot` reaches for, under the real mixin."""

    def __init__(self, directory, map_id="map-two", settled=True, lands=True):
        self.map_lock = threading.RLock()
        self._lock = threading.Lock()
        self.move_mutex = threading.Lock()
        self.map_msg = object()
        self.map_id = map_id
        self.map_note = "the map is kept"
        self.map_restored = True
        self.map_settled = settled
        self.driving = self.exploring = False
        self.saved = mapstore.SavedMap(directory)
        self.loads = []
        self.lands = lands
        self.said = []
        self.serialize_client = types.SimpleNamespace(
            wait_for_service=lambda timeout_sec=None: True,
            call_async=self._serialize)

    def _serialize(self, request):
        for path in nav_snapshot._graph_paths(request.filename):
            with open(path, "w") as handle:
                handle.write("graph of " + str(self.map_id))
        return types.SimpleNamespace(result=lambda: Result())

    def wait(self, _future, _limit):
        return True

    def pose_deg(self):
        return LIVE

    def travelled_deg(self):
        return (0.0, 0.0, 0.0)

    def map_trustworthy(self):
        return self.map_settled or not self.map_restored

    def load_graph(self, pose, drop_trail=False, stem=None):
        self.loads.append((pose, stem))
        if not self.lands:
            return False, "nothing arrived", None
        return True, "the map is loaded", pose

    def get_logger(self):
        return types.SimpleNamespace(info=self.said.append, warn=self.said.append)


def _take(node, root, name="20261011-120000-abcd"):
    directory = os.path.join(root, name)
    return directory, node.snapshot_map(directory)


def _with_root(test):
    def run():
        with tempfile.TemporaryDirectory() as root:
            was = os.environ.get(nav_snapshot.ENV_ROOT)
            os.environ[nav_snapshot.ENV_ROOT] = os.path.join(root, "snapshots")
            os.makedirs(os.environ[nav_snapshot.ENV_ROOT])
            try:
                test(root, os.environ[nav_snapshot.ENV_ROOT])
            finally:
                if was is None:
                    os.environ.pop(nav_snapshot.ENV_ROOT, None)
                else:
                    os.environ[nav_snapshot.ENV_ROOT] = was
    run.__name__ = test.__name__
    return run


@_with_root
def test_a_snapshot_keeps_the_graph_and_where_the_rover_was(root, snapshots):
    section("a snapshot's copy of the map")
    node = Node(os.path.join(root, "map"))
    directory, kept = _take(node, snapshots)
    check("the graph is kept", kept["saved"])
    check("...in the snapshot's own directory",
          all(os.path.exists(path) for path in
              nav_snapshot._graph_paths(os.path.join(directory, "map"))))
    with open(os.path.join(directory, "map.json")) as handle:
        note = json.load(handle)
    check("...beside a note naming the map", note["map_id"], "map-two")
    check("...and where the rover was", (note["pose"]["x_m"],
                                         note["pose"]["heading_deg"]), (1.0, 10.0))
    check("...and whether that was believed", note["settled"], True)
    check("the saved map is not touched", node.saved.held(), None)

    _, elsewhere = _take(node, root, "loose")
    check("a directory outside the snapshots is refused", elsewhere["saved"],
          False)
    node.map_msg = None
    check("...and so is a mapper with no map yet",
          _take(node, snapshots, "20261011-120001-abcd")[1]["saved"], False)


@_with_root
def test_on_the_same_map_the_rover_stays_where_it_is(root, snapshots):
    section("a snapshot of the map the rover is on")
    node = Node(os.path.join(root, "map"))
    directory, _ = _take(node, snapshots)
    answer = node.restore_map(directory)
    check("it is restored", answer["restored"])
    check("...keeping the rover where it is", answer["here"])
    check("...which is the pose the mapper was handed", node.loads[0][0], LIVE)
    check("...with the snapshot's graph",
          node.loads[0][1], os.path.join(os.path.realpath(directory), "map"))
    check("...and the place believed", node.map_settled, True)


@_with_root
def test_on_another_map_the_rover_is_put_where_the_snapshot_stood(root, snapshots):
    section("a snapshot of another map")
    node = Node(os.path.join(root, "map"), map_id="map-one")
    directory, _ = _take(node, snapshots)
    with open(os.path.join(directory, "map.json")) as handle:
        note = json.load(handle)
    note["pose"] = {"x_m": STOOD[0], "y_m": STOOD[1], "heading_deg": STOOD[2]}
    with open(os.path.join(directory, "map.json"), "w") as handle:
        json.dump(note, handle)
    node.saved.make()
    for path in node.saved.graph_paths():
        with open(path, "w") as handle:
            handle.write("graph of map-two")
    node.saved._write_note({"map_id": "map-two", "pose": None})
    node.map_id = "map-two"

    answer = node.restore_map(directory)
    check("it is restored", answer["restored"])
    check("...with the rover put where the snapshot stood", node.loads[0][0], STOOD)
    check("...on the snapshot's map", node.map_id, "map-one")
    check("...which is now the saved map", node.saved.held()["map_id"], "map-one")
    check("...graph and all", open(node.saved.graph_paths()[0]).read(),
          "graph of map-one")
    check("...with the pose a boot would come back to", node.saved.start_pose(),
          STOOD)
    check("...and the lidar asked at once whether the rover is really there",
          (node._restore_unchecked, node._map_drift_at), (True, None))

    note["settled"] = False
    with open(os.path.join(directory, "map.json"), "w") as handle:
        json.dump(note, handle)
    node.map_id = "map-two"
    node.restore_map(directory)
    check("a pose the snapshot did not believe is not believed now",
          node.map_settled, False)

    node.map_id = "map-two"
    refused = node.restore_map(directory, here=True)
    check("keeping the rover where it is on another map is refused",
          (refused["restored"], refused["touched"]), (False, False))


@_with_root
def test_a_map_that_does_not_load_leaves_the_saved_one(root, snapshots):
    section("a snapshot's map that will not load")
    node = Node(os.path.join(root, "map"), map_id="map-one")
    directory, _ = _take(node, snapshots)
    node.map_id, node.lands = "map-two", False
    node.saved.make()
    for path in node.saved.graph_paths():
        with open(path, "w") as handle:
            handle.write("graph of map-two")
    node.saved._write_note({"map_id": "map-two", "pose": None})
    answer = node.restore_map(directory)
    check("it is not restored", answer["restored"], False)
    check("...after the mapper was asked", answer["touched"], True)
    check("...and the saved map is the one it was",
          node.saved.held()["map_id"], "map-two")
    check("...as is the identity", node.map_id, "map-two")

    node.lands = True
    node.move_mutex.acquire()
    try:
        moving = node.restore_map(directory)
    finally:
        node.move_mutex.release()
    check("a rover that is moving is refused",
          (moving["restored"], moving["touched"]), (False, False))


TESTS = (
    test_a_snapshot_keeps_the_graph_and_where_the_rover_was,
    test_on_the_same_map_the_rover_stays_where_it_is,
    test_on_another_map_the_rover_is_put_where_the_snapshot_stood,
    test_a_map_that_does_not_load_leaves_the_saved_one,
)


if __name__ == "__main__":
    for test in TESTS:
        test()
