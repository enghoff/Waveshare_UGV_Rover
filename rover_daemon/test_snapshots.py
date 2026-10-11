"""Snapshots of the rover's state: taken, named, put back, undone and deleted.

The daemon's half, end to end, against a stand-in navigator that keeps one graph
in memory the way slam_toolbox does. The two halves below it have their own
checks (world_state/test_snapshot.py, ros_nav/test_snapshot.py); what is proved
here is the order and the refusals: that a restore keeps the state it replaces
first, that a map which will not come back stops the world coming back too, and
that a refusal which changed nothing leaves nothing behind.
"""
from __future__ import annotations

import json
import os
import tempfile

from test_fakes import FakeLink
from test_harness import check


class Navigator:
    """A mapper holding one graph, with an identity, as far as snapshots see it."""

    def __init__(self):
        self.map_id = "map-one"
        self.graph = "graph one"
        self.driving = False
        self.refusal = None
        self.saves = 0

    def status(self, since_seq=None):
        return {"map_id": self.map_id, "driving": self.driving, "exploring": False}

    def snapshot_map(self, directory):
        self.saves += 1
        os.makedirs(directory, exist_ok=True)
        with open(os.path.join(directory, "map.posegraph"), "w") as handle:
            handle.write(self.graph)
        note = {"map_id": self.map_id, "settled": True, "moving": False,
                "pose": {"x_m": 1.0, "y_m": 2.0, "heading_deg": 30.0}}
        with open(os.path.join(directory, "map.json"), "w") as handle:
            json.dump(note, handle)
        return dict(note, saved=True, bytes=len(self.graph))

    def ask(self, request, timeout_s):
        return {"ok": True, "width": 1, "height": 1, "resolution_m": 0.05,
                "origin_x_m": 0.0, "origin_y_m": 0.0, "data": "eJxjAAAAAQAB",
                "pose": None}

    def restore_map(self, directory, here=None):
        if self.refusal is not None:
            return dict(self.refusal, restored=False)
        with open(os.path.join(directory, "map.json")) as handle:
            self.map_id = json.load(handle)["map_id"]
        with open(os.path.join(directory, "map.posegraph")) as handle:
            self.graph = handle.read()
        return {"restored": True, "touched": True, "map_id": self.map_id,
                "here": False, "settled": True, "why": "back"}


def _environment(root):
    names = ("UGV_WORLD_DIR", "UGV_WORLD_FAKE", "UGV_SNAPSHOT_DIR")
    was = {name: os.environ.get(name) for name in names}
    os.environ["UGV_WORLD_DIR"] = os.path.join(root, "world")
    os.environ["UGV_WORLD_FAKE"] = "1"
    os.environ["UGV_SNAPSHOT_DIR"] = os.path.join(root, "snapshots")
    return was


def _put_back(was):
    for name, value in was.items():
        if value is None:
            os.environ.pop(name, None)
        else:
            os.environ[name] = value


def _look(store):
    import world_state

    frame = store.save_frame(b"\xff\xd8 a picture \xff\xd9")
    store.record([world_state.Sighting(bbox=[0.2, 0.2, 0.4, 0.4], dino=b"",
                                       siglip=b"")], capture={"frame_id": frame})
    return frame


def test_a_snapshot_is_taken_named_restored_undone_and_deleted():
    import rover_daemon
    import rover_snapshots

    with tempfile.TemporaryDirectory() as root:
        was = _environment(root)
        changes = rover_snapshots.CHANGES_DIR
        rover_snapshots.CHANGES_DIR = os.path.join(root, "changes")
        try:
            rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/null")
            rover.nav = navigator = Navigator()
            store = rover._world_store()
            first = _look(store)
            _look(store)
            rover._world_follow_map()
            session = store.map_session()
            os.makedirs(rover_snapshots.CHANGES_DIR)
            tallies = os.path.join(rover_snapshots.CHANGES_DIR, "map-one.npz")
            with open(tallies, "wb") as handle:
                handle.write(b"tallies")

            saved = rover.call("snapshot_save", {})
            check("a snapshot is taken", saved["ok"], True)
            kept = saved["snapshot"]
            check("...under the date and time until somebody names it",
                  kept["name"].startswith("snapshot 20"), True)
            check("...holding the world as it is",
                  (kept["entities"], kept["observations"], kept["frames"]), (0, 2, 2))
            check("...on the map the rover is on", (kept["map_id"], kept["same_map"]),
                  ("map-one", True))
            check("...and where it stood", kept["pose"]["heading_deg"], 30.0)
            directory = os.path.join(root, "snapshots", kept["id"])
            check("...with the change watch's tallies beside them",
                  os.path.exists(os.path.join(directory, "changes", "map-one.npz")),
                  True)

            renamed = rover.call("snapshot_rename", {"id": kept["id"],
                                                     "name": "  before\tthe drive "})
            check("it can be named", renamed["snapshot"]["name"], "before the drive")
            check("...but not with nothing",
                  rover.call("snapshot_rename", {"id": kept["id"], "name": " "})["ok"],
                  False)
            check("...and only if it exists",
                  rover.call("snapshot_rename", {"id": "../world", "name": "x"})["ok"],
                  False)

            # The rover moves on: the world is emptied for a drive, the map is
            # cleared and drawn again, and the watch starts that map afresh.
            rover.call("world_state_clear", {})
            later = _look(store)
            navigator.map_id, navigator.graph = "map-two", "graph two"
            rover._world_follow_map()
            os.remove(tallies)

            back = rover.call("snapshot_restore", {"id": kept["id"]})
            check("it is restored", back["ok"], True)
            check("...the map with it", (navigator.map_id, navigator.graph),
                  ("map-one", "graph one"))
            summary = rover.call("world_state_summary", {})["summary"]
            check("...and the world as it was", summary["observations"], 2)
            check("...pictures and all",
                  (os.path.exists(store.frame_path(first)),
                   os.path.exists(store.frame_path(later))), (True, False))
            check("...on the session it was recorded under, since the map is the one "
                  "it was measured in", store.map_session(), session)
            check("...and the store follows that map without moving on",
                  store.follow_map("map-one")["changed"], False)
            check("...and the watch's tallies are back where it looks",
                  os.path.exists(tallies), True)
            undo = back["undo"]
            check("the state it replaced is kept first",
                  undo["name"], "before restoring before the drive")
            check("...holding that state", (undo["observations"], undo["map_id"]),
                  (1, "map-two"))

            again = rover.call("snapshot_restore", {"id": undo["id"]})
            check("restoring that undoes the restore",
                  (again["ok"], navigator.map_id,
                   rover.call("world_state_summary", {})["summary"]["observations"]),
                  (True, "map-two", 1))

            listed = rover.call("snapshot_list", {})
            count = len(listed["snapshots"])
            check("every snapshot is listed, newest first", count, 3)
            navigator.refusal = {"touched": False, "why": "the rover is moving"}
            refused = rover.call("snapshot_restore", {"id": kept["id"]})
            check("a map the navigator will not load is not restored",
                  refused["ok"], False)
            check("...and neither is the world",
                  rover.call("world_state_summary", {})["summary"]["observations"], 1)
            check("...and a refusal that touched nothing leaves no undo behind",
                  len(rover.call("snapshot_list", {})["snapshots"]), count)
            navigator.refusal = None
            navigator.driving = True
            check("a moving rover is refused before anything is kept",
                  (rover.call("snapshot_restore", {"id": kept["id"]})["ok"],
                   len(rover.call("snapshot_list", {})["snapshots"])), (False, count))
            navigator.driving = False

            gone = rover.call("snapshot_delete", {"id": kept["id"]})
            check("a snapshot can be deleted", gone["ok"], True)
            check("...and is gone", os.path.exists(directory), False)
            check("...leaving the pictures it shared with the store",
                  os.path.exists(store.frame_path(later)), True)

            def refuse_the_map(directory):
                return {"saved": False, "why": "slam_toolbox is not answering"}

            navigator.snapshot_map = refuse_the_map
            failed = rover.call("snapshot_save", {"name": "no map"})
            check("a snapshot whose map cannot be kept is not taken",
                  failed["ok"], False)
            check("...saying why", "not answering" in failed["error"], True)
            check("...and leaves nothing half made",
                  len(os.listdir(os.path.join(root, "snapshots"))), count - 1)
            rover.close_world()
        finally:
            rover_snapshots.CHANGES_DIR = changes
            _put_back(was)


def test_snapshots_are_a_persons_acts_on_the_rover_itself():
    import rover_autonomy
    import rover_daemon

    for name in ("snapshot_save", "snapshot_restore", "snapshot_delete",
                 "snapshot_rename"):
        check(f"{name} is served on the rover itself only",
              name in rover_daemon.LOCAL_ONLY, True)
    check("putting one back ends an autonomous run",
          "snapshot_restore" in rover_autonomy.HUMAN_MOVES, True)
    check("...and says so in the record",
          "snapshot_restore" in rover_autonomy.TAKEOVER, True)


TESTS = (
    test_a_snapshot_is_taken_named_restored_undone_and_deleted,
    test_snapshots_are_a_persons_acts_on_the_rover_itself,
)
