#!/usr/bin/env python3
"""The map's half of a snapshot: the graph copied out, and a kept one put back.

A snapshot (rover_daemon/rover_snapshots.py) holds the rover's state as it was
at one moment -- the semantic world state, the map it was measured in, and where
the rover stood on that map -- so that the state can be had back later. The
world state is the daemon's to copy; the map is slam_toolbox's, and only this
process can ask it for one, so the daemon asks here.

## Copying out

`snapshot_map` asks slam_toolbox to serialise the graph it is holding *now* into
the snapshot's own directory, beside a note saying which map it is and where the
rover was on it. Not a copy of `~/.ugv/map`, which can be far older than it
looks: the keeper writes the graph only once the rover has driven half a metre,
so on a parked rover the copy on disk is whatever it was when the rover last
moved, and a snapshot is meant to be the map as it is.

Nothing about the saved map changes. The graph goes to another file and the note
to another directory, and the keeper's gates are left alone.

## Putting one back

`restore_map` loads the kept graph into the mapper, makes it the saved map --
graph, note and identity -- and decides where the rover is on it. Like `refit`
it takes the move mutex and is refused while the rover is moving: the route being
followed is in a frame this is about to replace.

**Where the rover is put is a decision, and it is the one that matters.** The
snapshot says where the rover stood when it was taken, which is only true now if
the rover is standing there again. So:

- If the kept map is the map the rover is on now -- the same identity, so the
  same frame -- and the rover's place on it is believed, the rover stays where it
  is. Its own pose is newer than the snapshot's and is measured in the same
  coordinates.
- Otherwise the rover is put where the snapshot says it stood, which is what a
  boot does with a saved map and for the same reason: in that frame there is no
  other candidate. Whether it really is there is then the restore's own check --
  the first look at the lidar withdraws "settled" where the scan does not lie on
  the map (`RESTORE_SUPPORT` in nav_map.py) -- and a refit moves it the rest of
  the way.

Everything after the load is what `map_restore` does after a boot, on purpose: a
restored snapshot is a restored map, and the rules that stop a doubtful anchor
being written over the saved pose apply to it unchanged.
"""

import json
import os
import time

from slam_toolbox.srv import SerializePoseGraph

from nav_map import GRAPH_TIMEOUT_S

#: The stem the graph is written under inside a snapshot's directory;
#: slam_toolbox appends `.posegraph` and `.data` itself.
STEM = "map"
#: The note beside it: which map, and where the rover was on it.
NOTE = "map.json"
#: Where snapshots live, the daemon's choice and the same default. Checked here
#: because this process writes a thirteen-megabyte graph wherever it is told,
#: and a loopback request should not be able to put one anywhere else.
#: Overridable for the same reason the world store's directory is: tests.
ENV_ROOT = "UGV_SNAPSHOT_DIR"


def snapshot_root():
    return os.environ.get(ENV_ROOT) or os.path.expanduser("~/.ugv/snapshots")


def snapshot_dir(directory):
    """`directory` resolved, or None unless it is one snapshot's own directory."""
    if not directory:
        return None
    where = os.path.realpath(directory)
    if os.path.dirname(where) != os.path.realpath(snapshot_root()):
        return None
    return where


def _graph_paths(stem):
    return (stem + ".posegraph", stem + ".data")


def _rounded(pose):
    return None if pose is None else {
        "x_m": round(float(pose[0]), 3), "y_m": round(float(pose[1]), 3),
        "heading_deg": round(float(pose[2]), 1)}


def _write_json(path, payload):
    """Under a temporary name and renamed, so a reader never sees half of one."""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, sort_keys=True)
    os.replace(tmp, path)


class NavSnapshot:
    """The half of `NavBridge` that copies the map out for a snapshot, and back."""

    def snapshot_map(self, directory):
        """Write the graph the mapper holds now, and where the rover is on it.

        Returns a dict with `saved`, and on success the note that was written
        beside the graph: the map's identity, the pose, whether that pose was
        believed (`settled`) and whether the rover was moving. Under the map lock,
        so the keeper never writes the saved map halfway through this.
        """
        where = snapshot_dir(directory)
        if where is None:
            return {"saved": False,
                    "why": "a snapshot's map goes in a directory of its own "
                           "under %s" % (snapshot_root(),)}
        with self.map_lock:
            with self._lock:
                mapped = self.map_msg is not None
                moving = bool(self.driving or self.exploring)
            if not mapped:
                return {"saved": False,
                        "why": "slam_toolbox has not published a map yet, so "
                               "there is no graph to keep"}
            if self.map_id is None:
                return {"saved": False,
                        "why": "the map keeper has not said which map this is "
                               "yet: %s" % (self.map_note,)}
            pose = self.pose_deg()
            if pose is None:
                return {"saved": False,
                        "why": "the rover has no position, so there is nothing "
                               "to record as where it stood"}
            if not self.serialize_client.wait_for_service(timeout_sec=2.0):
                return {"saved": False, "why": "slam_toolbox is not answering"}
            os.makedirs(where, exist_ok=True)
            stem = os.path.join(where, STEM)
            request = SerializePoseGraph.Request()
            request.filename = stem
            future = self.serialize_client.call_async(request)
            if not self.wait(future, GRAPH_TIMEOUT_S):
                return {"saved": False,
                        "why": "slam_toolbox did not finish writing the graph in "
                               "%.0f seconds" % (GRAPH_TIMEOUT_S,)}
            result = future.result()
            if result is None or result.result != result.RESULT_SUCCESS:
                return {"saved": False,
                        "why": "slam_toolbox could not write the graph"}
            paths = _graph_paths(stem)
            if not all(os.path.exists(path) for path in paths):
                return {"saved": False,
                        "why": "slam_toolbox said it wrote the graph and the "
                               "files are not there"}
            note = {"map_id": str(self.map_id), "pose": _rounded(pose),
                    # Whether the pose was believed when it was kept. A restore
                    # that puts the rover back here claims no more than this did.
                    "settled": bool(self.map_trustworthy()),
                    "moving": moving, "map_note": self.map_note,
                    "saved_at": time.time(),
                    "bytes": sum(os.path.getsize(path) for path in paths)}
            _write_json(os.path.join(where, NOTE), note)
        return dict(note, saved=True)

    def restore_map(self, directory, here=None):
        """Load a snapshot's graph, make it the saved map, and put the rover on it.

        `here` is where the rover goes: True keeps it where it is, which is only
        allowed on the same map with a believed pose; False puts it where the
        snapshot says it stood; None decides, as the module docstring says.

        Returns a dict with `restored`, and `touched` -- whether the mapper was
        asked to load anything, so a caller can tell a refusal that changed
        nothing from a load that failed part way.
        """
        where = snapshot_dir(directory)
        if where is None:
            return {"restored": False, "touched": False,
                    "why": "a snapshot's map is in a directory of its own under "
                           "%s" % (snapshot_root(),)}
        try:
            with open(os.path.join(where, NOTE), encoding="utf-8") as fh:
                note = json.load(fh)
            kept = str(note["map_id"])
            stood = (float(note["pose"]["x_m"]), float(note["pose"]["y_m"]),
                     float(note["pose"]["heading_deg"]))
        except (OSError, ValueError, KeyError, TypeError):
            return {"restored": False, "touched": False,
                    "why": "the snapshot has no map that can be read"}
        stem = os.path.join(where, STEM)
        if not all(os.path.exists(path) for path in _graph_paths(stem)):
            return {"restored": False, "touched": False,
                    "why": "the snapshot's graph files are missing"}
        if not self.move_mutex.acquire(blocking=False):
            return {"restored": False, "touched": False,
                    "why": "the rover is moving, and the route it is following "
                           "is in the frame this would replace -- stop it first"}
        try:
            with self.map_lock:
                return self._restore_map_now(stem, note, kept, stood, here)
        finally:
            self.move_mutex.release()

    def _restore_map_now(self, stem, note, kept, stood, here):
        """The restore itself, with the move mutex and the map lock held."""
        live = self.pose_deg()
        same = self.map_id is not None and str(self.map_id) == kept
        believed = same and live is not None and self.map_trustworthy()
        if here is None:
            here = believed
        elif here and not believed:
            return {"restored": False, "touched": False,
                    "why": "the rover cannot stay where it is: "
                           + ("the snapshot was taken on another map" if not same
                              else "it has no position" if live is None
                              else "its place on this map is not confirmed")}
        pose = live if here else stood
        ok, why, landed = self.load_graph(pose, drop_trail=True, stem=stem)
        if landed is None:
            return {"restored": False, "touched": True,
                    "why": "the snapshot's map did not load: %s; the saved map "
                           "on disk is untouched" % (why,)}
        # The mapper holds the snapshot's graph now, so it becomes the saved map
        # too: otherwise the next boot would load the map this replaced, and the
        # world state restored beside it would be measured in a frame that had
        # quietly gone again.
        try:
            self.saved.install(stem, kept, pose)
            stored = ""
        except OSError as error:
            stored = ("; it could not be made the saved map (%s), so a restart "
                      "would bring the old one back" % (error,))
        self.map_id = kept
        self.map_restored = True
        self.map_saved_at = None if stored else time.time()
        self.map_parked_at = pose
        self.map_anchored_at = landed
        # Believed only as far as the snapshot believed it, and only where the
        # mapper put the rover where it was asked: the rule `map_restore` uses.
        self.map_settled = bool(ok) and (bool(here)
                                         or bool(note.get("settled", True)))
        self._restore_unchecked = self.map_settled
        self.map_fit = None
        # And the lidar is asked at once rather than in five minutes, because
        # "the rover is where it was" is the claim most worth testing here.
        self.map_drift = None
        self._map_drift_at = None
        self._map_drift_said = False
        if not ok:
            self.map_note = ("a snapshot's map is back, but %s -- the rover is "
                             "taken to be where the snapshot left it, and a refit "
                             "is what moves it if it is not" % (why,))
        elif here:
            self.map_note = ("a snapshot's map is back, and the rover is where it "
                             "was before, on the same map")
        elif self.map_settled:
            self.map_note = ("a snapshot's map is back, and the rover is taken to "
                             "be where it stood when the snapshot was taken")
        else:
            self.map_note = ("a snapshot's map is back at the pose it was taken "
                             "at, which was not confirmed then either -- a refit "
                             "where the rover really is puts it right")
        self.map_note += stored
        self.saved.restored(self.travelled_deg())
        self.get_logger().info(self.map_note)
        return {"restored": True, "touched": True, "map_id": kept,
                "pose": _rounded(pose), "landed": _rounded(landed),
                "settled": self.map_settled, "here": bool(here),
                "saved": not stored, "why": self.map_note}
