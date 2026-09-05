#!/usr/bin/env python3
"""The pose graph on disk: where it is kept, when it is written, and what says
it is usable.

Its own module with no ROS in it, like `frontier.py` and `refit.py`, so that the
rules can be argued with by `selftest.py` on a workstation. What actually writes
the file is `slam_toolbox`'s own serialisation service -- this decides *when* to
ask for it and holds the note that goes beside it.

## Why the map is kept at all

Until this existed, every boot started an empty map. Everything measured in the
old one -- where the rover had been, and every position in the semantic world
state -- was measured in coordinates that no longer meant anything, which is why
the world state used to be thrown away on every boot. A rover that keeps its map
keeps all of that, and the price is one file that has to be written often enough
to be worth having and rarely enough not to cost anything.

## Where it lives, and why not beside the code

`~/.ugv/map`, next to `~/.ugv/world` and the rover's secrets, and deliberately
outside `~/ugv` where the deploy tree is. A deploy replaces the deploy tree; the
map is not code and must survive one. See AGENTS.md, which draws that line for
the same reason.

## What "usable" means

Three files: `current.posegraph` and `current.data`, which are slam_toolbox's,
and `current.json`, which is this module's note about them -- where the rover
last was, and the identity of the map they hold.

**The note is written last and is the only thing that says the pair is
complete.** A serialisation interrupted by a power cut leaves a truncated
`.data`, and a rover that loaded one would come up with half a house. So the
graph is written under a second name and renamed into place, and the note is
written after both renames: no note, no restore, and the rover falls back to
exactly what it did before any of this existed, which is to map the room again.

## Where the rover is, written far more often than the map it is on

**The pose is not the graph and must not be saved at the graph's pace.** A
house-sized graph is thirteen megabytes and slam_toolbox holds its own mutex to
write one, so a write is seconds of work and is worth doing about once a minute.
The rover's position inside that graph is three numbers, changes every time a
wheel turns, and is the one thing a boot cannot work out for itself: `refit.py`
searches a metre around where it is told the rover was, and a pose a minute stale
is a rover that drove out of that window before it was switched off.

Measured on this rover on 2026-09-05, and this is why it is written down here.
The stack went down mid-drive, four seconds after a graph write had begun -- the
serialisation never finished, so the note that came back was the one from the
save before it, up to a minute of driving earlier. The boot put the rover
faithfully back where that older note said, the scan matched the map there at
56% against the 90% a fit needs, and the rover came up on the far side of its own
search window with no way back onto its map.

So `note_pose` rewrites the pose alone -- a hundred and twenty bytes, no mapper,
no mutex -- on the cadence the wheels set, while `commit` goes on writing the
graph on its own. The two can disagree by a minute of driving and that is the
point: the graph's coordinates do not go stale, only the rover's place in them,
and it is the *newer* half that a restore needs.

## The identity, and what the rest of the rover does with it

`map_id` is a random string minted whenever a genuinely new map is started --
first boot, a `clear_map`, or a restore that failed -- and carried across every
restore of the same graph. It is how anything holding coordinates can tell
whether they still mean anything: the semantic world state keeps its rows and
compares this against the one it recorded them under, and starts a new map
session only when they differ. Without it, "the rover has rebooted" was the only
question anything could ask, and the answer to that is no longer a reason to
throw work away.
"""

import json
import math
import os
import time
import uuid

#: Where the graph and its note live. Outside the deploy tree on purpose; see
#: the module docstring.
MAP_DIR = os.path.expanduser("~/.ugv/map")

#: The name slam_toolbox is given. It appends `.posegraph` and `.data` itself,
#: which is why this is a stem rather than a filename.
STEM = "current"
#: And the stem it is asked to write under first, so that a half-written graph is
#: never the one a boot would load.
STAGING = "next"

#: How often the graph is written, at most, and how far the rover has to have
#: driven since the last one for a write to be worth doing at all.
#:
#: The second condition is not a saving so much as a statement of what the graph
#: is: `minimum_travel_distance` in config/slam_toolbox.yaml means a parked rover
#: adds no nodes, so a second write of a graph nothing has changed is bytes for
#: nothing. Half a metre is two and a half nodes' worth of driving, and a minute
#: bounds what a power cut can cost to a minute of mapping.
SAVE_EVERY_S = 60.0
SAVE_AFTER_M = 0.5
SAVE_AFTER_DEG = 20.0

#: The same three questions asked of the *pose* alone, which is why they are a
#: different size. Rewriting the note is a small atomic write of three numbers
#: rather than a serialisation of the whole graph, so what it costs is a rename
#: and what it buys is the difference between a boot that knows where the rover
#: was a second ago and one that knows where it was a minute ago. Ten
#: centimetres is a fifth of the search window a boot has to find the rover
#: within, and a second is below the time it takes this chassis to cross that.
#:
#: **Dead reckoning gates this exactly as it gates the graph**, for the reason
#: `due` gives at length: a parked rover's believed heading walks with the gyro's
#: residual bias, and gating on the belief would write that drift down as though
#: the rover had turned. Three degrees rather than twenty only because this is
#: the cheap write; a rover left standing still crosses it on drift alone every
#: few minutes, and what lands is the same drifted heading the graph's own gate
#: would have recorded later.
POSE_EVERY_S = 1.0
POSE_AFTER_M = 0.10
POSE_AFTER_DEG = 3.0


def new_id():
    """A fresh map identity. Short enough to read in a log line, random enough
    that two rovers or two maps never collide."""
    return uuid.uuid4().hex[:12]


class SavedMap(object):
    """The saved graph, its note, and the rule about when to write again.

    Takes a directory so the selftest can point it at a temporary one; the
    default is the rover's.
    """

    def __init__(self, directory=None):
        self.dir = directory or MAP_DIR
        #: When the last write finished, and how far the wheels had carried the
        #: rover by then. Both None until this process has written one -- a
        #: restored graph does not count as a write, because what matters is
        #: whether *this* run has anything new to record.
        self.saved_at = None
        self.saved_odom = None
        #: The same pair for the pose on its own, which is written far more often
        #: than the graph it belongs to and therefore keeps its own reckoning of
        #: when it last was.
        self.posed_at = None
        self.posed_odom = None

    # --- where things are -----------------------------------------------------

    @property
    def stem(self):
        """What slam_toolbox is given to load, without an extension."""
        return os.path.join(self.dir, STEM)

    @property
    def staging_stem(self):
        return os.path.join(self.dir, STAGING)

    @property
    def note_path(self):
        return os.path.join(self.dir, STEM + ".json")

    def graph_paths(self, stem=None):
        stem = self.stem if stem is None else stem
        return (stem + ".posegraph", stem + ".data")

    def make(self):
        """Create the directory. Called before every write, because `~/.ugv` on a
        fresh rover has never been near this code."""
        os.makedirs(self.dir, exist_ok=True)

    # --- reading --------------------------------------------------------------

    def held(self):
        """The note beside a complete saved graph, or None.

        None covers every way there can fail to be one -- never saved, half
        saved, hand-deleted, unreadable -- because the caller does the same thing
        in all of them, which is to start a new map and say so.
        """
        graph, data = self.graph_paths()
        if not (os.path.exists(graph) and os.path.exists(data)):
            return None
        try:
            with open(self.note_path, "r", encoding="utf-8") as fh:
                note = json.load(fh)
        except (OSError, ValueError):
            return None
        if not isinstance(note, dict) or not note.get("map_id"):
            return None
        return note

    def start_pose(self):
        """`(x_m, y_m, heading_deg)` the rover was last at, or None.

        This is where a restore puts the rover before it has matched anything:
        the rover is assumed to be where it was when it was switched off, which
        is true unless somebody moved it -- and moving it is what `refit.py` is
        for.

        Last written rather than written with the graph: see `note_pose`. The
        graph beneath it can be up to a minute older, which costs a minute of
        mapping and does not move the frame this pose is measured in.
        """
        note = self.held() or {}
        pose = note.get("pose")
        if not isinstance(pose, dict):
            return None
        try:
            return (float(pose["x_m"]), float(pose["y_m"]),
                    float(pose["heading_deg"]))
        except (KeyError, TypeError, ValueError):
            return None

    # --- writing --------------------------------------------------------------

    def commit(self, map_id, pose, odom=None, **extra):
        """Move a freshly written staging graph into place and write its note.

        The order is the whole point: both graph files are renamed first -- a
        rename within one directory is atomic -- and the note that declares them
        usable is written last. A power cut anywhere in here leaves either the
        previous complete map or no map, never half of this one.
        """
        self.make()
        for src, dst in zip(self.graph_paths(self.staging_stem),
                            self.graph_paths()):
            os.replace(src, dst)
        note = {"map_id": map_id, "saved_at": time.time(),
                "pose": _pose_fields(pose)}
        note.update(extra)
        self._write_note(note)
        self.saved_at = time.monotonic()
        self.saved_odom = None if odom is None else tuple(odom)
        # The pose in it is this fresh too, so the cheap writer has nothing to
        # add for another second.
        self.posed_at = self.saved_at
        self.posed_odom = self.saved_odom
        return note

    def note_pose(self, map_id, pose, odom=None):
        """Write where the rover is now into the note beside the graph it has.

        Returns whether anything was written. The graph is untouched: this is the
        rest of the answer to "where was the rover when it went down", written at
        the pace the rover actually moves rather than at the pace a thirteen
        megabyte serialisation can be afforded. See the module docstring for the
        drive that paid for it.

        **It refuses unless a complete saved map is already there, and unless it
        is this map.** A pose is a coordinate and means nothing without the frame
        it was measured in, so writing one beside a graph that is missing, half
        written, or from a map the rover has since thrown away would be worse
        than writing nothing: it would be a note claiming a rover is somewhere in
        a room it is not in. `held` answers the first question and `map_id`
        answers the second -- the keeper mints a new identity the moment a
        restore fails, and the note on disk still carries the old one until the
        next graph write replaces the pair.
        """
        if pose is None:
            return False
        note = self.held()
        if note is None:
            return False
        if map_id is not None and str(note.get("map_id")) != str(map_id):
            return False
        note["pose"] = _pose_fields(pose)
        note["pose_at"] = time.time()
        self._write_note(note)
        self.posed_at = time.monotonic()
        self.posed_odom = None if odom is None else tuple(odom)
        return True

    def _write_note(self, note):
        """The note, atomically. Written under a temporary name and renamed, so
        that a reader never sees half of one and a power cut never leaves half of
        one behind."""
        tmp = self.note_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(note, fh, sort_keys=True)
        os.replace(tmp, self.note_path)

    def forget(self):
        """Throw the saved map away, for a `clear_map` that means it.

        Removing the note first, for `commit`'s reason read backwards: what makes
        a map loadable is the note, so a crash halfway through this leaves a map
        that will not be loaded rather than a note pointing at files that are
        gone.
        """
        for path in (self.note_path,) + self.graph_paths() + \
                    self.graph_paths(self.staging_stem):
            try:
                os.remove(path)
            except OSError:
                pass
        self.saved_at = None
        self.saved_odom = None
        self.posed_at = None
        self.posed_odom = None

    def due(self, odom, now=None):
        """Whether it is worth writing the graph again, given what the wheels did.

        **Dead reckoning and not the rover's belief about where it is on the map,
        and that distinction was measured rather than reasoned about.** A parked
        rover's map pose is not still: `map -> odom` is only corrected when the
        mapper folds in a scan, which needs motion, so between scans the gyro's
        residual bias walks the believed heading round. Measured on the rover on
        2026-09-05, standing still: 0.8 degrees a minute, and no position drift at
        all. Judged on that, a rover left alone for half an hour would look like a
        rover that had turned twenty degrees, and the graph would be written with
        a heading that had drifted rather than one the rover had. The wheels are
        the honest witness -- they are also what slam_toolbox counts nodes by.

        Answers False on no reading at all: with nothing to say the rover has
        moved, the older graph is the better one to keep.
        """
        if odom is None:
            return False
        now = time.monotonic() if now is None else now
        if self.saved_at is None or self.saved_odom is None:
            return True
        if now - self.saved_at < SAVE_EVERY_S:
            return False
        moved = math.hypot(odom[0] - self.saved_odom[0],
                           odom[1] - self.saved_odom[1])
        turned = abs((odom[2] - self.saved_odom[2] + 180.0) % 360.0 - 180.0)
        return moved >= SAVE_AFTER_M or turned >= SAVE_AFTER_DEG

    def pose_due(self, odom, now=None):
        """Whether the rover has moved enough to be worth writing down again.

        `due` for the pose alone, with the same shape and the same witness, and
        smaller numbers because the write it decides on is a hundred and twenty
        bytes rather than a graph. What it is buying is that the pose on disk is
        never more than a few centimetres behind the rover, so that a boot after a
        power cut mid-drive puts it back inside `refit.py`'s search window instead
        of a minute of driving outside it.
        """
        if odom is None:
            return False
        now = time.monotonic() if now is None else now
        if self.posed_at is None or self.posed_odom is None:
            return True
        if now - self.posed_at < POSE_EVERY_S:
            return False
        moved = math.hypot(odom[0] - self.posed_odom[0],
                           odom[1] - self.posed_odom[1])
        turned = abs((odom[2] - self.posed_odom[2] + 180.0) % 360.0 - 180.0)
        return moved >= POSE_AFTER_M or turned >= POSE_AFTER_DEG


def _pose_fields(pose):
    """`(x_m, y_m, heading_deg)` as the note records it, or None.

    Rounded here rather than at the callers, because two writers of one field
    that round differently make a file whose history reads as movement that did
    not happen.
    """
    if pose is None:
        return None
    return {"x_m": round(float(pose[0]), 3),
            "y_m": round(float(pose[1]), 3),
            "heading_deg": round(float(pose[2]), 1)}
