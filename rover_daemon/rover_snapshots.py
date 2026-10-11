"""Snapshots of the rover's state: taken, listed, named, put back and deleted.

A snapshot is what the rover knows about the room at one moment, kept so it can
be had back: the semantic world state, the map that world state was measured in,
where the rover stood on that map, and what the change watch had seen. One
directory each under `~/.ugv/snapshots/`, outside the deploy tree like everything
else the rover learns:

    <id>/snapshot.json      its name, when, and what it holds -- written last, and
                            the only thing that says the rest is complete
    <id>/map.posegraph      slam_toolbox's graph, serialised for the snapshot
    <id>/map.data           (ros_nav/nav_snapshot.py)
    <id>/map.json           which map, and where the rover was on it
    <id>/grid.json          the occupancy grid as the navigator served it
    <id>/world.db           the world store (world_state/snapshot.py)
    <id>/frames/            its pictures, shared with the store by hard link
    <id>/changes/<map>.npz  the change watch's tallies for that map

The id is when it was taken and never changes. The name is the person's, lives
only in the note, and starts as the date and time; renaming is rewriting the note.

Not in one: the trail, which a restore redraws from where the rover stands; the
autonomy record, which is a log of what happened and is not un-happened by
going back; and anything that describes the rover rather than the room --
calibration, the battery log, deploy state.

## Putting one back

A person's act: a control call and never a model tool, and it ends an autonomous
run the way clearing the map does (`rover_autonomy.HUMAN_MOVES`). In order:

1. The rover's looks are held off, and the store stops following the map while
   the map changes underneath it.
2. The state as it is now is kept as a snapshot of its own, "before restoring
   ...", so a restore is undone by restoring that.
3. The change watch's tallies go back where the watch looks for them, unless it
   already holds tallies for that map, which are newer evidence of the same room.
4. The navigator loads the snapshot's graph, makes it the saved map, and decides
   where the rover stands on it (nav_snapshot.py says how).
5. The world store becomes the snapshot's, rows and pictures.

If the map cannot be put back, nothing else is either: a world state restored
onto a map it was not measured in is a room full of things in the wrong places.
When the navigator refused without touching anything, the undo snapshot goes
again, because there is nothing to undo.
"""
from __future__ import annotations

import contextlib
import json
import os
import re
import shutil
import sys
import threading
import time
import uuid
from typing import Any

from rover_world import world_state

#: Where snapshots live. The navigator checks the same place, and the same
#: variable moves both, which only the tests do.
ENV_ROOT = "UGV_SNAPSHOT_DIR"
NOTE = "snapshot.json"
#: The change watch's directory (ros_nav/change_node.py), one tally per map.
CHANGES_DIR = os.path.expanduser("~/.ugv/changes")
#: How long a save or a restore waits for the look in flight to finish: the same
#: twenty seconds a clear waits, for the same reason (`inspector.not_looking`).
LOOK_WAIT_S = 20.0
#: How long to wait for the occupancy grid that goes in beside the graph.
GRID_ASK_S = 8.0
#: A name is a line in a list, not a description.
NAME_MAX = 80
ID_PATTERN = re.compile(r"\d{8}-\d{6}-[0-9a-f]{4}")

#: One save or restore at a time, across the daemon. A restore takes a snapshot of
#: its own on the way, and two of either at once would hold the looks off twice
#: and race for the map lock in the navigator.
_BUSY = threading.Lock()


class _Refused(Exception):
    """A snapshot that will not be taken, in the sentence that says why."""


def snapshot_root() -> str:
    return os.environ.get(ENV_ROOT) or os.path.expanduser("~/.ugv/snapshots")


def _clean_name(value: Any) -> str:
    """One line of printable text, trimmed, or "" for none."""
    text = "".join(ch if ch.isprintable() else " " for ch in str(value or ""))
    return " ".join(text.split())[:NAME_MAX].strip()


def _write_json(path: str, payload: dict[str, Any]) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, sort_keys=True)
    os.replace(tmp, path)


def _read_note(directory: str) -> dict[str, Any] | None:
    try:
        with open(os.path.join(directory, NOTE), encoding="utf-8") as handle:
            note = json.load(handle)
    except (OSError, ValueError):
        return None
    return note if isinstance(note, dict) and note.get("id") else None


def _remove_tree(path: str) -> None:
    """Delete a snapshot's directory, note first so a half-deleted one is not
    listed. A file Windows calls read-only is made writable and tried again."""
    try:
        os.remove(os.path.join(path, NOTE))
    except OSError:
        pass

    def again(function, where, *_):
        try:
            os.chmod(where, 0o700)
            function(where)
        except OSError:
            pass

    if sys.version_info >= (3, 12):
        shutil.rmtree(path, onexc=again)
    else:
        shutil.rmtree(path, onerror=again)


def _bytes(directory: str) -> int:
    """What a snapshot costs on disk. The pictures are left out: they are shared
    with the store and with every other snapshot holding them, so deleting this
    one gives none of them back."""
    total = 0
    for here, folders, files in os.walk(directory):
        if os.path.basename(here) == "frames":
            folders[:] = []
            continue
        total += sum(os.path.getsize(os.path.join(here, name)) for name in files)
    return total


def _listed(note: dict[str, Any], live_map: str | None) -> dict[str, Any]:
    """One snapshot as the console lists it."""
    kept = note.get("map") or {}
    world = note.get("world") or {}
    return {
        "id": note.get("id"), "name": note.get("name"),
        "created_at": note.get("created_at"), "auto": bool(note.get("auto")),
        "map_id": kept.get("map_id"), "pose": kept.get("pose"),
        "settled": kept.get("settled"),
        # Whether restoring it changes which map the rover is on. Worth saying,
        # because the same map means the rover can stay where it is.
        "same_map": bool(kept.get("map_id")) and kept.get("map_id") == live_map,
        "entities": world.get("entities"), "placed": world.get("placed"),
        "observations": world.get("observations"), "frames": world.get("frames"),
        "bytes": note.get("bytes"), "notes": note.get("notes") or [],
    }


class RoverSnapshots:
    """Snapshot control calls, mixed into Rover."""

    # --- where they are -------------------------------------------------------

    def _snapshot_live_map(self) -> str | None:
        navigator = getattr(self, "nav", None)
        if navigator is None:
            return None
        try:
            return navigator.status().get("map_id")
        except Exception:
            return None

    def _snapshot_find(self, ident: Any) -> tuple[str, dict[str, Any]]:
        ident = str(ident or "")
        if not ID_PATTERN.fullmatch(ident):
            raise _Refused("that is not a snapshot's id")
        directory = os.path.join(snapshot_root(), ident)
        note = _read_note(directory)
        if note is None or note.get("id") != ident:
            raise _Refused(f"there is no snapshot {ident}")
        return directory, note

    # --- the calls ------------------------------------------------------------

    def _tool_snapshot_list(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """Every complete snapshot, newest first. A control call, and read-only."""
        root = snapshot_root()
        live = self._snapshot_live_map()
        rows = []
        try:
            names = os.listdir(root)
        except FileNotFoundError:
            names = []
        for name in names:
            note = _read_note(os.path.join(root, name))
            if note is not None and note.get("id") == name:
                rows.append(_listed(note, live))
        rows.sort(key=lambda row: float(row.get("created_at") or 0.0), reverse=True)
        try:
            free = shutil.disk_usage(root if os.path.isdir(root)
                                     else os.path.expanduser("~")).free
        except OSError:
            free = None
        return {"ok": True, "snapshots": rows, "map_id": live, "free_bytes": free,
                "doing": getattr(self, "_snapshot_doing", "")}

    def _tool_snapshot_save(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Keep the rover's state as it is now. A control call.

        `name` is optional, and the date and time stand in for one until somebody
        renames it. Holds the rover's looks off for the second the world store
        takes to copy; the map's copy before that holds nothing but the map.
        """
        if not _BUSY.acquire(blocking=False):
            return {"ok": False, "error": "a snapshot is already being saved or "
                                          "restored"}
        try:
            self._snapshot_doing = "saving"
            return self._snapshot_save(_clean_name(arguments.get("name")))
        finally:
            self._snapshot_doing = ""
            _BUSY.release()

    def _tool_snapshot_rename(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Give a snapshot a name. A control call; nothing but the note changes."""
        try:
            directory, note = self._snapshot_find(arguments.get("id"))
        except _Refused as why:
            return {"ok": False, "error": str(why)}
        name = _clean_name(arguments.get("name"))
        if not name:
            return {"ok": False, "error": "a snapshot needs a name"}
        note["name"] = name
        _write_json(os.path.join(directory, NOTE), note)
        return {"ok": True, "snapshot": _listed(note, self._snapshot_live_map())}

    def _tool_snapshot_delete(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Delete a snapshot. A control call, and it cannot be undone.

        Refused while a save or a restore is running, which may be reading it.
        Pictures it shares with the store or another snapshot stay where they are.
        """
        try:
            directory, note = self._snapshot_find(arguments.get("id"))
        except _Refused as why:
            return {"ok": False, "error": str(why)}
        if not _BUSY.acquire(blocking=False):
            return {"ok": False, "error": "a snapshot is being saved or restored; "
                                          "try again when it has finished"}
        try:
            _remove_tree(directory)
        finally:
            _BUSY.release()
        return {"ok": True, "deleted": {"id": note.get("id"),
                                        "name": note.get("name")}}

    def _tool_snapshot_restore(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Put the rover's state back as a snapshot kept it. A control call.

        The state it replaces is kept first as a snapshot of its own, which is
        how a restore is undone. Refused while the rover is moving. See the module
        docstring for the order, and ros_nav/nav_snapshot.py for where the rover
        is put: the navigator decides unless `rover` says "here" (keep it where
        it is, on the same map only) or "snapshot" (where the snapshot stood).
        """
        try:
            directory, note = self._snapshot_find(arguments.get("id"))
        except _Refused as why:
            return {"ok": False, "error": str(why)}
        rover = arguments.get("rover")
        if rover not in (None, "here", "snapshot"):
            return {"ok": False, "error": "rover is \"here\" or \"snapshot\", or "
                                          "left out for the navigator to decide"}
        if not _BUSY.acquire(blocking=False):
            return {"ok": False, "error": "a snapshot is already being saved or "
                                          "restored"}
        self._snapshot_doing = "restoring"
        self._world_restoring = True
        try:
            return self._snapshot_restore(
                directory, note, None if rover is None else rover == "here")
        except _Refused as why:
            return {"ok": False, "error": str(why)}
        finally:
            self._world_restoring = False
            self._snapshot_doing = ""
            _BUSY.release()

    # --- taking one -----------------------------------------------------------

    def _snapshot_save(self, name: str, auto: bool = False,
                       held: bool = False) -> dict[str, Any]:
        """Take one. `auto` is the undo snapshot a restore takes, which keeps what
        it can rather than refusing over a map that cannot be had; `held` says the
        caller already holds the looks off."""
        navigator = getattr(self, "nav", None)
        world_why = self._world_ready() if self.world_installed else ""
        if navigator is None and not self.world_installed:
            return {"ok": False, "error": "this rover has neither a map nor a "
                                          "world state to keep"}
        if world_why:
            return {"ok": False, "error": world_why}
        began = time.time()
        ident = (time.strftime("%Y%m%d-%H%M%S", time.localtime(began)) + "-"
                 + uuid.uuid4().hex[:4])
        directory = os.path.join(snapshot_root(), ident)
        os.makedirs(directory)
        note: dict[str, Any] = {
            "id": ident, "created_at": round(began, 3), "auto": bool(auto),
            "name": name or time.strftime("snapshot %Y-%m-%d %H:%M",
                                          time.localtime(began)),
            "map": None, "world": None, "changes": None, "notes": []}
        try:
            if navigator is None:
                note["notes"].append("no navigator, so no map was kept")
            else:
                self._snapshot_map_out(navigator, directory, note, auto)
            if self.world_installed:
                store = self._world_store()
                if held:
                    note["world"] = world_state.snapshot.copy_out(store, directory)
                else:
                    with self._world_inspector().not_looking(LOOK_WAIT_S) as idle:
                        if not idle:
                            raise _Refused(f"an inspection has been running for "
                                           f"longer than {LOOK_WAIT_S:.0f} s")
                        note["world"] = world_state.snapshot.copy_out(store,
                                                                      directory)
            else:
                note["notes"].append("no world state component, so no world "
                                     "state was kept")
            note["bytes"] = _bytes(directory)
            note["took_s"] = round(time.time() - began, 2)
            _write_json(os.path.join(directory, NOTE), note)
        except _Refused as why:
            _remove_tree(directory)
            return {"ok": False, "error": f"nothing was kept: {why}"}
        except Exception as error:
            _remove_tree(directory)
            return {"ok": False, "error": f"nothing was kept: "
                                          f"{type(error).__name__}: {error}"}
        return {"ok": True, "snapshot": _listed(note, self._snapshot_live_map())}

    def _snapshot_map_out(self, navigator, directory: str,
                          note: dict[str, Any], auto: bool) -> None:
        """The graph, the grid and the change watch's tallies, into `directory`."""
        kept = navigator.snapshot_map(directory)
        if not kept.get("saved"):
            why = str(kept.get("why") or "the navigator did not say why")
            if not auto:
                raise _Refused(f"the map could not be kept: {why}")
            note["notes"].append(f"no map was kept: {why}")
            return
        note["map"] = {key: kept.get(key) for key in (
            "map_id", "pose", "settled", "moving", "bytes", "map_note")}
        # The occupancy grid too: a snapshot can then be looked at -- or scored
        # against, as M4 does with its own copies -- without slam_toolbox.
        ask = getattr(navigator, "ask", None)
        grid = ask({"op": "map"}, GRID_ASK_S) if ask is not None else {}
        if grid.get("ok") and grid.get("data"):
            _write_json(os.path.join(directory, "grid.json"), {
                key: grid.get(key) for key in (
                    "width", "height", "resolution_m", "origin_x_m",
                    "origin_y_m", "data", "pose")})
        tallies = os.path.join(CHANGES_DIR, f"{kept.get('map_id')}.npz")
        if os.path.exists(tallies):
            os.makedirs(os.path.join(directory, "changes"), exist_ok=True)
            shutil.copy2(tallies, os.path.join(directory, "changes"))
            note["changes"] = os.path.basename(tallies)

    # --- putting one back -----------------------------------------------------

    def _snapshot_restore(self, directory: str, note: dict[str, Any],
                          here: bool | None = None) -> dict[str, Any]:
        navigator = getattr(self, "nav", None)
        has_map, has_world = bool(note.get("map")), bool(note.get("world"))
        if has_map and navigator is not None:
            status = navigator.status()
            if status.get("driving") or status.get("exploring"):
                raise _Refused("the rover is moving, and the map is the ground it "
                               "is driving on -- stop it first")
        if has_world and not self.world_installed:
            raise _Refused("this snapshot holds a world state and this rover has "
                           "no world_state component to take it back")
        if has_world and self._world_ready():
            raise _Refused(self._world_ready())
        inspector = self._world_inspector() if self.world_installed else None
        holding = (inspector.not_looking(LOOK_WAIT_S) if inspector is not None
                   else contextlib.nullcontext(True))
        notes: list[str] = []
        with holding as idle:
            if not idle:
                raise _Refused(f"an inspection has been running for longer than "
                               f"{LOOK_WAIT_S:.0f} s; nothing was restored")
            undo = self._snapshot_save(
                _clean_name(f"before restoring {note.get('name')}"),
                auto=True, held=True)
            if not undo.get("ok"):
                raise _Refused("the state as it is now could not be kept first, so "
                               "nothing was restored: " + str(undo.get("error")))
            kept = undo["snapshot"]
            notes += self._snapshot_changes_back(directory, note)
            moved = None
            if has_map and navigator is None:
                notes.append("no navigator, so the snapshot's map was not loaded")
            elif has_map:
                moved = navigator.restore_map(directory, here)
                if not moved.get("restored"):
                    if not moved.get("touched"):
                        # Nothing changed, so there is nothing to undo.
                        _remove_tree(os.path.join(snapshot_root(), kept["id"]))
                        kept = None
                    return {"ok": False, "undo": kept,
                            "error": "the map was not restored, so nothing was: "
                                     + str(moved.get("why"))}
                self._snapshot_rover_moved()
            elif navigator is not None:
                notes.append("the snapshot holds no map, so the rover's map is "
                             "the one it had")
            world = None
            if has_world:
                store = self._world_store()
                try:
                    world = world_state.snapshot.copy_in(store, directory)
                except Exception as error:
                    return {"ok": False, "undo": kept, "map": moved,
                            "error": ("the map was restored but the world state was "
                                      "not" if moved else "the world state was not "
                                      "restored") + f": {type(error).__name__}: "
                                                    f"{error}"}
                self._snapshot_world_replaced(inspector, store)
            elif self.world_installed:
                notes.append("the snapshot holds no world state, so the rover's "
                             "own was kept")
        # Told which map it is on only now, with the looks free again: the store
        # and the map came back together, so this is ordinarily no change at all.
        self._world_restoring = False
        self._world_map_id = None
        self._world_map_now()
        return {"ok": True, "restored": _listed(note, self._snapshot_live_map()),
                "undo": kept, "map": moved, "world": world, "notes": notes,
                "why": _restored_sentence(note, moved, world, kept)}

    def _snapshot_changes_back(self, directory: str,
                               note: dict[str, Any]) -> list[str]:
        """The change watch's tallies, back where it looks, unless it has its own.

        Before the map is restored, because the watch picks its tallies up the
        moment it sees the map identity change (change_node.switch_map).
        """
        name = note.get("changes")
        if not name:
            return []
        source = os.path.join(directory, "changes", str(name))
        target = os.path.join(CHANGES_DIR, str(name))
        if os.path.exists(target) or not os.path.exists(source):
            return []
        os.makedirs(CHANGES_DIR, exist_ok=True)
        shutil.copy2(source, target)
        return ["the change watch's tallies for that map were put back"]

    def _snapshot_rover_moved(self) -> None:
        """The rover's place on the map was replaced, so nothing measured about it
        before speaks for it now: the heading check's last correction, and the
        grid the looks were bounded by."""
        self._world_grid_cache = None
        self._world_build_from = None
        self._world_rest = None
        inspector = getattr(self, "_world_inspector_cache", None)
        heading = getattr(inspector, "heading", None)
        if heading is not None:
            heading.forget()

    def _snapshot_world_replaced(self, inspector, store) -> None:
        """The store holds other rows now, so what was worked out from the old
        ones goes: the picture a look is compared against, the last settling
        pass, and the groups file, which the daemon then has made again."""
        if inspector is not None:
            inspector.forget_picture()
        self._world_settled = {}
        self._world_groups_cache = None
        try:
            os.remove(os.path.join(store.dir, "groups.json"))
        except OSError:
            pass


def _restored_sentence(note: dict[str, Any], moved: dict[str, Any] | None,
                       world: dict[str, Any] | None,
                       undo: dict[str, Any] | None) -> str:
    """What a restore did, in one line, for a caller with no console."""
    parts = []
    if moved:
        parts.append("the map" + (", with the rover where it was" if moved.get("here")
                                  else ", with the rover where it stood then"))
    if world:
        parts.append(f"{world.get('entities', 0)} things and "
                     f"{world.get('observations', 0)} looks")
    said = f"restored {note.get('name')}: " + (", ".join(parts) or "nothing")
    if undo:
        said += f"; the state before is kept as {undo.get('name')}"
    return said
