"""The console's side of snapshots: one popup and a connection of its own.

A snapshot is the rover's state kept so it can be had back -- the world state, the
map it was measured in and where the rover stood on it (see
rover_daemon/rover_snapshots.py). The popup lists them, takes a new one, names
one, puts one back and deletes one.

**Putting one back and deleting one are pressed twice**, like clearing the map
and for its reason: one replaces the rover's map and world at once, the other
cannot be undone, and `confirm()` would block the script that holds the stop
button. The arming is kept here rather than in the page, so two browsers agree
on what the next press will do.

**Its own connection**, because a restore is up to two minutes -- the rover's
pose has to land on the map that comes back -- and a save is seconds. Neither may
sit in front of a status poll, a stop or the world popup's calls.

The list is fetched rather than pushed, from `/snapshots.json` under a tag that
moves only when the list does, like the network list and for its reason: it is
kilobytes, and the state goes out ten times a second.
"""
from __future__ import annotations

import time
from typing import Any

import _paths  # noqa: F401 -- console_model
from console_model import CLEAR_ARM_S, Channel

#: How often an open popup asks for the list again, which is how a snapshot an
#: agent or a second browser took turns up without anybody pressing anything.
SNAP_POLL_S = 5.0
#: How long the connection waits for an answer. A restore keeps the state it
#: replaces first, then waits up to three minutes for the map to come back
#: (ros_navigator.RESTORE_MAP_TIMEOUT_S); this is past both.
SNAP_TIMEOUT_S = 260.0
#: What the popup's header says while each of the three slow calls runs.
DOING = {"snapshot_save": "saving", "snapshot_restore": "restoring",
         "snapshot_delete": "deleting"}


class SessionSnapshots:
    """Snapshot actions, replies and payload, mixed into Session."""

    def snaps_reset(self) -> None:
        """The fields, in one place so the constructor and a reconnect agree."""
        #: What rides in the pushed state.
        self.snaps: dict[str, Any] = {
            "open": False, "doing": "", "elsewhere": "", "note": "", "error": "",
            "count": 0, "free_bytes": None, "gen": 0,
        }
        #: What `/snapshots.json` serves.
        self.snap_rows: list[dict[str, Any]] = []
        self.snap_link: Channel | None = None
        self.snap_listing = False
        self.snap_listed_at = 0.0
        #: Which press is waiting for its second: `(what, id, until)`.
        self.snap_armed: tuple[str, str, float] = ("", "", 0.0)

    def snap_connect(self, address: str) -> Channel:
        self.snap_link = Channel("snapshot", address, self.replies,
                                 timeout=SNAP_TIMEOUT_S)
        return self.snap_link

    def snap_abandon(self) -> None:
        """The link was remade, so no reply is coming for what was in flight."""
        self.snap_link = None
        self.snap_listing = False
        if self.snaps["doing"]:
            self.snaps["error"] = (f"the link to the rover was remade while "
                                   f"{self.snaps['doing']}; the list says what "
                                   f"became of it")
            self.snaps["doing"] = ""

    # --- what the buttons ask for --------------------------------------------

    def snap_act(self, action: dict[str, Any]) -> None:
        what = str(action.get("what") or "")
        ident = str(action.get("id") or "")
        if what == "open":
            self.snaps["open"] = True
            self.snaps["error"] = ""
            self.snap_list()
        elif what == "close":
            self.snaps["open"] = False
            self.snap_armed = ("", "", 0.0)
        elif what == "save":
            self.snap_call("snapshot_save", {"name": str(action.get("name") or "")})
        elif what == "rename":
            self.snap_call("snapshot_rename",
                           {"id": ident, "name": str(action.get("name") or "")})
        elif what in ("restore", "delete"):
            if what == "restore" and (self.busy_since is not None or self.exploring):
                self.snaps["error"] = "the rover is moving; stop it first"
                return
            if self.snap_arm(what, ident):
                self.snap_call(f"snapshot_{what}", {"id": ident})

    def snap_arm(self, what: str, ident: str) -> bool:
        """Whether this press is the second of two, arming the first."""
        now = time.monotonic()
        armed_what, armed_id, until = self.snap_armed
        if (armed_what, armed_id) == (what, ident) and now < until:
            self.snap_armed = ("", "", 0.0)
            return True
        self.snap_armed = (what, ident, now + CLEAR_ARM_S)
        return False

    def snap_call(self, name: str, arguments: dict[str, Any]) -> None:
        if self.snap_link is None:
            self.snaps["error"] = f"not connected, so {name} was not sent"
            return
        doing = DOING.get(name, "")
        if doing:
            if self.snaps["doing"]:
                self.snaps["error"] = f"still {self.snaps['doing']}"
                return
            self.snaps.update(doing=doing, note="", error="")
        self.snap_link.submit(name, arguments)

    def snap_list(self) -> None:
        if self.snap_link is None or self.snap_listing:
            return
        self.snap_listing = True
        self.snap_listed_at = time.monotonic()
        self.snap_link.submit("snapshot_list")

    def snap_due(self, now: float) -> None:
        """Ask for the list again while the popup is open, and never when shut."""
        if (self.snaps["open"] and not self.snap_listing
                and now - self.snap_listed_at > SNAP_POLL_S):
            self.snap_list()

    # --- what comes back ------------------------------------------------------

    def snap_handle(self, name: str, body: dict[str, Any], seconds: float) -> None:
        if name == "snapshot_list":
            self.snap_listing = False
            if not body.get("ok"):
                self.snaps["error"] = str(body.get("error") or "no answer")
                return
            rows = body.get("snapshots") or []
            if rows != self.snap_rows:
                self.snap_rows = rows
                self.snaps["gen"] += 1
            self.snaps["count"] = len(rows)
            self.snaps["free_bytes"] = body.get("free_bytes")
            # A save or a restore somebody else started, which this console's own
            # buttons have to wait for too.
            self.snaps["elsewhere"] = "" if self.snaps["doing"] else str(
                body.get("doing") or "")
            return
        if name in DOING:
            self.snaps["doing"] = ""
        if not body.get("ok"):
            error = str(body.get("error") or "no answer")
            undo = body.get("undo") or {}
            if undo.get("name"):
                error += f" -- the state before is kept as {undo['name']}"
            self.snaps["note"] = ""
            self.snaps["error"] = error
            self.snap_list()
            return
        self.snaps["error"] = ""
        if name == "snapshot_save":
            kept = body.get("snapshot") or {}
            self.snaps["note"] = (
                f"saved -- {kept.get('name')}: {kept.get('entities') or 0} things, "
                f"{kept.get('observations') or 0} looks, {seconds:.0f} s")
        elif name == "snapshot_rename":
            self.snaps["note"] = f"renamed -- {(body.get('snapshot') or {}).get('name')}"
        elif name == "snapshot_delete":
            self.snaps["note"] = f"deleted -- {(body.get('deleted') or {}).get('name')}"
        elif name == "snapshot_restore":
            self.snaps["note"] = _restored_note(body, seconds)
            # On the main page too: the popup is usually shut again by the time
            # anybody looks at the map that changed under it.
            self.say(self.snaps["note"], "good")
            self.snap_restored()
        self.snap_list()

    def snap_restored(self) -> None:
        """The map and the world on screen are the ones that were replaced."""
        self.refresh_map()
        self.world_frames.clear()
        self.world_selected = ""
        had = bool(self.world_payload)
        self.world_payload = {}
        if self.world["open"]:
            self.world_refresh()
        if had:
            self.world_bump()

    def snap_state(self) -> dict[str, Any]:
        """The part of snapshots that rides in every pushed state."""
        what, ident, until = self.snap_armed
        return dict(self.snaps, gen=self.tag(self.snaps["gen"]),
                    armed=({"what": what, "id": ident}
                           if what and until > time.monotonic() else None))


def _restored_note(body: dict[str, Any], seconds: float) -> str:
    """What a restore did, in the popup's one line."""
    kept = body.get("restored") or {}
    moved = body.get("map") or {}
    world = body.get("world") or {}
    parts = []
    if moved:
        parts.append("rover kept where it was" if moved.get("here")
                     else "rover placed where it stood then")
        if not moved.get("settled"):
            parts.append("its place on the map unconfirmed")
    if world:
        parts.append(f"{world.get('entities', 0)} things, "
                     f"{world.get('observations', 0)} looks")
    note = f"restored -- {kept.get('name')}"
    if parts:
        note += ": " + ", ".join(parts)
    undo = body.get("undo") or {}
    if undo.get("name"):
        note += f"; previous state kept as {undo['name']}"
    return note + f" -- {seconds:.0f} s"
