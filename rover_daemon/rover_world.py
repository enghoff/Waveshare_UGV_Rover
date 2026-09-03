"""The daemon's side of the semantic world state: control calls, and one camera.

Everything here is a control call and none of it is a model tool. `tools()` builds
its list from the schemas in `tool_schemas.py` and nothing in this file is in them,
so a voice model is never shown any of it -- which is the intention. This slice
exists to find out whether the world state is worth trusting; giving a model the
authority to write to it, or to clear it, before that question is answered would be
the wrong order.

Why the store lives in this process at all, when the *model* deliberately does not:
the camera has exactly one owner. An inspection needs the frame the gimbal is
actually looking at, and a second process opening the camera behind the tracking
loop's back is the failure this daemon exists to prevent. So the picture is taken
here, by the same `_whole_jpeg` that answers `camera_jpeg` and `look`, and only the
model -- the part that can run out of memory and take a fault -- is out of process.
"""
from __future__ import annotations

import base64
import math
import os
import sys
import threading
import time
from typing import Any

#: `world_state` is a package rather than a flat module, and it sits in a different
#: place in the repository than it does on the rover: beside this file's directory
#: in the checkout, and inside it once everything has been deployed under ~/ugv.
#: The same two-candidate dance `ros_navigator.py` does, for the same reason.
_HERE = os.path.dirname(os.path.abspath(__file__))
for _candidate in (_HERE, os.path.dirname(_HERE)):
    if os.path.isdir(os.path.join(_candidate, "world_state")):
        if _candidate not in sys.path:
            sys.path.insert(0, _candidate)
        break

try:
    import world_state
    from world_state import view as world_view
except Exception as _error:                     # deployed without the component
    world_state = None
    world_view = None
    WORLD_IMPORT_ERROR = f"{type(_error).__name__}: {_error}"
else:
    WORLD_IMPORT_ERROR = ""

#: How many observations one entity's detail carries. Enough to see whether the
#: descriptions have drifted, bounded because this crosses a socket to a phone.
DETAIL_LIMIT = 40
#: How many of an entity's newest observations become rays on the map, in the
#: list where every entity is drawn at once.
RAY_LIMIT = 6
#: And how many for the one entity somebody has chosen. Higher because the
#: question changes: in the list the map has to stay readable with every thing on
#: it at once, while a chosen thing's sightings all end at its own settled
#: position, so more of them is more evidence about whether they converge rather
#: than more clutter. Bounded all the same -- the rover records a look a second,
#: and this crosses a socket to a phone.
SELECTED_RAY_LIMIT = 24
#: How long to wait before asking the camera a second time. Long enough for the
#: previous v4l2-ctl to be well out of the way, short enough to be nothing beside
#: the minute of model that follows. See :meth:`RoverWorld._world_capture`.
CAMERA_RETRY_S = 0.5
#: Substitutes the deterministic fake for the real sidecar. For bringing the
#: console up on the rover before the encoders are installed, and for nothing
#: else -- every row it writes is stamped with the backend that wrote it and the
#: popup shows that, so a rover left in this state says so rather than looking
#: like one that is really measuring anything.
ENV_FAKE = "UGV_WORLD_FAKE"


#: The fastest the rover will ever look around by itself. **It was 15 s, and 15 s
#: is why a three-minute drive came back with four pictures.** What set it was the
#: resolver reading the whole pending pool on every look -- so a rover recording
#: faster than it could place things got slower and slower at placing them -- and
#: that is no longer what happens: the pool is settled on its own schedule below,
#: and one pass over it got 145 times cheaper when the geometry was allowed to
#: throw a pair out before its appearance was compared.
#:
#: What sets it now is the camera and the lidar. A bounded capture is 0.29 s on
#: the Orin and a look through the encoders 0.16 s, so a look a second is a 45%
#: duty cycle; measured against the scan matcher on 2026-09-03, that costs it
#: nothing -- 9.95 revolutions a second with a capture every second against 9.90
#: with none, and no dropped scans in either. **That measurement is the Orin's
#: and does not transfer.** The same experiment on the Pi's four cores, recorded
#: in `uvc_camera.snapshot`, lost 22% of revolutions to a camera held open, and
#: the scan matcher is the only odometer this rover has.
LOOK_EVERY_S = 1.0
#: How often identity is decided, in seconds. Separate from looking because the
#: two cost wildly different amounts: a look is a near-constant 0.45 s, while one
#: resolver pass is 1.4 s at 500 pending bearings and 8 s at 2000 -- it compares
#: every pair, so it grows as the square of what is waiting. Settling after every
#: look is what made looking often unaffordable.
SETTLE_EVERY_S = 10.0
#: What counts as somewhere new. **Deliberately shorter than the 0.4 m the
#: geometry calls a baseline**, which is what this was: two looks this close
#: cannot be triangulated against *each other*, but they can be against the look
#: three back, and in the meantime each is a picture of the room from a place the
#: rover has not photographed. At the 0.35 m/s it explores at this is a look every
#: 0.4 s, so `LOOK_EVERY_S` above is the real governor while driving and this one
#: takes over as the rover slows down.
MOVED_ENOUGH_M = 0.15
#: And what counts as a new direction. **Where the camera points, not where the
#: chassis does**: heading minus pan, which is what a bearing is built from in the
#: first place. Measured against the chassis alone, turning the gimbal through
#: three positions recorded nothing at all, which is the one case a rover standing
#: still can still learn something from.
TURNED_ENOUGH_DEG = 25.0
#: A rover that has not moved at all still looks this often, so that a parked
#: rover in a room that changes notices, and so that "nothing is happening" is
#: distinguishable from "it stopped working".
LOOK_ANYWAY_S = 300.0
#: And how often it looks when it cannot tell whether it has moved -- when the
#: scan matcher has stopped trusting its own position. Slower than a look a second
#: because none of these can be triangulated with anything, and far faster than
#: the five minutes above because the pictures are still worth having and this is
#: the part of the building that just confused the rover.
LOOK_BLIND_S = 5.0

#: How long a fetched occupancy grid is reused for. One resolve pass asks how far
#: the rover could see for every bearing in the pending pool -- a few hundred
#: questions of the same map -- and refetching for each would be a few hundred
#: round trips to the bridge and a few hundred decompressions of the same 80 kB.
#: Short enough that a map five seconds further explored is used on the next look,
#: which is far finer than the rover can drive.
MAP_CACHE_S = 5.0
#: No sighting is bounded beyond this, in metres. It is `locate.MAX_RANGE_M`,
#: which already refuses a crossing further out, so walking past it would only
#: cost time.
REACH_LIMIT_M = 12.0
#: What counts as a wall in the grid the bridge sends. The same threshold
#: `ros_navigator.GRID_OCCUPIED_AT` renders a map with, stated here rather than
#: imported because this module already survives that one being missing.
OCCUPIED_AT = 50
#: How long to wait for the bridge to send the map. The same patience `map_png`
#: has, for the same request.
MAP_ASK_S = 8.0


class RoverWorld:
    """Semantic world state, as calls on the daemon's existing protocol.

    A mixin on `Rover` rather than a service of its own, because the console
    already holds several connections to this daemon and the alternative was
    another LAN port to secure, discover and keep alive for four control calls.
    """

    # --- building it without being asked --------------------------------------

    def start_world_building(self) -> None:
        """Look around on a schedule, from the moment the daemon starts.

        **On by default**, because a world state that only records when somebody
        presses a button is a world state that is empty whenever it is wanted. The
        rover drives across a building and learns nothing on the way unless
        something asks it to look, and nothing did.

        Its own thread, and it only ever makes the same call the console's button
        makes. Nothing here reaches into the store or the resolver directly, so a
        fault in this loop cannot corrupt anything -- at worst it stops looking.
        """
        self._world_build = True
        self._world_build_stop = threading.Event()
        self._world_build_at = 0.0
        self._world_settle_at = 0.0
        self._world_settled: dict[str, Any] = {}
        self._world_build_from = None
        self._world_build_looks = 0
        self._world_build_error = ""
        thread = threading.Thread(target=self._world_building_loop,
                                  name="world-building", daemon=True)
        self._world_build_thread = thread
        thread.start()

    def world_building(self) -> bool:
        return bool(getattr(self, "_world_build", False))

    def _world_worth_looking(self, now: float) -> bool:
        """Whether a look from where the rover stands now would tell it anything.

        A look is worth taking when the rover is somewhere it has not looked from,
        or pointing somewhere it has not pointed, or when enough time has passed
        that the room may simply have changed. Standing still and looking the same
        way over and over records observations that can never be triangulated with
        the ones already there, and every one of them slows the next look down.

        **No pose is a reason to keep looking, not to stop.** It used to return
        False here, so a rover whose scan matcher had lost confidence fell back to
        one look every five minutes -- exactly while it was driving through the
        part of the building that had confused it, and exactly when a picture is
        worth most. What a look with no pose stores is the frame, the regions and
        the vectors with no bearing, which is a state the store already handles
        honestly, so the picture is kept and only the direction is missing.
        """
        since = now - self._world_build_at
        if since < LOOK_EVERY_S:
            return False
        if since >= LOOK_ANYWAY_S:
            return True
        before = self._world_build_from
        if before is None:
            return True
        pose = self._world_pose()
        if pose is None:
            return since >= LOOK_BLIND_S
        moved = math.hypot(pose["x_m"] - before["x_m"],
                           pose["y_m"] - before["y_m"])
        turned = abs((self._world_camera_deg(pose)
                      - before["camera_deg"] + 180.0) % 360.0 - 180.0)
        return moved >= MOVED_ENOUGH_M or turned >= TURNED_ENOUGH_DEG

    def _world_camera_deg(self, pose: dict[str, Any]) -> float:
        """Where the camera is looking: the chassis, less the gimbal's pan.

        The gimbal takes pan positive to the right and the map takes bearings
        positive to the left, which is the same conversion `view.ray` makes when
        it turns a look into a bearing. Using the chassis alone would mean a rover
        that swung its camera across the whole room counted as having seen
        nothing new.
        """
        return pose["heading_deg"] - float(getattr(self, "pan", 0.0) or 0.0)

    def _world_building_loop(self) -> None:
        """Never raises, and looks while the rover drives.

        **It used to refuse to look while the wheels were turning at all, and
        that is why a several-minute drive came back with four pictures.** The
        argument for refusing was sound as far as it went -- a look taken mid-
        drive carries the pose the rover had reached rather than the pose the
        shutter opened at, and a bearing is only as good as the pose behind it --
        but the remedy was far too blunt. What replaced it is a measurement:
        `Inspector` reads the pose on both sides of the capture, uses the
        midpoint, and drops the bearing on the looks where the rover covered more
        ground than that midpoint can account for. So a drive down a corridor now
        records a picture a second, with a bearing on every one taken steadily
        enough to support one, instead of recording nothing at all.

        Two things had to be true before that was affordable, and both were
        measured on 2026-09-03 rather than assumed. A capture every second costs
        the scan matcher nothing on this host, and the resolver no longer runs
        inside every look -- it runs on `SETTLE_EVERY_S` below, because a look is
        a flat 0.45 s and a resolver pass grows as the square of the pool.
        """
        while not self._world_build_stop.wait(0.2):
            try:
                if not self._world_build:
                    continue
                now = time.monotonic()
                if self._world_ready():
                    # No component, or no database. Wait out the long gap rather
                    # than asking again every fifth of a second for the life of
                    # the daemon.
                    self._world_build_at = now
                    self._world_settle_at = now
                    continue
                # **Settling comes first when it is due, and it takes the whole
                # turn.** It is due ten times less often than a look, so the cost
                # of that is one look's slot in ten; the other order starves it
                # completely, because a driving rover has a look due every second
                # and identity would then never be decided until it stopped.
                if now - self._world_settle_at >= SETTLE_EVERY_S:
                    self._world_settle_at = now
                    outcome = self._world_inspector().settle()
                    if outcome.get("ok"):
                        self._world_settled = {
                            "at": time.time(),
                            "considered": outcome.get("considered", 0),
                            "matched": outcome.get("matched", 0),
                            "created": outcome.get("created", 0),
                            "waiting": outcome.get("still_waiting", 0)}
                    continue
                if self._world_worth_looking(now):
                    self._world_build_at = now
                    # Recorded and not settled: identity is decided above, on its
                    # own clock, so that looking often stays as cheap as one look.
                    answer = self._tool_world_inspect({"settle": False})
                    where = self._world_pose()
                    if where is not None:
                        where["camera_deg"] = self._world_camera_deg(where)
                    self._world_build_from = where
                    if answer.get("ok"):
                        self._world_build_looks += 1
                        self._world_build_error = ""
                    else:
                        self._world_build_error = str(answer.get("error") or "")
            except Exception as error:              # never past here: it is a loop
                self._world_build_error = f"{type(error).__name__}: {error}"

    def _tool_world_building(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Read or set whether the rover builds its world state. A control call.

        Control rather than a model tool for the reason the rest of the world
        state is: a model that could switch off the rover's own record of the room
        could quietly stop it learning, and nobody would see a failure. See
        "Authority boundaries" in docs/task-semantic-world-state.md.
        """
        if "on" in arguments and arguments["on"] is not None:
            self._world_build = bool(arguments["on"])
            if self._world_build:
                # Look now rather than in fifteen seconds: somebody has just
                # pressed a button and an empty panel is what they are watching.
                self._world_build_at = 0.0
                self._world_build_from = None
        return {"ok": True, "building": self.world_building(),
                "looks": getattr(self, "_world_build_looks", 0),
                "every_s": LOOK_EVERY_S, "settle_every_s": SETTLE_EVERY_S,
                # What the last resolver pass did, which is the only place a
                # rover recording steadily and placing nothing would say so.
                "settled": getattr(self, "_world_settled", {}),
                "error": getattr(self, "_world_build_error", "")}

    def _world_ready(self) -> str:
        """Empty if the world state can be used, otherwise why not, in a sentence."""
        if world_state is None:
            return (f"this rover has no world_state component installed: "
                    f"{WORLD_IMPORT_ERROR}")
        try:
            self._world_store()
        except Exception as error:
            return f"the world-state database could not be opened: {error}"
        return ""

    def _world_store(self):
        """The store, opened once and kept.

        Opened lazily rather than at startup: a daemon on a bench with no ~/.ugv
        should still start, and the first thing that wants the world state is a
        console popup that nobody may ever open.
        """
        store = getattr(self, "_world_store_cache", None)
        if store is None:
            store = world_state.WorldStore()
            self._world_store_cache = store
        return store

    def _world_inspector(self):
        inspector = getattr(self, "_world_inspector_cache", None)
        if inspector is None:
            # **The encoders are the whole of an inspection.** A language model
            # used to sit behind this call, describing the room in words; it
            # cost ten seconds against a fifth of one, and its names drifted
            # between "black leather recliner" and "blue leather recliner" on a
            # byte-identical frame, so nothing downstream could read them. It is
            # gone from the rover entirely. A person who wants prose about what
            # the camera can see asks `look`, which puts the frame in front of
            # the conversation's own model.
            eyes = (world_state.FakeEyes() if os.environ.get(ENV_FAKE) == "1"
                    else world_state.SidecarEyes())
            inspector = world_state.Inspector(
                self._world_store(), eyes, self._world_capture,
                self._world_pose, fov_deg=self.camera_fov_deg,
                reach=self._world_reach)
            self._world_inspector_cache = inspector
        return inspector

    # --- what the rover measures ----------------------------------------------

    def _world_capture(self) -> dict[str, Any]:
        """One frame, through the path that already owns the camera.

        This is `camera_jpeg`'s picture, not a second one: if the tracking loop has
        the camera it is the loop's newest frame, and otherwise it is a bounded
        one-shot grab that closes the device again. Nothing here opens the camera
        a second time, which is the whole rule.

        **The second attempt is a backstop now rather than the fix it was.** It
        was put here because a grab that followed another one closely came back
        empty, and the retry a moment later worked; what was actually happening,
        measured properly on 2026-09-03, is that two grabs *overlapping* lose one
        of the two, and the pause simply outlasted the other grab. `_snapshot`
        holds the camera for the length of a grab now, so overlaps cannot happen
        and this should never fire. It stays because it is half a second in front
        of a look that is worth keeping, and because an empty grab from some cause
        nobody has measured yet is better retried than recorded as a failed
        inspection.
        """
        if self.device is None:
            return {"ok": False, "error": "this rover has no camera attached"}
        jpeg, why = self._whole_jpeg()
        if jpeg is None and not self._tracking.is_set():
            time.sleep(CAMERA_RETRY_S)
            jpeg, again = self._whole_jpeg()
            if jpeg is None:
                why = f"{why} (and again {CAMERA_RETRY_S:.1f} s later: {again})"
        if jpeg is None:
            return {"ok": False, "error": why}
        with self._lock:
            pan, tilt = self.pan, self.tilt
        width, height = self.size
        return {"ok": True, "jpeg": jpeg, "pan": round(pan, 1),
                "tilt": round(tilt, 1), "live": self._tracking.is_set(),
                "width": width, "height": height}

    def _world_pose(self) -> dict[str, Any] | None:
        """Where the rover is standing now, as SLAM has it, or None.

        Measured, not inferred -- which is the line this whole experiment draws.
        Where the camera was is a reading the rover already takes; how far away the
        sofa is would be a guess the model made from one photograph, and no amount
        of it goes in the database.

        **Asked of the navigator rather than read off the last map picture, and
        that is a fix rather than a preference.** `nav.slam` is the grid the
        renderer was last handed, and the pose on it is whoever last called
        `map_png` -- a console polling the map, or nobody. With no console open it
        stands still while the rover drives; on a freshly started daemon it is the
        placeholder's `(0, 0, 0)`. The rover recorded both on 2026-09-02: two
        inspections a minute apart, straight after the daemon restarted, put
        twenty-two regions on bearings drawn from the map origin, and those
        crossed real bearings 4.8 m away at a healthy parallax and placed six
        things that were never there.

        **A pose the navigator does not trust is no pose at all.** `position_
        trusted` is slam_toolbox still publishing where the rover is; without it
        the coordinates are dead reckoning wearing a map's clothes, and an
        observation with no pose is a thing this store already handles honestly --
        it keeps the picture and records no bearing.
        """
        navigator = getattr(self, "nav", None)
        if navigator is None:
            return None
        try:
            status = navigator.status()
        except Exception:
            return None
        if not status.get("position_trusted"):
            return None
        where = status.get("pose")
        if not isinstance(where, dict):
            return None
        try:
            return {"x_m": round(float(where["x_m"]), 3),
                    "y_m": round(float(where["y_m"]), 3),
                    "heading_deg": round(float(where["heading_deg"]), 1)}
        except (KeyError, TypeError, ValueError):
            return None

    def _world_grid(self):
        """The occupancy grid, decoded, or None. Cached for `MAP_CACHE_S`.

        Fetched from the navigator rather than read off `nav.slam`, for the
        reason `_world_pose` no longer reads that either: `nav.slam` is whatever
        the map renderer was last handed, and a world state that only knew about
        walls while somebody had the console open would be worse than one that
        knew about none.
        """
        now = time.monotonic()
        held = getattr(self, "_world_grid_cache", None)
        if held is not None and now - held[0] < MAP_CACHE_S:
            return held[1]
        self._world_grid_cache = (now, None)
        navigator = getattr(self, "nav", None)
        if navigator is None:
            return None
        try:
            import zlib

            import numpy as np

            answer = navigator.ask({"op": "map"}, MAP_ASK_S)
            if not answer.get("ok") or not answer.get("data"):
                return None
            width, height = int(answer["width"]), int(answer["height"])
            cells = np.frombuffer(
                zlib.decompress(base64.b64decode(answer["data"])), dtype=np.int8)
            if cells.size != width * height:
                return None
            grid = (width, height, float(answer["resolution_m"]),
                    float(answer["origin_x_m"]), float(answer["origin_y_m"]),
                    cells.reshape(height, width))
        except Exception:
            # No map is a state the resolver handles: every bearing is unbounded,
            # which is what it did before it could ask.
            return None
        self._world_grid_cache = (now, grid)
        return grid

    def _world_reach(self, x_m: float, y_m: float,
                     bearing_deg: float) -> float | None:
        """How far the rover could see from there in that direction, in metres.

        **This is the strongest thing the world state has, and the rover had
        already measured it.** A bearing carries no range, so two bearings cross
        somewhere whatever they are pointed at -- and on 2026-09-02 the rover
        placed two things outside the edge of its own map, and one of them nine
        metres past a wall 55 cm in front of it, from bearings that were each
        individually correct. The occupancy grid answers the question that
        settles it: you cannot see a thing through a wall, so the first obstacle
        along a bearing is the furthest that sighting can possibly be.

        None means the map cannot say, and None leaves the bearing unbounded
        rather than refused. What the resolver does with the number is
        `locate.beyond_reach`, and the margin it allows lives there, because a
        thing standing against a wall *is* the obstacle.
        """
        grid = self._world_grid()
        if grid is None:
            return None
        width, height, resolution, origin_x, origin_y, cells = grid
        step = resolution / 2.0
        dx = math.cos(math.radians(bearing_deg)) * step
        dy = math.sin(math.radians(bearing_deg)) * step
        reached = 0.0
        for count in range(1, int(REACH_LIMIT_M / step) + 1):
            at_x = float(x_m) + dx * count
            at_y = float(y_m) + dy * count
            # **Floor, not truncation.** `int()` rounds toward zero, so a point
            # five centimetres the wrong side of the origin lands in cell 0 and a
            # walk off that edge of the map never notices it left.
            ix = math.floor((at_x - origin_x) / resolution)
            iy = math.floor((at_y - origin_y) / resolution)
            if not (0 <= ix < width and 0 <= iy < height):
                # Off the edge of what has been mapped, which bounds a sighting
                # as firmly as a wall does: the rover has never seen anything out
                # there, so it cannot have been looking at a thing out there.
                break
            if cells[iy, ix] >= OCCUPIED_AT:
                break
            reached = step * count
        # The last sample that was clear, so the answer is short by up to half a
        # cell rather than long by it. Erring short means erring toward refusing
        # a placement, and `locate.SEE_PAST_M` is ten times the error either way.
        return reached

    # --- the control calls ----------------------------------------------------

    def _tool_world_state_summary(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """Counts, the last inference and the last few outcomes. A control call.

        The recent outcomes are the point rather than a decoration: a popup that
        showed nothing new after an inspection would otherwise leave "the model
        failed" and "the model found nothing" looking identical, and those are
        fixed in completely different places.
        """
        why = self._world_ready()
        if why:
            return {"ok": False, "error": why}
        store = self._world_store()
        inspector = self._world_inspector()
        return {"ok": True, "summary": store.summary(),
                "inferences": store.inferences(),
                # What would answer an inspection, named so the popup can tell a
                # rover that is really measuring from one that has the fake
                # standing in.
                "backend": world_state.describe_eyes(inspector.eyes),
                "busy": inspector.busy,
                "building": self.world_building(),
                "built_looks": getattr(self, "_world_build_looks", 0),
                "building_error": getattr(self, "_world_build_error", ""),
                "settled": getattr(self, "_world_settled", {}),
                "camera_fov_deg": self.camera_fov_deg,
                "pose": self._world_pose()}

    def _tool_world_state_search(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Find me the thing I described. A control call.

        The phrase goes through SigLIP2's text tower, whose image tower produced
        every stored region vector, so it lands in the same space and the
        comparison is a dot product over a few hundred of them. It is also the
        only thing that turns what the rover saw into words -- nothing names a
        region any more -- so this is how a person finds anything by describing
        it. Which is why the answer arrives
        in milliseconds once the query itself has been embedded -- that part goes
        to the sidecar and, on the GPU, loads the text engine for the call.
        """
        why = self._world_ready()
        if why:
            return {"ok": False, "error": why}
        query = str(arguments.get("query") or "").strip()
        if not query:
            return {"ok": False, "error": "nothing to look for"}
        if len(query) > 200:
            return {"ok": False, "error": "that is a paragraph, not a description"}
        eyes = self._world_inspector().eyes
        if eyes is None:
            return {"ok": False,
                    "error": "this host has no perception sidecar to embed a "
                             "query with"}
        vectors, error = eyes.embed([query.lower()])
        if error or not vectors:
            return {"ok": False, "error": error or "the sidecar sent no vector"}
        store = self._world_store()
        rows = store.searchable(map_session=arguments.get("map_session"))
        answer = world_state.search.rank(
            vectors[0], rows,
            limit=max(1, min(int(arguments.get("limit") or 10), 50)),
            backend=str(arguments.get("backend") or ""))
        # Where a match is, when it belongs to something the rover has placed.
        placements = {one["id"]: one.get("placement")
                      for one in store.entities()}
        for match in answer.get("matches", []):
            match["placement"] = placements.get(match.get("entity_id"))
        return {"ok": True, "query": query, **answer}

    def _tool_world_state_entities(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """Every entity, with the rays its recent observations support."""
        why = self._world_ready()
        if why:
            return {"ok": False, "error": why}
        store = self._world_store()
        entities = store.entities()
        for entity in entities:
            observations = store.observations(entity["id"], limit=RAY_LIMIT)
            # With the placement, each ray also carries how it stands to it --
            # the range, how far off the bearing is and whether that is inside
            # what the resolver allows. The map draws a sighting rather than an
            # arrow of arbitrary length, and the numbers are the resolver's own.
            entity["rays"] = world_view.rays(observations, self.camera_fov_deg,
                                             limit=RAY_LIMIT,
                                             placement=entity.get("placement"))
        return {"ok": True, "entities": entities,
                # Everything the model has ever said, newest first, and the
                # observations no entity was made for. Both are here so the popup
                # can show repeated creation of the same thing under new
                # identifiers, which is the failure this slice exists to measure
                # and which an entity list alone makes look like a busy room.
                "recent": store.observations(limit=DETAIL_LIMIT),
                "unmatched": store.observations(unmatched=True, limit=DETAIL_LIMIT),
                "summary": store.summary()}

    def _tool_world_state_entity(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """One entity, its whole recent history, and the raw answer behind each."""
        why = self._world_ready()
        if why:
            return {"ok": False, "error": why}
        entity_id = str(arguments.get("id") or "")
        store = self._world_store()
        entity = store.entity(entity_id)
        if entity is None:
            return {"ok": False, "error": f"no such entity: {entity_id}"}
        observations = store.observations(entity_id, limit=DETAIL_LIMIT)
        return {"ok": True, "entity": entity, "observations": observations,
                "rays": world_view.rays(observations, self.camera_fov_deg,
                                        limit=SELECTED_RAY_LIMIT,
                                        placement=entity.get("placement"))}

    def _tool_world_state_observations(self,
                                       arguments: dict[str, Any]) -> dict[str, Any]:
        why = self._world_ready()
        if why:
            return {"ok": False, "error": why}
        entity_id = arguments.get("entity_id")
        store = self._world_store()
        return {"ok": True, "observations": store.observations(
            None if entity_id is None else str(entity_id),
            limit=int(arguments.get("limit") or DETAIL_LIMIT),
            unmatched=bool(arguments.get("unmatched")))}

    def _tool_world_state_frame(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """The stored JPEG an observation was read from, as base64.

        The same argument `camera_jpeg` makes for existing beside `look`: a browser
        on a desk wants the bytes in the reply, and routing them through a frame
        server to reach the machine that asked would be silly.
        """
        why = self._world_ready()
        if why:
            return {"ok": False, "error": why}
        frame_id = str(arguments.get("frame_id") or "")
        jpeg = self._world_store().frame(frame_id)
        if jpeg is None:
            return {"ok": False, "error": f"no stored frame {frame_id}"}
        return {"ok": True, "frame_id": frame_id, "bytes": len(jpeg),
                "jpeg_base64": base64.b64encode(jpeg).decode("ascii")}

    def _tool_world_state_clear(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """Empty the semantic world. The SLAM map is not touched.

        Refused while an inspection is running, rather than racing it: the
        inspection is holding a frame identifier and an inference row that this
        would delete underneath it, and the answer to "clear during an inspection"
        should be a sentence rather than a half-cleared database.
        """
        why = self._world_ready()
        if why:
            return {"ok": False, "error": why}
        if self._world_inspector().busy:
            return {"ok": False,
                    "error": "an inspection is running; nothing was cleared"}
        return self._world_store().clear()

    def _tool_world_map_session(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """The SLAM map was cleared, so start a new map session. A control call.

        The console owns the button that clears the map, so it tells the store
        rather than the store polling for it. Nothing is deleted here: entities and
        their history are meant to outlive a map, and only the stamp on new
        observations moves.
        """
        why = self._world_ready()
        if why:
            return {"ok": False, "error": why}
        return {"ok": True, "map_session": self._world_store().new_map_session()}

    def _tool_world_inspect(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Take a picture, measure what is in it, and record that.

        Half a second on this board -- 0.29 s of camera and 0.16 s of encoders --
        and still on a connection of its own, because deciding identity from the
        pool afterwards is what is slow and the button does both.

        `settle` is how the rover's own looking loop asks for the cheap half
        alone; the console's button leaves it out and gets an answer that says
        what was matched and placed, which is what somebody who has just pressed
        it is watching for.

        It runs on the caller's own thread, so the daemon goes on answering STOP,
        status and the map throughout, and a failure of any kind leaves the world
        state exactly as it was.
        """
        why = self._world_ready()
        if why:
            return {"ok": False, "error": why}
        began = time.monotonic()
        settle = arguments.get("settle")
        result = self._world_inspector().inspect(
            settle=True if settle is None else bool(settle))
        result["took_s"] = round(time.monotonic() - began, 2)
        return result

    def close_world(self) -> None:
        # The loop first, and joined, so that nothing is part-way through an
        # inspection when the store underneath it closes.
        self._world_build = False
        stop = getattr(self, "_world_build_stop", None)
        if stop is not None:
            stop.set()
        thread = getattr(self, "_world_build_thread", None)
        if thread is not None:
            thread.join(timeout=10.0)
            self._world_build_thread = None
        store = getattr(self, "_world_store_cache", None)
        if store is not None:
            store.close()
            self._world_store_cache = None
