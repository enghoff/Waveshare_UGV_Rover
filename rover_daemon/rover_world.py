"""Semantic world-state control, capture and background inspection for the daemon.

The daemon owns the camera; the perception sidecar owns model inference. Control
calls here are distinct from the voice tool schemas. Map-session identity,
measured pose and camera provenance accompany every stored observation.
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

# Bounded pages and ray lists keep control replies small enough for Wi-Fi.
DETAIL_LIMIT = 40
PAGE_MAX = 200
RAY_LIMIT = 6
SELECTED_RAY_LIMIT = 24
SIGHT_LIMIT = 60
CAMERA_RETRY_S = 0.5
ENV_FAKE = "UGV_WORLD_FAKE"
# The selected camera is fixed for a deployment; observations record its identity.
# Cross-camera ranges require a measured OAK mount (world_state.oak.MEASURED).
WORLD_CAMERA = "gimbal"
# Capture and resolution have separate schedules; a pass schedules from completion.
LOOK_EVERY_S = 1.0
SETTLE_EVERY_S = 10.0
FOLLOW_MAP_S = 5.0
CLEAR_WAIT_S = 20.0
# A new look need not itself provide the 0.4 m triangulation baseline.
MOVED_ENOUGH_M = 0.15
TURNED_ENOUGH_DEG = 25.0
LOOK_ANYWAY_S = 300.0
LOOK_BLIND_S = 5.0
# Cache one map per short interval rather than refetch it for every pending ray.
MAP_CACHE_S = 5.0
REACH_LIMIT_M = 12.0
OCCUPIED_AT = 50
MAP_ASK_S = 8.0


class RoverWorld:
    """Semantic world state on the daemon's existing control protocol."""

    # --- building it without being asked --------------------------------------

    def _world_kept(self) -> str:
        """The line to log at startup: what the world state came up holding.

        **It used to say what a reboot had thrown away, because it threw the
        whole thing away.** The reason was the map: nothing saved the SLAM map,
        so every boot began an empty one, and every position and every bearing in
        the store had been measured in a map that no longer existed. ros_nav
        keeps its pose graph between sessions now -- see `nav_map.py` -- so a
        rover switched on in the room it was switched off in comes back to
        coordinates that still mean what they meant.

        Which map they belong to is not settled here and cannot be: the daemon
        starts the looking loop before it attaches the navigator, and the
        navigation stack takes half a minute to come up. `_world_follow_map` does
        it a few seconds later, when there is something to ask.

        Answers with the line to log, or with nothing when there is nothing worth
        saying, and never raises: a world state that could not be counted is
        worth a daemon that starts anyway.
        """
        if self._world_ready():
            return ""
        try:
            counts = self._world_store().summary()
        except Exception as error:
            return (f"[rover] world state kept, but it could not be counted: "
                    f"{type(error).__name__}: {error}")
        if not counts["observations"] and not counts["entities"]:
            return ""
        return (f"[rover] world state kept from an earlier session: "
                f"{counts['entities']} entities, {counts['observations']} "
                f"observations, map session {counts['map_session']}")

    def _world_follow_map(self) -> None:
        """Tell the store which SLAM map the rover is on, once the stack says.

        The store stamps every observation with a map session, and a session
        exists to answer one question: are these coordinates comparable with the
        map on screen? So it has to move when the map underneath it changes --
        which is now a rarer event than a reboot. The navigation stack carries an
        identity for the graph it is holding: it survives a restore of the same
        graph across a boot or a deploy, and it changes when the map is cleared,
        when a saved one could not be read, and when there was none to read.

        **No answer is not an answer.** The stack takes half a minute to come up
        and may not be running at all, and both look like `None` here -- neither
        is a reason to touch anything, because the alternative is a world state
        that starts a new session every time somebody restarts navigation.

        Never raises, and asks at most every `FOLLOW_MAP_S`: it runs on the
        looking loop, which has a look due every second.
        """
        now = time.monotonic()
        if now - getattr(self, "_world_map_at", 0.0) < FOLLOW_MAP_S:
            return
        self._world_map_at = now
        navigator = getattr(self, "nav", None)
        if navigator is None or self._world_ready():
            return
        try:
            told = navigator.status().get("map_id")
        except Exception:
            return
        if not told or told == getattr(self, "_world_map_id", None):
            return
        try:
            answer = self._world_store().follow_map(str(told))
        except Exception as error:
            self._world_map_note = (f"the world state could not be moved onto "
                                    f"this map: {type(error).__name__}: {error}")
            return
        self._world_map_id = str(told)
        if answer.get("changed"):
            self._world_map_note = (
                f"{answer['reason']}, so what was recorded before -- "
                f"{answer.get('entities', 0)} entities, "
                f"{answer.get('observations', 0)} observations -- is kept but no "
                f"longer drawn on the map; new looks go on map session "
                f"{answer['map_session']}")
        else:
            self._world_map_note = answer.get("reason", "")

    def start_world_building(self) -> str:
        """Look around on a schedule, from the moment the daemon starts.

        **Always, with nothing to switch it off**, because a world state that only
        records when somebody presses a button is a world state that is empty
        whenever it is wanted. The rover drives across a building and learns
        nothing on the way unless something asks it to look, and nothing did --
        and a rover that had been switched off looked exactly like one recording
        steadily until somebody thought to check.

        Its own thread, and it only ever makes the same call an inspection from
        the console makes. Nothing here reaches into the store or the resolver
        directly, so a fault in this loop cannot corrupt anything -- at worst it
        stops looking.

        Answers with the one line the caller should log, which is what the
        store came up holding and is usually nothing at all.
        """
        note = self._world_kept()
        self._world_build_stop = threading.Event()
        self._world_build_at = 0.0
        self._world_settle_at = 0.0
        self._world_settled: dict[str, Any] = {}
        self._world_build_from = None
        self._world_build_looks = 0
        self._world_build_error = ""
        #: The map the store has been told about, and what happened when it was.
        #: Both start empty: which map this is takes a running navigation stack
        #: to answer, and this runs before the daemon has attached one.
        self._world_map_id = None
        self._world_map_at = 0.0
        self._world_map_note = ""
        thread = threading.Thread(target=self._world_building_loop,
                                  name="world-building", daemon=True)
        self._world_build_thread = thread
        thread.start()
        return note

    def world_building(self) -> bool:
        """Whether the looking loop is running, which it is until shutdown.

        Reported and not controlled. The only answer other than yes is that the
        thread has gone, and that is a fault the panel should say out loud rather
        than a state anything can ask for.
        """
        thread = getattr(self, "_world_build_thread", None)
        return thread is not None and thread.is_alive()

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
        """Where the camera is looking: the chassis, less the camera's own pan.

        The gimbal takes pan positive to the right and the map takes bearings
        positive to the left, which is the same conversion `view.ray` makes when
        it turns a look into a bearing. Using the chassis alone would mean a rover
        that swung its camera across the whole room counted as having seen
        nothing new.

        **Whose pan depends on which camera is looking.** The OAK is bolted to
        the chassis, so turning the gimbal changes nothing it can see and only
        the rover turning does -- and reading the gimbal's pan here would have a
        parked rover think it had found a new direction every time the tracking
        loop moved a servo.
        """
        if WORLD_CAMERA == world_state.oak.OAK:
            return pose["heading_deg"] - world_state.oak.pan_deg()
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
                now = time.monotonic()
                # Before anything is recorded, because what a look is stamped
                # with is the map session, and that is what this settles.
                self._world_follow_map()
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
                    outcome = self._world_inspector().settle()
                    # **Timed from when it finished, not from when it started.**
                    # A pass that ran longer than `SETTLE_EVERY_S` was instantly
                    # due again the moment it returned, so a rover whose pending
                    # pool had grown large enough would settle for ever and never
                    # look again. Measured on the Orin, a pass over a 500-bearing
                    # pool is tens of seconds against a cadence of ten, and the
                    # pool only stayed small enough to hide it while three
                    # quarters of every look was recording no bearing at all.
                    self._world_settle_at = time.monotonic()
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

    def _tool_world_building(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """How the rover's own looking is going. A control call, and read-only.

        **It used to take an `on`, and there is nothing to set here any more.**
        The reason it was a control call rather than a model tool was that a model
        able to switch off the rover's record of the room could quietly stop it
        learning with nobody seeing a failure -- and that argument applies just as
        well to the person at the console, who had the same switch and no reason
        to want it. See "Authority boundaries" in
        docs/task-semantic-world-state.md.
        """
        return {"ok": True, "building": self.world_building(),
                "looks": getattr(self, "_world_build_looks", 0),
                "every_s": LOOK_EVERY_S, "settle_every_s": SETTLE_EVERY_S,
                # What the last resolver pass did, which is the only place a
                # rover recording steadily and placing nothing would say so.
                "settled": getattr(self, "_world_settled", {}),
                # Which SLAM map the rows in the store belong to, in a sentence.
                # Empty until the navigation stack has said; see
                # `_world_follow_map`.
                "map": getattr(self, "_world_map_note", ""),
                "error": getattr(self, "_world_build_error", "")}

    @property
    def world_installed(self) -> bool:
        """Whether this rover has the world_state component deployed at all.

        The cheap half of `_world_ready`, and the half that decides which tools
        exist: it is a property of the deployment rather than of the moment, so it
        can be asked on every `list_tools` without opening a database. Whether the
        store opens *now* is a fault to be reported in a sentence when somebody
        calls the tool, not a reason to have hidden the tool.
        """
        return world_state is not None

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
            # Which camera takes the picture is `WORLD_CAMERA`, and it is chosen
            # here rather than inside the inspection because the camera has one
            # owner and it is this class. The depth camera goes in either way:
            # through it, every region has a range; through the gimbal, the
            # regions that happen to fall inside its view do. Neither can happen
            # until the mount is measured, which `world_state.oak` refuses
            # silently rather than approximately.
            through_oak = WORLD_CAMERA == world_state.oak.OAK
            inspector = world_state.Inspector(
                self._world_store(), eyes,
                self._world_capture_oak if through_oak else self._world_capture,
                self._world_pose, fov_deg=self.camera_fov_deg,
                reach=self._world_reach, ranger=self._world_ranger())
            self._world_inspector_cache = inspector
        return inspector

    def _world_ranger(self):
        """The depth camera, opened once and kept, or None on a rover with none.

        None rather than a failure at every look: the OAK is a second camera on a
        loopback port, and a rover whose depth service is not running records
        exactly what this component recorded before ranges existed. The client
        itself never raises, so the only thing that lands here is the component
        not being installed at all.
        """
        ranger = getattr(self, "_world_ranger_cache", None)
        if ranger is None:
            try:
                ranger = world_state.depth_client.SidecarRanger()
            except Exception:                       # never past here
                return None
            self._world_ranger_cache = ranger
        return ranger

    # --- what the rover measures ----------------------------------------------

    def _world_capture_oak(self) -> dict[str, Any]:
        """One frame from the depth camera, with the range behind every pixel.

        **The other camera, and the reason it is worth having one.** The gimbal
        camera can look anywhere and cannot say how far away anything is; this one
        cannot look anywhere at all and says how far away everything is, because
        the depth is warped into this very picture's geometry on the device. A box
        found here indexes the ranges directly.

        It opens no device: the depth service already holds the OAK, for the
        reason its own README gives -- a booted Myriad with no host dies in 1500
        ms, so being awake *is* a process holding it -- and this is an HTTP call to
        that process.

        The pan and tilt recorded are the mount's own, which is what makes this
        camera a gimbal that never moves as far as everything downstream is
        concerned. The picture is a little older than this instant, because the
        service holds each depth frame until the colour frame exposed with it has
        arrived; `taken_at` says how much and the bearing arithmetic reads it
        rather than assuming a figure that changes with the camera's rate.
        """
        ranger = self._world_ranger()
        if ranger is None:
            return {"ok": False, "error": "this rover has no depth camera "
                                          "component installed"}
        if not world_state.oak.MEASURED:
            # Refused rather than recorded without a bearing, because unlike a
            # missing pose this is not a passing condition: *every* look through
            # this camera would be bearingless until somebody runs the bench, and
            # a rover quietly filling its store with directionless pictures looks
            # exactly like one that is working.
            return {"ok": False,
                    "error": "the OAK's mount has never been measured, so a "
                             "bearing through it would be a guess -- run "
                             "world_state/bench_oak.py"}
        frame = ranger.frame()
        if not frame.ok:
            return {"ok": False, "error": frame.error}
        return {"ok": True, "jpeg": frame.jpeg, "camera": world_state.oak.OAK,
                "pan": round(world_state.oak.pan_deg(), 1),
                "tilt": round(world_state.oak.tilt_deg(), 1),
                "live": True, "width": frame.width, "height": frame.height,
                "age_s": frame.age_s, "taken_at": frame.taken_at}

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
        # **The one caller that asks when the picture was taken.** A bearing is
        # only as good as the heading at the instant the shutter opened, and
        # `Inspector._where` interpolates to it rather than averaging across the
        # grab -- which is what turns a look taken while turning from no bearing
        # at all into a wide one. `camera_jpeg` and `look` want a picture and
        # nothing else, so they do not ask.
        jpeg, why, taken_at = self._whole_jpeg(when=True)
        if jpeg is None and not self._tracking.is_set():
            time.sleep(CAMERA_RETRY_S)
            jpeg, again, taken_at = self._whole_jpeg(when=True)
            if jpeg is None:
                why = f"{why} (and again {CAMERA_RETRY_S:.1f} s later: {again})"
        if jpeg is None:
            return {"ok": False, "error": why}
        with self._lock:
            pan, tilt = self.pan, self.tilt
        width, height = self.size
        return {"ok": True, "jpeg": jpeg, "camera": world_state.oak.GIMBAL,
                "pan": round(pan, 1),
                "tilt": round(tilt, 1), "live": self._tracking.is_set(),
                "width": width, "height": height, "taken_at": taken_at}

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
        it. The whole of it is milliseconds once the sidecar has the text engine
        open, which it does from the first search after a start-up onwards.
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
        matches = answer.get("matches", [])
        # The whole of every matching look, and not only the handful of columns
        # the ranking itself needed. **The console narrows its observation
        # stream down to these rows**, so each one has to carry what an ordinary
        # row in that stream carries -- the pose, the note, the raw measurement
        # -- or opening a look the filter found would show less than opening the
        # same look with the filter off. The ranking is over the vectors, which
        # are the one thing not sent.
        whole = {row["id"]: row for row in
                 store.observations(ids=[match["observation_id"]
                                         for match in matches],
                                    limit=max(1, len(matches)))}
        # Where a match is, when it belongs to something the rover has placed.
        placements = {one["id"]: one.get("placement")
                      for one in store.entities()}
        for match in matches:
            for key, value in whole.get(match.get("observation_id"), {}).items():
                # The scores win where the two disagree, which they do not: the
                # shared columns are the same columns read twice.
                match.setdefault(key, value)
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
                # The newest looks, whatever they were decided to be, so the
                # popup can show repeated creation of the same thing under new
                # identifiers -- the failure this slice exists to measure, and
                # one an entity list alone makes look like a busy room. Only the
                # newest, because this reply is sent again every time the rover
                # records: the stream walks back through the rest of the history
                # by asking `world_state_observations` for a page at a time, and
                # how many of them have no entity is a count in the summary.
                "recent": store.observations(limit=DETAIL_LIMIT),
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

    def _tool_world_state_viewpoint(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Where the rover would have to stand to look at one thing. A control call.

        **It works out a place to drive to and does not drive there**, which is
        the whole shape of this call. The console owns driving: it holds the
        connection a move blocks for minutes on, the STOP that ends one, and the
        rule that a new destination outranks whatever is running. A world call
        that drove would be a second, worse copy of all three on the connection
        the inspections are queued on. So this answers in a millisecond or two
        with a point and a heading, and `drive_to` takes it from there.

        The point is a place on the map and not an offset, for the reason every
        destination in this daemon is: the answer is worked out from where the
        rover was standing when it was asked, and by the time the wheels turn it
        is standing somewhere else.

        `heading_deg` is the way to be facing on arrival, and it is what makes
        the difference between somewhere the thing *can* be seen from and
        somewhere it *is* seen. `turn_deg` beside it is the same bearing measured
        from where the rover is pointing now: nothing acts on it, and it is here
        because it is what says whether the move about to happen is a drive or
        mostly a turn on the spot -- which is the difference between a rover that
        has not moved yet and one that is not going to.
        """
        why = self._world_ready()
        if why:
            return {"ok": False, "error": why}
        entity_id = str(arguments.get("id") or "")
        store = self._world_store()
        entity = store.entity(entity_id)
        if entity is None:
            return {"ok": False, "error": f"no such entity: {entity_id}"}
        place = entity.get("placement")
        if not place:
            # The ordinary state of everything the rover has seen once from one
            # place, and not a failure: there is nothing to drive at because
            # nothing has crossed a second bearing with the first yet.
            return {"ok": False,
                    "error": f"{entity_id} has no position yet, so there is "
                             f"nowhere to be sent to look at it"}
        session = store.map_session()
        if entity.get("placement_map_session") != session:
            # Its coordinates were measured in a map that no longer exists, so
            # they name a place in this one only by coincidence.
            return {"ok": False,
                    "error": f"{entity_id} was placed under map "
                             f"{entity.get('placement_map_session')} and the "
                             f"rover is now on map {session}, so its position "
                             f"is not a place on this map"}
        pose = self._world_pose()
        if pose is None:
            return {"ok": False,
                    "error": "nothing is publishing the rover's position, so "
                             "there is nowhere to measure a route from"}
        grid = self._world_grid()
        if grid is None:
            return {"ok": False,
                    "error": "the rover has no map yet, so it cannot tell floor "
                             "it could stand on from the middle of a wall"}
        found = world_state.approach.viewpoint(
            place, world_state.approach.Grid(*grid), (pose["x_m"], pose["y_m"]),
            self._world_sight_lines(entity_id, place, session))
        if not found.get("ok"):
            return {"ok": False, "error": found.pop("why"), "id": entity_id,
                    "placement": place, "pose": pose, **found}
        turn = found["heading_deg"] - pose["heading_deg"]
        return {"ok": True, "id": entity_id, "placement": place, "pose": pose,
                "turn_deg": round((turn + 180.0) % 360.0 - 180.0, 1), **found}

    def _world_sight_lines(self, entity_id: str, place: dict[str, Any],
                           session: int) -> list[dict[str, Any]]:
        """The looks a thing was seen in, as the rays `approach` reads directions
        from.

        **Only the looks taken under the map the rover is on now.** Where the
        rover was standing is a position in the map of the day, and a pose
        recorded before the map was cleared names a place in this one by
        coincidence -- the same rule the placement itself is held to a few lines
        above, applied to the looks behind it. Without it a thing placed in this
        map would be approached from a standing point measured in the last one.

        The rays are asked for with the placement, which is what puts
        `relation.agrees` on each of them: `approach.seen_from` leaves out the
        looks the resolver would no longer attach, and that decision is the
        resolver's own rather than a second opinion formed here.
        """
        observations = self._world_store().observations(entity_id,
                                                        limit=SIGHT_LIMIT)
        drawn = world_view.rays(observations, self.camera_fov_deg,
                                limit=SIGHT_LIMIT, placement=place)
        return [one for one in drawn if one.get("map_session") == session]

    def _tool_world_state_observations(self,
                                       arguments: dict[str, Any]) -> dict[str, Any]:
        """One page of the history, newest first, starting below a given row.

        **This is how the console shows more than its newest handful.** What
        rides in the popup's body is capped, because that body is re-sent every
        time the rover records anything; the rest of the history is walked back
        through here, a page at a time, as somebody scrolls. `before_at` and
        `before_id` are the oldest row the caller already holds -- a place in the
        history rather than a count of rows to skip, since the rover goes on
        recording while it is being read.

        `more` is whether the page came back full, which is the only cheap thing
        that can be said about what is under it.
        """
        why = self._world_ready()
        if why:
            return {"ok": False, "error": why}
        entity_id = arguments.get("entity_id")
        limit = max(1, min(int(arguments.get("limit") or DETAIL_LIMIT), PAGE_MAX))
        before_at = arguments.get("before_at")
        before = (None if before_at in (None, "")
                  else (float(before_at), int(arguments.get("before_id") or 0)))
        rows = self._world_store().observations(
            None if entity_id is None else str(entity_id),
            limit=limit, unmatched=bool(arguments.get("unmatched")), before=before)
        return {"ok": True, "observations": rows, "more": len(rows) == limit}

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

        Never run beside an inspection, because the inspection is holding a frame
        identifier and an inference row that this would delete underneath it. It
        waits for the look in flight rather than refusing it, and only a look
        that outlasts `CLEAR_WAIT_S` comes back as a sentence -- see
        `inspector.not_looking`, which has the measurement that decided that.
        """
        why = self._world_ready()
        if why:
            return {"ok": False, "error": why}
        inspector = self._world_inspector()
        with inspector.not_looking(CLEAR_WAIT_S) as idle:
            if not idle:
                return {"ok": False,
                        "error": f"an inspection has been running for longer "
                                 f"than {CLEAR_WAIT_S:.0f} s; nothing was "
                                 f"cleared"}
            cleared = self._world_store().clear()
            # A look is skipped when its picture is the one already recorded, and
            # every one of those recordings has just been deleted -- so the rover
            # must record the room again rather than recognise it. See
            # `inspector.SAME_PICTURE_SHARE`.
            inspector.forget_picture()
        return cleared

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
