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
#: How many of an entity's newest observations become rays on the map.
RAY_LIMIT = 6
#: How long to wait before asking the camera a second time. Long enough for the
#: previous v4l2-ctl to be well out of the way, short enough to be nothing beside
#: the minute of model that follows. See :meth:`RoverWorld._world_capture`.
CAMERA_RETRY_S = 0.5
#: Substitutes the deterministic fake for the real sidecar. For bringing the
#: console up on the rover before the model is installed, and for nothing else --
#: every row it writes is stamped with the backend that wrote it and the popup
#: shows that, so a rover left in this state says so rather than looking like one
#: where Cosmos is working.
ENV_FAKE = "UGV_COSMOS_FAKE"


#: The fastest the rover will ever look around by itself. A look is a fifth of a
#: second of GPU and it is not the cost that sets this -- it is that the resolver
#: reads the whole pending pool on every look, so a rover that records faster than
#: it can place things gets slower at placing them.
LOOK_EVERY_S = 15.0
#: What counts as somewhere new: the same distance the geometry calls a baseline,
#: because a look from here can pair with a look from there and one any closer
#: cannot. **Recording from a place already looked from is not free.** It cannot be
#: triangulated against the looks already there, and it enlarges the pool every
#: later look has to scan.
MOVED_ENOUGH_M = 0.4
#: And what counts as a new direction, for a rover that turned on the spot without
#: going anywhere. The camera is then pointed at a different part of the room,
#: which is worth recording even though no new baseline came of it.
TURNED_ENOUGH_DEG = 25.0
#: A rover that has not moved at all still looks this often, so that a parked
#: rover in a room that changes notices, and so that "nothing is happening" is
#: distinguishable from "it stopped working".
LOOK_ANYWAY_S = 300.0


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
            return False
        moved = math.hypot(pose["x_m"] - before["x_m"],
                           pose["y_m"] - before["y_m"])
        turned = abs((pose["heading_deg"] - before["heading_deg"] + 180.0)
                     % 360.0 - 180.0)
        return moved >= MOVED_ENOUGH_M or turned >= TURNED_ENOUGH_DEG

    def _world_building_loop(self) -> None:
        """Never raises, and never looks while the wheels are turning.

        A look taken mid-drive carries the pose it was given when the shutter
        opened and a picture blurred by the move, and a bearing is only as good as
        the pose behind it -- so the whole point of the measurement is lost. The
        rover is stopped between one `drive` call and the next, which is when this
        gets its chance.
        """
        while not self._world_build_stop.wait(1.0):
            try:
                if not self._world_build:
                    continue
                navigator = getattr(self, "nav", None)
                if navigator is not None and navigator.driving:
                    continue
                now = time.monotonic()
                if not self._world_worth_looking(now):
                    continue
                if self._world_ready():
                    # No component, or no database. Wait out the long gap rather
                    # than asking again every second for the life of the daemon.
                    self._world_build_at = now
                    continue
                self._world_build_at = now
                answer = self._tool_world_inspect({})
                self._world_build_from = self._world_pose()
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
                "every_s": LOOK_EVERY_S,
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
            if os.environ.get(ENV_FAKE) == "1":
                reasoner = world_state.FakeReasoner(model_id="fake (no model)")
                eyes = world_state.FakeEyes()
            else:
                reasoner = world_state.CosmosReasoner()
                eyes = world_state.SidecarEyes()
            # **The encoders are what an inspection uses, not the language
            # model.** A look through them costs a fifth of a second against ten
            # seconds, it comes back with the two vectors and the box that
            # identity will actually be decided from, and the model's own names
            # were measured drifting between "black leather recliner" and "blue
            # leather recliner" on a byte-identical frame. The language model
            # stays for the conversational `look`, where a person is waiting for
            # prose and a slow answer is fine.
            inspector = world_state.Inspector(
                self._world_store(), reasoner, self._world_capture,
                self._world_pose, eyes=eyes, fov_deg=self.camera_fov_deg)
            self._world_inspector_cache = inspector
        return inspector

    # --- what the rover measures ----------------------------------------------

    def _world_capture(self) -> dict[str, Any]:
        """One frame, through the path that already owns the camera.

        This is `camera_jpeg`'s picture, not a second one: if the tracking loop has
        the camera it is the loop's newest frame, and otherwise it is a bounded
        one-shot grab that closes the device again. Nothing here opens the camera
        a second time, which is the whole rule.

        **The second attempt is worth its half second here and nowhere else.**
        Measured on the rover: a grab that follows another one closely comes back
        empty -- v4l2-ctl exits at once, says nothing on stderr, and hands back no
        whole picture -- and the next attempt a moment later works. Three times out
        of three, with a standalone inspection working every time. What makes it
        worth handling rather than reporting is what is about to happen: this
        picture is the front of a minute of model, and throwing that minute away
        over a hiccup that a pause of half a second fixes is a poor trade. A tool
        call that is over in a second, like `camera_jpeg`, is better off saying so
        at once and letting whoever asked press the button again.
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
        """Where the rover was standing, as SLAM has it, or None.

        Measured, not inferred -- which is the line this whole experiment draws.
        Where the camera was is a reading the rover already takes; how far away the
        sofa is would be a guess the model made from one photograph, and no amount
        of it goes in the database.
        """
        navigator = getattr(self, "nav", None)
        if navigator is None:
            return None
        try:
            x, y, heading = navigator.slam.pose
        except Exception:
            return None
        return {"x_m": round(x, 3), "y_m": round(y, 3),
                "heading_deg": round(math.degrees(heading), 1)}

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
                # What would answer an inspection, which is the encoders when
                # there are any. The language model is named separately because
                # it is still what the conversational `look` uses, and a popup
                # that showed only one of the two would be describing the wrong
                # thing half the time.
                "backend": (world_state.describe_eyes(inspector.eyes)
                            if inspector.eyes is not None
                            else world_state.describe_backend(inspector.reasoner)),
                "language_model": world_state.describe_backend(inspector.reasoner),
                "busy": inspector.busy,
                "building": self.world_building(),
                "built_looks": getattr(self, "_world_build_looks", 0),
                "building_error": getattr(self, "_world_build_error", ""),
                "camera_fov_deg": self.camera_fov_deg,
                "pose": self._world_pose()}

    def _tool_world_state_search(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Find me the thing I described. A control call.

        The phrase is embedded by the same text tower that named every region,
        so it lands in the same space as the stored vectors and the comparison is
        a dot product over a few hundred of them. Which is why the answer arrives
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
            entity["rays"] = world_view.rays(observations, self.camera_fov_deg,
                                             limit=RAY_LIMIT)
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
                                        limit=RAY_LIMIT)}

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

    def _tool_world_inspect(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """Take a picture, ask the model about it, and record what it said.

        Slow -- tens of seconds on this board -- and therefore deliberately not on
        the connection anything else uses. It runs on the caller's own thread, so
        the daemon goes on answering STOP, status and the map throughout, and a
        failure of any kind leaves the world state exactly as it was.
        """
        why = self._world_ready()
        if why:
            return {"ok": False, "error": why}
        began = time.monotonic()
        result = self._world_inspector().inspect()
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
