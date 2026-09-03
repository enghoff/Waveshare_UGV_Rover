"""One inspection, end to end: take a picture, measure what is in it, keep that.

A look goes through the encoders and nothing else. There was a language model
here once, asked in words what it could see; it cost sixty seconds against a
fifth of one, it named the same chair three different things on one frame, and
what identity is decided from turned out to be a box and two vectors rather than
a sentence. `world_state/README.md` keeps the measurements.

What is stored is what was measured. Which lasting thing each region is does not
come from here: an inspection ends with observations carrying the gimbal angles
and the rover pose behind them, and identity is settled afterwards from those.

The order of the steps here is most of the failure behaviour. Nothing touches the
camera until the sidecar has said it is there, nothing touches the database until
an answer has arrived, and every path -- including the ones that never reach the
encoders -- writes exactly one line to the diagnostics log, so a popup showing
nothing new can always say whether that was a sidecar that failed or a look that
found nothing.

No failure in here is allowed to reach the caller as an exception. The caller is
the process that owns STOP.
"""
from __future__ import annotations

import math
import sqlite3
import threading
import time
from typing import Any, Callable

from .perception_client import describe_eyes

#: A picture older than this is not what the camera is looking at now. Only ever
#: reached through the tracking loop's frame, which is the one path here that hands
#: back something it took for its own reasons rather than for this call.
FRAME_MAX_AGE_S = 5.0

#: How far the rover may travel while the shutter is open before the look is not
#: worth a bearing at all, in metres. **This exists because taking the picture is
#: not instant**: the pose is read on both sides of the capture and the midpoint
#: used, which leaves the origin of the ray wrong by half of whatever the rover
#: travelled.
#:
#: **It stood at 0.12 and that number threw away three quarters of a driven run.**
#: The figure was derived from a 0.29 s capture at the 0.35 m/s the rover explores
#: at, which is 0.10 m; on the run of 2026-09-03 neither half of that held. A
#: bounded grab measured through the daemon is 0.36 s now, and the travel recorded
#: on the looks that lost their bearing puts the explore speed at 0.47 m/s -- so an
#: ordinary look taken while driving straight covers 0.17 m and was refused by a
#: hair. Of the 214 looks that run took, 163 recorded no bearing, and 63 of those
#: had turned less than three degrees: they were straight-line drives whose
#: bearings were fine. That left 94 usable bearings out of 866 regions, from eight
#: standing places, and one entity out of a thirteen-minute drive.
#:
#: 0.30 m is twice an ordinary look's travel, so an unusually fast stretch still
#: keeps its bearing, and it refuses the genuine outliers -- the worst on that run
#: was 0.65 m. **What it no longer does is hide the residual.** Half of whatever
#: was travelled is written on the observation as `origin_sigma_m` and charged to
#: the answer by `locate.fix`, so a look taken on the move places a thing less
#: precisely instead of not at all.
MOVED_WHILE_LOOKING_M = 0.30
#: And how far it may turn, in degrees. Rotation is the term that actually hurts:
#: it swings the whole bearing rather than shifting its origin. The midpoint
#: halves it, so 3.0 leaves 1.5 -- exactly `locate.BEARING_SIGMA_DEG`, which is
#: the error the geometry is already told to expect from the gimbal.
#:
#: **Unchanged while the limit above was loosened, and the asymmetry is the
#: point.** Travel moves where a ray starts, which shifts the crossing by about as
#: much and is reported as uncertainty; a turn swings where the ray points, and a
#: bearing wrong by ten degrees crosses another one somewhere there is nothing at
#: all. That is how a phantom is made, so this one still refuses the look. It cost
#: 100 of the 163 bearings lost on the run of 2026-09-03, and those are the ones
#: worth losing.
TURNED_WHILE_LOOKING_DEG = 3.0


class Inspector:
    """The one place an inspection happens, and the lock that makes it one.

    `capture` and `pose` are supplied rather than reached for, because the things
    they read belong to the daemon: the camera has one owner and the rover's place
    on the map is the navigator's to report. This class is given two functions and
    knows nothing about either.
    """

    def __init__(self, store, eyes, capture: Callable[[], dict[str, Any]],
                 pose: Callable[[], dict[str, Any] | None] | None = None,
                 fov_deg: float | None = None,
                 source: str = "perception",
                 reach: Callable[[float, float, float], float | None] | None = None
                 ) -> None:
        self.store = store
        #: The perception sidecar, and the only thing an inspection asks. It is
        #: required rather than optional: an inspector with nothing to look
        #: through can do nothing, and a constructor that accepted one would push
        #: that discovery to the first look instead of to the caller.
        self.eyes = eyes
        self.capture = capture
        self.pose = pose
        #: How far the rover could see from a place in a direction, out of the
        #: occupancy grid. Supplied rather than reached for, like the camera and
        #: the pose, because the map belongs to the navigator -- and optional,
        #: because a rover with no map still measures perfectly good regions and
        #: should still record them. What it buys when it is there is in
        #: `locate.beyond_reach`, and it is the strongest gate the resolver has.
        self.reach = reach
        #: The camera's horizontal field of view, which the daemon owns. Without
        #: it a box cannot become an angle, so the bearing columns stay null and
        #: say so rather than being filled in from a guess.
        self.fov_deg = fov_deg
        self.source = source
        self._lock = threading.Lock()
        self.started_at = 0.0

    @property
    def busy(self) -> bool:
        return self._lock.locked()

    def inspect(self, settle: bool = True) -> dict[str, Any]:
        """Look once, and answer with what happened rather than with what was found.

        A second request while one is running is refused rather than queued. An
        inspection holds the camera and the sidecar's engines, and two at once on
        this board means both are slower than one and neither is looking at the
        picture anybody asked for.

        **`settle=False` records the look and does not decide any identities**,
        and it exists because the two halves cost wildly different amounts.
        Measured on the rover: taking a look is 0.29 s of camera and 0.16 s of
        GPU, near enough constant, while one resolver pass over the pending pool
        is 1.4 s at 500 bearings and 8 s at 2000 -- it compares every pair, so it
        grows as the square of what the rover has seen. Settling after every look
        therefore makes a rover that looks often slower and slower at looking,
        until the looking stops. Whoever is driving the looks decides how often it
        is worth asking, and `settle` is that call.
        """
        if not self._lock.acquire(blocking=False):
            return {"ok": False, "status": "busy", "busy": True,
                    "error": f"an inspection has been running for "
                             f"{time.monotonic() - self.started_at:.0f} s; "
                             f"this one was not started"}
        self.started_at = time.monotonic()
        try:
            return self._inspect(settle=settle)
        except Exception as error:            # never past here: the daemon owns STOP
            return self._failed("error", f"{type(error).__name__}: {error}")
        finally:
            self._lock.release()

    def settle(self) -> dict[str, Any]:
        """Decide identities from everything now pending, and never raise.

        Separate from `inspect` so that a rover looking once a second can settle
        once every so often instead, and so that the console's button can settle
        on its own without holding the camera. It takes the same lock, because a
        pass that ran while a look was being recorded would read half of it.
        """
        if not self._lock.acquire(blocking=False):
            return {"ok": False, "error": "an inspection is running"}
        self.started_at = time.monotonic()
        try:
            return {"ok": True, **self._settle()}
        finally:
            self._lock.release()

    # --- the steps ------------------------------------------------------------

    def _inspect(self, settle: bool = True) -> dict[str, Any]:
        """One inspection: a frame, the encoders, and what they measured.

        The failure discipline is the whole of the order here -- the sidecar is
        asked before the camera is touched, nothing is written until an answer
        arrives, and every path writes exactly one diagnostics row. What is kept
        is a box per region, two vectors per box, and a bearing worked out from
        the pose and the gimbal angle behind them.

        There is no scene sentence and no prompt version, because nothing here
        was prompted. Those columns stay empty rather than being filled with a
        plausible-looking substitute.
        """
        began = time.time()
        backend = describe_eyes(self.eyes)

        ready, why = self.eyes.available()
        if not ready:
            return self._failed("unavailable", why, began=began, backend=backend)

        # **Read before the shutter as well as after it.** A bounded grab is
        # 0.36 s on the Orin and the rover may be driving through all of it, so
        # one reading taken afterwards is the pose the rover had arrived at
        # rather than the pose the picture was taken from -- and a bearing is
        # only as good as the pose behind it. Two readings bracket the picture,
        # and how far apart they are is a measurement of how much this
        # particular look can be trusted.
        #
        # **The second reading is taken the moment the picture is in hand and
        # before it is written down**, so that the bracket measures the shutter
        # and nothing else. Saving the frame is 1.4 ms of disk and one row,
        # measured, so this buys almost nothing back -- but a bracket that
        # includes work done after the shutter is measuring the wrong thing, and
        # what it measures is charged to every bearing in the look.
        before = self._pose()
        frame = self._frame()
        after = self._pose()
        if not frame.get("ok"):
            return self._failed("no_frame", str(frame.get("error", "no picture")),
                                began=began, backend=backend)

        jpeg = frame["jpeg"]
        frame_id = self.store.save_frame(jpeg, frame.get("width"),
                                         frame.get("height"))
        where, moved, turned = self._where(before, after)
        capture = {"frame_id": frame_id,
                   "frame_path": self.store.frame_path(frame_id),
                   "pan": frame.get("pan"), "tilt": frame.get("tilt"),
                   "pose": where,
                   # Half of what the rover covered while the shutter was open,
                   # which is how far out the ray's own starting point may be.
                   # Kept with the observation rather than folded into the pose,
                   # because the pose is a measurement and this is how good it
                   # is; `locate.fix` charges it to the answer.
                   "origin_sigma_m": None if where is None else round(moved / 2.0, 3)}
        inference_id = self.store.record_inference(
            started_at=began, status="running", backend=backend,
            frame_id=frame_id, frame_live=1 if frame.get("live") else 0,
            map_session=self.store.map_session())

        look = self.eyes.look(jpeg)
        if not look.ok:
            return self._failed("model_error", look.error, began=began,
                                backend=backend, inference_id=inference_id,
                                frame_id=frame_id, duration_s=look.duration_s,
                                model_id=look.backend)

        try:
            stored = self.store.record(
                look.regions, capture=capture, source=self.source,
                model_id=look.backend, inference_id=inference_id,
                fov_deg=self.fov_deg, region_source="yoloe",
                vectors_from=look.backend)
        except sqlite3.Error as error:
            return self._failed("store_error", f"{type(error).__name__}: {error}",
                                began=began, backend=backend,
                                inference_id=inference_id, frame_id=frame_id,
                                duration_s=look.duration_s, model_id=look.backend)

        # Identity is settled here rather than in `record`, and after the write
        # rather than during it. What was measured is history the moment it is
        # stored; which lasting thing it belongs to is an opinion formed from
        # that history and from everything already in it, and a failure to form
        # one must leave the measurement untouched.
        settled = self._settle() if settle else {}
        detail = self._measured_detail(look, stored, settled, moved, turned)
        self.store.update_inference(
            inference_id, duration_s=round(time.time() - began, 2), status="ok",
            detail=detail or None, model_id=look.backend,
            returned=look.kept, stored=stored["stored"],
            matched=settled.get("matched", 0), created=settled.get("created", 0),
            rejected=max(0, look.kept - stored["stored"]),
            raw_json=None)
        return {"ok": True, "status": "ok", "inference_id": inference_id,
                "frame_id": frame_id, "scene": "",
                "duration_s": round(time.time() - began, 2),
                "returned": look.kept, "stored": stored["stored"],
                "placed": stored["placed"],
                "matched": settled.get("matched", 0),
                "created": settled.get("created", 0),
                "ambiguous": settled.get("ambiguous", 0),
                "still_waiting": settled.get("still_waiting", 0),
                "settled": bool(settled),
                "moved_m": moved, "turned_deg": turned,
                "pose": where,
                "rejected": max(0, look.kept - stored["stored"]),
                "entities": [], "detail": detail,
                "decisions": settled.get("decisions", []),
                "map_session": stored["map_session"], "model_id": look.backend,
                "found": look.found, "timings": look.timings,
                "look_s": look.took_s}

    def _settle(self) -> dict[str, Any]:
        """Run the resolver over the pending pool, and never let it break a look.

        A resolver that raises must not turn an inspection that measured twelve
        regions perfectly well into a failure: the measurements are already
        stored and are the thing of value, and identity can be settled again on
        the next look.
        """
        from . import resolve as resolver

        try:
            return resolver.resolve(self.store, reach=self.reach)
        except Exception as error:                 # never past here
            return {"error": f"{type(error).__name__}: {error}"}

    def _measured_detail(self, look, stored, settled, moved=0.0,
                         turned=0.0) -> str:
        """One sentence a person can act on, in the popup's own column.

        The numbers that matter are how many regions were kept, how many got a
        bearing, and what the resolver then did with the pool -- because an
        inspection that stored twelve observations and settled none of them is a
        rover that has not yet driven far enough, which is a different thing
        from one that is broken.
        """
        if getattr(look, "dark", False):
            # Said first and on its own, because it is the one outcome that is
            # about the camera rather than about the room, and the two read
            # identically otherwise.
            return ("the frame was too dark to see anything in -- nothing was "
                    "measured, and this is not an empty room")
        parts = [f"{stored['stored']} of {look.found} regions kept"]
        if getattr(look, "blank", 0):
            parts.append(f"{look.blank} of them with no picture in it "
                         f"(a blown-out window, a bare wall)")
        if stored["placed"] < stored["stored"]:
            missing = stored["stored"] - stored["placed"]
            if not (moved or turned):
                why = "no pose, no gimbal angle, or no field of view"
            elif turned > TURNED_WHILE_LOOKING_DEG:
                why = (f"the rover turned {turned:.1f} deg while the shutter was "
                       f"open, which swings the bearing")
            else:
                why = (f"the rover moved {moved:.2f} m while the shutter was open")
            parts.append(f"{missing} without a bearing ({why})")
        if not settled:
            parts.append("identity not settled yet")
            return "; ".join(parts)
        if settled.get("error"):
            parts.append(f"identity was not settled: {settled['error']}")
            return ", ".join(parts)
        settled_parts = []
        if settled.get("matched"):
            settled_parts.append(f"{settled['matched']} matched")
        if settled.get("created"):
            settled_parts.append(f"{settled['created']} placed")
        if settled.get("ambiguous"):
            settled_parts.append(f"{settled['ambiguous']} ambiguous")
        waiting = settled.get("still_waiting", 0)
        if waiting:
            settled_parts.append(f"{waiting} waiting for a look from elsewhere")
        parts.append(", ".join(settled_parts) if settled_parts
                     else "nothing to settle")
        return "; ".join(parts)

    def _where(self, before, after):
        """Where the picture was taken from, and how much the rover moved for it.

        The midpoint of the two readings, because the shutter opened somewhere
        between them and the middle is the best a caller with two samples can do.
        `None` where either reading is missing, or where the rover covered more
        ground than `MOVED_WHILE_LOOKING_M` or `TURNED_WHILE_LOOKING_DEG` allow --
        and `None` here is not a failure. The picture, the regions and the vectors
        are all kept; what is dropped is the one thing that was not measured well
        enough to keep, which is the direction.

        **How much movement is too much is a different question for the two
        kinds, and treating them alike is what starved the run of 2026-09-03 of
        bearings.** Travelling shifts where the ray starts, so the midpoint leaves
        a residual the caller can measure and `locate.fix` can charge to the
        answer; turning swings where the ray points, and there is nothing to
        charge that to but a crossing in the wrong place. So the caller keeps the
        travel as `origin_sigma_m` and only a turn still costs the look its
        bearing.
        """
        if not isinstance(before, dict) or not isinstance(after, dict):
            return (after if isinstance(after, dict) else before), 0.0, 0.0
        moved = math.hypot(after["x_m"] - before["x_m"],
                           after["y_m"] - before["y_m"])
        # Signed and wrapped, so that halving it is halving the turn the rover
        # actually made rather than averaging 179 and -179 into zero.
        swing = (after["heading_deg"] - before["heading_deg"] + 180.0) % 360.0 - 180.0
        turned = abs(swing)
        if moved > MOVED_WHILE_LOOKING_M or turned > TURNED_WHILE_LOOKING_DEG:
            return None, round(moved, 3), round(turned, 1)
        return ({"x_m": round((before["x_m"] + after["x_m"]) / 2.0, 3),
                 "y_m": round((before["y_m"] + after["y_m"]) / 2.0, 3),
                 "heading_deg": round(
                     (before["heading_deg"] + swing / 2.0 + 180.0) % 360.0 - 180.0,
                     1)},
                round(moved, 3), round(turned, 1))

    def _frame(self) -> dict[str, Any]:
        try:
            frame = self.capture() or {}
        except Exception as error:
            return {"ok": False, "error": f"{type(error).__name__}: {error}"}
        if not frame.get("ok"):
            return {"ok": False,
                    "error": frame.get("error", "the camera gave nothing")}
        if not frame.get("jpeg"):
            return {"ok": False, "error": "the camera gave an empty picture"}
        age = frame.get("age_s")
        if age is not None and age > FRAME_MAX_AGE_S:
            return {"ok": False,
                    "error": f"the newest picture is {age:.1f} s old, which is not "
                             f"what the camera is looking at now"}
        return frame

    def _pose(self) -> dict[str, Any] | None:
        """Where the rover was standing, if anything can say.

        None rather than a failure. An inspection with no pose is a perfectly good
        observation of the room -- it is where the camera was pointing that is
        missing, not what it saw -- and refusing to look because SLAM has not
        settled would make the world state depend on the navigator being up.
        """
        if self.pose is None:
            return None
        try:
            pose = self.pose()
        except Exception:
            return None
        return pose if isinstance(pose, dict) and pose else None

    def _failed(self, status: str, why: str, *, began: float | None = None,
                backend: str = "", inference_id: int | None = None,
                **fields: Any) -> dict[str, Any]:
        """Record why, change nothing else, and answer in a sentence.

        Every failure lands here, which is what makes "no world-state mutation on
        failure" a property of one function rather than a promise repeated at nine
        call sites.
        """
        began = time.time() if began is None else began
        row = {"duration_s": round(time.time() - began, 2), "status": status,
               "detail": why, "backend": backend, **fields}
        try:
            if inference_id is None:
                row.setdefault("started_at", began)
                row.setdefault("map_session", self.store.map_session())
                inference_id = self.store.record_inference(**row)
            else:
                row.pop("started_at", None)
                row.pop("frame_id", None)
                self.store.update_inference(inference_id, **row)
        except sqlite3.Error:
            # The diagnostics line is a nicety; not mutating the world on a failure
            # is the requirement, and it is already met by being here.
            inference_id = None
        return {"ok": False, "status": status, "error": why,
                "inference_id": inference_id,
                "duration_s": row["duration_s"]}
