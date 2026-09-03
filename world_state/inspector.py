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

from . import locate
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
#: And how far it may turn, in degrees, when nothing can say *when* the picture
#: was taken. Rotation is the term that actually hurts: it swings the whole
#: bearing rather than shifting its origin. The midpoint halves it, so 3.0 leaves
#: 1.5 -- exactly `locate.BEARING_SIGMA_DEG`, which is the error the geometry is
#: already told to expect from the gimbal.
#:
#: **This is the fallback now rather than the rule, and what replaced it is the
#: frame's own timestamp.** It cost 71 of the 108 looks of the drive of
#: 2026-09-03 their bearing -- two thirds of a run recording no direction for
#: anything it saw -- and the reason was never that the turn made the bearing
#: unknowable. It was that a bracket of two pose readings cannot say where in
#: itself the shutter opened. The camera has always known: every frame it hands
#: back carries the moment it was taken, and both paths through
#: `rover_camera._whole_jpeg` were throwing it away. Given that instant the
#: heading is interpolated to it, and what is left over is charged to the bearing
#: rather than used to discard the look. A frame with no timestamp still meets
#: this limit, because then there is nothing to interpolate to.
TURNED_WHILE_LOOKING_DEG = 3.0

#: How well the moment a frame was taken is known, in seconds.
#:
#: **Two terms, and only one of them is measurable from here.** What
#: `uvc_camera.snapshot` stamps a one-shot grab with is a single clock reading as
#: v4l2-ctl exits, less an estimate of the camera's pipeline lag -- not the
#: per-buffer V4L2 timestamps the tracking feed pairs up. So every frame in a
#: burst carries the same stamp, and the question is where that stamp sits
#: relative to the exposure.
#:
#: **The jitter is measured and it is small.** Over 18 bursts on the rover
#: (`bench_shutter.py`), the stamp lands 94% of the way through the grab with
#: 7.8 ms of spread about that: 0.23 degrees of bearing at the 29 degrees a
#: second this rover turns in the median, and 0.74 at the 94.8 it reached at
#: worst. Both are inside `locate.BEARING_SIGMA_DEG` and neither is what this
#: number is sized for.
#:
#: **The bias is not measurable from here and does not wash out.** A stamp
#: systematically late swings every bearing taken while turning the same way, and
#: nothing on this rover knows the true exposure instant to compare against.
#: Its scale is one frame interval -- the newest frame was exposed within one of
#: the stamp, which at 30 fps is 33 ms -- so 30 ms is that scale, and the
#: measured jitter fits inside it with room to spare. Measuring the bias means a
#: turn of known rate or a strobe, and neither has been done.
#:
#: It is used to *widen* the answer and never to narrow it -- `locate.sigma_of`
#: floors every bearing at `BEARING_SIGMA_DEG` -- so being wrong optimistic
#: understates a fast-turning look's cone and being wrong pessimistic only costs
#: precision.
#:
#: **What the same measurement says about the fault this replaced**: the stamp
#: landing at 94% of the grab means the midpoint the old code assumed was wrong
#: by 44% of whatever the rover turned, which on that drive's median 15.3-degree
#: turn is 6.7 degrees. That is why refusing the look was the only safe answer
#: before the instant was asked for, and why interpolating to it is worth this
#: much.
FRAME_TIME_SIGMA_S = 0.03

#: How wide a bearing may be before it is not a direction any more, in degrees.
#: Past this the bearing is dropped and the picture, the regions and the vectors
#: are kept, which is what this has always done with a look it could not aim.
#:
#: **Half of `locate.MIN_PARALLAX_DEG`, because that is what a bearing is for.**
#: Two rays closer together than 12 degrees are refused as a crossing -- the
#: intersection runs away down the line of sight and the answer is noise wearing
#: a number -- so a ray whose own error approaches that angle cannot take part in
#: a crossing whatever else is true of it. Half of it leaves the pair's combined
#: error inside the angle the pair itself has to clear.
#:
#: **It has to be reachable to be worth having, and the first value chosen was
#: not.** 15 degrees cannot happen: a turn wraps at 180 and the grab bracket runs
#: about 0.4 to 0.5 s, so the widest bearing physically obtainable was 13.5
#: degrees and the check was decoration. At 6 it takes about 200 degrees a second
#: to trip, which is a rover spinning on the spot rather than driving. The drive
#: of 2026-09-03 never reached it -- its worst was 52.3 degrees over 0.52 s, which
#: is 100 a second and 3 degrees of cone -- so it costs that recording nothing and
#: is there for the case that recording did not contain.
MAX_BEARING_SIGMA_DEG = 6.0


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
        # The bracket is timed as well as read, because where in it the shutter
        # opened is the whole question. See `_where`.
        before_at = time.time()
        before = self._pose()
        frame = self._frame()
        after_at = time.time()
        after = self._pose()
        if not frame.get("ok"):
            return self._failed("no_frame", str(frame.get("error", "no picture")),
                                began=began, backend=backend)

        jpeg = frame["jpeg"]
        frame_id = self.store.save_frame(jpeg, frame.get("width"),
                                         frame.get("height"))
        where, moved, turned, sigma_deg = self._where(
            before, after, before_at=before_at, after_at=after_at,
            taken_at=frame.get("taken_at"))
        capture = {"frame_id": frame_id,
                   "frame_path": self.store.frame_path(frame_id),
                   "pan": frame.get("pan"), "tilt": frame.get("tilt"),
                   # Which capture mode this was, because it chooses the lens
                   # the bearing is worked out through: a mode is a window onto
                   # the sensor as well as a pixel count, and this camera's two
                   # modes do not see the same angle. See `view.azimuth_deg`.
                   "frame_size": (None if not frame.get("width")
                                  or not frame.get("height")
                                  else (int(frame["width"]),
                                        int(frame["height"]))),
                   "pose": where,
                   # Half of what the rover covered while the shutter was open,
                   # which is how far out the ray's own starting point may be.
                   # Kept with the observation rather than folded into the pose,
                   # because the pose is a measurement and this is how good it
                   # is; `locate.fix` charges it to the answer.
                   "origin_sigma_m": None if where is None else round(moved / 2.0, 3),
                   # And how well the bearing itself is known, which travel does
                   # not answer: this is the turn the rover was still making,
                   # multiplied by how well the moment of the picture is known.
                   # None on a rover whose camera does not timestamp its frames,
                   # and None means the constant. See `locate.sigma_of`.
                   "bearing_sigma_deg": None if where is None else sigma_deg}
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
        detail = self._measured_detail(look, stored, settled, moved, turned,
                                       sigma_deg)
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
                         turned=0.0, sigma_deg=None) -> str:
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
            elif moved > MOVED_WHILE_LOOKING_M:
                why = (f"the rover moved {moved:.2f} m while the shutter was "
                       f"open")
            elif turned:
                why = (f"the rover turned {turned:.1f} deg while the shutter was "
                       f"open, which swings the bearing past "
                       f"{MAX_BEARING_SIGMA_DEG:.0f} deg")
            else:
                why = "no pose, no gimbal angle, or no field of view"
            parts.append(f"{missing} without a bearing ({why})")
        elif sigma_deg and turned:
            # A bearing was kept while the rover was turning. **Reported as the
            # geometry will spend it and not as it was computed**: the residual
            # is stored raw because it is the measurement, and `locate.sigma_of`
            # floors it at `BEARING_SIGMA_DEG` because nothing here can beat what
            # the gimbal and the heading are worth standing still. A line saying
            # 0.7 deg for a bearing every crossing treats as 1.5 would be the
            # console claiming a precision the rover does not act on.
            spent = max(locate.BEARING_SIGMA_DEG, sigma_deg)
            parts.append(f"the rover turned {turned:.1f} deg while the shutter "
                         f"was open, leaving the bearing good to "
                         f"{spent:.1f} deg")
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

    def _where(self, before, after, *, before_at=None, after_at=None,
               taken_at=None):
        """Where the picture was taken from, how much the rover moved for it, and
        how well that leaves the bearing known.

        **Interpolated to the moment the picture was taken, where the camera can
        say when that was.** Taking a picture is not instant: the shutter opens
        somewhere inside a grab that measures about a third of a second, and a
        rover turning at the 29 degrees a second this one manages in the median
        swings the whole bearing while it is open. The old answer was the midpoint
        of the two readings, which is the best a caller with two samples and no
        third fact can do -- and it cost 71 of the 108 looks of the drive of
        2026-09-03 their bearing, two thirds of a run recording no direction for
        anything it saw.

        The third fact was there all along. Every frame the camera hands back
        carries the moment it was taken, both through the tracking loop and
        through a one-shot grab, and both paths were dropping it. Given it, the
        pose is interpolated to that instant instead of averaged across the
        bracket, and what is left over -- the turn the rover was making,
        multiplied by how well the instant is known -- is *carried* as
        `bearing_sigma_deg` rather than being the reason to throw the look away.
        A fast turn buys a wide answer instead of no answer, which is what
        `MOVED_WHILE_LOOKING_M` already does for travel.

        `None` for the pose where either reading is missing, or where the rover
        covered more ground than `MOVED_WHILE_LOOKING_M` allows, or where the
        bearing would come out wider than `MAX_BEARING_SIGMA_DEG` -- and `None`
        here is not a failure. The picture, the regions and the vectors are all
        kept; what is dropped is the one thing that was not measured well enough
        to keep, which is the direction.

        With no timestamp there is nothing to interpolate to, so the midpoint and
        `TURNED_WHILE_LOOKING_DEG` are what remain, exactly as before.
        """
        if not isinstance(before, dict) or not isinstance(after, dict):
            return (after if isinstance(after, dict) else before), 0.0, 0.0, None
        moved = math.hypot(after["x_m"] - before["x_m"],
                           after["y_m"] - before["y_m"])
        # Signed and wrapped, so that a fraction of it is a fraction of the turn
        # the rover actually made rather than averaging 179 and -179 into zero.
        swing = (after["heading_deg"] - before["heading_deg"] + 180.0) % 360.0 - 180.0
        turned = abs(swing)
        if moved > MOVED_WHILE_LOOKING_M:
            return None, round(moved, 3), round(turned, 1), None

        share, sigma_deg = self._at_the_shutter(before_at, after_at, taken_at,
                                                turned)
        if share is None:
            if turned > TURNED_WHILE_LOOKING_DEG:
                return None, round(moved, 3), round(turned, 1), None
            share, sigma_deg = 0.5, None
        elif sigma_deg is not None and sigma_deg > MAX_BEARING_SIGMA_DEG:
            return None, round(moved, 3), round(turned, 1), None
        return ({"x_m": round(before["x_m"]
                              + (after["x_m"] - before["x_m"]) * share, 3),
                 "y_m": round(before["y_m"]
                              + (after["y_m"] - before["y_m"]) * share, 3),
                 "heading_deg": round(
                     (before["heading_deg"] + swing * share + 180.0)
                     % 360.0 - 180.0, 1)},
                round(moved, 3), round(turned, 1),
                None if sigma_deg is None else round(sigma_deg, 2))

    @staticmethod
    def _at_the_shutter(before_at, after_at, taken_at, turned_deg):
        """Where in the bracket the picture falls, and what that leaves unknown.

        `(None, None)` whenever the arithmetic would be a guess: no timestamp, a
        bracket of no length, or an instant outside the bracket. That last one is
        the real case rather than a defensive nicety -- the tracking loop hands
        back the newest frame it has seen, which may predate the first pose
        reading, and interpolating past the end of a bracket is extrapolation
        dressed as a measurement.

        What is left unknown is the turn rate multiplied by
        `FRAME_TIME_SIGMA_S`: how far the bearing could have swung in the time
        the instant itself is uncertain by.
        """
        if taken_at is None or before_at is None or after_at is None:
            return None, None
        try:
            span = float(after_at) - float(before_at)
            offset = float(taken_at) - float(before_at)
        except (TypeError, ValueError):
            return None, None
        if span <= 0.0 or not 0.0 <= offset <= span:
            return None, None
        rate = float(turned_deg) / span
        return offset / span, rate * FRAME_TIME_SIGMA_S

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
