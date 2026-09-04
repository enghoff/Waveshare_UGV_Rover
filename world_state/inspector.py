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
from . import oak
from . import view
from .depth_client import describe_ranger
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

#: How far a range may come back from the guess the box was drawn at before the
#: depth camera is asked again, as a fraction of that guess.
#:
#: **This exists because the two cameras are not in the same place.** Finding a
#: gimbal camera's box in the OAK's picture needs to know how far away the thing
#: is, which is the thing being asked -- so the box is drawn at
#: `oak.GUESS_RANGE_M` and any answer that turns out to be nothing like it is
#: asked again from where the thing now appears to be. The error being corrected
#: is the parallax between the two lenses, which is the offset between them
#: divided by the range: at two metres a guess wrong by half a metre moves the
#: box about four pixels, which is nothing beside a box tens of pixels wide, and
#: at sixty centimetres it moves it forty.
#:
#: 0.4 puts the second ask at nearer than 1.5 m or further than 3.5, so it fires
#: on the near things where it matters and on almost nothing else. It costs one
#: extra loopback call on the looks it fires for.
REASK_RANGE_FRAC = 0.4

#: How much of the picture has to be different from the last one recorded before
#: this look is worth recording at all, as a share of the frame.
#:
#: **A rover standing still in a still room recorded the same picture over and
#: over.** Parked overnight on 2026-09-04 it took one of those every five
#: minutes, and each one cost a frame on disk, a pass through three encoders and
#: two to four observations that can never be triangulated with anything -- two
#: rays from the same place do not cross, and the resolver compares every pair,
#: so the pool they join makes every later look slower for as long as the rover
#: is switched on.
#:
#: **The test is on the picture and not on the pose, because the pose is what
#: was wrong.** `rover_world._world_worth_looking` already refuses a look from a
#: place the rover has looked from, and a parked rover gets past it two ways:
#: `LOOK_ANYWAY_S`, which is deliberate, and a scan matcher whose position
#: wanders far enough to look like a move while the wheels are stopped, which is
#: not. Only the frame can tell those apart from a rover that really went
#: somewhere.
#:
#: Measured on this rover's own camera on 2026-09-04, parked in front of a wall,
#: a sofa and a cable, as the share of the grid below whose cells moved by more
#: than `PICTURE_CELL_CHANGE`:
#:
#:     40 frames a second apart, parked         0.000 between neighbours, and
#:                                              0.004 for the worst pair of any
#:                                              two in the burst
#:     40 frames fifteen seconds apart, parked  0.039 between neighbours at
#:                                              worst, and 0.199 for the worst
#:                                              pair across the whole ten minutes
#:     consecutive looks, the rover moved       0.152 at worst, 0.277 at the
#:                                              fifth percentile, 0.55 median
#:
#: **The room was the same picture to the eye throughout both bursts**, so the
#: ten-minute figure is the camera rather than the room: auto-exposure hunts by
#: several grey levels and does not hunt evenly, which is a change of contrast
#: that removing the mean cannot remove. **So this cannot judge two looks taken
#: minutes apart, and it is not asked to.** What it judges reliably is two looks
#: taken close together, which is the whole of what this was costing: a rover
#: whose pose jitters looks once a second, and 40 such looks come back as one
#: recorded and 39 discarded at any setting between 0.02 and 0.30.
#:
#: 0.05 is twelve times the worst pair of a second-apart burst and three times
#: below the smallest change a real move produced. It is deliberately nearer the
#: still end than the middle, because the two mistakes do not cost the same: a
#: look wrongly discarded is a picture of the room that no longer exists, and a
#: look wrongly kept is half a second of GPU.
SAME_PICTURE_SHARE = 0.05
#: The grid two pictures are compared on. **A cell is 1/256 of the frame, which
#: is near enough `perceive.MIN_AREA`** -- the smallest box a look would keep. So
#: a change too small to move one cell is a change too small to have become an
#: observation, and this is blind to exactly what the pipeline behind it is
#: blind to.
PICTURE_GRID = 16
#: And how far one cell has to move to count, in grey levels out of 255. Two
#: frames a second apart from a parked rover move their worst cell by under 2;
#: a real change of viewpoint moves the cells it touches by 50 and more.
PICTURE_CELL_CHANGE = 15.0


def picture(jpeg: bytes):
    """The frame reduced to the grid two looks are compared on, or None.

    Grey, because a change of colour with no change of shape is a change of
    light rather than a change of room. Averaged down rather than sampled down,
    so that a rover rocking a pixel on its springs does not read as a room that
    moved.

    None whenever the picture cannot be reduced -- a frame that will not decode,
    or a host without numpy and OpenCV. **Nothing above may treat that as a
    fault.** A look that cannot be compared is a look that gets recorded, which
    is exactly what this rover did before the comparison existed.
    """
    try:
        import cv2
        import numpy as np
    except Exception:                       # noqa: BLE001 -- not this rover's job
        return None
    try:
        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8),
                             cv2.IMREAD_GRAYSCALE)
        if image is None or not image.size:
            return None
        return cv2.resize(image, (PICTURE_GRID, PICTURE_GRID),
                          interpolation=cv2.INTER_AREA).astype("float32")
    except Exception:                       # noqa: BLE001
        return None


def picture_changed(before, after) -> float | None:
    """How much of the picture is different, as a share of its cells.

    **Each frame's own mean brightness is taken off first**, so that the whole
    picture getting brighter is not a change. Auto-exposure walks the mean by
    several grey levels over five minutes of a parked rover, and by six in one
    step when it stops driving, and none of that is anything happening in the
    room.

    **A share of the picture rather than an average over it**, because an
    average is carried by whatever is brightest and a room does not change
    evenly. Over the same frames the average separates a still rover from a
    moving one by a factor of five -- 1.8 grey levels at worst standing still
    against 9.0 for the smallest real move -- while the share separates them by
    twenty. It also says what it means, which is how much of this picture is not
    the picture the rover already has.
    """
    try:
        import numpy as np
    except Exception:                       # noqa: BLE001
        return None
    if before is None or after is None or before.shape != after.shape:
        return None
    moved = np.abs((after - after.mean()) - (before - before.mean()))
    return float((moved > PICTURE_CELL_CHANGE).mean())


def _speed(moved_m: float, before_at, after_at) -> float:
    """How fast the rover was going over the shutter bracket, in metres a second.

    Zero when the bracket cannot be timed, which is the safe direction: it is
    only ever used to *widen* a range's uncertainty, so an unknown speed costs
    precision rather than claiming any.
    """
    try:
        span = float(after_at) - float(before_at)
    except (TypeError, ValueError):
        return 0.0
    return 0.0 if span <= 0.0 else max(0.0, float(moved_m)) / span


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
                 reach: Callable[[float, float, float], float | None] | None = None,
                 ranger=None) -> None:
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
        #: The depth camera, and the only thing here that can say how *far* away
        #: anything is. Optional in the strongest sense: a rover whose OAK has
        #: been unplugged, or one whose mount has never been measured, records
        #: exactly what this component recorded before ranges existed -- a box,
        #: two vectors and a bearing -- and every gate downstream abstains rather
        #: than refusing. See `locate.stands_at_range`.
        self.ranger = ranger
        #: The camera's horizontal field of view, which the daemon owns. Without
        #: it a box cannot become an angle, so the bearing columns stay null and
        #: say so rather than being filled in from a guess.
        self.fov_deg = fov_deg
        self.source = source
        self._lock = threading.Lock()
        self.started_at = 0.0
        #: The last picture that was actually recorded, reduced to its grid.
        #: **The last one kept and not the last one seen**, because the point of
        #: comparing at all is that a room may drift: against the last frame seen
        #: a slow change is never big enough to notice and the rover records
        #: nothing for ever, while against the last frame kept the same drift
        #: accumulates until it crosses the line and is recorded once.
        self._kept_picture = None
        #: The run of unchanged looks the diagnostics log is currently showing as
        #: one line: `(inference_id, how many)`. See `_unchanged`.
        self._unchanged_run = None

    def forget_picture(self) -> None:
        """Take the next look whatever it looks like.

        Emptying the store leaves the rover holding a picture whose observations
        have all been deleted, and comparing against that would refuse to record
        the room again until something in it moved. Whoever clears the store
        calls this.
        """
        self._kept_picture = None
        self._unchanged_run = None

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
        # The run of unchanged looks ends here and is picked up again only by the
        # one path below that continues it, so every other outcome -- a look
        # recorded, a camera that failed, a model that failed -- starts a fresh
        # line in the diagnostics log without having to say so.
        run, self._unchanged_run = self._unchanged_run, None

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
        # **Before the frame is written down and before the encoders are asked.**
        # A look the rover already has is worth nothing at all, so the cheapest
        # possible test comes first: the frame is not saved, the sidecar is not
        # called, and nothing joins the pool the resolver has to compare. See
        # SAME_PICTURE_SHARE.
        seen = picture(jpeg)
        share = picture_changed(self._kept_picture, seen)
        if share is not None and share < SAME_PICTURE_SHARE:
            return self._unchanged(share, run, began=began, backend=backend)

        frame_id = self.store.save_frame(jpeg, frame.get("width"),
                                         frame.get("height"))
        where, moved, turned, sigma_deg = self._where(
            before, after, before_at=before_at, after_at=after_at,
            taken_at=frame.get("taken_at"))
        # Which camera this picture came from, and where that camera actually
        # is. A ray has to start where the lens is, and the two cameras on this
        # rover are not in the same place -- ten centimetres between them is
        # three degrees of bearing at two metres, which is twice what the
        # geometry is told to expect. `oak.pose_at` moves the pose to the OAK's
        # optical centre and leaves a gimbal look exactly as it was.
        camera = frame.get("camera") or oak.GIMBAL
        if camera == oak.OAK:
            where = oak.pose_at(where)
        capture = {"frame_id": frame_id,
                   "camera": camera,
                   # How fast the rover was going, from the bracket that already
                   # measures it. Not stored -- `store.record` ignores what it
                   # does not have a column for -- and here because a range read
                   # from a frame two thirds of a second old is only true of
                   # where the camera was then. See `_aged_sigma`.
                   "speed_mps": _speed(moved, before_at, after_at),
                   # The lens to read this picture's pixels through, which is the
                   # device's own for the OAK and None -- meaning the swept fit
                   # in `face_tracking/lens.py` -- for the gimbal camera. Carried
                   # on the capture rather than looked up in `view`, because only
                   # the inspection holding the frame knows which camera took it.
                   "lens": self._lens_for(camera),
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

        # How far away each region is, asked of the depth camera now that there
        # are boxes to ask about. After the encoders because that is when the
        # boxes exist, and before the write because a range is part of what the
        # look measured rather than something added to it afterwards.
        ranges, ranged_note = self._ranges(capture, look.regions)

        try:
            stored = self.store.record(
                look.regions, capture=capture, source=self.source,
                model_id=look.backend, inference_id=inference_id,
                fov_deg=self.fov_deg, region_source="yoloe",
                vectors_from=look.backend, ranges=ranges)
        except sqlite3.Error as error:
            return self._failed("store_error", f"{type(error).__name__}: {error}",
                                began=began, backend=backend,
                                inference_id=inference_id, frame_id=frame_id,
                                duration_s=look.duration_s, model_id=look.backend)

        # **Remembered only now that it is written down.** A picture kept after a
        # model or a store that failed would be compared against on the next
        # look, and a still room would then answer "nothing has changed" for as
        # long as the fault lasted, which is the fault hiding itself.
        self._kept_picture = seen

        # Identity is settled here rather than in `record`, and after the write
        # rather than during it. What was measured is history the moment it is
        # stored; which lasting thing it belongs to is an opinion formed from
        # that history and from everything already in it, and a failure to form
        # one must leave the measurement untouched.
        settled = self._settle() if settle else {}
        detail = self._measured_detail(look, stored, settled, moved, turned,
                                       sigma_deg, ranged_note)
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

    def _unchanged(self, share: float, run, *, began: float,
                   backend: str) -> dict[str, Any]:
        """The room had not changed, so record that and nothing else.

        **Not a failure.** The rover looked, the picture was the one it already
        has, and the right thing to do with it is nothing -- so the row says `ok`
        with nothing offered and nothing stored, which is what happened, rather
        than a status of its own that the console would paint as a warning.

        **A run of them is one line rather than a line each**, because the log
        the console shows is twelve rows deep and a rover parked for an hour
        would otherwise push every real look out of it. The row is restamped as
        it grows, so a person watching a parked rover sees a look that is seconds
        old rather than one that appears to have stopped an hour ago.
        """
        detail = f"the same picture as the last look -- {share:.1%} of it differed"
        row = {"status": "ok", "backend": backend, "detail": detail,
               "started_at": began, "duration_s": round(time.time() - began, 2),
               "returned": 0, "stored": 0, "matched": 0, "created": 0,
               "rejected": 0}
        running = 1 if run is None else run[1] + 1
        if running > 1:
            row["detail"] = f"{detail}, {running} looks running"
        try:
            if run is None:
                inference_id = self.store.record_inference(
                    map_session=self.store.map_session(), **row)
            else:
                inference_id = run[0]
                self.store.update_inference(inference_id, **row)
        except sqlite3.Error:
            # The same rule the failure path keeps: the diagnostics line is a
            # nicety, and not having recorded anything is already true.
            inference_id, running = None, 1
        self._unchanged_run = None if inference_id is None else (inference_id,
                                                                 running)
        return {"ok": True, "status": "unchanged", "unchanged": True,
                "inference_id": inference_id, "changed_share": round(share, 4),
                "duration_s": row["duration_s"], "detail": row["detail"],
                "returned": 0, "stored": 0, "placed": 0, "matched": 0,
                "created": 0, "ambiguous": 0, "rejected": 0, "settled": False,
                "entities": [], "decisions": []}

    def _lens_for(self, camera: str):
        """The optics to read this camera's pixels through, or None for the
        gimbal camera's own swept fit.

        None is the answer for the gimbal camera and is not a failure: `view`
        falls back to `face_tracking/lens.py`, which is where that camera has
        been described since a sweep on this rover fitted it. The OAK's live on
        the OAK, so they are asked for -- and a rover that cannot reach the depth
        service gets None there too, which means an OAK look records its picture,
        its regions and its vectors with no bearing at all. That is the same
        silence a missing pose earns, for the same reason: a bearing drawn
        through a lens nobody could read is a guess wearing a number.
        """
        if camera != oak.OAK or self.ranger is None:
            return None
        try:
            return self.ranger.lens()
        except Exception:                          # never past here
            return None

    def _ranges(self, capture: dict[str, Any], regions: list):
        """How far away each region is, and one clause for the diagnostics line.

        `([], "")` whenever the question cannot be asked, which is most of the
        time and is not a failure: no depth camera on this rover, a mount nobody
        has measured, a service that is restarting, or a gimbal turned to look at
        something the OAK cannot see. Everything downstream treats a missing
        range as abstention.

        **Two shapes, because the two cameras stand differently to the depth
        map.** A look taken through the OAK is already in the depth map's own
        frame -- the depth is warped into the colour camera's geometry on the
        device -- so a box goes straight across. A look taken through the gimbal
        is a box on a different lens on a mount that turns, so each box becomes
        four directions in the rover's frame and `oak.box_for` finds where those
        land in the OAK's picture, if they land in it at all. About half of a
        centred gimbal frame does, and a look taken over the rover's shoulder
        does not.
        """
        if self.ranger is None or not regions:
            return [], ""
        camera = capture.get("camera") or oak.GIMBAL
        try:
            if camera == oak.OAK:
                return self._ranges_here(capture, regions)
            return self._ranges_across(capture, regions)
        except Exception as error:                 # never past here
            return [], f"no ranges ({type(error).__name__}: {error})"

    def _ranges_here(self, capture: dict[str, Any], regions: list):
        """Ranges for boxes already drawn on the depth camera's own picture."""
        answers, error = self.ranger.ranges([list(region.bbox)
                                             for region in regions])
        if error:
            return [], f"no ranges ({error})"
        speed = capture.get("speed_mps") or 0.0
        got = 0
        for one in answers:
            if one is None or one.range_m is None:
                continue
            one.sigma_m = self._aged_sigma(one, speed)
            got += 1
        return answers, (f"{got} of {len(regions)} ranged"
                         if got else "nothing in the frame could be ranged")

    @staticmethod
    def _aged_sigma(one, speed_mps: float) -> float:
        """What a range is worth once its own staleness is charged to it.

        **A range is true of where the camera was when the frame was taken.** The
        depth camera runs at two frames a second and holds each frame back until
        the picture it belongs with has come through the encoder, so a reading is
        about two thirds of a second old when it is read -- and on a rover
        exploring at 0.47 m/s that is thirty centimetres, against a stereo error
        of two to seven at these distances. Ignoring it would make the world state
        trust a stale range far more than a fresh one deserves.

        Added in quadrature with what the camera said the reading was worth, the
        same way `Inspector._where` adds the turn to the bearing: the two are
        independent, one is the camera's and one is the rover's.

        It is a *widening* and never a narrowing -- a rover standing still adds
        nothing -- so being wrong optimistic about the speed only costs precision.
        """
        camera = float(one.sigma_m or 0.0)
        stale = max(0.0, float(speed_mps)) * max(0.0, float(one.age_s or 0.0))
        return round(math.hypot(camera, stale), 3)

    def _ranges_across(self, capture: dict[str, Any], regions: list):
        """Ranges for boxes drawn on the gimbal camera, found in the OAK's picture.

        **The offset between the two cameras is what makes this more than a
        rotation.** They sit a few centimetres apart, so they see a thing two
        metres away in slightly different directions and how different depends on
        how far away it is -- which is the thing being asked. The box is worked
        out at `oak.GUESS_RANGE_M`, and any answer that comes back a long way
        from that guess is asked again from where it now appears to be. One extra
        loopback call, and only for the near things where the parallax is worth
        correcting: at two metres a wrong guess of half a metre moves the box by
        about four pixels, and at sixty centimetres it moves it by forty.
        """
        try:
            lens = self.ranger.lens()
        except Exception:                          # never past here
            lens = None
        if lens is None or not oak.MEASURED:
            return [], ""
        size = capture.get("frame_size")
        pan = capture.get("pan") or 0.0
        tilt = capture.get("tilt") or 0.0
        corners: list[Any] = []
        boxes: list[Any] = []
        for region in regions:
            found = self._corners_of(region.bbox, pan, tilt, size)
            corners.append(found)
            boxes.append(None if found is None else oak.box_for(found, lens))
        asked = [index for index, box in enumerate(boxes) if box is not None]
        if not asked:
            return [], "none of it was in the depth camera's picture"
        answers, error = self.ranger.ranges([boxes[index] for index in asked])
        if error:
            return [], f"no ranges ({error})"
        found: list[Any] = [None] * len(regions)
        again: list[int] = []
        for slot, index in enumerate(asked):
            if slot >= len(answers):
                break
            one = answers[slot]
            found[index] = one
            if one is not None and one.range_m is not None and (
                    abs(one.range_m - oak.GUESS_RANGE_M)
                    > REASK_RANGE_FRAC * oak.GUESS_RANGE_M):
                again.append(index)
        if again:
            self._reask(again, corners, found, lens)
        return self._as_gimbal(corners, found, len(regions),
                               capture.get("speed_mps") or 0.0)

    def _reask(self, again, corners, found, lens) -> None:
        """Ask a second time for the boxes whose range was nothing like the guess.

        Silent on failure and deliberately so: the first answer is already in
        hand and is at worst a few pixels off, so a second call that does not
        come back leaves a slightly worse number rather than none at all.
        """
        redrawn = []
        for index in again:
            placed = oak.box_for(corners[index], lens, found[index].range_m)
            if placed is not None:
                redrawn.append((index, placed))
        if not redrawn:
            return
        answers, error = self.ranger.ranges([box for _index, box in redrawn])
        if error:
            return
        for slot, (index, _box) in enumerate(redrawn):
            if slot >= len(answers):
                break
            one = answers[slot]
            if one is not None and one.range_m is not None:
                found[index] = one

    def _as_gimbal(self, corners, found, total: int, speed_mps: float = 0.0):
        """The OAK's ranges, as lengths along the rays they will be stored against.

        **A range is a length along a particular ray from a particular point**,
        and these were measured from the other camera. The observation's ray
        starts at the gimbal camera, so an OAK range put on it unchanged would be
        a few centimetres wrong in a way that grows as things get closer -- and
        `locate` would then spend it against a crossing measured from somewhere
        else. `oak.range_from_gimbal` is the correction, run with the range that
        actually came back rather than with the guess the box was drawn at.
        """
        ranged = 0
        for index, one in enumerate(found):
            if one is None or one.range_m is None:
                continue
            corrected = oak.range_from_gimbal(corners[index], one.range_m)
            if corrected is None or corrected <= 0.0:
                found[index] = None
                continue
            one.range_m = round(corrected, 3)
            one.sigma_m = self._aged_sigma(one, speed_mps)
            ranged += 1
        return found, (f"{ranged} of {total} ranged by the depth camera"
                       if ranged else
                       "the depth camera saw none of it well enough to range")

    @staticmethod
    def _corners_of(bbox, pan_deg: float, tilt_deg: float, size):
        """A box on the gimbal camera as four directions in the rover's frame.

        None when the box is unusable or the lens cannot be reached, which is the
        same silence everything else here keeps. Four corners rather than a
        centre because what the depth camera is asked for is an area of its own
        picture, and the two lenses do not agree about shape: a box near the edge
        of a 130-degree fisheye maps to a very different rectangle on a pinhole.
        """
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            return None
        try:
            left, top, right, bottom = (float(value) for value in bbox)
        except (TypeError, ValueError):
            return None
        found = []
        for x_frac, y_frac in ((left, top), (right, top),
                               (left, bottom), (right, bottom)):
            direction = view.chassis_direction(x_frac, y_frac, pan_deg, tilt_deg,
                                               size)
            if direction is None:
                return None
            found.append(direction)
        return found

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
                         turned=0.0, sigma_deg=None, ranged_note="") -> str:
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
        if ranged_note:
            parts.append(ranged_note)
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
