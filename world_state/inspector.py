"""One inspection, end to end: take a picture, ask the model, believe some of it.

What is believed is what was in the picture. Which lasting thing each of those
things is does not come from here and no longer comes from the model at all; an
inspection ends with observations that carry the gimbal angles and the rover pose
behind them, and identity is settled afterwards from those measurements.

The order of the steps here is most of the failure behaviour. Nothing touches the
camera until the sidecar has said it is there, nothing touches the database until
the answer has been validated, and every path -- including the ones that never
reach the model -- writes exactly one line to the diagnostics log, so a popup
showing nothing new can always say whether that was a model that failed or a model
that found nothing.

No failure in here is allowed to reach the caller as an exception. The caller is
the process that owns STOP.
"""
from __future__ import annotations

import sqlite3
import threading
import time
from typing import Any, Callable

from .contract import PROMPT_VERSION, extract_json, validate
from .reasoner import describe_backend

#: A picture older than this is not what the camera is looking at now. Only ever
#: reached through the tracking loop's frame, which is the one path here that hands
#: back something it took for its own reasons rather than for this call.
FRAME_MAX_AGE_S = 5.0


class Inspector:
    """The one place an inspection happens, and the lock that makes it one.

    `capture` and `pose` are supplied rather than reached for, because the things
    they read belong to the daemon: the camera has one owner and the rover's place
    on the map is the navigator's to report. This class is given two functions and
    knows nothing about either.
    """

    def __init__(self, store, reasoner, capture: Callable[[], dict[str, Any]],
                 pose: Callable[[], dict[str, Any] | None] | None = None,
                 source: str = "cosmos_visual") -> None:
        self.store = store
        self.reasoner = reasoner
        self.capture = capture
        self.pose = pose
        self.source = source
        self._lock = threading.Lock()
        self.started_at = 0.0

    @property
    def busy(self) -> bool:
        return self._lock.locked()

    def inspect(self) -> dict[str, Any]:
        """Look once, and answer with what happened rather than with what was found.

        A second request while one is running is refused rather than queued. An
        inspection holds the camera and a couple of gigabytes of model for the best
        part of a minute, and two of them at once on this board means both are
        slower than one and neither is the picture anybody asked for.
        """
        if not self._lock.acquire(blocking=False):
            return {"ok": False, "status": "busy", "busy": True,
                    "error": f"an inspection has been running for "
                             f"{time.monotonic() - self.started_at:.0f} s; "
                             f"this one was not started"}
        self.started_at = time.monotonic()
        try:
            return self._inspect()
        except Exception as error:            # never past here: the daemon owns STOP
            return self._failed("error", f"{type(error).__name__}: {error}")
        finally:
            self._lock.release()

    # --- the steps ------------------------------------------------------------

    def _inspect(self) -> dict[str, Any]:
        began = time.time()
        backend = describe_backend(self.reasoner)

        ready, why = self.reasoner.available()
        if not ready:
            # Asked before the camera is touched. A sidecar that is not running
            # means this inspection cannot happen at all, and opening the camera to
            # find that out costs the rover a second and the face tracker its feed.
            return self._failed("unavailable", why, began=began, backend=backend)

        frame = self._frame()
        if not frame.get("ok"):
            return self._failed("no_frame", str(frame.get("error", "no picture")),
                                began=began, backend=backend)

        jpeg = frame["jpeg"]
        frame_id = self.store.save_frame(jpeg, frame.get("width"),
                                         frame.get("height"))
        capture = {"frame_id": frame_id,
                   "frame_path": self.store.frame_path(frame_id),
                   "pan": frame.get("pan"), "tilt": frame.get("tilt"),
                   "pose": self._pose()}
        inference_id = self.store.record_inference(
            started_at=began, status="running", backend=backend,
            prompt_version=PROMPT_VERSION, frame_id=frame_id,
            frame_live=1 if frame.get("live") else 0,
            map_session=self.store.map_session())

        # Nothing about the world goes into the call. The model is asked what is in
        # front of it and is told nothing about what the rover has seen before,
        # because on this rover's own frames that context corrupted the detections
        # themselves rather than merely failing to settle identity.
        answer = self.reasoner.inspect(jpeg)
        if not answer.ok:
            return self._failed("model_error", answer.error, began=began,
                                backend=backend, inference_id=inference_id,
                                frame_id=frame_id, duration_s=answer.duration_s,
                                model_id=answer.model_id)

        payload, why = extract_json(answer.text)
        if payload is None:
            # The model's own words are kept even here, and especially here: the
            # answer that could not be read is the one somebody will want to look
            # at, and the frame it was looking at is already stored beside it.
            return self._failed("bad_json", why, began=began, backend=backend,
                                inference_id=inference_id, frame_id=frame_id,
                                duration_s=answer.duration_s,
                                model_id=answer.model_id,
                                raw_json=answer.text[:4000])

        result = validate(payload)
        if not result.ok:
            return self._failed("invalid", result.error, began=began,
                                backend=backend, inference_id=inference_id,
                                frame_id=frame_id, duration_s=answer.duration_s,
                                model_id=answer.model_id,
                                raw_json=answer.text[:4000])

        try:
            stored = self.store.record(
                result.seen, capture=capture, scene=result.scene,
                source=self.source, model_id=answer.model_id,
                prompt_version=PROMPT_VERSION, inference_id=inference_id)
        except sqlite3.Error as error:
            return self._failed("store_error", f"{type(error).__name__}: {error}",
                                began=began, backend=backend,
                                inference_id=inference_id, frame_id=frame_id,
                                duration_s=answer.duration_s,
                                model_id=answer.model_id)

        detail = result.detail()
        if answer.usage.get("truncated"):
            detail = ("the model was cut off at its token limit; "
                      + detail if detail else
                      "the model was cut off at its token limit")
        self.store.update_inference(
            inference_id, duration_s=round(time.time() - began, 2), status="ok",
            detail=detail or None, model_id=answer.model_id,
            # What the model offered and what was not kept, counted as
            # observations. A complaint about a bounding box is neither.
            returned=len(result.seen) + result.refused,
            stored=stored["stored"],
            matched=stored["matched"], created=stored["created"],
            rejected=stored["rejected"] + result.refused,
            raw_json=answer.text[:8000])
        return {"ok": True, "status": "ok", "inference_id": inference_id,
                "frame_id": frame_id, "scene": result.scene,
                "duration_s": round(time.time() - began, 2),
                "returned": len(result.seen) + result.refused,
                "stored": stored["stored"],
                "matched": stored["matched"], "created": stored["created"],
                "rejected": stored["rejected"] + result.refused,
                "entities": stored["entities"], "detail": detail,
                "map_session": stored["map_session"], "model_id": answer.model_id}

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
               "detail": why, "backend": backend,
               "prompt_version": PROMPT_VERSION, **fields}
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
