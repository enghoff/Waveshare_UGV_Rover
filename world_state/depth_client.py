"""Asking the depth camera how far away things are, and what comes back.

The same shape as [perception_client.py](perception_client.py) next door, for the
same reason: the caller is an inspection running inside the process that owns
STOP, so nothing here may raise at it, and a camera that is not running is an
ordinary answer rather than an exception. A rover whose OAK has been unplugged
records exactly what it recorded before ranges existed.

**Three things come back and they are one measurement.** The colour picture, the
ranges inside boxes drawn on it, and the lens those boxes are read through. They
are one measurement because the depth is warped into the colour camera's geometry
on the device -- see `oak_depth/depth_server.py` -- so the same fraction of the
picture and of the depth map is the same ray, with no remapping here and no
second lens model to drift.

**The camera can be switched off, and this is where that is asked for.** The
rover switches it off after half a minute of standing still and on again when it
drives -- the rule is `rover_daemon/rover_depth.py` and it calls `set_power`
here. Nothing in a look changes: a switched-off camera returns the same "no
range" every consumer here already treats as abstention, so the only difference
between an OAK that is off and an OAK that was never fitted is the sentence in
the diagnostics line. What that costs a recording is that a parked rover's looks
carry no distances, and neither do the first few seconds of a drive, which is the
firmware upload the camera needs every time it wakes.

**The lens is fetched and never written down.** `face_tracking/lens.py` holds the
gimbal camera's optics because a sweep on this rover fitted them and the gimbal is
driven through them; the OAK's are stored on the OAK, and the service reads them
off the device for the exact frame size it emits. A copy here could only ever
disagree with the camera, so there is none: no lens, no bearings, which is the
same silence `view._lens_for` keeps.
"""
from __future__ import annotations

import http.client
import json
import os
import time
from dataclasses import dataclass, field
from typing import Any
from urllib.parse import urlparse

#: Where the depth camera listens. 8769 is the daemon, 8770 this, 8771 the
#: console, 8772 and 8773 the ROS bridges, 8774 the frame service and 8776
#: perception.
DEFAULT_URL = "http://127.0.0.1:8770"
ENV_URL = "UGV_DEPTH_URL"

#: The wall clock on one call. Short, deliberately: this sits inside a look, the
#: service answers out of a frame it already holds, and a range that arrives late
#: is worth less than a look taken on time. An inspection that gets no range
#: stores everything else exactly as it did before.
TIMEOUT_S = 3.0
#: How long the lens is kept before it is asked for again. It cannot change while
#: the service is running -- it is read off the device when the pipeline opens --
#: so this is only about noticing a restart, and a minute is far finer than a
#: camera gets replaced.
LENS_CACHE_S = 60.0
#: A picture older than this is not what the camera is looking at now. The
#: service runs at fifteen frames a second, so this is many tens of frames; it is
#: deliberately loose because all it has to catch is a stream that has stalled,
#: and how stale a frame is gets *charged* to the answer rather than refused --
#: see `Observation.age_s`.
FRAME_MAX_AGE_S = 2.0
#: And how far apart the picture and the depth map behind it may have been taken.
#:
#: **This stopped being the binding constraint when the camera went to 15 fps on
#: 2026-09-04.** Measured on the rover that day: at the old 2 fps the gap ran
#: 0.041 to 0.069 s and *climbed steadily* through a 40-second sample, about
#: 0.7 ms/s -- the two streams' frame boundaries sliding relative to each other,
#: bounded by the half frame interval at which the pairing picks the adjacent
#: frame instead. At 15 fps that bound is 33 ms and the measurement is **0.000 to
#: 0.001 s, stable over a minute**: the two halves are now exposed together.
#:
#: So 0.30 is left where it is and has become a stall guard rather than a pairing
#: check. It cannot be the latter any more -- a whole-frame mispairing is 67 ms at
#: this rate, well under the threshold -- but nothing has been seen within two
#: orders of magnitude of it, and the phase is charged to the answer anyway rather
#: than relied upon being small.
#:
#: **The phase is charged rather than merely tolerated.** A fifth of a second is
#: 9 cm at the speed this rover explores at, and the box was drawn on one frame
#: while the range came off the other; `Inspector._aged_sigma` adds it to the
#: frame's own age and charges the pair to every range it produces.
MAX_APART_S = 0.30


@dataclass
class Ranged:
    """How far away one box is, as measured.

    `range_m` is None when there was nothing in the box to measure -- a dark or
    textureless surface returns no disparity at all -- and `valid` beside it is
    what tells that apart from a box the camera could not see into. The two are
    kept separate rather than collapsed to None because the resolver treats "not
    measured" as abstention and would treat "measured as nothing" as a fault.
    """

    range_m: float | None = None
    sigma_m: float | None = None
    valid: float = 0.0
    pixels: int = 0
    #: How old the depth frame this came from was, in seconds. **Carried because
    #: a range is only true of where the camera was when it was taken**: the
    #: depth camera holds each frame back until the picture it belongs with has
    #: arrived, so a reading is always somewhat older than the moment it is read
    #: at -- and on a rover exploring at 0.47 m/s that age is distance. Measured
    #: on the rover on 2026-09-04: a median of 0.768 s when the camera ran at
    #: 2 fps, which is 36 cm, and **0.102 s at the 15 it runs at now, which is 5**.
    #: Read off the reply rather than assumed, which is what let the rate change
    #: without touching any of this. The caller knows how fast the rover was going
    #: and is the only one that can charge it to the answer; see
    #: `Inspector._aged_sigma`.
    age_s: float = 0.0
    #: And how far apart the picture the box was drawn on and the depth map the
    #: range came off were exposed. A fifth of a second on this camera, because
    #: its colour sensor and its mono pair free-run on their own clocks -- see
    #: `MAX_APART_S`. Charged the same way and for the same reason as the age:
    #: both are time the rover was moving through between the two halves of one
    #: measurement.
    apart_s: float = 0.0


@dataclass
class Frame:
    """One colour picture from the depth camera, with what it is worth.

    `apart_s` is how far the depth map behind it was taken from it, and it is the
    reason this is one object rather than a picture and a separate call: a
    picture and a set of ranges half a second apart on a moving rover are two
    different rooms, and the caller has to be able to see that.
    """

    ok: bool = False
    error: str = ""
    jpeg: bytes = b""
    width: int = 0
    height: int = 0
    taken_at: float | None = None
    age_s: float = 0.0
    apart_s: float = 0.0


@dataclass
class Power:
    """Whether the depth camera is switched on, and how long it has been that way.

    Four states rather than two, and the two extra ones are the point. `waking`
    is the several seconds the host spends uploading firmware to a VPU with no
    flash, during which a camera that has been switched on is answering nothing;
    an empty `state` with an `error` beside it is the service itself not
    answering, which is a different thing again from a camera that is off and
    saying so. Anything drawing a switch has to be able to tell all four apart,
    because three of them look identical from the outside -- no frames.
    """

    state: str = ""
    since_s: float = 0.0
    error: str = ""

    @property
    def on(self) -> bool:
        """Whether the camera is meant to be awake, waking included.

        Waking counts as on because it is what was asked for and what it is on
        its way to being. A switch that read `off` for the seconds after it
        was turned on would invite a second press, and the second press is the
        one that costs a firmware upload.
        """
        return self.state in ("on", "waking")


@dataclass
class Lens:
    """The colour camera's optics, as the device stores them.

    A pinhole, and it is one honestly rather than for convenience: the OAK's
    rational distortion model has its numerator and denominator terms within a
    tenth of each other -- k1 -3.478 against k4 -3.574, k2 3.165 against k5
    3.554, k3 15.88 against k6 15.25 -- so they very nearly cancel and what is
    left is a lens that puts angle in proportion to the tangent of the pixel
    offset. That is measured on this unit and stated so that anybody who finds a
    residual at the edge of the frame knows where to look first.
    """

    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    width: int = 0
    height: int = 0
    hfov_deg: float = 0.0
    vfov_deg: float = 0.0


class Ranger:
    """A camera that answers how far away the things in its picture are."""

    name = "depth"

    def available(self) -> tuple[bool, str]:
        return True, ""

    def frame(self) -> Frame:
        raise NotImplementedError

    def ranges(self, boxes: list[list[float]]) -> tuple[list[Ranged], str]:
        raise NotImplementedError

    def lens(self) -> Lens | None:
        return None

    def power(self) -> Power:
        """Whether the camera is switched on. Never raises, like everything here."""
        return Power(error=f"{self.name} cannot be switched")

    def set_power(self, on: bool) -> Power:
        """Switch it, and answer with what that did rather than with success.

        What comes back is a state and not an acknowledgement, because switching
        on does not finish: it starts a firmware upload that takes several
        seconds, and the honest answer at the moment of the call is `waking`.
        """
        return Power(error=f"{self.name} cannot be switched")

    def describe(self) -> str:
        return self.name


class FakeRanger(Ranger):
    """A depth camera that answers from a script, for tests and for a desk.

    Enough to exercise the wiring, the geometry and the store without a Myriad X
    on the bus. It proves nothing about stereo and an experiment about whether
    the rover measures a real chair at two metres cannot be run against it.
    """

    name = "fake-depth"

    def __init__(self, frames: list[Frame] | None = None,
                 answers: list[list[Ranged]] | None = None,
                 lens: Lens | None = None, fail: str = "") -> None:
        self.frames = list(frames or [])
        self.answers = list(answers or [])
        self.fail = fail
        self.switched = "on"
        self.switches: list[bool] = []
        self._lens = lens or Lens(fx=456.5, fy=456.4, cx=321.1, cy=189.8,
                                  width=640, height=360,
                                  hfov_deg=70.1, vfov_deg=43.0)
        self.asked: list[list[list[float]]] = []

    def available(self) -> tuple[bool, str]:
        return (False, self.fail) if self.fail else (True, "")

    def frame(self) -> Frame:
        if self.fail:
            return Frame(ok=False, error=self.fail)
        if self.frames:
            return self.frames.pop(0)
        return Frame(ok=True, jpeg=b"\xff\xd8fake", width=640, height=360,
                     taken_at=time.time())

    def ranges(self, boxes: list[list[float]]) -> tuple[list[Ranged], str]:
        self.asked.append([list(box) for box in boxes])
        if self.fail:
            return [], self.fail
        if self.answers:
            answer = self.answers.pop(0)
            # Short scripts are padded rather than raising: a test that cares
            # about the first two regions should not have to spell out the rest.
            return [answer[index] if index < len(answer) else Ranged()
                    for index in range(len(boxes))], ""
        return [Ranged() for _ in boxes], ""

    def lens(self) -> Lens | None:
        return None if self.fail else self._lens

    def power(self) -> Power:
        if self.fail:
            return Power(error=self.fail)
        return Power(state=self.switched)

    def set_power(self, on: bool) -> Power:
        self.switches.append(on)
        if self.fail:
            return Power(error=self.fail)
        # `waking` rather than `on`, because that is what the real one answers
        # and a test written against an instant switch-on is a test that would
        # not have caught the toggle snapping straight to "on" and lying for the
        # seconds the firmware upload takes.
        self.switched = "waking" if on else "off"
        return Power(state=self.switched)


class SidecarRanger(Ranger):
    """The depth service on loopback, over HTTP.

    No exception escapes any call here. A service that is not running, a device
    still uploading its firmware, a connection dropped mid-answer and a reply
    that is not JSON all come back as an empty answer and a sentence saying
    which.
    """

    name = "oak-depth"

    def __init__(self, url: str | None = None,
                 timeout_s: float = TIMEOUT_S) -> None:
        self.url = (url or os.environ.get(ENV_URL) or DEFAULT_URL).rstrip("/")
        parsed = urlparse(self.url)
        self.host = parsed.hostname or "127.0.0.1"
        self.port = parsed.port or 80
        self.timeout_s = timeout_s
        self._lens: Lens | None = None
        self._lens_at = 0.0

    def describe(self) -> str:
        return f"{self.name} at {self.url}"

    def available(self) -> tuple[bool, str]:
        return (True, "") if self.lens() is not None else (
            False, f"the depth camera at {self.url} is not answering, or its "
                   f"stored calibration would not read")

    def lens(self) -> Lens | None:
        """The colour camera's optics, cached, or None if they cannot be had.

        None rather than a raise and rather than a constant: a host that cannot
        reach the camera has no business drawing bearings through it, and the
        store already knows what to do with an observation that has none.
        """
        now = time.monotonic()
        if self._lens is not None and now - self._lens_at < LENS_CACHE_S:
            return self._lens
        payload, error = self._json("GET", "/health", None)
        if error:
            return None
        colour = payload.get("colour") or {}
        got = colour.get("intrinsics") or {}
        try:
            lens = Lens(fx=float(got["fx"]), fy=float(got["fy"]),
                        cx=float(got["cx"]), cy=float(got["cy"]),
                        width=int(got["width"]), height=int(got["height"]),
                        hfov_deg=float(payload.get("hfov_deg") or 0.0),
                        vfov_deg=float(payload.get("vfov_deg") or 0.0))
        except (KeyError, TypeError, ValueError):
            return None
        self._lens, self._lens_at = lens, now
        return lens

    def power(self) -> Power:
        """Whether the camera is switched on, asked of the service each time.

        Not cached, unlike the lens: the lens cannot change while the service
        runs and this is the one thing about it that changes on purpose. It is
        also the answer a person is watching after pressing a switch, so a
        cached one would be a console that had stopped keeping up.
        """
        payload, error = self._json("GET", "/power", None)
        if error:
            return Power(error=error)
        state = str(payload.get("power") or "")
        if state not in ("on", "waking", "off"):
            return Power(error=f"the depth camera answered {state!r}, which is "
                                f"not one of on, waking or off")
        return Power(state=state,
                     since_s=_number(payload.get("since_s"), 0.0) or 0.0)

    def set_power(self, on: bool) -> Power:
        """Switch the camera off or on, and say what that did.

        The service answers this one immediately whichever way it goes -- a wake
        is a firmware upload it does on another thread -- so this is a short call
        even though what it starts is not. The caller finds out how the wake went
        by asking `power` again, which is what the console's poll is for.
        """
        body = json.dumps({"on": bool(on)}).encode()
        payload, error = self._json("POST", "/power", body)
        if error:
            return Power(error=error)
        if not payload.get("ok"):
            return Power(error=str(payload.get("error")
                                   or "the depth camera would not switch"))
        return Power(state=str(payload.get("power") or ""),
                     since_s=_number(payload.get("since_s"), 0.0) or 0.0)

    def frame(self) -> Frame:
        """The newest colour picture, or a sentence saying why not.

        The age and the gap to the depth map ride on the reply's own headers
        rather than in a second call, because a second call would be asking
        about a different frame.
        """
        connection = None
        try:
            connection = http.client.HTTPConnection(self.host, self.port,
                                                    timeout=self.timeout_s)
            connection.request("GET", "/frame")
            reply = connection.getresponse()
            body = reply.read()
            headers = reply.headers
            status = reply.status
        except OSError as error:
            return Frame(ok=False, error=f"{type(error).__name__}: {error}")
        except Exception as error:                     # never past here
            return Frame(ok=False, error=f"{type(error).__name__}: {error}")
        finally:
            if connection is not None:
                connection.close()
        if status != 200 or not body:
            return Frame(ok=False, error=_why(status, body))
        age = _number(headers.get("X-Frame-Age"), 0.0)
        apart = _number(headers.get("X-Depth-Apart"), 0.0)
        width, height = _size_of(headers.get("X-Frame-Size"))
        if age > FRAME_MAX_AGE_S:
            return Frame(ok=False, age_s=age, apart_s=apart,
                         error=f"the depth camera's newest picture is {age:.1f} s "
                               f"old, which is not what it is looking at now")
        if apart > MAX_APART_S:
            return Frame(ok=False, age_s=age, apart_s=apart,
                         error=f"the picture and the depth behind it were taken "
                               f"{apart:.1f} s apart, which is not one look")
        return Frame(ok=True, jpeg=body, width=width, height=height,
                     age_s=age, apart_s=apart, taken_at=time.time() - age)

    def ranges(self, boxes: list[list[float]]) -> tuple[list[Ranged], str]:
        """(one answer per box, error). Never raises, and never short.

        A caller lines these up with the regions it asked about, so a reply that
        is short or long is turned into one that is not: the answers it did give
        are kept in order and the rest abstain. Getting no range is a state every
        consumer already handles, and a misaligned list is one nothing would
        notice.
        """
        if not boxes:
            return [], ""
        body = json.dumps({"boxes": [list(box) for box in boxes]}).encode()
        payload, error = self._json("POST", "/ranges", body)
        if error:
            return [], error
        if not payload.get("ok"):
            return [], str(payload.get("error") or "the depth camera refused")
        answers = []
        given = payload.get("ranges") or []
        for index in range(len(boxes)):
            one = given[index] if index < len(given) else None
            if not isinstance(one, dict):
                answers.append(Ranged())
                continue
            answers.append(Ranged(
                range_m=_number(one.get("range_m"), None),
                sigma_m=_number(one.get("sigma_m"), None),
                valid=_number(one.get("valid"), 0.0) or 0.0,
                pixels=int(one.get("pixels") or 0),
                age_s=_number(payload.get("age_s"), 0.0) or 0.0,
                apart_s=_number(payload.get("depth_apart_s"), 0.0) or 0.0))
        return answers, ""

    # --- the wire -------------------------------------------------------------

    def _json(self, method: str, path: str,
              body: bytes | None) -> tuple[dict[str, Any], str]:
        """(payload, error). Never raises."""
        connection = None
        try:
            connection = http.client.HTTPConnection(self.host, self.port,
                                                    timeout=self.timeout_s)
            headers = {"Content-Type": "application/json"} if body else {}
            connection.request(method, path, body=body, headers=headers)
            reply = connection.getresponse()
            raw = reply.read()
            status = reply.status
        except OSError as error:
            return {}, f"{type(error).__name__}: {error}"
        except Exception as error:                     # never past here
            return {}, f"{type(error).__name__}: {error}"
        finally:
            if connection is not None:
                connection.close()
        try:
            return json.loads(raw.decode("utf-8", "replace")), ""
        except ValueError:
            return {}, (f"the depth camera answered {status} with {len(raw)} "
                        f"bytes that were not JSON")


def _why(status: int, body: bytes) -> str:
    """What a non-200 from the depth service was about, in a sentence."""
    try:
        payload = json.loads(body.decode("utf-8", "replace"))
        if isinstance(payload, dict) and payload.get("error"):
            return str(payload["error"])
    except ValueError:
        pass
    return f"the depth camera answered {status}"


def _number(value: Any, fallback: float | None) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback


def _size_of(header: Any) -> tuple[int, int]:
    """`640x360` as two integers, or zeros if the header was not sent."""
    try:
        width, _, height = str(header).partition("x")
        return int(width), int(height)
    except (AttributeError, TypeError, ValueError):
        return 0, 0


def describe_ranger(ranger: Ranger | None) -> str:
    """What to write in the diagnostics row for this depth backend."""
    if ranger is None:
        return "none"
    describe = getattr(ranger, "describe", None)
    return describe() if callable(describe) else getattr(ranger, "name", "unknown")
