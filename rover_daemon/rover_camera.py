"""Camera, vision POST, face tracking. Mixed into Rover."""
from __future__ import annotations

import base64
import glob
import json
import math
import os
import socket
import sys
import threading
import time
from typing import Any

from rover_util import _number


def default_camera() -> str:
    """The USB camera, not whichever /dev/video0 the kernel numbered first.

    On the Pi 1 the Xitech is video0. On this Allwinner board video0 is the
    cedrus decoder and the same camera lands later. The by-id name is the same
    on both and stays put when another node appears.
    """
    named = sorted(glob.glob("/dev/v4l/by-id/*-video-index0"))
    if named:
        return named[0]
    nodes = []
    for path in glob.glob("/dev/video[0-9]*"):
        try:
            n = int(os.path.basename(path)[5:])
        except ValueError:
            continue
        nodes.append((n, path))
    for _n, path in sorted(nodes):
        name = os.path.basename(path)
        try:
            device = os.path.realpath(f"/sys/class/video4linux/{name}/device")
        except OSError:
            continue
        if "usb" in device:
            return path
    return "/dev/video0"

# How long a "show me somebody else" suppression lasts, and how wide it is in
# multiples of the face's own width. Long enough to let the sweep carry the
# camera off the person it was on, short enough that they are not banished.
SKIP_FOR_S = 6.0
SKIP_RADIUS = 1.5

# How long the picture-taking service gets. A frame is ~35kB and the POST is
# milliseconds on this LAN, but the model host is also the one holding a
# conversation, so this is sized to outlast a busy moment rather than to be
# tight. The voice service's own patience for a tool is 12s and includes this.
VISION_TIMEOUT_S = 6.0
# How long to spend finding out whether the face detector is there before
# starting to track. Short, because this is paid on the way into a tool call the
# model is waiting on, and because the answer on a LAN is immediate either way --
# except when the host is off rather than refusing, which is the case worth
# bounding. See `_detector_ready` for why this check exists at all.
DETECT_PROBE_S = 2.0
# A frame this old is not what the camera is looking at any more. Only reached
# while the tracking loop owns the camera, where the loop's newest frame is used
# rather than opening a second one -- which is impossible anyway.
FRAME_STALE_S = 2.0

# The camera feed is closed this long after the last thing that needed it. Only
# face tracking opens the feed now -- one-shot pictures do not, see `_snapshot` --
# and tracking closes it on its way out, so in ordinary running nothing reaches
# this. It stays as the backstop for the case it was always really for: an open
# camera is a camera nothing else can open, and a feed left behind by a crash
# between `_open_camera` and tracking's own `finally` would otherwise sit there
# taking a quarter of the core off the scan matcher until the daemon restarted.
CAMERA_IDLE_S = 20.0
# How many frames a one-shot picture asks the camera for. Named here as well as in
# track_face_pi because it is in two error messages the model reads out loud.
SNAPSHOT_FRAMES = 3

class VisionLink:
    """The voice service's `/frame`, over one kept-open connection.

    Modelled on `track_face_pi.Detector` rather than on anything new: this
    machine already POSTs a JPEG for every tracked frame, and this is the same
    POST to a different host and port. One request at a time, a stale keep-alive
    costs a retry rather than a picture, and a service that is not there comes
    back as a failure to report rather than an exception to raise -- the model
    has to be told it could not see, in words it can repeat.
    """

    def __init__(self, address: str, timeout: float = VISION_TIMEOUT_S) -> None:
        import http.client

        self._client = http.client
        host, _, port = address.partition(":")
        self.host = host
        self.port = int(port) if port else 8767
        self.timeout = timeout
        self.connection = None
        self._lock = threading.Lock()

    def describe(self) -> str:
        return f"http://{self.host}:{self.port}/frame"

    def post(self, jpeg: bytes) -> dict[str, Any]:
        """Send one frame. Returns the service's answer, or {"ok": false, ...}."""
        with self._lock:
            for attempt in (1, 2):
                try:
                    if self.connection is None:
                        self.connection = self._client.HTTPConnection(
                            self.host, self.port, timeout=self.timeout)
                        self.connection.connect()
                        # As in Detector: headers and body are two writes, and
                        # Nagle can hold the second until the first is acked.
                        self.connection.sock.setsockopt(
                            socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.connection.request(
                        "POST", "/frame", body=jpeg,
                        headers={"Content-Type": "image/jpeg",
                                 "Content-Length": str(len(jpeg))})
                    payload = json.loads(self.connection.getresponse().read())
                except Exception as error:
                    self.close()
                    if attempt == 2:
                        return {"ok": False,
                                "error": f"could not send the picture to {self.describe()}: "
                                         f"{type(error).__name__}: {error}"}
                    continue
                if not isinstance(payload, dict) or not payload.get("image"):
                    return {"ok": False,
                            "error": str(payload.get("error") if isinstance(payload, dict)
                                         else "the vision service gave no answer")}
                return {"ok": True, "image": payload["image"],
                        "w": payload.get("w"), "h": payload.get("h")}
        return {"ok": False, "error": "unreachable"}

    def close(self) -> None:
        if self.connection is not None:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None


# --- the network, read cheaply -----------------------------------------------


def _where(face, width: int, height: int) -> dict[str, Any]:
    """One box described the way a person would say it, not in pixels.

    The model is going to read this out loud, so "on your left, quite close" has
    to be derivable from it without the model doing arithmetic on coordinates --
    which a 4B model at int4 will get wrong, confidently.
    """
    x, y, w, h = face[0], face[1], face[2], face[3]
    centre_x = (x + w / 2) / max(width, 1)
    side = "left" if centre_x < 0.38 else "right" if centre_x > 0.62 else "centre"
    # By apparent width. A face is about 16 cm across, so this is a crude range
    # estimate and is deliberately reported in words rather than metres.
    share = w / max(width, 1)
    distance = "near" if share > 0.22 else "far" if share < 0.10 else "mid"
    return {"where": side, "distance": distance,
            "score": round(float(face[4]), 3) if len(face) > 4 else None}


class RoverCamera:
    """Pictures, the detector, and the tracking loop."""

    # --- the camera ---------------------------------------------------------
    #
    # There are two ways to get a picture here and the difference is not the
    # picture, it is what is still running afterwards.
    #
    # `_open_camera` is the 30 fps feed, and only face tracking uses it: the loop
    # wants every frame it can get, and pays for them. Everything else -- `look`,
    # `camera_jpeg`, `count_faces` -- goes through `_snapshot`, which opens the
    # camera for three frames and closes it. That is not a micro-optimisation. The
    # feed costs this one core about a quarter of the lidar's revolutions, and
    # leaving it warm for CAMERA_IDLE_S meant one photograph degraded the scan
    # matcher for twenty seconds; since the matcher is this rover's only odometer,
    # that is twenty seconds of a drive measuring itself wrong. The numbers are in
    # `track_face_pi.snapshot`, which is also where the capture lives.

    def _open_camera(self):
        """The shared camera feed, opened on demand. Caller holds the lock.

        **Only call this from a thread that will outlive the camera.** v4l2-ctl is
        started with PR_SET_PDEATHSIG, and the kernel counts the *thread* that
        started it as the parent, so a feed opened on a connection thread dies when
        that request finishes. Tracking's loop thread is the one caller that
        qualifies, and it is now the only caller.
        """
        from track_face_pi import Camera

        if self._camera is not None and self._camera.alive():
            self._camera_used = time.monotonic()
            return self._camera
        if self._camera is not None:
            self._camera.close()
        # MJPEG, even though the detector could take raw pixels instead of
        # decoding one. Taking the cheaper conversion made the loop three times
        # *slower* on the Pi 1, and the reason it stays MJPEG on a board that is
        # not that Pi is the second cost below rather than the first: this board
        # decodes a 640x480 frame in 7 ms, so the decode has stopped mattering,
        # and 18 MB/s of uncompressed frames on a bus shared with the wifi
        # adapter and the OAK has not.
        #
        # v4l2-ctl's stdout is a queue, not a slot. Uncompressed 640x480 at 30 fps
        # is 18 MB/s and this host's reader drains about 7, so the pipe stayed full
        # and every fresh frame sat behind stale ones that had to be read first --
        # in full, to be thrown away. Measured live: 390-635 ms per turn waiting
        # for a frame, against 14 ms of actual conversion. MJPEG is 39 kB a frame
        # instead of 614, the reader keeps up at the full 30 fps, and the frame in
        # hand is at most 33 ms old rather than half a second.
        #
        # The decode is not wasted work either: YuNet works at the frame's own 640
        # pixels, so what comes out of libjpeg goes straight to the network with no
        # resize between them. See yunet.py.
        #
        # The 18 MB/s had a second cost that settles the question on its own: it
        # starved the wlan adapter off the same USB controller and took the rover
        # off the network, which is the failure docs/oak-on-the-pi.md feared.
        camera = Camera(self.device, self.size, "MJPG")
        self._camera = camera
        self._camera_used = time.monotonic()
        return camera

    def _snapshot(self, frames: int = SNAPSHOT_FRAMES):
        """A few whole frames from a camera that is shut again straight away.

        The seam the self-test replaces, and the reason it is a method rather than a
        bare import at each call site. It holds the camera and nothing else -- not
        the board lock, no open feed, no thread -- so this is still safe to call
        while the rover is driving, which is the whole point of it.

        **One at a time, because this camera hands the second caller nothing.**
        Two v4l2-ctl processes on it at once is not two pictures: one of them
        exits in about 30 ms having written no bytes and said nothing at all on
        stderr, so what reaches the person at the console is "the camera gave no
        whole picture" with no reason after it. Measured on the rover on
        2026-09-03, over 60 grabs: 46 that had the camera to themselves all came
        back with three frames, and 12 of the 14 that overlapped another grab came
        back empty. The gap between them makes no difference -- back to back is as
        good as a second and a half apart -- so it is the overlap and only the
        overlap.

        It became a daily fault rather than a rare one when the world state
        started looking once a second: the console asks for a frame every two
        seconds and the looking loop asks for one every second, both through here,
        and better than half of the console's pictures were lost to the collision.
        Waiting costs the loser about a third of a second, which is what one grab
        takes.
        """
        from track_face_pi import snapshot

        with self._camera_lock:
            return snapshot(self.device, self.size, frames=frames)

    def _close_camera(self) -> None:
        if self._camera is not None:
            self._camera.close()
            self._camera = None

    def _open_detector(self):
        """The face detector, in this process or over HTTP.

        `--service local` runs YuNet here, on this board's four cores, which is
        both faster than the OAK's VPU was and free of the loopback round trip
        that used to carry every frame to it -- 146 ms against 190, measured.
        Anything host:port keeps the old path, which is how the detector can be
        put back on `face_detect/` on MEDIA without changing anything else.

        Either way the thresholds are `aiming.py`'s, because either way the
        network is YuNet. That was not true while the OAK was here: an SSD scores
        this room's furniture and its people both lower, so it carried its own
        pair and the two could not be mixed up.
        """
        if self._detector is not None:
            return self._detector
        from aiming import ACQUIRE_SCORE

        if self.service == "local":
            from yunet import KEEP_SCORE, LocalDetector

            self._detector = LocalDetector(score=KEEP_SCORE, size=self.size)
        else:
            from track_face_pi import Detector

            self._detector = Detector(self.service)
        self._acquire_score = ACQUIRE_SCORE
        return self._detector

    def _tool_detect_in(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Run the detector over a supplied picture. A diagnostic, not a tool.

        Dispatched like a tool because that is the only protocol this daemon
        speaks, and deliberately absent from :meth:`tools`, so no model is shown
        it -- the same arrangement as `set_vision` above.

        It exists because "does it see me?" and "does it aim at me properly?" are
        different questions that were repeatedly answered as one. The camera is
        moving, the frame rate is low and the boxes are gone by the time anybody
        looks, so a claim about detection made from a live run is a claim about a
        picture nobody kept. This takes a picture that *was* kept -- one a person
        has looked at and can point to the face in -- and says what the detector
        makes of it, as many times as you like, with the answer checkable against
        the image it came from.
        """
        blob = arguments.get("jpeg_base64")
        if not isinstance(blob, str):
            return {"ok": False, "error": "jpeg_base64 must be a base64 string"}
        try:
            jpeg = base64.b64decode(blob, validate=True)
        except Exception as error:
            return {"ok": False, "error": f"not base64: {error}"}
        with self._detector_lock:
            detector = self._open_detector()
            faces = detector.detect(jpeg, time.monotonic())
        if faces is None:
            return {"ok": False, "error": "the detector gave no answer"}
        return {"ok": True, "count": len(faces),
                # x, y, w, h, score -- in the picture's own pixels
                "faces": [[round(v, 1) for v in face] for face in faces],
                "acquire_at": getattr(self, "_acquire_score", None)}

    def _tool_set_vision(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Point `look` at whoever is asking. A control call, not a model tool.

        It is dispatched like a tool because that is the only protocol this
        daemon speaks, and it is deliberately absent from :meth:`tools`, so no
        model is ever shown it or can call it.

        It exists because the picture's destination was a constant, and a
        constant was wrong. `look` posts the JPEG straight to the model's host
        rather than passing it back through the client, which is what keeps a
        35kB frame off a desk that only has a microphone on it -- but it means
        this daemon has to be told an address, and it was told one at startup, by
        whoever last edited a crontab. When the model moved off that host the
        pictures kept going to it, and `look` failed with "No route to host"
        while everything else on the rover worked perfectly.

        So the client that is about to hold a conversation says where it is
        listening, every time it connects. The address it gives is the one its
        own socket to this daemon is bound to, so it is right by construction on
        a rover that has moved between eth0 and wlan0.

        Naming no address switches the picture path off, which also withdraws
        `look` from the tool list -- a tool that cannot reach the model's host is
        worse than a missing one.
        """
        address = arguments.get("address")
        if address is None or (isinstance(address, str) and not address.strip()):
            was, self.vision = self.vision, None
            if was is not None:
                was.close()
            return {"ok": True, "vision": None, "tools": [t["function"]["name"]
                                                          for t in self.tools()]}
        if not isinstance(address, str):
            return {"ok": False, "error": "set_vision wants an address like host:port"}
        host, _, port = address.strip().partition(":")
        if not host or (port and not port.isdigit()):
            return {"ok": False, "error": f"{address!r} is not a host:port"}
        link = VisionLink(address.strip())
        was, self.vision = self.vision, link
        if was is not None and was is not link:
            was.close()
        print(f"[rover] pictures now go to {link.describe()}", flush=True)
        return {"ok": True, "vision": link.describe(),
                "tools": [t["function"]["name"] for t in self.tools()]}

    def _tool_look_at(self, arguments: dict[str, Any]) -> dict[str, Any]:
        from aiming import PAN_LIMIT, TILT_LIMITS

        stopped = self.stop_tracking()
        with self._lock:
            if "pan" in arguments and arguments["pan"] is not None:
                self.pan = min(max(_number(arguments["pan"], "pan"), -PAN_LIMIT), PAN_LIMIT)
            if "tilt" in arguments and arguments["tilt"] is not None:
                self.tilt = min(max(_number(arguments["tilt"], "tilt"),
                                    TILT_LIMITS[0]), TILT_LIMITS[1])
            if not self._send_gimbal():
                return {"ok": False, "error": "the driver board did not answer"}
            return {"ok": True, "pan": round(self.pan), "tilt": round(self.tilt),
                    "stopped_tracking": stopped}

    def _tool_center_camera(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        stopped = self.stop_tracking()
        if not self.centre_gimbal():
            return {"ok": False, "error": "the driver board did not answer"}
        # Read back rather than written down: rest is not level any more, and a
        # second copy of that number here is a second place for it to be wrong.
        return {"ok": True, "pan": round(self.pan), "tilt": round(self.tilt),
                "stopped_tracking": stopped}

    def _tool_count_faces(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        if self.device is None:
            return {"ok": False, "error": "this rover has no camera attached"}
        # While the loop is running it owns the camera, so the honest answer is
        # what the loop last saw rather than a second opinion nobody can take.
        if self._tracking.is_set():
            seen = dict(self._seen)
            age = time.monotonic() - seen["at"] if seen["at"] else None
            if age is None or age > 2.0:
                return {"ok": False, "error": "tracking is running but has not seen a frame yet"}
            return {"ok": True, "count": seen["faces"], "faces": seen["where"],
                    "from": "the tracking loop"}
        # Deliberately outside the lock, and without opening the feed: a capture
        # holds nothing, so counting faces cannot delay a stop or a gimbal command
        # the way waiting on a camera under the lock could.
        got, why = self._snapshot()
        if not got:
            return {"ok": False, "error": f"the camera gave nothing: {why}"}
        # Newest first. More than one frame is worth having because the detector
        # rejecting a frame arrives here as the same None a dead service gives, so
        # a single undecodable picture used to read as "the host is away" -- but the
        # old reason for it has gone: these frames are whole buffers rather than
        # whatever a reader starting mid-stream happened to find, so the fragment
        # that made a cold `count_faces` fail every time cannot occur here.
        faces = None
        with self._detector_lock:
            detector = self._open_detector()
            for jpeg, at in reversed(got):
                faces = detector.detect(jpeg, at)
                if faces is not None:
                    break
        if faces is None:
            return {"ok": False,
                    "error": f"the face detector did not answer, or "
                             f"rejected {len(got)} frames running"}
        width, height = self.size
        where = [_where(face, width, height) for face in faces]
        return {"ok": True, "count": len(faces), "faces": where}

    def _whole_jpeg(self) -> tuple[bytes | None, str]:
        """One complete frame from the camera, or (None, why).

        Complete is checked rather than assumed, and it is still worth checking even
        though the capture below now hands back whole buffers: a picture is about to
        cross a network and be looked at by a model, and two bytes is a cheaper way
        to find out it is a fragment than either of those. Nothing is decoded on
        this machine; that costs 93 ms here and the picture is not for us.

        Newest of the three, because that is the frame the camera settled on -- see
        SNAPSHOT_FRAMES for why a cold camera's first frame is not the one to send.
        """
        if self._tracking.is_set():
            # The loop has the camera. Nothing else can open it, so the honest
            # answer is the newest frame the loop has seen -- which is also the
            # frame the camera is actually pointing at.
            frame = self._frame
            if frame is None:
                return None, "tracking is running but has not seen a frame yet"
            jpeg, at = frame
            if time.monotonic() - at > FRAME_STALE_S:
                return None, "tracking is running but its last frame is stale"
            # Every frame the loop holds is already a picture, because the camera
            # is opened in MJPEG. The encode stays as the path for a frame that is
            # not -- a detector taking raw pixels would leave one here -- and it is
            # only ever reached when somebody asks to see something.
            if not jpeg.startswith(b"\xff\xd8"):
                encoder = self._detector
                jpeg = encoder.encode_jpeg(jpeg) if encoder is not None else None
                if jpeg is None:
                    return None, "the frame could not be turned into a picture"
            return jpeg, ""
        got, why = self._snapshot()
        if not got:
            return None, f"the camera gave nothing: {why}"
        for jpeg, _at in reversed(got):
            if jpeg.startswith(b"\xff\xd8"):
                return jpeg, ""
        return None, (f"the camera gave {len(got)} frames running that were not "
                      f"whole pictures")

    def _tool_look(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        if self.vision is None:
            return {"ok": False, "error": "this rover cannot show you a picture"}
        jpeg, why = self._whole_jpeg()
        if jpeg is None:
            return {"ok": False, "error": why}
        # The picture goes straight to the model's host; what comes back is the
        # name it was filed under, and that name is the whole of this result.
        # The client between here and there never sees the frame.
        sent = self.vision.post(jpeg)
        if not sent.get("ok"):
            return {"ok": False, "error": sent.get("error", "the picture was not accepted")}
        # Nothing but the name. This result carried a note once -- "the picture
        # is in front of you; describe what is actually in it" -- and that one
        # sentence, arriving immediately before the picture on every single
        # look, was read as an instruction for the turn: the model described the
        # whole picture whatever it had been asked, and took a fresh one for
        # every follow-up, 3/3 against 0/3 with the note removed. A tool result
        # is context, and context that reads like an order is an order. What the
        # model should do with a picture belongs in the system prompt, where it
        # is said once. See voice_chat/README.md.
        return {"ok": True, "image": sent["image"]}

    def _tool_camera_jpeg(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """One frame, as base64 JPEG in the reply. A control call, not a model tool.

        `look` is the model's version and posts the picture to the model's host,
        because a tool result cannot carry an image into a conversation. A window on
        a desk has no such problem, and routing a frame through a frame server to get
        it onto the screen of the machine that asked for it would be silly -- the
        same argument that gives `map_png` its own existence beside `show_map`.

        It needs a camera and not a vision host, which is the practical difference:
        a daemon started without `--vision` cannot `look` at all, and can still be
        asked for a picture from here.

        The bytes are the camera's own, undecoded. There is no image library on this
        Pi -- see [face_tracking/track_face_pi.py](../face_tracking/track_face_pi.py),
        where v4l2-ctl does the capturing precisely because of that -- so JPEG is the
        only thing this end can send, and turning it into something a widget can show
        is the caller's problem. `_whole_jpeg` still checks it is a whole picture and
        not the tail of one, which costs two bytes rather than the 93 ms a decode
        would.
        """
        if self.device is None:
            return {"ok": False, "error": "this rover has no camera attached"}
        jpeg, why = self._whole_jpeg()
        if jpeg is None:
            return {"ok": False, "error": why}
        width, height = self.size
        return {"ok": True, "bytes": len(jpeg), "width": width, "height": height,
                # Which of the two paths it came off, because they mean different
                # things: the loop's newest frame is what the camera is pointing at
                # while it sweeps, and a fresh grab is a camera opened for this call.
                "live": self._tracking.is_set(),
                "pan": round(self.pan), "tilt": round(self.tilt),
                "jpeg_base64": base64.b64encode(jpeg).decode("ascii")}

    def _detector_ready(self) -> str:
        """Empty if the face detector answers, otherwise why not, in a sentence.

        Face tracking needs the detector, and its being away is still an expected
        state even now that it runs in this process: OpenCV is unpacked onto this
        board by a deploy step rather than installed by a package manager, and a
        rover that has been rebuilt without it can see nothing at all. The loop is
        written to hold still through a detector that is not answering rather than
        to die. That is right for a loop already running and wrong for one being
        started: the loop starts, holds still, reports itself as tracking, and the
        model says "I started tracking people" while the camera never moves. That
        is the failure this whole directory's prompt wording exists to prevent,
        arriving from underneath the prompt.

        A refusal is instant and a host that is off takes the timeout, which is
        why the socket case is bounded rather than left to the first detect call.

        For the detector in this process there is no socket to probe, so the
        readiness question is whether the library and the model load -- which is
        the same question, and answering it here means the load is paid once, on
        the first call, rather than being mistaken for a camera that will not
        answer.
        """
        if self.service == "local":
            try:
                self._open_detector()
                return ""
            except Exception as error:
                return (f"the face detector could not be loaded ({error}), so "
                        f"tracking a face is not possible right now")
        host, _, port = self.service.partition(":")
        try:
            with socket.create_connection((host, int(port or 8768)), DETECT_PROBE_S):
                return ""
        except OSError as error:
            return (f"the face detector at {self.service} is not answering "
                    f"({error.strerror or type(error).__name__}), so tracking a "
                    f"face is not possible right now")

    def _tool_start_tracking(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        if self.device is None:
            return {"ok": False, "error": "this rover has no camera attached"}
        if self._tracking.is_set():
            return {"ok": True, "tracking": True, "already": True}
        if (why := self._detector_ready()):
            return {"ok": False, "error": why}
        self._tracking.set()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return {"ok": True, "tracking": True}

    def _tool_stop_tracking(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        was = self.stop_tracking()
        return {"ok": True, "tracking": False, "was_tracking": was}

    def _tool_track_next(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        started = False
        if not self._tracking.is_set():
            result = self._tool_start_tracking({})
            if not result.get("ok"):
                return result
            started = True
        with self._lock:
            centre = self._seen.get("centre")
            if centre is None and not started:
                return {"ok": True, "skipped": False,
                        "note": "nobody is being followed at the moment, so there is "
                                "nobody to let go of"}
            self._skip_centre = centre
            self._skip_until = time.monotonic() + SKIP_FOR_S
        return {"ok": True, "skipped": True, "started": started,
                "note": "the rover cannot tell people apart, so it may find the same "
                        "person again if nobody else is in view"}

    def _tool_tracking_status(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        seen = dict(self._seen)
        fresh = seen["at"] and time.monotonic() - seen["at"] < 2.0
        status = {
            "ok": True,
            "tracking": self._tracking.is_set(),
            "following_someone": bool(seen["locked"]) if fresh else False,
            "faces_in_view": seen["faces"] if fresh else 0,
            # What the camera is doing with nobody to follow: sweeping the room, or
            # holding still and watching where the rover is driving. Absent while
            # somebody is being followed, and while the loop is still inside the
            # couple of seconds it gives a lost face before it does either.
            "searching": seen.get("searching") if fresh else None,
            "pan": round(self.pan), "tilt": round(self.tilt),
        }
        # How fast the loop is actually going round, which is the difference
        # between "it cannot see me" and "it sees me twice a second". Only the
        # in-process detector keeps these; over HTTP the service's own /health
        # has them instead.
        detector = self._detector
        if detector is not None and hasattr(detector, "convert_ms"):
            status["loop_fps"] = round(self._loop_fps, 1)
            status["frame_ms"] = {"convert": round(detector.convert_ms, 1),
                                  "detect": round(detector.detect_ms, 1),
                                  "convert_busy": round(detector.convert_busy_ms, 1),
                                  "detect_busy": round(detector.detect_busy_ms, 1),
                                  "wait": round(self._wait_ms, 1),
                                  "aim": round(self._aim_ms, 1)}
            # Frames whose exposure stamp could not be matched to them, against
            # frames captured and thrown away. An unpaired frame is not a dropped
            # one: it still gets used, but with a *guessed* exposure time, and the
            # whole dead-time compensation is built on that stamp being real.
            camera = self._camera
            if camera is not None:
                status["frames"] = {"unpaired": camera.unpaired,
                                    "dropped": camera.dropped}
            status["acquire_at"] = self._acquire_score
            status["recent_scores"] = list(detector.recent)
            if self._aim:
                status["aim"] = list(self._aim)
        return status

    # --- the loop -----------------------------------------------------------

    def stop_tracking(self) -> bool:
        """Stop the loop and wait for it to let go. True if it had been running."""
        if not self._tracking.is_set():
            return False
        self._tracking.clear()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=5.0)
        self.centre_gimbal()
        return True

    def _searching(self, current, gimbal):
        """What to do with the camera when there is nobody to look at.

        Two answers, and which one is right depends on whether the wheels are
        turning. Parked, the rover sweeps: it has nothing else to do and somebody
        walking into the room is only found by looking for them. Driving, it holds
        still and looks where it is going -- see `Ahead`, which is where that is
        argued -- and picks the sweep up again when it stops.

        Asked once a frame rather than switched by the navigator when a move
        begins, so that a move which ends between two frames is noticed by the
        thing that cares, and so that neither driving nor tracking has to know the
        other is running. Whichever behaviour is already in hand is kept, because
        both carry the state of a move in progress -- which way the sweep was
        going, how far the camera still has to turn -- and rebuilding one every
        frame would leave the camera starting its journey over and over.
        """
        from aiming import Ahead, Scan

        wanted = Ahead if self.driving else Scan
        return current if isinstance(current, wanted) else wanted(gimbal)

    def _loop(self) -> None:
        """The face-tracking control loop, from track_face_pi.py's main().

        Deliberately the same `aiming.py` as the standalone script rather than a
        second control law: two implementations of how this rover aims would
        become two different robots, which is the reason aiming.py exists.
        """
        from aiming import (
            GAIN, GRACE_FRAMES, LOST_GRACE_S, MAX_DT, SCAN_AFTER_S, Gimbal, Target,
            clamp, scan_rate_for,
        )

        width, height = self.size
        try:
            # The feed under the same lock a one-shot grab takes, and for the
            # same reason: `_tracking` is set before this thread runs, so a
            # picture that was already in flight when tracking started would
            # otherwise still be holding the camera as the feed opens on it --
            # and a feed that opens on a busy camera comes up with no frames at
            # all while `start_tracking` has already answered "ok".
            with self._camera_lock, self._lock:
                camera = self._open_camera()
            detector = self._open_detector()
        except Exception as error:
            print(f"[rover] cannot start tracking: {error}", file=sys.stderr, flush=True)
            self._tracking.clear()
            return

        gimbal = Gimbal(clamp(GAIN, 0.05, 1.0), self.size)
        # The angles are a model; this is what makes the model true. Start it
        # from wherever the camera actually is rather than assuming centre -- and
        # seed the history with it too, or was_at() will answer the first frames
        # with zero. See Gimbal.begin().
        gimbal.begin(time.monotonic(), self.pan, self.tilt)
        target = Target(self._acquire_score)
        search = None
        last_tick = time.monotonic()
        self._loop_fps = 0.0
        self._wait_ms = self._aim_ms = 0.0
        self._aim = []
        service_ok_at = time.monotonic()
        stalled = False

        try:
            while self._tracking.is_set():
                waited = time.monotonic()
                got = camera.latest(timeout=1.0)
                now = time.monotonic()
                self._wait_ms += 0.2 * ((now - waited) * 1e3 - self._wait_ms)
                if got is None:
                    if not camera.alive():
                        why = "; ".join(camera.complaints) or "it stopped without saying why"
                        print(f"[rover] the camera stopped: {why}", file=sys.stderr, flush=True)
                        break
                    continue
                frame, exposed_at = got
                # Kept before the detector is consulted rather than after, so
                # that `look` still has a picture during the seconds when the
                # detector is not answering and this loop is only holding still.
                self._frame = (frame, now)
                # Clamped: a frame that took a second to arrive must not be
                # answered with a second's worth of sweep.
                elapsed = now - last_tick
                dt, last_tick = min(elapsed, MAX_DT), now
                # Smoothed from the *unclamped* elapsed time. Using dt here would
                # cap the reported rate at 1/MAX_DT, so a loop running at 1.6 fps
                # reported a steady 4.0 -- a floor reading as a measurement, and
                # it hid the slowdown it was put there to show.
                if elapsed > 0:
                    self._loop_fps += 0.2 * (1.0 / elapsed - self._loop_fps)
                # Hold a lock for whichever is longer, the measured 0.7 s or four
                # frames. This loop runs at four frames a second with the detector
                # on the rover, where 0.7 s is not "a frame or two" but two and a
                # half, and one turn of a head then drops somebody the camera is
                # still pointing straight at.
                target.grace = max(LOST_GRACE_S, GRACE_FRAMES * dt)

                faces = detector.detect(frame, exposed_at)
                if faces is None:
                    if now - service_ok_at > 3.0 and not stalled:
                        print("[rover] the face detector is not answering; holding still",
                              file=sys.stderr, flush=True)
                        target.drop()
                        search = None
                        stalled = True
                    continue
                if stalled:
                    stalled, last_tick, dt = False, now, 0.0
                service_ok_at = now

                # "Show me somebody else": drop detections near whoever was being
                # followed, so Target acquires the next largest face instead. Done
                # here rather than in aiming.py, which is shared with the desktop
                # script and has no business knowing about conversations.
                visible = faces
                if self._skip_centre is not None and now < self._skip_until:
                    x0, y0 = self._skip_centre
                    visible = [
                        face for face in faces
                        if math.hypot(face[0] + face[2] / 2 - x0,
                                      face[1] + face[3] / 2 - y0)
                        >= max(face[2], 60) * SKIP_RADIUS
                    ]
                elif self._skip_centre is not None:
                    self._skip_centre = None

                tracking = target.update(visible, now)
                if tracking and not target.fresh:
                    # The lock is being held open on grace, with no detection of
                    # its own this frame. Carry on to the angle already worked
                    # out; re-reading the remembered pixel would apply the same
                    # correction again -- see Gimbal.keep_going().
                    search = None
                    gimbal.keep_going(dt)
                elif tracking:
                    search = None
                    # Positive x is right of centre and positive y is *above* it,
                    # which is not the picture's own row order.
                    error_x = (target.centre[0] - width / 2) / (width / 2)
                    error_y = (height / 2 - target.centre[1]) / (height / 2)
                    gimbal.track(error_x, error_y, dt, now, exposed_at=exposed_at)
                    # The pixel the angles were computed from, beside the angles.
                    # Kept together because the fault being looked for is a
                    # disagreement between them, and either alone looks reasonable.
                    step = dict(gimbal.last)
                    step["face_px"] = [round(target.centre[0]), round(target.centre[1])]
                    step["frame"] = [width, height]
                    self._aim.append(step)
                    del self._aim[:-10]
                else:
                    if target.centre is not None:
                        target.drop()
                        gimbal.forget()
                    if now - target.seen_at > SCAN_AFTER_S:
                        search = self._searching(search, gimbal)
                        search.step(gimbal, scan_rate_for(dt, gimbal.pan_gain), dt)
                gimbal.record(now)

                aimed = time.monotonic()
                with self._lock:
                    if gimbal.changed():
                        self.pan, self.tilt = gimbal.pan, gimbal.tilt
                        self._send_gimbal()
                    self._seen = {
                        "faces": len(faces),
                        "locked": tracking,
                        "at": now,
                        "centre": target.centre if tracking else None,
                        "where": [_where(face, width, height) for face in faces],
                        # Sweeping and watching the road ahead look identical from
                        # outside -- a camera pointing somewhere, with nobody in
                        # front of it -- and the console said "sweeping" through
                        # both. In its own words, so that one place decides.
                        "searching": None if search is None else search.state(),
                    }
                self._aim_ms += 0.2 * ((time.monotonic() - aimed) * 1e3 - self._aim_ms)
        finally:
            self._tracking.clear()
            self._seen = {"faces": 0, "locked": False, "at": 0.0, "where": []}
            self._frame = None
            with self._lock:
                self._close_camera()

    def idle_tick(self) -> None:
        """Let go of the camera when nothing has wanted it for a while."""
        with self._lock:
            if (self._camera is not None and not self._tracking.is_set()
                    and time.monotonic() - self._camera_used > CAMERA_IDLE_S):
                self._close_camera()

