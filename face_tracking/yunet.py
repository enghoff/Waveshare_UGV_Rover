"""YuNet in the rover's own process: the detector the OAK used to be.

The rover ran its face detector on the OAK camera's Myriad X for as long as its
host was a Pi 1, because a 700 MHz ARM11 with no NEON cannot run a CNN at any
useful rate -- `oak_detect/` in this repo's history is that arrangement. Every
host since has been enough to run YuNet here and beat the inference stick doing
it: first a Banana Pi M4 Zero, four Cortex-A53 cores at 1.416 GHz with NEON,
which is the board the figures below were taken on, and since 2026-08-31 the
Jetson Orin Nano the rover carries now.

Measured 2026-08-23 on the rover, on one 640x480 frame from its own camera, with
the daemon and the lidar's scan matcher running as usual:

    OpenCV's YuNet, 640x480         1 thread 310 ms   2 threads 180   4 threads 146
                    320x240                   88               49                38
    the OAK on the same frame       89.4 ms of inference, 100 ms of loopback HTTP
    decoding that frame             7.0 ms whole, 3.8 ms at libjpeg's 1/2 scale

So the CPU beats the VPU outright, and the HTTP round trip that used to carry the
frame to it disappears as well. There is nothing left on this board to offload to:
the H618 has no NPU, its Mali-G31 is `disabled` in this board's device tree and
has no OpenCL driver in any case, and its video engine decodes H.264 and HEVC but
not JPEG. NEON is the acceleration, OpenCV's own kernels already use it, and the
same graph under ONNX Runtime 1.29 measured slower (237 ms against 185 at
640x640), so this is what the board can do.

**The detector's working size is the frame's own, and that is deliberate.**
`aiming.DETECT_WIDTH` is 640 and the camera captures 640x480, so the picture goes
to the network unscaled and `ACQUIRE_SCORE` and `KEEP_SCORE` mean here exactly
what they meant when they were measured -- which is the property this module is
most at risk of quietly breaking. Detecting on a half-scale copy would cost 38 ms
instead of 146 and is genuinely tempting at 20 frames a second; it also changes
how a network scores a face and a sofa, and this repository has already spent an
afternoon on a detector swapped in under thresholds measured for another one. If
that trade is wanted, measure the pair again first, the way `aiming.py` says.

`LocalDetector` is shaped like `track_face_pi.Detector` and like the OAK's
detector before it -- same `detect(frame, exposed_at)`, same `None` for "no
answer", same `rtt_ms` and `detect_ms` -- so the tracking loop cannot tell them
apart and did not change to take this.

**OpenCV is not packaged for this board's Python.** There is no `python3-opencv`
installed, no `pip`, and `sudo` here wants a password no script has, so the wheel
is unpacked beside this file by `install_opencv.sh` and found on `sys.path` from
there. That is a deploy step and not a build one; see the script.
"""

from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path

from aiming import ACQUIRE_SCORE, DETECT_WIDTH, KEEP_SCORE, NMS_THRESHOLD

__all__ = ["ACQUIRE_SCORE", "KEEP_SCORE", "LocalDetector", "YuNetError"]

HERE = Path(__file__).resolve().parent
# Where install_opencv.sh unpacks the wheel. Beside this file, because on the
# rover this file is in ~/ugv and so is the vendor directory; on a desk the
# import below finds a real installation first and this never comes up.
VENDOR = HERE / "vendor"
DEFAULT_MODEL = HERE / "face_detection_yunet.onnx"

# How many of the four cores the network may have. The whole board is 146 ms a
# frame and three quarters of it is 160 -- 4% slower for a core left to the scan
# matcher, which is the trade that matters on a rover whose only odometer is the
# lidar. Tracking runs while the wheels turn, so the core this leaves free is the
# one the rover is navigating on and not merely mapping on: a matcher that drops
# revolutions while the rover watches somebody is one that has to be re-convinced
# where it is, in the middle of a move.
#
# It is a process-wide setting in OpenCV, not a per-detector one, which is why it
# is set once here and named as a constant rather than passed about.
THREADS = 3
# The detector is created at the first frame's size rather than at a size given
# ahead of time, because the frame is what it is. YuNet takes top_k boxes before
# non-maximum suppression; 5000 is OpenCV's own default and costs nothing on a
# frame with two faces in it.
TOP_K = 5000
# libjpeg-turbo scales while it decodes, but only by these fractions, and landing
# on the detector's width by decoding smaller is free where resizing afterwards
# is not. 1/1 is the case on this rover and the reason the dict has an entry for
# it: it keeps the arithmetic in one place instead of a special case.
DECODE_SCALES = {1: 0, 2: 1, 4: 2, 8: 3}


class YuNetError(RuntimeError):
    """OpenCV or the model is not here, said in a sentence a tool can read out."""


def _import_cv2():
    """OpenCV, from wherever this host keeps it.

    Tried as an ordinary import first, so that a desk with it installed properly
    is not affected by the rover's arrangement, and only then from the unpacked
    wheel. The failure is turned into `YuNetError` because the tracking tools
    report a missing detector to whoever asked to track a face, and
    "ModuleNotFoundError: cv2" is not something to say out loud.
    """
    try:
        import cv2

        return cv2
    except ImportError:
        pass
    if VENDOR.is_dir() and str(VENDOR) not in sys.path:
        sys.path.insert(0, str(VENDOR))
        try:
            import cv2

            return cv2
        except ImportError as error:
            raise YuNetError(
                f"OpenCV is unpacked at {VENDOR} but will not import ({error})"
            ) from None
    raise YuNetError(
        "OpenCV is not installed on this host, so faces cannot be detected here; "
        f"run install_opencv.sh to unpack it into {VENDOR}")


class LocalDetector:
    """YuNet, held ready for the life of the process.

    Frames arrive as JPEG -- the camera is opened in MJPEG for the reasons in
    `rover_camera._open_camera`, and this is the one place on the rover that
    decodes one. Boxes come back in the *frame's* pixels, not the network's,
    which matters on any host that detects on a reduced copy.
    """

    def __init__(self, model: str | os.PathLike | None = None,
                 score: float = KEEP_SCORE, size=(640, 480),
                 width: int = DETECT_WIDTH, threads: int = THREADS,
                 quality: int = 80) -> None:
        self.cv2 = _import_cv2()
        import numpy

        self.numpy = numpy
        self.model = Path(model) if model is not None else DEFAULT_MODEL
        if not self.model.is_file():
            raise YuNetError(f"the face detector's model file is missing: {self.model}")
        self.score = score
        self.size = size
        self.width = width
        self.quality = quality
        self.threads = threads
        self.cv2.setNumThreads(threads)
        self.rtt_ms = 0.0
        self.detect_ms = 0.0
        self.convert_ms = 0.0
        # The same two spans as this thread's own CPU time rather than as elapsed
        # time. The gap between each pair is the diagnostic and it is worth
        # keeping permanently: both spans are C with the GIL dropped, so a wall
        # time far above the busy time is not slow work but a thread that was
        # ready and not allowed to run. The two faults look identical from
        # outside and have opposite fixes -- do less work, or stop competing.
        self.convert_busy_ms = 0.0
        self.detect_busy_ms = 0.0
        self.frames = 0
        self.errors = 0
        # The top score of each of the last few frames. Cheap, and the only way
        # to tell "it cannot see me" from "it sees me and the bar is too high".
        self.recent: list[float] = []
        self._lock = threading.Lock()
        self._detector = None
        self._detector_size: tuple[int, int] | None = None

    def describe(self) -> str:
        return (f"YuNet in this process, {self.width}px wide, "
                f"{self.threads} of 4 threads")

    def _for_size(self, width: int, height: int):
        """The detector, made or re-made for this frame's size.

        YuNet is created with an input size and `setInputSize` moves it, which is
        cheap; making one per frame is not. A rover changing capture mode mid-run
        is not a thing that happens, so this ordinarily runs once.
        """
        if self._detector is None:
            self._detector = self.cv2.FaceDetectorYN_create(
                str(self.model), "", (width, height), self.score,
                NMS_THRESHOLD, TOP_K)
        elif self._detector_size != (width, height):
            self._detector.setInputSize((width, height))
        self._detector_size = (width, height)
        return self._detector

    def _decode(self, frame: bytes):
        """One JPEG as BGR pixels at the detector's width, or None.

        Scaled by the decoder where the ratio is one libjpeg understands, since
        that costs nothing, and by a resize afterwards where it is not. On this
        rover neither happens: the camera's 640 is the detector's 640.
        """
        cv2 = self.cv2
        buffer = self.numpy.frombuffer(frame, self.numpy.uint8)
        ratio = max(1, round(self.size[0] / self.width)) if self.width else 1
        flag = DECODE_SCALES.get(ratio)
        if flag:
            image = cv2.imdecode(buffer, [cv2.IMREAD_REDUCED_COLOR_2,
                                          cv2.IMREAD_REDUCED_COLOR_4,
                                          cv2.IMREAD_REDUCED_COLOR_8][flag - 1])
        else:
            image = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if image is None:
            return None
        if image.shape[1] > self.width:
            height = round(image.shape[0] * self.width / image.shape[1])
            image = cv2.resize(image, (self.width, height),
                               interpolation=cv2.INTER_AREA)
        return image

    def detect(self, frame: bytes, exposed_at: float | None = None):
        """Faces for this frame, or None if it could not be asked at all.

        `exposed_at` is accepted and ignored, as in the OAK's detector: the stamp
        exists in the HTTP one because it has to survive a round trip and come
        back attached to the right frame, and here the answer cannot belong to
        any frame but this one.

        A frame that will not decode is an empty list rather than None. The
        difference is what the tracking loop does with it: no faces is a fact
        about the room and it aims accordingly, None is "the detector is not
        answering" and it holds still.
        """
        with self._lock:
            started, busy_started = time.monotonic(), time.thread_time()
            try:
                image = self._decode(frame)
                if image is None:
                    self.errors += 1
                    return []
                converted, busy_converted = time.monotonic(), time.thread_time()
                height, width = image.shape[:2]
                _count, raw = self._for_size(width, height).detect(image)
            except Exception:
                # Nothing here can go away the way a USB device can, so this is a
                # bug rather than an absence -- but the loop is the wrong place to
                # discover that, and it already knows how to hold still.
                self.errors += 1
                return None
            finished, busy_finished = time.monotonic(), time.thread_time()

        self.convert_ms = (converted - started) * 1e3
        self.detect_ms = (finished - converted) * 1e3
        self.rtt_ms = (finished - started) * 1e3
        self.convert_busy_ms = (busy_converted - busy_started) * 1e3
        self.detect_busy_ms = (busy_finished - busy_converted) * 1e3
        self.frames += 1
        faces = self._boxes(raw, width)
        self.recent.append(round(max((f[4] for f in faces), default=0.0), 3))
        del self.recent[:-40]
        return faces

    def _boxes(self, raw, detected_width: int):
        """YuNet's rows as (x, y, w, h, score) in the *frame's* own pixels.

        Each row is a box, five landmarks and a score. The landmarks are dropped:
        aiming.py wants a rectangle, and the eyes and mouth would be one more
        thing for the two callers of this to disagree about.
        """
        if raw is None:
            return []
        scale = self.size[0] / detected_width if detected_width else 1.0
        faces = []
        for row in raw:
            x, y, w, h = (float(v) * scale for v in row[:4])
            faces.append([round(x, 1), round(y, 1), round(w, 1), round(h, 1),
                          round(float(row[14]), 4)])
        return faces

    def encode_jpeg(self, frame: bytes) -> bytes:
        """A frame as JPEG, for the paths that have to show a picture.

        Kept because the OAK's detector had it and `rover_camera._whole_jpeg`
        calls it when a frame is not already a picture. Here a frame always is
        one -- the camera is opened in MJPEG -- so this is the identity, and the
        encode is only reached by a caller holding raw pixels from somewhere else.
        """
        if frame[:2] == b"\xff\xd8":
            return frame
        with self._lock:
            image = self.numpy.frombuffer(frame, self.numpy.uint8)
            width, height = self.size
            if image.size == width * height * 2:
                image = self.cv2.cvtColor(image.reshape(height, width, 2),
                                          self.cv2.COLOR_YUV2BGR_YUYV)
            else:
                return frame
            ok, encoded = self.cv2.imencode(
                ".jpg", image, [int(self.cv2.IMWRITE_JPEG_QUALITY), self.quality])
            return encoded.tobytes() if ok else frame

    def close(self) -> None:
        """Let the network go. Nothing is held open, so this is bookkeeping."""
        with self._lock:
            self._detector = None
            self._detector_size = None
