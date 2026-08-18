"""The OAK in the caller's own process, with no HTTP and no JPEG.

`server.py` exists because the detector used to be on another machine and the
protocol was already written. On the rover that protocol costs two things that
are pure waste: a loopback round trip through Python's HTTP stack, measured at
**41 ms a frame** on this host under load, and a JPEG that only exists to be
decoded again at the other end, at **85 ms**. Between them they were most of the
frame time, and the inference was the cheapest part of the loop.

So this is the same detector reached directly, and the boxes come back in the
same process. Measured on the rover, per frame, as elapsed time and as the
calling thread's own CPU time:

    convert, from YUYV      25.7 ms elapsed     14 ms CPU
    convert, from JPEG      85 ms elapsed
    infer                   49.7 ms elapsed      4 ms CPU
    both, from YUYV         73.2 ms             (against ~190 ms through HTTP)

The two CPU figures are the useful ones and they say the detector is nearly free:
18 ms of core per frame, the rest of it blocked on USB while the Myriad works.

**Which is why the rover feeds this JPEG rather than YUYV**, despite the 85 ms.
Taking the cheaper conversion made the loop three times slower, because 18 MB/s of
uncompressed frames overran the reader and the loop spent 390-635 ms a turn waiting
for one that was not stale -- see `_open_camera` in rover_daemon.py. At 640x480 the
decode scales 2:1 as it goes and lands exactly on the graph's input, so the resize
that would follow it does not exist.

`LocalDetector` is deliberately shaped like `track_face_pi.Detector` -- same
`detect(frame, exposed_at)`, same `None` for "no answer", same `rtt_ms` and
`detect_ms` -- so the tracking loop cannot tell them apart and did not have to
change to use it. What it gains is that `frame` may now be raw pixels.

**Only one process can hold the device.** With this in use the standalone service
must not also be running; `run_oak_detect.sh` and a daemon started with
`--service local` are alternatives, not companions.
"""

import os
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oak import Oak, OakError  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_BLOB = os.path.join(HERE, "face-detection-retail-0004-640x480.blob")

# **This detector's thresholds, measured the way aiming.py's were.** They are not
# aiming.py's, and using aiming.py's here is why face tracking looked broken for an
# afternoon: YuNet needs 0.85 to acquire because it scored a yellow wall in this
# room at 0.79, and this SSD scores everything lower -- including faces. A standing
# person at conversational distance measured 0.71-0.98 depending on the angle the
# camera came at them from, so a bar of 0.85 saw them, called them "in view", and
# never once started a lock.
#
# Measured 2026-08-18, 35 pan/tilt positions with nobody in the room: **not one
# false positive reached 0.60**, against YuNet's 0.79. So the separation here is
# wide where YuNet's was narrow, and the bar can come down rather than the room
# having to be tidied.
#
#     furniture, walls, framed pictures      nothing above 0.60
#     a person, 1-3 m, various angles        0.71 - 0.98
#
# Acquiring still demands more than keeping, for the reason aiming.py gives: a face
# that turns away or is blurred by the servos should not be dropped for one bad
# frame, and proximity to the last known position is what makes the low bar safe.
ACQUIRE_SCORE = 0.65
KEEP_SCORE = 0.50

# The SSD writes a fixed 200 rows of seven float16s: image id, label, confidence,
# then the corners as fractions of the frame. A negative image id ends the list.
ROW_FLOATS = 7
ROW_BYTES = ROW_FLOATS * 2


def parse_detections(raw, frame_width, frame_height, score):
    """The device's output tensor as (x, y, w, h, score) in full-frame pixels.

    Shared with server.py so that the two ways in cannot disagree about what a
    box means -- which would be a bug visible only as the camera aiming slightly
    wrongly through one path and not the other.
    """
    rows = len(raw) // ROW_BYTES
    values = struct.unpack("<%de" % (rows * ROW_FLOATS), raw[:rows * ROW_BYTES])
    faces = []
    for row in range(rows):
        image_id, _label, confidence, xmin, ymin, xmax, ymax = \
            values[row * ROW_FLOATS:(row + 1) * ROW_FLOATS]
        if image_id < 0:
            break
        if confidence < score:
            continue
        x, y = xmin * frame_width, ymin * frame_height
        faces.append([round(x, 1), round(y, 1),
                      round(xmax * frame_width - x, 1),
                      round(ymax * frame_height - y, 1),
                      round(float(confidence), 4)])
    return faces


class LocalDetector:
    """One device, held open for the life of the process.

    Frames arrive either as raw YUYV, which is the point of this, or as JPEG for
    anything that still has one -- `count_faces` takes its own snapshot, and that
    path is a stills capture rather than the loop.
    """

    def __init__(self, blob=DEFAULT_BLOB, score=KEEP_SCORE, size=(640, 480),
                 quality=80):
        self.score = score
        self.size = size
        self.quality = quality
        self.rtt_ms = 0.0
        self.detect_ms = 0.0
        self.convert_ms = 0.0
        # The same two spans measured as *this thread's own CPU time* rather than
        # as elapsed time. The gap between the pair is the diagnostic: both these
        # spans are C with the GIL dropped, so a wall time far above the busy time
        # is not slow work, it is a thread that was ready and not allowed to run.
        # Worth keeping permanently, because the two faults look identical from
        # outside and have opposite fixes -- do less work, or stop competing.
        self.convert_busy_ms = 0.0
        self.detect_busy_ms = 0.0
        self.frames = 0
        self.errors = 0
        # The top score of each of the last few frames. Cheap, and the only way to
        # tell "it cannot see me" from "it sees me and the bar is too high" -- the
        # distinction that took an afternoon to find the first time.
        self.recent = []
        self._lock = threading.Lock()
        self._oak = Oak(blob).open()
        import ctypes

        self._input = ctypes.create_string_buffer(self._oak.input_bytes)

    def describe(self):
        _, height, width = self._oak.input_shape
        return "the OAK in this process, %dx%d graph" % (width, height)

    def detect(self, frame, exposed_at=None):
        """Faces for this frame, or None if the device could not be asked.

        `exposed_at` is accepted and ignored. It exists in the HTTP detector
        because the stamp has to survive a round trip and come back attached to
        the right frame; here the answer cannot belong to any other frame, since
        nothing is in flight but this call.
        """
        width, height = self.size
        with self._lock:
            started, busy_started = time.monotonic(), time.thread_time()
            try:
                if len(frame) == width * height * 2:
                    ok = self._oak.yuyv_to_input(frame, self._input, width, height)
                    source = (width, height)
                else:
                    sizes = self._oak.jpeg_to_input(frame, self._input)
                    ok = sizes is not None
                    source = sizes[:2] if sizes else (width, height)
                if not ok:
                    self.errors += 1
                    return []          # an unusable frame, not an absent device
                converted, busy_converted = time.monotonic(), time.thread_time()
                raw = self._oak.infer(self._input)
            except OakError:
                # The device has gone or wedged. None is "no answer", which is
                # what the tracking loop already knows how to hold still through.
                self.errors += 1
                return None
            finished, busy_finished = time.monotonic(), time.thread_time()

        self.convert_ms = (converted - started) * 1e3
        self.detect_ms = (finished - converted) * 1e3
        self.rtt_ms = (finished - started) * 1e3
        self.convert_busy_ms = (busy_converted - busy_started) * 1e3
        self.detect_busy_ms = (busy_finished - busy_converted) * 1e3
        self.frames += 1
        faces = parse_detections(raw, source[0], source[1], self.score)
        self.recent.append(round(max((f[4] for f in faces), default=0.0), 3))
        del self.recent[:-40]
        return faces

    def encode_jpeg(self, frame):
        """A raw frame as JPEG, for the paths that have to show a picture.

        Costs about what decoding one used to, which is affordable for a picture
        every few seconds and would not be per frame -- so nothing in the loop
        calls this.
        """
        width, height = self.size
        if len(frame) != width * height * 2:
            return frame                      # already a JPEG
        with self._lock:
            return self._oak.yuyv_to_jpeg(frame, width, height, self.quality)

    def close(self):
        with self._lock:
            self._oak.close()
