"""Find a face on the rover's camera and keep the pan/tilt aimed at it.

Two components at once, which makes this the first host-side script in the suite
that is not a single-component instrument: the rover's USB camera module supplies
the picture and the ESP32's two ST3215 servos are steered to keep the face in the
middle of it. Nothing else on the rover is involved -- no Raspberry Pi, no ROS,
and no OAK camera, which is not a UVC device and is not the one on the gimbal.

    python track_face.py                      # over WiFi: the rover's AP, else this LAN
    python track_face.py --host 192.168.1.22  # straight to a known address
    python track_face.py --serial             # over USB, port auto-detected
    python track_face.py --no-move            # detect and draw, command nothing
    python track_face.py --no-scan            # stay put when there is nobody about

With nobody in shot it sweeps the mechanism's whole range looking for someone --
pan end to end, then a step of tilt and back the other way -- and switches to
following the moment a face appears anywhere in the frame. How fast it may sweep
and still see anything is a measured question, not a matter of taste; SCAN_RATE
carries the numbers.

**It never drives the wheels.** The only command it sends that moves anything is
CMD_GIMBAL_CTRL_SIMPLE (`{"T":133,...}`), which reaches the two camera servos and
nothing else. It also leaves the firmware's heartbeat alone, deliberately: the
heartbeat exists to stop the *base* when commands stop arriving, and `T:133` does
not feed it, so setting it short here would achieve nothing but a stream of stop
commands to motors that were never started.

The detector is YuNet, OpenCV's own small CNN face detector. Measured here it is
5.8 ms of a 33 ms frame, the rest being spent waiting on the camera: the loop runs
at the camera's 30 fps and detection is not remotely what limits it, which is why
the frame is not decimated or the detector run every other pass. Its model is a
230 kB ONNX file that OpenCV does not ship; the first run fetches it to sit beside
this script, and `--model` points at one already downloaded. Haar cascades would
need no file, but OpenCV 5 dropped them from the wheel -- `cv2.data.haarcascades`
is an empty directory here -- so a file has to come from somewhere regardless.

Control is closed through the world -- the camera is on the thing being aimed, so
every correction changes the next measurement -- but open around the servos, which
report nothing back. Two consequences shape everything below:

* **The angles here are a model, not a reading.** The firmware has a
  `getGimbalFeedback()` that no JSON command reaches, so where the camera is
  pointing is only known by having put it there. The camera is centred at startup
  and on exit, and this is the only thing commanding it in between.
* **The loop has 266 ms of dead time**, measured, which at 30 fps is eight frames.
  Nothing commanded in that window has shown up in the picture yet, so a controller
  that simply answers what it can see answers the same error eight times over and
  diverges -- it drives past the face and pins itself against a limit, which looks
  from outside exactly like a camera avoiding people. DEAD_TIME_S explains the
  arithmetic and Gimbal.track() the remedy: correct from where the camera was when
  the frame was exposed, so motion still in flight is subtracted rather than
  commanded twice.

MEASURED, on this rover, by taking a patch of the scene, commanding a known move,
and finding that same patch in the new frame by template matching -- plain pixel
coordinates, no sign conventions to misread (1280x720, matches 0.97-0.99):

    pan  +25 deg  ->  the scene moved LEFT  243 px       9.7 px per degree
    pan  -25 deg  ->  the scene moved RIGHT 240 px       9.6
    tilt +20 deg  ->  the scene moved DOWN  188 px       9.4
    tilt -20 deg  ->  the scene moved UP    192 px       9.6

So +X pans right and +Y tilts up, symmetric both ways. A face right of centre is
therefore centred by *raising* X, and one above centre by raising Y, which is what
PAN_SIGN and TILT_SIGN say. Sign errors here are worth real suspicion: an earlier
pass took these figures from cv2.phaseCorrelate, whose sign convention is easy to
read backwards, and template matching is used above precisely because its answer is
a position rather than a convention.

A half frame is 640 px, so 66 of these degrees. An earlier pass here concluded from
that the firmware's "degrees" must be about half a real one, on the grounds that no
sane lens is 132 degrees wide. The firmware source says otherwise: gimbalCtrlSimple
maps them through map(X, 0, 360, 0, 4095), which is 11.375 counts per degree
against the ST3215's 4096 counts per turn -- one commanded degree is one real
degree. The lens really is that wide, as the barrel distortion in any frame it
takes will confirm. Note that this makes px-per-degree a centre-of-frame figure
rather than a constant, which is harmless here: the point of the exercise is to
drive the error towards the centre, where the figure is measured.

The gains below are still quoted as degrees per half frame, measured end to end,
rather than derived from a lens FOV -- a number that would have to be guessed and
would then be silently absorbed into the gain, which is how the mistake above
survived as long as it did.
Re-measure with the probe in the docstring of aim_gains() if the lens or the servo
horns change; nothing else in the file needs touching.
"""

import argparse
import collections
import json
import math
import os
import subprocess
import sys
import time
import urllib.request

import cv2

# --- the detector ---------------------------------------------------------

MODEL_FILE = "face_detection_yunet.onnx"
MODEL_URL = (
    "https://github.com/opencv/opencv_zoo/raw/main/models/"
    "face_detection_yunet/face_detection_yunet_2023mar.onnx"
)
MODEL_BYTES = 232589  # what the fetch above should produce; a short file is a bad one

# Detection runs on a reduced copy: YuNet's accuracy is set by the model's own
# 640-ish working size, not by handing it more pixels, and the box is scaled back
# up afterwards. Faces smaller than this can resolve are out of range anyway.
DETECT_WIDTH = 640
# Two thresholds, not one, and both measured. A false positive is not a cosmetic
# problem here: the camera locks onto the wrong thing and stays there, and unlike a
# person -- who moves, turns away, leaves -- a sofa goes on being detected forever.
#
# Scored on this rover, in this room: clear frontal faces came out at 0.88, 0.90
# and 0.91, a distant half-profile one at 0.73, and the arm of a black sofa against
# a yellow wall at up to **0.79**. No single threshold separates that sofa from the
# distant face, so acquiring a new target demands a score no piece of furniture
# reached, while keeping an existing one is far more forgiving -- a face that turns
# away, is half shadowed or is briefly blurred by the servos should not be dropped
# merely for scoring badly for a moment. Proximity to the last known position is
# what protects the low bar: see Target.update().
ACQUIRE_SCORE = 0.85
KEEP_SCORE = 0.60
NMS_THRESHOLD = 0.3

# --- the camera -----------------------------------------------------------

# The rover's own module, a plain UVC device -- not the OAK, which is not UVC.
ROVER_CAMERA_ID = "VID_0ABD&PID_8050"
REQUEST_SIZE = (1280, 720)
# Ask for the size first and MJPG second. That order is not cosmetic: this camera
# offers 1280x720 as MJPG at 30 fps and YUY2 at 10 fps, runs no auto-exposure at
# all in the YUY2 one, and DirectShow picks uncompressed unless asked otherwise.
# Setting FOURCC before the size is silently ignored. See docs/usb-cameras.md --
# a black picture from this camera is nearly always this, not the sensor.
PREFERRED_FOURCC = cv2.VideoWriter_fourcc(*"MJPG")
MAX_PROBE = 8
PROBE_READS = 3
MAX_READ_FAILURES = 30

# --- aiming ---------------------------------------------------------------

# Commanded degrees that swing the view by one half frame, from the measurement in
# the module docstring: 640 px / 9.65 px per degree, and 360 px / 9.5.
PAN_DEG_PER_HALF_FRAME = 66
TILT_DEG_PER_HALF_FRAME = 38

# Between a command going out and the picture showing any sign of it: 266 ms,
# measured over five 50-degree steps (203, 219, 266, 281, 312). Camera pipeline,
# WiFi, the firmware and the servo starting to turn, all together.
#
# This is the single most important number in the file, because at 30 fps it is
# **eight frames**. Everything commanded within one dead time is still in flight
# and invisible, so a controller that answers what it can see is answering a
# picture it has already acted on eight times over. Correcting half the visible
# error per frame is then an effective loop gain of four, and the result is not
# sluggishness but divergence -- the camera drives past the face, corrects harder
# the other way, and pins itself against a limit. That is what it does, observed
# and then measured, and it is why Gimbal.track() works from where the camera was
# when the frame was taken rather than from where it has since been sent.
DEAD_TIME_S = 0.27

# The fraction of the *remaining* error to correct per frame -- remaining meaning
# what is left once motion already in flight is subtracted. With the dead time
# accounted for this is a genuine proportional gain again, and 0.5 halves the error
# every frame. Left lower than it could be because the compensation is only as good
# as DEAD_TIME_S, which itself varied by +-50 ms across trials.
GAIN = 0.4
# Inside this fraction of a half frame, the face counts as centred and nothing is
# sent. Servos hunting around a target they cannot quite hold is the failure this
# prevents, and it also keeps the shared servo bus quiet when the subject is still.
DEADBAND = 0.04
# How fast the modelled angle may move, in degrees per second -- and now a measured
# figure rather than a policy: the servos cross 50 degrees in 422 ms at SPD 0, so
# about 118 deg/s is all they have. Commanding the model past that would put it
# somewhere the camera cannot be, which is exactly what the dead time compensation
# relies on it not doing: the history it reads back is only a fair account of where
# the camera was if the camera could keep up with it.
PAN_RATE = 120
TILT_RATE = 120
PAN_SIGN = 1   # +1 puts a face on the right of the picture at a higher X. Measured.
TILT_SIGN = 1  # +1 puts a face above centre at a higher Y. Measured.

# The firmware's own limits, out of gimbalCtrlSimple, which clamps to them whatever
# it is sent. Tilt is asymmetric because the mast is in the way looking down.
PAN_LIMIT = 180
TILT_LIMITS = (-30, 90)

# Servo speed, and the reason the camera used to step from pose to pose.
#
# Both gimbal commands carry a speed, and the firmware hands it to the ST3215 in
# the servo's own units: gimbalCtrlSimple maps SPD through map(spd, 0, 360, 0, 4095)
# -- 11.375 counts per degree, which is exactly the servo's 4096 counts over 360
# degrees -- so SPD is plain degrees per second. Verified: SPD 20 measured 20.4
# deg/s, 40 gave 40.2, 80 gave 75.3, and 150 gave 114.6 against a ceiling of about
# 130. **Zero means unlimited**, which is what this sent for a long time: every
# correction was "get there as fast as you can", thirty times a second, so the
# camera bolted a degree and then stood still until the next frame. Smooth motion
# is not a matter of commanding more often -- measured, 4 Hz to 32 Hz made no
# difference -- but of naming the speed the motion actually wants.
#
# T:134 is used rather than T:133 because it takes the two axes separately, in raw
# counts per second; T:133 has one SPD for both, which would drag a barely-moving
# tilt along at whatever pan happened to need.
COUNTS_PER_DEGREE = 4095.0 / 360.0
SERVO_MAX_COUNTS = 2500  # the firmware's own constrain() on SX and SY
SERVO_MAX_DEG_S = 130    # measured ceiling; asking for more just saturates
# What the camera is moved at when it is being placed rather than tracking -- the
# centring at startup and on exit. Brisk, but not a slam.
PLACE_DEG_S = 90

# The box centre jitters by a few pixels frame to frame on a motionless face, and
# every pixel of that becomes a servo command. This is the weight of a new
# measurement in the smoothed position: low enough to settle, high enough not to
# lag a person walking across the frame.
SMOOTHING = 0.5
# A face is not lost the instant it is not detected -- a blink, a turn of the head
# or the motion blur of the servos moving all drop a frame or two. Hold the aim
# this long before admitting it is gone.
LOST_GRACE_S = 0.7

# --- scanning -------------------------------------------------------------

# With no face in sight the camera sweeps its pan range end to end, reverses, steps
# tilt and sweeps back -- a serpentine over the whole field of regard, which is
# 360 degrees of pan by 120 of tilt.
#
# Two tilt levels, and the number is derived rather than chosen: the frame is 720 px
# tall at the measured 9.5 px per degree, so it takes in 76 degrees at once against
# a tilt range of 120. One level cannot cover that and three would re-scan ground
# already seen. Levels set a frame's half-height inside each end therefore reach
# both limits and overlap in the middle.
SCAN_TILTS = (8, 52)
# Degrees per second -- the whole pacing question, and measured rather than guessed.
# At this rate the picture carries about 2 px of motion smear, against a detector
# that still finds a 50 px face (someone across a room) under 9 px of it and a
# 100 px face under 27. It is also unhurried enough that a face stays in frame for
# 132/25 = 5.3 s, some 160 frames, so no single missed detection matters.
#
# Faster measures fine for detection -- 90 deg/s still finds faces -- but two things
# argue against it. Smear grows with exposure time, and this room's 4 ms is short, so
# a dim room would eat the margin. And the sweep is visibly rougher above about this
# speed: at 25 deg/s the picture moves 8.0 px a frame against the 7.5 expected, with
# 1-4% of frames not moving at all, while at 45 it delivers 11 px of an expected 13
# and stalls on nearly a fifth of them. Slower is smoother and sees no less.
# --scan-rate moves it either way.
SCAN_RATE = 25
# How long without a face before sweeping starts. Long enough that setting the rover
# down in front of somebody does not send it hunting before it has looked at them.
SCAN_AFTER_S = 2.0

# A stalled frame must not become a lurch, and this is not hypothetical: dt
# multiplies the scan step, so one slow frame commands a large jump, the board takes
# longer to answer a large jump, and the next frame is slower still. Measured, that
# spiral took the loop from 25 commands a second to 0.9 within seconds, at which
# point the sweep is a series of lunges. Clamping dt breaks it -- with this in
# place every rate tried held 25 fps with no commands lost.
MAX_DT = 0.25

# --- the link -------------------------------------------------------------

DEFAULT_HOST = "192.168.4.1"  # the ESP32's own AP: SSID "UGV", password "12345678"
BAUD = 115200
PROBE_COMMAND = {"T": 130}
PROBE_REPLY = b'"T":1001'
PROBE_TIMEOUT = 0.4
PROBE_WORKERS = 64

WINDOW = "Face tracking -- rover pan/tilt"


def aim_gains():
    """How PAN_DEG_PER_HALF_FRAME and TILT_DEG_PER_HALF_FRAME were arrived at.

    Not called. Kept as the recipe, because the two constants are the only things
    in this file that are properties of the hardware rather than of the algorithm,
    and a changed lens or a servo horn refitted a spline out makes them wrong:

        centre the gimbal and grab a frame. Cut a patch out of the middle of it,
        command a known step, grab another frame, and find that patch again with
        cv2.matchTemplate. Where it has got to, minus where it started, is the
        picture's shift in pixels. Repeat both ways on both axes. Half the frame
        width divided by the pan figure is PAN_DEG_PER_HALF_FRAME; half the height
        over the tilt figure is the other.

    Template matching rather than phase correlation, deliberately: it answers with
    a position, which cannot be misread, where a correlation answers with a shift
    whose sign depends on which argument came first. That distinction cost a whole
    debugging pass here.

    Getting the sign wrong is not subtle -- but neither is it distinguishable, from
    the outside, from having the gain too high for the dead time. Both send the
    camera to a limit and hold it there. Tell them apart by watching one correction
    from a standstill: the wrong sign moves away from the face immediately, while
    too much gain moves towards it first and overshoots.
    """


# --- model ----------------------------------------------------------------


def ensure_model(path):
    """The ONNX file, fetched once if it is not already here.

    A network fetch on first run is a poor fit for a suite whose whole point is
    running with nothing else working, so it happens exactly once and lands beside
    the script, where the next run finds it. `--model` skips it entirely for a copy
    that arrived some other way.
    """
    if os.path.exists(path) and os.path.getsize(path) > MODEL_BYTES // 2:
        return path
    print(f"fetching the face detector ({MODEL_BYTES // 1024} kB, once) -> {path}")
    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        # To a temporary name first: an interrupted download that keeps the real
        # name would be loaded as a model on the next run and fail obscurely.
        partial = path + ".part"
        with urllib.request.urlopen(MODEL_URL, timeout=30) as response:
            data = response.read()
        if len(data) < MODEL_BYTES // 2:
            raise ValueError(f"got {len(data)} bytes, expected about {MODEL_BYTES}")
        with open(partial, "wb") as handle:
            handle.write(data)
        os.replace(partial, path)
    except Exception as error:
        sys.exit(
            f"Could not fetch the face detector: {error}\n"
            f"Download it by hand from {MODEL_URL}\n"
            f"and put it at {path}, or name it with --model."
        )
    return path


class Detector:
    """YuNet, run on a reduced copy of the frame.

    Faces come back as (x, y, w, h, score) in full-frame pixels. The reduction is
    where the speed comes from: the detector's cost is set by its input size, and
    a 640-wide copy of a 720p frame costs 14 ms against 40 for the whole thing,
    with no difference to a face big enough to be worth aiming at.
    """

    def __init__(self, model_path, frame_size):
        try:
            # The network runs at the lower bar and Target decides what is worth
            # locking onto: a detection too weak to acquire may still be the face
            # already being followed, and the detector cannot know which is which.
            self.net = cv2.FaceDetectorYN_create(
                model_path, "", (320, 320), KEEP_SCORE, NMS_THRESHOLD, 5000
            )
        except cv2.error as error:
            sys.exit(f"Cannot load the face detector from {model_path}: {error}")
        width, height = frame_size
        self.scale = min(DETECT_WIDTH / width, 1.0)
        self.size = (int(width * self.scale), int(height * self.scale))
        self.net.setInputSize(self.size)

    def detect(self, frame):
        small = cv2.resize(frame, self.size) if self.scale < 1.0 else frame
        _, raw = self.net.detect(small)
        if raw is None:
            return []
        back = 1.0 / self.scale
        faces = []
        for row in raw:
            x, y, w, h = (float(v) * back for v in row[:4])
            faces.append((x, y, w, h, float(row[-1])))
        return faces


class Target:
    """Which face is being followed, and where it is, smoothed.

    Re-detecting every frame says nothing about which of several faces is the one
    from last frame, so the lock is kept by proximity: the detection nearest the
    last known position, within a radius that scales with the face's own size, is
    the same person. With no lock, the largest face wins -- the nearest, in
    practice, and the one a person putting themselves in front of the rover means.

    Proximity is also what makes the two thresholds safe. A weak detection is
    accepted only where the face already was, so a turning head keeps its lock
    while a sofa across the room, however persuasively face-shaped, is never
    strong enough to start one.
    """

    def __init__(self):
        self.centre = None   # smoothed (x, y), the thing actually aimed at
        self.box = None      # the last raw detection, for drawing
        # Now, rather than zero: --search counts from this, and a rover that has
        # only just started has not been failing to find anyone since the epoch.
        self.seen_at = time.monotonic()

    def update(self, faces, now):
        if not faces:
            return self.centre is not None and now - self.seen_at < LOST_GRACE_S
        pick = None
        if self.centre is not None and now - self.seen_at < LOST_GRACE_S:
            x0, y0 = self.centre
            near = [
                face for face in faces
                # A face travels at most its own width between frames; anything
                # further away is a different person, not this one having moved.
                if math.hypot(face[0] + face[2] / 2 - x0, face[1] + face[3] / 2 - y0)
                < max(face[2], 60) * 1.5
            ]
            if near:
                pick = max(near, key=lambda f: f[2] * f[3])
        if pick is None:
            strong = [face for face in faces if face[4] >= ACQUIRE_SCORE]
            if not strong:
                # Something face-shaped, but not enough to point the camera at.
                return self.centre is not None and now - self.seen_at < LOST_GRACE_S
            pick = max(strong, key=lambda f: f[2] * f[3])
        x, y, w, h, _ = pick
        fresh = (x + w / 2, y + h / 2)
        if self.centre is None or now - self.seen_at >= LOST_GRACE_S:
            self.centre = fresh  # a new lock starts where the face is, not part way
        else:
            self.centre = tuple(
                previous + SMOOTHING * (new - previous)
                for previous, new in zip(self.centre, fresh)
            )
        self.box = pick
        self.seen_at = now
        return True

    def drop(self):
        self.centre = self.box = None

    def locked(self, now):
        return self.centre is not None and now - self.seen_at < LOST_GRACE_S


class Gimbal:
    """Where the camera is pointed, in the firmware's degrees. A model, not a reading.

    Kept true by centring at startup and being the only thing commanding the servos
    thereafter -- there is no way to ask them where they are. Commands go out only
    when the rounded target changes, so a still subject is silent on the wire.
    """

    def __init__(self, gain=GAIN):
        self.pan = 0.0
        self.tilt = 0.0
        self.gain = gain
        self.sent = None
        # How fast each axis is currently meant to be travelling, degrees a second.
        # Sent with every command so the servo paces itself across the gap between
        # frames instead of arriving instantly and waiting -- see COUNTS_PER_DEGREE.
        self.speed_pan = PLACE_DEG_S
        self.speed_tilt = PLACE_DEG_S
        # Where the camera has been told to point, and when. Read back one dead
        # time later to find out what the angles were when a frame was exposed.
        self.history = collections.deque([(0.0, 0.0, 0.0)])

    def record(self, now):
        self.history.append((now, self.pan, self.tilt))
        # Two dead times is all that is ever asked for; the rest is memory.
        while len(self.history) > 1 and self.history[0][0] < now - DEAD_TIME_S * 2:
            self.history.popleft()

    def was_at(self, when):
        """The angles as they had been commanded at `when`.

        The most recent entry at or before that moment, which for a deque filled
        once a frame is never more than a frame stale.
        """
        found = self.history[0]
        for entry in self.history:
            if entry[0] > when:
                break
            found = entry
        return found[1], found[2]

    def track(self, error_x, error_y, dt, now):
        """One step towards a face at (error_x, error_y), each -1..1 from centre.

        error_y is positive upwards, unlike the picture's rows, so that both axes
        share the sign of the servo they drive and neither needs a flip hidden in
        the middle of the arithmetic.

        The error was measured in a frame exposed a dead time ago, so it locates
        the face relative to where the camera pointed *then*. Adding the two gives
        the face's angle outright -- a fixed thing, independent of everything
        commanded since -- and what is worth correcting is the gap between that and
        where the camera is already on its way to. Subtracting the in-flight motion
        this way is the whole difference between converging and running away.

        The axes are deadbanded separately: a face level with the camera but off to
        one side should be panned to without the tilt twitching along with it.
        """
        pan_then, tilt_then = self.was_at(now - DEAD_TIME_S)
        # Where the face is, as an angle. The signs live here, and only here: they
        # describe how the picture relates to the servos, which is this conversion.
        face_pan = pan_then + PAN_SIGN * error_x * PAN_DEG_PER_HALF_FRAME
        face_tilt = tilt_then + TILT_SIGN * error_y * TILT_DEG_PER_HALF_FRAME
        remaining_pan = face_pan - self.pan
        remaining_tilt = face_tilt - self.tilt
        # Deadband in degrees, on what is left to do rather than on what can be
        # seen: a face already centred in a stale frame may still need correcting
        # back if the camera has since been sent past it.
        step_pan = 0.0 if abs(remaining_pan) < DEADBAND * PAN_DEG_PER_HALF_FRAME \
            else self.gain * remaining_pan
        step_tilt = 0.0 if abs(remaining_tilt) < DEADBAND * TILT_DEG_PER_HALF_FRAME \
            else self.gain * remaining_tilt
        self.move(
            clamp(step_pan, -PAN_RATE * dt, PAN_RATE * dt),
            clamp(step_tilt, -TILT_RATE * dt, TILT_RATE * dt),
            dt,
        )

    def move(self, delta_pan, delta_tilt, dt):
        """Move the target by so many degrees over dt. No signs: see track().

        The speed sent with the command comes from the move itself -- how far,
        over how long -- so the servo is asked to cover exactly the ground the
        next frame expects, arriving as that frame does rather than long before
        it. Taken from where the axis actually ended up, so a step clipped by a
        limit slows the servo rather than leaving it straining at the stop.
        """
        was_pan, was_tilt = self.pan, self.tilt
        self.pan = clamp(self.pan + delta_pan, -PAN_LIMIT, PAN_LIMIT)
        self.tilt = clamp(self.tilt + delta_tilt, *TILT_LIMITS)
        if dt > 0:
            self.speed_pan = abs(self.pan - was_pan) / dt
            self.speed_tilt = abs(self.tilt - was_tilt) / dt

    def centre(self, speed=PLACE_DEG_S):
        self.pan = self.tilt = 0.0
        self.speed_pan = self.speed_tilt = speed

    def command(self):
        return {"T": 134, "X": round(self.pan), "Y": round(self.tilt),
                "SX": counts(self.speed_pan), "SY": counts(self.speed_tilt)}

    def changed(self):
        """True if the camera is being asked to point somewhere new.

        Position only: the speed rides along with whatever command the position
        earns, and a speed that has changed by itself is nothing worth waking
        the board up for.
        """
        where = (round(self.pan), round(self.tilt))
        if where == self.sent:
            return False
        self.sent = where
        return True


def clamp(value, low, high):
    return min(max(value, low), high)


def counts(degrees_per_second):
    """Degrees a second -> the servo's own speed units, as T:134 wants them.

    Never zero: the firmware constrains SX and SY to 1..2500, and zero would mean
    unlimited in any case -- the very thing being avoided here.
    """
    wanted = min(degrees_per_second, SERVO_MAX_DEG_S) * COUNTS_PER_DEGREE
    return int(clamp(round(wanted), 1, SERVO_MAX_COUNTS))


class Scan:
    """The serpentine sweep run when there is no face to follow.

    Pan crosses its whole range, reverses at the limit, steps tilt to the other
    level and crosses back, so everything the mechanism can see is looked at in
    turn. It is one state machine with two states, panning and tilting, rather
    than both at once: moving the two together would sweep a diagonal and leave
    wedges of the range unseen at each end.

    The tilt step is paced at the same rate as the pan sweep instead of being
    taken at the servo's own speed, because the transition is a second of looking
    at a part of the range like any other -- there is no reason to blur through it.
    """

    def __init__(self, gimbal):
        # Start by continuing the way it is already facing and from the level it is
        # already nearest, so switching from tracking to scanning does not begin
        # with the camera swinging back across ground it can already see.
        self.direction = 1 if gimbal.pan <= 0 else -1
        self.level = min(
            range(len(SCAN_TILTS)),
            key=lambda i: abs(SCAN_TILTS[i] - gimbal.tilt),
        )
        self.tilting = True  # settle onto a level before sweeping along it

    def step(self, gimbal, rate, dt):
        if self.tilting:
            remaining = SCAN_TILTS[self.level] - gimbal.tilt
            if abs(remaining) <= rate * dt:
                gimbal.move(0, remaining, dt)
                self.tilting = False
            else:
                gimbal.move(0, math.copysign(rate * dt, remaining), dt)
            return
        gimbal.move(self.direction * rate * dt, 0, dt)
        if abs(gimbal.pan) >= PAN_LIMIT:
            self.direction = -self.direction
            self.level = (self.level + 1) % len(SCAN_TILTS)
            self.tilting = True

    def state(self):
        return "scanning, tilting" if self.tilting else "scanning"


# --- camera ---------------------------------------------------------------


def silence_opencv():
    """Probing indices that are not there is noisy, and the warnings are expected."""
    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
    except AttributeError:
        pass


def camera_ids():
    """Device paths for the machine's cameras, in DirectShow's enumeration order.

    Sorted by PNPDeviceID, which reproduces that order: DirectShow builds its list
    from registry keys named after the device path, so they come back
    lexicographically, while Get-CimInstance's own order does not match and puts
    every label one place out. Same trick as usb_cameras/preview_usb_cameras.py.
    """
    if sys.platform != "win32":
        return []
    query = (
        "Get-CimInstance Win32_PnPEntity "
        "-Filter \"PNPClass='Camera' or PNPClass='Image'\" "
        "| Sort-Object PNPDeviceID "
        "| Select-Object -ExpandProperty PNPDeviceID"
    )
    try:
        done = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", query],
            capture_output=True, text=True, timeout=15,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    return [line.strip() for line in done.stdout.splitlines() if line.strip()]


def find_rover_camera():
    """The capture index of the rover's own camera module, or None.

    Windows' list is positional and includes devices that never stream, so this is
    a good guess rather than an identity -- open_camera() checks that whatever it
    picked actually delivers frames, and falls back to a scan if it does not.
    """
    for index, device in enumerate(camera_ids()):
        if ROVER_CAMERA_ID in device.upper():
            return index
    return None


def open_camera(index):
    """Open one camera at the size and format that keep its automatics working."""
    backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQUEST_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQUEST_SIZE[1])
    cap.set(cv2.CAP_PROP_FOURCC, PREFERRED_FOURCC)  # after the size, always
    # Every queued frame is dead time in a loop that is closed through the servos,
    # so ask for the shallowest buffer the driver will give. Best effort: several
    # backends accept the call and keep their own depth.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not any(cap.read()[0] for _ in range(PROBE_READS)):
        cap.release()
        return None
    return cap


def choose_camera(requested):
    """(capture, index): the one asked for, the rover's, or the first that works."""
    if requested is not None:
        cap = open_camera(requested)
        if cap is None:
            sys.exit(f"Camera index {requested} does not open, or delivers no frames.")
        return cap, requested
    rover = find_rover_camera()
    if rover is not None:
        cap = open_camera(rover)
        if cap is not None:
            return cap, rover
        print(f"the rover's camera looked like index {rover}, which will not stream")
    for index in range(MAX_PROBE):
        cap = open_camera(index)
        if cap is not None:
            return cap, index
    sys.exit("No camera found. Plug the rover's camera in, or name one with --camera.")


def fourcc_name(value):
    packed = int(value)
    if packed <= 0:
        return "?"
    name = "".join(chr((packed >> (8 * i)) & 0xFF) for i in range(4))
    return name if name.isprintable() else "?"


# --- link -----------------------------------------------------------------


def js_path(command):
    """A command as the board wants it: JSON in the query string of `/js`."""
    from urllib.parse import quote

    return "/js?json=" + quote(json.dumps(command, separators=(",", ":")), safe="")


class HttpLink:
    """JSON commands over the ESP32's own `/js` endpoint, on one kept-open socket."""

    def __init__(self, host, timeout=0.5):
        import http.client

        self._client = http.client
        self.host = host
        self.timeout = timeout
        self.connection = None

    def describe(self):
        return f"http://{self.host}/js"

    def send(self, command):
        path = js_path(command)
        for attempt in (1, 2):  # a stale keep-alive costs one retry, not a command
            if self.connection is None:
                self.connection = self._client.HTTPConnection(
                    self.host, timeout=self.timeout)
            try:
                self.connection.request("GET", path)
                self.connection.getresponse().read()
                return True
            except Exception:
                self.close()
                if attempt == 2:
                    return False
        return False

    def close(self):
        if self.connection is not None:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None


class SerialLink:
    """JSON commands over the ESP32's Type-C port -- the one *not* labelled LIDAR."""

    def __init__(self, port):
        import serial

        self.port = port
        self.link = serial.Serial(port, BAUD, timeout=0.1)

    def describe(self):
        return f"{self.port} at {BAUD}"

    def send(self, command):
        try:
            self.link.write(json.dumps(command, separators=(",", ":")).encode() + b"\n")
            self.link.reset_input_buffer()  # the board chatters; nothing here reads it
            return True
        except Exception:
            return False

    def close(self):
        try:
            self.link.close()
        except Exception:
            pass


class NoLink:
    """--no-move: everything runs, nothing is commanded."""

    def describe(self):
        return "nothing (--no-move)"

    def send(self, command):
        return True

    def close(self):
        pass


def find_serial_port():
    """The board's port, found by asking each candidate something only it answers.

    Both Type-C ports enumerate alike, so USB identity cannot separate them: the
    ESP32 replies to base feedback with a JSON line, while the lidar port only ever
    streams binary. Ports with no VID are skipped -- those are Bluetooth SPP, and
    merely opening one blocks for as long as Windows spends raising a radio link.
    """
    import serial
    from serial.tools import list_ports

    for port in list_ports.comports():
        if port.vid is None:
            continue
        try:
            with serial.Serial(port.device, BAUD, timeout=0.1) as link:
                link.reset_input_buffer()
                link.write(b'{"T":130}\n')
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    if link.readline().startswith(b"{"):
                        return port.device
        except Exception:
            continue
    return None


def probe_host(host):
    """True if the driver board answers at this address.

    A connection proves nothing -- plenty of things on a home LAN serve port 80 --
    so this reads the reply and insists on the firmware's own feedback line.
    """
    import http.client

    try:
        connection = http.client.HTTPConnection(host, timeout=PROBE_TIMEOUT)
        try:
            connection.request("GET", js_path(PROBE_COMMAND))
            return PROBE_REPLY in connection.getresponse().read()
        finally:
            connection.close()
    except Exception:
        return False


def local_network():
    """Every address on this machine's own /24, minus this machine.

    The interface is chosen by opening a UDP socket towards the rover; nothing is
    sent, but it names the interface rover traffic would leave by, which is the one
    worth sweeping on a machine that also carries VM and VPN adapters.
    """
    import socket

    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((DEFAULT_HOST, 80))
        address = probe.getsockname()[0]
    except OSError:
        return []
    finally:
        probe.close()
    prefix, _, own = address.rpartition(".")
    if not prefix or address.startswith("0."):
        return []
    return [f"{prefix}.{octet}" for octet in range(1, 255) if str(octet) != own]


def find_host():
    """The board's address: its own AP first, then a sweep of this LAN.

    The firmware publishes no mDNS name and sets no DHCP hostname, so a rover that
    has joined a home network is an anonymous lease with nothing to look up. What
    it does have is an answer nothing else gives, so every address is asked for
    base feedback and whoever replies is the rover.
    """
    from concurrent import futures

    if probe_host(DEFAULT_HOST):
        return DEFAULT_HOST
    candidates = local_network()
    if not candidates:
        return None
    print(f"searching {candidates[0].rsplit('.', 1)[0]}.0/24 for the driver board...")
    with futures.ThreadPoolExecutor(PROBE_WORKERS) as pool:
        pending = {pool.submit(probe_host, host): host for host in candidates}
        try:
            for done in futures.as_completed(pending):
                if done.result():
                    host = pending[done]
                    print(f"found it at {host} -- pass --host {host} to skip this")
                    return host
        finally:
            for future in pending:
                future.cancel()
    return None


def open_link(args):
    if not args.move:
        return NoLink()
    if args.serial is None:
        host = args.host or find_host()
        if host is None:
            sys.exit("No driver board found on its own AP or this network. "
                     "Name it, e.g. --host 192.168.1.22")
        return HttpLink(host)
    port = args.serial if args.serial != "auto" else find_serial_port()
    if port is None:
        sys.exit("No driver board found on any serial port. Name it, e.g. --serial COM7")
    try:
        return SerialLink(port)
    except Exception as error:
        sys.exit(f"Cannot open {port}: {error}")


# --- drawing --------------------------------------------------------------


def draw(frame, target, faces, lines, now, held):
    height, width = frame.shape[:2]
    centre = (width // 2, height // 2)
    grey = (140, 140, 140)
    cv2.line(frame, (centre[0] - 18, centre[1]), (centre[0] + 18, centre[1]), grey, 1)
    cv2.line(frame, (centre[0], centre[1] - 18), (centre[0], centre[1] + 18), grey, 1)
    # The deadband, drawn: inside this box nothing is commanded, so a face sitting
    # in it with the servos quiet is the loop working, not the loop stalled.
    cv2.rectangle(
        frame,
        (int(centre[0] - DEADBAND * width / 2), int(centre[1] - DEADBAND * height / 2)),
        (int(centre[0] + DEADBAND * width / 2), int(centre[1] + DEADBAND * height / 2)),
        grey, 1,
    )
    # Everything the detector offered, faintly, with its score: a box drawn thin is
    # something seen and passed over, which is the difference between a detector
    # that found nothing and one whose findings were not good enough to aim at.
    for x, y, w, h, score in faces:
        if target.box is None or (x, y, w, h) != tuple(target.box[:4]):
            weak = (110, 110, 190)
            cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), weak, 1)
            cv2.putText(frame, f"{score:.2f}", (int(x), int(y) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, weak, 1, cv2.LINE_AA)
    if target.box is not None and target.locked(now):
        x, y, w, h, score = target.box
        colour = (90, 200, 255) if held else (80, 255, 120)
        cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), colour, 2)
        cv2.putText(frame, f"{score:.2f}", (int(x), int(y) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2, cv2.LINE_AA)
        if target.centre is not None:
            point = (int(target.centre[0]), int(target.centre[1]))
            cv2.circle(frame, point, 4, colour, -1)
            cv2.line(frame, centre, point, colour, 1)
    annotate(frame, lines)


def annotate(frame, lines):
    # putText stretches glyph advances once thickness exceeds 1, so an outline drawn
    # as a heavier pass drifts right of the text it should bound. Build it from
    # offset copies instead, every pass at thickness 1.
    def put(text, origin, colour):
        cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 1,
                    cv2.LINE_AA)

    for row, text in enumerate(lines):
        x, y = 12, 28 + row * 24
        for dx, dy in ((-2, 0), (2, 0), (0, -2), (0, 2),
                       (-1, -1), (1, -1), (-1, 1), (1, 1)):
            put(text, (x + dx, y + dy), (0, 0, 0))
        put(text, (x, y), (255, 255, 255))


# --- main -----------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Track a face with the rover's pan/tilt camera.")
    parser.add_argument(
        "--host", default=None, metavar="ADDRESS",
        help="the ESP32's address over WiFi; by default its own AP, then this LAN")
    parser.add_argument(
        "--serial", nargs="?", const="auto", default=None, metavar="PORT",
        help="command over USB instead; bare, or with a port such as COM7")
    parser.add_argument(
        "--camera", type=int, default=None, metavar="INDEX",
        help="capture index; by default the rover's own module, found by USB id")
    parser.add_argument(
        "--model", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        MODEL_FILE),
        help="the YuNet ONNX file; fetched beside this script if absent")
    parser.add_argument(
        "--no-move", dest="move", action="store_false",
        help="detect and draw, but command nothing -- check the picture first")
    parser.add_argument(
        "--no-scan", dest="scan", action="store_false",
        help="stay put when there is no face, instead of sweeping to look for one")
    parser.add_argument(
        "--scan-rate", type=float, default=SCAN_RATE, metavar="DEG",
        help=f"sweep speed in degrees per second (default {SCAN_RATE}); "
             "slower in a dim room, where the longer exposure smears the picture")
    parser.add_argument(
        "--gain", type=float, default=GAIN, metavar="G",
        help=f"fraction of the error corrected per frame (default {GAIN})")
    args = parser.parse_args()

    silence_opencv()
    model = ensure_model(args.model)
    cap, index = choose_camera(args.camera)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"camera index {index}: {width}x{height} "
          f"{fourcc_name(cap.get(cv2.CAP_PROP_FOURCC))}")
    detector = Detector(model, (width, height))

    link = open_link(args)
    print(f"commanding {link.describe()}; q quits")

    gimbal = Gimbal(clamp(args.gain, 0.05, 1.0))
    # The one thing that moves before a face is seen: the angles are a model, and
    # this is what makes the model true.
    if not link.send(gimbal.command()):
        cap.release()
        link.close()
        sys.exit(f"No answer from the driver board on {link.describe()}. Is it powered?")
    gimbal.changed()  # the centring above counts as sent; do not repeat it

    target = Target()
    scan = None  # built when sweeping starts, from wherever the camera then points
    scan_rate = clamp(args.scan_rate, 1.0, 200.0)
    held = False       # h pauses commanding without stopping detection
    failures = 0
    frames = 0
    rate = 0.0
    last_tick = time.monotonic()
    read_failures = 0

    try:
        while True:
            ok, frame = cap.read()
            now = time.monotonic()
            # Clamped, not raw: a frame that took a second to arrive must not be
            # answered with a second's worth of sweep. See MAX_DT.
            dt, last_tick = min(now - last_tick, MAX_DT), now
            if not ok:
                read_failures += 1
                if read_failures >= MAX_READ_FAILURES:
                    print("\nlost the camera.", file=sys.stderr)
                    break
                continue
            read_failures = 0

            faces = detector.detect(frame)
            tracking = target.update(faces, now)

            scanning = False
            if tracking and not held:
                # A face again: the sweep is abandoned, and the next one will be
                # built afresh from wherever tracking has left the camera pointing.
                scan = None
                # Positive x is right of centre and positive y is *above* it, which
                # is not the picture's own row order -- see Gimbal.track().
                error_x = (target.centre[0] - width / 2) / (width / 2)
                error_y = (height / 2 - target.centre[1]) / (height / 2)
                gimbal.track(error_x, error_y, dt, now)
            elif not tracking:
                if target.centre is not None:
                    target.drop()
                if args.scan and not held and now - target.seen_at > SCAN_AFTER_S:
                    if scan is None:
                        scan = Scan(gimbal)
                    scan.step(gimbal, scan_rate, dt)
                    scanning = True

            # Every frame, moved or not: track() reads this back a dead time
            # later to find where the camera was when a frame was exposed.
            gimbal.record(now)

            if not held and gimbal.changed():
                failures = 0 if link.send(gimbal.command()) else failures + 1

            # Smoothed, because a per-frame figure flickers too fast to read -- and
            # worth reading, since the loop's rate is most of its dead time.
            frames += 1
            rate = rate + 0.1 * (1.0 / max(dt, 1e-3) - rate) if frames > 1 else 0.0

            state = ("holding" if held else
                     "tracking" if tracking else
                     scan.state() if scanning else "no face")
            offset = ""
            if tracking:
                offset = (f"  err {(target.centre[0] - width / 2) / (width / 2):+.2f},"
                          f"{(height / 2 - target.centre[1]) / (height / 2):+.2f}")
            draw(frame, target, faces, [
                f"{state}  faces {len(faces)}{offset}",
                f"pan {gimbal.pan:+4.0f}  tilt {gimbal.tilt:+3.0f}  {rate:4.1f} fps"
                + ("" if not failures else f"  link {failures} lost"),
                "q quit   c centre   space re-target   h hold",
            ], now, held)
            cv2.imshow(WINDOW, frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("c"):
                gimbal.centre()
            elif key == ord(" "):
                target.drop()  # next frame re-locks on the largest face
            elif key == ord("h"):
                held = not held
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        # Back to centre, which is where the next run will assume it is. Nothing
        # else needs undoing: the wheels were never touched and the heartbeat was
        # left at the firmware's own default throughout.
        gimbal.centre()
        link.send(gimbal.command())
        link.close()
        cap.release()
        cv2.destroyAllWindows()
        print("centred, stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
