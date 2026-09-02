"""Where to point the camera, given where a face is. No pictures, no network.

Split out of track_face.py when the detector moved onto the MEDIA host and a
second caller appeared -- track_face_pi.py, which runs on the rover and has no
OpenCV to import. Everything here is arithmetic and policy: what counts as a
face worth following, how far to move for a given error, how fast, and what to
do when there is nobody about.

The split is not tidiness. These constants were measured on this rover, against
this lens and these servo horns, and the two scripts must agree on every one of
them or they are two different robots. Keeping them in one file makes that
structural rather than a matter of remembering. The measurement provenance rides
along with each one, because a number without its recipe cannot be re-measured
when the hardware changes.

The lens and the geometry that follows from it are in `lens.py` beside this, for
the same reason the constants are shared: they are the hardware's properties
rather than this loop's, and they change when the camera does and not when the
control law does. They are re-exported here, so `aiming.solve` and
`from aiming import LENS` go on meaning what they always did.

What is deliberately *not* here: anything that decodes a picture, opens a camera
or talks to the board. A caller supplies face boxes in full-frame pixels and a
clock, and gets angles back.
"""

import collections
import math
import time

from lens import (
    ACQUIRE_SCORE, DETECT_WIDTH, KEEP_SCORE, LENS, NMS_THRESHOLD,
    PAN_DEG_PER_HALF_FRAME, TILT_DEG_PER_HALF_FRAME, gains_for, lens_for,
    lens_recipe, ray_at, solve, theta_of,
)


# Between a command going out and the picture showing any sign of it: 266 ms,
# measured over five 50-degree steps (203, 219, 266, 281, 312), with the camera on
# the workstation's own USB and the board commanded over WiFi. Camera pipeline,
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
#
# It is a *fallback*. Any caller that can say when a frame was actually exposed
# should pass that to track() instead, and both callers now can: track_face_pi.py
# reads V4L2's start-of-exposure stamp, and the detector on MEDIA echoes it back
# with the boxes. This constant then only covers the case of a caller with no
# such clock. Note the figure varied by +-50 ms across the five trials, which is
# the whole argument for measuring it per frame rather than trusting this.
DEAD_TIME_S = 0.27


# Between a command leaving and the camera *beginning* to move: 155 ms in pan,
# 120 in tilt, measured on the rover by commanding a step and template-matching
# each following frame against the scene before it. The picture is bit-identical
# at +23, +55 and +87 ms and only then starts to shift.
#
# This is the other half of the old 266 ms, and separating the two is the whole
# reason the exposure stamp was worth carrying. DEAD_TIME_S lumped together how
# stale the picture is and how late the servo is; an exposure stamp answers only
# the first. Subtracting it alone leaves was_at() reporting angles the servo has
# been *told* about but not yet reached -- so the controller believes it has
# already moved further than it has, and commands further still. At 25 fps this
# is four frames of invisible motion, and GAIN applied to each of them is an
# effective loop gain near 1.6: the camera hunts, which is what it did.
#
# The two axes differ by more than the measurement is worth arguing about; one
# figure is used for both, nearer the slower one because overestimating this is
# the safe direction (it makes the loop sluggish, not unstable).
COMMAND_LATENCY_S = 0.14


# How much commanded history to keep, and therefore the oldest moment track()
# can be asked about -- which is an exposure stamp *minus* COMMAND_LATENCY_S, so
# it has to cover both delays and not just the one. Two dead times leaves room to
# spare: on the rover the pair comes to about 190 ms against this 540. Asking for
# something older reads back the earliest entry there is, which is a stale answer
# rather than a wrong one, and only happens if the picture has stalled entirely.
# ...and it has to reach back as far as the oldest frame that might still be
# answered. On the rover a frame is up to a second old when the loop gets it --
# see MAX_FRAME_AGE_S -- so two dead times is not enough and was_at() was falling
# back to its earliest entry, reporting a camera angle newer than the truth.
HISTORY_S = max(2.5, DEAD_TIME_S * 2)


# The fraction of the *remaining* error to correct per frame -- remaining meaning
# what is left once motion already in flight is subtracted. With the dead time
# accounted for this is a genuine proportional gain again, and 0.5 halves the error
# every frame.
#
# Raised from 0.4 once the two things it was being held down for were fixed: the
# exposure stamps now pair, so "when was this taken" is answered rather than
# guessed, and was_at() now reads a move as a ramp rather than as an arrival. At
# 0.4 a face 40 degrees off axis took four frames to reach -- nearly two seconds
# at this loop rate, and visibly a series of ever smaller nudges. Simulated under
# the measured conditions (430 ms frames, 227 ms old, servo ceiling 130 deg/s):
#
#     gain 0.4     4 frames to settle,  7 deg of overshoot
#     gain 0.7     3 frames,           14 deg
#     gain 1.0     2 frames,           19 deg
#
# 0.7 buys most of the speed for a bounded amount of swinging past. Going higher
# is tempting and is what the servo ceiling punishes: the bigger the single step,
# the longer the camera spends mid-move while frames are being taken of it.
#
# **0.9 was tried on 2026-08-19 and put back the same day.** The argument for it was
# that the frames this loop acts on are a measured 1.4 s old -- the camera delivers
# 30 a second and this host reassembles five, so everything else queues -- and that a
# higher gain covers more of the ground on the one correction anybody watching the
# rover actually sees. Simulated at that frame age rather than the 227 ms the table
# above assumes, a stationary face 39 degrees off centre, pixels from the middle:
#
#     gain 0.5    250 250 127  64  76 118 101  44  11    worst swing back  42 px
#     gain 0.7    250 250 102  31  84 156  96  19  19                      72 px
#     gain 0.9    250 250 102  10  97 190  78  74  25                     102 px
#
# 0.9 does land within 10 px on the third frame where 0.7 needs 31 -- and then swings
# back out half a frame, which is worse to watch than arriving a frame later. Every
# gain rings while the picture is a second old and the higher ones ring harder, so
# the answer is not the gain: it is the frame age, and that is a property of the
# host. At a 200 ms frame age the ranking is the one the table above describes and
# 0.7 rings barely at all.
#
# **That host arrived.** On the Banana Pi M4 Zero, with YuNet detecting on its own
# cores, the loop measured 6.6 frames a second on 2026-08-23 -- a frame about
# 190 ms old by the time the angles are worked out, against 1.4 s on the Pi 1. So
# this is now the frame age the table above was written for, and 0.7 is the value
# for it rather than a compromise. It has not been re-tuned upwards: the argument
# for going higher was always about covering ground while the picture was stale.
GAIN = 0.7


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


# Where the camera rests: straight ahead, and ten degrees above level rather than
# on it. Not a calibration -- the aiming maths is the same at any tilt -- but a
# choice about what the rover is pointed at when nothing has asked it to point
# anywhere. The camera sits low on a rover that sits on the floor, so level puts
# the bottom two thirds of every frame on the ground a metre in front of it. Ten
# degrees is enough to spend that on the room instead, and small enough that
# what was in the middle of the picture stays in it: the lens takes in 76
# degrees vertically, so this moves the view by about one part in eight.
#
# The sweep is unaffected and stays at SCAN_TILT, which is 45: looking for a
# face is a different job from resting, and it is argued where that number is.
# What this changes is the picture anything else takes -- a `look`, a
# count_faces, the world state's own inspections -- because all of them read
# whatever frame the camera happens to be giving, and off the tracking loop
# that is this one.
REST_TILT_DEG = 10


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


# The longest a correction may take to travel, whatever the gap between frames.
#
# move() paces each step to cover its ground in exactly one frame interval, so the
# servo arrives as the next frame does rather than long before it. That is right
# while the interval is short. It stops being right when the loop slows down: the
# camera is then moving throughout the exposure of every frame that follows a
# correction, and the picture blurs. Measured on the rover at 4.5 fps, the mean
# gradient of a frame taken while tracking was **1.99 against 3.68** with the camera
# still -- half the sharpness -- and a blurred frame is one the detector is likelier
# to miss, which drops the lock, which starts the sweep, which blurs the next one.
#
# So the pacing is capped rather than removed. Below this interval nothing changes
# and the servo still arrives with the frame; above it the step is made at this
# pace and the camera is standing still by the time the picture is taken.
SETTLE_WITHIN_S = 0.08


# The box centre jitters by a few pixels frame to frame on a motionless face, and
# every pixel of that becomes a servo command. This is the weight of a new
# measurement in the smoothed estimate: low enough to settle, high enough not to
# lag a person walking across the frame.
#
# **What is smoothed is the face's angle, not its pixel, and the difference is not
# a nicety.** A pixel means nothing on its own -- it is a position in a particular
# frame, taken from a particular camera pose -- so averaging the pixels of two
# frames taken while the camera was chasing averages two measurements of different
# things, and track() then reads the result as though it had all been measured at
# the newest pose. The face's angle is a fact about the room and stays true however
# the camera moves, so successive estimates of it can be averaged honestly.
#
# This was smoothing the pixel until 2026-08-19, and the cost was not subtle. Driving
# the real Target and Gimbal from a simulated loop at the rover's own 2.4 frames a
# second, with the servo modelled -- command latency, then travel at the speed the
# command carried -- a stationary face 39 degrees off centre went
#
#     250 -> 191 -> 41 -> 110 -> 141 -> 96 -> 40 -> 3 px from the middle
#
# reaching the middle on the third frame and then swinging back out past a fifth of
# the frame. From the corner it did not settle at all inside eight frames, and
# raising GAIN made it worse rather than better, which is the signature of a lag in
# the loop and not of a gain set too low. Smoothing the angle instead:
#
#     250 -> 191 -> 41 -> 62 -> 19 -> 6 -> 6 -> 6
#
# which is the monotone approach the gain was chosen to give.
#
# **The weight itself is a trade and 0.5 was the wrong end of it.** Filtering costs
# frames, and the thing it is paid to suppress is already suppressed by DEADBAND,
# which holds the camera still for anything under 2.6 degrees. Simulated on the same
# loop -- frames for a face to reach the middle, against the servo commands and the
# total travel provoked over forty frames by a *motionless* face whose box wanders:
#
#      weight    frames to settle       box wanders 3 px      wanders 8 px
#                39 deg   corner        cmds   travel         cmds   travel
#       0.3        10       12           1.2    2.2 deg       2.5    5.5 deg
#       0.5         7        9           1.7    3.2          5.7   11.6
#       0.7         5        6           1.8    3.4         10.8   21.8
#       1.0         5        5           2.5    4.8         17.5   43.2
#
# At three pixels of wander the filter buys nothing at all -- the deadband has
# already eaten it -- and at eight it is the only thing standing between a still
# face and a twitching servo. 0.7 takes essentially all of the settling speed while
# keeping half the protection, which is the right place to sit while the wander is
# unmeasured.
#
# **And it is unmeasured.** Measure it: track somebody sitting still and take the
# frame-to-frame change in `measured_at` out of tracking_status, which is the face's
# angle in the room and so is free of the camera's own motion. Divide by the lens's
# 0.197 degrees per pixel for the figure in the table above. Nobody was in front of
# the rover on 2026-08-19 when this was written.
SMOOTHING = 0.7


# A face is not lost the instant it is not detected -- a blink, a turn of the head
# or the motion blur of the servos moving all drop a frame or two. Hold the aim
# this long before admitting it is gone.
#
# **This is really a frame count wearing a stopwatch.** At the 25 fps this was
# measured at it is seventeen frames, which is generous; on the rover's own
# detector the loop runs at four, where the same 0.7 s is under three frames and a
# single turn of the head drops the lock. `Target.grace` is therefore settable, and
# the loops raise it when they are slow -- see GRACE_FRAMES.
LOST_GRACE_S = 0.7


# What that 0.7 s is worth in frames, and the floor to keep when frames are scarce.
GRACE_FRAMES = 4


# --- scanning -------------------------------------------------------------

# With no face in sight the camera sweeps its pan range end to end and reverses,
# at one fixed height. Not a raster over the whole field of regard: the mechanism
# can look anywhere from 30 degrees below the horizontal to 90 above, but almost
# none of that is anywhere a face will be. The rover sits on the floor, so a person
# standing or seated in front of it is always well above the camera, and time spent
# sweeping the floor is time not spent looking at people.
#
# The frame is 720 px tall at the measured 9.5 px per degree, so it takes in 76
# degrees at once: from here it sees roughly 7 to 83 degrees up, which covers
# anybody upright at conversational distance in one pass. Below about 7 degrees is
# given up deliberately -- that is the floor immediately in front of the rover.
SCAN_TILT = 45


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
#
# **Also a frame count wearing a stopwatch**, like LOST_GRACE_S. 25 deg/s at the
# 25 fps this was measured at is one degree between looks; on the rover's own
# detector, at four frames a second, it is six -- so the camera sweeps past
# somebody, acquires them from the frame where they were at the edge, and has
# already moved on by the time the correction is worked out. Cap the sweep by how
# far it may travel between two looks instead, which leaves the measured rate
# untouched wherever frames are plentiful.
SCAN_RATE = 90


# ...or rather, as much of a frame as may be crossed between two looks. Three
# degrees was set as an absolute, and on a lens this wide it is absurdly timid:
# the camera sees about 123 degrees at once, so a 3 degree step re-examines 97% of
# what it just looked at and a full turn takes the better part of a minute.
#
# What actually has to hold is that nothing is stepped *over*. Advancing a third
# of a frame between looks means every direction appears in three consecutive
# frames before it is left behind, which is generous cover for a face that is only
# found on four frames out of five. At this rover's rate that is about 40 degrees
# a look and a full sweep in four seconds, against fifty-one.
SCAN_FRAME_FRACTION = 1 / 3


def scan_rate_for(dt, half_frame=PAN_DEG_PER_HALF_FRAME, rate=SCAN_RATE):
    """The sweep rate a loop running at this frame period can actually follow.

    `half_frame` is the gimbal's own degrees-per-half-frame for the mode being
    captured, so the sweep follows the lens rather than a number written down for
    one of them -- pass `gimbal.pan_gain`.
    """
    if dt <= 0:
        return rate
    return min(rate, half_frame * 2 * SCAN_FRAME_FRACTION / dt)


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


def clamp(value, low, high):
    return min(max(value, low), high)


def counts(degrees_per_second):
    """Degrees a second -> the servo's own speed units, as T:134 wants them.

    Never zero: the firmware constrains SX and SY to 1..2500, and zero would mean
    unlimited in any case -- the very thing being avoided here.
    """
    wanted = min(degrees_per_second, SERVO_MAX_DEG_S) * COUNTS_PER_DEGREE
    return int(clamp(round(wanted), 1, SERVO_MAX_COUNTS))


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

    Faces arrive as (x, y, w, h, score) in full-frame pixels, whoever found them.
    """

    def __init__(self, acquire_score=ACQUIRE_SCORE, grace=LOST_GRACE_S):
        # Whose score has to be beaten to *start* a lock. An argument rather than
        # the constant, because it belongs to the detector rather than to aiming.
        self.acquire_score = acquire_score
        # How long a lock survives without a detection. Raised by a slow loop, so
        # that "a frame or two" stays a frame or two rather than becoming none.
        self.grace = grace
        self.centre = None   # (x, y) of the last detection, as the detector gave it
        self.box = None      # the last raw detection, for drawing
        # Whether the *last* update had a detection of its own, as opposed to
        # holding the lock open on grace. The caller has to know: `centre` is a
        # pixel, and a pixel is only meaningful against the frame it came from.
        # See Gimbal.keep_going() for what goes wrong when the two are confused.
        self.fresh = False
        # Now, rather than zero: the scan delay counts from this, and a rover that
        # has only just started has not been failing to find anyone since the epoch.
        self.seen_at = time.monotonic()

    def update(self, faces, now):
        self.fresh = False
        if not faces:
            return self.centre is not None and now - self.seen_at < self.grace
        pick = None
        if self.centre is not None and now - self.seen_at < self.grace:
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
            strong = [face for face in faces if face[4] >= self.acquire_score]
            if not strong:
                # Something face-shaped, but not enough to point the camera at.
                return self.centre is not None and now - self.seen_at < self.grace
            pick = max(strong, key=lambda f: f[2] * f[3])
        x, y, w, h, _ = pick
        # Unsmoothed, deliberately. This is where the face was in *this* frame, and
        # both the things that read it need that: the proximity gate above is asking
        # which detection is the same person, and track() is about to say what angle
        # this frame's pixel corresponds to. The averaging that used to happen here
        # is in Gimbal.track() now, on the angle -- see SMOOTHING.
        self.centre = (x + w / 2, y + h / 2)
        self.box = pick
        self.seen_at = now
        self.fresh = True
        return True

    def drop(self):
        self.centre = self.box = None

    def locked(self, now):
        return self.centre is not None and now - self.seen_at < self.grace


class Gimbal:
    """Where the camera is pointed, in the firmware's degrees. A model, not a reading.

    The firmware has a getGimbalFeedback() that no JSON command reaches, so where
    the camera is pointing is only known by having put it there. Kept true by
    centring at startup and being the only thing commanding the servos thereafter.
    Commands go out only when the rounded target changes, so a still subject is
    silent on the wire.
    """

    def __init__(self, gain=GAIN, frame_size=(1280, 720)):
        self.pan = 0.0
        self.tilt = REST_TILT_DEG
        self.gain = gain
        # The lens of the mode actually being captured, and where the middle of its
        # picture points. Both are fixed for the life of the loop, so they are
        # worked out once here rather than per frame.
        self.frame_size = frame_size
        self.lens = lens_for(*frame_size)
        self.middle = ray_at(frame_size[0] / 2.0, frame_size[1] / 2.0, self.lens)
        # Degrees per half frame, which no longer aim anything but still size the
        # sweep and the deadband -- see gains_for().
        self.pan_gain, self.tilt_gain = gains_for(*frame_size)
        self.sent = None
        # How fast each axis is currently meant to be travelling, degrees a second.
        # Sent with every command so the servo paces itself across the gap between
        # frames instead of arriving instantly and waiting -- see COUNTS_PER_DEGREE.
        self.speed_pan = PLACE_DEG_S
        self.speed_tilt = PLACE_DEG_S
        # Where the camera has been told to point, and when. Read back one dead
        # time later to find out what the angles were when a frame was exposed.
        self.history = collections.deque([(0.0, 0.0, 0.0, 0.0, 0.0)])
        # The face's angle as last worked out from a picture. An angle rather than
        # a pixel, because an angle is a fact about the world and stays true while
        # the camera moves -- which is exactly what keep_going() needs.
        self.aim_pan = None
        self.aim_tilt = None
        # What the last step was computed from -- see track(), which fills it in.
        # Aiming is the one part of this rover whose faults are all invisible in
        # the result: a mirrored picture, a wrong gain, a stale exposure clock and
        # a servo that has not arrived yet all present as the same symptom, a
        # camera that will not settle on a face it can plainly see. They are told
        # apart only by the intermediate numbers, so the intermediate numbers are
        # kept.
        self.last = None

    def begin(self, now, pan, tilt):
        """Seed the angles and the history together, before the loop starts.

        The two have to be set at once. Assigning `pan`/`tilt` alone leaves the
        history holding the placeholder this object was built with, and was_at()
        then answers every exposure older than the first frame with **zero** --
        which is not where the camera is, so the first corrections of every lock
        are computed against a position it was never in. Seen on the rover: the
        camera parked at pan 60, and the first four frames aimed as though it were
        pointing straight ahead.
        """
        self.pan, self.tilt = pan, tilt
        self.history.clear()
        self.history.append((now, pan, tilt, 0.0, 0.0))

    def record(self, now):
        # The speeds go in beside the angles because a command is not a position:
        # it is a move that takes time, and was_at() has to be able to say how far
        # along it the camera had got. See there for what this costs when omitted.
        self.history.append((now, self.pan, self.tilt, self.speed_pan, self.speed_tilt))
        while len(self.history) > 1 and self.history[0][0] < now - HISTORY_S:
            self.history.popleft()

    def was_at(self, when):
        """Where the camera actually was at `when` -- along its move, not at the end.

        The obvious reading of the history is "the most recent angle commanded at
        or before that moment", and it is wrong whenever the servo is still on its
        way there. It is *always* still on its way there on this rover: the ceiling
        is SERVO_MAX_DEG_S, so a 40 degree correction takes 310 ms, and the frame
        that arrives next was exposed about 60 ms into that move -- eight degrees
        in, with thirty-two still to go.

        Reporting the destination therefore tells the controller the correction has
        already happened, so it issues the whole thing again. Simulated at this
        rover's frame rate, that is the difference between overshooting a face by
        **48 degrees and by 19**, and it is what kept the gain pinned at 0.4.

        So each entry is read as the move it actually was: starting from the angle
        before it, at the speed it was sent with, for as long as it has had. An
        exposure older than the whole history reads back the earliest angle there
        is -- stale, but not wrong.
        """
        history = self.history
        found = 0
        for i, entry in enumerate(history):
            if entry[0] > when:
                break
            found = i
        entry = history[found]
        if found == 0:
            return entry[1], entry[2]
        start = history[found - 1]
        elapsed = max(0.0, when - entry[0])

        def along(from_angle, to_angle, speed):
            gap = to_angle - from_angle
            if not gap:
                return to_angle
            moved = min(abs(gap), max(speed, 0.0) * elapsed)
            return from_angle + math.copysign(moved, gap)

        return (along(start[1], entry[1], entry[3]),
                along(start[2], entry[2], entry[4]))

    def track(self, error_x, error_y, dt, now, exposed_at=None):
        """One step towards a face at (error_x, error_y), each -1..1 from centre.

        error_y is positive upwards, unlike the picture's rows, so that both axes
        share the sign of the servo they drive and neither needs a flip hidden in
        the middle of the arithmetic.

        `exposed_at` is when the frame this error came from was taken, on the same
        clock as `now`. Pass it whenever it can be known -- V4L2 stamps every
        buffer at start of exposure, and the detector on MEDIA hands that stamp
        back with the boxes -- because the alternative is DEAD_TIME_S, a constant
        that was measured once and varied by +-50 ms while being measured.

        The error locates the face relative to where the camera pointed *then*.
        Adding the two gives the face's angle outright -- a fixed thing,
        independent of everything commanded since -- and what is worth correcting
        is the gap between that and where the camera is already on its way to.
        Subtracting the in-flight motion this way is the whole difference between
        converging and running away.

        The axes are deadbanded separately: a face level with the camera but off to
        one side should be panned to without the tilt twitching along with it.
        """
        if exposed_at is None:
            # No exposure clock: DEAD_TIME_S already covers both delays together.
            when = now - DEAD_TIME_S
        else:
            # The camera reaches an angle COMMAND_LATENCY_S after being told to,
            # so where it was pointing when this frame was exposed is the angle
            # commanded that much earlier. Leaving this out is what made it hunt.
            when = exposed_at - COMMAND_LATENCY_S
        pan_then, tilt_then = self.was_at(when)
        # Where the face is, as an angle. The signs live here, and only here: they
        # say which way the picture's axes run against the servos', so they belong
        # on the pixel and not on the answer -- flipping the answer of a coupled
        # solve() would give a mirrored rover a differently wrong tilt as well.
        width, height = self.frame_size
        x = width / 2.0 * (1.0 + PAN_SIGN * error_x)
        y = height / 2.0 * (1.0 - TILT_SIGN * error_y)
        # What one move from where the camera was would have to be to put that
        # pixel in the middle of the picture. Adding it to where the camera was
        # gives the face's angle outright.
        step_pan, step_tilt = solve(ray_at(x, y, self.lens), self.middle, tilt_then)
        face_pan = pan_then + step_pan
        face_tilt = tilt_then + step_tilt
        # Smoothed here, where the quantity is an angle in the room rather than a
        # pixel in a picture -- see SMOOTHING for what averaging the pixel cost. A
        # lock that has just been taken has nothing to average against and starts
        # where the face is, exactly as Target used to for the same reason.
        if self.aim_pan is None:
            self.aim_pan, self.aim_tilt = face_pan, face_tilt
        else:
            self.aim_pan += SMOOTHING * (face_pan - self.aim_pan)
            self.aim_tilt += SMOOTHING * (face_tilt - self.aim_tilt)
        remaining_pan = self.aim_pan - self.pan
        remaining_tilt = self.aim_tilt - self.tilt
        # Deadband in degrees, on what is left to do rather than on what can be
        # seen: a face already centred in a stale frame may still need correcting
        # back if the camera has since been sent past it.
        step_pan = 0.0 if abs(remaining_pan) < DEADBAND * self.pan_gain \
            else self.gain * remaining_pan
        step_tilt = 0.0 if abs(remaining_tilt) < DEADBAND * self.tilt_gain \
            else self.gain * remaining_tilt
        self.move(
            clamp(step_pan, -PAN_RATE * dt, PAN_RATE * dt),
            clamp(step_tilt, -TILT_RATE * dt, TILT_RATE * dt),
            dt,
        )
        self.last = {
            # Read left to right, this is the whole chain: the face was this far
            # off centre, in a frame exposed this long ago, when the camera was
            # pointed there; so the face is at that angle; so go here.
            "error": [round(error_x, 3), round(error_y, 3)],
            "frame_age_ms": None if exposed_at is None else round((now - exposed_at) * 1e3),
            "was_at": [round(pan_then, 1), round(tilt_then, 1)],
            "face_at": [round(self.aim_pan, 1), round(self.aim_tilt, 1)],
            "measured_at": [round(face_pan, 1), round(face_tilt, 1)],
            "sent_to": [round(self.pan, 1), round(self.tilt, 1)],
        }

    def keep_going(self, dt):
        """Carry on to the angle already worked out, without re-reading the picture.

        For a frame the detector answered with nothing, while Target is still
        holding the lock open on its grace window. **Do not call track() with the
        remembered pixel on such a frame.** The face's angle has not changed, but
        the pixel it was measured at has, because the camera has moved since; and
        track() measures a pixel against where the camera is *now*. Feeding it a
        stale pixel therefore states that the face is still the full original
        distance away, and the whole correction is applied a second time on top of
        the one already in flight.

        Measured in simulation, at this rover's frame rate with detections landing
        on about half the frames: the camera crossed a stationary face 37 degrees
        off axis, carried **23 degrees past it**, and hunted. Holding the angle
        instead settles within 1 degree. The same fault at 25 frames a second
        costs 11 degrees, which is why it survived for as long as the detector was
        on another machine and the loop was fast -- it read as a slight wobble.
        """
        if self.aim_pan is None:
            return
        remaining_pan = self.aim_pan - self.pan
        remaining_tilt = self.aim_tilt - self.tilt
        step_pan = 0.0 if abs(remaining_pan) < DEADBAND * self.pan_gain             else self.gain * remaining_pan
        step_tilt = 0.0 if abs(remaining_tilt) < DEADBAND * self.tilt_gain             else self.gain * remaining_tilt
        self.move(
            clamp(step_pan, -PAN_RATE * dt, PAN_RATE * dt),
            clamp(step_tilt, -TILT_RATE * dt, TILT_RATE * dt),
            dt,
        )

    def forget(self):
        """Stop aiming at a remembered angle. For when the lock is given up."""
        self.aim_pan = self.aim_tilt = None

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
            # Capped, so a slow loop does not mean a camera that is never still --
            # see SETTLE_WITHIN_S, where the blur this costs is measured.
            travel = min(dt, SETTLE_WITHIN_S)
            self.speed_pan = abs(self.pan - was_pan) / travel
            self.speed_tilt = abs(self.tilt - was_tilt) / travel

    def centre(self, speed=PLACE_DEG_S):
        """Back to rest: straight ahead, and REST_TILT_DEG above level."""
        self.pan, self.tilt = 0.0, float(REST_TILT_DEG)
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


class Scan:
    """The sweep run when there is no face to follow.

    Pan crosses its whole range and reverses at each end, at the one height where
    faces are: SCAN_TILT, and never anywhere else. Tilt is not swept at all, which
    is the difference between looking for people and surveying a room.

    Two states rather than both axes at once, still: the camera settles onto the
    scanning height first and only then starts across. Moving both together would
    sweep a diagonal, and the corner it cut would be the part of the first pass
    most likely to hold somebody standing in front of the rover.

    That settling move is paced at the same rate as the pan sweep rather than
    taken at the servo's own speed, because it is a second of looking at the room
    like any other -- there is no reason to blur through it.
    """

    def __init__(self, gimbal):
        # Carry on the way it is already facing, so switching from tracking back to
        # scanning does not begin by swinging across ground it can already see.
        self.direction = 1 if gimbal.pan <= 0 else -1
        self.levelling = abs(gimbal.tilt - SCAN_TILT) > 1.0

    def step(self, gimbal, rate, dt):
        if self.levelling:
            remaining = SCAN_TILT - gimbal.tilt
            if abs(remaining) <= rate * dt:
                gimbal.move(0, remaining, dt)
                self.levelling = False
            else:
                gimbal.move(0, math.copysign(rate * dt, remaining), dt)
            return
        gimbal.move(self.direction * rate * dt, 0, dt)
        if abs(gimbal.pan) >= PAN_LIMIT:
            self.direction = -self.direction

    def state(self):
        return "sweeping, levelling" if self.levelling else "sweeping"


class Ahead:
    """Where the camera looks while the rover is driving and nobody is in view.

    Straight forward, at the same height the sweep uses, and then still. It is the
    sweep's sibling and takes the same `step(gimbal, rate, dt)`, so the loop can
    hold one or the other and never both.

    A sweep is the wrong thing to do with a camera on a moving rover. Panning and
    driving compound: the picture moves at the sum of the two, so the smear the
    sweep rate was measured against is no longer what the detector gets, and the
    one direction the camera is *not* looking often enough is the one the rover is
    about to arrive in. Somebody the rover is driving towards walks into this view
    by itself, which is the case the sweep exists to cover and the one case it
    cannot help with here.

    Held at SCAN_TILT rather than level for the reason that number exists: the
    rover is on the floor and faces are above it. Level would spend the drive
    looking at carpet.
    """

    #: Within this many degrees of the pose counts as arrived. A degree is the
    #: resolution the command is rounded to anyway -- see Gimbal.changed() -- so
    #: anything finer is a servo command the board would not act on.
    CLOSE_DEG = 1.0

    def __init__(self, gimbal):
        self.turning = self._far(gimbal)

    def _far(self, gimbal):
        return (abs(gimbal.pan) > self.CLOSE_DEG
                or abs(gimbal.tilt - SCAN_TILT) > self.CLOSE_DEG)

    def step(self, gimbal, rate, dt):
        """Walk to the pose at the sweep's own pace, then stop moving.

        Paced rather than placed, because the trip is seconds of looking at the
        room like any other: a camera slewed across at the servo's top speed sees
        nothing on the way, and somebody standing off to one side is exactly who
        this is turning away from. Both axes at once, unlike the sweep, which
        separates them so its first pass is not a diagonal -- here there is no
        pass to spoil, only a pose to reach.
        """
        if not self.turning:
            return
        travel = rate * dt
        gimbal.move(clamp(-gimbal.pan, -travel, travel),
                    clamp(SCAN_TILT - gimbal.tilt, -travel, travel), dt)
        self.turning = self._far(gimbal)

    def state(self):
        return "watching ahead, turning back" if self.turning else "watching ahead"
