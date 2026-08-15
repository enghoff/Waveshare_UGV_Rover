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
when the hardware changes -- the recipe for the two aiming gains is in
aim_gains() below.

What is deliberately *not* here: anything that decodes a picture, opens a camera
or talks to the board. A caller supplies face boxes in full-frame pixels and a
clock, and gets angles back.
"""

import collections
import math
import time

# --- what counts as a face ------------------------------------------------

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

# --- aiming ---------------------------------------------------------------

# How far the picture moves for one commanded degree, by capture mode, measured
# with the probe in aim_gains(). Not one number: a mode is a window onto the
# sensor as well as a pixel count, and this camera's 16:9 mode is a *crop* rather
# than a letterboxed 4:3, so the two modes do not see the same angle and their
# scales are not a simple ratio.
#
#                  px per degree        degrees per half frame
#                  pan     tilt         pan     tilt
#   1280x720       9.65    9.50          66      38
#    640x480       4.36    4.90          73      49
#
# The consequence is that the constants calibrated at 720p are wrong by 10% in pan
# and 22% in tilt when the rover runs at 480p -- both in the direction of
# under-correcting, so a loop that used them was merely sluggish rather than
# unstable, which is exactly the kind of error that survives a long time.
PX_PER_DEGREE = {
    (1280, 720): (9.65, 9.50),
    (640, 480): (4.36, 4.90),
}
# What the constants were before the modes were told apart, and still the answer
# for 1280x720. Kept named because the README and the docstrings quote them.
PAN_DEG_PER_HALF_FRAME = 66
TILT_DEG_PER_HALF_FRAME = 38


def gains_for(width, height):
    """Degrees that swing the view by one half frame, in this capture mode.

    An unmeasured mode is scaled from a measured one of the same shape, since
    within one crop the pixels-per-degree goes with the pixel count. A mode of
    some *other* shape is a different window onto the sensor and cannot be
    derived at all, so it falls back to the 720p figures and should be measured.
    """
    measured = PX_PER_DEGREE.get((width, height))
    if measured is None:
        shape = round(width / height, 2)
        for (w, h), (pan, tilt) in PX_PER_DEGREE.items():
            if round(w / h, 2) == shape:
                measured = (pan * width / w, tilt * height / h)
                break
    if measured is None:
        return float(PAN_DEG_PER_HALF_FRAME), float(TILT_DEG_PER_HALF_FRAME)
    return (width / 2) / measured[0], (height / 2) / measured[1]


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
HISTORY_S = DEAD_TIME_S * 2

# The fraction of the *remaining* error to correct per frame -- remaining meaning
# what is left once motion already in flight is subtracted. With the dead time
# accounted for this is a genuine proportional gain again, and 0.5 halves the error
# every frame. Left lower than it could be because the compensation is only as good
# as the exposure time it is given.
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

    MEASURED that way, on this rover -- plain pixel coordinates, no sign
    conventions to misread (1280x720, matches 0.97-0.99):

        pan  +25 deg  ->  the scene moved LEFT  243 px       9.7 px per degree
        pan  -25 deg  ->  the scene moved RIGHT 240 px       9.6
        tilt +20 deg  ->  the scene moved DOWN  188 px       9.4
        tilt -20 deg  ->  the scene moved UP    192 px       9.6

    So +X pans right and +Y tilts up, symmetric both ways. A face right of centre
    is therefore centred by *raising* X, and one above centre by raising Y, which
    is what PAN_SIGN and TILT_SIGN say.

    A half frame is 640 px, so 66 of these degrees. An earlier pass concluded from
    that the firmware's "degrees" must be about half a real one, on the grounds
    that no sane lens is 132 degrees wide. The firmware source says otherwise:
    gimbalCtrlSimple maps them through map(X, 0, 360, 0, 4095), which is 11.375
    counts per degree against the ST3215's 4096 counts per turn -- one commanded
    degree is one real degree. The lens really is that wide, as the barrel
    distortion in any frame it takes will confirm. Note that this makes
    px-per-degree a centre-of-frame figure rather than a constant, which is
    harmless here: the point of the exercise is to drive the error towards the
    centre, where the figure is measured.

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

    def __init__(self):
        self.centre = None   # smoothed (x, y), the thing actually aimed at
        self.box = None      # the last raw detection, for drawing
        # Now, rather than zero: the scan delay counts from this, and a rover that
        # has only just started has not been failing to find anyone since the epoch.
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

    The firmware has a getGimbalFeedback() that no JSON command reaches, so where
    the camera is pointing is only known by having put it there. Kept true by
    centring at startup and being the only thing commanding the servos thereafter.
    Commands go out only when the rounded target changes, so a still subject is
    silent on the wire.
    """

    def __init__(self, gain=GAIN, frame_size=(1280, 720)):
        self.pan = 0.0
        self.tilt = 0.0
        self.gain = gain
        # Degrees per half frame, for the mode actually being captured -- see
        # gains_for(). Defaulting to 720p keeps every existing caller as it was.
        self.pan_gain, self.tilt_gain = gains_for(*frame_size)
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
        while len(self.history) > 1 and self.history[0][0] < now - HISTORY_S:
            self.history.popleft()

    def was_at(self, when):
        """The angles as they had been commanded at `when`.

        The most recent entry at or before that moment, which for a deque filled
        once a frame is never more than a frame stale. An exposure older than the
        history reads back its earliest entry -- stale, but not wrong.
        """
        found = self.history[0]
        for entry in self.history:
            if entry[0] > when:
                break
            found = entry
        return found[1], found[2]

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
        # describe how the picture relates to the servos, which is this conversion.
        face_pan = pan_then + PAN_SIGN * error_x * self.pan_gain
        face_tilt = tilt_then + TILT_SIGN * error_y * self.tilt_gain
        remaining_pan = face_pan - self.pan
        remaining_tilt = face_tilt - self.tilt
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
        return "scanning, levelling" if self.levelling else "scanning"
