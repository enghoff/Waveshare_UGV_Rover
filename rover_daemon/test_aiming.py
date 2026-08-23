"""Aiming-loop checks: missed frames, one-shot centring, approach."""
from __future__ import annotations

import math

from test_harness import check


def test_aiming_through_a_missed_frame():
    """A face that never moves must not push the camera further on every miss.

    The detector on this rover answers about half the frames, and the lock is
    deliberately held open across the gaps. What must not happen in a gap is a
    second correction for the same error: `centre` is a pixel, it was measured
    against an older frame, and the camera has moved since. Feeding it back to
    track() walked the camera 23 degrees past a stationary face 37 away, and it
    read from outside as the rover avoiding the person it could plainly see.
    """
    import aiming
    from aiming import Gimbal, Target

    width, height = 640, 480
    face_px = (480, 240)               # half a frame right of centre, and still
    detection = [[face_px[0] - 40, face_px[1] - 40, 80, 80, 0.95]]
    dt, now = 0.4, 1000.0

    gimbal = Gimbal(aiming.GAIN, (width, height))
    target = Target(0.65, grace=max(aiming.LOST_GRACE_S, aiming.GRACE_FRAMES * dt))

    def turn(faces):
        nonlocal now
        now += dt
        if target.update(faces, now):
            if target.fresh:
                gimbal.track((target.centre[0] - width / 2) / (width / 2),
                             (height / 2 - target.centre[1]) / (height / 2),
                             dt, now, exposed_at=now - 0.05)
            else:
                gimbal.keep_going(dt)
        gimbal.record(now)

    turn(detection)
    after_one, aim = gimbal.pan, gimbal.aim_pan
    check("one look at a face off to the right pans right", after_one > 1.0, True)
    for _ in range(3):
        turn([])                        # the grace window, with nothing detected
    # Converging on the angle is right; passing it is the fault. keep_going()
    # closes the remaining gap and stops there however many frames are missed.
    check("missed frames keep closing on the face", gimbal.pan > after_one, True)
    check("missed frames never pan past it", gimbal.pan <= aim + 0.1, True)

    # And the fault itself, kept beside the fix so the number is not just a
    # claim in a docstring: re-reading the remembered pixel each missed frame,
    # which is what the loop used to do, sails past the face instead.
    naive = Gimbal(aiming.GAIN, (width, height))
    naive_target = Target(0.65, grace=999.0)
    when = 2000.0
    for turn_number in range(4):
        when += dt
        faces = detection if turn_number == 0 else []
        if naive_target.update(faces, when):
            naive.track((naive_target.centre[0] - width / 2) / (width / 2),
                        (height / 2 - naive_target.centre[1]) / (height / 2),
                        dt, when, exposed_at=when - 0.05)
        naive.record(when)
    check("re-reading the stale pixel would pan well past it",
          naive.pan > aim + 10, True)

    # Once the lock is given up, the remembered angle goes with it: the next
    # person is not aimed at through the last one's coordinates.
    target.drop()
    gimbal.forget()
    check("dropping the lock forgets the angle", gimbal.aim_pan, None)
    before = gimbal.pan
    gimbal.keep_going(dt)
    check("and keep_going then does nothing", gimbal.pan, before)


def test_one_move_puts_a_face_in_the_middle():
    """Aiming at a corner of the frame must arrive, not set off in roughly the way.

    The gimbal pans about the world's vertical and then tilts about its own
    horizontal, and the lens is a 130 degree fisheye, so the degrees that centre a
    face are not the horizontal error times one number and the vertical error times
    another. That is what this file used to do. Measured on the rover it left a face
    up to 20 degrees out -- a sixth of the frame -- and worst of all in the ordinary
    case of the camera already tilted up at somebody standing.

    So the check is the whole claim end to end: put a face at a pixel, let track()
    make one correction at full gain, then work out where that pixel has got to and
    insist it is in the middle. Nothing here consults the model being tested -- the
    forward direction is turned by the pose that was commanded -- so a solve() with
    its arctangents in the wrong order fails rather than agreeing with itself.
    """
    import aiming
    from aiming import Gimbal

    size = (640, 480)
    lens = aiming.lens_for(*size)
    middle = aiming.ray_at(size[0] / 2, size[1] / 2, lens)

    def turned(vector, axis, degrees):
        """The same fixed direction, seen from a camera that has turned. y is down."""
        angle = math.radians(degrees)
        cos, sin = math.cos(angle), math.sin(angle)
        x, y, z = vector
        if axis == "pan":                       # about the vertical, positive right
            return (x * cos - z * sin, y, x * sin + z * cos)
        return (x, y * cos + z * sin, -y * sin + z * cos)   # positive up

    def where_it_ends_up(face, tilt_now, pan, tilt):
        fixed = turned(aiming.ray_at(face[0], face[1], lens), "tilt", -tilt_now)
        return turned(turned(fixed, "pan", pan), "tilt", tilt)

    def off_by(seen):
        together = sum(a * b for a, b in zip(seen, middle))
        return math.degrees(math.acos(min(1.0, together)))

    def one_correction(face, tilt_now):
        gimbal = Gimbal(1.0, size)              # all of it, in one step
        gimbal.begin(1000.0, 0.0, tilt_now)
        gimbal.track((face[0] - size[0] / 2) / (size[0] / 2),
                     (size[1] / 2 - face[1]) / (size[1] / 2),
                     1.0, 1000.5, exposed_at=1000.5)
        return gimbal.pan, gimbal.tilt

    # Corners, edges and the two centre lines, from level and from tilted up, which
    # is where the old arithmetic went furthest wrong.
    worst = 0.0
    for face in ((600, 60), (600, 420), (40, 60), (40, 420), (600, 240), (320, 40),
                 (320, 240), (500, 150)):
        for tilt_now in (0.0, 15.0, 30.0):
            pan, tilt = one_correction(face, tilt_now)
            worst = max(worst, off_by(where_it_ends_up(face, tilt_now, pan, tilt)))
    check(f"one move centres a face anywhere in the frame (worst {worst:.2f} deg)",
          worst < 0.5, True)

    # And the fault it replaced, so the size of it is recorded rather than claimed.
    # This is the arithmetic that used to be in track(): the two errors, each times
    # a degrees-per-half-frame constant, with no coupling between them.
    face, tilt_now = (600, 60), 30.0
    pan_gain, tilt_gain = aiming.gains_for(*size)
    separable = ((face[0] - size[0] / 2) / (size[0] / 2) * pan_gain,
                 tilt_now + (size[1] / 2 - face[1]) / (size[1] / 2) * tilt_gain)
    check("the separable version it replaced would have missed by 10 degrees or more",
          off_by(where_it_ends_up(face, tilt_now, *separable)) > 10.0, True)

    # A frame size nobody has measured must still aim, and aim sensibly: the desk
    # script offers 1280x720 and a caller may ask for anything its camera does.
    for size_tried in ((1280, 720), (320, 240), (800, 600)):
        pan_half, tilt_half = aiming.gains_for(*size_tried)
        check(f"{size_tried[0]}x{size_tried[1]} has a believable half frame",
              60 < pan_half < 75 and 30 < tilt_half < 60, True)


def test_the_approach_to_a_face_never_turns_back():
    """Following a face must close on it, not close on it and swing out again.

    A whole simulated loop, because the fault this catches is not visible in any one
    call: the camera comes most of the way to a face on the third frame, then heads
    back out past a fifth of the frame before returning. What did that was smoothing
    the face's *pixel* across frames -- a pixel is a position in one picture taken
    from one camera pose, and averaging two of them while the camera is moving
    averages measurements of two different things. Gimbal.track() then reads the
    average as though it were all measured at the newest pose, which is a lag inside
    the feedback path, and a lag inside the feedback path rings.

    The tell is in the last check: with the fault present, *raising* the gain makes
    the overshoot worse rather than the approach quicker. That is what says the loop
    is not merely sluggish.
    """
    import aiming
    from aiming import Gimbal, Target

    size = (640, 480)
    lens = aiming.lens_for(*size)

    def turned(vector, axis, degrees):
        angle = math.radians(degrees)
        cos, sin = math.cos(angle), math.sin(angle)
        x, y, z = vector
        if axis == "pan":
            return (x * cos - z * sin, y, x * sin + z * cos)
        return (x, y * cos + z * sin, -y * sin + z * cos)

    def seen_from(world, pan, tilt):
        """Where a fixed direction lands in the picture, at this pose."""
        x, y, z = turned(turned(world, "pan", pan), "tilt", tilt)
        flat = math.hypot(x, y)
        theta = math.atan2(flat, z)
        radius, scale, bend = theta / lens[0], lens[0], lens[1]
        for _ in range(20):               # invert theta_of, which is monotone
            guess = aiming.theta_of(radius, lens)
            slope = scale * (1.0 + 3.0 * bend * (radius / lens[4]) ** 2)
            radius -= (guess - theta) / slope
        along = radius / flat if flat > 1e-9 else 0.0
        return lens[2] + x * along, lens[3] + y * along

    def approach(start_px, gain, frames=8):
        """How far the face is from the middle, frame by frame, in pixels."""
        dt = 0.42                          # the rate the rover's own detector gives
        gimbal = Gimbal(gain, size)
        target = Target(0.65, grace=max(aiming.LOST_GRACE_S, aiming.GRACE_FRAMES * dt))
        now = 1000.0
        gimbal.begin(now, 0.0, 0.0)
        world = aiming.ray_at(*start_px, lens)
        pose, offsets = (0.0, 0.0), []
        for _ in range(frames):
            now += dt
            x, y = seen_from(world, *pose)
            offsets.append(math.hypot(x - size[0] / 2, y - size[1] / 2))
            if target.update([[x - 45, y - 45, 90, 90, 0.95]], now):
                gimbal.track((target.centre[0] - size[0] / 2) / (size[0] / 2),
                             (size[1] / 2 - target.centre[1]) / (size[1] / 2),
                             min(dt, aiming.MAX_DT), now, exposed_at=now - 0.2)
            gimbal.record(now)
            pose = (gimbal.pan, gimbal.tilt)

        return offsets

    for start in ((570, 240), (600, 60), (60, 400)):
        offsets = approach(start, aiming.GAIN)
        # Never further from the middle than it was two frames ago. Two rather than
        # one because a frame exposed part way through a move can legitimately read
        # a little worse than the one before it.
        turning_back = max((offsets[i] - offsets[i - 2] for i in range(2, len(offsets))),
                           default=0.0)
        check(f"the approach from {start} closes on the face "
              f"(worst turn back {turning_back:+.0f} px)", turning_back < 8.0, True)
        check(f"and gets inside a fifth of the frame from {start}",
              min(offsets) < size[0] / 10, True)

    # More gain must buy a quicker approach, not a bigger swing. Where the loop
    # carries a lag it buys the opposite, which is how this fault announces itself.
    slow, fast = approach((600, 60), 0.4), approach((600, 60), 1.0)
    check("more gain arrives sooner rather than overshooting further",
          min(fast) <= min(slow) + 1.0, True)
