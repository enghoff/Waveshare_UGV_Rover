"""The lens this camera actually has, and the geometry that follows from it.

Split out of aiming.py because it is the one part of aiming that is a property of
the *hardware* rather than of the algorithm: a changed lens, or a servo horn
refitted a spline out, makes everything here wrong and nothing in the control loop
beside it wrong. It is also the part with no state and no clock -- pixels in,
angles out -- so it can be read, checked and re-measured on its own.

The provenance rides along with each number, because a number without its recipe
cannot be re-measured when the hardware changes. `lens_recipe()` at the end is
that recipe, written out in full; it is never called.

Everything here is re-exported by aiming.py, which is what callers import.
"""

import math


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
#
# **These two are YuNet's numbers and no other detector's.** They are a property
# of how a particular network scores this room's furniture, not of how tracking
# should behave, so a different detector needs its own pair measured the same way.
# That is not hypothetical: while the rover detected on the OAK camera's VPU, the
# SSD it ran there scored both faces and furniture lower -- a standing person came
# out at 0.71 where nothing in an empty room passed 0.60 -- and it never acquired
# anybody at all against a bar set here. `Target` takes the pair as arguments for
# exactly that reason, and both callers now pass these because both now run YuNet.
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
#    640x480       5.22    5.32          61      45
#
# The consequence is that the constants calibrated at 720p are wrong when the rover
# runs at 480p -- in the direction of under-correcting, so a loop that used them was
# merely sluggish rather than unstable, which is exactly the kind of error that
# survives a long time.
#
# **Let the servo arrive before taking the second frame.** The 480p pair was first
# measured at 4.36 and 4.90 with two and a half seconds of settling, which is not
# enough for a 25 degree step: the picture had not finished moving, so the shift
# came out short and the figures came out low -- by 20% in pan and 9% in tilt, both
# in the same direction, which is the signature. Re-measured 2026-08-18 with five
# seconds, +-15 and +-25 degrees in pan and +-12 and +-20 in tilt, all four readings
# on each axis within 2% of each other and template matches 0.88-0.97. The 720p pair
# above was taken the same way as the old 480p one and has never been re-measured,
# so treat it as suspect for the same reason.
# **The lens, not a pixels-per-degree pair.** What used to be here was one number
# per axis and a multiplication: a face this fraction of a half frame to the right
# is that many degrees to the right. That is only true along the two centre lines.
# Everywhere else it is wrong, and measurably so -- see lens_recipe() for what a
# probe on the rover found and what replaced it.
#
# A mode is a window onto the sensor as well as a pixel count, and this camera's
# 16:9 mode is a *crop* rather than a letterboxed 4:3, so the two modes do not see
# the same angle and one cannot be derived from the other.
LENS = {
    # Measured 2026-08-19 by usb_cameras/calibrate_fov.py, two sweeps that share no
    # motion: panning gave 11.85 arcmin per pixel with a distortion term of +0.025,
    # tilting 11.79 and +0.035. Half a percent apart on the scale, which is the part
    # the aiming leans on. The centre comes one axis from each run, because a sweep
    # pins the coordinate it moves along and says next to nothing about the other.
    (640, 480): (11.82, 0.030, (315.9, 227.4)),
    # Converted from the old template-match pair (9.65 and 9.50 pixels per degree,
    # which agree to 1.5% and so describe an equidistant lens at 9.575) and never
    # sweep-fitted. Treat the distortion term and the centre as guesses: this mode
    # is not what the rover captures, and the desk script that does use it should
    # have calibrate_fov.py run on it before anything is concluded from it.
    (1280, 720): (6.27, 0.0, None),
}


# What half a frame comes to in degrees, kept because the README and the docstrings
# quote them and because the sweep and the deadband are still sized in frames. Now
# read off the lens rather than measured separately, so there is one description of
# the optics and not two that can disagree.
PAN_DEG_PER_HALF_FRAME = 65


TILT_DEG_PER_HALF_FRAME = 48


def lens_for(width, height):
    """(radians per pixel on the axis, distortion, centre x, centre y, normal).

    An unmeasured mode is scaled from a measured one of the same shape, since within
    one crop the angular scale goes with the pixel count and the distortion term is
    written against a normalised radius and so does not. A mode of some *other*
    shape is a different window onto the sensor and cannot be derived at all, so it
    falls back to the mode this rover captures and should be measured.
    """
    known = LENS.get((width, height))
    scale, bend, centre = known if known else (None, None, None)
    if known is None:
        shape = round(width / height, 2)
        for (w, h), (arcmin, term, middle) in LENS.items():
            if round(w / h, 2) == shape:
                scale, bend = arcmin * w / width, term
                centre = None if middle is None else \
                    (middle[0] * width / w, middle[1] * height / h)
                break
    if scale is None:
        scale, bend, centre = LENS[(640, 480)]
    if centre is None:
        centre = (width / 2.0, height / 2.0)
    return math.radians(scale / 60.0), bend, centre[0], centre[1], width / 2.0


def theta_of(radius, lens):
    """Angle off the lens axis, in radians, for a point this many pixels out.

    An equidistant fisheye puts angle in proportion to radius -- which is what this
    camera turned out to be, near enough -- and the distortion term is the one thing
    that lets the fit say otherwise. It is written against a normalised radius so
    that it comes out around a hundredth rather than around 1e-9. The same function
    is in usb_cameras/calibrate_fov.py, which fits it; this is the one that flies.
    """
    scale, bend, _, _, normal = lens
    return radius * scale * (1.0 + bend * (radius / normal) ** 2)


def ray_at(x, y, lens):
    """The direction a pixel looks along: x right, y down, z out of the lens."""
    _, _, cx, cy, _ = lens
    dx, dy = x - cx, y - cy
    radius = math.hypot(dx, dy)
    if radius < 1e-9:
        return 0.0, 0.0, 1.0
    theta = theta_of(radius, lens)
    across = math.sin(theta) / radius
    return dx * across, dy * across, math.cos(theta)


def solve(seen, wanted, tilt_now):
    """Degrees of pan and tilt that move the direction `seen` onto `wanted`.

    One move, exactly, and no iterating towards it. **This is the whole correction
    that used to be two multiplications**, and the reason it cannot be two is that
    the gimbal pans about the world's vertical and then tilts about its own
    horizontal, so the axes are not independent: how much pan centres a face
    depends on how high in the frame the face is, and on how far the camera is
    tilted already.

    Undo the tilt first, which puts the direction back in the frame the pan turns
    within. A pan cannot change how far a direction sits out of that frame's
    forward plane, so the pan wanted is the one that leaves it exactly as far out
    as the destination is; the tilt that remains then follows outright.

    Measured on the rover, the old separable version left a face 2 degrees off at
    20 degrees from the middle, 5 to 9 at 35 to 45, and 13 to 20 -- a sixth of the
    frame -- out towards the corners, and it got worse the more the camera was
    already tilted. See lens_recipe().
    """
    cos, sin = math.cos(math.radians(tilt_now)), math.sin(math.radians(tilt_now))
    ax = seen[0]
    ay = seen[1] * cos - seen[2] * sin
    az = seen[1] * sin + seen[2] * cos
    across = math.hypot(ax, az)
    # Only reachable by pointing a camera at something further round than the
    # gimbal can swing it, which a face in the picture never is. Standing still
    # beats moving somewhere arbitrary.
    if across < 1e-12 or abs(wanted[0]) > across:
        return 0.0, 0.0
    pan = math.atan2(ax, az) - math.asin(wanted[0] / across)
    forward = math.sqrt(max(across * across - wanted[0] * wanted[0], 0.0))
    tilt = math.atan2(wanted[1], wanted[2]) - math.atan2(ay, forward)
    return math.degrees(pan), math.degrees(tilt) - tilt_now


def gains_for(width, height):
    """Degrees that swing the view by one half frame, in this capture mode.

    Not the aiming any more -- solve() does that -- but still what sizes the sweep
    and the deadband, both of which are naturally spoken in frames: step a third of
    a frame between looks, hold still while the face is within a twenty-fifth of
    one. Read off the lens so that there is nothing here to disagree with it.
    """
    lens = lens_for(width, height)
    return (math.degrees(theta_of(width / 2.0, lens)),
            math.degrees(theta_of(height / 2.0, lens)))


def lens_recipe():
    """How LENS was arrived at, and why it is a lens rather than two gains.

    Not called. Kept as the recipe, because LENS is the only thing in this file
    that is a property of the hardware rather than of the algorithm, and a changed
    lens or a servo horn refitted a spline out makes it wrong.

    **Measure it with usb_cameras/calibrate_fov.py**, which turns the camera by a
    known angle and fits the projection -- angular scale, one distortion term and
    the principal point -- to the motion of every feature in the room. Run it both
    ways, `--axis pan` and `--axis tilt`: they constrain different halves of the
    answer, the scale twice over and one coordinate of the centre each. On this
    rover, 2026-08-19, at 640x480:

        pan  sweep   11.85 arcmin per pixel, distortion +0.025, cx 315.9
        tilt sweep   11.79 arcmin per pixel, distortion +0.035, cy 227.4

    which is 129.6 by 96.2 degrees of room in one picture, and a lens axis thirteen
    pixels above the middle of the frame.

    **Then check the aiming itself with usb_cameras/calibrate_aim.py**, which is a
    different question and was the one that mattered: not how wide the lens is but
    whether the degrees this file works out actually put a face in the middle in
    one move. It cuts a patch out at a known pixel, commands what the model says,
    and measures where the patch really went. Measured that way on this rover,
    degrees still owed after a single correction:

        how far off the middle      the old separable gains      solve()
        20 degrees                        2.0 - 2.4                0.9 - 1.1
        35 to 45                          4.7 - 9.1                1.3 - 3.6
        50 to 65                          8.0 - 19.7               2.4 - 6.2

    Nineteen degrees is a sixth of the frame, and the error grows with the tilt
    already on -- about 9 degrees from level for a corner target, 21 from +15, 36
    from +30 -- so the old model was at its worst in the ordinary case of following
    somebody standing up. It converged anyway, because the loop re-measures every
    frame; it converged by walking, and at the two and a bit frames a second the
    on-board detector manages, walking reads as hunting.

    What is left is a degree or three, and most of it is not the model. The gimbal
    pivots a few centimetres behind the lens, so a subject nearer than about a
    metre moves by parallax as well as by rotation and no rotation-only model can
    take that out. Measure in a room with something far away in it.

    **The old recipe, kept because it is the cheap check.** Two gains, one per
    axis, measured by template matching rather than fitted:

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

    And again at 640x480, the mode the rover actually captures, after waiting five
    seconds rather than two and a half for the servo to arrive:

        pan  +15 deg  ->  the scene moved LEFT   78 px       5.20 px per degree
        pan  -15 deg  ->  the scene moved RIGHT  78 px       5.20
        pan  +25 deg  ->  the scene moved LEFT  132 px       5.28
        pan  -25 deg  ->  the scene moved RIGHT 130 px       5.20
        tilt +12 deg  ->  the scene moved DOWN   64 px       5.33
        tilt -12 deg  ->  the scene moved UP     64 px       5.33
        tilt +20 deg  ->  the scene moved DOWN  108 px       5.40
        tilt -20 deg  ->  the scene moved UP    104 px       5.20

    The signs agree with the 720p run in every case, which is the part worth
    re-reading: +pan moves the scene left, so +pan aims the camera right.

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

    Note that those two gains came out 5.22 and 5.32 pixels per degree where the
    sweep fit says 5.08, three per cent apart and in the direction that says the
    servo had still not quite arrived. That is the argument for the sweep: it uses
    thousands of points across twenty frames and throws out the ones that move in a
    way no rotation explains, where the template match uses one patch and believes
    whatever the picture did.
    """
