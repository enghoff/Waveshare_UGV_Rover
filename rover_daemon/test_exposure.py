"""Camera exposure: telling a blank frame from a picture, and getting out of one.

The fault these cover is quiet rather than loud. This camera settles on an
exposure of its own, keeps it when the device is closed and reopened, and can end
up a long way from what the room needs; a capture then comes back pure white, and
nothing downstream can tell that from a photograph of an empty wall. A whole
validation drive was recorded that way on 2026-09-02 before a person opened one of
the pictures.

There is no camera here, so what is covered is the judgement and the plumbing: which
frames are called blank, and whether a blank one makes the capture reach for the
camera's controls and take the picture again. What the controls then do to the
hardware is on the rover, and only the rover can say.
"""
from __future__ import annotations

from test_fakes import HERE  # noqa: F401  -- importing it sets sys.path up
from test_harness import SKIP, check


def _jpeg(value, blown_rows=0):
    """A 640x480 grey picture at `value`, with `blown_rows` of it at full white."""
    import cv2
    import numpy

    frame = numpy.full((480, 640), value, numpy.uint8)
    if blown_rows:
        frame[:blown_rows, :] = 255
    return cv2.imencode(".jpg", frame)[1].tobytes()


def test_what_counts_as_a_blank_frame():
    """The line between a bright room and a camera that has not stopped down.

    Drawn where it is because of two measurements on the rover on 2026-09-03: the
    most extreme honest picture in this house -- the gimbal aimed into a sunlit
    window at midday -- has 13% of its pixels at full white, and the failure has
    been seen at 35% and at 89%. So a quarter is comfortably above one and well
    below the other.
    """
    try:
        from track_face_pi import nothing_in_it
    except ImportError as error:            # no OpenCV, or no aiming.py beside it
        SKIP.append(f"blank frames ({error})")
        return

    check("an ordinary room is a picture", nothing_in_it(_jpeg(120)), False)
    check("a bright room is still a picture", nothing_in_it(_jpeg(200)), False)
    # 62 rows of 480 is 13%: the window, which is a picture of a window.
    check("a window in the frame is still a picture",
          nothing_in_it(_jpeg(150, blown_rows=62)), False)
    check("a frame that is a quarter blown out is not",
          nothing_in_it(_jpeg(150, blown_rows=120)), True)
    check("pure white is not", nothing_in_it(_jpeg(255)), True)
    check("pure black is not", nothing_in_it(_jpeg(0)), True)
    check("a nearly black frame is not", nothing_in_it(_jpeg(4)), True)
    # Some light in the room is enough. The floor is far below any exposure a lit
    # scene produces, so a genuinely dark picture is not thrown away as broken.
    check("a dim room is a picture", nothing_in_it(_jpeg(20)), False)
    # A frame that will not decode is a different fault with its own report, and
    # must not be turned into this one.
    check("rubbish is not called blank", nothing_in_it(b"\xff\xd8not a picture\xff\xd9"),
          False)


def test_a_blank_picture_is_taken_again():
    """A capture that came back white must reach for the controls, not hand it on.

    The camera is stubbed out, because what is being checked is the decision: that
    a blank first capture leads to the exposure being restored and a second, longer
    capture taken, that the second one's picture is what comes back, and that a
    picture which was fine the first time costs neither of those.
    """
    try:
        import uvc_camera
        from track_face_pi import snapshot
    except ImportError as error:
        SKIP.append(f"retaking a blank picture ({error})")
        return
    try:
        _jpeg(120)
    except ImportError as error:            # no OpenCV: nothing can judge a frame
        SKIP.append(f"retaking a blank picture ({error})")
        return

    white, room = _jpeg(255), _jpeg(120)
    restored, asked, answers, pinned = [], [], [], []

    def capture(device, size, frames, timeout):
        """The next answer in `answers`, repeating the last one once they run out."""
        asked.append(frames)
        return [(answers[min(len(asked), len(answers)) - 1], 1.0)], ""

    was_capture, uvc_camera._capture = uvc_camera._capture, capture
    was_restore = uvc_camera.restore_automatic
    was_manual = uvc_camera.under_manual_control
    uvc_camera.restore_automatic = lambda device: restored.append(device)
    uvc_camera.under_manual_control = lambda device: bool(pinned)
    try:
        answers[:] = [white, room]
        got, why = snapshot("/dev/null", (640, 480), frames=3)
        check("a blank picture is taken again", len(asked), 2)
        check("...with the camera given longer to expose",
              asked[1] > asked[0], True)
        check("...after the camera is given its exposure back", restored, ["/dev/null"])
        check("...and the second picture is the one handed back", got[0][0], room)
        check("...with nothing to complain about", why, "")

        answers[:], asked[:], restored[:] = [room], [], []
        got, why = snapshot("/dev/null", (640, 480), frames=3)
        check("a picture that was fine is not taken again", asked, [3])
        check("...and its controls are left alone", restored, [])
        check("...and it is what comes back", got[0][0], room)

        # Aimed into the sun, or a camera that really cannot cope: the frames still
        # come back, because somebody looking should see what there is, but the
        # complaint says so rather than letting a blank frame pass for a room.
        answers[:], asked[:], restored[:] = [white], [], []
        got, why = snapshot("/dev/null", (640, 480), frames=3)
        check("a picture still blank after the second look is still returned",
              got[0][0], white)
        check("...but is complained about", "could not expose" in why, True)

        # A camera somebody has pinned to a fixed exposure. The picture it returns
        # is a perfectly ordinary one -- pinned *shut* looks exactly like a dark
        # room -- so nothing in the frame can raise this, and the controls have to
        # be read instead. It costs no second capture: the exposure is handed back
        # before the picture is taken rather than after it is found wanting.
        answers[:], asked[:], restored[:] = [room], [], []
        pinned.append(True)
        got, why = snapshot("/dev/null", (640, 480), frames=3)
        check("a camera left in manual has its exposure handed back",
              restored, ["/dev/null"])
        check("...before the picture is taken, not after", asked, [3])
        check("...and the picture is what comes back", got[0][0], room)
        pinned[:] = []

        # The `recover=False` door, for a caller that wants the raw capture: the
        # self-test's own timing runs, and anything measuring the camera itself.
        answers[:], asked[:], restored[:] = [white], [], []
        pinned.append(True)
        got, why = snapshot("/dev/null", (640, 480), frames=3, recover=False)
        check("recovery can be turned off", asked, [3])
        check("...and then nothing is touched", restored, [])
        check("...and the blank frame comes back unremarked", why, "")
        pinned[:] = []
    finally:
        uvc_camera._capture = was_capture
        uvc_camera.restore_automatic = was_restore
        uvc_camera.under_manual_control = was_manual


def test_the_exposure_is_handed_back_through_manual():
    """Setting a control the value it already holds resets nothing.

    Measured on the rover: `auto_exposure=3` on a camera already at 3 takes 7 ms and
    changes the picture by nothing at all, three times running, which is why the
    version of this that wrote it was never rescuing anything. The round trip
    through manual is what clears a camera that was pinned, so the order of these
    writes is the whole point of the function and worth pinning down.
    """
    try:
        import uvc_camera
    except ImportError as error:
        SKIP.append(f"handing the exposure back ({error})")
        return

    # The module's own reference to subprocess is swapped, rather than `run` inside
    # the real subprocess module: this check must not leave a stub where the rest of
    # the daemon's checks reach for a process.
    class Fake:
        def __init__(self):
            self.ran = []

        def run(self, argv, **kw):
            self.ran.append(argv)

    fake = Fake()
    was, uvc_camera.subprocess = uvc_camera.subprocess, fake
    try:
        uvc_camera.restore_automatic("/dev/video9")
    finally:
        uvc_camera.subprocess = was
    ran = fake.ran

    wrote = [argv[argv.index("-c") + 1] for argv in ran]
    check("the exposure goes to manual first", wrote[0], "auto_exposure=1")
    check("...then to the camera's own default time", wrote[1],
          "exposure_time_absolute=%d" % uvc_camera.EXPOSURE_DEFAULT)
    check("...and only then back to automatic", wrote[2], "auto_exposure=3")
    check("...with white balance automatic too", wrote[3], "white_balance_automatic=1")
    check("...all of it on the device it was given",
          all(argv[argv.index("-d") + 1] == "/dev/video9" for argv in ran), True)


def test_reading_whether_the_camera_is_on_automatic():
    """What v4l2-ctl says back, and what each answer has to mean.

    The consequence of getting this wrong is not a wrong picture but a needless
    one: read "manual" off a camera that is fine and every capture pays for the
    round trip and restarts an exposure that was already right.
    """
    try:
        import uvc_camera
    except ImportError as error:
        SKIP.append(f"reading the exposure mode ({error})")
        return

    class Fake:
        def __init__(self, stdout):
            self.stdout = stdout

        def run(self, argv, **kw):
            return self

    def says(stdout):
        was, uvc_camera.subprocess = uvc_camera.subprocess, Fake(stdout)
        try:
            return uvc_camera.under_manual_control("/dev/video9")
        finally:
            uvc_camera.subprocess = was

    check("a camera looking after itself is left alone",
          says("white_balance_automatic: 1\nauto_exposure: 3 (Aperture Priority Mode)"),
          False)
    check("one pinned to a fixed exposure is not",
          says("white_balance_automatic: 1\nauto_exposure: 1 (Manual Mode)"), True)
    check("nor is one with its white balance held",
          says("white_balance_automatic: 0\nauto_exposure: 3 (Aperture Priority Mode)"),
          True)
    # A camera that does not offer these controls, or a v4l2-ctl that failed: there
    # is nothing to put right and nothing that should be written to it.
    check("a camera that says nothing is left alone", says(""), False)
    check("...and so is one that says something else entirely",
          says("VIDIOC_G_EXT_CTRLS: failed: Inappropriate ioctl for device"), False)


TESTS = (
    test_what_counts_as_a_blank_frame,
    test_a_blank_picture_is_taken_again,
    test_the_exposure_is_handed_back_through_manual,
    test_reading_whether_the_camera_is_on_automatic,
)
