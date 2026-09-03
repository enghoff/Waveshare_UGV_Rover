"""Gimbal camera checks: opening it, looking, and counting faces.

No camera and no detector are present, so what is covered is the arithmetic and
the plumbing around them -- which device is chosen, what a frame is split into,
whether a detector's boxes come back in the frame's own pixels, and what the
camera does when there is nobody in view.
"""
from __future__ import annotations

import json

from test_fakes import FakeLink
from test_harness import SKIP, check

def test_no_camera():
    import rover_daemon

    rover = rover_daemon.Rover(FakeLink(), "unused", device=None)
    # A rover with no camera must say so rather than hanging or claiming to look.
    for tool in ("count_faces", "start_tracking", "camera_jpeg"):
        got = rover.call(tool, {})
        check(f"{tool} without a camera is refused", got["ok"], False)
        check(f"...and says why", "camera" in got["error"], True)
    check("tracking_status is answerable regardless",
          rover.call("tracking_status", {})["tracking"], False)
    check("stop_tracking when not tracking is not an error",
          rover.call("stop_tracking", {}), {"ok": True, "tracking": False, "was_tracking": False})


def test_default_camera():
    import rover_camera

    real_glob = rover_camera.glob.glob
    try:
        rover_camera.glob.glob = lambda pat: []
        check("no camera name falls back to video0",
              rover_camera.default_camera(), "/dev/video0")
        rover_camera.glob.glob = lambda pat: [
            "/dev/v4l/by-id/usb-Xitech_USB_Camera-video-index0"]
        check("a USB by-id name is preferred",
              rover_camera.default_camera(),
              "/dev/v4l/by-id/usb-Xitech_USB_Camera-video-index0")
        rover_camera.glob.glob = lambda pat: [
            "/dev/v4l/by-id/usb-Other-video-index0",
            "/dev/v4l/by-id/usb-Xitech_USB_Camera-video-index0"]
        check("the first by-id name is stable",
              rover_camera.default_camera(),
              "/dev/v4l/by-id/usb-Other-video-index0")
    finally:
        rover_camera.glob.glob = real_glob


def test_look():
    """The picture path: what is offered, what is sent, and what is refused.

    None of it decodes an image -- that is the design, not an omission -- so all
    of it is checkable here against a stub standing in for the model's host.
    """
    import http.server
    import threading

    import rover_daemon

    posted = []

    class Vision(http.server.BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_POST(self):
            body = self.rfile.read(int(self.headers["Content-Length"]))
            posted.append(body)
            reply = json.dumps({"ok": True, "image": f"frame-{len(posted)}",
                                "w": 640, "h": 480}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(reply)))
            self.end_headers()
            self.wfile.write(reply)

        def log_message(self, *args):
            pass

    def captures(*frames):
        """Stand in for a one-shot capture: frames oldest first, as v4l2-ctl gives
        them, plus the complaint that explains an empty list."""
        stamped = [(frame, 1.0) for frame in frames]
        why = "" if stamped else "no frame arrived"
        return lambda frames=None: (stamped, why)

    whole = b"\xff\xd8" + b"jpeg bytes" + b"\xff\xd9"
    fragment = b"middle of a picture" + b"\xff\xd9"

    # Offered only when there is somewhere to send a picture. This is the
    # rollback lever: no --vision, no tool, and no client has to be redeployed.
    plain = rover_daemon.Rover(FakeLink(), "unused", device="/dev/video0")
    check("look is not offered without --vision",
          [t["function"]["name"] for t in plain.tools()].count("look"), 0)
    check("...and calling it anyway is refused", plain.call("look", {})["ok"], False)
    blind = rover_daemon.Rover(FakeLink(), "unused", device=None, vision="127.0.0.1:1")
    check("a rover with no camera offers no look either",
          [t["function"]["name"] for t in blind.tools()].count("look"), 0)

    # Threading, and daemon threads: the client keeps its connection open on
    # purpose -- that is what makes a call cost one round trip and not two -- so
    # a single-threaded stub sits in that connection and never reaches shutdown.
    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Vision)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    address = f"127.0.0.1:{server.server_address[1]}"
    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/video0", vision=address)
    try:
        check("look is offered with --vision",
              [t["function"]["name"] for t in rover.tools()].count("look"), 1)

        # The half-frame is the one that matters, and it is put *last* on purpose:
        # frames come back oldest first and the newest is the one worth sending, so
        # a fragment in that position is the one a naive "take the last" would
        # send. Caught by the two bytes at the front rather than by decoding
        # anything.
        rover._snapshot = captures(whole, fragment)
        got = rover.call("look", {})
        check("a picture is sent and named", got.get("image"), "frame-1")
        check("...and it is the whole frame, not the fragment", posted[-1], whole)
        # The name and nothing else. A sentence in here is not a comment, it is
        # context handed to a model immediately before a picture, and the one
        # that used to sit in this result ("describe what is actually in it")
        # made every follow-up take a fresh photograph and describe the lot.
        check("...and the result is the name and nothing else", sorted(got), ["image", "ok"])

        # Three fragments running is a camera that is not producing pictures.
        rover._snapshot = captures(fragment, fragment, fragment)
        got = rover.call("look", {})
        check("nothing but fragments is a failure", got["ok"], False)
        check("...that says what happened", "whole pictures" in got["error"], True)

        # A camera that gives nothing at all, which is what an unplugged one does,
        # and what a camera another process is holding open does too. Either way
        # the capture's own complaint is what reaches the model.
        rover._snapshot = captures()
        got = rover.call("look", {})
        check("no frame at all is a failure", got["ok"], False)
        check("...carrying the capture's complaint", "no frame arrived" in got["error"], True)

        # While tracking owns the camera, the loop's newest frame is what there
        # is -- and a stale one is refused rather than passed off as now.
        rover._tracking.set()
        rover._frame = (whole, 1.0)  # monotonic 1.0 is long ago
        got = rover.call("look", {})
        check("a stale tracking frame is refused", got["ok"], False)
        import time as _time
        rover._frame = (whole, _time.monotonic())
        check("a fresh tracking frame is used", rover.call("look", {}).get("image"), "frame-2")
        rover._tracking.clear()
    finally:
        rover.vision.close()  # let go of the kept-open connection first
        server.shutdown()
        server.server_close()

    # And the service being away: a failure the model can say out loud, not an
    # exception in the middle of a turn.
    rover._snapshot = captures(whole)
    got = rover.call("look", {})
    check("a vision service that has gone is reported", got["ok"], False)
    check("...naming where it tried", "/frame" in got["error"], True)
    rover.close()


def test_snapshot_splitting():
    """Cutting a run of concatenated JPEGs back into pictures.

    This is what makes a one-shot capture possible, and a bug in it is silent in
    the worst way: a frame with someone else's bytes on the end still decodes, so
    a model would be shown a picture and describe it without anything looking
    wrong. `--stream-count=3` genuinely returns three pictures back to back with
    nothing between them, so the markers are the only boundary there is.
    """
    try:
        from track_face_pi import split_jpegs
    except ImportError as error:            # no aiming.py beside it, e.g. a bare copy
        SKIP.append(f"snapshot splitting ({error})")
        return

    a = b"\xff\xd8" + b"first" + b"\xff\xd9"
    b = b"\xff\xd8" + b"second" + b"\xff\xd9"
    c = b"\xff\xd8" + b"third" + b"\xff\xd9"
    check("three frames come back as three", split_jpegs(a + b + c), [a, b, c])
    check("one frame comes back as one", split_jpegs(a), [a])
    check("nothing at all is no frames", split_jpegs(b""), [])
    # The case the streaming reader could not tell apart, and the reason a start
    # marker is required as well as an end one: a capture joined part-way through
    # has an end-of-image with no start before it, and that end must not be taken
    # as the end of a picture that was never whole.
    check("a leading part-frame is dropped, not returned",
          split_jpegs(b"tail of a picture\xff\xd9" + a), [a])
    # A capture cut short by the timeout: what did arrive whole is still worth
    # having, and the unterminated remainder is not returned as if it were.
    check("an unfinished last frame is left out",
          split_jpegs(a + b"\xff\xd8" + b"cut off here"), [a])
    check("...and its whole predecessors are kept", split_jpegs(a + b + b"\xff\xd8no end"),
          [a, b])


def test_two_pictures_at_once_do_not_share_the_camera():
    """Only one thing may have this camera, because the loser gets nothing.

    Measured on the rover on 2026-09-03, over 60 grabs: 46 that had the camera to
    themselves all came back with three frames, and 12 of the 14 that overlapped
    another grab came back empty -- in about 30 ms, with nothing written to stderr
    to say why. The gap between grabs made no difference at all, so it is the
    overlap and nothing else. It became a daily fault when the world state started
    looking once a second, because the console asks for a frame every two seconds
    through the same path, and better than half of the console's pictures were
    lost to the collision.

    What is checked is that the second caller waits rather than being handed the
    empty result the camera would really give it.
    """
    import rover_daemon
    import sys
    import threading
    import time

    try:
        import track_face_pi
    except ImportError as error:            # no aiming.py beside it, e.g. a bare copy
        SKIP.append(f"one grab at a time ({error})")
        return

    whole = b"\xff\xd8" + b"jpeg" + b"\xff\xd9"
    inside = threading.Semaphore(0)
    go = threading.Event()
    overlapped, holding = [], []
    count = threading.Lock()

    def grabber(device, size, frames=3):
        with count:
            holding.append(1)
            overlapped.append(len(holding))
        inside.release()
        go.wait(5.0)
        with count:
            holding.pop()
        return [(whole, time.monotonic())], ""

    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/video0")
    was, track_face_pi.snapshot = track_face_pi.snapshot, grabber
    try:
        got = []
        for _ in range(2):
            threading.Thread(target=lambda: got.append(rover._snapshot()),
                             daemon=True).start()
        check("one grab reaches the camera", inside.acquire(timeout=5.0), True)
        # The second one must still be waiting: if it were inside the capture
        # alongside the first, this would come back at once.
        check("...and the other is made to wait for it",
              inside.acquire(timeout=0.5), False)
        go.set()
        check("...and then gets in", inside.acquire(timeout=5.0), True)
        deadline = time.monotonic() + 5.0
        while len(got) < 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        check("both callers get their picture", len(got), 2)
        check("...and never two on the camera at once", max(overlapped), 1)
    finally:
        track_face_pi.snapshot = was
        rover.close()


def test_counting_faces_does_not_hold_the_board():
    """A slow detector on another host must not lock the rover up.

    `count_faces` waits on MEDIA, and MEDIA being slow or away is an expected state
    here rather than an exceptional one. It used to do that waiting while holding
    the lock that also serialises the driver board, so a detector taking its time
    was a rover that could not switch a light or be told to stop. The wait now has
    its own lock: still strictly one request at a time, because the detector is one
    kept-open connection, but not at the board's expense.
    """
    import rover_daemon
    import threading
    import time

    in_detect, release = threading.Event(), threading.Event()

    class SlowDetector:
        def detect(self, jpeg, at):
            in_detect.set()
            release.wait(5.0)
            return []

    whole = b"\xff\xd8" + b"jpeg" + b"\xff\xd9"
    board = FakeLink()
    rover = rover_daemon.Rover(board, "unused", device="/dev/video0")
    rover._snapshot = lambda frames=None: ([(whole, 1.0)], "")
    rover._open_detector = lambda: SlowDetector()

    counted = []
    threading.Thread(target=lambda: counted.append(rover.call("count_faces", {})),
                     daemon=True).start()
    check("the detector is reached", in_detect.wait(5.0), True)
    # The board, while that call is still inside the detector -- and *timed*, because
    # the old behaviour did not refuse this, it simply made it wait for the detector.
    # Passing slowly is the exact failure being pinned, so the clock is the check.
    started = time.monotonic()
    got = rover.call("set_lights", {"level": 7})
    waited = time.monotonic() - started
    check("the lights still answer while a count is waiting", got["ok"], True)
    check("...without waiting for the detector", waited < 1.0, True)
    check("...and the board really was told", board.sent[-1],
          {"T": 132, "IO4": 7, "IO5": 7})
    release.set()
    for _ in range(50):
        if counted:
            break
        time.sleep(0.1)
    check("the count finishes once the detector answers",
          counted and counted[0]["ok"], True)
    rover.close()


def test_the_local_detector_scales_its_boxes_back_up():
    """A box is measured in the copy the network saw and used in the frame's pixels.

    The one piece of arithmetic in `yunet.py` that no amount of watching the rover
    would show as wrong. Detecting on a reduced copy and forgetting to scale the
    box back up aims the camera at a point a fixed fraction of the way towards the
    face, on both axes, which looks exactly like a gain that is too low -- and this
    repository has spent weeks on aiming faults that presented that way. On the
    rover as it stands the scale is 1.0 and the bug would be invisible; it appears
    the moment somebody captures at 720p, which is a supported mode.

    Skipped where OpenCV is not installed, which is every host but the rover and
    the desk.
    """
    try:
        from yunet import LocalDetector, YuNetError
    except ImportError as error:
        SKIP.append(f"the local detector's boxes ({error})")
        return
    try:
        detector = LocalDetector(size=(1280, 720), width=640)
    except YuNetError as error:
        SKIP.append(f"the local detector's boxes ({error})")
        return

    # One synthetic YuNet row: a 30x40 box at (10, 20), the five landmarks it also
    # returns, and the score. Real detections are unavailable without a real face,
    # and this is not about detection -- it is about what happens to a box after.
    row = [10.0, 20.0, 30.0, 40.0] + [0.0] * 10 + [0.9]
    faces = detector._boxes([row], 640)
    check("a box detected on a half-scale copy is doubled",
          faces, [[20.0, 40.0, 60.0, 80.0, 0.9]])
    check("a box detected at the frame's own width is left alone",
          LocalDetector(size=(640, 480))._boxes([row], 640),
          [[10.0, 20.0, 30.0, 40.0, 0.9]])
    check("the frame's width decides the decode ratio",
          detector._decode(b"not a jpeg at all"), None)

    # A frame that will not decode is "no faces", not "no detector". The two are
    # different in the loop: one aims, the other holds still.
    got = detector.detect(bytes([0xFF, 0xD8]) + b" not really a jpeg "
                          + bytes([0xFF, 0xD9]))
    check("an undecodable frame is no faces rather than no answer", got, [])
    check("...and is counted as an error", detector.errors, 1)
    check("the description says what is running and how wide",
          detector.describe().startswith("YuNet in this process, 640px wide"), True)
    detector.close()


def test_camera_cone():
    """The one conversion between the gimbal's angles and the map's.

    The gimbal counts pan positive to the right and everything in lidar_slam counts
    bearings positive to the left, so the map is handed minus the pan. A sign error
    there draws a perfectly ordinary cone over the wrong half of the room, and
    nothing about the resulting picture looks wrong -- which is exactly why it is
    worth a check rather than a comment.
    """
    import rover_daemon

    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/video0")
    # Against the constant rather than against a number written out here, so that
    # re-measuring the lens does not leave a stale figure in a passing test.
    check("centred, the cone points straight ahead", rover._camera_cone(),
          (0.0, rover_daemon.CAMERA_FOV_DEG))

    # Panning right must give a negative bearing, because the map's positive is left.
    rover.call("look_at", {"pan": 40})
    bearing, fov = rover._camera_cone()
    check("panning right aims the cone to the map's right", bearing < 0, True)
    check("...by the same amount", abs(bearing), 40.0)
    rover.call("look_at", {"pan": -40})
    check("panning left aims the cone to the map's left",
          rover._camera_cone()[0], 40.0)

    # The width is a setting, not a constant, because nobody has measured the lens.
    wide = rover_daemon.Rover(FakeLink(), "unused", device="/dev/video0",
                              camera_fov_deg=100.0)
    check("the field of view is what it was told", wide._camera_cone()[1], 100.0)

    # No camera, no cone: a wedge drawn for a lens that is not fitted is the map
    # making a claim the hardware cannot keep.
    blind = rover_daemon.Rover(FakeLink(), "unused", device=None)
    check("a rover with no camera draws no cone", blind._camera_cone(), None)


def test_what_the_camera_does_with_nobody_in_view():
    """Sweep when parked, watch the road when driving, and change over mid-run.

    The tracking loop asks this once a frame instead of being told by the
    navigator when a move starts and stops, so this is the whole of the decision
    and worth having offline: the loop itself needs a camera, a detector and a
    person to stand in front of it.
    """
    try:
        from aiming import Ahead, Gimbal, Scan
    except ImportError as exc:
        SKIP.append(f"searching behaviour ({type(exc).__name__}: needs aiming.py)")
        return
    import rover_daemon

    class FakeNav:
        """Only the one thing the camera asks a navigator."""

        def __init__(self):
            self.driving = False

    rover = rover_daemon.Rover(FakeLink(), "unused", device=None)
    gimbal = Gimbal(0.5, (640, 480))

    check("a rover with no navigator is not driving", rover.driving, False)
    parked = rover._searching(None, gimbal)
    check("...so an idle camera sweeps", isinstance(parked, Scan), True)

    rover.nav = FakeNav()
    check("a navigator with the wheels free is not driving either", rover.driving, False)
    check("and the sweep already running is kept, not restarted",
          rover._searching(parked, gimbal), parked)

    rover.nav.driving = True
    check("the wheels turning is driving", rover.driving, True)
    moving = rover._searching(parked, gimbal)
    check("...and the sweep gives way to watching ahead",
          isinstance(moving, Ahead), True)
    check("which is then kept for as long as the move lasts",
          rover._searching(moving, gimbal), moving)

    rover.nav.driving = False
    check("the move ending puts the sweep back",
          isinstance(rover._searching(moving, gimbal), Scan), True)


TESTS = (
    test_no_camera,
    test_default_camera,
    test_look,
    test_snapshot_splitting,
    test_two_pictures_at_once_do_not_share_the_camera,
    test_counting_faces_does_not_hold_the_board,
    test_the_local_detector_scales_its_boxes_back_up,
    test_camera_cone,
    test_what_the_camera_does_with_nobody_in_view,
)
