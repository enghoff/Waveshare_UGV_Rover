"""Offline checks for the rover daemon: no board, no camera, no detector.

What is covered is the part where a bug is silent rather than loud -- argument
coercion, the limits the gimbal is held to, and the fact that every schema the
model is shown corresponds to something that will actually run. A tool whose
name does not match its handler fails as "no such tool" out loud, in the middle
of a conversation, which is a poor place to find out.

    python3 selftest.py                  # on the rover, where everything is flat
    python rover_daemon/selftest.py      # in the repo

The hardware paths are not covered and cannot be: they need the rover.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
# In the repo, aiming.py lives one directory over; on the rover everything is
# deployed flat into ~/ugv and this does nothing.
SIBLING = os.path.join(os.path.dirname(HERE), "face_tracking")
if os.path.isdir(SIBLING):
    sys.path.insert(0, SIBLING)
# The same for the map renderer, and it needs both layouts rather than one:
# `lidar_slam` is a sibling of this file in the repository and a *subdirectory*
# of the rover's ~/ugv, which is the two-candidate dance ros_navigator.py does
# for the same import. One check draws a real map and reads the picture back,
# and with only the repository layout it passes here and fails on the rover.
for RENDERER in (os.path.join(os.path.dirname(HERE), "lidar_slam"),
                 os.path.join(HERE, "lidar_slam")):
    if os.path.isdir(RENDERER):
        sys.path.insert(0, RENDERER)
        break

from test_aiming import (
    test_aiming_through_a_missed_frame, test_one_move_puts_a_face_in_the_middle,
    test_the_approach_to_a_face_never_turns_back,
)
from test_harness import FAIL, PASS, SKIP, check
from test_ros_nav import TESTS as ROS_NAV_TESTS


class FakeLink:
    """A driver board that answers, or does not, and remembers what it was told."""

    def __init__(self, works=True, volts=1153):
        self.sent = []
        self.works = works
        # What its telemetry says the pack is at, in hundredths of a volt, and how
        # many times that has been asked for. None is a board that says nothing
        # back, which is what an unpowered one and the wrong serial port both look
        # like from here.
        self.volts = volts
        self.reads = 0

    def send(self, command):
        self.sent.append(command)
        return self.works

    def telemetry(self):
        self.reads += 1
        return None if self.volts is None else {"T": 1001, "v": self.volts}

    def describe(self):
        return "fake"

    def close(self):
        pass


def test_levels():
    import rover_daemon

    # A 4B model at int4 writes arguments loosely, and the tool caring about the
    # difference costs the user a whole turn to hear "I could not do that".
    for given, want in (("255", 255), (255.0, 255), (True, 255), (False, 0),
                        ("on", 255), ("off", 0), ("50%", 128), (999, 255), (-5, 0)):
        check(f"level {given!r} means {want}", rover_daemon._level(given), want)
    for bad in ("bright", None, [1]):
        try:
            rover_daemon._level(bad)
            FAIL.append(f"level {bad!r} should have been refused")
        except (TypeError, ValueError):
            PASS.append(f"level {bad!r} is refused")


def test_battery():
    """The pack voltage, out of the one line the board sends without being asked.

    Four things worth holding onto here: that a whole line is picked out of a
    stream which starts and ends mid-message, that the percentage comes off the
    discharge curve rather than a straight line between full and empty, that a
    board running from USB with no pack fitted is its own answer rather than 0%,
    and that a console polling every few seconds does not read the UART every few
    seconds.
    """
    import rover_daemon

    stream = (b'01,"v":1152}\n'                    # the tail of an earlier line
              b'{"T":1001,"ax":148,"v":1153}\n'
              b'{"T":1001,"ax":150,"v":1149}\n'
              b'{"T":1001,"ax":1')                  # and the start of the next
    check("the newest whole line is the one read",
          rover_daemon._newest_telemetry(stream)["v"], 1149)
    check("half a line is not a reading",
          rover_daemon._newest_telemetry(b'{"T":1001,"v":11'), None)
    check("a line that is not telemetry is passed over",
          rover_daemon._newest_telemetry(b'{"T":1051,"v":1153}\n'), None)

    # 11.53 V is 3.84 V/cell, in the flat middle of the discharge where lithium-ion
    # spends most of its life. A straight line from 12.6 V to 9.9 V calls that 60%;
    # the curve calls it 55%, and that gap is the whole reason there is a table.
    check("the flat middle is read off the table",
          rover_daemon._battery_percent(11.53), 55)
    check("a pack off the charger is 100%", rover_daemon._battery_percent(12.6), 100)
    check("nothing reads below zero", rover_daemon._battery_percent(6.0), 0)
    for volts, want in ((12.5, "full"), (11.5, "ok"), (11.0, "low"),
                        (10.5, "critical"), (0.3, "absent")):
        check(f"{volts} V is {want}", rover_daemon._battery_state(volts), want)

    link = FakeLink()
    rover = rover_daemon.Rover(link, "unused", device=None)
    reading = rover.call("battery", {})
    check("the board is read", reading["ok"], True)
    check("...in volts", reading["volts"], 11.53)
    check("...as a percentage", reading["percent"], 55)
    check("...and as a sentence something can say out loud",
          "55%" in reading["summary"], True)
    # The console polls this, and two clients may poll at once. Every poll being a
    # read of the UART the wheels are steered down is what the cache exists to
    # prevent.
    for _ in range(5):
        rover.call("battery", {})
    check("polling does not read the board every time", link.reads, 1)

    # No pack fitted: the ESP32 runs from USB alone and reports a few tenths of a
    # volt. No percentage comes back, because there is nothing for it to be a
    # percentage of.
    empty = rover_daemon.Rover(FakeLink(volts=31), "unused",
                               device=None).call("battery", {})
    check("a board with no pack says so", empty["state"], "absent")
    check("...and offers no percentage", "percent" in empty, False)
    check("...in words that do not sound like a flat battery",
          "no battery pack" in empty["summary"], True)

    # A board that says nothing has to come back as a sentence rather than raise:
    # this is reached from a window with a live panel on it as well as from a model.
    silent = rover_daemon.Rover(FakeLink(volts=None), "unused",
                                device=None).call("battery", {})
    check("a silent board is refused rather than raising", silent["ok"], False)
    check("...and says what it could not do", "voltage" in silent["error"], True)


def test_schemas():
    import rover_daemon

    # Every schema this rover could ever offer, whatever it is configured with.
    # `look` is conditional -- see test_look -- and so are the driving tools and the
    # map, which need a lidar; but a conditional schema still has to have a handler,
    # and the point of this test is that none of them lie.
    every = (rover_daemon.TOOLS + [rover_daemon.LOOK_TOOL]
             + rover_daemon.NAV_TOOLS
             + [rover_daemon.MAP_TOOL, rover_daemon.MAP_POINT_TOOL]
             + [rover_daemon.SCRIPT_TOOL, rover_daemon.START_SCRIPT_TOOL,
                rover_daemon.STOP_SCRIPT_TOOL])
    # The schemas cross a network and go into a prompt, so they have to be JSON.
    json.dumps(every)
    names = [t["function"]["name"] for t in every]
    check("every schema is a function", {t["type"] for t in every}, {"function"})
    check("no tool is listed twice", len(set(names)), len(names))
    # The check this file exists for: a schema whose handler is missing fails as
    # "no such tool" mid-conversation rather than here.
    missing = [n for n in names if not hasattr(rover_daemon.Rover, f"_tool_{n}")]
    check("every schema has a handler", missing, [])
    # ...and the reverse, so a tool that works but is never offered is noticed.
    #
    # Minus the control calls, which are dispatched like tools because that is
    # the only protocol this daemon speaks, and are deliberately kept out of
    # `list_tools` so that no model is ever shown one. They are named here rather
    # than detected, so that adding a handler still has to be a deliberate
    # decision about which of the two kinds it is.
    handlers = sorted(m[len("_tool_"):] for m in dir(rover_daemon.Rover)
                      if m.startswith("_tool_"))
    control = ["set_vision", "nav_status", "map_png", "camera_jpeg", "clear_map",
               "detect_in",
               # Replugging the lidar in software, which is a control call for the
               # same reason wifi_join is: it is the right thing for a person
               # watching a stale map and the wrong thing for a model, since it
               # takes the camera down with it for a few seconds.
               "reset_lidar",
               # Two of the five scripting calls. The other three are model
               # tools now, offered to a client on loopback -- which since the
               # rover started holding its own conversation includes the model.
               # `start_script` and `script_stop` joined `run_script` there when
               # a behaviour stopped having a time limit: what ends one is being
               # stopped, so a model able to start one has to be able to stop
               # it. See LOCAL_ONLY in rover_daemon.py, and the script tool test
               # below for the gate.
               "script_status", "list_api",
               # And the network, which is a control call for a reason of its own:
               # a model that moved the rover onto another access point would be
               # cutting the wire its own conversation arrives on, and no wording
               # of a description makes that a good idea.
               "wifi_status", "wifi_join"]
    for name in control:
        check(f"{name} is a control call, not a tool", name in handlers, True)
        check(f"...and is not offered to any model", name in names, False)
    check("every other handler is offered",
          sorted(names), [h for h in handlers if h not in control])
    for tool in every:
        function = tool["function"]
        check(f"{function['name']} describes itself", bool(function.get("description")), True)
        check(f"{function['name']} has a parameter object",
              function["parameters"]["type"], "object")

    # The model's map knobs have to match what the renderer will actually draw.
    # A schema that offers 50 m across is a tool that then silently clamps to 24,
    # and a 4B model will not notice.
    props = rover_daemon.MAP_TOOL["function"]["parameters"]["properties"]
    check("the model can ask how many metres across", "across_m" in props, True)
    check("...and how big a picture", "pixels" in props, True)
    check("...but is not shown the half-extent map_png takes",
          "half_extent_m" in props, False)
    check("...nor pixels-per-cell", "scale" in props, False)
    check("neither map knob is required, so 'show me the map' still works",
          rover_daemon.MAP_TOOL["function"]["parameters"].get("required"), [])
    check("the across ceiling is twice the drawing half-extent",
          props["across_m"]["maximum"], 2 * rover_daemon.MAP_MAX_HALF_EXTENT_M)
    check("the across floor is twice the drawing floor",
          props["across_m"]["minimum"], 1.0)
    check("the picture floor is the drawing floor",
          props["pixels"]["minimum"], rover_daemon.MAP_MIN_PIXELS)
    check("the picture ceiling is the drawing ceiling",
          props["pixels"]["maximum"], rover_daemon.MAP_MAX_PIXELS)


def test_lights():
    import rover_daemon

    link = FakeLink()
    rover = rover_daemon.Rover(link, "unused", device=None)

    check("starts dark", rover.call("get_lights", {}),
          {"ok": True, "level": 0, "on": False})
    check("set_lights answers with the level", rover.call("set_lights", {"level": 128}),
          {"ok": True, "level": 128, "on": True})
    check("both channels are driven together", link.sent[-1],
          {"T": 132, "IO4": 128, "IO5": 128})
    check("the level is remembered", rover.call("get_lights", {}),
          {"ok": True, "level": 128, "on": True})
    check("an unknown tool is refused", rover.call("nope", {})["ok"], False)

    dead = rover_daemon.Rover(FakeLink(works=False), "unused", device=None)
    check("a dead board reports failure", dead.call("set_lights", {"level": 255})["ok"], False)
    check("...and does not move its own idea of the level", dead.level, 0)


def test_gimbal():
    try:
        from aiming import PAN_LIMIT, TILT_LIMITS
    except ImportError as exc:
        SKIP.append(f"gimbal limits ({type(exc).__name__}: needs aiming.py)")
        return
    import rover_daemon

    link = FakeLink()
    rover = rover_daemon.Rover(link, "unused", device=None)

    check("look_at aims where it is told", rover.call("look_at", {"pan": -40, "tilt": 10}),
          {"ok": True, "pan": -40, "tilt": 10, "stopped_tracking": False})
    check("...and commands the gimbal, not the wheels", link.sent[-1]["T"], 133)
    # Clamped rather than refused: "look all the way round" has an obvious
    # intention, and the servos have limits whatever the model asks for.
    check("pan is clamped to the servo's range",
          rover.call("look_at", {"pan": 999})["pan"], PAN_LIMIT)
    check("tilt is clamped downwards",
          rover.call("look_at", {"tilt": -999})["tilt"], TILT_LIMITS[0])
    check("tilt is clamped upwards",
          rover.call("look_at", {"tilt": 999})["tilt"], TILT_LIMITS[1])
    # One axis at a time: asking to look left must not also level the camera.
    rover.call("look_at", {"pan": 0, "tilt": 30})
    check("an omitted axis is left alone", rover.call("look_at", {"pan": 20}),
          {"ok": True, "pan": 20, "tilt": 30, "stopped_tracking": False})
    check("a non-numeric angle is refused", rover.call("look_at", {"pan": "left"})["ok"], False)
    check("center_camera returns to zero", rover.call("center_camera", {}),
          {"ok": True, "pan": 0, "tilt": 0, "stopped_tracking": False})


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


def test_driving_takes_the_core():
    """Driving releases the camera, not just the loop that was reading it.

    The navigator calls `park_tracking` the instant before the wheels move, and what
    it has to accomplish is that nothing else is competing for this one core -- the
    scan matcher is the rover's only odometer, so a camera left streaming is a drive
    measuring itself wrong. Tracking closes its own camera on the way out, so the
    case worth pinning is the other one: a feed open with no loop reading it, which
    is what a crash between the two leaves behind, and what a photograph used to
    leave behind for twenty seconds.
    """
    import rover_daemon

    closed = []

    class FakeFeed:
        def close(self):
            closed.append(True)

    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/video0")
    # No tracking loop running, but a camera open: park_tracking has to notice.
    rover._camera = FakeFeed()
    rover.park_tracking()
    check("driving closes a camera nobody was reading", closed, [True])
    check("...and leaves nothing behind to reopen", rover._camera, None)
    # And it must not then hand tracking back, because tracking was never taken.
    started = []
    rover._tool_start_tracking = lambda _a: started.append(True) or {"ok": True}
    rover.unpark_tracking()
    check("a drive that never parked tracking does not start it", started, [])
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


def test_map_png_names_the_clock():
    """`map_png` times itself. A missing `import time` used to reach the page as
    `NameError: name 'time' is not defined` and leave the map blank."""
    import rover_daemon

    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/video0")

    class FakeNav:
        class slam:
            class config:
                resolution_m = 0.05
            pose = (0.0, 0.0, 0.0)

        def map_png(self, *args, **kwargs):
            # Signature + IHDR so the handler can read the width at bytes 16:20.
            png = (b"\x89PNG\r\n\x1a\n" + b"\x00\x00\x00\rIHDR"
                   + (640).to_bytes(4, "big") + (480).to_bytes(4, "big"))
            return png, "a fake room"

    rover.nav = FakeNav()
    got = rover.call("map_png", {"half_extent_m": 3, "scale": 3})
    check("map_png answers rather than raising", got.get("ok"), True)
    check("...and names the picture size it drew", got.get("pixels"), 640)
    check("...and times the draw", isinstance(got.get("render_s"), (int, float)), True)


def test_show_map_takes_across_and_size():
    """The model can pick how much room is in frame and how big a picture.

    `across_m` is metres of room, not the half-extent `map_png` takes: a model
    told "six metres across" and handed `half_extent_m` would pass 6 and get
    twelve. Leave both out and it is still a room at the console's default
    picture size -- pixels per cell derived, so widening the view does not
    resize the picture.
    """
    import http.server
    import threading

    import rover_daemon

    posted = []
    asked = []

    class Vision(http.server.BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_POST(self):
            body = self.rfile.read(int(self.headers["Content-Length"]))
            posted.append(body)
            reply = json.dumps({"ok": True, "image": f"frame-{len(posted)}"}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(reply)))
            self.end_headers()
            self.wfile.write(reply)

        def log_message(self, *args):
            pass

    class FakeNav:
        class slam:
            class config:
                resolution_m = 0.05
            pose = (0.0, 0.0, 0.0)

        def map_png(self, half, scale, rover_up=False, camera=None):
            asked.append((half, scale))
            png = (b"\x89PNG\r\n\x1a\n" + b"\x00\x00\x00\rIHDR"
                   + (640).to_bytes(4, "big") + (480).to_bytes(4, "big"))
            return png, f"a fake room of {2 * half:.0f} m"

        def describe(self):
            return {"clear_ahead_m": 2.0, "text": "a room"}

    res = FakeNav.slam.config.resolution_m
    room = rover_daemon._model_map_view({}, res)
    wide = rover_daemon._model_map_view({"across_m": 24}, res)
    close = rover_daemon._model_map_view({"across_m": 3}, res)
    small = rover_daemon._model_map_view({"pixels": 320}, res)
    large = rover_daemon._model_map_view({"pixels": 800}, res)
    check("nothing asked for is a room", room[0], rover_daemon.MAP_HALF_EXTENT_M)
    check("six metres across is that same room, not twelve",
          rover_daemon._model_map_view({"across_m": 6}, res)[0],
          rover_daemon.MAP_HALF_EXTENT_M)
    check("twenty-four metres across is the drawing ceiling",
          wide[0], rover_daemon.MAP_MAX_HALF_EXTENT_M)
    check("a close view is closer than a room", close[0] < room[0], True)
    check("a bigger picture is more pixels per cell, not more room",
          (large[0], large[1] > small[1]), (small[0], True))
    check("widening the view does not raise the magnification",
          wide[1] <= room[1], True)

    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Vision)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    address = f"127.0.0.1:{server.server_address[1]}"
    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/video0",
                               vision=address)
    rover.nav = FakeNav()
    try:
        check("show_map is offered once there is somewhere to send a picture",
              "show_map" in [t["function"]["name"] for t in rover.tools()], True)
        got = rover.call("show_map", {})
        check("nothing asked for still draws", got.get("ok"), True)
        check("...at the room-sized default", asked[-1], room)
        check("...and the picture was posted", len(posted), 1)
        check("a string six means six metres across",
              rover.call("show_map", {"across_m": "6"}).get("ok"), True)
        check("...and is a room, not a floor", asked[-1][0], room[0])
        rover.call("show_map", {"across_m": 24, "pixels": 320})
        check("a floor view at a small picture reaches the renderer as such",
              asked[-1], rover_daemon._model_map_view(
                  {"across_m": 24, "pixels": 320}, res))
        check("the caption still goes to the model, not the picture size",
              "caption" in got, True)
        check("...and the result does not name pixels-per-cell",
              "scale" in got, False)
    finally:
        server.shutdown()


def test_drive_to_takes_a_place_on_the_map():
    """`drive_to` will take a point on the map, and only a console is told so.

    A tap on the console's map has to keep its meaning while the rover is still
    driving: the click stops the move in flight, and the rover carries on until the
    stop lands, so an offset from "where the rover is" is measured from somewhere
    the cursor never was. A point in the map's own frame is not, which is why the
    console sends every tap that way.

    The second half of this is the more important one. The pair is deliberately
    absent from the schema a model is shown, because nothing a model can see says
    where the rover is in that frame -- the room comes back to it as bearings and
    the map as a picture centred on itself -- so a model offered map coordinates
    could only invent them, and an invented pair is a fifteen-metre drive to a
    place nobody chose.
    """
    import rover_daemon
    import tool_schemas

    asked = []

    class FakeNav:
        class Outcome:
            reason = "arrived"

            def asdict(self):
                return {"reason": "arrived", "travelled_m": 1.0}

        def drive_to(self, **kwargs):
            asked.append(kwargs)
            return self.Outcome()

        def describe(self):
            return {"clear_ahead_m": 2.0, "text": "a room"}

    rover = rover_daemon.Rover(FakeLink(), "unused", device=None)
    rover.nav = FakeNav()

    got = rover.call("drive_to", {"x_m": 3.0, "y_m": -1.25})
    check("a place on the map is accepted", got.get("ok"), True)
    check("...and reaches the navigator as a place",
          (asked[-1].get("x_m"), asked[-1].get("y_m")), (3.0, -1.25))
    check("...with no offset invented alongside it",
          "ahead_m" in asked[-1], False)

    rover.call("drive_to", {"ahead_m": 1.0, "left_m": -0.4, "speed_ms": 0.15})
    check("an offset still reaches it as an offset",
          (asked[-1].get("ahead_m"), asked[-1].get("left_m")), (1.0, -0.4))
    check("...and the speed goes with it", asked[-1].get("speed_ms"), 0.15)

    # Half a coordinate is not a place, and guessing the other half would drive
    # somewhere nobody named.
    half = rover.call("drive_to", {"x_m": 3.0})
    check("one coordinate on its own is refused", half.get("ok"), False)
    check("...and says what is missing", "y_m" in str(half.get("error")), True)

    schema = next(s for s in tool_schemas.NAV_TOOLS
                  if s["function"]["name"] == "drive_to")
    offered = set(schema["function"]["parameters"]["properties"])
    check("a model is not offered the map's coordinates",
          offered & {"x_m", "y_m"}, set())
    check("...only the offsets it can actually work out",
          offered, {"ahead_m", "left_m", "speed_ms"})


def test_a_point_on_the_map_picture_is_the_place_it_looks_like():
    """A fraction of the map picture means where it appears to on the picture.

    The one thing this tool has to get right, and the one thing nothing else
    would notice if it got wrong. A model is handed a top-down picture and says
    where on it to go; the daemon turns that into a point in the map's own frame
    using the pose the picture was drawn at. Get the axes, the flip or the pose
    wrong and every part still works -- a picture is drawn, a fraction is
    accepted, a route is planned, the rover drives -- to somewhere else in the
    room, and the only symptom is a rover that goes the wrong way.

    So this is read off the picture rather than out of the arithmetic that drew
    it. Three obstacles go onto a synthetic map at known places, the rover's own
    renderer draws them, and their blobs are found in the PNG by colour. Their
    pixel centroids are then handed to the tool as fractions, and what comes back
    has to be the obstacle that was pointed at. Nothing here works a pixel out
    from a world coordinate, which is what would reduce it to the renderer
    agreeing with itself.

    Deliberately at a pose that is neither the origin nor an axis-aligned
    heading, because both of those hide a swapped axis and a dropped rotation.
    """
    import threading

    import mapimg
    import numpy as np

    import rover_daemon

    resolution, cells = 0.05, 800
    pose = (0.6, -0.35, math.radians(40.0))
    obstacles = [(1.6, 0.9), (-0.4, 1.4), (2.2, -1.2)]

    class Synthetic:
        """The three things `mapimg.render` asks a map for."""

        class config:
            resolution_m = resolution
            grid_cells = cells
            occupied_at = 50

        def __init__(self):
            self.lock = threading.Lock()
            self.pose = pose
            self.trail = ()
            grid = np.zeros((cells, cells), dtype=np.int8)
            # Four metres of seen floor around the rover, so that anything left
            # grey in the picture is grey on purpose.
            span = int(4.0 / resolution)
            cx = int(pose[0] / resolution) + cells // 2
            cy = int(pose[1] / resolution) + cells // 2
            grid[cx - span:cx + span, cy - span:cy + span] = -100
            for wx, wy in obstacles:
                ix = int(round(wx / resolution)) + cells // 2
                iy = int(round(wy / resolution)) + cells // 2
                grid[ix - 2:ix + 3, iy - 2:iy + 3] = 100        # 25 cm of solid
            self._grid = grid

        def grid(self):
            return self._grid

    asked = []

    class FakeNav:
        def __init__(self):
            self.slam = Synthetic()

        def map_png(self, half, scale, rover_up=False, camera=None):
            return mapimg.render(self.slam, half, scale, self.slam.trail,
                                 rover_up=rover_up, camera=camera)

        def describe(self):
            return {"clear_ahead_m": 2.0, "text": "an invented room"}

        class Outcome:
            reason = "arrived"

            def asdict(self):
                return {"reason": "arrived", "travelled_m": 1.0, "turned_deg": 0.0}

        def drive_to(self, **kwargs):
            asked.append(kwargs)
            return self.Outcome()

    def blobs(mask):
        """Four-connected runs of at least four True cells, as (row, col) lists."""
        seen = np.zeros(mask.shape, dtype=bool)
        found = []
        for start in zip(*np.nonzero(mask)):
            if seen[start]:
                continue
            stack, blob = [start], []
            seen[start] = True
            while stack:
                row, col = stack.pop()
                blob.append((row, col))
                for drow, dcol in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nrow, ncol = row + drow, col + dcol
                    if (0 <= nrow < mask.shape[0] and 0 <= ncol < mask.shape[1]
                            and mask[nrow, ncol] and not seen[nrow, ncol]):
                        seen[nrow, ncol] = True
                        stack.append((nrow, ncol))
            if len(blob) >= 4:
                found.append(blob)
        return found

    # A vision address nothing answers on: the picture cannot be posted, which
    # `show_map` reports and carries on from, and the view is recorded either way.
    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/video0",
                               vision="127.0.0.1:9")
    # No camera, so no violet cone is washed over the map. The blobs below are
    # found by exact colour and the cone recolours every solid pixel under it.
    rover.device = None
    rover.nav = FakeNav()

    first = rover.call("drive_to_map_point", {"across": 0.5, "down": 0.4})
    check("pointing at a map that has not been taken is refused",
          first.get("ok"), False)
    check("...and says to take one", "show_map" in first.get("error", ""), True)

    shown = rover.call("show_map", {"across_m": 6})
    check("the map is drawn", shown.get("ok"), True)
    check("...and its caption says the picture may be pointed at",
          "drive_to_map_point" in shown.get("caption", ""), True)
    view = rover._map_shown
    png, _caption = rover.nav.map_png(view["half_extent_m"], view["scale"])
    image = mapimg._decode(png)
    check("the picture is the size the remembered view says it is",
          image.shape[0], view["pixels"])

    solid = np.all(image == np.array(mapimg.C_OCCUPIED, dtype=np.uint8), axis=2)
    edge = max(6, view["scale"] * 3)          # the border and the one-metre bar
    solid[:edge, :] = solid[-edge:, :] = solid[:, :edge] = solid[:, -edge:] = False
    pointed = []
    for blob in blobs(solid):
        row = sum(r for r, _ in blob) / len(blob)
        col = sum(c for _, c in blob) / len(blob)
        answer = rover.call("drive_to_map_point",
                            {"across": col / view["pixels"],
                             "down": row / view["pixels"]})
        pointed.append((answer["pointed_at"]["x_m"],
                        answer["pointed_at"]["y_m"], answer))
    check("every obstacle drawn is one blob in the picture",
          len(pointed), len(obstacles))
    for wx, wy in obstacles:
        near = min(pointed, key=lambda p: math.hypot(p[0] - wx, p[1] - wy))
        # A cell and a half: the blob's own centroid is quantised to whole
        # pixels, and the picture is drawn at four pixels to a five-centimetre
        # cell. Anything wrong with the axes or the pose is metres out, not this.
        check(f"the obstacle at {wx:+.1f},{wy:+.1f} is where the picture puts it",
              math.hypot(near[0] - wx, near[1] - wy) < 0.08, True)
        check("...and driving onto it is refused", near[2].get("ok"), False)
        check("...as something solid", "solid" in near[2].get("error", ""), True)

    middle = rover.call("drive_to_map_point",
                        {"across": 0.5, "down": 0.5})["pointed_at"]
    check("the middle of the picture is where the rover is",
          (abs(middle["x_m"] - pose[0]) < 0.08,
           abs(middle["y_m"] - pose[1]) < 0.08), (True, True))
    check("...so pointing at it is no distance away", middle["range_m"] < 0.08, True)

    # Somewhere green, a good way out but not against the edge of what has been
    # seen, so the route is a real one rather than a nudge.
    free = np.all(image == np.array(mapimg.C_REACHABLE, dtype=np.uint8), axis=2)
    rows, cols = np.nonzero(free)
    middle_px = view["pixels"] / 2.0
    out = np.hypot(rows - middle_px, cols - middle_px)
    pick = int(np.argmin(np.abs(out - view["pixels"] * 0.3)))
    answer = rover.call("drive_to_map_point",
                        {"across": cols[pick] / view["pixels"],
                         "down": rows[pick] / view["pixels"], "speed_ms": 0.2})
    check("a green pixel is driven to", answer.get("ok"), True)
    check("...as a place on the map rather than an offset",
          sorted(k for k in asked[-1] if k != "speed_ms"), ["x_m", "y_m"])
    check("...at the point the picture named",
          (round(asked[-1]["x_m"], 2), round(asked[-1]["y_m"], 2)),
          (answer["pointed_at"]["x_m"], answer["pointed_at"]["y_m"]))
    check("...with the speed that was asked for", asked[-1]["speed_ms"], 0.2)
    check("...and says where it went, in the rover's own terms",
          sorted(answer["pointed_at"]),
          ["ahead_m", "left_m", "range_m", "x_m", "y_m"])

    # Grey needs a view wider than the rover has seen for there to be any.
    rover.call("show_map", {"across_m": 12})
    wide = rover._map_shown
    wide_png, _ = rover.nav.map_png(wide["half_extent_m"], wide["scale"])
    grey = np.all(mapimg._decode(wide_png)
                  == np.array(mapimg.C_UNKNOWN, dtype=np.uint8), axis=2)
    rows, cols = np.nonzero(grey)
    unseen = rover.call("drive_to_map_point",
                        {"across": cols[0] / wide["pixels"],
                         "down": rows[0] / wide["pixels"]})
    check("a grey pixel is refused", unseen.get("ok"), False)
    check("...as somewhere never seen rather than as somewhere empty",
          "grey" in unseen.get("error", ""), True)

    off = rover.call("drive_to_map_point", {"across": 1.4, "down": 0.5})
    check("a fraction past the edge of the picture is refused",
          off.get("ok"), False)
    check("...and is not quietly clamped to the edge instead",
          "fraction" in off.get("error", ""), True)

    rover._map_shown["at"] -= rover_daemon.MAP_POINT_MAX_AGE_S + 1
    stale = rover.call("drive_to_map_point", {"across": 0.5, "down": 0.4})
    check("a map the model is no longer looking at is refused",
          stale.get("ok"), False)
    check("...and says to take a fresh one",
          "show_map" in stale.get("error", ""), True)

    schema = next(s for s in [rover_daemon.MAP_POINT_TOOL]
                  if s["function"]["name"] == "drive_to_map_point")
    check("the model is offered the picture and not the metres",
          set(schema["function"]["parameters"]["properties"]),
          {"across", "down", "speed_ms"})
    check("...and both fractions are required",
          schema["function"]["parameters"]["required"], ["across", "down"])


def test_wifi_status_without_the_helper_still_reports_the_link():
    """The console's network panel has to work without NetworkManager.

    `wifi_ctl.sh` is the privileged helper. When it is missing the page used to
    show only that sentence -- no SSID, no address -- even though the kernel
    already knew both.

    The dual-radio manager is stubbed away rather than left to chance, and that
    is not tidiness either: this test passed on a desk and failed on the rover
    the moment `wifi_dual.py` was armed there, because the manager's status file
    exists on one of those machines and not the other. A check whose answer
    depends on what the host it runs on happens to be running is not checking
    what it says it is.
    """
    import rover_daemon
    import rover_wifi

    rover = rover_daemon.Rover(FakeLink(), "unused", device=None)
    real_ctl = rover_wifi._wifi_ctl
    real_live = rover_wifi._wifi_from_kernel
    real_dual = rover_wifi._dual_status
    rover_wifi._dual_status = lambda: None
    rover_wifi._wifi_ctl = lambda *args, **kwargs: (
        False, "/usr/local/sbin/wifi_ctl.sh is not installed on this rover; "
               "run wifi_roam/install.sh")
    rover_wifi._wifi_from_kernel = lambda iface="wlan0": {
        "interface": iface, "connected": "TheGreatLord", "level_dbm": -47,
        "address": "192.168.1.47",
        "networks": [{"ssid": "TheGreatLord", "signal": -47, "security": "",
                      "in_use": True, "configured": True}],
        "configured": ["TheGreatLord"], "scanned": False, "list_age_s": 0.0,
    }
    try:
        got = rover.call("wifi_status", {})
    finally:
        rover_wifi._wifi_ctl = real_ctl
        rover_wifi._wifi_from_kernel = real_live
        rover_wifi._dual_status = real_dual
    check("wifi_status still answers", got.get("ok"), True)
    check("...with the associated network", got.get("connected"), "TheGreatLord")
    check("...and the address", got.get("address"), "192.168.1.47")
    check("...and says the helper is missing", "install" in str(got.get("note", "")), True)


def test_wifi_status_with_two_radios():
    """What the console is told on a rover that has a spare radio.

    Every field the panel has read since this call existed still has to mean
    what it meant -- `connected` is the network the traffic is going through and
    `address` is where to reach the rover -- while now being about whichever
    radio is currently active, and with the whole picture underneath for a
    console that knows to look for it.

    The address is the one worth pinning down. On a rover with a service address
    it is that, not the active interface's DHCP lease, because the point of
    having one is that it is the answer that stays true across a failover.
    """
    import rover_daemon
    import rover_wifi

    rover = rover_daemon.Rover(FakeLink(), "unused", device=None)
    real_dual = rover_wifi._dual_status
    rover_wifi._dual_status = lambda: {
        "active": "wlan1", "standby": "wlan0", "switches": 1,
        "service_ip": "192.168.1.80", "service_on": "wlan1",
        "radios": [
            {"iface": "wlan0", "kind": "onboard", "role": "standby",
             "ssid": "TheGreatLord 5G", "band": "5", "dbm": -71,
             "address": "192.168.1.139", "usable": True,
             "seen": [{"ssid": "TheMaharaja 5G", "dbm": -60, "band": "5"}]},
            {"iface": "wlan1", "kind": "usb", "role": "active",
             "ssid": "TheMaharaja", "band": "2.4", "dbm": -55,
             "address": "192.168.1.100", "usable": True,
             "seen": [{"ssid": "TheGreatViking", "dbm": -80, "band": "2.4"}]},
        ],
    }
    try:
        got = rover.call("wifi_status", {})
    finally:
        rover_wifi._dual_status = real_dual
    check("wifi_status answers from the manager", got.get("ok"), True)
    check("...naming the network the traffic is on",
          got.get("connected"), "TheMaharaja")
    check("...with that radio's signal", got.get("level_dbm"), -55)
    check("...and the address that survives a failover",
          got.get("address"), "192.168.1.80")
    check("...with both radios underneath",
          len(got.get("dual", {}).get("radios", [])), 2)
    # A 5 GHz network is audible to exactly one of these radios, so a list built
    # from either alone would hide half the neighbourhood.
    heard = {n["ssid"] for n in got.get("networks", [])}
    check("the list merges what both radios heard",
          {"TheMaharaja 5G", "TheGreatViking"} <= heard, True)
    check("...and includes the networks they are actually on, which neither "
          "scan can contain", {"TheMaharaja", "TheGreatLord 5G"} <= heard, True)


def test_an_unfilled_signal_column_is_a_moment_not_an_answer():
    """The M4 Zero's driver leaves /proc/net/wireless at -256 now and then.

    -256 is "not filled in" rather than a reading, and the next read is usually
    good, so it is worth re-reading before falling back. The fallback is no use on
    this board anyway -- `iw` reports `signal: 0 dBm` here, which is not a level
    either -- so giving up on one sample cost the console its signal entirely.
    """
    import rover_wifi

    reads = []

    def fake(iface="wlan0"):
        reads.append(iface)
        return None if len(reads) < 3 else -41

    slept = []
    real_proc, real_iw = rover_wifi._proc_level_dbm, rover_wifi._iw_signal_dbm
    real_sleep = rover_wifi.time.sleep
    rover_wifi._proc_level_dbm = fake
    rover_wifi._iw_signal_dbm = lambda iface="wlan0": "asked iw"
    rover_wifi.time.sleep = slept.append
    try:
        check("an unfilled column is read again", rover_wifi._wifi_level_dbm(), -41)
        check("...and iw is not asked while /proc still answers", len(reads), 3)
        # The driver refreshes the figure on a timer, so a re-read that does not
        # wait is the same read again and cannot come back different.
        check("...having waited between the tries", slept, [rover_wifi.PROC_LEVEL_GAP_S] * 2)
        reads.clear()
        del slept[:]
        rover_wifi._proc_level_dbm = lambda iface="wlan0": reads.append(iface)
        check("a column that never fills falls back to iw",
              rover_wifi._wifi_level_dbm(), "asked iw")
        check("...after trying /proc a few times",
              len(reads), rover_wifi.PROC_LEVEL_TRIES)
        check("...and does not wait after the last try",
              len(slept), rover_wifi.PROC_LEVEL_TRIES - 1)
    finally:
        rover_wifi._proc_level_dbm, rover_wifi._iw_signal_dbm = real_proc, real_iw
        rover_wifi.time.sleep = real_sleep


def test_control_calls_without_hardware():
    """The two newest control calls refuse rather than raise when the hardware they
    need is not fitted.

    Worth its own check because both are reached from a window with live buttons
    rather than from a model that would read an explanation: a daemon started
    without `--ros-nav` still shows a `clear map` button, and pressing it has to
    come back as a sentence rather than as a traceback that closes the connection.
    """
    import rover_daemon

    rover = rover_daemon.Rover(FakeLink(), "unused", device=None)
    blind = rover.call("clear_map", {})
    check("clear_map with nothing to drive is refused", blind["ok"], False)
    # Deliberately not "lidar". The sensor belongs to the ROS stack, so a daemon
    # started without --ros-nav is one whose lidar is spinning perfectly well, and
    # a refusal that named it would send somebody to check a cable that is fine.
    check("...and says why", "--ros-nav" in blind["error"], True)
    # camera_jpeg needs a camera and *not* a vision host, which is the whole
    # difference between it and `look`: a daemon with nowhere to post a picture can
    # still hand one back in the reply.
    check("camera_jpeg is not gated on a vision host",
          "vision" in rover.call("camera_jpeg", {})["error"], False)
    # The script runner is attached by main() once the port is known, so a Rover
    # built any other way -- here, or by anything embedding it -- has none. The
    # four that need one have to say so rather than raise on a None.
    for name in ("run_script", "start_script", "script_status", "script_stop"):
        refused = rover.call(name, {"source": "print(1)"})
        check(f"{name} without a runner is refused", refused["ok"], False)
        check(f"...and says why", "not running scripts" in refused["error"], True)
    # `list_api` is the exception and needs nothing: it describes the primitives
    # by looking at the module, which is a question worth answering on a daemon
    # that is not currently in a position to run anything.
    reference = rover.call("list_api", {})
    check("list_api answers without a runner", reference["ok"], True)
    check("...with the primitives in it",
          all(word in reference["reference"] for word in ("gimbal.look_at", "every")),
          True)


def test_the_api_only_calls_tools_that_exist():
    """Every daemon call `rover_api.py` makes has a handler behind it.

    The same check `test_schemas` makes for the model's schemas, for the other
    surface: a script's primitive that names a tool which has since been renamed
    fails as "no such tool" in the middle of a behaviour, several minutes into a
    run, which is a poor place to find out. Read out of the source rather than
    from a list kept beside it, because a list kept beside it is a list that
    stops being true.
    """
    import re

    import rover_daemon

    with open(os.path.join(HERE, "rover_api.py"), "r", encoding="utf-8") as handle:
        source = handle.read()
    called = sorted(set(re.findall(r'_call\("([a-z_]+)"', source)))
    check("rover_api calls something", len(called) > 8, True)
    missing = [n for n in called if not hasattr(rover_daemon.Rover, f"_tool_{n}")]
    check("every primitive names a tool that exists", missing, [])
    # And a script must not be able to start a script: one slot, and a behaviour
    # that can spawn behaviours is a slot that means nothing.
    check("no primitive starts another script",
          [n for n in called if n.endswith("_script")], [])


def test_scripts_run_and_say_what_happened():
    """A script runs, prints, and comes back as an outcome rather than an exit code.

    These spawn a real `python3`, which is the point -- the isolation being tested
    is a process boundary, and a fake one would be testing nothing. They need no
    rover: none of these scripts calls a primitive.
    """
    import scripting

    runner = scripting.Runner("127.0.0.1:1")  # nothing there; nothing calls it
    try:
        done = runner.run("print('hello'); print(6 * 7)")
        check("a script's output comes back", done["output"].split(), ["hello", "42"])
        check("...and it says it finished", done["outcome"], "finished")

        # The line number is the whole point of compiling the source as
        # `<script>`: a traceback pointing into a temp file names something the
        # person who asked for the behaviour cannot look at.
        broken = runner.run("a = 1\nb = facse\n")
        check("a broken script fails", broken["outcome"], "failed")
        check("...and names the line", broken["error"].startswith("line 2: NameError"),
              True)
        check("a syntax error names its line too",
              runner.run("def (:")["error"].startswith("line 1: SyntaxError"), True)

        # A program starts with the primitives already defined, because the import
        # line was the step a model kept leaving a name out of. Proved against an
        # address with nothing on it: reaching the daemon and failing to is a
        # different error from never having heard of `lights`, and it is the one
        # that says the name was there.
        bare = runner.run("lights.set(255)")
        check("a script needs no import to reach a primitive",
              bare["error"].startswith("line 1: RoverError"), True)
        check("...and the exceptions are there to be caught by name",
              runner.run("try:\n"
                         "    lights.set(255)\n"
                         "except RoverError:\n"
                         "    print('refused')\n")["output"].strip(),
              "refused")
        # And the import that a program written by hand would use still works.
        check("importing them anyway changes nothing",
              runner.run("from rover_api import lights\n"
                         "lights.set(255)\n")["error"].startswith(
                             "line 2: RoverError"), True)
        # `ok` on a blocking run is the script's own fate, because that is what
        # the caller asked. Everywhere else it means the daemon answered.
        check("a failed script reports ok false", broken["ok"], False)
        check("a status call about a failed script still reports ok true",
              runner.status()["ok"], True)

        # The one that matters, and the one an interpreter inside the daemon
        # could not do without help from the language: a script with no exit in
        # it, spinning, is still stopped.
        started = time.monotonic()
        runaway = runner.run("while True:\n    pass\n", limit_s=1.0)
        took = time.monotonic() - started
        check("a runaway script is stopped", runaway["outcome"], "stopped")
        # Bounded in terms of the module's own allowance rather than a number
        # written here. Starting an interpreter is four seconds on the rover and
        # a fifth of one on a workstation, so a fixed bound would be measuring
        # which machine the test is running on.
        check(f"...at about the time it was given ({took:.1f}s)",
              0.9 < took < scripting.STARTUP_S + 1.0 + scripting.GRACE_S + 4.0, True)
    finally:
        runner.close()


class _StandInDaemon:
    """A daemon on loopback that takes its time over a turn, and remembers when.

    The offline checks could not ask about two calls at once before, because that
    needs something at the far end still holding the first when the second
    arrives. `turn_in_place` sleeps here; everything else answers at once; and
    what was called and when is kept both ways round, since the question these
    tests ask is about order in time rather than about answers.
    """

    TURN_S = 1.5

    def __init__(self) -> None:
        import socketserver
        import threading

        heard = self.heard = []  # (seconds in, name, "->" arriving or "<-" answered)
        began = time.monotonic()
        turn_s = self.TURN_S

        class Handler(socketserver.StreamRequestHandler):
            def handle(self) -> None:
                for line in self.rfile:
                    if not line.strip():
                        continue
                    name = json.loads(line).get("call")
                    heard.append((time.monotonic() - began, name, "->"))
                    if name == "turn_in_place":
                        time.sleep(turn_s)
                    heard.append((time.monotonic() - began, name, "<-"))
                    self.wfile.write(json.dumps({"ok": True}).encode() + b"\n")
                    self.wfile.flush()

        class Server(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True

        self._server = Server(("127.0.0.1", 0), Handler)
        self.address = "127.0.0.1:%d" % self._server.server_address[1]
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

    def when(self, name: str, arrow: str):
        """The times a call arrived, or was answered, in the order it happened."""
        return [at for at, called, direction in self.heard
                if called == name and direction == arrow]

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()


def test_two_things_at_once():
    """A job runs while the rover turns, and is over when the block that started it is.

    Everything that moves this rover blocks until the move is over, so a program
    written as one list of calls can only do one thing at a time -- and a model
    asked to turn and flash the headlights together produced a turn and then some
    flashing. What is checked here is the fix from the far end's point of view:
    light changes arriving while the turn the daemon is still working on has not
    been answered yet.

    Then the two failures that made threads unusable on their own. A job that
    raised used to leave a run reported as finished, with the traceback printed
    into the output where a model reads it as something the program meant to say;
    and a thread the program walked away from used to hold the rover's one script
    slot open for as long as it ran, behind a script that thought it had finished.
    """
    import scripting

    fake = _StandInDaemon()
    runner = scripting.Runner(fake.address)
    try:
        done = runner.run(
            "from rover_api import lights, drive, every, alongside\n"
            "def flashing():\n"
            "    for tick in every(0.3):\n"
            "        lights.set(255 if tick % 2 == 0 else 0)\n"
            "with alongside(flashing):\n"
            "    drive.turn(90)\n"
            "print('turned')\n", limit_s=15)
        check("a script with a job alongside it finishes", done["outcome"], "finished")
        began = (fake.when("turn_in_place", "->") or [0.0])[0]
        ended = (fake.when("turn_in_place", "<-") or [0.0])[0]
        during = [at for at in fake.when("set_lights", "->") if began < at < ended]
        check(f"the lights change while the rover is still turning ({len(during)}x)",
              len(during) >= 2, True)
        # One more may already be on its way when the block ends -- the job is
        # stopped at its next `every`, not mid-call -- and none after that.
        check("...and stop when the block that started them ends",
              [at for at in fake.when("set_lights", "->") if at > ended + 0.5], [])

        # The other way round, which is how the model actually wrote it: the slow
        # move is the job and the quick repeated thing is the block. Leaving the
        # block has to wait for a job that is one long call rather than cut it
        # off, or the rover stops half way through a drive it then reports as
        # done. The flashing here is over in a third of a second and the turn
        # takes the stand-in a second and a half.
        before = len(fake.when("turn_in_place", "<-"))
        inverted = runner.run(
            "def turning():\n"
            "    drive.turn(180)\n"
            "with alongside(turning):\n"
            "    for tick in every(0.1, ticks=3):\n"
            "        lights.set(255 if tick % 2 == 0 else 0)\n"
            "print('both done')\n", limit_s=15)
        check("a block whose job is the slow half finishes",
              inverted["outcome"], "finished")
        check("...having waited for the move rather than cutting it short",
              len(fake.when("turn_in_place", "<-")) > before, True)
        check(f"...which is why it took the turn's own time "
              f"({inverted['script_seconds']:.1f}s)",
              inverted["script_seconds"] >= _StandInDaemon.TURN_S, True)

        failed = runner.run(
            "from rover_api import drive, alongside\n"
            "def bad():\n"
            "    raise RuntimeError('the job fell over')\n"
            "with alongside(bad):\n"
            "    drive.turn(90)\n", limit_s=15)
        check("a job that fails fails the script", failed["outcome"], "failed")
        check("...and names the line inside the job",
              failed["error"], "line 3: RuntimeError: the job fell over")

        started = time.monotonic()
        left = runner.run(
            "import threading, time\n"
            "threading.Thread(target=time.sleep, args=(30,)).start()\n"
            "print('done')\n", limit_s=15)
        took = time.monotonic() - started
        check("a script is over when its last line has run", left["outcome"],
              "finished")
        # In terms of the module's own startup allowance rather than a number
        # written here, for the reason the runaway test gives: a fixed bound
        # measures which machine the test is running on. The thread sleeps thirty
        # seconds, so there is no reading of this that passes by accident.
        check(f"...even with a thread of its own still going ({took:.1f}s)",
              took < scripting.STARTUP_S + 4.0, True)
    finally:
        runner.close()
        fake.close()


def test_the_script_tools_are_offered_to_the_rover_and_not_to_the_lan():
    """Who is shown the three scripting tools, and whether they are usable.

    The gate is the interesting half. All three are refused on anything but
    loopback (`LOCAL_ONLY`), so a client across the LAN that was shown one of the
    schemas would be holding a tool whose every call comes back "reach it through
    an ssh tunnel" -- and a model with a tool like that reports doing things it
    has not done, which is the failure `Rover.tools` exists to prevent.

    Then that `run_script`'s description arrives finished. It is a literal with
    `{api}` in it until something fills it in, and a schema handed to a model with
    a formatting placeholder still in it is a schema that teaches it nothing.

    And last, that starting is not offered without stopping. A behaviour has no
    deadline any more, so those two are one facility: a model that can take the
    rover's single script slot and cannot give it back is worse off than one that
    was never able to take it.
    """
    import rover_daemon
    import scripting

    rover = rover_daemon.Rover(FakeLink(), "unused", device=None)
    def names(**kw):
        return [t["function"]["name"] for t in rover.tools(**kw)]

    check("a daemon not running scripts offers none, even on loopback",
          [n for n in names(local=True) if "script" in n], [])

    rover.scripts = scripting.Runner("127.0.0.1:1")  # nothing there; nothing calls it
    try:
        check("a client on the LAN is shown none of them",
              [n for n in names() if "script" in n], [])
        check("...and the default is the LAN, so a caller has to say otherwise",
              names(), names(local=False))
        check("a client on the rover is shown all three",
              names(local=True)[-3:], ["run_script", "start_script", "script_stop"])
        # Which is also the ordering check: they come after the tools whose
        # order was measured, and a start is never offered without its stop.
        check("no scripting tool comes before the measured ones",
              [n for n in names(local=True)[:-3] if "script" in n], [])

        described = rover.script_tools()[0]["function"]["description"]
        check("run_script's description is filled in, not the literal",
              "{api}" in described or "{limit_s}" in described, False)
        # Two primitives and the limit, read from the modules that own them
        # rather than written here: the point of generating this is that a
        # renamed primitive cannot go on being advertised.
        check("...with the primitives a program is written against",
              all(word in described
                  for word in ("gimbal.look_at", "drive.forward", "every(")), True)
        check("...and the runner's own limit in it",
              f"{scripting.RUN_LIMIT_S:.0f} seconds" in described, True)
        # The other two point at that list rather than carrying a second copy,
        # which is what keeps a realtime session from paying for it twice.
        starting = rover.script_tools()[1]["function"]["description"]
        check("start_script sends the model to run_script for the primitives",
              "run_script" in starting and "gimbal.look_at" not in starting, True)
        check("...and says what ends it", "script_stop" in starting, True)
        check("...and that only one runs at a time",
              "one program runs at a time" in starting, True)
        check("the list is the same object next time, built once",
              rover.script_tools() is rover.script_tools(), True)
    finally:
        rover.scripts.close()


def test_one_script_at_a_time():
    """The single slot, which is the whole of what keeps behaviours from piling up.

    It carries more weight than it did: a behaviour has no deadline, so nothing
    frees the slot on its own any more and the refusal has to name what is
    holding it. Started here with a limit, because this one is meant to be over
    quickly whichever way the test ends.
    """
    import scripting

    runner = scripting.Runner("127.0.0.1:1")
    try:
        first = runner.start("import time\ntime.sleep(30)\n", limit_s=30)
        check("the first script starts", first["ok"], True)
        second = runner.start("print(1)")
        check("a second is refused rather than queued", second["ok"], False)
        check("...and says which one is running", first["id"] in second["error"], True)
        stopped = runner.stop()
        check("stopping succeeds even though the script did not", stopped["ok"], True)
        check("...and the run is recorded as stopped", stopped["outcome"], "stopped")
        check("a slot freed by a stop takes the next script",
              runner.run("print('after')")["outcome"], "finished")
    finally:
        runner.close()


def test_a_behaviour_runs_until_it_is_stopped():
    """No deadline on a `start`, so a stop is the only thing that ends this one.

    The script here is the same runaway `while True` that the blocking test
    shoots on time, and the point is that nothing shoots it: the run carries no
    wall limit at all, the child is told so, and it is still going a second
    later. What it is checked against is the runner's own state rather than a
    number written here, because the assertion is "there is no deadline" and not
    "the deadline is long".
    """
    import scripting

    runner = scripting.Runner("127.0.0.1:1")
    try:
        started = runner.start("while True:\n    pass\n")
        check("a behaviour starts", started["ok"], True)
        check("...with no limit reported, because it has none",
              started["limit_s"], None)
        check("...and none recorded either", runner._run.wall_limit, None)
        # A second is nothing next to the five minutes this used to get, so it
        # is evidence about the watcher rather than about the clock: what it
        # proves is that the run is still alive with nobody having stopped it.
        time.sleep(1.0)
        check("it is still running with nothing to end it",
              runner.status()["outcome"], "running")
        stopped = runner.stop()
        check("a stop is what ends it", stopped["outcome"], "stopped")
        check("...and it says who did", stopped.get("error"), "stopped")
        check("the slot is free afterwards",
              runner.run("print('after')")["outcome"], "finished")

        # Asking for a deadline still works, and is still bounded below at a
        # second: this is the caller who does want its behaviour to end.
        asked = runner.start("import time\ntime.sleep(30)\n", limit_s=45)
        check("a behaviour may ask for a limit and gets it", asked["limit_s"], 45.0)
        runner.stop()
        # And nought is how a caller says "no limit" out loud rather than by
        # leaving the argument out, which the console has to be able to do.
        explicit = runner.start("import time\ntime.sleep(30)\n", limit_s=0)
        check("nought asks for no limit at all", explicit["limit_s"], None)
        runner.stop()
    finally:
        runner.close()


def test_where():
    import rover_daemon

    # Positions are described in words because the model reads them out loud;
    # asking a 4B model to do arithmetic on pixel coordinates goes badly.
    wide = 640
    left = rover_daemon._where([10, 100, 80, 80, 0.9], wide, 480)
    middle = rover_daemon._where([280, 100, 80, 80, 0.9], wide, 480)
    right = rover_daemon._where([550, 100, 80, 80, 0.9], wide, 480)
    check("a face on the left is called left", left["where"], "left")
    check("a face in the middle is called centre", middle["where"], "centre")
    check("a face on the right is called right", right["where"], "right")
    near = rover_daemon._where([100, 100, 200, 200, 0.9], wide, 480)
    far = rover_daemon._where([100, 100, 40, 40, 0.9], wide, 480)
    check("a big face is near", near["distance"], "near")
    check("a small face is far", far["distance"], "far")


def test_map_view():
    import rover_daemon

    res = 0.05

    def picture(half, pixels):
        """The size the map comes out, and the extent it covers."""
        got_half, scale = rover_daemon._map_view(half, pixels, res)
        return got_half, rover_daemon._map_cells(got_half, res) * scale

    # The whole point of deriving pixels per cell rather than asking for it: zooming
    # changes what is in frame and leaves the picture the size it was. Whole cells at
    # whole pixels cannot hit every size exactly, so this allows a few percent -- but
    # nothing like the five-fold swing you get from fixing the magnification instead.
    wanted = 480
    sizes = [picture(half, wanted)[1]
             for half in (0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0)]
    check("every zoom step returns about the size asked for",
          all(abs(size - wanted) <= wanted * 0.06 for size in sizes), True)
    check("...so widening the view does not resize the picture",
          max(sizes) - min(sizes) <= 40, True)

    # Past that a cell is down to one or two whole pixels and the size cannot be held.
    # It still has to degrade rather than break: smaller, never bigger than asked.
    check("wider than the console offers still returns a sane picture",
          all(0 < picture(half, wanted)[1] <= wanted for half in (8.0, 10.0)), True)

    # Asking for a bigger picture is the separate control, and it must actually work.
    small = picture(3.0, 320)[1]
    large = picture(3.0, 800)[1]
    check("a bigger picture was asked for and is bigger", large > small * 1.8, True)
    check("...and covers the same ground",
          picture(3.0, 320)[0], picture(3.0, 800)[0])

    # Nonsense is pulled to the ends rather than refused: these are view settings, and
    # a picture at the nearest sane setting beats an error where a map should be.
    check("a negative extent lands on the floor", rover_daemon._map_view(
        -5.0, wanted, res)[0], 0.5)
    check("a huge extent lands on the ceiling", rover_daemon._map_view(
        500.0, wanted, res)[0], rover_daemon.MAP_MAX_HALF_EXTENT_M)
    check("an absurd size is capped",
          picture(3.0, 99999)[1] <= rover_daemon.MAP_MAX_PIXELS, True)
    check("a tiny size still draws something",
          picture(3.0, 1)[1] >= rover_daemon._map_cells(3.0, res), True)

    # Never below one pixel a cell, however wide: a coarse map is still a map, and
    # zero pixels per cell is an empty picture rather than a cheap one.
    for half in sorted({2.0, 6.0, rover_daemon.MAP_MAX_HALF_EXTENT_M}):
        got_half, scale = rover_daemon._map_view(half, wanted, res)
        check(f"{half:g} m across draws at least a pixel a cell", scale >= 1, True)
        check(f"...and {half:g} m is not silently narrowed", got_half, half)


def test_flags():
    import rover_daemon

    # A small quantised model writes booleans loosely, and refusing those means
    # refusing the tool -- the same reasoning as _level.
    for value in (True, 1, "true", "True", " yes ", "on", "1"):
        check(f"{value!r} means yes", rover_daemon._flag(value, "f"), True)
    for value in (False, 0, "false", "no", "off", "0", ""):
        check(f"{value!r} means no", rover_daemon._flag(value, "f"), False)
    # But a word nobody can read is an error, not a silent False: quietly turning a
    # typo into "no" would hand back a picture facing the wrong way and looking fine.
    for value in ("maybe", "upwards", None, [], {}):
        try:
            rover_daemon._flag(value, "rover_up")
            check(f"{value!r} is refused", "accepted", "ValueError")
        except ValueError as error:
            check(f"{value!r} is refused, saying which argument",
                  "rover_up" in str(error), True)


def test_reading_the_network():
    """Parsing what nmcli says, and refusing a network there is no key for.

    The parsing earns a check of its own because its input is a string a stranger
    chose. An SSID may contain a colon, `nmcli -t` escapes it, and a `split(":")`
    gets away with that until the day somebody's router is called something
    awkward -- at which point the panel shows the wrong signal against the wrong
    name and looks like it is working.

    The rest is what happens on a machine with no wifi helper installed at all,
    which is every machine but the rover: a refusal in words, and never a
    traceback, because both calls are wired to live buttons in a window.
    """
    import rover_daemon

    check("an escaped colon stays inside its field",
          rover_daemon._terse_fields(r"*:My\:Net:84:WPA2"),
          ["*", "My:Net", "84", "WPA2"])
    check("an escaped backslash does too",
          rover_daemon._terse_fields(r"\\:x:1"), ["\\", "x", "1"])

    rows = "\n".join(("*:TheGreatLord:52:WPA2",
                      " :TheGreatLord:40:WPA2",      # the same router's other radio
                      " :TheMaharaja:84:WPA2",
                      " :Stranger:99:WPA2",
                      " ::70:WPA2"))                 # a hidden network
    seen = rover_daemon._wifi_networks(rows, {"TheGreatLord", "TheMaharaja"})
    check("one row per network, not per radio",
          [n["ssid"] for n in seen], ["TheMaharaja", "TheGreatLord", "Stranger"])
    check("...at the strongest signal that network was heard on",
          [n["signal"] for n in seen if n["ssid"] == "TheGreatLord"], [52])
    check("...still marked as the one in use",
          [n["in_use"] for n in seen if n["ssid"] == "TheGreatLord"], [True])
    check("...and a hidden network is not offered as a choice",
          any(not n["ssid"] for n in seen), False)
    check("the ones with a passphrase come first",
          [n["configured"] for n in seen], [True, True, False])

    # And the two calls themselves, which answer differently depending on where
    # this is run -- so both worlds are checked rather than the convenient one.
    # On the rover the helper is installed and this reads the real radio; anywhere
    # else it is absent, and what comes back has to be a sentence rather than a
    # traceback, because both calls are wired to live buttons in a window.
    rover = rover_daemon.Rover(FakeLink(), "unused", device=None)
    check("wifi_join wants an ssid", rover.call("wifi_join", {})["ok"], False)
    # And one *with* a name, which is the only path a console button ever takes and
    # the only one that reaches the rest of the call. The check above returns two
    # lines in, so it was covering none of it -- which is how a wifi_join that
    # raised TypeError before it did anything went unnoticed on a rover that had
    # been asked to switch networks by hand. `call` turns that into a sentence like
    # any other refusal, so it reads as the rover declining rather than as a bug.
    #
    # A name no rover holds a passphrase for, so this is safe to run on the rover
    # itself: there it is refused before the radio is touched, and on a desk, where
    # there is no helper to ask, the request is accepted and the thread behind it
    # finds nothing to run.
    named = rover.call("wifi_join", {"ssid": "NoSuchNetworkHere"})
    check("wifi_join with a name is answered rather than raising",
          named.get("ok") is True or "no passphrase" in str(named.get("error", "")),
          True)
    asked = rover.call("wifi_status", {})
    if asked["ok"]:
        check("wifi_status answers with every field the panel reads",
              [key for key in ("interface", "connected", "level_dbm", "address",
                               "networks", "configured") if key not in asked], [])
    else:
        check("wifi_status without the helper says how to install it",
              "install" in asked["error"], True)


class FakePort:
    """A serial port that hands over exactly what it has been given."""

    def __init__(self):
        self.pending = bytearray()
        self.closed = False

    def feed(self, text):
        self.pending += text.encode()

    @property
    def in_waiting(self):
        return len(self.pending)

    def read(self, n):
        out, self.pending = bytes(self.pending[:n]), bytearray(self.pending[n:])
        return out

    def write(self, data):
        return len(data)

    def reset_input_buffer(self):
        self.pending = bytearray()

    def close(self):
        self.closed = True


def _link_over(port):
    """A SerialLink around a fake port, with its backstop thread never started.

    Constructed without running __init__, because that opens a real port and
    starts a thread -- neither of which this wants. What is under test is the
    draining and the folding, which are ordinary methods.
    """
    import rover_daemon

    link = rover_daemon.SerialLink.__new__(rover_daemon.SerialLink)
    link.port = "fake"
    link.link = port
    link._lock = rover_daemon.threading.Lock()
    link._motion_lock = rover_daemon.threading.Lock()
    link._pump_lock = rover_daemon.threading.Lock()
    link._newest = None
    link._newest_at = 0.0
    link._sample_at = None
    link._gz_lsb_s = 0.0
    link._ticks = None
    link._samples = 0
    link._breaks = 0
    link._buffered = bytearray()
    link._drained_at = None
    link._stop = rover_daemon.threading.Event()
    link._reader = rover_daemon.threading.Thread(target=lambda: None)
    return link


LINE = ('{{"T":1001,"L":0,"R":0,"ax":104,"ay":-132,"az":8392,'
        '"gx":8,"gy":5,"gz":{gz},"mx":190,"my":346,"mz":1468,'
        '"odl":{odl},"odr":{odr},"v":1208}}\n')


def test_reading_the_board():
    """The gyro and the wheel counts, picked out of the board's own chatter.

    This parsing is hand-rolled rather than left to json.loads, because on the
    rover it runs at the board's rate rather than a human's -- so it is exactly
    the sort of thing that works on the happy line and quietly returns nothing on
    a real one. See _field_number.
    """
    import rover_daemon

    line = LINE.format(gz=-650, odl=9222, odr=8883).encode()
    check("the yaw rate comes out of a raw line",
          rover_daemon._field_number(line, b'"gz":'), -650.0)
    check("and so does a wheel count",
          rover_daemon._field_number(line, b'"odl":'), 9222.0)
    check("a field the board did not send is absent, not zero",
          rover_daemon._field_number(line, b'"nope":'), None)
    check("a field at the end of the line still parses",
          rover_daemon._field_number(line, b'"v":'), 1208.0)
    check("a float field parses as one",
          rover_daemon._field_number(b'{"T":1001,"L":0.19,"R":0}', b'"L":'), 0.19)

    # The board's own boot noise, and half a line, are both ordinary here.
    port = FakePort()
    link = _link_over(port)
    port.feed("garbage that is not json\n")
    check("noise on the port folds to nothing", link.pump(), 0)
    port.feed('{"T":1002,"other":1}\n')
    check("another message type is not telemetry", link.pump(), 0)
    check("and none of it counted as a sample", link.motion(), None)

    port.feed(LINE.format(gz=10, odl=100, odr=100)[:40])
    check("half a line is not a sample yet", link.pump(), 0)
    port.feed(LINE.format(gz=10, odl=100, odr=100)[40:])
    check("and is one once the rest arrives", link.pump(), 1)
    check("the wheel count is the mean of the two sides",
          link.motion()["ticks"], 100.0)

    # The first drain has nothing to measure an interval against, so it cannot
    # integrate -- and must not invent an interval to do it with.
    check("the first line integrates nothing", link.motion()["gz_lsb_s"], 0.0)

    # Two lines drained together were taken at the board's own spacing, not at the
    # instant they happened to be read. Sharing the interval between them is what
    # keeps that true; stamping on arrival would give the first one all of it.
    port.feed(LINE.format(gz=100, odl=110, odr=110))
    port.feed(LINE.format(gz=100, odl=120, odr=120))
    before = link.motion()["gz_lsb_s"]
    # The expected figure is derived from the interval the fold actually saw, not
    # from the one asked for here. Two Python statements are not 100 ms apart to
    # any particular precision on the rover's Pi, and a tolerance loose enough to
    # cover that would stop checking the arithmetic.
    started = rover_daemon.time.monotonic() - 0.1
    link._drained_at = started
    check("both lines of a batch are counted", link.pump(), 2)
    turned = link.motion()["gz_lsb_s"] - before
    want = 100.0 * (link.motion()["at"] - started)
    check("a batch integrates its whole interval at the rate reported",
          abs(turned - want) < 1e-6, True)
    check("and that interval was about the 100 ms asked for",
          0.09 < link.motion()["at"] - started < 0.5, True)
    check("and the newest wheel count is the one kept",
          link.motion()["ticks"], 120.0)

    # A gap this thread was not awake for is the one thing that must not be
    # integrated: a yaw rate multiplied by it is rotation that never happened.
    breaks = link.motion()["breaks"]
    port.feed(LINE.format(gz=500, odl=130, odr=130))
    link._drained_at = rover_daemon.time.monotonic() - 30.0
    link.pump()
    check("a thirty-second hole is counted, not integrated",
          link.motion()["breaks"], breaks + 1)

    # Two drains in the same instant are ordinary -- the navigator's loop and the
    # backstop thread share this port -- and must not read as a hole. Calling one
    # of those a hole marks the span untrustworthy, which switches off the prior
    # and the witness for it.
    breaks = link.motion()["breaks"]
    port.feed(LINE.format(gz=0, odl=140, odr=140))
    link._drained_at = rover_daemon.time.monotonic()
    link.pump()
    check("two drains at the same instant are not a hole",
          link.motion()["breaks"], breaks)

    # A board that has restarted begins its counters again, and the difference
    # across that is metres of travel that never happened.
    breaks = link.motion()["breaks"]
    port.feed(LINE.format(gz=0, odl=9000, odr=9000))
    link.pump()
    check("a board that restarted its counters is caught",
          link.motion()["breaks"], breaks + 1)

    # The battery still comes out of the same stream, parsed only when asked.
    port.feed(LINE.format(gz=0, odl=0, odr=0))
    link.pump()
    check("the pack voltage survives the cheap path",
          link.telemetry()["v"], 1208)



def test_the_probe_waits_to_be_answered():
    """A write is not an answer, and this is the check the whole boot rests on.

    `probe` used to be `link.send(...)`, which reports whether the write left this
    host. A serial write succeeds into an unplugged cable, so it said yes to a
    board that was not there -- and because `run_daemon.sh` only retries when the
    daemon *exits*, and the daemon only exits when this returns False, the retry
    loop written for exactly this race never ran. The rover came up at boot holding
    a port the ESP32 was not yet talking on, and stayed that way: no telemetry, no
    odometry, no transform, and slam_toolbox dropping every scan it was given.
    """
    import rover_daemon

    talking = rover_daemon.Rover(FakeLink(), "unused", device=None)
    check("a board that answers passes the probe", talking.probe(wait_s=0.2), True)

    # volts=None is a link whose `telemetry()` returns nothing -- an unpowered
    # board, the wrong serial port, and an ESP32 still booting all look like this.
    silent = FakeLink(volts=None)
    rover = rover_daemon.Rover(silent, "unused", device=None)
    began = time.monotonic()
    answered = rover.probe(wait_s=0.2)
    check("a silent board fails it", answered, False)
    check("...having actually waited rather than returned at once",
          time.monotonic() - began >= 0.2, True)
    check("...and it kept asking while it waited", silent.reads > 1, True)

    # The write still succeeding is the point: nothing about the old check was
    # broken in a way a caller could have noticed from its return value.
    check("...even though every write to it succeeded",
          all(rover.link.send(c) for c in ({"T": 130},)), True)


def test_a_board_that_goes_quiet_gets_its_port_reopened():
    """The lidar has a replug ladder; the board had nothing at all.

    Reopening is not a general repair -- it cannot fix a cable -- but it is the
    whole of the repair for the case that actually bit: a port opened before the
    thing at the other end was ready, held open and dead from then on. What has to
    hold is that silence is noticed, that the reopen is not attempted twice a
    second, and that a board which is talking is left alone.
    """
    import board_link

    class Port:
        """A serial port that can be told to stop delivering, and counts opens."""

        opens = 0

        def __init__(self, *_a, **_k):
            Port.opens += 1
            self.closed = False

        in_waiting = 0

        def read(self, _n):
            return b""

        def write(self, _line):
            return len(_line)

        def reset_input_buffer(self):
            pass

        def close(self):
            self.closed = True

    import serial
    real, Port.opens = serial.Serial, 0
    serial.Serial = Port
    try:
        link = board_link.SerialLink("/dev/null")
        link._stop.set()                     # the reader thread is not the subject
        check("opening the link opened the port", Port.opens, 1)

        # Just opened, nothing heard yet, but not yet silent for long enough.
        check("a port only just opened is left alone", link.watch(), False)

        # Silence measured from the open, not from the last line -- a board that
        # has never spoken is the case this exists for, and a "time since the last
        # line" test would never fire on it.
        link._spoke_at = time.monotonic() - (board_link.BOARD_SILENT_S + 1.0)
        check("silence since the open is noticed", link.watch(), True)
        check("...and the port was reopened", Port.opens, 2)
        check("...and it is counted where a console can see it", link.reopens, 1)
        check("...and says what it did", "silence" in (link.reopen_note or ""), True)
        check("...and marks a hole in the gyro's integral", link._breaks >= 1, True)

        # Immediately quiet again, but the backoff has not elapsed.
        check("a second attempt waits for the backoff", link.watch(), False)
        check("...so the port was not reopened again", Port.opens, 2)

        # And the backoff really doubles across a board that never comes back.
        # It only does so because reopening does not count as the board speaking:
        # marking it as such made every `watch` take the "heard from it" branch and
        # reset the interval, so a rover with an unplugged cable would have
        # reopened its port every five seconds for as long as it was switched on.
        waits = []
        for _ in range(4):
            waits.append(link._reopen_wait)
            link._reopen_at = 0.0            # pretend the interval elapsed
            link.watch()
        check("the backoff doubles while the board stays silent",
              waits, [waits[0] * 2 ** i for i in range(4)])
        check("...and each of those was a real reopen", Port.opens, 6)
        check("...and it is capped rather than doubling for ever",
              min(board_link.BOARD_REOPEN_MAX_S,
                  board_link.BOARD_REOPEN_S * 2 ** 40),
              board_link.BOARD_REOPEN_MAX_S)

        # And a board that starts talking again resets the backoff, so the next
        # fault does not inherit a two-minute wait from the last one.
        link._reopen_wait = board_link.BOARD_REOPEN_MAX_S
        link._spoke_at = time.monotonic()
        check("hearing from the board clears the backoff", link.watch(), False)
        check("...back to the first interval",
              link._reopen_wait, board_link.BOARD_REOPEN_S)
    finally:
        serial.Serial = real


def main():
    for test in (test_levels, test_battery, test_reading_the_board,
                 test_schemas, test_lights, test_gimbal,
                 test_no_camera, test_default_camera, test_look, test_snapshot_splitting,
                 test_driving_takes_the_core,
                 test_counting_faces_does_not_hold_the_board,
                 test_the_local_detector_scales_its_boxes_back_up, test_camera_cone,
                 test_map_png_names_the_clock,
                 test_show_map_takes_across_and_size,
                 test_drive_to_takes_a_place_on_the_map,
                 test_a_point_on_the_map_picture_is_the_place_it_looks_like,
                 test_wifi_status_without_the_helper_still_reports_the_link,
                 test_wifi_status_with_two_radios,
                 test_an_unfilled_signal_column_is_a_moment_not_an_answer,
                 test_control_calls_without_hardware,
                 test_the_probe_waits_to_be_answered,
                 test_a_board_that_goes_quiet_gets_its_port_reopened,
                 test_reading_the_network,
                 test_the_api_only_calls_tools_that_exist,
                 test_scripts_run_and_say_what_happened,
                 test_two_things_at_once,
                 test_the_script_tools_are_offered_to_the_rover_and_not_to_the_lan,
                 test_one_script_at_a_time,
                 test_a_behaviour_runs_until_it_is_stopped, test_where,
                 test_map_view, test_flags, test_aiming_through_a_missed_frame,
                 test_one_move_puts_a_face_in_the_middle,
                 test_the_approach_to_a_face_never_turns_back,
                 *ROS_NAV_TESTS):
        try:
            test()
        except Exception as exc:
            FAIL.append(f"{test.__name__} raised {type(exc).__name__}: {exc}")

    for name in PASS:
        print(f"  ok   {name}")
    for name in SKIP:
        print(f"  skip {name}")
    for name in FAIL:
        print(f"  FAIL {name}")
    print(f"\n{len(PASS)} passed, {len(FAIL)} failed, {len(SKIP)} skipped")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
