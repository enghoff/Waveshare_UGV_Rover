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

PASS, FAIL, SKIP = [], [], []


def check(name, got, want):
    if got == want:
        PASS.append(name)
    else:
        FAIL.append(f"{name}\n     got:  {got!r}\n     want: {want!r}")


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
             + rover_daemon.NAV_TOOLS + [rover_daemon.MAP_TOOL])
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
               # The scripting calls, which are control calls twice over: no model
               # is shown them, and four of the five are refused on anything but
               # loopback -- see LOCAL_ONLY in rover_daemon.py.
               "run_script", "start_script", "script_status", "script_stop",
               "list_api"]
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
    except Exception as exc:
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


def test_control_calls_without_hardware():
    """The two newest control calls refuse rather than raise when the hardware they
    need is not fitted.

    Worth its own check because both are reached from a window with live buttons
    rather than from a model that would read an explanation: a daemon started
    without `--lidar` still shows a `clear map` button, and pressing it has to come
    back as a sentence rather than as a traceback that closes the connection.
    """
    import rover_daemon

    rover = rover_daemon.Rover(FakeLink(), "unused", device=None)
    blind = rover.call("clear_map", {})
    check("clear_map with no lidar is refused", blind["ok"], False)
    check("...and says why", "lidar" in blind["error"], True)
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


def test_one_script_at_a_time():
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


def main():
    for test in (test_levels, test_battery, test_schemas, test_lights, test_gimbal,
                 test_no_camera, test_look, test_snapshot_splitting,
                 test_driving_takes_the_core,
                 test_counting_faces_does_not_hold_the_board, test_camera_cone,
                 test_control_calls_without_hardware,
                 test_the_api_only_calls_tools_that_exist,
                 test_scripts_run_and_say_what_happened,
                 test_one_script_at_a_time, test_where,
                 test_map_view, test_flags, test_aiming_through_a_missed_frame,
                 test_one_move_puts_a_face_in_the_middle,
                 test_the_approach_to_a_face_never_turns_back):
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
