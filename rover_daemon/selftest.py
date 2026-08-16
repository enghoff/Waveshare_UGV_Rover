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
import os
import sys

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

    def __init__(self, works=True):
        self.sent = []
        self.works = works

    def send(self, command):
        self.sent.append(command)
        return self.works

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


def test_schemas():
    import rover_daemon

    # Every schema this rover could ever offer, whatever it is configured with.
    # `look` is conditional -- see test_look -- but it is still a schema that has
    # to have a handler, and the point of this test is that none of them lie.
    every = rover_daemon.TOOLS + [rover_daemon.LOOK_TOOL]
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
    handlers = sorted(m[len("_tool_"):] for m in dir(rover_daemon.Rover)
                      if m.startswith("_tool_"))
    check("every handler is offered", sorted(names), handlers)
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
    for tool in ("count_faces", "start_tracking"):
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

    class FakeCamera:
        """Frames on demand, including the half-frame a cold camera really gives."""

        def __init__(self, frames):
            self.frames = list(frames)
            self.complaints = []

        def latest(self, timeout=None):
            return (self.frames.pop(0), 1.0) if self.frames else None

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

        # The half-frame is the one that matters: the reader starts mid-stream,
        # so the first end-of-image marker can end a fragment. Caught by the
        # two bytes at the front rather than by decoding anything.
        rover._open_camera = lambda: FakeCamera([fragment, whole])
        got = rover.call("look", {})
        check("a picture is sent and named", got.get("image"), "frame-1")
        check("...and it is the whole frame, not the fragment", posted[-1], whole)
        # The name and nothing else. A sentence in here is not a comment, it is
        # context handed to a model immediately before a picture, and the one
        # that used to sit in this result ("describe what is actually in it")
        # made every follow-up take a fresh photograph and describe the lot.
        check("...and the result is the name and nothing else", sorted(got), ["image", "ok"])

        # Three fragments running is a camera that is not producing pictures.
        rover._open_camera = lambda: FakeCamera([fragment, fragment, fragment])
        got = rover.call("look", {})
        check("nothing but fragments is a failure", got["ok"], False)
        check("...that says what happened", "whole pictures" in got["error"], True)

        # A camera that gives nothing at all, which is what an unplugged one does.
        rover._open_camera = lambda: FakeCamera([])
        check("no frame at all is a failure", rover.call("look", {})["ok"], False)

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
    rover._open_camera = lambda: FakeCamera([whole])
    got = rover.call("look", {})
    check("a vision service that has gone is reported", got["ok"], False)
    check("...naming where it tried", "/frame" in got["error"], True)
    rover.close()


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


def main():
    for test in (test_levels, test_schemas, test_lights, test_gimbal,
                 test_no_camera, test_look, test_where):
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
