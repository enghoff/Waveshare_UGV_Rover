"""The rover as a program sees it: the primitives a script is written against.

This is imported by scripts, not by the daemon. It runs in a child process that
the daemon started, and it reaches the hardware the same way any other client
does -- a line of JSON to `rover_daemon.py` on the loopback port. That is the
whole isolation story: a script can ask for things and cannot touch anything, so
however badly one is written it cannot take the UART or the camera away from the
daemon that owns them, and stopping it is a signal rather than a language
feature. See [docs/scripting.md](../docs/scripting.md).

    from rover_api import camera, drive, gimbal, every

    gimbal.look_at(pan=0, tilt=0)
    for _ in every(2.0, for_s=60):
        print(len(camera.faces()), "people in view")

**A failed call raises rather than returning a flag.** The daemon answers
`{"ok": false, "error": ...}` because that reads well in a conversation, but a
program that has to check every result is a program a model will write wrong, so
the failure arrives here as :class:`RoverError` carrying the daemon's own
sentence. The harness turns that into "line 4: RoverError: the driver board did
not answer", which is what the person who asked for the behaviour ends up
hearing. Catch it where a failure is expected and ignore it everywhere else.

**This is not a sandbox against the filesystem.** The child is an ordinary
process with the daemon's own permissions; it can read and write files like
anything else running as `admin`. What it is isolated from is the hardware, and
what it is bounded by is a wall clock, a memory ceiling and a kill -- which are
the three ways a program on this rover can do damage without meaning to.

**Nothing here paces itself except :func:`every`.** This host has one core, and
one `camera.faces()` is a JPEG decode plus a round trip to the OAK -- about 0.4 s
of it. A loop that asks twice a second has taken the machine away from the scan
matcher that is keeping the rover off the walls. Use `every`; it is also where
the run's deadline and a stop are noticed.
"""

from __future__ import annotations

import json
import os
import signal
import socket
import sys
import threading
import time

# `base64` and `traceback` are imported where they are used rather than here, and
# that is not tidiness. Starting a CPython on this Pi costs about 1.8 s of a core
# that the daemon is already more than half using, and every stdlib module this
# file names is paid again on top of it before the script's first line runs --
# measured at 4.2 s all told against 1.8 s for a bare interpreter. One of the two
# is only needed by a script that touches the camera and the other only by one
# that has already failed, so neither belongs in the path every script pays.

DEFAULT_ADDRESS = "127.0.0.1:8769"
# Long enough for the slowest call this can make. `count_faces` with the camera
# cold has to start v4l2-ctl and wait for its first buffer, and a `drive` blocks
# in the daemon until the move is over -- which is a move, not a message, and can
# be a minute. The runner's wall-clock cap is the real bound; this only exists so
# that a daemon which has stopped answering is an error rather than a hang.
TIMEOUT_S = 120.0
CONNECT_TIMEOUT_S = 3.0


class RoverError(Exception):
    """A call the rover refused, carrying the daemon's own words."""


class Stopped(Exception):
    """Somebody stopped this script. Raised at the next `every` or `wait`."""


class Deadline(Exception):
    """The run's wall-clock limit came up. Raised where the script can see it."""


# --- the wire -------------------------------------------------------------

class _Daemon:
    """One connection to the daemon, remade once if it has gone.

    Remade, and the request sent again -- but only where there was an old
    connection to discover was closed, and never after a timeout. A timeout here
    is a daemon that has the request and is still working on it, and a script's
    patience is two minutes: what would be sent twice is a whole drive.

    Deliberately not `voice_chat/rover_tools.py`, which does the same job for the
    conversation clients: that file lives with the voice client and is not
    deployed to this Pi, and it discovers a rover across the LAN, which is
    exactly what this must not do. This end knows where the daemon is, because
    the daemon started it.
    """

    def __init__(self, address: str) -> None:
        host, _, port = address.partition(":")
        self.host = host or "127.0.0.1"
        self.port = int(port) if port else 8769
        self._sock: socket.socket | None = None
        self._file = None
        self._lock = threading.Lock()

    def call(self, name: str, arguments: dict | None = None) -> dict:
        request = json.dumps({"call": name, "arguments": arguments or {}})
        with self._lock:
            for attempt in (1, 2):
                reused = self._sock is not None
                try:
                    if self._sock is None:
                        self._connect()
                    self._file.write(request.encode() + b"\n")
                    self._file.flush()
                    reply = self._file.readline()
                    if not reply:
                        raise ConnectionError("the daemon closed the connection")
                    return json.loads(reply)
                except (OSError, ValueError) as error:
                    self._drop()
                    if (attempt == 2 or not reused
                            or isinstance(error, (socket.timeout, TimeoutError))):
                        raise RoverError(
                            f"no answer from the rover daemon: {error}") from None
        raise RoverError("unreachable")

    def _connect(self) -> None:
        sock = socket.create_connection((self.host, self.port), CONNECT_TIMEOUT_S)
        sock.settimeout(TIMEOUT_S)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock, self._file = sock, sock.makefile("rwb")

    def _drop(self) -> None:
        for handle in (self._file, self._sock):
            try:
                if handle is not None:
                    handle.close()
            except OSError:
                pass
        self._sock = self._file = None


_daemon = _Daemon(os.environ.get("ROVER_ADDRESS", DEFAULT_ADDRESS))


def _call(name: str, **arguments):
    """One tool, or RoverError. Every primitive below goes through here."""
    reply = _daemon.call(name, arguments)
    if not reply.get("ok"):
        raise RoverError(str(reply.get("error", "the rover refused, without saying why")))
    return reply


def call(name: str, **arguments):
    """Any tool the daemon has, by name -- the escape hatch under the namespaces.

    Everything below is a wrapper over this. It is public because the daemon's
    tool list grows and this file does not have to, so a behaviour written
    against a tool that arrived after this module did is one line rather than a
    deploy: `call("show_map")`. Ask `list_api` for what is currently offered.
    """
    return _call(name, **arguments)


# --- being stopped, and pacing --------------------------------------------

_stop = threading.Event()
_deadline: float | None = None


def _check() -> None:
    """Raise if this run is over. Called from every waiting primitive."""
    if _stop.is_set():
        raise Stopped("stopped")
    if _deadline is not None and time.monotonic() > _deadline:
        raise Deadline("the script ran past the time it was given")


def wait(seconds: float) -> None:
    """Sleep, but notice a stop while doing it."""
    _check()
    if _stop.wait(max(0.0, float(seconds))):
        raise Stopped("stopped")
    _check()


def every(period_s: float, for_s: float | None = None, ticks: int | None = None):
    """Tick on a schedule, yielding the tick number, until told otherwise.

    This is the loop primitive, and using it is not optional politeness: it is
    the one place that notices a stop, enforces the run's deadline, and yields
    the core between passes. A `while True` with a `time.sleep` in it does none
    of those things and is how a script becomes something that has to be killed.

        for tick in every(2.0, for_s=600):   # every two seconds, for ten minutes

    `for_s` and `ticks` end the loop normally, so the script carries on to
    whatever it meant to say afterwards. The run's own limit is different and
    raises :class:`Deadline`, because reaching that means the script did not
    finish, and a caller that is told "done" would be told something false.

    A tick that overruns its period does not try to catch up. On this host
    overrunning is what asking for too much per pass looks like, and catching up
    means never sleeping again -- so the schedule slips and the rover keeps
    breathing.
    """
    period_s = max(0.0, float(period_s))
    started = time.monotonic()
    ends_at = None if for_s is None else started + float(for_s)
    tick = 0
    while True:
        if ticks is not None and tick >= ticks:
            return
        if ends_at is not None and time.monotonic() >= ends_at:
            return
        _check()
        yield tick
        tick += 1
        rest = (started + tick * period_s) - time.monotonic()
        if rest > 0:
            if ends_at is not None:
                rest = min(rest, ends_at - time.monotonic())
            if rest > 0:
                wait(rest)
        else:
            _check()


def time_left() -> float | None:
    """Seconds before the run is stopped, or None if it was given no limit."""
    return None if _deadline is None else max(0.0, _deadline - time.monotonic())


# --- the hardware ----------------------------------------------------------

class _Lights:
    def set(self, level):
        """Headlights, 0 to 255. Accepts what the daemon accepts, including True."""
        return _call("set_lights", level=level)

    def get(self):
        """The last level that was set -- the board cannot be read back."""
        return _call("get_lights")


class _Gimbal:
    def look_at(self, pan=None, tilt=None):
        """Point the camera, in degrees from straight ahead: pan right, tilt up.

        Absolute, not relative, and either may be left out to keep the current
        one. Stops face tracking, which cannot aim the same servos at once.
        """
        return _call("look_at", pan=pan, tilt=tilt)

    def centre(self):
        """Straight ahead and level. Stops face tracking."""
        return _call("center_camera")

    center = centre

    def where(self):
        """(pan, tilt) as the daemon last commanded them, in degrees.

        `.get`, so that a rover which does not report where its camera is says
        so with a None rather than raising from inside a primitive -- a script
        that asked a reasonable question deserves a reasonable answer to check.
        """
        status = _call("tracking_status")
        return status.get("pan"), status.get("tilt")


class _Power:
    """The battery, which is the one thing on this rover that runs out."""

    def battery(self):
        """Charge left: percent, volts, and one word for the condition.

        There is no fuel gauge here -- the driver board measures the pack voltage
        and nothing measures current -- so the percentage is a discharge curve
        applied to a voltage taken under whatever load the rover is under. Good
        enough to decide whether a behaviour should keep going, not good enough to
        compare one run against another.
        """
        return _call("battery")

    def volts(self):
        """The pack voltage on its own, for a behaviour that logs it."""
        return _call("battery")["volts"]

    def percent(self):
        """Charge left as a number, or None on a rover with no pack fitted."""
        return _call("battery").get("percent")


class _Camera:
    def jpeg(self) -> bytes:
        """One frame, as JPEG bytes.

        The camera's own bytes, undecoded: there is no image library on this Pi.
        While face tracking is running this is the loop's newest frame rather
        than a fresh grab, because the loop owns the camera and nothing else may
        open it.
        """
        import base64

        return base64.b64decode(_call("camera_jpeg")["jpeg_base64"])

    def faces(self):
        """The people in view: a list of where each one is, left/centre/right.

        Costs a frame and a round trip to the detector -- about 0.4 s of this
        host's only core, which is the number that decides how fast a loop
        calling it is allowed to go round.
        """
        return _call("count_faces").get("where", [])

    def detect(self, jpeg: bytes | None = None):
        """Run the face detector over a picture and get the boxes back.

        `[x, y, w, h, score]` in the picture's own pixels. With no argument it
        takes a frame first, which is `faces()` with the geometry left in.
        """
        import base64

        if jpeg is None:
            jpeg = self.jpeg()
        blob = base64.b64encode(jpeg).decode("ascii")
        return _call("detect_in", jpeg_base64=blob)["faces"]

    def look(self):
        """Post a frame to the model's host so it can be asked about.

        Only on a daemon started with `--vision`, and it does not answer the
        question -- it puts the picture where whoever is holding the conversation
        can see it. A script cannot yet ask about a picture and get an answer
        back; see the open question in docs/scripting.md.
        """
        return _call("look")


class _Tracking:
    def start(self):
        """Follow a face with the camera, sweeping for one if nobody is there."""
        return _call("start_tracking")

    def stop(self):
        return _call("stop_tracking")

    def next(self):
        """Let go of whoever is being followed and take the next face found."""
        return _call("track_next")

    def status(self):
        """Whether it is running, whether it has somebody, and how fast it goes."""
        return _call("tracking_status")


class _Drive:
    """Moving the rover. Every one of these blocks until the move is over.

    There is no raw motor control here on purpose. The only real failsafe on this
    rover is the driver board's own heartbeat, which the navigator sets to 500 ms
    when it starts a move and then keeps fed; a script commanding PWM directly
    would have to reproduce that correctly, and getting it wrong is a rover still
    driving after the program steering it has died.
    """

    def forward(self, distance_m, speed_ms=None):
        """Drive straight, stopping short of anything the live scan sees."""
        return _call("drive", distance_m=distance_m, speed_ms=speed_ms)

    def to(self, ahead_m=0.0, left_m=0.0, speed_ms=None):
        """Drive to a place relative to where the rover is standing now."""
        return _call("drive_to", ahead_m=ahead_m, left_m=left_m, speed_ms=speed_ms)

    def turn(self, angle_deg):
        """Turn on the spot. Positive is left, the lidar's convention."""
        return _call("turn_in_place", angle_deg=angle_deg)

    def stop(self):
        return _call("stop_driving")

    def surroundings(self):
        """The room as the lidar has it: walls, objects, gaps, and a sentence.

        This is the live scan rather than the map, which is the distinction that
        matters -- the map drifts and holds geometry that has since moved, and
        this does not. `clear_ahead_m` is the number to steer by; `gaps` is the
        one to explore by.
        """
        return _call("describe_surroundings")

    def status(self):
        """Every number the driving loop has: PWM, measured speed, scan age."""
        return _call("nav_status")


lights = _Lights()
gimbal = _Gimbal()
camera = _Camera()
tracking = _Tracking()
drive = _Drive()
power = _Power()


def reference() -> str:
    """The whole surface above as text, built by looking at it.

    What `list_api` hands out. Generated rather than written down, so that a
    primitive whose signature changed cannot go on being documented the way it
    used to be -- the same reason the daemon answers `list_tools` instead of
    every client carrying a copy of the schemas.
    """
    import inspect

    lines = [(__doc__ or "").strip().split("\n\n")[0], ""]
    for name, thing in (("lights", lights), ("gimbal", gimbal), ("camera", camera),
                        ("tracking", tracking), ("drive", drive), ("power", power)):
        doc = (thing.__class__.__doc__ or "").strip().split("\n")[0]
        lines.append(f"{name}  -- {doc}" if doc else name)
        for attr in sorted(dir(thing)):
            if attr.startswith("_"):
                continue
            member = getattr(thing, attr)
            if not callable(member):
                continue
            signature = str(inspect.signature(member))
            summary = (member.__doc__ or "").strip().split("\n")[0]
            lines.append(f"    {name}.{attr}{signature}")
            if summary:
                lines.append(f"        {summary}")
        lines.append("")
    for func in (every, wait, time_left, call):
        lines.append(f"{func.__name__}{inspect.signature(func)}")
        lines.append(f"    {(func.__doc__ or '').strip().split(chr(10))[0]}")
    lines += ["", "Raises RoverError when the rover refuses a call, Stopped when "
                  "somebody stops the script, Deadline when it runs past its limit."]
    return "\n".join(lines)


# --- the harness -----------------------------------------------------------

def main() -> int:
    """Run one script, and leave an answer behind for the daemon to read.

    Invoked as `python3 -c "import rover_api; rover_api.main()"` rather than as a
    file, so that this module is loaded once under its own name: the script's own
    `import rover_api` then finds it in `sys.modules` and shares this copy. Run
    the file directly and there are two of them, with two stop flags, one of
    which nothing is listening to.

    The source is compiled as `<script>` so that a traceback points at the lines
    the model wrote rather than at a temporary file nobody will ever see.
    """
    global _deadline

    import traceback

    source_path = os.environ["ROVER_SCRIPT"]
    result_path = os.environ["ROVER_RESULT"]
    limit_s = float(os.environ.get("ROVER_LIMIT_S", "0")) or None
    if limit_s:
        _deadline = time.monotonic() + limit_s

    # A stop is SIGTERM, and it lands as an exception at the next `every` or
    # `wait` rather than killing the interpreter, so the script unwinds through
    # its own `finally` blocks -- which is where a well-written behaviour puts
    # stopping the wheels. The runner sends SIGKILL shortly afterwards for the
    # ones that are not well written.
    signal.signal(signal.SIGTERM, lambda *_: _stop.set())

    with open(source_path, "r", encoding="utf-8") as handle:
        source = handle.read()

    started = time.monotonic()
    result = {"ok": True}
    try:
        code = compile(source, "<script>", "exec")
        namespace = {"__name__": "__main__", "__builtins__": __builtins__}
        exec(code, namespace)
    except Stopped:
        result = {"ok": False, "stopped": True, "error": "stopped"}
    except Deadline as error:
        # Reported the same way an outside stop is, because to whoever asked for
        # the behaviour they are the same event -- it did not finish, and nothing
        # about the script itself was wrong.
        result = {"ok": False, "stopped": True, "error": str(error)}
    except SyntaxError as error:
        result = {"ok": False,
                  "error": f"line {error.lineno}: {type(error).__name__}: {error.msg}",
                  "traceback": traceback.format_exc(limit=0)}
    except BaseException as error:  # SystemExit included: a script may exit()
        if isinstance(error, SystemExit) and not error.code:
            result = {"ok": True}
        else:
            result = {"ok": False, "error": _where(error),
                      "traceback": traceback.format_exc()}
    result["seconds"] = round(time.monotonic() - started, 2)
    try:
        sys.stdout.flush()
    except Exception:
        pass
    with open(result_path, "w", encoding="utf-8") as handle:
        json.dump(result, handle)
    return 0 if result.get("ok") else 1


def _where(error: BaseException) -> str:
    """"line 4: NameError: name 'facse' is not defined", from the traceback.

    The last frame that is in the script itself, so a failure inside a primitive
    is still reported at the line that called it -- which is the line the person
    reading this can do something about.
    """
    import traceback

    frames = [f for f in traceback.extract_tb(error.__traceback__)
              if f.filename == "<script>"]
    where = f"line {frames[-1].lineno}: " if frames else ""
    return f"{where}{type(error).__name__}: {error}"
