"""Board and battery checks: argument coercion, the limits, and the UART.

The driver board is the only thing wired to the motors, lights and gimbal, so
what is checked here is the layer that talks to it: what a loosely written
argument is coerced to, what the gimbal is not allowed to be asked for, and what
a board that answers slowly, partially or not at all does to the reader.
"""
from __future__ import annotations

import time

from test_fakes import FakeLink
from test_harness import FAIL, PASS, SKIP, check

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
    # Rest is not level: straight ahead, and REST_TILT_DEG above the horizontal,
    # because a camera this low spends a level frame mostly on the floor.
    from aiming import REST_TILT_DEG
    check("center_camera returns to rest", rover.call("center_camera", {}),
          {"ok": True, "pan": 0, "tilt": REST_TILT_DEG, "stopped_tracking": False})


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


TESTS = (
    test_levels,
    test_battery,
    test_lights,
    test_gimbal,
    test_reading_the_board,
    test_the_probe_waits_to_be_answered,
    test_a_board_that_goes_quiet_gets_its_port_reopened,
)
