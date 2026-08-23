"""The ESP32 link: UART or Wi-Fi HTTP, plus pack-voltage helpers."""
from __future__ import annotations

import json
import os
import re
import threading
import time
from typing import Any

# Banana Pi M4 Zero puts the 40-pin UART on UART4 as ttyS4. The Pi 1 used
# ttyAMA0 for the same header pins. Prefer whichever exists so one default
# works on both; fall back to ttyS4 so a missing port still has a name to report.
SERIAL_CANDIDATES = ("/dev/ttyS4", "/dev/ttyAMA0")


def default_serial() -> str:
    for port in SERIAL_CANDIDATES:
        if os.path.exists(port):
            return port
    return SERIAL_CANDIDATES[0]


DEFAULT_SERIAL = default_serial()
BAUD = 115200
CMD_LIGHTS = 132  # CMD_LED_CTRL, both channels driven together as one headlight
CMD_PROBE = 130   # a harmless query; the board answers with its usual telemetry
# The board talks back, in one JSON object per line tagged T:1001, whether or not
# anything asked. Most of what is in that line this daemon has no use for -- a
# 9-DoF IMU, a magnetometer and wheel encoders, all of which the lidar's scan
# matcher beats as an odometer. The pack voltage is the exception: nothing else on
# this rover measures it, and there is no second opinion to be had.
TELEMETRY_T = 1001
# The same thing as a substring, for picking the board's lines out of its chatter
# without parsing everything it says. It writes its JSON with no spaces.
TELEMETRY_TAG = b'"T":1001'
# How long to wait for a whole line. One arrives about every 60 ms, so this is
# several chances at it rather than a tight budget.
TELEMETRY_WAIT_S = 0.4
# Older than this and the board has stopped talking rather than been slow, so the
# answer is "nothing" rather than a number from before whatever went wrong.
TELEMETRY_MAX_AGE_S = 1.0
# The longest gap between two lines that a yaw rate may be integrated across.
# Lines arrive about every 50 ms; a gap ten times that is the reader having been
# starved of the core, and multiplying a rate by it invents rotation that never
# happened. Such an interval is counted instead of integrated -- see SerialLink.
MAX_SAMPLE_GAP_S = 0.5
# A wheel count that moves further than this between two lines did not come from
# wheels. The board restarts its counters from zero when it reboots, and the
# difference across that reads as several metres of travel.
MAX_TICK_STEP = 5000
# How often the reader wakes to drain the board's stream.
#
# This is the number that decides what reading the gyro costs, and on this host it
# had to be got right rather than merely got working. The core is single and the
# scan matcher has most of it, so the expensive part of a second thread is not the
# work it does but the times it wakes: every wakeup preempts the matcher's loop and
# forces a hand-off of the interpreter lock. Two versions measured against the
# rover's real 9.6 Hz, over 40 s each:
#
#   asking for bytes as soon as any arrive   8.7 Hz, 10% of revolutions dropped
#   draining once every 50 ms                see below
#
# The first looks like the attentive design and is the trap: at 115200 baud a line
# takes 13 ms to clock in, so a reader that returns the moment a byte lands spends
# that whole 13 ms going round again for the next one, twenty times a second.
#
# Draining on a clock got most of that back, and the rest came from noticing that
# the thread need not be the one doing it. The navigator already has a loop running
# at the lidar's rate, and a drain folded into a loop that was going to run anyway
# costs no wakeup at all -- so `pump` is public, the navigator calls it once a
# revolution, and this thread drops to a slow backstop for the case where nothing
# else is pumping. On the rover that is the daemon started without --lidar, where
# the pack voltage is all anyone wants out of the stream.
#
#   asking for bytes as soon as any arrive   8.7 Hz, 10% of revolutions dropped
#   draining on a 50 ms clock               9.34 Hz,  5%
#   the navigator draining, this at 250 ms   see the README
TELEMETRY_POLL_S = 0.25

# Three 18650 cells in series -- the UPS in docs/d500-lidar.md -- reported as
# hundredths of a volt.
BATTERY_CELLS = 3
# Volts per cell against percentage left. A table rather than a straight line
# because lithium-ion is nearly flat through the middle of its discharge, where
# 40% to 70% is a tenth of a volt: interpolating from full to empty would read
# twenty points high for most of a run. Both ends of it are Waveshare's own
# numbers rather than a guess -- the module's balancing chips start bleeding a cell
# at 4.200 V and its published gauge calls 9.0 V empty -- and only the shape between
# them is this table's. See the battery section of README.md for the sources.
BATTERY_CURVE = ((3.00, 0), (3.45, 5), (3.68, 10), (3.74, 20), (3.77, 30),
                 (3.79, 40), (3.82, 50), (3.87, 60), (3.92, 70), (3.98, 80),
                 (4.06, 90), (4.20, 100))
# Below this there is no pack at all: the ESP32 runs from USB alone with the
# battery out or the main switch off, and reports a few tenths of a volt. Its own
# state rather than 0%, because a flat battery and a missing one call for
# different things being done about them.
BATTERY_ABSENT_V = 6.0
# Read off the curve above rather than picked: 11.2 V is 3.73 V/cell, which is
# about a fifth left, and 10.8 V is 3.6 V/cell, which is nearly nothing and is
# also where the cells start to suffer. Both trip early on a rover that is
# driving, because a reading under load sags -- which is the right direction for
# a warning to be wrong in.
BATTERY_LOW_V = 11.2
BATTERY_CRITICAL_V = 10.8
# What a full pack reads once the rover's own draw has taken the surface charge
# off it. Not 12.6, because the host, the lidar and the OAK are always pulling
# something and every reading here is a reading under load.
BATTERY_FULL_V = 12.45
# How long one reading is served for before the board is asked again.
BATTERY_MAX_AGE_S = 5.0

def _field_number(line: bytes, key: bytes) -> float | None:
    """One numeric field out of a JSON line, without parsing the line.

    For the three fields of the board's telemetry that are read at its own rate
    rather than at a human's -- the yaw rate and the two wheel counts. Everything
    else in the line goes through `json.loads` like normal, just far less often.
    Returns None when the field is absent or is not a bare number, which is the
    same answer a parse would give and is what the caller already handles.
    """
    at = line.find(key)
    if at < 0:
        return None
    at += len(key)
    end = at
    while end < len(line) and line[end] not in b",}":
        end += 1
    try:
        return float(line[at:end])
    except ValueError:
        return None


def _newest_telemetry(chatter: bytes) -> dict[str, Any] | None:
    """The last complete T:1001 object in a chunk of the board's own chatter.

    Complete, hence the dropped tail: a read of a stream lands mid-line as often
    as not, and half an object parses as nothing. Newest rather than first,
    because a buffer may hold a second of history and only its end is now.
    """
    for line in reversed(chatter.split(b"\n")[:-1]):
        try:
            message = json.loads(line.strip())
        except (ValueError, UnicodeDecodeError):
            continue     # a truncated first line, or the board's own boot noise
        if isinstance(message, dict) and message.get("T") == TELEMETRY_T:
            return message
    return None


def _battery_percent(volts: float) -> int:
    """Roughly how much charge is left, from the pack voltage.

    Rounded to five points, because the reading does not deserve more: it is taken
    under whatever the host, the lidar and the servos happen to be drawing, and the
    sag from that alone is worth several points. What is worth having is the shape
    of the number over an afternoon, not the number.
    """
    per_cell = min(max(volts / BATTERY_CELLS, BATTERY_CURVE[0][0]),
                   BATTERY_CURVE[-1][0])
    for (low_v, low_pc), (high_v, high_pc) in zip(BATTERY_CURVE, BATTERY_CURVE[1:]):
        if per_cell <= high_v:
            share = (per_cell - low_v) / (high_v - low_v)
            return int(round((low_pc + share * (high_pc - low_pc)) / 5.0) * 5)
    return 100


def _battery_state(volts: float) -> str:
    """One word for the pack, for something that has to say it out loud."""
    if volts < BATTERY_ABSENT_V:
        return "absent"
    if volts < BATTERY_CRITICAL_V:
        return "critical"
    if volts < BATTERY_LOW_V:
        return "low"
    if volts >= BATTERY_FULL_V:
        return "full"
    return "ok"


def _battery_summary(volts: float, state: str) -> str:
    """The reading as a sentence, since a model reads this and repeats the gist.

    Written as an answer rather than as a row of fields, because the two ends of
    the range are the ones that get repeated wrongly: a percentage on its own gets
    read out as a fact about the rover, and "absent" gets read out as a flat
    battery, which is a different thing to go and do something about.
    """
    if state == "absent":
        return (f"There is no battery pack on this rover -- the board reads "
                f"{volts:.1f} V, which is what it says when it is running from USB "
                f"with the pack out or the main power switch off.")
    percent = _battery_percent(volts)
    if state == "full":
        return f"The battery is full, at {percent}% and {volts:.1f} volts."
    if state == "critical":
        return (f"The battery is nearly flat, at {percent}% and {volts:.1f} volts. "
                f"It needs charging now.")
    if state == "low":
        return f"The battery is low, at {percent}% and {volts:.1f} volts."
    return f"The battery is at about {percent}%, or {volts:.1f} volts."

class SerialLink:
    """JSON commands down the GPIO UART to the ESP32, and its telemetry back.

    Locked on the way out, unlike the copies in the other scripts, because this is
    the one that is genuinely shared: a tool call arrives on a connection thread
    while the tracking loop is commanding servos on its own, and two interleaved
    writes are one line of JSON the board cannot parse.

    The way in is read by a thread of its own that never stops, and that is a
    change of kind rather than of degree. This port used to be *sampled* -- thrown
    away on every write and read only when somebody asked about the battery --
    because the pack voltage was the only thing wanted out of the stream and it
    moves over hours. The gyro is in the same stream and moves in tens of
    milliseconds, and a rate cannot be sampled: what it did between two looks is
    the whole of it. So lines are folded in as they arrive, into a running integral
    of yaw rate and the newest wheel counts, which `motion` hands out. See
    `lidar_slam/odometry.py` for what reads them and what it takes to believe them.
    """

    def __init__(self, port: str) -> None:
        import serial

        self.port = port
        self.link = serial.Serial(port, BAUD, timeout=0.1)
        self._lock = threading.Lock()
        # Guards everything the reader thread writes and everyone who reads it.
        # Deliberately not the write lock: the two directions of a serial port do
        # not interfere with each other, and sharing one lock would mean a line
        # arriving could delay the PWM going to the wheels.
        self._motion_lock = threading.Lock()
        self._newest: dict[str, Any] | None = None
        self._newest_at = 0.0
        self._sample_at: float | None = None
        self._gz_lsb_s = 0.0      # integral of raw gz over time, LSB-seconds
        self._ticks: float | None = None
        self._samples = 0
        self._breaks = 0          # intervals the integral cannot vouch for
        # Guards the port's read side and the half-line left over from the last
        # drain. Separate from _lock, which is the write side, because a drain and
        # a command have no reason to wait for each other.
        self._pump_lock = threading.Lock()
        self._buffered = bytearray()
        self._drained_at = None
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_forever,
                                        name="telemetry", daemon=True)
        self._reader.start()

    def describe(self) -> str:
        return f"{self.port} at {BAUD}"

    def send(self, command: dict[str, Any]) -> bool:
        line = json.dumps(command, separators=(",", ":")).encode() + b"\n"
        with self._lock:
            try:
                self.link.write(line)
                if not self._reader.is_alive():
                    # The board streams telemetry continuously whether or not
                    # anything asked, and left undrained it fills within seconds.
                    # The reader thread is what drains it now, and throwing the
                    # input away here would eat the very lines the gyro is read
                    # from -- so this is a fallback for a reader that has died,
                    # not the standing arrangement it used to be.
                    self.link.reset_input_buffer()
                return True
            except Exception:
                return False

    def _read_forever(self) -> None:
        """Drain the board's stream for as long as the link is open.

        A slow backstop, and deliberately slow -- see TELEMETRY_POLL_S. Whoever
        has a loop of their own calls `pump` from it and this thread finds nothing
        left to do; without one it keeps the buffer from filling and the pack
        voltage current on its own.
        """
        while not self._stop.wait(TELEMETRY_POLL_S):
            self.pump()

    def pump(self) -> int:
        """Drain whatever the board has said and fold it in. Returns lines read.

        Safe to call from any thread and from several, which is the point: the
        navigator calls it once a revolution to keep the gyro's timing as fine as
        the scan matcher's, and the backstop thread calls it when nobody else is.
        Never blocks -- it takes what has arrived and returns.
        """
        with self._pump_lock:
            try:
                waiting = self.link.in_waiting
                chunk = self.link.read(waiting) if waiting else b""
            except Exception:
                # A port that has gone away, or one being closed under us. Neither
                # is worth spinning on, and neither is this call's to fix.
                return 0
            if not chunk:
                return 0
            self._buffered += chunk
            lines = []
            while b"\n" in self._buffered:
                line, _, rest = self._buffered.partition(b"\n")
                self._buffered = bytearray(rest)
                if TELEMETRY_TAG in line:
                    lines.append(line)
            if len(self._buffered) > 4096:
                # Nothing the board says is remotely this long, so a buffer this
                # size is line noise or a board mid-reset. Keep the tail, which is
                # where a real line will resume.
                del self._buffered[:-512]
            if not lines:
                return 0
            now = time.monotonic()
            self._fold(lines, now, self._drained_at)
            self._drained_at = now
            return len(lines)

    def _fold(self, lines: list, now: float, previous: float | None) -> None:
        """A drain's worth of lines into the running state.

        The elapsed time is shared out evenly across the lines rather than each
        being stamped when it was parsed, and that is more faithful than stamping
        would be, not less: the board samples on its own fixed clock, so two lines
        pulled from the buffer together were taken 50 ms apart however close
        together they were read. Stamping on arrival would hand the first of them
        the whole interval and the second none of it.

        Values are picked out of the bytes rather than parsed as JSON, because a
        `json.loads` of a 150-byte line twenty times a second is a real slice of
        this core and only three of the fifteen fields are wanted. The whole line
        is kept as it arrived, for `telemetry` to parse when somebody actually
        asks about the battery -- which is seconds apart, not milliseconds.
        """
        share = None
        hole = False
        if previous is not None:
            elapsed = now - previous
            if elapsed > MAX_SAMPLE_GAP_S:
                hole = True
            elif elapsed > 0.0:
                share = elapsed / len(lines)
            # An elapsed of zero is neither. Two pumpers share this port -- the
            # navigator's loop and the backstop thread -- so two drains landing in
            # the same instant is ordinary rather than a fault, and calling it a
            # hole would mark the span around it untrustworthy and quietly switch
            # off both the prior and the witness. There is simply nothing to
            # integrate across no time, so nothing is.
        with self._motion_lock:
            self._newest = lines[-1]
            self._newest_at = now
            self._sample_at = now
            self._samples += len(lines)
            if hole:
                # A gap nothing was awake for. A yaw rate multiplied by it is
                # invented rotation, so it is counted instead of integrated and a
                # consumer can refuse the span it falls in.
                self._breaks += 1
            for line in lines:
                if share is not None:
                    gz = _field_number(line, b'"gz":')
                    if gz is not None:
                        self._gz_lsb_s += gz * share
                odl = _field_number(line, b'"odl":')
                odr = _field_number(line, b'"odr":')
                if odl is not None and odr is not None:
                    mean = (odl + odr) / 2.0
                    if (self._ticks is not None
                            and abs(mean - self._ticks) > MAX_TICK_STEP):
                        self._breaks += 1     # the board restarted its counters
                    self._ticks = mean

    def motion(self) -> dict[str, Any] | None:
        """Where the wheels and the gyro have got to, in the board's own units.

        Raw on purpose. Turning LSB-seconds into degrees needs a scale factor
        nobody has measured on this rover, and guessing it produces a prior that
        looks plausible while quietly dragging the scan match off true -- so what
        crosses this boundary is what the board actually said, and the
        interpretation happens where the evidence to calibrate it is.

        `breaks` is the part worth reading rather than skipping: it counts the
        intervals that could not be integrated, so a span whose count has not moved
        is a span this can vouch for, and one whose count has moved has a hole in
        it. None until the board has said anything at all.
        """
        with self._motion_lock:
            if self._samples == 0:
                return None
            return {"at": self._sample_at, "gz_lsb_s": self._gz_lsb_s,
                    "ticks": self._ticks, "samples": self._samples,
                    "breaks": self._breaks}

    def telemetry(self) -> dict[str, Any] | None:
        """The newest line the board has sent, or None if none arrived in time.

        Nearly always instant now, because the reader thread has one in hand
        already: the wait below is for a board that has only just been powered up
        or has stopped talking, not for the ordinary case. It still takes no lock
        the writer wants, so asking about the battery cannot stall the PWM.

        Fresh or nothing, as it always was. A line from before whatever went wrong
        would be stamped with the time it was *read*, and a battery reading that
        old should show up as an absence rather than as a number.
        """
        deadline = time.monotonic() + TELEMETRY_WAIT_S
        while True:
            with self._motion_lock:
                newest, at = self._newest, self._newest_at
            if newest is not None and time.monotonic() - at <= TELEMETRY_MAX_AGE_S:
                # Parsed here and not in the reader: this is asked once every few
                # seconds, and the reader runs twenty times a second.
                return _newest_telemetry(bytes(newest) + b"\n")
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.02)

    def close(self) -> None:
        self._stop.set()
        if self._reader.is_alive():
            self._reader.join(timeout=1.0)
        try:
            self.link.close()
        except Exception:
            pass


class HttpLink:
    """JSON commands over the ESP32's own `/js` endpoint, for a board on WiFi."""

    def __init__(self, host: str, timeout: float = 1.0) -> None:
        import http.client
        from urllib.parse import quote

        self._client = http.client
        self._quote = quote
        self.host = host
        self.timeout = timeout
        self.connection = None
        self._lock = threading.Lock()

    def describe(self) -> str:
        return f"http://{self.host}/js"

    def send(self, command: dict[str, Any]) -> bool:
        return self._ask(command) is not None

    def telemetry(self) -> dict[str, Any] | None:
        """The easy end of the job `SerialLink.telemetry` does the hard way.

        Over WiFi the reply to a command *is* the telemetry, so there is no stream
        to catch a whole line out of and nothing to wait for beyond the request.
        """
        body = self._ask({"T": CMD_PROBE})
        return None if body is None else _newest_telemetry(body + b"\n")

    def _ask(self, command: dict[str, Any]) -> bytes | None:
        """One command, and what the board said back -- None if it said nothing."""
        path = "/js?json=" + self._quote(
            json.dumps(command, separators=(",", ":")), safe="")
        with self._lock:
            for attempt in (1, 2):  # a stale keep-alive costs one retry
                if self.connection is None:
                    self.connection = self._client.HTTPConnection(
                        self.host, timeout=self.timeout)
                try:
                    self.connection.request("GET", path)
                    return self.connection.getresponse().read()
                except Exception:
                    self._close()
                    if attempt == 2:
                        return None
        return None

    def _close(self) -> None:
        if self.connection is not None:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None

    def close(self) -> None:
        with self._lock:
            self._close()

def open_link(serial_port: str | None, host: str | None):
    if host:
        return HttpLink(host)
    serial_port = serial_port or DEFAULT_SERIAL
    if not serial_port.startswith("/") and not re.fullmatch(r"COM\d+", serial_port, re.I):
        return HttpLink(serial_port)
    return SerialLink(serial_port)
