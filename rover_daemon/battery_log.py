"""The pack voltage over time, kept on the rover, for fitting the charge curve.

The percentage the `battery` tool reports is a table of volts against charge
left, and on 2026-10-10 that table was shown to be badly wrong at the bottom:
the rover drove for more than half an hour after it called the pack 5%, and was
still driving at 0%. A table can only be fitted to this pack from a record of
what its voltage did across whole charges and discharges, and nothing kept one --
the only voltages on record were the ones the autonomy recorder happened to note
at its decisions. This is that record. See docs/plans/battery-charge-curve.md
for what it is for and how it gets used.

One row every second, appended to one file per boot under ~/.ugv/battery/:

    time         wall clock, seconds since the epoch. Wrong until the clock is
                 synchronised after a cold boot, which is what `uptime_s` is for
    uptime_s     seconds since the host booted, which is right from the start
    v_min        the lowest, mean and highest pack voltage the board reported in
    v_mean       the interval, at its own 17 Hz. The minimum is what a brownout
    v_max        is about and the mean is what a fit wants
    samples      how many readings went into them; 0 is a board that said nothing
    wheel_ticks  how far the wheels turned in the interval, left and right
                 together, in encoder counts -- a spin on the spot moves both,
                 which a mean of the two would cancel. Non-zero is a rover under
                 drive load
    host_mw      the Orin module's own input power, from its INA3221, which is
                 most of what the rover draws standing still. Blank elsewhere
    event        `start` when the daemon starts and `stop` when it is stopped,
                 including by the SIGTERM of a restart or a reboot. A file that
                 ends without `stop` ended in a crash or a power cut

Per boot rather than per day because the wall clock cannot be trusted until it
is synchronised, and a file named by a wrong date is worse than one named by
something that is never wrong. Each row is flushed and synced as it is written,
so a brownout loses at most the interval it happened in.
"""
from __future__ import annotations

import glob
import os
import threading
import time
from typing import Any, Callable

from board_link import MAX_TICK_STEP, _field_number

DIRECTORY = "~/.ugv/battery"
# A second rather than the five it was at first, because what a row cannot say
# is what happened after it: the rover hard-reset twice on 2026-10-10 and the
# question was whether the wheels had moved in the moments before, which a
# five-second row left open for up to five seconds.
INTERVAL_S = 1.0
# A row is about seventy bytes, so a day is six megabytes and half a year about
# one gigabyte, on a disk with eight hundred free. Old discharges stop
# describing the pack as it ages anyway.
KEEP_DAYS = 180
COLUMNS = ("time", "uptime_s", "v_min", "v_mean", "v_max", "samples",
           "wheel_ticks", "host_mw", "event")
# Where the Orin's power monitor is, and the channel that is the module's input.
INA3221_GLOB = "/sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*"
HOST_RAIL = "VDD_IN"


def _boot_id() -> str:
    try:
        with open("/proc/sys/kernel/random/boot_id") as f:
            return f.read().strip().replace("-", "")[:8] or "unknown"
    except OSError:
        return "unknown"


def _host_power_reader() -> Callable[[], float | None]:
    """A function returning the host's input power in milliwatts, or None.

    Found once, by label rather than by channel number, because the channel the
    module input sits on is the carrier board's choice and not a constant.
    """
    for hwmon in glob.glob(INA3221_GLOB):
        for label_path in glob.glob(os.path.join(hwmon, "in*_label")):
            try:
                with open(label_path) as f:
                    if f.read().strip() != HOST_RAIL:
                        continue
            except OSError:
                continue
            channel = os.path.basename(label_path)[2:-len("_label")]
            volts_path = os.path.join(hwmon, f"in{channel}_input")
            amps_path = os.path.join(hwmon, f"curr{channel}_input")

            def read(volts_path=volts_path, amps_path=amps_path) -> float | None:
                try:
                    with open(volts_path) as v, open(amps_path) as a:
                        return float(v.read()) * float(a.read()) / 1000.0
                except (OSError, ValueError):
                    return None
            return read
    return lambda: None


class BatteryLog:
    """Readings folded in as the board sends them, written out every interval.

    Two halves on two threads, so that neither the navigator's loop nor the wheels
    ever wait on a disk. `fold` is called from whoever pumps the board's stream,
    and only adds to running totals; `tick` is called from the link's slow backstop
    thread, and is the only thing that touches the file.
    """

    def __init__(self, directory: str = DIRECTORY, interval_s: float = INTERVAL_S,
                 clock: Callable[[], float] = time.time,
                 uptime: Callable[[], float] = time.monotonic,
                 host_power: Callable[[], float | None] | None = None,
                 boot_id: str | None = None) -> None:
        self.directory = os.path.expanduser(directory)
        self.interval_s = interval_s
        self._clock = clock
        self._uptime = uptime
        self._host_power = host_power if host_power is not None else _host_power_reader()
        self._lock = threading.Lock()
        self._closed = False
        self._fresh()
        self._left: float | None = None
        self._right: float | None = None
        os.makedirs(self.directory, exist_ok=True)
        self._prune()
        self.path = os.path.join(self.directory,
                                 f"boot-{boot_id or _boot_id()}.csv")
        new = not os.path.exists(self.path) or os.path.getsize(self.path) == 0
        self._file = open(self.path, "a")
        if new:
            self._file.write(",".join(COLUMNS) + "\n")
        self._due = self._uptime() + self.interval_s
        self._write({}, "start")

    def _fresh(self) -> None:
        self._low = None
        self._high = None
        self._sum = 0.0
        self._count = 0
        self._ticks = 0.0

    def _prune(self) -> None:
        cutoff = self._clock() - KEEP_DAYS * 86400.0
        for path in glob.glob(os.path.join(self.directory, "*.csv")):
            try:
                if os.path.getmtime(path) < cutoff:
                    os.remove(path)
            except OSError:
                pass

    def fold(self, lines: list) -> None:
        """A drain's worth of the board's T:1001 lines into the running totals."""
        with self._lock:
            for line in lines:
                volts = _field_number(line, b'"v":')
                if volts is not None:
                    volts /= 100.0
                    self._low = volts if self._low is None else min(self._low, volts)
                    self._high = volts if self._high is None else max(self._high, volts)
                    self._sum += volts
                    self._count += 1
                left = _field_number(line, b'"odl":')
                right = _field_number(line, b'"odr":')
                if left is None or right is None:
                    continue
                if self._left is not None:
                    step = abs(left - self._left) + abs(right - self._right)
                    # A board that restarts its counters reads as metres of travel.
                    if step <= 2 * MAX_TICK_STEP:
                        self._ticks += step
                self._left, self._right = left, right

    def tick(self) -> bool:
        """Write the interval out if it is over. True if a row was written."""
        if self._uptime() < self._due:
            return False
        self._due += self.interval_s
        if self._uptime() >= self._due:
            # A backstop that was starved for longer than an interval. Catching up
            # would write rows of nothing; the gap in `uptime_s` says what happened.
            self._due = self._uptime() + self.interval_s
        with self._lock:
            taken = {"low": self._low, "high": self._high, "sum": self._sum,
                     "count": self._count, "ticks": self._ticks}
            self._fresh()
        self._write(taken, "")
        return True

    def _write(self, taken: dict[str, Any], event: str) -> None:
        count = taken.get("count") or 0
        power = self._host_power()
        row = [f"{self._clock():.1f}", f"{self._uptime():.1f}",
               f"{taken['low']:.2f}" if count else "",
               f"{taken['sum'] / count:.3f}" if count else "",
               f"{taken['high']:.2f}" if count else "",
               str(count),
               f"{taken.get('ticks') or 0:.0f}",
               f"{power:.0f}" if power is not None else "",
               event]
        try:
            self._file.write(",".join(row) + "\n")
            self._file.flush()
            os.fsync(self._file.fileno())
        except (OSError, ValueError):
            pass   # a full disk or a closed file is not worth stopping the wheels for

    def close(self) -> None:
        """The `stop` row, once. Called from a signal handler as well as on exit."""
        if self._closed:
            return
        self._closed = True
        with self._lock:
            taken = {"low": self._low, "high": self._high, "sum": self._sum,
                     "count": self._count, "ticks": self._ticks}
            self._fresh()
        self._write(taken, "stop")
        try:
            self._file.close()
        except OSError:
            pass
