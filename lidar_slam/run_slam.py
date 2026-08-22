#!/usr/bin/env python3
"""Run slam2d against the rover's real sensors and report what it makes of them.

On the Pi the lidar is /dev/ttyACM0 at 230400 -- the CH343 behind the driver
board's Type-C LIDAR socket, claimed by cdc_acm, so there is no /dev/ttyUSB* to
look for. The driver board's own telemetry is a separate port, /dev/ttyAMA0 at
115200, and it is left alone by default because rover_daemon.py normally owns it
and only one process can.

    ssh rpi 'cd ~/ugv/lidar_slam && ./build.sh && python3 run_slam.py --seconds 30 --map room.pgm'

Lidar-only is the supported path. Reading the driver board as well is opt-in for
the port's sake rather than the prior's: with `--telemetry` this measures the
resting gyro, reports whether the gyro and the scan match agree about rotation --
which needs no calibration and is the more useful half -- and centres the search
window with whichever scale factors have been measured. Those live in
`odometry.json` once a confirmed turn or drive has produced them; the flags below
override them for an experiment.
"""
import argparse
import json
import math
import signal
import sys
import time

import serial

from odometry import MAX_SAMPLE_GAP_S, MAX_TICK_STEP, Odometry
from slam2d import Slam2D, default_config

LIDAR_BAUD = 230400
TELEM_BAUD = 115200
# What counts as the rover standing still, so the resting gyro can be measured.
# The matcher's own numbers, so loose enough to clear its noise.
REST_MAX_DPS = 2.0
REST_MAX_MS = 0.02


class BoardStream:
    """The driver board's `T:1001` stream, accumulated into running totals.

    Reading only. What the numbers mean -- the resting bias, the two scale
    factors, whether the gyro agrees with the scan match -- lives in
    `odometry.Odometry`, which this hands raw totals to. That split is why the
    same interpretation serves this script and the daemon: on the rover proper
    `rover_daemon.py` owns `/dev/ttyAMA0` and does this accumulation in its own
    reader thread, and only one process may have the port. The shape of what
    crosses the boundary -- `motion()` returning those totals -- is deliberately
    the same in both, and is the only thing that has to stay in step.
    """

    def __init__(self, port):
        # timeout=0 so this never blocks the lidar loop; the stream is continuous
        # at ~2.6 kB/s and a read loop here that waited for a quiet moment would
        # wait for ever.
        self.ser = serial.Serial(port, TELEM_BAUD, timeout=0)
        self.buf = bytearray()
        self.lines = 0
        self.volts = None
        self._at = None
        self._gz_lsb_s = 0.0
        self._ticks = None
        self._breaks = 0

    def pump(self):
        """Drain whatever has arrived. Reads in bulk on purpose: this host drops
        audio and wastes a measurable slice of its one core on processes that wake
        up often, so one big read beats many small ones."""
        n = self.ser.in_waiting
        if n:
            self.buf += self.ser.read(n)
        while b"\n" in self.buf:
            line, _, rest = self.buf.partition(b"\n")
            self.buf = bytearray(rest)
            self._consume(line)

    def _consume(self, line):
        try:
            msg = json.loads(line.decode("ascii", "replace").strip())
        except (ValueError, UnicodeDecodeError):
            return
        if msg.get("T") != 1001:
            return
        self.lines += 1
        self.volts = msg.get("v")
        now = time.monotonic()
        previous, self._at = self._at, now
        span = None if previous is None else now - previous
        gz = msg.get("gz")
        if isinstance(gz, (int, float)):
            if span is not None and 0.0 < span <= MAX_SAMPLE_GAP_S:
                self._gz_lsb_s += gz * span
            elif span is not None:
                # A yaw rate multiplied by a gap nothing was awake for is invented
                # rotation. Counted rather than integrated, so a consumer can
                # refuse the span it falls in. See Span.intact.
                self._breaks += 1
        odl, odr = msg.get("odl"), msg.get("odr")
        if isinstance(odl, (int, float)) and isinstance(odr, (int, float)):
            mean = (odl + odr) / 2.0
            if self._ticks is not None and abs(mean - self._ticks) > MAX_TICK_STEP:
                self._breaks += 1        # the board restarted its counters
            self._ticks = mean

    def motion(self):
        if not self.lines:
            return None
        return {"at": self._at, "gz_lsb_s": self._gz_lsb_s, "ticks": self._ticks,
                "samples": self.lines, "breaks": self._breaks}

    def close(self):
        self.ser.close()


def sector_bar(sectors, reach):
    """A crude plan view of clearance, forward in the middle. Reads left-to-right
    like the room does: rover's left on the left."""
    glyphs = " .:-=+*#%@"
    order = list(range(len(sectors) // 2, -1, -1)) + \
            list(range(len(sectors) - 1, len(sectors) // 2, -1))
    out = []
    for i in order:
        frac = min(max(sectors[i] / reach, 0.0), 0.999)
        out.append(glyphs[int(frac * len(glyphs))])
    return "".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lidar", default="/dev/ttyACM0",
                    help="lidar serial port (default: %(default)s)")
    ap.add_argument("--telemetry", metavar="PORT", default=None,
                    help="driver board port for the motion prior, e.g. /dev/ttyAMA0. "
                         "Off by default: rover_daemon.py owns that port and only "
                         "one process can.")
    ap.add_argument("--ticks-per-metre", type=float, default=None,
                    help="encoder scale, overriding odometry.json. Without either, "
                         "the prior carries no translation.")
    ap.add_argument("--gyro-lsb-per-dps", type=float, default=None,
                    help="gyro scale, overriding odometry.json. Without either, "
                         "the prior carries no rotation. Signed: it carries which "
                         "way round a positive gz is.")
    ap.add_argument("--seconds", type=float, default=30.0,
                    help="how long to run, 0 for until interrupted")
    ap.add_argument("--map", metavar="FILE.pgm", default=None,
                    help="write the occupancy grid here on exit")
    ap.add_argument("--report-hz", type=float, default=1.0)
    ap.add_argument("--sectors", type=int, default=0, metavar="N",
                    help="also draw an N-sector clearance bar (try 37)")
    ap.add_argument("--cells", type=int, default=None, help="override grid size")
    ap.add_argument("--resolution", type=float, default=None, help="metres per cell")
    ap.add_argument("--mount-deg", type=float, default=None,
                    help="lidar zero relative to rover forward, ccw (default 90)")
    args = ap.parse_args(argv)

    cfg = default_config()
    if args.cells:
        cfg.grid_cells = args.cells
    if args.resolution:
        cfg.resolution_m = args.resolution
    if args.mount_deg is not None:
        cfg.mount_deg = args.mount_deg

    stopping = []
    signal.signal(signal.SIGINT, lambda *_: stopping.append(True))

    lidar = serial.Serial(args.lidar, LIDAR_BAUD, timeout=0.05)
    telem = odo = None
    if args.telemetry:
        try:
            telem = BoardStream(args.telemetry)
        except serial.SerialException as e:
            print(f"telemetry port unavailable ({e}); continuing on lidar alone",
                  file=sys.stderr)
        else:
            odo = Odometry(telem)
            if args.gyro_lsb_per_dps:
                odo.gyro_lsb_per_dps = args.gyro_lsb_per_dps
            if args.ticks_per_metre:
                odo.ticks_per_metre = args.ticks_per_metre
            if not (odo.gyro_lsb_per_dps or odo.ticks_per_metre):
                print("telemetry open and no scale factor measured yet, so the "
                      "prior stays zero -- the gyro is still read, and still says "
                      "whether it agrees the rover turned", file=sys.stderr)

    print(f"lidar {args.lidar} at {LIDAR_BAUD}, map "
          f"{cfg.grid_cells}x{cfg.grid_cells} at {cfg.resolution_m*100:.0f} cm "
          f"({cfg.grid_cells*cfg.resolution_m:.0f} m across), mount {cfg.mount_deg:.0f} deg")

    started = time.monotonic()
    deadline = started + args.seconds if args.seconds > 0 else float("inf")
    next_report = started + 1.0 / args.report_hz
    dropped = processed = rejects = disagreements = 0
    worst_loop = 0.0
    bytes_in = 0

    with Slam2D(cfg) as slam:
        while not stopping and time.monotonic() < deadline:
            loop_t0 = time.monotonic()

            n = lidar.in_waiting
            chunk = lidar.read(n if n > 0 else 1)
            bytes_in += len(chunk)
            if telem:
                telem.pump()

            if chunk:
                revs = slam.feed(chunk)
                if revs:
                    # More than one means the loop fell a whole revolution behind and
                    # slam2d kept only the newest, which is the right choice but has
                    # to be visible rather than silent.
                    dropped += revs - 1
                    span = None
                    if odo:
                        # Before the match, because centring the search window is
                        # the whole of what a prior does.
                        span = odo.span()
                        slam.set_prior(*odo.prior(span))
                    was = slam.pose
                    if slam.update():
                        processed += 1
                        rejects += slam.rejected
                        if odo:
                            moved = slam.pose
                            dth = (moved[2] - was[2] + math.pi) % (2*math.pi) - math.pi
                            step = math.hypot(moved[0]-was[0], moved[1]-was[1])
                            # Standing still is where the gyro's zero and the
                            # spread around it are measured, and it is what this
                            # script mostly does. Judged by the matcher, since
                            # nothing here knows whether something else is
                            # driving the rover.
                            if (abs(math.degrees(dth)) < REST_MAX_DPS * 0.2
                                    and step < REST_MAX_MS * 0.2):
                                odo.learn_rest(span)
                            why = odo.disagreement(span, dth)
                            if why:
                                disagreements += 1
                                print(f"  !! {why}")

            worst_loop = max(worst_loop, time.monotonic() - loop_t0)

            now = time.monotonic()
            if now >= next_report:
                next_report = now + 1.0 / args.report_hz
                x, y, th = slam.pose
                if processed:
                    fwd = slam.sectors(37)[0]
                    line = (f"t={now-started:5.1f}s scan {processed:4d}  "
                            f"x={x:+.3f} y={y:+.3f} th={math.degrees(th):+6.1f}deg  "
                            f"score {slam.score:.2f}  pts {slam.points:3d}  "
                            f"ahead {fwd:5.2f}m  drop {dropped}")
                    if telem and telem.volts:
                        line += f"  {telem.volts/100:.2f}V"
                    print(line)
                    if args.sectors:
                        print("   " + sector_bar(slam.sectors(args.sectors),
                                                 cfg.max_range_m))
                else:
                    print(f"t={now-started:5.1f}s  {bytes_in} bytes in, no complete "
                          f"revolution yet -- if this stays at 0, the rover's power "
                          f"switch is off: the port enumerates without it but the "
                          f"lidar only spins when the 5 V rail is up")

        elapsed = time.monotonic() - started
        print(f"\n{processed} revolutions in {elapsed:.1f}s "
              f"({processed/max(elapsed,1e-9):.1f} Hz), {dropped} dropped, "
              f"{rejects} matches rejected, worst loop {worst_loop*1000:.1f} ms")
        if odo:
            state = odo.status()
            print(f"telemetry: {telem.lines} T:1001 lines "
                  f"({telem.lines/max(elapsed,1e-9):.0f} Hz), gyro bias "
                  f"{state['gyro_bias_lsb']} LSB, resting spread "
                  f"{state['gyro_noise_lsb']} LSB over {state['rest_spans']} spans")
            bar = odo.threshold_lsb()
            print(f"           the gyro will vouch for rotation past "
                  f"{'unmeasured' if bar is None else f'{bar:.1f} LSB'}; "
                  f"scale factors gyro={state['gyro_lsb_per_dps']} "
                  f"ticks={state['ticks_per_metre']}")
            print(f"           {disagreements} revolutions where the gyro and the "
                  f"scan match could not both be right")
        x, y, th = slam.pose
        print(f"final pose x={x:+.3f} m y={y:+.3f} m heading {math.degrees(th):+.1f} deg")

        if args.map:
            print(f"wrote {slam.write_pgm(args.map)}")

    lidar.close()
    if telem:
        telem.close()
    return 0 if processed else 1


if __name__ == "__main__":
    sys.exit(main())
