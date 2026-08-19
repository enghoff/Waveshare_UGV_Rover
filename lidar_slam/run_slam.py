#!/usr/bin/env python3
"""Run slam2d against the rover's real sensors and report what it makes of them.

On the Pi the lidar is /dev/ttyACM0 at 230400 -- the CH343 behind the driver
board's Type-C LIDAR socket, claimed by cdc_acm, so there is no /dev/ttyUSB* to
look for. The driver board's own telemetry is a separate port, /dev/ttyAMA0 at
115200, and it is left alone by default because rover_daemon.py normally owns it
and only one process can.

    ssh rpi 'cd ~/ugv/lidar_slam && ./build.sh && python3 run_slam.py --seconds 30 --map room.pgm'

Lidar-only is the supported path. The telemetry prior is wired up but its two
scale factors have never been measured on this rover, so it stays opt-in; see
--ticks-per-metre below and the README.
"""
import argparse
import json
import math
import signal
import sys
import time

import serial

from slam2d import Slam2D, default_config

LIDAR_BAUD = 230400
TELEM_BAUD = 115200


class Telemetry:
    """The driver board's T:1001 stream, accumulated into a motion prior.

    Both scale factors are arguments rather than constants because neither has been
    measured on this rover: the encoders' counts-per-metre depends on the gearbox
    and wheel, and the gyro's LSB-per-deg/s depends on which full-scale range the
    firmware selected. Guessing either would produce a prior that looks plausible
    and quietly drags the scan match off true, which is worse than no prior at all.
    """

    def __init__(self, port, ticks_per_metre, gyro_lsb_per_dps, bias_samples=34):
        # timeout=0 so this never blocks the lidar loop; the stream is continuous at
        # ~2.6 kB/s and a read loop here that waited for a quiet moment would wait
        # for ever.
        self.ser = serial.Serial(port, TELEM_BAUD, timeout=0)
        self.ticks_per_metre = ticks_per_metre
        self.gyro_lsb_per_dps = gyro_lsb_per_dps
        self.buf = bytearray()
        self.gz_bias = None
        self._bias_acc, self._bias_n, self._bias_want = 0, 0, bias_samples
        self.last_ticks = None
        self.last_t = None
        self.d_forward = 0.0
        self.d_yaw = 0.0
        self.lines = 0
        self.volts = None

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

        gz = msg.get("gz")
        if gz is not None:
            if self.gz_bias is None:
                # The rover has to be still for this first second: an un-biased
                # yaw rate integrates into invented rotation faster than anything
                # else here.
                self._bias_acc += gz
                self._bias_n += 1
                if self._bias_n >= self._bias_want:
                    self.gz_bias = self._bias_acc / self._bias_n
            elif self.last_t is not None and self.gyro_lsb_per_dps:
                dps = (gz - self.gz_bias) / self.gyro_lsb_per_dps
                self.d_yaw += math.radians(dps) * (now - self.last_t)

        odl, odr = msg.get("odl"), msg.get("odr")
        if odl is not None and odr is not None and self.ticks_per_metre:
            mean = (odl + odr) / 2.0
            if self.last_ticks is not None:
                self.d_forward += (mean - self.last_ticks) / self.ticks_per_metre
            self.last_ticks = mean

        self.last_t = now

    def take(self):
        """The prior since the last call, and reset."""
        out = (self.d_forward, self.d_yaw)
        self.d_forward = self.d_yaw = 0.0
        return out

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
                    help="encoder scale. UNMEASURED on this rover -- without it the "
                         "prior carries no translation.")
    ap.add_argument("--gyro-lsb-per-dps", type=float, default=None,
                    help="gyro scale. UNMEASURED on this rover -- without it the "
                         "prior carries no rotation.")
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
    telem = None
    if args.telemetry:
        try:
            telem = Telemetry(args.telemetry, args.ticks_per_metre,
                              args.gyro_lsb_per_dps)
        except serial.SerialException as e:
            print(f"telemetry port unavailable ({e}); continuing on lidar alone",
                  file=sys.stderr)
        else:
            if not (args.ticks_per_metre or args.gyro_lsb_per_dps):
                print("telemetry open but neither scale factor given, so the prior "
                      "stays zero -- reading it only for the battery voltage",
                      file=sys.stderr)

    print(f"lidar {args.lidar} at {LIDAR_BAUD}, map "
          f"{cfg.grid_cells}x{cfg.grid_cells} at {cfg.resolution_m*100:.0f} cm "
          f"({cfg.grid_cells*cfg.resolution_m:.0f} m across), mount {cfg.mount_deg:.0f} deg")

    started = time.monotonic()
    deadline = started + args.seconds if args.seconds > 0 else float("inf")
    next_report = started + 1.0 / args.report_hz
    dropped = processed = rejects = 0
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
                    if telem:
                        slam.set_prior(*telem.take())
                    if slam.update():
                        processed += 1
                        rejects += slam.rejected

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
        if telem:
            print(f"telemetry: {telem.lines} T:1001 lines "
                  f"({telem.lines/max(elapsed,1e-9):.0f} Hz), "
                  f"gyro bias {telem.gz_bias}")
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
