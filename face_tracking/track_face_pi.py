"""Track a face from the rover itself: the board holds the loop and does the seeing.

The rover's own machine is the only one wired to the ESP32, so the control loop
belongs here whatever else is true. What has changed is whether it can also do the
detecting. On a Pi 1 -- one ARM1176 at 700 MHz with no NEON -- it could not, and
the picture had to go out to a detector and the boxes come back: first to YuNet on
MEDIA over the network, then to the Myriad X inside the OAK camera over USB.

Every host since has had cores enough to beat both, and YuNet here is faster than
either of those was -- 146 ms a frame against 190 through the OAK's loopback
service, measured on the Banana Pi M4 Zero that followed the Pi 1, and quicker
again on the Jetson Orin Nano the rover has carried since 2026-08-31. So
`--service local` runs it in this process (yunet.py) and that is the default; a
host:port still POSTs to `face_detect/` on MEDIA, which is the same protocol as
ever and why neither this script nor the daemon had to change shape when the
detector moved.

    python3 track_face_pi.py                          # camera, YuNet here, the UART
    python3 track_face_pi.py --service 192.168.1.3:8768   # or a detector elsewhere
    python3 track_face_pi.py --host 192.168.1.22      # command the board over WiFi
    python3 track_face_pi.py --no-move                # detect and report, command nothing
    python3 track_face_pi.py --no-scan                # stay put when there is nobody about

Headless, unlike track_face.py: this host has no display by choice, so there is a
status line rather than a window. The aiming, the thresholds and the sweep are the
same in both -- they come from aiming.py, which exists so that these two scripts
cannot drift into being two different robots.

**Whether this decodes a picture depends on where the detector is.** Against a
service it does not: the camera is asked for MJPEG and those exact bytes are
forwarded unopened. Against `--service local` the decode happens here, which on
the Banana Pi was 7 ms and on the Pi 1 was 93 -- the whole reason the choice used
to matter. The figures below were measured on the Pi 1 and are kept because they
are what forwarding costs, and because a slow host is a thing this repository
expects to meet again.

MEASURED on the Pi 1 rover, over WiFi (an RTL8188FTV dongle on wlan0):

    what                                    320x240    640x480    1280x720
    exposure to a whole frame in hand        39 ms      41 ms      43 ms
    the picture to MEDIA, raw TCP             4 ms       9 ms      20 ms
    sustained rate, forwarded                30.0 fps   30.0 fps   29.8 fps
    of the Pi's one core                      ~15%       ~30%       52%

640x480 is the default, and the reason is the last row: 720p costs half the
machine and buys nothing, because the detector works at DETECT_WIDTH = 640 and
throws the rest away. The link is not the constraint either way -- the WiFi
ceiling here measured 38 Mbit/s, against 8.4 for this stream.

The rest of the round trip, measured from the Pi against the service on MEDIA:
**22.6 ms** for a 35 kB POST, of which about 6 is the detection and 13 this
machine's own HTTP stack -- a 700 MHz ARM11 is not fast at anything, including
sockets. So a box was in hand roughly 65 ms after the light that made it, against
the 266 ms of dead time measured for track_face.py with the camera on the
workstation's own USB. Moving the camera onto the rover made the loop faster, not
slower, and the command path is now a wire rather than a radio.

**Against the OAK on that Pi it was slower than that, and the reason was JPEG.**
Measured 2026-08-18, one frame end to end over loopback: 85 ms to decode a 640x480
MJPEG frame, 41 ms for the detection itself, and 41 ms of that machine's HTTP
stack, for about 190 ms a frame -- and with the scan matcher also running, the
tracking loop settled at **2.3 fps**. The inference was the cheapest part of it,
and what cost was that the picture had to be decoded on the rover instead of being
handed to a 5700G.

**On the Banana Pi none of that arithmetic survives, and the detector came home.**
Measured 2026-08-23 through the daemon, which runs the same aiming and the same
thresholds: 7 ms to decode the frame, 146 ms for YuNet on three of the four cores,
and no HTTP at all, for **6.6 frames a second** with the scan matcher running
beside it. The OAK is faster at nothing here -- its own inference measured 89 ms
against YuNet's 146, but the loopback POST that carried the frame to it cost 100 ms
of the 190 -- so the camera has stopped being a detector and become what it is
built to be, a depth sensor. See oak_depth/.

That figure was 73 ms until the detector stopped writing its replies in two
sends: see the Nagle note in face_detect/server.py. Worth recording how that hid,
because the wrong answer was reached twice. The service was first reached through
an SSH tunnel, which measured 85 ms, and a tunnel is an easy thing to blame --
but the direct path measured 73, so the tunnel was never more than a few
milliseconds of it. What made the tunnel look guilty was comparing it against a
stub on the Pi's own loopback, which was quick for a reason that had nothing to
do with the network: that stub set disable_nagle_algorithm and the real service
did not. A control that differs from the thing it is controlling for is not a
control. The link itself is 3-4 ms, and always was.

**The dead time is measured per frame, not assumed.** V4L2 stamps every buffer
with the start of its exposure (`ts-monotonic, ts-src-soe`), v4l2-ctl prints that
on stderr, and this pairs it back with the frame it belongs to and sends it along
as an opaque `ts` that the detector echoes. Gimbal.track() is then told exactly
when the picture it is answering was taken, rather than assuming DEAD_TIME_S --
a constant that was measured once and varied by +-50 ms while being measured.
Nothing has to agree about clocks: the stamp only ever means something here.

**Frames are dropped on purpose.** The camera runs at 30 fps and a round trip
takes longer than a frame, so a queue would form -- and a queue here is not
slowness, it is a rover aiming at where somebody was a second ago, which looks
exactly like the divergence aiming.py's dead time compensation exists to prevent.
So the reader keeps only the newest complete frame and the loop sends one at a
time, waiting for the boxes before sending the next. The rate falls out of the
round trip and the staleness cannot accumulate. Dropped frames are counted and
shown, because a silently decimated stream would look identical to a healthy one.

**If the detector goes away, the rover stops.** That was written when the
detector was a desktop that reboots, and it still holds now that it is a service
on this same Pi: it drives a USB device that can be unplugged or brown out, and it
spends about 6 s uploading firmware and a graph before it answers again after a
restart. LOST_GRACE_S covers a blink, not that. After SERVICE_GRACE_S of failures
the camera is centred and left alone
rather than sweeping blind, and the loop keeps retrying quietly until the service
answers again.
"""

import argparse
import json
import os
import signal
import socket
import sys
import time

# `snapshot` and `split_jpegs` are re-exported: the daemon and its checks import
# them from here, which is where they were when those callers were written. The
# three exposure names join them for the checks' sake and for the same reason.
from uvc_camera import (           # noqa: F401
    Camera, DEFAULT_DEVICE, DEFAULT_SIZE, brightness, nothing_in_it,
    restore_automatic, snapshot, split_jpegs, too_dark_for_this_camera,
    under_manual_control,
)
from aiming import (
    DETECT_WIDTH, GAIN, KEEP_SCORE, MAX_DT, SCAN_AFTER_S, SCAN_RATE, Gimbal, Scan,
    Target, clamp, scan_rate_for,
)

# --- the camera -----------------------------------------------------------


# --- the detector ---------------------------------------------------------

# "local" is YuNet in this process; anything else is host:port and is POSTed to.
# When it is an address, give an address rather than a name. The rover is reached
# by name because it has two addresses and which one is live varies; MEDIA has one
# fixed address, so a name buys no agility there and costs mDNS. Measured from the
# rover, three lookups of `media.local` in a row: 344 ms, **5193 ms**, 194 ms.
# This sits in a control loop with a 1 s service timeout, so that outlier is a
# stall and a transient resolver failure is a frame nobody looked at.
DEFAULT_SERVICE = "local"  # yunet.py, on this board's own cores


# One round trip is ~15 ms of network and detection. This is long enough that a
# service busy with another client is waited for and short enough that a dead one
# is noticed within a frame or two.
SERVICE_TIMEOUT_S = 1.0


# How long the detector may be unreachable before the rover gives up and centres.
# Longer than a dropped packet, far shorter than a service restart.
SERVICE_GRACE_S = 3.0


# Between retries once it has been declared gone. The service takes seconds to
# come back at best, and a Pi 1 has better things to do than knock every 40 ms.
SERVICE_RETRY_S = 1.0


# --- the board ------------------------------------------------------------

# One name per board for the same three header pins, and only one of them is
# ever present: ttyTHS1 is UART1 on the Jetson Orin Nano's 40-pin header, ttyS4
# is UART4 on the Banana Pi M4 Zero, ttyAMA0 is the Pi 1. Kept in step with
# SERIAL_CANDIDATES in rover_daemon/board_link.py, which this deliberately does
# not import: this script is meant to run on a bare checkout with the daemon
# stopped, which is exactly when there is nothing else to depend on.
SERIAL_CANDIDATES = ("/dev/ttyTHS1", "/dev/ttyS4", "/dev/ttyAMA0")


DEFAULT_SERIAL = next((p for p in SERIAL_CANDIDATES if os.path.exists(p)),
                      SERIAL_CANDIDATES[0])


BAUD = 115200


SERIAL_CONSOLE_HINT = ("Is a getty still on it? `systemctl status serial-getty@"
                       + DEFAULT_SERIAL.rsplit("/", 1)[-1] + "`")


STATUS_HZ = 5


class Stopping(Exception):
    """SIGTERM arrived. Raised in the main thread so the `finally` block runs."""


def stop_on_sigterm():
    """Make `kill`, `timeout` and systemd stop this the way Ctrl-C does.

    Not a nicety. The camera's angles are a model kept true by centring on the
    way out, so a run that is terminated rather than interrupted leaves the servos
    somewhere the next run will not know about -- and the next run's first
    correction is then wrong by however far they were left. See aiming.Gimbal.
    """
    def handler(signum, frame):
        raise Stopping()

    signal.signal(signal.SIGTERM, handler)
    signal.signal(signal.SIGHUP, handler)


class Detector:
    """The face detector, over one kept-open connection.

    Strictly one request at a time, which is the backpressure: the next frame is
    not sent until this one's boxes are back, so nothing can queue anywhere and
    the loop rate is simply whatever the round trip allows.

    A failed request is reported rather than raised. The service being away is an
    expected state on a rover -- see the docstring -- and the loop handles it by
    stopping, not by dying.
    """

    def __init__(self, address, score=KEEP_SCORE, width=DETECT_WIDTH,
                 timeout=SERVICE_TIMEOUT_S):
        import http.client

        self._client = http.client
        host, _, port = address.partition(":")
        self.host = host
        self.port = int(port) if port else 8768
        self.timeout = timeout
        self.query = f"?score={score}&width={width}&ts="
        self.connection = None
        self.rtt_ms = 0.0
        self.detect_ms = 0.0

    def describe(self):
        return f"http://{self.host}:{self.port}/detect"

    def detect(self, jpeg, exposed_at):
        """Faces for this frame, or None if the service did not answer.

        The exposure time goes out as an opaque string and comes back untouched.
        It is checked on return: a reply carrying somebody else's stamp would be
        a reply to a different frame, which the controller must not act on.
        """
        stamp = repr(exposed_at)
        for attempt in (1, 2):  # a stale keep-alive costs one retry, not a frame
            started = time.monotonic()
            try:
                if self.connection is None:
                    self.connection = self._client.HTTPConnection(
                        self.host, self.port, timeout=self.timeout)
                    # Connecting here rather than letting request() do it, so the
                    # socket exists to be configured -- and inside the try, because
                    # a detector that is simply not there must come back as None
                    # like any other failure. It is the expected state on a rover.
                    self.connection.connect()
                    # http.client writes the headers and then the body, so Nagle
                    # can hold the second until the first is acknowledged -- the
                    # classic 40 ms. Measured, it changes nothing here; the stall
                    # that mattered was the same mistake on the server's side.
                    self.connection.sock.setsockopt(
                        socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.connection.request(
                    "POST", "/detect" + self.query + stamp, body=jpeg,
                    headers={"Content-Type": "image/jpeg",
                             "Content-Length": str(len(jpeg))})
                payload = json.loads(self.connection.getresponse().read())
            except Exception:
                self.close()
                if attempt == 2:
                    return None
                continue
            if payload.get("ts") != stamp or "faces" not in payload:
                return None
            self.rtt_ms = (time.monotonic() - started) * 1e3
            self.detect_ms = payload.get("detect_ms", 0.0)
            return [tuple(face) for face in payload["faces"]]
        return None

    def close(self):
        if self.connection is not None:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None


class SerialLink:
    """JSON commands down the GPIO UART to the ESP32.

    The wire the whole arrangement is built around: the detector may be a room
    away, but the servos are commanded over a cable that cannot drop out.
    """

    def __init__(self, port):
        import serial

        self.port = port
        self.link = serial.Serial(port, BAUD, timeout=0.1)

    def describe(self):
        return f"{self.port} at {BAUD}"

    def send(self, command):
        try:
            self.link.write(json.dumps(command, separators=(",", ":")).encode() + b"\n")
            # The board streams T:1001 telemetry continuously and nothing here
            # reads it; left alone it would fill the buffer within seconds.
            self.link.reset_input_buffer()
            return True
        except Exception:
            return False

    def close(self):
        try:
            self.link.close()
        except Exception:
            pass


class HttpLink:
    """JSON commands over the ESP32's own `/js` endpoint, for a board on WiFi."""

    def __init__(self, host, timeout=0.5):
        import http.client
        from urllib.parse import quote

        self._client = http.client
        self._quote = quote
        self.host = host
        self.timeout = timeout
        self.connection = None

    def describe(self):
        return f"http://{self.host}/js"

    def send(self, command):
        path = "/js?json=" + self._quote(
            json.dumps(command, separators=(",", ":")), safe="")
        for attempt in (1, 2):
            if self.connection is None:
                self.connection = self._client.HTTPConnection(
                    self.host, timeout=self.timeout)
            try:
                self.connection.request("GET", path)
                self.connection.getresponse().read()
                return True
            except Exception:
                self.close()
                if attempt == 2:
                    return False
        return False

    def close(self):
        if self.connection is not None:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None


class NoLink:
    """--no-move: everything runs, nothing is commanded."""

    def describe(self):
        return "nothing (--no-move)"

    def send(self, command):
        return True

    def close(self):
        pass


def open_link(args):
    if not args.move:
        return NoLink()
    if args.host:
        return HttpLink(args.host)
    try:
        return SerialLink(args.serial)
    except Exception as error:
        sys.exit(f"Cannot open {args.serial}: {error}\n{SERIAL_CONSOLE_HINT}")


def parse_size(text):
    try:
        width, _, height = text.lower().partition("x")
        return int(width), int(height)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a size: {text!r} (try 640x480)")


def main():
    parser = argparse.ArgumentParser(
        description="Track a face from the rover, detecting on its own cores.")
    parser.add_argument(
        "--service", default=DEFAULT_SERVICE, metavar="HOST[:PORT]",
        help=f"the face detector (default {DEFAULT_SERVICE})")
    parser.add_argument(
        "--serial", default=DEFAULT_SERIAL, metavar="PORT",
        help=f"the ESP32's serial port (default {DEFAULT_SERIAL})")
    parser.add_argument(
        "--host", default=None, metavar="ADDRESS",
        help="command the board over WiFi at this address instead of the UART")
    parser.add_argument(
        "--device", default=DEFAULT_DEVICE, metavar="PATH",
        help=f"the camera (default {DEFAULT_DEVICE})")
    parser.add_argument(
        "--size", type=parse_size, default=DEFAULT_SIZE, metavar="WxH",
        help="capture size (default %dx%d); larger costs the host and buys nothing"
             % DEFAULT_SIZE)
    parser.add_argument(
        "--no-move", dest="move", action="store_false",
        help="detect and report, but command nothing")
    parser.add_argument(
        "--no-scan", dest="scan", action="store_false",
        help="stay put when there is no face, instead of sweeping to look for one")
    parser.add_argument(
        "--scan-rate", type=float, default=SCAN_RATE, metavar="DEG",
        help=f"sweep speed in degrees per second (default {SCAN_RATE})")
    parser.add_argument(
        "--gain", type=float, default=GAIN, metavar="G",
        help=f"fraction of the error corrected per frame (default {GAIN})")
    parser.add_argument(
        "--quiet", action="store_true", help="no status line")
    args = parser.parse_args()

    if not os.path.exists(args.device):
        sys.exit(f"No camera at {args.device}.")
    stop_on_sigterm()
    camera = Camera(args.device, args.size)
    width, height = args.size
    # The same choice the daemon makes in `rover_camera._open_detector`, and made
    # the same way for the same reason: two ways of picking a detector would be
    # two robots that see differently.
    if args.service == "local":
        from yunet import KEEP_SCORE, LocalDetector

        detector = LocalDetector(score=KEEP_SCORE, size=args.size)
    else:
        detector = Detector(args.service)
    link = open_link(args)
    print(f"camera {args.device} {width}x{height} MJPG -> {detector.describe()}, "
          f"commanding {link.describe()}; Ctrl-C to stop")

    gimbal = Gimbal(clamp(args.gain, 0.05, 1.0), args.size)
    # The one thing that moves before a face is seen: the angles are a model, and
    # this is what makes the model true.
    if not link.send(gimbal.command()):
        camera.close()
        link.close()
        sys.exit(f"No answer from the driver board on {link.describe()}. Is it powered?")
    gimbal.changed()  # the centring above counts as sent; do not repeat it

    target = Target()
    scan = None
    scan_rate = clamp(args.scan_rate, 1.0, 200.0)
    failures = 0
    frames = 0
    rate = 0.0
    lag_ms = 0.0
    last_tick = time.monotonic()
    service_ok_at = time.monotonic()
    stalled = False  # the detector has been gone long enough to give up on
    last_status = 0.0

    try:
        while True:
            got = camera.latest()
            now = time.monotonic()
            if got is None:
                if not camera.alive():
                    why = "; ".join(camera.complaints) or "it stopped without saying why"
                    print(f"\nno frames from {args.device} -- {why}", file=sys.stderr)
                    if any("busy" in line.lower() for line in camera.complaints):
                        print("Something else has the camera open -- find it with "
                              "`fuser -v /dev/video0`. A v4l2-ctl left behind by a "
                              "killed run is the usual one.", file=sys.stderr)
                    break
                continue
            frame, exposed_at = got
            # Clamped, not raw: a frame that took a second to arrive must not be
            # answered with a second's worth of sweep. See MAX_DT.
            dt, last_tick = min(now - last_tick, MAX_DT), now

            faces = detector.detect(frame, exposed_at)
            if faces is None:
                # The detector did not answer. Not a frame without faces in it --
                # a frame nobody looked at -- so the target is left exactly as it
                # was and the grace period decides what happens next.
                if now - service_ok_at > SERVICE_GRACE_S:
                    if not stalled:
                        print(f"\nno answer from {detector.describe()} for "
                              f"{SERVICE_GRACE_S:.0f}s -- centring and waiting",
                              file=sys.stderr)
                        target.drop()
                        scan = None
                        gimbal.centre()
                        gimbal.changed()
                        link.send(gimbal.command())
                        stalled = True
                    time.sleep(SERVICE_RETRY_S)
                continue
            if stalled:
                print(f"\n{detector.describe()} is answering again", file=sys.stderr)
                stalled = False
                last_tick = now
                dt = 0.0
            service_ok_at = now
            lag_ms = (now - exposed_at) * 1e3

            tracking = target.update(faces, now)

            scanning = False
            if tracking and not target.fresh:
                # Holding the lock on grace, with nothing detected this frame.
                # The angle is still known; the pixel is not -- see
                # Gimbal.keep_going(), which is also why this is not a track().
                scan = None
                gimbal.keep_going(dt)
            elif tracking:
                # A face again: the sweep is abandoned, and the next one will be
                # built afresh from wherever tracking has left the camera pointing.
                scan = None
                # Positive x is right of centre and positive y is *above* it, which
                # is not the picture's own row order -- see Gimbal.track().
                error_x = (target.centre[0] - width / 2) / (width / 2)
                error_y = (height / 2 - target.centre[1]) / (height / 2)
                # The measured exposure time, not DEAD_TIME_S: this is the whole
                # reason the stamp is carried out to the detector and back.
                gimbal.track(error_x, error_y, dt, now, exposed_at=exposed_at)
            else:
                if target.centre is not None:
                    target.drop()
                    gimbal.forget()
                if args.scan and now - target.seen_at > SCAN_AFTER_S:
                    if scan is None:
                        scan = Scan(gimbal)
                    scan.step(gimbal, scan_rate_for(dt, gimbal.pan_gain, scan_rate), dt)
                    scanning = True

            # Every frame, moved or not: track() reads this back to find where the
            # camera was when a frame was exposed.
            gimbal.record(now)

            if gimbal.changed():
                failures = 0 if link.send(gimbal.command()) else failures + 1

            frames += 1
            rate = rate + 0.1 * (1.0 / max(dt, 1e-3) - rate) if frames > 1 else 0.0

            if not args.quiet and now - last_status > 1.0 / STATUS_HZ:
                last_status = now
                state = ("tracking" if tracking else
                         scan.state() if scanning else "no face")
                offset = ""
                if tracking:
                    offset = (f"  err {error_x:+.2f},{error_y:+.2f}")
                print(f"\r {state:<20} faces {len(faces)}{offset}   "
                      f"pan {gimbal.pan:+4.0f} tilt {gimbal.tilt:+3.0f}   "
                      f"{rate:4.1f} fps  lag {lag_ms:3.0f} ms  "
                      f"(rtt {detector.rtt_ms:3.0f}, det {detector.detect_ms:4.1f})  "
                      f"dropped {camera.dropped}"
                      + ("" if not failures else f"  link {failures} lost")
                      + "   ", end="", flush=True)
    except (KeyboardInterrupt, Stopping):
        pass
    finally:
        print()
        # Back to centre, which is where the next run will assume it is. Nothing
        # else needs undoing: the wheels were never touched and the heartbeat was
        # left at the firmware's own default throughout -- T:134 does not feed it.
        gimbal.centre()
        link.send(gimbal.command())
        link.close()
        detector.close()
        camera.close()
        print(f"centred, stopped -- {frames} frames tracked, {camera.dropped} dropped, "
              f"{camera.unpaired} without a V4L2 stamp")
    return 0


if __name__ == "__main__":
    sys.exit(main())
