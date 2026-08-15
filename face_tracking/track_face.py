"""Find a face on the rover's camera and keep the pan/tilt aimed at it.

Two components at once, which makes this the first host-side script in the suite
that is not a single-component instrument: the rover's USB camera module supplies
the picture and the ESP32's two ST3215 servos are steered to keep the face in the
middle of it. Nothing else on the rover is involved -- no Raspberry Pi, no ROS,
and no OAK camera, which is not a UVC device and is not the one on the gimbal.

    python track_face.py                      # over WiFi: the rover's AP, else this LAN
    python track_face.py --host 192.168.1.22  # straight to a known address
    python track_face.py --serial             # over USB, port auto-detected
    python track_face.py --no-move            # detect and draw, command nothing
    python track_face.py --no-scan            # stay put when there is nobody about

With nobody in shot it sweeps from side to side looking for someone -- pan end to
end and back, held at the one height faces are found at rather than rastering over
a range that is mostly floor and ceiling -- and switches to following the moment a
face appears anywhere in the frame. How fast it may sweep and still see anything is
a measured question, not a matter of taste; SCAN_RATE carries the numbers, and
SCAN_TILT the height.

**It never drives the wheels.** The only command it sends that moves anything is
CMD_GIMBAL_CTRL_SIMPLE (`{"T":133,...}`), which reaches the two camera servos and
nothing else. It also leaves the firmware's heartbeat alone, deliberately: the
heartbeat exists to stop the *base* when commands stop arriving, and `T:133` does
not feed it, so setting it short here would achieve nothing but a stream of stop
commands to motors that were never started.

The detector is YuNet, OpenCV's own small CNN face detector. Measured here it is
5.8 ms of a 33 ms frame, the rest being spent waiting on the camera: the loop runs
at the camera's 30 fps and detection is not remotely what limits it, which is why
the frame is not decimated or the detector run every other pass. Its model is a
230 kB ONNX file that OpenCV does not ship; the first run fetches it to sit beside
this script, and `--model` points at one already downloaded. Haar cascades would
need no file, but OpenCV 5 dropped them from the wheel -- `cv2.data.haarcascades`
is an empty directory here -- so a file has to come from somewhere regardless.

Control is closed through the world -- the camera is on the thing being aimed, so
every correction changes the next measurement -- but open around the servos, which
report nothing back. The consequences of that, and every constant they produced,
now live in **aiming.py**, which this imports: the angles are a model rather than
a reading, the loop has a dead time worth eight frames, and Gimbal.track() answers
the error from where the camera was when the frame was exposed rather than from
where it has since been sent. That file carries the measurements and the recipes
for re-taking them; nothing here needs touching when the lens or the servo horns
change.

The split happened when the detector moved onto the MEDIA host: track_face_pi.py
runs the same control law on the rover, with no OpenCV to import, and two copies
of these numbers would be two different robots. What is left in this file is the
half that is specific to running on the workstation -- a DirectShow camera, YuNet
in-process, a window to watch it in, and a link to the board over WiFi or USB.
"""

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request

import cv2

from aiming import (
    DEADBAND, DETECT_WIDTH, GAIN, KEEP_SCORE, MAX_DT, NMS_THRESHOLD,
    SCAN_AFTER_S, SCAN_RATE, Gimbal, Scan, Target, clamp,
)

# --- the detector ---------------------------------------------------------

MODEL_FILE = "face_detection_yunet.onnx"
MODEL_URL = (
    "https://github.com/opencv/opencv_zoo/raw/main/models/"
    "face_detection_yunet/face_detection_yunet_2023mar.onnx"
)
MODEL_BYTES = 232589  # what the fetch above should produce; a short file is a bad one

# --- the camera -----------------------------------------------------------

# The rover's own module, a plain UVC device -- not the OAK, which is not UVC.
ROVER_CAMERA_ID = "VID_0ABD&PID_8050"
REQUEST_SIZE = (1280, 720)
# Ask for the size first and MJPG second. That order is not cosmetic: this camera
# offers 1280x720 as MJPG at 30 fps and YUY2 at 10 fps, runs no auto-exposure at
# all in the YUY2 one, and DirectShow picks uncompressed unless asked otherwise.
# Setting FOURCC before the size is silently ignored. See docs/usb-cameras.md --
# a black picture from this camera is nearly always this, not the sensor.
PREFERRED_FOURCC = cv2.VideoWriter_fourcc(*"MJPG")
MAX_PROBE = 8
PROBE_READS = 3
MAX_READ_FAILURES = 30

# --- the link -------------------------------------------------------------

DEFAULT_HOST = "192.168.4.1"  # the ESP32's own AP: SSID "UGV", password "12345678"
BAUD = 115200
PROBE_COMMAND = {"T": 130}
PROBE_REPLY = b'"T":1001'
PROBE_TIMEOUT = 0.4
PROBE_WORKERS = 64

WINDOW = "Face tracking -- rover pan/tilt"


# --- model ----------------------------------------------------------------


def ensure_model(path):
    """The ONNX file, fetched once if it is not already here.

    A network fetch on first run is a poor fit for a suite whose whole point is
    running with nothing else working, so it happens exactly once and lands beside
    the script, where the next run finds it. `--model` skips it entirely for a copy
    that arrived some other way.
    """
    if os.path.exists(path) and os.path.getsize(path) > MODEL_BYTES // 2:
        return path
    print(f"fetching the face detector ({MODEL_BYTES // 1024} kB, once) -> {path}")
    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        # To a temporary name first: an interrupted download that keeps the real
        # name would be loaded as a model on the next run and fail obscurely.
        partial = path + ".part"
        with urllib.request.urlopen(MODEL_URL, timeout=30) as response:
            data = response.read()
        if len(data) < MODEL_BYTES // 2:
            raise ValueError(f"got {len(data)} bytes, expected about {MODEL_BYTES}")
        with open(partial, "wb") as handle:
            handle.write(data)
        os.replace(partial, path)
    except Exception as error:
        sys.exit(
            f"Could not fetch the face detector: {error}\n"
            f"Download it by hand from {MODEL_URL}\n"
            f"and put it at {path}, or name it with --model."
        )
    return path


class Detector:
    """YuNet, run on a reduced copy of the frame.

    Faces come back as (x, y, w, h, score) in full-frame pixels. The reduction is
    where the speed comes from: the detector's cost is set by its input size, and
    a 640-wide copy of a 720p frame costs 14 ms against 40 for the whole thing,
    with no difference to a face big enough to be worth aiming at.
    """

    def __init__(self, model_path, frame_size):
        try:
            # The network runs at the lower bar and Target decides what is worth
            # locking onto: a detection too weak to acquire may still be the face
            # already being followed, and the detector cannot know which is which.
            self.net = cv2.FaceDetectorYN_create(
                model_path, "", (320, 320), KEEP_SCORE, NMS_THRESHOLD, 5000
            )
        except cv2.error as error:
            sys.exit(f"Cannot load the face detector from {model_path}: {error}")
        width, height = frame_size
        self.scale = min(DETECT_WIDTH / width, 1.0)
        self.size = (int(width * self.scale), int(height * self.scale))
        self.net.setInputSize(self.size)

    def detect(self, frame):
        small = cv2.resize(frame, self.size) if self.scale < 1.0 else frame
        _, raw = self.net.detect(small)
        if raw is None:
            return []
        back = 1.0 / self.scale
        faces = []
        for row in raw:
            x, y, w, h = (float(v) * back for v in row[:4])
            faces.append((x, y, w, h, float(row[-1])))
        return faces


# --- camera ---------------------------------------------------------------


def silence_opencv():
    """Probing indices that are not there is noisy, and the warnings are expected."""
    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
    except AttributeError:
        pass


def camera_ids():
    """Device paths for the machine's cameras, in DirectShow's enumeration order.

    Sorted by PNPDeviceID, which reproduces that order: DirectShow builds its list
    from registry keys named after the device path, so they come back
    lexicographically, while Get-CimInstance's own order does not match and puts
    every label one place out. Same trick as usb_cameras/preview_usb_cameras.py.
    """
    if sys.platform != "win32":
        return []
    query = (
        "Get-CimInstance Win32_PnPEntity "
        "-Filter \"PNPClass='Camera' or PNPClass='Image'\" "
        "| Sort-Object PNPDeviceID "
        "| Select-Object -ExpandProperty PNPDeviceID"
    )
    try:
        done = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", query],
            capture_output=True, text=True, timeout=15,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    return [line.strip() for line in done.stdout.splitlines() if line.strip()]


def find_rover_camera():
    """The capture index of the rover's own camera module, or None.

    Windows' list is positional and includes devices that never stream, so this is
    a good guess rather than an identity -- open_camera() checks that whatever it
    picked actually delivers frames, and falls back to a scan if it does not.
    """
    for index, device in enumerate(camera_ids()):
        if ROVER_CAMERA_ID in device.upper():
            return index
    return None


def open_camera(index):
    """Open one camera at the size and format that keep its automatics working."""
    backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQUEST_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQUEST_SIZE[1])
    cap.set(cv2.CAP_PROP_FOURCC, PREFERRED_FOURCC)  # after the size, always
    # Every queued frame is dead time in a loop that is closed through the servos,
    # so ask for the shallowest buffer the driver will give. Best effort: several
    # backends accept the call and keep their own depth.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not any(cap.read()[0] for _ in range(PROBE_READS)):
        cap.release()
        return None
    return cap


def choose_camera(requested):
    """(capture, index): the one asked for, the rover's, or the first that works."""
    if requested is not None:
        cap = open_camera(requested)
        if cap is None:
            sys.exit(f"Camera index {requested} does not open, or delivers no frames.")
        return cap, requested
    rover = find_rover_camera()
    if rover is not None:
        cap = open_camera(rover)
        if cap is not None:
            return cap, rover
        print(f"the rover's camera looked like index {rover}, which will not stream")
    for index in range(MAX_PROBE):
        cap = open_camera(index)
        if cap is not None:
            return cap, index
    sys.exit("No camera found. Plug the rover's camera in, or name one with --camera.")


def fourcc_name(value):
    packed = int(value)
    if packed <= 0:
        return "?"
    name = "".join(chr((packed >> (8 * i)) & 0xFF) for i in range(4))
    return name if name.isprintable() else "?"


# --- link -----------------------------------------------------------------


def js_path(command):
    """A command as the board wants it: JSON in the query string of `/js`."""
    from urllib.parse import quote

    return "/js?json=" + quote(json.dumps(command, separators=(",", ":")), safe="")


class HttpLink:
    """JSON commands over the ESP32's own `/js` endpoint, on one kept-open socket."""

    def __init__(self, host, timeout=0.5):
        import http.client

        self._client = http.client
        self.host = host
        self.timeout = timeout
        self.connection = None

    def describe(self):
        return f"http://{self.host}/js"

    def send(self, command):
        path = js_path(command)
        for attempt in (1, 2):  # a stale keep-alive costs one retry, not a command
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


class SerialLink:
    """JSON commands over the ESP32's Type-C port -- the one *not* labelled LIDAR."""

    def __init__(self, port):
        import serial

        self.port = port
        self.link = serial.Serial(port, BAUD, timeout=0.1)

    def describe(self):
        return f"{self.port} at {BAUD}"

    def send(self, command):
        try:
            self.link.write(json.dumps(command, separators=(",", ":")).encode() + b"\n")
            self.link.reset_input_buffer()  # the board chatters; nothing here reads it
            return True
        except Exception:
            return False

    def close(self):
        try:
            self.link.close()
        except Exception:
            pass


class NoLink:
    """--no-move: everything runs, nothing is commanded."""

    def describe(self):
        return "nothing (--no-move)"

    def send(self, command):
        return True

    def close(self):
        pass


def find_serial_port():
    """The board's port, found by asking each candidate something only it answers.

    Both Type-C ports enumerate alike, so USB identity cannot separate them: the
    ESP32 replies to base feedback with a JSON line, while the lidar port only ever
    streams binary. Ports with no VID are skipped -- those are Bluetooth SPP, and
    merely opening one blocks for as long as Windows spends raising a radio link.
    """
    import serial
    from serial.tools import list_ports

    for port in list_ports.comports():
        if port.vid is None:
            continue
        try:
            with serial.Serial(port.device, BAUD, timeout=0.1) as link:
                link.reset_input_buffer()
                link.write(b'{"T":130}\n')
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    if link.readline().startswith(b"{"):
                        return port.device
        except Exception:
            continue
    return None


def probe_host(host):
    """True if the driver board answers at this address.

    A connection proves nothing -- plenty of things on a home LAN serve port 80 --
    so this reads the reply and insists on the firmware's own feedback line.
    """
    import http.client

    try:
        connection = http.client.HTTPConnection(host, timeout=PROBE_TIMEOUT)
        try:
            connection.request("GET", js_path(PROBE_COMMAND))
            return PROBE_REPLY in connection.getresponse().read()
        finally:
            connection.close()
    except Exception:
        return False


def local_network():
    """Every address on this machine's own /24, minus this machine.

    The interface is chosen by opening a UDP socket towards the rover; nothing is
    sent, but it names the interface rover traffic would leave by, which is the one
    worth sweeping on a machine that also carries VM and VPN adapters.
    """
    import socket

    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((DEFAULT_HOST, 80))
        address = probe.getsockname()[0]
    except OSError:
        return []
    finally:
        probe.close()
    prefix, _, own = address.rpartition(".")
    if not prefix or address.startswith("0."):
        return []
    return [f"{prefix}.{octet}" for octet in range(1, 255) if str(octet) != own]


def find_host():
    """The board's address: its own AP first, then a sweep of this LAN.

    The firmware publishes no mDNS name and sets no DHCP hostname, so a rover that
    has joined a home network is an anonymous lease with nothing to look up. What
    it does have is an answer nothing else gives, so every address is asked for
    base feedback and whoever replies is the rover.
    """
    from concurrent import futures

    if probe_host(DEFAULT_HOST):
        return DEFAULT_HOST
    candidates = local_network()
    if not candidates:
        return None
    print(f"searching {candidates[0].rsplit('.', 1)[0]}.0/24 for the driver board...")
    with futures.ThreadPoolExecutor(PROBE_WORKERS) as pool:
        pending = {pool.submit(probe_host, host): host for host in candidates}
        try:
            for done in futures.as_completed(pending):
                if done.result():
                    host = pending[done]
                    print(f"found it at {host} -- pass --host {host} to skip this")
                    return host
        finally:
            for future in pending:
                future.cancel()
    return None


def open_link(args):
    if not args.move:
        return NoLink()
    if args.serial is None:
        host = args.host or find_host()
        if host is None:
            sys.exit("No driver board found on its own AP or this network. "
                     "Name it, e.g. --host 192.168.1.22")
        return HttpLink(host)
    port = args.serial if args.serial != "auto" else find_serial_port()
    if port is None:
        sys.exit("No driver board found on any serial port. Name it, e.g. --serial COM7")
    try:
        return SerialLink(port)
    except Exception as error:
        sys.exit(f"Cannot open {port}: {error}")


# --- drawing --------------------------------------------------------------


def draw(frame, target, faces, lines, now, held):
    height, width = frame.shape[:2]
    centre = (width // 2, height // 2)
    grey = (140, 140, 140)
    cv2.line(frame, (centre[0] - 18, centre[1]), (centre[0] + 18, centre[1]), grey, 1)
    cv2.line(frame, (centre[0], centre[1] - 18), (centre[0], centre[1] + 18), grey, 1)
    # The deadband, drawn: inside this box nothing is commanded, so a face sitting
    # in it with the servos quiet is the loop working, not the loop stalled.
    cv2.rectangle(
        frame,
        (int(centre[0] - DEADBAND * width / 2), int(centre[1] - DEADBAND * height / 2)),
        (int(centre[0] + DEADBAND * width / 2), int(centre[1] + DEADBAND * height / 2)),
        grey, 1,
    )
    # Everything the detector offered, faintly, with its score: a box drawn thin is
    # something seen and passed over, which is the difference between a detector
    # that found nothing and one whose findings were not good enough to aim at.
    for x, y, w, h, score in faces:
        if target.box is None or (x, y, w, h) != tuple(target.box[:4]):
            weak = (110, 110, 190)
            cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), weak, 1)
            cv2.putText(frame, f"{score:.2f}", (int(x), int(y) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, weak, 1, cv2.LINE_AA)
    if target.box is not None and target.locked(now):
        x, y, w, h, score = target.box
        colour = (90, 200, 255) if held else (80, 255, 120)
        cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), colour, 2)
        cv2.putText(frame, f"{score:.2f}", (int(x), int(y) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2, cv2.LINE_AA)
        if target.centre is not None:
            point = (int(target.centre[0]), int(target.centre[1]))
            cv2.circle(frame, point, 4, colour, -1)
            cv2.line(frame, centre, point, colour, 1)
    annotate(frame, lines)


def annotate(frame, lines):
    # putText stretches glyph advances once thickness exceeds 1, so an outline drawn
    # as a heavier pass drifts right of the text it should bound. Build it from
    # offset copies instead, every pass at thickness 1.
    def put(text, origin, colour):
        cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 1,
                    cv2.LINE_AA)

    for row, text in enumerate(lines):
        x, y = 12, 28 + row * 24
        for dx, dy in ((-2, 0), (2, 0), (0, -2), (0, 2),
                       (-1, -1), (1, -1), (-1, 1), (1, 1)):
            put(text, (x + dx, y + dy), (0, 0, 0))
        put(text, (x, y), (255, 255, 255))


# --- main -----------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Track a face with the rover's pan/tilt camera.")
    parser.add_argument(
        "--host", default=None, metavar="ADDRESS",
        help="the ESP32's address over WiFi; by default its own AP, then this LAN")
    parser.add_argument(
        "--serial", nargs="?", const="auto", default=None, metavar="PORT",
        help="command over USB instead; bare, or with a port such as COM7")
    parser.add_argument(
        "--camera", type=int, default=None, metavar="INDEX",
        help="capture index; by default the rover's own module, found by USB id")
    parser.add_argument(
        "--model", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        MODEL_FILE),
        help="the YuNet ONNX file; fetched beside this script if absent")
    parser.add_argument(
        "--no-move", dest="move", action="store_false",
        help="detect and draw, but command nothing -- check the picture first")
    parser.add_argument(
        "--no-scan", dest="scan", action="store_false",
        help="stay put when there is no face, instead of sweeping to look for one")
    parser.add_argument(
        "--scan-rate", type=float, default=SCAN_RATE, metavar="DEG",
        help=f"sweep speed in degrees per second (default {SCAN_RATE}); "
             "slower in a dim room, where the longer exposure smears the picture")
    parser.add_argument(
        "--gain", type=float, default=GAIN, metavar="G",
        help=f"fraction of the error corrected per frame (default {GAIN})")
    args = parser.parse_args()

    silence_opencv()
    model = ensure_model(args.model)
    cap, index = choose_camera(args.camera)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"camera index {index}: {width}x{height} "
          f"{fourcc_name(cap.get(cv2.CAP_PROP_FOURCC))}")
    detector = Detector(model, (width, height))

    link = open_link(args)
    print(f"commanding {link.describe()}; q quits")

    gimbal = Gimbal(clamp(args.gain, 0.05, 1.0), (width, height))
    # The one thing that moves before a face is seen: the angles are a model, and
    # this is what makes the model true.
    if not link.send(gimbal.command()):
        cap.release()
        link.close()
        sys.exit(f"No answer from the driver board on {link.describe()}. Is it powered?")
    gimbal.changed()  # the centring above counts as sent; do not repeat it

    target = Target()
    scan = None  # built when sweeping starts, from wherever the camera then points
    scan_rate = clamp(args.scan_rate, 1.0, 200.0)
    held = False       # h pauses commanding without stopping detection
    failures = 0
    frames = 0
    rate = 0.0
    last_tick = time.monotonic()
    read_failures = 0

    try:
        while True:
            ok, frame = cap.read()
            now = time.monotonic()
            # Clamped, not raw: a frame that took a second to arrive must not be
            # answered with a second's worth of sweep. See MAX_DT.
            dt, last_tick = min(now - last_tick, MAX_DT), now
            if not ok:
                read_failures += 1
                if read_failures >= MAX_READ_FAILURES:
                    print("\nlost the camera.", file=sys.stderr)
                    break
                continue
            read_failures = 0

            faces = detector.detect(frame)
            tracking = target.update(faces, now)

            scanning = False
            if tracking and not held:
                # A face again: the sweep is abandoned, and the next one will be
                # built afresh from wherever tracking has left the camera pointing.
                scan = None
                # Positive x is right of centre and positive y is *above* it, which
                # is not the picture's own row order -- see Gimbal.track().
                error_x = (target.centre[0] - width / 2) / (width / 2)
                error_y = (height / 2 - target.centre[1]) / (height / 2)
                gimbal.track(error_x, error_y, dt, now)
            elif not tracking:
                if target.centre is not None:
                    target.drop()
                if args.scan and not held and now - target.seen_at > SCAN_AFTER_S:
                    if scan is None:
                        scan = Scan(gimbal)
                    scan.step(gimbal, scan_rate, dt)
                    scanning = True

            # Every frame, moved or not: track() reads this back a dead time
            # later to find where the camera was when a frame was exposed.
            gimbal.record(now)

            if not held and gimbal.changed():
                failures = 0 if link.send(gimbal.command()) else failures + 1

            # Smoothed, because a per-frame figure flickers too fast to read -- and
            # worth reading, since the loop's rate is most of its dead time.
            frames += 1
            rate = rate + 0.1 * (1.0 / max(dt, 1e-3) - rate) if frames > 1 else 0.0

            state = ("holding" if held else
                     "tracking" if tracking else
                     scan.state() if scanning else "no face")
            offset = ""
            if tracking:
                offset = (f"  err {(target.centre[0] - width / 2) / (width / 2):+.2f},"
                          f"{(height / 2 - target.centre[1]) / (height / 2):+.2f}")
            draw(frame, target, faces, [
                f"{state}  faces {len(faces)}{offset}",
                f"pan {gimbal.pan:+4.0f}  tilt {gimbal.tilt:+3.0f}  {rate:4.1f} fps"
                + ("" if not failures else f"  link {failures} lost"),
                "q quit   c centre   space re-target   h hold",
            ], now, held)
            cv2.imshow(WINDOW, frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("c"):
                gimbal.centre()
            elif key == ord(" "):
                target.drop()  # next frame re-locks on the largest face
            elif key == ord("h"):
                held = not held
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        # Back to centre, which is where the next run will assume it is. Nothing
        # else needs undoing: the wheels were never touched and the heartbeat was
        # left at the firmware's own default throughout.
        gimbal.centre()
        link.send(gimbal.command())
        link.close()
        cap.release()
        cv2.destroyAllWindows()
        print("centred, stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
