"""Stereo depth from the OAK, on the rover, kept alive from boot. Millimetres out.

    python3 depth_server.py                 # loopback 8770, 320x240 depth at 2 fps
    python3 depth_server.py --bind 0.0.0.0  # ...reachable from the LAN as well
    python3 depth_server.py --fps 10        # video rate, and five times the USB traffic

    GET /health     is the device up, what is it, and how is the depth
    GET /depth      the newest frame as a coarse grid and per-sector ranges
    GET /depth.png  the same frame as a greyscale picture, for a person

**This is what "upload the firmware at boot" means on this camera.** The Myriad X
inside an OAK has no flash: it enumerates in ROM bootloader state as `03e7:2485`,
waits for a host to hand it firmware over USB, and comes back as `03e7:f63b` once
booted. depthai does that upload every time a pipeline opens -- so the firmware
version *is* the depthai version, and there is nothing to flash and walk away
from. Worse, a booted device that stops hearing from its host kills itself on a
1500 ms watchdog. "Bring the OAK up when the rover boots" therefore has exactly
one shape: a process that opens it and stays. This is that process, started from
`crontab` by `run_oak_depth.sh`, and its being alive is the whole of the camera
being awake. That shape was worked out while ruling the OAK off the *old*
rover, and it survived the move to this one unchanged.

The camera used to be the rover's face detector, running an SSD on that same VPU
because a Pi 1 could not run one at all. Every board since has run YuNet faster
than the VPU did -- 146 ms a frame on the Banana Pi's four A53 cores against 190
through the old loopback service, and 24 ms on the Jetson Orin Nano the rover
carries now -- so face detection has moved to the CPU
(`face_tracking/yunet.py`) and the camera is free to do the thing it is actually
built for. Its two mono sensors and 7.5 cm baseline are the only depth sense on
this rover that sees anything above or below the lidar's one horizontal plane.

**Nothing on the rover consumes this yet.** It is served rather than wired into
navigation on purpose: the lidar is what keeps the rover off walls today, the
floor reads as an obstacle to a forward-looking depth camera, and a range that
has not been checked against the room is a poor thing to steer by. Read `/depth`,
look at `/depth.png`, and see [README.md](README.md) for what wiring it in would
have to answer first.
"""

import argparse
import json
import os
import statistics
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent

# depthai is unpacked here by install.sh, for the same reason OpenCV is unpacked
# beside yunet.py: this board's Debian has no pip and `sudo` wants a password no
# deploy script has. An installed copy is preferred, so a desk is unaffected --
# see _import_depthai.
VENDOR = HERE / "vendor"

DEFAULT_PORT = 8770
DEFAULT_BIND = "127.0.0.1"
# Mono at 480p because the OV7251 sensors are native there and 400p is a crop, and
# depth decimated 2x on the device.
#
# **Two frames a second, not ten.** Nothing on this rover consumes /depth yet, and
# a parked rover is not looking at anything new; the camera is here so that a
# range is available when somebody asks for one, which two frames a second
# satisfies with a worst-case staleness of half a second. 320x240 of uint16 at 2
# fps is 300 kB/s on a bus this rover also runs its camera, its wifi dongle and
# its lidar over -- where the old 10 fps default cost 1.5 MB/s and the undecimated
# 640x480 at 15 would cost 9. The link is the thing worth spending carefully: 40
# MB/s is where it saturates, and losing the wifi adapter means losing the way to
# say "stop".
#
# Raise it with --fps when something actually reads this at rate. The rate is
# fixed when the pipeline is built, so changing it is a restart and a fresh
# firmware upload -- there is no way to retune a running device.
DEFAULT_FPS = 2
DECIMATION = 2
# How long a frame may be missing before this process gives up and lets the
# supervisor open the device again from scratch. Nothing else recovers a Myriad
# that has been browned out or unplugged, so exiting *is* the repair. Generous
# enough not to fire on a slow moment: at the default 2 fps this is ten frames.
FRAME_TIMEOUT_S = 5.0
# Depths outside this are not measurements. The 7.5 cm baseline stops overlapping
# below about 20 cm, and beyond 6 m one disparity step is more than a metre.
MIN_MM, MAX_MM = 200, 6000
# What `/depth` reduces a frame to. Eight columns is about 9 degrees each across
# the mono lens, which is the width of a doorway at three metres.
GRID_COLS, GRID_ROWS = 8, 6
# Which rows the per-sector range is taken over: the middle band, because the top
# of the frame is ceiling and the bottom is the floor a metre in front of the
# tracks, and both are things the rover drives happily under and over.
BAND = (0.30, 0.70)
# The near edge of a sector as a percentile of its valid pixels rather than as
# their minimum. A single pixel is noise -- stereo mismatches produce isolated
# very-near readings on textureless walls -- and the fifth percentile of a
# thousand-pixel cell is a surface.
NEAR_PERCENTILE = 5
STAT_WINDOW = 100


class DepthError(RuntimeError):
    """The device is not there, said in a sentence a log can carry."""


def _import_depthai():
    """depthai, installed or unpacked. The version matters -- see README.md."""
    try:
        import depthai

        return depthai
    except ImportError:
        pass
    if VENDOR.is_dir() and str(VENDOR) not in sys.path:
        sys.path.insert(0, str(VENDOR))
        try:
            import depthai

            return depthai
        except ImportError as error:
            raise DepthError(f"depthai is unpacked at {VENDOR} but will not "
                             f"import ({error})") from None
    raise DepthError(f"depthai is not installed on this host; run install.sh to "
                     f"unpack it into {VENDOR}")


class Depth:
    """One device, one pipeline, and the newest frame it has produced.

    The pull runs in its own thread and the HTTP handlers read what it left. That
    is not only for tidiness: an XLink queue that nobody drains backs up on the
    device, so the frames have to be taken whether or not anybody is asking, and
    the rate they arrive at is the device's business rather than a request's.
    """

    def __init__(self, fps: int = DEFAULT_FPS, decimation: int = DECIMATION) -> None:
        self.dai = _import_depthai()
        self.fps = fps
        self.decimation = decimation
        self.frame = None            # newest depth frame, uint16 millimetres
        self.frame_at = 0.0          # time.monotonic() when it arrived
        self.frames = 0
        self.errors = 0
        self.valid = 0.0             # share of pixels with a depth, last frame
        self.rate = 0.0
        self.device_name = ""
        self.usb_speed = ""
        self.hfov_deg = None
        self.baseline_cm = None
        self.started_at = time.monotonic()
        self.last_error = ""
        self._periods: list[float] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()

    # --- the pipeline -------------------------------------------------------

    def _pipeline(self):
        dai = self.dai
        pipeline = dai.Pipeline()

        left = pipeline.create(dai.node.MonoCamera)
        left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        left.setFps(self.fps)

        right = pipeline.create(dai.node.MonoCamera)
        right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        right.setFps(self.fps)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Left-right check throws away disparities the two views disagree about,
        # which is most of what would otherwise read as a near obstacle on a bare
        # wall. It also fixes which camera the depth is aligned to.
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        config = stereo.initialConfig.get()
        config.postProcessing.decimationFilter.decimationFactor = self.decimation
        # Median over a small window: cheap on the device, and it is the filter
        # that removes single-pixel disparity errors rather than smoothing edges.
        config.postProcessing.median = \
            self.dai.MedianFilter.KERNEL_5x5
        stereo.initialConfig.set(config)
        left.out.link(stereo.left)
        right.out.link(stereo.right)

        out = pipeline.create(dai.node.XLinkOut)
        out.setStreamName("depth")
        stereo.depth.link(out.input)
        return pipeline

    def run(self) -> None:
        """Open the device and pull frames until something goes wrong.

        Returns when it does, having recorded why. The caller exits and the
        supervisor opens it again -- see FRAME_TIMEOUT_S.
        """
        dai = self.dai
        try:
            # USB2, always. depthai asks for USB3 by default and uploads the
            # USB3-enabled firmware, which on this camera's link usually fails to
            # come back on the bus after boot -- 5 opens in 13 against 13 in 13,
            # measured. See docs/oak-usb-link.md.
            with dai.Device(self._pipeline(), maxUsbSpeed=dai.UsbSpeed.HIGH) as device:
                self._describe(device)
                queue = device.getOutputQueue("depth", maxSize=4, blocking=False)
                while not self._stop.is_set():
                    packet = queue.get()
                    now = time.monotonic()
                    depth = packet.getFrame()
                    with self._lock:
                        if self.frame_at:
                            self._periods.append(now - self.frame_at)
                            del self._periods[:-STAT_WINDOW]
                            if self._periods:
                                self.rate = 1.0 / statistics.fmean(self._periods)
                        self.frame, self.frame_at = depth, now
                        self.frames += 1
                        self.valid = float((depth != 0).mean())
        except Exception as error:
            self.errors += 1
            self.last_error = f"{type(error).__name__}: {error}"
            raise DepthError(self.last_error) from None

    def _describe(self, device) -> None:
        """What this device is, asked of it rather than remembered.

        The horizontal field of view comes off the stored calibration because it
        is the number `/depth`'s sector angles are made of, and a lens described
        twice is two lenses that will disagree -- the mistake `aiming.LENS` exists
        to have stopped making.
        """
        dai = self.dai
        self.device_name = device.getDeviceName()
        self.usb_speed = device.getUsbSpeed().name
        try:
            calibration = device.readCalibration()
            self.hfov_deg = round(calibration.getFov(dai.CameraBoardSocket.CAM_C), 1)
            self.baseline_cm = round(calibration.getBaselineDistance(), 2)
        except Exception as error:      # a device with no stored calibration
            self.last_error = f"calibration unreadable: {error}"

    def stop(self) -> None:
        self._stop.set()

    # --- what it has seen ---------------------------------------------------

    def newest(self):
        with self._lock:
            if self.frame is None:
                return None, 0.0
            return self.frame, time.monotonic() - self.frame_at

    def health(self) -> dict:
        frame, age = self.newest()
        return {
            "ok": frame is not None and age < FRAME_TIMEOUT_S,
            "device": self.device_name,
            "usb": self.usb_speed,
            # The firmware version and the library version are the same number on
            # this camera, because the host uploads the firmware out of the wheel
            # on every open. See README.md.
            "depthai": getattr(self.dai, "__version__", "?"),
            "hfov_deg": self.hfov_deg,
            "baseline_cm": self.baseline_cm,
            "size": list(reversed(frame.shape)) if frame is not None else None,
            "fps": round(self.rate, 1),
            "frames": self.frames,
            "valid": round(self.valid, 3),
            "age_s": round(age, 2) if frame is not None else None,
            "uptime_s": round(time.monotonic() - self.started_at, 1),
            "errors": self.errors,
            "last_error": self.last_error,
        }

    def summary(self) -> dict:
        """The newest frame as a grid and as per-sector ranges.

        Computed here rather than as each frame arrives, because ten frames a
        second of numpy on a rover that is also running SLAM is a cost nobody has
        asked for -- and this way the answer is always about the frame in hand.
        """
        import numpy

        frame, age = self.newest()
        if frame is None:
            return {"ok": False, "error": "no depth frame yet"}
        height, width = frame.shape
        valid = (frame >= MIN_MM) & (frame <= MAX_MM)

        def near(cell_depth, cell_valid):
            values = cell_depth[cell_valid]
            if values.size < 8:        # too few pixels to be a surface
                return None
            return int(numpy.percentile(values, NEAR_PERCENTILE))

        columns = [(c * width // GRID_COLS, (c + 1) * width // GRID_COLS)
                   for c in range(GRID_COLS)]
        grid = []
        for row in range(GRID_ROWS):
            y0, y1 = row * height // GRID_ROWS, (row + 1) * height // GRID_ROWS
            grid.append([near(frame[y0:y1, x0:x1], valid[y0:y1, x0:x1])
                         for x0, x1 in columns])

        band = slice(int(height * BAND[0]), int(height * BAND[1]))
        sectors = []
        span = self.hfov_deg or 0.0
        for x0, x1 in columns:
            cell, mask = frame[band, x0:x1], valid[band, x0:x1]
            # Positive is to the right of the camera's axis, which is the same
            # sign convention aiming.py uses for a face in the picture.
            sectors.append({
                "from_deg": round(span * (x0 / width - 0.5), 1) if span else None,
                "to_deg": round(span * (x1 / width - 0.5), 1) if span else None,
                "near_mm": near(cell, mask),
                "valid": round(float(mask.mean()), 3),
            })
        nearest = [s["near_mm"] for s in sectors if s["near_mm"] is not None]
        return {
            "ok": True,
            "age_s": round(age, 2),
            "size": [width, height],
            "valid": round(float(valid.mean()), 3),
            "near_mm": min(nearest) if nearest else None,
            "band": [round(BAND[0], 2), round(BAND[1], 2)],
            "sectors": sectors,
            # Row 0 is the top of the picture. Nulls are cells with too few valid
            # pixels to call anything, which is not the same as nothing being there.
            "grid_mm": grid,
        }

    def picture(self):
        """The newest frame as a greyscale PNG: near is bright, invalid is black.

        The encoder is the map's -- `lidar_slam/mapimg.py` -- because writing a
        second PNG writer on the same rover is how two things that should look
        alike start to differ. It is a sibling directory in the repo and a sibling
        directory in ~/ugv, so one path works in both.
        """
        import numpy

        sys.path.insert(0, str(HERE.parent / "lidar_slam"))
        try:
            from mapimg import png_grey
        except ImportError as error:
            return None, f"no PNG encoder here: {error}"
        frame, _age = self.newest()
        if frame is None:
            return None, "no depth frame yet"
        clipped = numpy.clip(frame, MIN_MM, MAX_MM).astype(numpy.float32)
        # Near bright, far dark, and invalid pixels black rather than near: a zero
        # is "no measurement" and reading it as 20 cm would put a wall in front of
        # the rover every time the room went textureless.
        shade = 255 - ((clipped - MIN_MM) / (MAX_MM - MIN_MM) * 254).astype(numpy.uint8)
        shade[(frame == 0) | (frame > MAX_MM)] = 0
        return png_grey([row.tobytes() for row in shade]), ""


class Handler(BaseHTTPRequestHandler):
    server_version = "oak-depth/1.0"
    depth: Depth = None            # set on the server, read here

    def _reply(self, code: int, payload: dict) -> None:
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.partition("?")[0]
        if path in ("/health", "/"):
            health = self.depth.health()
            return self._reply(200 if health["ok"] else 503, health)
        if path == "/depth":
            summary = self.depth.summary()
            return self._reply(200 if summary["ok"] else 503, summary)
        if path == "/depth.png":
            png, why = self.depth.picture()
            if png is None:
                return self._reply(503, {"ok": False, "error": why})
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(png)))
            self.end_headers()
            return self.wfile.write(png)
        return self._reply(404, {"error": "GET /health, /depth, /depth.png"})

    def log_message(self, fmt, *args):
        """Quiet by default: the supervisor's log is for events, not requests."""
        if os.environ.get("OAK_DEPTH_ACCESS_LOG"):
            super().log_message(fmt, *args)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bind", default=DEFAULT_BIND,
                        help=f"address to listen on (default {DEFAULT_BIND}; "
                             f"nothing here authenticates)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS,
                        help=f"mono camera rate (default {DEFAULT_FPS})")
    parser.add_argument("--decimation", type=int, default=DECIMATION,
                        choices=(1, 2, 3, 4),
                        help=f"on-device downscale of the depth map "
                             f"(default {DECIMATION}, so 320x240)")
    args = parser.parse_args()

    depth = Depth(fps=args.fps, decimation=args.decimation)
    Handler.depth = depth
    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()

    # The device opens on this thread, so a camera that is not there is a message
    # and an exit rather than a service that answers 503 forever. The HTTP port is
    # already up, though, which is what lets restart.sh tell "still booting the
    # VPU" from "not listening at all".
    watchdog = threading.Thread(target=_watch, args=(depth,), daemon=True)
    watchdog.start()
    try:
        depth.run()
    except DepthError as error:
        print(f"[oak_depth] the camera stopped: {error}", file=sys.stderr, flush=True)
        return 1
    return 0


def _watch(depth: Depth) -> None:
    """Exit the process if the frames stop, so the supervisor can reopen the device.

    A depthai queue that has gone quiet does not raise: the host sits in `get()`
    and the device, having heard nothing, is already dead on its own watchdog. So
    silence has to be noticed by a clock rather than by an exception.
    """
    while True:
        time.sleep(1.0)
        _frame, age = depth.newest()
        started = time.monotonic() - depth.started_at
        if depth.frames == 0 and started < 30.0:
            continue                    # still booting the VPU; that takes seconds
        if depth.frames == 0 or age > FRAME_TIMEOUT_S:
            print(f"[oak_depth] no depth for {age:.1f}s after {depth.frames} "
                  f"frames; letting the supervisor reopen the device",
                  file=sys.stderr, flush=True)
            os._exit(2)


if __name__ == "__main__":
    sys.exit(main())
