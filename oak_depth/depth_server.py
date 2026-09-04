"""The OAK as a camera with a range on every pixel, on the rover, kept alive from
boot. A colour picture, and millimetres out.

    python3 depth_server.py                 # loopback 8770, at 2 fps
    python3 depth_server.py --bind 0.0.0.0  # ...reachable from the LAN as well
    python3 depth_server.py --fps 10        # video rate, and five times the USB traffic

    GET  /health     is the device up, what is it, and how is the depth
    GET  /depth      the newest frame as a coarse grid and per-sector ranges
    GET  /depth.png  the same frame as a greyscale picture, for a person
    GET  /frame      the newest colour picture, as a JPEG
    POST /ranges     how far away the things in these boxes are

**The depth is aligned to the colour camera, and that is what makes the two
answers one measurement.** `StereoDepth.setDepthAlign(CAM_A)` warps the disparity
out of the mono pair's geometry into the colour camera's, so a box drawn on the
picture `/frame` returns indexes the same fraction of the depth map and needs no
remapping on the host. The consequences are worth stating because they change
what older readings meant: depth here is measured from the *colour* camera's
optical centre rather than the right mono's, and every angle this service quotes
is now in the colour camera's frame -- 70.1 degrees across and 43.0 high, read
off the intrinsics the device stores rather than off `getFov`'s rounded 69.

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

**Nothing steers by this, and that is still deliberate.** The lidar is what keeps
the rover off walls, the floor reads as an obstacle to a forward-looking depth
camera, and a range that has not been checked against the room is a poor thing to
drive by. What does read it is the semantic world state, which asks `/ranges` how
far away the things it has just found are and spends the answer on deciding which
lasting thing each of them is -- see `world_state/oak.py`. That is a use with no
authority over the wheels, which is the order this was always meant to happen in.
"""

import argparse
import json
import math
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
# the disparity decimated 2x on the device before it is warped.
#
# **Two frames a second, not ten**, and it survived the colour camera being added
# beside the depth. What reads this asks about once a second at most and a parked
# rover is not looking at anything new, so the job is to have a picture and a
# range ready when somebody asks rather than to stream either; two a second means
# neither answer is ever more than half a second old. The link is the thing worth
# spending carefully -- 40 MB/s is where it saturates, everything on this rover
# shares one 480 Mbps root port, and losing the wifi adapter means losing the way
# to say "stop" -- and the arithmetic is 230 kB/s of depth plus 40 kB/s of
# colour, which is *less* than the 307 kB/s the unaligned depth alone cost before,
# because the aligned map is 320x180 where the old one was 320x240.
#
# Raise it with --fps when something actually reads this at rate. The rate is
# fixed when the pipeline is built, so changing it is a restart and a fresh
# firmware upload -- there is no way to retune a running device.
DEFAULT_FPS = 2
DECIMATION = 2
#: What the colour camera emits, and what the depth is warped to match.
#:
#: **640x360 because 1080p is the widest thing this sensor offers, not because
#: 16:9 was wanted.** Asked of the device rather than assumed: the IMX214 offers
#: 1920x1080, 3840x2160, 4056x3040 and 4208x3120 and nothing else, the two 4:3
#: modes are twelve megapixels apiece, and the 1080p mode is a *vertical* crop of
#: the sensor rather than a horizontal one -- 70.1 degrees across either way, and
#: 43.0 high against the full sensor's 54.9. So the wide mode costs a third of the
#: vertical field and saves the ISP eleven megapixels a frame, and a thing that
#: falls off the top of the picture is a case the consumer of this already handles
#: (`view.clipped_vertically`). The 4:3 route is `THE_12_MP` with an ISP downscale
#: if that third ever turns out to matter.
#:
#: `setIspScale(1, 3)` takes 1920x1080 to 640x360 on the device, so nothing but
#: the finished frame crosses the wire.
COLOUR_SIZE = (640, 360)
COLOUR_ISP_SCALE = (1, 3)
#: MJPEG on the device's own encoder rather than on the host: this board has no
#: OpenCV in this process, the VPU has an encoder sitting idle, and a 22 kB JPEG
#: at 2 fps is 40 kB/s against the 460 kB a raw frame would be.
JPEG_QUALITY = 90
#: And what size the aligned depth comes off the device at -- half the colour
#: frame in each direction, so `depth[y, x]` is the colour frame's `(2x, 2y)` and
#: the same *fraction* of both is the same ray.
#:
#: Half rather than full because the link is the thing worth spending carefully:
#: 320x180 of uint16 at 2 fps is 230 kB/s, which is less than the 307 kB/s the
#: unaligned 320x240 map cost before the colour camera was added at all. The
#: stereo pair still runs at 640x480 and is still decimated on the device; this is
#: only how much of the result is worth sending.
DEPTH_SIZE = (320, 180)
#: How many colour frames to keep while waiting for the depth frame they belong
#: with. Four is two seconds at the default rate, which is far longer than the
#: stereo pipeline's own lag and short enough that the buffer is never the reason
#: this process holds memory.
COLOUR_HISTORY = 4
# How long a frame may be missing before this process gives up and lets the
# supervisor open the device again from scratch. Nothing else recovers a Myriad
# that has been browned out or unplugged, so exiting *is* the repair. Generous
# enough not to fire on a slow moment: at the default 2 fps this is ten frames.
FRAME_TIMEOUT_S = 5.0
# Depths outside this are not measurements. The 7.5 cm baseline stops overlapping
# below about 20 cm, and beyond 6 m one disparity step is more than a metre.
MIN_MM, MAX_MM = 200, 6000
# What `/depth` reduces a frame to. Eight columns is about 9 degrees each across
# the lens, which is the width of a doorway at three metres.
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

#: How `/ranges` picks one distance out of a box full of pixels, as a percentile
#: of the valid ones.
#:
#: **A box is a thing in front of a room, and the question is how far the thing
#: is.** A region drawn round a chair also contains the floor beside it and the
#: wall behind it, so the median of the box is a blend of three surfaces and the
#: minimum is one bad pixel. What is wanted is the near *surface*: this
#: percentile finds roughly where its front face is, and `RANGE_BAND_M` below
#: gathers the pixels that belong to it.
#:
#: Twenty rather than the five the sectors use, because the two are asking
#: different questions. A sector is asking "is there anything I could hit", where
#: the nearest real surface is the whole answer; a region is asking "how far away
#: is this object", where the nearest corner of it is an underestimate.
RANGE_PERCENTILE = 20
#: How far behind that percentile a pixel may lie and still be the same surface,
#: as a fraction of the range and as a floor in metres. Whichever is larger: a
#: sideboard is half a metre deep at any range, and the stereo noise itself grows
#: with the square of the range.
RANGE_BAND_FRAC = 0.15
RANGE_BAND_M = 0.30
#: The fewest valid pixels in a box worth answering from. Below this a range is a
#: handful of stereo mismatches rather than a surface, and `null` is the honest
#: answer -- which is not the same as nothing being there.
RANGE_MIN_PIXELS = 12
#: How wrong one disparity reading is, in pixels, which is the only assumed term
#: in what a range is worth.
#:
#: A stereo range's error is `z^2 * sigma_disparity / (focal_px * baseline_m)`,
#: and the other two terms are read off the device rather than assumed -- 455.8 px
#: of focal length on CAM_C at 640x480 and a 7.50 cm baseline. A fifth of a pixel
#: is what subpixel mode's eighth-of-a-pixel steps make plausible, and it comes to
#: 0.00585 metres per metre squared: 2.3 cm at two metres, 9.4 at four and 21 at
#: six, which is why `MAX_MM` stops where it does.
#:
#: **It is a model and not a measurement, and it wants a tape measure.** What it
#: produces is spent by `locate` as the weight on a range residual, so being wrong
#: optimistic makes the world state trust a range more than it should. The spread
#: of the pixels behind each reading is added to it, which covers a thing being
#: deep or slanted rather than flat, and covers nothing about the model itself.
DISPARITY_SIGMA_PX = 0.2


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


def _stamp_of(packet) -> float:
    """When the device says this packet was captured, in seconds, or 0.

    depthai hands back a `timedelta` on a clock it has already synchronised to
    this host's monotonic one, which is what makes it worth keeping: the colour
    frame and the depth frame carry stamps that can be *compared*, so a consumer
    can be told how far apart the picture and the ranges in it actually were
    rather than being left to assume they were simultaneous.
    """
    try:
        return packet.getTimestamp().total_seconds()
    except Exception:                  # a build that does not stamp its packets
        return 0.0


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
        self.frame_stamp = 0.0       # the device's own clock, for pairing
        self.frames = 0
        self.errors = 0
        self.valid = 0.0             # share of pixels with a depth, last frame
        self.rate = 0.0
        # The colour half. Kept beside the depth rather than in a class of its own
        # because the two are one measurement: the depth is warped into this
        # picture's geometry, and what makes them usable together is that they
        # arrive from the same device at the same rate.
        self.jpeg = b""              # newest colour frame, already encoded
        self.jpeg_at = 0.0
        self.jpeg_stamp = 0.0
        self.jpegs = 0
        self.device_name = ""
        self.usb_speed = ""
        self.hfov_deg = None
        self.vfov_deg = None
        #: The colour camera's intrinsics at `COLOUR_SIZE`, as
        #: (fx, fy, cx, cy) in pixels. This is what turns a box on the picture
        #: into a direction, and it is published rather than kept, so that
        #: whoever draws bearings from these frames does it through the lens the
        #: device says it has instead of a copy that can drift. Null if the
        #: stored calibration would not read, and null means no bearings.
        self.colour_intrinsics = None
        self.baseline_cm = None
        self.focal_px = None         # CAM_C's, which is what the depth is made of
        self.started_at = time.monotonic()
        self.last_error = ""
        self._periods: list[float] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()

    # --- the pipeline -------------------------------------------------------

    def _pipeline(self):
        dai = self.dai
        pipeline = dai.Pipeline()

        # The colour camera, downscaled on the device's own ISP and encoded by
        # its own MJPEG encoder, so what crosses the USB link is a finished
        # picture. `video` rather than `preview` because the encoder wants NV12
        # and `preview` is planar BGR for a host that is about to display it.
        colour = pipeline.create(dai.node.ColorCamera)
        colour.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        colour.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        colour.setIspScale(*COLOUR_ISP_SCALE)
        colour.setVideoSize(*COLOUR_SIZE)
        colour.setInterleaved(False)
        colour.setFps(self.fps)

        encoder = pipeline.create(dai.node.VideoEncoder)
        encoder.setDefaultProfilePreset(
            self.fps, dai.VideoEncoderProperties.Profile.MJPEG)
        encoder.setQuality(JPEG_QUALITY)
        colour.video.link(encoder.input)

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
        # **Warp the disparity into the colour camera's geometry, on the
        # device.** This is what makes a box drawn on `/frame` index the same
        # fraction of the depth map, with no remapping and no second lens model
        # on the host. It moves where depth is measured from -- the colour
        # camera's optical centre rather than the right mono's -- and it crops to
        # the colour camera's field, which the mono pair's wider one covers.
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(*DEPTH_SIZE)
        config = stereo.initialConfig.get()
        config.postProcessing.decimationFilter.decimationFactor = self.decimation
        # Median over a small window: cheap on the device, and it is the filter
        # that removes single-pixel disparity errors rather than smoothing edges.
        config.postProcessing.median = \
            self.dai.MedianFilter.KERNEL_5x5
        stereo.initialConfig.set(config)
        left.out.link(stereo.left)
        right.out.link(stereo.right)

        for name, source in (("depth", stereo.depth),
                             ("jpeg", encoder.bitstream)):
            out = pipeline.create(dai.node.XLinkOut)
            out.setStreamName(name)
            source.link(out.input)
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
                queues = {name: device.getOutputQueue(name, maxSize=4,
                                                      blocking=False)
                          for name in ("depth", "jpeg")}
                # **The two streams do not arrive together, and the colour is
                # the late one.** Both are exposed at the same instant on the
                # same device, but the colour frame goes through the MJPEG
                # encoder before it crosses the link, and measured on this
                # camera that puts it a whole exposure behind its own depth
                # frame -- pairing whatever was newest of each gave 0.49 s at 2
                # fps, which is 23 cm of rover at the speed it explores at, and
                # a box drawn on one of those and a range taken from the other
                # are not one measurement.
                #
                # So a depth frame is *held* rather than published on arrival,
                # and goes out once the colour frame exposed with it has turned
                # up. `recent` is the colour frames waiting to be claimed and
                # `pending` is the depth frame waiting for one.
                recent: list[tuple[float, bytes]] = []
                pending = None
                while not self._stop.is_set():
                    # The depth queue is what this loop waits on, because it is
                    # what the watchdog counts: silence there has to be
                    # noticeable, and a colour stream that stopped on its own
                    # must not be able to stall the depth.
                    packet = queues["depth"].get()
                    now = time.monotonic()
                    while True:
                        picture = queues["jpeg"].tryGet()
                        if picture is None:
                            break
                        recent.append((_stamp_of(picture),
                                       bytes(picture.getData())))
                        self.jpegs += 1
                        del recent[:-COLOUR_HISTORY]
                    if pending is not None:
                        self._publish(pending, recent, now)
                    pending = (packet.getFrame(), _stamp_of(packet), now)
        except Exception as error:
            self.errors += 1
            self.last_error = f"{type(error).__name__}: {error}"
            raise DepthError(self.last_error) from None

    def _publish(self, pending, recent: list, now: float) -> None:
        """Make one held depth frame, and the picture it belongs with, the answer.

        The two go out together under one lock, so `/frame` and `/ranges` always
        describe one instant rather than two -- which is the whole point of
        holding the depth frame back. The picture is a little older than the
        colour path could have delivered it, and that is the right way round.

        The depth goes out whether or not a matching picture was found. A colour
        stream that has stopped is a reason to have no picture, not a reason to
        stop measuring distance.
        """
        depth, stamp, arrived = pending
        matched = self._nearest(recent, stamp)
        with self._lock:
            if self.frame_at:
                self._periods.append(arrived - self.frame_at)
                del self._periods[:-STAT_WINDOW]
                if self._periods:
                    self.rate = 1.0 / statistics.fmean(self._periods)
            self.frame, self.frame_at = depth, arrived
            self.frame_stamp = stamp
            self.frames += 1
            self.valid = float((depth != 0).mean())
            if matched is not None:
                self.jpeg_stamp, self.jpeg = matched
                # Aged from when its depth frame arrived rather than from when
                # the picture did, because the two are now one reading and a
                # consumer interpolating a pose to it needs one instant.
                self.jpeg_at = arrived

    @staticmethod
    def _nearest(recent: list, stamp: float):
        """The colour frame exposed closest to this depth frame, or None.

        The newest one when the device stamps nothing, which is what this did
        before the two were paired properly and is still the honest answer for a
        build that cannot say when anything was taken.
        """
        if not recent:
            return None
        if not stamp:
            return recent[-1]
        usable = [one for one in recent if one[0]]
        if not usable:
            return recent[-1]
        return min(usable, key=lambda one: abs(one[0] - stamp))

    def _describe(self, device) -> None:
        """What this device is, asked of it rather than remembered.

        **The lens is read off the stored calibration for the size this service
        actually emits**, because everything downstream turns pixels into angles
        with it and a lens described twice is two lenses that will disagree --
        the mistake `face_tracking/lens.py` exists to have stopped making for the
        other camera on this rover. `getCameraIntrinsics(socket, w, h)` accounts
        for the crop and the scale of the mode in use, so the numbers here
        describe the frame `/frame` returns and not the full sensor.

        It is also why the field of view quoted here is 70.1 degrees rather than
        the 69 `getFov` reports: `getFov` returns the spec figure for the sensor,
        and the fitted intrinsics are what the pixels obey.
        """
        dai = self.dai
        self.device_name = device.getDeviceName()
        self.usb_speed = device.getUsbSpeed().name
        try:
            calibration = device.readCalibration()
            width, height = COLOUR_SIZE
            matrix = calibration.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_A, width, height)
            fx, fy = float(matrix[0][0]), float(matrix[1][1])
            self.colour_intrinsics = {
                "fx": round(fx, 2), "fy": round(fy, 2),
                "cx": round(float(matrix[0][2]), 2),
                "cy": round(float(matrix[1][2]), 2),
                "width": width, "height": height}
            self.hfov_deg = round(math.degrees(2 * math.atan(width / (2 * fx))), 1)
            self.vfov_deg = round(math.degrees(2 * math.atan(height / (2 * fy))), 1)
            self.baseline_cm = round(calibration.getBaselineDistance(), 2)
            # The stereo pair's own focal length, at the resolution the pair runs
            # at, because it is a term in what a range is worth -- see
            # `DISPARITY_SIGMA_PX`. Not the colour camera's: the disparity is
            # measured between the two monos whatever frame it is later warped
            # into.
            right = calibration.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_C, 640, 480)
            self.focal_px = round(float(right[0][0]), 2)
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

    def newest_jpeg(self):
        """The newest colour frame, how old it is, and how far the depth it goes
        with was taken from it.

        The third number is the one worth having and is why this is not two
        calls: a picture and a set of ranges measured a second apart on a moving
        rover are two different rooms, and the consumer has to be able to see
        that rather than assume it away. Zero when the device stamps neither.
        """
        with self._lock:
            if not self.jpeg:
                return b"", 0.0, 0.0
            apart = (abs(self.jpeg_stamp - self.frame_stamp)
                     if self.jpeg_stamp and self.frame_stamp else 0.0)
            return self.jpeg, time.monotonic() - self.jpeg_at, apart

    def health(self) -> dict:
        frame, age = self.newest()
        jpeg, jpeg_age, apart = self.newest_jpeg()
        return {
            "ok": frame is not None and age < FRAME_TIMEOUT_S,
            "device": self.device_name,
            "usb": self.usb_speed,
            # The firmware version and the library version are the same number on
            # this camera, because the host uploads the firmware out of the wheel
            # on every open. See README.md.
            "depthai": getattr(self.dai, "__version__", "?"),
            # The colour camera's field, not the mono pair's: the depth is warped
            # into the colour camera's geometry, so this is the frame every angle
            # this service quotes is measured in.
            "hfov_deg": self.hfov_deg,
            "vfov_deg": self.vfov_deg,
            "baseline_cm": self.baseline_cm,
            "size": list(reversed(frame.shape)) if frame is not None else None,
            "fps": round(self.rate, 1),
            "frames": self.frames,
            "valid": round(self.valid, 3),
            "age_s": round(age, 2) if frame is not None else None,
            # What the colour half is doing, and how well the two halves line up
            # in time. A consumer that draws bearings from these pictures reads
            # `colour` for the lens to draw them through.
            "colour": {
                "size": list(COLOUR_SIZE),
                "bytes": len(jpeg),
                "frames": self.jpegs,
                "age_s": round(jpeg_age, 2) if jpeg else None,
                "depth_apart_s": round(apart, 3),
                "intrinsics": self.colour_intrinsics,
            },
            "uptime_s": round(time.monotonic() - self.started_at, 1),
            "errors": self.errors,
            "last_error": self.last_error,
        }

    def ranges(self, boxes: list) -> dict:
        """How far away the thing in each box is, in metres, or null.

        The boxes are fractions of the frame -- `[left, top, right, bottom]` in
        0..1 -- because the colour picture and the depth map are different pixel
        sizes covering the same field, so a fraction is the one coordinate that
        means the same thing in both. That is the whole benefit of aligning the
        depth on the device: the caller draws its box on the picture it was given
        and asks about it directly.

        **Null is a real answer and is not an error.** A dark or textureless
        surface returns no disparity at all, so a box over a bare wall has
        nothing in it to measure; `valid` beside each answer is how to tell that
        from a box the camera simply could not see.

        The distance is **out along the ray to the box**, which is not what the
        depth map holds -- see `_secant`. `/depth` and `/depth.png` still speak in
        the map's own axial millimetres, because what a sector is for is "could I
        hit that", and the distance in front of the camera plane is the honest
        answer to that one.
        """
        import numpy

        frame, age = self.newest()
        if frame is None:
            return {"ok": False, "error": "no depth frame yet"}
        height, width = frame.shape
        answers = []
        for box in boxes:
            answers.append(self._range_in(numpy, frame, width, height, box))
        _jpeg, jpeg_age, apart = self.newest_jpeg()
        return {"ok": True, "age_s": round(age, 2),
                "size": [width, height],
                "frame_age_s": round(jpeg_age, 2),
                "depth_apart_s": round(apart, 3),
                "ranges": answers}

    def _angle_across(self, x: float, width: int) -> float | None:
        """Where a column of the depth map lies, in degrees right of the axis.

        None when the calibration would not read, which is the same silence
        everything else here keeps rather than quoting an angle off a guess. The
        intrinsics are the colour camera's at `COLOUR_SIZE`, so they are scaled
        to whatever width the depth map is emitted at.
        """
        if not self.colour_intrinsics:
            return None
        scale = width / float(self.colour_intrinsics["width"])
        fx = self.colour_intrinsics["fx"] * scale
        cx = self.colour_intrinsics["cx"] * scale
        return round(math.degrees(math.atan2(x - cx, fx)), 1)

    def _range_in(self, numpy, frame, width, height, box) -> dict | None:
        """One box, as a distance to the near surface inside it.

        Two steps, and the second is what stops the wall behind a chair being
        averaged into the chair. A low percentile of the valid pixels finds where
        the front of the thing is; everything within `RANGE_BAND_M` behind that
        is taken to belong to it and the median of *those* is the answer. A plain
        median over the box would blend the object, the floor beside it and the
        wall behind it into a distance that is none of the three.
        """
        try:
            left, top, right, bottom = (float(value) for value in box)
        except (TypeError, ValueError):
            return None
        x0 = max(0, min(width - 1, int(round(min(left, right) * width))))
        x1 = max(x0 + 1, min(width, int(round(max(left, right) * width))))
        y0 = max(0, min(height - 1, int(round(min(top, bottom) * height))))
        y1 = max(y0 + 1, min(height, int(round(max(top, bottom) * height))))
        cell = frame[y0:y1, x0:x1]
        mask = (cell >= MIN_MM) & (cell <= MAX_MM)
        share = float(mask.mean()) if cell.size else 0.0
        values = cell[mask]
        if values.size < RANGE_MIN_PIXELS:
            return {"range_m": None, "sigma_m": None,
                    "valid": round(share, 3), "pixels": int(values.size)}
        near_mm = float(numpy.percentile(values, RANGE_PERCENTILE))
        band_mm = max(RANGE_BAND_M * 1000.0, RANGE_BAND_FRAC * near_mm)
        surface = values[values <= near_mm + band_mm]
        if surface.size < RANGE_MIN_PIXELS:
            surface = values
        along_m = float(numpy.median(surface)) / 1000.0
        # **Out along the ray, not along the axis, and the difference is not
        # small.** What a stereo pipeline puts in a depth map is `focal *
        # baseline / disparity`, which is the *Z* coordinate -- how far the
        # surface is in front of the camera plane -- and what anybody asking
        # "how far away is that" means is the length of the line to it. The two
        # are the same only dead ahead: they differ by one over the cosine of the
        # angle off the axis, which on this lens is 22% at the side of the frame
        # and 32% in the corner. `world_state.locate` compares this against a
        # distance measured on the map, so shipping the axial figure would have
        # put every off-centre range a fifth short.
        range_m = along_m * self._secant(x0, x1, y0, y1, width, height)
        # What it is worth: the stereo model's own error at this range, and the
        # spread of the surface that produced it, which is the thing being deep
        # or slanted rather than flat.
        focal = self.focal_px or 455.8
        baseline_m = (self.baseline_cm or 7.5) / 100.0
        model = range_m * range_m * DISPARITY_SIGMA_PX / (focal * baseline_m)
        spread = float(numpy.std(surface)) / 1000.0
        return {"range_m": round(range_m, 3),
                "sigma_m": round(math.hypot(model, spread / 2.0), 3),
                "valid": round(share, 3),
                "pixels": int(surface.size)}

    def _secant(self, x0: int, x1: int, y0: int, y1: int,
                width: int, height: int) -> float:
        """One over the cosine of the angle from the lens axis to this box.

        1.0 when the calibration would not read, which leaves the axial figure
        alone rather than scaling it by a guess -- and a service with no
        calibration already reports no angles at all, so nothing is drawing
        bearings through it either.

        Taken at the box's middle. A box is at most a few degrees wide as far as
        this factor is concerned, and it varies slowly, so the middle is the box.
        """
        lens = self.colour_intrinsics
        if not lens:
            return 1.0
        scale_x = width / float(lens["width"])
        scale_y = height / float(lens["height"])
        x = ((x0 + x1) / 2.0 - lens["cx"] * scale_x) / (lens["fx"] * scale_x)
        y = ((y0 + y1) / 2.0 - lens["cy"] * scale_y) / (lens["fy"] * scale_y)
        return math.sqrt(1.0 + x * x + y * y)

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
        for x0, x1 in columns:
            cell, mask = frame[band, x0:x1], valid[band, x0:x1]
            # Positive is to the right of the camera's axis, which is the same
            # sign convention aiming.py uses for a face in the picture.
            #
            # **Through the lens rather than across the frame.** These used to be
            # the field of view multiplied by how far across the picture the
            # column sat, which is only right at the centre and at the two edges:
            # a quarter of the way out it reads 17.5 degrees where the lens says
            # 19.3. Now that something draws bearings from this camera the
            # difference is worth the arctangent.
            sectors.append({
                "from_deg": self._angle_across(x0, width),
                "to_deg": self._angle_across(x1, width),
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
        if path == "/frame":
            jpeg, age, apart = self.depth.newest_jpeg()
            if not jpeg:
                return self._reply(503, {"ok": False,
                                         "error": "no colour frame yet"})
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            # How old the picture is and how far the depth that goes with it was
            # taken from it, on the reply rather than in a second call: a caller
            # about to work out a bearing from this frame needs both, and a
            # second round trip would be answering about a different frame.
            self.send_header("X-Frame-Age", f"{age:.3f}")
            self.send_header("X-Depth-Apart", f"{apart:.3f}")
            self.send_header("X-Frame-Size",
                             f"{COLOUR_SIZE[0]}x{COLOUR_SIZE[1]}")
            self.end_headers()
            return self.wfile.write(jpeg)
        return self._reply(404, {"error": "GET /health, /depth, /depth.png, "
                                          "/frame; POST /ranges"})

    def do_POST(self):
        if self.path.partition("?")[0] != "/ranges":
            return self._reply(404, {"error": "POST /ranges"})
        try:
            length = int(self.headers.get("Content-Length") or 0)
            payload = json.loads(self.rfile.read(length).decode("utf-8", "replace"))
            boxes = payload["boxes"]
            if not isinstance(boxes, list):
                raise TypeError("boxes must be a list")
        except (KeyError, TypeError, ValueError) as error:
            return self._reply(400, {"ok": False,
                                     "error": f"send {{\"boxes\": [[left, top, "
                                              f"right, bottom], ...]}} as "
                                              f"fractions of the frame: {error}"})
        answer = self.depth.ranges(boxes)
        return self._reply(200 if answer.get("ok") else 503, answer)

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
                        help=f"camera rate, colour and depth alike "
                             f"(default {DEFAULT_FPS})")
    parser.add_argument("--decimation", type=int, default=DECIMATION,
                        choices=(1, 2, 3, 4),
                        help=f"on-device downscale of the disparity before "
                             f"it is warped into the colour frame "
                             f"(default {DECIMATION})")
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
