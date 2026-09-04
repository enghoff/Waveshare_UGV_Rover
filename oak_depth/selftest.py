#!/usr/bin/env python3
"""Prove the depth camera on the machine that runs it, from the library up.

    ssh orin 'python3 ~/ugv/oak_depth/selftest.py'
    ssh orin 'python3 ~/ugv/oak_depth/selftest.py --frames 60 --png /tmp/depth.png'

Each stage is one more thing that can be wrong, in the order they fail: depthai
imports, the udev rule is in place, the camera is on the bus, the device opens at
USB2, the firmware and the pipeline upload, frames arrive, and only then is the
depth in them worth reading. The first line that fails is the one that says what
to fix, which is why it goes in this order rather than starting the service and
reading a stack trace out of its log.

**One process at a time owns the camera.** Run this with the service stopped, or
it will fail at "device opens" for a reason that has nothing to do with the
camera: `~/ugv/oak_depth/restart.sh` afterwards puts the service back.
"""

import argparse
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from depth_server import (  # noqa: E402
    COLOUR_SIZE, MAX_MM, MIN_MM, Depth, _import_depthai,
)

FAILURES = []
UDEV_RULE = "/etc/udev/rules.d/97-myriad-usbboot.rules"


def check(label, fn):
    """Run one stage, print its line, and keep going unless it was fatal."""
    try:
        detail = fn()
    except Exception as error:
        FAILURES.append(label)
        print("  FAIL  %-24s %s" % (label, error))
        return None
    print("  ok    %-24s %s" % (label, detail if detail is not None else ""))
    return detail if detail is not None else True


def report():
    if FAILURES:
        print(f"\n{len(FAILURES)} failed: {', '.join(FAILURES)}")
        return 1
    print("\nall stages passed")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--frames", type=int, default=30,
                        help="how many depth frames to take (default 30)")
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--png", metavar="PATH",
                        help="write the last frame as a greyscale PNG")
    args = parser.parse_args()

    print("oak_depth selftest")

    def library():
        dai = _import_depthai()
        # The version *is* the firmware version -- the host uploads it out of the
        # wheel on every open -- so this line is the answer to "which firmware is
        # on the camera". 3.x on this unit has no working left mono sensor and so
        # no stereo depth at all; see README.md.
        if not dai.__version__.startswith("2."):
            raise RuntimeError(
                f"depthai {dai.__version__} is a 3.x release, which cannot do "
                f"stereo depth on this camera -- install.sh pins 2.32.0.0")
        return f"depthai {dai.__version__}"

    def udev():
        if not os.path.exists(UDEV_RULE):
            raise RuntimeError(
                f"{UDEV_RULE} is missing, so libusb will fail with "
                f"LIBUSB_ERROR_ACCESS and report it as no camera")
        return "03e7 is readable by group users"

    def on_the_bus():
        listing = subprocess.run(["lsusb"], capture_output=True, text=True).stdout
        for product, state in (("2485", "ROM bootloader, waiting for firmware"),
                               ("f63b", "already booted by something")):
            if f"03e7:{product}" in listing:
                return f"03e7:{product} -- {state}"
        raise RuntimeError("no 03e7 device in lsusb; the camera is unplugged, "
                           "or its cable carries power only")

    def not_taken():
        # Two things can hold the device: this repository's own service, and a
        # crashed process that has not let go. The first is a stop-and-retry, the
        # second is why run_oak_depth.sh exists.
        held = subprocess.run(["pgrep", "-af", "depth_server.py"],
                              capture_output=True, text=True).stdout.strip()
        held = [line for line in held.splitlines() if "selftest" not in line]
        if held:
            raise RuntimeError(f"the depth service is running and owns the "
                               f"camera: {held[0]}")
        return "nothing else has the camera"

    if check("depthai imports", library) is None:
        return report()
    check("udev rule installed", udev)
    if check("camera on the USB bus", on_the_bus) is None:
        return report()
    if check("nothing else holds it", not_taken) is None:
        return report()

    depth = Depth(fps=args.fps)
    frames = []

    def pull():
        """Open the device and take `--frames` frames, then stop the puller.

        `Depth.run` is the service's own loop and it does not return until the
        device fails, so this stops it from the outside once enough frames have
        arrived -- which also exercises the stop path the supervisor relies on.
        """
        started = time.monotonic()

        def watch():
            while depth.frames < args.frames and time.monotonic() - started < 60:
                time.sleep(0.05)
            depth.stop()

        threading.Thread(target=watch, daemon=True).start()
        depth.run()
        if depth.frames < args.frames:
            raise RuntimeError(f"only {depth.frames} frames in "
                               f"{time.monotonic() - started:.0f} s")
        frames.append(depth.newest()[0])
        return (f"{depth.device_name} on USB {depth.usb_speed}, {depth.frames} "
                f"frames at {depth.rate:.1f} fps")

    if check("device opens, depth flows", pull) is None:
        return report()

    def calibrated():
        if depth.hfov_deg is None or not depth.colour_intrinsics:
            raise RuntimeError("the stored calibration would not read, so no "
                               "angle this service quotes can be trusted -- and "
                               "nothing can draw a bearing through this camera")
        lens = depth.colour_intrinsics
        return (f"{depth.hfov_deg} x {depth.vfov_deg} deg, fx={lens['fx']} "
                f"cx={lens['cx']} cy={lens['cy']}, "
                f"{depth.baseline_cm} cm baseline")

    def colour():
        """The colour half, which is half of what makes a range usable.

        A depth map on its own says how far away *something* is. What the world
        state needs is how far away the thing it has just found in the picture
        is, and that needs the picture and the depth to be the same frame -- so
        this checks that both arrived and how far apart in time they were.
        """
        jpeg, age, apart = depth.newest_jpeg()
        if not jpeg:
            raise RuntimeError(f"{depth.frames} depth frames arrived but no "
                               f"colour frame did, so the MJPEG encoder or the "
                               f"ISP scale is wrong for this sensor")
        if not jpeg.startswith(b"\xff\xd8"):
            raise RuntimeError(f"the colour stream is not JPEG: it starts "
                               f"{jpeg[:4].hex()}")
        return (f"{COLOUR_SIZE[0]}x{COLOUR_SIZE[1]}, {len(jpeg)} bytes, "
                f"{age:.2f} s old, {apart * 1000:.0f} ms from its depth frame")

    def ranged():
        """A range for a box, which is the call the world state actually makes.

        Three boxes across the middle of the picture rather than one, because
        the interesting failure is not "no answer" but "the same answer
        everywhere", which one box cannot show.
        """
        boxes = [[0.05, 0.35, 0.35, 0.65],
                 [0.35, 0.35, 0.65, 0.65],
                 [0.65, 0.35, 0.95, 0.65]]
        answer = depth.ranges(boxes)
        if not answer.get("ok"):
            raise RuntimeError(answer.get("error", "no answer"))
        got = [one for one in answer["ranges"] if one and one["range_m"]]
        if not got:
            shares = ", ".join(f"{one['valid']:.2f}" for one in answer["ranges"]
                               if one)
            raise RuntimeError(f"no box had enough valid pixels to range "
                               f"(valid shares {shares}) -- point the camera at "
                               f"something textured between "
                               f"{MIN_MM / 1000:.1f} and {MAX_MM / 1000:.0f} m")
        return ", ".join(f"{one['range_m']:.2f}+-{one['sigma_m']:.2f} m"
                         for one in got)

    def measures():
        frame = frames[0]
        summary = depth.summary()
        ranged = [s["near_mm"] for s in summary["sectors"] if s["near_mm"]]
        if not ranged:
            raise RuntimeError(
                f"no sector had enough valid pixels to give a range "
                f"({summary['valid'] * 100:.0f}% of the frame is valid) -- point "
                f"the camera at something textured between "
                f"{MIN_MM / 1000:.1f} and {MAX_MM / 1000:.0f} m")
        return (f"{frame.shape[1]}x{frame.shape[0]}, "
                f"{summary['valid'] * 100:.0f}% valid, nearest "
                f"{summary['near_mm']} mm, {len(ranged)}/8 sectors ranged")

    def picture():
        png, why = depth.picture()
        if png is None:
            raise RuntimeError(why)
        if args.png:
            Path(args.png).write_bytes(png)
            return f"{len(png)} bytes, written to {args.png}"
        return f"{len(png)} bytes (pass --png to keep it)"

    check("calibration reads", calibrated)
    check("the depth means something", measures)
    check("the colour frame arrives", colour)
    check("a box has a range", ranged)
    check("greyscale PNG encodes", picture)
    return report()


if __name__ == "__main__":
    sys.exit(main())
