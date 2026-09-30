#!/usr/bin/env python3
"""Capture reviewable MJPEG frames and factory lens data from OAK CAM_A.

Run only while the resident depth service has released the device. This is a bench
capture, not a replacement for the paired 640x360 colour/depth runtime stream.

    PYTHONPATH=~/ugv/oak_depth/vendor:~/ugv/oak_depth python3 capture_rgb.py \
      /tmp/oak-rgb --size 1280x720 --frames 5

The lens it records comes from `oak_depth/colour_lens.py`, the same arithmetic the
service uses, and it needs to: a size below 1920x1080 here is the *middle* of the
1080p picture rather than a scaled copy of it, and 1080p is itself only the middle
of the sensor. depthai's `getCameraIntrinsics(socket, w, h)` assumes neither, and
what this wrote before 2026-09-30 was 9.6% short in focal length at 1920x1080 and
39% short at 1280x720.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from datetime import datetime
from pathlib import Path

import depthai as dai

# `colour_lens` lives with the depth service: beside this directory in a checkout,
# and in ~/ugv/oak_depth on the rover, which is where the command above points.
sys.path.append(str(Path(__file__).resolve().parent.parent / "oak_depth"))
from colour_lens import colour_intrinsics  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("folder", type=Path)
    parser.add_argument("--size", default="1280x720",
                        choices=("960x540", "1280x720", "1920x1080"))
    parser.add_argument("--frames", type=int, default=5)
    args = parser.parse_args()
    width, height = (int(value) for value in args.size.split("x"))
    if args.frames < 1 or args.frames > 20:
        parser.error("--frames must be in 1..20")
    if args.folder.exists():
        parser.error(f"refusing to overwrite {args.folder}")
    args.folder.mkdir(parents=True)

    pipeline = dai.Pipeline()
    camera = pipeline.create(dai.node.ColorCamera)
    camera.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camera.setVideoSize(width, height)
    camera.setInterleaved(False)
    camera.setFps(15)
    camera.initialControl.setAutoFocusMode(
        dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO
    )

    encoder = pipeline.create(dai.node.VideoEncoder)
    encoder.setDefaultProfilePreset(15, dai.VideoEncoderProperties.Profile.MJPEG)
    encoder.setQuality(95)
    camera.video.link(encoder.input)
    output = pipeline.create(dai.node.XLinkOut)
    output.setStreamName("jpeg")
    encoder.bitstream.link(output.input)

    rows = []
    with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:
        calibration = device.readCalibration()
        lens = colour_intrinsics(calibration, dai, "THE_1080_P", (1, 1),
                                 (width, height))
        distortion = calibration.getDistortionCoefficients(
            dai.CameraBoardSocket.CAM_A
        )
        queue = device.getOutputQueue("jpeg", maxSize=4, blocking=True)
        # Autofocus takes materially longer than exposure after firmware upload.
        # Three seconds was added after the first 10-frame preflight enlarged the
        # still-blurred image without recovering any markers.
        packets = [queue.get() for _ in range(args.frames + 45)]
        for number, packet in enumerate(packets[-args.frames:], 1):
            data = bytes(packet.getData())
            path = args.folder / f"oak-{number}.jpg"
            path.write_bytes(data)
            rows.append({
                "file": path.name,
                "bytes": len(data),
                "sha256": hashlib.sha256(data).hexdigest(),
                "sequence": int(packet.getSequenceNum()),
            })
        meta = {
            "captured_at": datetime.now().astimezone().isoformat(),
            "device": device.getDeviceName(),
            "usb": device.getUsbSpeed().name,
            "colour": {
                "size": [width, height],
                "intrinsics": lens,
                "distortion": [float(value) for value in distortion],
            },
            "frames": rows,
        }
    (args.folder / "calibration-health.json").write_text(
        json.dumps(meta, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(meta))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
