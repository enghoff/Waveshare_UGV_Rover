#!/usr/bin/env python3
"""Step the gimbal and keep both cameras' pictures and the OAK's IMU at each stop.

    ssh orin 'pkill -f "oak_depth/run_oak_depth[.]sh"'
    scp usb_cameras/capture_lens_sweep.py orin:/tmp/
    ssh orin 'PYTHONPATH=~/ugv/oak_depth/vendor python3 /tmp/capture_lens_sweep.py /tmp/sweep'
    ssh orin '~/ugv/oak_depth/restart.sh'
    scp -r orin:/tmp/sweep captures/YYYY-MM-DD-fisheye-sweep

The evidence `fit_fisheye.py` fits the fisheye's outer shape from, and the check
it is judged against: at every stop the gimbal camera's picture (through the
daemon, which owns it), the OAK's colour picture in the depth service's own mode,
and the OAK's accelerometer and gyro. **No angle here comes from a servo.** A tilt
step's true size is how far gravity turned in the OAK's accelerometer, which has
no scale in it to be wrong; a pan step has no such truth, and its pictures are
used only for shape, with each pair's rotation left free.

Every step is taken from below, so the servos' backlash is on the same side each
time. The OAK is held by this script for the length of the sweep, so the depth
service has to be stopped first and restarted after -- the device reports a crash
when the service lets go of it, which is its watchdog and not a fault.
"""
from __future__ import annotations

import base64
import json
import os
import socket
import sys
import threading
import time

import cv2
import depthai as dai
import numpy as np

SETTLE_S = 2.5
HOLD_S = 1.0
DAEMON = ("127.0.0.1", 8769)
#: Tilt from -15 to +40 at pan 0, then pan from -40 to +40 at tilt 10, in fives.
POSES = ([(0, tilt) for tilt in range(-15, 45, 5)]
         + [(pan, 10) for pan in range(-40, 45, 5)])
#: Where the pan sweep starts in `POSES`. The move onto it is not a step.
PAN_FROM = 12


def call(name, arguments=None):
    with socket.create_connection(DAEMON, 20) as link:
        stream = link.makefile("rwb")
        stream.write((json.dumps({"call": name, "arguments": arguments or {}})
                      + "\n").encode())
        stream.flush()
        return json.loads(stream.readline())


class Readers:
    """The IMU and the colour camera, drained on threads so neither queue backs up."""

    def __init__(self, device):
        self.samples, self.lock = [], threading.Lock()
        self.frame, self.frame_at, self.lens_position = None, 0.0, None
        self.stop = threading.Event()
        threading.Thread(target=self._imu, args=(device.getOutputQueue("imu", 50, True),),
                         daemon=True).start()
        threading.Thread(target=self._colour,
                         args=(device.getOutputQueue("colour", 4, False),),
                         daemon=True).start()

    def _imu(self, queue):
        while not self.stop.is_set():
            for packet in queue.get().packets:
                a, g = packet.acceleroMeter, packet.gyroscope
                with self.lock:
                    self.samples.append((a.x, a.y, a.z, g.x, g.y, g.z, time.time(),
                                         a.getTimestampDevice().total_seconds()))

    def _colour(self, queue):
        while not self.stop.is_set():
            frame = queue.get()
            self.frame, self.frame_at = frame.getCvFrame(), time.time()
            try:
                self.lens_position = frame.getLensPosition()
            except Exception:                                     # noqa: BLE001
                pass

    def between(self, start, end):
        with self.lock:
            rows = [one for one in self.samples if start <= one[6] <= end]
        return np.array(rows) if rows else np.zeros((0, 8))


def pipeline():
    built = dai.Pipeline()
    imu = built.create(dai.node.IMU)
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW,
                         dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(5)
    imu.setMaxBatchReports(20)
    # The depth service's own colour mode, so the pictures are the ones it serves.
    colour = built.create(dai.node.ColorCamera)
    colour.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    colour.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    colour.setIspScale(1, 3)
    colour.setVideoSize(640, 360)
    colour.setInterleaved(False)
    colour.setFps(15)
    for name, source in (("imu", imu.out), ("colour", colour.video)):
        out = built.create(dai.node.XLinkOut)
        out.setStreamName(name)
        source.link(out.input)
    return built


def open_device(built):
    """The OAK, once the service has let it go -- which takes a few seconds."""
    for _attempt in range(30):
        try:
            return dai.Device(built, maxUsbSpeed=dai.UsbSpeed.HIGH)
        except RuntimeError as error:
            print(f"waiting for the device: {error}", flush=True)
            time.sleep(2)
    raise SystemExit("the OAK never came free; is the depth service stopped?")


def main() -> int:
    folder = sys.argv[1] if len(sys.argv) > 1 else "/tmp/sweep"
    os.makedirs(folder, exist_ok=True)
    meta = []
    with open_device(pipeline()) as device:
        readers = Readers(device)
        time.sleep(4.0)                          # exposure and focus
        call("look_at", {"pan": POSES[0][0] - 5, "tilt": POSES[0][1] - 5})
        time.sleep(2.0)
        previous_end = time.time()
        for index, (pan, tilt) in enumerate(POSES):
            if index == PAN_FROM:                # onto the pan sweep, from below
                call("look_at", {"pan": pan - 5, "tilt": tilt - 5})
                time.sleep(2.0)
                call("look_at", {"pan": pan - 5, "tilt": tilt})
                time.sleep(2.0)
                previous_end = time.time()
            call("look_at", {"pan": pan, "tilt": tilt})
            time.sleep(SETTLE_S)
            hold_from = time.time()
            time.sleep(HOLD_S)
            hold_to = time.time()
            picture = call("camera_jpeg")
            while readers.frame_at < hold_from:  # a colour frame of this pose
                time.sleep(0.02)
            with open(f"{folder}/{index:02d}-gimbal.jpg", "wb") as handle:
                handle.write(base64.b64decode(picture["jpeg_base64"]))
            cv2.imwrite(f"{folder}/{index:02d}-oak.png", readers.frame)
            still = readers.between(hold_from, hold_to)
            moving = readers.between(previous_end, hold_from)
            meta.append({"index": index, "pan": pan, "tilt": tilt,
                         "acc": still[:, 0:3].mean(axis=0).tolist(),
                         "gyro_bias": still[:, 3:6].mean(axis=0).tolist(),
                         "moving": moving[:, [3, 4, 5, 7]].tolist(),
                         "lens_position": readers.lens_position})
            previous_end = hold_to
            print(index, pan, tilt, len(still), readers.lens_position, flush=True)
        readers.stop.set()
        call("center_camera")
    with open(f"{folder}/meta.json", "w", encoding="utf-8") as handle:
        json.dump({"pan_from": PAN_FROM, "stops": meta}, handle)
    print(f"{len(meta)} stops kept in {folder}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
