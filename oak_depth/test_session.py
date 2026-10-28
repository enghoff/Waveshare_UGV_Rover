#!/usr/bin/env python3
"""A recording session's depth and IMU, without a camera.

    python3 oak_depth/test_session.py

The depth service writes depth and the OAK's IMU into a session's directory
while the daemon announces one (rover_daemon/rover_recording.py). This checks
what can be checked without the device: that a session's pipeline carries the
IMU and an ordinary one does not, which needs depthai but no camera; that a
session announced while the device runs asks for it to be reopened; and what the
two streams write from packets made by hand. Running the two together on the
OAK is commissioning's.
"""
import datetime
import json
import os
import sys
import tempfile
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import depth_server  # noqa: E402
import session_record  # noqa: E402

FAILED = []


def check(label, got, want):
    ok = got == want
    print(("  ok    " if ok else "  FAIL  ") + label + ("" if ok else f": got {got!r}, want {want!r}"))
    if not ok:
        FAILED.append(label)


class Stamped:
    def __init__(self, host_s, device_s):
        self._host = datetime.timedelta(seconds=host_s)
        self._device = datetime.timedelta(seconds=device_s)

    def getTimestamp(self):
        return self._host

    def getTimestampDevice(self):
        return self._device


class Frame(Stamped):
    def __init__(self, host_s, device_s, value):
        super().__init__(host_s, device_s)
        import numpy as np
        self._frame = np.full((180, 320), value, dtype=np.uint16)

    def getFrame(self):
        return self._frame


class Vector(Stamped):
    def __init__(self, host_s, device_s, x, y, z):
        super().__init__(host_s, device_s)
        self.x, self.y, self.z = x, y, z


class Report:
    def __init__(self, t):
        self.acceleroMeter = Vector(t, t - 100, 0.08, 9.28, -2.13)
        self.gyroscope = Vector(t, t - 100, -0.002, -0.008, -0.003)


class Batch:
    def __init__(self, ts):
        self.packets = [Report(t) for t in ts]


class OneBatch:
    """An IMU queue that gives one batch and then reports the device closed."""

    def __init__(self, batch):
        self.batch = batch

    def get(self):
        if self.batch is None:
            raise RuntimeError("closed")
        batch, self.batch = self.batch, None
        return batch


def main():
    try:
        depth = depth_server.Depth()
    except depth_server.DepthError as error:
        print("  skip  no depthai here: %s" % error)
        return 0
    depth.imu_wanted = False
    depth._pipeline()
    check("an ordinary pipeline carries no IMU", depth.pipeline_has_imu, False)
    depth.imu_wanted = True
    depth._pipeline()
    check("a session's pipeline carries the IMU, built by this depthai", depth.pipeline_has_imu,
          True)

    with tempfile.TemporaryDirectory() as root:
        directory = os.path.join(root, "2026-10-28T1200-commissioning")
        os.makedirs(directory)
        depth.device_open, depth.pipeline_has_imu, depth.imu_wanted = True, False, False
        streams = depth._session_opened({"dir": directory, "depth_hz": 5.0})
        check("a session announced while the device runs without the IMU reopens it",
              (depth.rebuilding, depth._stop.is_set(), depth.imu_wanted), (True, True, True))
        depth.rebuilding = False
        depth._stop.clear()
        depth._opened_for_recording()
        for n in range(6):
            depth._record_depth(Frame(100.0 + n / 15, 5.0 + n / 15, n))
        depth._drain_imu(OneBatch(Batch([100.000, 100.005])))
        counts = depth._session_closed(streams)
        check("depth at 5 a second keeps every third frame of 15, and the IMU every report",
              counts, {"depth": {"written": 2, "dropped": 0}, "oak_imu": {"written": 2, "dropped": 0}})
        lines = [json.loads(l) for l in open(os.path.join(directory, "depth", "index.jsonl"))]
        check("the reopening for the IMU is a restart: the device's clock began again",
              lines[1], {"restart": True, "epoch": 1, "seq": 0, "t_arr": lines[1]["t_arr"]})
        check("each depth record carries the host-mapped stamp and the device's own",
              (lines[2]["t_dev"], lines[2]["t_dev_raw"], lines[2]["width"], lines[2]["height"]),
              (100_000_000_000, 5_000_000_000, 320, 180))
        size = os.path.getsize(os.path.join(directory, "depth", "frames.u16"))
        check("...and its frame is 320 x 180 little-endian 16-bit millimetres", size,
              2 * 320 * 180 * 2)
        imu = [json.loads(l) for l in open(os.path.join(directory, "oak_imu.jsonl"))]
        check("the IMU restarts with the device too",
              imu[1].get("restart"), True)
        check("...and its records carry DepthAI's conversion, rad/s and m/s^2",
              (imu[2]["gyro"], imu[2]["accel"]), ([-0.002, -0.008, -0.003], [0.08, 9.28, -2.13]))
    print("\n%d failed" % len(FAILED))
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
