#!/usr/bin/env python3
"""Sweep the gimbal's whole pan travel and record where it really went.

    ssh orin 'pkill -f "oak_depth/run_oak_depth[.]sh"'
    scp usb_cameras/capture_pan_sweep.py orin:/tmp/
    ssh orin 'PYTHONPATH=~/ugv/oak_depth/vendor python3 /tmp/capture_pan_sweep.py /tmp/pansweep'
    ssh orin '~/ugv/oak_depth/restart.sh'
    scp -r orin:/tmp/pansweep captures/YYYY-MM-DD-pan-sweep

The pan calibration of 2026-09-07 covers commanded pan -20 to +20 and nothing
beyond, so a look panned further keeps its picture and loses its direction. This
records what widening that needs, at the rest tilt the rover's looks use.

**The truth is the OAK's gyro, which rides the gimbal and owes the servo
nothing.** It streams throughout; each stop is held still long enough to read its
bias, and a pan step is the rate about gravity integrated across the move. Its
scale on this axis is not taken on trust: the sweep runs the whole circle, and a
picture at -180 and one at +180 look the same way, so the travel between them is
360 degrees plus whatever small angle the two pictures differ by. That fixes the
gyro's scale with no lens and no servo in it. The gimbal camera's picture at every
stop is kept as a second, independent account of each step.

Three passes at tilt 20, then one at tilt 0:

  A  development   ascending, -180 to +180 in tens
  C  backlash      descending, +180 to -180 in tens
  B  held out      ascending, the fives in between, ending at +180
  D  tilt 0        ascending, in tens

then a set of single moves from rest and back, for how long a pan takes.

Every ascending stop is reached from below, as the daemon's own aimed looks are.

**The pass rule, fixed before the first capture.** A candidate is the actual pan
pass A measured at each commanded stop, interpolated linearly between them. It is
judged on pass B, which it never saw: a commanded pan is inside the widened
envelope when every held-out stop from zero out to it, on that side, lands within
0.5 degrees of the candidate's prediction. That is the bound the ±20 envelope
passed with. The reference has to earn that judgement first: the closures of
passes A and B must agree on the gyro's scale within 0.3%, the camera's account
of each ten-degree step of A must agree with the gyro's to 0.3 degrees at the
95th percentile after one common scale, and the pictures at rest before and after
must agree within 0.2 degrees, or the rover moved. A reference that fails is
reported as inconclusive, not as a servo result. Nothing here writes a constant.

**The held-out session (passes H, K and R), and its rule, fixed before it ran.**
The first session's verdict was inconclusive on two gates that measured
something other than the reference: the camera's ten-degree steps carry 2.6% of
parallax from a lens ahead of the pan axis, and the rest pictures differed
because a return from +180 lands on the far side of the backlash. So:

  H  ascending, -180 to +180 in fives, each swing waited out
  K  descending, +170 to -180 in tens
  R  returns to rest from 30, ±180, ±135 and 90 degrees

The candidate is frozen in `pan_candidate_2026-10-09.json` (pass A of the first
session, tilt 20). The reference must pass two gates: closing the circle from
H's ends gives a gyro scale within 0.3% of the first session's 1.01824 (if the
two end pictures do not match here, the first session's scale is used and the
gate is reported as not taken); and at every ten-degree stop the gyro's
ascending-minus-descending (H against K) agrees with the pictures' to 0.25
degrees at the 95th percentile, which the pictures measure with no parallax
because the lens is in nearly the same place both times. A commanded pan is
inside the validated envelope when every H stop from zero out to it, on that
side, lands within 0.5 degrees of the candidate; the envelope adopted is the
smaller side. The daemon's return to rest passes when every R return lands
within 0.3 degrees of the first one, by the pictures.
"""
from __future__ import annotations

import base64
import json
import os
import socket
import sys
import threading
import time

import depthai as dai
import numpy as np

SETTLE_S = 1.2
HOLD_S = 0.8
DAEMON = ("127.0.0.1", 8769)
TENS = list(range(-180, 181, 10))
FIVES = list(range(-175, 180, 10)) + [180]
TIMING = (30, -30, 60, -60, 90, -90, 135, -135, 180, -180)
#: Every five degrees, for the held-out session's ascending pass.
EVERY_FIVE = list(range(-180, 181, 5))
#: Where the held-out session swings to before each return to rest, the first
#: one short enough that its return is the reference the others are judged by.
RETURNS = (30, 180, -180, 135, -135, 90, 180, 30)
#: A swing needs this long to land within 0.2 degrees, measured on 2026-10-09:
#: 0.49 s for 24 degrees up to 1.20 s for 179. A stop's settle is at least this
#: plus half a second, so no hold begins while the gimbal is still moving, which
#: the first session's did after every swing of 180.
SWING_FLOOR_S, SWING_DPS = 0.5, 200.0


def call(name, arguments=None):
    with socket.create_connection(DAEMON, 20) as link:
        stream = link.makefile("rwb")
        stream.write((json.dumps({"call": name, "arguments": arguments or {}})
                      + "\n").encode())
        stream.flush()
        return json.loads(stream.readline())


class Imu:
    """The OAK's accelerometer and gyro, drained on a thread."""

    def __init__(self, device):
        self.rows, self.lock = [], threading.Lock()
        self.stop = threading.Event()
        threading.Thread(target=self._drain,
                         args=(device.getOutputQueue("imu", 50, True),),
                         daemon=True).start()

    def _drain(self, queue):
        while not self.stop.is_set():
            for packet in queue.get().packets:
                a, g = packet.acceleroMeter, packet.gyroscope
                with self.lock:
                    self.rows.append((a.x, a.y, a.z, g.x, g.y, g.z, time.time(),
                                      g.getTimestampDevice().total_seconds()))

    def array(self):
        with self.lock:
            return np.array(self.rows)


def pipeline():
    built = dai.Pipeline()
    imu = built.create(dai.node.IMU)
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW,
                         dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(5)
    imu.setMaxBatchReports(20)
    out = built.create(dai.node.XLinkOut)
    out.setStreamName("imu")
    imu.out.link(out.input)
    return built


def open_device(built):
    for _attempt in range(30):
        try:
            return dai.Device(built, maxUsbSpeed=dai.UsbSpeed.HIGH)
        except RuntimeError as error:
            print(f"waiting for the device: {error}", flush=True)
            time.sleep(2)
    raise SystemExit("the OAK never came free; is the depth service stopped?")


class Sweep:
    def __init__(self, folder):
        self.folder, self.stops, self.moves = folder, [], []
        self.pan, self.tilt = 0, 20

    def go(self, pan, tilt, label):
        sent = time.time()
        answer = call("look_at", {"pan": pan, "tilt": tilt})
        self.moves.append({"label": label, "pan": pan, "tilt": tilt,
                           "sent": sent, "ok": bool(answer.get("ok"))})

    def settle_s(self, pan):
        swing = abs(pan - self.pan)
        return max(SETTLE_S, SWING_FLOOR_S + swing / SWING_DPS + 0.5)

    def stop(self, pan, tilt, label, picture=True):
        settle = self.settle_s(pan)
        self.go(pan, tilt, label)
        self.pan, self.tilt = pan, tilt
        time.sleep(settle)
        hold_from = time.time()
        time.sleep(HOLD_S)
        hold_to = time.time()
        name = None
        if picture:
            got = call("camera_jpeg")
            if got.get("jpeg_base64"):
                name = f"{len(self.stops):03d}-{label}-{pan:+04d}-{tilt}.jpg"
                with open(os.path.join(self.folder, name), "wb") as handle:
                    handle.write(base64.b64decode(got["jpeg_base64"]))
        self.stops.append({"index": len(self.stops), "label": label, "pan": pan,
                           "tilt": tilt, "hold_from": hold_from,
                           "hold_to": hold_to, "picture": name})
        print(len(self.stops) - 1, label, pan, tilt, name is not None, flush=True)

    def rest(self, label):
        settle = self.settle_s(-5) + 0.6
        sent = time.time()
        call("center_camera")
        self.moves.append({"label": label, "pan": 0, "tilt": 20, "sent": sent,
                           "ok": True, "centre": True})
        self.pan, self.tilt = 0, 20
        time.sleep(settle)
        hold_from = time.time()
        time.sleep(HOLD_S)
        got = call("camera_jpeg")
        name = f"{len(self.stops):03d}-{label}.jpg"
        with open(os.path.join(self.folder, name), "wb") as handle:
            handle.write(base64.b64decode(got.get("jpeg_base64", "")))
        self.stops.append({"index": len(self.stops), "label": label, "pan": 0,
                           "tilt": 20, "hold_from": hold_from,
                           "hold_to": time.time(), "picture": name})
        print(len(self.stops) - 1, label, flush=True)


def main() -> int:
    folder = sys.argv[1] if len(sys.argv) > 1 else "/tmp/pansweep"
    passes = sys.argv[2] if len(sys.argv) > 2 else "ACBDT"
    os.makedirs(folder, exist_ok=True)
    sweep = Sweep(folder)
    began = time.time()
    with open_device(pipeline()) as device:
        imu = Imu(device)
        time.sleep(2.0)
        try:
            sweep.rest("rest-start")
            if "A" in passes:
                sweep.stop(-180, 20, "A")
                for pan in TENS[1:]:
                    sweep.stop(pan, 20, "A")
            if "C" in passes:
                for pan in reversed(TENS[:-1]):
                    sweep.stop(pan, 20, "C")
            if "B" in passes:
                sweep.stop(-180, 20, "B")
                for pan in FIVES:
                    sweep.stop(pan, 20, "B")
            if "D" in passes:
                sweep.rest("rest-mid")
                sweep.stop(-180, 0, "D")
                for pan in TENS[1:]:
                    sweep.stop(pan, 0, "D")
            if "H" in passes:
                # Held out: ascending in fives, the swing out to -180 waited
                # for; then descending in tens for the backlash the pictures
                # can check the gyro against with no parallax in it.
                sweep.stop(-180, 20, "H")
                for pan in EVERY_FIVE[1:]:
                    sweep.stop(pan, 20, "H")
                for pan in reversed(TENS[:-1]):
                    sweep.stop(pan, 20, "K")
            if "R" in passes:
                for pan in RETURNS:
                    sweep.stop(pan, 20, "R-out", picture=False)
                    sweep.rest("R-back")
            sweep.rest("rest-end")
            if "T" in passes:
                for pan in TIMING:
                    sweep.stop(pan - 5, 20, "T-under", picture=False)
                    sweep.stop(pan, 20, "T", picture=False)
                    sweep.rest("T-back")
        finally:
            call("center_camera")
            time.sleep(1.5)
            imu.stop.set()
            np.save(os.path.join(folder, "imu.npy"), imu.array())
            with open(os.path.join(folder, "meta.json"), "w",
                      encoding="utf-8") as handle:
                json.dump({"began": began, "ended": time.time(),
                           "settle_s": SETTLE_S, "hold_s": HOLD_S,
                           "stops": sweep.stops, "moves": sweep.moves}, handle)
    print(f"{len(sweep.stops)} stops kept in {folder}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
