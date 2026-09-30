#!/usr/bin/env python3
"""The colour lens arithmetic, checked against what the rover's own OAK printed.

No camera needed: the numbers below were read off the device on 2026-09-30, and
what is checked is that `colour_lens` reproduces depthai where depthai is right
and departs from it by exactly the window where it is not.

    python oak_depth/test_colour_lens.py
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import colour_lens                                                 # noqa: E402

#: `getDefaultIntrinsics(CAM_A)` on this unit: the calibration as stored, for the
#: whole 4208 x 3120 sensor.
FULL = [[3001.77099609375, 0.0, 2111.37841796875],
        [0.0, 3001.005615234375, 1624.1234130859375],
        [0.0, 0.0, 1.0]]
FULL_SIZE = (4208, 3120)
#: And what `getCameraIntrinsics(CAM_A, 640, 360)` returned, which is what the
#: depth service published until this was found.
DEPTHAI_640 = {"fx": 456.54, "fy": 456.43, "cx": 321.12, "cy": 189.75}
#: What the colour picture measured against the right mono camera, fitted with
#: the stored distortion applied (the service treats the lens as a pinhole, which
#: moves this by about a percent).
MEASURED_640 = {"fx": 499.59, "fy": 497.42}

FAILED = []


def check(name, got, want, within=0.0):
    if isinstance(want, (int, float)) and not isinstance(want, bool):
        ok = abs(got - want) <= within
    else:
        ok = got == want
    print(f"  {'ok  ' if ok else 'FAIL'} {name}" + ("" if ok else f": got {got!r}, want {want!r}"))
    if not ok:
        FAILED.append(name)


def main() -> int:
    full_width = colour_lens.intrinsics(FULL, FULL_SIZE, None, (640, 360), (640, 360))
    for key, want in DEPTHAI_640.items():
        check(f"a full-width mode is depthai's own answer ({key})",
              full_width[key], want, 0.01)

    service = colour_lens.intrinsics(
        FULL, FULL_SIZE, colour_lens.SENSOR_WINDOW["THE_1080_P"],
        colour_lens.isp_size(colour_lens.MODE_SIZE["THE_1080_P"], (1, 3)),
        (640, 360))
    check("the service's picture is 640 x 360", (service["width"], service["height"]),
          (640, 360))
    check("its focal length is the 4K window's, a sixth of the sensor's",
          service["fx"], 500.30, 0.01)
    check("which is 9.6% longer than depthai said",
          round(service["fx"] / DEPTHAI_640["fx"], 3), 1.096, 0.001)
    check("and within 0.2% of what the pixels measured across",
          service["fx"] / MEASURED_640["fx"], 1.0, 0.002)
    check("and within 0.6% down, which is as well as 360 rows can say",
          service["fy"] / MEASURED_640["fy"], 1.0, 0.006)
    check("the window is centred, so the middle barely moves (cx)",
          service["cx"], 321.23, 0.01)
    check("(cy)", service["cy"], 190.69, 0.01)

    # The bench capture at 1280 x 720 does not scale the 1080p picture at all:
    # it cuts the middle out of it, so its focal length is 1080p's in pixels.
    crop = colour_lens.intrinsics(
        FULL, FULL_SIZE, colour_lens.SENSOR_WINDOW["THE_1080_P"],
        colour_lens.isp_size(colour_lens.MODE_SIZE["THE_1080_P"], (1, 1)),
        (1280, 720))
    check("a 1280 x 720 cut keeps 1080p's focal length", crop["fx"], 1500.89, 0.01)
    check("and moves the middle by the margin it cut away", crop["cx"], 643.69, 0.01)

    print(f"\n{len(FAILED)} failed")
    return 1 if FAILED else 0


if __name__ == "__main__":
    raise SystemExit(main())
