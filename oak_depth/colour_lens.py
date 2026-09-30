"""What a pixel of the colour camera means, in the mode it is actually run in.

The stored calibration describes the whole IMX214, 4208 x 3120 pixels, and depthai
offers `getCameraIntrinsics(socket, width, height)` to turn it into any output
size. **That call knows the size and not the mode.** It scales the calibration by
the width and trims the height to the new shape, which is only right for a mode
that reads the sensor's full width -- and the 1080p mode this rover runs does not.
It reads the middle 3840 x 2160 of the sensor and bins it, so every lens this
component published until 2026-09-30 was 9.6% too short in focal length: 456.5
pixels at 640 x 360 where the pixels obey 500.3, a 70.1-degree field of view
quoted for a picture that takes in 65.2.

Measured rather than read off a datasheet. Matching the colour picture against
the right mono camera -- whose native 640 x 480 has no mode to get wrong -- put
the colour focal length at 499.6 x 497.4 with a residual of 0.11 degrees, against
500.3 x 500.2 from the window below; the lens depthai returned left 0.19 degrees
over a third of the matches and could not hold the rest. The same factor is what
the OAK mount measurement of 2026-09-07 could not account for: its forward offset
moved 12.4 mm between two target distances 0.13 m apart, which is 9.6% of 0.13 m.

The device's own depth alignment was never wrong: the firmware knows the window,
and depth edges sit on colour edges out to the corners. Only the numbers handed
to the host were.

Plain arithmetic on plain numbers, so it can be checked without a camera; the one
function that touches depthai is `colour_intrinsics` at the bottom.
"""
from __future__ import annotations

#: The part of the sensor each colour mode reads, in full-resolution pixels,
#: centred on it. 1080p is the 4K window binned two by two, which is what the
#: measurement above found (a focal length 1.094 times depthai's, against the
#: 1.096 that 4208 over 3840 predicts). A mode missing here is assumed to read
#: the full width, which is what depthai assumes for all of them.
SENSOR_WINDOW = {
    "THE_1080_P": (3840, 2160),
    "THE_4_K": (3840, 2160),
}


def intrinsics(full: list, full_size: tuple[int, int],
               window: tuple[int, int] | None,
               isp_size: tuple[int, int],
               output_size: tuple[int, int]) -> dict:
    """The lens of a picture cut from the calibrated sensor in three steps.

    `full` is the calibration's own 3x3 matrix at `full_size`. The mode reads
    `window` out of the middle of the sensor (the full width, trimmed to the
    window's shape, when None); the ISP scales that to `isp_size`; and the output
    is the middle `output_size` of what the ISP made -- which is what
    `setVideoSize` does when it is asked for less than the ISP produces. Scaling
    multiplies the principal point exactly as depthai does, with no half-pixel
    term, so a mode that reads the full width comes out identical to
    `getCameraIntrinsics`.
    """
    full_w, full_h = full_size
    if window is None:
        window = (full_w, round(full_w * isp_size[1] / isp_size[0]))
    win_w, win_h = window
    scale = isp_size[0] / float(win_w)
    fx = float(full[0][0]) * scale
    fy = float(full[1][1]) * scale
    cx = (float(full[0][2]) - (full_w - win_w) / 2.0) * scale
    cy = (float(full[1][2]) - (full_h - win_h) / 2.0) * scale
    out_w, out_h = output_size
    cx -= (isp_size[0] - out_w) / 2.0
    cy -= (isp_size[1] - out_h) / 2.0
    return {"fx": round(fx, 2), "fy": round(fy, 2),
            "cx": round(cx, 2), "cy": round(cy, 2),
            "width": out_w, "height": out_h}


def isp_size(mode_size: tuple[int, int], scale: tuple[int, int]) -> tuple[int, int]:
    """What the ISP makes of a mode, at `setIspScale(numerator, denominator)`."""
    numerator, denominator = scale
    return (mode_size[0] * numerator // denominator,
            mode_size[1] * numerator // denominator)


#: The size each colour mode delivers before any ISP scaling.
MODE_SIZE = {"THE_1080_P": (1920, 1080), "THE_4_K": (3840, 2160)}


def colour_intrinsics(calibration, dai, mode: str,
                      scale: tuple[int, int], output_size: tuple[int, int]) -> dict:
    """The colour camera's lens for this mode, ISP scale and output size.

    `calibration` is `device.readCalibration()`. The full-sensor matrix comes
    from `getDefaultIntrinsics`, which is the calibration as it was stored rather
    than anything already resized.
    """
    full, full_w, full_h = calibration.getDefaultIntrinsics(
        dai.CameraBoardSocket.CAM_A)
    return intrinsics(full, (int(full_w), int(full_h)), SENSOR_WINDOW.get(mode),
                      isp_size(MODE_SIZE[mode], scale), output_size)
