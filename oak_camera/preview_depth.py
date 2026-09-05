"""Live stereo depth from the OAK-D-Lite, colour-mapped, with a distance readout.

    python preview_depth.py             # invalid pixels keep their last depth for 1 s
    python preview_depth.py --fade 2.5  # ...or for 2.5 s
    python preview_depth.py --fade 0    # no holding: each frame stands alone

Move the mouse over the window to read the depth of that pixel in millimetres.
Press q to quit.

Depth is distance along the optical axis from the right mono camera's optical
centre, not from the front of the housing. Zero means the pixel has no valid
disparity: it is closer than the stereo pair can triangulate, textureless, or
occluded in one of the two views. The 7.5 cm baseline sets the useful range --
below roughly 20 cm the two views stop overlapping.

Those zeros are rarely still: on a textureless wall the valid pixels flicker in
and out frame to frame, and a view of one frame on its own strobes. So by
default a pixel that goes invalid holds its last depth and darkens to black over
the fade time -- a pixel that recovers within the window never blinks, one that
is genuinely gone still disappears -- and the readout under the cursor marks a
held value with its age, so nothing stale reads as fresh. `--fade 0` turns the
holding off and shows each frame exactly as the device reported it.
"""

import argparse
import sys
import time

import cv2
import depthai as dai
import numpy as np

from depth_colour import colourise

# OV7251 mono sensors run at 640x480 natively; 400_P would crop.
MONO_RES = dai.MonoCameraProperties.SensorResolution.THE_480_P
FPS = 15
DEFAULT_FADE_S = 1.0
# USB2-only firmware; the USB3 build usually fails to boot on this link. See docs/oak-usb-link.md.
MAX_USB_SPEED = dai.UsbSpeed.HIGH

WINDOW = "OAK-D-Lite depth"


def build_pipeline():
    pipeline = dai.Pipeline()

    left = pipeline.create(dai.node.MonoCamera)
    left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    left.setResolution(MONO_RES)
    left.setFps(FPS)

    right = pipeline.create(dai.node.MonoCamera)
    right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    right.setResolution(MONO_RES)
    right.setFps(FPS)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    left.out.link(stereo.left)
    right.out.link(stereo.right)

    out = pipeline.create(dai.node.XLinkOut)
    out.setStreamName("depth")
    stereo.depth.link(out.input)

    return pipeline




class Fader:
    """Holds the last valid depth per pixel and ages it out over `seconds`.

    Held depths are what the view and the readout both use; `age` is seconds
    since that pixel was last refreshed by the device, and drives the fade to
    black. A pixel refreshed every frame never ages, so a fully valid frame
    renders exactly as it would without fading.
    """

    def __init__(self, seconds: float):
        self.seconds = seconds
        self.depth = None
        self.age = None
        self.last = None

    def update(self, depth: np.ndarray) -> np.ndarray:
        """Fold a new frame in and return the held depth map."""
        now = time.monotonic()
        if self.depth is None or self.depth.shape != depth.shape:
            self.depth = depth.copy()
            self.age = np.zeros(depth.shape, np.float32)
            self.last = now
            return self.depth

        # Wall-clock, not frame count: a stalled link should still fade out
        # rather than freeze the last frame on screen indefinitely.
        dt, self.last = now - self.last, now
        fresh = depth != 0
        self.age += dt
        self.age[fresh] = 0.0
        self.depth[fresh] = depth[fresh]
        # Past the window a pixel is no different from one never seen.
        self.depth[self.age > self.seconds] = 0
        return self.depth

    def shade(self, colour: np.ndarray) -> np.ndarray:
        """Darken each pixel towards black in proportion to its age."""
        weight = 1.0 - np.clip(self.age / self.seconds, 0.0, 1.0)
        return (colour * weight[..., None]).astype(np.uint8)

    def age_at(self, y: int, x: int) -> float:
        return float(self.age[y, x])


def main():
    parser = argparse.ArgumentParser(
        description="Live stereo depth from the OAK-D-Lite's mono pair, "
                    "colour-mapped, with a distance readout."
    )
    parser.add_argument(
        "--fade", type=float, default=DEFAULT_FADE_S, metavar="SECONDS",
        help="hold the last valid depth for a pixel the device stops "
             f"reporting, fading it to black over SECONDS (default "
             f"{DEFAULT_FADE_S:g}); 0 turns the holding off, so invalid pixels "
             "are black immediately",
    )
    args = parser.parse_args()

    if args.fade < 0:
        parser.error("--fade needs a positive number of seconds, or 0 to disable")

    fader = Fader(args.fade) if args.fade else None
    cursor = [0, 0]

    def on_mouse(event, x, y, flags, param):
        cursor[0], cursor[1] = x, y

    with dai.Device(build_pipeline(), maxUsbSpeed=MAX_USB_SPEED) as device:
        print(f"{device.getDeviceName()} on USB {device.getUsbSpeed().name}"
              f" -- depth at {FPS} fps"
              f"{f', {args.fade:g} s fade' if fader else ', no fade'}")
        queue = device.getOutputQueue("depth", maxSize=4, blocking=False)

        cv2.namedWindow(WINDOW)
        cv2.setMouseCallback(WINDOW, on_mouse)

        while True:
            depth = queue.get().getFrame()  # uint16, millimetres

            if fader:
                depth = fader.update(depth)
                view = fader.shade(colourise(depth))
            else:
                view = colourise(depth)

            x, y = cursor
            if 0 <= y < depth.shape[0] and 0 <= x < depth.shape[1]:
                mm = int(depth[y, x])
                label = f"{mm} mm" if mm else "no depth"
                if mm and fader:
                    age = fader.age_at(y, x)
                    if age > 0:
                        label += f"  ({age:.1f} s old)"
                cv2.putText(
                    view, label, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
                )

            cv2.imshow(WINDOW, view)
            if cv2.waitKey(1) == ord("q"):
                break

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
