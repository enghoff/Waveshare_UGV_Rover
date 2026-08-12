"""Live colour preview from the OAK-D-Lite's centre camera, optionally with the
stereo depth aligned to it and blended on top.

    python preview_rgb.py           # colour only, 30 fps
    python preview_rgb.py --depth   # colour + aligned depth, 15 fps

Keys: q quits. With --depth, m cycles the view (blend / depth / colour) and
[ / ] change the blend weight; move the mouse to read the depth under the
cursor in millimetres.

--depth does its alignment in two calls. `StereoDepth.setDepthAlign(CAM_A)`
warps the disparity out of the mono pair's geometry into the colour camera's,
and `setOutputSize` emits it at the colour preview's resolution -- so
depth[y, x] and rgb[y, x] are the same ray and the two can be blended directly,
with no host-side remapping.

Two consequences worth knowing:

* Aligning moves the depth origin. Depth here is measured from the *colour*
  camera's optical centre, not the right mono camera's as in `preview_depth.py`.
* The mono pair sees 73 deg horizontally against the colour camera's 69, so the
  warp covers the whole colour frame. Aligning the other way would not.

Sizing is set by the USB2 link, which saturates near 40 MB/s -- see docs/oak-usb-link.md.
960x540 at 15 fps fills it with both streams intact; asking for 25 fps or 720p
gets throttled back to the same ceiling for no gain. Colour on its own has the
link to itself and holds 30.
"""

import argparse
import sys

import cv2
import depthai as dai
import numpy as np

PREVIEW_SIZE = (960, 540)
RGB_FPS = 30
RGBD_FPS = 15  # both streams over one USB2 link; see the note above
DEPTH_RANGE_MM = (200, 4000)
# This link only ever negotiates USB2, and the USB3-enabled firmware fails to
# come back up on the bus after boot roughly six times in seven. See docs/oak-usb-link.md.
MAX_USB_SPEED = dai.UsbSpeed.HIGH
# OV7251 mono sensors run at 640x480 natively; 400_P would crop.
MONO_RES = dai.MonoCameraProperties.SensorResolution.THE_480_P

MODES = ["blend", "depth", "colour"]


def build_pipeline(with_depth, fps):
    width, height = PREVIEW_SIZE
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setPreviewSize(width, height)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setFps(fps)

    sources = [("rgb", cam.preview)]

    if with_depth:
        left = pipeline.create(dai.node.MonoCamera)
        left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        left.setResolution(MONO_RES)
        left.setFps(fps)

        right = pipeline.create(dai.node.MonoCamera)
        right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        right.setResolution(MONO_RES)
        right.setFps(fps)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(width, height)
        left.out.link(stereo.left)
        right.out.link(stereo.right)

        sources.append(("depth", stereo.depth))

    for name, source in sources:
        out = pipeline.create(dai.node.XLinkOut)
        out.setStreamName(name)
        source.link(out.input)

    return pipeline


def colourise(depth: np.ndarray) -> np.ndarray:
    """uint16 millimetres -> BGR, near = red, far = blue, invalid = black."""
    lo, hi = DEPTH_RANGE_MM
    scaled = ((np.clip(depth, lo, hi) - lo) / (hi - lo) * 255).astype(np.uint8)
    colour = cv2.applyColorMap(255 - scaled, cv2.COLORMAP_TURBO)
    colour[depth == 0] = 0
    return colour


def run_rgb(device):
    window = "OAK-D-Lite rgb"
    queue = device.getOutputQueue("rgb", maxSize=4, blocking=False)

    while True:
        cv2.imshow(window, queue.get().getCvFrame())
        if cv2.waitKey(1) == ord("q"):
            break


def run_rgbd(device):
    window = "OAK-D-Lite rgb+depth"
    queues = {n: device.getOutputQueue(n, maxSize=4, blocking=False)
              for n in ("rgb", "depth")}

    state = {"cursor": (0, 0), "mode": 0, "alpha": 0.5}

    def on_mouse(event, x, y, flags, param):
        state["cursor"] = (x, y)

    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)

    # The two streams are produced by different nodes, so pair them by sequence
    # number rather than assuming they arrive in lockstep -- one dropped frame
    # would otherwise skew colour against depth for the rest of the run.
    waiting = {"rgb": {}, "depth": {}}
    frame = {}
    # waitKey(1) cycles far faster than frames arrive at 15 fps, so only
    # re-blend when something has actually changed -- otherwise the loop burns
    # a core recolouring the same pair sixty times over.
    dirty, last_cursor = False, state["cursor"]

    while True:
        for name, queue in queues.items():
            packet = queue.tryGet()
            if packet is None:
                continue
            other = "depth" if name == "rgb" else "rgb"
            seq = packet.getSequenceNum()
            mate = waiting[other].pop(seq, None)
            if mate is None:
                waiting[name][seq] = packet
                if len(waiting[name]) > 30:  # the other stream has stalled
                    waiting[name].clear()
                continue
            pair = {name: packet, other: mate}
            frame = {"rgb": pair["rgb"].getCvFrame(),
                     "depth": pair["depth"].getFrame()}
            dirty = True

        if dirty and frame:
            rgb, depth = frame["rgb"], frame["depth"]
            mode = MODES[state["mode"]]
            if mode == "colour":
                view = rgb.copy()
            elif mode == "depth":
                view = colourise(depth)
            else:
                view = cv2.addWeighted(rgb, 1 - state["alpha"],
                                       colourise(depth), state["alpha"], 0)
                # Nothing to show where there is no disparity: keep the colour.
                view[depth == 0] = rgb[depth == 0]

            x, y = state["cursor"]
            if 0 <= y < depth.shape[0] and 0 <= x < depth.shape[1]:
                mm = int(depth[y, x])
                label = f"{mm} mm" if mm else "no depth"
                cv2.putText(view, f"{label}   {mode}  a={state['alpha']:.1f}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (255, 255, 255), 2)

            cv2.imshow(window, view)
            dirty = False

        key = cv2.waitKey(1)
        if key == ord("q"):
            break
        if key == ord("m"):
            state["mode"] = (state["mode"] + 1) % len(MODES)
            dirty = True
        if key == ord("["):
            state["alpha"] = max(0.0, state["alpha"] - 0.1)
            dirty = True
        if key == ord("]"):
            state["alpha"] = min(1.0, state["alpha"] + 0.1)
            dirty = True
        if state["cursor"] != last_cursor:  # the readout tracks the mouse
            last_cursor, dirty = state["cursor"], True


def main():
    parser = argparse.ArgumentParser(
        description="Live colour preview from the OAK-D-Lite's centre camera, "
                    "optionally with the stereo depth blended on top."
    )
    parser.add_argument(
        "--depth", action="store_true",
        help="also stream the stereo depth, aligned to CAM_A and blended over "
             "the colour frame",
    )
    parser.add_argument(
        "--fps", type=int,
        help=f"frame rate (default {RGB_FPS}, or {RGBD_FPS} with --depth)",
    )
    args = parser.parse_args()

    fps = args.fps or (RGBD_FPS if args.depth else RGB_FPS)
    pipeline = build_pipeline(args.depth, fps)

    with dai.Device(pipeline, maxUsbSpeed=MAX_USB_SPEED) as device:
        print(f"{device.getDeviceName()} on USB {device.getUsbSpeed().name}"
              f" -- {'rgb+depth' if args.depth else 'rgb'} at {fps} fps")
        if args.depth:
            run_rgbd(device)
        else:
            run_rgb(device)

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
