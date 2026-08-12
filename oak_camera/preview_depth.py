"""Live stereo depth from the OAK-D-Lite, colour-mapped, with a distance readout.

Move the mouse over the window to read the depth of that pixel in millimetres.
Press q to quit.

Depth is distance along the optical axis from the right mono camera's optical
centre, not from the front of the housing. Zero means the pixel has no valid
disparity: it is closer than the stereo pair can triangulate, textureless, or
occluded in one of the two views. The 7.5 cm baseline sets the useful range --
below roughly 20 cm the two views stop overlapping.
"""

import cv2
import depthai as dai
import numpy as np

# OV7251 mono sensors run at 640x480 natively; 400_P would crop.
MONO_RES = dai.MonoCameraProperties.SensorResolution.THE_480_P
FPS = 15
DEPTH_RANGE_MM = (200, 4000)
# USB2-only firmware; the USB3 build usually fails to boot on this link. See docs/oak-usb-link.md.
MAX_USB_SPEED = dai.UsbSpeed.HIGH

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

cursor = [0, 0]


def on_mouse(event, x, y, flags, param):
    cursor[0], cursor[1] = x, y


with dai.Device(pipeline, maxUsbSpeed=MAX_USB_SPEED) as device:
    print(f"{device.getDeviceName()} on USB {device.getUsbSpeed().name}")
    queue = device.getOutputQueue("depth", maxSize=4, blocking=False)

    cv2.namedWindow("OAK-D-Lite depth")
    cv2.setMouseCallback("OAK-D-Lite depth", on_mouse)

    lo, hi = DEPTH_RANGE_MM
    while True:
        depth = queue.get().getFrame()  # uint16, millimetres

        scaled = ((np.clip(depth, lo, hi) - lo) / (hi - lo) * 255).astype(np.uint8)
        colour = cv2.applyColorMap(255 - scaled, cv2.COLORMAP_TURBO)
        colour[depth == 0] = 0

        x, y = cursor
        if 0 <= y < depth.shape[0] and 0 <= x < depth.shape[1]:
            mm = int(depth[y, x])
            cv2.putText(
                colour, f"{mm} mm" if mm else "no depth", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
            )

        cv2.imshow("OAK-D-Lite depth", colour)
        if cv2.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()
