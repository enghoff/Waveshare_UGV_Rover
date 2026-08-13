#!/usr/bin/env python3
"""Save one rectified-right frame, as an independent check on the scan.

The scan says there is a narrow object slightly right of straight ahead and
another narrow one to the left. If the camera agrees on which side things are,
the lidar transform is not mirrored -- and it is an independent sensor, so it
cannot be fooled by the same TF error.
"""

import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Grab(Node):
    def __init__(self):
        super().__init__("grab_frame")
        self.create_subscription(Image, "/oak/right/image_rect", self.cb, 10)
        self.frame = None

    def cb(self, msg):
        if self.frame is not None:
            return
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = buf.reshape(msg.height, msg.step)[:, : msg.width]
        self.frame = img.copy()
        self.get_logger().info(f"got {msg.width}x{msg.height} {msg.encoding}")


def main():
    rclpy.init()
    node = Grab()
    for _ in range(120):
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.frame is not None:
            break
    if node.frame is None:
        print("no frame")
        return 1

    img = node.frame
    # Brighten: the scene is dim and the check is about where things are.
    img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # Mark image centre and the bearings the scan reported, using the mono
    # camera's 73 deg horizontal FOV to map bearing -> column.
    h, w = img.shape
    fx = (w / 2.0) / np.tan(np.deg2rad(73.0) / 2.0)
    cv2.line(vis, (w // 2, 0), (w // 2, h), (0, 255, 0), 1)
    for bearing, label in ((-1.7, "bottle?"), (23.6, "can?")):
        # +bearing is to the rover's LEFT, which is -x in image columns.
        u = int(w / 2.0 - fx * np.tan(np.deg2rad(bearing)))
        if 0 <= u < w:
            cv2.line(vis, (u, 0), (u, h), (0, 128, 255), 1)
            cv2.putText(vis, label, (max(0, u - 30), 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 255), 1)

    cv2.imwrite("/tmp/rect.png", vis)
    print(f"wrote /tmp/rect.png  {w}x{h}  fx~{fx:.0f}px")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
