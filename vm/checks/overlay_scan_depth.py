#!/usr/bin/env python3
"""Reproject the OAK's depth into the lidar's scan plane and measure the disagreement.

This is the actual alignment instrument. Everything before it established that
both sensors produce data in a common frame; this asks whether they agree about
where things are, and answers with a number rather than an impression.

Method: every valid depth pixel becomes a 3D point using the camera's own
calibrated intrinsics from /oak/stereo/camera_info, is carried into base_link
through TF, and is kept if it lies within a thin slab at the lidar's scan plane
height. Per image column the surviving points collapse to one sample, giving a
"camera pseudo-scan" directly comparable to /scan over the mono pair's 73 deg
overlap. The residual is the radial difference at matched bearing.

Read the two failure signatures differently:

  a roughly constant radial offset  -> extrinsics; nudge cam_x / lidar_x
  a residual that grows with range  -> scale, i.e. stereo calibration
  a bow, small centre, large edges  -> rectification, and CAM_B's distortion
                                       coefficients are the first suspect
                                       (docs/oak-d-lite.md records k1=+41.95)
"""

import math
import sys

import cv2
import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, LaserScan

PLANE_TOL = 0.012      # metres; half-thickness of the slab at the scan plane
MAX_RANGE = 3.0
CANVAS = 900
N_SCANS = 10


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tf_to_Rt(tf):
    R = quat_to_R(tf.transform.rotation)
    t = np.array([tf.transform.translation.x,
                  tf.transform.translation.y,
                  tf.transform.translation.z])
    return R, t


class Overlay(Node):
    def __init__(self):
        super().__init__("overlay_scan_depth")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(Image, "/oak/stereo/image_raw", self.on_depth, 5)
        # The RIGHT camera's info, not the stereo node's. Unaligned depth
        # originates at the right mono camera, and /oak/stereo/camera_info
        # publishes 1280x720 colour intrinsics (fx=913) for a 640x480 depth
        # image -- a different resolution AND a different aspect ratio, so not
        # even a rescale. Using it put every pixel at the wrong height and the
        # residual came out at a metre.
        self.create_subscription(CameraInfo, "/oak/right/camera_info", self.on_info, 5)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.depth = None
        self.info = None
        self.scans = []

    def on_depth(self, msg):
        if self.depth is None and msg.encoding == "16UC1":
            buf = np.frombuffer(msg.data, dtype=np.uint16)
            self.depth = (buf.reshape(msg.height, msg.step // 2)[:, : msg.width].copy(),
                          msg.header.frame_id)

    def on_info(self, msg):
        if self.info is None:
            self.info = msg

    def on_scan(self, msg):
        if len(self.scans) < N_SCANS:
            self.scans.append((np.asarray(msg.ranges, dtype=np.float32),
                               msg.angle_min, msg.angle_increment,
                               msg.header.frame_id))


def lidar_profile(node, n=720):
    """Median lidar range on a fixed bearing grid, in base_link."""
    grid = np.full((len(node.scans), n), np.nan, dtype=np.float32)
    frame = node.scans[0][3]
    tf = node.buf.lookup_transform("base_link", frame, rclpy.time.Time())
    R, t = tf_to_Rt(tf)
    yaw = math.atan2(R[1, 0], R[0, 0])

    for row, (vals, amin, ainc, _) in enumerate(node.scans):
        vals = vals.astype(np.float64).copy()
        vals[~np.isfinite(vals)] = np.nan
        vals[vals <= 0.0] = np.nan
        b = amin + np.arange(len(vals)) * ainc
        x = t[0] + vals * np.cos(b + yaw)
        y = t[1] + vals * np.sin(b + yaw)
        bb = np.arctan2(y, x) % (2 * math.pi)
        rr = np.hypot(x, y)
        idx = np.rint(bb / (2 * math.pi / n)).astype(int) % n
        ok = np.isfinite(rr)
        grid[row, idx[ok]] = rr[ok]

    with np.errstate(all="ignore"):
        return np.nanmedian(grid, axis=0)


def main():
    rclpy.init()
    node = Overlay()
    for _ in range(240):
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.depth is not None and node.info is not None and len(node.scans) >= N_SCANS:
            break

    if node.depth is None or node.info is None or not node.scans:
        print(f"missing input: depth={node.depth is not None} "
              f"info={node.info is not None} scans={len(node.scans)}")
        return 1

    depth, cam_frame = node.depth
    K = np.asarray(node.info.k).reshape(3, 3)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    print(f"camera_info: {node.info.width}x{node.info.height} "
          f"fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}  frame={cam_frame}")
    print(f"depth image: {depth.shape[1]}x{depth.shape[0]}")

    # Intrinsics that do not belong to this image produce a plausible-looking
    # picture and a meaningless residual, so refuse rather than guess.
    if (node.info.width, node.info.height) != (depth.shape[1], depth.shape[0]):
        print("REFUSING: camera_info resolution does not match the depth image")
        return 1

    tf_cam = node.buf.lookup_transform("base_link", cam_frame, rclpy.time.Time())
    Rc, tc = tf_to_Rt(tf_cam)
    print(f"camera in base_link: t={np.round(tc, 3)}")

    tf_lid = node.buf.lookup_transform("base_link", node.scans[0][3], rclpy.time.Time())
    plane_z = tf_lid.transform.translation.z
    print(f"scan plane height: {plane_z:.4f} m\n")

    h, w = depth.shape
    Z = depth.astype(np.float64) / 1000.0
    valid = (Z > 0.2) & (Z < MAX_RANGE)

    u = np.arange(w)[None, :].repeat(h, 0)
    v = np.arange(h)[:, None].repeat(w, 1)
    P = np.stack([(u - cx) * Z / fx, (v - cy) * Z / fy, Z], axis=-1)
    B = P @ Rc.T + tc                       # into base_link

    in_slab = valid & (np.abs(B[:, :, 2] - plane_z) < PLANE_TOL)
    print(f"depth pixels valid: {valid.sum()} of {h * w} "
          f"({100.0 * valid.sum() / (h * w):.0f}%)")
    print(f"pixels inside the {2000 * PLANE_TOL:.0f} mm slab: {in_slab.sum()}")

    # One sample per column: the camera's pseudo-scan.
    cam_pts = []
    for col in range(w):
        rows = np.nonzero(in_slab[:, col])[0]
        if rows.size:
            cam_pts.append((float(np.median(B[rows, col, 0])),
                            float(np.median(B[rows, col, 1]))))
    cam_pts = np.asarray(cam_pts)
    if cam_pts.size == 0:
        print("no camera points landed in the scan plane -- check cam_z / lidar_z")
        return 1
    print(f"camera pseudo-scan samples: {len(cam_pts)}\n")

    prof = lidar_profile(node)
    nb = len(prof)

    cam_b = np.arctan2(cam_pts[:, 1], cam_pts[:, 0])
    cam_r = np.hypot(cam_pts[:, 0], cam_pts[:, 1])
    idx = np.rint((cam_b % (2 * math.pi)) / (2 * math.pi / nb)).astype(int) % nb
    lid_r = prof[idx]

    good = np.isfinite(lid_r) & (lid_r > 0.2) & (lid_r < MAX_RANGE)
    resid = (cam_r[good] - lid_r[good]) * 1000.0
    print(f"matched bearings: {good.sum()}")
    if good.sum() >= 10:
        print(f"residual (camera - lidar):  median {np.median(resid):+7.1f} mm   "
              f"RMS {np.sqrt(np.mean(resid ** 2)):6.1f} mm")
        print(f"{'range band':>12} {'n':>5} {'median':>9} {'RMS':>8}")
        for lo, hi in ((0.3, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 3.0)):
            m = (lid_r[good] >= lo) & (lid_r[good] < hi)
            if m.sum() >= 5:
                print(f"{lo:5.1f}-{hi:4.1f} m {m.sum():5d} "
                      f"{np.median(resid[m]):+8.1f} {np.sqrt(np.mean(resid[m] ** 2)):8.1f}")
        # A bow shows as centre and edges disagreeing.
        centre = np.abs(np.degrees(cam_b[good])) < 10
        edge = np.abs(np.degrees(cam_b[good])) > 25
        if centre.sum() >= 5 and edge.sum() >= 5:
            print(f"\ncentre (<10 deg): {np.median(resid[centre]):+7.1f} mm   "
                  f"edges (>25 deg): {np.median(resid[edge]):+7.1f} mm")

    # Top-down picture, same orientation as lidar_view.py: rover forward is up.
    canvas = np.zeros((CANVAS, CANVAS, 3), np.uint8)
    c = CANVAS // 2
    scale = (CANVAS / 2 - 40) / MAX_RANGE

    def to_px(x, y):
        return int(c - y * scale), int(c - x * scale)

    for r in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0):
        cv2.circle(canvas, (c, c), int(r * scale), (40, 40, 40), 1)
        cv2.putText(canvas, f"{r:.1f}m", (c + 4, c - int(r * scale) + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)
    cv2.line(canvas, (c, c), (c, 30), (60, 110, 60), 1)

    bg = np.arange(nb) * (2 * math.pi / nb)
    for r, b in zip(prof, bg):
        if np.isfinite(r) and 0.05 < r <= MAX_RANGE:
            px, py = to_px(r * math.cos(b), r * math.sin(b))
            if 0 <= px < CANVAS and 0 <= py < CANVAS:
                cv2.circle(canvas, (px, py), 2, (230, 230, 230), -1)

    for x, y in cam_pts:
        px, py = to_px(x, y)
        if 0 <= px < CANVAS and 0 <= py < CANVAS:
            cv2.circle(canvas, (px, py), 2, (60, 160, 255), -1)

    cv2.putText(canvas, "white: lidar /scan", (12, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)
    cv2.putText(canvas, "orange: OAK depth at the scan plane", (12, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 160, 255), 1)
    cv2.imwrite("/tmp/overlay.png", canvas)
    print("\nwrote /tmp/overlay.png")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
