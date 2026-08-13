#!/usr/bin/env python3
"""Find discrete objects in the scan and report where they are in base_link.

This is the end-to-end check of the base_link -> lidar_link transform: it goes
through TF rather than applying the yaw by hand, so a wrong sign or a mirrored
scan shows up as objects in the wrong place rather than being quietly cancelled.

Bearings are reported in base_link, REP-103: 0 deg is straight ahead, +90 deg is
to the rover's LEFT, -90 deg to its right. Ranges are median-filtered across
several revolutions so a single dropout does not invent or erase a cluster.
"""

import math
import sys

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

MAX_RANGE = 3.0        # metres; beyond this is room, not the test objects
GAP = 0.12             # metres; a jump larger than this starts a new cluster
MIN_POINTS = 3
N_SCANS = 15


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScanProbe(Node):
    def __init__(self):
        super().__init__("scan_probe")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(LaserScan, "/scan", self.cb, 10)
        self.scans = []
        self.meta = None

    def cb(self, msg):
        if len(self.scans) < N_SCANS:
            self.scans.append((np.asarray(msg.ranges, dtype=np.float32),
                               msg.angle_min, msg.angle_increment))
            self.meta = msg


def main():
    rclpy.init()
    node = ScanProbe()
    while rclpy.ok() and len(node.scans) < N_SCANS:
        rclpy.spin_once(node, timeout_sec=1.0)

    msg = node.meta
    if msg is None:
        print("no scans received")
        return 1

    try:
        tf = node.buf.lookup_transform("base_link", msg.header.frame_id,
                                       rclpy.time.Time())
    except Exception as exc:                                  # noqa: BLE001
        print(f"TF lookup failed: {exc}")
        return 1

    tx = tf.transform.translation.x
    ty = tf.transform.translation.y
    yaw = yaw_of(tf.transform.rotation)
    print(f"TF {msg.header.frame_id} -> base_link: "
          f"t=({tx:.3f}, {ty:.3f})  yaw={math.degrees(yaw):.1f} deg")
    print(f"scan: {len(msg.ranges)} rays, angle_min={math.degrees(msg.angle_min):.1f}, "
          f"increment={math.degrees(msg.angle_increment):.3f} deg")

    # The driver emits a variable number of rays per revolution (504/503/502
    # observed), so the scans cannot be stacked by index. Rebin each onto a
    # fixed angular grid first, then take the median per bin.
    n = 504
    bearings = np.arange(n) * (2.0 * math.pi / n)
    grid = np.full((len(node.scans), n), np.nan, dtype=np.float32)
    for row, (vals, amin, ainc) in enumerate(node.scans):
        vals = vals.astype(np.float32).copy()
        vals[~np.isfinite(vals)] = np.nan
        vals[vals <= 0.0] = np.nan
        b = (amin + np.arange(len(vals)) * ainc) % (2.0 * math.pi)
        idx = np.rint(b / (2.0 * math.pi / n)).astype(int) % n
        grid[row, idx] = vals

    with np.errstate(all="ignore"):
        ranges = np.nanmedian(grid, axis=0)

    # Cluster over consecutive rays that are close in range.
    clusters, current = [], []
    for i in range(n):
        r = ranges[i]
        ok = np.isfinite(r) and 0.05 < r <= MAX_RANGE
        if ok and (not current or abs(r - ranges[current[-1]]) < GAP):
            current.append(i)
        else:
            if len(current) >= MIN_POINTS:
                clusters.append(current)
            current = [i] if ok else []
    if len(current) >= MIN_POINTS:
        clusters.append(current)

    print(f"\n{len(clusters)} clusters within {MAX_RANGE} m\n")
    print(f"{'bearing':>9} {'range':>7} {'pts':>4} {'width':>7}   position in base_link")
    print(f"{'(deg)':>9} {'(m)':>7} {'':>4} {'(mm)':>7}")

    rows = []
    for c in clusters:
        r = float(np.nanmedian(ranges[c]))
        # Transform through TF into base_link, then back to a bearing.
        xs = tx + ranges[c] * np.cos(bearings[c] + yaw)
        ys = ty + ranges[c] * np.sin(bearings[c] + yaw)
        xm, ym = float(np.nanmedian(xs)), float(np.nanmedian(ys))
        bear = math.degrees(math.atan2(ym, xm))
        width = float(np.hypot(xs[-1] - xs[0], ys[-1] - ys[0]) * 1000.0)
        side = "AHEAD" if abs(bear) < 20 else ("LEFT" if bear > 0 else "RIGHT")
        rows.append((abs(bear), bear, r, len(c), width, xm, ym, side))

    for _, bear, r, npts, width, xm, ym, side in sorted(rows):
        print(f"{bear:9.1f} {r:7.2f} {npts:4d} {width:7.0f}   "
              f"x={xm:+.2f} y={ym:+.2f}  {side}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
