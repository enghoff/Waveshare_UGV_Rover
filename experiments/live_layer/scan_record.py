#!/usr/bin/env python3
"""Record what the planner's live obstacle layer is given, so its marks can be replayed.

    bash -c 'source ~/ugv/ros_nav/env.sh; python3 scan_record.py --seconds 120 --out rec.json'

**Why.** The owner sees orange marks -- what the live layer put on the planner's
costmap -- on open floor that nothing stands on, and they do not stay. The layer
(`ros_nav/behaviors/src/live_obstacle_layer.cpp`) places the latest scan with the
map transform *as it is when the costmap updates* (`tf2::TimePointZero`), and a
scan may be up to `max_age_s` (0.5 s) old by then. Turning, the rover's heading
changes in that time, so a wall's returns would land beside the wall, past the
10 cm the layer allows for the map already having it. That is a guess until a
recording shows it. This records, per scan:

    the ranges, the scan's own stamp and how old it was when received;
    map -> laser as the layer takes it (the latest transform);
    map -> laser at the scan's stamp, through odom, with the map correction
      as it is now (what the layer would get with tf2's six-argument lookup);

and, from the daemon at 2 Hz, the cells the layer actually marked (`nav_grid`'s
`live`) beside the rover's pose, plus the map once at the start. `replay.py`
recomputes the marks both ways and says which one the rover's own marks match.

Read-only: it publishes nothing and asks the daemon only for `nav_grid`.
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def pose_of(transform) -> list[float]:
    t, q = transform.transform.translation, transform.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return [round(t.x, 4), round(t.y, 4), round(yaw, 5)]


class Recorder(Node):
    def __init__(self, map_frame: str, fixed_frame: str):
        super().__init__("live_layer_scan_record")
        self.map_frame, self.fixed_frame = map_frame, fixed_frame
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        TransformListener(self.buffer, self)
        self.scans: list[dict] = []
        self.failures = 0
        self.lock = threading.Lock()
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan: LaserScan) -> None:
        now = self.get_clock().now()
        stamp = Time.from_msg(scan.header.stamp)
        row = {"recv": round(now.nanoseconds * 1e-9, 4),
               "stamp": round(stamp.nanoseconds * 1e-9, 4),
               "frame": scan.header.frame_id,
               "amin": scan.angle_min, "ainc": scan.angle_increment,
               "tinc": scan.time_increment, "rmin": scan.range_min, "rmax": scan.range_max,
               "ranges": [round(r, 3) if math.isfinite(r) else None for r in scan.ranges]}
        try:
            latest = self.buffer.lookup_transform(self.map_frame, scan.header.frame_id, Time())
            row["latest"] = pose_of(latest)
            row["latest_tf_stamp"] = round(Time.from_msg(latest.header.stamp).nanoseconds * 1e-9, 4)
        except Exception as error:                       # noqa: BLE001 -- recorded, not hidden
            row["latest_error"] = str(error)[:120]
        try:
            then = self.buffer.lookup_transform_full(
                self.map_frame, Time(), scan.header.frame_id, stamp, self.fixed_frame,
                timeout=Duration(seconds=0.05))
            row["at_stamp"] = pose_of(then)
        except Exception as error:                       # noqa: BLE001
            row["at_stamp_error"] = str(error)[:120]
            self.failures += 1
        with self.lock:
            self.scans.append(row)


def daemon(call: str, host: str = "127.0.0.1", port: int = 8769) -> dict:
    with socket.create_connection((host, port), 10) as s:
        f = s.makefile("rwb")
        f.write(json.dumps({"call": call, "arguments": {}}).encode() + b"\n")
        f.flush()
        return json.loads(f.readline())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--seconds", type=float, default=120.0)
    parser.add_argument("--out", required=True)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--fixed-frame", default="odom")
    args = parser.parse_args()

    grid = daemon("nav_grid")
    marks: list[dict] = []
    stop = threading.Event()

    def poll() -> None:
        while not stop.is_set():
            try:
                got = daemon("nav_grid")
                marks.append({"wall": round(time.time(), 3), "pose": got.get("pose"),
                              "live": got.get("live") or []})
            except Exception as error:                   # noqa: BLE001
                marks.append({"wall": round(time.time(), 3), "error": str(error)[:120]})
            stop.wait(0.5)

    rclpy.init()
    node = Recorder(args.map_frame, args.fixed_frame)
    poller = threading.Thread(target=poll, daemon=True)
    poller.start()
    began = time.monotonic()
    try:
        while time.monotonic() - began < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        stop.set()
        poller.join(timeout=2.0)
        with node.lock:
            scans = list(node.scans)
        with open(args.out, "w", encoding="utf-8") as handle:
            json.dump({"recorded_at": time.time(), "seconds": args.seconds,
                       "map": {k: grid.get(k) for k in ("width", "height", "resolution_m",
                                                         "origin_x_m", "origin_y_m", "data")},
                       "scans": scans, "marks": marks,
                       "at_stamp_failures": node.failures}, handle)
        node.destroy_node()
        rclpy.shutdown()
    print(f"{len(scans)} scans, {len(marks)} mark samples, "
          f"{node.failures} at-stamp lookups failed -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
