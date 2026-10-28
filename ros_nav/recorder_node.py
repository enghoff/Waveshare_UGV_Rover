"""The ROS side of a recording session: odometry, the map pose, the scan, the clock.

A recording session for Perception (rover_daemon/rover_recording.py) keeps the
rover's ROS streams beside the daemon's: odometry as base_node publishes it,
the localizer's pose at every step with the fit it had, the 360-direction scan
the rover navigates by, and the monotonic, wall and ROS clocks read together
once a second. The daemon starts this process when a session opens
(run_recorder.sh); it writes until the session's announcement is gone, then
closes its files and exits. It runs only for sessions because a subscriber to
the scan and odometry costs the Orin a few percent of a core for as long as it
exists, and outside a session nobody wants what it would write.

**The fit** reduces the localizer's state to good, poor, lost or unknown by the
rule agreed with Perception (docs/plans/recording-for-perception.md): lost with
no map, an unconfirmed restore (`map_settled` false), the gyro fault latched, or
no step searched against the map in the last 2 s; poor when the last searched
step scored under the localizer's own 0.55 and kept odometry, or the latest
drift check disagrees with the map; good otherwise; unknown until the localizer
has said anything. Only a good fit bounds where the rover is.

ROS on this rover stamps with the system's wall clock (measured 2026-10-28), so
a ROS stamp is moved onto the monotonic clock by the two clocks' difference
read at the moment the message arrives.
"""
from __future__ import annotations

import json
import math
import os
import socket
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

_HERE = os.path.dirname(os.path.abspath(__file__))
for _candidate in (_HERE, os.path.join(_HERE, "..", "rover_daemon")):
    if os.path.exists(os.path.join(_candidate, "session_record.py")):
        sys.path.append(os.path.abspath(_candidate))
        break
import session_record as rec  # noqa: E402

#: The localizer's own bar for believing a match (localizer.py, MIN_SCORE).
MIN_SCORE = 0.55
#: A step searched against the map within this long, or the fit is lost.
SEARCHED_WITHIN_S = 2.0
#: How often the navigation bridge is asked whether the map is settled, the gyro
#: sound and the drift check agreeing.
NAV_EVERY_S = 1.0
NAV_PORT = 8773


def ros_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def to_mono(ros_stamp_ns: int) -> int:
    """A wall-clock (ROS) stamp on the monotonic clock, by the clocks' difference now."""
    return ros_stamp_ns - (time.time_ns() - time.monotonic_ns())


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def map_odom(base, odom):
    """map -> odom from map -> base and odom -> base: base composed with odom's
    inverse, all as (x, y, yaw in radians)."""
    bx, by, byaw = base
    ox, oy, oyaw = odom
    yaw = byaw - oyaw
    c, s = math.cos(yaw), math.sin(yaw)
    return [bx - (c * ox - s * oy), by - (s * ox + c * oy),
            math.atan2(math.sin(yaw), math.cos(yaw))]


def fit_of(status: dict | None, nav: dict | None, last_searched: tuple | None,
           now_mono: float) -> str:
    """The agreed rule, from the localizer's status, the bridge's, and the last
    searched step as (arrival in monotonic seconds, score)."""
    if status is None:
        return "unknown"
    if not status.get("started") or not status.get("map"):
        return "lost"
    nav = nav or {}
    if nav.get("map_settled") is False or nav.get("gyro_fault"):
        return "lost"
    if last_searched is None or now_mono - last_searched[0] > SEARCHED_WITHIN_S:
        return "lost"
    if last_searched[1] < MIN_SCORE:
        return "poor"
    drift = nav.get("map_drift") or {}
    if drift.get("trusted") and drift.get("agrees") is False:
        return "poor"
    return "good"


class Recorder(Node):

    def __init__(self, session: dict):
        super().__init__("session_recorder")
        directory = session["dir"]
        self.streams = {
            "odometry": rec.Stream(directory, "odometry", "odometry.jsonl"),
            "map_pose": rec.Stream(directory, "map_pose", "map_pose.jsonl"),
            "lidar_scan": rec.Stream(directory, "lidar_scan", "lidar/scans360.jsonl"),
            "clock": rec.Stream(directory, "clock", "clock.jsonl"),
        }
        self.status: dict | None = None
        self.status_text: str | None = None
        self.nav: dict | None = None
        self.last_searched: tuple | None = None
        self.last_pose: dict | None = None
        self.odom: tuple | None = None
        self.fit = None
        self.dir = directory
        self._lock = threading.Lock()
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, "localizer/step", self.on_step, 50)
        self.create_subscription(String, "localizer/status", self.on_status, 10)
        self.create_subscription(String, "nav/pose_events", self.on_pose_event, 10)
        self.create_timer(1.0, self.on_clock)
        threading.Thread(target=self.ask_nav, name="nav-status", daemon=True).start()

    # --- streams that are copied -------------------------------------------

    def on_odom(self, msg):
        stamp = ros_ns(msg.header.stamp)
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.streams["odometry"].put(
            {"t_stamp": to_mono(stamp), "t_stamp_ros": stamp, "x": p.position.x,
             "y": p.position.y, "yaw": yaw_of(p.orientation),
             "vx": msg.twist.twist.linear.x, "wz": msg.twist.twist.angular.z,
             "derived_from": "board_feedback"})

    def on_scan(self, msg):
        stamp = ros_ns(msg.header.stamp)
        self.streams["lidar_scan"].put(
            {"t_stamp": to_mono(stamp), "t_stamp_ros": stamp, "frame": msg.header.frame_id,
             "angle_min": msg.angle_min, "angle_increment": msg.angle_increment,
             "time_increment": msg.time_increment, "scan_time": msg.scan_time,
             "range_min": msg.range_min, "range_max": msg.range_max,
             "ranges": [r if math.isfinite(r) else None for r in msg.ranges],
             "derived_from": "lidar"})

    def on_clock(self):
        self.streams["clock"].put({"wall_ns": time.time_ns(),
                                   "ros_ns": self.get_clock().now().nanoseconds})
        # This process's queues and drops, where the daemon's health stream reads them.
        status = {n: {"queue": s.depth(), "dropped": s.dropped, "queued": s.queued}
                  for n, s in self.streams.items()}
        path = os.path.join(self.dir, ".status-ros.json")
        try:
            with open(path + ".tmp", "w", encoding="utf-8") as f:
                json.dump(status, f)
            os.replace(path + ".tmp", path)
        except OSError:
            pass

    # --- the map pose ---------------------------------------------------------

    def _pose_record(self, trigger, event, pose):
        """One map_pose record from `pose` ({stamp_ns, map_odom, map_base}, or
        None) with the fit as it stands now."""
        fit = fit_of(self.status, self.nav, self.last_searched, time.monotonic())
        record = {"trigger": trigger, "t_stamp": None, "t_stamp_ros": None,
                  "map_odom": None, "map_base": None, "status": self.status_text,
                  "fit": fit, "event": event}
        if pose is not None:
            record.update(t_stamp=to_mono(pose["stamp_ns"]), t_stamp_ros=pose["stamp_ns"],
                          map_odom=pose.get("map_odom"), map_base=pose.get("map_base"))
        if record["fit"] == "good" and (record["map_odom"] is None or record["map_base"] is None):
            record["fit"] = "poor"  # a good fit carries a pose; with none it is not good
        return record

    def _after(self, record):
        """Write a record, then one for a change of fit if that is what it was."""
        # Copied before it is written: writing gives it its seq, and a copy of a
        # written record would carry the same one (2026-10-28, seq 4 twice).
        change = dict(record, trigger="status_change", event=None)
        self.streams["map_pose"].put(record)
        if record["fit"] != self.fit and self.fit is not None:
            self.streams["map_pose"].put(change)
        self.fit = record["fit"]

    def on_step(self, msg):
        try:
            step = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            if step.get("searched"):
                self.last_searched = (time.monotonic(), float(step.get("score", 0.0)))
            self.last_pose = step
            self._after(self._pose_record("periodic", None, step))
            if step.get("accepted"):
                self.streams["map_pose"].put(self._pose_record("event", "correction", step))

    def on_status(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self.status, self.status_text = status, msg.data
            fit = fit_of(self.status, self.nav, self.last_searched, time.monotonic())
            if self.fit is not None and fit != self.fit:
                record = self._pose_record("status_change", None, self.last_pose)
                self.streams["map_pose"].put(record)
                self.fit = record["fit"]

    def on_pose_event(self, msg):
        try:
            event = json.loads(msg.data)
        except ValueError:
            return
        if event.get("event") not in ("restore", "refit"):
            return
        x, y, deg = event["pose"]
        base = (x, y, math.radians(deg))
        with self._lock:
            # The localizer takes a handed pose as "where odometry is now", so the
            # map -> odom it sets is the pose composed with odometry's inverse.
            pose = {"stamp_ns": time.time_ns(), "map_base": list(base),
                    "map_odom": None if self.odom is None else map_odom(base, self.odom)}
            self.streams["map_pose"].put(self._pose_record("event", event["event"], pose))

    def ask_nav(self):
        """The bridge's status, once a second: the map's restore, the gyro, the drift."""
        while rclpy.ok():
            try:
                with socket.create_connection(("127.0.0.1", NAV_PORT), 2.0) as s:
                    s.settimeout(3.0)
                    s.sendall(b'{"op": "status"}\n')
                    line = s.makefile("rb").readline()
                answer = json.loads(line)
                with self._lock:
                    self.nav = answer.get("status", answer)
            except (OSError, ValueError):
                pass
            time.sleep(NAV_EVERY_S)

    def close(self) -> dict:
        return {name: s.close() for name, s in self.streams.items()}


def main():
    session = rec.announced()
    if session is None:
        print("[recorder] no session is open; nothing to record", flush=True)
        return 0
    rclpy.init()
    node = Recorder(session)
    stop = threading.Event()

    def watch():
        while rec.announced() is not None and rec.announced().get("dir") == session["dir"]:
            time.sleep(0.5)
        stop.set()

    threading.Thread(target=watch, daemon=True).start()
    try:
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        counts = node.close()
        rec.closed_marker(session["dir"], "ros", counts)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
