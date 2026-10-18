"""The lidar and navigation's pose, written to disk for the world model's job.

    python3 scan_log.py [--dir DIR]     # inside navigation's ROS environment

The job (`job.py`) runs on the system Python, where version 4 was verified,
and the lidar is reached only from navigation's own (RoboStack's conda
environment, whose Python has neither the same numpy nor PIL). So this
process listens -- to `/scan` and to `/tf`'s odom -> base_link and map ->
odom, as `bag_extract.py` reads a drive recording -- and every FLUSH_S writes
what arrived since the last write to `DIR/<first stamp>-<last stamp>.npz`;
files older than KEEP_S are removed. The job reads the files that cover the
half hour it needs (`job.lidar`).

A SIGTERM ends it, after what it holds is written, and so does ROS stopping
under it, so that run_job.sh starts another. **rclpy's own handlers are off**:
they took a SIGTERM as "shut ROS down" and left the process running, so every
restart_job.sh -- every deploy of world_state -- left a logger that heard
nothing and that the supervisor never replaced, until the next boot. On
2026-10-18 the job filed without lidar from 07:46, its restart then, to 09:43.
"""
import argparse
import math
import os
import signal
import threading
import time

import numpy as np

DIR = os.path.expanduser(os.environ.get("UGV_WORLD_DIR", "~/.ugv/world") + "/visit_model/lidar")
FLUSH_S = 30.0
KEEP_S = 2400.0


class Listener:
    def __init__(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage
        self.rclpy = rclpy
        self.lock = threading.Lock()
        self.scans, self.meta, self.odom, self.map = [], [], [], []
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("visit_model_scan_log")
        self.node.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.node.create_subscription(TFMessage, "/tf", self._tf, 100)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            self.rclpy.spin(self.node)
        except Exception:  # noqa: BLE001 - ROS shut down; `hearing` says so
            pass

    def close(self):
        """ROS shut down and its thread finished: exiting with it still
        spinning aborted the process ("terminate called without an active
        exception")."""
        try:
            self.rclpy.shutdown()
        except Exception:  # noqa: BLE001 - already shut down
            pass
        self.thread.join(timeout=5.0)

    def hearing(self):
        """Whether ROS is still up and the thread spinning it still running."""
        return self.rclpy.ok() and self.thread.is_alive()

    @staticmethod
    def _stamp(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    #: Stamped before the clock was set: the Orin reads 1970 for about 75 s
    #: after a cold boot. One such scan made a chunk that "covered" 1970 to
    #: 2026, and every sighting without a drive recording was judged against
    #: it and found no lidar there (2026-10-18).
    CLOCK_SET_AFTER = 1.7e9

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _scan(self, msg):
        if self._stamp(msg.header) < self.CLOCK_SET_AFTER:
            return
        with self.lock:
            self.scans.append(np.asarray(msg.ranges, dtype=np.float32))
            self.meta.append((self._stamp(msg.header), time.time(), msg.angle_min,
                              msg.angle_increment, msg.time_increment, msg.scan_time))

    def _tf(self, msg):
        with self.lock:
            for t in msg.transforms:
                if self._stamp(t.header) < self.CLOCK_SET_AFTER:
                    continue
                row = (self._stamp(t.header), time.time(), t.transform.translation.x,
                       t.transform.translation.y, self._yaw(t.transform.rotation))
                if t.header.frame_id == "odom" and t.child_frame_id == "base_link":
                    self.odom.append(row)
                elif t.header.frame_id == "map" and t.child_frame_id == "odom":
                    self.map.append(row)

    def take(self):
        """What arrived since the last call, in the recordings' arrays."""
        with self.lock:
            scans, meta, odom, mp = self.scans, self.meta, self.odom, self.map
            self.scans, self.meta, self.odom, self.map = [], [], [], []
        if not meta:
            return None
        width = max(len(s) for s in scans)
        ranges = np.full((len(scans), width), np.nan, dtype=np.float32)
        for i, s in enumerate(scans):
            ranges[i, :len(s)] = s
        return {"ranges": ranges, "scan_meta": np.array(meta),
                "odom_tf": np.array(odom).reshape(-1, 5), "map_tf": np.array(mp).reshape(-1, 5)}


def write(directory, part):
    if part is None:
        return
    name = "%.3f-%.3f.npz" % (part["scan_meta"][0, 0], part["scan_meta"][-1, 0])
    tmp = os.path.join(directory, "." + name)
    with open(tmp, "wb") as f:
        np.savez(f, **part)
        # On disk before it is named: a power cut on 2026-10-17 left
        # an empty file under a real name.
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, os.path.join(directory, name))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=DIR)
    args = ap.parse_args()
    os.makedirs(args.dir, exist_ok=True)
    listener = Listener()

    def stop(_signum, _frame):
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, stop)
    try:
        while listener.hearing():
            began = time.time()
            while time.time() - began < FLUSH_S and listener.hearing():
                time.sleep(1.0)
            write(args.dir, listener.take())
            for name in os.listdir(args.dir):
                p = os.path.join(args.dir, name)
                if time.time() - os.path.getmtime(p) > KEEP_S:
                    os.remove(p)
        print("ROS stopped under the lidar logger: exiting, for run_job.sh to start another",
              flush=True)
        raise SystemExit(1)
    finally:
        write(args.dir, listener.take())
        listener.close()


if __name__ == "__main__":
    main()
