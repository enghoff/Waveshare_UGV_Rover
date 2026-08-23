#!/usr/bin/env python3
"""Publish the D500's revolutions as ROS 2 `/scan`.

The packet parsing is not done here. `lidar_slam/libslam2d.so` already reads this
sensor -- the 47-byte packets, the CRC-8 over each of them, and the wrap that
marks the end of a revolution -- and it does it in 0.3 ms where Python takes 25.
So this creates a `Slam2D` purely as a parser: bytes go in with `feed()`, and the
completed revolution comes back out of `scan_xy()` in the rover's own frame, with
the 90-degree mount rotation already undone and the points that land on the
rover's own body already dropped.

Nothing else about that library is used. `update()` -- the scan match and the
occupancy grid -- is never called, so the matcher costs nothing and the grid is
never written. That work now belongs to `slam_toolbox`, which does it with loop
closure, and running both would be paying twice for the worse answer.

    python3 lidar_node.py --help
"""

import argparse
import glob
import math
import os
import sys

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan

import serial

# lidar_slam/ is a sibling on the workstation and a subdirectory of ~/ugv on the
# rover, which is flat. Try both rather than making the caller set PYTHONPATH.
_HERE = os.path.dirname(os.path.abspath(__file__))
for _candidate in (os.path.join(_HERE, "..", "lidar_slam"),
                   os.path.join(_HERE, "..", "..", "lidar_slam")):
    if os.path.isdir(_candidate):
        _candidate = os.path.abspath(_candidate)
        if _candidate not in sys.path:
            sys.path.insert(0, _candidate)

import slam2d                                                  # noqa: E402

LIDAR_BAUD = 230400
# What the sensor actually delivers: about 450 points in each of ten revolutions a
# second. Everything below is measured from that rather than chosen.
NOMINAL_HZ = 10.0


def find_lidar(preferred=None):
    """The lidar's port, preferring the by-id name that survives a replug.

    Deliberately the same rule as `lidar_slam/nav_types.py`, and for the same
    reason: this adapter has re-enumerated from ttyACM0 to ttyACM1 under a running
    process, and a node holding the old node number reports a frozen scan as
    though it were current. The by-id symlink carries the adapter's serial number.
    """
    if preferred and os.path.exists(preferred):
        return preferred
    for pattern in ("/dev/serial/by-id/*1a86*", "/dev/serial/by-id/*10c4*",
                    "/dev/ttyACM*"):
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    return None


class LidarNode(Node):

    def __init__(self, args):
        super().__init__("lidar_node")
        self.args = args
        self.frame_id = args.frame

        # A parser, not a SLAM. `max_points` is raised well past the sensor's own
        # ~450 because the default 300 is a budget for the scan matcher that used
        # to live downstream of this: it keeps the strongest returns and drops the
        # rest, which is the right trade when 300 poses are about to be scored
        # against them and the wrong one when the scan is the output.
        cfg = slam2d.default_config()
        cfg.max_points = args.max_points
        cfg.min_range_m = args.range_min
        cfg.max_range_m = args.range_max
        self.slam = slam2d.Slam2D(cfg)
        self.slam.mapping = False

        self.bins = args.bins
        self.increment = 2.0 * math.pi / self.bins
        self.scan_time = 1.0 / NOMINAL_HZ

        # Sensor data QoS: best-effort and shallow. A scan that is late is a scan
        # that is wrong, and slam_toolbox would rather miss one than be handed a
        # queue of stale ones after a hiccup. This is also what Nav2's costmap
        # subscribes with, and a reliable publisher satisfies a best-effort
        # subscriber, but not the other way round.
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(LaserScan, args.topic, qos)

        self.port = None
        self.port_path = None
        self.revolutions = 0
        self.thin = 0
        self._reopen_at = 0.0
        self._last_report = self.now()

        # Faster than the sensor by a good margin, because this loop's job is to
        # keep the kernel's serial buffer empty: a reader that wakes at the
        # revolution rate hands the C parser a backlog it will resync away.
        #
        # Ten milliseconds and not five. 47-byte packets at 230400 baud arrive
        # about every two milliseconds, so a wake every 10 ms collects five of
        # them -- far inside the parser's buffer -- and halves the number of times
        # a Python callback, a pyserial call and an ioctl are paid for. Measured
        # on this board that is the difference between about half a core and
        # about a quarter of one, for no change in the delivered scan rate.
        self.timer = self.create_timer(args.poll, self.poll)
        self.get_logger().info(
            "publishing %s in frame '%s', %d bins, %.2f-%.1f m"
            % (args.topic, self.frame_id, self.bins, args.range_min, args.range_max))

    def now(self):
        return self.get_clock().now()

    # --- the port -------------------------------------------------------------
    def open_port(self):
        """Open the lidar if it is not open, no more often than every few seconds.

        Failing to open is not an error worth stopping for. The lidar enumerates
        about ninety seconds after the kernel starts on this board, so a node
        launched at boot will not find it on the first several hundred attempts,
        and a node that exited on that would never come up at all.
        """
        if self.port is not None:
            return True
        now = self.now().nanoseconds / 1e9
        if now < self._reopen_at:
            return False
        self._reopen_at = now + 3.0
        path = find_lidar(None if self.args.port == "auto" else self.args.port)
        if not path:
            return False
        try:
            self.port = serial.Serial(path, LIDAR_BAUD, timeout=0)
        except (OSError, serial.SerialException) as exc:
            self.get_logger().warn("cannot open %s: %s" % (path, exc))
            self.port = None
            return False
        self.port_path = path
        # The sensor has been spinning all along, so the first wrap seen is the
        # tail of a revolution joined in the middle. Publishing it would put a
        # wedge-shaped scan into slam_toolbox as its very first observation, which
        # is the one it builds the whole map on.
        self.slam.resync()
        self.get_logger().info("lidar open on %s" % path)
        return True

    def drop_port(self):
        try:
            if self.port is not None:
                self.port.close()
        except Exception:
            pass
        self.port = None

    # --- the loop -------------------------------------------------------------
    def poll(self):
        if not self.open_port():
            return
        try:
            waiting = self.port.in_waiting
            data = self.port.read(waiting or 1)
        except (OSError, serial.SerialException) as exc:
            self.get_logger().warn("lidar read failed (%s); reopening" % exc)
            self.drop_port()
            return
        if not data:
            return
        # feed() returns how many revolutions completed inside those bytes. More
        # than one means this loop fell behind; only the newest is still in the
        # buffer, so the older ones are gone either way and saying so is the
        # useful part.
        completed = self.slam.feed(data)
        if completed <= 0:
            return
        if completed > 1:
            self.get_logger().debug(
                "fell behind: %d revolutions in one read" % completed)
        self.publish()

    def publish(self):
        points = self.slam.scan_xy()
        msg = self.to_scan(points) if points else None
        if msg is not None:
            self.pub.publish(msg)
            self.revolutions += 1
        else:
            self.thin += 1
        self.report()

    def to_scan(self, points):
        """A revolution of (x, y) metres in the rover frame as a LaserScan.

        The library hands back cartesian points in no particular order, because
        what used to consume them was a scan matcher that only cared where they
        were. A LaserScan is an array indexed by angle, so they have to be binned,
        and where two land in the same bin the nearer one wins: the message is
        read by an obstacle costmap as well as by the mapper, and rounding a chair
        leg away from the rover is the error that gets something hit.

        The frame is already ROS's. `slam2d` puts x along rover-forward and y
        along rover-left, which is REP-103, so no axes are swapped here -- and the
        90-degree mount rotation is applied inside the library, so this sees a
        sensor that is pointing where the rover is.
        """
        ranges = [float("inf")] * self.bins
        used = 0
        for x, y in points:
            r = math.hypot(x, y)
            if r < self.args.range_min or r > self.args.range_max:
                continue
            # Modulo rather than clamp. `atan2` returns exactly +pi along the
            # negative x axis -- straight behind the rover -- which divides out to
            # index `bins`, one past the end. Clamping it into the last bin puts
            # "straight behind" there, while the identical direction expressed as
            # -pi goes to the first bin, so the two halves of the same bearing
            # land at opposite ends of the array. Wrapping puts both in bin zero.
            i = int((math.atan2(y, x) + math.pi) / self.increment) % self.bins
            if r < ranges[i]:
                ranges[i] = r
            used += 1
        if used < self.args.min_points:
            # A revolution this thin is the sensor being blocked or the parser
            # having just resynced, not a room with nothing in it. Publishing it
            # would tell the costmap that everything it knew about is now clear.
            return None

        msg = LaserScan()
        # Stamped at the *start* of the revolution, which is where the first point
        # was measured, with scan_time and time_increment saying how the rest are
        # spread after it. Stamping at the end -- the obvious thing, since that is
        # when this code runs -- puts every point up to 100 ms in the future of a
        # transform lookup, and on a rover that is turning, 100 ms is degrees.
        msg.header.stamp = (self.now() - rclpy.duration.Duration(
            seconds=self.scan_time)).to_msg()
        msg.header.frame_id = self.frame_id
        # Half an increment in, because these are bins and not samples. Bin i
        # holds whatever fell in [-pi + i*inc, -pi + (i+1)*inc), so the bearing
        # that best represents it is the middle of that interval, not its lower
        # edge. Reporting the edge -- the obvious reading of angle_min -- biases
        # every point in the scan half a bin anticlockwise, which is a systematic
        # 0.5 degrees here and seven centimetres at the far wall.
        msg.angle_min = -math.pi + self.increment / 2.0
        msg.angle_max = msg.angle_min + (self.bins - 1) * self.increment
        msg.angle_increment = self.increment
        msg.scan_time = self.scan_time
        msg.time_increment = self.scan_time / self.bins
        msg.range_min = float(self.args.range_min)
        msg.range_max = float(self.args.range_max)
        msg.ranges = ranges
        return msg

    def report(self):
        if self.args.quiet:
            return
        now = self.now()
        elapsed = (now - self._last_report).nanoseconds / 1e9
        if elapsed < 10.0:
            return
        self.get_logger().info(
            "%.1f Hz on %s (%d revolutions, %d thin)"
            % (self.revolutions / elapsed, self.port_path or "?",
               self.revolutions, self.thin))
        self.revolutions = 0
        self._last_report = now


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--port", default="auto", help="lidar port, or 'auto'")
    p.add_argument("--topic", default="scan")
    p.add_argument("--frame", default="laser", help="frame_id on the scan")
    p.add_argument("--bins", type=int, default=360,
                   help="angular bins in the message; the sensor gives ~450 points")
    p.add_argument("--range-min", type=float, default=0.12)
    p.add_argument("--range-max", type=float, default=8.0)
    p.add_argument("--max-points", type=int, default=1200,
                   help="parser buffer; above the sensor's ~450 so none are dropped")
    p.add_argument("--min-points", type=int, default=40,
                   help="below this a revolution is treated as blocked, not empty")
    p.add_argument("--poll", type=float, default=0.010,
                   help="seconds between serial reads; the cost knob on this node")
    p.add_argument("--quiet", action="store_true")
    return p.parse_args(strip_ros_args(argv))


def strip_ros_args(argv):
    """Drop the `--ros-args ...` tail that `ros2 launch` appends.

    Everything after it belongs to rclpy, not to argparse, and there is no
    terminator -- it runs to the end of the command line.
    """
    if "--ros-args" in argv:
        return argv[:argv.index("--ros-args")]
    return list(argv)


def main():
    rclpy.init()
    args = parse_args(sys.argv[1:])
    node = LidarNode(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # ExternalShutdownException is what a SIGTERM looks like from inside spin,
        # so it is the normal way this node ends -- `ros2 launch` sends one to
        # every node on the way down. Letting it propagate prints a traceback for
        # a clean shutdown, which is how a supervisor log fills with alarming
        # nothing.
        pass
    finally:
        node.drop_port()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
