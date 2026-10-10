"""Turn a recorded bag into plain arrays: scans, odom -> base_link, map -> odom.

Run on the rover inside the ROS environment (ros_nav/env.sh):
    python3 /tmp/bag_extract.py BAG OUT.npz
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_of(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def main(bag, out):
    reader = rosbag2_py.SequentialReader()
    storage = "mcap" if bag.endswith(".mcap") else ""
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    scans, scan_meta, odom_tf, map_tf, odom = [], [], [], [], []
    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == "/scan":
            scans.append(np.asarray(msg.ranges, dtype=np.float32))
            scan_meta.append((stamp_of(msg.header), recv_ns * 1e-9, msg.angle_min,
                              msg.angle_increment, msg.time_increment, msg.scan_time))
        elif topic == "/tf":
            for t in msg.transforms:
                row = (stamp_of(t.header), recv_ns * 1e-9, t.transform.translation.x,
                       t.transform.translation.y, yaw_of(t.transform.rotation))
                if t.header.frame_id == "odom" and t.child_frame_id == "base_link":
                    odom_tf.append(row)
                elif t.header.frame_id == "map" and t.child_frame_id == "odom":
                    map_tf.append(row)
        elif topic == "/odom":
            odom.append((stamp_of(msg.header), recv_ns * 1e-9,
                         yaw_of(msg.pose.pose.orientation), msg.twist.twist.angular.z))
    width = max(len(s) for s in scans) if scans else 0
    ranges = np.full((len(scans), width), np.nan, dtype=np.float32)
    for i, s in enumerate(scans):
        ranges[i, :len(s)] = s
    np.savez_compressed(out, ranges=ranges, scan_meta=np.array(scan_meta),
                        odom_tf=np.array(odom_tf), map_tf=np.array(map_tf),
                        odom=np.array(odom))
    print("scans", len(scans), "odom_tf", len(odom_tf), "map_tf", len(map_tf),
          "odom", len(odom))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
