"""The helper half of graph_replay.sh: run on the rover, on the replay's domain.

    python3 graph_replay_tool.py STEM X Y HEADING_DEG OUT.json SECONDS

Relays the recording's /tf (played as /tf_bag) onto /tf without the live
mapper's map -> odom, asks the replay mapper to load the graph STEM at the pose
given (as nav_map.load_graph does), and records every map -> odom the replay
mapper publishes, with its stamp, until SECONDS of the bag's clock have passed
since the first transform arrived. Writes OUT.json: [[stamp, x, y, yaw], ...].
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from slam_toolbox.srv import DeserializePoseGraph
from tf2_msgs.msg import TFMessage

stem, x, y, heading, out, seconds = (sys.argv[1], float(sys.argv[2]), float(sys.argv[3]),
                                     float(sys.argv[4]), sys.argv[5], float(sys.argv[6]))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Tool(Node):
    def __init__(self):
        super().__init__("graph_replay_tool")
        self.pub = self.create_publisher(TFMessage, "/tf", 100)
        self.create_subscription(TFMessage, "/tf_bag", self.relay, 200)
        self.create_subscription(TFMessage, "/tf", self.watch, 200)
        self.rows, self.first, self.last = [], None, None
        self.relayed = 0

    def relay(self, msg):
        keep = [t for t in msg.transforms
                if not (t.header.frame_id == "map" and t.child_frame_id == "odom")]
        if keep:
            self.pub.publish(TFMessage(transforms=keep))
            self.relayed += 1
            stamp = keep[0].header.stamp.sec + keep[0].header.stamp.nanosec * 1e-9
            self.first = stamp if self.first is None else self.first
            self.last = stamp

    def watch(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                s = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                self.rows.append([s, t.transform.translation.x, t.transform.translation.y,
                                  yaw_of(t.transform.rotation)])


rclpy.init()
node = Tool()
client = node.create_client(DeserializePoseGraph, "/slam_toolbox/deserialize_map")
if not client.wait_for_service(timeout_sec=60.0):
    raise SystemExit("the replay mapper offers no deserialize_map")
req = DeserializePoseGraph.Request()
req.filename = stem
req.match_type = req.START_AT_GIVEN_POSE
req.initial_pose.x, req.initial_pose.y, req.initial_pose.theta = x, y, math.radians(heading)
future = client.call_async(req)
started = time.time()
while not future.done() and time.time() - started < 300:
    rclpy.spin_once(node, timeout_sec=0.2)
print("graph loaded" if future.done() else "graph load timed out", "in %.0f s" % (time.time() - started),
      flush=True)
open(out + ".loaded", "w").write("1")
last_save = time.time()
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.2)
    if node.first is not None and node.last - node.first >= seconds:
        break
    if time.time() - last_save > 30:
        json.dump(node.rows, open(out, "w"))
        last_save = time.time()
        print("bag at +%.0f s, %d map->odom, %d relayed" % (
            (node.last or 0) - (node.first or 0), len(node.rows), node.relayed), flush=True)
json.dump(node.rows, open(out, "w"))
print("done:", len(node.rows), "map->odom", flush=True)
