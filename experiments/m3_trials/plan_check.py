#!/usr/bin/env python3
"""Ask the planner for a route without moving, and say what the live layer marked.

    . ~/ugv/ros_nav/env.sh; . ~/ugv/ros_nav/dds.sh; python3 /tmp/plan_check.py X Y

A bench tool copied to the rover by hand, not deployed. dds.sh matters: without
it discovery never finds the costmap and the script says there is none.

Reads the global costmap (`/global_costmap/get_costmap`) and asks Nav2's own
planner (`compute_path_to_pose`) for a route from where the rover stands to
(X, Y). Prints the lethal cells within 3 m of the rover, how far the route
strays from the straight line, and the route itself every 0.25 m. Run once with
the live layer on and once off (`{"op": "live_layer", "enabled": false}` on the
bridge) with somebody standing in the way, and the two answers say whether the
layer makes the planner route round them. Nothing here publishes or moves.
"""
import json
import math
import sys
import time

import rclpy
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from rclpy.action import ActionClient
from rclpy.node import Node


def main():
    goal_xy = (float(sys.argv[1]), float(sys.argv[2]))
    rclpy.init()
    node = Node("plan_check")
    costmap_client = node.create_client(GetCostmap, "/global_costmap/get_costmap")
    planner = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    if not costmap_client.wait_for_service(timeout_sec=20.0):
        raise SystemExit("no global costmap service")
    future = costmap_client.call_async(GetCostmap.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    grid = future.result().map
    meta = grid.metadata
    res, ox, oy = meta.resolution, meta.origin.position.x, meta.origin.position.y

    if not planner.wait_for_server(timeout_sec=20.0):
        raise SystemExit("no planner")
    goal = ComputePathToPose.Goal()
    goal.goal.header.frame_id = "map"
    goal.goal.pose.position.x, goal.goal.pose.position.y = goal_xy
    goal.goal.pose.orientation.w = 1.0
    goal.use_start = False
    sent = planner.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sent, timeout_sec=5.0)
    handle = sent.result()
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=10.0)
    path = [(p.pose.position.x, p.pose.position.y) for p in result.result().result.path.poses]
    if not path:
        print(json.dumps({"route": None, "why": "the planner returned no route"}))
        return
    start = path[0]
    lethal = []
    for row in range(meta.size_y):
        for col in range(meta.size_x):
            if grid.data[row * meta.size_x + col] >= 254 and grid.data[row * meta.size_x + col] != 255:
                x, y = ox + (col + 0.5) * res, oy + (row + 0.5) * res
                if math.hypot(x - start[0], y - start[1]) <= 3.0:
                    lethal.append((round(x, 2), round(y, 2)))
    dx, dy = goal_xy[0] - start[0], goal_xy[1] - start[1]
    length = math.hypot(dx, dy) or 1e-9
    strays = max(abs((p[0] - start[0]) * dy - (p[1] - start[1]) * dx) / length for p in path)
    route_m = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
    every, last = [], None
    for p in path:
        if last is None or math.hypot(p[0] - last[0], p[1] - last[1]) >= 0.25:
            every.append((round(p[0], 2), round(p[1], 2)))
            last = p
    print(json.dumps({"at": time.strftime("%H:%M:%S"), "start": [round(v, 2) for v in start],
                      "route_m": round(route_m, 2), "straight_m": round(length, 2),
                      "strays_m": round(strays, 2), "lethal_within_3m": len(lethal),
                      "route": every, "lethal": lethal}))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
