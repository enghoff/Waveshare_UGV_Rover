"""Ask live Nav2 for a path only; never sends navigation or motor commands.

Run under the rover's ROS environment. Coordinates/headings are map metres/degrees.
Without --start, Nav2 uses the current pose. Explicit starts are hypothetical.
"""
import argparse
import json
import math


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', nargs=3, type=float, required=True)
    parser.add_argument('--start', nargs=3, type=float)
    args = parser.parse_args()
    import rclpy
    from rclpy.action import ActionClient
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose

    rclpy.init()
    node = rclpy.create_node('visibility_route_check')
    client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')

    def pose(values):
        result = PoseStamped()
        result.header.frame_id = 'map'
        result.header.stamp = node.get_clock().now().to_msg()
        result.pose.position.x, result.pose.position.y = values[:2]
        yaw = math.radians(values[2])
        result.pose.orientation.z = math.sin(yaw / 2)
        result.pose.orientation.w = math.cos(yaw / 2)
        return result

    def wait(future):
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done():
            raise RuntimeError('Planner timed out; do not drive this unchecked route.')
        return future.result()

    handle = None
    try:
        if not client.wait_for_server(timeout_sec=3):
            raise RuntimeError('Planner unavailable.')
        request = ComputePathToPose.Goal()
        request.goal = pose(args.goal)
        request.planner_id = 'GridBased'
        request.use_start = args.start is not None
        if args.start:
            request.start = pose(args.start)
        handle = wait(client.send_goal_async(request))
        if handle is None or not handle.accepted:
            raise RuntimeError('Planner refused the request.')
        answer = wait(handle.get_result_async())
        points = [[p.pose.position.x, p.pose.position.y,
                   math.degrees(2 * math.atan2(p.pose.orientation.z,
                                              p.pose.orientation.w))]
                  for p in answer.result.path.poses]
        ok = answer.status == 4 and answer.result.error_code == 0 and len(points) >= 2
        print(json.dumps(dict(ok=ok, status=answer.status,
                              error_code=answer.result.error_code,
                              hypothetical_start=args.start, goal=args.goal,
                              length_m=sum(math.dist(a[:2], b[:2])
                                           for a, b in zip(points, points[1:])),
                              path=points)))
        return 0 if ok else 1
    finally:
        if handle is not None and handle.accepted:
            # Cancelling a planning action cannot command the wheels.
            future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, future, timeout_sec=1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
