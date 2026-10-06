"""Read-only global/local Nav2 costmap capture, under the rover ROS environment.

Wrap this command in an external timeout: DDS can outlast Python spin timeouts.
Prints JSON; never publishes velocity, navigates, changes maps or refits a pose.
"""
import base64
import json
import math
import time


def main():
    import rclpy
    from nav2_msgs.srv import GetCostmap
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node('visibility_costmap_capture')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    output = {'capture_started_at': time.time(), 'grids': {}}
    try:
        for name in ('global', 'local'):
            client = node.create_client(GetCostmap, '/' + name + '_costmap/get_costmap')
            if not client.wait_for_service(timeout_sec=2):
                raise RuntimeError(name + ' costmap unavailable')
            future = client.call_async(GetCostmap.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            if not future.done():
                raise RuntimeError(name + ' costmap timed out')
            message = future.result().map
            metadata = message.metadata
            origin = metadata.origin
            deadline = time.monotonic() + 2
            while True:
                try:
                    transform = buffer.lookup_transform(message.header.frame_id,
                                                        'base_link', rclpy.time.Time())
                    break
                except Exception:
                    if time.monotonic() >= deadline:
                        raise RuntimeError(name + ' costmap has no current body transform')
                    rclpy.spin_once(node, timeout_sec=0.05)
            position = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                             1 - 2 * (rotation.y ** 2 + rotation.z ** 2))
            now_ns = node.get_clock().now().nanoseconds
            age = lambda stamp: (now_ns - stamp.sec * 10**9 - stamp.nanosec) / 10**9
            output['grids'][name] = {
                'frame': message.header.frame_id,
                'stamp': {'sec': message.header.stamp.sec,
                          'nanosec': message.header.stamp.nanosec},
                'width': metadata.size_x, 'height': metadata.size_y,
                'resolution': metadata.resolution,
                'origin': [origin.position.x, origin.position.y],
                'origin_orientation': [origin.orientation.x, origin.orientation.y,
                                       origin.orientation.z, origin.orientation.w],
                'age_s': age(message.header.stamp),
                'body_pose': {'frame': message.header.frame_id,
                              'x_m': position.x, 'y_m': position.y,
                              'heading_deg': math.degrees(yaw),
                              'age_s': age(transform.header.stamp)},
                'data': base64.b64encode(bytes(message.data)).decode(),
            }
        output['captured_at'] = time.time()
        print(json.dumps(output))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
