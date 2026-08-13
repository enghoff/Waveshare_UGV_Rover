#!/usr/bin/env python3
"""D500 (STL-19P / LD19) on the rover's driver board, as reached from the VM.

Differs from the vendor's ld19.launch.py in three ways:

  * port_name is /dev/rover-lidar, the udev symlink matching the CH343's own
    vid:pid. The board enumerates as CDC-ACM here, so the underlying node is
    /dev/ttyACM0 rather than the /dev/ttyUSB0 the vendor assumes -- and the host
    has five other CH340 adapters that would make an enumeration-order path a
    coin flip.

  * No static base_link->base_laser transform. The vendor file hardcodes
    z=0.18; the real offsets are measured and published from the URDF instead,
    so there is exactly one place where the geometry lives.

  * angle_crop is wired up but left disabled. Enable it before a hand-pushed
    mapping run: whoever pushes the rover sits inside a 360 degree scan at a
    fixed bearing, which is exactly what a scan matcher will latch onto.

A watchdog runs alongside the node, because the vendor node cannot survive its
device re-enumerating; see bin/lidar_watchdog.sh for what it watches and why.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WATCHDOG = os.path.expanduser("~/ugv/bin/lidar_watchdog.sh")


def generate_launch_description():
    port = DeclareLaunchArgument("port", default_value="/dev/rover-lidar")
    crop = DeclareLaunchArgument(
        "crop",
        default_value="false",
        description="mask the rear sector, for runs where someone is pushing the rover",
    )
    crop_min = DeclareLaunchArgument("crop_min", default_value="135.0")
    crop_max = DeclareLaunchArgument("crop_max", default_value="225.0")

    # respawn is what makes the watchdog's kill a repair rather than an outage.
    # The node itself never exits -- not on a lost device, not on a port that
    # cannot be opened -- so respawn on its own would never fire; the two only
    # work as a pair. respawn_delay gives udev time to publish the new symlink
    # before the replacement tries to open it.
    ldlidar = Node(
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        name="ldlidar",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {"product_name": "LDLiDAR_LD19"},
            {"topic_name": "scan"},
            {"frame_id": "lidar_link"},
            {"port_name": LaunchConfiguration("port")},
            {"port_baudrate": 230400},
            {"laser_scan_dir": True},
            {"enable_angle_crop_func": LaunchConfiguration("crop")},
            {"angle_crop_min": LaunchConfiguration("crop_min")},
            {"angle_crop_max": LaunchConfiguration("crop_max")},
        ],
    )

    watchdog = ExecuteProcess(
        cmd=["bash", WATCHDOG, LaunchConfiguration("port")],
        name="lidar_watchdog",
        output="screen",
    )

    return LaunchDescription([port, crop, crop_min, crop_max, ldlidar, watchdog])
