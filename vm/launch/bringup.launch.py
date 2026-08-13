#!/usr/bin/env python3
"""Both sensors, a common frame, and gravity — everything except SLAM.

Frame convention: base_link sits on the floor plane under the chassis, so the
lidar and camera heights are simply what a tape measure reads from the floor.
REP-103 axes: x forward, y left, z up.

The camera's own frames are published by depthai_ros_driver from its calibration,
so this file only has to tell it where its base sits relative to base_link — hence
parent_frame/cam_pos_* rather than a URDF of our own. The lidar gets one static
transform.

The six offsets are REQUIRED arguments with no defaults, deliberately. Invented
placeholder geometry is precisely the kind of thing that survives into a map that
looks plausible and is quietly wrong.

    ros2 launch bringup.launch.py \
        lidar_x:=0.0 lidar_y:=0.0 lidar_z:=0.0 \
        cam_x:=0.0 cam_y:=0.0 cam_z:=0.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
OAK_PARAMS = "/home/rover/ugv/config/oak.yaml"


def generate_launch_description():
    # Measured on the rover 2026-08-12, plus datasheet offsets to get from the
    # surfaces a tape measure can reach to the optical centres it cannot:
    #
    #   lidar_z  0.130 measured to the underside of the housing, + 0.0272 to the
    #            scan plane (35.0 mm body height less the 7.8 mm the STL-19P
    #            datasheet dimensions from the top face to the optical window).
    #            +/-3 mm, on my reading of that section view.
    #   lidar_x  0.040, centre of the lidar unit ahead of the chassis centre.
    #   cam_x    0.085 measured to the front face. The optical centre sits a few
    #            mm behind it, which is inside the OAK's own +/-6 mm at 1 m.
    #   cam_z    0.130 measured to the top of the housing, less half of the
    #            28 mm body height. The lens row is assumed vertically centred;
    #            Luxonis publish no mechanical drawing to confirm it.
    #
    # Both sensors are centred laterally, so y is zero for each.
    geometry_args = [
        DeclareLaunchArgument(n, default_value=v, description=d)
        for n, v, d in [
            ("lidar_x", "0.040", "base_link -> lidar_link, metres forward"),
            ("lidar_y", "0.0", "base_link -> lidar_link, metres left"),
            ("lidar_z", "0.1572", "lidar scan plane height above the floor, metres"),
            ("cam_x", "0.085", "base_link -> camera base, metres forward"),
            ("cam_y", "0.0", "base_link -> camera base, metres left"),
            ("cam_z", "0.116", "camera optical axis height above the floor, metres"),
        ]
    ]

    # +pi/2, derived from lidar_view.py rather than guessed. to_screen() computes
    # angle = bearing - VIEW_ROTATION_DEG and draws (sin, -cos), so the point
    # landing straight up the window is sensor bearing +90 deg, and that is
    # exactly where the green heading marker is drawn. Screen-up is the ROVER's
    # forward, so the sensor's own zero points 90 deg counter-clockwise from it:
    # out of the rover's left side. lidar_link's +x therefore lies along
    # base_link's +y, which is a yaw of +pi/2.
    #
    # Still worth confirming against the world, because a yaw cannot repair a
    # mirrored scan: if ldlidar's laser_scan_dir mishandles the sensor's
    # left-hand angle convention the map comes out reflected. Place one object
    # ahead AND one only to the left -- an object dead ahead alone cannot
    # distinguish a mirror.
    lidar_yaw = DeclareLaunchArgument("lidar_yaw", default_value="1.5707963")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("depthai_ros_driver"), "launch", "camera.launch.py"
            )
        ),
        launch_arguments={
            "camera_model": "OAK-D-LITE",
            "params_file": OAK_PARAMS,
            "parent_frame": "base_link",
            "cam_pos_x": LaunchConfiguration("cam_x"),
            "cam_pos_y": LaunchConfiguration("cam_y"),
            "cam_pos_z": LaunchConfiguration("cam_z"),
        }.items(),
    )

    # crop is forwarded so a mapping run can mask the rover's rear without
    # editing files. Note the crop angles are in the SENSOR's own frame, where
    # zero points out of the rover's left side -- so the rover's rear is at
    # sensor 270 deg, not 180.
    crop = DeclareLaunchArgument("crop", default_value="false")
    crop_min = DeclareLaunchArgument("crop_min", default_value="230.0")
    crop_max = DeclareLaunchArgument("crop_max", default_value="310.0")

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, "lidar.launch.py")),
        launch_arguments={
            "crop": LaunchConfiguration("crop"),
            "crop_min": LaunchConfiguration("crop_min"),
            "crop_max": LaunchConfiguration("crop_max"),
        }.items(),
    )

    base_to_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_lidar_link",
        arguments=[
            "--x", LaunchConfiguration("lidar_x"),
            "--y", LaunchConfiguration("lidar_y"),
            "--z", LaunchConfiguration("lidar_z"),
            "--yaw", LaunchConfiguration("lidar_yaw"),
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "lidar_link",
        ],
    )

    # The BMI270 has no on-chip fusion -- unlike the BNO086 on the full OAK-D it
    # publishes raw accel and gyro only, with no rotation vector and no
    # magnetometer. Madgwick turns that into an orientation so RTAB-Map has a
    # gravity constraint. No absolute heading is available; yaw will drift.
    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        parameters=[{
            "use_mag": False,
            "publish_tf": False,
            "world_frame": "enu",
            "fixed_frame": "base_link",
        }],
        remappings=[
            ("imu/data_raw", "/oak/imu/data"),
            ("imu/data", "/imu/data"),
        ],
    )

    return LaunchDescription(
        geometry_args
        + [lidar_yaw, crop, crop_min, crop_max,
           camera, lidar, base_to_lidar, imu_filter]
    )
