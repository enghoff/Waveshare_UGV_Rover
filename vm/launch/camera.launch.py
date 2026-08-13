#!/usr/bin/env python3
"""The OAK-D-Lite alone, for attaching to an already-running stack.

slam.launch.py deliberately omits the camera, so RViz's image panel shows
"No Image" during a mapping run. This brings the camera up as a separate
process against the same ROS graph, which means it can be started and killed
without disturbing slam_toolbox's map.

Geometry defaults match bringup.launch.py; see that file for how each number
was measured. Only base_link -> camera base is published here -- the camera's
internal frames come from depthai_ros_driver's own calibration.

    ros2 launch camera.launch.py            # camera + madgwick
    ros2 launch camera.launch.py imu:=false # camera only
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

OAK_PARAMS = "/home/rover/ugv/config/oak.yaml"
NODES = "/home/rover/ugv/nodes"


def generate_launch_description():
    geometry_args = [
        DeclareLaunchArgument(n, default_value=v, description=d)
        for n, v, d in [
            ("cam_x", "0.085", "base_link -> camera base, metres forward"),
            ("cam_y", "0.0", "base_link -> camera base, metres left"),
            ("cam_z", "0.116", "camera optical axis height above the floor, metres"),
        ]
    ]

    imu = DeclareLaunchArgument("imu", default_value="true")

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

    # depthai stamps IMU messages with oak_imu_frame, a frame its own URDF never
    # publishes -- so nothing in TF connects the IMU to the robot, and a consumer
    # that has to rotate the measurement (robot_localization does) drops every
    # message in silence. This supplies it.
    #
    # The rotation is measured, not assumed. A stationary accelerometer reads +g
    # along whichever of its own axes points up, and checks/imu_bias.py returns
    # gravity as (+0.147, +9.516, +0.094): the chip's +y is vertical, 1.1 deg off
    # true. The remaining freedom -- rotation about that vertical -- is taken as
    # the Luxonis convention of x right, z out of the back, giving yaw -pi/2 and
    # roll +pi/2. That is the one unverified assumption here, and it deliberately
    # does not matter for what we fuse: any rotation carrying IMU +y onto
    # base_link +z maps the gyro's y channel onto base_link yaw rate identically,
    # whatever it does with the horizontal pair. It would matter for fusing linear
    # acceleration, which ekf.yaml does not do.
    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_oak_imu",
        arguments=[
            "--x", LaunchConfiguration("cam_x"),
            "--y", LaunchConfiguration("cam_y"),
            "--z", LaunchConfiguration("cam_z"),
            "--yaw", "-1.5707963", "--pitch", "0", "--roll", "1.5707963",
            "--frame-id", "base_link",
            "--child-frame-id", "oak_imu_frame",
        ],
    )

    # Removes the gyro bias and stamps the covariances that both the OAK and rf2o
    # publish as zeros. See nodes/fusion_prep.py -- without it the EKF reads
    # "infinitely certain" from every source and the fusion does nothing.
    fusion_prep = ExecuteProcess(
        cmd=["python3", os.path.join(NODES, "fusion_prep.py")],
        name="fusion_prep",
        condition=IfCondition(LaunchConfiguration("imu")),
        output="screen",
    )

    # Raw accel and gyro only from the BMI270; madgwick turns that into an
    # orientation. Fed the de-biased stream so its output inherits the correction.
    # Cheap enough to leave on, and RTAB-Map wants it later.
    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        condition=IfCondition(LaunchConfiguration("imu")),
        parameters=[{
            "use_mag": False,
            "publish_tf": False,
            "world_frame": "enu",
            "fixed_frame": "base_link",
        }],
        remappings=[
            ("imu/data_raw", "/imu/data_unbiased"),
            ("imu/data", "/imu/data"),
        ],
    )

    return LaunchDescription(
        geometry_args + [imu, camera, imu_tf, fusion_prep, imu_filter]
    )
