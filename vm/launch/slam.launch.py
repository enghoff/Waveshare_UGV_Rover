#!/usr/bin/env python3
"""2D mapping: lidar, laser odometry, slam_toolbox, and the camera.

This rover has no wheel encoders, and the BMI270 cannot substitute -- it gives
orientation and turn rate, not position. So odometry comes from rf2o, which
matches consecutive scans against each other. slam_toolbox does the mapping and
loop closure on top.

The camera contributes nothing to a 2D lidar map; it is here to be watched, so
RViz can show what the rover is looking at while the map builds. It was excluded
outright when the guest had 4 vCPUs, because the depth pipeline at 15 Hz plus its
transport, with Ceres and RViz alongside, saturated them hard enough that sshd
could no longer answer. At 6 vCPUs the whole stack measures 0.63 load average, so
it is back on by default. camera:=false still turns it off -- worth doing for a
long bag recording, where the cores are better spent on the writer.

    ros2 launch slam.launch.py                   # mapping + camera
    ros2 launch slam.launch.py crop:=true        # mask the rover's rear first
    ros2 launch slam.launch.py camera:=false     # lidar only

Use crop:=true whenever a person is pushing the rover. At a fixed bearing and
half a metre away they are a large, consistent return, and a scan matcher will
happily treat them as a landmark that never moves.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
SLAM_PARAMS = "/home/rover/ugv/config/slam_toolbox.yaml"
EKF_PARAMS = "/home/rover/ugv/config/ekf.yaml"


def generate_launch_description():
    crop = DeclareLaunchArgument("crop", default_value="false")
    # ekf, rf2o or static -- who owns the odom -> base_link transform.
    #
    #   ekf     robot_localization fusing rf2o's translation with the OAK's gyro.
    #           The default. rf2o still runs, but with publish_tf off.
    #   rf2o    rf2o alone, as before. Stationary it invents 44 mm/min and drifts
    #           in yaw at 8.4 deg/min, with a yaw-rate standard deviation of
    #           7.2 deg/s -- the reason the EKF exists. Keep it for comparison.
    #   static  odom pinned to base_link, leaving slam_toolbox's correlative scan
    #           matcher to account for all motion. It loses the between-scan
    #           motion prior, affordable at hand-pushing speed (2 cm per scan at
    #           0.2 m/s, far inside the 0.5 m search window).
    #
    # ekf needs the IMU, and the IMU comes from the camera, so ekf with
    # camera:=false gives an EKF with nothing to fuse. bin/start_slam.sh refuses
    # that combination rather than letting it degrade quietly.
    odom_source = DeclareLaunchArgument("odom_source", default_value="ekf")
    crop_min = DeclareLaunchArgument("crop_min", default_value="230.0")
    crop_max = DeclareLaunchArgument("crop_max", default_value="310.0")

    # Measured on the rover; see bringup.launch.py for provenance.
    lidar_x = DeclareLaunchArgument("lidar_x", default_value="0.040")
    lidar_y = DeclareLaunchArgument("lidar_y", default_value="0.0")
    lidar_z = DeclareLaunchArgument("lidar_z", default_value="0.1572")
    lidar_yaw = DeclareLaunchArgument("lidar_yaw", default_value="1.5707963")

    camera_arg = DeclareLaunchArgument("camera", default_value="true")
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, "camera.launch.py")),
        condition=IfCondition(LaunchConfiguration("camera")),
    )

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
            "--pitch", "0", "--roll", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "lidar_link",
        ],
    )

    # freq matched to the lidar's own 10 Hz. The vendor default of 20 asks for
    # odometry updates between scans, which cannot carry new information.
    #
    # Two instances, differing only in publish_tf. Under ekf the filter owns
    # odom -> base_link, and if rf2o kept publishing it too, both would write the
    # same TF edge -- TF does not arbitrate, it just keeps whichever arrived last,
    # so the fusion would appear to work while roughly half the transforms came
    # from the unfused source.
    rf2o_params = {
        "laser_scan_topic": "/scan",
        "odom_topic": "/odom_rf2o",
        "base_frame_id": "base_link",
        "odom_frame_id": "odom",
        "init_pose_from_topic": "",
        "freq": 10.0,
    }

    rf2o = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        condition=LaunchConfigurationEquals("odom_source", "rf2o"),
        parameters=[dict(rf2o_params, publish_tf=True)],
    )

    rf2o_for_ekf = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        condition=LaunchConfigurationEquals("odom_source", "ekf"),
        parameters=[dict(rf2o_params, publish_tf=False)],
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=LaunchConfigurationEquals("odom_source", "ekf"),
        parameters=[EKF_PARAMS],
    )

    static_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_base_link_static",
        condition=LaunchConfigurationEquals("odom_source", "static"),
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--yaw", "0", "--pitch", "0", "--roll", "0",
            "--frame-id", "odom",
            "--child-frame-id", "base_link",
        ],
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[SLAM_PARAMS],
    )

    return LaunchDescription([
        crop, crop_min, crop_max, odom_source, camera_arg,
        lidar_x, lidar_y, lidar_z, lidar_yaw,
        camera, lidar, base_to_lidar,
        rf2o, rf2o_for_ekf, ekf, static_odom, slam,
    ])
