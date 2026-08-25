#!/usr/bin/env python3
"""Everything in slam.launch.py, plus Nav2 on top of it.

    ros2 launch ~/ugv/ros_nav/nav.launch.py

So the rover maps as it drives and can be sent somewhere on the map it is
building. Sending it somewhere is a `NavigateToPose` action:

    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
        "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}}}}"

What is *not* here is AMCL and the map server. Nav2's stock bringup starts both,
because the stock arrangement is a rover localising against a map somebody saved
earlier. This rover is mapping as it goes, and slam_toolbox already publishes the
`map` -> `odom` transform that AMCL would -- starting AMCL as well would leave two
processes publishing the same transform, which is how a robot ends up teleporting
between two versions of where it thinks it is.

The lifecycle manager is what makes the Nav2 servers actually run. Every one of
them comes up `unconfigured` and does nothing until something walks it through
configure and activate; slam_toolbox is the same and is transitioned by
slam.launch.py, which is included here rather than repeated.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))

# SmacPlannerLattice will not resolve a relative lattice_filepath against this
# package, and an empty one loads Nav2's ackermann test set. The overlay below
# is the only path the plugin actually reads.
LATTICE_FILE = os.path.join(HERE, "config", "lattices", "diff_5cm_0.5m.json")

# The order is the order they are brought up in, and it matters: a costmap that
# activates before the map it subscribes to sits empty, and a controller that
# activates before its costmap has no idea what it is avoiding.
NAV2_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]


def generate_launch_description():
    params = LaunchConfiguration("nav_params")

    return LaunchDescription([
        DeclareLaunchArgument(
            "nav_params", default_value=os.path.join(HERE, "config", "nav2.yaml"),
            description="Nav2 parameters"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(HERE, "slam.launch.py"))),

        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen", parameters=[params],
             # The controller's output goes through the velocity smoother, which
             # republishes it on /cmd_vel for base_node. Without this remap both
             # would publish /cmd_vel and the wheels would get whichever arrived
             # last.
             remappings=[("cmd_vel", "cmd_vel_nav")]),
        Node(package="nav2_smoother", executable="smoother_server",
             name="smoother_server", output="screen", parameters=[params]),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen",
             parameters=[params,
                         {"GridBased.lattice_filepath": LATTICE_FILE}]),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen", parameters=[params]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen", parameters=[params]),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower",
             name="waypoint_follower", output="screen", parameters=[params]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother", output="screen", parameters=[params],
             remappings=[("cmd_vel", "cmd_vel_nav"),
                         ("cmd_vel_smoothed", "cmd_vel")]),

        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"autostart": True},
                         {"node_names": NAV2_NODES},
                         # Long, because these are C++ nodes loading plugins on a
                         # board where that takes seconds rather than
                         # milliseconds. The manager's stock patience is shorter
                         # than this hardware's startup, and running out of it
                         # looks like a node that crashed.
                         {"bond_timeout": 10.0},
                         {"attempt_respawn_reconnection": True}]),
    ])
