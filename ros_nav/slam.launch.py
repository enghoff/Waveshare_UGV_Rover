#!/usr/bin/env python3
"""Bring up mapping: the lidar, the wheels, and slam_toolbox on top of both.

    ros2 launch ~/ugv/ros_nav/slam.launch.py

Launched by path rather than by package name, and there is no package: the nodes
here are plain scripts and there is nothing to `colcon build`. That is a
deliberate choice for this repository, where deployment is `scp` and the failure
mode being avoided is a build step somebody forgets -- `lidar_slam/` already has
one of those, and a stale `libslam2d.so` is a rover running last week's code with
this week's file on disk. A launch file takes an absolute path for `executable`
when no package is named, so nothing is lost but the `ros2 run` shorthand.

What comes up, and why in this order:

  lidar_node   the D500 as /scan, and the room in words as /surroundings
  base_node    the driver board as /odom and the odom -> base_link transform
  slam_toolbox /scan plus that transform as /map, and map -> odom on top
  nav_bridge   all of the above, served to the rover daemon on loopback 8773

The bridge is here rather than in nav.launch.py on purpose. Most of what it hands
over -- the map, the pose, what is around the rover -- exists as soon as
slam_toolbox does, so a rover brought up for mapping alone still gives its
console a live map and a description of the room. What it cannot do without Nav2
is drive, and asked to, it says exactly that.

slam_toolbox is asynchronous rather than synchronous. The synchronous node
guarantees every scan reaches the mapper and blocks until it has, which is the
right trade on a machine with cores to spare and the wrong one here: when the
graph thread is busy with a loop closure, blocking stops the transform being
published, and a stalled map -> odom is a rover whose controller is steering on a
pose that has stopped moving. Async drops scans under load instead, and dropping
scans is what `minimum_time_interval` in the config is already doing on purpose.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    params = LaunchConfiguration("params")
    lidar_port = LaunchConfiguration("lidar_port")
    bridge_port = LaunchConfiguration("bridge_port")
    nav_port = LaunchConfiguration("nav_port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params", default_value=os.path.join(HERE, "config", "slam_toolbox.yaml"),
            description="slam_toolbox parameters"),
        DeclareLaunchArgument(
            "lidar_port", default_value="auto",
            description="lidar device, or 'auto' for the stable by-id name"),
        DeclareLaunchArgument(
            "bridge_port", default_value="8772",
            description="the daemon's board bridge"),
        DeclareLaunchArgument(
            "nav_port", default_value="8773",
            description="where the daemon reaches this stack; it must match the "
                        "daemon's own --ros-nav"),

        Node(executable=os.path.join(HERE, "lidar_node.py"),
             name="lidar_node", output="screen",
             arguments=["--port", lidar_port]),

        Node(executable=os.path.join(HERE, "base_node.py"),
             name="base_node", output="screen",
             arguments=["--bridge-port", bridge_port]),

        Node(package="slam_toolbox", executable="async_slam_toolbox_node",
             name="slam_toolbox", output="screen", parameters=[params]),

        # Not a lifecycle node and deliberately started before slam_toolbox has
        # finished coming up: everything it serves it serves by subscription, so
        # the worst a client gets in the first few seconds is an honest "the map
        # has not arrived yet".
        Node(executable=os.path.join(HERE, "nav_bridge.py"),
             name="nav_bridge", output="screen",
             arguments=["--port", nav_port]),

        # slam_toolbox is a *lifecycle* node in Jazzy, and it comes up
        # `unconfigured`: the process runs, answers `ros2 node list`, and has
        # subscribed to nothing. That failure is quiet in an unhelpful way -- the
        # log says nothing is wrong, the node sits at half a percent of a core,
        # and `ros2 topic info /scan` reports "Subscription count: 0", which reads
        # as a QoS mismatch and is not one.
        #
        # Walking it through the transitions with launch's own EmitEvent works and
        # is what slam_toolbox's stock launch file does -- but it races here. The
        # configure event is emitted the moment the launch starts, and on this
        # board the node needs seconds to load its plugins and advertise the
        # services that would receive it; when it loses that race nothing retries
        # and the stack comes up dead. nav2_lifecycle_manager waits for the
        # services to exist before transitioning, which turns a race into a wait.
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_slam", output="screen",
             parameters=[{"autostart": True},
                         {"node_names": ["slam_toolbox"]},
                         # Bond checking off, and this one is not a shortcut.
                         # After configure and activate the manager opens a
                         # heartbeat "bond" with each node it manages and expects
                         # the node to have created its end. Nav2's own servers
                         # do; slam_toolbox does not, so the manager waits ten
                         # seconds, logs "Server slam_toolbox was unable to be
                         # reached ... This server may be misconfigured", and
                         # declares "Failed to bring up all requested nodes.
                         # Aborting bringup" -- all of it while slam_toolbox is
                         # sitting there perfectly active and mapping. Zero
                         # disables the check, which is the documented way to
                         # manage a node that has no bond.
                         {"bond_timeout": 0.0},
                         {"attempt_respawn_reconnection": True}]),
    ])
