# Jetson Orin navigation decision

Status: implemented. Current operation and unresolved faults are in
[`ros_nav/README.md`](../ros_nav/README.md).

The rover runs ROS 2 Jazzy, `slam_toolbox` and Nav2 on the Jetson Orin Nano. The
driver-board UART remains owned by `rover_daemon`; ROS receives odometry and
sends motor commands through the loopback board bridge. This preserves the one
hardware-owner rule and keeps the daemon's tools as the external control surface.

## Decisions

- Keep `slam_toolbox` as the 2D mapper.
- Keep the measured DWB controller configuration as the production baseline.
- Keep ROS in the RoboStack environment under `~/miniforge3`.
- Use the D500 lidar for navigation and the OAK-D-Lite for semantic range only.
- Save the pose graph and validate localization against a live scan on restart.
- Implement exploration in the repository rather than adding `explore_lite`.
- Confine the rover's DDS discovery to loopback.

Waveshare's example stack is useful as hardware reference material, but its
controller assumptions do not fit this chassis. The rover has a measured minimum
drive response, legitimate pivot motion, and limited reverse visibility. Those
constraints are encoded in the current mixer and Nav2 configuration.

## Alternatives tested

RTAB-Map was installed and tested on the Orin, then removed. On this rover it
produced poorer mapping than `slam_toolbox`, added a second ROS installation, and
did not justify its resource or maintenance cost.

MPPI may be reconsidered only against recorded paths and the real chassis. A
candidate must follow the same route with fewer stalls or steering reversals,
stay within measured motor commands, and fit the Orin's memory budget alongside
the deployed services. It is not a migration prerequisite.

OAK visual SLAM is not part of the current navigation path. Depth is useful to
semantic association, but no validated model lets it replace the lidar's obstacle
authority or detect every drop-off hazard.

## Evidence required for change

Navigation faults must first reproduce from a real recording using the replay and
simulation tools under `ros_nav/`. A proposed configuration then has to improve
that case without breaking the offline suite, and finally has to be observed on
the rover through TCP 8769.

The source configuration and deploy manifest are authoritative. This decision
record does not duplicate parameter values that can change independently.
