# Pin ROS 2 discovery to this board.
#
#     . ~/ugv/ros_nav/env.sh
#     . ~/ugv/ros_nav/dds.sh
#
# Sourced after env.sh, never instead of it. RoboStack's activate hook sets
# `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, which is how a laptop on the LAN can
# run rviz without being told a domain. On this rover that is the wrong default.
# There are two radios and a /32 service address that wifi_dual moves, so
# CycloneDDS keeps trying leftover peers (wlan1's .47, a previous failover's
# .101) and `ddsi_udp_conn_write` fails. The graph then goes silent -- /scan,
# TF, the map all stop arriving at nav_bridge -- while every process is still
# listed. The daemon reports the stack dead. The lidar is still logging 9.9 Hz.
#
# The console talks TCP 8769 / 8773, not ROS, so discovery can stay on loopback.
# A laptop that wants rviz on the LAN has to unset these after sourcing.

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOCALHOST_ONLY=1
