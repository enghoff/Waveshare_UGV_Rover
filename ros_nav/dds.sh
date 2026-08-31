# Pin ROS 2 discovery to this board.
#
#     . ~/ugv/ros_nav/env.sh
#     . ~/ugv/ros_nav/dds.sh
#
# Sourced after env.sh, never instead of it. RoboStack's activate hook sets
# `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, which is how a laptop on the LAN can
# run rviz without being told a domain. On this rover that is the wrong default.
# The rover's addresses change under it -- this LAN has a second DHCP server
# answering beside the router, and a person at the console can put the rover on
# another network -- so CycloneDDS keeps trying leftover peers and
# `ddsi_udp_conn_write` fails. The
# graph then goes silent -- /scan, TF, the map all stop arriving at nav_bridge --
# while every process is still listed. The daemon reports the stack dead. The
# lidar is still logging 9.9 Hz.
#
# The console talks TCP 8769 / 8773, not ROS, so discovery can stay on loopback.
# A laptop that wants rviz on the LAN has to unset these after sourcing, and
# CYCLONEDDS_URI with them.
#
# The XML is the part that actually works, and the reason it exists is worth
# keeping. `ROS_LOCALHOST_ONLY=1` was set here for months and was believed to be
# the guard. It was not. On 2026-08-26 the stack came up healthy at 15:07:46,
# the roaming manager that then ran moved wlan0 to another router at 15:14:19 --
# nothing roams by itself any more, but a lease still moves -- and the graph died at
# that second -- because every Nav2 node was holding a UDP socket bound to
# 192.168.1.102, wlan0's address when the launch started. The variable was set
# in all of them the whole time. Whatever it is meant to do, in this build of
# rmw_cyclonedds it did not bind anything to loopback, and nothing in the stack
# noticed, because the check that was supposed to catch this looked for the
# variable rather than for the sockets.
#
# So the interface is named in a config file Cyclone has to obey, and
# restart.sh now checks the sockets. ROS_LOCALHOST_ONLY is deliberately *not*
# set: it is deprecated, it made rcl discard ROS_AUTOMATIC_DISCOVERY_RANGE
# ("'localhost_only' is enabled, 'automatic_discovery_range' and 'static_peers'
# will be ignored"), and it bought nothing in exchange. Dropping it leaves two
# guards that are both live instead of one loud one that was not.

# BASH_SOURCE, not $0 -- this file is sourced, so $0 is the caller. Both
# run_ros_nav.sh and restart.sh already source it from bash.
_dds_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_dds_cfg="$_dds_dir/cyclonedds-loopback.xml"

if [ -r "$_dds_cfg" ]; then
    export CYCLONEDDS_URI="file://$_dds_cfg"
else
    # Loud, because the silent version of this is a stack that comes up looking
    # perfect and dies at the next roam.
    echo "dds.sh: $_dds_cfg is missing -- CycloneDDS will discover on the LAN" \
         "and the next change of the rover's address will take the graph down" >&2
fi

unset _dds_dir _dds_cfg

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
