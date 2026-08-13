#!/usr/bin/env bash
# Is mapping actually working? Wants: one of each process, an odom->base_link
# transform from rf2o, a map->odom from slam_toolbox, and a /map with non-unknown cells.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

echo "=== load / procs ==="
uptime
# Anchor on the whole argv[0], not a bare substring. "rf2o" also occurs in the
# ros2 launch command line as odom_source:=rf2o, so a loose -f count reports two
# and a healthy stack looks like a stacked one -- the single failure this check
# exists to catch. The (^|/) alternation covers both forms argv[0] takes: an
# absolute path for the nodes launch execs, a bare name for rviz2 off PATH.
for p in ldlidar_stl_ros2_node rf2o_laser_odometry_node async_slam_toolbox_node \
         component_container ekf_node fusion_prep rviz2; do
    # || true, not || echo 0: pgrep -c already prints 0 on no match and merely
    # exits non-zero, so echoing again produces "0\n0" and breaks the test below.
    # (\.py)? because the loose nodes are run as "python3 .../fusion_prep.py",
    # where the anchor would otherwise hit ".py" instead of a space and report 0
    # for a node that is running perfectly well.
    n=$(pgrep -c -f "(^|/)$p(\.py)?( |$)" 2>/dev/null || true)
    n=${n:-0}
    if [ "$n" -gt 1 ]; then
        echo "  $p: $n   <-- STACKED, results from this run are not trustworthy"
    else
        echo "  $p: $n"
    fi
done

echo "=== TF chain ==="
echo "-- odom -> base_link (rf2o)"
timeout 10 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -A3 -m1 Translation || echo "  MISSING"
echo "-- map -> odom (slam_toolbox)"
timeout 10 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -A3 -m1 Translation || echo "  MISSING"

echo "=== /map ==="
timeout 20 ros2 topic echo /map --once --no-arr 2>/dev/null \
    | grep -E 'resolution|width|height|frame_id' || echo "  no map yet"

echo "=== map occupancy ==="
timeout 25 python3 - <<'PYEOF'
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class M(Node):
    def __init__(self):
        super().__init__('mapstat')
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
        q = QoSProfile(depth=1)
        q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        q.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self.cb, q)
        self.got = None
    def cb(self, msg):
        self.got = msg

rclpy.init()
n = M()
for _ in range(40):
    rclpy.spin_once(n, timeout_sec=0.5)
    if n.got:
        break
if n.got:
    d = np.asarray(n.got.data, dtype=np.int16)
    print(f"  cells {d.size}: unknown {int((d < 0).sum())}, "
          f"free {int((d == 0).sum())}, occupied {int((d > 50).sum())}")
    print(f"  grid {n.got.info.width}x{n.got.info.height} @ {n.got.info.resolution} m")
else:
    print("  no /map received")
rclpy.shutdown()
PYEOF
