#!/usr/bin/env bash
# Kill anything still holding the sensors.
#
# Processes launched over SSH survive the session closing, so a previous run's
# component_container keeps the camera claimed and the next launch fails with
# X_LINK_DEVICE_ALREADY_IN_USE -- while ros2 topic list still shows the OLD
# instance's topics, which makes the failure look like a configuration problem.
set -eo pipefail

# Every node type the stack can launch must appear here. A node missing from this
# list does not fail loudly -- it survives the restart and quietly runs a second
# copy of itself. ekf_node and fusion_prep were both missing when the EKF was
# added, and five copies accumulated across five restarts, all publishing to
# /odometry/filtered. The consumer sees the five filters' outputs interleaved,
# which reads as one filter behaving erratically: the same measurement gave
# 0.00 deg/min of drift on one run and -5.45 deg/min on the next.
#
# lidar_watchdog comes first on purpose. It exists to restart the lidar node
# whenever the node's serial port moves, and it cannot tell a deliberate
# shutdown from a device that vanished -- left running through this loop it
# would answer the kill below by asking launch to respawn what was just stopped.
for pattern in lidar_watchdog component_container ldlidar_stl_ros2_node 'ros2 launch' \
               robot_state_publisher \
               rectify static_transform_publisher rf2o slam_toolbox imu_filter \
               ekf_node fusion_prep; do
    pkill -f "$pattern" 2>/dev/null && echo "killed: $pattern" || true
done

sleep 3
pkill -9 -f component_container 2>/dev/null || true
sleep 1

echo "=== remaining ros processes ==="
pgrep -af 'ros2|component_container|ldlidar|ekf_node|fusion_prep' || echo "  (none)"

# Duplicates are the failure this file exists to prevent, so say plainly whether
# any survived rather than leaving it to be inferred from the list above.
# pgrep -c prints "0" AND exits non-zero when nothing matches, so "|| echo 0"
# yields the string "0\n0" and every numeric test then fails. || true is what is
# wanted: keep pgrep's own output, just don't let set -e see the exit status.
for exe in ekf_node fusion_prep component_container ldlidar_stl_ros2_node; do
    n=$(pgrep -c -f "(^|/)$exe" 2>/dev/null || true)
    [ "${n:-0}" -gt 0 ] && echo "  WARNING: $n x $exe still running"
done
exit 0

echo "=== is the camera free? ==="
"$HOME/venvs/oak/bin/python" - <<'PYEOF'
import depthai as dai

infos = dai.Device.getAllAvailableDevices()
for i in infos:
    print(f"  {i.getMxId()}  state={i.state.name}")
if not infos:
    print("  no devices visible")
PYEOF
