#!/bin/bash
# A second mapper, on domain 43, fed a recorded session from the graph it started with.
#
#     graph_replay.sh BAG STEM X Y HEADING SECONDS OUT [-- -p name:=value ...]
#
# STEM is the session's starting graph (the session script keeps it), X Y HEADING
# the pose the session started from. The recording's /tf is relayed without the
# live mapper's map -> odom (graph_replay_tool.py), so the replay mapper's own
# correction is the only one, and OUT.json records it. With STABLE=cells.npz in the
# environment, the scans pass through stable_filter.py first. Kills only what it started.
set -eu
DIR="$(cd "$(dirname "$0")" && pwd)"
BAG="$1"; STEM="$2"; X="$3"; Y="$4"; H="$5"; SECONDS_="$6"; OUT="$7"; shift 7
EXTRA=()
if [ "${1:-}" = "--" ]; then shift; EXTRA=("$@"); fi

cd ~/ugv/ros_nav
. ./env.sh >/dev/null 2>&1
. ./dds.sh >/dev/null 2>&1
export ROS_DOMAIN_ID=43

pids=()
cleanup() {
    for p in "${pids[@]+"${pids[@]}"}"; do kill "$p" 2>/dev/null || true; done
    sleep 2
    for p in "${pids[@]+"${pids[@]}"}"; do kill -9 "$p" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

"$(ros2 pkg prefix slam_toolbox)/lib/slam_toolbox/async_slam_toolbox_node" --ros-args \
    --params-file ~/ugv/ros_nav/config/slam_toolbox.yaml \
    -p use_sim_time:=true "${EXTRA[@]+"${EXTRA[@]}"}" > "$OUT.mapper.log" 2>&1 &
pids+=($!)
sleep 4
kill -0 "${pids[0]}" 2>/dev/null || { tail -5 "$OUT.mapper.log"; exit 1; }
for _ in $(seq 20); do
    ros2 lifecycle set /slam_toolbox configure > /dev/null 2>&1 && break
    sleep 1
done
ros2 lifecycle set /slam_toolbox activate > /dev/null 2>&1 || true

rm -f "$OUT.loaded"
python3 "$DIR/graph_replay_tool.py" "$STEM" "$X" "$Y" "$H" "$OUT" "$SECONDS_" > "$OUT.tool.log" 2>&1 &
tool=$!
pids+=("$tool")
for _ in $(seq 300); do [ -f "$OUT.loaded" ] && break; sleep 1; done
[ -f "$OUT.loaded" ] || { echo "graph never loaded"; tail -5 "$OUT.tool.log"; exit 1; }
cat "$OUT.tool.log"

REMAPS=(/tf:=/tf_bag)
if [ -n "${STABLE:-}" ]; then
    # Only returns from cells that stayed put reach the mapper (stable_filter.py).
    python3 "$DIR/stable_filter.py" "$STABLE" > "$OUT.filter.log" 2>&1 &
    pids+=($!)
    REMAPS+=(/scan:=/scan_bag)
    sleep 3
fi
# One --remap with every mapping: given twice, the second replaces the first.
ros2 bag play "$BAG" --clock --remap "${REMAPS[@]}" \
    --topics /scan /tf /tf_static /odom --playback-duration "$((SECONDS_ + 20))" > "$OUT.play.log" 2>&1 &
pids+=($!)
wait "$tool" || true
tail -2 "$OUT.tool.log"
