#!/bin/bash
# Replay a recorded drive into a mapper and keep the map it produces.
#
#     ~/ugv/ros_nav/replay_bag.sh recordings/bags/DRIVE --mapper rtabmap
#     ~/ugv/ros_nav/replay_bag.sh recordings/bags/DRIVE --mapper slam_toolbox
#     ~/ugv/ros_nav/replay_bag.sh recordings/bags/DRIVE --mapper rtabmap \
#         -- --RGBD/LinearUpdate 0.05 --Icp/CorrespondenceRatio 0.15
#
# Everything after `--` is handed to RTAB-Map as parameters, so a setting can be
# tried against a drive that has already happened instead of against a rover that
# has to be pushed around a room again.
#
# What comes out is written beside the bag as NAME-MAPPER.pgm, .png and .txt: the
# occupancy grid the mapper ended up with, a picture of it, and the numbers that
# make two runs comparable -- how big the map is, how many cells are walls, and
# for RTAB-Map how many loops it closed.
#
# ## It runs on its own DDS domain, and that is not a detail
#
# The rover is normally driving while somebody is doing this, and a replay
# publishes `/scan`, `/tf` and `/map` -- the same topics the live stack lives on.
# On one domain that is a second lidar feed arriving from a recording and a second
# `map -> odom`, which is a rover steering on a map of somewhere it was an hour
# ago. So the replay sets ROS_DOMAIN_ID to 43 (the rover is 42) after env.sh and
# dds.sh have had their say, and nothing it publishes can reach the rover at all.
#
# ## It kills only what it started, by PID
#
# **No pkill, no pattern, nowhere in this file.** A pattern that matches
# `async_slam_toolbox_node` or `/lib/rtabmap_slam/rtabmap` matches the mapper the
# rover is steering on, and a replay that tidies up after itself by pattern is a
# replay that stops the rover mapping. Every process started here is remembered
# by PID and killed by PID; sweep.sh is for the live stack and is not called.

set -eu

DIR="$(cd "$(dirname "$0")" && pwd)"
MAPPER=rtabmap
RATE=1.0
OUT=""
EXTRA=()

BAG="${1:-}"
if [ -z "$BAG" ]; then
    echo "usage: $0 BAG [--mapper rtabmap|slam_toolbox] [--out NAME] [--rate R] [-- params...]" >&2
    exit 2
fi
shift
while [ $# -gt 0 ]; do
    case "$1" in
        --mapper) MAPPER="$2"; shift ;;
        --out)    OUT="$2"; shift ;;
        --rate)   RATE="$2"; shift ;;
        --)       shift; EXTRA=("$@"); break ;;
        *) echo "replay_bag.sh: unknown argument $1" >&2; exit 2 ;;
    esac
    shift
done

case "$MAPPER" in
    rtabmap|slam_toolbox) ;;
    *) echo "replay_bag.sh: --mapper must be rtabmap or slam_toolbox" >&2; exit 2 ;;
esac
[ -d "$BAG" ] || { echo "replay_bag.sh: no such bag: $BAG" >&2; exit 2; }
[ -n "$OUT" ] || OUT="$(basename "$BAG")-$MAPPER"
RESULT="$(dirname "$BAG")/$OUT"

# shellcheck disable=SC1091
. "$DIR/env.sh"
# shellcheck disable=SC1091
. "$DIR/dds.sh"
export ROS_DOMAIN_ID=43

pids=()
cleanup() {
    for p in "${pids[@]+"${pids[@]}"}"; do
        kill "$p" 2>/dev/null || true
    done
    sleep 2
    for p in "${pids[@]+"${pids[@]}"}"; do
        kill -9 "$p" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

echo "--- replaying $(basename "$BAG") into $MAPPER on domain 43"

if [ "$MAPPER" = slam_toolbox ]; then
    ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
        --params-file "$DIR/config/slam_toolbox.yaml" \
        -p use_sim_time:=true > "$RESULT.log" 2>&1 &
    pids+=($!)
    # A lifecycle node comes up doing nothing at all until something walks it
    # through configure and activate. The live stack has nav2_lifecycle_manager
    # for this; here it is two calls, retried, because the services take a few
    # seconds to appear and a transition sent before they exist is simply lost.
    for _ in $(seq 20); do
        if ros2 lifecycle set /slam_toolbox configure > /dev/null 2>&1; then break; fi
        sleep 1
    done
    ros2 lifecycle set /slam_toolbox activate > /dev/null 2>&1 || true
else
    rm -f "$RESULT.db"
    # Started here rather than through run_rtabmap.sh because this is a different
    # job: its own database, its own domain, simulated time, and parameters from
    # the command line. What it shares with the live one is config/rtabmap.yaml,
    # which is the point -- a replay of the deployed settings has to be the
    # deployed settings.
    "$DIR/native.sh" ros2 run rtabmap_slam rtabmap --delete_db_on_start \
        --ros-args \
            -r __ns:=/rtabmap -r __node:=rtabmap \
            --params-file "$DIR/config/rtabmap.yaml" \
            -p use_sim_time:=true \
            -p publish_tf:=true \
            -p database_path:="$RESULT.db" \
            -r scan:=/scan -r map:=/map \
            "${EXTRA[@]+"${EXTRA[@]}"}" > "$RESULT.log" 2>&1 &
    pids+=($!)
fi

sleep 10

# The map collector starts *before* the bag and stays subscribed to the end, and
# that ordering is load-bearing rather than tidy. RTAB-Map assembles and
# publishes its occupancy grid only while something is subscribed to it, so a
# collector run after the replay finds nothing to collect -- a mapper that ran
# perfectly, processed every scan, and published no map because nobody had asked
# for one. The subscription is the asking.
#
# It is given the bag's own length plus a margin, because the mapper is behind
# the bag by whatever it has queued and both mappers rebuild the grid on a timer
# rather than per scan.
duration=$(ros2 bag info "$BAG" 2>/dev/null |
           sed -n 's/^Duration: *\([0-9]*\).*/\1/p' | head -1)
hold=$(( ${duration:-300} + 30 ))
python3 "$DIR/map_score.py" --out "$RESULT" --label "$OUT" --hold "$hold" &
collector=$!
pids+=("$collector")

echo "--- playing the bag at ${RATE}x"
ros2 bag play "$BAG" --clock --rate "$RATE"

echo "--- letting the mapper finish"
# Waited for rather than killed: it writes the map when its hold expires, and a
# collector killed first writes nothing at all.
wait "$collector" || true

if [ "$MAPPER" = rtabmap ]; then
    # Loop closures are RTAB-Map's own count and there is no equivalent from
    # slam_toolbox, which is why the map itself is the comparable number and this
    # is printed beside it rather than instead of it.
    closures=$(grep -c "Prox=" "$RESULT.log" 2>/dev/null || true)
    links=$("$DIR/native.sh" python3 -c "
import sqlite3
c = sqlite3.connect('file:$RESULT.db?mode=ro', uri=True).cursor()
by = dict(c.execute('select type, count(*) from Link group by type'))
print('neighbour links %d, loop/proximity links %d'
      % (by.get(0, 0), sum(v for k, v in by.items() if k != 0)))" 2>/dev/null || echo "database unreadable")
    echo "  $links"
fi

echo "--- kept: $RESULT.png, $RESULT.pgm, $RESULT.txt"
