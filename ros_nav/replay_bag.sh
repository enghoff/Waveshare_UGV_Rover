#!/bin/bash
# Replay a recorded drive into the mapper and keep the map it produces.
#
#     ~/ugv/ros_nav/replay_bag.sh recordings/bags/DRIVE
#     ~/ugv/ros_nav/replay_bag.sh recordings/bags/DRIVE --out tighter \
#         -- -p minimum_travel_distance:=0.1
#
# Everything after `--` is handed to slam_toolbox as ROS parameters, so a setting
# can be tried against a drive that has already happened instead of against a
# rover that has to be pushed around a room again.
#
# What comes out is written beside the bag as NAME.pgm, .png and .txt: the
# occupancy grid the mapper ended up with, a picture of it, and the numbers that
# make two runs comparable -- how big the map is and how many cells are walls.
#
# This had a second mapper in it until 2026-08-31, and the README's "Why RTAB-Map
# was tried and removed" says how that ended. What survived is the more useful
# half: a recorded drive is worth more than the argument it was recorded to
# settle, because every later question about mapping can be asked of it for the
# price of six minutes rather than a rover pushed round a room again.
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
# `async_slam_toolbox_node` matches the mapper the rover is steering on, and a
# replay that tidies up after itself by pattern is a replay that stops the rover
# mapping. Every process started here is remembered by PID and killed by PID;
# sweep.sh is for the live stack and is not called.
#
# The reverse is true and worth knowing before it wastes ten minutes: **a deploy
# or a restart.sh while a replay is running kills the replay.** Those do call
# sweep.sh, which kills mappers by pattern because that is the right thing for
# the live stack, and the pattern does not know that one of them is a replay on
# another domain. Wait for the replay, or expect a half-finished map.

set -eu

DIR="$(cd "$(dirname "$0")" && pwd)"
RATE=1.0
OUT=""
EXTRA=()

BAG="${1:-}"
if [ -z "$BAG" ]; then
    echo "usage: $0 BAG [--out NAME] [--rate R] [-- -p name:=value ...]" >&2
    exit 2
fi
shift
while [ $# -gt 0 ]; do
    case "$1" in
        --out)  OUT="$2"; shift ;;
        --rate) RATE="$2"; shift ;;
        --)     shift; EXTRA=("$@"); break ;;
        *) echo "replay_bag.sh: unknown argument $1" >&2; exit 2 ;;
    esac
    shift
done

[ -d "$BAG" ] || { echo "replay_bag.sh: no such bag: $BAG" >&2; exit 2; }
[ -n "$OUT" ] || OUT="$(basename "$BAG")-slam_toolbox"
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

echo "--- replaying $(basename "$BAG") on domain 43"

ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
    --params-file "$DIR/config/slam_toolbox.yaml" \
    -p use_sim_time:=true \
    "${EXTRA[@]+"${EXTRA[@]}"}" > "$RESULT.log" 2>&1 &
pids+=($!)

# Is it actually running? A mapper that rejected an argument is gone within a
# second, and without this the next thing that happens is five minutes of bag
# played into nothing, ending in "no map published" -- which reads as a mapper
# that failed at the job rather than one that never started it.
sleep 4
for p in "${pids[@]+"${pids[@]}"}"; do
    if ! kill -0 "$p" 2>/dev/null; then
        echo "--- the mapper died on startup; its last words:" >&2
        tail -5 "$RESULT.log" >&2
        exit 1
    fi
done

# A lifecycle node comes up doing nothing at all until something walks it through
# configure and activate. The live stack has nav2_lifecycle_manager for this;
# here it is two calls, retried, because the services take a few seconds to
# appear and a transition sent before they exist is simply lost.
for _ in $(seq 20); do
    if ros2 lifecycle set /slam_toolbox configure > /dev/null 2>&1; then break; fi
    sleep 1
done
ros2 lifecycle set /slam_toolbox activate > /dev/null 2>&1 || true

# The map collector starts *before* the bag and stays subscribed to the end, and
# that ordering is load-bearing rather than tidy: a mapper publishes its grid on
# a timer, so a collector started afterwards can find nothing but the last one,
# and one that was never asked for the grid at all can find nothing whatever.
# It is given the bag's own length plus a margin, because the mapper is behind
# the bag by whatever it has queued.
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

echo "--- kept: $RESULT.png, $RESULT.pgm, $RESULT.txt"
