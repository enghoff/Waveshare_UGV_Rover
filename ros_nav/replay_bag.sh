#!/bin/bash
# Replay a recorded drive into a mapper and keep the map it produces.
#
#     ~/ugv/ros_nav/replay_bag.sh recordings/bags/DRIVE --mapper rtabmap
#     ~/ugv/ros_nav/replay_bag.sh recordings/bags/DRIVE --mapper slam_toolbox
#     ~/ugv/ros_nav/replay_bag.sh recordings/bags/DRIVE --mapper rtabmap+icp
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
#
# The reverse is true and worth knowing before it wastes ten minutes: **a deploy
# or a restart.sh while a replay is running kills the replay.** Those do call
# sweep.sh, which kills mappers by pattern because that is the right thing for
# the live stack, and the pattern does not know that one of them is a replay on
# another domain. Wait for the replay, or expect a half-finished map.

set -eu

DIR="$(cd "$(dirname "$0")" && pwd)"
MAPPER=rtabmap
RATE=1.0
OUT=""
EXTRA=()
# Every topic, unless a mapper mode needs some of them held back.
PLAY_TOPICS=()

BAG="${1:-}"
if [ -z "$BAG" ]; then
    echo "usage: $0 BAG [--mapper rtabmap|rtabmap+icp|slam_toolbox] [--out NAME] [--rate R] [-- params...]" >&2
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
    rtabmap|slam_toolbox|rtabmap+icp) ;;
    *) echo "replay_bag.sh: --mapper must be rtabmap, rtabmap+icp or slam_toolbox" >&2
       exit 2 ;;
esac

# **RTAB-Map is two command line conventions wearing one name**, and getting them
# mixed up costs a whole replay. `rtabmap-reprocess`, which is where most of this
# rover's parameter sweeps have been run, takes `--Icp/CorrespondenceRatio 0.2`.
# The ROS node takes `-p Icp/CorrespondenceRatio:=0.2` and throws
# UnknownROSArgsError at anything else -- so the reprocess spelling reaches it as
# an unknown argument, it aborts in the first second, and the replay then plays
# five minutes of bag into a mapper that is not there.
#
# So the reprocess spelling is what this file accepts, and it is translated here.
# `-p Name:=value` is passed through for anybody who already knows the other one.
PARAMS=()
i=0
while [ "$i" -lt "${#EXTRA[@]}" ]; do
    arg="${EXTRA[$i]}"
    next="${EXTRA[$((i + 1))]:-}"
    case "$arg" in
        -p)  PARAMS+=(-p "$next"); i=$((i + 2)) ;;
        --*) if [ -z "$next" ]; then
                 echo "replay_bag.sh: $arg has no value" >&2; exit 2
             fi
             name="${arg#--}"
             case "$name" in
                 # Every RTAB-Map core parameter is declared as a *string* by
                 # the ROS node, whatever it looks like, and the CLI parses a
                 # bare 0.1 as a double -- refused with
                 # InvalidParameterTypeException before the node starts. The
                 # ones with a slash in the name are RTAB-Map's; the rest are
                 # ROS's own and mean their own types.
                 */*) PARAMS+=(-p "$name:='$next'") ;;
                 *)   PARAMS+=(-p "$name:=$next") ;;
             esac
             i=$((i + 2)) ;;
        *)   echo "replay_bag.sh: expected --Some/Parameter value, got '$arg'" >&2
             exit 2 ;;
    esac
done
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
    if [ "$MAPPER" = "rtabmap+icp" ]; then
        # Lidar odometry in front of the mapper, which is what RTAB-Map's own 2D
        # lidar setups do and what this rover has never had: instead of taking
        # the wheels off TF and correcting them once per keyframe, it matches
        # every scan against a running local map and produces `odom` itself.
        #
        # **The wheels are not played in this mode**, and that is not squeamish-
        # ness about spending CPU on them. A frame in TF has exactly one parent,
        # so base_node's `odom -> base_link` from the recording and this node's
        # cannot both exist: tf2 would take whichever arrived last and the
        # comparison would be of neither. PLAY_TOPICS below drops /tf and /odom
        # and keeps /tf_static, which carries base_link -> laser and belongs to
        # the rover rather than to whatever is producing odometry.
        #
        # What that costs is the guess. In production icp_odometry would be given
        # the wheels through guess_frame_id, with base_node publishing them under
        # a frame of their own; here it starts each match from where the last one
        # ended, which is the harder version of the job and therefore the honest
        # one to measure.
        PLAY_TOPICS=(--topics /scan /tf_static)
        "$DIR/native.sh" ros2 run rtabmap_odom icp_odometry \
            --ros-args \
                -r __node:=icp_odometry \
                --params-file "$DIR/config/rtabmap.yaml" \
                -p use_sim_time:=true \
                -p frame_id:=base_link \
                -p odom_frame_id:=odom \
                -p publish_tf:=true \
                -p subscribe_scan:=true \
                -p "Odom/Strategy:='0'" \
                -p "Odom/ScanKeyFrameThr:='0.8'" \
                -p "OdomF2M/ScanSubtractRadius:='0.05'" \
                -p "OdomF2M/ScanMaxSize:='15000'" \
                -r scan:=/scan > "$RESULT-odom.log" 2>&1 &
        pids+=($!)
        sleep 5
    fi
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
            "${PARAMS[@]+"${PARAMS[@]}"}" > "$RESULT.log" 2>&1 &
    pids+=($!)
fi

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
ros2 bag play "$BAG" --clock --rate "$RATE" "${PLAY_TOPICS[@]+"${PLAY_TOPICS[@]}"}"

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
