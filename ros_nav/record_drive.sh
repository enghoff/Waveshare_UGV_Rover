#!/bin/bash
# Record a drive as a rosbag, so a mapper can be argued about without a rover.
#
#     ssh orin '~/ugv/ros_nav/record_drive.sh --seconds 300'
#     ssh orin '~/ugv/ros_nav/record_drive.sh --seconds 600 --name kitchen-loop'
#
# Drive the rover however you normally do while this runs -- the console, the
# voice chat, a goal. **This records and changes nothing else.** It starts no
# mapper, stops nothing, and touches neither the lidar nor the wheels; it
# subscribes to topics the running stack is already publishing. The rover
# behaves exactly as it would if this were not running.
#
# ## Why a bag, when the map database is already a recording
#
# It is, and for most questions it is the better one: `rtabmap-reprocess` replays
# it in twenty seconds and that is how the smeared-map fault was found. But a
# database holds the keyframes RTAB-Map *chose* -- about one every 13 cm -- and
# not the ten scans a second the lidar produced. So it cannot answer any question
# about what happens between keyframes, and three of the open questions are
# exactly that:
#
#   - how often a keyframe should be taken (RGBD/LinearUpdate, RGBD/AngularUpdate)
#   - whether lidar odometry beats the wheels, which needs consecutive scans 3 cm
#     apart rather than 13 -- replayed from a database it collapses, not because
#     the idea is wrong but because the data is not there
#   - slam_toolbox against RTAB-Map on identical input, which is the only fair
#     version of that comparison. compare_run.sh runs them side by side on one
#     drive, which is fair but costs a drive every time; a bag costs one drive
#     ever.
#
# ## What is recorded, and why each of them
#
#   /scan        the lidar, and the whole point
#   /tf          odom -> base_link from base_node, which is the motion guess
#                every mapper here starts from. Recorded rather than recomputed
#                because the wheels and the gyro are what the mapper actually saw.
#   /tf_static   base_link -> laser, which is the identity on this rover and will
#                not be once the lidar moves
#   /odom        the same numbers as a topic. Nav2 and nav_bridge read this rather
#                than TF, so a replay that wants them needs it.
#   /imu/data_raw  not used by either mapper today, and cheap. It is here so that
#                a future gyro-aided odometry can be tried against a drive that
#                has already happened.
#
# **Not** /map or the costmaps: they are what a mapper produces from the above, so
# recording them would be recording the answer along with the question.

set -eu

DIR="$(cd "$(dirname "$0")" && pwd)"
SECONDS_TO_RECORD=300
NAME="drive-$(date +%Y-%m-%d-%H%M)"

while [ $# -gt 0 ]; do
    case "$1" in
        --seconds) SECONDS_TO_RECORD="$2"; shift ;;
        --name)    NAME="$2"; shift ;;
        *) echo "usage: $0 [--seconds N] [--name NAME]" >&2; exit 2 ;;
    esac
    shift
done

# Under recordings/, which is on the deploy manifest's preserve list -- so a
# deploy cannot delete a drive that cost somebody twenty minutes of pushing a
# rover around a room.
OUT="$DIR/recordings/bags/$NAME"
mkdir -p "$(dirname "$OUT")"
if [ -e "$OUT" ]; then
    echo "record_drive.sh: $OUT already exists; pick another --name" >&2
    exit 2
fi

# shellcheck disable=SC1091
. "$DIR/env.sh"
# shellcheck disable=SC1091
. "$DIR/dds.sh"

echo "--- recording ${SECONDS_TO_RECORD}s to $OUT"
echo "--- drive the rover now"

# **In the foreground under `timeout`, and both halves of that are bought
# experience.**
#
# The recorder has to be stopped by SIGINT, because that is what rosbag2 treats
# as "close this bag properly" -- it then writes the metadata that makes the
# directory a bag rather than a heap of fragments. The obvious way to arrange
# that is to start it with `&`, sleep, and signal it. It does not work, and it
# fails silently: **a background process started by a non-interactive shell has
# SIGINT set to ignore**, inherited from the shell, so the signal lands on a
# process that has been told to discard it. What that looked like here was a
# 25-second recording that was still writing six minutes later while the script
# that started it sat in `wait` for a child that would never exit.
#
# In the foreground the recorder keeps the default disposition and stops when it
# is told to, and `timeout` is what tells it. --kill-after is the backstop for a
# recorder wedged inside its own shutdown; it costs the metadata, and a bag that
# cannot be replayed is better than a recorder nobody can stop.
#
# `--max-bag-duration` is deliberately not used. It splits the recording every N
# seconds and carries on recording for ever, which is what it says and not what
# anybody wants from a flag that looks like a length.
timeout --signal=INT --kill-after=30 "$SECONDS_TO_RECORD" \
    ros2 bag record \
        --output "$OUT" \
        --storage mcap \
        /scan /tf /tf_static /odom /imu/data_raw || true

echo "--- recorded:"
du -sh "$OUT" 2>/dev/null || true
ros2 bag info "$OUT" 2>&1 | grep -E "Duration|Messages|Topic information" || true
echo
echo "Replay it into a mapper with:"
echo "  ~/ugv/ros_nav/replay_bag.sh $OUT --mapper rtabmap"
echo "  ~/ugv/ros_nav/replay_bag.sh $OUT --mapper slam_toolbox"
