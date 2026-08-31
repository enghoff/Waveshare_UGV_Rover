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

# --max-bag-duration rather than a `timeout` around it: bag writes its metadata
# when it closes the bag itself, and a bag killed mid-write is a bag that cannot
# be replayed. This is the difference between a recording and a directory of
# fragments.
ros2 bag record \
    --output "$OUT" \
    --max-bag-duration "$SECONDS_TO_RECORD" \
    --storage mcap \
    /scan /tf /tf_static /odom /imu/data_raw &
child=$!

# Stop when this script is signalled, and stop the recorder the way it wants to
# be stopped -- SIGINT, which is what bag treats as "close the bag properly".
# A Ctrl-C in an ssh session lands here, and what it must not do is leave an
# unreadable bag behind.
stop() {
    echo
    echo "--- stopping the recording"
    kill -INT "$child" 2>/dev/null || true
    wait "$child" 2>/dev/null || true
    exit 0
}
trap stop INT TERM

sleep "$SECONDS_TO_RECORD"
kill -INT "$child" 2>/dev/null || true
wait "$child" 2>/dev/null || true

echo "--- recorded:"
du -sh "$OUT" 2>/dev/null || true
ros2 bag info "$OUT" 2>&1 | grep -E "Duration|Messages|Topic information" || true
echo
echo "Replay it into a mapper with:"
echo "  ~/ugv/ros_nav/replay_bag.sh $OUT --mapper rtabmap"
echo "  ~/ugv/ros_nav/replay_bag.sh $OUT --mapper slam_toolbox"
