#!/bin/bash
# Run one measured comparison of slam_toolbox against RTAB-Map, and put the
# rover back the way it was found.
#
#     ~/ugv/ros_nav/compare_run.sh                     # 180 s, rover stationary
#     ~/ugv/ros_nav/compare_run.sh --seconds 420       # long enough to drive a route
#     ~/ugv/ros_nav/compare_run.sh --closed-loop       # you drove back to the start
#
# It stops the boot stack, brings the same stack up with both mappers running,
# watches for a while, prints the numbers, and then restores the boot
# configuration by calling restart.sh -- which reads its arguments from the
# crontab, so what comes back is what boots and not what this script felt like.
# The restore runs from a trap, so a Ctrl-C or a dropped ssh session still puts
# the rover back.
#
# **This exists because starting compare mode by hand is genuinely dangerous,**
# and not in a subtle way. The obvious way to do it is to type
#
#     ssh orin 'pkill -f "ros_nav/run_ros_nav[.]sh"; \
#               bash ~/ugv/ros_nav/run_ros_nav.sh --nav rtabmap:=compare'
#
# and that kills the ssh session that is running it, because the second half of
# the command line contains the text the first half is searching for. The
# brackets do not save you -- they stop the *pattern* matching itself, not the
# other command sitting next to it on the same line. restart.sh's header
# describes this trap, sweep.sh is a separate file because of it, and it has now
# cost this repository a fourth session. A pattern inside a script on disk cannot
# match the command line that invoked the script, which is the whole reason this
# is a file.
#
# The other reason it is a file: `rtabmap:=compare` has nowhere permanent to
# live. The supervisor takes its arguments from the crontab, deliberately, so
# that a hand relaunch cannot silently drop `--nav` the way one once dropped
# `--vision` from the daemon. A comparison run is temporary by definition, so it
# gets a script that is explicit about putting things back rather than a boot
# setting somebody has to remember to undo.

set -eu

DIR="$(cd "$(dirname "$0")" && pwd)"
SECONDS_TO_WATCH=180
EXTRA=()

while [ $# -gt 0 ]; do
    case "$1" in
        --seconds)      SECONDS_TO_WATCH="$2"; shift ;;
        --closed-loop)  EXTRA+=(--closed-loop) ;;
        --out)          EXTRA+=(--out "$2"); shift ;;
        *) echo "compare_run.sh: unknown argument $1" >&2; exit 2 ;;
    esac
    shift
done

restore() {
    echo
    echo "--- putting the boot stack back"
    # restart.sh with no supervisor running reads the crontab for its flags, so
    # this restores what boots rather than what was asked for here.
    bash "$DIR/restart.sh" --supervisor 2>&1 | tail -12
}
trap restore EXIT INT TERM

echo "--- stopping the boot stack"
# Patterns naming files under $DIR. Safe here for the reason in the header: this
# script's own command line is `compare_run.sh`, which none of them match.
pkill -f "$DIR/run_ros_nav[.]sh" 2>/dev/null || true
sleep 3
bash "$DIR/sweep.sh" 2>&1 | tail -3 || true

echo "--- starting both mappers"
setsid nohup bash "$DIR/run_ros_nav.sh" --nav rtabmap:=compare \
    > /dev/null 2>&1 < /dev/null &

# Long, and worth being long. Nav2's servers load their plugins, slam_toolbox
# allocates its correlation grids, and RTAB-Map opens a database and waits for a
# transform before it will accept its first scan. Measuring during that is
# measuring the startup.
echo "--- letting it settle (60 s)"
sleep 60

for name in async_slam_toolbox_node /lib/rtabmap_slam/rtabmap; do
    n=$(pgrep -fc "$name" 2>/dev/null || true)
    printf '  %-30s %s\n' "$name" "${n:-0}"
done

echo "--- watching for ${SECONDS_TO_WATCH}s"
# Through native.sh: slam_compare.py reads rtabmap_msgs, which exists only in
# the /opt/ros/jazzy install.
"$DIR/native.sh" python3 "$DIR/slam_compare.py" \
    --seconds "$SECONDS_TO_WATCH" "${EXTRA[@]+"${EXTRA[@]}"}"
