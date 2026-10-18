#!/bin/bash
# The day's sessions, detached so that a dropped ssh does not stop them mid-drive.
#     bash ~/multiday/launch.sh NAME DAY
NAME=$1
DAY=$2
[ -n "$NAME" ] && [ -n "$DAY" ] || { echo "usage: launch.sh NAME DAY" >&2; exit 1; }
cd ~/ugv/ros_nav && . ./env.sh > /dev/null 2>&1
. ./dds.sh > /dev/null 2>&1
cd ~/multiday || exit 1
if [ -e ~/.ugv/sessions/$NAME.json ]; then echo "$NAME already recorded" >&2; exit 1; fi
setsid nohup /usr/bin/python3 day.py run "$NAME" "$DAY" > ~/.ugv/sessions/$NAME.out 2>&1 < /dev/null &
echo "day $DAY started as $NAME, pid $!"
