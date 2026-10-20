#!/bin/bash
# The fixes' session, detached so that a dropped ssh does not stop it mid-drive.
#     bash ~/multiday/insight.sh NAME          then, once the owner has moved things:
#     touch ~/.ugv/sessions/NAME.go
NAME=$1
[ -n "$NAME" ] || { echo "usage: insight.sh NAME" >&2; exit 1; }
cd ~/ugv/ros_nav && . ./env.sh > /dev/null 2>&1
. ./dds.sh > /dev/null 2>&1
cd ~/multiday || exit 1
if [ -e ~/.ugv/sessions/$NAME.json ]; then echo "$NAME already recorded" >&2; exit 1; fi
setsid nohup /usr/bin/python3 insight.py run "$NAME" > ~/.ugv/sessions/$NAME.out 2>&1 < /dev/null &
echo "$NAME started, pid $!: parked until ~/.ugv/sessions/$NAME.go"
