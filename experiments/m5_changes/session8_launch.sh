#!/bin/bash
# M5 session 8: every step in order, detached, in navigation's environment.
# Run on the rover once the owner has unplugged it.
NAME=session8-1018
cd ~/ugv/ros_nav && . ./env.sh > /dev/null 2>&1
. ./dds.sh > /dev/null 2>&1
cd ~/m5 || exit 1
if [ -e ~/.ugv/sessions/$NAME.json ]; then echo "$NAME already recorded" >&2; exit 1; fi
setsid nohup python3 session.py all $NAME --b-first > ~/.ugv/sessions/$NAME.out 2>&1 < /dev/null &
echo "session $NAME started, pid $!"
