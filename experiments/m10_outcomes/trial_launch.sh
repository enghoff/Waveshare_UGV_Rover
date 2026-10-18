#!/bin/bash
# M10 look trial: detached, in navigation's environment. Run once the owner has unplugged the rover.
NAME=${1:-m10trial1-$(date +%m%d)}
cd ~/ugv/ros_nav && . ./env.sh > /dev/null 2>&1
. ./dds.sh > /dev/null 2>&1
cd ~/m10 || exit 1
if [ -e ~/.ugv/sessions/$NAME.json ]; then echo "$NAME already recorded" >&2; exit 1; fi
setsid nohup /usr/bin/python3 look_trial.py run $NAME > ~/.ugv/sessions/$NAME.out 2>&1 < /dev/null &
echo "trial $NAME started, pid $!"
