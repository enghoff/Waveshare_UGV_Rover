#!/bin/bash
# After a staged session, on the rover: stop the recording, turn it into
# arrays, copy the world store through sqlite and colour every region, all
# into /tmp/NAME/ for the desk to fetch. The rover only sits on its charger.
#
#     bash session_collect.sh NAME
#
# Written after M5 session 5, whose collection was typed step by step
# (2026-10-16). colour.py is the visit model's (experiments/visit_model).
set -e
NAME="$1"
OUT="/tmp/$NAME"
mkdir -p "$OUT"
BAG="$HOME/ugv/ros_nav/recordings/bags/session-$NAME"
pkill -INT -f "ros2 bag record --output $BAG" || true
for i in $(seq 1 30); do pgrep -f "ros2 bag record --output $BAG" > /dev/null || break; sleep 1; done
bash -c "source ~/ugv/ros_nav/env.sh > /dev/null 2>&1; python3 ~/m5/bag_extract.py $BAG/session-${NAME}_0.mcap $OUT/$NAME.npz"
python3 - "$OUT/world-after.db" <<'EOF'
import os, sqlite3, sys
src = sqlite3.connect(os.path.expanduser("~/.ugv/world/world.db"))
dst = sqlite3.connect(sys.argv[1])
src.backup(dst)
dst.close()
print("world store copied")
EOF
python3 ~/m5/colour.py "$OUT/colour.npz" 67
cp ~/.ugv/sessions/$NAME.json ~/.ugv/sessions/$NAME.out "$OUT/" 2>/dev/null || true
cp ~/.ugv/whatwhere/shadow-$(date +%Y%m%d).jsonl "$OUT/" 2>/dev/null || true
DAY=$(date +%Y%m%d)
tar cf "$OUT/frames.tar" -C ~/.ugv/world/frames $(cd ~/.ugv/world/frames && ls | grep "^$DAY-")
ls -la "$OUT"
