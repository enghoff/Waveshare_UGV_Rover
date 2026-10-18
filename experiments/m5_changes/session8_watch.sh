#!/bin/bash
# M5 session 8: the photo check, run by itself as soon as loop A has finished
# its last living-room stop, so the move list is ready while the rover is still
# driving the corridor rather than after it has parked (plan-ahead rule, after
# session 5). Triggers on the stop's "viewpoint" note, which is written after
# its photo sweep, not on its "leg" note, written on arrival before it.
#
#     bash session8_watch.sh [NAME] [PLANNED.json] [session_check.py options]
#
# SESSIONS overrides where the session's record is looked for.
NAME=${1:-session8-1018}
PLANNED=${2:-session8-planned.json}
SESSIONS=${SESSIONS:-$HOME/.ugv/sessions}
REC="$SESSIONS/$NAME.json"
until grep -q '"viewpoint": "living-4"' "$REC" 2>/dev/null; do
    if [ -e "$SESSIONS/$NAME.stop" ]; then
        echo "the session stopped before loop A reached living-4" >&2
        exit 1
    fi
    sleep 2
done
cd ~/m5 && python3 session_check.py "$NAME" "$PLANNED" --sessions "$SESSIONS" "${@:3}"
