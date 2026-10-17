#!/bin/sh
# Reload the world model's job, and say whether it came back.
#
#     ssh orin '~/ugv/world_state/visit_model/restart_job.sh'
#     ssh orin '~/ugv/world_state/visit_model/restart_job.sh --supervisor'
#
# As restart_perception.sh: the patterns live in this file, because a `pkill -f`
# typed into an ssh command matches that command. Only the job and the lidar
# logger are killed; the supervisor starts the next ones. --supervisor replaces the supervisor too, which
# is needed when run_job.sh itself has changed.

DIR="$(cd "$(dirname "$0")" && pwd)"
LOG="$HOME/.ugv/world/visit_model/job.log"

if [ "${1:-}" = "--supervisor" ]; then
    pkill -f 'visit_model/run_job[.]sh' || true
    sleep 1
fi
if pgrep -f 'visit_model/run_job[.]sh' > /dev/null; then
    pkill -f 'visit_model/(job|scan_log)[.]py' || echo "nothing was running; the supervisor starts both within 30 s"
    sleep 31
else
    echo "--- no supervisor; starting one as the crontab does"
    setsid nohup "$DIR/run_job.sh" > /dev/null 2>&1 < /dev/null &
    sleep 5
fi
if pgrep -f 'visit_model/job[.]py' > /dev/null && pgrep -f 'visit_model/scan_log[.]py' > /dev/null; then
    echo "the world model's job and its lidar logger are running"
    tail -n 3 "$LOG" 2>/dev/null
else
    echo "the world model's job or its lidar logger is not running" >&2
    tail -n 10 "$LOG" >&2 2>/dev/null
    exit 1
fi
