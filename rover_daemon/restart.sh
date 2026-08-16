#!/bin/sh
# Reload the rover daemon after deploying a new rover_daemon.py.
#
#     ssh rpi ~/ugv/restart.sh
#
# Separate from run_daemon.sh so that its own command line cannot match the
# pattern it greps for -- a pkill run from an ssh command whose text contains the
# pattern kills that ssh session instead, which is a confusing way to lose a
# shell and cost this three attempts.
#
# Only the daemon is killed, never the supervisor. That is not tidiness: the
# supervisor holds the arguments the daemon was started with, and starting a new
# one here means guessing them. This script used to do exactly that, and guessed
# wrong -- it relaunched `run_daemon.sh` with no arguments, so a reload quietly
# dropped `--vision` and the rover lost its camera tool until somebody noticed
# `(9 tools)` in the log. The crontab entry is where the arguments are decided:
#
#     crontab -l    # @reboot /home/admin/ugv/run_daemon.sh --vision
DIR="$(cd "$(dirname "$0")" && pwd)"

if pgrep -f 'ugv/run_daemon.sh' > /dev/null; then
    pkill -f 'ugv/rover_daemon.py'
else
    # Nothing is supervising it -- after `pkill -f run_daemon.sh`, or before the
    # first boot since the crontab entry was installed. Start it the way boot
    # would, arguments and all, rather than inventing a second set here.
    args="$(crontab -l 2>/dev/null | sed -n 's|^@reboot .*run_daemon\.sh *||p' | head -1)"
    # shellcheck disable=SC2086 -- word splitting is the point: these are flags.
    setsid nohup "$DIR/run_daemon.sh" $args > /dev/null 2>&1 < /dev/null &
fi

sleep 20
tail -3 "$DIR/rover_daemon.log"
