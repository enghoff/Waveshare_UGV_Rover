#!/bin/sh
# Put the network recorder on the rover and start it. Idempotent -- run it again
# after changing anything here.
#
#     scp -r netwatch bpi-m4zero:~/ugv/
#     cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" ~/ugv/netwatch/install.sh'
#
# It is a systemd unit rather than a `@reboot` crontab entry like the daemon for
# two reasons that the daemon does not have. It has to be running before the
# first association, so that a boot which never reaches the network is recorded
# from the beginning rather than from whenever cron got to it; and it has to be
# stopped with SIGTERM on the way down, because the record it writes when asked
# to stop is the only thing that distinguishes a shutdown somebody asked for from
# a board that fell over. cron gives neither.
#
# Reading the log needs no privilege. /var/lib/netwatch and the log inside it are
# left world-readable on purpose, so `netwatch_report.py` works over an ordinary
# ssh session -- diagnosing a rover that keeps disappearing should not also need
# a password typed into it.

set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
LOGDIR=/var/lib/netwatch

[ "$(id -u)" = 0 ] || { echo "run this with sudo"; exit 1; }

# Prove the code on this machine before making it the thing that runs, since the
# self-test needs no radio and takes a second. A copy that arrived with CRLF line
# endings, or arrived half written, fails here rather than at three in the morning
# on a rover that has stopped answering.
if [ -r "$HERE/selftest.py" ]; then
    if out=$(python3 "$HERE/selftest.py" 2>&1); then
        echo "selftest: $(echo "$out" | tail -1)"
    else
        echo "$out" | grep -E 'FAIL|Error' || echo "$out" | tail -5
        echo "not installing"
        exit 1
    fi
fi

install -m 755 "$HERE/netwatch.py" /usr/local/sbin/netwatch.py
install -m 755 "$HERE/netwatch_report.py" /usr/local/bin/netwatch-report
install -m 644 "$HERE/netwatch.service" /etc/systemd/system/netwatch.service

mkdir -p "$LOGDIR"
chmod 755 "$LOGDIR"

systemctl daemon-reload
systemctl enable netwatch.service
# Restart rather than start, because this script is also how a change gets to the
# rover: `enable --now` leaves an already-running service running the old code,
# which is the deploy that looks like it worked and did nothing. The restart costs
# one `stop` record and one `boot` record saying `prev=clean`, which is exactly
# what it should say -- somebody asked for this one.
systemctl restart netwatch.service

# A service that is running is not the same as a service that is recording, and
# the difference is worth one second of waiting: a permissions mistake or a
# missing python module shows up here rather than being discovered next week when
# somebody goes looking for the outage they meant to diagnose.
sleep 2
systemctl is-active netwatch.service
chmod 644 "$LOGDIR/netwatch.log" 2>/dev/null || true
echo "--- first records"
tail -3 "$LOGDIR/netwatch.log" 2>/dev/null || echo "nothing written yet -- check journalctl -u netwatch"
echo "--- read it back with: netwatch-report"
sync
