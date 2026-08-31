#!/bin/sh
# Start the drive console at boot. Idempotent -- run it again after a deploy.
#
#     ssh rover '~/ugv/drive_web/install.sh'
#
# This is the rover user's own crontab, so it needs no sudo. A crontab write
# wants a `sync` behind it: the Banana Pi mounted its root with commit=120, and
# a reset before the write landed undid the entry. That happened once here.
#
# The entry is written from this script's own location rather than from a
# spelled-out /home/admin, because cron will not expand `~` and the rover has
# not always been the same board with the same user: it was admin on the Banana
# Pi and is jetson on the Orin, and a path naming the wrong one installs an
# entry that silently does nothing at every boot.

set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
LINE="@reboot $HERE/run_drive_web.sh"
current=$(crontab -l 2>/dev/null || true)

if printf '%s\n' "$current" | grep -q 'drive_web/run_drive_web.sh'; then
    echo "crontab already starts the drive console"
else
    if [ -n "$current" ]; then
        printf '%s\n%s\n' "$current" "$LINE" | crontab -
    else
        printf '%s\n' "$LINE" | crontab -
    fi
    echo "added: $LINE"
fi
sync
