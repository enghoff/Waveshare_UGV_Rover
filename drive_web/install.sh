#!/bin/sh
# Start the drive console at boot. Idempotent -- run it again after a deploy.
#
#     ssh bpi-m4zero '~/ugv/drive_web/install.sh'
#
# This is admin's crontab, so it needs no sudo. A crontab write on this card
# needs a `sync` behind it: commit=120, and a reset before the write lands
# undoes the entry. That happened once here.

set -eu

LINE='@reboot /home/admin/ugv/drive_web/run_drive_web.sh'
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
