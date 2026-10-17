#!/bin/sh
# Start the world model's job at every boot: the crontab entry for the rover
# user that every service here has. Run once on a new rover.
#
#     ssh orin '~/ugv/world_state/visit_model/install_job.sh'

HERE="$(cd "$(dirname "$0")" && pwd)"
LINE="@reboot $HERE/run_job.sh"
current=$(crontab -l 2>/dev/null || true)
if printf '%s\n' "$current" | grep -q 'visit_model/run_job.sh'; then
    echo "crontab already starts the world model's job"
elif [ -n "$current" ]; then
    printf '%s\n%s\n' "$current" "$LINE" | crontab -
    echo "added: $LINE"
else
    printf '%s\n' "$LINE" | crontab -
    echo "added: $LINE"
fi
