#!/bin/sh
# Put the wifi keeper on the Pi: the three network profiles, the script, and the
# timer that runs it. Idempotent -- run it again after changing any of them.
#
#     ssh rpi 'sudo ~/ugv/wifi_roam/install.sh EverGreen'   # first time
#     ssh rpi 'sudo ~/ugv/wifi_roam/install.sh'             # script/timer only
#
# The passphrase is only needed for profiles that do not exist yet; an existing
# profile is left holding the key it already has, because a working link is not
# worth risking to a typo. It lives in `secrets/wifi.key` in the repo.
#
# This is the one part of the rover that is a systemd unit rather than a crontab
# entry like the daemon. Not a change of heart: scanning and activating a
# connection are polkit-guarded, polkit grants them to root or to an active local
# session, and `admin`'s cron is neither. Root's own crontab would do it too, but
# a minute is its finest interval and a rover crosses a house in less.

set -eu

PSK=${1:-}
NETS="TheGreatLord TheMaharaja TheGreatViking"
HERE=$(cd "$(dirname "$0")" && pwd)

[ "$(id -u)" = 0 ] || { echo "run this with sudo"; exit 1; }

for s in $NETS; do
    if nmcli -t -f NAME con show | grep -qx "$s"; then
        echo "$s: already known"
    elif [ -z "$PSK" ]; then
        echo "$s: missing, and no passphrase given -- skipped"
        continue
    else
        nmcli con add type wifi ifname wlan0 con-name "$s" ssid "$s" \
            wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PSK" > /dev/null
        echo "$s: added"
    fi
    # Autoconnect is what reconnects the rover at all; the retry limit is what
    # decides whether it still tries an hour later. NM's default of four attempts
    # blocks a profile after a handful of failures, which is exactly what a rover
    # parked out of range does before it is carried back inside.
    nmcli con mod "$s" \
        connection.autoconnect yes \
        connection.autoconnect-priority 0 \
        connection.autoconnect-retries 0
done

# Prove the script on this machine before making it the one that runs, since the
# self-test needs no radio and takes a few seconds. A copy that arrived with CRLF
# line endings, or arrived half written, fails here rather than at three in the
# morning on a rover that has driven out of range.
if [ -r "$HERE/selftest.sh" ]; then
    if out=$(sh "$HERE/selftest.sh" 2>&1); then
        echo "selftest: $(echo "$out" | tail -1)"
    else
        echo "$out" | grep -E 'FAIL|failed'
        echo "not installing"
        exit 1
    fi
fi

install -m 755 "$HERE/wifi_roam.sh" /usr/local/sbin/wifi_roam.sh
install -m 755 "$HERE/wifi_ctl.sh" /usr/local/sbin/wifi_ctl.sh
install -m 644 "$HERE/wifi-roam.service" "$HERE/wifi-roam.timer" \
    "$HERE/wifi-radio-on.service" /etc/systemd/system/

# The daemon runs as `admin` and needs two privileged things -- a scan and a
# switch -- for the console's network panel. This is the narrow way to give it
# them: one path, no arguments constrained here because wifi_ctl.sh constrains
# them itself, and no password, since a daemon has nowhere to type one.
#
# Written through a temporary file and checked before it is put in place. A
# malformed file in /etc/sudoers.d makes *every* sudo on the box fail, including
# the one that would be used to repair it, and this Pi has no console to fall back
# to.
rule=/etc/sudoers.d/wifi-roam
tmp=$(mktemp)
printf '%s ALL=(root) NOPASSWD: /usr/local/sbin/wifi_ctl.sh\n' "${SUDO_USER:-admin}" \
    > "$tmp"
if visudo -c -q -f "$tmp"; then
    install -m 440 -o root -g root "$tmp" "$rule"
    echo "sudo rule: $(cat "$rule")"
else
    echo "refusing to install a sudoers rule that visudo will not accept" >&2
    rm -f "$tmp"
    exit 1
fi
rm -f "$tmp"
systemctl daemon-reload

# The radio switch first, and `--now` on purpose: NetworkManager restores that
# switch from a state file at boot, so a rover found with its wifi off stays off
# however healthy everything else is, and running this script is then the repair
# as well as the install.
systemctl enable --now wifi-radio-on.service
systemctl enable --now wifi-roam.timer
echo "radio: $(nmcli radio wifi)"
systemctl list-timers --no-pager wifi-roam.timer
