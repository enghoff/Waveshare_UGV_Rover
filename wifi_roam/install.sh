#!/bin/sh
# Put the wifi keeper on the rover: the three network profiles, the script, and the
# timer that runs it. Idempotent -- run it again after changing any of them.
#
#     ssh bpi-m4zero 'sudo ~/ugv/wifi_roam/install.sh EverGreen'   # first time
#     ssh bpi-m4zero 'sudo ~/ugv/wifi_roam/install.sh'             # script/timer only
#     ssh orin 'sudo ~/ugv/wifi_roam/install.sh --helper-only'     # console helper alone
#
# `--helper-only` puts down the privileged helper and the sudo rule the console
# needs to list and switch networks, and nothing else -- no roamer, no timer, no
# profile edits. That is what a host wants when the console's network panel is
# blank but the keeper itself has not been ported to it: the Jetson runs
# NetworkManager and holds two radios on deliberately different routers, and the
# roamer is a one-radio script that would start moving them. See docs/hosts.md.
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

HELPER_ONLY=0
if [ "${1:-}" = "--helper-only" ]; then
    HELPER_ONLY=1
    shift
fi
PSK=${1:-}
NETS="TheGreatLord TheMaharaja TheGreatViking"
HERE=$(cd "$(dirname "$0")" && pwd)

[ "$(id -u)" = 0 ] || { echo "run this with sudo"; exit 1; }

# The Pi still runs NetworkManager. The Banana Pi runs netplan and
# wpa_supplicant, and has no nmcli: its house networks live in /etc/netplan, so
# there are no profiles to add here. Everything below is installed on both, and
# the roamer works on both -- it reaches the radio through wifi_ctl.sh, which
# speaks whichever stack it finds.
if [ "$HELPER_ONLY" = 1 ]; then
    HAS_NM=0
    echo "helper only: leaving the network profiles exactly as they are"
elif command -v nmcli >/dev/null 2>&1; then
    HAS_NM=1
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
else
    HAS_NM=0
    echo "no NetworkManager: the networks live in /etc/netplan on this board"
fi

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

# The daemon needs two privileged things -- a scan and a switch -- for the
# console's network panel. This is the narrow way to give it them: one path, no
# arguments constrained here because wifi_ctl.sh constrains them itself, and no
# password, since a daemon has nowhere to type one.
#
# The rule goes down before the script it names, and not after. The other way
# round leaves a few seconds in which the helper exists and may not be run, and
# a console that asks for a scan in that window is told a password is required
# -- which is true, and says nothing about what is actually wrong. A rule naming
# a script that is not there yet fails instead as "not installed", which is the
# message that sends somebody to the right place.
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

# The helper goes down on every host, because it is the half the console
# needs, and it needs no timer, no unit and no decision about roaming.
install -m 755 "$HERE/wifi_ctl.sh" /usr/local/sbin/wifi_ctl.sh
if [ "$HELPER_ONLY" = 0 ]; then
    install -m 755 "$HERE/wifi_roam.sh" /usr/local/sbin/wifi_roam.sh
    install -m 644 "$HERE/wifi-roam.service" "$HERE/wifi-roam.timer" \
        "$HERE/wifi-radio-on.service" /etc/systemd/system/
fi

if [ "$HELPER_ONLY" = 1 ]; then
    echo "helper only: no roamer, no timer and no units on this host"
    echo "  the console can list and switch networks now; nothing roams"
    exit 0
fi

systemctl daemon-reload

# The radio switch first, and `--now` on purpose: both stacks restore that switch
# across a reboot, so a rover found with its wifi off stays off however healthy
# everything else is, and running this script is then the repair as well as the
# install.
systemctl enable --now wifi-radio-on.service

# Then the roamer, unless somebody has asked for it to be left alone. `ROAM=off`
# exists for one situation and it is worth naming: the first install on a board
# this has never run on, with nobody in the building. Every fault this thing
# answers ends in an association being spent, and an association that does not
# come back on a wifi-only rover needs a person to walk over and power-cycle it.
# So the code can be put in place while it is still switched off, and armed by
# somebody who is there to watch the first hour of it:
#
#     systemctl enable --now wifi-roam.timer
if [ "${ROAM:-on}" = off ]; then
    echo "roam timer: left disabled (ROAM=off)"
    echo "  arm it with: systemctl enable --now wifi-roam.timer"
else
    systemctl enable --now wifi-roam.timer
    systemctl list-timers --no-pager wifi-roam.timer
fi

# What the script would decide right now, without doing any of it. A dry run costs
# 64 ms on a healthy link and is the one line of this install that proves the
# thing can actually read this particular rover.
echo "--- one dry run"
/usr/local/sbin/wifi_roam.sh -n || echo "(dry run exited $?)"

# <hostname>.local has to be advertised or `ssh bpi-m4zero` dies in the
# resolver. Raspberry Pi OS already runs avahi-daemon; leave that alone.
# Ubuntu on the Banana Pi has systemd-resolved with MulticastDNS off and no
# avahi, so the name existed only in ~/.ssh/config. Enabling resolved's own
# responder is a drop-in and a restart -- no apt, and nothing that takes the
# radio off the air.
sh "$HERE/install-mdns.sh"
