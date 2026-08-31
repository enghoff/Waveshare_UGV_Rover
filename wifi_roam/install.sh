#!/bin/sh
# Put the wifi keeper on the rover: the three network profiles, the script, and the
# timer that runs it. Idempotent -- run it again after changing any of them.
#
#     ssh bpi-m4zero 'sudo ~/ugv/wifi_roam/install.sh EverGreen'   # first time
#     ssh bpi-m4zero 'sudo ~/ugv/wifi_roam/install.sh'             # script/timer only
#     ssh orin 'sudo ~/ugv/wifi_roam/install.sh --no-roamer'       # everything but the keeper
#
# `--no-roamer` puts down the house networks and the privileged helper the
# console needs to list and switch between them, and stops there -- no roamer,
# no timer, no units. That is what the Jetson wants: it runs NetworkManager,
# where the keeper does not, and the keeper is a one-radio script that would
# fight the two radios this rover runs at once. See docs/hosts.md.
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

NO_ROAMER=0
DUAL=0
case ${1:-} in
    --no-roamer) NO_ROAMER=1; shift ;;
    # Everything --no-roamer does, and then the dual-radio manager armed on top.
    # The manager and the one-radio roamer are alternative owners of the same
    # radios and must never both run, so this is the pair that goes together.
    --dual)      NO_ROAMER=1; DUAL=1; shift ;;
esac
PSK=${1:-}
NETS="TheGreatLord TheMaharaja TheGreatViking"
HERE=$(cd "$(dirname "$0")" && pwd)

[ "$(id -u)" = 0 ] || { echo "run this with sudo"; exit 1; }

# The house networks themselves. Installed on every host that has
# NetworkManager, whether or not the roamer is going on with them, because a
# rover that cannot see a network it holds the key for is the fault this
# whole directory exists to prevent. The Banana Pi runs netplan and
# wpa_supplicant and has no nmcli: its networks live in /etc/netplan, so
# there is nothing to add there.
if command -v nmcli >/dev/null 2>&1; then
    sh "$HERE/install-profiles.sh"
else
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
if [ "$NO_ROAMER" = 0 ]; then
    install -m 755 "$HERE/wifi_roam.sh" /usr/local/sbin/wifi_roam.sh
    install -m 644 "$HERE/wifi-roam.service" "$HERE/wifi-roam.timer" \
        "$HERE/wifi-radio-on.service" /etc/systemd/system/
fi

if [ "$NO_ROAMER" = 1 ]; then
    echo "no roamer: the networks and the helper are in place; no timer, no units"
    if [ "$DUAL" = 1 ]; then
        echo "--- and the dual-radio manager"
        sh "$HERE/install-dual.sh" DUAL=on
    else
        echo "  the console can list and switch networks; nothing moves them by itself"
    fi
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
