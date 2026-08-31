#!/bin/sh
# Put the rover's wifi on the rover: the three network profiles, the privileged
# helper the console calls, and the sudo rule that lets it. Idempotent -- run it
# again after changing any of them.
#
#     ssh orin 'sudo ~/ugv/wifi_roam/install.sh'
#
# There is nothing to arm and no daemon to start. The rover joins
# `TheGreatViking` at boot because that profile autoconnects and the other two
# do not; a person at the console can look for networks and join one of the
# others; nothing moves the rover between networks on its own. See README.md.
#
# The passphrase is only needed for profiles that do not exist yet; an existing
# profile is left holding the key it already has, because a working link is not
# worth risking to a typo. It is read from `~/.ugv/wifi.key` on the rover, never
# from an argument -- see install-profiles.sh.

set -eu

HERE=$(cd "$(dirname "$0")" && pwd)

[ "$(id -u)" = 0 ] || { echo "run this with sudo"; exit 1; }

# --- what used to run here -------------------------------------------------
#
# The rover carried a dual-radio failover manager and, before it, a single-radio
# roaming timer. Both are gone: they were alternative owners of the two radios,
# and the rover now has one radio, one network it comes up on, and no opinions.
#
# This block is what removes them from a rover that still has them, and it is
# the reason a first run of this script wants a reboot afterwards. Units are
# disabled rather than stopped: stopping the failover manager takes the rover's
# service address down with it, which would cut the very SSH session running
# this install. Disabled units do not come back, and the reboot that applies the
# new profiles is also what retires the old manager -- it handles SIGTERM and
# puts the radios back on its way out.
retired=0
for unit in wifi-dual.service wifi-roam.timer wifi-roam.service \
            wifi-radio-on.service dongle-keeper.timer dongle-keeper.service; do
    if systemctl list-unit-files "$unit" 2>/dev/null | grep -q "^$unit"; then
        systemctl disable "$unit" > /dev/null 2>&1 || true
        rm -f "/etc/systemd/system/$unit"
        echo "retired: $unit"
        retired=1
    fi
done
for old in /usr/local/sbin/wifi_dual.py /usr/local/sbin/wifi_roam.sh \
           /usr/local/sbin/dongle-keeper.sh \
           /etc/sysctl.d/99-dual-wifi.conf \
           /etc/systemd/network/20-usb-wlan.link; do
    if [ -e "$old" ]; then
        rm -f "$old"
        echo "retired: $old"
        retired=1
    fi
done
[ "$retired" = 0 ] || systemctl daemon-reload

# --- the networks ----------------------------------------------------------
#
# The house networks, pinned to the onboard radio, with only the home one set to
# autoconnect. This is where the rover's whole network policy lives.
if command -v nmcli >/dev/null 2>&1; then
    sh "$HERE/install-profiles.sh"
else
    echo "no NetworkManager on this host: nothing here can configure its networks" >&2
    exit 1
fi

# Prove the helper on this machine before making it the one that runs, since the
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

# --- the helper and its sudo rule ------------------------------------------
#
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
# the one that would be used to repair it.
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

install -m 755 "$HERE/wifi_ctl.sh" /usr/local/sbin/wifi_ctl.sh
echo "helper: /usr/local/sbin/wifi_ctl.sh"

# <hostname>.local has to be advertised or `ssh orin` dies in the resolver, and
# it is the way back in if the service address is ever unreachable.
sh "$HERE/install-mdns.sh"

# --- and what is not done yet ----------------------------------------------
#
# Profiles are written, not applied. NetworkManager keeps the connection that is
# already up, so a rover mid-install stays exactly where it is and this script
# never cuts the session running it. A reboot brings up the new arrangement all
# at once: the retired units do not start, the dongle comes back unmanaged, and
# the onboard radio autoconnects the home network with the service address on it.
if [ "$retired" = 1 ]; then
    echo "--- reboot to apply: the old manager is still running until you do"
else
    echo "--- profiles updated; they take effect on the next connect or reboot"
fi
