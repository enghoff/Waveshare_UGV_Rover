#!/bin/sh
# The privileged half of the rover's wifi, for the daemon to call.
#
#     wifi_ctl.sh list           # the access points NetworkManager already knows of
#     wifi_ctl.sh scan           # ...after asking the radio to look again
#     wifi_ctl.sh join <ssid>    # switch to one of the configured networks
#     wifi_ctl.sh profiles       # the networks that have a passphrase on this rover
#
# It exists because the daemon runs as `admin` and two of these need root:
# scanning and activating a connection are polkit-guarded, and polkit grants those
# to an active local session, which a daemon is not. The alternative was to give
# `admin` blanket control of NetworkManager through a polkit rule; this is the
# narrow version of the same thing, and `install.sh` grants it a passwordless sudo
# rule for this one path.
#
# **A join can only ever reach a network that is already configured here.** The
# SSID is checked against NetworkManager's own list of wifi profiles before it is
# used, so the worst a caller can ask for is one of the networks somebody has
# already put a passphrase in for. Nothing here interpolates its argument into a
# shell command.
#
# `list` needs no privilege and is here anyway, so that the daemon has one way in
# rather than two -- and so that the difference between a cached list and a fresh
# one is a word in one place. That difference matters on this hardware: a scan
# goes off-channel and interrupts the link, on a dongle that shares a weakly fused
# USB bus with the camera, so nothing polls `scan`.

set -u

IFACE=${IFACE:-wlan0}

# NAME:TYPE, so a wifi profile can be told from the wired one and from `lo`.
profiles() {
    nmcli -t -f NAME,TYPE con show |
        awk -F: '$2 == "802-11-wireless" { print $1 }'
}

case ${1:-} in
    list|scan)
        rescan=no
        [ "$1" = scan ] && rescan=yes
        nmcli -t -f IN-USE,SSID,SIGNAL,SECURITY dev wifi list --rescan "$rescan"
        ;;
    profiles)
        profiles
        ;;
    join)
        ssid=${2:-}
        if [ -z "$ssid" ]; then
            echo "join needs an SSID" >&2
            exit 2
        fi
        if ! profiles | grep -qxF -- "$ssid"; then
            echo "no configured network called $ssid on this rover" >&2
            exit 3
        fi
        # Not `dev wifi connect`, which would invent a new profile and want a
        # passphrase: the profile already exists and holds the key.
        nmcli con up "$ssid" ifname "$IFACE"
        ;;
    *)
        echo "usage: wifi_ctl.sh list|scan|profiles|join <ssid>" >&2
        exit 2
        ;;
esac
