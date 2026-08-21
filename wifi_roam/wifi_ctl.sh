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

# The roamer's own lock and state file, which a deliberate join has to touch for
# two separate reasons.
#
# It has to hold the lock, because `nmcli con up` takes the interface down for the
# ten seconds it spends associating, and a roam tick that reads /proc inside that
# window sees a rover with no association and no address, decides that is a fault
# and goes off to choose a network of its own. That is not a hypothetical: it is
# how a hand-picked network lasted 43 seconds before the rover was carried back to
# the one it came from.
#
# And it has to clear the strike count and stamp the clock, so that the network
# somebody chose gets the same cooldown as one the roamer chose itself, instead of
# being graded on the first reading taken after it arrives.
LOCK=${LOCK:-/run/wifi-roam.lock}
STATE=${STATE:-/run/wifi-roam.state}

# How long to wait for a roam tick that is already mid-scan, and how long to let
# nmcli spend associating. Both sit inside the 60 s the daemon allows this call, so
# a join that cannot happen says so rather than being cut off mid-sentence. The
# wait is the larger of the two because the roamer's scan is the slow thing here --
# thirty-two seconds, measured -- and it only ever scans when the link is genuinely
# in trouble, so this is a queue that in practice nothing joins.
LOCK_WAIT=${LOCK_WAIT:-30}
JOIN_WAIT=${JOIN_WAIT:-25}

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
        # Nothing else may be moving the link while this does. A roamer already
        # mid-check is waited out rather than fought with; one that has got as far
        # as its own `con up` finishes first and is then overridden by this, which
        # is the right way round -- the person asking wins.
        if command -v flock > /dev/null 2>&1; then
            exec 9> "$LOCK"
            if ! flock -w "$LOCK_WAIT" 9; then
                echo "the wifi keeper is busy checking the link; try again" >&2
                exit 4
            fi
        fi
        # Stamped before the join and not after it, so that a roam tick running the
        # moment this releases the lock already reads a fresh clock and no strikes.
        # Written with the same two fields wifi_roam.sh reads back, and its failure
        # is not this command's failure: a join that worked should not be reported
        # as broken because /run was not writable.
        printf '0 %s\n' "$(date +%s)" > "$STATE" 2>/dev/null || true
        # Not `dev wifi connect`, which would invent a new profile and want a
        # passphrase: the profile already exists and holds the key. `-w` so that a
        # join which is never going to complete gives up inside the time the daemon
        # is prepared to wait, rather than being killed at 90 s with nobody told.
        nmcli -w "$JOIN_WAIT" con up "$ssid" ifname "$IFACE"
        ;;
    *)
        echo "usage: wifi_ctl.sh list|scan|profiles|join <ssid>" >&2
        exit 2
        ;;
esac
