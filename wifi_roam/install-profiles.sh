#!/bin/sh
# One NetworkManager profile per house network, and every one of them usable on
# either radio.
#
#     sh install-profiles.sh          # called by install.sh; needs root
#
# **Nothing here is pinned to an interface.** A profile tied to one radio is a
# profile the other radio cannot use, and that is the whole of what the second
# radio is for: whichever one is spare takes an access point the active one is
# not on, so that a router going down does not take both. Pinning also made the
# interface names load-bearing, which is why `20-usb-wlan.link` could not be
# installed on the Jetson -- renaming the dongle would have taken its profile
# away from it.
#
# Profiles are matched by the network each one is *for*, never by its name. The
# rover had one called `TheGreatViking-dongle`, from the days when each radio
# had its own, and a loop looking for a profile called `TheGreatViking` would
# have added a second profile for the same network rather than finding it.
#
# Split out of install.sh so it can be driven by the self-test, which has no
# NetworkManager, no radio and no root. See selftest.sh.

set -eu

# The three house access points this rover is allowed to join. They share one
# passphrase.
NETS=${NETS:-"TheGreatLord TheMaharaja TheGreatViking"}
# Where NetworkManager keeps its profiles, and where the passphrase for a new
# one is read from. Both overridable for the self-test.
NM_DIR=${NM_DIR:-/etc/NetworkManager/system-connections}

# The passphrase is only needed for a profile that does not exist yet; an
# existing one is left holding the key it already has, because a working link is
# not worth risking to a typo.
#
# It is read from a file the rover keeps outside the deploy tree, and never
# taken as an argument, because an argument is visible in `ps` to every account
# on the machine and survives in shell history and in whatever transcript the
# install was run from. Put it there once, from the copy in `secrets/`:
#
#     scp secrets/wifi.key orin:.ugv/wifi.key && ssh orin 'chmod 600 ~/.ugv/wifi.key'
#
# Under sudo the invoking user's home is the one that has it, not root's.
if [ -z "${PSK_FILE:-}" ]; then
    home=$(getent passwd "${SUDO_USER:-$(id -un)}" 2>/dev/null | cut -d: -f6 || true)
    PSK_FILE=${home:-${HOME:-/root}}/.ugv/wifi.key
fi
PSK=""
if [ -r "$PSK_FILE" ]; then
    PSK=$(tr -d '\r\n' < "$PSK_FILE")
fi

wifi_profiles() {
    # TYPE first and the prefix stripped rather than a field split, because
    # `nmcli -t` escapes a colon inside a name instead of dropping it.
    nmcli -t -f TYPE,NAME con show | sed -n 's/^802-11-wireless://p'
}

ssid_of() {
    nmcli -g 802-11-wireless.ssid con show "$1" 2>/dev/null || true
}

# A new profile is written as a keyfile rather than added with `nmcli con add`,
# for one reason: `con add` wants the passphrase as an argument, and an argument
# is world-readable in `ps` for as long as the process lives. Everything else
# about the profile is set through nmcli below, where none of it is secret.
add_profile() {
    ssid=$1
    file=$NM_DIR/$ssid.nmconnection
    uuid=$(cat /proc/sys/kernel/random/uuid 2>/dev/null || true)
    [ -n "$uuid" ] || uuid=$(uuidgen 2>/dev/null || true)
    # Only a machine with neither gets this far, which in practice means
    # the self-test, where there is no NetworkManager to mind the format.
    [ -n "$uuid" ] || uuid=$(date +%s)-$$
    (
        umask 077
        cat > "$file" <<PROFILE
[connection]
id=$ssid
uuid=$uuid
type=wifi

[wifi]
mode=infrastructure
ssid=$ssid

[wifi-security]
key-mgmt=wpa-psk
psk=$PSK

[ipv4]
method=auto

[ipv6]
method=auto
PROFILE
    )
    chmod 600 "$file"
    nmcli con reload
}

for want in $NETS; do
    keep=""
    # A line at a time, and fed in by redirection rather than through a pipe, so
    # that a profile whose name has a space in it stays one profile -- NM has one
    # called "Wired connection 1" out of the box -- and so that `keep` survives
    # the loop instead of being set inside a subshell.
    while IFS= read -r name; do
        [ -n "$name" ] || continue
        [ "$(ssid_of "$name")" = "$want" ] || continue
        if [ -z "$keep" ]; then
            keep=$name
            continue
        fi
        # A second profile for the same network is not redundancy. With nothing
        # pinning either of them, NetworkManager can bring both up -- one per
        # radio, both on the same access point -- and the rover loses the second
        # path it believes it has.
        nmcli con delete "$name" > /dev/null
        echo "$want: deleted a duplicate profile ($name)"
    done <<PROFILES
$(wifi_profiles)
PROFILES

    if [ -z "$keep" ]; then
        if [ -z "$PSK" ]; then
            echo "$want: no profile, and no passphrase in $PSK_FILE -- skipped"
            continue
        fi
        add_profile "$want"
        keep=$want
        echo "$want: added"
    elif [ "$keep" != "$want" ]; then
        # The name is not what a profile is matched by any more, but a name that
        # says which radio a network belongs to is a leftover of the arrangement
        # this script exists to undo, and it reads as a fact about the rover.
        nmcli con mod "$keep" connection.id "$want"
        echo "$want: renamed from $keep, which no longer describes it"
        keep=$want
    else
        echo "$want: already known"
    fi

    # An empty interface name is what unpins it. Autoconnect is what reconnects
    # the rover at all, and the retry limit is what decides whether it still
    # tries an hour later: NM's default of four attempts blocks a profile after
    # a handful of failures, which is exactly what a rover parked out of range
    # does before it is carried back inside -- and what left this rover's dongle
    # off the air for six hours after one link timeout.
    nmcli con mod "$keep" \
        connection.interface-name "" \
        connection.autoconnect yes \
        connection.autoconnect-priority 0 \
        connection.autoconnect-retries 0
done
