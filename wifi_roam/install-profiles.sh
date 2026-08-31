#!/bin/sh
# The rover's three NetworkManager profiles, on the onboard radio, with
# `TheGreatViking` as the one it comes up on.
#
#     sh install-profiles.sh          # called by install.sh; needs root
#
# **Only `TheGreatViking` autoconnects.** The rover joins it at every boot if it
# is on the air, and joins nothing else by itself. The other two networks keep
# profiles -- passphrase, address, everything ready -- but with autoconnect off,
# so the only thing that ever puts the rover on one of them is a person pressing
# `join` in the console. That is the whole of the network policy: one network by
# default, the other two on request.
#
# Profiles are matched by the network each one is *for*, never by its name. The
# rover has had profiles called `TheGreatViking-dongle` in the past, and a loop
# looking for one called `TheGreatViking` would have added a second profile for
# the same network rather than finding the one already there.
#
# Split out of install.sh so it can be driven by the self-test, which has no
# NetworkManager, no radio and no root. See selftest.sh.

set -eu

# The three house access points this rover is allowed to join, the first of them
# the one it comes up on. They share one passphrase.
HOME_NET=${HOME_NET:-TheGreatViking}
NETS=${NETS:-"TheGreatViking TheGreatLord TheMaharaja"}
# Where NetworkManager keeps its profiles and its drop-in configuration, and
# where the passphrase for a new profile is read from. All overridable for the
# self-test.
NM_DIR=${NM_DIR:-/etc/NetworkManager/system-connections}
NM_CONF_DIR=${NM_CONF_DIR:-/etc/NetworkManager/conf.d}

# The radio the rover uses. Everything here is pinned to it, which is the
# opposite of what this file used to do and is the point of the change: with no
# failover there is no spare radio to leave a network free for, and an unpinned
# profile is one NetworkManager could bring up on the USB dongle instead.
#
# Found rather than named, because the onboard Realtek is `wlP1p1s0` here and
# was `wlan0` on both earlier boards. It is the wireless interface that is not a
# USB one: a USB device has a `device/driver` link under `/sys/bus/usb`, which
# is what the `wlx` name and the dongle's whole existence hang off.
onboard_iface() {
    if [ -n "${WIFI_IFACE:-}" ]; then
        echo "$WIFI_IFACE"
        return
    fi
    for dir in "${SYSNET:-/sys/class/net}"/*/wireless; do
        [ -d "$dir" ] || continue
        iface=$(basename "$(dirname "$dir")")
        # `readlink` on the device link says which bus it is on. A dongle's
        # resolves through `/usb`; the onboard card's through PCI.
        case $(readlink -f "${SYSNET:-/sys/class/net}/$iface/device" 2>/dev/null) in
            */usb*) continue ;;
        esac
        echo "$iface"
        return
    done
    # No non-USB radio at all. Nothing here can be pinned sensibly, and saying so
    # is better than pinning all three to a dongle.
    echo ""
}

IFACE=$(onboard_iface)

# The address the rover answers on, put on every profile because the three house
# networks are bridged onto one LAN -- so it is reachable whichever of them the
# rover is on, including one a person chose by hand.
#
# It used to be moved between the two radios by a failover manager, and this is
# what is left of that: a fixed address on the one radio, alongside the DHCP
# lease rather than instead of it. A /32, which is the form that manager used and
# the form proven on this LAN -- the DHCP lease in the same prefix is what
# carries the subnet and default routes, so this address needs to add neither.
#
# The lease is the way back in if this address is ever taken by something else;
# NetworkManager checks for that before claiming it and logs a conflict rather
# than starting an address war.
SERVICE_IP=${SERVICE_IP:-192.168.1.80/32}

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
        # A second profile for the same network is a second answer to "how do I
        # join this", and which one NetworkManager picks is not something to
        # leave to chance when one of them may be a leftover pinned to the
        # dongle.
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
        # The name is not what a profile is matched by any more, but a name like
        # `TheGreatViking-dongle` is a leftover of an arrangement that is gone
        # and it reads as a fact about the rover.
        nmcli con mod "$keep" connection.id "$want"
        echo "$want: renamed from $keep, which no longer describes it"
        keep=$want
    fi

    # Autoconnect for the home network and nothing else. `autoconnect-retries 0`
    # is what decides whether the rover is still trying an hour later: NM's
    # default of four attempts blocks a profile after a handful of failures,
    # which is exactly what a rover parked out of range does before it is
    # carried back inside.
    if [ "$want" = "$HOME_NET" ]; then
        auto=yes
    else
        auto=no
    fi
    nmcli con mod "$keep" \
        connection.interface-name "$IFACE" \
        connection.autoconnect "$auto" \
        connection.autoconnect-priority 0 \
        connection.autoconnect-retries 0 \
        ipv4.method auto \
        ipv4.addresses "$SERVICE_IP"
    if [ "$auto" = yes ]; then
        echo "$want: on the air at boot, on ${IFACE:-any radio}, holding $SERVICE_IP"
    else
        echo "$want: ready, joined only when somebody asks, holding $SERVICE_IP"
    fi
done

# The USB dongle carries nothing. Its driver stays -- see dongle_driver/ -- and
# the interface stays on the bus, so the hardware is still there to be picked up
# again; what is gone is NetworkManager's licence to put a network on it. Without
# this, `TheGreatViking` autoconnecting on any free radio would come up on the
# dongle as well and the rover would hold the same address twice.
#
# Matched by driver rather than by interface name because the name is the
# kernel's MAC-derived `wlx...`, and a different dongle would have a different
# one while still being the same thing: a Realtek USB radio that is not the
# rover's link.
unmanaged=$NM_CONF_DIR/99-unmanaged-usb-wifi.conf
if [ -d "$NM_CONF_DIR" ]; then
    cat > "$unmanaged" <<'UNMANAGED'
# The USB Wi-Fi dongle is not a network path on this rover. Its driver is built
# and loaded (see dongle_driver/), the device enumerates, and NetworkManager
# leaves it alone. Delete this file and restart NetworkManager to hand it back.
[device-usb-wifi]
match-device=driver:rtl8xxxu
managed=0
UNMANAGED
    chmod 644 "$unmanaged"
    echo "dongle: left unmanaged (driver loaded, no connection)"
fi
