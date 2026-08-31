#!/bin/sh
# The privileged half of the rover's wifi, for the daemon to call.
#
#     wifi_ctl.sh list [iface]        # the access points last heard, without looking
#     wifi_ctl.sh scan [iface]        # ...after asking the radio to look again
#     wifi_ctl.sh join <ssid> [iface] # move the rover onto a configured network
#     wifi_ctl.sh profiles [iface]    # the networks with a passphrase on this rover
#     wifi_ctl.sh status              # every radio, one line each
#     wifi_ctl.sh radio-on            # undo a wifi switch somebody turned off
#
# **`join` takes the link down and brings another up**, so every connection to
# the rover dies, including the one that asked. That is what a join costs on a
# rover with one radio, and it is why nothing calls it unprompted: the console
# offers it as a button a person presses, having seen which network they are on.
# The rover returns to `TheGreatViking` at the next reboot regardless, because
# that is the only profile that autoconnects.
#
# It exists because the daemon runs as an ordinary account and two of these need
# root: scanning and switching are polkit-guarded, and polkit grants them to an
# active local session, which a daemon is not. The alternative was a blanket
# grant; this is the narrow version, and `install.sh` gives it a passwordless
# sudo rule for this one path.
#
# **A join can only ever reach a network that is already configured here.** The
# SSID is checked against the profiles this rover holds a passphrase for before
# it is used, so the worst a caller can ask for is one of those. Nothing here
# interpolates its argument into a shell command.
#
# `list` needs no privilege and is here anyway, so that the daemon has one way in
# rather than two -- and so that the difference between a cached list and a fresh
# one is a word in one place. That difference matters on this hardware: a scan
# goes off-channel and interrupts the link.

set -u

# Where the radios and the routing table are read from, overridable so the
# self-test can build a board whose radio has whatever name it likes.
SYSNET=${SYSNET:-/sys/class/net}
ROUTES=${ROUTES:-/proc/net/route}
WIRELESS=${WIRELESS:-/proc/net}

# Which radio a command is about when nobody has named one. It used to be
# `wlan0`, which is what both earlier boards called their only radio and what
# the Jetson calls neither of its two -- the onboard Realtek is `wlP1p1s0` and
# the dongle keeps the kernel's `wlx002e2d3074d0`. So the radio is found: a
# wireless interface is the one with a `wireless/` directory beside it in sysfs,
# and the one to default to is whichever of them the kernel is currently sending
# the rover's traffic through, by lowest-metric default route. `wlan0` survives
# only as the answer for a host with no radio at all, where nothing was going to
# work anyway.
radios() {
    for dir in "$SYSNET"/*/wireless; do
        [ -d "$dir" ] || continue
        basename "$(dirname "$dir")"
    done
}

default_iface() {
    found=$(radios)
    [ -n "$found" ] || { echo wlan0; return; }
    for routed in $(awk '$2 == "00000000" { print $7, $1 }' "$ROUTES" 2>/dev/null |
                        sort -n | awk '{ print $2 }'); do
        for radio in $found; do
            [ "$routed" = "$radio" ] && { echo "$radio"; return; }
        done
    done
    echo "$found" | head -1
}

IFACE=${IFACE:-$(default_iface)}

# The house access points this rover is allowed to join, used only when there is
# no NetworkManager to ask -- which on this rover means something is badly
# wrong, but the console's list is still worth drawing.
NETS=${NETS:-"TheGreatViking TheGreatLord TheMaharaja"}

# How long to let a join spend associating. It sits inside the 60 s the daemon
# allows this call, so a join that cannot happen says so rather than being cut
# off mid-sentence.
JOIN_WAIT=${JOIN_WAIT:-25}

find_bin() {
    # $1 = name, $2 = well-known fallback. Root's PATH has /sbin; the daemon's
    # account often does not, and `list` is the unprivileged call.
    if command -v "$1" >/dev/null 2>&1; then
        command -v "$1"
    elif [ -x "$2" ]; then
        echo "$2"
    else
        echo "$1"
    fi
}

# The wifi profiles on this host, by the name NetworkManager files them under.
# TYPE first and the prefix stripped, rather than splitting fields, because `-t`
# escapes a colon inside a name instead of dropping it and a split would cut the
# name in half at it.
wifi_profile_names() {
    nmcli -t -f TYPE,NAME con show | sed -n 's/^802-11-wireless://p'
}

# ...and the network each of them is *for*, which is not the same thing. This
# rover has had a profile called `TheGreatViking-dongle`, from the days when
# each radio had its own. Reporting names here is what made the console label a
# network the rover holds the key for "no passphrase" and then refuse to join it.
profiles() {
    if ! command -v nmcli >/dev/null 2>&1; then
        # shellcheck disable=SC2086
        printf '%s\n' $NETS
        return
    fi
    wifi_profile_names | while IFS= read -r name; do
        ssid=$(nmcli -g 802-11-wireless.ssid con show "$name" 2>/dev/null)
        printf '%s\n' "${ssid:-$name}"
    done
}

# The other direction, for the join, which has to name a profile rather than a
# network. Nothing is returned for a network this rover has no profile for, and
# the caller has already refused that case by then.
profile_for_ssid() {
    wifi_profile_names | while IFS= read -r name; do
        ssid=$(nmcli -g 802-11-wireless.ssid con show "$name" 2>/dev/null)
        if [ "${ssid:-$name}" = "$1" ]; then
            printf '%s\n' "$name"
            break
        fi
    done
}

current_ssid() {
    iw=$(find_bin iw /sbin/iw)
    "$iw" dev "$1" link 2>/dev/null |
        awk '/^[[:space:]]*SSID:/ { sub(/^[[:space:]]*SSID:[[:space:]]*/, ""); print; exit }'
}

iface_address() {
    ip=$(find_bin ip /sbin/ip)
    "$ip" -4 -o addr show dev "$1" 2>/dev/null |
        awk '{ print $4 }' | cut -d/ -f1 | paste -sd, -
}

iface_dbm() {
    awk -v want="$1:" '$1 == want { print int($4); exit }' \
        "$WIRELESS/wireless" 2>/dev/null
}

# Which radio a command is about. Every verb below takes it as a trailing
# argument, and without one it is the radio carrying the rover's traffic --
# see default_iface above -- so every existing caller keeps working unchanged.
NAMED=0
case ${1:-} in
    list|scan|profiles) [ -n "${2:-}" ] && { IFACE=$2; NAMED=1; } ;;
    join)               [ -n "${3:-}" ] && { IFACE=$3; NAMED=1; } ;;
esac

case ${1:-} in
    status)
        # Unprivileged on purpose, and read from the kernel rather than from
        # NetworkManager, so that it answers on a rover whose sudo rule was
        # never installed and on one whose NetworkManager is the thing that is
        # broken. One line per radio, the dongle included -- it is deliberately
        # unmanaged here and showing it with no network is the honest report,
        # not a fault.
        found=$(radios)
        if [ -z "$found" ]; then
            echo "no wireless interface on this host" >&2
            exit 5
        fi
        for radio in $found; do
            ssid=$(current_ssid "$radio")
            dbm=$(iface_dbm "$radio")
            printf '%s\t%s\t%s\t%s\n' "$radio" \
                "${ssid:-not associated}" \
                "${dbm:+$dbm dBm}" \
                "$(iface_address "$radio")"
        done
        ;;
    list|scan)
        if ! command -v nmcli >/dev/null 2>&1; then
            echo "no NetworkManager on this host; cannot list networks" >&2
            exit 5
        fi
        rescan=no
        [ "$1" = scan ] && rescan=yes
        # Only one radio's hearing when a caller asked for one radio. Without an
        # interface nmcli merges whatever every managed radio heard, which on
        # this rover is the one radio anyway.
        if [ "$NAMED" = 1 ]; then
            nmcli -t -f IN-USE,SSID,SIGNAL,SECURITY dev wifi list \
                ifname "$IFACE" --rescan "$rescan"
        else
            nmcli -t -f IN-USE,SSID,SIGNAL,SECURITY dev wifi list \
                --rescan "$rescan"
        fi
        ;;
    profiles)
        profiles
        ;;
    radio-on)
        # **Nothing in this repository ever turns that switch off**, and
        # NetworkManager restores it across a reboot from a state file of its
        # own. So an `off` that nothing checked does not cost one boot; it costs
        # every boot after it, on a board whose only other way in is an ethernet
        # cable that a rover does not have. This is the repair, by hand.
        nmcli radio wifi on
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
        # passphrase: the profile already exists and holds the key. `-w` so that
        # a join which is never going to complete gives up inside the time the
        # daemon is prepared to wait, rather than being killed at 90 s with
        # nobody told.
        profile=$(profile_for_ssid "$ssid")
        [ -n "$profile" ] || profile=$ssid
        # Every profile on this rover is pinned to the onboard radio, and a
        # pinned profile has to come up on that radio -- naming a different one
        # here is a join that fails with the profile and the interface
        # contradicting each other. An unpinned profile still takes the radio
        # the caller named, which is what the self-test exercises.
        if [ -n "$(nmcli -g connection.interface-name con show "$profile" 2>/dev/null)" ]
        then
            nmcli -w "$JOIN_WAIT" con up "$profile"
        else
            nmcli -w "$JOIN_WAIT" con up "$profile" ifname "$IFACE"
        fi
        ;;
    *)
        echo "usage: wifi_ctl.sh list|scan|profiles|status|radio-on [iface] |" \
             "join <ssid> [iface]" >&2
        exit 2
        ;;
esac
