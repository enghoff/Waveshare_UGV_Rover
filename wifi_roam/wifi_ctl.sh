#!/bin/sh
# The privileged half of the rover's wifi, for the daemon to call.
#
#     wifi_ctl.sh list [iface]        # the access points last heard, without looking
#     wifi_ctl.sh scan [iface]        # ...after asking the radio to look again
#     wifi_ctl.sh join <ssid> [iface] # move the rover onto a configured network
#     wifi_ctl.sh profiles [iface]    # the networks with a passphrase on this rover
#     wifi_ctl.sh startup             # ...and which of them it joins at a boot
#     wifi_ctl.sh status              # every radio, one line each
#     wifi_ctl.sh radio-on            # undo a wifi switch somebody turned off
#     wifi_ctl.sh add <ssid> <wpa-psk|sae> [iface]   # store a network and join it;
#                                     # the passphrase is the first line of stdin
#     wifi_ctl.sh forget <ssid> [iface]               # delete a stored network
#
# **`add` takes the passphrase on stdin and nowhere else.** An argument is
# readable in `ps` by every account on the machine. It is written into a
# root-only keyfile, the way install-profiles.sh writes the house networks, and
# NetworkManager files it away from there. A network added here is pinned to
# the onboard radio, keeps trying for ever, and does not autoconnect: it is
# joined now, and again whenever somebody asks, and the rover still comes up on
# the home network at a boot. If the join fails -- a wrong passphrase, a network
# gone out of range -- the new profile is deleted, so a typo is not kept, and
# the network the rover was on is brought back up.
#
# **`forget` refuses the network in use and the one the rover joins at a boot.**
# Forgetting the first cuts the link it was asked over; forgetting the second
# leaves a rover that comes up on nothing after its next power cycle, and the
# only other way in is a cable the rover does not have.
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
# One to a line: `TheGreatViking 5G` is the router's 5 GHz radio under a name
# of its own, and the space in it makes a space-separated list two networks.
NETS=${NETS:-"TheGreatViking
TheGreatViking 5G
TheGreatLord
TheMaharaja"}

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
        printf '%s\n' "$NETS"
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
    join|forget)        [ -n "${3:-}" ] && { IFACE=$3; NAMED=1; } ;;
    add)                [ -n "${4:-}" ] && { IFACE=$4; NAMED=1; } ;;
esac

# A network name this script will write into a keyfile: one to 32 bytes, no
# control characters, and none of the three things a keyfile value would read
# differently from how they were typed -- a backslash, a semicolon (a list of
# bytes to NetworkManager) and a space at either end (trimmed).
ssid_ok() {
    case $1 in
        ''|' '*|*' '|*[[:cntrl:]]*|*';'*|*'\'*) return 1 ;;
    esac
    [ "$(printf '%s' "$1" | wc -c)" -le 32 ]
}

# A WPA passphrase: 8 to 63 printable ASCII characters, or 64 hex digits.
psk_ok() {
    n=$(printf '%s' "$1" | wc -c)
    case $1 in
        *[![:print:]]*) return 1 ;;
    esac
    if [ "$n" -eq 64 ]; then
        case $1 in *[!0-9a-fA-F]*) return 1 ;; esac
        return 0
    fi
    [ "$n" -ge 8 ] && [ "$n" -le 63 ]
}

# The profile names holding a network, one to a line.
profiles_for_ssid() {
    wifi_profile_names | while IFS= read -r name; do
        ssid=$(nmcli -g 802-11-wireless.ssid con show "$name" 2>/dev/null)
        [ "${ssid:-$name}" = "$1" ] && printf '%s\n' "$name"
    done
}

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
    startup)
        # Unprivileged, like `profiles`: which stored networks autoconnect, so a
        # console can leave `forget` off the one the rover comes up on.
        command -v nmcli >/dev/null 2>&1 || exit 0
        wifi_profile_names | while IFS= read -r name; do
            [ "$(nmcli -g connection.autoconnect con show "$name" 2>/dev/null)" = yes ] ||
                continue
            ssid=$(nmcli -g 802-11-wireless.ssid con show "$name" 2>/dev/null)
            printf '%s\n' "${ssid:-$name}"
        done
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
    add)
        ssid=${2:-}
        mgmt=${3:-}
        export LC_ALL=C
        if ! ssid_ok "$ssid"; then
            echo "not a network name this rover can store" >&2
            exit 2
        fi
        case $mgmt in
            wpa-psk|sae) ;;
            *) echo "add needs wpa-psk or sae" >&2; exit 2 ;;
        esac
        IFS= read -r psk || true
        if ! psk_ok "$psk"; then
            echo "a passphrase is 8 to 63 characters, or 64 hex digits" >&2
            exit 2
        fi
        if [ -n "$(profiles_for_ssid "$ssid")" ]; then
            echo "$ssid is already stored on this rover" >&2
            exit 3
        fi
        uuid=$(cat /proc/sys/kernel/random/uuid 2>/dev/null || true)
        [ -n "$uuid" ] || uuid=$(uuidgen 2>/dev/null || true)
        # Sixteen random bytes in a UUID's shape, where neither exists -- the
        # self-test's machine, in practice.
        [ -n "$uuid" ] || uuid=$(od -An -N16 -tx1 /dev/urandom 2>/dev/null |
            tr -d ' \n' |
            sed 's/^\(.\{8\}\)\(.\{4\}\)\(.\{4\}\)\(.\{4\}\)\(.\{12\}\)$/\1-\2-\3-\4-\5/')
        [ -n "$uuid" ] || { echo "no uuid to name the profile by" >&2; exit 5; }
        # The keyfile escapes for a value: a backslash doubled, and every space
        # spelled \s, since a trailing one would otherwise be trimmed off the
        # passphrase. printf and the here-document are shell builtins, so the
        # passphrase is on no command line on the way in either.
        key=$(printf '%s' "$psk" | sed -e 's/\\/\\\\/g' -e 's/ /\\s/g')
        file=${NM_DIR:-/etc/NetworkManager/system-connections}/$uuid.nmconnection
        (
            umask 077
            cat > "$file" <<PROFILE
[connection]
id=$ssid
uuid=$uuid
type=wifi
interface-name=$IFACE
autoconnect=false
autoconnect-retries=0

[wifi]
mode=infrastructure
ssid=$ssid

[wifi-security]
key-mgmt=$mgmt
psk=$key

[ipv4]
method=auto

[ipv6]
method=auto
PROFILE
        )
        chmod 600 "$file"
        before=$(nmcli -g GENERAL.CONNECTION dev show "$IFACE" 2>/dev/null)
        if ! nmcli con load "$file" > /dev/null 2>&1; then
            rm -f "$file"
            echo "NetworkManager would not load a profile for $ssid" >&2
            exit 5
        fi
        if said=$(nmcli -w "$JOIN_WAIT" con up "$uuid" 2>&1); then
            exit 0
        fi
        # Not kept: a wrong passphrase stored is a network the console offers to
        # join that can never be joined.
        nmcli con delete "$uuid" > /dev/null 2>&1 || true
        rm -f "$file"
        back=""
        if [ -n "$before" ] && nmcli -w "$JOIN_WAIT" con up "$before" > /dev/null 2>&1
        then
            back="; back on $before"
        fi
        echo "could not join $ssid, and it was not stored$back: $(echo "$said" | tail -1)" >&2
        exit 4
        ;;
    forget)
        ssid=${2:-}
        if [ -z "$ssid" ]; then
            echo "forget needs an SSID" >&2
            exit 2
        fi
        names=$(profiles_for_ssid "$ssid")
        if [ -z "$names" ]; then
            echo "no stored network called $ssid on this rover" >&2
            exit 3
        fi
        if [ "$(current_ssid "$IFACE")" = "$ssid" ]; then
            echo "$ssid is the network in use" >&2
            exit 6
        fi
        # Read the names a line at a time from a here-document rather than a
        # pipe, so that a name with a space in it stays one name and the exit
        # below leaves the script rather than a subshell.
        while IFS= read -r name; do
            [ -n "$name" ] || continue
            if [ "$(nmcli -g connection.autoconnect con show "$name" 2>/dev/null)" = yes ]
            then
                echo "$ssid is the network the rover joins at start-up" >&2
                exit 6
            fi
        done <<NAMES
$names
NAMES
        while IFS= read -r name; do
            [ -n "$name" ] || continue
            nmcli con delete "$name" > /dev/null || exit 5
        done <<NAMES
$names
NAMES
        ;;
    *)
        echo "usage: wifi_ctl.sh list|scan|profiles|startup|status|radio-on [iface] |" \
             "join|forget <ssid> [iface] | add <ssid> <wpa-psk|sae> [iface]" >&2
        exit 2
        ;;
esac
