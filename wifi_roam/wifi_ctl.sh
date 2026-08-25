#!/bin/sh
# The privileged half of the rover's wifi, for the daemon to call.
#
#     wifi_ctl.sh list [iface]        # the access points last heard, without looking
#     wifi_ctl.sh scan [iface]        # ...after asking the radio to look again
#     wifi_ctl.sh join <ssid> [iface] # move the rover onto a configured network
#     wifi_ctl.sh profiles [iface]    # the networks with a passphrase on this rover
#     wifi_ctl.sh status              # both radios, from the dual-radio manager
#
# **`join` means two quite different things depending on whether the dual-radio
# manager is running**, and the difference is worth knowing because one of them
# is much better. On a rover with one radio it means what it has always meant:
# take the link down, bring another up, and every connection to the rover dies
# including the one that asked. With `wifi_dual.py` running it instead hands the
# request to the manager, which puts the *standby* radio on the network and
# moves the traffic across only once that radio is associated, addressed and
# answering the gateway -- so nothing drops at all. See wifi_dual.py.
#
# It exists because the daemon runs as `admin` and two of these need root:
# scanning and switching. On the Pi 1 that is NetworkManager, and polkit grants
# those to an active local session, which a daemon is not. On the Banana Pi it is
# wpa_supplicant, and the control socket is root:root. The alternative was a
# blanket grant; this is the narrow version, and `install.sh` gives it a
# passwordless sudo rule for this one path.
#
# **A join can only ever reach a network that is already configured here.** The
# SSID is checked against the profiles this rover holds a passphrase for before
# it is used, so the worst a caller can ask for is one of those. Nothing here
# interpolates its argument into a shell command.
#
# `list` needs no privilege and is here anyway, so that the daemon has one way in
# rather than two -- and so that the difference between a cached list and a fresh
# one is a word in one place. That difference matters on this hardware: a scan
# goes off-channel and interrupts the link, on a dongle that shares a weakly fused
# USB bus with the camera, so nothing polls `scan`.

set -u

IFACE=${IFACE:-wlan0}

# The three house access points this rover is allowed to join. Used when there
# is no NetworkManager to ask, and as the fallback when wpa_cli cannot be
# reached unprivileged -- the control socket on the Banana Pi is root:root.
NETS=${NETS:-"TheGreatLord TheMaharaja TheGreatViking"}

# Which stack to talk to. The Pi still runs NetworkManager; the Banana Pi runs
# netplan and wpa_supplicant. Override is for the self-test, so each path can
# be driven without the other tool being on PATH.
backend() {
    if [ -n "${WIFI_BACKEND:-}" ]; then
        echo "$WIFI_BACKEND"
        return
    fi
    if command -v nmcli >/dev/null 2>&1; then
        echo nmcli
    else
        echo wpa
    fi
}

find_bin() {
    # $1 = name, $2 = well-known fallback. Root's PATH has /sbin; admin's often
    # does not, and `list` is the unprivileged call.
    if command -v "$1" >/dev/null 2>&1; then
        command -v "$1"
    elif [ -x "$2" ]; then
        echo "$2"
    else
        echo "$1"
    fi
}

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

# Where the dual-radio manager leaves the whole picture, and where a request for
# a particular network is dropped for it to pick up. Both under /run, so they go
# with the process rather than outliving it -- a stale status file describing two
# healthy radios on a board where the manager died would be worse than none.
DUAL_STATUS=${DUAL_STATUS:-/run/wifi-dual.json}
DUAL_REQUEST=${DUAL_REQUEST:-/run/wifi-dual.request}
# How stale that file may be before the manager counts as not running. It writes
# it every tick, which is once a second, so fifteen seconds is a wide margin
# against a board under load and still narrow enough that a manager killed a
# minute ago does not get sent requests nothing will ever read.
DUAL_MAX_AGE=${DUAL_MAX_AGE:-15}

dual_running() {
    [ -r "$DUAL_STATUS" ] || return 1
    now=$(date +%s)
    # `stat -c %Y` on this board's coreutils; the fallback is "assume it counts",
    # because the failure that matters is sending a join to a manager that is
    # gone, and a `stat` that will not run is not evidence either way -- while
    # refusing on that basis would silently turn every console join back into the
    # kind that drops the connection asking for it.
    then=$(stat -c %Y "$DUAL_STATUS" 2>/dev/null) || return 0
    [ $((now - then)) -le "$DUAL_MAX_AGE" ]
}

# How long to wait for a roam tick that is already mid-scan, and how long to let
# a join spend associating. Both sit inside the 60 s the daemon allows this call,
# so a join that cannot happen says so rather than being cut off mid-sentence.
# The wait is the larger of the two because the roamer's scan is the slow thing
# here -- thirty-two seconds, measured -- and it only ever scans when the link is
# genuinely in trouble, so this is a queue that in practice nothing joins.
LOCK_WAIT=${LOCK_WAIT:-30}
JOIN_WAIT=${JOIN_WAIT:-25}

# NAME:TYPE, so a wifi profile can be told from the wired one and from `lo`.
profiles_nmcli() {
    nmcli -t -f NAME,TYPE con show |
        awk -F: '$2 == "802-11-wireless" { print $1 }'
}

profiles_wpa() {
    wpa=$(find_bin wpa_cli /sbin/wpa_cli)
    if out=$("$wpa" -i "$IFACE" list_networks 2>/dev/null); then
        echo "$out" | awk -F'\t' 'NR > 1 && $2 != "" { print $2 }'
        return
    fi
    # Unprivileged, the control socket refuses. The house nets are still the
    # ones this rover holds a passphrase for, and the panel needs that list
    # so it can put a join button on the ones that were heard.
    # shellcheck disable=SC2086
    printf '%s\n' $NETS
}

profiles() {
    case $(backend) in
        nmcli) profiles_nmcli ;;
        *)     profiles_wpa ;;
    esac
}

# `iw` dump and `wpa_cli scan_results` both become IN-USE:SSID:SIGNAL:SECURITY,
# which is what the daemon already parses. SIGNAL is NetworkManager's 0-100
# column, not dBm, so a Pi and a Banana Pi draw the same kind of list.
#
# dBm → 0-100 is 2*(dBm+100), clamped. Measured on this dongle: the associated
# AP at -46 becomes 100, a neighbour at -68 becomes 64.

parse_iw() {
    awk -v current="$1" '
    function esc(s) {
        gsub(/\\/, "\\\\", s)
        gsub(/:/, "\\:", s)
        return s
    }
    function qual(dbm) {
        q = int(2 * (dbm + 100) + 0.5)
        if (q < 0) return 0
        if (q > 100) return 100
        return q
    }
    function emit() {
        if (ssid == "") return
        in_use = (ssid == current) ? "*" : " "
        print in_use ":" esc(ssid) ":" qual(signal) ":" sec
    }
    /^BSS / {
        emit()
        ssid = ""; signal = -100; sec = ""
        next
    }
    /signal:/ { signal = int($2); next }
    /^[[:space:]]*SSID:/ {
        sub(/^[[:space:]]*SSID:[[:space:]]*/, "")
        ssid = $0
        next
    }
    /^[[:space:]]*RSN:/ { sec = "WPA2"; next }
    /^[[:space:]]*WPA:/ { if (sec == "") sec = "WPA"; next }
    END { emit() }
    '
}

parse_wpa_results() {
    awk -F'\t' -v current="$1" '
    function esc(s) {
        gsub(/\\/, "\\\\", s)
        gsub(/:/, "\\:", s)
        return s
    }
    function qual(dbm) {
        q = int(2 * (dbm + 100) + 0.5)
        if (q < 0) return 0
        if (q > 100) return 100
        return q
    }
    NR == 1 { next }
    NF < 5 { next }
    {
        ssid = $5
        flags = $4
        dbm = int($3)
        sec = ""
        if (flags ~ /WPA2|RSN/) sec = "WPA2"
        else if (flags ~ /WPA/) sec = "WPA"
        else if (flags ~ /WEP/) sec = "WEP"
        if (ssid == "") next
        in_use = (ssid == current) ? "*" : " "
        print in_use ":" esc(ssid) ":" qual(dbm) ":" sec
    }
    '
}

current_ssid() {
    iw=$(find_bin iw /sbin/iw)
    "$iw" dev "$IFACE" link 2>/dev/null |
        awk '/^[[:space:]]*SSID:/ { sub(/^[[:space:]]*SSID:[[:space:]]*/, ""); print; exit }'
}

list_wpa() {
    iw=$(find_bin iw /sbin/iw)
    current=$(current_ssid)
    "$iw" dev "$IFACE" scan dump 2>/dev/null | parse_iw "$current"
}

scan_wpa() {
    # Eight seconds was enough for this dongle to name TheGreatViking and a
    # handful of neighbours; the daemon will wait twenty. Poll after the third
    # second so a fast scan is not held for the whole budget, and give up at
    # twelve so a neighbourhood of one is reported rather than hung.
    wpa=$(find_bin wpa_cli /sbin/wpa_cli)
    iw=$(find_bin iw /sbin/iw)
    current=$(current_ssid)
    "$wpa" -i "$IFACE" scan >/dev/null || true
    n=0
    wait_s=${SCAN_WAIT:-3}
    give_s=${SCAN_GIVE_UP:-12}
    while [ "$n" -lt "$give_s" ]; do
        sleep 1
        n=$((n + 1))
        [ "$n" -lt "$wait_s" ] && continue
        count=$("$wpa" -i "$IFACE" scan_results 2>/dev/null | awk 'NR > 1 { n++ } END { print n+0 }')
        [ "$count" -gt 1 ] && break
    done
    results=$("$wpa" -i "$IFACE" scan_results 2>/dev/null || true)
    if [ -n "$results" ]; then
        echo "$results" | parse_wpa_results "$current"
        return
    fi
    "$iw" dev "$IFACE" scan dump 2>/dev/null | parse_iw "$current"
}

hold_join_lock() {
    # The roamer calls this script rather than duplicating the privileged half of
    # it, and by the time it does it is already holding this exact lock -- so
    # taking it again would wait out LOCK_WAIT and then refuse the join it asked
    # for. WIFI_LOCK_HELD is that caller saying so. It also leaves the stamp
    # alone, because a caller that holds the lock owns the state file: the roamer
    # writes its own strike count and clock when it hears how the join went, and
    # a second writer here would be racing it over two fields it is mid-edit on.
    if [ "${WIFI_LOCK_HELD:-0}" = 1 ]; then
        return 0
    fi
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
}

join_nmcli() {
    # Not `dev wifi connect`, which would invent a new profile and want a
    # passphrase: the profile already exists and holds the key. `-w` so that a
    # join which is never going to complete gives up inside the time the daemon
    # is prepared to wait, rather than being killed at 90 s with nobody told.
    nmcli -w "$JOIN_WAIT" con up "$1" ifname "$IFACE"
}

join_wpa() {
    wpa=$(find_bin wpa_cli /sbin/wpa_cli)
    ssid=$1
    id=$("$wpa" -i "$IFACE" list_networks | awk -F'\t' -v s="$ssid" '
        NR > 1 && $2 == s { print $1; exit }
    ')
    if [ -z "$id" ]; then
        echo "no configured network called $ssid on this rover" >&2
        exit 3
    fi
    if ! "$wpa" -i "$IFACE" select_network "$id" | grep -qx OK; then
        echo "could not select $ssid" >&2
        exit 1
    fi
    deadline=$(( $(date +%s) + JOIN_WAIT ))
    while [ "$(date +%s)" -lt "$deadline" ]; do
        state=$("$wpa" -i "$IFACE" status 2>/dev/null || true)
        echo "$state" | grep -q "^wpa_state=COMPLETED$" || { sleep 1; continue; }
        echo "$state" | grep -qxF "ssid=$ssid" || { sleep 1; continue; }
        # Re-enable the others so a later fade can still pick them. select_network
        # turns them off; leaving them off would pin the rover to this one AP.
        "$wpa" -i "$IFACE" enable_network all >/dev/null || true
        return 0
    done
    # And re-enable them just as carefully when the join did *not* work, which is
    # the case that matters more. `select_network` disabled every other network to
    # make this attempt; returning without undoing that would leave a wifi-only
    # board holding one network it has just proved it cannot join and forbidden
    # from trying the two it can -- a rover that would have recovered by itself in
    # a minute, needing a person instead.
    "$wpa" -i "$IFACE" enable_network all >/dev/null || true
    echo "could not associate with $ssid" >&2
    exit 1
}

# Which radio a command is about. Every verb below takes it as a trailing
# argument, defaulting to wlan0, which is the onboard radio on this board and the
# only radio on the Pi -- so every existing caller keeps working unchanged.
case ${1:-} in
    list|scan|profiles) [ -n "${2:-}" ] && IFACE=$2 ;;
    join)               [ -n "${3:-}" ] && IFACE=$3 ;;
esac

case ${1:-} in
    status)
        # Unprivileged on purpose. The console polls this every few seconds and
        # it is the whole picture of both radios, so it must not cost a `sudo`,
        # a process, or a call to the radio: the manager has already written it.
        if [ -r "$DUAL_STATUS" ]; then
            cat "$DUAL_STATUS"
        else
            echo "the dual-radio manager is not running on this rover" >&2
            exit 5
        fi
        ;;
    list|scan)
        case $(backend) in
            nmcli)
                rescan=no
                [ "$1" = scan ] && rescan=yes
                nmcli -t -f IN-USE,SSID,SIGNAL,SECURITY dev wifi list --rescan "$rescan"
                ;;
            *)
                if [ "$1" = scan ]; then
                    scan_wpa
                else
                    list_wpa
                fi
                ;;
        esac
        ;;
    profiles)
        profiles
        ;;
    radio-on)
        # Asked for at every boot by wifi-radio-on.service, and by the roamer when
        # it finds a link with no association and a switch that is off. Idempotent
        # and silent on a radio that is already on, which is every boot but the
        # bad one.
        #
        # **Nothing in this repository ever turns that switch off**, on either
        # stack, and both of them restore it across a reboot -- NetworkManager
        # from a state file of its own, netplan's from systemd's saved rfkill
        # state. So an `off` that was interrupted, or that nothing checked, does
        # not cost one boot; it costs every boot after it, on a board whose only
        # other way in is an ethernet cable that a rover does not have.
        case $(backend) in
            nmcli) nmcli radio wifi on ;;
            *)     $(find_bin rfkill /usr/sbin/rfkill) unblock wifi ;;
        esac
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
        # With two radios there is a way to do this that costs nothing, so take
        # it. The manager owns both radios and will not be argued with by a join
        # behind its back -- it re-pins every radio to its intended network once
        # a second, so a direct `select_network` here would be undone inside a
        # second and would have dropped the link on the way. Handing it the
        # request instead gets the standby moved and the traffic transferred
        # after it, and the caller keeps its connection.
        if dual_running; then
            # The interface is only named if the caller named one, so the manager
            # is free to pick the spare -- which is the whole point.
            if [ -n "${3:-}" ]; then
                printf '{"ssid": "%s", "iface": "%s", "carry": true}\n' \
                    "$ssid" "$3" > "$DUAL_REQUEST"
            else
                printf '{"ssid": "%s", "carry": true}\n' "$ssid" > "$DUAL_REQUEST"
            fi
            chmod 600 "$DUAL_REQUEST" 2>/dev/null || true
            echo "asked the wifi manager for $ssid; watch status for how it goes"
            exit 0
        fi

        # Nothing else may be moving the link while this does. A roamer already
        # mid-check is waited out rather than fought with; one that has got as far
        # as its own association finishes first and is then overridden by this,
        # which is the right way round -- the person asking wins.
        hold_join_lock
        case $(backend) in
            nmcli) join_nmcli "$ssid" ;;
            *)     join_wpa "$ssid" ;;
        esac
        ;;
    *)
        echo "usage: wifi_ctl.sh list|scan|profiles|status [iface] |" \
             "join <ssid> [iface]" >&2
        exit 2
        ;;
esac
