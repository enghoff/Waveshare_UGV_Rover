#!/bin/sh
# Put the second radio to work: name it, give it the house networks, and install
# the manager that decides which of the two carries the rover's traffic.
#
#     python deploy/deploy.py --only wifi_roam --system        # installs, armed
#     sudo sh ~/ugv/wifi_roam/install-dual.sh                  # installed, off
#     sudo sh ~/ugv/wifi_roam/install-dual.sh DUAL=on          # and armed
#
# Idempotent. Run it again after changing the manager, the unit, or the link
# file. Separate from `install.sh` for the same reason `install-mdns.sh` is: it
# is a distinct act with its own arming step, and it is the riskier of the two.
#
# It works on both of the boards this rover has had, and three sections below
# are for the older one only -- naming the dongle and giving it a netplan stanza,
# neither of which NetworkManager needs. See wifi_dual.py.
#
# **It leaves the manager installed and switched off unless told otherwise**,
# which is the same rule `ROAM=off` follows in `install.sh` and for the same
# reason. This board is wifi-only, has no ethernet socket and no console, and
# everything here reaches for the radios. The way it fails is a rover that needs
# carrying to a socket, so it gets armed with somebody in the building:
#
#     systemctl enable --now wifi-dual
#
# What the two halves must never do is run at once. `wifi_roam.sh` moves one
# radio when it thinks the link has failed; the manager holds both radios where
# it put them and re-pins them every second. Between them that is a fight with
# the rover's only way in as the prize, so arming one disables the other.

set -eu

for arg in "$@"; do
    case $arg in
        DUAL=*|SERVICE_IP=*) eval "$arg" ;;
        *) echo "unknown argument: $arg" >&2; exit 2 ;;
    esac
done
DUAL=${DUAL:-off}
SERVICE_IP=${SERVICE_IP:-192.168.1.80}
HERE=$(cd "$(dirname "$0")" && pwd)

[ "$(id -u)" = 0 ] || { echo "run this with sudo"; exit 1; }

# Which stack this board has, because three sections below are only for one of
# them. The Banana Pi ran netplan with a supplicant per interface, and there the
# dongle had to be renamed to wlan1 and given a copy of wlan0's netplan stanza
# before it had any networks at all. NetworkManager needs neither: the profiles
# already exist, none of them is tied to an interface, so either radio can use
# any of them, and the manager finds its radios out of sysfs rather than by name.
if command -v nmcli >/dev/null 2>&1; then
    HAS_NM=1
else
    HAS_NM=0
fi

# Prove it here before making it the thing that holds the network. The self-test
# needs no radio, and its first act is to replay a recording of this rover losing
# the network and check that the manager grades that recording exactly as the
# rover graded it -- so a copy that arrived with CRLF endings, a half-written
# file, or a change that quietly altered how a link is judged all fail on this
# board rather than at three in the morning out of range.
if [ -r "$HERE/test_wifi_dual.py" ]; then
    if out=$(cd "$HERE" && python3 test_wifi_dual.py 2>&1); then
        echo "selftest: $(echo "$out" | tail -1)"
    else
        echo "$out" | grep -E 'FAIL|failed' || echo "$out" | tail -5
        echo "not installing"
        exit 1
    fi
fi

install -m 755 "$HERE/wifi_dual.py" /usr/local/sbin/wifi_dual.py
install -m 644 "$HERE/wifi-dual.service" /etc/systemd/system/
install -m 644 "$HERE/99-dual-wifi.conf" /etc/sysctl.d/
# The .link file renames the dongle to wlan1, and it is only wanted where
# something depends on that name. Under NetworkManager nothing does, and
# installing it would rename an interface out from under whatever profile is
# currently active on it for no gain at all.
if [ "$HAS_NM" = 0 ]; then
    install -m 644 "$HERE/20-usb-wlan.link" /etc/systemd/network/
fi
sysctl -q -p /etc/sysctl.d/99-dual-wifi.conf
echo "arp_ignore=$(sysctl -n net.ipv4.conf.all.arp_ignore)" \
     "arp_announce=$(sysctl -n net.ipv4.conf.all.arp_announce)"

if [ "$HAS_NM" = 1 ]; then
    # Neither of the next two sections is wanted here. The manager
    # finds its radios in sysfs, so no name has to be arranged, and
    # the house networks are already installed as profiles that any
    # radio may use -- see install-profiles.sh.
    echo "NetworkManager: interface names and profiles left alone"
else
    # --- name the dongle ---------------------------------------------------------
    #
    # A .link file is only consulted when the device appears, so an adapter that is
    # already up keeps whatever name it has -- and udev will not rename an interface
    # that is administratively up, which is the trap here: `udevadm trigger` on a
    # live interface reports success and changes nothing.
    usbwlan=""
    for path in /sys/class/net/*; do
        name=$(basename "$path")
        [ -d "$path/wireless" ] || continue
        [ "$name" = wlan1 ] && { usbwlan=wlan1; break; }
        case $(readlink -f "$path/device" 2>/dev/null) in
            *usb*) usbwlan=$name ;;
        esac
    done
    udevadm control --reload
    if [ -n "$usbwlan" ] && [ "$usbwlan" != wlan1 ]; then
        echo "renaming $usbwlan to wlan1"
        ip link set "$usbwlan" down || true
        udevadm trigger --action=add --subsystem-match=net --sysname-match="$usbwlan"
        udevadm settle || true
        usbwlan=$(for path in /sys/class/net/*; do
            n=$(basename "$path"); [ -d "$path/wireless" ] || continue
            case $(readlink -f "$path/device" 2>/dev/null) in *usb*) echo "$n" ;; esac
        done)
    fi
    echo "usb radio: ${usbwlan:-none found}"

    # --- give it the house networks ---------------------------------------------
    #
    # Copied from the stanza wlan0 already has rather than asked for again, so the
    # passphrase never has to come near this script or an ssh command line, and so
    # that a network added to one radio cannot silently be missing from the other.
    # Written 600 because netplan refuses to read a world-readable file holding a
    # passphrase, and says so in a way that reads as a syntax error.
    SRC=/etc/netplan/30-wifi.yaml
    DST=/etc/netplan/31-wifi-usb.yaml
    if [ ! -r "$SRC" ]; then
        echo "no $SRC to copy the networks from; skipping the wlan1 stanza" >&2
    elif [ "${usbwlan:-}" != wlan1 ]; then
        echo "no wlan1 yet, so not writing $DST" >&2
    else
        tmp=$(mktemp)
        sed 's/^\( *\)wlan0:/\1wlan1:/' "$SRC" > "$tmp"
        if cmp -s "$tmp" "$DST" 2>/dev/null; then
            echo "$DST: unchanged"
            rm -f "$tmp"
        else
            install -m 600 -o root -g root "$tmp" "$DST"
            rm -f "$tmp"
            echo "$DST: written from $SRC"
            # `netplan generate` rather than `netplan apply`, deliberately. `apply`
            # reconfigures every interface including the one this ssh session is
            # arriving on, which drops the link for ten seconds and reads from a desk
            # exactly like a board that has fallen over. `generate` writes the units
            # and the supplicant config without touching anything that is running,
            # and then only the new interface is brought up.
            netplan generate
            systemctl daemon-reload
            # `start`, not `enable --now`. netplan's generated supplicant units carry
            # no [Install] section on purpose -- they are pulled in by the generated
            # .network file rather than wanted by a target -- so enabling one prints
            # six lines of explanation and does nothing, which reads as a failure and
            # is not one.
            systemctl start netplan-wpa-wlan1.service || true
            networkctl reload || systemctl reload systemd-networkd || true
        fi
    fi

fi

systemctl daemon-reload

# What it makes of this rover right now, deciding everything and doing none
# of it. One tick, and the one line of this install that proves the manager
# can actually read this particular board -- both radios, their signals, and
# which access point each would be put on.
#
# Before the arming below and not after it, and it gates the arming: a manager
# that cannot read the board is one that should not be started on it, and
# finding that out from a running service is finding it out the hard way.
echo "--- one dry run"
if ! SERVICE_IP="$SERVICE_IP" /usr/local/sbin/wifi_dual.py --dry-run --once; then
    echo "the dry run failed on this board, so nothing is being armed" >&2
    exit 1
fi

# --- arm it, or do not -------------------------------------------------------
if [ "$DUAL" = on ]; then
    # One thing moves this rover's radios at a time. The roamer's whole model is
    # single-radio -- notice the link has failed, spend an association finding
    # another -- and the manager's is the opposite, so leaving both enabled is a
    # fight over the only way into the board.
    if systemctl is-enabled wifi-roam.timer >/dev/null 2>&1; then
        systemctl disable --now wifi-roam.timer
        echo "wifi-roam.timer: disabled; the dual-radio manager owns the radios now"
    fi
    # `enable` then `restart`, not `enable --now`. `--now` starts a unit that is
    # stopped and does nothing at all to one that is already running, so running
    # this installer to deploy a change left the *old* manager holding both
    # radios and reported success -- the same trap `sweep.sh` fell into over in
    # ros_nav/, where a reload came back reporting one of each node and running
    # the previous deploy's code.
    systemctl enable wifi-dual
    systemctl restart wifi-dual
    echo "restarting: it waits a minute before touching anything, on purpose"
    sleep 5
    systemctl --no-pager --lines=3 status wifi-dual || true
else
    systemctl disable wifi-dual >/dev/null 2>&1 || true
    echo "wifi-dual: installed and left switched off (DUAL=off)"
    echo "  see what it would decide: wifi_dual.py --dry-run --once"
    echo "  arm it with:              systemctl enable --now wifi-dual"
fi

