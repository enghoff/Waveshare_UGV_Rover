#!/bin/sh
# Advertise this host as <hostname>.local so a desk can find it without an
# address. Idempotent -- run it again after changing the drop-ins.
#
#     ssh orin 'sudo ~/ugv/wifi_roam/install-mdns.sh'
#
# Raspberry Pi OS already ships avahi-daemon, which is why `rpi.local` resolved.
# Ubuntu on the Banana Pi does not, and systemd-resolved has MulticastDNS off,
# so `ssh bpi-m4zero` looked up bpi-m4zero.local and the LAN had nobody answering.
# This is the missing half: either confirm avahi is already doing it, or turn
# resolved's own responder on and pin it so a reboot does not forget.

set -eu

[ "$(id -u)" = 0 ] || { echo "run this with sudo"; exit 1; }

HERE=$(cd "$(dirname "$0")" && pwd)
HOST=$(hostname)

if systemctl is-active --quiet avahi-daemon 2>/dev/null; then
    echo "mdns: $HOST.local via avahi-daemon (already running)"
    exit 0
fi

if ! systemctl is-active --quiet systemd-resolved 2>/dev/null; then
    echo "mdns: neither avahi-daemon nor systemd-resolved is running;" >&2
    echo "mdns: $HOST.local will not resolve" >&2
    exit 1
fi

# Global default, so a new interface inherits it rather than coming up silent.
install -d /etc/systemd/resolved.conf.d
install -m 644 "$HERE/mdns-resolved.conf" \
    /etc/systemd/resolved.conf.d/mdns.conf

# netplan writes the .network units into /run. A drop-in of the same name under
# /etc still applies, and is what survives a reboot; `resolvectl mdns` below is
# only until then. Do not `networkctl reload` from here -- that has bounced this
# link before, and this script is reached over it.
if [ -d /run/systemd/network ]; then
    for unit in /run/systemd/network/10-netplan-*.network; do
        [ -f "$unit" ] || continue
        name=$(basename "$unit")
        dest=/etc/systemd/network/${name}.d
        install -d "$dest"
        install -m 644 "$HERE/mdns-networkd.conf" "$dest/mdns.conf"
    done
fi

systemctl reload-or-restart systemd-resolved

# Immediate, so this run is the proof rather than the next reboot.
for iface in /sys/class/net/*; do
    name=$(basename "$iface")
    [ "$name" = lo ] && continue
    resolvectl mdns "$name" yes >/dev/null 2>&1 || true
done

echo "mdns: $HOST.local via systemd-resolved"
resolvectl status | sed -n 's/^/  /; /Protocols:/p; /Link /p'
