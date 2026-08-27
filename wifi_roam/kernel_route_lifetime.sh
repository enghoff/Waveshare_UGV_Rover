#!/bin/sh
# Which of the manager's routes survive the radio's address changing under them.
#
#     sudo sh kernel_route_lifetime.sh
#
# `wifi_dual.py` gives each radio a routing table of its own so a packet is
# answered out of the interface it arrived on. Those routes used to be written
# with `src <the radio's DHCP address>`, and this is the measurement that says
# why they no longer are: the kernel deletes every route carrying an address as
# its preferred source when that address goes away, in every table, without a
# word in any log. A DHCP renewal that changes the lease -- which this house
# does several times an hour, because a second server answers alongside the
# router -- therefore emptied the table that the service address's policy rule
# points at, and nothing rebuilt it until the next failover happened to.
#
# It runs on a dummy interface, so it touches no radio and can be run on the
# rover while it is flying. `wifi_world.py` models the rule this measures; if
# this script ever stops agreeing with it, the model is the one that is wrong.
set -eu

[ "$(id -u)" = 0 ] || { echo "run this with sudo"; exit 1; }

DEV=dummy9
TABLE=199
ADDR=10.99.9.2
GW=10.99.9.1

cleanup() {
    ip route flush table "$TABLE" 2>/dev/null || true
    ip link del "$DEV" 2>/dev/null || true
}
trap cleanup EXIT

cleanup
ip link add "$DEV" type dummy
ip link set "$DEV" up
ip addr add "$ADDR/24" dev "$DEV"

echo "== anchored to the address, which is what the fault was =="
ip route replace 10.99.9.0/24 dev "$DEV" scope link src "$ADDR" table "$TABLE"
ip route replace default via "$GW" dev "$DEV" src "$ADDR" table "$TABLE"
echo "with the lease:    $(ip route show table $TABLE | wc -l) routes"
ip addr del "$ADDR/24" dev "$DEV"
anchored=$(ip route show table "$TABLE" | wc -l)
echo "after it changes:  $anchored routes"

ip route flush table "$TABLE"
ip addr add "$ADDR/24" dev "$DEV"

echo "== not anchored, which is what is installed now =="
ip route replace 10.99.9.0/24 dev "$DEV" scope link table "$TABLE"
ip route replace default via "$GW" dev "$DEV" onlink table "$TABLE"
echo "with the lease:    $(ip route show table $TABLE | wc -l) routes"
ip addr del "$ADDR/24" dev "$DEV"
free=$(ip route show table "$TABLE" | wc -l)
echo "after it changes:  $free routes"
ip route show table "$TABLE" | sed 's/^/    /'

echo
if [ "$anchored" -eq 0 ] && [ "$free" -eq 2 ]; then
    echo "as expected: anchored routes die with the address, unanchored ones do not"
else
    echo "UNEXPECTED: anchored=$anchored unanchored=$free -- the fix rests on this"
    exit 1
fi
