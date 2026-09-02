#!/bin/sh
# Drive the two scripts in this directory with no radio present.
#
#     ./selftest.sh          # anywhere: the workstation, the rover, a VM
#
# `install-profiles.sh` decides what the rover joins and what it does not, and
# `wifi_ctl.sh` is the privileged helper the console calls. Both reach the world
# through nmcli, sysfs and /proc, and every one of those is replaceable from
# here, so a fake nmcli and a fake sysfs are enough to put the rover on a board
# with two radios, or one, or none, without owning any of them.
#
# It is run by `install.sh` before the helper is put in place, so a copy that
# arrived with CRLF line endings or arrived half written fails here rather than
# on a rover that has driven out of range.

set -u

HERE=$(cd "$(dirname "$0")" && pwd)
WORK=${TMPDIR:-/tmp}/wifi-selftest.$$
mkdir -p "$WORK/bin"
trap 'rm -rf "$WORK"' EXIT INT TERM

pass=0
fail=0

check() {   # what, expected-substring, actual
    if case "$3" in *"$2"*) true ;; *) false ;; esac; then
        pass=$((pass + 1))
        echo "  ok    $1"
    else
        fail=$((fail + 1))
        echo "  FAIL  $1"
        echo "        wanted: $2"
        echo "        got:    $(echo "$3" | tr '\n' '|')"
    fi
}

check_silent() {   # what, actual
    if [ -z "$2" ]; then
        pass=$((pass + 1))
        echo "  ok    $1"
    else
        fail=$((fail + 1))
        echo "  FAIL  $1 -- said: $(echo "$2" | tr '\n' '|')"
    fi
}

# --- the board ------------------------------------------------------------
#
# The Jetson as it is: an onboard Realtek on PCI called `wlP1p1s0`, a USB dongle
# keeping the kernel's MAC-derived name, and a wired port that is down. Built
# here rather than looked for, because the machine running this test is not that
# board -- and the interesting case, a rover whose radio is not called `wlan0`,
# cannot be arranged on it.
mkdir -p "$WORK/net/enP8p1s0" \
         "$WORK/net/wlP1p1s0/wireless" \
         "$WORK/net/wlx002e2d3074d0/wireless" \
         "$WORK/devices/pci0000:00/0000:01:00.0" \
         "$WORK/devices/platform/usb1/1-2"
ln -sf "$WORK/devices/pci0000:00/0000:01:00.0" "$WORK/net/wlP1p1s0/device"
ln -sf "$WORK/devices/platform/usb1/1-2" "$WORK/net/wlx002e2d3074d0/device"

printf 'Iface\tDestination\tGateway\tFlags\tRefCnt\tUse\tMetric\tMask\n' \
    > "$WORK/netroute"
printf 'wlx002e2d3074d0\t00000000\t0101A8C0\t0003\t0\t0\t600\t00000000\n' \
    >> "$WORK/netroute"
printf 'wlP1p1s0\t00000000\t0101A8C0\t0003\t0\t0\t100\t00000000\n' \
    >> "$WORK/netroute"

# /proc/net/wireless, for the signal `status` reports.
mkdir -p "$WORK/proc"
{
    printf 'Inter-| sta-|   Quality        |   Discarded packets\n'
    printf ' face | tus | link level noise |  nwid  crypt   frag\n'
    printf 'wlP1p1s0: 0000   66.  -41.  -256  0 0 0 0 572 0\n'
} > "$WORK/proc/wireless"

echo "the house networks, and the pinning that decides what the rover joins"
# The rover as it was found under the old failover arrangement: three unpinned
# profiles, every one of them set to autoconnect, one still named after the
# radio it used to belong to, a stray duplicate, and nothing for the third
# network. This drives the fixer against a fake NetworkManager that remembers
# what it was told, so the assertions are about the state it ends in rather than
# the commands it was sent.
mkdir -p "$WORK/nmbin" "$WORK/nm" "$WORK/nmconf"
cat > "$WORK/nmbin/nmcli" <<'FAKE'
#!/bin/sh
# name|ssid|pinned-interface|autoconnect|addresses, one line per profile.
echo "nmcli $*" >> "$NMCLI_LOG"
rewrite() { mv "$PROFILES.new" "$PROFILES"; }
case "$1 $2" in
    "-t -f")
        awk -F'|' '{ print "802-11-wireless:" $1 }' "$PROFILES"
        echo "802-3-ethernet:Wired connection 1"
        exit 0 ;;
esac
case "$*" in
    "-g 802-11-wireless.ssid con show "*)
        awk -F'|' -v n="$5" '$1 == n { print $2 }' "$PROFILES" ;;
    "-g connection.interface-name con show "*)
        awk -F'|' -v n="$5" '$1 == n { print $3 }' "$PROFILES" ;;
    "con delete "*)
        awk -F'|' -v n="$3" '$1 != n' "$PROFILES" > "$PROFILES.new"; rewrite ;;
    "con reload")
        # What NetworkManager does with a keyfile somebody wrote by hand.
        for f in "$NM_DIR"/*.nmconnection; do
            [ -e "$f" ] || continue
            id=$(sed -n 's/^id=//p' "$f")
            grep -q "^$id|" "$PROFILES" && continue
            printf '%s|%s|||\n' "$id" "$(sed -n 's/^ssid=//p' "$f")" >> "$PROFILES"
        done ;;
    "con mod "*" connection.id "*)
        awk -F'|' -v o="$3" -v n="$5" 'BEGIN { OFS = "|" }
            $1 == o { $1 = n } { print }' "$PROFILES" > "$PROFILES.new"; rewrite ;;
    "con mod "*"connection.interface-name"*)
        # The settings run is one call: read the pairs off the command line so
        # the fake records what the profile actually ends up holding.
        name=$3; shift 3
        iface=; auto=; addr=
        while [ $# -gt 1 ]; do
            case $1 in
                connection.interface-name) iface=$2 ;;
                connection.autoconnect)    auto=$2 ;;
                ipv4.addresses)            addr=$2 ;;
            esac
            shift 2
        done
        awk -F'|' -v n="$name" -v i="$iface" -v a="$auto" -v d="$addr" \
            'BEGIN { OFS = "|" }
             $1 == n { $3 = i; $4 = a; $5 = d } { print }' \
            "$PROFILES" > "$PROFILES.new"; rewrite ;;
esac
exit 0
FAKE
chmod +x "$WORK/nmbin/nmcli"

printf 'TheGreatLord|TheGreatLord||yes|\n' > "$WORK/profiles"
printf 'TheGreatViking-dongle|TheGreatViking||yes|\n' >> "$WORK/profiles"
printf 'TheGreatViking spare|TheGreatViking||yes|\n' >> "$WORK/profiles"
echo not-the-real-key > "$WORK/wifi.key"

profiles_run() {
    env PATH="$WORK/nmbin:$PATH" NMCLI_LOG="$WORK/nmcli.log" \
        PROFILES="$WORK/profiles" NM_DIR="$WORK/nm" NM_CONF_DIR="$WORK/nmconf" \
        PSK_FILE="$WORK/wifi.key" SYSNET="$WORK/net" \
        sh "$HERE/install-profiles.sh" 2>&1
}

: > "$WORK/nmcli.log"
said=$(profiles_run)

check "the network with no profile at all is given one" \
    "TheMaharaja: added" "$said"
check "and the passphrase is in the profile that was written" \
    "psk=not-the-real-key" "$(cat "$WORK/nm/TheMaharaja.nmconnection")"
# `nmcli con add` takes the passphrase as an argument, and an argument is
# readable in `ps` by every account on the machine for as long as it runs.
check_silent "and never appears on a command line" \
    "$(grep not-the-real-key "$WORK/nmcli.log" || true)"

check "the duplicate profile for one network is deleted" \
    "deleted a duplicate profile" "$said"
check "and the one named after a radio is renamed to its network" \
    "renamed from TheGreatViking-dongle" "$said"
check "every network ends up with exactly one profile" \
    "TheGreatLord TheGreatViking TheGreatViking 5G TheMaharaja" \
    "$(cut -d'|' -f2 "$WORK/profiles" | sort | tr '\n' ' ' | sed 's/ $//')"

# The router puts its 5 GHz radio on the air as `TheGreatViking 5G`. A list that
# splits on spaces makes that two networks, neither of which is on the air, and
# the console then labels the only Viking the rover can hear as one it holds no
# passphrase for.
check "the network whose name has a space in it is one network" \
    "TheGreatViking 5G: added" "$said"
check_silent "and gets one profile, not one per word" \
    "$(awk -F'|' '$2 == "TheGreatViking 5G"' "$WORK/profiles" | sed 1d)"

echo
echo "one network at boot, and only one"
# The whole of the rover's network policy. The home network comes up by itself;
# the other two are ready and wait to be asked for. A second profile set to
# autoconnect is the failover behaviour coming back by the side door -- the
# rover would join whichever it saw first and nobody would have chosen.
check "the home network is the one that autoconnects" \
    "TheGreatViking|wlP1p1s0|yes" "$(cat "$WORK/profiles")"
check_silent "and it is the only one that does" \
    "$(awk -F'|' '$4 == "yes" && $2 != "TheGreatViking"' "$WORK/profiles")"
check "the others are kept, ready to be joined by hand" \
    "TheGreatLord|wlP1p1s0|no" "$(cat "$WORK/profiles")"

# A profile that is not pinned is one NetworkManager may bring up on the dongle,
# which is the radio this rover deliberately does not use.
check_silent "not one profile is left free to land on the dongle" \
    "$(awk -F'|' '$3 != "wlP1p1s0"' "$WORK/profiles")"
check "the onboard radio is found by its bus, not by being called wlan0" \
    "on wlP1p1s0" "$said"

# The address the rover answers on. It is on every profile because the three
# networks are bridged onto one LAN, so a network chosen by hand is still
# reachable at the address the console and the certificate are written for.
check_silent "every network carries the rover's service address" \
    "$(awk -F'|' '$5 != "192.168.1.80/32"' "$WORK/profiles")"
check "and keeps its DHCP lease alongside it" \
    "ipv4.method auto" "$(cat "$WORK/nmcli.log")"
# NM gives up on a profile after four failed attempts. That is what left this
# rover's dongle off the air for six hours after a single link timeout.
check "every profile is told to keep trying for ever" \
    "connection.autoconnect-retries 0" "$(cat "$WORK/nmcli.log")"

echo
echo "the dongle, whose driver stays and whose networking does not"
check "NetworkManager is told to leave it alone" \
    "managed=0" "$(cat "$WORK/nmconf/99-unmanaged-usb-wifi.conf")"
check "and told by driver, so a replacement dongle is covered too" \
    "match-device=driver:rtl8xxxu" "$(cat "$WORK/nmconf/99-unmanaged-usb-wifi.conf")"

# Run twice: an install that is not idempotent is one nobody dares repeat, and
# this one is re-run by every system deploy.
: > "$WORK/nmcli.log"
again=$(profiles_run)
check_silent "a second run adds no profile" \
    "$(echo "$again" | grep -- ': added' || true)"
check_silent "and deletes nothing" \
    "$(echo "$again" | grep -- 'deleted a duplicate' || true)"
check "and leaves the same one network autoconnecting" \
    "TheGreatViking|wlP1p1s0|yes" "$(cat "$WORK/profiles")"

echo
echo "which radio a command is about when nobody names one"
# The Jetson's onboard Realtek is wlP1p1s0 and its dongle is wlx002e2d3074d0, so
# a helper that defaults to wlan0 asks the kernel about an interface that is not
# there and answers nothing -- which is what left the console's network panel
# empty on the day the rover became a Jetson.
cat > "$WORK/bin/nmcli" <<'FAKE'
#!/bin/sh
echo "nmcli $*" >> "$NMCLI_LOG"
case "$*" in
    "-t -f TYPE,NAME con show")
        printf '802-11-wireless:TheGreatViking\n'
        printf '802-11-wireless:TheGreatLord\n'
        printf '802-3-ethernet:Wired connection 1\n' ;;
    "-g 802-11-wireless.ssid con show "*) echo "$5" ;;
    "-g connection.interface-name con show "*) echo "${PINNED:-}" ;;
    *"dev wifi list"*)
        printf '*:TheGreatViking:88:WPA2\n :TheGreatLord:64:WPA2\n' ;;
esac
exit "${NMCLI_STATUS:-0}"
FAKE
chmod +x "$WORK/bin/nmcli"

ctl() {
    env PATH="$WORK/bin:$PATH" NMCLI_LOG="$WORK/nmcli.log" \
        SYSNET="$WORK/net" ROUTES="$WORK/netroute" WIRELESS="$WORK/proc" \
        sh "$HERE/wifi_ctl.sh" "$@" 2>&1
}

: > "$WORK/nmcli.log"
ctl list > /dev/null
check_silent "no interface is assumed to be called wlan0" \
    "$(grep 'wlan0' "$WORK/nmcli.log" || true)"
check_silent "and with no radio named the list is not narrowed to one" \
    "$(grep 'dev wifi list ifname' "$WORK/nmcli.log" || true)"

: > "$WORK/nmcli.log"
ctl list wlx002e2d3074d0 > /dev/null
check "a caller that names a radio is obeyed" \
    "dev wifi list ifname wlx002e2d3074d0" "$(cat "$WORK/nmcli.log")"

: > "$WORK/nmcli.log"
ctl scan > /dev/null
check "a scan asks the radio to look again" "--rescan yes" "$(cat "$WORK/nmcli.log")"
: > "$WORK/nmcli.log"
ctl list > /dev/null
check "and a list settles for what it last heard" \
    "--rescan no" "$(cat "$WORK/nmcli.log")"

echo
echo "profiles, and the joins they do and do not allow"
check "profiles are reported as networks, not as profile names" \
    "TheGreatViking" "$(ctl profiles)"

# With no NetworkManager to ask there is nothing but the built-in list, and that
# list is what a join is checked against. `IFACE` is given so nothing has to go
# looking for a radio, and the shell is named by path because emptying PATH is
# the whole point of the run.
mkdir -p "$WORK/empty"
check "with NetworkManager gone the built-in list keeps a spaced name whole" \
    "TheGreatViking 5G" \
    "$(env PATH="$WORK/empty" IFACE=wlP1p1s0 "$(command -v sh)" \
        "$HERE/wifi_ctl.sh" profiles)"

: > "$WORK/nmcli.log"
joined=$(env PINNED=wlP1p1s0 PATH="$WORK/bin:$PATH" NMCLI_LOG="$WORK/nmcli.log" \
    SYSNET="$WORK/net" ROUTES="$WORK/netroute" WIRELESS="$WORK/proc" \
    sh "$HERE/wifi_ctl.sh" join TheGreatLord 2>&1)
check "a join brings up the profile that already holds the passphrase" \
    "con up TheGreatLord" "$(cat "$WORK/nmcli.log")"
check_silent "and says nothing when it worked" "$joined"
# A pinned profile and a named interface contradict each other, and NM refuses
# the pair rather than choosing one. Every profile on this rover is pinned.
check_silent "and does not argue with the radio the profile is pinned to" \
    "$(grep 'con up TheGreatLord ifname' "$WORK/nmcli.log" || true)"

: > "$WORK/nmcli.log"
ctl join TheGreatViking wlx002e2d3074d0 > /dev/null
check "an unpinned profile still goes on the radio the caller named" \
    "ifname wlx002e2d3074d0" "$(cat "$WORK/nmcli.log")"

: > "$WORK/nmcli.log"
check "a network this rover has no passphrase for is refused" \
    "no configured network called" "$(ctl join Alister)"
check_silent "and nothing is brought up on the way to refusing it" \
    "$(grep 'con up' "$WORK/nmcli.log" || true)"
check "a join with no network named is refused too" \
    "join needs an SSID" "$(ctl join)"

echo
echo "status, which has to answer when everything else is broken"
# It is what a person reaches for when the console is not loading, so it goes to
# the kernel rather than to NetworkManager and needs no privilege.
cat > "$WORK/bin/iw" <<'FAKE'
#!/bin/sh
[ "$2" = wlP1p1s0 ] && echo "        SSID: TheGreatViking"
exit 0
FAKE
cat > "$WORK/bin/ip" <<'FAKE'
#!/bin/sh
case "$*" in
    *wlP1p1s0*) echo "2: wlP1p1s0    inet 192.168.1.80/32 scope global" ;;
esac
exit 0
FAKE
chmod +x "$WORK/bin/iw" "$WORK/bin/ip"

: > "$WORK/nmcli.log"
state=$(ctl status)
check "it names the network the rover is on" "TheGreatViking" "$state"
check "with the signal the driver reports" "-41 dBm" "$state"
check "and the address to reach it at" "192.168.1.80" "$state"
# The dongle is deliberately carrying nothing. A status that hid it would look
# like a radio had gone missing.
check "the dongle is listed, and honestly" "not associated" "$state"
check_silent "and none of it costs a call to NetworkManager" \
    "$(cat "$WORK/nmcli.log")"

# A host with no radio at all is the one case where there is nothing to say, and
# saying nothing quietly would read as a healthy rover with no networks.
mkdir -p "$WORK/empty"
check "a host with no radio says so rather than answering blank" \
    "no wireless interface" \
    "$(env PATH="$WORK/bin:$PATH" NMCLI_LOG="$WORK/nmcli.log" SYSNET="$WORK/empty" \
        ROUTES="$WORK/netroute" sh "$HERE/wifi_ctl.sh" status 2>&1)"

echo
echo "$pass passed, $fail failed"
[ "$fail" -eq 0 ]
