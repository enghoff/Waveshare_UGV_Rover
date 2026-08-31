#!/bin/sh
# Notice when the spare radio has stopped working, and reload its driver.
#
#     keeper.sh          # one check; the timer runs this every minute
#     keeper.sh -n       # say what it would do, and do nothing
#
# **This is a backstop, not the fix.** The fix is `rx-urb-recovery.patch`, which
# stops the driver retiring its receive buffers one USB error at a time. This
# exists because the failure it addresses was silent and total for six hours on
# a rover that had nothing watching: the interface stayed up, the device stayed
# on the bus, and only a person asking found out. A radio nobody watches is not
# a spare, whatever its driver does.
#
# **It only ever touches the spare.** The interface is found by asking which one
# the `rtl8xxxu` driver has bound, never by name, and the rover's primary radio
# is a different driver on a different bus (`rtl8822ce`, PCIe). So the worst this
# can do is take down the radio that is already not working. Nothing here can
# drop the link a console or an SSH session is arriving over.
#
# It reloads rather than trying anything gentler because gentler does not work:
# once the receive path is dead, `ip link` cycles, `nmcli device reapply` and a
# fresh association all leave a radio that hears nothing. Reloading the module
# is the only thing measured to bring it back.

set -u

DRIVER=${DRIVER:-rtl8xxxu}
SYSNET=${SYSNET:-/sys/class/net}
STATE=${STATE:-/run/dongle-keeper.state}
# How many consecutive minutes of no association before the driver is reloaded.
# Three, because a radio moving between access points is legitimately down for a
# few seconds and a reload in the middle of that would be the keeper causing the
# outage it exists to end.
STRIKES=${STRIKES:-3}
# And how long to leave it alone afterwards. A reload takes a few seconds and
# NetworkManager then needs to scan and associate, so anything shorter than this
# would count that as another strike and reload on top of the last one.
COOLDOWN=${COOLDOWN:-300}
MODPROBE=${MODPROBE:-modprobe}
IW=${IW:-iw}
NOW=${NOW:-$(date +%s)}

DRY=0
[ "${1:-}" = "-n" ] && DRY=1

say() { echo "dongle-keeper: $*"; }

# Which interface the driver has, if any. Asked of sysfs rather than assumed,
# because this dongle appears under the kernel's own `wlx002e2d3074d0` on this
# board and would appear under something else on the next one -- and because the
# question that actually matters is "which interface is the one with the driver
# that goes wrong", not "which interface has this name".
#
# Read out of `device/uevent`, which names the bound driver on a line of its
# own, rather than followed through the `device/driver` symlink beside it. Both
# are true; only one of them can be built in a directory by a self-test running
# on a machine that cannot make symlinks.
find_iface() {
    for dir in "$SYSNET"/*; do
        [ -r "$dir/device/uevent" ] || continue
        if grep -qx "DRIVER=$DRIVER" "$dir/device/uevent"; then
            basename "$dir"
            return
        fi
    done
}

reload() {
    if [ "$DRY" = 1 ]; then
        say "would reload $DRIVER"
        return
    fi
    $MODPROBE -r "$DRIVER" 2>/dev/null || true
    sleep 2
    $MODPROBE "$DRIVER"
    printf '0 %s\n' "$NOW" > "$STATE" 2>/dev/null || true
}

strikes=0
last=0
if [ -r "$STATE" ]; then
    read -r strikes last < "$STATE" || { strikes=0; last=0; }
    case $strikes in ''|*[!0-9]*) strikes=0 ;; esac
    case $last in ''|*[!0-9]*) last=0 ;; esac
fi

iface=$(find_iface)

if [ -z "$iface" ]; then
    # No interface means the module is not loaded, or the device came back on
    # the bus after a reload and nothing claimed it. Both are cured by loading
    # the driver, and neither is worth three strikes: there is no link here to
    # be patient about.
    if [ $(( NOW - last )) -lt "$COOLDOWN" ]; then
        say "no $DRIVER interface, but the last reload was $(( NOW - last ))s ago; waiting"
        exit 0
    fi
    say "no interface bound to $DRIVER; loading it"
    reload
    exit 0
fi

ssid=$($IW dev "$iface" link 2>/dev/null | sed -n 's/^[[:space:]]*SSID: //p')
if [ -n "$ssid" ]; then
    [ "$strikes" -gt 0 ] && say "$iface is back on $ssid"
    printf '0 %s\n' "$last" > "$STATE" 2>/dev/null || true
    exit 0
fi

strikes=$(( strikes + 1 ))
printf '%s %s\n' "$strikes" "$last" > "$STATE" 2>/dev/null || true

if [ "$strikes" -lt "$STRIKES" ]; then
    say "$iface is not associated ($strikes of $STRIKES)"
    exit 0
fi

if [ $(( NOW - last )) -lt "$COOLDOWN" ]; then
    say "$iface is still not associated, but it was reloaded $(( NOW - last ))s ago; waiting"
    exit 0
fi

say "$iface has been unassociated for $strikes checks; reloading $DRIVER"
reload
