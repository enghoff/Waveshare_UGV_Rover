#!/bin/sh
# Drive keeper.sh through every decision it can make, with no radio present.
#
#     ./selftest.sh          # anywhere: the workstation, the rover, a VM
#
# The keeper reads one thing (which interface the driver has bound, out of
# sysfs), asks one question (`iw dev X link`) and takes one action (reload the
# module), and all three are replaceable from here. So a directory standing in
# for /sys/class/net, a fake `iw` and a fake `modprobe` are enough to put the
# rover in every state this has to tell apart: working, briefly between access
# points, dead, and dead again too soon after the last repair.
#
# The states that matter are the ones where it must do *nothing*. A keeper that
# reloads the driver too eagerly is worse than no keeper: it turns a two-second
# roam into a ten-second outage, over and over.

set -u

HERE=$(cd "$(dirname "$0")" && pwd)
WORK=${TMPDIR:-/tmp}/dongle-keeper-selftest.$$
mkdir -p "$WORK/bin" "$WORK/net"
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

# --- the world the keeper gets to see -------------------------------------

# A fake modprobe, which records rather than loads.
cat > "$WORK/bin/modprobe" <<'FAKE'
#!/bin/sh
echo "modprobe $*" >> "$LOG"
FAKE
chmod +x "$WORK/bin/modprobe"

# A fake `iw`, answering with whatever network the scenario put in $SSID.
cat > "$WORK/bin/iw" <<'FAKE'
#!/bin/sh
[ -n "${SSID:-}" ] && printf 'Connected to ec:75:0c:3d:1e:d2 (on %s)\n\tSSID: %s\n' "$2" "$SSID"
exit 0
FAKE
chmod +x "$WORK/bin/iw"

# Two interfaces: the onboard radio, which this must never touch, and the
# dongle. The keeper tells them apart by which driver has them bound, because
# the names differ from board to board and only the driver is a fact.
mkdir -p "$WORK/net/wlP1p1s0/device" "$WORK/net/wlx002e2d3074d0/device"
printf 'DEVTYPE=wlan\nDRIVER=rtl88x2ce\n' > "$WORK/net/wlP1p1s0/device/uevent"
printf 'DEVTYPE=usb_interface\nDRIVER=rtl8xxxu\n' \
    > "$WORK/net/wlx002e2d3074d0/device/uevent"

run() {   # SSID (empty for not associated), then env overrides
    ssid=$1
    shift
    env PATH="$WORK/bin:$PATH" LOG="$WORK/log" SSID="$ssid" \
        SYSNET="$WORK/net" STATE="$WORK/state" \
        MODPROBE=modprobe IW=iw "$@" sh "$HERE/keeper.sh" 2>&1
}

echo "a spare radio that is working"
: > "$WORK/log"
rm -f "$WORK/state"
out=$(run TheMaharaja NOW=1000)
check_silent "says nothing about a radio that is on a network" "$out"
check_silent "and does not touch the driver" "$(cat "$WORK/log")"

echo
echo "a radio that is briefly between access points"
# Two strikes is a roam, not a fault. Reloading here would turn a couple of
# seconds off the air into ten, and then do it again on the next roam.
: > "$WORK/log"
rm -f "$WORK/state"
one=$(run "" NOW=1000)
two=$(run "" NOW=1060)
check "counts the first miss without acting" "1 of 3" "$one"
check "and the second" "2 of 3" "$two"
check_silent "and still has not touched the driver" "$(cat "$WORK/log")"
# ...and coming back clears the count, so an hour of occasional single misses
# never adds up to a reload.
back=$(run TheGreatViking NOW=1120)
check "coming back is worth saying" "is back on TheGreatViking" "$back"
after=$(run "" NOW=1180)
check "and the count starts again from there" "1 of 3" "$after"

echo
echo "a radio that has genuinely stopped"
: > "$WORK/log"
rm -f "$WORK/state"
run "" NOW=1000 > /dev/null
run "" NOW=1060 > /dev/null
out=$(run "" NOW=1120)
check "the third miss is the one that acts" "reloading rtl8xxxu" "$out"
check "and it reloads the driver" "modprobe rtl8xxxu" "$(cat "$WORK/log")"
check "taking it out first, since a reload is what cures this" \
    "modprobe -r rtl8xxxu" "$(cat "$WORK/log")"
# The onboard radio carries the traffic this rover is reached over, and it is a
# different driver. Nothing here may go near it.
check_silent "and never goes near the radio that is working" \
    "$(grep -- '-r rtl8822ce\|rtl8822ce' "$WORK/log" || true)"

echo
echo "and then not again, straight away"
# A reload takes seconds and the association takes seconds more. Counting that
# as three more strikes would have the keeper reloading on top of its own repair
# for as long as the radio stayed unhappy.
: > "$WORK/log"
printf '3 1120\n' > "$WORK/state"
out=$(run "" NOW=1180)
check "a radio still down just after a reload is left alone" "waiting" "$out"
check_silent "and the driver is not touched again" "$(cat "$WORK/log")"
out=$(run "" NOW=1500)
check "but once the cooldown is past it tries again" "reloading" "$out"

echo
echo "a driver with no interface at all"
# What a rover looks like after the module was unloaded, or after a reload where
# nothing claimed the device. There is no link to be patient about here, so this
# does not wait for three strikes.
: > "$WORK/log"
rm -f "$WORK/state"
rm "$WORK/net/wlx002e2d3074d0/device/uevent"
out=$(run "" NOW=2000)
check "loads the driver rather than counting strikes" "loading it" "$out"
check "and does load it" "modprobe rtl8xxxu" "$(cat "$WORK/log")"

echo
echo "and the dry run, which is how this is checked on a live rover"
: > "$WORK/log"
rm -f "$WORK/state"
printf '3 0\n' > "$WORK/state"
printf 'DEVTYPE=usb_interface\nDRIVER=rtl8xxxu\n' \
    > "$WORK/net/wlx002e2d3074d0/device/uevent"
out=$(env PATH="$WORK/bin:$PATH" LOG="$WORK/log" SSID="" SYSNET="$WORK/net" \
    STATE="$WORK/state" MODPROBE=modprobe IW=iw NOW=9000 \
    sh "$HERE/keeper.sh" -n 2>&1)
check "says what it would do" "would reload rtl8xxxu" "$out"
check_silent "and does not do it" "$(cat "$WORK/log")"

echo
echo "$pass passed, $fail failed"
[ "$fail" = 0 ]
