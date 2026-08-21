#!/bin/sh
# Drive wifi_roam.sh through every decision it can make, with no radio present.
#
#     ./selftest.sh          # anywhere: the workstation, the Pi, a VM
#
# The script reads the link out of three files and acts through nmcli, and all
# four of those are overridable, so a fake /proc and a fake nmcli are enough to
# put the rover anywhere: healthy, fading, associated with no address, or off the
# air entirely. That matters because the interesting branches are the ones that
# only happen when something has gone wrong, and waiting for a real dongle to go
# wrong is not a test strategy -- the last time one did, it took the rover off
# the network for the afternoon.
#
# One path is deliberately not covered: the second look at the signal just before
# switching, which catches a link that recovered while its strikes were counted.
# It reads the same files twice in one run, so a fake that answers differently the
# second time would be testing the fake. That one was verified on the hardware.

set -u

HERE=$(cd "$(dirname "$0")" && pwd)
WORK=${TMPDIR:-/tmp}/wifi-roam-selftest.$$
mkdir -p "$WORK/bin"
trap 'rm -rf "$WORK"' EXIT INT TERM

PATH="$WORK/bin:$PATH"
export PATH

pass=0
fail=0

# --- the world the script gets to see -------------------------------------

# A fake nmcli. The scan it answers with is the real one from the rover's usual
# spot, trimmed; anything that changes the radio is recorded rather than done.
cat > "$WORK/bin/nmcli" <<'FAKE'
#!/bin/sh
echo "nmcli $*" >> "$NMCLI_LOG"
case "$*" in
    *"dev wifi list"*)
        [ -n "${SCAN_EMPTY:-}" ] && exit 0
        printf '%s\n' "$SCAN"
        ;;
    *"con up"*) exit "${CON_UP_STATUS:-0}" ;;
esac
exit 0
FAKE
chmod +x "$WORK/bin/nmcli"

# The rover as the kernel would describe it. Level is the driver's dBm.
world() {   # carrier level route
    echo "$1" > "$WORK/carrier"
    printf 'Inter-|\n face |\n wlan0: 0000   66.  %s.  -256  0 0 0 0 572 0\n' \
        "$2" > "$WORK/wireless"
    if [ "$3" = 1 ]; then
        printf 'Iface\tDestination\nwlan0\t00000000\nwlan0\t0001A8C0\n' \
            > "$WORK/routes"
    else
        printf 'Iface\tDestination\neth0\t00000000\n' > "$WORK/routes"
    fi
}

_run() {   # $1 = flags for the script, then any VAR=value overrides
    flags=$1
    shift
    : > "$WORK/nmcli.log"
    env NMCLI_LOG="$WORK/nmcli.log" \
        SCAN="$SCAN" \
        CARRIER="$WORK/carrier" WIRELESS="$WORK/wireless" \
        ROUTES="$WORK/routes" STATE="$WORK/state" \
        "$@" sh "$HERE/wifi_roam.sh" $flags 2>&1
}

run() { _run "" "$@"; }
dry() { _run "-n" "$@"; }

# TheGreatLord is the one we are on, and TheMaharaja is the loudest alternative.
SCAN='*:TheGreatLord:52
 :TheMaharaja:84
 :TheGreatViking:66
 :Alister:90'

# --- the assertions --------------------------------------------------------

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

echo "a link that is fine"
rm -f "$WORK/state"
world 1 -41 1
check_silent "says nothing at all" "$(run)"
check_silent "and does not touch nmcli" "$(cat "$WORK/nmcli.log")"

echo
echo "a link that is fading"
rm -f "$WORK/state"
world 1 -83 1
check "counts the first strike" "1 of 3" "$(run)"
check "counts the second" "2 of 3" "$(run)"
out=$(run)
check "switches on the third" "joining TheMaharaja at 84" "$out"
check "to the loudest of ours, not the loudest AP" \
    "con up TheMaharaja" "$(cat "$WORK/nmcli.log")"
check "and only scans once it has decided to look" \
    "dev wifi list --rescan yes" "$(cat "$WORK/nmcli.log")"
run > /dev/null
run > /dev/null
check "then sits out the cooldown before moving again" \
    "last looked around" "$(run)"

echo
echo "a link that recovers before the strikes run out"
rm -f "$WORK/state"
world 1 -83 1
check "one strike" "1 of 3" "$(run)"
world 1 -41 1
check_silent "and the count is dropped" "$(run)"
world 1 -83 1
check "so the next bad check starts over" "1 of 3" "$(run)"

echo
echo "associated, but DHCP never answered"
rm -f "$WORK/state"
world 1 -41 0
check "is a fault in its own right" "associated with no address" "$(run)"

echo
echo "off the air altogether"
rm -f "$WORK/state"
world 0 -41 0
out=$(run)
check "does not wait for three strikes" "not associated" "$out"
check "and joins the strongest known network" "joining TheMaharaja at 84" "$out"
check "waits a minute before trying again" "last looked around" "$(run)"

echo
echo "off the air, and staying that way"
printf '9 0\n' > "$WORK/state"
world 0 -41 0
check "cycles the radio rather than scanning again" "cycling the radio" "$(run)"
check "which it does through nmcli" "radio wifi off" "$(cat "$WORK/nmcli.log")"

echo
echo "a radio that answers nothing"
rm -f "$WORK/state"
world 0 -41 0
check "is not the same as an empty neighbourhood" \
    "scan came back empty" "$(run SCAN_EMPTY=1)"

echo
echo "an association that fails"
rm -f "$WORK/state"
world 0 -41 0
check "is reported, not swallowed" "could not bring up" "$(run CON_UP_STATUS=4)"

echo
echo "nothing worth moving to"
rm -f "$WORK/state"
world 1 -83 1
SCAN='*:TheGreatLord:20
 :TheMaharaja:12'
run > /dev/null; run > /dev/null
check "leaves a bad link alone" "nothing better is audible" "$(run)"
SCAN='*:TheGreatLord:52
 :TheMaharaja:84
 :TheGreatViking:66'

echo
echo "a dry run"
rm -f "$WORK/state"
world 1 -83 1
check "reaches a decision" "joining TheMaharaja" "$(dry STRIKES=1)"
check_silent "without bringing anything up" "$(grep 'con up' "$WORK/nmcli.log" || true)"
if [ -e "$WORK/state" ]; then
    fail=$((fail + 1))
    echo "  FAIL  and without leaving state behind"
else
    pass=$((pass + 1))
    echo "  ok    and without leaving state behind"
fi

echo
echo "a state file left half written"
printf 'nonsense\n' > "$WORK/state"
world 1 -41 1
check_silent "is read as no history rather than crashing" "$(run)"

echo
echo "$pass passed, $fail failed"
[ "$fail" -eq 0 ]
