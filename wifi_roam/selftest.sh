#!/bin/sh
# Drive wifi_roam.sh through every decision it can make, with no radio present.
#
#     ./selftest.sh          # anywhere: the workstation, the Pi, a VM
#
# The script reads the link out of three files and acts through nmcli and
# systemctl, and every one of those is replaceable from here, so a fake /proc, a
# fake nmcli and a fake systemctl are enough to put the rover anywhere: healthy,
# fading, associated with no address, off the air entirely, or holding a radio
# that somebody switched off. That matters because the interesting branches are the ones that
# only happen when something has gone wrong, and waiting for a real dongle to go
# wrong is not a test strategy -- the last time one did, it took the rover off
# the network for the afternoon.
#
# The races are covered too, which took a fake that answers differently the second
# time it is asked. Both of the script's second looks -- the one under the lock and
# the one after the scan -- exist because a rover can recover between two reads of
# /proc in the same run, and a static fake cannot express that at all. So the fake
# nmcli rewrites the fake /proc while it is pretending to scan, which is exactly
# what a deliberate join finishing does, and the lock itself is tested by holding
# it from here while the script runs.

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
# spot, trimmed; anything that would change the radio is recorded rather than
# done. It records twice: once per run for the assertions below, and once
# cumulatively for the one assertion that spans every scenario in this file --
# that the radio is never, in any state, switched off.
cat > "$WORK/bin/nmcli" <<'FAKE'
#!/bin/sh
echo "nmcli $*" >> "$NMCLI_LOG"
echo "nmcli $*" >> "$NMCLI_ALL"
case "$*" in
    "radio wifi")    echo "${RADIO:-enabled}" ;;
    "radio wifi on") exit "${RADIO_ON_STATUS:-0}" ;;
    *"dev wifi list"*)
        [ -n "${SCAN_EMPTY:-}" ] && exit 0
        # A rover that came back while the scan was running. The scan is the slow
        # part -- thirty-two seconds, measured -- so it is the window a deliberate
        # join finishes in, and rewriting the fake /proc here is the only way to
        # put a script that reads it twice through that race. The paths come from
        # the script's own environment, so this heals whatever world was set up.
        if [ -n "${HEAL_ON_SCAN:-}" ]; then
            echo up > "$OPERSTATE"
            printf 'Inter-|\n face |\n wlan0: 0000   66.  -41.  -256  0 0 0 0 572 0\n' \
                > "$WIRELESS"
            printf 'Iface\tDestination\nwlan0\t00000000\nwlan0\t0001A8C0\n' \
                > "$ROUTES"
        fi
        printf '%s\n' "$SCAN"
        ;;
    *"con up"*) exit "${CON_UP_STATUS:-0}" ;;
    # What wifi_ctl.sh checks an SSID against before it will join it.
    "-t -f NAME,TYPE con show")
        printf 'TheGreatLord:802-11-wireless\nTheMaharaja:802-11-wireless\n'
        printf 'TheGreatViking:802-11-wireless\nWired connection 1:802-3-ethernet\n'
        ;;
esac
exit 0
FAKE
chmod +x "$WORK/bin/nmcli"

# And a fake systemctl, which the one remaining repair acts through.
cat > "$WORK/bin/systemctl" <<'FAKE'
#!/bin/sh
echo "systemctl $*" >> "$NMCLI_LOG"
echo "systemctl $*" >> "$NMCLI_ALL"
exit "${SYSTEMCTL_STATUS:-0}"
FAKE
chmod +x "$WORK/bin/systemctl"

# The cumulative log, created up front so that the assertion spanning every
# scenario has something to read even if no fake is ever called.
: > "$WORK/nmcli.all"

# The rover as the kernel would describe it. The first argument is the operstate
# word the kernel would show: "up" is associated, "dormant" is an interface that is
# up and looking, "down" is one that is not up at all. Level is the driver's dBm.
world() {   # operstate level route
    echo "$1" > "$WORK/operstate"
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
    env NMCLI_LOG="$WORK/nmcli.log" NMCLI_ALL="$WORK/nmcli.all" \
        SCAN="$SCAN" \
        OPERSTATE="$WORK/operstate" WIRELESS="$WORK/wireless" \
        ROUTES="$WORK/routes" STATE="$WORK/state" LOCK="$WORK/lock" \
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

check_loud() {   # what, actual
    if [ -n "$2" ]; then
        pass=$((pass + 1))
        echo "  ok    $1"
    else
        fail=$((fail + 1))
        echo "  FAIL  $1 -- said nothing at all"
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
world up -41 1
check_silent "says nothing at all" "$(run)"
check_silent "and does not touch nmcli" "$(cat "$WORK/nmcli.log")"

echo
echo "a link that is fading"
rm -f "$WORK/state"
world up -83 1
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
world up -83 1
check "one strike" "1 of 3" "$(run)"
world up -41 1
check_silent "and the count is dropped" "$(run)"
world up -83 1
check "so the next bad check starts over" "1 of 3" "$(run)"

echo
echo "associated, but DHCP never answered"
rm -f "$WORK/state"
world up -41 0
check "is a fault in its own right" "associated with no address" "$(run)"

echo
echo "off the air altogether"
rm -f "$WORK/state"
world dormant -41 0
out=$(run)
check "does not wait for three strikes" "not associated" "$out"
check "and joins the strongest known network" "joining TheMaharaja at 84" "$out"
check "waits a minute before trying again" "last looked around" "$(run)"

echo
echo "off the air, and staying that way"
printf '9 0\n' > "$WORK/state"
world dormant -41 0
check "repairs instead of scanning again" "restarting the supplicant" "$(run)"
check "and does it through a service, which a reboot would undo anyway" \
    "systemctl try-restart wpa_supplicant.service" "$(cat "$WORK/nmcli.log")"
printf '9 0\n' > "$WORK/state"
check "and says so when even that will not go through" \
    "could not restart the supplicant" "$(run SYSTEMCTL_STATUS=1)"

echo
echo "a driver that will not say how strong the link is"
rm -f "$WORK/state"
world up -256 1
# -256 is this dongle's "no value" sentinel, and it turns up in the level column
# for the odd single sample -- three times in two minutes of 1 Hz sampling, with
# good reads either side. Read as a number it is a link 200 dB down, which used to
# put "down to -256 dBm (1 of 3)" in the journal five times in half an hour, and
# three in a row would have carried the rover off a link measuring -42.
check "is not read as a link 200 dB down" "is not a reading" "$(run)"
check_silent "and nothing is scanned over it" \
    "$(grep 'dev wifi list' "$WORK/nmcli.log" || true)"
world up -83 1
check "nor does it leave a strike behind it" "1 of 3" "$(run)"

echo
echo "a link that comes good while the scan is running"
rm -f "$WORK/state"
world dormant -41 0
out=$(run HEAL_ON_SCAN=1)
check "spends no association on a fault that is already over" \
    "came good at -41 dBm" "$out"
check_silent "and brings nothing up" \
    "$(grep 'con up' "$WORK/nmcli.log" || true)"

echo
echo "a deliberate join already in flight"
rm -f "$WORK/state"
world dormant -41 0
if command -v flock > /dev/null 2>&1; then
    # Held by this shell on a descriptor of its own, for exactly the scenarios that
    # need it and not a moment longer. The obvious version -- a backgrounded
    # `flock "$WORK/lock" sleep N` -- is wrong twice over: the lock lives on the
    # descriptor the child inherits, so killing flock does not release it, and a
    # timeout long enough for this Pi, where a process spawn costs a hundred times
    # what it does on a desk, is long enough to leak into the scenarios below. It
    # did: nine seconds took out the next nine assertions in this file.
    exec 8> "$WORK/lock"
    if flock -n 8; then
        check "stands aside rather than choosing a network of its own" \
            "being changed by something else" "$(run)"
        check_silent "and brings nothing up" \
            "$(grep 'con up' "$WORK/nmcli.log" || true)"
        printf '2 0\n' > "$WORK/state"
        run > /dev/null
        check "and charges the link no strike for somebody else's work" \
            "2 0" "$(cat "$WORK/state")"
    else
        fail=$((fail + 1))
        echo "  FAIL  could not take the lock to test it with"
    fi
    exec 8>&-
else
    echo "  --    no flock here, so the lock goes untested"
fi

echo
echo "a radio that answers nothing"
rm -f "$WORK/state"
world dormant -41 0
check "is not the same as an empty neighbourhood" \
    "scan came back empty" "$(run SCAN_EMPTY=1)"

echo
echo "an association that fails"
rm -f "$WORK/state"
world dormant -41 0
check "is reported, not swallowed" "could not bring up" "$(run CON_UP_STATUS=4)"

echo
echo "an interface that is not even up"
rm -f "$WORK/state"
world down -41 0
# The one this file was too kind to catch. `carrier` answers EINVAL rather than 0
# for an interface that is administratively down, an awk that read it printed
# nothing at all, and the defaults then reported a flawless link for a rover that
# had no radio -- silently, three times a minute, until somebody noticed by hand.
check "is a fault, not a healthy link" "not associated" "$(run)"

echo
echo "no interface there at all"
rm -f "$WORK/state"
check "is the same fault and not a reason to keep quiet" \
    "not associated" "$(run OPERSTATE=$WORK/no-such-dongle)"

echo
echo "a radio that is switched off"
rm -f "$WORK/state"
world down -41 0
out=$(run RADIO=disabled)
check "is turned back on" "the radio is switched off; turning it on" "$out"
check "which is the one thing here that touches that switch" \
    "nmcli radio wifi on" "$(cat "$WORK/nmcli.log")"
check_silent "and nothing is scanned while it is off" \
    "$(grep 'dev wifi list' "$WORK/nmcli.log" || true)"
rm -f "$WORK/state"
check "and a switch that will not move is reported" \
    "could not turn the radio on" "$(run RADIO=disabled RADIO_ON_STATUS=1)"

echo
echo "nothing worth moving to"
rm -f "$WORK/state"
world up -83 1
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
world up -83 1
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
world up -41 1
check_silent "is read as no history rather than crashing" "$(run)"

echo
echo "a /proc that cannot be read"
rm -f "$WORK/state"
world up -41 1
# Which awk is installed decides *what* this says -- mawk gives up on a directory
# where gawk skips it and reads on -- so the only assertion that holds on both is
# that it says something. That is the whole of the branch: the version that
# substituted a healthy link for a link it could not read is why a rover with its
# radio switched off looked fine for fifteen minutes, in silence.
check_loud "is never passed off as a healthy link" "$(run WIRELESS=$WORK)"

echo
echo "the daemon's own join, the other thing that moves this link"
rm -f "$WORK/state"
: > "$WORK/nmcli.log"
ctl=$(env NMCLI_LOG="$WORK/nmcli.log" NMCLI_ALL="$WORK/nmcli.all" SCAN="$SCAN" \
    STATE="$WORK/state" LOCK="$WORK/lock" \
    sh "$HERE/wifi_ctl.sh" join TheMaharaja 2>&1)
check "brings up the profile that already holds the passphrase" \
    "con up TheMaharaja" "$(cat "$WORK/nmcli.log")"
check "and bounds the wait, so the daemon is never left guessing" \
    "nmcli -w" "$(cat "$WORK/nmcli.log")"
# The stamp is what gives a hand-picked network the same cooldown as one the roamer
# picked itself. Without it the rover was moved back off it on the next bad reading.
check "clears the roamer's strikes and stamps its clock" \
    "0 " "$(cat "$WORK/state")"
check_silent "and says nothing when it worked" "$ctl"
: > "$WORK/nmcli.log"
check "a network with no passphrase here is still refused" \
    "no configured network called" \
    "$(env NMCLI_LOG="$WORK/nmcli.log" NMCLI_ALL="$WORK/nmcli.all" \
        STATE="$WORK/state" LOCK="$WORK/lock" \
        sh "$HERE/wifi_ctl.sh" join Alister 2>&1)"
check_silent "and brings nothing up on the way to refusing it" \
    "$(grep 'con up' "$WORK/nmcli.log" || true)"

echo
echo "the same helper, on a rover with no NetworkManager"
# The Banana Pi: netplan and wpa_supplicant, no nmcli. WIFI_BACKEND forces
# that path even though the fake nmcli is still on PATH for the roam tests
# above. The fakes record rather than associate, and the scan they answer
# with is the one measured on that dongle -- TheGreatLord plus a neighbour.
cat > "$WORK/bin/wpa_cli" <<'FAKE'
#!/bin/sh
echo "wpa_cli $*" >> "$NMCLI_LOG"
case "$*" in
    *list_networks*)
        printf 'network id / ssid / bssid / flags\n'
        printf '0\tTheMaharaja\tany\t\n'
        printf '1\tTheGreatViking\tany\t\n'
        printf '2\tTheGreatLord\tany\t[CURRENT]\n'
        ;;
    *scan_results*)
        printf 'bssid / frequency / signal level / flags / ssid\n'
        printf 'b0:19:21:b9:4e:fe\t2427\t-46\t[WPA2-PSK-CCMP][ESS]\tTheGreatLord\n'
        printf 'e0:cc:7a:97:21:74\t2437\t-68\t[WPA2-PSK-CCMP][ESS]\tTheGreatViking\n'
        ;;
    *scan*) echo OK ;;
    *select_network*) echo OK ;;
    *enable_network*) echo OK ;;
    *status*)
        printf 'wpa_state=COMPLETED\nssid=TheMaharaja\n'
        ;;
esac
exit 0
FAKE
chmod +x "$WORK/bin/wpa_cli"
cat > "$WORK/bin/iw" <<'FAKE'
#!/bin/sh
echo "iw $*" >> "$NMCLI_LOG"
case "$*" in
    *link*)
        printf 'Connected to b0:19:21:b9:4e:fe (on wlan0)\n\tSSID: TheGreatLord\n'
        ;;
    *scan\ dump*)
        printf 'BSS b0:19:21:b9:4e:fe(on wlan0) -- associated\n'
        printf '\tsignal: -46.00 dBm\n'
        printf '\tSSID: TheGreatLord\n'
        printf '\tRSN:\n'
        ;;
esac
exit 0
FAKE
chmod +x "$WORK/bin/iw"

: > "$WORK/nmcli.log"
listed=$(env WIFI_BACKEND=wpa NMCLI_LOG="$WORK/nmcli.log" \
    SCAN_WAIT=0 SCAN_GIVE_UP=0 \
    sh "$HERE/wifi_ctl.sh" list 2>&1)
check "lists the cached BSS without asking the radio to look" \
    "*:TheGreatLord:100:WPA2" "$listed"
check_silent "and does not start a scan to do it" \
    "$(grep 'wpa_cli .*scan' "$WORK/nmcli.log" || true)"

: > "$WORK/nmcli.log"
scanned=$(env WIFI_BACKEND=wpa NMCLI_LOG="$WORK/nmcli.log" \
    SCAN_WAIT=0 SCAN_GIVE_UP=0 \
    sh "$HERE/wifi_ctl.sh" scan 2>&1)
check "a scan names the neighbour as well" \
    "TheGreatViking" "$scanned"
check "at the 0-100 figure the panel already ranks by" \
    "TheGreatViking:64:" "$scanned"

named=$(env WIFI_BACKEND=wpa NMCLI_LOG="$WORK/nmcli.log" \
    sh "$HERE/wifi_ctl.sh" profiles 2>&1)
check "profiles are the ones wpa_supplicant already holds" \
    "TheMaharaja" "$named"

: > "$WORK/nmcli.log"
ctl=$(env WIFI_BACKEND=wpa NMCLI_LOG="$WORK/nmcli.log" \
    STATE="$WORK/state" LOCK="$WORK/lock" \
    sh "$HERE/wifi_ctl.sh" join TheMaharaja 2>&1)
check "a join selects the network that already holds the passphrase" \
    "select_network 0" "$(cat "$WORK/nmcli.log")"
check_silent "and says nothing when it worked" "$ctl"
check "a network with no passphrase here is still refused" \
    "no configured network called" \
    "$(env WIFI_BACKEND=wpa NMCLI_LOG="$WORK/nmcli.log" \
        STATE="$WORK/state" LOCK="$WORK/lock" \
        sh "$HERE/wifi_ctl.sh" join Alister 2>&1)"

echo
echo "across every scenario above"
check_silent "the radio is never switched off, in any of them" \
    "$(grep -F 'radio wifi off' "$WORK/nmcli.all" 2>/dev/null || true)"

echo
echo "$pass passed, $fail failed"
[ "$fail" -eq 0 ]
