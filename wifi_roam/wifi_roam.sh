#!/bin/sh
# Keep the rover's wlan0 on an access point that works, and prefer the strongest
# one when it has to choose.
#
# NetworkManager already does half of this. Every profile has autoconnect on, so
# when the link drops NM rescans and rejoins whichever known network it can still
# hear. What it will not do is choose by signal: its ordering is profile priority
# first and last-used time second, so a rover that has driven into the far end of
# the house can sit on the AP it happened to use yesterday while a much closer one
# goes unused. Nor will NM ever leave an association that is merely bad -- as long
# as beacons keep arriving it stays, however few packets survive.
#
# The three house APs (TheGreatLord, TheMaharaja, TheGreatViking) are three
# separate routers bridged onto one 192.168.1.0/24 LAN, and that single fact is
# what makes choosing between them on signal alone safe: whichever one the rover
# lands on, it keeps a 192.168.1.x address, the workstation can still reach the
# daemon and the daemon can still reach the face detector on MEDIA. Adding an AP
# that routed its own subnet would break that assumption rather than bend it.
#
#     wifi_roam.sh            # one check; wifi-roam.timer runs this every 20 s
#     wifi_roam.sh -n         # say what it would do, change nothing
#
# Root only: scanning and activating a connection are both polkit-guarded, and
# polkit grants those to an active local session, which a timer is not and an ssh
# session is not either.

set -u

# The networks worth being on, and nothing else. A stranger's SSID is never a
# target however loud it is -- and see the note above before adding one.
NETS=${NETS:-"TheGreatLord TheMaharaja TheGreatViking"}

# The signal at which the current association counts as failing, in dBm, from the
# driver rather than from a scan. -78 sits clear of two kinds of wobble measured on
# this dongle: a healthy link alternates between two readings about 9 dB apart as
# the driver switches between beacon and data measurements, and the occasional
# read comes back tens of dB low for no visible reason. This rover sits at -35 to
# -44 dBm in the lab.
LOW=${LOW:--78}

# How strong a candidate has to look before it is worth leaving for. Not a
# comparison against the current link: NM's 0-100 scan figure is far noisier than
# the driver's dBm -- consecutive scans reported the same association anywhere from
# 74 to 88, and one AP swung from 50 to 97 and back inside a minute -- so it is
# only good enough to answer "is anything decent over there", never "is that one
# better than this one by twenty".
FLOOR=${FLOOR:-45}

# Consecutive failing checks before moving. Switching costs a DHCP round and every
# TCP connection the daemon is holding, so it should take a bad link rather than
# one bad moment: three checks at 20 s is a minute of genuinely poor signal, and
# the level is read once more at the end to catch a link that recovered while they
# were being counted. A link that is not associated at all skips all of this and
# is dealt with on the first check -- there is nothing left to protect -- but see
# RETRY below for the limit that replaces it.
STRIKES=${STRIKES:-3}

# And after a switch, how long to leave the new one alone, so that a rover parked
# in a bad spot cannot cycle the whole list every minute.
COOLDOWN=${COOLDOWN:-180}

# When there is no association at all the strike count does not apply -- there is
# nothing left to protect -- but scanning is the one operation that can wedge this
# dongle, and hammering it three times a minute while it is already down is how a
# rover that would have recovered on its own stays offline instead. So an attempt
# is allowed once a minute, not once a tick.
RETRY=${RETRY:-60}

# And after this many consecutive checks with nothing working at all, stop trying
# to choose and try to repair: take the radio down and up before scanning again.
WEDGED=${WEDGED:-9}

IFACE=${IFACE:-wlan0}

# Strike count, and the last time this spent a scan or an association attempt --
# the two operations that cost the link something. /run is tmpfs, so this forgets
# across a reboot, which is right: a fresh boot has no history worth trusting.
STATE=${STATE:-/run/wifi-roam.state}

# The three files the healthy path is read out of. Overridable so that
# selftest.sh can hand this a rover that is not here: without that, every branch
# below could only be exercised on a Pi with a real radio in a real house.
CARRIER=${CARRIER:-/sys/class/net/$IFACE/carrier}
WIRELESS=${WIRELESS:-/proc/net/wireless}
ROUTES=${ROUTES:-/proc/net/route}

DRY=""
[ "${1:-}" = "-n" ] && DRY=yes

say() { echo "$*"; }

# Every write to the state file goes through here, so that -n really does change
# nothing -- including the strike count a later real run would read back.
remember() {
    [ -n "$DRY" ] || printf '%s %s\n' "$1" "$2" > "$STATE"
}

# "carrier level heard route" -- is the radio associated, how strong is the link
# in dBm, did the driver report a signal at all, and does the interface have a
# route.
#
# The healthy path must not cost a single nmcli call, and barely a process either.
# One nmcli takes 1.8 s of wall time and half a second of CPU on this Pi against
# 64 ms for all four of these answers, and this runs three times a minute beside
# SLAM on the one armv6 core. That is not tuning; it is what makes a 20-second
# timer affordable here.
#
# A route rather than an address, because it comes free in the same /proc pass: a
# wlan0 entry in /proc/net/route only exists once the interface has an address, so
# its absence is the associated-but-DHCP-failed case.
probe() {
    if [ -r "$CARRIER" ]; then
        awk -v i="$IFACE" -v c="$CARRIER" -v w="$WIRELESS" -v r="$ROUTES" '
            FILENAME == c                  { carrier = $1 + 0 }
            FILENAME == w && $1 == i ":"   { level = $4 + 0; heard = 1 }
            FILENAME == r && $1 == i       { route = 1 }
            END { printf "%d %d %d %d\n", carrier, level, heard, route }
        ' "$CARRIER" "$WIRELESS" "$ROUTES" 2>/dev/null
    else
        # No such interface: the dongle is out, or the kernel renamed it.
        echo "0 0 0 0"
    fi
}

# The four trailing values are defaults for an awk that printed nothing at all,
# and they say "healthy" on purpose: a watchdog that cannot read the system should
# stay out of the way rather than thrash a link it knows nothing about.
set -- $(probe) 1 0 1 1
carrier=$1 level=$2 heard=$3 route=$4

strikes=0
last=0
[ -r "$STATE" ] && read -r strikes last < "$STATE"
# A file half written when the power went out would otherwise take this down on
# its first arithmetic, three times a minute, with nobody watching.
case $strikes in '' | *[!0-9]*) strikes=0 ;; esac
case $last in '' | *[!0-9]*) last=0 ;; esac

if [ "$carrier" = 1 ] && [ "$route" = 1 ] && [ "$heard" = 1 ] &&
        [ "$level" -gt "$LOW" ]; then
    # Nothing to do, and nothing to say: the journal would be nothing else.
    [ "$strikes" -ne 0 ] && remember 0 "$last"
    exit 0
fi

# Something is wrong. Which of the four things it is decides how patient to be.
strikes=$((strikes + 1))
if [ "$carrier" != 1 ]; then
    why="not associated"
elif [ "$heard" != 1 ]; then
    why="not reporting a signal"
elif [ "$route" != 1 ]; then
    why="associated with no address"
else
    why="down to ${level} dBm"
fi

now=$(date +%s)
since=$((now - last))
remember "$strikes" "$last"

if [ "$carrier" = 1 ]; then
    if [ "$strikes" -lt "$STRIKES" ]; then
        say "$IFACE $why ($strikes of $STRIKES)"
        exit 0
    fi
    if [ "$since" -lt "$COOLDOWN" ]; then
        say "$IFACE $why, but it last looked around ${since}s ago"
        exit 0
    fi
elif [ "$since" -lt "$RETRY" ]; then
    say "$IFACE $why, and it last looked around ${since}s ago"
    exit 0
elif [ "$strikes" -ge "$WEDGED" ]; then
    # Minutes of nothing at all. Taking the radio down and up costs nothing that
    # is working, and it clears a supplicant that has lost track of its interface
    # -- the cheap half of what a power cycle does. A dongle that has genuinely
    # fallen off the USB bus needs the other half, and a person.
    say "$IFACE $why after $strikes checks; cycling the radio"
    remember 0 "$now"
    if [ -z "$DRY" ]; then
        nmcli radio wifi off
        sleep 3
        nmcli radio wifi on
    fi
    exit 0
fi

# One last look before spending a scan and an association on it. Three bad reads
# in a row is already unlikely to be noise, but this costs 64 ms against a
# reconnect the rover would feel, and it is the one check that catches a link that
# came back while the strikes were being counted.
if [ "$carrier" = 1 ]; then
    set -- $(probe) 1 0 1 1
    if [ "$1" = 1 ] && [ "$3" = 1 ] && [ "$4" = 1 ] && [ "$2" -gt "$LOW" ]; then
        say "$IFACE came back to $2 dBm on the way to switching; staying"
        remember 0 "$last"
        exit 0
    fi
fi

# One scan, and it answers two questions at once: the IN-USE row names the AP we
# are on, so a second nmcli call to ask that is unnecessary.
LIST=$(nmcli -t -f IN-USE,SSID,SIGNAL dev wifi list --rescan yes 2>/dev/null)
cur=$(printf '%s\n' "$LIST" | awk -F: '$1 == "*" { print $2; exit }')

# Not "no known network in range" but "the radio told us nothing at all", which in
# a house this full of access points means the dongle, not the neighbourhood.
if [ -z "$LIST" ]; then
    say "$IFACE $why, and the scan came back empty"
    remember "$strikes" "$now"
    exit 0
fi

# "-" rather than an empty string: the winner is read back as a word below, and an
# unheard-of network would otherwise vanish a field.
best="-"
best_sig=-1
for s in $NETS; do
    [ -n "$cur" ] && [ "$s" = "$cur" ] && continue
    # The strongest sighting, not the first: a router with two radios answers on
    # several BSSIDs and only its best one is what we would associate with.
    sig=$(printf '%s\n' "$LIST" |
        awk -F: -v s="$s" '$2 == s && $3 + 0 > m { m = $3 + 0 } END { print m + 0 }')
    [ "$sig" -gt "$best_sig" ] && { best=$s; best_sig=$sig; }
done

if [ "$best_sig" -lt "$FLOOR" ]; then
    say "$IFACE $why, and nothing better is audible (best $best at $best_sig)"
    remember "$strikes" "$now"
    exit 0
fi

say "$IFACE $why; joining $best at $best_sig"
if [ -n "$DRY" ]; then
    say "(dry run, staying put)"
    exit 0
fi

if nmcli con up "$best" ifname "$IFACE" > /dev/null 2>&1; then
    remember 0 "$now"
else
    # The stamp goes down even though nothing came up. It is what stops a failing
    # attempt being repeated every twenty seconds, and the strike count is left
    # standing so that enough failures still escalate to cycling the radio.
    remember "$strikes" "$now"
    say "could not bring up $best"
fi
