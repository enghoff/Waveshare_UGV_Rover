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

# The range within which the driver's dBm is a measurement at all. -256 is the
# "no value" sentinel this dongle keeps permanently in the noise column, and it
# turns up in the level column too: sampled at 1 Hz for two minutes, the link held
# between -33 and -50 dBm with three isolated -256 reads scattered through it, each
# one a single sample with good ones either side. Read at face value that is a link
# 200 dB down, which counts a strike -- five of them in half an hour, and "down to
# -256 dBm" in the journal each time, drowning the faults that were real. A number
# outside this range is therefore not a weak signal but no signal reported, which
# is a state this script answers separately and far more calmly.
SANE_DBM=${SANE_DBM:--110}

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

# And the lock that says somebody else is already moving this link. `wifi_ctl.sh`
# holds it for the length of a deliberate join, because a join takes the interface
# down for the ten seconds it spends associating and there is nothing in /proc that
# tells that apart from a rover which has lost the network. See the second look
# below for what that cost.
#
# `flock` rather than a lock file of this script's own making: a kernel lock dies
# with the process holding it, where a script killed between creating a file and
# removing it would lock the rover out of its own network keeper for good. Where
# `flock` is not installed -- the desks this self-tests on, never a rover -- the
# checks below simply go unserialised.
#
# `-n` takes it too, and is the one thing a dry run does touch. Reporting a
# decision it would never actually have taken would be the worse dishonesty of
# the two.
LOCK=${LOCK:-/run/wifi-roam.lock}

# The three files the healthy path is read out of. Overridable so that
# selftest.sh can hand this a rover that is not here: without that, every branch
# below could only be exercised on a Pi with a real radio in a real house.
#
# `operstate` and not `carrier`, which is the more direct question and is a trap:
# reading `carrier` on an interface that is not administratively up fails with
# EINVAL rather than answering 0. That killed the awk below, which left probe()
# printing nothing, which handed this script the healthy defaults -- so a rover
# whose radio was switched off read as a perfect link, three times a minute, with
# nothing in the journal. `operstate` is a word rather than a flag and is readable
# in every state: "up" once associated, "dormant" while the supplicant is looking,
# "down" when the interface is not up at all.
OPERSTATE=${OPERSTATE:-/sys/class/net/$IFACE/operstate}
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

# "assoc level heard route" -- is the radio associated, how strong is the link in
# dBm, is that number one a radio could actually have reported, and does the
# interface have a route.
#
# The healthy path must not cost a single nmcli call, and barely a process either.
# One nmcli takes 1.8 s of wall time and half a second of CPU on this Pi against
# 64 ms for all four of these answers, and this runs three times a minute beside
# SLAM on the one armv6 core. That is not tuning; it is what makes a 20-second
# timer affordable here.
#
# The state word is read by the shell rather than handed to the awk below, which
# costs nothing -- a builtin, no process, one file fewer for awk to open -- and
# buys the difference between "not associated" and "cannot tell", deterministically:
# a `read` from a path that is missing, is a directory or errors simply fails,
# where the same file inside awk is a fatal error to mawk and a skipped argument
# with a warning to gawk, and this self-tests on both.
#
# A route rather than an address, because it comes free in the same /proc pass: a
# wlan0 entry in /proc/net/route only exists once the interface has an address, so
# its absence is the associated-but-DHCP-failed case.
probe() {
    if ! read -r state 2>/dev/null < "$OPERSTATE"; then
        # No such interface: the dongle is out, or the kernel renamed it. Not a
        # reason to keep quiet -- it is the plainest fault there is.
        echo "0 0 0 0"
        return
    fi
    assoc=0
    [ "$state" = up ] && assoc=1
    awk -v i="$IFACE" -v w="$WIRELESS" -v r="$ROUTES" -v a="$assoc" \
            -v s="$SANE_DBM" '
        FILENAME == w && $1 == i ":"   { level = $4 + 0
                                         heard = (level < 0 && level >= s + 0) }
        FILENAME == r && $1 == i       { route = 1 }
        END { printf "%d %d %d %d\n", a, level, heard, route }
    ' "$WIRELESS" "$ROUTES" 2>/dev/null
}

# A dash where the awk printed nothing at all, which now takes /proc itself being
# unreadable rather than merely a radio that is off. Reading nothing still means
# doing nothing -- a watchdog that cannot read the system should stay out of the
# way rather than thrash a link it knows nothing about -- but it says so now
# instead of substituting a healthy link, because the silent version of this is
# what let a rover with its radio switched off look fine for fifteen minutes.
set -- $(probe) - - - -
if [ "$1" = "-" ]; then
    say "cannot read the state of $IFACE; leaving the link alone"
    exit 0
fi
assoc=$1 level=$2 heard=$3 route=$4

strikes=0
last=0
[ -r "$STATE" ] && read -r strikes last < "$STATE"
# A file half written when the power went out would otherwise take this down on
# its first arithmetic, three times a minute, with nobody watching.
case $strikes in '' | *[!0-9]*) strikes=0 ;; esac
case $last in '' | *[!0-9]*) last=0 ;; esac

if [ "$assoc" = 1 ] && [ "$route" = 1 ] && [ "$heard" = 1 ] &&
        [ "$level" -gt "$LOW" ]; then
    # Nothing to do, and nothing to say: the journal would be nothing else.
    [ "$strikes" -ne 0 ] && remember 0 "$last"
    exit 0
fi

# Something is wrong -- or something else is deliberately putting it right, which
# from /proc looks exactly the same. So before charging this to the link, take the
# lock, and then look again.
#
# The lock is taken here rather than at the top of the script because the healthy
# path -- almost every tick -- must not pay for it, and it is released by this
# shell exiting. Failing to get it is not a fault to be charged to the link
# either: somebody is already moving it, and the next tick will see wherever they
# moved it to.
if command -v flock > /dev/null 2>&1; then
    exec 9> "$LOCK"
    if ! flock -n 9; then
        say "$IFACE is being changed by something else; leaving this tick alone"
        exit 0
    fi
fi

# The second look, and the reason this script needed one. A deliberate join to
# another access point had been in flight for seven seconds when a tick read
# `operstate`, found no association, called that a fault and went off to choose a
# network of its own -- so the network somebody had picked lasted 43 seconds before
# the rover was carried back to the one it came from. Under the lock that
# particular race is already over; this catches the rest of them for 64 ms -- the
# tail of a join that began before the lock was taken, and this dongle's isolated
# bad samples.
set -- $(probe) - - - -
if [ "$1" != "-" ]; then
    assoc=$1 level=$2 heard=$3 route=$4
    if [ "$assoc" = 1 ] && [ "$route" = 1 ] && [ "$heard" = 1 ] &&
            [ "$level" -gt "$LOW" ]; then
        say "$IFACE is fine on a second look, at $level dBm; leaving it alone"
        [ "$strikes" -ne 0 ] && remember 0 "$last"
        exit 0
    fi
fi

if [ "$assoc" = 1 ] && [ "$route" = 1 ] && [ "$heard" != 1 ]; then
    # Associated, addressed, and the driver will not put a number to the signal.
    # There is nothing here worth acting on: the two faults that actually take a
    # rover off the network -- losing the association and losing the address -- are
    # both answered above, and a link that cannot be graded is not a link to go
    # moving on the strength of the grade. Charging no strike for it is the point,
    # because a run of these would otherwise carry the rover off a perfectly good
    # association. Said out loud rather than passed off as healthy, since the
    # silent version of that is the bug probe() was rewritten for.
    say "$IFACE is associated with an address, and ${level} is not a reading"
    exit 0
fi

# So it is a real fault, and which of the three it is decides how patient to be.
strikes=$((strikes + 1))
if [ "$assoc" != 1 ]; then
    why="not associated"
elif [ "$route" != 1 ]; then
    why="associated with no address"
else
    why="down to ${level} dBm"
fi

now=$(date +%s)
since=$((now - last))
remember "$strikes" "$last"

if [ "$assoc" = 1 ]; then
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
    # Minutes of nothing at all. Restarting the supplicant clears one that has
    # lost track of its interface -- the cheap half of what a power cycle does --
    # and it costs nothing that is working, because nothing is. A dongle that has
    # genuinely fallen off the USB bus needs the other half, and a person.
    #
    # This used to take the radio down and up instead, and that is the one mistake
    # in here worth naming: `nmcli radio wifi off` writes NetworkManager's own
    # state file, which NM restores at boot. A power cut in the three seconds
    # before the `on` -- or an `on` that simply failed, which nothing checked --
    # therefore left the rover soft-blocked on that boot and on every boot after,
    # with no radio to reach it by and no journal surviving the reboot to say why.
    # Restarting a service leaves behind nothing that a boot does not undo.
    say "$IFACE $why after $strikes checks; restarting the supplicant"
    remember 0 "$now"
    if [ -z "$DRY" ] && ! systemctl try-restart wpa_supplicant.service; then
        say "could not restart the supplicant"
    fi
    exit 0
fi

# One thing left before spending a scan and an association, and it applies only to
# a link with no association at all. The check that used to sit beside it -- one
# more look at a link that is merely bad -- now happens further up, on every
# faulting tick rather than only on the third, and again below once the scan has
# answered.
if [ "$assoc" != 1 ] && [ "$(nmcli radio wifi 2>/dev/null)" = disabled ]; then
    # A link with no association at all may have no radio to associate with, and
    # nothing further down can help while that switch is off: a scan comes back
    # empty and there is no radio for `con up` to bring a profile up on. Worse, NM
    # keeps the switch in a state file and restores it at boot, so an
    # `nmcli radio wifi off` from any source -- an older copy of this script, a
    # hand at a console -- outlives every reboot until something turns it back on.
    #
    # This is that something, and turning it on is the only thing in this script
    # that touches that switch at all. Nothing here ever turns it off.
    #
    # Asked here rather than at the top because it costs an nmcli call, so only a
    # rover that is already off the air pays for it, at most once a minute.
    say "$IFACE $why, and the radio is switched off; turning it on"
    remember 0 "$now"
    if [ -z "$DRY" ] && ! nmcli radio wifi on; then
        say "could not turn the radio on"
    fi
    exit 0
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

# The last look, and the one that costs least for what it saves. The scan is the
# slow part of this script -- thirty-two seconds, measured, on a Pi that was also
# running SLAM -- so by the time it answers, the fault it was sent to answer may be
# half a minute old and long since over. Spending an association on it would take
# down a link that is working, which is the entire cost this script exists to
# avoid.
set -- $(probe) - - - -
if [ "$1" = 1 ] && [ "$3" = 1 ] && [ "$4" = 1 ] && [ "$2" -gt "$LOW" ]; then
    say "$IFACE came good at $2 dBm while the scan ran; staying on ${cur:-it}"
    remember 0 "$now"
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
