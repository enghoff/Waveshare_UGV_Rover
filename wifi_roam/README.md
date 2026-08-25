# Keeping the rover on the network

A rover that drives out of one router's range and loses the network does not come
back on its own in any useful sense. Neither stack this has run on chooses well:
NetworkManager reconnects eventually to whichever access point it happens to pick
— the one it used most recently, not the one now shouting through the nearest
wall — and `wpa_supplicant` picks by signal but only at the moment it associates,
and the three house networks are three *different* SSIDs, so nothing roams
between them at all. Worse, neither will leave an association while beacons keep
arriving, so the rover can sit at the edge of an AP's range dropping most of its
packets while a much stronger AP goes unused three metres away.

This directory is the fix: the house networks, a small script on a 20-second
timer that watches the link and moves it when it has to, and — on the board
that is the rover today — a dual-radio manager that has replaced that timer.
**Skip to [Two radios](#two-radios-which-is-what-this-board-does-now) for
what this board actually runs.**

**On this board the roam timer is installed and switched off**, and since
2026-08-25 that is not caution but arithmetic: both radios are up and
`wifi_dual.py` owns them. Do not enable the timer while the manager is
running, and do not run `install.sh` without `ROAM=off` — that installer
enables the timer.

## Two radios, which is what this board does now

**On the Banana Pi none of the above is what is running.** Since 2026-08-25 the
rover uses both of its radios at once — the onboard Broadcom and the USB dongle
that was retired in August — and [wifi_dual.py](wifi_dual.py) decides which of
them carries the traffic. `wifi-roam.timer` is disabled while that is on, and
the installer disables it, because the two have opposite models of the problem
and letting both move the link is a fight over the rover's only way in.

The reason is one recording. `netwatch/` was running on 2026-08-24 when the
rover lost the network twice in fourteen minutes, and what it wrote down is not
a radio failing:

```
00:05:15 → 00:07:04   associated with TheGreatViking at -70 to -76 dBm,
                      addressed, and not one ping answered.   120 s
00:09:45 → 00:11:34   the same again, ending in
                      DISCONNECTED reason=4 → SCANNING        120 s
00:11:35              re-associated with TheGreatLord, which had been
                      sitting at -40 dBm the whole time
```

Four minutes of a rover that was associated, addressed and carrying nothing,
with a much better access point audible throughout. That is the document's *"one
AP has RF signal but no LAN connectivity"* case, and no amount of choosing
better with one radio helps: the cost is not the choice, it is that a single
radio has to give up the link it has before it can test the one it wants.

With two radios the second one is already associated and already tested, so the
move is an address changing interface and a gratuitous ARP — tens of
milliseconds, and TCP connections live through it. Replayed against that
recording, the four minutes of carrying nothing become **zero seconds and three
failovers**. See [wifi_world.py](wifi_world.py) for how that replay is done and
why it is allowed to count.

Four things about the design differ from
[docs/bpi_dual_wifi_redundancy.md](../docs/bpi_dual_wifi_redundancy.md), and it
is worth knowing which:

- **There is no NetworkManager here**, so nothing is pinned by BSSID through
  `nmcli`. A radio is held on a network by enabling exactly that one in its own
  `wpa_supplicant` and disabling the rest.
- **BSSID pinning turns out not to be needed at all.** The document assumes one
  SSID across three access points; this house has six SSIDs, being three routers
  with a 2.4 and a 5 GHz name each. Keeping the two radios apart is then a
  matter of choosing different names. The mistake worth guarding against is the
  opposite one — `TheGreatLord` and `TheGreatLord 5G` are *one box*, and a radio
  on each would look like redundancy and provide none.
- **The two radios are not interchangeable**, so they are not treated as though
  they were. The onboard BCM4345/6 is dual band at 31 dBm; the dongle is
  2.4 GHz, 1T1R, 0 dBm, and is the one that failed its own PHY, RF and LLT
  initialisation and fell off the USB bus in August. That maps onto the
  document's own advice — 5 GHz for bandwidth, 2.4 GHz for reach — so the
  onboard radio gets first pick of routers and wins a tie by 3 dB, and the
  unreliable adapter is now the spare, where its failing costs nothing.
- **Scanning stops costing anything**, which is the largest gain here and was
  free. A scan takes a radio off channel for seconds; it is why the roamer below
  only ever scans when the link is already broken, and why a burst of forced
  scans once took this rover off the network for an afternoon. The standby does
  all the scanning now and the active radio is never interrupted — so the
  console's list of networks is always fresh and nobody has to press anything.

### One address that does not move when the radio does

The rover answers on **192.168.1.80**, and that address is moved between the two
interfaces rather than belonging to either. It is what makes a failover survivable
by an open SSH session, the console's websocket and the rover's own conversation
with Alibaba, none of which would survive the source address changing underneath
them. `ssh 192.168.1.80` is the way to reach this board now; `.139` and `.100`
are the two interfaces' own DHCP leases and each still works, through a routing
rule per interface so a packet is answered out of the radio it arrived on.

It is ARP-probed before every claim, not once at startup, because the thing to
check for is somebody being handed it by DHCP in between two failovers. If
anything answers, the manager says so and runs **without** a service address,
which degrades exactly to the document's Option 1 — two DHCP addresses and a
route metric — and costs only the connections that were already open.

Two kernel settings make this mean anything, and without them the design
inverts rather than degrades: `arp_ignore=1` and `arp_announce=2`, installed as
[99-dual-wifi.conf](99-dual-wifi.conf). By default Linux answers an ARP request
for any local address on any interface, so the standby would answer for the
rover's address, the access point it is on would learn it, and the upstream
bridge would deliver the rover's traffic to the radio that is not carrying it.

### What it decides, and on what evidence

Two decisions, on different clocks and different evidence — the same split
`wifi_roam.sh` arrived at below, for the same reason:

| | how often | from what | costs |
|---|---|---|---|
| which radio carries traffic | every second | measured: signal, round trip, loss on links that are both already up | an address and three ARP frames |
| where the standby sits | every 30 s | a scan, which can only report a signal | an association and a DHCP round |

Everything is scored in **dB-equivalents**, so the document's "prefer an access
point that is 8–10 dB better" is expressible directly: a link's score starts at
its signal in dBm and has penalties subtracted for latency and loss. That keeps
a score readable — −78 is a link as good as a clean −78 dBm one, however it got
there — and it makes the document's own worked example come out the way it says
it should, which `test_wifi_dual.py` asserts so that a change of weights cannot
quietly reverse it.

A link is called **unusable** on three facts and deliberately not on five:
associated, addressed, and the gateway answering. Neither the signal nor the
SSID is one of them, and both of those are corrections the recording forced
rather than choices — see the docstring on `Radio.usable`, which has the numbers.

### The thing that could strand a wifi-only rover, and the four nets under it

Holding a radio on one network means `select_network`, and `select_network`
works by **disabling every other configured network**. A manager that died
without undoing that would leave each radio holding one access point it may not
be able to reach and forbidden from trying the others, on a board with no
ethernet socket and no console. So there are four separate undos:

1. a signal handler, for `systemctl stop` and for Ctrl-C;
2. an exception handler around the main loop, for a crash;
3. the **dead-man** — two minutes with neither radio reaching the gateway and
   the manager frees both radios, drops the service address and stands back
   until something answers, which is strictly worse behaviour and strictly
   better than a manager staying certain about a plan that is not working;
4. `ExecStopPost=/usr/local/sbin/wifi_dual.py --restore` in the unit, for the
   case where it was killed outright and got to run none of the above.

And, as everywhere else here, **nothing in it ever switches a radio off**. The
self-test asserts that twice: once by parsing the manager's source and checking
no string it could hand to anything mentions rfkill, and once by asking every
scenario's model world whether anything reached for one.

### Choosing a network without losing the page you chose it from

`wifi_join` used to mean one thing: take the link down, bring another up, and
hope whoever asked reconnects. With two radios it means something better, and
the console does this now — the request goes to the **spare**, which associates,
gets an address and is tested, and only then does the traffic move across.
Nothing drops, and the browser does not reconnect.

`wifi_ctl.sh join` therefore behaves differently depending on whether the
manager is running, and it checks rather than assuming: with a manager it writes
the request to `/run/wifi-dual.request` and returns immediately; without one it
does what it always did. A join behind the manager's back would be undone within
a second anyway, since it re-pins both radios every tick.

A network somebody chose is not argued with for ten minutes — not by the
placement, and not by the scoring either. That second half was missing at first
and the reproduction caught it: the model promoted the asked-for radio, waited
out the hold-down, found the other one 38 dB louder and undid the whole thing,
which from the console looks exactly like the button not working.

### Running it, and seeing what it thinks

```bash
scp -r wifi_roam bpi-m4zero:~/ugv/
cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" sh ~/ugv/wifi_roam/install-dual.sh'
cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" sh ~/ugv/wifi_roam/install-dual.sh DUAL=on'
ssh bpi-m4zero 'cat /run/wifi-dual.json'          # both radios, no privilege needed
ssh bpi-m4zero 'journalctl -u wifi-dual -f'       # silent unless something moves
```

`install-dual.sh` names the dongle `wlan1` through
[20-usb-wlan.link](20-usb-wlan.link) — matched on `ID_BUS=usb` rather than on a
MAC, because the dongle is the part most likely to be replaced — and copies the
house networks into a `wlan1` stanza from the one `wlan0` already has, so the
passphrase never has to come near a command line again. Like `install.sh` it
leaves the thing switched off unless told otherwise, for the same reason: the
way it fails is a rover that needs carrying to a socket.

### Checking it without a rover, and the rule it exists to satisfy

```bash
python3 test_wifi_dual.py     # anywhere; ~1.4 s on the rover
```

CLAUDE.md says a fix for a fault nobody has reproduced is a guess, and that a
reproduction has to be validated against a recording of the real fault before it
may be used to judge anything. Both halves are here.

**The calibration runs first and gates everything after it.** Every sample in
`fixtures/outage-2026-08-24.log` is fed through the manager's own idea of
whether a link is carrying traffic and compared with what the rover recorded at
the time. It agrees on all 84, and on all 9,437 samples of the full two-day log
it was cut from. If it ever disagrees, the file stops rather than printing
scenario results underneath a failed calibration.

Getting to 100% took two corrections, and both were the model being wrong rather
than the rover:

- requiring a readable signal scored **91.3%**, and every disagreement was a
  link the rover had up and pinging in milliseconds on which the driver had not
  filled in the level column;
- requiring a named SSID as well scored **99.4%**, and the remaining 59 were the
  same thing one field along — up, addressed, answering in 2 ms, and `iw` naming
  no network.

**Then the scenarios**, which are failures no recording contains because they
have not happened yet: an access point with a signal and no path to the LAN, a
router switched off under an associated radio, the dongle falling off the USB
bus, only one radio present, both radios starting on one router, a rover driving
across the house, a service address somebody else already answers for, a network
chosen by hand, and everything failing at once.

One of them is there because the rover did it. Five minutes after the manager
was first armed the journal read `moving wlan1 from TheMaharaja to
TheGreatViking at -50 dBm` and, twenty-nine seconds later, `moving wlan1 from
TheGreatViking to TheMaharaja at -66 dBm`, in a house where nothing had moved.
That is the scan noise this file already documents further down — the same
access point read twenty-three decibels apart inside a minute — and it matters
because a spare that is re-associating is a spare that is not ready. The model's
scans were noiseless and had said nothing about it; they are not now.

## The three networks

`TheGreatLord`, `TheMaharaja` and `TheGreatViking` are three separate routers
bridged onto one `192.168.1.0/24` LAN. The ARP table is the evidence:
TheGreatViking's router **is** the gateway at `192.168.1.1`, TheGreatLord's AP sits
on the LAN at `.2` and TheMaharaja's at `.232`, each answering on the same MAC it
beacons with.

That single fact is what makes choosing between them on signal strength alone
safe: whichever one the rover lands on it keeps a `192.168.1.x` address, so the
workstation can still reach the daemon on port 8769 and the daemon can still reach
the face detector on MEDIA. **An AP that routed its own subnet would break that**,
and adding one to `NETS` would give the rover a way to be "connected" and useless
at the same time.

All three share one passphrase, kept in `secrets/wifi.key` in the repo, which is
gitignored.

## What the script decides

`wifi_roam.sh` runs once per tick and asks two questions in order.

**Is anything wrong?** Answered without touching NetworkManager at all, out of
`/sys/class/net/wlan0/operstate`, `/proc/net/wireless` and `/proc/net/route`: is
the interface associated, does it have a route — which it only has once DHCP has
answered — and is the signal above −78 dBm. If all three hold, the script exits
without a word, in about 64 ms. Going through `nmcli` instead would cost 1.8
seconds of wall time and half a second of CPU on the Pi 1 this first ran on,
three times a minute, on the one armv6 core that was also running SLAM. The cheap path is not an
optimisation; it is the reason the timer can run this often.

`operstate`, and not `carrier`, which is the more obvious question and took the
rover off the network for an evening. Reading `carrier` on an interface that is not
administratively up fails with `EINVAL` rather than answering `0`: the `awk` that
read it died on the spot, printed nothing, and the fallback for an `awk` that
printed nothing was four values meaning *healthy* — on the sound principle that a
watchdog unable to read the system should leave the link alone rather than thrash
it. The result was a flawless link reported for a rover whose radio was switched
off, three times a minute, with not one line in the journal. `operstate` is a word
rather than a flag and is readable in every state: `up` once associated, `dormant`
while the supplicant is looking, `down` when the interface is not up at all. It is
read by the shell rather than handed to `awk`, which costs nothing — a builtin, and
one file fewer to open — and turns "no such interface" into a plain answer instead
of an error. The fallback is still there for a `/proc` that cannot be read, but it
now says so out loud instead of assuming the best.

**Where should it go?** Only reached when something is wrong. It scans, and takes
the strongest of the three networks other than the one it is on, provided that one
reaches 45 on NetworkManager's 0–100 scale.

Both thresholds are loose for reasons measured on this dongle rather than guessed:

- **−78 dBm** for "this link is failing". The driver's own reading is far steadier
  than a scan's, but it alternates between two values about 9 dB apart as it
  switches between beacon and data measurements — this rover reads −35 to −44 dBm
  in the lab — and the occasional read comes back tens of dB low for no visible
  reason. The threshold has to clear both.
- **45, as a floor rather than a comparison.** `nmcli`'s scan figure is the noisy
  one: consecutive scans reported the *same* association anywhere from 74 to 88,
  and one AP swung from 50 to 97 and back inside a minute. That is good enough to
  answer "is anything decent over there" and nowhere near good enough to answer
  "is that one twenty points better than this one", so the script never compares
  the two numbers against each other. Whether to leave is decided from the driver;
  where to go is decided from the scan.
- **Anything below −110 dBm is not a signal at all.** `−256` is the "no value"
  sentinel the radio keeps permanently in the *noise* column of
  `/proc/net/wireless`, and it turns up in the *level* column too. Measured on
  the `rtl8xxxu` dongle and still true of the `brcmfmac` radio that replaced
  it. Sampled at 1 Hz
  for two minutes, the link held between −33 and −50 dBm with three isolated
  `−256` reads scattered through it, each a single sample with good ones either
  side. Read as a number that is a link 200 dB down, and it used to be: five
  strikes in half an hour, `down to -256 dBm (1 of 3)` in the journal each time,
  and three landing in a row would have carried the rover off an association
  measuring −42. A reading outside what a radio can report is therefore not a weak
  signal, it is **no signal reported** — which the script answers by saying so and
  leaving the link exactly where it is. It still knows when the association or the
  address has gone; a link it cannot grade is simply not a link to move on the
  strength of the grade.

Then there is patience, and how much of it each fault deserves.

A link that is **associated but bad** has to fail three consecutive checks — about
a minute. Switching costs a DHCP round and every TCP connection the daemon is
holding, so it should take a bad link rather than one bad moment. After a move,
three minutes of cooldown stop a rover parked in a bad spot from cycling the whole
list.

And the link is read again twice on the way to a switch, because a decision that
was right when it was taken can be wrong by the time it is acted on:

- **Once as soon as anything looks wrong**, before the strike is even counted. It
  costs 64 ms of `/proc` against a reconnect the rover would feel, and it is what
  makes a single bad sample cost nothing at all.
- **Once more after the scan**, immediately before the association. This is the
  one that matters most, because the scan is the slow part of the whole script —
  **thirty-two seconds, measured**, on a Pi also running SLAM. A fault answered
  half a minute late is a fault that may be long over, and spending an association
  on it takes down a link that is working, which is the entire cost this thing
  exists to avoid.

A link that is **not associated at all** skips the strikes entirely: there is
nothing left to protect, and it is the case this whole thing exists for. What
limits it instead is one attempt per minute, because **scanning is the dangerous
operation on this hardware**. That was first measured when the wifi was a USB
dongle sharing a weakly fused bus with the camera and the lidar: a burst of
forced scans during a run that was also streaming camera frames is what took the
rover off the network for an afternoon while this was being written. The radio
moved off that bus on 2026-08-24 and onto the board's own SDIO one, so the
contention is gone — but the limit stays, because a scan still takes the radio
off channel for seconds and the link is the only way in. Hammering a radio that is already down, three times a
minute, is how a rover that would have recovered on its own stays offline instead.

And after three minutes of nothing working at all, the script stops trying to
choose and tries to repair: it restarts `wpa_supplicant`, which costs nothing that
is working and clears a supplicant that has lost track of its interface. That is
the cheap half of what a power cycle does. A radio the kernel has genuinely lost
needs the other half, and a person.

## The radio switch, and why nothing here turns it off

That repair used to be `nmcli radio wifi off`, three seconds, and `nmcli radio
wifi on`. What that cost is worth writing down, because what replaced it is a rule
rather than a patch.

`nmcli radio wifi off` is not a transient act. NetworkManager keeps the switch in
`/var/lib/NetworkManager/NetworkManager.state` and restores it at every boot —
`rfkill: Wi-Fi enabled by radio killswitch; disabled by state file` is the line it
logs while doing so. So a power cut inside those three seconds, or an `on` that
merely failed, which nothing checked, left the rover soft-blocked on that boot and
on every boot after it: hardware killswitch enabled, `wlan0` parked at
`unavailable`, NetworkManager not even attempting to associate. The only other way
into a rover is an ethernet cable, and that is unplugged the moment it drives off.
This Pi has no console, and its journal lives in RAM, so the reboot that made the
fault permanent also took away the evidence for it.

**Changing stacks does not retire that lesson; it only moves the state file.**
The Banana Pi has no NetworkManager to keep a switch for it, but a soft rfkill
block is saved and restored across reboots by systemd, so an `rfkill block wifi`
there is exactly as permanent — and that board has no ethernet socket at all, so
the cable that rescued the Pi does not exist. Both boards are therefore held to
the same rule below, and the self-test asserts it for both.

Two changes, so that neither half of that can happen again:

- **Nothing in `wifi_roam.sh` turns the radio off.** The only thing it does to
  that switch is turn it *on*, which it checks for whenever the link has no
  association at all — one `nmcli` call, on a path that is already broken, at most
  once a minute. Nothing else it could do would work while the switch is off
  anyway: the scan comes back empty and there is no radio for `con up` to bring a
  profile up on. The self-test asserts the absence across every scenario in the
  file, not only in the repair.
- **`wifi-radio-on.service` asks for the radio on at every boot.** One call
  through `wifi_ctl.sh` — `nmcli radio wifi on` or `rfkill unblock wifi`,
  whichever this board takes — idempotent and silent on a radio that is already
  on. It is what makes the guarantee independent of how the switch came to
  be off — this script, an older copy of it, or a hand at a console: no setting of
  that switch survives a reboot.

The timer also waits **three minutes after boot** before its first check, for the
same reason. NetworkManager takes 46 seconds of this Pi's boot on its own and the
first association and DHCP land somewhere past a minute, so a timer that starts at
a minute finds `wlan0` not yet associated, calls that a fault, and begins scanning
on top of NM's own attempt. That is not hypothetical: an earlier version started at
60 s and took the rover off the network twice in one afternoon. Nothing here
improves on NetworkManager during its first go.

## Two things move this link, and only one at a time

`wifi_roam.sh` on its timer is one of them. A person at the console asking for a
particular network, through `wifi_ctl.sh join`, is the other. They used to be able
to run at the same moment, and the result was the fault this section exists for.

`nmcli con up` takes the interface down for the ten seconds it spends
authenticating and associating, and **nothing in `/proc` tells that apart from a
rover that has lost the network.** So a tick landing inside that window read
`operstate`, found no association, called it a fault — the not-associated fault,
which skips the strikes entirely because there is supposedly nothing left to
protect — and went off to choose a network of its own. What happened next is worth
reading off the journal in full, because every number in it is load-bearing:

```
17:57:52  nmcli: connection-activate TheGreatViking      <- somebody asks
17:57:59  wifi_roam.sh probes /proc: not associated      <- mid-join, reads as broken
17:58:02  NetworkManager: TheGreatViking activated       <- the join worked
17:58:31  wifi_roam.sh: joining TheGreatLord at 90       <- 32 s later, the scan answers
17:58:52  NetworkManager: TheGreatLord activated         <- and undoes it
```

The chosen network lasted 43 seconds. From the outside this is exactly the
complaint: *sometimes it joins the new network briefly and then reverts.* The
other half of the complaint — *sometimes it drops off for a long time* — is the
same race with the strike counter left standing, since a `con up` that fought
another one could leave the rover on neither.

Three changes, and they are ordered by how much they cover:

- **`wifi_ctl.sh join` holds a lock for the length of the join**, and
  `wifi_roam.sh` takes it the moment it stops liking the look of the link. A tick
  that cannot get it says so and does nothing at all — not even a strike, because
  somebody else is already moving the link and the next tick will see wherever
  they moved it to. It is `flock`, so the lock dies with the process holding it; a
  lock file of the script's own making would have locked the rover out of its own
  network keeper the first time one was killed. The lock is taken where the cheap
  path ends rather than at the top of the script, so the ticks that find nothing
  wrong — almost all of them — still cost 64 ms and no processes.
- **`wifi_ctl.sh join` clears the strike count and stamps the clock** before it
  activates anything, in the same file `wifi_roam.sh` reads back. A network
  somebody chose then gets exactly the cooldown a network the script chose gets,
  instead of being graded on the first reading taken after it arrives.
- **The link is read again immediately before the association**, which is
  described above and is what catches the tail of a join that began before the
  lock was taken.

Order between the two is now decided rather than raced, and it comes out the right
way round either way: a roamer already mid-scan is waited out and then overridden,
and a roamer that arrives during a join stands aside. The person asking wins.

## Choosing by hand, from the console

`wifi_roam.sh` is what happens when nobody is watching. When somebody is,
[drive_web.py](../drive_web/drive_web.py) has a panel that shows which
access point the rover is on and offers the others, and it reaches them through
two calls on the daemon — `wifi_status` and `wifi_join`.

Neither is offered to the voice model. They are dispatched like tools because that
is the only protocol the daemon speaks, and kept out of `list_tools` for a reason
particular to this pair: a model that decided to move the rover onto another access
point would be cutting the wire its own conversation arrives on, and no wording of
a description makes that a good idea. A person at a console, who can see which
network they are on and will notice the reconnect, is a different matter.

The daemon runs as `admin`, and scanning and switching need root, so both go
through `wifi_ctl.sh` with a passwordless sudo rule for that one path — installed
by `install.sh`, which writes it through `visudo -c` first, because a malformed
file in `/etc/sudoers.d` breaks every `sudo` on a box that has no console to
repair it from. **A join can only ever reach a network that is already configured
here**: the SSID is checked against NetworkManager's own list of wifi profiles
before it is used, so the worst a caller can ask for is one of the networks
somebody has already put a passphrase in for.

`wifi_status` answers in about two seconds and the console polls it every five, so
the list of networks is cached for twenty and only the signal strength is read
fresh — out of `/proc`, for nothing. `wifi_join` answers *before* it acts, because
the switch takes the link down and the reply would be written into the connection
it is breaking; what actually happened comes back as `last_join` on the next
`wifi_status`, once the caller has reconnected.

## Checking it without a rover

```bash
./selftest.sh      # 47 assertions, anywhere: the workstation, the rover, a VM
```

It takes a second or two on a desk and a couple of minutes on the rover, which is
not a hang: it runs the script thirty-odd times, and on the Pi 1 a process spawn
under load costs a hundred times what it does here.

Every input the script reads and both commands it acts through are replaceable,
so the self-test hands it a fake `/proc`, a fake `nmcli` and a fake `systemctl`,
and drives it through all of it: healthy, fading, recovering, associated with no
address, off the air, off the air and staying that way, an interface that is not
even up, no interface at all, a radio somebody switched off, a switch that will
not move, a radio that answers an empty scan, an association that fails, a
neighbourhood with nothing worth moving to, a `/proc` that cannot be read, and a
state file left half written. The interesting branches are the ones that only
happen when something has gone wrong, and waiting for a real dongle to go wrong is
not a test strategy.

Four of those scenarios are there because they were missed once. An interface that
is not up and a radio that is switched off both used to read as a healthy link;
a `−256` in the level column used to read as a link 200 dB down; and a deliberate
join in flight used to read as a rover that had lost the network. The assertion
that spans the whole file — that no scenario in it ever emits `radio wifi off` — is
the one that keeps the rover recoverable by rebooting it.

The races are covered too, which needed fakes that answer differently the second
time they are asked. Both of the script's second looks exist because a rover can
recover between two reads of `/proc` inside one run, and a static fake cannot
express that at all — so the fake `nmcli` rewrites the fake `/proc` while it is
pretending to scan, which is precisely what a deliberate join finishing does. The
lock is tested by holding it from the test itself while the script runs; that one
needs `flock`, so it is skipped, out loud, on a desk that has none.

The self-test also drives `wifi_ctl.sh`, which is not the same script but is the
other thing allowed to move this link: that a join reaches the profile already
holding the passphrase, that it bounds its own wait, that it leaves the stamp
`wifi_roam.sh` reads back, and that a network with no passphrase on this rover is
still refused without anything being brought up on the way to refusing it.

## Installing and checking it on the rover

```bash
scp -r wifi_roam bpi-m4zero:~/ugv/
cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" ~/ugv/wifi_roam/install.sh EverGreen'   # passphrase only needed once
```

`ROAM=off` installs everything and leaves the roam timer disabled, which is how
the Banana Pi is set up today — the dual-radio manager owns the radios, and
arming this timer would fight it. On a board that still has one radio:

```bash
cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" sh ~/ugv/wifi_roam/install.sh'
```

## The same script on two quite different stacks

The Pi 1 runs NetworkManager. The Banana Pi runs netplan, `systemd-networkd` and
`wpa_supplicant`, has no `nmcli` at all, and keeps the house networks in
`/etc/netplan` rather than in NM profiles. Three things differ, and only three:

| | Pi 1 | Banana Pi |
|---|---|---|
| looking around, and moving the link | `nmcli dev wifi list`, `nmcli con up` | `wpa_cli scan_results`, `select_network` |
| the radio switch | NetworkManager's own state file | rfkill, saved and restored by systemd |
| the supplicant to restart when wedged | `wpa_supplicant.service` | `netplan-wpa-wlan0.service` |

The first of those is not in `wifi_roam.sh` at all. Scanning and joining live in
`wifi_ctl.sh`, which the daemon already calls for the console's network panel and
which already spoke both dialects, so the roamer delegates and stays one script.
That has a second benefit worth having: the list the roamer chooses from and the
list a person sees on the console are the same list, from the same code, on the
same 0–100 scale — `wifi_ctl.sh` converts the supplicant's dBm into
NetworkManager's scale precisely so that one set of thresholds fits both boards.

The last of those three is the one that would have failed quietly. A netplan box
*also* has a `wpa_supplicant.service`, dbus-activated and managing nothing at all,
so restarting it succeeds, logs a repair and leaves the wedged supplicant exactly
where it was. The script asks systemd which unit is actually active before it
restarts anything.

There is also a hazard in the wpa join that is worth knowing about, because it is
the one way this code could strand a wifi-only rover. `select_network` disables
every *other* configured network in order to make its attempt, so a join that
never completes would leave the rover holding one AP it has just proved it cannot
reach and forbidden from trying the two it can. Both the success and the failure
paths re-enable them, and the self-test asserts the failure one.

`install.sh` is idempotent, and it will not touch the passphrase of a profile that
already exists — a working link is not worth risking to a typo. What it does always
set is `autoconnect-retries 0`, unlimited: NetworkManager's default of four
attempts blocks a profile after a handful of failures, which is precisely what a
rover parked out of range does before somebody carries it back inside.

It installs two units, not one: `wifi-roam.timer`, which runs the script, and
`wifi-radio-on.service`, which asks for the radio on at boot. Both are enabled
with `--now`, so running `install.sh` is also the repair for a rover found with
its radio switched off.

```bash
ssh bpi-m4zero 'systemctl list-timers --no-pager wifi-roam.timer'
ssh bpi-m4zero 'journalctl -u wifi-roam --since -1h --no-pager'   # silent when all is well
cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" wifi_roam.sh -n'   # one check, changes nothing
cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" env LOW=-20 STRIKES=1 wifi_roam.sh -n'   # force the decision path
```

Every threshold is an environment variable, so the last of those is how to see
what it would do about a link it currently considers fine. It needs `sudo` because
scanning and activating a connection are both polkit-guarded, and polkit grants
those to an active local session — which a timer is not, and an ssh session is not
either.

This is the one part of the rover that is a systemd unit rather than a `@reboot`
crontab entry like the daemon. Not a change of heart about which suits this box:
root's crontab would serve, but a minute is its finest interval and a rover
crosses a house in less.
