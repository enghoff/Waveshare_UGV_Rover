# Keeping the rover on the network

A rover that drives out of one router's range and loses the network does not come
back on its own in any useful sense. NetworkManager reconnects, eventually, to
whichever access point it happens to pick — and the one it picks is the one it
used most recently, not the one now shouting through the nearest wall. Worse, it
will not leave an association at all while beacons keep arriving, so the rover can
sit at the edge of an AP's range dropping most of its packets while a much
stronger AP goes unused three metres away.

This directory is the fix: three saved network profiles, and a small script on a
20-second timer that watches the link and moves it when it has to.

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
seconds of wall time and half a second of CPU on this Pi, three times a minute, on
the one armv6 core that is also running SLAM. The cheap path is not an
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
  sentinel this `rtl8xxxu` dongle keeps permanently in the *noise* column of
  `/proc/net/wireless`, and it turns up in the *level* column too. Sampled at 1 Hz
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
operation on this hardware**. The wifi dongle shares a weakly fused USB bus with
the camera and the lidar, and a burst of forced scans during a run that was also
streaming camera frames is what took the Pi off the network for an afternoon while
this was being written. Hammering a radio that is already down, three times a
minute, is how a rover that would have recovered on its own stays offline instead.

And after three minutes of nothing working at all, the script stops trying to
choose and tries to repair: it restarts `wpa_supplicant`, which costs nothing that
is working and clears a supplicant that has lost track of its interface. That is
the cheap half of what a power cycle does. A dongle that has genuinely fallen off
the USB bus needs the other half, and a person.

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

Two changes, so that neither half of that can happen again:

- **Nothing in `wifi_roam.sh` turns the radio off.** The only thing it does to
  that switch is turn it *on*, which it checks for whenever the link has no
  association at all — one `nmcli` call, on a path that is already broken, at most
  once a minute. Nothing else it could do would work while the switch is off
  anyway: the scan comes back empty and there is no radio for `con up` to bring a
  profile up on. The self-test asserts the absence across every scenario in the
  file, not only in the repair.
- **`wifi-radio-on.service` asks for the radio on at every boot.** One `nmcli`
  call, ordered after NetworkManager, idempotent and silent on a radio that is
  already on. It is what makes the guarantee independent of how the switch came to
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
[drive_web.py](../voice_chat/drive_web.py) has a panel that shows which
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
./selftest.sh      # 47 assertions, anywhere: the workstation, the Pi, a VM
```

It takes a second or two on a desk and a couple of minutes on the rover, which is
not a hang: it runs the script thirty-odd times, and on that Pi a process spawn
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

## Installing and checking it on the Pi

```bash
scp -r wifi_roam rpi:~/ugv/
ssh rpi 'sudo ~/ugv/wifi_roam/install.sh EverGreen'   # passphrase only needed once
```

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
ssh rpi 'nmcli radio wifi'                                 # enabled, or nothing works
ssh rpi 'systemctl list-timers --no-pager wifi-roam.timer'
ssh rpi 'journalctl -u wifi-roam --since -1h --no-pager'   # silent when all is well
ssh rpi 'sudo wifi_roam.sh -n'                             # one check, changes nothing
ssh rpi 'sudo LOW=-20 STRIKES=1 wifi_roam.sh -n'           # force the decision path
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
