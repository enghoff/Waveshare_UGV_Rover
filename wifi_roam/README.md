# The rover's Wi-Fi

One radio, one network it comes up on, and nothing that changes networks by
itself.

The rover joins `TheGreatViking` at every boot, on its onboard radio, and stays
there. It holds profiles for the two other house networks as well, with
autoconnect switched off, so the only thing that ever puts it on one of those is
a person at the console pressing `join`. Nothing scans unprompted, nothing roams,
and nothing hands the link between radios.

The address to reach it at is:

```text
192.168.1.80
```

That address is a fixed extra address on each of the three profiles, alongside
the DHCP lease NetworkManager also takes. It is on all three because the house
networks are separate SSIDs bridged onto one LAN, so the rover answers there
whichever of them it is on -- including one somebody chose by hand. The console's
certificate names it, which is why the browser gets a clean padlock at
`https://192.168.1.80:8771/`.

The per-radio DHCP lease still works and is the way back in if the service
address is ever unreachable, as is `jetson-orin.local`.

## What is here

| File | What it does |
|---|---|
| [`install-profiles.sh`](install-profiles.sh) | writes the three NetworkManager profiles and decides which one autoconnects |
| [`wifi_ctl.sh`](wifi_ctl.sh) | the privileged helper: list, scan, join, profiles, status |
| [`install.sh`](install.sh) | puts both on the rover, with the sudo rule the console needs |
| [`install-mdns.sh`](install-mdns.sh) | makes `jetson-orin.local` resolvable |
| [`selftest.sh`](selftest.sh) | drives both scripts against a fake board, with no radio |

## The profiles, which are the whole policy

`install-profiles.sh` is where the rover's network behaviour actually lives.
There is no daemon and no timer; there is a set of NetworkManager profiles, and
what they say is what the rover does.

| Profile | Autoconnect | Pinned to | Address |
|---|---|---|---|
| `TheGreatViking` | **yes** | the onboard radio | DHCP + `192.168.1.80` |
| `TheGreatLord` | no | the onboard radio | DHCP + `192.168.1.80` |
| `TheMaharaja` | no | the onboard radio | DHCP + `192.168.1.80` |

Three settings in that table are each load-bearing.

**Only one profile autoconnects.** A second one set to autoconnect would put the
rover on whichever network it happened to see first, which is a choice nobody
made -- and it is how the old roaming behaviour would come back by the side door.

**Every profile is pinned to the onboard radio.** This is the opposite of what
this file used to do, and deliberately so: with no failover there is no spare
radio to keep a network free for, and an unpinned profile is one NetworkManager
may bring up on the USB dongle instead. The radio is found by its bus rather than
by name, because the onboard Realtek is `wlP1p1s0` here and was `wlan0` on both
earlier boards.

**`autoconnect-retries 0`.** NetworkManager's default of four attempts blocks a
profile after a handful of failures, which is exactly what a rover parked out of
range does before it is carried back inside -- and what once left this rover's
radio off the air for six hours after a single link timeout.

The profiles are matched by the network each one is *for*, never by its name. The
rover has carried a profile called `TheGreatViking-dongle`, and a loop looking
for one called `TheGreatViking` would have added a second profile for the same
network rather than finding the one already there. Duplicates are deleted, since
two answers to "how do I join this" is one too many.

All three networks share one passphrase, which lives in `~/.ugv/wifi.key` on the
rover, outside the deploy tree. It is only used for a profile that does not exist
yet -- an existing one keeps the key it has, because a working link is not worth
risking to a typo -- and it is read from that file rather than taken as an
argument, because an argument is readable in `ps` by every account on the machine.

## The USB dongle

Its driver is built, loaded and kept working -- see
[`dongle_driver/`](../dongle_driver) -- and NetworkManager is told to leave the
interface alone, in `/etc/NetworkManager/conf.d/99-unmanaged-usb-wifi.conf`. The
radio is on the bus and available; nothing routes through it.

The match is by driver (`rtl8xxxu`) rather than by interface name, because the
name is the kernel's MAC-derived `wlx...` and a replacement dongle would have a
different one while being the same thing.

To hand it back, delete that file and restart NetworkManager.

## Choosing a network by hand

The console's network panel has a "look for networks" button and a `join` button
on each network the rover holds a passphrase for. Both go through
`wifi_ctl.sh`, because scanning and activating a connection are polkit-guarded
and polkit grants those to an active local session, which a daemon is not. The
sudo rule `install.sh` writes covers that one path and nothing else.

**A join costs the link.** The rover has one radio: it takes the current network
down and brings another up, so every connection to the rover dies, including the
browser that asked. The console warns before it happens and reconnects a few
seconds later. This is why nothing calls it unprompted.

A chosen network is held until somebody chooses another or the rover reboots.
There is no cooldown and no reconsidering, because nothing is doing any
considering -- **a reboot always comes back to `TheGreatViking`.**

```bash
ssh orin 'sudo -n /usr/local/sbin/wifi_ctl.sh profiles'   # what it can join
ssh orin '/usr/local/sbin/wifi_ctl.sh status'             # every radio, one line each
ssh orin 'sudo -n /usr/local/sbin/wifi_ctl.sh join TheGreatLord'
```

`status` is deliberately unprivileged and reads the kernel rather than
NetworkManager, so it still answers on a rover whose sudo rule was never
installed and on one where NetworkManager is the thing that is broken. It lists
the dongle too, with no network beside it, because a status that hid it would
look like a radio had gone missing.

## Install and deploy

Normal source deployment stages the scripts and runs the self-test:

```bash
python deploy/deploy.py --only wifi_roam
```

That deliberately does not replace the privileged running copy. After reviewing
the change and making sure there is a way back in:

```bash
python deploy/deploy.py --system --only wifi_roam
```

**A first system install wants a reboot afterwards.** `install.sh` writes the
profiles, installs the helper and its sudo rule, and retires the units the rover
used to run -- but it does not activate anything, because activating would cut
the SSH session running the install. NetworkManager keeps the connection that is
already up, so the rover stays exactly where it is until it is rebooted, and the
reboot brings the whole new arrangement up at once. The script says so when there
was something to retire.

What it retires, on a rover that still has them: `wifi-dual.service`,
`wifi-roam.timer` and `wifi-roam.service`, `wifi-radio-on.service`,
`dongle-keeper.timer` and `dongle-keeper.service`, and the privileged scripts,
sysctl drop-in and link file that went with them.

## What used to be here, and why it is not

The rover ran a dual-radio failover manager (`wifi_dual.py`), and before that a
single-radio roaming timer (`wifi_roam.sh`). Both are gone, along with their
units, their simulations and the recordings they were calibrated against. Git
history has them.

They solved a real problem -- a radio can stay associated to an access point that
still beacons while packet loss makes the link unusable, and testing another one
with the same radio means interrupting the only path the rover has -- and they
solved it by keeping two radios associated and moving the service address between
them. That bought a failover measured at about twenty seconds on this rover.

It cost a manager that owned both radios and could be argued with by anything
else that touched them, four separate recovery layers against the case where it
died holding a radio somewhere it could not associate, per-radio routing tables
that had to be rebuilt every time a lease changed, and a console panel explaining
which radio was carrying what. The rover sits in a house within range of three
access points that share a passphrase. Joining one of them and staying there is
the behaviour that was actually wanted.

## Tests and reproduction

```bash
sh wifi_roam/selftest.sh
```

No radio, no root and no NetworkManager needed: it builds a sysfs with an onboard
radio and a USB one, a routing table, a `/proc/net/wireless`, and a fake nmcli
that remembers what it was told, then checks the state the scripts leave things
in. The assertions worth knowing about are that exactly one profile autoconnects,
that none of them is left free to land on the dongle, that the passphrase never
reaches a command line, and that a second run changes nothing.

It is also run by `install.sh` before the helper is put in place, so a copy that
arrived with CRLF line endings or arrived half written fails there rather than on
a rover that has driven out of range.

## Relationship to `netwatch`

[`netwatch/`](../netwatch) records the network and changes nothing. It was kept
separate so that an outage had evidence from a component that was not also trying
to repair the link; with nothing left here that repairs anything, the separation
costs nothing and is still the right shape.
