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
`/sys/class/net/wlan0/carrier`, `/proc/net/wireless` and `/proc/net/route` in a
single `awk` pass: is the interface associated, does it have a route — which it
only has once DHCP has answered — and is the signal above −78 dBm. If all three
hold, the script exits without a word, in about 64 ms. Going through `nmcli`
instead would cost 1.8 seconds of wall time and half a second of CPU on this Pi,
three times a minute, on the one armv6 core that is also running SLAM. The cheap
path is not an optimisation; it is the reason the timer can run this often.

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

Then there is patience, and how much of it each fault deserves.

A link that is **associated but bad** has to fail three consecutive checks — about
a minute — and the signal is read once more immediately before switching, which
catches a link that recovered while its strikes were being counted. Switching
costs a DHCP round and every TCP connection the daemon is holding, so it should
take a bad link rather than one bad moment. After a move, three minutes of
cooldown stop a rover parked in a bad spot from cycling the whole list.

A link that is **not associated at all** skips the strikes entirely: there is
nothing left to protect, and it is the case this whole thing exists for. What
limits it instead is one attempt per minute, because **scanning is the dangerous
operation on this hardware**. The wifi dongle shares a weakly fused USB bus with
the camera and the lidar, and a burst of forced scans during a run that was also
streaming camera frames is what took the Pi off the network for an afternoon while
this was being written. Hammering a radio that is already down, three times a
minute, is how a rover that would have recovered on its own stays offline instead.

And after three minutes of nothing working at all, the script stops trying to
choose and tries to repair: it takes the radio down and up, which costs nothing
that is working and clears a supplicant that has lost track of its interface. That
is the cheap half of what a power cycle does. A dongle that has genuinely fallen
off the USB bus needs the other half, and a person.

The timer also waits **three minutes after boot** before its first check, for the
same reason. NetworkManager takes 46 seconds of this Pi's boot on its own and the
first association and DHCP land somewhere past a minute, so a timer that starts at
a minute finds `wlan0` not yet associated, calls that a fault, and begins scanning
on top of NM's own attempt. That is not hypothetical: an earlier version started at
60 s and took the rover off the network twice in one afternoon. Nothing here
improves on NetworkManager during its first go.

## Choosing by hand, from the console

`wifi_roam.sh` is what happens when nobody is watching. When somebody is,
[drive_console.py](../voice_chat/drive_console.py) has a panel that shows which
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
./selftest.sh      # 24 assertions, anywhere: the workstation, the Pi, a VM
```

It takes a second or two on a desk and a couple of minutes on the rover, which is
not a hang: it runs the script thirty-odd times, and on that Pi a process spawn
under load costs a hundred times what it does here.

Every input the script reads and the one command it acts through are overridable,
so the self-test hands it a fake `/proc` and a fake `nmcli` and drives it through
all of it: healthy, fading, recovering, associated with no address, off the air,
off the air and staying that way, a radio that answers an empty scan, an
association that fails, a neighbourhood with nothing worth moving to, and a state
file left half written. The interesting branches are the ones that only happen
when something has gone wrong, and waiting for a real dongle to go wrong is not a
test strategy.

One path is not covered there and was verified on the hardware instead: the second
look at the signal just before switching. It reads the same files twice in one run,
so a fake that answered differently the second time would only be testing the
fake.

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

```bash
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
