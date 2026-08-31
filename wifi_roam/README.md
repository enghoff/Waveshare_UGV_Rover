# Dual-radio Wi-Fi on the rover

The Banana Pi runs both Wi-Fi radios at once and keeps a stable rover service
address on whichever healthy path is better. This is the current network manager;
the older single-radio roam timer is retained only as an installed recovery
alternative and must remain disabled while `wifi_dual` is active.

Current radios:

- `wlan0` — onboard Broadcom BCM4345/6, dual-band, preferred radio;
- `wlan1` — USB Realtek RTL8188FTV, 2.4 GHz, redundant/standby radio.

The application address is:

```text
192.168.1.80
```

The two interfaces also keep their own DHCP addresses, but applications should
normally use the stable service address. `wifi_dual.py` moves `.80` between the
interfaces and sends gratuitous ARP so open TCP sessions can survive a radio
handover without changing source address.

Moving the address is only half of a handover. Each address here also gets a
policy rule pinning its traffic to one routing table per radio, so a reply
leaves by the radio the request arrived on. Without that the main table decides,
and with two radios on one subnet it holds one connected route each at the same
metric and breaks the tie the same way regardless of which radio is carrying
traffic. `.80` had no such rule until 2026-08-26, which is why a failover that
worked in every other respect left the rover unreachable at its own service
address for eleven and a half minutes; see
[`docs/rover-unresponsive.md`](../docs/rover-unresponsive.md).

Those routes also have to be rebuilt when a lease changes, which is not the same
event as a failover. The kernel deletes a route when the address it is anchored
to goes away -- `kernel_route_lifetime.sh` measures that on the board, and
dropping the `src` does not save them either, because the gateway they point
through is in the prefix that left. Until 2026-08-27 they were written only when
traffic moved between radios, so a renewal emptied the table the service address
points at and nothing put it back. That matters here because this LAN has a
second DHCP server on it besides the router -- a TP-Link extender at
`192.168.1.232`, which also beacons as `TheMaharaja` -- and whichever answers
first decides, so the rover's addresses change several times an hour without a
radio ever losing its association.

## When a radio cannot join what it is being held on

Holding a radio on one router is `wpa_cli select_network`, and that works by
disabling every other network the radio knows. So a radio held on an access
point that will not keep it has nowhere it is permitted to fall back to: it
joins, loses carrier, retries the same one, and the console shows a spare that
is "not associated" beside a list of networks at full strength that it is not
allowed to try.

After `STRANDED_S` (90 s) with no association and no address, the manager
therefore stops holding it: `release` re-enables every network, the supplicant
takes whatever it can actually hold, and the network that would not have it is
left out of that radio's next placement for `REFUSED_S` (10 minutes). Being on
the air somewhere beats being on the router the placement rules would prefer.

A placement that has not changed is also no longer re-announced. Repeating the
same choice meant another `select_network`, which restarts the association --
so a radio slow to join was interrupted every time the manager scanned, and the
log filled with "moving from nothing to" lines about a radio nothing was moving.

## Why two associated radios

The failure this solves is not simply "choose the strongest AP". A single radio
can stay associated to a weak access point that still emits beacons while packet
loss makes the link practically unusable. Testing another AP with that same radio
requires interrupting the only path the rover currently has.

With two radios the standby can remain associated and tested while the active
radio carries traffic. A handover is then an address/routing change rather than a
scan/association/DHCP sequence performed after the rover is already unreachable.

A `netwatch` recording from 2026-08-24 captured exactly that failure: the rover
spent two roughly two-minute periods associated and addressed on a poor link while
a much stronger AP was available. Replaying the recording against the dual-radio
logic produces immediate failovers instead of minutes of dead connectivity.
`test_wifi_dual.py` contains that replay and refuses to grade synthetic scenarios
if its interpretation disagrees with the recording.

## What runs

`wifi_dual.py` owns active-path selection. It evaluates the already-associated
links every second using current signal plus measured reachability/latency/loss.
The standby periodically scans and may move to a better independent access point.
The active radio is not taken off channel merely to refresh the network list.

The manager distinguishes two decisions:

| Decision | Evidence | Typical cadence |
|---|---|---:|
| which associated radio carries `.80` | association, address, gateway reachability, signal and live path quality | 1 s |
| where the standby should associate | scan results plus configured-network/router grouping | ~30 s |

A link is usable only when it is associated, addressed and can reach the gateway.
Signal by itself is not proof of connectivity; conversely a driver temporarily
omitting a signal value is not proof that a link carrying packets is dead.

The onboard radio wins a near tie because it is dual-band and has proven faster
and more reliable. The USB dongle remains valuable as an independent path even
though it is the weaker adapter.

## Keeping the two radios independent

The house networks are multiple SSIDs bridged onto the same LAN. Names on the
same physical router are grouped so `wlan0` on a router's 5 GHz SSID and `wlan1`
on that same router's 2.4 GHz SSID are not mistaken for redundancy.

The manager holds each radio on one chosen network, and how it does that depends
on the board. On the Banana Pi -- netplan, `systemd-networkd`, one
`wpa_supplicant` per interface -- it enables exactly one of that supplicant's
networks and disables the rest. On the Jetson it brings a NetworkManager profile
up on the interface, and disables nothing at all: an active connection is left
alone and the same profile is never put on two devices at once, so the two radios
cannot collapse onto one network without anything being taken away from them.

The second is much the safer, because it removes the only way this design could
strand a wifi-only rover. A manager that dies under NetworkManager leaves nothing
to undo and every network still autoconnectable.

The service address is safe only with the accompanying ARP settings in
[`99-dual-wifi.conf`](99-dual-wifi.conf):

- `arp_ignore=1`
- `arp_announce=2`

Without them Linux can answer ARP for `.80` on the wrong interface and teach the
LAN to deliver rover traffic to the standby radio.

Before claiming `.80`, the manager ARP-probes it. If another host answers, the
manager runs without the service address rather than creating a duplicate IP.
The individual DHCP addresses remain usable for recovery.

## Manual network selection

A console request to join a network is staged on the spare radio. The manager
associates it, obtains an address, checks reachability and only then promotes it.
The browser therefore does not need to disconnect merely because the user chose a
different AP.

Requests go through `/run/wifi-dual.request`; status is published to:

```text
/run/wifi-dual.json
```

The normal console/network helpers use the same mechanism rather than moving a
radio behind the manager's back, which the manager would undo within the second.

A manually chosen network is held long enough to make the user's choice meaningful
instead of immediately being undone by a small score difference.

## Failure containment

Selecting one network on an interface disables the alternatives in that
supplicant. A manager that crashed without undoing its choices could therefore
strand a Wi-Fi-only rover. Four recovery layers exist:

1. signal/normal shutdown restores both interfaces;
2. exceptions in the manager restore them;
3. a dead-man condition releases both when neither path can reach the gateway for
   a sustained period;
4. systemd `ExecStopPost` runs the explicit `--restore` path even after an
   abnormal service stop.

The manager does not use `rfkill` to turn radios off. A degraded manager should
leave normal supplicant behavior available rather than disable the hardware it
would need for recovery.

## Install and deploy

Normal source deployment:

```bash
python deploy/deploy.py --only wifi_roam
```

That stages/tests source but deliberately does not replace privileged running
files. After reviewing a network change and ensuring there is a recovery path:

```bash
python deploy/deploy.py --system --only wifi_roam
```

On the current rover that system install is `install.sh --dual`: the three house
networks, the `wifi_ctl.sh` helper the console needs to list and switch between
them, and the dual-radio manager armed. The one-radio roamer is deliberately not
installed and must never run alongside the manager -- they are alternative owners
of the same radios, one moving a radio when it thinks the link has failed and the
other holding both where it put them, and arming either disables the other.

The profiles are handled by `install-profiles.sh`, which is where the rule that
matters lives: **one profile per network and not one of them pinned to a radio.**
A pinned profile is one the spare radio cannot use, which is the whole point of
having a spare. It also matches profiles by the network each is for rather than
by its name, deletes duplicates, and sets `autoconnect-retries 0` so a profile
is never given up on. The passphrase for a network that has no profile yet is
read from `~/.ugv/wifi.key` on the rover, never from an argument.

Manual equivalent:

```bash
scp -r wifi_roam bpi-m4zero:~/ugv/
cat secrets/bpi-sudo.key | ssh bpi-m4zero \
  'sudo -S -p "" sh ~/ugv/wifi_roam/install-dual.sh DUAL=on'
```

Check it without privilege:

```bash
ssh bpi-m4zero 'cat /run/wifi-dual.json'
ssh bpi-m4zero 'journalctl -u wifi-dual -f'
```

`install-dual.sh` installs the USB-interface naming rule, system configuration and
service. It also disables the old `wifi-roam.timer` when dual mode is enabled.
Do **not** enable that timer while `wifi-dual` is running: they are alternative
owners of the same link and will fight each other's decisions.

## Tests and reproduction

```bash
python3 wifi_roam/test_wifi_dual.py
```

The test suite first replays the captured rover outage and checks that the
manager's classification agrees with what actually happened. Only after that
calibration passes does it run synthetic scenarios such as:

- signal present but gateway unreachable;
- one router disappearing;
- USB dongle disappearing;
- only one radio present;
- both radios initially on one physical router;
- service-address conflict;
- explicit user network choice;
- both links failing.

This follows the repository rule for network changes: reproduce a real failure
before changing the logic, then verify the same recording with the proposed fix.

## Relationship to `netwatch`

[`netwatch/`](../netwatch) is deliberately separate. `wifi_dual` changes the
network; `netwatch` only records it. Keeping the recorder independent means an
outage has evidence from a component that was not also trying to repair the link.

If the rover becomes unreachable, inspect `netwatch-report`, `/run/wifi-dual.json`
and the external `netprobe.py` log together before changing thresholds.
