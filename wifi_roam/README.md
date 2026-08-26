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

The manager holds each radio on one selected configured network by controlling
that interface's `wpa_supplicant` network selection. It does not use
NetworkManager; this Banana Pi runs netplan, `systemd-networkd` and
`wpa_supplicant`.

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

The normal console/network helpers use the same mechanism rather than changing
`wpa_supplicant` behind the manager's back.

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
