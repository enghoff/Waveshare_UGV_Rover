# Netwatch: evidence before the rover disappears

`netwatch` is a recorder, not a network manager. It writes down the rover's link,
board and kernel state while things are still working so a later outage can be
diagnosed from evidence instead of from whatever survived the recovery reboot.

It runs on the Banana Pi as a lightweight system service. A companion
`netprobe.py` runs on any desk/workstation that can observe the rover from the
outside.

```bash
python deploy/deploy.py --only netwatch
python deploy/deploy.py --system --only netwatch

ssh bpi-m4zero 'netwatch-report'
python3 netwatch/netprobe.py --log probe.log
```

Measured on the rover: about 0.03% of one core, roughly 7.5 MB resident and about
2.8 MB of persistent log data per day.

## What the rover records

Every ten seconds the recorder samples facts that fail independently:

- interface association/state;
- IP address and default route;
- gateway reachability and round-trip time;
- SSID/BSSID and signal where the driver provides one;
- USB device count;
- per-core CPU/load/temperature/free memory;
- whether the main rover processes are alive.

Between samples it also records unsolicited `wpa_supplicant` events and selected
kernel messages about Wi-Fi, USB, memory, voltage and watchdog/reset behavior.

A transition is written immediately rather than waiting for the next periodic
flush.

## Distinguishing reboot from failure

The most useful record is often the one that is absent. On SIGTERM the service
writes a `stop` record and syncs it. A clean `reboot`, service stop or shutdown
therefore leaves an explicit ending. A hard reset or power loss cannot.

At the next boot, `prev=` records how the earlier run ended and `lastwords` copies
forward the final sample/event before an unclean ending. That puts the state at
the moment of failure next to the verdict instead of hundreds of lines earlier.

This distinction matters because a rover recovered by a manual power cycle and a
board that reset spontaneously otherwise leave very similar journal evidence.

**What an unclean ending does not mean is that the board failed.** The recorder
cannot see a plug being pulled, so it reports the only thing it knows — that no
`stop` record was written — and `netwatch-report` phrases that as `hard -- no
shutdown recorded -- a recovery power cycle, or a reset`. On this rover the only
way back into a machine that has gone silent on the network is a power cycle, so
almost every one of those endings is **the operator recovering the rover**. It
marks a repair, not a fault.

Read them that way round or the diagnosis inverts: a tally of hard endings
becomes a tally of crashes, the board starts looking like failing hardware, and
attention goes to the end of a run when the thing that needs explaining is in
the samples before it — a healthy, still-scheduling board that had stopped
being reachable. See
[`docs/rover-unresponsive.md`](../docs/rover-unresponsive.md).

## Why it writes under `/var/lib`

Logs live at:

```text
/var/lib/netwatch/netwatch.log
```

They do not live only under `/var/log`: this installation uses volatile/zram
logging there, which can lose exactly the final minutes needed after a reset.

The root filesystem also uses `commit=120`, so important transitions are fsynced
as they occur and steady-state samples are periodically synced. A recorder that
loses the lead-up to a power-cycle is not useful merely because it wrote lots of
lines earlier in the day.

The log is rotated at a bounded size and is readable without sudo so diagnosis
does not add another credential dependency to an already-stranded rover.

## Record types

One record per line, using simple `key=value` fields:

| Kind | Meaning |
|---|---|
| `boot` | recorder started; includes how the previous run ended |
| `lastwords` | last known state before an unclean previous ending |
| `sample` | regular ten-second snapshot |
| `change` | association/address/route/network/USB/process state changed |
| `wpa` | unsolicited supplicant event |
| `kmsg` | selected kernel message |
| `clock` | wall clock jumped after boot; the board has no RTC |
| `stop` | SIGTERM/clean service stop |

Two details are deliberately represented honestly rather than filled with guesses:

- missing signal is empty rather than the driver's `-256` sentinel becoming a
  fabricated very-bad RSSI;
- a USB device disappearing is visible through the device-count change even when
  the higher-level service only reports that it stopped receiving data.

The onboard Wi-Fi radio is SDIO rather than USB, so USB-count changes now mainly
help with the camera, lidar, OAK and USB standby Wi-Fi adapter.

## External probe

The rover cannot observe every failure from inside itself. A process can remain
associated, addressed and able to ping its gateway while SSH or the daemon is no
longer serving correctly.

`netprobe.py` therefore runs off-rover and checks, independently:

1. ICMP reachability;
2. TCP 22 connect;
3. whether SSH actually sends an `SSH-2.0-` banner;
4. whether the rover daemon answers on TCP 8769.

```bash
python3 netwatch/netprobe.py --log probe.log
python3 netwatch/netprobe.py --report probe.log
```

A machine that pings and accepts TCP 22 but never sends the SSH banner is recorded
as `stalled`, which is much more useful than treating an indefinitely hanging SSH
client as merely a slow network.

Run the external probe from whatever desk is observing the rover; there is no
special monitoring host in the current system.

## Reading it alongside the rover's Wi-Fi

Nothing on the rover manages the network any more: it autoconnects one profile
and stays there until a person says otherwise, so there is no manager whose
decisions have to be told apart from the weather. Netwatch remains independent
and passive, which still matters -- the component used as evidence should not
also be the component changing anything.

During an outage, combine:

- `netwatch-report` from the rover;
- the off-rover `netprobe.py` log;
- `wifi_ctl.sh status` and `journalctl -u NetworkManager`.

## Tests

```bash
python3 netwatch/selftest.py
```

The tests use a fake filesystem/proc environment and require no radio. They cover
important failure representations such as missing signal, an interface going
down, a USB device disappearing, an unclean log ending and a normal clean stop.

`install.sh` runs the self-test before replacing the system service.

## Install

Normal source staging:

```bash
python deploy/deploy.py --only netwatch
```

Privileged install after review:

```bash
python deploy/deploy.py --system --only netwatch
```

Manual fallback:

```bash
scp -r netwatch bpi-m4zero:~/ugv/
cat secrets/bpi-sudo.key | ssh bpi-m4zero \
  'sudo -S -p "" sh ~/ugv/netwatch/install.sh'
```

Verify the service itself, not merely the copied files:

```bash
ssh bpi-m4zero 'systemctl is-active netwatch.service'
ssh bpi-m4zero 'netwatch-report | tail -3'
```
