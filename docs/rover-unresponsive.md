# When the rover stops answering

A focused investigation, 2026-08-26, into the rover disappearing from the network
several times an hour and only coming back after a power cycle.

The short version: at least some of these are **not the board crashing**. The
Banana Pi keeps running the whole time. What stops is the data path on the
built-in Wi-Fi radio, while that radio goes on reporting a strong, healthy
association. From a workstation the two are indistinguishable, which is why the
power switch looks like the only cure.

This document is evidence and a possible workaround, not a closed case. One
outage is explained; another one on the same afternoon is not, and other causes
of an unresponsive rover almost certainly remain.

## The board does not crash

`netwatch` samples every ten seconds and fsyncs its transitions to
`/var/lib/netwatch/netwatch.log`, which survives a power cut. Across the two
events examined on 2026-08-26 there is **no gap in that ten-second cadence** up
to the instant power was pulled, and the samples are unremarkable throughout:
load around 6 on four cores, every watched process alive, 3.2–3.4 GB free, CPU at
70 °C against a critical trip at 110 °C, and no throttling. A board that is
scheduling a Python process every ten seconds is not hung.

So `netwatch-report`'s tally of runs that "ended without a shutdown" is counting
**operator power cycles**, not spontaneous resets. That distinction is easy to
get backwards and it inverts the whole diagnosis.

Also ruled out for these events, from the same records: no out-of-memory kill
anywhere in the logs, no kernel panic or oops, no USB over-current, nothing
holding `/dev/watchdog`, and `RuntimeWatchdogUSec=0` so systemd is not resetting
the board either.

## What actually stops

The 12:27 event, from `netwatch`, one line per ten seconds:

```text
12:27:15  sig=-32  rtt=1.9   rx=661  tx=985   load=8.31
12:27:25  sig=-29  rtt=1.9   rx=1    tx=0     load=7.81
12:27:36  sig=-31  rtt=none  rx=1    tx=1     load=7.38
...
12:29:06  sig=-28  rtt=none  rx=0    tx=1     load=6.50   <- power cycled here
```

Traffic falls from roughly 660 kB in and 985 kB out per ten seconds to nothing,
and eleven seconds later the rover cannot get an ICMP reply out of its own
gateway one hop away. Meanwhile the link still claims `wpa=COMPLETED` at −28 dBm,
still holds its address and default route, and every process is still alive. It
stayed that way for ninety seconds until the power went off.

A second event the same afternoon, 12:37 to 12:38, looked identical and then
**recovered on its own after about sixty seconds** — but at 112 ms round-trip
instead of the usual 2–5 ms, which is what carrying the traffic on the other
radio looks like.

## The one mechanism that is proven

`wlan0` is the built-in Broadcom SDIO radio (`brcmfmac`, `phy1`); `wlan1` is the
USB Realtek adapter (`rtl8xxxu`, `phy0`). At the start of the 12:37 event the
driver recorded that the chip stopped answering it:

```text
t=2026-08-26T12:37:06 brcmf_proto_bcdc_query_dcmd: brcmf_proto_bcdc_msg failed w/status -110
t=2026-08-26T12:37:06 brcmf_cfg80211_get_station: GET STA INFO failed, -110
```

`-110` is `ETIMEDOUT`. This is the driver saying it put a control message to the
firmware and got nothing back — a genuine lockup of the chip's control channel,
not an interpretation of one. The same signature appears at four other moments in
four days, including a sustained run on 2026-08-25 at 10:40–10:41 where even
`brcmf_set_multicast_list` timed out.

That explains why the link kept advertising a strong association while passing
nothing: the association state the rest of the system reads is the host driver's
cached copy, and nothing had told it otherwise. Note that `netwatch`'s `sig`
comes from `/proc/net/wireless` rather than from a live query, so a plausible
signal reading during an outage is neither evidence for this nor against it.

## What is not explained

**The 12:27 event — the one actually power-cycled — has no kernel message at
all.** The traffic simply stopped. Generalising the firmware lockup to every
outage is an assumption, and the following are not excluded:

- the access point dropping or ceasing to forward for this client, which from the
  rover looks exactly the same: associated, strong, no traffic;
- anything that leaves the board up and `sshd` unable to serve, which is what
  `netwatch/netprobe.py` exists to observe from outside;
- causes unrelated to Wi-Fi entirely.

The discriminator is cheap, and nothing was recording it: **the two radios are
deliberately kept on different routers.** If `wlan0` goes dead while `wlan1` still
reaches its own gateway, the fault is the rover's radio. If both go quiet
together, it is not.

## Why the system journal is no help here

The root filesystem is mounted `commit=120` and `journald` syncs on a five-minute
interval, so a power cycle discards everything since the last sync. In practice
**every previous boot retains only its first eight seconds or so** — the initial
runtime-to-persistent flush — and nothing from the failure. `journalctl
--list-boots` is misleading for a second reason: the board has no battery-backed
clock, so each boot restores a stale timestamp and only jumps to real time when
NTP catches up, which is why three separate boots can all appear to start at
`12:13:34`.

`netwatch` writes to `/var/lib/netwatch/` and fsyncs precisely because of this.
It is the only record of these events that exists.

The gap this leaves: **`wifi_dual` logs its failover decisions only to the
journal**, so whether it moved the service address during either outage, and how
fast, is not recorded anywhere. Its thresholds say it should call a link dead
after three unanswered pings and its deadman should hand both radios back after
120 s, but neither can be confirmed against these events.

## Getting in without the power switch

The rover holds three addresses, and only one of them depends on the radio that
fails:

```text
wlan0 (built-in Broadcom)  192.168.1.139   + 192.168.1.80  <- the one that vanishes
wlan1 (USB Realtek)        192.168.1.100
```

`192.168.1.80` is the service address `wifi_dual` parks on whichever radio is
carrying traffic — normally `wlan0`. `192.168.1.100` is the USB adapter's own
DHCP lease on a different radio, different chipset and different router, and
nothing moves it.

**So the first thing to try when the rover is unresponsive is
`ssh admin@192.168.1.100`, before reaching for the power switch.** Getting a
shell there while `.80` is dead would confirm the fault is one radio rather than
the board, which is the single measurement this investigation still lacks.

## Resetting the Broadcom radio without rebooting

`brcmfmac` is a loadable module and the chip sits on a bus that can be unbound,
so the radio can be reloaded in place. Reloading makes the driver download
firmware to the chip again, which is what clears a wedged control channel;
`ip link set wlan0 down` is not enough, because that asks the same wedged
firmware to act.

`~/diag/reset-wifi.sh` on the rover does this. **It has not been tested against a
real outage, or at all** — it is staged, not proven.

```bash
cat secrets/bpi-sudo.key | ssh admin@192.168.1.100 '~/diag/reset-wifi.sh'
```

Two things about how it must be run, both of which the script also enforces:

- **over `192.168.1.100`, the other radio.** The reset destroys `wlan0`, and
  `.80` normally lives on `wlan0`, so driving it over `.80` or `.139` severs the
  connection carrying the command halfway through — the same trap as an unguarded
  `pkill` pattern matching the SSH session that typed it. The script re-execs
  itself detached so that a dropped session cannot leave the radio unloaded.
- **the sudo password on stdin**, held in a private file only for the seconds the
  detached child needs it and then removed. Nothing is stored on the rover.

After reloading it restarts `netplan-wpa-wlan0.service`, which does not come back
by itself: it has `Requires=sys-subsystem-net-devices-wlan0.device`, so it is
stopped when the interface disappears and is not retriggered when it returns.

## The recorder left running

`~/diag/blackbox.py` writes one line every two seconds to `~/diag/blackbox.log`,
fsynced, recording for **each radio separately** whether it can reach the router,
plus which interface currently holds `.80`:

```text
13:09:28 up=1794 holds80=wlan0 wlan0[ip=192.168.1.139 sig=-22 gw=2ms arp=REACHABLE rx=4045 tx=132744 up] \
                              wlan1[ip=192.168.1.100 sig=-62 gw=3ms arp=REACHABLE rx=819  tx=170    up]
```

Reachability is a TCP connect to the router rather than a ping, because ICMP here
needs a capability this recorder does not have (`ping_group_range` is closed and
it does not run as root). Signal comes from `/proc/net/wireless` rather than
nl80211, because the thing that fails is exactly the driver's control path — a
recorder that blocks on `GET STA INFO` records nothing.

It is **not started at boot**. After any power cycle, bring it back with
`~/diag/start.sh`.

These files live only in `~/diag/` on the rover. They are diagnostic scratch
rather than a deployed component, so they are one reflash away from gone.

## What would close this

Next time the rover is unresponsive, do not power cycle it. Instead:

1. `ssh admin@192.168.1.100`;
2. read `~/diag/blackbox.log`.

`wlan0[gw=none]` alongside `wlan1[gw=4ms]` proves it is one radio's firmware and
makes the reset script the right fix — and makes the real repair a faster
failover in `wifi_dual` rather than a reboot. Both radios dead at once means the
cause is somewhere else entirely and this document covers only half the problem.
