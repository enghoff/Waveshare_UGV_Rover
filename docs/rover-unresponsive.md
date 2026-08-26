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

## Recovery is a power cycle, so the record is full of repairs, not faults

There is no remote way back into a rover that has gone silent. Nothing on the
board notices and rebuilds the network path by itself, no watchdog is armed
(`/dev/watchdog` is unheld, `bootstatus` is clean and `RuntimeWatchdogUSec=0`),
and the one alternative below — reaching the radio that is still working —
only helps when a radio still is. **Somebody walks over and cuts the power.**
That is the recovery procedure for an unresponsive rover, and until something
else is shown to work on a real outage it is the only one.

Every record downstream of that has to be read accordingly. `netwatch-report`
marks any run that ended without a SIGTERM as `hard -- no shutdown recorded --
a recovery power cycle, or a reset`, because nothing on the board can see a plug
being pulled and the two leave the same evidence behind. In this installation
those endings are overwhelmingly **the operator recovering the rover**: they
mark the moment a fault was cleared, not the moment one began. That note used to
read "the board went down unasked", which is how this got taken backwards in the
first place.

Reading them as crashes inverts the whole diagnosis. It turns the repair into
the symptom, makes the board look like failing hardware, and draws attention to
the end of the log when the thing that needs explaining — whatever made the
rover go quiet while it was still running — is earlier, in the samples before
the ending. Those samples describe a healthy machine: associated, memory free,
well under thermal limits, every watched process alive, sitting there
unreachable until the power went off.

So: the board dying is not a root cause here. It is what recovery looks like in
the log.

## The board does not crash

`netwatch` samples every ten seconds and fsyncs its transitions to
`/var/lib/netwatch/netwatch.log`, which survives a power cut. Across the two
events examined on 2026-08-26 there is **no gap in that ten-second cadence** up
to the instant power was pulled, and the samples are unremarkable throughout:
load around 6 on four cores, every watched process alive, 3.2–3.4 GB free, CPU at
70 °C against a critical trip at 110 °C, and no throttling. A board that is
scheduling a Python process every ten seconds is not hung.

This is the evidence behind the section above: the runs `netwatch-report` tallies
as ending without a shutdown are operator power cycles, and the board was still
scheduling work normally right up to the moment the power went off.

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

## A second proven mechanism: the failover that left the address behind

Observed 2026-08-26, 16:50:31, for 11m33s, and it is the mechanism above with a
second fault stacked on top of it.

`wlan0`'s data path died in the way this document already describes: it went on
reporting `wpa=COMPLETED` on `TheGreatLord` at −44 to −48 dBm, holding `.139`
and its default route, while passing **zero bytes and answering no gateway ping
for eleven and a half minutes**. `wifi_dual` handled that correctly and quickly
— it had already moved traffic, and the service address with it, onto `wlan1` at
16:50:25, six seconds before the data path stopped.

The failover still did not move the service. `wifi_dual` gives each radio's own
DHCP address a routing rule pinning that address's traffic to that radio, exactly
so a reply leaves by the radio it arrived on. It never adds the same rule for the
service address. The main table holds one connected route per radio at equal
metric and resolves the tie towards `wlan0`, so with `.80` sitting on `wlan1` the
kernel still said:

```text
$ ip route get 192.168.1.206 from 192.168.1.80
192.168.1.206 from 192.168.1.80 dev wlan0
```

Requests to `.80` arrived correctly on `wlan1`; the replies left through the dead
radio. `wlan1`'s own address went on serving SSH normally throughout, so the
rover was reachable the whole time by anyone who knew to ask for it that way.

## Nothing was recovering during those eleven minutes

Worth being explicit about, because the duration invites the opposite reading.
No part of the system was working on this outage.

- `wifi_dual` had done its job as it understands it at 16:50:25 and had nothing
  left to do. Its health test is a gateway ping bound to each radio with
  `SO_BINDTODEVICE`, deliberately per-radio — and `wlan1` was answering, so the
  deadman never came near firing. **Nothing anywhere tests the path the service
  address actually uses**, which is the one that was broken.
- `wlan0` believed it was connected. A radio that claims `COMPLETED` at −45 dBm
  is not something anything on the rover currently kicks, however long it has
  been passing nothing.
- `netwatch` did record the whole window, because it watches `wlan0` alone
  (`NETWATCH_IFACE`) — but it is a recorder and takes no action.

What ended it came from outside. At 17:02:01 the access point answered the rover
with `CTRL-EVENT-DISCONNECTED ... reason=6`, telling it that as far as the AP was
concerned it had not been associated for some time. `wpa_supplicant` re-associated
immediately, `rtt` was 1.8 ms again four seconds later, and `wifi_dual` moved
traffic back to `wlan0` at 17:02:09 — which put `.80` back on the radio the main
table already prefers, and ended the outage as a side effect. Eleven and a half
minutes is simply how long the access point took to say something.

## What to put in place

Two mechanisms, and the first is much the more important:

1. **Pin the service address to whichever radio holds it**, in `route_through()`
   in [`wifi_roam/wifi_dual.py`](../wifi_roam/wifi_dual.py) — the same rule the
   radios' own addresses already get. With it, this outage lasts the six seconds
   of the failover instead of eleven minutes, and the dead radio stops mattering.
   Not yet written, reproduced or deployed.
2. **Kick a radio that claims association and passes nothing.** The evidence is
   already in hand: `wifi_dual`'s bound ping for `wlan0` failed continuously from
   16:50:31, and nothing is attached to that beyond declining to route through
   it. Reassociating the supplicant is the cheap first move; reloading the driver
   is the heavier one, and `~/diag/reset-wifi.sh` exists for it and has still
   never been tested against a real outage.

A health check on the service address itself would catch both this class and the
next one, since it is the only thing that tests what callers actually use.

3. **Stop holding a radio on a router that will not have it.** Watched live on
   2026-08-26: the spare sat unassociated for minutes with two routers at full
   strength in the console list, because holding a radio on one network disables
   every other one and the one it was held on kept dropping it. That matters
   here and not only in the Wi-Fi component -- a spare that cannot associate is
   a rover with nothing to fail over to, which turns the first fault above into
   an outage rather than a six-second blip. Done: `wifi_dual` now frees a radio
   that has been held somewhere for ninety seconds without joining it.

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
wlan0 (built-in Broadcom)  192.168.1.139
wlan1 (USB Realtek)        192.168.1.47
whichever carries traffic  192.168.1.80    <- the one that vanishes
```

`192.168.1.80` is the service address `wifi_dual` parks on whichever radio is
carrying traffic, and that is not always `wlan0`. The other two are the radios'
own DHCP leases, on different chipsets associated to different routers. A lease
is not a promise: `wlan1` answered on `.100` when this was first written and on
`.47` on 2026-08-26. Remember the pair, not the numbers, and read the current
ones with `ip -4 -o addr` whenever a shell is available.

**So the first thing to try when the rover is unresponsive is a shell on a
radio's own address, before reaching for the power switch.** Getting in there
while `.80` is dead shows the fault is one radio, or one route, rather than the
board. On 2026-08-26 that worked: `.47` served SSH throughout an eleven-minute
outage of `.80`.

Multicast DNS is the other way in, and it is more robust than any single address
because it is answered per interface: a query arriving on a radio is answered on
that radio, carrying that radio's own lease. `ssh bpi-m4zero` therefore follows
whichever radio is alive without anyone having to know which one that is. It
resolved to `.139` and to `.47` on the same afternoon.

## Resetting the Broadcom radio without rebooting

`brcmfmac` is a loadable module and the chip sits on a bus that can be unbound,
so the radio can be reloaded in place. Reloading makes the driver download
firmware to the chip again, which is what clears a wedged control channel;
`ip link set wlan0 down` is not enough, because that asks the same wedged
firmware to act.

`~/diag/reset-wifi.sh` on the rover does this. **It has not been tested against a
real outage, or at all** — it is staged, not proven.

```bash
cat secrets/bpi-sudo.key | ssh admin@192.168.1.47 '~/diag/reset-wifi.sh'   # wlan1's lease
```

Two things about how it must be run, both of which the script also enforces:

- **over `wlan1`'s own lease, the other radio** (`.47` as of 2026-08-26; check
  it rather than trusting the number). The reset destroys `wlan0`, and `.80` is
  usually on `wlan0`, so driving it over `.80` or `.139` severs the connection
  carrying the command halfway through — the same trap as an unguarded
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

1. `ssh bpi-m4zero`, or the USB radio's own lease directly;
2. read `~/diag/blackbox.log`.

`wlan0[gw=none]` alongside `wlan1[gw=4ms]` proves it is one radio's firmware and
makes the reset script the right fix — and makes the real repair a faster
failover in `wifi_dual` rather than a reboot. Both radios dead at once means the
cause is somewhere else entirely and this document covers only half the problem.

That measurement now exists, and it came out as this document predicted. During
the 2026-08-26 outage `wlan0` sat at −45 dBm claiming `COMPLETED` and moved no
bytes for eleven and a half minutes while `wlan1` served SSH normally throughout.
A silent rover is not a dead board, and the fault is one radio.

What the same event added is that failing over off the dead radio is not by
itself enough, because the service address does not follow — see the failover
section above. The remaining unknown is narrower than it was: whether the dead
data path is the firmware control channel wedging, as the `-110` timeouts show
elsewhere, or the access point silently dropping the client, which is what
`reason=6` at the end of this one suggests. Neither was in evidence at 16:50:31;
the traffic simply stopped.
