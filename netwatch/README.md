# `netwatch/` — why the rover fell off the network, written down before it does

A rover that has to be power-cycled to come back has already destroyed the
evidence for why. This is the recorder that writes the evidence down first: a
line every ten seconds of what the link and the board were doing, every word the
supplicant and the kernel said in between, and one record on the way down that is
the only thing separating *somebody rebooted it* from *it fell over*.

```bash
scp -r netwatch bpi-m4zero:~/ugv/
cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" ~/ugv/netwatch/install.sh'
ssh bpi-m4zero 'netwatch-report'                 # boots, outages, and what caused them
python3 netwatch/netprobe.py --log probe.log   # ...and the same thing from a desk
```

Measured on the rover over its first half hour: 0.03% of one of the four cores,
7.5 MB resident, and 2.8 MB of card a day. It holds
no device the rover needs, and cannot move the link it is watching. That last
part is deliberate: this is an instrument, and an instrument that reaches for the
radio is one more suspect rather than one fewer.

## The three questions, and why they need different evidence

**Was the board up?** A `boot` record whose `prev=` says `hard` means the
previous run ended without a shutdown — a reset or a power cut, not a `reboot`.
This works because a `reboot`, a `systemctl stop` and a shutdown all deliver
SIGTERM, and the service answers one by writing `kind=stop` and calling `sync`;
a board that dies mid-sentence writes nothing, and the absence is the signal. The
line before the gap is copied forward into the next boot as `lastwords`, so the
load, temperature and link state at the moment of death sit next to the verdict
rather than a thousand lines above it.

**Was the radio on the network?** Association, an address, a default route and a
gateway that answers are four separate facts, and the interesting failures are
the ones where three of them hold. Associated with no address is DHCP; an address
with no ping is a link carrying nothing. All four are sampled, and the report
names which one failed rather than saying "down".

**What did the driver say?** wpa_supplicant announces every disconnection with a
reason code exactly once, as an unsolicited message, and never mentions it again.
So this attaches to the supplicant's control socket the way `wpa_cli` does and
copies those messages into the same file as the samples, alongside filtered
kernel messages about USB, the wifi driver, memory and voltage. The cause and the
context end up in one file, in order, which is the difference between a diagnosis
and a correlation exercise.

## What it does not know, and the second half that does

The rover cannot observe the failure that matters most from a desk: a board that
is up, associated, addressed, pinging its own gateway and still not answering
ssh. Twice now this one has left `sshd` accepting TCP and never sending a banner
while ping kept working.

`netprobe.py` is that second half and runs anywhere except the rover — the
workstation or MEDIA — asking four questions every five seconds: does it answer
ICMP, does TCP 22 connect, does `sshd` actually say `SSH-2.0-`, and is the daemon
listening on 8769. The verdict is one word, and `stalled` is the one worth
having: ping answers, the connection is accepted, nothing is ever said. A desk
sees that as ssh hanging, which is indistinguishable from a slow network until
something writes down which of the four failed.

```bash
python3 netwatch/netprobe.py --log probe.log      # leave it running
python3 netwatch/netprobe.py --report probe.log   # what it saw
```

Read the two logs together. The rover's own log ends when the rover does, so the
desk's log is what dates the outage, and the rover's is what explains it.

## Where it writes, and why not `/var/log`

`/var/lib/netwatch/netwatch.log`, on the SD card, rotated at 16 MB and keeping
two.

**Not `/var/log`, which on this board is a zram ramlog that is synced to disk on
a schedule** — so the last minutes before a hard reset, the only minutes worth
having, are exactly what it loses. That is also why `sync` matters here at all:
the root filesystem is mounted `commit=120`, so an ordinary write can sit in RAM
for two minutes waiting for a reset to take it. Every transition and every kernel
or supplicant event is `fsync`ed as it lands, and the steady-state samples are
synced every 30 seconds, which bounds what a reset can erase at half a minute of
"nothing was happening" rather than two minutes of the run-up.

The log is world-readable and so is its directory, on purpose: diagnosing a rover
that keeps disappearing should not also need a password typed into it.

## Reading the log by hand

One record a line, `key=value`, spaces never inside a value:

```
t=2026-08-23T15:05:44 kind=boot boot=3e6e58d6 up=2756.4 prev=unknown kernel=6.18.44-current-sunxi64 wpactrl=1
t=2026-08-23T15:05:44 kind=sample up=2756.6 state=up ip=192.168.1.47 gw=192.168.1.1 ssid=TheGreatLord
    bssid=b0:19:21:b9:4e:fe freq=2427 wpa=COMPLETED sig=-33 qual=70 misc=1112 beacon=0 load=0.34
    cpu=5,5,5,0 mhz=1416 temp=49.1 memfree=3517 usbn=27 rxkb=0 txkb=6 rxdropped=34 daemon=1 oak=1 web=1 rtt=2
```

| kind | when |
|---|---|
| `boot` | the service started; `prev=` is how the last run ended |
| `lastwords` | the final line before a `hard` ending, copied forward |
| `sample` | the ten-second heartbeat |
| `change` | a sample where association, address, route, network, USB count or one of the rover's processes changed — written and synced immediately |
| `wpa` | an unsolicited supplicant event, verbatim |
| `kmsg` | a kernel line matching USB, wifi, memory, voltage or watchdog |
| `clock` | wall time jumped — this board has no RTC, so every timestamp before the first of these is fake-hwclock's guess |
| `stop` | SIGTERM: somebody asked it to stop |

Two fields repay knowing. `usbn` is how many devices are on the USB tree, and it
is the one number that separates a dongle that has fallen off the bus from a
dongle that merely cannot associate — they look identical everywhere else. And
`sig` is empty rather than `-256` when the driver has no reading: that sentinel
lives permanently in this dongle's noise column and turns up in the level column
too, and averaging it into a report would manufacture outages that never
happened.

## Checking it without a rover

```bash
python3 netwatch/selftest.py      # 72 assertions, anywhere, no radio needed
```

Every file it reads is faked into a temporary directory, so the failures are made
to order: an interface that is not up, a dongle that has left the bus, a driver
reporting a signal it does not have, a log that ends mid-sentence. The two that
matter most are there because getting them wrong would quietly turn this into an
instrument that lies — a `-256` read as a signal invents an outage, and a log
whose last line is an ordinary sample read as a clean shutdown would report that
the rover had been politely switched off every time it fell over.

`install.sh` runs the self-test before it installs anything, so a file that
arrived with CRLF line endings or arrived half written fails on the way in rather
than at three in the morning on a rover that has stopped answering.

## What this is for

Two open questions about this board, both of which have been argued from
inference for want of a record.

**Whether it resets on its own.** [docs/hosts.md](../docs/hosts.md) recorded
seventeen boots in one working day with no shutdown in the journal and read them
as spontaneous. Most were a person power-cycling a rover that had gone quiet —
which is the same evidence and the opposite conclusion, and exactly the ambiguity
`prev=` and `stop` exist to remove. From here on, a reset that nobody asked for
is a `hard` with `lastwords` attached, and a reboot is not.

**Whether the wifi keeper is working.** It is not running on this board at all:
[wifi_roam/](../wifi_roam) drives NetworkManager, and the Banana Pi has netplan,
systemd-networkd and wpa_supplicant instead, so `install.sh` there installs the
helper and skips the timer. The rover therefore has whatever roaming
wpa_supplicant does by itself, which between three *different* SSIDs is: pick the
best one at association time, and then never reconsider. This records what that
actually costs before anything is changed to fix it.
