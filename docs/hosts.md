# The two other machines: `rpi` and `media`

Most of this repo runs on the workstation or in the SLAM VM. Two named hosts sit
outside both, and neither is interchangeable with the other: `rpi` is the only
machine physically wired to the rover, and `media` is the only one with a GPU.

|  | `admin@rpi` | `root@media` |
|---|---|---|
| what it is | Raspberry Pi Model B Rev 2, on the rover | Ubuntu 22.04 under WSL2, on a desktop |
| why it exists | it is wired to the ESP32 over the GPIO UART | it has the RTX 3070 |
| CPU / RAM | BCM2835 armv6l, 1 core, 474 MB | Ryzen 7 5700G, 8 threads, 21 GB |
| storage | 115 GB SD card, 4.7 GB used | 1 TB rootfs, 45 GB used |
| OS | Raspbian 13 (trixie), 6.18.34+rpt-rpi-v6 | Ubuntu 22.04.5, 6.18.33.2-microsoft-standard-WSL2 |
| address | `192.168.1.4` (eth0) / `192.168.1.47` (wlan0) | `192.168.1.3`, `MEDIA.local` |
| key | `~/.ssh/id_ed25519_rpi` | `~/.ssh/id_ed25519` (the default one) |

Both are on the same 192.168.1.0/24 home LAN as the rover's ESP32
(`192.168.1.22`). The SLAM VM is *not* — it lives on VMware's NAT segment at
192.168.80.x and reaches nothing here directly.

## `rpi` — the machine on the rover

A Pi 1 Model B, 700 MHz single core, 474 MB of RAM. It is slow enough that this
matters: it exists to hold a serial link and forward gamepad input, not to run
vision. Anything that opens an OpenCV window runs somewhere else — the host is
headless by choice (`multi-user.target`, `lightdm` disabled, cloud-init
disabled, `gpu_mem=16`), which is also where that 474 MB came from. Boot is
~2m07s, most of it NetworkManager and SD-card enumeration.

The rover's General Driver board is wired to the **GPIO UART**, not USB. There is
no `/dev/ttyUSB*` or `/dev/ttyACM*` here; use `/dev/ttyAMA0` at 115200. Freeing
that port meant masking `serial-getty@ttyAMA0` and stripping
`console=serial0,115200` from `/boot/firmware/cmdline.txt` — so there is now no
serial-console rescue path, and on a Pi with no built-in WiFi a bad `cmdline.txt`
means pulling the SD card.

The firmware streams `T:1001` telemetry continuously at ~2.6 kB/s without being
asked. A read loop that extends its deadline whenever bytes arrive never returns;
use a fixed deadline, or read line by line.

**Network.** `eth0` (`b8:27:eb:56:8a:3f`) is primary at route metric 100.
`wlan0` comes from a Realtek RTL8188FTV USB dongle (`0bda:f179`) on SSID
`TheGreatLord` at metric 600. Both land on the same /24, so the WiFi is a
redundant path rather than a second network — which is what makes it worth
having when the rover drives away from its cable. `nmcli` needs `sudo` over SSH:
polkit only grants network control to an active local session.

A rover that has driven off has no `eth0` at all, so `192.168.1.47` is the
address that matters and `ssh rpi` — an mDNS name — is the part that is not
guaranteed to resolve. Measured over that WiFi: 38 Mbit/s, CPU-bound at 72% of
the core rather than limited by the link, and 30 fps of 640x480 MJPEG forwarded
for ~30% of it.

**Peripherals.** A CSR Bluetooth dongle (`0a12:0001`) as `hci0`, with an Xbox
Wireless Controller paired and trusted at `/dev/input/js0`. That pairing needs
L2CAP ERTM disabled, persisted in `/etc/modprobe.d/xbox-controller.conf`. Also
paired and trusted: a **JBL Flip** (`20:18:5B:7C:2E:44`) for output over A2DP and
a **Sony WI-XB400** (`30:53:C1:A4:66:86`) for the microphone over HFP, which
WirePlumber negotiates as mSBC — 16kHz mono, the rate Whisper wants, with no
resampling in between. PipeWire and WirePlumber run as `admin`'s user services;
the node names are what scripts should address, since ids change on reconnect.

The adapter came up `off-blocked` after a reboot: an rfkill soft block that
`systemd-rfkill` restores from `/var/lib/systemd/rfkill/` at every boot. It looks
like broken hardware — `bluetoothctl power on` answers `org.bluez.Error.Failed`
and the log says `Failed to set mode: Failed (0x03)`, which is *hardware
failure*. `rfkill` is not installed and `sudo` wants a password, but a udev ACL
leaves `/dev/rfkill` writable by `admin`, so the block can be cleared by writing
the 8-byte `struct rfkill_event` to it directly.

Two things about this dongle are worth knowing before believing a measurement:

- It floods `Bluetooth: hci0: corrupted SCO packet` whenever the HFP microphone
  is streaming. Audio still arrives intact, and A2DP to the JBL still completes
  alongside it — 2s of audio took 2.85s with the mic shut and 3.51s with it open,
  slower but not stalled. `ACL MTU: 310:10` is only ten buffers, so there is not
  much headroom for both.
- `pw-play --raw` **hangs forever on a file argument** and must be fed on stdin
  instead. This is a `pw-cat` quirk, not a fault of the dongle or the speaker,
  and it imitates a wedged Bluetooth stack convincingly enough to waste an hour.
- `pw-top -b -n 1` reports **all zeros and state `C`**, because it prints deltas
  and the first sample has nothing to difference against. Use `-n 3` and read the
  last block, or conclude that nothing is playing when it is.
- After `bluetoothctl connect` returns *Connection successful*, WirePlumber still
  takes **10-20s** to build the card. A script that checks for the node right
  away decides the device is absent.

**Audio is realtime here since 2026-08-15, and was not before.** `admin` had to
be added to the `pipewire` group for the `rtprio 95` in
`/etc/security/limits.d/25-pw-rlimits.conf` to apply — `usermod -aG pipewire
admin` plus a **reboot**, since group membership is only read at login and
`systemctl --user restart` will not pick it up. `rtkit` was installed and running
the whole time; the rlimit was what it lacked. Verify on the right thread:

```bash
for t in /proc/$(pgrep -u admin -x pipewire | head -1)/task/*; do
    echo "$(cat $t/comm): $(chrt -p $(basename $t) | tr '\n' ' ')"
done   # want data-loop.0 -> SCHED_FIFO, priority 88
```

`ps -eLo cls,rtprio,comm | grep pipewire` does **not** answer this: the realtime
thread is named `data-loop.0`, so that pattern misses it and reports `TS` for the
idle main threads.

Without that, the failure is counter-intuitive and cost hours. A deliberate CPU
hog does *not* break playback (0-2 dropouts) — it competes for throughput, which
the scheduler handles. A process waking 50 times a second does (36 dropouts in
15s, 11.7% of the audio) — it competes for latency, which it does not. So "the Pi
is not busy" says nothing about whether audio will be clean, and pipes feeding
audio should be read in bulk regardless.

Two more traps when measuring any of this: `pw-record --target <sink>` silently
records a *source*, not that sink's monitor — use
`-P "stream.capture.sink=true"` — and xrun counters are meaningless against a
client that holds a stream open while idle, since it underruns every quantum by
design. See [voice_chat/README.md](../voice_chat/README.md).

The dongle, the wifi dongle and the camera all share one weakly fused USB bus,
which is the first thing to suspect if the wifi drops during a run that is also
streaming audio and video.

**Access.** `ssh rpi` (a `~/.ssh/config` alias for `rpi.local`, user `admin`,
key `id_ed25519_rpi`). Key-only since 2026-08-13; `sudo` still prompts for the
account password. The gotcha is the sshd drop-ins: cloud-init owns
`50-cloud-init.conf` and writes `PasswordAuthentication yes` into it, and sshd
takes the *first* value it sees for a keyword — so the key-only settings live in
`00-key-only.conf` to sort ahead of it. A `99-` file silently loses.

**Code.** Repo files are deployed to `~/ugv/`, mirroring their path here; the
repo stays source of truth. Plain `scp` is fine (unlike the VM — these are `.py`
with no shebang, so CRLF does not bite).

## `media` — the GPU host

Not a separate box: an Ubuntu instance under WSL2 on a Windows desktop, which is
why `hostnamectl` reports chassis `container` and a `-microsoft-standard-WSL2`
kernel. It is on the LAN in its own right at `192.168.1.3`, and resolves as
`MEDIA.local` — over link-local IPv6, in practice.

It exists for the RTX 3070 (8 GB, driver 591.86, compute 8.6). Windows has
already taken a share of that card, so a service gets roughly 6.5 GB. Three
model services live in `/opt` and **share the one card**, so they are mutually
exclusive:

| service | port | what it is | source |
|---|---|---|---|
| `voice-chat` | 8767 | Whisper distil-large-v3 + Qwen3-4B int4 + Kokoro-82M | `voice_chat/` in this repo |
| `qwen3-vl` | 8766 | Qwen3-VL-4B-Instruct vision-language | the **mt4** repo |
| `grounding-dino` | 8765 | open-vocabulary detection | the **mt4** repo |

Switch between them with the interlock, which stops whatever is running first:

```bash
ssh root@media ~/switch_service.sh voice     # or: dino, qwen
```

Only `qwen3-vl.service` is enabled at boot, so an instance that has been
restarted comes back on vision, not voice — expect to switch after any reboot of
the Windows host. `voice-chat` is the slow one to start: it loads three models
and then compiles and warms decode before binding, ~150s measured, and `/health`
answering is the signal that it is warm.

A fourth service does **not** share the card and is not part of that trade:

| service | port | what it is | source |
|---|---|---|---|
| `face-detect` | 8768 | YuNet on the **CPU**, ~6.5 ms a frame | `face_detect/` in this repo |

It is on the CPU precisely so it can run alongside whichever of the three owns
the card — the rover must not stop seeing because somebody is talking to it — and
it is enabled at boot for the same reason. Do not add it to
`~/switch_service.sh`. See [face_detect/README.md](../face_detect/README.md).

It is no longer the only one on the rover's control path. It closes the
face-tracking loop for `face_tracking/track_face_pi.py`, and `voice-chat` now
does the same for `voice_chat/talk_pi.py` — the rover's microphone and speaker,
the models here. Both therefore bind `0.0.0.0` and neither is tunnelled.

The two vision services are still reached from whatever machine has a person at
it, over a tunnel:

```bash
ssh -N -L 8766:127.0.0.1:8766 root@media    # qwen3-vl, grounding-dino
```

**The firewall.** WSL's mirrored networking puts a Hyper-V firewall in front of
this host whose `DefaultInboundAction` is `Block`, so binding a port on the LAN
is not enough to be reachable on it — inbound TCP is dropped silently, which
looks like a timeout rather than a refusal. Rules are made in an elevated
PowerShell on the Windows desktop; WSL cannot make one, since interop gets
"Access is denied". Two exist: `OpenPI WSL SSH TCP 22 (LAN only)` and
`MEDIA services TCP 8765-8774 (LAN only)`, both scoped to `LocalSubnet`.

That range is deliberately larger than the four ports in use, so a new service
does not need a new rule. It also means a port in that block is on the LAN as
soon as something binds it, and nothing here authenticates — which is a reason to
keep binding `127.0.0.1` for anything reached from a machine with a person at it.
`face-detect` binds `0.0.0.0` because a rover has to reach it unattended; the
three GPU services stay on loopback and are tunnelled.

Deployment is `scp` into `/opt/<service>/` plus a `systemctl restart`; see
[voice_chat/README.md](../voice_chat/README.md) for the service's own details,
measurements and tuning knobs. `~/switch_service.sh` is deployed here but its
source of truth is `services/switch_service.sh` in the mt4 repo — do not edit it
in place.
