# The two other machines: `rpi` and `media`

**This is a local-setup document.** It describes one particular installation —
its hostnames, addresses, keys and firewall rules — rather than anything general
about the repository. Nothing here is required to run the code; the components
that reach these machines take the host as an argument. Read it as a worked
example of what deploying to a Pi and a GPU box actually involves, and expect
every name and number in it to be different on your own network.

Most of this repo runs on the workstation. Two named hosts sit outside it, and
neither is interchangeable with the other: `rpi` is the only machine physically
wired to the rover, and `media` is the only one with a GPU.

|  | `admin@rpi` | `root@media` |
|---|---|---|
| what it is | Raspberry Pi Model B Rev 2, on the rover | Ubuntu 22.04 under WSL2, on a desktop |
| why it exists | it is wired to the ESP32 over the GPIO UART | it has the RTX 3070 |
| CPU / RAM | BCM2835 armv6l, 1 core, 474 MB | Ryzen 7 5700G, 8 threads, 21 GB |
| storage | 115 GB SD card, 4.7 GB used | 1 TB rootfs, 45 GB used |
| OS | Raspbian 13 (trixie), 6.18.34+rpt-rpi-v6 | Ubuntu 22.04.5, 6.18.33.2-microsoft-standard-WSL2 |
| address | `rpi.local` — `192.168.1.4` (eth0) / `192.168.1.47` (wlan0) | `media.local` — `192.168.1.3` |
| key | `~/.ssh/id_ed25519_rpi` | `~/.ssh/id_ed25519` (the default one) |

Both are on the same 192.168.1.0/24 home LAN as the rover's ESP32
(`192.168.1.22`).

**Address them by name, not by number.** `rpi.local` and `media.local` both
resolve by mDNS from the workstation, from the Pi and from MEDIA, and the name is
the only identifier that is right in every state the rover can be in: the Pi has
two addresses, `eth0` is primary at metric 100 while it is docked, a rover that
has driven off has no `eth0` at all, and `rpi.local` follows whichever interface
is up. Hardcoding `192.168.1.4` cost exactly that bug once — the rover daemon
serving happily on wlan0 while a client reported no rover at all.

Measured, so the cost is known rather than assumed:

| lookup | from the workstation | from the Pi |
|---|---|---|
| `rpi.local` | 147 ms | 16 ms |
| `media.local` | 99 ms | 387 ms |
| a name that does not exist | **7.3 s** | — |

That last row is why code that falls back keeps names *first* and addresses
after: a name that works is cheap, a name that does not is not, and an address
that is not there refuses in milliseconds. The ESP32 is the exception and stays a
number — it advertises no mDNS name.

## `rpi` — the machine on the rover

A Pi 1 Model B, 700 MHz single core, 474 MB of RAM. It is slow enough that this
matters: it exists to hold a serial link and forward gamepad input, not to run
vision. It is not, however, too slow for everything — [`lidar_slam/`](../lidar_slam)
does scan-matched 2D SLAM here in 33.5% of the core, which took writing the inner
loops in C, since the same algorithm under numpy came to 2.3× the sensor's own
10 Hz budget. The chip has scalar VFP and no NEON (`Features: half thumb fastmult
vfp edsp java tls`), so there is no SIMD to recover the difference with, and the
lesson generalises: on this host, reaching for numpy on arrays of a few hundred
elements loses to a plain loop in C by one to two orders of magnitude.

Anything that opens an OpenCV window runs somewhere else — the host is
headless by choice (`multi-user.target`, `lightdm` disabled, cloud-init
disabled, `gpu_mem=16`), which is also where that 474 MB came from. Boot is
~2m07s, most of it NetworkManager and SD-card enumeration.

The rover's General Driver board is wired to the **GPIO UART**, not USB, so its
control link is `/dev/ttyAMA0` at 115200. Freeing that port meant masking
`serial-getty@ttyAMA0` and stripping `console=serial0,115200` from
`/boot/firmware/cmdline.txt` — so there is now no serial-console rescue path, and
on a Pi with no built-in WiFi a bad `cmdline.txt` means pulling the SD card.

**The lidar is on this Pi too, on a second and quite separate port.** The D500's
data leaves the board's Type-C socket marked *LIDAR* through a CH343 USB-UART
(`1a86:55d3`, behind the FE1.1S hub `1a40:0101`), and the `cdc_acm` driver claims
it as **`/dev/ttyACM0`** at 230400. There is genuinely no `/dev/ttyUSB*` here, but
this document claimed for a while that there was no `ttyACM*` either, which was
wrong and sent [`lidar_slam/`](../lidar_slam) looking for the wrong device.
Measured 2026-08-17 straight off the port: 19.6 kB/s, 418 packets/s, 9.94 Hz
rotation, zero CRC failures in 836 packets, ~419 points per revolution.

The firmware streams `T:1001` telemetry continuously at ~2.6 kB/s without being
asked, at a measured **~20 Hz**, and it carries much more than motor state: a 9-DoF
IMU as `ax/ay/az`, `gx/gy/gz` and — unlike the OAK's BMI270 — a magnetometer in
`mx/my/mz`, wheel encoder counts in `odl`/`odr`, and pack volts in `v` (1208 =
12.08 V). Everything is raw LSB rather than SI; `az` reads 8392 for 1 g, and the
gyro's resting `gz` bias measured 6.9. That ~20 Hz is the ceiling on any dead
reckoning done here, and it is set by the firmware, not by the reader.

Measure that rate with a bulk read, not with `readline`. A line-at-a-time loop with
a 0.2 s timeout reported 17 Hz on this stream where draining `in_waiting` reported
19.9 — the missing sixth of the samples was the reader's, not the firmware's. And a
read loop that extends its deadline whenever bytes arrive never returns at all; use
a fixed deadline.

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
L2CAP ERTM disabled, persisted in `/etc/modprobe.d/xbox-controller.conf`. A **JBL Flip** and a **Sony WI-XB400** are also paired and trusted, from a voice
client that ran on this box and was removed on 2026-08-15 — the Pi never held a
reliable conversation, and speech now happens on a desk instead. Two facts from
that work are still true of this machine and worth keeping:

- **PipeWire here is realtime, and was not before.** `admin` had to be added to
  the `pipewire` group for the `rtprio 95` in
  `/etc/security/limits.d/25-pw-rlimits.conf` to apply — `usermod -aG pipewire
  admin` plus a **reboot**, since group membership is only read at login.
  Verify on the right thread; it is called `data-loop.0`, so
  `ps -eLo cls,rtprio,comm | grep pipewire` misses it and reports `TS` for the
  idle main threads:

  ```bash
  for t in /proc/$(pgrep -u admin -x pipewire | head -1)/task/*; do
      echo "$(cat $t/comm): $(chrt -p $(basename $t) | tr '
' ' ')"
  done   # want data-loop.0 -> SCHED_FIFO, priority 88
  ```

- **A process waking 50 times a second breaks audio here; a CPU hog does not.**
  A deliberate spin loop cost 0–2 dropouts in 15s, a 20 ms read loop cost 36
  (11.7% of the audio). Throughput the scheduler handles, latency it does not. So
  "the Pi is not busy" says nothing about whether anything realtime will survive,
  and anything reading a pipe here should read it in bulk.

The dongle, the wifi dongle and the camera all share one weakly fused USB bus,
which is the first thing to suspect if the wifi drops during a run that is also
streaming audio and video.

**What runs here.** `rover_daemon.py` (from `rover_daemon/` in this repo) is
the one process that may own the UART and the camera, and everything that
commands the rover goes through it: headlights, gimbal, face tracking, exposed as
tools on TCP 8769. [`lidar_slam/`](../lidar_slam) is the exception that does not
conflict, because it reads the *lidar* port rather than the UART — but it needs
`ttyAMA0` for its motion prior, so it belongs inside the daemon eventually rather
than beside it. Started with `--vision` it offers one more, `look`, which
POSTs a frame to `voice-chat` on MEDIA so the model can be asked what it sees;
without the flag the tool is not offered at all. `drive_gamepad_pi.py` and
`track_face_pi.py` are still standalone and still take the UART directly, so do
not run them at the same time as the daemon.

**Access.** `ssh rpi` (a `~/.ssh/config` alias for `rpi.local`, user `admin`,
key `id_ed25519_rpi`). Key-only since 2026-08-13; `sudo` still prompts for the
account password. The gotcha is the sshd drop-ins: cloud-init owns
`50-cloud-init.conf` and writes `PasswordAuthentication yes` into it, and sshd
takes the *first* value it sees for a keyword — so the key-only settings live in
`00-key-only.conf` to sort ahead of it. A `99-` file silently loses.

**Code.** Repo files are deployed to `~/ugv/`, mirroring their path here; the
repo stays source of truth. Plain `scp` is fine — these are `.py` with no
shebang, so CRLF does not bite.

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
| `voice-chat` | 8767 | Whisper distil-large-v3 + Qwen3-VL-4B int4 + Kokoro-82M | `voice_chat/` in this repo |
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

Its reply model is now the same `Qwen3-VL-4B-Instruct` weights `qwen3-vl` uses,
so the rover can be asked what it can see; the rover POSTs a frame straight to
8767 when the model asks to look. Which model `voice-chat` loads is two lines in
its unit — `VOICE_LLM_MODEL` and `VOICE_VISION` — and both models are in the HF
cache here, so going back to the text one costs a restart and no download. See
[voice_chat/README.md](../voice_chat/README.md#seeing).

A fourth service does **not** share the card and is not part of that trade:

| service | port | what it is | source |
|---|---|---|---|
| `face-detect` | 8768 | YuNet on the **CPU**, ~6.5 ms a frame | `face_detect/` in this repo |

It is on the CPU precisely so it can run alongside whichever of the three owns
the card — the rover must not stop seeing because somebody is talking to it — and
it is enabled at boot for the same reason. Do not add it to
`~/switch_service.sh`. See [face_detect/README.md](../face_detect/README.md).

It is no longer the only one on the rover's control path. It closes the
face-tracking loop for both `face_tracking/track_face_pi.py` and the same loop
run under `rover_daemon.py`, so it binds `0.0.0.0` and is not tunnelled.
`voice-chat` binds the LAN for the same kind of reason — its client is on a desk,
not on this box — though nothing on the rover talks to it any more.

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
