# The two other machines: `bpi` and `media`

**This is a local-setup document.** It describes one particular installation —
its hostnames, addresses, keys and firewall rules — rather than anything general
about the repository. How to push code onto these machines is
[deploy.md](deploy.md). Nothing here is required to run the code; the
components that reach these machines take the host as an argument. Read it as
a worked example of what deploying to the rover's board and a GPU box actually
involves, and expect every name and number in it to be different on your own
network.

Most of this repo runs on the workstation. Two named hosts sit outside it, and
neither is interchangeable with the other: `bpi` is the only machine physically
wired to the rover, and `media` is the only one with a GPU.

|  | `admin@bpi-m4zero` | `root@media` |
|---|---|---|
| what it is | Banana Pi M4 Zero v2, on the rover | Ubuntu 22.04 under WSL2, on a desktop |
| why it exists | it is wired to the ESP32 over the GPIO UART | it has the RTX 3070 |
| CPU / RAM | 4× Cortex-A53 at 1.416 GHz, aarch64 with NEON, 3.9 GB | Ryzen 7 5700G, 8 threads, 21 GB |
| storage | 29 GB card, ext4 mounted `commit=120` | 1 TB rootfs, 45 GB used |
| OS | Armbian, kernel 6.18.44-current-sunxi64, Debian trixie, CPython 3.13.5 | Ubuntu 22.04.5, 6.18.33.2-microsoft-standard-WSL2 |
| address | `192.168.1.80` (service address, either radio); also `bpi-m4zero.local` | `media.local` — `192.168.1.3` |
| key | `~/.ssh/id_ed25519_rpi` | `~/.ssh/id_ed25519` (the default one) |

Both are on the same 192.168.1.0/24 home LAN as the rover's ESP32
(`192.168.1.22`).

**The rover's address is `192.168.1.80`.** That is a service address
`wifi_dual` moves between the two radios with a gratuitous ARP, so an open
ssh session, the console's websocket and the rover's own conversation all
survive a failover. `.139` (onboard, `wlan0`) and `.100` (dongle, `wlan1`)
are the interfaces' own DHCP leases and each still answers. `bpi-m4zero.local`
and `media.local` also resolve by mDNS when it works; Windows OpenSSH asks
unicast DNS for `.local` and the router answers NXDOMAIN, which is why the
`bpi-m4zero` Host entry in `~/.ssh/config` goes through a multicast proxy
(and why `192.168.1.80` is the reliable way to reach this board from a
Windows desk). The ESP32 stays a number — it advertises no mDNS name.

Measured, so the cost is known rather than assumed:

| lookup | from the workstation | from the rover |
|---|---|---|
| `bpi-m4zero.local` | ~150 ms | ~16 ms |
| `media.local` | 99 ms | 387 ms |
| a name that does not exist | **7.3 s** | — |

That last row is why code that falls back keeps names *first* and addresses
after: a name that works is cheap, a name that does not is not, and an address
that is not there refuses in milliseconds.

## `bpi` — the machine on the rover

A BananaPi BPI-M4-Zero v2, Allwinner H618 (`sun50i-h618`), four Cortex-A53 at
1.416 GHz, 3.9 GB, aarch64 with NEON. Measured 2026-08-23. It replaced a
Raspberry Pi 1 Model B; what moved with the rover is the same driver board, the
same camera, the same lidar, and the same daemon on TCP 8769. What changed is
the computer under them.

The rover's General Driver board is wired to the **GPIO UART**, so its control
link is `/dev/ttyS4` (UART4 on the 40-pin header) at 115200. The Pi 1 used
`/dev/ttyAMA0` for the same pins; `board_link.py` accepts either so one default
works on both.

**The lidar is on this board too, on a second and quite separate port.** The
D500's data leaves the board's Type-C socket marked *LIDAR* through a CH343
USB-UART (`1a86:55d3`, behind the FE1.1S hub `1a40:0101`), and the `cdc_acm`
driver claims it as **`/dev/ttyACM0`** at 230400. There is genuinely no
`/dev/ttyUSB*` here. Measured 2026-08-17 straight off the port: 19.6 kB/s, 418
packets/s, 9.94 Hz rotation, zero CRC failures in 836 packets, ~419 points per
revolution.

The firmware streams `T:1001` telemetry continuously at ~2.6 kB/s without being
asked, at a measured **~20 Hz**, and it carries much more than motor state: a
9-DoF IMU as `ax/ay/az`, `gx/gy/gz` and — unlike the OAK's BMI270 — a
magnetometer in `mx/my/mz`, wheel encoder counts in `odl`/`odr`, and pack volts
in `v` (1208 = 12.08 V). Everything is raw LSB rather than SI; `az` reads 8392
for 1 g, and the gyro's resting `gz` bias measured 6.9. That ~20 Hz is the
ceiling on any dead reckoning done here, and it is set by the firmware, not by
the reader.

Measure that rate with a bulk read, not with `readline`. A line-at-a-time loop
with a 0.2 s timeout reported 17 Hz on this stream where draining `in_waiting`
reported 19.9 — the missing sixth of the samples was the reader's, not the
firmware's. And a read loop that extends its deadline whenever bytes arrive
never returns at all; use a fixed deadline.

**Network.** Wifi only: this form factor has no Ethernet. Two radios, both
associated: onboard Broadcom BCM4345/6 as `wlan0`, USB Realtek dongle as
`wlan1`. `nmcli` is not on this board — it runs netplan and `wpa_supplicant`
— and scanning or switching still needs root. Six SSIDs are saved, not one:
`TheGreatLord`, `TheMaharaja` and `TheGreatViking` are three separate routers
bridged onto that same /24, each with a 2.4 GHz name and a 5 GHz twin, so the
rover keeps a `192.168.1.x` address whichever it lands on. TheGreatViking's
router *is* the gateway at `192.168.1.1`; the other two answer on the LAN at
`.2` and `.232`. [`wifi_roam/wifi_dual.py`](../wifi_roam/wifi_dual.py) is
what chooses which radio carries traffic; see
[wifi_roam/README.md](../wifi_roam/README.md).

**There is no acceleration to offload to.** No NPU; the Mali-G31 is `disabled`
in this board's device tree and `card0` is only the display engine
(`sun4i-drm`), so there is no OpenCL or Vulkan; and the video engine (`cedrus`,
`/dev/video0`) decodes MPEG-2, H.264, HEVC and VP8 but **not** JPEG. NEON is
what makes the CPU fast enough to run YuNet, and OpenCV already uses it.

**No `pip`, no `python3-venv`, and `secrets/rpi-sudo.key` is not this board's
password.** `admin` is a full sudoer but every `sudo` prompts, and the only
passwordless entry is `/usr/local/sbin/wifi_ctl.sh`. Anything needing a Python
package is therefore a pinned wheel unpacked into a `vendor/` directory —
`install_opencv.sh` and `oak_depth/install.sh` — and anything needing root is
fed `secrets/bpi-sudo.key` over stdin. The Pi's password is silently refused
here, which reads as a board that has lost its password rather than as the
wrong file.

**It goes off the network and has to be power-cycled, and `commit=120` means
recent writes go with it.** Seventeen boots in the working day of 2026-08-23, each
ending with no shutdown in the journal, were read here as spontaneous resets under
load. That was the wrong conclusion from the right evidence: most of them were a
person power-cycling a rover that had stopped answering, and that leaves exactly
the same trace as a board falling over. So the certain fault is the *network* one
— it disappears from the LAN and a restart is what brings it back — and whether it
also resets on its own is an open question rather than a finding. Twice it left
sshd accepting TCP and never sending a banner while ping still answered, which is
the one symptom that is definitely not the network's fault.

The practical consequences hold either way: a `crontab` change needs a `sync`
behind it or the next restart undoes it — that happened once here — and a long
remote command may simply stop mid-sentence. Temperature under four-core load is
48–55 °C, no throttling, clock stays at 1416 MHz, so heat is not it.

[netwatch/](../netwatch) was installed on 2026-08-23 to settle the question. It
writes a line every ten seconds to `/var/lib/netwatch/` — on the card, not in the
zram ramlog that `/var/log` is here — and one line on the way down when it is
asked to stop. A boot with no `stop` record before it is a board that went down
without being asked; a boot with one is somebody rebooting it. `ssh bpi-m4zero
netwatch-report` reads it back. The 5 V rail everything on the USB tree shares is
still the thing to suspect if the resets turn out to be real.

**Its network is netplan, `systemd-networkd` and `wpa_supplicant` — not
NetworkManager.** There is no `nmcli` on this board at all. The house SSIDs
live in `/etc/netplan/30-wifi.yaml`, from which netplan renders one
`wpa_supplicant` per interface. The single-radio keeper in
[wifi_roam/](../wifi_roam) was written against NetworkManager and, once
ported, still stays off here: since 2026-08-25 both radios are up at once and
[`wifi_dual.py`](../wifi_roam/wifi_dual.py) owns them. The roamer's model is
"notice the link has failed, spend an association finding another"; the
manager's is the opposite, so letting both move the link is a fight over the
rover's only way in. `install-dual.sh` disables `wifi-roam.timer` when it
arms the manager.

**Two radios, as of 2026-08-25.** The onboard Broadcom BCM4345/6 (AP6256,
`brcmfmac`, SDIO, dual-band) is `wlan0`. The Realtek RTL8188FTV (`0bda:f179`)
USB dongle that was retired on 2026-08-24 is back as `wlan1`, pinned by
[`wifi_roam/20-usb-wlan.link`](../wifi_roam/20-usb-wlan.link) on `ID_BUS=usb`
rather than a MAC, because the dongle is the part most likely to be replaced.
The onboard radio is the better one (dual-band, 31 dBm against the dongle's
2.4 GHz 0 dBm) and gets first pick; the dongle is the spare, where its
failing costs nothing. Names, not BSSIDs, keep the two radios on different
boxes — `TheGreatLord` and `TheGreatLord 5G` are one router, and a radio on
each would look like redundancy and provide none.

Three things on the host make the onboard radio keep the name `wlan0`, and
none of them live in this repo:

- `/etc/systemd/network/10-onboard-wlan.link` pins the radio's MAC
  (`ac:6a:a3:41:53:53`) to `wlan0`. That is why netplan's existing `wlan0`
  stanza, `netwatch`, the daemon's `wifi_status` and `wifi_ctl.sh` all kept
  working without a line of change.
- `dhcp-identifier: mac` in `/etc/netplan/30-wifi.yaml`. Left out,
  `systemd-networkd` derives the DHCP client id from the *interface name*, so
  renaming the radio to `wlan0` asks the router for a fresh lease and moves
  that interface's address for no visible reason.
- `/etc/modprobe.d/blacklist-onboard-wifi.conf`, now deleted, used to
  blacklist `brcmfmac` outright. It is why the board looked as though it had
  no radio of its own.

Power saving is off. The udev rule that sets it (`/sbin/iw dev wlan0 set
power_save off`) is harmless either way, since the driver's default here is
already off, but it logs a failure whenever it fires at an interface that is
being renamed underneath it — and that log line is the first red herring
anybody reading this journal will find.

**What runs here.** Four `@reboot` crontab entries for `admin`, plus two
systemd units that need root:

- `rover_daemon.py` on TCP 8769 — the one process that may own the UART and
  the camera. Headlights, gimbal, face tracking, and (with `--ros-nav`) the
  driving tools. Started as
  `@reboot ~/ugv/run_daemon.sh --vision --board-bridge --ros-nav`.
- `oak_depth/depth_server.py` on TCP 8770 — the OAK kept awake as a depth
  camera, from a crontab entry of its own.
- `drive_web.py` on TCP 8771 — the browser console, idle until a tab is
  open; the rover also holds its own conversation with Alibaba from here.
- `ros_nav/` — slam_toolbox and Nav2. Takes the lidar's USB port; borrows
  the UART's encoders, gyro and motor commands from the daemon over loopback
  8772, and lends navigation back over 8773.
- `wifi-dual.service` — both radios. `wifi-roam.timer` is installed and
  stays off.
- `netwatch` — records the link to `/var/lib/netwatch/`.

Started with `--vision` the daemon offers `look`, which POSTs a frame to
whichever host is holding the conversation. `drive_gamepad_pi.py` and
`track_face_pi.py` are still standalone and still take the UART directly, so
do not run them at the same time as the daemon. How to push code here is
[deploy.md](deploy.md).

**Access.** `ssh bpi-m4zero` (a `~/.ssh/config` alias, user `admin`, key
`id_ed25519_rpi`, HostName `192.168.1.80`). Key-only; `sudo` still prompts
for the account password. The filename of the key is leftover from the Pi 1;
the secret itself is what both boards accept.

**Code.** Repo files are deployed to `~/ugv/`; the daemon lands flat, the
rest mostly mirror their path here. The repo stays source of truth. See
[deploy.md](deploy.md).

## The Raspberry Pi it replaced

A Pi 1 Model B, 700 MHz single core, 474 MB, reached as `rpi.local` /
`192.168.1.4` on `eth0`. `ssh rpi` and `secrets/rpi-sudo.key` still mean that
board, if it is on the LAN. Nothing in this repository is deployed there now.
Measurements taken on it — scan-match cost, JPEG decode, the OAK as an
inference stick — stay in the documents that recorded them, named as the Pi 1,
because they are why the C matcher, the YuNet-on-host path and
[`oak_depth/`](../oak_depth) look the way they do.

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
face-tracking loop when a caller asks for `--service`, so it binds `0.0.0.0`
and is not tunnelled. The rover's own loop now runs YuNet in-process; this
service is the fallback for a host that cannot. `voice-chat` binds the LAN for
the same kind of reason — its client is on a desk, not on this box — though
nothing on the rover talks to it any more.

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
