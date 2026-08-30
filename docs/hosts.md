# Current rover host

This is a local-installation document for the Banana Pi physically mounted on
this rover. It records the names, addresses and hardware facts needed to operate
and deploy the current system. General deployment instructions are in
[`deploy.md`](deploy.md).

No GPU host serves the rover: voice inference is provided by Alibaba's hosted
realtime Qwen Omni service, and nothing the rover runs depends on a local GPU.
A Jetson Orin Nano does exist on the same network as a bench machine and is
recorded at the end of this document, but it is not a deploy target and no rover
service calls it.

## Banana Pi M4 Zero

| | Current installation |
|---|---|
| SSH alias | `bpi-m4zero` |
| user | `admin` |
| board | Banana Pi M4 Zero v2, Allwinner H618 |
| CPU | 4× Cortex-A53 at 1.416 GHz, aarch64 with NEON |
| RAM | about 3.9 GB |
| OS | Armbian / Debian trixie |
| Python | system CPython 3.13 |
| stable rover address | `192.168.1.80` |
| mDNS | `bpi-m4zero.local` where the client resolver supports it |
| GPIO driver-board UART | `/dev/ttyS4` at 115200 |
| lidar serial | `/dev/ttyACM0` at 230400 |

The rover service address is **`192.168.1.80`**. `wifi_dual` moves that address
between the two associated Wi-Fi interfaces and sends a gratuitous ARP on
failover, so callers do not need to know which radio is active. The interfaces
also retain their own DHCP leases (`.139` for onboard `wlan0` and `.47` for the
USB `wlan1` on 2026-08-26), but those are interface addresses, not the address
applications should normally bookmark, and they change when a lease does. They
are worth knowing anyway: when the rover stops answering on `192.168.1.80` a
radio's own address, or the name over multicast DNS, is the way in that does not
need the power switch. See
[`rover-unresponsive.md`](rover-unresponsive.md).

On Windows, `.local` resolution is not dependable with every SSH setup. The
`bpi-m4zero` SSH config entry should therefore target the stable service address
or a resolver that genuinely performs mDNS.

## Hardware links

### Driver board

The Waveshare General Driver board is connected to the Banana Pi's GPIO UART:

```text
/dev/ttyS4  115200 baud
```

The board streams `T:1001` telemetry continuously. That stream carries wheel
encoder counts, IMU values and battery voltage as well as motor state. The daemon
owns this serial port. ROS does not open it directly; the daemon lends the values
and motor-command path over loopback TCP 8772.

Direct tools such as `driver_board/drive_gamepad_pi.py` and
`face_tracking/track_face_pi.py` can also open the UART, but must not be run at the
same time as the daemon.

### Lidar

The D500/LD19 data path is a separate USB serial device:

```text
/dev/ttyACM0  230400 baud
```

The CH343 adapter enumerates as `1a86:55d3`; `cdc_acm` owns it. A serial node can
exist while the rover's main power switch is off because the USB adapter is bus
powered; a live port with no packets can therefore mean the lidar motor itself
has no 5 V rover power.

The ROS lidar node owns this port during normal operation and reuses the C parser
in `lidar_slam/`.

### Camera and face detection

The gimbal camera is a UVC device discovered by stable `/dev/v4l/by-id` identity
where possible rather than by assuming `/dev/video0`. On this Allwinner board,
`/dev/video0` may be the SoC video decoder rather than the USB camera.

Face detection is **local YuNet** in `face_tracking/yunet.py`. OpenCV is a pinned
aarch64 wheel unpacked into `vendor/` by `face_tracking/install_opencv.sh`; it is
not installed with pip. The detector deliberately uses three OpenCV threads so
one core remains available to the rest of the rover.

### OAK-D-Lite

The OAK is owned by `oak_depth` during normal operation and is used as a stereo
depth camera. Stop that service before running another OAK program. DepthAI is
also supplied as a pinned unpacked wheel rather than a system/pip install.

## Network

This Banana Pi is Wi-Fi only. Current interface roles:

- `wlan0`: onboard Broadcom BCM4345/6, dual-band, preferred radio;
- `wlan1`: USB Realtek RTL8188FTV, standby/redundant radio.

The host uses netplan, `systemd-networkd` and `wpa_supplicant`, not
NetworkManager. [`wifi_roam/wifi_dual.py`](../wifi_roam/wifi_dual.py) owns active
radio selection. The older `wifi-roam.timer` is installed but must remain disabled
while the dual-radio manager is active.

The two radios are kept associated to different physical access points where
possible. `192.168.1.80` is moved to the better healthy path rather than waiting
for the active path to fail before beginning a new association.

[`netwatch/`](../netwatch) records link/board evidence persistently under
`/var/lib/netwatch/`. This matters because `/var/log` is backed by volatile/zram
logging on this installation and evidence would otherwise disappear across the
power-cycle used to recover a lost rover. A power cycle is the only recovery
known to work on a rover that has gone silent, so the unclean endings in that
record are repairs rather than faults; see
[`rover-unresponsive.md`](rover-unresponsive.md) before reading them as crashes.

## Services and ports

| Port | Binding/owner | Purpose |
|---:|---|---|
| 8769 | rover LAN / `rover_daemon` | hardware/tool protocol |
| 8770 | rover LAN / `oak_depth` | depth service |
| 8771 | rover LAN / `drive_web` | HTTPS console + audio WebSocket |
| 8772 | loopback / daemon | board telemetry + motor bridge for ROS |
| 8773 | loopback / ROS nav bridge | navigation backend for daemon |
| 8774 | loopback / current voice session | image handoff for `look` |

The current `admin` startup set comprises the daemon, OAK depth service,
`drive_web` and ROS navigation, with privileged systemd services for dual Wi-Fi
and netwatch. The daemon supervisor arguments are:

```text
--vision --board-bridge --ros-nav
```

The console is normally reached at:

```text
https://192.168.1.80:8771/
```

## Voice/cloud credentials

The current speech path runs from the browser to the rover and from the rover to
Alibaba DashScope. No model weights or GPU service are deployed locally for it.

Runtime credentials live outside `~/ugv`:

```text
~/.ugv/alibaba.key     DashScope API key
~/.ugv/console.token   token gating microphone/session creation
~/.ugv/tls/            console certificate/CA
```

`~/.ugv/alibaba.key` should be mode 600. These files are runtime state and must
never be copied into Git.

## Installation constraints

This Debian installation has no `pip` or `python3-venv`. Python packages needed
on the rover are therefore either Debian-provided or pinned wheels unpacked into
component `vendor/` directories. In particular:

- `face_tracking/install_opencv.sh` installs the OpenCV wheel for local YuNet;
- `oak_depth/install.sh` installs the pinned DepthAI wheel.

`admin` can use sudo but it prompts. Deployment uses `secrets/bpi-sudo.key` from
the workstation and feeds it over stdin. Do not substitute the old Raspberry Pi
password; the two are different.

The root filesystem is mounted with `commit=120`. Follow important system/crontab
changes with `sync`, because an abrupt power loss can otherwise discard recent
metadata.

## Access and source of truth

```bash
ssh bpi-m4zero
```

Repository source is deployed under `~/ugv/` using
[`deploy/deploy.py`](../deploy/deploy.py). Do not repair tracked code by editing
it in place on the rover: fix the repository and deploy it.

When this document and executable source/config disagree, the executable source
or config is authoritative and this document should be corrected.

## Jetson Orin Nano (bench GPU host, not a rover service)

This board is on the same LAN but is **not** part of the running rover: it is not
a deploy target, `deploy/manifest.json` knows only the `bpi` host, and no rover
service talks to it. It is recorded here because it is the only machine on this
network with a usable GPU, and because its login model has no password fallback.

| | Current installation |
|---|---|
| hostname | `jetson-orin` |
| user | `jetson` |
| board | NVIDIA Jetson Orin Nano Engineering Reference Developer Kit (Super) |
| CPU | 6× Cortex-A78AE at 1.728 GHz, aarch64 |
| RAM | 7.5 GB, shared with the GPU |
| swap | **none** — no swap file and no zram unit |
| storage | 915 GB NVMe (`/dev/nvme0n1`), root on p1, about 1% used |
| OS | Ubuntu 24.04.4 LTS, kernel `6.8.12-tegra` |
| L4T/JetPack | R39.2.1 (`/etc/nv_tegra_release`), fully updated 2026-08-30 |
| boot firmware | 39.2.1 in QSPI, slot A, both slots `normal` (`nvbootctrl`) |
| GPU driver | 595.78, reporting CUDA 13.2 |
| Python | system CPython 3.12 |
| power mode | `nvpmodel` **1 = 25W**; mode 2 `MAXN_SUPER` is available and unused |
| time zone | `Etc/UTC`, clock synchronised |

The CUDA *runtime* is present through the L4T packages, but there is **no CUDA
toolkit**: `/usr/local/cuda` does not exist, there is no `nvcc`, and PyTorch is
not installed. Nothing on this box can currently compile or run GPU code beyond
what the driver itself provides. Docker is running and
`nvidia-container-toolkit` is installed, so the container route to CUDA is the
one that is closest to working.

### Network

Both radios are up and the board answers on two addresses:

- `enP8p1s0` — wired, `192.168.1.86`, MAC `74:25:54:da:e3:13`, netplan connection
  `netplan-lan`;
- `wlP1p1s0` — Wi-Fi, `192.168.1.88`, MAC `f0:68:e3:a8:b4:87`, associated to SSID
  `TheGreatLord` on 5 GHz.

Both are DHCP, so either address can move. As of 2026-08-30 the wired interface is
**down with no carrier** — a cable or switch-port problem, not configuration — so
Wi-Fi is currently the only way in. Note that `192.168.1.86` may still answer ping
while the Jetson is unreachable, because the lease moves or an ARP entry goes
stale; a successful ping to that address is not evidence the board is up. Unlike the rover, this host **is** managed by NetworkManager (`nmcli`
works here), and there is no service address that floats between the interfaces.
`l4tbr0`/`usb0`/`usb1` are the USB-device-mode bridge and stay down; `can0` is
down.

### Access

SSH is **key-only** (`allow-pw: false` from the autoinstall). The authorised key
is `~/.ssh/id_ed25519` on the Windows workstation — the default identity, so no
`-i` is needed:

```bash
ssh jetson@192.168.1.86
```

There is no `jetson-orin` entry in the workstation's `~/.ssh/config`; the name
resolves over mDNS on this network but the address is the dependable form.

`sudo` prompts — it is not NOPASSWD — and the password is
`secrets/jetson-orin.key`, fed over stdin the same way the rover's is:

```bash
cat secrets/jetson-orin.key | ssh jetson@192.168.1.86 'sudo -S -p "" <command>'
```

As on the rover, `-S` reads until EOF, so one `cat` feeds exactly one `sudo`.

Because `sudo` authenticates the invoking user's own password through PAM, that
file **is** the `jetson` account password. A keyboard and monitor on the board is
therefore a genuine way back in if SSH is ever lost. This was not true of the
as-installed system, where the password was an unknown random hash and losing the
SSH key meant reflashing.

### Known-failing services

`dnsmasq`, `isc-dhcp-server` and `isc-dhcp-server6` fail on every boot and have
done since before the 2026-08-30 update. They serve the USB device-mode bridge
`l4tbr0`, which nothing here uses; `dnsmasq` cannot bind port 53 because another
resolver already holds it. Treat them as expected noise rather than a fault.

`nvpmodel.service` is a different case and is now fixed. The 39.2.1 update
repointed `/etc/nvpmodel.conf` at the Super board config and left no saved-mode
file, so at boot the service tried to apply default mode 1, found that the change
needed a reboot, and prompted for confirmation it could never receive under
systemd. It died leaving **no power mode set at all**. Committing the mode
(`echo YES | nvpmodel -m 1`, which reboots) wrote `/var/lib/nvpmodel/status` and
the service has run clean since. If `nvpmodel -q` ever reports "power mode is not
set", this is the cause and the cure.
