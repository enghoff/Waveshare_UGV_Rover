# Current rover host

This is a local-installation document for the Banana Pi physically mounted on
this rover. It records the names, addresses and hardware facts needed to operate
and deploy the current system. General deployment instructions are in
[`deploy.md`](deploy.md).

There is no separate MEDIA/GPU host in the current system. Voice inference is
provided by Alibaba's hosted realtime Qwen Omni service.

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
