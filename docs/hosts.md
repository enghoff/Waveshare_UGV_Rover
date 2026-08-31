# Current rover host

This is a local-installation document for the Jetson Orin Nano physically
mounted on this rover. It records the names, addresses and hardware facts needed
to operate and deploy the current system. General deployment instructions are in
[`deploy.md`](deploy.md).

The Orin replaced the Banana Pi M4 Zero on 2026-08-31. The chassis, the driver
board, the lidar and the gimbal camera all came across unchanged; what moved is
the computer they are wired to, and with it every device name. The Banana Pi is
off the rover and off the network, and is no longer a deploy target. Where a
constant in this repository was "measured on the rover", it was measured through
the Banana Pi or the Pi 1 before it -- the mechanics are the same rover, so those
numbers stand, with the one exception recorded under
[What is still missing](#what-is-still-missing).

Voice inference is still Alibaba's hosted realtime Qwen Omni. The Orin has a
usable GPU and nothing on the rover uses it: no local model is deployed, and the
speech path is unchanged.

## Jetson Orin Nano

| | Current installation |
|---|---|
| SSH alias | `orin` |
| hostname | `jetson-orin` |
| user | `jetson` |
| board | NVIDIA Jetson Orin Nano Engineering Reference Developer Kit (Super) |
| CPU | 6x Cortex-A78AE at 1.728 GHz, aarch64 |
| RAM | 7.3 GB, shared with the GPU |
| swap | **none** -- no swap file and no zram unit |
| storage | 915 GB NVMe (`/dev/nvme0n1`), root on p1, about 3% used |
| OS | Ubuntu 24.04.4 LTS, kernel `6.8.12-1021-tegra` |
| L4T/JetPack | R39.2.1 |
| GPU driver | 595.78, reporting CUDA 13.2; no CUDA toolkit and no `nvcc` |
| Python | system CPython 3.12, and unlike the Banana Pi it **has pip** |
| power mode | `nvpmodel` **1 = 25W**; mode 2 `MAXN_SUPER` is available and unused |
| rover address | `192.168.1.88` (onboard radio) and `192.168.1.77` (dongle), both by DHCP |
| mDNS | `jetson-orin.local`, which Windows does resolve here |
| GPIO driver-board UART | `/dev/ttyTHS1` at 115200 |
| lidar serial | `/dev/ttyACM0` at 230400 |

**There is no floating service address.** The Banana Pi kept `192.168.1.80` on
whichever radio was healthy and `wifi_dual` moved it. The Orin has two working
radios since 2026-08-31, but nothing moves an address between them, so each
answers on its own DHCP lease and either will do. Those leases can move -- this
LAN has a second DHCP server on it -- so `jetson-orin.local` is the name to
prefer, and the addresses are the thing to re-check when it stops answering. See
[What is still missing](#what-is-still-missing).

## Hardware links

### Driver board

The Waveshare General Driver board is on the Orin's 40-pin header, which brings
out UART1:

```text
/dev/ttyTHS1  115200 baud
```

That is a third name for the same three pins -- it was `ttyS4` on the Banana Pi
and `ttyAMA0` on the Pi 1 -- so `board_link.py` takes whichever of the three
exists rather than being told which board it is on. Everything else about the
board is unchanged: it streams `T:1001` continuously with wheel encoder counts,
IMU values and battery voltage, the daemon owns the port, and ROS reaches it
through the daemon over loopback TCP 8772 rather than opening it directly.

Measured here on 2026-08-31: 20 Hz, live IMU and magnetometer values, pack at
12.2 V. The board also answers `{"T":126}` sent from the host, so the write
direction is proved rather than assumed.

### Lidar

Unchanged, and still its own USB serial device:

```text
/dev/ttyACM0  230400 baud
```

The CH343 adapter enumerates as `1a86:55d3` and `cdc_acm` owns it; the stable
name is `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79023845-if00`, which is
what `lidar_node.py` finds for itself. Running at 9.9 Hz here with no thin
revolutions.

Note the trap that survived the move: the USB adapter is bus powered, so a serial
node can exist while the rover's main power switch is off. A live port with no
packets still means the lidar motor has no 5 V rover power.

### Camera and face detection

The gimbal camera is the same Xitech UVC module (`0abd:8050`), found by stable
`/dev/v4l/by-id` identity:

```text
/dev/v4l/by-id/usb-Xitech_USB_Camera_20250606105-video-index0
```

Face detection is local YuNet. Two things about it differ from the Banana Pi. The
model file `face_detection_yunet.onnx` is tracked in git now and arrives with a
deploy -- it never used to, and on this host that showed up as a detector
answering every `count_faces` with a missing file. And capture needs `v4l2-ctl`,
which is not in the Ubuntu base image: `apt install v4l-utils`.

OpenCV is still a pinned aarch64 wheel unpacked into `vendor/` rather than a pip
install, and that wheel is `cp37-abi3`, so one file serves 3.12 here and 3.13 on
the old board. Tracking measures 30 fps with 24 ms of detection per frame,
against roughly a third of that on four Cortex-A53 cores.

### OAK-D-Lite

**Not fitted.** No Movidius device is on the bus, so `oak_depth` is not deployed
and the depth service on 8770 is not running. `oak_depth/install.sh` already
picks its depthai wheel by interpreter, so it is ready for 3.12 whenever the
camera is plugged back in.

## Network

Wi-Fi only in practice, and **managed by NetworkManager** -- `nmcli` works here,
which it did not on the Banana Pi. That is the largest single difference from the
old host, and the reason the roaming half of `wifi_roam` is staged but not
installed. Its privileged helper `/usr/local/sbin/wifi_ctl.sh` **is** installed,
along with a `NOPASSWD` rule letting `jetson` run that one path, because the
console's network panel cannot list or switch networks without it. That is what
`wifi_roam/install.sh --helper-only` puts down, and it is what the manifest's
system install for `wifi_roam` runs on this host; nothing roams by itself here.

- `wlP1p1s0` -- onboard Realtek RTL8822CE, `192.168.1.88`, dual band;
- `wlx002e2d3074d0` -- the USB Realtek RTL8188FTV dongle, `192.168.1.77`,
  2.4 GHz only, working since 2026-08-31 -- see [The second radio](#the-second-radio);
- `enP8p1s0` -- wired, **down with no carrier**: a cable or switch-port problem
  rather than configuration, unchanged since 2026-08-30.

Both radios associate at boot and are deliberately held on **different routers**,
the onboard on `TheGreatLord` and the dongle on `TheGreatViking`, so that a
router going down does not take both. Each has its own DHCP lease and either
answers SSH and the daemon on 8769. That is two ways in, not failover: nothing
moves a service address between them, which is what `wifi_dual` did on the Banana
Pi and what still needs porting.

### Which networks this rover can join

Two of the three house networks, not three. NetworkManager holds one profile per
radio, and each is pinned to its radio by `connection.interface-name`, which is
how the two stay on different routers:

| Profile | Network | Pinned to |
|---|---|---|
| `TheGreatLord` | `TheGreatLord` | `wlP1p1s0` |
| `TheGreatViking-dongle` | `TheGreatViking` | `wlx002e2d3074d0` |

**There is no profile for `TheMaharaja`**, so the console lists it as a network
with no passphrase and will not join it. Adding one needs the key in
`secrets/wifi.key` and a decision about which radio it is for.

Note that a profile's name is not its network. `wifi_ctl.sh` reports and joins by
network and looks the profile up, and a join does not name an interface for a
profile that already pins one, because NetworkManager refuses that rather than
resolving it.

The onboard radio also fills its scan list in **over several scans** rather than
all at once: measured on 2026-08-31, three consecutive scans returned 2, then 8,
then 27 access points. One press of the console's "look for networks" can
therefore show almost nothing on a radio that has been quiet for a while; the
list is cumulative and the next press fills it in.

### The second radio

The dongle kept its old name from the Banana Pi days -- the MAC is the same
`00:2e:2d:30:74:d0` that `wifi_roam/20-usb-wlan.link` mentions -- but nothing
renames it here, so it appears under the kernel's own `wlx...` name. Its
NetworkManager profile pins that name, so installing that `.link` file would
rename the interface out from under the profile and the radio would come up
unconfigured.

Two things had to be fixed to make it work at all, and both are traps for any
other device on this board that needs a driver or firmware:

**NVIDIA's L4T kernel is built with `CONFIG_RTL8XXXU` unset**, and with no
`drivers/net/wireless/realtek` directory in `/lib/modules` at all, so the dongle
sat on the USB bus with no driver bound and no interface. The driver it needs is
in-tree and has supported this exact device (`0bda:f179`, RTL8188FU) for
several releases; it simply was not compiled. It is now built out of tree from
unmodified `linux-6.8.12` sources from kernel.org -- the same version this kernel
is based on, and the Makefile in the kernel's own headers tree is byte-identical
to it -- and registered with **DKMS as `rtl8xxxu/6.8.12`**, so a JetPack kernel
update rebuilds it instead of silently dropping it. The kernel does not enforce
module signatures (`CONFIG_MODULE_SIG_FORCE` unset, lockdown off), which is why
an unsigned out-of-tree module loads at all.

**This kernel cannot read Ubuntu's compressed firmware.** `CONFIG_FW_LOADER_COMPRESS`
is unset, while `linux-firmware` on Ubuntu 24.04 ships every blob as `.zst`. The
driver therefore found the device, identified it correctly, and then failed with
`Direct firmware load for rtlwifi/rtl8188fufw.bin failed with error -2` beside a
`/lib/firmware/rtlwifi/rtl8188fufw.bin.zst` that plainly exists. The fix is an
uncompressed copy alongside it:

```bash
sudo zstd -d /lib/firmware/rtlwifi/rtl8188fufw.bin.zst            -o /lib/firmware/rtlwifi/rtl8188fufw.bin
```

Expect the same failure from any other firmware-loading device added to this
board, and read "no such file" as "the file is there, compressed".

`l4tbr0`/`usb0`/`usb1` are the USB-device-mode bridge and stay down; `can0` is
down.

`dnsmasq`, `isc-dhcp-server` and `isc-dhcp-server6` fail on every boot and serve
only that unused bridge. Treat them as expected noise rather than a fault.

## Services and ports

| Port | Binding/owner | Purpose |
|---:|---|---|
| 8769 | rover LAN / `rover_daemon` | hardware/tool protocol |
| 8770 | rover LAN / `oak_depth` | depth service -- **not running, no camera** |
| 8771 | rover LAN / `drive_web` | HTTPS console + audio WebSocket |
| 8772 | loopback / daemon | board telemetry + motor bridge for ROS |
| 8773 | loopback / ROS nav bridge | navigation backend for daemon |
| 8774 | loopback / current voice session | image handoff for `look` |

Started from the `jetson` crontab -- the same `@reboot` arrangement the Banana Pi
used, and for the same reason, that a system unit needs a sudo password no script
here has:

```text
@reboot /home/jetson/ugv/run_daemon.sh --vision --board-bridge --ros-nav
@reboot /home/jetson/ugv/ros_nav/run_ros_nav.sh --nav
@reboot /home/jetson/ugv/drive_web/run_drive_web.sh
```

The console is at:

```text
https://192.168.1.88:8771/
```

Its certificate is a **new** one, from a CA generated on this host and named
`UGV rover console CA (jetson-orin)`. The workstation trusts the Banana Pi's old
CA and not this one, so a browser will warn until the new
`~/.ugv/tls/console-ca.crt` is trusted there.

## ROS 2

ROS 2 Jazzy from RoboStack, installed by `ros_nav/install.sh` with no sudo,
exactly as on the Banana Pi:

```text
~/miniforge3/envs/ros    6.9 GB, 316 packages
```

Ubuntu 24.04 does have native Jazzy packages, and they are deliberately not used:
one install path that works on both boards is worth more than a second one that
works only here.

## Voice/cloud credentials

Unchanged in shape, and still outside `~/ugv`:

```text
~/.ugv/alibaba.key     DashScope API key, mode 600
~/.ugv/console.token   token gating microphone/session creation
~/.ugv/tls/            console certificate/CA
~/.ugv/deploy-state.json
```

`console.token` and the TLS material were generated on this host on first run;
the DashScope key was copied from `secrets/`. These are runtime state and must
never be copied into Git.

## What is still missing

Three things, in the order they matter.

**The chassis calibration.** `~/ugv/odometry.json` holds the gyro's scale and the
wheels' counts per metre, is deliberately gitignored as belonging to the machine
rather than to the repository, and did not come across from the Banana Pi.
Without it `base_node.py` refuses to start rather than guess, so ROS has no
odometry at all: `slam_toolbox` drops every scan for want of a transform and Nav2
cannot drive. Sensing is unaffected -- the lidar, the map and
`describe_surroundings` all work. The cure is `ros_nav/calibrate_chassis.py`,
which measures it by driving the rover, or recovering the old file from the
Banana Pi's disk, which describes the same chassis and is therefore still true.

**Failover between the two radios.** Both radios now work and are associated to
different routers, but that is redundancy a person can use, not redundancy the
rover uses: nothing watches the two paths and nothing moves a service address to
the healthier one. `wifi_dual.py` does exactly that on the Banana Pi by driving
`wpa_cli` against netplan and `systemd-networkd`, and this host runs
NetworkManager, so it needs porting rather than installing. Two interfaces on one
subnet also want a policy rule each, or replies leave by whichever radio the main
routing table prefers -- the fault that once left the rover unreachable at its own
address for eleven minutes. The roamer is staged and its replay test passes; only
the console helper has been installed, and the timer, the units and the
dual-radio manager have deliberately **not** been. The roamer is a one-radio
script that would start moving the two radios this rover deliberately keeps
apart.

The dongle's profile also still carries NetworkManager's default retry limit,
which the roamer's full install is what normally clears. It went off the air at
03:31 on 2026-08-31 after a link timeout and had not been retried six hours
later, so a transient outage currently costs the second radio until somebody
brings it back:

```bash
ssh orin 'nmcli con mod TheGreatViking-dongle connection.autoconnect-retries 0'
ssh orin 'nmcli con up TheGreatViking-dongle'
```

**netwatch.** Staged, not installed, for the same reason: it is ordered after
`wpa_supplicant` and reads that daemon's control socket, and neither exists here.
It would run, and record less than it appears to.

## Access and source of truth

```bash
ssh orin
```

SSH is key-only (`allow-pw: false` from the autoinstall) and the authorised key
is `~/.ssh/id_ed25519` on the Windows workstation -- the default identity. The
`orin` entry resolves `jetson-orin.local` through the same mDNS proxy the
`bpi-m4zero` entry used, because Windows OpenSSH asks unicast DNS for `.local`
and the router answers NXDOMAIN.

`sudo` prompts -- it is not NOPASSWD -- and the password is
`secrets/jetson-orin.key`, fed over stdin:

```bash
ssh orin 'sudo -S -p "" <command>' < secrets/jetson-orin.key
```

Because `sudo` authenticates the `jetson` account's own password through PAM,
that file **is** the login password, so a keyboard and monitor on the board is a
genuine way back in if SSH is ever lost.

Repository source is deployed under `~/ugv/` using
[`deploy/deploy.py`](../deploy/deploy.py). Do not repair tracked code by editing
it in place on the rover: fix the repository and deploy it.

When this document and executable source/config disagree, the executable source
or config is authoritative and this document should be corrected.

## The board it replaced

The Banana Pi M4 Zero ran the rover until 2026-08-31: Armbian on Debian trixie,
CPython 3.13, no pip and no `python3-venv`, netplan with `systemd-networkd` and
`wpa_supplicant`, two radios with `192.168.1.80` floating between them, and the
driver board on `/dev/ttyS4`. Its root filesystem was mounted `commit=120`, which
is why several scripts here still follow a crontab write with `sync`. It is
powered off, and its details are in this file's history rather than above.
