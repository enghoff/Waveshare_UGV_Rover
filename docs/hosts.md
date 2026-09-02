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
| storage | 915 GB NVMe (`/dev/nvme0n1`), root on p1, about 7% used |
| OS | Ubuntu 24.04.4 LTS, kernel `6.8.12-1021-tegra` |
| L4T/JetPack | R39.2.1 |
| GPU driver | 595.78, reporting CUDA 13.2 |
| CUDA / cuDNN / TensorRT | **installed 2026-09-01**: `nvidia-jetpack` 7.2.1, CUDA 13.2.86 with `nvcc`, cuDNN 9.20, TensorRT 10.16.2.10 with its Python bindings |
| Python | system CPython 3.12, and unlike the Banana Pi it **has pip** |
| power mode | `nvpmodel` **1 = 25W**; mode 2 `MAXN_SUPER` is available and unused |
| rover address | **`192.168.1.80`**, the service address, held by whichever radio is healthy |
| mDNS | `jetson-orin.local`, which Windows does resolve here |
| GPIO driver-board UART | `/dev/ttyTHS1` at 115200 |
| lidar serial | `/dev/ttyACM0` at 230400 |

**The GPU is reached two different ways and neither of them is ONNX Runtime.**
The language model goes through Vulkan; perception goes through TensorRT. No
build of ONNX Runtime exists for JetPack 7 -- the community Jetson wheel index
stops at JetPack 6, and the official aarch64 wheel on PyPI carries kernels for
every architecture except this Orin's own sm_87, so it opens a session on the
GPU and then dies at the first launch with "no kernel image is available".
Installing more CUDA does not fix that; the gap is inside the wheel.
`nvidia-jetpack` is 9.4 GB and installs cleanly from the repository already
configured here, with nothing removed and no kernel touched.

**Use `192.168.1.80`.** It is the address that stays true: it is written into
each of the rover's three network profiles as a fixed address, so it is there
whichever house network the rover is on. The radio also has a DHCP lease of its
own -- `192.168.1.88` at the time of writing -- which is useful and does move,
because this LAN has a second DHCP server answering alongside the router.
`jetson-orin.local` is the name to use when the service address is not available.

NetworkManager checks `.80` is free before claiming it and logs a conflict rather
than starting an address war.

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

Fitted and awake since 2026-08-31, serving stereo depth on loopback 8770. It is
on the USB2 bus as `03e7:2485` when idle and `03e7:f63b` once a host has booted
it, because the Myriad X has no flash and is handed its firmware over USB on
every open -- so **the firmware version is the depthai version**, pinned at
2.32.0.0 by `oak_depth/install.sh`, which picked the CPython 3.12 wheel here
without being told to. Intel's udev rule was already in
`/etc/udev/rules.d/97-myriad-usbboot.rules` from the installation, and `jetson`
is in group `users`, so libusb can open the camera.

Measured here with the rest of the rover running: OAK-D-LITE at USB `HIGH`,
320x240 depth, 43-48% of pixels valid, 73.0 degrees across a 7.5 cm baseline, no
device errors. It costs **1.5% of one core and 157 MB**, against 13% of a core on
the Banana Pi. The gimbal camera and the driver board are unaffected beside it.

That was taken at 10 fps, which was the default until later the same day. **The
service now runs at 2 fps**, because nothing reads `/depth` yet and a parked rover
has nothing new to look at; `oak_depth/depth_server.py` explains the reasoning and
`--fps` raises it again. At 2 fps it costs 0.5% of a core, and the board's
`VDD_IN` rail reads 6.32 W against 6.49 W at 10 -- a saving of about 180 mW, which
is small and near the noise. The gimbal camera and the driver board are unaffected
either way.

`~/ugv/oak_depth/run_oak_depth.sh` from the `jetson` crontab is what keeps it
open, and its being alive is the whole of the camera being awake: a booted
device that stops hearing from its host watchdogs itself after 1500 ms. See
[`oak_depth/README.md`](../oak_depth/README.md).

## Network

Wi-Fi only in practice, and **managed by NetworkManager** -- `nmcli` works here,
which it did not on the Banana Pi. Nothing else manages it: the rover joins one
network at boot because one profile is set to autoconnect, and it moves only when
somebody at the console tells it to. There is no roamer and no failover manager;
both used to run here and both were removed on 2026-08-31, along with the second
radio's part in carrying traffic. See
[`wifi_roam/README.md`](../wifi_roam/README.md).

The privileged helper `/usr/local/sbin/wifi_ctl.sh` is installed, with a
`NOPASSWD` rule letting `jetson` run that one path, because the console's network
panel cannot list or switch networks without it. That, the three profiles and the
mDNS responder are the whole of what `wifi_roam/install.sh` puts on this host.

- `wlP1p1s0` -- onboard Realtek RTL8822CE, dual band, **the rover's link**. It
  holds `192.168.1.80` and a DHCP lease of its own;
- `wlx002e2d3074d0` -- the USB Realtek RTL8188FTV dongle, 2.4 GHz only,
  **carrying nothing**. Its driver is loaded and the interface exists;
  NetworkManager is told to leave it alone by
  `/etc/NetworkManager/conf.d/99-unmanaged-usb-wifi.conf`. See
  [The second radio](#the-second-radio);
- `enP8p1s0` -- wired, **down with no carrier**: a cable or switch-port problem
  rather than configuration, unchanged since 2026-08-30.

`192.168.1.80` is a fixed address written into each of the three profiles,
alongside the DHCP lease NetworkManager also takes. It does not move between
radios any more -- there is only one radio in play -- and it is on all three
profiles because the house networks are bridged onto one LAN, so the rover
answers there whichever of them it is on.

```bash
ssh orin '/usr/local/sbin/wifi_ctl.sh status'           # every radio, one line each
ssh orin 'journalctl -u NetworkManager -n 20'           # and what it did about it
```

`status` needs no sudo and reads the kernel rather than NetworkManager, so it
answers on a rover where NetworkManager is itself the problem. The dongle appears
in it with no network beside it, which is the honest report rather than a fault.

Measured here on 2026-08-31, by rebooting the rover twice and choosing a network
by hand in between. It came up on `TheGreatViking` by itself both times, once
from a boot where the last thing anybody had asked for was `TheGreatLord` -- a
hand-picked network does not survive a reboot, and is not meant to. A join from
the console cost **about six seconds** of the rover being unreachable, after
which it answered at `192.168.1.80` again on the new network. The dongle came up
`unmanaged` with no address.

**The link is a good deal weaker than it was.** `TheGreatViking` is 2.4 GHz and
reads **-65 to -66 dBm** from where the rover stands, against the -37 to -40 dBm
the onboard radio saw on `TheGreatLord`'s 5 GHz. That is the price of choosing
one network and staying on it, and it is worth knowing before blaming a slow
camera stream on something else.

`TheGreatViking 5G` is the same router's 5 GHz radio under a name of its own. It
read -88 dBm from where the rover stood that day, which was not worth having; from
where it stands on 2026-09-02 it is the strongest Viking on the air at 42 %, and
the 2.4 GHz `TheGreatViking` is not heard at all. It has had a profile of its own
since then, joined by hand like the others.

### Which networks this rover can join

Every house network, on the onboard radio -- the three house SSIDs and the 5 GHz
name the Viking router advertises alongside its own. NetworkManager holds one
profile per network, named after it, every one pinned to that radio, with
`autoconnect-retries 0` and the service address:

| Profile | Autoconnect | Pinned to | Address |
|---|---|---|---|
| `TheGreatViking` | **yes** | `wlP1p1s0` | DHCP + `192.168.1.80` |
| `TheGreatViking 5G` | no | `wlP1p1s0` | DHCP + `192.168.1.80` |
| `TheGreatLord` | no | `wlP1p1s0` | DHCP + `192.168.1.80` |
| `TheMaharaja` | no | `wlP1p1s0` | DHCP + `192.168.1.80` |

**Only `TheGreatViking` comes up by itself**, which is why a reboot always puts
the rover back on it whatever was chosen before. The others are joined only
from the console, and that join costs the link: one radio means taking the
current network down to bring another up, so the browser reconnects a few seconds
later.

All four profiles carry the one passphrase in `~/.ugv/wifi.key` on the rover,
outside the deploy tree. The three house SSIDs are known to share it. Whether
`TheGreatViking 5G` takes the same key is untested, because the only way to find
out is a join, and a join that fails leaves the rover on nothing -- the profile
that autoconnects is the 2.4 GHz Viking, which is not audible where the rover
now stands. `wifi_roam/install-profiles.sh` owns this arrangement
and is re-run by every `--system` deploy of `wifi_roam`; it writes a missing
profile as a keyfile rather than using `nmcli con add`, which would put the
passphrase in `ps` for every account on the machine to read.

Two earlier habits are worth knowing because both look reasonable and neither is
what is wanted now. Profiles were pinned to one radio each, so neither radio
could take the other's network; then they were unpinned entirely, so either could
take any -- which is right when a spare radio is meant to carry traffic and wrong
now, because an unpinned profile is one NetworkManager may bring up on the
dongle. And the dongle's profile was called `TheGreatViking-dongle`, which is why
`wifi_ctl.sh` matches profiles by the network each is for and never by name: a
console comparing names against SSIDs called the rover's own second network "no
passphrase" and refused to join it.

The onboard radio fills its scan list in **over several scans** rather than all
at once: measured on 2026-08-31, three consecutive scans returned 2, then 8, then
27 access points. One press of the console's "look for networks" can therefore
show almost nothing on a radio that has been quiet for a while; the list is
cumulative and the next press fills it in.

### The second radio

**It carries nothing, and that is deliberate.** The rover uses its onboard
radio; NetworkManager is told to leave the dongle alone, matched by driver rather
than by interface name. The hardware stays present and working so it can be
picked up again -- `wifi_ctl.sh join <ssid> <iface>` will use it, and so will
whatever wants a second radio next -- rather than because anything currently
depends on it.

It appears under the kernel's own MAC-derived `wlx...` name, `wlx002e2d3074d0`.
That name used to be load-bearing, back when a profile pinned it; no profile
names it now, so a `.link` file renaming it would be a question of taste rather
than a trap.

Its driver is built on this host rather than shipped with the kernel, and is a
deploy component of its own -- see [`dongle_driver/README.md`](../dongle_driver/README.md),
which has the whole story. Two facts from it are general to this board rather
than to this radio, and will catch the next device that needs either:

**NVIDIA's L4T kernel leaves out drivers you would expect to be there.**
`CONFIG_RTL8XXXU` is unset and there is no `drivers/net/wireless/realtek`
directory in `/lib/modules` at all, so the dongle sat on the USB bus with no
driver bound and no interface. Nothing was broken; the module simply was not
compiled. It is built out of tree and registered with DKMS so a JetPack update
rebuilds it. The kernel does not enforce module signatures
(`CONFIG_MODULE_SIG_FORCE` unset, lockdown off), which is why an unsigned
out-of-tree module loads at all.

**This kernel cannot read Ubuntu's compressed firmware.** `CONFIG_FW_LOADER_COMPRESS`
is unset, while `linux-firmware` on Ubuntu 24.04 ships every blob as `.zst`. The
driver therefore found the device, identified it correctly, and then failed with
`Direct firmware load for rtlwifi/rtl8188fufw.bin failed with error -2` beside a
`.zst` that plainly exists. Read "no such file" as "the file is there,
compressed"; `dongle_driver/install.sh` unpacks this one, and the next device
will need the same done for its own blob.

`l4tbr0`/`usb0`/`usb1` are the USB-device-mode bridge and stay down; `can0` is
down.

`dnsmasq`, `isc-dhcp-server` and `isc-dhcp-server6` fail on every boot and serve
only that unused bridge. Treat them as expected noise rather than a fault.

## Services and ports

| Port | Binding/owner | Purpose |
|---:|---|---|
| 8769 | rover LAN / `rover_daemon` | hardware/tool protocol |
| 8770 | loopback / `oak_depth` | stereo depth from the OAK-D-Lite |
| 8771 | rover LAN / `drive_web` | HTTPS console + audio WebSocket |
| 8772 | loopback / daemon | board telemetry + motor bridge for ROS |
| 8773 | loopback / ROS nav bridge | navigation backend for daemon |
| 8774 | loopback / current voice session | image handoff for `look` |
| 8776 | loopback / `world_state` perception sidecar | segmentation and embedding models |

Started from the `jetson` crontab -- the same `@reboot` arrangement the Banana Pi
used, and for the same reason, that a system unit needs a sudo password no script
here has:

```text
@reboot /home/jetson/ugv/run_daemon.sh --vision --board-bridge --ros-nav
@reboot /home/jetson/ugv/drive_web/run_drive_web.sh
@reboot /home/jetson/ugv/oak_depth/run_oak_depth.sh
@reboot /home/jetson/ugv/ros_nav/run_ros_nav.sh --nav rtabmap:=off
```

**That `rtabmap:=off` is leftover and should go.** It was added while RTAB-Map was
being tried as a mapper on 2026-08-31 and was not taken out when RTAB-Map was
removed a few hours later. `nav.launch.py` no longer declares that argument, so it
is inert rather than harmful, and the stack is running normally with it. Re-running
`ros_nav/install-boot.sh --nav` rewrites the entry without it; the crontab is
machine state that no deploy touches.

The console is at:

```text
https://192.168.1.80:8771/
```

**That address is the one that stays true**, and the certificate happens to be
valid for it. `192.168.1.80` is written into all three of the rover's network
profiles as a fixed address, so a browser pointed at it keeps working whichever
house network the rover is on and whatever DHCP does with the lease beside it.

`https://jetson-orin.local:8771/` is the fallback and also validates. It follows
whichever radio is up, because `avahi-daemon` publishes the name on every
interface and the console listens on `0.0.0.0`, but not instantly: mDNS answers
are cached for a couple of minutes, so a browser can sit on a dead address until
that expires. The service address has no such lag.

A raw per-radio address -- `192.168.1.88` and the like -- works but gets a
certificate warning every time, and stops working when that lease moves.

The certificate is signed by a CA generated on this host and named `UGV rover
console CA (jetson-orin)`, and the workstation does trust it -- it is in both
`Cert:\CurrentUser\Root` and `Cert:\LocalMachine\Root`, alongside the Banana
Pi's older one. So both URLs above give a clean padlock. (`curl` on Windows still
calls it broken with "the revocation status is unknown", which is schannel's
complaint about a private CA rather than anything wrong with the certificate.)

Its subject-alternative names are `jetson-orin`, `jetson-orin.local`,
`localhost`, `127.0.0.1`, `192.168.55.1` and `192.168.1.80`. That last one was
put there for the Banana Pi and looked like a leftover for a day; now that this
rover holds the same service address it is the most useful name in the
certificate. Measured 2026-08-31: connecting by name or to `.80` validates,
connecting to a per-radio lease fails with an IP address mismatch.

## ROS 2

ROS 2 Jazzy from RoboStack, installed by `ros_nav/install.sh` with no sudo,
exactly as on the Banana Pi:

```text
~/miniforge3/envs/ros    6.9 GB, 882 packages
```

Ubuntu 24.04 does have native Jazzy packages, and they are deliberately not used:
one install path that works on both boards is worth more than a second one that
works only here.

That rule was broken once, on 2026-08-31, and then repaired. RTAB-Map was tried
as a second mapper and RoboStack packages none, so it came from Ubuntu's ROS
packages into `/opt/ros/jazzy` — 302 MB and 232 packages beside the conda
environment, with a wrapper script to keep the two off one process's library
path. RTAB-Map turned out to map this rover worse than slam_toolbox
([`../ros_nav/README.md`](../ros_nav/README.md) has the measurements), so all of
it was purged and `/opt/ros` no longer exists. **Nothing on this board runs ROS
from anywhere but the conda environment.**

## Voice/cloud credentials

Unchanged in shape, and still outside `~/ugv`:

```text
~/.ugv/alibaba.key     DashScope API key, mode 600
~/.ugv/wifi.key        the one passphrase all three house networks share, mode 600
~/.ugv/console.token   token gating microphone/session creation
~/.ugv/tls/            console certificate/CA
~/.ugv/deploy-state.json
```

`console.token` and the TLS material were generated on this host on first run;
the DashScope key was copied from `secrets/`. These are runtime state and must
never be copied into Git.

## What is still missing

One thing. Failover between the two radios used to be on this list; it was
built, it worked, and it was then removed on 2026-08-31 in favour of one radio on
one network -- see [the Network section](#network) and
[`wifi_roam/README.md`](../wifi_roam/README.md). The chassis calibration is done
too, below.

**The chassis calibration is done.** `~/ugv/odometry.json` holds the gyro's scale
and the wheels' counts per metre, and is deliberately gitignored as belonging to
the machine rather than to the repository. It did not come across from the Banana
Pi and was measured here on 2026-08-31 by `ros_nav/calibrate_chassis.py` driving
the rover: 15.310723 gyro LSB per degree per second and 107.206 ticks per metre,
over 28 measured turns and 10 measured drives. Until it existed `base_node.py`
refused to start rather than guess, so ROS had no odometry, `slam_toolbox`
dropped every scan for want of a transform, and Nav2 could not drive; all of that
now works. Losing the file puts the rover straight back there, which is why it is
worth knowing it is the one piece of rover state no deploy can restore.

**The dongle used to go deaf, and the rover now repairs it by itself.**
Measured on 2026-08-31: it would associate and get a lease, then lose the link
and hear nothing at all -- `iw scan` succeeding and returning zero access points
while the onboard radio heard twenty-seven, with the device on the bus, the
driver bound and the interface up. Six minutes on `TheGreatViking` in the
morning and then silence for six hours; after a driver reload, 29 seconds on
`TheMaharaja` before the same thing. Only reloading the module ever brought it
back.

Three things were done about it, and it is worth being clear which is proved.

**A driver patch**, which is the suspected root cause but is not proved to be
this one. The stock driver frees a receive buffer for good whenever one
completes with a USB error and never makes another, so a device that produces
the occasional transient error eventually has none left and receives nothing
while still looking associated and up -- exactly the symptom. That code is
unchanged in v6.19 today. See
[`dongle_driver/README.md`](../dongle_driver/README.md). It ships with counters,
so `rx_urb_retired` climbing towards thirty-two is the theory being confirmed on
this rover in its own time.

**A keeper**, which was proved and is now gone. `dongle-keeper.timer` checked
the dongle every minute and reloaded its driver after three consecutive checks
with no association; demonstrated twice on the hardware, recovering the radio
within 29 seconds in one case and two and a half minutes in the other. It was
removed on 2026-08-31 with the rest of the second radio's networking: nothing
associates the dongle any more, so there is no association for a keeper to miss.

**A setting**, `autoconnect-retries 0`, where NM's default of four had a profile
given up on hours before anybody looked. That one is still in place and applies
to the radio the rover actually uses.

What could not be done is a reproduction on demand. After the profiles were
unpinned the dongle held `TheGreatViking` -- the network it had died on twice --
for **22 minutes with the patch's recovery deliberately switched off**, which is
to say behaving exactly like the stock driver. So the patch is a real fix for a
real bug that matches the symptom, and it is not yet known to be the fix for
this one.

None of it costs the rover anything now. A deaf dongle is a spare part that is
not working, not a link that is down: the onboard radio carries the traffic and
holds the service address, and it is a different driver on a different bus.

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
