# Why the OAK is not on the rover

Measured 2026-08-18 with the camera plugged into the Pi as the rover stood. The
conclusion was that this camera does not go on this rover: the Pi cannot run it, and
could not have powered it if it could.

Nothing here is a verdict on the camera or on the tooling for it. It runs from a
workstation exactly as it always has — that is what [`oak_camera/`](../oak_camera) is
for, and what [oak-d-lite.md](oak-d-lite.md) and [oak-usb-link.md](oak-usb-link.md)
describe — and it runs in ROS 2 in [`vm/`](../vm/README.md), which keeps a depthai of
its own in `~/venvs/oak`. What is ruled out is the third thing: the Pi driving it, on
the rover, as part of the daemon.

This is the record of the three findings that decided it. Any of them would have been
enough on its own.

## Waking one is not something you do once

The OAK has no flash to boot from. It enumerates as a Movidius MyriadX in ROM
bootloader state — `03e7:2485` in `lsusb` — and waits for a host to upload firmware
over USB, which is what depthai does every time a pipeline opens. A camera sitting in
that state is idle rather than broken.

**So there is no flash-at-boot-and-walk-away.** A booted device that stops hearing
from its host kills itself on a 1500 ms watchdog, which is what produces the
`errorId=9001` crash dumps in [oak-usb-link.md](oak-usb-link.md). The camera is alive
for exactly as long as some process holds it open, so "bring the OAK up when the Pi
boots" can only mean "run a service that keeps a pipeline open" — never a one-shot
firmware push. Flashing the device's own bootloader persists, but changes only which
firmware it boots into, not whether a host has to be there.

## depthai does not run on this Pi

| | |
|---|---|
| board | Raspberry Pi Model B Rev 2, revision `000e` |
| architecture | `armv6l` |
| OS / interpreter | Raspbian 13 (trixie), CPython **3.13.5** — the only Python trixie packages |

depthai's wheels, read off PyPI on 2026-08-18:

| release | 32-bit ARM wheels |
|---|---|
| 3.9.0 (current) | none at all — x86-64, aarch64, macOS and win-amd64 only |
| 2.32.0.0 (what the VM pins) | `linux_armv6l.linux_armv7l` for **CPython 3.9 and 3.11**, nothing later |
| piwheels | armv6 builds stop at depthai 2.13.3, for CPython 3.7 and 3.9 |

The wheel exists for the architecture and not for the interpreter, and there is no
source build worth attempting: depthai-core is a large C++ project and this is a
700 MHz single core with 474 MB of RAM. The only route would be building CPython 3.9
or 3.11 on the Pi first, a few hours of compiling, and it buys just the first of
these three problems.

## The 5 V rail could not have taken it anyway

This board is a Model B **Rev 2**, where the per-port 140 mA polyfuses of Rev 1 were
replaced with links: USB current is limited only by the input polyfuse that the board
itself also draws through, and `max_usb_current` does not exist on this generation.
Everything shares one budget. Summed from `bMaxPower` at each device:

```
2042 mA declared across 11 devices, all of it off one 5 V rail
```

— the wlan adapter at 500 mA, the face-tracking camera at 500 mA, **the OAK at
500 mA**, the lidar's serial adapter at 138 mA, USB audio at 100 mA, a Bluetooth
dongle at 100 mA, and two hubs. Declared is not drawn, and the OAK takes very little
while it sits unbooted; booting it is what would turn the claim into a load. With it
unbooted, `vcgencmd get_throttled` read `0x0` — no under-voltage, no throttling, and
no headroom to find out with.

The failure to fear was never a camera that fails to open. It is the rail dipping far
enough to drop *another* device off the bus, and the two that matter are the lidar and
the wlan adapter: a rover that loses its lidar mid-move stops navigating, and one that
loses wlan cannot be told to stop.

## And the link and the core were already spoken for

Both of these were measured before any of the above, elsewhere in this repo, and both
bite well before the camera is useful rather than merely awake. Everything on the Pi
shares one 480 Mbps root port that is also the Ethernet adapter, and
[oak-usb-link.md](oak-usb-link.md) has the OAK alone saturating near 40 MB/s with
colour and aligned depth paired. SLAM is about a third of the one core and face
tracking another 30% — already why the daemon parks tracking whenever the wheels turn.

If the OAK is wanted on a rover again, the host for it is the Jetson: aarch64, where
depthai has current wheels, with the bus and the compute to use what comes back.
