# The OAK camera's USB link

Why every script in [`oak_camera/`](../oak_camera) pins USB2, what the link can
carry, and how to recover a camera that has stopped being found.

## Always pin `maxUsbSpeed=HIGH`

This camera negotiates `HIGH` — USB 2.0, 480 Mbps — not `SUPER`. Most USB-C cables
sold for charging carry only the USB2 pairs, so the cable is the usual cause; the
VPU is the same either way. A USB3 cable into a USB3 port would lift the rates
below and raise available bus current from 500 mA to 900 mA.

**Every script in `oak_camera/` except `probe_device.py` passes
`maxUsbSpeed=dai.UsbSpeed.HIGH`. Do not remove it while the link is USB2.**
Without it, opening the device fails most of the time with

```
RuntimeError: Device already closed or disconnected: io error
```

depthai asks for USB3 by default and so uploads the USB3-enabled firmware, which
on this link often fails to come back up on the bus after boot. The host waits ~9 s
in `Searching for booted device` and gives up, and the device — booted, but never
given a host keepalive — kills itself on its 1500 ms watchdog and leaves a crash
dump with `errorId=9001`. That looks like a hardware fault and is not one.

Measured 2026-08-11 across two alternating-arm trials on the same 960×540
pipeline, plus the ad-hoc runs of the scripts here:

| | Opened successfully |
|---|---|
| default (USB3 firmware, `SUPER_PLUS`) | 5 of 13 |
| `maxUsbSpeed=HIGH` | 13 of 13 |

Every `HIGH` run that opened also delivered all 60 frames at 25–27 fps. The
failure is intermittent — the default arm strung three successes together
mid-trial — so testing a cable or port change needs a handful of runs, not one.
`probe_device.py` is the deliberate exception: it asks for `SUPER_PLUS` first and
falls back to `HIGH`, printing which it got, so it is how you find out whether a
change bought you USB3.

Luxonis document the same failure and workaround in the
[USB deployment guide](https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide);
`X_LINK_ERROR` and this `io error` on a USB2-only cable are expected there.

## What fits on the link

The USB2-only firmware is also *faster* here: 25–26 fps at 960×540 against
20.7 fps through the USB3 build. Measured throughput, single stream:

| Stream | Rate |
|--------|------|
| CAM_A 640×360 BGR | 15.0 fps over 150 frames |
| CAM_A 960×540 BGR preview | 25–26 fps |
| CAM_A 1920×1080 | 10.0 fps |
| Stereo depth 640×480, LR check + subpixel | 15.0 fps, ~68 % valid pixels |

With colour and aligned depth both running and paired, the link saturates near
40 MB/s, and that ceiling — not the request — decides the result:

| Requested | Achieved | Throughput |
|---|---|---|
| 640×360 @ 15 | 15.4 fps both | ~18 MB/s |
| 960×540 @ 15 | 15.4 fps both, 45/45 paired | ~40 MB/s |
| 960×540 @ 25 | throttled to 15.1 fps | ~39 MB/s |
| 1280×720 @ 15 | throttled to 9.3 fps | ~43 MB/s |

So 960×540 at 15 fps is the useful maximum for the pair — asking for more frames
or more pixels just redistributes the same ceiling. Aligning costs no coverage:
the valid-pixel share stays the same ~65–68 % as unaligned depth. Full-resolution
colour plus depth together will not fit on USB2.

## If the device stops being found at all

After a failed boot the camera stays in `X_LINK_BOOTED` for a while and rejects
every connection with `X_LINK_INSUFFICIENT_PERMISSIONS`. Neither
`XLinkConnection(info)` nor `XLinkConnection.bootBootloader(info)` can reset it
from the host — both are refused for the same reason. It drops back to
`X_LINK_UNBOOTED` on its own, so wait it out rather than replugging, but not always
quickly: measured recoveries ranged from a few seconds to longer than the 45 s
`probe_device.py`'s `wait_unbooted` allows. A failed USB3 boot is what wedges it,
which is one more reason to pin `HIGH` everywhere.

```powershell
Get-PnpDevice | ? InstanceId -like '*VID_03E7*'
```

shows the two states: PID `2485` is the ROM bootloader — idle and healthy — and
PID `F63B` is the booted device, listed as a phantom whenever the camera is in ROM
state.

### The other wedge, on the Pi: healthy in `lsusb`, and still refusing to boot

There is a second failure that looks nothing like the one above, and the difference
matters because the advice above — wait it out — does not touch it. Seen on `rpi`
after an unexpected reboot, 2026-08-19: the device enumerates perfectly as
`03e7:2485`, nothing else has it open, `/dev/bus/usb` is writable, and every attempt
to boot the VPU fails at the first step with

```
ncDeviceOpen (booting the VPU): communication error (-2)
```

The tell is in `dmesg`, not in the error: `usb 1-1.3.2: can't set config #1, error
-71`. The device answers enumeration and then fails to accept a configuration, which
is a USB-level wedge rather than an XLink one, and no amount of waiting clears it.
**A device-level reset is not enough** — neither the `USBDEVFS_RESET` ioctl on
`/dev/bus/usb/001/NNN` nor unbinding and rebinding the device itself changed
anything. What cleared it was re-binding the **parent hub**:

```bash
sudo sh -c 'echo 1-1.3 > /sys/bus/usb/drivers/usb/unbind; sleep 5;
            echo 1-1.3 > /sys/bus/usb/drivers/usb/bind'
```

after which the self-test passed first time — VPU booted in 4.1 s, inference back at
its usual 109 ms. Find the path with

```bash
for d in /sys/bus/usb/devices/*/idVendor; do
    [ "$(cat $d)" = "03e7" ] && echo "${d%/idVendor}"
done
```

and take the parent: `1-1.3.2` is on hub `1-1.3`. Everything else on that hub — on
this rover the camera, the audio dongle and the lidar's serial adapter — is
re-enumerated too and comes back on its own; the ESP32 is on the GPIO UART and is
not affected, so the wheels are not part of this. Stop the daemon first if it is
tracking, so nothing is holding `/dev/video0` across the reset.

That this followed a spontaneous reboot is probably not a coincidence, and
[oak-on-the-pi.md](oak-on-the-pi.md) has the reason to suspect: the 5 V rail. Treat a
Pi that reboots by itself and an OAK that will not boot afterwards as one event.
