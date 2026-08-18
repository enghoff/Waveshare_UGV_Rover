# Face detection on the rover, on the OAK's VPU

JPEG in, boxes out, on `127.0.0.1:8768` — [face_detect/](../face_detect)'s protocol
answered by the rover itself instead of by MEDIA. The rover now sees with nothing
else switched on.

```
  admin@rpi (on the rover)
  ------------------------
  USB camera --MJPEG--> rover_daemon.py --POST /detect--> server.py
                             ^                                |
  ST3215 servos <--UART--  aiming.py                     oak.py -> liboak.so
                             |                                |
                             +---------- boxes ---------------+
                                                          USB | XLink
                                                  the OAK's Myriad X
```

## The point: this is not depthai, and the camera is not a camera here

[oak-on-the-pi.md](../docs/oak-on-the-pi.md) ruled the OAK out on this rover, and
every word of it still holds — depthai has no wheel for armv6 and CPython 3.13, and
building one would be the smallest of the three problems it lists. This does not
contradict that. It goes around it.

The Myriad X in an OAK-D-Lite is the same chip Intel sold as a **Neural Compute
Stick 2**, and it cannot tell you apart from Intel: it enumerates as `03e7:2485` in
ROM bootloader state, waits for somebody to upload firmware, and comes back as
`03e7:f63b` once booted. So this uploads Intel's own `usb-ma2x8x.mvcmd`, hands the
device a graph compiled ahead of time on a workstation, and pushes frames through
it. No pipeline, no depthai, and **the camera's three image sensors are never
opened**. It is being used as an inference stick that happens to have lenses on it.

That also means none of this protocol was reverse-engineered. `vendor/movidius/`
is Intel's own XLink and mvnc, lifted unmodified from OpenVINO 2021.4.2 and built
for armv6 by `build.sh` — the same C the OpenVINO MYRIAD plugin calls. The four
calls that matter are `ncDeviceOpen`, which boots the chip, `ncGraphAllocate`,
which uploads the graph, and the two FIFO calls.

## What is here

| | |
|---|---|
| `build.sh` | builds `liboak.so` on the host that runs it — 13 min the first time, seconds after |
| `oakjpeg.c` | JPEG to planar BGR; the only code here that is ours rather than Intel's |
| `oak.py` | ctypes over the `nc*` API: boot, load a graph, one frame at a time |
| `server.py` | the HTTP shape around it, on loopback |
| `selftest.py` | library, boot, graph, inference, decode, and a real face — in that order |
| `run_oak_detect.sh` | keeps it running, and starts it at boot from `crontab` |
| `usb-ma2x8x.mvcmd` | Intel's Myriad X firmware, uploaded on every open |
| `face-detection-retail-0004-320x240.blob` | the graph |
| `97-myriad-usbboot.rules` | udev, and the thing that will catch you |

## The graph, and why 320x240

`face-detection-retail-0004` is an SSD whose native input is 300x300. This ships it
reshaped to **320x240**, which is not a tuning choice but a decode choice:
libjpeg-turbo can scale while it decodes, but only by a fixed set of fractions, and
320x240 is exactly half of the camera's 640x480. The decoder therefore lands
precisely on the graph's input and the resize step disappears — 70 ms of bilinear
interpolation on this host, measured, which was more than the inference it fed. It
keeps the frame's 4:3 shape as well, where the stock 300x300 input squashes it.

Reshaping an SSD moves its priors, so this was checked rather than assumed. On the
same photograph, against the model at its native size, on the CPU:

| | box | confidence |
|---|---|---|
| native 300x300 | (209,191)-(355,390) | 1.000 |
| reshaped 320x240 | (206,199)-(359,388) | 1.000 |

and through this service on the Pi, against what YuNet on MEDIA returned for the
same picture:

| | box (x, y, w, h) | score |
|---|---|---|
| YuNet, on MEDIA | 207.8, 182.5, 145.9, 206.9 | 0.909 |
| this, on the OAK | 207.4, 198.8, 151.1, 188.5 | 0.9995 |

To rebuild the blob, on a machine with OpenVINO 2021.4.2 — the last release that
shipped a MYRIAD plugin, and the pip wheels never did, so it has to come from the
[runtime archive](https://storage.openvinotoolkit.org/repositories/openvino/packages/2021.4.2/w_openvino_toolkit_runtime_p_2021.4.752.zip):
read the IR, `net.reshape({input: (1, 3, 240, 320)})`, serialize, then

```bash
myriad_compile -m fd0004_320x240.xml -o face-detection-retail-0004-320x240.blob -ip U8
```

`-ip U8` matters: it costs nothing on the device and it is the difference between
sending 230 kB a frame and 920 kB. Measured on a workstation, the same inference
was 42.6 ms with a float input and **24.8 ms** with this one.

## What it costs

Measured 2026-08-18 on the rover, a 640x480 frame from its own camera:

| | |
|---|---|
| boot the VPU and upload the graph | 4.4–6.1 s, once, at startup |
| decode one 640x480 MJPEG frame | **85–87 ms** |
| the same at 320x240, which the camera also offers | 64 ms |
| the inference | 41 ms (24.8 on a workstation — the rest is this host's USB) |
| this Pi's HTTP stack, `GET /health` under load | 41 ms |
| one whole `POST /detect` | ~190 ms |
| the tracking loop, with the scan matcher also running | **2.3 fps** |

**The inference is the cheapest part, and JPEG is the problem.** This is the cost
[face_detect/server.py](../face_detect/server.py) predicted when it argued the
picture should cross the network instead: decoding a frame here costs 93 ms, it
said, and it was right. What has changed is that the alternative is no longer 6 ms
on a 5700G plus a LAN — it is a desktop that has to be awake.

So this trades frame rate for not needing one. The way to get the rate back is to
stop sending JPEG at all: this camera offers uncompressed YUYV at 640x480, the
graph would take those pixels directly, and the 85 ms would simply not exist. That
is a change to how frames are captured rather than anything about the detector, so
it is not done here.

## Building it

Native, on the machine that will run it — the Pi is armv6 and nothing else here is.

```bash
scp -r oak_detect rpi:~/ugv/
ssh rpi '~/ugv/oak_detect/build.sh && python3 ~/ugv/oak_detect/selftest.py'
```

Neither `-dev` package is installed on the Pi and `sudo` there wants a password, so
the build links `libusb-1.0.so.0` and `libturbojpeg.so.0` by path, carries its own
copy of `libusb.h`, and writes out the five TurboJPEG prototypes it uses rather
than including a header. Objects are kept between builds, so changing `oakjpeg.c`
costs seconds; `build.sh --clean` starts over.

## The udev rule — this is the part that will catch you

`/dev/bus/usb/*` is `root:root` at mode 0664, so libusb cannot open the device as
`admin` and every call fails with `LIBUSB_ERROR_ACCESS`. The selftest reports this
as the device not being there, because from the library's side that is
indistinguishable. Intel's own rule is shipped here and grants group `users`, which
`admin` is already in:

```bash
sudo cp ~/ugv/oak_detect/97-myriad-usbboot.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

It covers `2485` and `f63b` both, which it has to: the device changes product ID
when it boots, and a rule for only the unbooted state grants access to upload the
firmware and then loses it.

## Running it

`crontab` for `admin`, beside the daemon's own entry, for the reason
[run_daemon.sh](../rover_daemon/run_daemon.sh) gives — a system unit would need a
sudo password we do not have from a script.

```
@reboot /home/admin/ugv/oak_detect/run_oak_detect.sh
@reboot /home/admin/ugv/run_daemon.sh --vision --lidar
```

The daemon needs no flag for it: `DEFAULT_SERVICE` in
[rover_daemon.py](../rover_daemon/rover_daemon.py) is `127.0.0.1:8768` now. Pass
`--service` to point it somewhere else, which is also how you put it back on MEDIA.

```bash
ssh rpi '~/ugv/oak_detect/restart.sh'    # ~6 s; prints /health when it is back
tail -f ~/ugv/oak_detect/oak_detect.log
```

**Use `restart.sh` rather than typing the `pkill` yourself.** The pattern that
matches the server also matches the ssh command carrying it, so `ssh rpi 'pkill -f
oak_detect/server.py'` kills that ssh session as well -- the output disappears and
it reads as the service failing to come back when it is merely restarting. This
repository has now made that mistake twice, once per supervisor.

**Restarting it is the fix for almost anything.** The VPU has no flash and boots
from its host every time, so a device that has been unplugged, browned out, or left
booted by a crashed process is recovered by opening it again from scratch — which
is what a restart does and what nothing else does.

## Checks

```bash
ssh rpi 'python3 ~/ugv/oak_detect/selftest.py --jpeg /tmp/face.jpg'
ssh rpi 'curl -s http://127.0.0.1:8768/health'

# and the thing that actually matters, through the daemon:
printf '{"call":"start_tracking"}\n{"call":"tracking_status"}\n' | nc rpi.local 8769
```

`/health` carries running medians for decode and detection separately, which is the
quickest way to tell a slow host from a slow device.

## The power question, answered

[oak-on-the-pi.md](../docs/oak-on-the-pi.md) feared the 5 V rail: the OAK declares
500 mA on a bus already declaring 2042, and the failure to fear was the lidar or the
wlan adapter dropping rather than the camera failing to open. Measured through boot,
graph upload and sustained tracking, `vcgencmd get_throttled` stayed **`0x0`** and
neither the lidar nor the wifi dropped. Note that the OAK sits on the `05e3:0610`
hub, which reports itself self-powered — if that hub has its own supply then the
rail was never asked for the 500 mA, and this result says less than it appears to.
Worth re-checking on battery, away from a bench supply.
