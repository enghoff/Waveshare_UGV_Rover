# The OAK as the rover's depth camera

Millimetres out over HTTP on `127.0.0.1:8770`, from the OAK-D-Lite's mono pair,
kept awake from boot by a `crontab` entry. This is the camera doing what it is
built for: until 2026-08-23 it was the rover's *face detector*, because the Pi 1
could not run one, and every board since runs YuNet faster than the camera's VPU
did — see [face_tracking/yunet.py](../face_tracking/yunet.py).

```
  jetson@jetson-orin (on the rover)
  ---------------------------------
  OAK-D-LITE ==USB2/XLink==> depth_server.py ==> GET /health  is it awake
   CAM_B + CAM_C                 |                  /depth    ranges, in mm
   stereo on the VPU             |                  /depth.png a picture of it
                        run_oak_depth.sh
                        (@reboot, and restarts it)
```

## "Upload the firmware at boot" means a process that stays

The Myriad X in an OAK has **no flash**. It enumerates as `03e7:2485` in ROM
bootloader state, waits for a host to hand it firmware over USB, and comes back as
`03e7:f63b` once booted; depthai does that upload every time a pipeline opens,
which takes a few seconds. Two consequences, and they are the shape of this whole
directory:

* **The firmware version is the depthai version.** There is no separate firmware
  to flash and no newer one to move to — `getBootloaderVersion()` returns None on
  this board and the crash log says `Invalid Flash JEDEC ID... No NOR available`.
  Choosing a wheel is choosing a firmware, which is why `install.sh` pins one by
  hash and why the section below is a measurement rather than a preference.
* **A booted device with no host dies in 1500 ms.** It watchdogs itself, so
  "bring the OAK up at boot" cannot mean a one-shot push. It means a process that
  opens the device and stays, and the camera is awake for exactly as long as that
  process lives. That is `depth_server.py`, and `run_oak_depth.sh` is what starts
  it at boot and starts it again when it dies.

[docs/oak-on-the-pi.md](../docs/oak-on-the-pi.md) worked this out while ruling the
OAK *off* the old rover, and every word of it still holds; what changed is the
host it was measured against.

## Which firmware version is the best match: 2.32.0.0

Measured on the rover's own board — Banana Pi M4 Zero, aarch64, CPython 3.13 —
on 2026-08-23, each case in its own process because the failure has taken hosts
down with it before:

| | depthai 2.32.0.0 | depthai 3.9.0 |
|---|---|---|
| CAM_A, colour | — | 30 frames, 28.0 fps |
| CAM_C, right mono | (in the stereo pair) | 30 frames, 28.3 fps |
| CAM_B, left mono | (in the stereo pair) | **0 frames**, device crash dump |
| stereo depth | **10.0 fps, 61–66% valid** | **0 frames**, `X_LINK_ERROR`, crash dump |

So 2.32.0.0 — the newest 2.x, released 2026-01-27 — is what `install.sh` pins, and
3.x is unusable here for the only thing this directory wants. The regression is
not new and is not fixed: the desk measured the same failure on 3.8.0 in August,
on Windows, against the same camera, and 3.9.0 is the current release as of
2026-08-15. Reproducing it on Linux/aarch64 rules out the host OS as a confound
and leaves it where [docs/depthai-version-pin.md](../docs/depthai-version-pin.md)
put it: a v3 regression in the mono init path, with two open upstream issues.

Nothing else about 3.x recommends it here either. CAM_A works no better than on
2.x, and the colour sensor is not what this service is for.

The version is pinned; the *interpreter* is not. Unlike OpenCV's abi3 wheel,
depthai ships one build per CPython, so `install.sh` picks the file from whatever
`python3` it finds and checks it against the hash pinned for that interpreter.
It knows 3.12 and 3.13 — Ubuntu 24.04 on the Jetson and Debian trixie on the
Banana Pi — and tells you what to add if it meets anything else. All of them are
the same 2.32.0.0 release, so this changes which file is fetched and never which
firmware the camera gets.

## What it does, and what that costs

The pipeline is deliberately modest. Mono at 480p because the OV7251 sensors are
native there, left-right check and subpixel on, a 5×5 median, and the depth map
decimated 2× **on the device** so 320×240 crosses the wire rather than 640×480.

**The default is two frames a second**, lowered from ten on 2026-08-31. Nothing
consumes `/depth` yet and a parked rover is not looking at anything new, so the
job is to have a range ready when somebody asks rather than to stream one; two
frames a second means the answer is never more than half a second old. Raising it
is `--fps`, and because the rate is baked into the pipeline when it is built,
changing it is a restart and a fresh firmware upload rather than a live retune.

The table below was measured at the old 10 fps default, with the service running
and the rover otherwise doing its usual work. The rates and the link cost scale
with `--fps`; the rest does not:

| | on the Jetson Orin Nano, 2026-08-31 | on the Banana Pi, 2026-08-23 |
|---|---|---|
| depth | 320×240 at 10.0 fps, 43–48% of pixels valid | 320×240 at 10.0 fps, 61–66% valid |
| the lens, read off the stored calibration | 73.0° across, 7.5 cm baseline | the same camera |
| USB | negotiated `HIGH` | negotiated `HIGH`; about 1.5 MB/s of the link |
| this board | **1.5% of one core**, 157 MB resident | 13% of one core, 156 MB resident |
| face tracking beside it | still opens the gimbal camera and counts faces | 6.6 → 6.2 frames a second |
| the scan matcher beside it | not measured here — ROS is waiting on a chassis calibration | 8394 scans, 2 dropped, no window overruns |

The two valid-pixel figures are different rooms rather than different hardware:
the fraction of the frame stereo can range is a property of what the camera is
looking at, and a near, textureless wall returns no disparity at all.

The link cost is the number the pipeline was designed around rather than a
by-product — 1.5 MB/s at the 10 fps this was measured at, and about 300 kB/s at
today's default of 2. Everything on this rover shares one 480 Mbps root port — the
wifi dongle, the tracking camera, the lidar's serial adapter and this — and
[docs/oak-usb-link.md](../docs/oak-usb-link.md) has the OAK alone saturating near
40 MB/s when colour and aligned depth are paired. Losing the wifi adapter to USB
contention means losing the way to say "stop". `--fps` and `--decimation` are the
two knobs; raising either raises that number.

## What `/depth` says

```
$ curl -s http://127.0.0.1:8770/depth
{"ok": true, "age_s": 0.0, "size": [320, 240], "valid": 0.537,
 "near_mm": 2723, "band": [0.3, 0.7],
 "sectors": [{"from_deg": -36.5, "to_deg": -27.4, "near_mm": 2723, "valid": 0.529},
             ...
             {"from_deg": 27.4, "to_deg": 36.5, "near_mm": 3026, "valid": 0.139}],
 "grid_mm": [[2837, 2644, 2644, 2723, 4539, 4863, 4464, 486], ...]}
```

Eight sectors across the 73°, each about the width of a doorway at three metres,
and a 8×6 grid of the whole frame. Three choices in there are worth knowing about
before anything is concluded from the numbers:

* **A sector's range is the fifth percentile of its valid pixels, not their
  minimum.** A single pixel is noise — stereo mismatches put isolated very-near
  readings on textureless walls — and a percentile over a thousand-pixel cell is a
  surface.
* **The sector range is taken over the middle band of the frame** (rows 30–70%),
  because the top of the picture is ceiling and the bottom is floor a metre in
  front of the tracks, and the rover drives under and over both. The full grid is
  there so a consumer can decide differently.
* **`null` means too few valid pixels to call anything**, which is not the same as
  nothing being there. A dark or textureless surface returns no disparity at all.
  `valid` beside it is how to tell the two apart.

Angles are relative to the depth camera's own axis, positive to the right — the
same sign convention `aiming.py` uses for a face in a picture.

## Nothing consumes this yet, and here is what would have to be settled first

The service is deliberately a source rather than a input to navigation. The lidar
is what keeps the rover off walls today, and it is 2D: one horizontal plane at its
own height, which is precisely why a depth camera is worth having. Before a range
from here steers anything:

* **Where is this camera, in the rover's frame?** There are no extrinsics between
  the OAK, the lidar and the tracks anywhere in this repository. A range without a
  frame is not a distance to anything.
* **The floor is not an obstacle.** A forward-and-slightly-down camera sees it at
  a few metres and reports it faithfully. The band above is a crude answer; the
  real one is a ground-plane fit.
* **The gimbal.** The tracking camera moves and this one does not, so a face at
  pan 90° and a wall at 0° are not in the same picture.

## Running it

`crontab` for `jetson`, beside the daemon's own entry, for the reason
[run_daemon.sh](../rover_daemon/run_daemon.sh) gives — a system unit would need a
sudo password no script has, and cron needs none:

```
@reboot /home/jetson/ugv/run_daemon.sh --vision --board-bridge --ros-nav
@reboot /home/jetson/ugv/oak_depth/run_oak_depth.sh
```

The source is deployed by [`deploy/deploy.py`](../deploy/README.md), which
restarts and verifies it; the wheel and the udev rule are one-off installs that
a deploy does not do, because the wheel is not tracked and the rule needs root.

```bash
python deploy/deploy.py --only oak_depth       # source, restart, /health
ssh orin 'sh ~/ugv/oak_depth/install.sh'       # depthai, unpacked into vendor/
ssh orin 'python3 ~/ugv/oak_depth/selftest.py' # with the service stopped
ssh orin '~/ugv/oak_depth/restart.sh'          # ~10 s; prints /health
ssh orin 'curl -s http://127.0.0.1:8770/depth'
ssh orin 'tail -f ~/ugv/oak_depth/oak_depth.log'
```

**Use `restart.sh` rather than typing the `pkill` yourself.** The pattern that
matches the server also matches the ssh command carrying it, so
`ssh orin 'pkill -f oak_depth/depth_server.py'` kills that ssh session as well —
the output disappears and it reads as the service failing to come back when it is
merely restarting. This repository has now made that mistake three times: twice
by hand, and once in the deploy manifest, which carried the supervisor
replacement inline and so failed the first deploy of this component to the
Jetson with exit 255. Replacing the supervisor is `restart.sh --supervisor`, and
that is what the manifest calls.

**Restarting it is the fix for almost anything.** The VPU has no flash and boots
from its host every time, so a device that has been unplugged, browned out, or
left booted by a crashed process is recovered by opening it again from scratch —
which is what a restart does and what nothing else does. `depth_server.py` helps by
exiting when the frames stop for five seconds: a depthai queue that has gone quiet
does not raise, so silence has to be noticed by a clock.

**Only one process can hold the camera.** The selftest says so rather than failing
obscurely, but the rule is worth stating: run the selftest with the service
stopped, and never point two things at the OAK.

## The udev rule — this is the part that will catch you

`/dev/bus/usb/*` is `root:root` at mode 0664, so libusb cannot open the camera as
an ordinary user and every call fails with `LIBUSB_ERROR_ACCESS` — which from the
library's side is indistinguishable from the camera not being plugged in. Intel's
own rule is shipped here and grants group `users`, which both `jetson` on the
Orin and `admin` on the old board are already in:

```bash
sudo cp ~/ugv/oak_depth/97-myriad-usbboot.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

It covers `2485` and `f63b` both, which it has to: the device changes product ID
when it boots, so a rule for only the unbooted state grants access to upload the
firmware and then loses it. `install.sh` checks the file is there and refuses to
claim success without it.

## Always pin USB2

`dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH)`, as every script in
[`oak_camera/`](../oak_camera) does. Without it depthai asks for USB3, uploads the
USB3-enabled firmware, and on this camera's link that firmware usually fails to
come back on the bus — 5 successful opens in 13 against 13 in 13, measured. The
host then waits ~9 s in `Searching for booted device`, the device watchdogs itself,
and what is left is a crash dump with `errorId=9001` that reads as a hardware
fault. It is not one. See [docs/oak-usb-link.md](../docs/oak-usb-link.md).
