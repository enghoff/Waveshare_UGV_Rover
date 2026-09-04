# The OAK as the rover's depth camera

A colour picture and a range for anything in it, over HTTP on
`127.0.0.1:8770`, kept awake from boot by a `crontab` entry. This is the camera
doing what it is built for: until 2026-08-23 it was the rover's *face detector*,
because the Pi 1 could not run one, and every board since runs YuNet faster than
the camera's VPU did — see [face_tracking/yunet.py](../face_tracking/yunet.py).

```
  jetson@jetson-orin (on the rover)
  ---------------------------------
  OAK-D-LITE ==USB2/XLink==> depth_server.py ==> GET  /health  is it awake
   CAM_A colour, 640x360         |                   /depth    ranges, in mm
   CAM_B + CAM_C stereo,         |                   /depth.png a picture of it
   warped into CAM_A's frame     |                   /frame    the colour picture
   all on the VPU                |                   /power    on, off or waking
                                 |              POST /ranges   how far are these
                        run_oak_depth.sh                       boxes away
                        (@reboot, and restarts it)  /power    switch it off,
                                                              and on again
```

**The depth is aligned to the colour camera, since 2026-09-04, and that is what
turned this from a source into something usable.** `setDepthAlign(CAM_A)` warps
the disparity out of the mono pair's geometry into the colour camera's on the
device, so the same *fraction* of `/frame` and of the depth map is the same ray:
a box drawn on the picture indexes the ranges directly, with no remapping on the
host and no second lens model to drift out of step. Three consequences, all of
which change what an old reading meant:

* depth is measured from the **colour** camera's optical centre now, not the
  right mono's;
* every angle this service quotes is in the colour camera's frame — **70.1°
  across and 43.0° high**, read off the intrinsics the device stores rather than
  off `getFov`'s rounded 69, and worked out through the lens rather than
  straight across the frame (a quarter of the way out those differ by 1.8°);
* the depth map is 320×180 rather than 320×240, because it now covers a 16:9
  colour frame.

**Something reads it now.** The semantic world state asks `/ranges` how far away
the things it has just found are, and spends the answer on deciding which lasting
thing each of them is — see [world_state/oak.py](../world_state/oak.py). Nothing
still steers by it, which remains deliberate.

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

This was worked out while ruling the OAK *off* the Pi 1, in a document removed on
2026-08-25 along with the board it was about; the reasoning still holds and what
changed is the host it was measured against.

## Which is why the camera has a switch, since 2026-09-04

If holding the device open is the whole of being awake, letting go of it is the
whole of being off. So `POST /power {"on": false}` closes the device and **keeps
the process**: the Myriad falls back to ROM bootloader inside its 1500 ms
watchdog and waits there, and the port goes on answering rather than going
silent, which is the only way a camera somebody switched off can be told from a
service that has died.

**Nobody presses it. The wheels do, since 2026-09-04.** `rover_daemon`
([rover_depth.py](../rover_daemon/rover_depth.py)) watches the navigator's move
mutex twice a second and posts here: on the moment the rover drives, off thirty
seconds after it stops. The console had a tick box for this and no longer does,
because the question it asked — is this rover doing anything that needs a depth
camera? — is one the rover can answer for itself and a person at a screen usually
cannot. What is left on the console is a lamp beside the battery heading, red
for off, green for on and amber for waking, which reports and does not set.

Two consequences worth knowing before reading a recording. **The first four to
six seconds of every drive have no ranges in them**, because that is the
firmware upload, and a look taken during it stores its regions with no distance
— which is the same abstention `world_state` already records for a box the OAK
cannot see into. And **a parked rover measures nothing**, which is the saving
rather than a fault: the looks it takes while standing still are of a room it has
already ranged from that spot. Forcing it awake by hand is the `curl` below, and
the rule will switch it back at its next tick.

Three states, not two, and the third is the reason anything about this is
visible on a screen:

| | |
|---|---|
| `on` | the device is open and frames are arriving |
| `waking` | the firmware is going up the USB link and the pipeline is building |
| `off` | the device is closed; `03e7:2485` on the bus, waiting for a host |

**`waking` is not a detail.** This camera has no flash, so switching it on
uploads the firmware and rebuilds the stereo pipeline every single time, and
that is **4 to 6 seconds** on this link, measured over three cycles on the rover
on 2026-09-04 — 4.1 s from cold, then 5.8 and 4.3 s — during which the camera is
switched on and answering nothing. `POST /power` therefore returns immediately with `waking` and
the wake happens on another thread — a console that blocked on it would be a
console with no map, no lights and no stop button for the duration.

While it is off, `/frame`, `/depth`, `/depth.png` and `/ranges` all answer *the
depth camera is switched off* rather than "no frame yet", and the held frames are
thrown away the moment the switch is thrown, because a service that went on
handing out the last picture it happened to have would be handing out a
photograph of a room the rover may since have left. `/health` answers **200**
when the camera is off: it is this service correctly reporting a camera that is
doing as it was told, and the only 503 left is a camera that is meant to be awake
and has no frame.

**The camera really does go down, and that is the part that was checked rather
than assumed.** Measured on the rover on 2026-09-04: five seconds after the
switch, `lsusb` moves from `03e7:f63b` — booted, running — to `03e7:2485`, the
ROM bootloader, which is a Myriad with no firmware in it waiting for a host. That
is the same state the camera sits in when nothing has ever opened it, and it is
the whole of what "off" can mean on a device with no flash.

**What it saves cannot be measured on this rover, and the honest answer is that
it is not known.** Three conditions the same day, thirty one-second samples each,
with the rest of the rover running:

| | Orin `VDD_IN` |
|---|---|
| streaming at 2 fps | 5.931 W |
| switched off, camera in ROM bootloader | 5.920 W |
| the whole service stopped as well | 5.902 W |

Eleven milliwatts for the switch and twenty-nine for stopping the process
outright — against a sample-to-sample spread of 5.856 to 6.2 W in *every* one of
the three. So this instrument says nothing: the differences are well inside its
noise, and an earlier reading of "about 50 mW" from forty samples was noise too.

The reason is that the instrument is looking in the wrong place. `VDD_IN` is the
**Jetson module's** input, and the OAK hangs off the carrier board's USB 5 V,
which is upstream of it — so the camera's own draw never crosses that shunt, and
what the table can show is only the host-side cost of no longer carrying 300 kB/s
over USB. The driver board reports pack voltage and no current at all, so there
is no second instrument to ask.

What is left is a manufacturer's figure and an inference: Luxonis quote about
2.5 W for an OAK-D-Lite under load, and a Myriad in ROM bootloader with three
sensors dark is a small fraction of that. That makes the saving plausibly the
largest single load a person can shed from this rover, and **it remains
unmeasured**. Settling it wants a USB power meter inline with the camera; until
somebody puts one there, the switch is worth having because the camera is
provably off, not because a number here says how much that is worth.

**Every switch-off writes a depthai crash dump.** Closing the device while the
pipeline is streaming makes the library log `Device with id ... has crashed` and
save a dump under `~/.cache/depthai/crashdumps/`. Nothing is wrong — that is the
Myriad being dropped, which is what was asked for — but it means
[`read_crash_dump.py`](../oak_camera/read_crash_dump.py) will find a dump after
any switch-off, and a dump timed to one is this rather than a fault.

The switch is not remembered. A reboot, a crashed process, or a deploy that
restarts this service all bring the camera back on, because the process starts
awake — the only state there is is the running process, deliberately, and a
switched-off camera that stayed off across a restart nobody was watching would be
a rover that had quietly lost a sensor. The daemon's rule then takes it off again
half a minute later if the rover is standing still, which is the same answer
arrived at by the same route rather than a state carried across the restart.

```bash
ssh orin 'curl -s http://127.0.0.1:8770/power'
ssh orin 'curl -s -X POST -H "Content-Type: application/json" \
          -d "{\"on\": false}" http://127.0.0.1:8770/power'
ssh orin 'lsusb | grep 03e7'     # 2485 is off, f63b is awake
```

## Which firmware version is the best match: 2.32.0.0

Measured on the board the rover ran then — Banana Pi M4 Zero, aarch64, CPython
3.13 — on 2026-08-23, each case in its own process because the failure has taken hosts
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
native there, left-right check and subpixel on, a 5×5 median, the disparity
decimated 2× **on the device** and then warped into the colour camera's geometry
and emitted at 320×180. Beside it the colour camera runs at 1080p, downscaled on
the device's own ISP to 640×360 and encoded to MJPEG by the VPU's own encoder —
so what crosses the link is a finished 18 kB picture rather than 460 kB of pixels,
and this process needs no image library at all.

**640×360 because that is the widest thing this sensor offers, not because 16:9
was wanted.** Asked of the device rather than assumed: the IMX214 offers
1920×1080, 3840×2160, 4056×3040 and 4208×3120 and nothing else. The 1080p mode is
a *vertical* crop rather than a horizontal one — 70.1° across either way, and
43.0° high against the full sensor's 54.9 — so the wide mode costs a third of the
vertical field and saves the ISP eleven megapixels a frame. The 4:3 route is
`THE_12_MP` with an ISP downscale, if that third ever turns out to matter.

**The default is two frames a second**, lowered from ten on 2026-08-31 and
unchanged when the colour camera was added beside the depth. What reads this asks
about once a second at most and a parked rover is not looking at anything new, so
the job is to have a picture and a range ready when somebody asks rather than to
stream either. Raising it is `--fps`, and because the rate is baked into the
pipeline when it is built, changing it is a restart and a fresh firmware upload
rather than a live retune.

**The two halves are paired on the device's own timestamps, and that was worth
doing.** The colour frame goes through the MJPEG encoder before it crosses the
link, so it arrives a whole exposure behind its own depth frame: pairing whatever
was newest of each put them **0.49 s apart at 2 fps -- one full frame interval,
and 23 cm of rover** at the speed it explores at. A box drawn on one of those and
a range taken from the other are not one measurement. So each depth frame is held
back until the picture it belongs with has arrived, and the two are published
together.

**What that leaves is a fifth of a second, and it is the sensors rather than the
pairing.** Measured on the rover on 2026-09-04 over sixty reads a second apart:
`depth_apart_s` has a **median of 0.197 s and a worst of 0.217**, never anywhere
near the 0.5 s a mispairing would give. The colour sensor and the mono pair
free-run on their own clocks -- this camera has no hardware sync between them --
so a correctly matched pair still sits about that far out of phase, and that is a
floor rather than a fault to chase. Raising `--fps` shrinks it proportionally,
which is the only lever there is.

The other cost is that the pair is about **0.65 s old** when it is read rather
than 0.03. Both numbers ride on the reply -- `X-Frame-Age` and `X-Depth-Apart` --
because both are time the rover was moving through, and the consumer is the only
thing that knows how fast: `world_state`'s inspection charges the pair of them to
every range it stores.

**What the drop to 2 fps actually bought, measured the same day:** the Orin's
`VDD_IN` rail fell from 6.49 W to 6.32 W, each averaged over forty one-second
samples with the rest of the rover running, and this service's own CPU from 1.5%
of a core to 0.5%. So about 180 mW, or 3% of the board — real, but small enough
to sit inside the rail's own 300 mW sample-to-sample spread, and it should be
read as "a little" rather than as a figure to plan around. Note also that
`VDD_IN` is the *Orin's* input, so how much of the camera's own USB draw it sees
is unproven. The link is the honest saving: 1.5 MB/s down to roughly 300 kB/s.

The table below was measured at the old 10 fps default, with the service running
and the rover otherwise doing its usual work. The rates and the link cost scale
with `--fps`; the rest does not:

| | on the Jetson Orin Nano, 2026-08-31 | on the Banana Pi, 2026-08-23 |
|---|---|---|
| depth | 320×240 at 10.0 fps, 43–48% of pixels valid | 320×240 at 10.0 fps, 61–66% valid |
| the lens, read off `getFov` | 73.0° across, 7.5 cm baseline | the same camera |
| USB | negotiated `HIGH` | negotiated `HIGH`; about 1.5 MB/s of the link |
| this board | **1.5% of one core**, 157 MB resident | 13% of one core, 156 MB resident |
| face tracking beside it | still opens the gimbal camera and counts faces | 6.6 → 6.2 frames a second |
| the scan matcher beside it | not measured here — ROS is waiting on a chassis calibration | 8394 scans, 2 dropped, no window overruns |

That table predates the colour camera and the alignment, so read its depth row as
the mono pair's own output rather than as what `/depth` returns today. What the
addition cost, measured on 2026-09-04 with the same board doing its usual work:
the aligned map is 320×180 at 2 fps, which is 230 kB/s where the unaligned 320×240
was 307, and the colour stream adds about 40 — so **the whole service moves less
over the link than the depth alone used to**, and the valid share was 33–59% in a
room the rover was parked a metre from.

The two valid-pixel figures are different rooms rather than different hardware:
the fraction of the frame stereo can range is a property of what the camera is
looking at, and a near, textureless wall returns no disparity at all.

The link cost is the number the pipeline was designed around rather than a
by-product — 1.5 MB/s at the 10 fps this was measured at, and about 300 kB/s at
today's default of 2. Everything on this rover shares one 480 Mbps root port — the
wifi dongle, the tracking camera, the lidar's serial adapter and this — and the
OAK alone has been measured saturating near 40 MB/s when colour and aligned depth
are paired. Losing the wifi adapter to USB
contention means losing the way to say "stop". `--fps` and `--decimation` are the
two knobs; raising either raises that number.

## What `/depth` says

```
$ curl -s http://127.0.0.1:8770/depth
{"ok": true, "age_s": 0.37, "size": [320, 180], "valid": 0.570,
 "near_mm": 548, "band": [0.3, 0.7],
 "sectors": [{"from_deg": -35.1, "to_deg": -27.8, "near_mm": 548, "valid": 0.203},
             ...
             {"from_deg": 27.6, "to_deg": 34.9, "near_mm": 1089, "valid": 0.650}],
 "grid_mm": [[1055, 1055, 1055, 1055, ...], ...]}
```

Eight sectors across the 70.1°, each about the width of a doorway at three metres,
and a 8×6 grid of the whole frame. Note that the sectors are **not** equal widths:
they are equal slices of the *picture*, and a slice at the edge of a lens covers
less angle than one in the middle. Reading them as the field of view divided by
eight is out by 1.8° a quarter of the way across, which mattered as soon as
something started drawing bearings through this camera. Three further choices are
worth knowing about before anything is concluded from the numbers:

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

Angles are relative to the colour camera's axis, positive to the right — the same
sign convention `aiming.py` uses for a face in a picture. They used to be
relative to the right mono camera's; the alignment moved them.

## What `/frame` and `/ranges` say

`/frame` is the newest colour picture as a JPEG, with three headers on it that a
consumer needs and a second call could not answer about the same frame:

```
X-Frame-Age: 0.680      how old the picture is, in seconds
X-Depth-Apart: 0.143    how far the depth behind it was exposed from it
X-Frame-Size: 640x360
```

`/ranges` takes boxes drawn on that picture, as fractions, and says how far away
the thing in each is:

```
$ curl -s -X POST -H 'Content-Type: application/json' \
       -d '{"boxes": [[0.35,0.35,0.65,0.65]]}' http://127.0.0.1:8770/ranges
{"ok": true, "age_s": 0.70, "size": [320, 180], "frame_age_s": 0.70,
 "depth_apart_s": 0.143,
 "ranges": [{"range_m": 1.116, "sigma_m": 0.048, "valid": 0.682, "pixels": 2366}]}
```

Fractions rather than pixels because the picture and the depth map are different
sizes covering the same field, and a fraction is the one coordinate that means the
same thing in both — which is the whole benefit of aligning the depth on the
device.

**A box is a thing in front of a room, and the question is how far the thing is.**
A region drawn round a chair also contains the floor beside it and the wall behind
it, so the median of the box is a blend of three surfaces and the minimum is one
bad pixel. What comes back is the near *surface*: the 20th percentile of the valid
pixels finds roughly where the front face is, everything within 30 cm or 15% behind
that is taken to belong to it, and the answer is the median of those.

`sigma_m` is what the reading is worth: the stereo model's own error at that
distance — `z² × 0.2 px / (455.8 px × 0.075 m)`, which is 2.3 cm at two metres, 9.4
at four and 21 at six — added to the spread of the surface behind it, which covers
a thing being deep or slanted rather than flat. **The 0.2 px of disparity noise is
assumed and not measured**; the other two terms are read off the device. It wants a
tape measure, and what it produces is spent as a weight by
[`world_state/locate.py`](../world_state/locate.py), so being wrong optimistic
makes the world state trust a range more than it should.

`null` for a range means the box had fewer than twelve valid pixels in it, which is
not the same as nothing being there — `valid` beside it is how to tell those apart.

## What still has to be settled before this steers anything

The service is deliberately a source rather than an input to **navigation**. The
lidar is what keeps the rover off walls today, and it is 2D: one horizontal plane
at its own height, which is precisely why a depth camera is worth having. The
semantic world state reads `/ranges` and has no authority over the wheels, which
is the order this was always meant to happen in. Before a range from here steers
anything:

* **Where is this camera, in the rover's frame?** Still nothing in this repository
  relates the OAK to the lidar or to the tracks. What *has* been built is the
  relation between the OAK and the **gimbal camera**, which is what the world
  state needs and which [`world_state/bench_oak.py`](../world_state/bench_oak.py)
  measures — the two cameras look at one room and the rotation that makes their
  answers agree is the mount. A range without a frame is still not a distance to
  anything, and a range in the *camera's* frame is not one in the chassis's.
* **The floor is not an obstacle.** A forward-and-slightly-down camera sees it at
  a few metres and reports it faithfully. The band above is a crude answer; the
  real one is a ground-plane fit.
* **The gimbal.** The tracking camera moves and this one does not, so a face at
  pan 90° and a wall at 0° are not in the same picture. The world state handles
  that by mapping a gimbal box into this camera's picture and taking no range at
  all when it lands outside — which is about half of a centred frame and all of a
  look taken over the rover's shoulder.

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
ssh orin 'curl -s -o /tmp/oak.jpg -D - http://127.0.0.1:8770/frame'
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
fault. It is not one.
