# Waveshare UGV Rover

A suite of host-side tools for exercising and validating the components of a
**Waveshare UGV Rover** — the rover platform built on Waveshare's *General Driver
for Robots* board, with a Raspberry Pi as host and an ESP32 on the driver board —
**one at a time, in isolation**. Each script drives exactly one thing — one camera
socket, one sensor, one stored blob — so that when something is wrong you can
establish which component is at fault before any of them are combined into a robot
that does something. These are bring-up, bench-test and triage instruments, and
they remain the right thing to reach for first when a component misbehaves.

[`vm/`](vm) is the other half, and the opposite in kind: the sensors combined,
in ROS 2, building a map. It runs in a Linux VM rather than on the Windows host
and does not share a line of code with the rest — but it depends on the same
measurements, which is why the two live together.

| Directory | Component | Needs |
|---|---|---|
| [`oak_camera/`](oak_camera) | Luxonis OAK-D-Lite depth camera, over USB | depthai |
| [`lidar/`](lidar) | D500 lidar, over serial via the rover's driver board | pyserial |
| [`usb_cameras/`](usb_cameras) | the host machine's own UVC webcams | OpenCV only |
| [`driver_board/`](driver_board) | the ESP32 that drives the motors, over WiFi or USB | nothing |
| [`face_tracking/`](face_tracking) | the pan/tilt camera and its two servos, as one loop | OpenCV |
| [`vm/`](vm) | both sensors together: ROS 2, SLAM, sensor fusion | a Linux VM |

The first four are independent: any can be run with the other components
unplugged or unpowered, so a result from one never needs the others to be working.
[`face_tracking/`](face_tracking) is the exception among the host-side scripts —
it is the one that closes a loop between two components rather than exercising
one — so reach for it after both halves have been checked on their own, not
instead of checking them.

Only [`driver_board/`](driver_board) makes the rover move; everything else is
sensing, with the rover pushed by hand. Every host-side script is standalone,
needs no arguments, shares no state, imports nothing from the others, and quits
on `q` — `driver_board/` being the exception on the last two counts, since it
takes a controller rather than a window and stops on the pad's Back button.

## Repository layout

```
oak_camera/
    probe_device.py          whole device   does it boot, and what is on it
    inspect_calibration.py   whole device   intrinsics and distortion, user vs factory
    read_crash_dump.py       whole device   read out and clear the firmware crash dump
    preview_depth.py         CAM_B + CAM_C  stereo depth with a distance readout
    preview_rgb.py           CAM_A          colour, and with --depth all three sensors
    crash_dumps/             JSON dumps written by read_crash_dump.py, not tracked
lidar/
    lidar_view.py            whole sensor   top-down view of the point cloud
usb_cameras/
    preview_usb_cameras.py   one at a time  cycle through the host's USB cameras
driver_board/
    drive_gamepad.py         the ESP32     teleop from a game pad, no Pi involved
face_tracking/
    track_face.py            camera+servos scan for a face, then follow it
vm/                        both sensors in ROS 2; deploys to ~/ugv in the guest
    bin/                   operate: start, stop, record, screenshot
    checks/                measure and verify; run when hardware moves
    setup/                 one-shot provisioning, to rebuild the VM
    launch/  config/  nodes/
docs/                      the detail: hardware facts, measurements, failure modes
requirements.txt           four pins; the depthai one is deliberate, see docs
.cache/depthai/            depthai's own crash-dump cache, written by the library
```

A component's directory holds everything belonging to it, output included — which
is why `crash_dumps/` sits under `oak_camera/` rather than at the top level.
`.cache/` is depthai's, created relative to the working directory, so it appears
wherever you run from. `.gitignore` excludes `.venv/`, `__pycache__/`, `*.pyc`,
`captures/`, `oak_camera/crash_dumps/`, `*.mp4`, `*.npy`, `*.onnx` and
`calibration_backup_*.json` — a downloaded model is a dependency rather than
source, and re-fetching it costs one run; a crash dump describes one device's one
crash, so it is local evidence, not something to carry in the repo; saved lidar PNGs
(`lidar-<timestamp>.png`) are *not* ignored.

## Setup

One environment covers every component. Run the scripts from the repository
root, by path:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe oak_camera\probe_device.py
```

Requirements are `depthai>=2.32,<3`, `opencv-python>=4.10`, `numpy>=1.26` and
`pyserial>=3.5`. Only `lidar/` needs pyserial, only `oak_camera/` needs depthai,
and `usb_cameras/` needs neither — so a missing dependency stops one component, not
the suite. `driver_board/` needs nothing from the file at all over WiFi, and
pyserial only for its `--serial` path; `face_tracking/` is the same but wants
OpenCV as well, plus one 230 kB model file it downloads for itself on first run.
The OAK camera needs no driver install on Windows, and a game controller needs none
either: any pad Windows presents as XInput will do, and XInput is a DLL Windows
already has.

## Usage

Every script prints what it found on stdout, and all but `drive_gamepad.py` open
an OpenCV window that `q` quits.

```powershell
# OAK-D-Lite, in triage order: each step assumes the previous one worked
python oak_camera\probe_device.py         # boots the device, reports what it is
python oak_camera\inspect_calibration.py  # stored calibration, user vs factory
python oak_camera\read_crash_dump.py      # last firmware crash, then clears it
python oak_camera\preview_depth.py          # mono pair + stereo engine
python oak_camera\preview_depth.py --fade 0 # ...without the 1 s depth hold
python oak_camera\preview_rgb.py          # colour only
python oak_camera\preview_rgb.py --depth  # colour + aligned depth, all 3 sensors

# D500 lidar -- the rover's main power switch must be on
python lidar\lidar_view.py                # auto-detects the serial port
python lidar\lidar_view.py COM14          # or name it

# the host's USB cameras
python usb_cameras\preview_usb_cameras.py

# drive it -- rover powered on, pad plugged in
python driver_board\drive_gamepad.py         # over WiFi: the rover's AP, else this LAN
python driver_board\drive_gamepad.py --host 192.168.1.22   # straight to a known address
python driver_board\drive_gamepad.py --serial   # over USB, port auto-detected

# track a face with the pan/tilt -- rover powered on, its camera plugged in
python face_tracking\track_face.py             # finds the board and the camera itself
python face_tracking\track_face.py --no-move   # detect and draw, command nothing
python face_tracking\track_face.py --no-scan   # stay put when no one is in shot
```

Window keys, beyond `q`:

| Script | Keys |
|---|---|
| `preview_depth.py` | mouse position reads out the distance under the cursor |
| `preview_rgb.py --depth` | `m` cycles blend / depth / colour, `[` `]` blend weight, mouse reads distance |
| `lidar_view.py` | `[` `]` range, `c` colour by intensity or distance, `s` save a PNG |
| `preview_usb_cameras.py` | click or `n`/`p` to change camera, `a` re-apply auto, `s` driver settings |
| `track_face.py` | `c` recentre, space re-target the largest face, `h` hold position |

The first three OAK scripts open no streams, which is what makes them useful for
separating a device fault from a pipeline fault. `preview_rgb.py --depth` is the
heaviest load in the suite.

`drive_gamepad.py` has no window, so it reads the pad through XInput and does not
need focus — you can watch the rover rather than the screen. The triggers drive —
right forward, left back — and the right stick steers; the left stick aims the
pan/tilt camera and it stays where you leave it, with a click of the stick to
recentre. `Y` switches the white LEDs and the D-pad sides dim them, a tap to a
notch and a hold to fade — they are PWM, not a switch. RB is full speed, LB a
crawl, D-pad up/down moves the speed cap, and Back quits. Triggers and sticks are
all proportional, through a deadzone and an expo curve, so a small push is a slow
crawl rather than a lurch. `--no-gimbal` for a rover with no camera fitted.

Over WiFi it finds the board itself: the ESP32's own AP first, at 192.168.4.1,
and failing that every address on the LAN, asking each for base feedback until
one answers. That search exists because the firmware publishes no mDNS name and
sets no DHCP hostname, so a rover that has joined a home network is an anonymous
lease with nothing to look up. `--host` skips it once you know the address.

It stops the motors on the way out, and sets the firmware's heartbeat to 500 ms
first, so the rover also stops itself if the script dies or the link drops. That
failsafe is not the power switch.

## Face tracking (`face_tracking/`)

`track_face.py` is the one host-side script that runs two components against each
other: the rover's USB camera module supplies the picture, OpenCV's YuNet detector
finds a face in it, and the pan/tilt's two servos are steered to keep that face in
the middle of the frame. It never touches the wheels — the only command it sends
that moves anything is `{"T":134,...}`, which reaches the camera servos and nothing
else — and it leaves the firmware's heartbeat at its default, since that timer
exists to stop the base and this never starts it.

It finds both halves itself: the driver board as `drive_gamepad.py` does, and the
camera by its USB id (`0abd:8050`) among the machine's other webcams, with
`--camera` to name one. The detector's model is a 230 kB ONNX file OpenCV does not
ship, fetched once on the first run to sit beside the script — Haar cascades would
need no download, but OpenCV 5 dropped them from the wheel entirely.

Aiming is calibrated rather than assumed. Taking a patch of the scene, commanding a
known move and finding that patch again by template matching gives **9.65 px of
image shift per commanded degree in pan and 9.5 in tilt** at 1280×720, symmetric in
both directions, from which `+X` pans right and `+Y` tilts up. A half frame is
640 px, so 66 of those degrees. That first suggested the firmware's "degrees" were
about half a real one, on the grounds that no sane lens is 132° wide — but the
[firmware source](https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/gimbal_module.h)
maps them `×11.375` into the ST3215's 4096 counts per turn, so a commanded degree
*is* a real degree and the lens really is that wide. The barrel distortion in any
frame it takes confirms it. The controller works in the measured units either way,
and never needs the lens FOV — which is how the wrong inference survived so long.

### Dead time is what makes this hard

The loop is closed through the world but open around the servos, which report
nothing back, so the angles are a model kept true by centring at startup and on
exit. The number that governs everything else is the delay between a command going
out and the picture showing any sign of it: **266 ms**, measured over five 50°
steps, which at 30 fps is **eight frames**. Everything commanded inside that window
is still in flight and invisible.

A controller that simply corrects what it can see therefore issues the same
correction eight times over, and the result is not sluggishness but divergence —
the camera sails past the face, corrects harder the other way, and pins itself
against a limit. From outside it looks precisely like a camera avoiding people, and
that is what this did before the compensation went in. The fix is to correct from
where the camera *was when the frame was exposed*, so motion already in flight is
subtracted instead of commanded again. Measured against a fixed target at a −55°
offset:

| | error, start → settled |
|---|---|
| correcting against the current angle | 0.80 → 0.67, swinging the full frame, ended pinned at pan +180 |
| correcting against the angle at exposure | **0.80 → −0.01**, no overshoot |

On a live subject it now reaches centre in under a second and holds a lock for ten
seconds at a stretch, with nothing pinned at a limit. A sign error looks the same
from the outside as too much gain, so tell them apart by watching one correction
from a standstill: the wrong sign moves away immediately, too much gain moves the
right way first and overshoots.

### Scanning, and how fast it may sweep

With nobody in shot it sweeps its whole range — pan end to end, then a step of tilt
and back the other way — and follows the moment a face appears. Two tilt levels
cover the range because the frame takes in 76° of the 120° available, so levels a
half-frame inside each end reach both limits and overlap.

The sweep rate is the pacing question, and it is answered with measurements rather
than taste. Motion smear was calibrated by blurring a still frame until its
sharpness matched what the moving camera produced: **~2 px at 25°/s**, against a
detector that still finds a 50 px face (someone across a room) under 9 px of smear
and a 100 px face under 27. Faster detects perfectly well — 90°/s still finds faces
— but smear grows with exposure time and a dim room lengthens it, and the sweep is
visibly rougher above about this speed: at 25°/s the picture moves 8.0 px a frame
against the 7.5 expected with 1–4% of frames not moving at all, while at 45°/s it
delivers 11 px of an expected 13 and stalls on nearly a fifth. Slower is smoother
and sees no less. `--scan-rate` moves it; `--no-scan` stops it sweeping.

One trap worth recording: `dt` multiplies the sweep step, so a single slow frame
commands a large jump, the board takes longer to answer a large jump, and the next
frame is slower still. That spiral took the loop from 25 commands a second to 0.9.
Clamping `dt` breaks it, after which every rate tried held 25 fps with nothing lost.

Detection uses two thresholds, not one, because a false positive here is not
cosmetic — the camera locks onto the wrong thing and, unlike a person, a sofa never
walks away. In this room the arm of a black sofa against a yellow wall scored
**0.79**, while real faces scored 0.88–0.91 and a distant half-profile one 0.73. No
single threshold separates those, so acquiring a target needs 0.85 while keeping one
needs only 0.60 — and the low bar is safe because a weak detection is accepted only
close to where the face already was.

### Why the camera used to step from pose to pose

Both gimbal commands carry a speed, and the firmware passes it to the servo in the
servo's own units — `map(spd, 0, 360, 0, 4095)`, which is plain degrees per second.
Sending `SPD: 0` means *unlimited*, so every correction was "get there as fast as
you can": a 1° lunge at the servo's full 130°/s, over in 8 ms, followed by 25 ms of
standing still, thirty times a second. Naming the speed the motion actually wants
fixes it, and `T:134` is used rather than `T:133` because it takes the two axes
separately — otherwise a barely-moving tilt is dragged along at whatever pan needed.
Measured, to calibrate the units: `SPD` 20 gave 20.4°/s, 40 gave 40.2, 80 gave 75.3
and 150 gave 114.6, against a ceiling of about 130.

Two limits of the mechanism remain, and no amount of commanding gets around either.
The firmware truncates angles to whole degrees, so **the smallest possible move is
9.65 px** of picture — fractional angles are silently rounded, and half-degree steps
produce one jump per *pair* of commands. And there is **about 2° of backlash**: after
a change of direction the first two commanded degrees produce no motion at all, then
it tracks linearly again. The deadband sits just outside that, which is what stops
the camera dithering across it.

`--no-move` runs everything and commands nothing, which is the way to check the
picture and the detections before letting it move the camera; rejected detections
are drawn thin with their scores, so it shows what was seen and passed over.

## The integrated stack (`vm/`)

Both sensors at once, in ROS 2 Humble, building a 2D map while the rover is pushed
by hand. It runs in a VMware guest because the OAK's depthai stack and the ROS 2
packages need Linux; the sensors reach it over USB passthrough. The tree deploys
to `~/ugv` in the guest, and [`vm/README.md`](vm/README.md) covers running it.

    bash ~/ugv/bin/start_slam.sh rviz     # lidar + camera + SLAM + RViz
    bash ~/ugv/checks/slam.sh             # confirm it is really mapping

What is established, all measured rather than assumed: both sensors run at their
native rates (depth 15.1 Hz, scan 10.000 Hz, IMU ~200 Hz), and they agree about
where objects are to **−14.9 mm median, 17.7 mm RMS over 0.3–1.0 m**. The rover
has no wheel encoders, so odometry comes from `rf2o` scan matching — which is
adequate at translation and poor at rotation, inventing about 5°/min of heading
while completely stationary. An EKF fusing rf2o's translation with the OAK's
de-biased gyro brings that to **0.02 °/min**, a factor of 260.

That gyro needs continuous bias correction, not a single calibration: its offset
measured −0.044, −0.150 and −0.154 °/s on three consecutive startups and keeps
moving as the device warms. `vm/nodes/fusion_prep.py` re-learns it whenever the
rover can be shown to be standing still.

## Documentation

The measurements, hardware facts and failure modes live in [`docs/`](docs), one
document per component plus two for the OAK camera's constraints:

| Document | Covers |
|---|---|
| [oak-d-lite.md](docs/oak-d-lite.md) | what the board is, each of the five tools, depth semantics, the calibration oddity |
| [oak-usb-link.md](docs/oak-usb-link.md) | why every script pins USB2, what throughput the link allows, recovering a wedged device |
| [depthai-version-pin.md](docs/depthai-version-pin.md) | why depthai is pinned `<3`, the evidence, upstream issues |
| [d500-lidar.md](docs/d500-lidar.md) | power, data path, packet protocol, view orientation |
| [usb-cameras.md](docs/usb-cameras.md) | how cameras are probed and named, why a black frame is usually the pixel format, forcing auto controls |

Read the relevant one before concluding a component is dead. Several documented
failures look exactly like broken hardware and are not: a camera that will not open
with `io error` (USB3 firmware — pin `HIGH`), a run that captures every frame and
then segfaults at shutdown (a stored crash dump on depthai 3.x), a device that
stops being found at all (still booted, recovers on its own), and an empty lidar
window on a live COM port (the rover's power switch).
