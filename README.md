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
| [`vm/`](vm) | both sensors together: ROS 2, SLAM, sensor fusion | a Linux VM |

The first four are independent: any can be run with the other components
unplugged or unpowered, so a result from one never needs the others to be working.

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
`captures/`, `oak_camera/crash_dumps/`, `*.mp4`, `*.npy` and
`calibration_backup_*.json` — a crash dump describes one device's one crash, so
it is local evidence, not something to carry in the repo; saved lidar PNGs
(`lidar-<timestamp>.png`) are *not* ignored.

## Setup

One environment covers all four components. Run the scripts from the repository
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
pyserial only for its `--serial` path. The OAK camera needs no driver install on
Windows, and a game controller needs none either: any pad Windows presents as
XInput will do, and XInput is a DLL Windows already has.

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
```

Window keys, beyond `q`:

| Script | Keys |
|---|---|
| `preview_depth.py` | mouse position reads out the distance under the cursor |
| `preview_rgb.py --depth` | `m` cycles blend / depth / colour, `[` `]` blend weight, mouse reads distance |
| `lidar_view.py` | `[` `]` range, `c` colour by intensity or distance, `s` save a PNG |
| `preview_usb_cameras.py` | click or `n`/`p` to change camera, `a` re-apply auto, `s` driver settings |

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
