# Waveshare UGV Rover

Tools for bringing up, driving and instrumenting a **Waveshare UGV Rover** — the
rover platform built on Waveshare's *General Driver for Robots* board, with an
ESP32 on the board and a Raspberry Pi as its host.

Half the repository is bench instruments. Each of those scripts drives exactly
one thing — one camera socket, one sensor, one stored blob — so that when
something is wrong you can establish which component is at fault before any of
them are combined into a robot that does something. They remain the right thing
to reach for first when a component misbehaves.

The other half is the rover actually doing something: a daemon on the Pi that
owns the hardware and hands it out as tools, a face detector and a voice
assistant that call those tools, a ROS 2 stack that fuses both sensors into a
map, and a much smaller SLAM written in C that fits on the rover's own Pi — which
matters because the ROS 2 stack, for all that it is better, runs in a VM that
cannot reach the rover.

| Directory | What it drives | Runs on | Needs |
|---|---|---|---|
| [`oak_camera/`](oak_camera) | Luxonis OAK-D-Lite depth camera, over USB | a workstation | depthai |
| [`lidar/`](lidar) | D500 lidar, over serial via the driver board | a workstation | pyserial |
| [`usb_cameras/`](usb_cameras) | the machine's own UVC webcams | a workstation | OpenCV only |
| [`driver_board/`](driver_board) | the ESP32 that drives the motors, over WiFi or USB | a workstation | nothing |
| [`face_tracking/`](face_tracking) | the pan/tilt camera and its two servos, as one loop | a workstation, or the Pi | OpenCV |
| [`face_detect/`](face_detect) | that loop's detector, as an HTTP service | any Linux box with spare CPU | OpenCV |
| [`rover_daemon/`](rover_daemon) | one owner of the board and the camera, as tools over TCP | the Pi | pyserial |
| [`lidar_slam/`](lidar_slam) | the lidar as a pose, a map, and a rover that drives itself | the Pi | a C compiler |
| [`voice_chat/`](voice_chat) | speech in, speech out, with the rover's tools attached | a Linux host with an 8 GB GPU | PyTorch |
| [`vm/`](vm) | both sensors together: ROS 2, SLAM, sensor fusion | a Linux VM | ROS 2 Humble |

The first four are independent: any can be run with the other components
unplugged or unpowered, so a result from one never needs the others to be
working. [`face_tracking/`](face_tracking) is the exception among the bench
scripts, being the one that closes a loop between two components, so reach for it
after both halves have been checked on their own.

Only [`driver_board/`](driver_board) makes the rover move. Every bench script is
standalone, needs no arguments, shares no state, imports nothing from the others,
and quits on `q` — `drive_gamepad.py` being the exception on the last two counts,
since it takes a controller rather than a window and stops on the pad's Back
button.

## Layout

```
oak_camera/     probe the device, read its calibration and crash dumps, preview
                depth and colour — five tools, in triage order
lidar/          lidar_view.py, a top-down view of the point cloud
usb_cameras/    preview_usb_cameras.py, cycling through the host's UVC cameras
driver_board/   drive_gamepad.py, teleop from a game pad, no Pi involved
face_tracking/  the control law (aiming.py) and the two programs that run it,
                one on a workstation and one on the rover
face_detect/    YuNet behind an HTTP request: JPEG in, boxes out, on the CPU
rover_daemon/   lights, gimbal and face tracking as tools over TCP
lidar_slam/     scan matching and an occupancy grid in C, sized for the rover's Pi
voice_chat/     Whisper + a vision-language model + Kokoro, a desktop client, and
                a window that drives the rover with no model in the loop
vm/             both sensors in ROS 2: bringup, SLAM, checks and provisioning
docs/           the detail — hardware facts, measurements, failure modes
```

A component's directory holds everything belonging to it, output included, which
is why `crash_dumps/` sits under `oak_camera/` rather than at the top level.
`.cache/` is depthai's, created relative to the working directory, so it appears
wherever you run from. A downloaded model is a dependency rather than source and
re-fetching it costs one run, and a crash dump describes one device's one crash,
so both are ignored; saved lidar PNGs are not.

## Setup

One environment covers every component that runs on the workstation. Run the
scripts from the repository root, by path:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe oak_camera\probe_device.py
```

Requirements are `depthai>=2.32,<3`, `opencv-python>=4.10`, `numpy>=1.26` and
`pyserial>=3.5`. Only `lidar/` needs pyserial, only `oak_camera/` needs depthai,
and `usb_cameras/` needs neither, so a missing dependency stops one component
rather than the suite. The depthai upper bound is deliberate and
[documented](docs/depthai-version-pin.md). The OAK camera needs no driver install
on Windows.

The three service components have their own environments and their own setup,
described in their own READMEs: [`face_detect/`](face_detect/README.md),
[`voice_chat/`](voice_chat/README.md) and [`rover_daemon/`](rover_daemon/README.md).
[`vm/`](vm/README.md) is provisioned by the scripts in `vm/setup/`.

## Usage

Every bench script prints what it found on stdout, and all but
`drive_gamepad.py` open an OpenCV window that `q` quits.

```powershell
# OAK-D-Lite, in triage order: each step assumes the previous one worked
python oak_camera\probe_device.py         # boots the device, reports what it is
python oak_camera\inspect_calibration.py  # stored calibration, user vs factory
python oak_camera\read_crash_dump.py      # last firmware crash, then clears it
python oak_camera\preview_depth.py        # mono pair + stereo engine
python oak_camera\preview_rgb.py --depth  # colour + aligned depth, all 3 sensors

# D500 lidar -- the rover's main power switch must be on
python lidar\lidar_view.py                # auto-detects the serial port

# the host's USB cameras
python usb_cameras\preview_usb_cameras.py

# drive it -- rover powered on, pad plugged in
python driver_board\drive_gamepad.py

# track a face with the pan/tilt -- rover powered on, its camera plugged in
python face_tracking\track_face.py
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

## The rest of the stack

[`rover_daemon/`](rover_daemon/README.md) is the one process allowed to own the
Pi's UART and camera, because the rover's hardware does not divide: two programs
that both want to command servos or look through the lens are two programs
corrupting each other. It exposes headlights, the gimbal and face tracking as
tools over TCP, and publishes their schemas so no client carries a copy.
[`voice_chat/`](voice_chat/README.md) is the client that matters — speech in,
speech out, with those tools attached — and [`face_detect/`](face_detect/README.md)
is the detector both tracking loops call, deliberately on a CPU so that the rover
does not stop seeing while somebody is talking to it.

`voice_chat/drive_console.py` is the same daemon with the model taken out: a
window of buttons for the driving tools, the navigator's own numbers polled beside
them, and the lidar map on screen. It is there because a conversation cannot
measure a move — a model asked to turn ninety degrees reports what it believed
happened, and what you need is what the navigator returned next to what you asked
for. `python voice_chat\mock_rover.py --drive` gives it an invented room to drive
in when there is no rover to hand.

[`vm/`](vm/README.md) is the opposite in kind to everything above and shares no
code with it: both sensors at once in ROS 2 Humble, building a 2D map while the
rover is pushed by hand. It runs in a Linux VM because the depthai stack and the
ROS 2 packages need Linux, and the sensors reach it over USB passthrough. What is
established there is measured rather than assumed — the two sensors agree about
where objects are to −14.9 mm median over 0.3–1.0 m, and since the rover has no
wheel encoders, an EKF fusing scan-matched translation with the camera's de-biased
gyro cuts stationary heading drift by a factor of 260.

## Documentation

The measurements, hardware facts and failure modes live in [`docs/`](docs).

| Document | Covers |
|---|---|
| [oak-on-the-pi.md](docs/oak-on-the-pi.md) | why this camera is not on the rover: the firmware upload, the wheel that does not exist for armv6, and the 5 V rail |
| [oak-d-lite.md](docs/oak-d-lite.md) | what the board is, each of the five tools, depth semantics, the calibration oddity |
| [oak-usb-link.md](docs/oak-usb-link.md) | why every script pins USB2, what throughput the link allows, recovering a wedged device |
| [depthai-version-pin.md](docs/depthai-version-pin.md) | why depthai is pinned `<3`, the evidence, upstream issues |
| [d500-lidar.md](docs/d500-lidar.md) | power, data path, packet protocol, view orientation |
| [usb-cameras.md](docs/usb-cameras.md) | how cameras are probed and named, why a black frame is usually the pixel format |
| [driver-board.md](docs/driver-board.md) | the gamepad controls, how the ESP32 is found, what the heartbeat failsafe does and does not cover |
| [face-tracking.md](docs/face-tracking.md) | the calibration, the 266 ms of dead time that makes it hard, the sweep, the servo's own limits |
| [vm-usb.md](docs/vm-usb.md) | how a sound card on a shared virtual hub kept killing the lidar, and the three-part fix |
| [hosts.md](docs/hosts.md) | the machines this rover shares work with here — a local-setup document, not a general one |
| [scaling-voice-chat.md](docs/scaling-voice-chat.md) | why batch-1 decode is bandwidth-bound, which GPUs are worth it, rent vs buy |
| [omni-architecture.md](docs/omni-architecture.md) | a clean-sheet design around one omni model: always-on sessions, barge-in, the safety supervisor |
| [omni-build.md](docs/omni-build.md) | the costed version of that design: what survives, what to write, and what to do first |

The last four are planning and local-setup documents, and are specific to one
installation rather than general. The rest describe the hardware and the code.

Read the relevant one before concluding a component is dead. Several documented
failures look exactly like broken hardware and are not: a camera that will not
open with `io error` (USB3 firmware — pin `HIGH`), a run that captures every
frame and then segfaults at shutdown (a stored crash dump on depthai 3.x), a
device that stops being found at all (still booted, recovers on its own), and an
empty lidar window on a live COM port (the rover's power switch).
