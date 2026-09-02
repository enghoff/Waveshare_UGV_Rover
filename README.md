# Waveshare UGV Rover

This repository is the source of truth for the software running on a Waveshare
UGV Rover built around the General Driver for Robots board and an NVIDIA Jetson
Orin Nano. It also contains small bench tools used to bring up and diagnose the
individual sensors and actuators.

The current system is deliberately simple about where work happens:

- the **Jetson Orin Nano** owns the rover hardware and runs the daemon, local
  YuNet face detection, ROS 2 mapping/navigation, the OAK depth service, the
  semantic world state and its perception sidecar, the web console and the
  network scripts;
- a **browser** supplies the microphone and speaker for voice interaction;
- **Alibaba DashScope** supplies the realtime Qwen Omni model;
- no separate GPU/MEDIA host is part of the running system, and nothing that is
  deployed uses the Orin's own GPU either.

The Orin replaced a Banana Pi M4 Zero on 2026-08-31. The chassis, driver board,
lidar and gimbal camera all came across unchanged; what moved is the computer
they are wired to, and with it every device name. Constants in this repository
that were "measured on the rover" were measured through one of the earlier
boards, and the mechanics are the same rover, so they stand.
[`docs/hosts.md`](docs/hosts.md) records the current host.

Superseded implementations are not kept beside the live ones merely as history.
Git already holds that history. A file in the current tree should either run,
help diagnose what runs, or document a current hardware fact or failure mode.

## What runs on the rover

| Directory | Current role |
|---|---|
| [`rover_daemon/`](rover_daemon) | owns the driver-board UART and gimbal camera; exposes hardware and navigation as tools on TCP 8769 |
| [`face_tracking/`](face_tracking) | shared aiming law plus **local YuNet** detection on the rover; the rover daemon imports this code |
| [`ros_nav/`](ros_nav) | ROS 2 Jazzy, `slam_toolbox` and Nav2; lidar in, odometry/motor commands through the daemon, navigation back to it |
| [`world_state/`](world_state) | what the rover has seen in the room, kept apart from where things are: segmented regions, the picture each look was read from, and a search that compares a description against what was actually seen |
| [`lidar_slam/`](lidar_slam) | the fast LD19 parser, room description, map renderer and USB recovery code still used by the ROS stack and daemon |
| [`oak_depth/`](oak_depth) | keeps the OAK-D-Lite open as a stereo depth sensor and serves depth locally |
| [`drive_web/`](drive_web) | HTTPS browser console, map, camera view and microphone/speaker bridge |
| [`voice_chat/`](voice_chat) | Alibaba realtime Qwen Omni session protocol, rover client helpers, prompts and console model shared with `drive_web` |
| [`wifi_roam/`](wifi_roam) | the rover's three NetworkManager profiles -- one it joins at boot, two it joins only when asked -- and the privileged helper the console scans and switches through |
| [`dongle_driver/`](dongle_driver) | builds and DKMS-registers the USB Wi-Fi dongle's kernel driver, which NVIDIA's L4T kernel does not ship |
| [`netwatch/`](netwatch) | persistent evidence for network/board failures; **staged but not installed** on the Orin, because it reads `wpa_supplicant`'s control socket and this host runs NetworkManager |

The driver board is the physical owner of the motors, lights, gimbal, encoders,
IMU and battery telemetry. The daemon keeps that UART open and lends the ROS stack
the odometry and motor path over loopback rather than letting two processes race
for the serial port.

Face tracking uses `face_tracking/yunet.py` on the rover itself. There is no
remote face-detection service in the current system. The detector and the aiming
loop are separate concerns: `yunet.py` finds faces; `aiming.py` decides where the
gimbal should move.

Voice interaction is also one current path. `drive_web/omni_bridge.py` runs the
session on the rover and `voice_chat/session.py` speaks Alibaba's realtime API.
Audio crosses the rover's Wi-Fi between browser and rover; tool calls stay on
loopback; a `look` frame is handed to the same cloud session through a loopback
frame server. See [`voice_chat/README.md`](voice_chat/README.md).

## Bench and diagnostic tools

These are intentionally kept even though they are not long-running rover
services. Each answers a useful question about the hardware without requiring the
whole stack to be healthy.

| Directory | What it is for |
|---|---|
| [`oak_camera/`](oak_camera) | probe the OAK, inspect calibration/crash state and preview colour/depth on a workstation |
| [`lidar/`](lidar) | live top-down lidar view from a desk |
| [`usb_cameras/`](usb_cameras) | UVC camera preview plus lens/FOV and aiming calibration |
| [`driver_board/`](driver_board) | direct gamepad/board bring-up tools |
| [`face_tracking/track_face.py`](face_tracking/track_face.py) | workstation face-tracking loop using the same YuNet/aiming model |
| [`voice_chat/mock_rover.py`](voice_chat/mock_rover.py) | invented rover for exercising the console and conversation plumbing without hardware |
| diagnostic scripts in [`ros_nav/`](ros_nav) | recordings, replay, controller simulations and chassis calibration used to reproduce navigation faults before changing the real rover |

A diagnostic remains worth keeping when it can answer a current question such as
"is the lidar producing valid packets?", "does YuNet see this face?" or "does
this controller reproduce the recorded doorway fault?". Historical alternatives
that no longer answer a current question belong in Git history instead.

## Workstation setup

One environment covers the ordinary workstation bench scripts:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Examples:

```powershell
python oak_camera\probe_device.py
python oak_camera\preview_depth.py
python lidar\lidar_view.py
python usb_cameras\preview_usb_cameras.py
python driver_board\drive_gamepad.py
python face_tracking\track_face.py
```

The rover does not use this venv. Its OpenCV and DepthAI dependencies are pinned
wheels unpacked beside the code by the component installers. The Orin does have
`pip` and could install them properly -- the Banana Pi before it had neither
`pip` nor `python3-venv` -- but the version is the thing being pinned, DepthAI's
wheel *is* the OAK's firmware, and one install path that worked on both boards is
worth more than a second that works only here. See
[`docs/deploy.md`](docs/deploy.md).

## Deploying the rover

A commit changes nothing on the rover until it is deployed. Normal committed-code
workflow:

```bash
python deploy/deploy.py --plan
python deploy/deploy.py
```

The deployer copies only affected registered components, uses their existing
restart/verification paths and advances per-component deployment state only after
that proof succeeds. Privileged network installs are deliberately a separate
`--system` step.

See:

- [`deploy/README.md`](deploy/README.md) for deployer behaviour and failure semantics;
- [`docs/deploy.md`](docs/deploy.md) for what runs where and the manual recovery path;
- [`docs/hosts.md`](docs/hosts.md) for this rover's host/network facts;
- [`CLAUDE.md`](CLAUDE.md) for working rules in this repository.

## Current data paths

### Driving and mapping

```text
D500 lidar -> ros_nav/lidar_node.py -> /scan -> slam_toolbox + Nav2  
                                                    |
driver board UART <- rover_daemon <- loopback 8772 -+
       ^                                            |
       +-------------- motor commands --------------+

Nav2 result/status -> loopback 8773 -> rover_daemon -> tools / web console
```

`lidar_slam/` keeps its historical name, but its old scan matcher/planner/controller
are gone. What remains is still used: the C parser, room-description helpers, map
renderer and USB reset path.

### Face tracking

```text
gimbal UVC camera -> MJPEG -> local YuNet -> aiming.py -> gimbal command -> driver board
```

The camera is kept as MJPEG because the host can decode a frame cheaply and
uncompressed capture needlessly consumes the shared USB path. Tracking runs while
the rover drives; with nobody in view it stops sweeping and watches the way it is
going until the wheels stop. The measured detector details and calibration
procedure are in [`docs/face-tracking.md`](docs/face-tracking.md).

### Voice

```text
browser mic/speaker
        |
        |  wss://rover:8771/audio
        v
 drive_web/omni_bridge.py
        |
        +-- 127.0.0.1:8769 -> rover tools
        +-- 127.0.0.1:8774 <- camera frames for `look`
        |
        +-- wss://dashscope-intl.aliyuncs.com/... -> Qwen realtime Omni
```

The DashScope key lives on the rover at `~/.ugv/alibaba.key`, outside the deploy
tree. The browser microphone is separately gated by `~/.ugv/console.token`.
Neither belongs in Git.

## Documentation

`docs/` is for current hardware facts, deployment instructions and focused
investigations whose evidence remains useful to the current system. Component
READMEs describe the component as it exists now.

Useful starting points:

| Document | Covers |
|---|---|
| [`docs/deploy.md`](docs/deploy.md) | deployment, restart and verification paths |
| [`docs/hosts.md`](docs/hosts.md) | current Jetson Orin, network facts, services and ports |
| [`docs/face-tracking.md`](docs/face-tracking.md) | local YuNet, aiming geometry and calibration |
| [`docs/driver-board.md`](docs/driver-board.md) | what the ESP32 board owns, and driving it from a game pad |
| [`docs/d500-lidar.md`](docs/d500-lidar.md) | lidar power/data/protocol facts |
| [`docs/oak-d-lite.md`](docs/oak-d-lite.md) | OAK-D-Lite hardware and depth semantics |
| [`docs/depthai-version-pin.md`](docs/depthai-version-pin.md) | why the rover pins DepthAI 2.x |
| [`docs/usb-cameras.md`](docs/usb-cameras.md) | UVC cameras from a workstation, and measuring a lens's field of view |
| [`docs/i2c.md`](docs/i2c.md) | why the 40-pin header's I2C is the ESP32's bus and not a free one |
| [`docs/jetson-orin-navigation.md`](docs/jetson-orin-navigation.md) | why Waveshare's Orin navigation stack was not adopted wholesale |
| [`docs/doorway-pivot.md`](docs/doorway-pivot.md) | a focused navigation-fault investigation; current config in `ros_nav/config/` remains authoritative |
| [`docs/rover-unresponsive.md`](docs/rover-unresponsive.md) | why the rover disappears from the network, and how to get in without the power switch |
| [`docs/scripting.md`](docs/scripting.md) | rover-side scripts exposed through the daemon |
| [`docs/task-semantic-world-state.md`](docs/task-semantic-world-state.md) | the plan the semantic world state is built to, and what replaying real runs found |
| [`docs/cosmos-reason2-integration.md`](docs/cosmos-reason2-integration.md) | closed: the local vision-language model that was tried and removed, and the measurements that ruled it out |

When a document disagrees with executable code or configuration, the code/config
is authoritative and the document should be corrected rather than the runtime
changed to match history.
