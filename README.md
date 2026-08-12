# Waveshare UGV Rover

A suite of host-side tools for exercising and validating the components of a
**Waveshare UGV Rover** — the rover platform built on Waveshare's *General Driver
for Robots* board, with a Raspberry Pi as host and an ESP32 on the driver board —
**one at a time, in isolation**. Each script drives exactly one thing — one camera
socket, one sensor, one stored blob — so that when something is wrong you can
establish which component is at fault before any of them are combined into a robot
that does something. There is no integrated application here and no
autonomy: these are bring-up, bench-test and triage instruments.

Three families of hardware are covered, one directory each:

| Directory | Component | Needs |
|---|---|---|
| [`oak_camera/`](oak_camera) | Luxonis OAK-D-Lite depth camera, over USB | depthai |
| [`lidar/`](lidar) | D500 lidar, over serial via the rover's driver board | pyserial |
| [`usb_cameras/`](usb_cameras) | the host machine's own UVC webcams | OpenCV only |

The three groups are independent: any of them can be run with the other components
unplugged or unpowered, so a result from one never needs the others to be working.
Nothing here talks to the rover's motors or its ESP32 — this is sensing only. Every
script is standalone, needs no arguments, shares no state, imports nothing from the
others, and quits on `q`.

## Repository layout

```
oak_camera/
    probe_device.py          whole device   does it boot, and what is on it
    inspect_calibration.py   whole device   intrinsics and distortion, user vs factory
    read_crash_dump.py       whole device   read out and clear the firmware crash dump
    preview_depth.py         CAM_B + CAM_C  stereo depth with a distance readout
    preview_rgb.py           CAM_A          colour, and with --depth all three sensors
    crash_dumps/             JSON dumps written by read_crash_dump.py, kept
lidar/
    lidar_view.py            whole sensor   top-down view of the point cloud
usb_cameras/
    preview_usb_cameras.py   one at a time  cycle through the host's USB cameras
docs/                      the detail: hardware facts, measurements, failure modes
requirements.txt           four pins; the depthai one is deliberate, see docs
.cache/depthai/            depthai's own crash-dump cache, written by the library
```

A component's directory holds everything belonging to it, output included — which
is why `crash_dumps/` sits under `oak_camera/` rather than at the top level.
`.cache/` is depthai's, created relative to the working directory, so it appears
wherever you run from. `.gitignore` excludes `.venv/`, `__pycache__/`, `*.pyc`,
`captures/`, `*.mp4`, `*.npy` and `calibration_backup_*.json`; saved lidar PNGs
(`lidar-<timestamp>.png`) are *not* ignored.

## Setup

One environment covers all three components. Run the scripts from the repository
root, by path:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe oak_camera\probe_device.py
```

Requirements are `depthai>=2.32,<3`, `opencv-python>=4.10`, `numpy>=1.26` and
`pyserial>=3.5`. Only `lidar/` needs pyserial, only `oak_camera/` needs depthai,
and `usb_cameras/` needs neither — so a missing dependency stops one component, not
the suite. The OAK camera needs no driver install on Windows.

## Usage

Every script opens an OpenCV window and prints what it found on stdout. `q` quits.

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

## Documentation

The measurements, hardware facts and failure modes live in [`docs/`](docs), one
document per component plus two for the OAK camera's constraints:

| Document | Covers |
|---|---|
| [oak-d-lite.md](docs/oak-d-lite.md) | what the board is, each of the five tools, depth semantics, the calibration oddity |
| [oak-usb-link.md](docs/oak-usb-link.md) | why every script pins USB2, what throughput the link allows, recovering a wedged device |
| [depthai-version-pin.md](docs/depthai-version-pin.md) | why depthai is pinned `<3`, the evidence, upstream issues |
| [d500-lidar.md](docs/d500-lidar.md) | power, data path, packet protocol, view orientation |
| [usb-cameras.md](docs/usb-cameras.md) | how cameras are probed, forcing auto controls, what the status words mean |

Read the relevant one before concluding a component is dead. Several documented
failures look exactly like broken hardware and are not: a camera that will not open
with `io error` (USB3 firmware — pin `HIGH`), a run that captures every frame and
then segfaults at shutdown (a stored crash dump on depthai 3.x), a device that
stops being found at all (still booted, recovers on its own), and an empty lidar
window on a live COM port (the rover's power switch).
