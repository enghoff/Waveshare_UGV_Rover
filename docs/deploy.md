# Deploying this repository

A commit changes nothing on the rover by itself. The repository remains the
source of truth: edit and commit here, deploy to the rover, restart the
affected service and prove the running system there. The working rules are in
[`CLAUDE.md`](../CLAUDE.md).

For ordinary committed work use [`deploy/deploy.py`](../deploy/README.md):

```bash
python deploy/deploy.py --plan
python deploy/deploy.py
```

The deployer compares each registered component with the commit last proved on
the rover, copies only affected source and advances state only after the existing
restart/readiness checks pass. The manual commands below remain the recovery path.

The current installation has one deploy target: the Jetson Orin Nano on the
rover (`orin`), which replaced the Banana Pi on 2026-08-31. The realtime Qwen
Omni model is Alibaba-hosted and has no repository deployment target.

## Where each directory runs

| Directory | Runs on | Lands at / role |
|---|---|---|
| `rover_daemon/`, `driver_board/`, `face_tracking/` | Jetson | flattened into `~/ugv/`; the daemon imports the board helpers and local YuNet/aiming code |
| `lidar_slam/` | Jetson | `~/ugv/lidar_slam/`; parser, room description, map renderer and USB reset support |
| `ros_nav/` | Jetson | `~/ugv/ros_nav/`; ROS 2 environment remains under `~/miniforge3` |
| `oak_depth/` | Jetson | `~/ugv/oak_depth/`; its unpacked DepthAI wheel remains in `vendor/` |
| `drive_web/` | Jetson | `~/ugv/drive_web/` |
| `world_state/` | Jetson | `~/ugv/world_state/`; the semantic store and the perception sidecar, with its models and TensorRT engines fetched or built into `vendor/` by its own installer |
| selected `voice_chat/` modules | Jetson | copied beside `drive_web` for the console/Alibaba realtime session |
| `wifi_roam/` | Jetson | staged at `~/ugv/wifi_roam/`, then privileged copies via its installer |
| `dongle_driver/` | Jetson | staged at `~/ugv/dongle_driver/`; its installer builds and DKMS-registers the USB radio's kernel module |
| `netwatch/` | Jetson | staged at `~/ugv/netwatch/` and **deliberately not installed** — it needs `wpa_supplicant`, which this host does not run |
| `oak_camera/`, `lidar/`, `usb_cameras/`, `face_tracking/track_face.py`, workstation `driver_board/` tools, `voice_chat/mock_rover.py` | desk/workstation | bench/diagnostic tools; no deploy |

The current `drive_web` deployment copies these shared voice modules beside the
web service:

```text
console_model.py
rover_tools.py
session.py
talk_frames.py
prompts.py
```

`session.py` is the Alibaba realtime protocol. There is no local GPU voice server
or remote face-detection service in the current deployment.

## Addresses and ports

The SSH host is `orin`. The rover answers on `192.168.1.80`, which is a fixed
address written into each of its three network profiles rather than anything that
moves or is managed -- so it is there whichever house network the rover is on.
The radio's own DHCP lease, `192.168.1.88` at the time of writing, and the name
`jetson-orin.local` both work and are the way back in if `.80` ever does not. See
[`hosts.md`](hosts.md) and [`wifi_roam/README.md`](../wifi_roam/README.md).

The browser console is:

```text
https://192.168.1.80:8771/
```

Main rover ports:

| Port | Owner |
|---:|---|
| 8769 | `rover_daemon` tool/control protocol |
| 8770 | `oak_depth` |
| 8771 | `drive_web` HTTPS/WebSocket console |
| 8772 | board bridge: daemon lends odometry/motors to ROS |
| 8773 | navigation bridge: ROS lends Nav2 back to daemon |
| 8774 | loopback image/frame handoff used by the Alibaba voice session |
| 8776 | `world_state` perception sidecar: the three ONNX models in a process of their own |

## First deployment state

The deployer refuses to invent a baseline. If the rover is already known to
match the checkout:

```bash
python deploy/deploy.py --adopt --host orin
```

If not, reconcile from source:

```bash
python deploy/deploy.py --full --host orin
```

Deployment state is per component in `~/.ugv/deploy-state.json`.

## Privileged network/system deployment

`wifi_roam`, `dongle_driver` and `netwatch` are deliberately two-stage. A normal
deploy may copy and test their source but does not replace `/usr/local`, systemd
files or kernel modules. After reviewing a privileged/network change:

```bash
python deploy/deploy.py --system --only wifi_roam
python deploy/deploy.py --system --only dongle_driver
python deploy/deploy.py --system --only netwatch
```

`dongle_driver` is idempotent and wants re-running after a kernel update, because
it builds an out-of-tree module. `netwatch` is staged and not installed here: it
is ordered after `wpa_supplicant` and reads that daemon's control socket, and
this host runs NetworkManager.

The deployer reads `secrets/jetson-orin.key` locally and feeds it to `sudo -S`. The
password is not put in the command line or copied to the rover. That file is the
`jetson` account's own login password, and is not either of the older boards'.
By hand, `-S` reads until EOF, so
one `cat` feeds exactly one `sudo`; two chained after a single `cat` leave the
second waiting with no password.

## Runtime state and secrets

Source deploys land under `~/ugv/`. Runtime state that must not be overwritten by
source lives elsewhere:

- `~/.ugv/alibaba.key` — Alibaba DashScope key for Qwen Omni;
- `~/.ugv/tls/` — console CA/leaf certificate material;
- `~/.ugv/deploy-state.json` — per-component deployment state;
- `~/.ugv/world/` — the semantic world state: `world.db` and one JPEG per
  inspection under `frames/`. An experiment's results, not source.

Keep these out of Git and out of `~/ugv`.

One piece of runtime state is *inside* `~/ugv` and survives because every
component that lands there is additive rather than mirrored:

- `~/ugv/odometry.json` — the chassis calibration.

### When the rover gets a new computer

No deploy carries any of the above, so a new host starts without them. The
distinction that decides what to do with each is whether it describes the
**chassis** or the **computer**.

`odometry.json` is the only one that describes the chassis. The gyro's scale
belongs to the IMU on the driver board, the ticks per metre to the wheel
encoders, and the PWM curves to the motors and the battery — none of which
changes when the computer does. **Copy it across rather than re-measuring it**,
which is what was done moving from the Banana Pi to the Orin on 2026-08-31:

```bash
scp <old-host>:~/ugv/odometry.json /tmp/odometry.json
scp /tmp/odometry.json orin:~/ugv/odometry.json
ssh orin '~/ugv/ros_nav/restart.sh --supervisor'
```

Without it `base_node.py` refuses to start rather than guess, and the failure
does not look like a missing file from the outside: Nav2 comes up, accepts
goals and never drives, while `slam_toolbox` discards every scan for want of
an `odom` -> `base_link` transform. The tell is in `ros_nav.log`, one line,
*no calibration from ~/ugv/odometry.json*. If the old host is genuinely gone,
`ros_nav/calibrate_chassis.py` measures it again by driving the rover.

Everything else describes the computer and is redone on the new one. The key
is copied from `secrets/`; the TLS material regenerates on first run, **and the
new CA then has to be trusted on the
workstation**, because it is a different CA rather than the old one moved. The
deployment state is written by `--full` or `--adopt`. The installed
dependencies — the ROS environment, the unpacked wheels, `v4l-utils`, the
host-built `libslam2d.so` — come from the installers listed under
[Manual deployment](#manual-deployment), and the `@reboot` entries from
`install-boot.sh` and `drive_web/install.sh`.

## Cross-cutting traps

**Use each component's `restart.sh`.** An unguarded `pkill` pattern typed over
SSH can match the SSH command carrying it. The restart scripts keep those
patterns in files and preserve the supervisor's arguments.

**Never relaunch `run_daemon.sh` by hand.** The current supervisor arguments are:

```text
--vision --board-bridge --ros-nav
```

Without `--ros-nav` the daemon has no driving backend; without `--board-bridge`
ROS has no odometry/motor path. The old direct-lidar daemon mode is gone.

**A changed supervisor needs replacement, not only a child restart.** For ROS,
changes to `run_ros_nav.sh`, `sweep.sh` or `dds.sh` are handled by
`restart.sh --supervisor` via the deploy manifest.

**Follow a crontab write with `sync`.** This was the Banana Pi's rule — its root
was mounted `commit=120` and recent writes had disappeared across an abrupt
restart. The Orin's NVMe root is `rw,relatime`, so the `sync` calls still in these
scripts are now cheap insurance rather than a requirement. Leave them.

**Shell scripts must remain LF.** `.gitattributes` pins scripts that carry a
shebang. The deployer also preserves Git's executable bit in the tar archive.

**Vendor wheels, not pip, on the rover.** OpenCV for local YuNet and DepthAI for
the OAK are pinned wheels unpacked into component `vendor/` directories by their
installers. The Orin has `pip` and could install both properly; the Banana Pi
before it had neither `pip` nor `python3-venv`. Unpacking is kept because the
version is the thing being pinned — DepthAI's wheel *is* the OAK's firmware — and
one install path that worked on both boards beats a second that works only here.

**Not everything the rover needs is a wheel.** Camera capture needs `v4l2-ctl`,
which the Ubuntu base image does not carry: `apt install v4l-utils`.

**ROS 2 lives outside the source deploy tree.** RoboStack is under
`~/miniforge3`; its environment scripts must be sourced from bash.

**The OAK has one owner.** Stop `oak_depth` before running another program that
opens the OAK.

**`lidar_slam/` is mirrored.** Its old SLAM/planner files have been removed, so
additive copying would leave dead source on the rover. The deployer mirrors that
directory while preserving the per-host `libslam2d.so` and `selftest` build
products.

## Manual deployment

Use this when recovering the deployment mechanism itself or deliberately working
through a component by hand. For normal work prefer `deploy/deploy.py`.

```bash
scp rover_daemon/*.py rover_daemon/*.sh driver_board/*.py orin:~/ugv/
scp face_tracking/*.py face_tracking/*.sh face_tracking/*.onnx orin:~/ugv/

rsync -a --delete --exclude 'libslam2d.so' --exclude selftest \
    lidar_slam/ orin:~/ugv/lidar_slam/
ssh orin 'cd ~/ugv/lidar_slam && ./build.sh && ./selftest | tail -2'

scp -r oak_depth orin:~/ugv/
scp -r ros_nav orin:~/ugv/
scp -r wifi_roam orin:~/ugv/
scp -r dongle_driver orin:~/ugv/
scp -r netwatch orin:~/ugv/

scp drive_web/*.py drive_web/*.html drive_web/*.sh drive_web/README.md \
    orin:~/ugv/drive_web/
scp voice_chat/{console_model,rover_tools,session,talk_frames,prompts}.py \
    orin:~/ugv/drive_web/
```

Then run the component's own verification/restart, for example:

```bash
ssh orin 'cd ~/ugv && python3 selftest.py | tail -2'
ssh orin '~/ugv/restart.sh'
ssh orin '~/ugv/oak_depth/restart.sh'
ssh orin '~/ugv/drive_web/restart.sh'
ssh orin '~/ugv/ros_nav/restart.sh'
```

One-time/repeatable installers:

```bash
ssh orin '~/ugv/install_opencv.sh'
ssh orin 'sh ~/ugv/oak_depth/install.sh'
ssh orin 'sh ~/ugv/drive_web/install.sh'
ssh orin 'sh ~/ugv/drive_web/install_websockets.sh'
ssh orin 'sh ~/ugv/ros_nav/install.sh'
ssh orin 'sh ~/ugv/ros_nav/install-boot.sh --nav'
ssh orin 'sudo -S -p "" sh ~/ugv/wifi_roam/install.sh' < secrets/jetson-orin.key
ssh orin 'sudo -S -p "" sh ~/ugv/dongle_driver/install.sh' < secrets/jetson-orin.key
```

`install_opencv.sh` also proves that `LocalDetector` can load after unpacking the
wheel. A copied file is still not final proof: verify the running service on the
rover after restart.
