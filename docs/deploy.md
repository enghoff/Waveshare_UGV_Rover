# Deploying this repository

Nothing here is rebuilt or synced automatically. Edit in the repo, push to
the host that runs the files, restart the service that loads them, and prove
it there. The rules around that are in [CLAUDE.md](../CLAUDE.md). This file
is the map: which directory lands where, and the commands that put it there.

The machines, addresses and keys are in [hosts.md](hosts.md). A component's
own README is the place for how it works and how to debug it.

## Where each directory runs

| Directory | Host | Lands at |
|---|---|---|
| `rover_daemon/`, `driver_board/`, `face_tracking/` | `bpi` | `~/ugv/` (flat for the daemon and the tracking loop; they import each other) |
| `lidar_slam/` | `bpi` | `~/ugv/lidar_slam/` — parser, map renderer, USB replug. Its SLAM and planner are deleted. |
| `ros_nav/` | `bpi` | `~/ugv/ros_nav/`, plus a conda environment at `~/miniforge3` built by its own `install.sh`. See [ros_nav/README.md](../ros_nav/README.md) |
| `oak_depth/` | `bpi` | `~/ugv/oak_depth/`, plus `vendor/` filled by its own `install.sh` |
| `wifi_roam/` | `bpi` | `~/ugv/wifi_roam/`, and from there into `/usr/local/sbin` and `/etc/systemd/system` by `install.sh` / `install-dual.sh` |
| `netwatch/` | `bpi` | `~/ugv/netwatch/`, and from there into `/usr/local/sbin`, `/usr/local/bin` and `/etc/systemd/system` by its own `install.sh`. `netprobe.py` is the desk half and is not deployed |
| `drive_web/` | `bpi` | `~/ugv/drive_web/`, plus copies of `voice_chat/{console_model,rover_tools,session,talk_frames,prompts,server}.py` next to it |
| `voice_chat/server.py`, `face_detect/` | `root@media` | `/opt/<service>/` |
| `lidar/`, `usb_cameras/`, `omni_bench/`, `voice_chat/mock_rover.py` | whichever desk is in use | nothing to deploy |

`scripting.py` and `rover_api.py` deploy flat with the daemon. If a behaviour
store ever appears on the rover, it is data, not source — a deploy must not
overwrite it. See [scripting.md](scripting.md).

The SSH host is `bpi-m4zero`. Reach the rover at **192.168.1.80** (the
service address `wifi_dual` moves between the two radios). `.139` and `.100`
are the interfaces' own DHCP leases. See [wifi_roam/README.md](../wifi_roam/README.md).

The drive console is `https://192.168.1.80:8771/`. See
[drive_web/README.md](../drive_web/README.md).

## Cross-cutting traps

**Use each service's `restart.sh`.** The pattern a `pkill` would match also
matches the ssh command carrying it, so typing the kill yourself takes down
the session that typed it. `restart.sh` exists so that pattern lives in a
file. It kills only the child; the supervisor brings it back with the
arguments from crontab (or the unit).

**Never relaunch `run_daemon.sh` by hand.** It drops the flags. The crontab
entry is `@reboot ~/ugv/run_daemon.sh --vision --board-bridge --ros-nav`.
Without `--ros-nav` the daemon offers no driving tools. Without
`--board-bridge` the ROS stack has no odometry. There is no `--lidar` any
more — argparse refuses it and the daemon will not start.

**A crontab change needs the supervisor replaced, and a `sync`.** The running
supervisor still holds the old arguments. This card is `commit=120`; a
crontab written and not flushed has already been lost to a restart here.

**`sh` in front of a script that arrived by `scp`.** The checkout is mode
644, so the shebang is never consulted and running the path directly fails
with "Permission denied" — which reads as a sudo problem and is not one.

**LF, not CRLF, on shell scripts.** `.py` files have no shebang, so `scp` is
fine. Scripts under `lidar_slam/`, `oak_depth/`, `wifi_roam/`, `netwatch/`,
`face_tracking/` and `drive_web/` do, and they are held to LF by
`.gitattributes`. A CRLF checkout turns the shebang into an interpreter with
a carriage return in its name.

**Vendor wheels, not pip.** This board's Debian has no `pip` and no
`python3-venv`. OpenCV and depthai are pinned wheels unpacked into `vendor/`
by `install_opencv.sh` and `oak_depth/install.sh`. Both are idempotent.

**ROS 2 lives outside `~/ugv`.** RoboStack is a conda environment at
`~/miniforge3` because a deploy overwrites `~/ugv`. `env.sh` **must be
sourced from bash** — RoboStack's hooks use `source`, which dash has not
got, and the failure names neither the file nor the shell.

**`~/.ugv/`, not `~/ugv/`, for secrets the rover must hold.**
`alibaba.key` (the rover's own conversation with Alibaba) and
`console.token` (gates the microphone only) live outside the deploy tree so
an `scp -r` cannot carry them back into the repository. `chmod 600` the key.

**`lidar_slam/` is mirrored, not copied into.** `scp` adds and never removes.
The directory lost its SLAM and planner; leftover files on the host are
stale. `libslam2d.so` is built per-host (aarch64) and is not committed — a
change to `slam2d.c` or `slam2d.h` needs a rebuild on the board.

**The OAK has one owner.** Stop `oak_depth` before running its selftest or
anything else against the camera.

## Banana Pi

```bash
scp rover_daemon/*.py bpi-m4zero:~/ugv/
scp face_tracking/*.py face_tracking/install_opencv.sh \
    face_tracking/face_detection_yunet.onnx bpi-m4zero:~/ugv/
rsync -a --delete --exclude 'libslam2d.so' --exclude selftest \
    lidar_slam/ bpi-m4zero:~/ugv/lidar_slam/
ssh bpi-m4zero 'cd ~/ugv/lidar_slam && ./build.sh && ./selftest | tail -2'
scp -r oak_depth bpi-m4zero:~/ugv/
scp -r wifi_roam bpi-m4zero:~/ugv/
scp -r netwatch bpi-m4zero:~/ugv/
scp -r ros_nav bpi-m4zero:~/ugv/
scp drive_web/*.py drive_web/*.html drive_web/*.sh drive_web/README.md \
    bpi-m4zero:~/ugv/drive_web/
scp voice_chat/{console_model,rover_tools,session,talk_frames,prompts,server}.py \
    bpi-m4zero:~/ugv/drive_web/

ssh bpi-m4zero 'cd ~/ugv && python3 selftest.py | tail -2'
ssh bpi-m4zero '~/ugv/restart.sh'                    # daemon; ~35 s; prints the tool count
ssh bpi-m4zero '~/ugv/oak_depth/restart.sh'          # ~10 s; prints /health
ssh bpi-m4zero '~/ugv/drive_web/restart.sh'          # prints /health
ssh bpi-m4zero '~/ugv/ros_nav/restart.sh'            # ~30 s; prints the node list
ssh bpi-m4zero '~/ugv/ros_nav/restart.sh --supervisor'  # when run_ros_nav.sh itself changed
```

Installers, once, or again after changing what they install:

```bash
ssh bpi-m4zero '~/ugv/install_opencv.sh'             # OpenCV 4.12, for YuNet
ssh bpi-m4zero '~/ugv/oak_depth/install.sh'          # depthai 2.32.0.0
ssh bpi-m4zero 'sh ~/ugv/drive_web/install.sh'       # crontab
ssh bpi-m4zero 'sh ~/ugv/drive_web/install_websockets.sh'
ssh bpi-m4zero 'sh ~/ugv/ros_nav/install.sh'         # ~20 min, 4.7 GB, no sudo
ssh bpi-m4zero 'sh ~/ugv/ros_nav/install-boot.sh --nav'
cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" sh ~/ugv/wifi_roam/install-dual.sh'
cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" sh ~/ugv/wifi_roam/install-dual.sh DUAL=on'
cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" sh ~/ugv/netwatch/install.sh'
```

`install-dual.sh` without `DUAL=on` copies files and leaves the manager off,
because the way it fails is a rover that needs carrying to a socket. Do not
run `wifi_roam/install.sh` on this board without `ROAM=off` — that installer
enables the roam timer, which fights the manager. Check with
`ssh bpi-m4zero 'cat /run/wifi-dual.json'` (no privilege needed) and
`python3 wifi_roam/test_wifi_dual.py` before treating a logic change as done.

`oak_depth` and `drive_web` each have their own supervisor. Reloading depth
is not the same act as reloading the daemon; reloading the console is not
either.

The daemon and the ROS stack start from the same crontab. The stack takes the
best part of a minute, and `--ros-nav` does not wait for it — each driving
tool connects when it is called and says when there is nothing to connect
to.

An empty map with a lidar that is reporting happily is usually the driver
board, not the lidar. See
[rover_daemon/README.md](../rover_daemon/README.md#when-the-board-does-not-answer).

## MEDIA

```bash
scp voice_chat/{server.py,voice_history.py,voice_stream.py,voice_http.py,requirements.txt,selftest.py,test_harness.py,test_server.py,test_talk.py} \
    root@media:/opt/voice_chat/
ssh root@media 'systemctl daemon-reload && systemctl restart voice-chat'
```

`voice-chat` takes ~150 s to come back — three models load and decode is
warmed before it binds — and `/health` answering is the signal that it is
ready. The three GPU services share one card and are mutually exclusive;
switch with `ssh root@media ~/switch_service.sh voice`. `face-detect` is on
the CPU and is not part of that trade. See [hosts.md](hosts.md).
