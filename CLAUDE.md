# Working in this repository

## Deploy every change to the host that runs it

**Nothing in this repository runs on the Windows workstation.** The rover code runs
on the Banana Pi and the model services run on MEDIA. Nothing is synced or rebuilt
automatically, so a change that has only been committed exists nowhere but the
repo — the rover goes on running the old code.

A change is therefore not finished when the self-tests pass locally. **Work out
which hosts the changed files run on, push to each, restart what needs restarting,
and verify it there — as part of the same piece of work, without being asked.**
Say in the report which hosts were deployed to and what was checked on them.

The repo stays source of truth: edit here and push, never edit in place on a host.

## Where each directory runs

| Directory | Host | Lands at |
|---|---|---|
| `rover_daemon/`, `driver_board/`, `face_tracking/` | `bpi` | `~/ugv/` (flat for the daemon; others mirror their repo path) |
| `lidar_slam/` | `bpi` | `~/ugv/lidar_slam/` — the lidar's C *parser*, the map renderer and the USB replug, all of which `ros_nav/` and the daemon reuse. Its SLAM, planner and drive controller are deleted, not merely unused |
| `ros_nav/` | `bpi` | `~/ugv/ros_nav/`, plus a conda environment at `~/miniforge3` built by its own `install.sh`. ROS 2 mapping and navigation; see [ros_nav/README.md](ros_nav/README.md) |
| `oak_depth/` | `bpi` | `~/ugv/oak_depth/`, plus `vendor/` filled by its own `install.sh` |
| `wifi_roam/` | `bpi` | `~/ugv/wifi_roam/`, and from there into `/usr/local/sbin` and `/etc/systemd/system` by its own `install.sh` |
| `netwatch/` | `bpi` | `~/ugv/netwatch/`, and from there into `/usr/local/sbin`, `/usr/local/bin` and `/etc/systemd/system` by its own `install.sh`. `netprobe.py` is the desk half and is not deployed |
| `behaviours/` | `bpi` | `~/ugv/behaviours/` — **planned, not built**; see [docs/scripting.md](docs/scripting.md). `scripting.py` and `rover_api.py`, which run scripts, deploy flat with the daemon; the agent-written store must never be overwritten by a deploy |
| `voice_chat/server.py`, `face_detect/` | `root@media` | `/opt/<service>/` |
| `drive_web/`, plus `voice_chat/console_model.py` and `voice_chat/rover_tools.py` | `bpi` | `~/ugv/drive_web/` |
| `lidar/`, `usb_cameras/`, `omni_bench/`, `voice_chat/mock_rover.py` | whatever desk is in use | nothing to deploy |

The drive console is hosted on the rover (`http://<rover>:8771/`). See
[drive_web/README.md](drive_web/README.md).

See [docs/hosts.md](docs/hosts.md) for what these machines are, their addresses and
their keys. The SSH host is `bpi-m4zero`.

## Credentials are in `secrets/`, so use them

Every password and token this repository needs is a one-line file in `secrets/`,
which is gitignored and exists only on the workstation. `bpi-sudo.key` is `admin`'s
password on the Banana Pi that is the rover now and `rpi-sudo.key` is the same
account's password on the Raspberry Pi it replaced — **the two are different, and
the Pi's is silently refused by the Banana Pi**, which reads as a board that has
lost its password rather than as the wrong file. `wifi.key` is the passphrase the
three house networks share, and `runpod.key` and `alibaba.key` are the API keys for
those accounts.

**Read the file rather than stopping to ask for it.** A good deal of the work here
needs root on the Banana Pi — the systemd units under `wifi_roam/`, anything under
`/etc`, and any scan or network switch — and `sudo` there asks for a password
rather than being passwordless. A deploy that stops at "somebody will have to type
this in" has not been deployed. Feed it over stdin, which keeps it out of both
shells' history and out of `ps` on the rover:

```bash
cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" ~/ugv/wifi_roam/install.sh'
```

One password per `sudo`, because `-S` reads until end of file: two `sudo -S` calls
chained after one `cat` leaves the second with nothing and it fails as "no password
was provided", which looks like the wrong password and is not. Pipe it once per
command, or give the remote side a small script to run under a single `sudo`.

Use them, but keep them where they are: none of these belongs in a commit, in a
chat transcript, on a command line where `ps` can see it, or copied onto a host.

## The Banana Pi

```bash
scp rover_daemon/*.py bpi-m4zero:~/ugv/     # includes ros_navigator.py, the
                                            # client half of the nav bridge
# lidar_slam/ lost eleven files when its SLAM and planner went; scp adds and never
# removes, so mirror it rather than copying into it, then rebuild -- libslam2d.so is
# per-host and a stale one now has the wrong struct layout
rsync -a --delete --exclude 'libslam2d.so' --exclude selftest lidar_slam/ bpi-m4zero:~/ugv/lidar_slam/
ssh bpi-m4zero 'cd ~/ugv/lidar_slam && ./build.sh && ./selftest | tail -2'
scp face_tracking/*.py face_tracking/install_opencv.sh     face_tracking/face_detection_yunet.onnx bpi-m4zero:~/ugv/   # the tracking loop and its detector
scp -r oak_depth bpi-m4zero:~/ugv/           # the OAK as a depth camera; then its own install.sh
scp -r wifi_roam bpi-m4zero:~/ugv/           # the wifi keeper; then its own install.sh
scp -r netwatch bpi-m4zero:~/ugv/            # the network recorder; then its own install.sh
ssh bpi-m4zero 'cd ~/ugv && python3 selftest.py | tail -2'
ssh bpi-m4zero '~/ugv/restart.sh'          # ~35 s; prints the new tool count
```

Both `restart.sh` scripts exist so that the pattern they kill on lives in a file
rather than on an ssh command line, where it would match -- and kill -- the very
session that typed it.

`restart.sh` kills only the daemon and lets the supervisor restart it, because the
crontab entry — `@reboot ~/ugv/run_daemon.sh --vision --board-bridge`, beside
`@reboot ~/ugv/oak_depth/run_oak_depth.sh` — is where the
arguments live. **Never relaunch `run_daemon.sh` by hand**; it drops the flags and
the rover silently loses tools.

**There is no `--lidar` any more.** The daemon used to be able to drive with its
own planner, holding the lidar's serial port itself; that planner has been deleted
and the flag with it, because the lidar belongs to `ros_nav/` and only one process
can hold a serial port -- the daemon would win it silently and `slam_toolbox` would
sit waiting for a scan that never comes. What the entry says is `--board-bridge
--ros-nav`, and the pair is the whole interface between the two halves of this
rover, running in opposite directions:

- `--board-bridge` lends the driver board's encoders, gyro and motor commands to
  the ROS stack over loopback TCP 8772, while keeping the UART, the lights, the
  gimbal and the pack voltage. See
  [rover_daemon/board_bridge.py](rover_daemon/board_bridge.py).
- `--ros-nav` borrows navigation back over loopback TCP 8773, so that `drive`,
  `drive_to`, `turn_in_place`, `show_map` and the rest are offered at all -- backed
  by Nav2. 17 tools, with the same names and the same replies the daemon's own
  planner used to give, so the voice chat and the drive console did not change. See
  [rover_daemon/ros_navigator.py](rover_daemon/ros_navigator.py) and
  [ros_nav/nav_bridge.py](ros_nav/nav_bridge.py).

Without `--ros-nav` the daemon offers no driving tools at all, and its startup line
says `driving off` rather than `driving ros2 on 127.0.0.1:8773`.

**When the map is empty, check the driver board before you check the lidar.** The
two are on different ports and the lidar is the misleading one: it can be reporting
happily at 10 Hz while there is no map at all. The chain is
`board telemetry -> base_node odometry -> odom->base_link -> slam_toolbox`, and it
breaks at the first link far more often than anywhere else -- the log says
`Message Filter dropping message ... queue is full` over and over, which reads as a
scan problem and is not one. One call settles it:

```bash
ssh bpi-m4zero 'python3 -c "import json,socket;s=socket.create_connection((\"127.0.0.1\",8769));s.sendall(b\"{\\\"call\\\":\\\"nav_status\\\"}\n\");print(s.makefile().readline())"'
```

`board_ok: false` there means the daemon is holding a port the ESP32 is not talking
on. The board answers over WiFi independently -- `curl "http://192.168.1.22/js?json=%7B%22T%22%3A130%7D"` returns a
`T:1001` line -- so that is how to tell a dead board from a dead serial link. The
daemon now notices this itself and reopens the port, and `nav_status` counts the
attempts as `board_reopens`; a count that climbs over an afternoon is a connector
working loose.

`--ros-nav` needs the ROS stack up, and it does not wait for it -- both are
started by the same crontab and the stack takes the best part of a minute, so a
daemon that insisted would never come up at boot. Each tool connects when it is
called and says plainly when there is nothing to connect to.

**Changing that crontab line is not enough on its own.** The running supervisor is
holding the arguments the daemon was started with, so it has to be replaced too:

```bash
ssh bpi-m4zero "crontab -l | sed 's|--vision --board-bridge$|& --ros-nav|' | crontab - && sync"
ssh bpi-m4zero "pkill -f 'ugv/run_daemon[.]sh'; sleep 1; ~/ugv/restart.sh"
```

Anything touching `slam2d.c` or `slam2d.h` also needs a rebuild on the host, since
`libslam2d.so` is built per-host (aarch64 here) and is not committed:

```bash
ssh bpi-m4zero 'cd ~/ugv/lidar_slam && ./build.sh && ./selftest'
```

**Two libraries the rover needs are unpacked rather than installed, and neither is
committed.** This board's Debian has no `pip` and no `python3-venv`, and `sudo`
here wants a password no script has, so each is a pinned wheel unzipped into a
`vendor/` directory by its own script — idempotent, so running it again after a
deploy costs one import check:

```bash
ssh bpi-m4zero '~/ugv/install_opencv.sh'          # OpenCV 4.12, for YuNet in the daemon
ssh bpi-m4zero '~/ugv/oak_depth/install.sh'       # depthai 2.32.0.0, for the OAK's depth
```

`oak_depth/` has its own supervisor and its own crontab entry, so reloading the
depth service is not the same act as reloading the daemon. It owns the OAK, and
only one process can, so stop it before running anything else against the camera:

```bash
ssh bpi-m4zero 'python3 ~/ugv/oak_depth/selftest.py'   # with the service stopped
ssh bpi-m4zero '~/ugv/oak_depth/restart.sh'            # ~10 s to boot the VPU and stereo
```

`drive_web/` is the browser console, on TCP 8771 (8770 is oak_depth). It has its
own supervisor and crontab entry. `console_model.py` and `rover_tools.py` stay
in `voice_chat/` (talk.py uses them) and are copied next to it on deploy:

```bash
scp drive_web/*.py drive_web/*.html drive_web/*.sh drive_web/README.md bpi-m4zero:~/ugv/drive_web/
scp voice_chat/console_model.py voice_chat/rover_tools.py bpi-m4zero:~/ugv/drive_web/
ssh bpi-m4zero '~/ugv/drive_web/install.sh'    # crontab, once
ssh bpi-m4zero '~/ugv/drive_web/restart.sh'    # prints /health
```

`ros_nav/` is ROS 2 Jazzy, installed from RoboStack into a conda environment
outside `~/ugv` because a deploy overwrites `~/ugv`. It has its own supervisor and
crontab entry, and its `env.sh` **must be sourced from bash** � RoboStack's
activation hooks use `source`, which dash has not got, and the failure names
neither the file nor the shell:

```bash
scp -r ros_nav bpi-m4zero:~/ugv/
ssh bpi-m4zero 'sh ~/ugv/ros_nav/install.sh'          # ~20 min, 4.7 GB, no sudo
ssh bpi-m4zero 'sh ~/ugv/ros_nav/install-boot.sh --nav'   # crontab; also checks the daemon's
ssh bpi-m4zero '~/ugv/ros_nav/restart.sh'             # ~30 s; prints the node list
```

Its `restart.sh` exists for the usual reason and one more: `ros2 launch` does
**not** reliably take its nodes down when killed, so every reload used to leave
another `lidar_node` behind, three of them ended up sharing one serial port, and
`/scan` arrived at 18 Hz from a 10 Hz sensor with nothing reporting an error. The
supervisor sweeps before every launch, and `restart.sh` both counts what is
running and looks for a node that died on the way up.

**That sweep is `sweep.sh`, a separate file, and it must stay one.** It was a
function inside `run_ros_nav.sh`, which is a trap with a long fuse: bash parses a
function once, at start, and that supervisor runs for weeks -- so the sweep that
ran was whichever copy was on disk when it last started. Adding `nav_bridge` to it
therefore did nothing, the old bridge survived a reload, the new one could not
bind its port and died in the log, and the stack came back answering with the
*previous* deploy's code while the reload reported one of each node and nothing
wrong. **Anything that adds a node to this stack adds it to `sweep.sh`**, and a
change to `run_ros_nav.sh` itself needs the supervisor replaced rather than just
the launch:

```bash
ssh bpi-m4zero '~/ugv/ros_nav/restart.sh --supervisor'
```

`wifi_roam/` is neither of those. It is the only thing here installed as a systemd
unit, because scanning and switching networks need root, and its `install.sh` is
what copies the script into `/usr/local/sbin` and the units into
`/etc/systemd/system`. It is idempotent, so run it again after changing any of
them:

```bash
cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" sh ~/ugv/wifi_roam/install.sh'
ssh bpi-m4zero 'systemctl list-timers --no-pager wifi-roam.timer'
```

**Its roam timer is installed and deliberately switched off on this board**, and
`ROAM=off` on that install line is what keeps it that way. The script drove
`nmcli` until 2026-08-23 and the Banana Pi has none, so it has never yet chosen an
access point here; the way it fails is a rover that needs carrying to a socket, so
it gets armed with somebody in the building. See
[wifi_roam/README.md](wifi_roam/README.md).

`netwatch/` is the other systemd unit, and it is the one to install first on any
board that keeps disappearing. It records the link, the load and every word the
supplicant and the kernel say, to `/var/lib/netwatch/` rather than to `/var/log`,
which on this board is a zram ramlog that loses exactly the minutes worth having.
It also writes a record when it is asked to stop, which is what separates a reboot
somebody asked for from a board that fell over:

```bash
cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" sh ~/ugv/netwatch/install.sh'
ssh bpi-m4zero 'netwatch-report'                 # boots, outages, and their causes
python3 netwatch/netprobe.py --log probe.log     # the desk half; not deployed
```

Note the `sh` in front of both installers. A checkout that arrived by `scp` is
mode 644, so the shebang is never consulted and running the path directly fails
with "Permission denied" — which reads as a sudo problem and is not one.

Plain `scp` is fine for the `.py` files — no shebang, so CRLF does not bite. The
shell scripts under `lidar_slam/`, `oak_depth/`, `wifi_roam/`, `netwatch/` and
`face_tracking/` do have one, and
they are held to LF by `.gitattributes`; a CRLF checkout turns their shebang into
an interpreter with a carriage return in its name.

## MEDIA

```bash
scp voice_chat/{server.py,voice_history.py,voice_stream.py,voice_http.py,requirements.txt,selftest.py,test_harness.py,test_server.py,test_talk.py} root@media:/opt/voice_chat/
ssh root@media 'systemctl daemon-reload && systemctl restart voice-chat'
```

`voice-chat` takes ~150 s to come back — three models load and decode is warmed
before it binds — and `/health` answering is the signal that it is ready. The three
GPU services share one card and are mutually exclusive; switch with
`ssh root@media ~/switch_service.sh voice`. `face-detect` is on the CPU and is not
part of that trade.

## Verify on the hardware, not by inference

Prove the deploy on the machine itself — for the Banana Pi, call the affected tool
over TCP on port 8769 and look at what comes back. "The self-test passes" and "the
file was copied" are not evidence that the running system changed.
