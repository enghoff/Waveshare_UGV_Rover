# Working in this repository

## Deploy every change to the host that runs it

**Nothing in this repository runs on the Windows workstation.** The rover code runs
on the Pi and the model services run on MEDIA. Nothing is synced or rebuilt
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
| `rover_daemon/`, `driver_board/`, `face_tracking/` | `rpi` | `~/ugv/` (flat for the daemon; others mirror their repo path) |
| `lidar_slam/` | `rpi` | `~/ugv/lidar_slam/` |
| `oak_depth/` | `rpi` | `~/ugv/oak_depth/`, plus `vendor/` filled by its own `install.sh` |
| `wifi_roam/` | `rpi` | `~/ugv/wifi_roam/`, and from there into `/usr/local/sbin` and `/etc/systemd/system` by its own `install.sh` |
| `behaviours/` | `rpi` | `~/ugv/behaviours/` — **planned, not built**; see [docs/scripting.md](docs/scripting.md). `scripting.py` and `rover_api.py`, which run scripts, deploy flat with the daemon; the agent-written store must never be overwritten by a deploy |
| `voice_chat/server.py`, `face_detect/` | `root@media` | `/opt/<service>/` |
| `lidar/`, `usb_cameras/`, `omni_bench/`, `voice_chat/drive_web.py`, `voice_chat/drive_web.html`, `voice_chat/console_model.py`, `voice_chat/mock_rover.py` | whatever desk is in use | nothing to deploy |

`drive_web.py` serves a browser page and is still a desk program: the HTTP is at
the desk end and the Pi answers the same six TCP connections it always has. There
is nothing to put on the rover for it, and nothing about it that the rover can
afford to run.

See [docs/hosts.md](docs/hosts.md) for what these machines are, their addresses and
their keys.

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
needs root on the Pi — the systemd units under `wifi_roam/`, anything under `/etc`,
and any scan or network switch, since polkit grants those only to an active local
session — and `sudo` there asks for a password rather than being passwordless. A
deploy that stops at "somebody will have to type this in" has not been deployed.
Feed it over stdin, which keeps it out of both shells' history and out of `ps` on
the Pi:

```bash
cat secrets/bpi-sudo.key | ssh rpi 'sudo -S -p "" ~/ugv/wifi_roam/install.sh'
```

One password per `sudo`, because `-S` reads until end of file: two `sudo -S` calls
chained after one `cat` leaves the second with nothing and it fails as "no password
was provided", which looks like the wrong password and is not. Pipe it once per
command, or give the remote side a small script to run under a single `sudo`.

Use them, but keep them where they are: none of these belongs in a commit, in a
chat transcript, on a command line where `ps` can see it, or copied onto a host.

## The Pi

```bash
scp rover_daemon/*.py rpi:~/ugv/
scp lidar_slam/*.py lidar_slam/README.md rpi:~/ugv/lidar_slam/
scp face_tracking/*.py face_tracking/install_opencv.sh     face_tracking/face_detection_yunet.onnx rpi:~/ugv/   # the tracking loop and its detector
scp -r oak_depth rpi:~/ugv/           # the OAK as a depth camera; then its own install.sh
scp -r wifi_roam rpi:~/ugv/           # the wifi keeper; then its own install.sh
ssh rpi 'cd ~/ugv && python3 selftest.py | tail -2'
ssh rpi '~/ugv/restart.sh'          # ~35 s; prints the new tool count
```

Both `restart.sh` scripts exist so that the pattern they kill on lives in a file
rather than on an ssh command line, where it would match -- and kill -- the very
session that typed it.

`restart.sh` kills only the daemon and lets the supervisor restart it, because the
crontab entry — `@reboot ~/ugv/run_daemon.sh --vision --lidar`, beside
`@reboot ~/ugv/oak_depth/run_oak_depth.sh` — is where the
arguments live. **Never relaunch `run_daemon.sh` by hand**; it drops the flags and
the rover silently loses tools.

Anything touching `slam2d.c` or `slam2d.h` also needs a rebuild on the host, since
`libslam2d.so` is built per-host (armv6) and is not committed:

```bash
ssh rpi 'cd ~/ugv/lidar_slam && ./build.sh && ./selftest'
```

**Two libraries the rover needs are unpacked rather than installed, and neither is
committed.** This board's Debian has no `pip` and no `python3-venv`, and `sudo`
here wants a password no script has, so each is a pinned wheel unzipped into a
`vendor/` directory by its own script — idempotent, so running it again after a
deploy costs one import check:

```bash
ssh rpi '~/ugv/install_opencv.sh'          # OpenCV 4.12, for YuNet in the daemon
ssh rpi '~/ugv/oak_depth/install.sh'       # depthai 2.32.0.0, for the OAK's depth
```

`oak_depth/` has its own supervisor and its own crontab entry, so reloading the
depth service is not the same act as reloading the daemon. It owns the OAK, and
only one process can, so stop it before running anything else against the camera:

```bash
ssh rpi 'python3 ~/ugv/oak_depth/selftest.py'   # with the service stopped
ssh rpi '~/ugv/oak_depth/restart.sh'            # ~10 s to boot the VPU and stereo
```

`wifi_roam/` is neither of those. It is the only thing here installed as a systemd
unit, because scanning and switching networks need root, and its `install.sh` is
what copies the script into `/usr/local/sbin` and the units into
`/etc/systemd/system`. It is idempotent, so run it again after changing any of
them:

```bash
ssh rpi 'sudo ~/ugv/wifi_roam/install.sh'      # add the passphrase on a new Pi
ssh rpi 'systemctl list-timers --no-pager wifi-roam.timer'
```

Plain `scp` is fine for the `.py` files — no shebang, so CRLF does not bite. The
shell scripts under `lidar_slam/`, `oak_depth/`, `wifi_roam/` and
`face_tracking/` do have one, and
they are held to LF by `.gitattributes`; a CRLF checkout turns their shebang into
an interpreter with a carriage return in its name.

## MEDIA

```bash
scp voice_chat/{server.py,voice_history.py,voice_stream.py,voice_http.py,requirements.txt,selftest.py,test_harness.py,test_server.py,test_talk.py,test_drive_web.py} root@media:/opt/voice_chat/
ssh root@media 'systemctl daemon-reload && systemctl restart voice-chat'
```

`voice-chat` takes ~150 s to come back — three models load and decode is warmed
before it binds — and `/health` answering is the signal that it is ready. The three
GPU services share one card and are mutually exclusive; switch with
`ssh root@media ~/switch_service.sh voice`. `face-detect` is on the CPU and is not
part of that trade.

## Verify on the hardware, not by inference

Prove the deploy on the machine itself — for the Pi, call the affected tool over TCP
on port 8769 and look at what comes back. "The self-test passes" and "the file was
copied" are not evidence that the running system changed.
