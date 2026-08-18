# Working in this repository

## Deploy every change to the host that runs it

**Nothing in this repository runs on the Windows workstation.** The rover code runs
on the Pi, the ROS 2 stack runs in the VM, the model services run on MEDIA. Nothing
is synced or rebuilt automatically, so a change that has only been committed exists
nowhere but the repo — the rover goes on running the old code.

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
| `oak_detect/` | `rpi` | `~/ugv/oak_detect/` |
| `voice_chat/server.py`, `face_detect/` | `root@media` | `/opt/<service>/` |
| `vm/` | the SLAM VM | `~/ugv/<same relative path>` |
| `lidar/`, `usb_cameras/`, `omni_bench/`, `voice_chat/drive_console.py`, `voice_chat/mock_rover.py` | whatever desk is in use | nothing to deploy |

See [docs/hosts.md](docs/hosts.md) for what these machines are, their addresses and
their keys.

## The Pi

```bash
scp rover_daemon/{rover_daemon.py,selftest.py} rpi:~/ugv/
scp lidar_slam/*.py lidar_slam/README.md rpi:~/ugv/lidar_slam/
scp -r oak_detect rpi:~/ugv/          # the face detector, on the OAK's VPU
ssh rpi 'cd ~/ugv && python3 selftest.py | tail -2'
ssh rpi '~/ugv/restart.sh'          # ~35 s; prints the new tool count
```

Both `restart.sh` scripts exist so that the pattern they kill on lives in a file
rather than on an ssh command line, where it would match -- and kill -- the very
session that typed it.

`restart.sh` kills only the daemon and lets the supervisor restart it, because the
crontab entry — `@reboot ~/ugv/run_daemon.sh --vision --lidar` — is where the
arguments live. **Never relaunch `run_daemon.sh` by hand**; it drops the flags and
the rover silently loses tools.

Anything touching `slam2d.c` or `slam2d.h` also needs a rebuild on the host, since
`libslam2d.so` is built per-host (armv6) and is not committed:

```bash
ssh rpi 'cd ~/ugv/lidar_slam && ./build.sh && ./selftest'
```

`oak_detect/` is the same story for the same reason — `liboak.so` is armv6 and is
not committed. Its supervisor is a separate crontab entry, so reloading the
detector is not the same act as reloading the daemon:

```bash
ssh rpi '~/ugv/oak_detect/build.sh && python3 ~/ugv/oak_detect/selftest.py | tail -2'
ssh rpi '~/ugv/oak_detect/restart.sh'      # ~6 s to boot the VPU and load the graph
```

Plain `scp` is fine here — these are `.py` files with no shebang, so CRLF does not
bite.

## MEDIA

```bash
scp voice_chat/{server.py,requirements.txt,selftest.py} root@media:/opt/voice_chat/
ssh root@media 'systemctl daemon-reload && systemctl restart voice-chat'
```

`voice-chat` takes ~150 s to come back — three models load and decode is warmed
before it binds — and `/health` answering is the signal that it is ready. The three
GPU services share one card and are mutually exclusive; switch with
`ssh root@media ~/switch_service.sh voice`. `face-detect` is on the CPU and is not
part of that trade.

## The VM

Guest at `rover@192.168.80.128`, key `D:\VMs\ugv-rover\ssh\id_ed25519`. Copy each
file to `~/ugv/<same relative path>`, but push it through a **base64 helper rather
than scp**: Windows CRLF, UTF-8 BOMs and OpenSSH argument quoting each corrupt shell
scripts on the way in. (`.gitattributes` pins `vm/**` to LF for the same reason.)
Launch files and config only take after `~/ugv/bin/start_slam.sh` restarts the stack.

## Verify on the hardware, not by inference

Prove the deploy on the machine itself — for the Pi, call the affected tool over TCP
on port 8769 and look at what comes back. "The self-test passes" and "the file was
copied" are not evidence that the running system changed.
