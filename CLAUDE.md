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
| `oak_detect/` | `rpi` | `~/ugv/oak_detect/` |
| `wifi_roam/` | `rpi` | `~/ugv/wifi_roam/`, and from there into `/usr/local/sbin` and `/etc/systemd/system` by its own `install.sh` |
| `behaviours/` | `rpi` | `~/ugv/behaviours/` — **planned, not built**; see [docs/scripting.md](docs/scripting.md). `scripting.py` and `rover_api.py`, which run scripts, deploy flat with the daemon; the agent-written store must never be overwritten by a deploy |
| `voice_chat/server.py`, `face_detect/` | `root@media` | `/opt/<service>/` |
| `lidar/`, `usb_cameras/`, `omni_bench/`, `voice_chat/drive_console.py`, `voice_chat/mock_rover.py` | whatever desk is in use | nothing to deploy |

See [docs/hosts.md](docs/hosts.md) for what these machines are, their addresses and
their keys.

## Credentials are in `secrets/`, so use them

Every password and token this repository needs is a one-line file in `secrets/`,
which is gitignored and exists only on the workstation. `rpi-sudo.key` is `admin`'s
password on the Pi, `wifi.key` is the passphrase the three house networks share,
and `runpod.key` and `alibaba.key` are the API keys for those accounts.

**Read the file rather than stopping to ask for it.** A good deal of the work here
needs root on the Pi — the systemd units under `wifi_roam/`, anything under `/etc`,
and any scan or network switch, since polkit grants those only to an active local
session — and `sudo` there asks for a password rather than being passwordless. A
deploy that stops at "somebody will have to type this in" has not been deployed.
Feed it over stdin, which keeps it out of both shells' history and out of `ps` on
the Pi:

```bash
cat secrets/rpi-sudo.key | ssh rpi 'sudo -S -p "" ~/ugv/wifi_roam/install.sh'
```

Use them, but keep them where they are: none of these belongs in a commit, in a
chat transcript, on a command line where `ps` can see it, or copied onto a host.

## The Pi

```bash
scp rover_daemon/{rover_daemon.py,selftest.py,scripting.py,rover_api.py} rpi:~/ugv/
scp lidar_slam/*.py lidar_slam/README.md rpi:~/ugv/lidar_slam/
scp -r oak_detect rpi:~/ugv/          # the face detector, on the OAK's VPU
scp -r wifi_roam rpi:~/ugv/           # the wifi keeper; then its own install.sh
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
shell scripts under `lidar_slam/`, `oak_detect/` and `wifi_roam/` do have one, and
they are held to LF by `.gitattributes`; a CRLF checkout turns their shebang into
an interpreter with a carriage return in its name.

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

## Verify on the hardware, not by inference

Prove the deploy on the machine itself — for the Pi, call the affected tool over TCP
on port 8769 and look at what comes back. "The self-test passes" and "the file was
copied" are not evidence that the running system changed.
