# Working in this repository

Rules for doing the work. Where each directory runs, how to push it, and how to
restart it are in [docs/deploy.md](docs/deploy.md). The machines, addresses and
keys are in [docs/hosts.md](docs/hosts.md). A component's own README is where
the rest lives.

## A change is not done until it runs on the host that uses it

The rover's services run on the Banana Pi (`bpi-m4zero`). The GPU model
services run on MEDIA. Bench scripts (`oak_camera/`, `lidar/`, `usb_cameras/`,
`driver_board/`, `omni_bench/`, `voice_chat/mock_rover.py`) run on whichever
desk is in use and need no deploy.

Nothing is synced or rebuilt automatically. A change that has only been
committed exists nowhere but the repo — the rover goes on running the old
code. Self-tests passing locally are not the finish line.

**Work out which hosts the changed files run on, push to each, restart what
needs restarting, and verify it there — as part of the same piece of work,
without being asked.** Say in the report which hosts were deployed to and
what was checked on them.

The repo stays source of truth: edit here and push, never edit in place on a
host.

## Credentials are in `secrets/`, so use them

Every password and token is a one-line file in `secrets/`, gitignored, only
on the workstation. **Read the file rather than stopping to ask for it.** A
deploy that stops at "somebody will have to type this in" has not been
deployed.

`bpi-sudo.key` is `admin`'s password on the Banana Pi. `rpi-sudo.key` is the
same account on the Raspberry Pi it replaced — **the two are different, and
the Pi's is silently refused by the Banana Pi**, which reads as a board that
has lost its password rather than as the wrong file.

`sudo` on the Banana Pi prompts. Feed the password over stdin, once per
`sudo`, because `-S` reads until end of file:

```bash
cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" sh ~/ugv/wifi_roam/install-dual.sh'
```

Two `sudo -S` calls chained after one `cat` leave the second with nothing.

Keep them where they are: none of these belongs in a commit, a chat
transcript, a command line where `ps` can see it, or copied onto a host. The
two files the rover itself must hold live in `~/.ugv/` on the board, outside
the deploy tree, for that reason — see [docs/deploy.md](docs/deploy.md).

## Do not fight the supervisors

Each long-running service has a supervisor (crontab or systemd) and a
`restart.sh`. The supervisor is where the arguments live. `restart.sh` kills
only the child and lets it come back with those arguments.

- Never type the `pkill` on an ssh command line. The pattern matches the
  session that typed it.
- Never relaunch `run_daemon.sh` by hand. It drops the flags and the rover
  silently loses tools.
- Changing a crontab line is not enough: the running supervisor still holds
  the old arguments, so replace it too, and `sync` afterwards — this card is
  `commit=120` and a restart otherwise undoes the write.
- `ros_nav/sweep.sh` must stay a separate file. Anything that adds a node to
  that stack adds it there. A change to `run_ros_nav.sh` itself needs
  `~/ugv/ros_nav/restart.sh --supervisor`.

Do not enable `wifi-roam.timer` while `wifi_dual` is running. They have
opposite models of the link and will fight over the rover's only way in.

## Reproduce it in simulation before you fix it

**A fix for a fault nobody has reproduced is a guess.** Whenever a fault can
be put in front of a model of the thing that misbehaves, build the
reproduction first, and do not change the running system until the
reproduction fails the same way the rover does.

Then hold the fix to the same standard: show it working *in the
reproduction* before deploying it, and say what the reproduction measured.
"This should help" is not a result.

Simulation does not replace the hardware: a fix that passes in the model
still has to be deployed and watched on the rover. And a model is not trusted
for being a model — a reproduction has to be validated against a recording of
the real fault before it can be used to judge anything. Where the model and
the hardware disagree, the hardware is right and the model has a bug.

For the navigation stack the pieces already exist: `ros_nav/nav_record.py`,
`ros_nav/corridor_sim.py`, `ros_nav/dwb_replay.py`. See
[ros_nav/README.md](ros_nav/README.md). For the dual-wifi manager,
`python3 wifi_roam/test_wifi_dual.py` replays a recording of this rover
losing the network and refuses to report anything if its grading disagrees.

## Verify on the hardware, not by inference

Prove the deploy on the machine itself — for the Banana Pi, call the affected
tool over TCP on port 8769 and look at what comes back. "The self-test
passes" and "the file was copied" are not evidence that the running system
changed.
