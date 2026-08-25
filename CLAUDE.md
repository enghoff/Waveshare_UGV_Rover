# Working in this repository

Rules for changing the rover. Directory ownership, deploy/restart commands and the
manual recovery path are in [docs/deploy.md](docs/deploy.md). The normal committed
path is [`python deploy/deploy.py`](deploy/README.md). Current host/network facts
are in [docs/hosts.md](docs/hosts.md). A component's README describes the component
itself.

## A change is not done until it runs on the host that uses it

The running rover services are on the Banana Pi (`bpi-m4zero`). The realtime voice
model is Alibaba's hosted Qwen Omni service; there is no local GPU/MEDIA deployment
in the current system. Bench scripts (`oak_camera/`, `lidar/`, `usb_cameras/`,
`driver_board/`, `face_tracking/track_face.py`, `voice_chat/mock_rover.py`) run on
whichever desk is in use and need no deploy.

A commit does not sync itself to the rover. For committed changes use
`python deploy/deploy.py`; it determines which registered components changed,
copies them, invokes their existing restart/verification paths and advances state
only after those checks pass. Use the manual commands in `docs/deploy.md` when
recovering the deployer itself or working on an unregistered component.

**Work out which host uses the changed files, deploy there, restart what needs
restarting, and verify the running service there as part of the same piece of
work.** Say what was deployed and what proved it.

The repository stays the source of truth: edit here and push, never edit a tracked
file in place on the rover. The deployer therefore refuses dirty tracked files;
the recorded commit must describe the bytes that were sent.

When prose and executable source/config disagree, **the source/config wins**.
Correct the document; do not revive an old setting because a README remembers it.

## Credentials

Local deployment credentials live as one-line files under `secrets/`, which is
gitignored. `bpi-sudo.key` is `admin`'s sudo password on the Banana Pi. The old
Raspberry Pi password is different and must not be substituted for it.

`sudo` on the Banana Pi prompts. Feed the password over stdin once per `sudo`;
`-S` reads until EOF, so two sudo commands chained after one `cat` leave the
second with no password. The deployer handles this for components with an
explicit `--system` installation. Manual equivalent:

```bash
cat secrets/bpi-sudo.key | ssh bpi-m4zero 'sudo -S -p "" sh ~/ugv/wifi_roam/install-dual.sh'
```

Runtime secrets the rover itself must hold live in `~/.ugv/`, outside the source
deploy tree. In particular:

- `~/.ugv/alibaba.key` — DashScope key for the realtime Qwen Omni session;
- `~/.ugv/console.token` — gates use of the browser microphone;
- `~/.ugv/deploy-state.json` — deployment state (not secret, but still runtime
  state rather than source).

Do not put credentials in a commit, chat transcript, process command line or a
path that source deployment can copy back to the workstation.

## Do not fight the supervisors

Each long-running service has a supervisor (crontab or systemd) and a
`restart.sh`. The supervisor is where its arguments live. `restart.sh` kills the
child and lets the supervisor bring it back with those arguments.
`deploy/manifest.json` contains explicit supervisor-replacement rules for files
that change the supervisor itself.

- Never type an unguarded `pkill` pattern on an SSH command line; the pattern can
  match the session carrying it.
- Never relaunch `run_daemon.sh` by hand; doing so drops the flags held by the
  supervisor and can silently remove tools.
- A changed crontab needs the running supervisor replaced too, followed by
  `sync`; the card is mounted with `commit=120`.
- `ros_nav/sweep.sh` stays a separate file so a long-lived shell cannot keep an
  old parsed copy. Changes to `run_ros_nav.sh`, `sweep.sh` or the DDS supervisor
  path require `~/ugv/ros_nav/restart.sh --supervisor`; the deployer selects that
  route when appropriate.

Do not enable `wifi-roam.timer` while `wifi_dual` is running. The two have
opposite ownership models and will fight over the rover's only network path.
Privileged Wi-Fi deployment is therefore staged/tested first and needs an
explicit `deploy.py --system` before the running system copy is replaced.

## Reproduce faults before fixing them

**A fix for a fault nobody reproduced is a guess.** Whenever a fault can be put
in front of a model of the subsystem, build/replay the reproduction first and do
not change the running system until the model fails the same way the rover did.
Then show the proposed fix succeeding in that reproduction before deploying it.

Simulation does not replace hardware. A model must be validated against a real
recording before it is trusted, and a fix that passes offline still has to be
deployed and observed on the rover. Where model and hardware disagree, the
hardware is right and the model needs work.

For navigation the existing tools include `ros_nav/nav_record.py`, replay and
controller simulations in `ros_nav/`; see [ros_nav/README.md](ros_nav/README.md).
For dual Wi-Fi, `python3 wifi_roam/test_wifi_dual.py` replays a captured outage and
checks the manager's grading against the real event.

## Verify the running service, not the copied file

Proof is taken on the machine that uses the change. For daemon-facing changes,
call the affected function over TCP 8769 and inspect the answer. "The file was
copied" and "a local unit test passes" do not prove the running rover changed.
The deployer deliberately reuses component restart scripts because those scripts
already know their readiness checks.

For pure documentation/deletion changes that do not affect a registered runtime
component, no rover restart is needed. If a deploy manifest or source set changes,
use `deploy.py --plan` to prove the deployer's interpretation of the new tree.

## Report in plain English

Lead with what is true in words a person can use: whether the rover moved, the
network stayed up, a service restarted, or a recorded fault reproduced. File
names, YAML keys and tick counts are supporting evidence, not the first sentence.

If the obvious story is wrong, say so briefly and name the real one. Prefer
numbers with physical meaning or a comparison over raw log volume. End with the
single next action and what result would count as proof rather than a menu of
untested guesses.
