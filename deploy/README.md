# Automated deployment

`deploy.py` is the preferred way to move committed source from this checkout to
the rover. It replaces the repeated `scp`/`rsync`/restart sequence in
[`docs/deploy.md`](../docs/deploy.md); the manual commands there remain the
recovery path and the detailed map of what lands where.

There is one deployment host in the current system: `orin`, the Jetson Orin Nano
on the rover, which replaced the Banana Pi on 2026-08-31. The realtime Qwen Omni
model is an Alibaba cloud service; local world-state perception uses TensorRT.

## Requirements

On the workstation:

- Python 3.10+
- `git`, `ssh` and `scp` on `PATH`
- the `orin` SSH entry described in [`docs/hosts.md`](../docs/hosts.md)

On the rover, `python3`, `tar` and `rsync` are part of the working installation. `rsync` runs on the remote side, so a Windows workstation does not
need a local rsync installation. The deployer itself has no third-party Python
dependencies.

## Normal use

Commit first, then inspect and deploy:

```bash
python deploy/deploy.py --plan
python deploy/deploy.py
```

A normal run reads `~/.ugv/deploy-state.json` on the rover and compares the
recorded SHA for each component with the checkout's `HEAD`. Only affected
components are copied/restarted. State advances only after the component's
verification commands succeed.

Useful limits:

```bash
python deploy/deploy.py --only rover_daemon
python deploy/deploy.py --only ros_nav
python deploy/deploy.py --only drive_web
python deploy/deploy.py --host orin
```

## First run

There is intentionally no guessed baseline. If the rover has no deployment
state, choose one explicitly:

```bash
# The running rover is already known to contain exactly this commit.
python deploy/deploy.py --adopt --host orin

# Reconcile selected components from the checkout instead.
python deploy/deploy.py --full --host orin
```

`--adopt` copies nothing; it establishes the per-component baseline. `--full`
packages every selected component from `HEAD`, then restarts and verifies it.

## Privileged system copies

`wifi_roam`, `dongle_driver` and `netwatch` have a second deployment boundary:
source is staged under `~/ugv`, while what actually runs also lives under
`/usr/local`, `/etc` and — for the dongle's driver — the kernel's module tree.
An ordinary deploy stops after staging/testing and does not touch those
privileged locations. The component remains outstanding until its system install
succeeds.

After reviewing the change and with a recovery path available for a network
mistake:

```bash
python deploy/deploy.py --system --only wifi_roam
python deploy/deploy.py --system --only dongle_driver
python deploy/deploy.py --system --only netwatch
```

`dongle_driver` also needs re-running after a kernel update, since it builds an
out-of-tree module. `netwatch` is staged but deliberately not installed on this
host: it is ordered after `wpa_supplicant` and reads that daemon's control
socket, and the Orin runs NetworkManager instead.

The deployer reads `secrets/jetson-orin.key` locally and feeds it to `sudo -S`;
the password is never placed in the command line or copied to the rover.

## What is deployed

[`manifest.json`](manifest.json) is executable deployment policy. It describes:

- which repository paths form each component;
- whether those paths are flattened or retain their directory shape;
- their destination on the rover;
- runtime files that must survive a mirror;
- restart/self-test/readiness commands;
- supervisor changes that need a supervisor replacement;
- privileged installers that require `--system`.

The daemon component currently flattens `rover_daemon/`, `driver_board/` and
`face_tracking/` into `~/ugv/`. `drive_web/` retains its own directory and also
receives the small set of shared current modules from `voice_chat/` that the
console/Alibaba session imports. That layout is historical but intentional for
now; changing the application package structure is a separate refactor.

`lidar_slam` remains a true mirror with `--delete`, preserving the host-built
`libslam2d.so` and `selftest`. Other component directories are additive, with
removed tracked source reconciled by the deployer from deployment history.

The archive is built from `git ls-files`, so ignored files, local secrets,
downloaded wheels, logs and other untracked state cannot accidentally be sent.
Git's executable bit is copied into tar metadata, avoiding a Windows checkout
turning shell scripts into non-executable files on the rover.

## State and failure semantics

Remote state lives at:

```text
~/.ugv/deploy-state.json
```

It records a SHA per component rather than one SHA for the whole machine. A
partial deploy therefore cannot make unrelated source look deployed.

The state file is written atomically and followed by `sync`. That was for the
Banana Pi, whose root filesystem was mounted `commit=120` and had lost recent
metadata on an abrupt reset; the Orin's NVMe root is mounted `rw,relatime`, so
the `sync` is now cheap insurance rather than a requirement.

If copying, restart, self-test, health check or privileged installation fails,
that component's SHA is not advanced. The next invocation still sees it as
outstanding.

Exit codes:

- `0`: selected work succeeded or nothing needed doing;
- `1`: one or more component deployments failed;
- `2`: usage/precondition/state error, including an unknown first-run baseline;
- `3`: source was staged but a privileged component still needs `--system`.

## Tests

Pure path/mapping rules have stdlib tests:

```bash
python -m unittest deploy.test_deploy -v
```

These do not contact the rover. The final proof remains the component's remote
restart/self-test/health output; a copied file is not evidence that the running
robot changed.

## Guards

`guards/` enforces two rules from [../AGENTS.md](../AGENTS.md) that are otherwise
only remembered: a shell command must not edit `~/ugv` on the rover in place or
carry the contents of a `secrets/` file, and a session that changed a deployed
component must not finish without deploying it. Both read
[manifest.json](manifest.json) and `secrets/`, never a list of their own, so they
follow the manifest as it changes.

They are plain scripts and are not deployed:

```bash
python deploy/guards/rover_guard.py --command "scp x.py orin:~/ugv/"   # exit 1 + why
python deploy/guards/rover_guard.py --staged                           # what a commit stages
python deploy/guards/deploy_watch.py --components world_state/resolve.py
```

Any agent or editor can call them. Two wirings exist: `git config core.hooksPath
.githooks` runs the credential check on every commit from this clone, and
[../.claude/settings.json](../.claude/settings.json) runs both as Claude Code
hooks, where the command is refused before it executes. A command that genuinely
must touch the deploy tree -- manual recovery per
[../docs/rover-unresponsive.md](../docs/rover-unresponsive.md) -- carries the
comment `# deploy-guard: allow`.
