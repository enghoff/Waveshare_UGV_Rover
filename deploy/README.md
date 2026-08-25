# Automated deployment

`deploy.py` is the preferred way to move committed code from this checkout to
the rover. It replaces the repeated `scp`/`rsync`/restart sequence in
[`docs/deploy.md`](../docs/deploy.md); the manual commands there remain the
recovery path and the detailed map of what runs where.

The deployer is deliberately small. It is not an agent on the rover and it does
not make the Banana Pi pull from GitHub. The workstation keeps the repository as
the source of truth, packages only Git-tracked files, copies them over SSH, runs
the existing restart/self-test commands on the target, and records the commit
that each component actually proved on that host.

## Requirements

On the workstation:

- Python 3.10+
- `git`, `ssh` and `scp` on `PATH`
- the SSH aliases in [`docs/hosts.md`](../docs/hosts.md)

On the Banana Pi, `python3`, `tar` and `rsync` are already part of the working
installation. `rsync` is used on the **remote** side, so a Windows workstation
does not need a local rsync installation.

No Python packages are required.

## Normal use

Commit first, then inspect and deploy:

```bash
python deploy/deploy.py --plan
python deploy/deploy.py
```

A normal run reads `~/.ugv/deploy-state.json` on each target and compares the
recorded SHA for each component with the checkout's `HEAD`. Only components with
matching source-path changes are copied and restarted. State advances only after
the component's verification commands succeed.

Useful limits:

```bash
python deploy/deploy.py --only rover_daemon
python deploy/deploy.py --only ros_nav
python deploy/deploy.py --host bpi
python deploy/deploy.py --only media_voice
```

`media_voice` is opt-in rather than part of a default deployment because the
MEDIA GPU is shared with other mutually-exclusive services.

## First run

There is intentionally no guessed baseline. If a target has no deployment state,
choose one of these explicitly:

```bash
# The running host is already known to contain exactly this commit.
python deploy/deploy.py --adopt --host bpi

# Reconcile the selected components from the checkout instead.
python deploy/deploy.py --full --host bpi
```

`--adopt` copies nothing; it only establishes the per-component baseline.
`--full` packages every selected component from `HEAD`, then restarts and verifies
it as usual.

## Wi-Fi and other privileged system copies

`wifi_roam` and `netwatch` have a second deployment boundary: source is first
copied under `~/ugv`, but the running copies live under `/usr/local` and
`/etc/systemd/system`. An ordinary run stops after staging/test and **does not**
touch those privileged locations. It exits as incomplete and leaves that
component's state unchanged.

After reviewing the change and with somebody able to recover the rover if a
network change goes wrong, run:

```bash
python deploy/deploy.py --system --only wifi_roam
python deploy/deploy.py --system --only netwatch
```

The deployer reads `secrets/bpi-sudo.key` locally and feeds it to `sudo -S`; the
password is never placed in the command line or copied to the rover.

## What is deployed

[`manifest.json`](manifest.json) is executable deployment policy, not a second
copy of application configuration. It describes:

- which repository paths make up a component;
- whether those paths are flattened (`rover_daemon`, `driver_board` and
  `face_tracking`) or retain their directory shape;
- the destination on the target;
- runtime files that must survive a mirror;
- the existing restart/self-test commands to run after copying;
- supervisor-script changes that need a supervisor replacement rather than only
  a child restart;
- privileged installers that require `--system`.

`lidar_slam` remains a true mirror with `--delete`, preserving the host-built
`libslam2d.so` and `selftest`. Other component directories are additive, but a
file that was deleted from Git since that component's recorded SHA is removed
explicitly from the target. This avoids deleting runtime state while still
preventing removed source files from lingering indefinitely.

The deployment archive is built from `git ls-files`, so ignored files, local
secrets, downloaded wheels, logs and other untracked state cannot accidentally be
sent. Git's executable bit is copied into the tar metadata, avoiding the old
Windows/`scp` mode-644 problem for shell scripts.

## State and failure semantics

Remote state is a small JSON file at:

```text
~/.ugv/deploy-state.json
```

It records a SHA per component rather than one SHA per machine. That matters when
only one component is deliberately deployed: advancing a host-wide SHA would
otherwise make unrelated changes look deployed when they were not.

The state file is written atomically and followed by `sync`, because this Banana
Pi's filesystem uses `commit=120` and has previously lost recent metadata on an
abrupt reset.

If copying, restart, self-test, health check or privileged installation fails,
that component's SHA is not advanced. The next invocation therefore still sees
it as outstanding.

Exit codes are:

- `0`: everything selected was successfully deployed, or nothing needed doing;
- `1`: one or more component deployments failed;
- `2`: usage/precondition/state error, including an unknown first-run baseline;
- `3`: files were staged but one or more privileged components still need
  `--system`.

## Tests

The deployer's pure path/mapping rules have stdlib unit tests:

```bash
python -m unittest deploy.test_deploy -v
```

These tests do not contact the rover. The real proof remains the component's
remote restart/self-test/health output; a copied file is not evidence that the
running robot changed.
