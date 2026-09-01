# Working in this repository

This is the canonical instruction file for coding agents working on the UGV
Rover. `CLAUDE.md` imports this file; maintain shared guidance here rather than
duplicating it in tool-specific files.

## Grounding

- This repository is the source of truth for the physical UGV Rover.
- The local `D:\UGV Rover` workspace is on the Windows control workstation. It
  is not the machine that runs the rover software.
- The rover computer and default runtime/administration target is the NVIDIA
  Jetson Orin Nano. Its SSH alias is `orin`, hostname is `jetson-orin`, and user
  is `jetson`.
- Unless the user explicitly names another host, interpret Ubuntu prompts,
  screenshots, package updates, service problems, hardware observations, and
  requests to run or verify rover software as referring to the Orin.
- Do not use the presence or absence of WSL to decide whether the Orin is
  reachable. Inspect this repository and try `ssh -o BatchMode=yes orin hostname`
  before reporting that the target is unavailable.
- The old Banana Pi is no longer on the rover or a deployment target.

Deployment/restart commands, directory ownership and manual recovery are in
`docs/deploy.md`; current host and network facts are in `docs/hosts.md`; recovery
for a rover that has dropped off the network is in
`docs/rover-unresponsive.md`. A component's README describes that component.

## Report in plain English

Write for the person who owns the rover but does not carry its code in their
head. Lead with the conclusion: what works, what does not, and what it means for
the rover. Investigation details, options considered, filenames and settings
are supporting material. Say plainly when something is unproven, failed or was
skipped. Finish with the next step when there is one.

## Reproduce faults before fixing them

A fix for a fault nobody reproduced is a guess. Replay the reproduction first
and leave the running system alone until the model fails the way the rover did.
Then show the fix succeeding there before deploying. Simulation does not replace
hardware: validate against a real recording, still observe the fix on the rover,
and treat hardware as authoritative where they disagree. Navigation has
`ros_nav/nav_record.py` with replay and controller simulations; Wi-Fi has
`wifi_roam/selftest.sh`, which runs against a fake board.

## Where things run

Rover services run on the Jetson Orin Nano (`orin`). The realtime voice model is
Alibaba's hosted Qwen Omni, and nothing deployed uses the Orin GPU. Deployment
scope is defined by `deploy/manifest.json`, not by a prose list. Bench scripts
under `oak_camera/`, `lidar/` and `usb_cameras/` are not deployed; selected files
from `face_tracking/`, `voice_chat/` and `driver_board/` are.

## The repository is the source of truth

Edit tracked files here and deploy them; never edit a tracked project file in
place on the rover. The deployer rejects dirty tracked files because its recorded
commit must describe the bytes sent. When prose disagrees with executable source
or configuration, the source wins: correct the document instead of reviving an
obsolete setting.

## A change is not done until it runs on its host

Deploy to the host that uses changed files, restart only what needs restarting,
and verify the running service as part of the same work. Say what was deployed
and what proved it. `deploy/deploy.py` copies registered changed components and
runs their restart and verification checks, advancing state only after they pass.
Proof is taken on the Orin; for example, call an affected function over TCP 8769
and inspect the result. A copied file or local unit test alone is not runtime
proof. A change touching no registered component needs no restart; a changed
manifest or source set requires `deploy.py --plan`.

## Orin access and credentials

- Prefer the configured `orin` SSH alias. Its stable service address is
  `192.168.1.80`; `jetson-orin.local` is the documented fallback.
- SSH is key-only from the workstation. The gitignored
  `secrets/jetson-orin.key` contains the `jetson` account's login and sudo
  password, not an SSH private key. Older board credentials are different.
- Never print, log, commit, or place a credential on a command line. Feed the
  sudo password to `sudo -S -p ''` over standard input as documented in
  `docs/deploy.md`.
- Secrets held by the rover live outside the deploy tree under `~/.ugv/`. Never
  copy them back into the repository.

## System-update requests

For a request to handle Ubuntu updates on the rover:

1. Connect to `orin` and check the hostname, OS/L4T release, free root space,
   `dpkg --audit`, pending upgrades, and `/var/run/reboot-required`.
2. Refresh package metadata and dry-run the upgrade before applying it. Review
   removals, held packages, and any NVIDIA/L4T or kernel changes explicitly.
3. Apply an ordinary noninteractive `apt-get upgrade` only when the dry run is
   clean. Do not enable Ubuntu Pro/ESM, run a release upgrade, use
   `full-upgrade`, or autoremove packages unless the user explicitly requests
   that broader change.
4. Do not reboot merely to clear a message. Check whether the package manager
   requires it and account for rover downtime before rebooting.
5. Verify zero ordinary pending upgrades, a clean `dpkg` audit, SSH and network
   health, rover listeners on ports 8769-8773, the daemon API, console and depth
   health endpoints, the ROS navigation bridge, and `~/ugv/selftest.py`.
6. Distinguish failures that predate the update from failures caused by it. Do
   not change unrelated services just to make `systemctl --failed` empty.

## Definition of done

Report what changed, what was verified on the host that uses it, whether a reboot
is needed, and anything deliberately left alone. Preserve unrelated worktree
changes.
