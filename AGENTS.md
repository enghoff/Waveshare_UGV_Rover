# Working in this repository

Rules for changing the rover. Deploy/restart commands, directory ownership and manual
recovery are in [docs/deploy.md](docs/deploy.md); host and network facts are in
[docs/hosts.md](docs/hosts.md); a rover that has dropped off the network is
[docs/rover-unresponsive.md](docs/rover-unresponsive.md). A component's README
describes the component.

## Report in plain English

Write for the person who owns the rover but does not carry its code in their head.
They want the conclusion and the decision, not the workings. Lead with what is the
case now: what works, what does not, and what it means for the rover. Anything they
cannot act on — the investigation, the options weighed, the names of files and
settings — is a supporting clause at most, and usually is not needed at all. Say
plainly when something is unproven, failed or was skipped. Two sentences beat two
paragraphs. Finish with the next step if there is one.

## Reproduce faults before fixing them

**A fix for a fault nobody reproduced is a guess.** Replay the reproduction first,
leave the running system alone until the model fails the way the rover did, then show
the fix succeeding there before deploying. Simulation does not replace hardware:
validate the model against a real recording, still observe the fix on the rover, and
where they disagree the hardware is right. Navigation has `ros_nav/nav_record.py`
with replay and controller simulations ([ros_nav/README.md](ros_nav/README.md));
`wifi_roam/selftest.sh` drives the network scripts against a fake board.

## Where things run

Rover services run on the Jetson Orin Nano (`orin`), which replaced the Banana Pi on
2026-08-31; the realtime voice model is Alibaba's hosted Qwen Omni, and nothing that
is deployed uses the Orin's GPU. What gets deployed is decided by
`deploy/manifest.json`, not by a list here — bench scripts under
`oak_camera/`, `lidar/` and `usb_cameras/` are not deployed, while files from
`face_tracking/`, `voice_chat/` and `driver_board/` are.

## A change is not done until it runs on the host that uses it

**Deploy to whichever host uses the changed files, restart what needs restarting, and
verify the running service there as part of the same work; say what was deployed and
what proved it.** A commit does not sync itself:
[`deploy/deploy.py`](deploy/README.md) copies the registered components that changed
and runs their own restart and verification checks, advancing state only once those
pass. Proof is taken on that machine — call the affected function over TCP 8769 and
read the answer, because "the file was copied" and "a local unit test passes" prove
nothing. A change touching no registered component needs no restart; a changed
manifest or source set wants `deploy.py --plan`.

## The repository is the source of truth

Edit here and push; never edit a tracked file in place on the rover. The deployer
refuses dirty tracked files, because the recorded commit must describe the bytes that
were sent. Where prose and executable source or config disagree, **the source wins**:
correct the document, and never revive an old setting because a README remembers it.

## Credentials

Deployment credentials are one-line gitignored files under `secrets/`; `jetson-orin.key`
is the `jetson` account's login password on the rover and therefore also its sudo
password. The old Banana Pi and Raspberry Pi keys are still there and all three are
different. The secrets the rover itself holds live in `~/.ugv/`, outside the deploy
tree: the DashScope key, the console token, the TLS material and deploy state. Never
put a credential in a commit, transcript, command line, or a path deployment can copy
back; [docs/deploy.md](docs/deploy.md) has the `sudo -S` mechanics.
