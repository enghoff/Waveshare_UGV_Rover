# Working in this repository

Rules for changing the rover. Deploy/restart commands, directory ownership and manual
recovery are in [docs/deploy.md](docs/deploy.md); host and network facts are in
[docs/hosts.md](docs/hosts.md); a rover that has dropped off the network is
[docs/rover-unresponsive.md](docs/rover-unresponsive.md). A component's README
describes the component.

## Report in plain English

Lead with what is true in words a person can use — the rover moved, the network
stayed up, a service restarted, a recorded fault reproduced. File names and tick
counts are supporting evidence, not the first sentence. If the obvious story is
wrong, say so briefly and name the real one. Prefer numbers with physical meaning
over raw log volume, and end with the single next action and what would count as
proof rather than a menu of guesses.

## Reproduce faults before fixing them

**A fix for a fault nobody reproduced is a guess.** Replay the reproduction first,
leave the running system alone until the model fails the way the rover did, then show
the fix succeeding there before deploying. Simulation does not replace hardware:
validate the model against a real recording, still observe the fix on the rover, and
where they disagree the hardware is right. Navigation has `ros_nav/nav_record.py`
with replay and controller simulations ([ros_nav/README.md](ros_nav/README.md));
`wifi_roam/test_wifi_dual.py` replays a captured outage against the real event.

## Where things run

Rover services run on the Banana Pi (`bpi-m4zero`); the realtime voice model is
Alibaba's hosted Qwen Omni, with no local GPU/MEDIA deployment. What gets deployed is
decided by `deploy/manifest.json`, not by a list here — bench scripts under
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

Deployment credentials are one-line gitignored files under `secrets/`; `bpi-sudo.key`
is `admin`'s sudo password on the Banana Pi, and the old Raspberry Pi password is
different. The secrets the rover itself holds live in `~/.ugv/`, outside the deploy
tree: the DashScope key, the console token, the TLS material and deploy state. Never
put a credential in a commit, transcript, command line, or a path deployment can copy
back; [docs/deploy.md](docs/deploy.md) has the `sudo -S` mechanics.
