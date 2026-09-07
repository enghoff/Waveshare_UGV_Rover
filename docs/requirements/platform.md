<!-- requirement-area: PLAT -->

# Host, deployment and network

What the machine the rover runs on has to guarantee, and what deployment must not
be able to break. The conventions for these records are in [README.md](README.md);
the procedures are in [runbooks/deploy.md](../runbooks/deploy.md) and
[runbooks/hosts.md](../runbooks/hosts.md).

<a id="r-plat-1"></a>
### R-PLAT-1 — Runtime state lives outside the deploy tree

- **State:** settled
- **Evidence:** [ros_nav/README.md](../../ros_nav/README.md) and
  [world_state/README.md](../../world_state/README.md) — the map under
  `~/.ugv/map/`, the visual memory under `~/.ugv/world/`, chassis calibration at
  `~/ugv/odometry.json`, deploy state at `~/.ugv/deploy-state.json`

A deploy replaces source. It must not be able to carry away the map, the
observations, the chassis calibration or the credentials, because those are the
things that cannot be regenerated from the repository. This is what makes
deploying safe enough to do often.

The corollary is that replacing the rover's computer loses everything in
`~/.ugv/` unless it is copied deliberately.

<a id="r-plat-2"></a>
### R-PLAT-2 — The rover's secrets exist only on the rover

- **State:** settled
- **Evidence:** [runbooks/deploy.md](../runbooks/deploy.md);
  `deploy/guards/rover_guard.py`; `.githooks/pre-commit`

The DashScope key and the TLS material live under `~/.ugv/`, outside the deploy
tree, so no deploy can copy them back into a place the repository would see.
Deployment credentials on the workstation are gitignored files under `secrets/`.
A shell command that would carry a credential is refused mechanically rather
than relying on care.

<a id="r-plat-3"></a>
### R-PLAT-3 — What the rover runs is described by a commit

- **State:** settled
- **Evidence:** [deploy/README.md](../../deploy/README.md) — the deployer refuses
  dirty tracked files and records the commit it sent

The recorded commit has to describe the bytes that were sent, or the record is
worse than none: it would name a version the rover is not running. This is why
tracked files are never edited in place on the rover, and why a blocked deploy is
worked around with a clean detached worktree rather than by deploying a dirty
tree.

<a id="r-plat-4"></a>
### R-PLAT-4 — Deployment state advances only after the component's own checks pass

- **State:** settled
- **Evidence:** [deploy/README.md](../../deploy/README.md);
  `python -m unittest deploy.test_deploy`

Each component in the manifest carries its own restart and verification
commands, and the recorded state moves forward only when they succeed. A failed
verification therefore leaves the component looking undeployed, which is the
honest answer — copying files is not deploying.

<a id="r-plat-5"></a>
### R-PLAT-5 — The rover comes up on one known network and never changes it by itself

- **State:** settled
- **Evidence:** [wifi_roam/README.md](../../wifi_roam/README.md)

It joins one network at every boot on its onboard radio and stays there. It holds
profiles for the other house networks with autoconnect off, so the only thing
that moves it is a person at the console. Nothing scans unprompted, nothing
roams, and nothing hands the link between radios.

The history behind that flatness is worth knowing before anything is made
cleverer: see [runbooks/rover-unresponsive.md](../runbooks/rover-unresponsive.md).

<a id="r-plat-6"></a>
### R-PLAT-6 — The rover answers at one address whichever network it is on

- **State:** settled
- **Evidence:** [wifi_roam/README.md](../../wifi_roam/README.md);
  [runbooks/hosts.md](../runbooks/hosts.md)

The service address is a fixed extra address on every profile, alongside the DHCP
lease. The house networks are separate names bridged onto one LAN, so the rover
is reachable at that one address including on a network somebody chose by hand,
and the console's certificate names it — which is what makes the browser trust it.
Per-radio DHCP leases still work and are the way back in when the service address
does not answer.

<a id="r-plat-7"></a>
### R-PLAT-7 — Perception degrades rather than failing when the GPU is unavailable

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md) — the
  sidecar prefers TensorRT engines and falls back to CPU ONNX Runtime; its
  `/health` response names the backend and the fallback reason

Falling back changes what the vectors mean, which is why every observation
records its backend — see [R-WS-4](world-state.md#r-ws-4). Silently comparing
across the two would be worse than not running at all.

<a id="r-plat-8"></a>
### R-PLAT-8 — Perception recovers from a boot that brings up no GPU

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md) —
  `install_gpu_recovery.sh`; the supervisor attempts recovery before each start

Some boots on this host come up with no working GPU. Recovery is attempted before
each start of the sidecar rather than left to a person noticing that inference
has become slow.

<a id="r-plat-9"></a>
### R-PLAT-9 — A kernel update does not silently drop the USB radio

- **State:** settled
- **Evidence:** [dongle_driver/README.md](../../dongle_driver/README.md) — the
  driver is built with one patch and registered with DKMS

The vendor kernel ships no driver for the rover's second radio, so it is built
here. Registering it with DKMS means a JetPack kernel update rebuilds it instead
of leaving the rover with one radio fewer and no error to explain it.

<a id="r-plat-10"></a>
### R-PLAT-10 — The depth camera's driver version is pinned deliberately

- **State:** settled
- **Evidence:** [decisions/depthai-version-pin.md](../decisions/depthai-version-pin.md);
  [oak_depth/README.md](../../oak_depth/README.md)

The OAK has no flash and the host uploads its firmware on every open, so the pin
is not a desk dependency — it decides what the camera runs. Tested 3.x releases
fail stereo on this unit. Relaxing it requires re-running the depth preview and a
single-camera capture on the rover.

<a id="r-plat-11"></a>
### R-PLAT-11 — Runtime data, model weights and databases stay out of Git

- **State:** settled
- **Evidence:** `.gitignore`; `deploy/guards/rover_guard.py`;
  [world_state/README.md](../../world_state/README.md) — models and TensorRT
  engines are host-built assets under `vendor/`

Frames, the observation database, the map, perception weights and engines are
never committed. They are large, they are specific to one host, and a repository
that carried them would stop being a description of the rover's source.

The consequence for anything that measures the rover is that evidence is
referenced by identifier and by the recording it came from, rather than
committed — which is why a capture keeps a manifest naming the run, the commit
and the host. See
[../progress/README.md](../progress/README.md).
