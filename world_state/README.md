# Semantic world state

This component records what the rover has seen, where it saw it, and the images
behind those observations. ROS owns the map, pose and routes. World state has no
authority over driving.

The rover does not assign names from a fixed vocabulary. Earlier vision-language
models produced unstable names and unreliable re-identification. Current
perception finds regions with YOLOE and stores DINOv2 and SigLIP2 vectors.
Identity comes primarily from measured geometry; text is used only to search
stored images.

## Runtime

`perception_server.py` listens on loopback port 8776. It prefers TensorRT engines
built for the Orin and falls back to CPU ONNX Runtime when those engines are not
available. Each observation records the backend because vectors from the two
backends are not comparable.

`rover_daemon/rover_world.py` owns capture and background scheduling. It records
through the gimbal camera by default. The OAK can supply ranges for regions that
fall inside its narrower fixed view. The OAK mount rotation is measured; its
translation and the gimbal camera's position relative to the SLAM pose remain
unmeasured.

Runtime data lives outside the deploy tree:

```text
~/.ugv/world/world.db
~/.ugv/world/frames/*.jpg
```

The database grows through additive migrations in `schema.py`. Historical
columns remain readable even when the current pipeline no longer writes them.

## How an observation becomes an entity

An observation keeps its frame, region, capture time, camera, pose, bearing,
elevation, uncertainty, optional OAK range, and appearance vectors. It stays
unplaced until the resolver has enough independent evidence.

The resolver:

1. rejects evidence from another map session;
2. crosses bearings taken from separated viewpoints with enough parallax;
3. checks map visibility, elevation and any measured ranges;
4. uses appearance to reject or choose among geometrically valid candidates;
5. leaves ambiguous observations pending.

A single viewpoint cannot locate a thing. Several objects on the same lines of
sight can still form a false crossing when ranges are absent. Range-assisted
association is implemented but has not yet been validated by a new driven run.

## Install and run

The deployer installs source. Models and TensorRT engines are host-built runtime
assets under `vendor/`:

```bash
ssh orin 'sh ~/ugv/world_state/install_perception.sh'
ssh orin 'sh ~/ugv/world_state/install_gpu_recovery.sh'
ssh orin '~/ugv/world_state/restart_perception.sh'
```

The supervisor starts `run_perception.sh` from the `jetson` user's crontab. It
loads models on the first request and attempts GPU recovery before each start.
Use `restart_perception.sh`; do not kill or launch the child directly.

Health:

```bash
ssh orin "curl -s http://127.0.0.1:8776/health"
```

The response identifies the selected backend, fallback reason, load time and
whether inference is busy.

## Control calls

The daemon exposes control calls on TCP 8769 for the console and diagnostics:

- `world_state_summary`
- `world_state_search`
- `world_state_entities`
- `world_state_entity`
- `world_state_observations`
- `world_state_frame`
- `world_state_viewpoint`
- `world_state_clear`
- `world_map_session`
- `world_inspect`

Voice tools are read-only: `find_thing`, `go_to_thing`, and
`distance_between_things`. Clearing and direct inspection are not shown to the
voice model.

## Verification and diagnostics

Run the offline suite from the repository:

```bash
python world_state/selftest.py
```

It covers storage, migration, geometry, association, perception contracts,
search and OAK range handling with fakes. It does not prove camera calibration,
encoder pose, GPU execution or real-room identity.

Useful replay and measurement tools remain beside the component:

- `replay.py` reruns stored observations through current resolution logic;
- `bench_oak.py` measures the relationship between the two cameras;
- `bench_bearing.py`, `bench_height.py` and `bench_cluster.py` compare geometry;
- `bench_perceive.py` and `bench_still.py` inspect model and capture behavior.

The next hardware proof is a driven run with the OAK awake. It should establish
whether measured range prevents the false crossings seen in bearing-only runs.
