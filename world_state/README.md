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

It survives a reboot, because the navigation stack keeps its pose graph and the
coordinates every row is measured in therefore still mean what they meant. Three
things move it:

- **Clearing the map clears the world state with it.** One button does both, and
  the rover does both: `clear_map` empties the store and re-points it at the new
  map before it answers, so the reply says what went. Everything the store holds
  is a position in the map's frame or a bearing from a pose in it, so what
  survived a map clear was a list of things with nowhere to be. The console used
  to make the world's half of the call itself, guarded by a flag its world panel
  sets and nothing else does; on 2026-09-06 a map cleared with that panel shut
  left 423 things behind, measured against a map that had gone, and said nothing.
- **A map that changed without being cleared starts a new map session and keeps
  the record.** A restore that failed, or a graph built from scratch, gives the
  navigation stack a different map identity; `follow_map` notices within seconds
  and moves the session. The rows stay, shown as measured against a map that has
  gone rather than drawn in this room.
- **The things an older map stranded can be carried onto this one in a block**,
  which is what clears a backlog: two maps of one room differ by a turn and a
  shift, and the things that appear in both are enough to find it. See
  `reanchor.py`, which is run by hand rather than on a schedule.
- **A thing whose map was replaced is recognised when it is seen again, rather
  than met as a stranger.** The coordinates expired; the crops did not. So the
  first crossing in the new map that plainly looks like something the rover
  already owns takes that thing's identity back, with its history intact --
  `resolve._adopt`, gated at `RECOGNISED` and refused where two known things look
  equally like it. Until this existed nothing could ever re-place an orphaned
  thing, because the resolver considers only what is placed in the map it is
  working in: replayed across the map change of 2026-09-06, 272 things came out
  of which 99 could be driven to and none were still what the rover had spent the
  previous day learning.
- **`world_state_clear` on its own empties the store and leaves the map alone**,
  which is what a repeatable experiment needs.

The clear is refused while a look is in flight, after waiting `CLEAR_WAIT_S` for
it. The console reports that as `not cleared` with the reason beside it, and the
session moves on regardless, so rows that survived a refusal are marked as
belonging to the map that has gone.

## How an observation becomes an entity

An observation keeps its frame, region, capture time, camera, pose, bearing,
elevation, uncertainty, optional OAK range, and appearance vectors. It stays
unplaced until the resolver has enough independent evidence.

The resolver:

1. rejects evidence from another map session, while still offering a new
   crossing to the things that map left behind;
2. crosses bearings taken from separated viewpoints with enough parallax;
3. checks map visibility, elevation and any measured ranges;
4. uses appearance to reject or choose among geometrically valid candidates;
5. leaves ambiguous observations pending.

A single viewpoint cannot locate a thing. Several objects on the same lines of
sight can still form a false crossing when ranges are absent. Range-assisted
association was validated on the driven run of 2026-09-07 and it does help: of
100 things bearings alone placed, 11 had no counterpart once the ranges were
carried, and the two that were checked by eye were plainly false -- a pool of
blown-out floor placed below the floor, and a blue case pooled with a red wooden
surface near the ceiling. It refuses more than it invents. See
[`docs/m0-semantic-world-state-baseline.md`](../docs/m0-semantic-world-state-baseline.md),
which also says why the ranges themselves are not yet trustworthy as ranges *to*
anything: the mount constant the boxes are placed through is 6.2 degrees out.

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
- `world_inspect`

There was a `world_map_session` here and it has been removed. It read as a
question and was an instruction: every call minted a new session, which is to say
it told the rover that everything it had located belonged to a map that no longer
existed. The session moves for one reason now -- the map identity underneath it
changing -- and the number is readable in `world_state_summary`.

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

- `replay.py` reruns stored observations through current resolution logic,
  following the map changes the recording itself went through (`--session`
  pins it to one), and reports how much survived them;
- `reanchor.py` lines an old map up with the one the rover is on and carries
  the things it stranded across, folding away the copies of them that were
  found again in the meantime; it writes nothing without `--apply`;
- `bench_oak.py` measures the relationship between the two cameras;
- `bench_bearing.py`, `bench_height.py` and `bench_cluster.py` compare geometry;
- `bench_perceive.py` and `bench_still.py` inspect model and capture behavior.

That proof was taken on 2026-09-07 and measured range does prevent false
crossings. The next hardware proof is a different one, and it is not about this
camera at all. Sweeping the gimbal through a position from either side -- which
needs an overshoot, or the approach is only ever opposed at pan 0 -- shows the
gimbal camera **carries about a degree and a half of backlash at every angle in
its travel**, against a same-direction floor of five hundredths. The OAK is bolted
to the chassis, so nothing but the gimbal's own pointing can account for it. That
alone is the whole of the 1.5 degrees a bearing here is believed to, and which way
the gimbal last moved is recorded nowhere, so it cannot be corrected afterwards.
Two more faults ride with it: a gain-like walk somewhere between four and eight
per cent -- the bench cannot pin it closer, because the room moves while it
measures -- which vanishes straight ahead and reaches one to two degrees at pan 30,
and a roll that moves with
pan and that neither of the other two can produce. This component never aims the
camera -- it captures wherever the gimbal is -- so its looks have happened to be
straight ahead so far, but a face being tracked or a `look_at` puts them out at
wide pan where the gain is worst. Measure the pan servo's commanded angle against
its actual one before anything else; it is the largest correctable term. Until
then the OAK's mount cannot honestly be re-measured, and `oak.MOUNT` has been left
alone rather than moved to one of several numbers that disagree.
