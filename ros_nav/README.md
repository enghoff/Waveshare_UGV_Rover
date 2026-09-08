# ROS 2 mapping and navigation

`ros_nav` runs ROS 2 Jazzy, `slam_toolbox` and Nav2 on the Jetson Orin. The rover
daemon remains the only owner of the driver-board UART. It lends odometry and
motor commands to ROS on loopback port 8772; `nav_bridge.py` returns navigation
status and actions to the daemon on port 8773.

The current mapper is `slam_toolbox`. RTAB-Map was tested and removed. The D500
lidar feeds `/scan`; `base_node.py` publishes odometry and accepts `/cmd_vel`.

## Install and start

ROS lives in `~/miniforge3/envs/ros`, installed without root:

```bash
ssh orin 'sh ~/ugv/ros_nav/install.sh'
ssh orin 'sh ~/ugv/ros_nav/install-boot.sh --nav'
ssh orin '~/ugv/ros_nav/restart.sh'
```

`install-boot.sh --nav` writes the ROS supervisor entry and checks that the
daemon starts with `--board-bridge --ros-nav`. Use `restart.sh`; pass
`--supervisor` after changing the launch environment or supervisor scripts.

For an interactive shell:

```bash
ssh orin
. ~/ugv/ros_nav/env.sh
. ~/ugv/ros_nav/dds.sh
ros2 topic hz /scan
ros2 lifecycle get /slam_toolbox
ros2 run tf2_ros tf2_echo map base_link
```

Both environment files require Bash. `dds.sh` confines rover discovery to
loopback. A workstation running RViz should source `env.sh` without `dds.sh`.

## Calibration

`base_node.py` refuses to run without `~/ugv/odometry.json`. That file contains
the chassis-specific gyro scale, encoder distance and motor curves. It is runtime
state and is not deployed. Copy it when replacing the rover computer; remeasure
with `calibrate_chassis.py` only if it cannot be recovered.

The current host was measured at 15.310723 gyro units per degree per second and
107.206 encoder ticks per metre. The source and runtime file remain authoritative
over these documentary values.

## Maps and localization

The stack periodically serializes its pose graph and last trusted pose under
`~/.ugv/map/`, outside the deploy tree so a deploy cannot take the map with
it. On restart it loads that graph and takes the rover to be standing where the
map says it was parked, because nobody drove it while it was switched off.

`slam_toolbox` anchors the graph it has just read with its own scan matcher,
which lands within a quarter of a metre and twenty degrees of the saved pose
ordinarily and much further when loop closure fires in a room whose two ends
look alike. That anchor is the mapper's answer rather than the rover's belief,
so an anchor landing on the saved pose confirms it and an anchor landing
somewhere else does not. Either way the map is kept; what changes is whether the
keeper may write over the saved pose. It may not until something has confirmed
where the rover stands, so a session that cannot place itself cannot overwrite
the map that would have let the next one try again. `nav_status` reports that as
`map_settled` alongside `map_kept`, and `map_note` says which happened.

**No boot fits the rover to the map on its own.** A fit moves the rover on one
scan matched against a stored graph, and the case it exists for — somebody
carried the rover while it was off — is the case where that scan agrees with the
map least, so it belongs to a person: the console's "refit to map" button, or
the daemon's `refit_pose`. While the rover has not moved since it woke and its
anchor is still unconfirmed, that fit searches around the pose the map was left
at rather than around the anchor, which is what lets it undo a large error at
all.

**But the rover does check, and says so.** Every five minutes, while it is not
driving, it matches the current scan against the whole map and reports the
disagreement as `map_drift` in `nav_status` — shown on the console's navigation
panel as `vs lidar`, reading `agrees`, `cannot say`, or `OFF BY 43 cm, 174 deg`.
Nothing acts on it: it writes no pose, touches no graph and does not move the
rover, and pressing "refit to map" is still what corrects one. The gap it fills
is that nothing else asks. `slam_toolbox` corrects `map -> odom` only when it
folds a scan into the graph, and it will not fold one until the rover has
apparently moved `minimum_travel_distance` or turned `minimum_travel_heading`
(0.2 m and 0.2 rad in `config/slam_toolbox.yaml`), so a parked rover has the
walls in plain sight and consults nobody. On 2026-09-07 that let the believed
heading creep 174 degrees round over a working day with `position` still reading
"trusted", because slam_toolbox was confident in a match it had made hours
earlier.

The creeping itself is fixed at the source — `base_node.debias` integrates
nothing at all while the wheels report the rover still, rather than subtracting
an estimated gyro offset and leaving whatever the estimate was wrong by. The
check remains, because "the rover cannot tell whether it is wrong" is a separate
fault from "the rover drifts".

Use the daemon's `clear_map` call to start a new map. That also advances the
world-state map session, so semantic placements from old coordinates are not
treated as current positions.

To save an additional visual map manually:

```bash
. ~/ugv/ros_nav/env.sh
ros2 run nav2_map_server map_saver_cli -f ~/house
```

## Movement

The daemon offers:

- `drive` for a short straight move checked against the local costmap;
- `turn_in_place` for a bounded rotation;
- `drive_to` for a relative metric goal planned around obstacles;
- `drive_to_map_point` for a point selected on the rendered map;
- `explore` for background frontier exploration;
- `stop_driving` to cancel movement.

The lidar sees one horizontal plane. None of these operations can detect drops,
steps, table tops or obstacles entirely above or below that plane.

Exploration chooses reachable frontiers and abandons a goal that makes no useful
progress. It stops when the time budget expires, no useful frontier remains, or
the user cancels it. Status reports why it stopped and how much it drove.

## Known limits

- A rover standing on a small island of mapped floor — which is what a cleared
  map leaves — treats the whole rim of unknown around it as one frontier, is sent
  to its centre, and retires the lot on arriving without having moved. Exploring
  then reports a finished house it has not driven in.
  `fixtures/ringed-2026-09-07.json.gz` is such a map and `selftest.py` replays
  it. Cutting the rim into pieces with a goal each was tried and reverted,
  because the pieces are all within a metre of the rover and point every way at
  once: 869 degrees of turning for 44 cm of progress. See
  [R-NAV-6](../docs/requirements/navigation.md#r-nav-6) and
  [the decision](../docs/decisions/rim-frontiers-are-not-cut-up.md).
- The local controller can aim around a corner into an inflated wall. Shorter
  plan pruning reduces this, but a full controller fix remains open.
- The minimum pivot response is coarser than the smallest angular velocity DWB
  can request. The floor should ultimately derive from the measured motor curve.
- A rover already touching inflated cost may have no valid route out. A short
  manual reverse can be required before replanning.
- Long routes may legitimately detour because this differential-drive chassis
  cannot follow every geometric shortcut.

These are hardware/navigation issues. Reproduce them with a recording or the
provided simulator before changing configuration.

## Reproduction and tests

`nav_record.py` records pose, plans, costmaps and controller evaluation.
`dwb_replay.py`, `smac_replay.py`, `trap_sim.py`, `steering_sim.py` and the other
bench scripts replay failures without moving the rover.

```bash
python ros_nav/selftest.py
python ros_nav/dwb_replay.py ros_nav/recordings/trap-2026-08-25-spin.json --drive
```

The offline suite verifies configuration, mapping, control and replay models. A
navigation change is complete only after the reproduced case passes and the
running rover is observed through TCP 8769.

## Troubleshooting

If Nav2 starts but the rover does not move, ask `nav_status` first. Check
`board_ok`, `lidar_live`, `position_trusted`, `nav2_ready`, and scan/transform
age. A healthy lidar with no odometry usually means the calibration file or
board bridge is missing.

If the rover is drawn on the map facing the wrong way, ask `nav_status` for
`map_note` and `map_fit`. A rover whose pose is further out than a fit can search
— the window is a metre and forty-five degrees around where it thinks it is — is
recoverable by widening the search once, by hand:

```bash
python3 -c 'import json,socket;s=socket.create_connection(("127.0.0.1",8773));\
s.sendall(json.dumps({"op":"refit","window_deg":180}).encode()+b"\n");\
print(s.makefile("r").readline())'
```

Adding `"min_score": 1.01` makes the same call measure without applying
anything, which is how to find out where the scan really fits before moving the
rover. A wide window can place the rover confidently in the wrong one of two
rooms that look alike, so it is worth measuring first.

A rover that was driven while badly anchored is the one case past what any of
this can find: the pose the map was left at is no longer where it is, and its
own pose is wrong by however far the anchor was out. Park it roughly where the
map thinks it is and refit, or clear the map and start again.

Logs are under `~/ugv/ros_nav/`. Restart the component through
`~/ugv/ros_nav/restart.sh`; the deploy manifest defines the required build,
restart and readiness checks.
