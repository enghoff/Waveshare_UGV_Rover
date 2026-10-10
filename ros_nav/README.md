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

The current host was measured at 16.14613 gyro units per degree per second
(2026-10-10, against still scan fits; the 15.31 before it had been checked
against slam_toolbox's heading on the spot, which is the gyro's own) and 107.206
encoder ticks per metre. The source and runtime file remain authoritative
over these documentary values.

### Turning scans and odometry's stamp

A scan taken while the rover turns is put back together before it is published.
The D500 sweeps in 0.1 s, from the rover's left clockwise, and on the spot this
chassis turns about 120 degrees a second, so the last point of a sweep is 12
degrees of heading after the first. `lidar_node.py` rotates each point by how far
odometry says the rover turned between the sweep's start and that point's moment
(`scan_deskew.py`), and stamps the scan at the start. `base_node.py` stamps
odometry `ODOM_LATENCY_S` (48 ms) before it arrives, the lag measured between the
two. Replayed over twelve recorded turns with the mapper's heading rule on, these
took the heading after a turn from 9 degrees typical and 16 at worst to 0.6 and
1.0 (`captures/2026-10-10-turns`). `--no-deskew` publishes scans as measured.

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
rover, and pressing "refit to map" is still what corrects one. Nor does it hold
up a drive: it takes no move mutex, so a goal sent during its second of
searching starts as usual (until 2026-10-10 such a goal was refused as busy). The gap it fills
is that nothing else asks. `slam_toolbox` corrects `map -> odom` only when it
folds a scan into the graph, and it will not fold one until the rover has
apparently moved `minimum_travel_distance` or turned `minimum_travel_heading`
(0.2 m and 0.2 rad in `config/slam_toolbox.yaml`), so a parked rover has the
walls in plain sight and consults nobody. The heading half of that rule only
applies with `check_min_dist_and_heading_precisely`, set since 2026-10-10: without
it slam_toolbox 2.8 gates on distance alone, so a turn on the spot was never
corrected and left the heading wrong by whatever the gyro had got wrong. On 2026-09-07 that let the believed
heading creep 174 degrees round over a working day with `position` still reading
"trusted", because slam_toolbox was confident in a match it had made hours
earlier.

The creeping itself is fixed at the source — `base_node.debias` integrates
nothing at all while the wheels report the rover still, rather than subtracting
an estimated gyro offset and leaving whatever the estimate was wrong by. The
check remains, because "the rover cannot tell whether it is wrong" is a separate
fault from "the rover drifts".

The same search is offered read-only, as the bridge's `measure` op
(`nav_map.measure_pose`): half a metre and 45 degrees around where the rover
thinks it is, about a tenth of a second, with no move mutex, refusing a moving
rover and never waiting behind a graph write. On the move it matches the newest
scan from where the rover was half way through that scan's sweep, read out of the
transform tree at the scan's own timestamp (`nav_bridge.pose_at`,
`nav_map.scan_moment`), and refuses a sweep turned faster than
`MEASURE_MAX_TURN_DPS` (30 degrees a second). The world state uses it to check
each look's heading, still or moving (world_state/headingcheck.py). Nothing on the
navigation side acts on the answer. An attempt to correct the rover with it inside every move held
the wheels for fifteen seconds and was reverted on 2026-10-01.

Use the daemon's `clear_map` call to start a new map. That also advances the
world-state map session, so semantic placements from old coordinates are not
treated as current positions.

To save an additional visual map manually:

```bash
. ~/ugv/ros_nav/env.sh
ros2 run nav2_map_server map_saver_cli -f ~/house
```

### What has changed since a place was last seen

`change_node.py` watches the scan for things moved since the rover was last in a
place: an armchair gone from where it stood, something new on floor that was
clear. It is the lidar's half of M5's change detection
([the plan](../docs/plans/autonomous-curiosity.md)), and it exists because the
camera's looks were measured unable to say whether a thing is still there
([2026-10-10](../docs/progress/2026-10-10-m5-what-reports-a-change.md)).

Every 5 cm cell of the map keeps two tallies, one for the visit in progress and
one for the visit before it: in how many scans a beam ended in the cell and in
how many one passed through (`change_watch.py`). A visit to a cell ends after
five minutes unseen. A cell solid in one visit and seen through in the other has
changed, with 10 cm of slack for the pose; changed cells are grouped, and a group
is reported once its evidence spans 20 s, so a person walking past is not one.
Groups 0.6 m or longer are marked furniture-sized; on the recordings it was
measured on, smaller ones were doors. Replayed through today's recordings in
order, it reported nothing for three runs with the armchair moved and then, on
the run after it was put back, the armchair back in its place and gone from
where it had stood, within 0.1 m of its world-state record
(`experiments/m5_changes/replay_watch.py`).

It counts scans only while navigation says the map is settled and the position
trusted and its drift check does not place the rover elsewhere, so a restore
nobody has confirmed is a pause rather than the whole flat changed. That alone
was not enough: on 2026-10-10 a restore 170 degrees out stayed settled for a
quarter of an hour and the watch logged the charger room as changed. So each scan
must also fit what the watch already knows, at least 60% of the hits that land on
known cells landing on solid ones, and one that does not is refused and counted.
Replayed 0.6 m and 20 degrees out, every scan of a 15-minute run was refused; at
the true pose, 10 in 3,169. It takes every third scan not turning faster than 40 degrees a second,
3 ms each on the Orin; the process as a whole takes an eighth of one core, most
of it following the transform tree. It writes `~/.ugv/changes/<map_id>.npz` (the
tallies, every two minutes and at exit), `<map_id>.json` (the changes found,
every 30 s), which the bridge's `changes` op returns, and `<map_id>.log.jsonl`,
each change once, when it is first found, which is what a run's changes are
counted from afterwards. It moves nothing, publishes
nothing, and nothing in the stack reads it, so it can stop without taking
anything with it. It sees one plane about 20 cm up: paintings, table tops and
shoes are not in it.

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

A `drive_to` refused because the rover is standing inside the costmap's
inscribed band (Nav2's START_OCCUPIED, "turn on the spot or back up") backs off
once to the nearest spot the body fits, at most half a metre away, and asks again.
An autonomous drive does this under its own guard, so a stop or the safe area ends
it before anything moves. A second refusal is handed back. The back-off's driving
is reported as part of the drive.

A goal less than 1.5 m away that must arrive facing a given way -- an
autonomous look, or the console's "go to" for a thing -- is driven as a turn to
face it, a straight drive and, when the heading asked for is more than 15
degrees off, a turn to that heading. Handed to Nav2 as one goal, the lattice
planner draws a several-metre loop rather than a turn on the spot, and the
controller, so close to the goal, turns back and forth instead of driving it
(2026-10-07, measured on the rover). Where the rover cannot turn, it falls back
to the one goal. A goal with no heading, which is what a map click sends, is
one goal as before.

Something in the way of a near goal is looked for on the live scan, because
the planner's map does not have a person on it: the planner draws its straight
line through them and the controller, which does see them, will not drive it.
Before it sets off, and every second while it drives, the rover checks the
straight line to the goal against the local costmap. Blocked, it stops and holds
still, checking every second, and drives on once the way clears. After 3 s it
goes round: a way over the live costmap that keeps its centre 0.3 m from
whatever the scan sees, cut into at most four straight legs, each a turn and a
straight drive that stops for anything in its way. With no such way the goal is
handed back as blocked, and one stopped three times is handed back too. It
never swings on the spot over a curve it will not follow.

A goal further away is one Nav2 goal, and the planner sees a person too. Its
costmap has a live layer (`behaviors/`, `LiveObstacleLayer`) that lays the
latest scan within 3 m onto the map at every update, twice a second, using the
map's position as it is at that moment, and keeps nothing between updates. The
planner, replanning once a second, draws its route round whoever stands in the
way, and nothing they leave behind can turn into a ghost wall, because nothing
is remembered. Scan points within 10 cm of a wall the map already has are left
to the map. On the rover a person standing on a 4.4 m leg was driven round
without a stop, the rover's centre passing about half a metre from them
([2026-10-08](../docs/progress/2026-10-08-live-layer.md)). The bridge's
`{"op": "live_layer", "enabled": false}` switches it off at run time and its
marks go at the next update; removing it from the plugin list in
`config/nav2.yaml` takes it out for good. A goal is fitted to the body on the
same costmap, so a spot next to a person or something moved is refused or
moved too, and a refusal for something only the scan has says so rather than
blaming a wall. The bridge's `map` reply carries the layer's marks as `live`,
map-frame cell centres, and the map picture draws them in orange.

Longer goals also check the next metre of Nav2's route on the live scan every
second. Something on it that is not a wall on the map stops the goal, and the
rover goes on as a near goal to a point 1.4 m further along the route, waiting
and going round as above, then on to the goal, at most three times. With the
live layer on, this is for what the planner has not routed round: someone
stepping in close, or standing where there is no way round. Whether something
is a wall is asked of slam_toolbox's map, not the planner's costmap, which has
the person on it as well.

Someone standing where there is no way round, in a doorway, leaves the planner
with no route at all, and Nav2's answer to that is its recoveries: on the rover
a spin of 225 degrees, a wait, a reverse and another spin, 2 m from the person.
So when Nav2 starts a recovery and the planner has sent no route for this goal
in the last 2 s, the goal is stopped there. The rover holds still and asks the
planner itself every second, for the same 3 s a near goal waits; with a route
it drives on, and without one the goal is handed back as blocked. A refusal of
any other kind goes back to Nav2 as it always did.

No goal turns on the spot for ever. A drive that has not got 0.5 m further on
in 25 s while Nav2 attempts no recovery is ended and says so. Nav2's own
progress check counts a 20-degree swing as progress, so it cannot see this.

Exploration chooses reachable frontiers and abandons a goal that makes no useful
progress. It stops when the time budget expires, no useful frontier remains, or
the user cancels it. Status reports why it stopped and how much it drove.

The occupancy map the bridge sends (`nav_grid` on the daemon) carries the body a
walk over it has to respect: `inscribed_radius_m`, how far the planner keeps the
rover's centre from anything occupied (0.20 m, read off the costmap node), and
`goal_fit_reach_m`, how far a goal is moved onto floor where the body fits
(0.5 m). The autonomy executive walks the map with both, so it does not choose
places only a point could get to. Both are missing until the costmap node has
answered once.

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
- The planner keeps the same margin from a person as from a wall: its centre
  0.20 m from what the scan sees, which is the leg, a few centimetres from the
  body's side. In a doorway on 2026-10-09 it took the 45 cm between the
  owner's legs and the wall, and the owner chose to keep that margin. That
  close, the rover counts itself inside an obstacle when it next plans, and
  the back-off for that (`back_off`) turned it on the spot beside them.

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

## Autonomous journey guards

Autonomous `goto` requests carry a stop sequence and the run's safe area. The
bridge refuses a stale sequence before dispatch and cancels a goal accepted after
a stop. It checks the adjusted destination, current position and every published
route against an inset of the safe area, including replans. `autonomy_guard.py`
uses the same boundary arithmetic deployed from the daemon's `permission.py`.
A violation cancels the goal; the daemon independently monitors position.

The inset reserves 0.6 m for the body and stopping, set from stops measured on the
rover on 2026-10-06 at its 0.40-0.45 m/s driving speed (`permission.FENCE_MARGIN_M`
says how). R-SAFE-10 and R-SAFE-12 remain open until supervised moving trials prove
the declared boundary is respected.
