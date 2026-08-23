# `ros_nav/` — ROS 2 mapping and navigation on the rover

A map that closes its loops, and a navigation stack that plans on it. This
replaces the SLAM and the planning in [`lidar_slam/`](../lidar_slam) with
`slam_toolbox` and Nav2, running on the Banana Pi itself under ROS 2 Jazzy.

## Why this exists

The scan matcher in `lidar_slam/slam2d.c` is good and fast and has one deliberate
hole, which its own README is candid about: **no loop closure**. The pose drifts
monotonically and for ever, so driving a circuit leaves the two ends of it in
different places, and "go back to where you started" is not something the rover
can do. The reason it has no loop closure is arithmetic — the note there works
out that one closure attempt over `slam_toolbox`'s default search window would be
about nineteen seconds on the 700 MHz single-core Pi that code was written for.

The rover is not that board any more. It is a quad-core Cortex-A53 with NEON, and
`slam_toolbox` does that search multi-resolution, threaded, in optimised C++. So
the thing that could not be afforded is now affordable, and the way to get it is
not to write it again — it is to run the implementation that thousands of robots
already run.

What is bought along with it: a pose graph rather than a dead-reckoned pose, a map
that grows to fit the building instead of a fixed 40 m square, Nav2's planner and
recoveries, and a standard set of topics that any ROS tool can look at.

## What runs where

```
  the driver board (ESP32, GPIO UART)
        |  owned by the rover daemon, which also has the lights, the gimbal
        |  and the pack voltage and should not have to give any of them up
        v
  rover_daemon --board-bridge          loopback TCP 8772
        |                              wheels and gyro out, motor commands in
        v
  base_node.py  --> /odom, /imu/data_raw, odom -> base_link
  lidar_node.py --> /scan                       (the D500, on its own USB port)
        |
        v
  slam_toolbox  --> /map, map -> odom
        |
        v
  Nav2          --> /cmd_vel  (back to base_node, and out to the wheels)
```

The daemon keeps the board because it is also everything else the board does.
Only one process can hold a UART, so `board_bridge.py` inside the daemon lends out
the two things ROS cannot do without — the encoders and gyro at 50 Hz, and the
motor commands — over loopback. See
[`rover_daemon/board_bridge.py`](../rover_daemon/board_bridge.py).

The lidar is a separate USB device, so ROS simply takes it. That is why the
daemon must now run **without `--lidar`**: two processes on one serial port is two
half-conversations, and the daemon would win.

`lidar_node.py` does not re-implement the D500's packet format. It creates a
`Slam2D` from `lidar_slam/` and uses it as a parser only — `feed()` in, `scan_xy()`
out — because that code already reads this sensor in 0.3 ms where Python takes 25.
`update()`, which is the scan match and the occupancy grid, is never called.

## Installing it

Three steps, and the first is the only slow one.

```bash
scp -r ros_nav bpi-m4zero:~/ugv/
ssh bpi-m4zero 'sh ~/ugv/ros_nav/install.sh'          # ~20 min, ~4.7 GB
ssh bpi-m4zero 'sh ~/ugv/ros_nav/install-boot.sh'     # the @reboot entry
```

Note the `sh` in front of both. A checkout that arrived by `scp` is mode 644, so
the shebang is never consulted and running the path directly fails with
"Permission denied" — which reads as a sudo problem and is not one.

`install.sh` needs no root at all, which is the whole reason it is conda and not
apt: this board runs Debian trixie and ROS 2 Jazzy's own packages are built for
Ubuntu noble, so there is nothing to `apt install`, and building from source on
four A53 cores is most of a day. [RoboStack](https://robostack.github.io/)
publishes the same releases as conda packages with `linux-aarch64` builds.

`install-boot.sh` also checks the daemon's crontab entry, because the two are a
pair: the daemon must have `--board-bridge` and must not have `--lidar`. If that
is wrong it prints the `sed` line that fixes it.

## Running it

```bash
ssh bpi-m4zero '~/ugv/ros_nav/restart.sh'        # ~30 s; prints the node list
ssh bpi-m4zero 'tail -f ~/ugv/ros_nav/ros_nav.log'
```

And by hand, which is what to do when something is wrong:

```bash
ssh bpi-m4zero
. ~/ugv/ros_nav/env.sh                # bash only -- see below
ros2 topic list
ros2 topic hz /scan                   # should be about 9.9
ros2 lifecycle get /slam_toolbox      # must say 'active'
ros2 run tf2_ros tf2_echo map base_link
```

**`env.sh` must be sourced from bash.** RoboStack's activation hooks are bash
scripts that call `source`, which dash does not have, so `sh -c '. env.sh'` fails
with "source: not found" — a message naming neither the file nor the shell, which
reads as a missing package. Every launcher here starts `#!/bin/bash` for that
reason.

## Saving a map

```bash
ssh bpi-m4zero
. ~/ugv/ros_nav/env.sh
ros2 run nav2_map_server map_saver_cli -f ~/ugv/maps/house
```

which writes `house.pgm` and `house.yaml`. `slam_toolbox` can also serialise its
whole pose graph, which is the thing to keep if the map is ever to be *continued*
rather than just used:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: /home/admin/ugv/maps/house}"
```

## Sending it somewhere

With Nav2 running (`install-boot.sh --nav`, or `run_ros_nav.sh --nav`):

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}}}}"
```

## The calibrations, and why they were not optional

`~/ugv/odometry.json` holds what has been measured about this chassis. The base
node reads all of it from there and invents none of it.

```bash
ssh bpi-m4zero
. ~/ugv/ros_nav/env.sh
python ~/ugv/ros_nav/calibrate_chassis.py --dry-run   # checks everything, moves nothing
python ~/ugv/ros_nav/calibrate_chassis.py --turns     # turn curve; needs room to rotate
python ~/ugv/ros_nav/calibrate_chassis.py             # and the speed and tick curves
```

**The gyro's scale** is right, and was re-checked here against the walls rather
than taken on trust: four bursts, `map → base_link` yaw as the reference, and the
two agreed to within half a percent (`calibrate_chassis.py --gyro`, which nudged
`gyro_lsb_per_dps` from 15.234 to 15.311). Without a scale the base node exits
rather than starting, because heading is the one thing dead reckoning cannot do
without — a mapper handed a rover that appears never to turn folds the room in on
itself.

**The gyro's zero-offset is the one that mattered, and it is not a scale at all.**
Standing perfectly still this gyro reports **+0.47 °/s** — 1708 degrees an hour,
more than four full rotations of pure invention. That is invisible in any short
measurement: over a four-second calibration burst it is two degrees and vanishes
into the noise. Over a circuit it is everything.

Two loop tests over the same route left dead reckoning 37° and 34° from where the
map put the rover. The route took about 80 seconds, and 0.47 °/s for 80 seconds is
38°. Both, near enough exactly.

Getting there meant ruling things out in order rather than guessing, and the order
is the useful part:

- **The scale was exonerated by measurement** — 0.5% against the walls, which
  cannot produce 37° without 7000° of rotation to accumulate it over.
- **The map was exonerated by inspection.** Either the rover really had rotated
  40° more than the map thought, or the map was mis-rotated, and those need
  different fixes. Saving the map settled it: straight single walls, square
  corners, beams fanning out through doorways. A 40° mis-rotation draws the same
  wall twice.
- Which leaves the offset, and an order-of-magnitude check that landed on it.

`base_node.debias` estimates it whenever the rover is genuinely still — nothing
commanded *and* the wheels not turning, since a pushed rover is not a still one —
as a slow exponential average, because the offset drifts with temperature over
minutes. It logs what it has found every thirty seconds. This is the part of
`robot_localization` that matters, done directly, until the full filter is fitted.

**The turn curve** had to be measured, and finding that out cost two wrong
answers. The constants in `lidar_slam/nav_types.py` say PWM 80 turns this rover at
31.6 deg/s and PWM 180 at 170. They were true of the rover as it was, on a
different board, floor and battery, and `nav_types.py` warns in as many words that
stale numbers show up as a consistent over- or under-shoot in the same direction
on every turn. Both models built on them failed on the hardware:

| how PWM was chosen for a commanded 20 deg/s | PWM | the rover actually turned |
|---|---|---|
| proportional from zero | 93 | 25 deg/s |
| straight-line fit to the two old constants | 72 | 8 deg/s |
| interpolated in a curve measured on this chassis | 86 | 19 deg/s |

Measured here, this chassis turns at 0.6 deg/s at PWM 60, 2.4 at 75, 26 at 90 and
61 at 120. It is not proportional and it does not pass near the origin — below
about PWM 85 the tracks do not turn, they shuffle. Those points are reported and
deliberately kept out of the curve, and `pwm_for` never extrapolates below the
slowest rate it has actually seen, because both of the failures above were
extrapolations.

Turn rates now land within about 15% of what was asked, against 60% out before.
That is good enough because Nav2 closes its own loop on the pose; it was not good
enough before, because a rotation that delivers 40% of what was requested makes
every recovery behaviour time out.

**The wheels** are measured now — `ticks_per_metre` is about 147, having been
`null` from the day the file was created. Getting there needed one more idea,
because the obvious way round is circular: the forward distance was to come from
the map, but `slam_toolbox` only adds a scan to its pose graph once *odometry*
says the rover has moved `minimum_travel_distance`, and odometry's distance is
exactly the number being calibrated. With no tick scale it reports the commanded
speed, which is zero when the calibration drives the board directly — so the map
sat frozen at the origin and every run dutifully reported moving 0.000 m while the
rover was crossing the room.

So forward distance comes from the **lidar** instead: how much closer the wall
ahead got. That depends on nothing but the wall. It does depend on going straight,
so the gyro is watched alongside and a run that curved more than 8 degrees is
thrown away rather than quietly recording the chord of an arc as the arc.

**The speed curve is the weakest thing here and should be re-measured.** Four
single runs gave 0.28, 0.37, 0.29 and 0.43 m/s at PWM 80, 95, 115 and 140 — not
monotonic, so PWM 115 apparently being slower than PWM 95 is noise, and the top
point exceeds the 0.35 m/s that `nav_types.py` calls this rover's maximum. The
tick ratios behind `ticks_per_metre` spanned 108 to 173 across the same four runs.
Both want repeats and a longer straight than the room allowed. The good news is
that it bootstraps: with a tick scale in place, odometry translates, so
`slam_toolbox` gets a real motion prior, the map unfreezes, and a re-run can use
the map — which does not care about wall geometry — for a cleaner answer.

**It drives the rover.** Turns happen on the spot and need only room to rotate;
the forward runs need clear floor and back the rover up between points to recover
it, because a sweep of turn bursts walks a skid-steer chassis along — measured
here, sixteen bursts moved it 2.4 m and left it 34 cm from a wall. It refuses to
start without room, stops if anything comes inside the margin, and stops the
wheels on every exit path including a crash. Somebody should be watching it.

Re-run it after anything that changes the drag: a different floor, worn tracks, or
a flat battery — and note that the runs above were taken as the pack went from 65%
to 10%, which is itself a reason to take them again.

## What this costs on the board

Measured with the mapping stack up and the daemon running beside it:

| process | CPU | RSS |
|---|---|---|
| `lidar_node.py` | ~32% of one core | 57 MB |
| `base_node.py` | ~27% of one core | 61 MB |
| `async_slam_toolbox_node` | ~4%, bursty — see below | 38 MB |
| `rover_daemon.py` | ~5% | 21 MB |

About 70% of one core out of four for mapping alone. With Nav2's seven servers on
top as well, and the depth service and drive console beside them, the whole board
measures:

```
over 10 s, 4 cores:  user 33.7%  system 6.0%  idle 58.6%
BUSY 41.4% of all four cores together -- 2.3 cores spare
```

**Ignore the load average here.** It sits between 4 and 6 on a four-core board,
which reads as fully saturated, while 58% of the machine is idle. Load counts
threads that are runnable, and a dozen ROS nodes each running several Cyclone DDS
threads produce a lot of those without any of them wanting much CPU. The number
that means something is `/proc/stat`:

```bash
ssh bpi-m4zero 'grep "^cpu " /proc/stat'   # twice, and difference the idle column
```

Both Python nodes are dearer than they look, and for the same reason: they turn a
few hundred numbers into messages, in Python, twenty or a hundred times a second.
That is the price of not writing them in C++, and it is worth paying for code
anybody can read. Two things have already been taken back cheaply, and both are
worth knowing about before reaching for a third:

- `lidar_node` polls the serial port every **10 ms**, not every 5. A wake collects
  five packets instead of two and delivers exactly the same 9.9 Hz scan, for
  half the CPU — it went from ~50% of a core to ~32%. `--poll` is the knob.
- `base_node` publishes on the **board's** timestamp rather than the bridge's. The
  bridge polls at 50 Hz and the ESP32 speaks at about 18, so keying on the bridge
  put duplicate poses on `/odom` and — worse — computed every velocity over an
  assumed 20 ms interval when the real one was nearer 60, reporting speeds three
  times what the rover was doing.

If more is ever needed, `lidar_node`'s binning loop is the next thing to move into
the C library that is already parsing the packets.

`slam_toolbox`'s steady cost is small; what it does is bursty, near nothing
between scans and real work on a loop closure, so the average above understates
the peaks. The number to watch is whether `map -> odom` keeps being published
while a closure runs — async mode exists so that it does.

## Proving the loop actually closes

`loop_test.py` drives a closed circuit and compares two poses at the end of it:
`odom -> base_link`, which is dead reckoning and has no idea it has been here
before, and `map -> base_link`, which is slam_toolbox having matched every scan
and optimised its graph. It also records `map -> odom` throughout, because that is
the correction, and a step change in it is a closure firing.

```bash
python ~/ugv/ros_nav/loop_test.py --dry-run              # surveys the room, moves nothing
python ~/ugv/ros_nav/loop_test.py --corners 2 --side 2.5 # an out-and-back
python ~/ugv/ros_nav/loop_test.py --side 1.5             # a square
```

It deliberately does not use Nav2. A test of SLAM should not depend on a
controller: if a recovery behaviour spun the rover the circuit would not be the
circuit, and the closure error would be measuring the controller. Driving Nav2
round the same square is a good test *of Nav2*, and a separate one.

**Measured, over a 5 m out-and-back on tile:**

| | position error | heading error |
|---|---|---|
| dead reckoning | 1.295 m | +37.1 deg |
| the map | 0.491 m | -0.8 deg |

with `map -> odom` moving 1.045 m and -38 deg over the run and a **single step of
0.619 m** in it. A scan match nudges the pose by centimetres; a step that size is
the pose graph bending, which is the thing `lidar_slam/` cannot do at all.

Read the two columns differently, though. The heading is conclusive: 37 degrees of
accumulated error became under one. The position is only suggestive, because
nothing guarantees the rover physically returned to its start -- a leg cut short by
the lidar guard means some of that 0.49 m is real distance rather than error. If a
run reports the two errors as similar, check whether the legs completed before
concluding that closure did not fire.

Both of those runs predate the gyro-offset fix above, so they are the *hard* case:
dead reckoning was carrying 38 degrees of phantom rotation and the graph removed
it anyway. Re-run on a charged pack to see what it does now.

## What is deliberately not here

**AMCL and the map server.** They localise against a map saved earlier, and this
rover maps as it goes. `slam_toolbox` already publishes the `map -> odom`
transform AMCL would, and running both puts two things in charge of one transform.

**The OAK-D and the gimbal camera.** The OAK is owned by
[`oak_depth/`](../oak_depth) and serves the drive console on 8770; only one
process can hold it, so folding its depth into the costmap means subscribing to
that service rather than opening the camera. `pointcloud_to_laserscan` is
installed ready for it. The gimbal camera is face tracking's, and face tracking
works; there is nothing to gain by moving it.

**`robot_localization`.** The EKF that would fuse the gyro, the encoders and the
accelerometer properly is installed and not configured. The base node already
publishes `/imu/data_raw` for it. It is the right next step once
`ticks_per_metre` is measured — fusing two sources when one of them has no scale
is fusing a guess.

**A colcon workspace.** The nodes are plain scripts and the launch files name them
by absolute path, so deployment is `scp` and there is no build step to forget.
`lidar_slam/` already has one of those, and a stale `libslam2d.so` is the rover
running last week's code with this week's file next to it on disk.

## Going back

The old stack is untouched and one crontab line away:

```bash
ssh bpi-m4zero 'sh ~/ugv/ros_nav/install-boot.sh --off'
ssh bpi-m4zero "crontab -l | sed 's/--board-bridge/--lidar/' | crontab - && sync"
ssh bpi-m4zero "pkill -f 'ugv/run_daemon[.]sh'; ~/ugv/restart.sh"
```

That is deliberate and should stay true until Nav2 has driven the rover around
the house enough times to have earned the trade. The daemon's own driving and
mapping tools — `drive_to`, `show_map`, `describe_surroundings` and the rest, which
the voice chat and the drive console call — come back with `--lidar` and are
**not** available while ROS owns the lidar. Giving them a Nav2 backend is the next
piece of work, and until it is done this rover maps well and is driven from ROS.
