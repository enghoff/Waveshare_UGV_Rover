# `ros_nav/` — ROS 2 mapping and navigation on the rover

A map that closes its loops, and a navigation stack that plans on it. This
replaced the rover's own SLAM and planner — which used to live in
[`lidar_slam/`](../lidar_slam) and have since been deleted — with `slam_toolbox`
and Nav2, running on the Banana Pi itself under ROS 2 Jazzy.

## Why this exists

The scan matcher this replaced was good and fast and had one deliberate hole:
**no loop closure**. The pose drifted monotonically and for ever, so driving a
circuit left the two ends of it in different places, and "go back to where you
started" was not something the rover could do. The reason it had no loop closure
was arithmetic — one closure attempt over `slam_toolbox`'s default search window
worked out at about nineteen seconds on the 700 MHz single-core Pi that code was
written for.

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
        |
        v
  nav_bridge.py                        loopback TCP 8773
        |                              goals in; the map, the pose and the
        |                              room out
        v
  rover_daemon --ros-nav               drive, drive_to, turn_in_place,
                                       stop_driving, describe_surroundings,
                                       show_map, map_png, nav_status, clear_map
```

The two loopback ports are the whole interface between the two halves of this
rover, and they run in opposite directions: **8772 is the daemon lending hardware
out, and 8773 is the daemon borrowing navigation back.** Neither is on the LAN,
because either one would put the wheels in front of something that authenticates
nothing.

The daemon keeps the board because it is also everything else the board does.
Only one process can hold a UART, so `board_bridge.py` inside the daemon lends out
the two things ROS cannot do without — the encoders and gyro at 50 Hz, and the
motor commands — over loopback. See
[`rover_daemon/board_bridge.py`](../rover_daemon/board_bridge.py).

The lidar is a separate USB device, so ROS simply takes it. The daemon has no way
to ask for it any more — its `--lidar` flag went with its planner — which is the
right shape, because two processes on one serial port is two half-conversations
and the daemon would win.

`lidar_node.py` does not re-implement the D500's packet format. It creates a
`Slam2D` from `lidar_slam/` and feeds bytes in — `feed()` in, `scan_xy()` out —
because that code already reads this sensor in 0.3 ms where Python takes 25. That
library is now only a parser: its scan matcher and occupancy grid were deleted
once nothing called them.

It does use one more thing from that library, and for the same reason: `describe()`,
which turns a revolution into walls, free-standing objects and the gaps between
them. That is what `describe_surroundings` answers with, and a language model can
say something useful about it where it would cheerfully hallucinate over a list of
360 ranges. It goes out on `/surroundings` as JSON in a `std_msgs/String`, twice a
second and only while something subscribes. What that library cannot supply is the
pose — there is no matcher in it — so the bridge adds it, along with the match
score and the scan count, rather than letting three plausible-looking zeroes reach
a console.

## The nav bridge, and why the daemon's tools work again

The daemon cannot import `rclpy`. It runs under the board's system Python because
that is what has the serial port, the camera and OpenCV; ROS 2 is a conda
environment with its own Python, and there is no making them one process. So
`nav_bridge.py` serves the stack on loopback 8773 and
[`rover_daemon/ros_navigator.py`](../rover_daemon/ros_navigator.py) is the client,
shaped like the `Navigator` the daemon used to drive with. Nothing in
`rover_nav.py`, in the tool schemas, or in either console had to change.

**Every move is a Nav2 action, not something written here.** That is the point of
the migration — the rover used to carry its own planner and its own follower and
both are now somebody else's problem:

| daemon tool | Nav2 action | what it gets from Nav2 |
|---|---|---|
| `drive` | `DriveOnHeading`, or `BackUp` backwards | drives straight, aborts on a collision the local costmap can see |
| `turn_in_place` | `Spin` | rotates on the spot, collision-checked through the sweep |
| `drive_to` | `NavigateToPose` | a planned route, a follower, and the recovery behaviours |
| `stop_driving` | goal cancel, plus a zero `Twist` | the cancel is the correct act, the Twist is the quick one |
| `clear_map` | `/slam_toolbox/reset` | throws the pose graph away |

`drive` is a narrower promise than it used to be, and the tool description says so:
it goes straight and stops, where the old one wove around obstacles. Weaving now
belongs to `drive_to`, which has a planner behind it.

Nav2's result codes are translated into the words the daemon's `Outcome` has always
used, in [`nav_codes.py`](nav_codes.py), and every code is listed there rather than
computed. They look systematic and are not: `BackUp` numbers invalid input 713 and
a collision 714, while `DriveOnHeading` numbers a collision 723 and invalid input
724 — the same two meanings, swapped, in adjacent blocks. A version of this that
did arithmetic on the last digit passed its tests and reported a rover stopped by a
wall as one that had timed out. The selftest checks the table against the
`.action` files themselves when it is run on the rover.

The map is drawn on the daemon's side. The bridge ships the occupancy grid as the
bytes it arrived as — zlib'd, which takes a room-sized map from 28 kB to under 2 —
and [`lidar_slam/mapimg.py`](../lidar_slam/mapimg.py) renders it, because that is
the renderer that already draws this rover's arrow, its track, the camera's cone
and the caption a model needs in order to read the picture. A second renderer on
the ROS side would have become a second, slowly diverging, picture of one room.

Two conversions happen on the way and both are invisible when wrong. ROS packs its
grid `[y][x]` and the renderer indexes `[forward][left]`; map `+x` is forward and
map `+y` is left on this rover, so the arrays agree once transposed. And
slam_toolbox's origin is wherever the map has grown to and is not a whole number of
cells from anywhere, so placing it rounds to the nearest — two and a half
centimetres, half the resolution of the thing being drawn. Checked two ways: a
self-test with a deliberately asymmetric mark, so a transpose cannot pass, and on
the rover against the lidar's own wall bearings, where every feature the map
contained preferred the placement as written and the nearest agreed to 2 cm.

**What is genuinely lost.** A pose graph has no per-revolution match score and a
velocity controller has no chosen steering arc, so `nav_status` reports None for
the first and the consoles show a dash. That is no loss: `position_trusted` now
means "slam_toolbox is still publishing where we are", which is the failure the old
number was really being watched for. `steering_deg` is filled in from the planned
route instead — the bearing to a point a lookahead ahead — which is a fair reading
of which way the rover is trying to go.

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
pair: the daemon must have `--board-bridge` and `--ros-nav`. If that is wrong it
prints the `sed` line that fixes it.

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

The ordinary way is the drive console at `http://<rover>:8771/` — tap the map — or
a conversation with the voice chat. Both reach the same tools, and both need the
daemon started with `--ros-nav`; without it the rover offers 11 tools instead of 17
and the console shows a map it cannot drive on.

Underneath, from the rover itself, either through the daemon — which is what the
consoles do, and which also hands face tracking over for the duration — or straight
at Nav2, which skips both:

```bash
python3 - <<'EOF'
import json, socket
s = socket.create_connection(("127.0.0.1", 8769))
s.sendall(json.dumps({"call": "drive_to",
                      "arguments": {"ahead_m": 2.0, "left_m": 0.0}}).encode() + b"\n")
print(s.makefile("r").readline())
EOF

. ~/ugv/ros_nav/env.sh
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}}}}"
```

The bridge itself can be poked directly, which is the quickest way to tell a broken
daemon from a broken stack:

```bash
python3 - <<'EOF'
import json, socket
s = socket.create_connection(("127.0.0.1", 8773))
s.sendall(b'{"op": "status"}\n')
print(json.dumps(json.loads(s.makefile("r").readline()), indent=1))
EOF
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
answers. The fallback constants in `lidar_slam/nav_types.py` say PWM 80 turns this
rover at 31.6 deg/s and PWM 180 at 170. They were true of the rover as it was, on a
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
the pose graph bending, which is the thing the rover's own SLAM could not do at
all.

Read the two columns differently, though. The heading is conclusive: 37 degrees of
accumulated error became under one. The position is only suggestive, because
nothing guarantees the rover physically returned to its start -- a leg cut short by
the lidar guard means some of that 0.49 m is real distance rather than error. If a
run reports the two errors as similar, check whether the legs completed before
concluding that closure did not fire.

Both of those runs predate the gyro-offset fix above, so they are the *hard* case:
dead reckoning was carrying 38 degrees of phantom rotation and the graph removed
it anyway. Re-run on a charged pack to see what it does now.

## Why the rover zig-zagged, and how to see it without a rover

It wandered the length of every route it drove, turning constantly instead of
holding a line. The cause was in the mixer, not in Nav2, and it is worth writing
down because it was invisible in every test that existed.

`MIN_TURN_DPS` lifts any rotation request under 12 deg/s to 12, and `pwm_for`
refuses to extrapolate below the slowest PWM anybody measured. Both are correct
rules, and both are rules about a wheel starting **from rest**: under about PWM 40
a stationary motor buzzes and does not turn, so asking for less than the slowest
thing ever measured is asking for nothing to happen.

Neither governs the *difference* between two wheels that are already turning. With
the from-rest floor applied to the steering term, this is what the mixer did with
a steering request at 0.35 m/s, on this chassis's own measured curves:

| asked | old: left, right | differential | fixed: left, right | differential |
|---|---|---|---|---|
| 0.5 °/s | −1, 177 | 89 | 83, 93 | 5 |
| 1 °/s | −1, 177 | 89 | 79, 97 | 9 |
| 2 °/s | −1, 177 | 89 | 69, 107 | 19 |
| 5 °/s | −1, 177 | 89 | 42, 134 | 46 |
| 10 °/s | −1, 177 | 89 | 2, 174 | 86 |

Five requests spanning a factor of twenty, one output: one wheel stopped and the
other at full. A path follower spends nearly all its time nearly on the path,
asking for a fraction of a degree a second — so nearly every command the rover
received became a violent pivot. It could not steer, only swerve, overshoot, and
swerve back.

[`steering_sim.py`](steering_sim.py) reproduces it with nothing plugged in, which
is how it was diagnosed while the rover was tethered to a charger. It closes the
loop — a pure-pursuit follower, this chassis's measured curves as the plant, the
real mixer in between — and counts how often the steering reverses:

```bash
python3 steering_sim.py --trace
```

|  | settled wander | steering reversals |
|---|---|---|
| old mixer | 1.1 cm | 4.0 per metre |
| fixed | 0.3 cm | 0.2 per metre |

The fix is that the floor is now conditional. Standing still, `steer_pwm` is
`turn_to_pwm` and both floors apply, because both wheels really are starting from
rest. Driving, the differential is interpolated straight to the origin — the curve
has to pass through it, since no difference between the wheels is no rotation.

A second fault showed at the other end while looking at this. When the pair ran
past the firmware's ceiling the old mixer scaled *both* wheels to fit, which reads
as fair and is not: it reduced the rotation as well as the speed, so a commanded
45 °/s arrived as 25. Speed is now given up first and rotation kept, because a
rover that advances too slowly still follows its route and one that turns too
slowly leaves it.

The mixer lives in [`drive_mixer.py`](drive_mixer.py), on its own and with no ROS
in it, so that the node, the selftest and the simulation all run the same code.
They used to carry copies. A copy of a table drifts visibly; a copy of a control
law drifts invisibly.

## The simulation that could not fail

That fix was real and it took the hard zig-zag out, and the rover still curved.
The reason it was not caught is worth more than the fix was.

`steering_sim.py` closed the loop around a simulated chassis whose rotation came
from the PWM difference *through the very curve the mixer used to choose that
difference*. Plant and controller were exact inverses, so the loop gain was 1.0
no matter what either of them believed. Started exactly on the line it drove a
perfect straight line for ever — and so did the broken mixer, the one that had
been seen zig-zagging the length of every route. Nothing could push either off
the line except the follower choosing to.

So it could detect exactly one class of fault: a mixer that cannot *express* a
small steering request. It could not detect a mixer that expresses the request
and then buys a completely different amount of rotation with it. A simulation
that cannot fail is worse than no simulation, because it is reported as evidence.
`selftest.py` now asserts that the simulated chassis and the mixer disagree.

## What steering actually costs, which is not what pivoting costs

`calibrate_chassis.py` measures rotation by spinning the wheels against each
other on the spot. That is a real measurement of a real manoeuvre and it is the
wrong one for steering, because a tracked chassis pivoting on the spot is
dragging its whole contact patch sideways. The pivot curve implies an effective
track width of 4.16 m at PWM 85 falling to 1.09 m at PWM 170, on a rover 0.22 m
wide — five to nineteen times the geometry, and all of it scrub. Rolling
forwards with one track a little faster than the other, almost none of that
scrub is present.

Measured with [`steer_gain.py`](steer_gain.py), which asks through `/cmd_vel`
exactly as Nav2 does and reads the gyro:

| asked | PWM pair | rover really turned |
|---|---|---|
| 2 °/s | 69, 107 | 5.8 °/s |
| 5 °/s | 134, 42 | 20.4 °/s |
| 10 °/s | 2, 174 | 85.6 °/s |
| 45 °/s | −68, 180 | 111.7 °/s |

Every steering request was over-served by between two and nine times. A follower
can only answer that by correcting back, which is a weave rather than a route.
So the store carries a second curve, `steer_pwm_points`, measured while rolling,
and the mixer inverts *that* one whenever the rover is going anywhere. The pivot
curve still governs turning on the spot, which is a different manoeuvre.

## The rover pulls to one side, and it is the rover and not the gyro

Asked for no rotation at all it curved left at about 1.1 °/s at 0.35 m/s. That
had to be attributed before it could be corrected: a rover curves either because
it really curved, or because its gyro said it did, and steering only fixes one
of those. `odom → base_link` is the gyro integrated and nothing else, while
`map → base_link` has slam_toolbox's scan matching on top, so the two are
independent witnesses. Over six runs they agreed to 0.05 °/s. It is the chassis.

`straight_bias_deg_per_m` in the store is that pull, held as degrees per metre
driven rather than per second, because a small mismatch between two tracks is a
constant *curvature* — go twice as fast and you turn twice as fast through the
same arc. It was measured at one speed only, so that scaling is reasoned rather
than observed. The mixer subtracts it from every request while driving, and not
at all while pivoting, which is why turning left 2 °/s now takes 4 PWM of
differential and turning right 2 °/s takes 16.

Measured on the rover after the change, over six 1.4 m runs: the pull fell from
+1.14 °/s to +0.07, the steering channel from 3.7× over-response to 1.2×, and a
2 m `drive_to` finished 1.97 m along its original heading with 7 cm of lateral
drift.

To re-measure either, on a rover with a couple of metres of clear floor:

```bash
python3 steer_gain.py --differentials 8,8,12,12,18,18,25,25,35,35,50,50 --save
python3 steer_gain.py --straight 1.4 --repeat 6 --save
~/ugv/ros_nav/restart.sh
```

Both return the rover to roughly where it started after every sample, and both
refuse to move with anything inside a corridor its own width ahead.

## The footprint is the thing that decides whether `drive_to` works

Measured on the rover, and worth knowing before concluding that Nav2 is broken.

`robot_radius: 0.25` does two different jobs and only one of them is obvious. It
is the collision model, and it is also the *inscribed* radius the inflation layer
uses — so every occupied cell in the map projects a 25 cm disc in which the
rover's centre may not be. A goal inside one is refused outright, and so is a
start. `inflation_radius: 0.45` is not part of this: beyond the inscribed radius,
inflation only adds preference, never prohibition.

What that costs, measured against a real map of this house at 5 cm:

| `robot_radius` | free floor still legal |
|---|---|
| 0.10 m | 89% |
| 0.16 m | 82% |
| 0.20 m | 76% |
| 0.25 m | 68% |
| 0.30 m | 63% |

The compounding problem is a sparse map. An isolated speckle cell projects the
same 50 cm disc a wall does, so a map with a few minutes' data in it is mostly
no-go: walked along a line the lidar reported as 2.1 m of clear floor, the costmap
read `inscribed` from 20 cm onwards while slam_toolbox's own map said `free`. That
is not the costmap being wrong — it is 25 cm of inflation around scattered cells
that a properly built map would have resolved into walls.

So two things make `drive_to` fail in a way that looks like a broken planner and
is not:

- **A map with only a few metres of driving in it.** Build one by driving the
  house before judging the navigation. The behaviours — `drive` and
  `turn_in_place` — need no plan and work regardless, which is how to get out of
  a spot the planner will not leave.
- **A footprint radius nobody has measured.** 0.25 m was chosen as a safe guess
  because the offset from the chassis centre to the lidar was never measured, and
  `base_link` is at the lidar. The library already knows more than that: the
  lidar's own returns off the rover's body span 8.5–11.2 cm behind it and
  8.2–10.7 cm to each side, over 397 revolutions (see `body_back_m` and
  `body_half_width_m` in [`lidar_slam/slam2d.c`](../lidar_slam/slam2d.c)). Only
  the forward extent is unmeasured, because the lidar sees past the body that way.
  Measuring it, and replacing the circle with the rectangle this chassis actually
  is, would take the prohibited ring from 25 cm to about 13 cm and give back
  roughly a seventh of the floor while modelling the corners *better* than a
  circle does.

A failed move now says how hard Nav2 tried: `blocked -- Nav2 gave up after 10
recovery attempts`. A bare "blocked" sends somebody to look at the rover, which is
the wrong place.

## The rover took absurd routes, and there were five reasons, not one

The rover started answering a two-metre goal with a five-metre route: out to one
side, past the goal, back again, 519 degrees of accumulated turning to cover two
metres of clear floor. It reads as a controller that cannot follow a line, and it
was not — the *plan* was that shape before the wheels turned. Asking
`ComputePathToPose` for a route while the rover stood still reproduces it in a
tenth of a second and moves nothing, which is the way to tell the two apart:

```bash
ssh bpi-m4zero 'bash -c "source ~/ugv/ros_nav/env.sh; python3 - <<EOF
... ComputePathToPose to a point 2 m ahead, and measure length against 2 m
EOF"'
```

**The cause was the global costmap's obstacle layer, and on a rover that is
mapping as it goes there is no setting that makes that layer safe.** It remembers
marked cells in the `map` frame and nothing ever re-registers them. Every time
slam_toolbox closes a loop it moves that frame — which is the whole reason for
running it — and the static layer moves with it because the static layer *is* the
map, while every cell the obstacle layer had marked stays where it was. Each
correction leaves a ghost copy of every wall a few centimetres off the real one,
and the ghosts only accumulate. Counted on this rover after 2h40m of driving:

| | cells |
|---|---|
| obstacles in slam_toolbox's map | 1119 |
| lethal cells in the global costmap | 2649 |
| …with no map obstacle within 10 cm | 1201 |
| …of those, in cells the map calls open floor | 523 |

A second mechanism kept them there. `lidar_node` publishes a bearing that got
nothing back as `inf`, which is what a LaserScan is supposed to say, and about a
quarter of every revolution is one — 360 bins against roughly 450 returns that
are not evenly spread. The projection that feeds the obstacle layer *drops* those
bins unless `inf_is_valid` is set, so a quarter of the directions around the rover
were never raytraced and nothing marked in them could ever be cleared.

The reason a few hundred stray cells wreck a route rather than merely
inconveniencing it is the arithmetic in the section above, plus how NavFn reads a
costmap. Anything at or above 253 — every cell within the rover's own radius of a
lethal one — is a *hard obstacle* to the planner, and 253 is also exactly what
unexplored space is worth. So the ghosts did not make the floor expensive, they
made it impassable, and unexplored space was no worse: 61% of the mapped free
floor was closed, and the cheapest way from here to there ran out through the
unknown. Replaying the captured grids through the same cost transform:

| costmap | free floor blocked | 2 m goal, straight and clear |
|---|---|---|
| as it was | 61% | 4.46 m, tortuosity 2.07 |
| static layer only | 35% | 2.33 m, tortuosity 1.17 |

So the global costmap has no obstacle layer any more, and the local one is told
`inf_is_valid: true`. Nothing was lost that this rover had: the static layer is
slam_toolbox's live map, republished every two seconds and re-registered by every
loop closure, so anything seen while driving is in the global picture within a
couple of seconds and *in the right place*. What the global planner no longer
knows is an obstacle that appeared since the last map update — and that one
belongs to the local costmap, which is rebuilt from the live scan and is what DWB
actually drives against. `selftest.py` checks both settings, because the failure
they cause looks like a controller fault and is not.

### The chassis has two forward speeds, and Nav2 was told it had a continuum

`calibrate_chassis.py` tried PWM 70 and PWM 80 and neither moved the rover at all.
The slowest speed it can hold is 0.33 m/s at PWM 85. **There is no creep**, and
almost nothing in a stock Nav2 configuration is true of a robot like that.

Recorded off `/cmd_vel` and `/odom` over one ordinary 1.8 m drive, before any of
this changed: **16 of the 37 velocity commands Nav2 issued were below the floor**,
the slowest of them 0.05 m/s against a rover already doing 0.34. Two consequences,
and both of them look like a controller that cannot steer:

- **Every acceleration and deceleration was a fiction.** DWB ramped up over 0.9 s
  and down over 0.7 s; the rover stepped to 0.33 m/s on the first command and held
  it until the command reached exactly zero, so it overshot the end of every leg
  and had to come back for it.
- **So were the tight curves.** DWB believed it could pair 0.05 m/s with
  0.78 rad/s, which is a 6 cm turning radius. The tightest arc this chassis can
  drive is 0.33 over 0.78 — **0.43 m of radius, about 0.9 m across**, and close to
  the 0.7 m measured by driving it. Asked for the tight one it drove the wide one,
  ran wide of the path, was corrected, and ran wide the other way.

The fix is to give DWB the velocity space the chassis has and no more: `vx_samples: 3`
over `-0.40 .. 0.40`, which its iterator turns into exactly {back, stop, forward},
and acceleration limits that describe a step rather than a ramp. The *sample*
window is the acceleration limit times the tick, not `min_vel_x`..`max_vel_x`, so
at the 0.5 m/s² that was there DWB could not ask for more than 0.05 m/s on its
first tick however high the maximum was.

`steering_sim.py` and `steer_gain.py` are about the other axis and are unaffected:
the differential between two wheels that are already turning has no stiction floor,
and the measured steering curve tracks a request to within a few per cent.

### The footprint was a guess, and the body had already been measured

`robot_radius: 0.25` was a circle chosen to be safe, because `base_link` is at the
lidar rather than at the middle of the chassis and the offset had never been
measured. The radius is also the *inscribed* radius, and the inscribed radius is
where the inflation layer writes 253, and 253 is a hard obstacle to NavFn rather
than an expensive cell. So every mapped obstacle projected a 25 cm disc of no-go:

| | share of the mapped free floor |
|---|---|
| blocked at `robot_radius: 0.25` | 30% |
| reachable from where the rover stood | 62% |
| blocked with the measured footprint | 15% |
| reachable with it | 81% |

It was worse than a statistic. The rover's *own* cell read 253, so the planner
could not leave the start and answered a 2 m goal with "there is no route to there
that the rover fits through" while the rover sat in the middle of a clear room.

The measurement was already in the repository. `lidar_slam/slam2d.c` drops the
returns that land on the rover's own body, and its bounds came from 397
revolutions of this lidar looking at this chassis: 0.16 m behind the sensor and
0.14 m to each side. Only forward is unmeasurable, because the lidar sees past the
body that way, so 0.20 m is a deliberate over-estimate. The inscribed radius of
that rectangle is its half-width, 0.14 m.

**The critic has to change with it.** `BaseObstacle` scores and vetoes on the one
cell the robot's centre is in, which is a collision test only while the inscribed
ring is as big as the whole robot. It no longer is — the rear corners are 0.21 m
out — so `ObstacleFootprint`, which traces the four edges, replaces it.

### What the controller was really weighing, and how to see it

`/evaluation` is DWB publishing its own decision: every candidate twist, the score
each critic gave it, and which one won. It is the only place that says why a rover
that could move is standing still, and two of the four faults here were found in
it and nowhere else.

The first was `BaseObstacle`, and the shape of it is worth remembering because it
is structural. That critic scores the costmap cost where a rollout *ends*, and a
rover that stays where it is ends where it already was, in open floor, at zero.
Anything that moves ends further into a room, which in a furnished one is inside
the inflation gradient at a cost of 190 to 240. **It is a standing bribe not to
move**, worth about 4 points at the stock scale — and standing still was winning
13.20 to 13.54. Stock DWB gets away with it because it has twenty forward samples
and the slow ones barely enter the gradient. This chassis has no slow ones.

The second was the rollout length. `sim_time: 1.5` at this speed is 0.6 m of arc,
and the two heaviest critics measure how far a rollout strays from the planned
path. NavFn plans on a 5 cm grid with no curvature limit in it, so its corners are
tighter than any arc this chassis can drive, and over 0.6 m the best forward option
was 10 points off the path where standing still was 1.6. The rover chose to stay
put on **282 of 342 ticks** with clear floor in front of it. A shorter rollout does
not make the corner followable, it just stops asking the rover to commit to it.

### A rotation shim was tried, and taken out again

A chassis with a half-metre turning radius ought to turn on the spot at a corner
rather than swing wide, and `RotationShimController` is Nav2's way of saying so.
It does not work here. It transforms a point off the path into the base frame with
a fixed 10 ms transform tolerance it does not expose as a parameter, and this
rover's transform tree is driven by the driver board at about 17 Hz, so the newest
`odom -> base_link` is 30 ms old on average and the lookup fails. It then throws,
logs at ERROR and delegates — ten times a second. With it in the chain the control
loop ran at 6-7 Hz against the 10 it is asked for, `compute_path_to_pose` began
timing out on acknowledgement, and moves were aborted mid-drive.

Nothing was lost by removing it, because {back, stop, forward} already contains the
pivot: DWB chooses it about a third of the time and forward the rest.

### A fifth reason: the goal was inside a wall, and nothing said so

The one after the four above, found by reading the log of a run that ended the way
the others had. The rover was sent to (4.34, -0.98), drove 1.55 m in about five
seconds, stopped 23 cm short and then stood making small heading corrections until
the allowance ran out twenty-five seconds later. The console said `timed out, after
1 replan`, which reads as a rover that could not find its way.

It had found its way. It could not *park*. That cell's cost was 216 — inside the
inflation gradient, five centimetres from the inscribed ring of a real wall — and
laying the measured body over it at each of twenty-four headings, the footprint
overlapped the inscribed ring at every one of them and covered an outright lethal
cell at the heading the bridge had asked for.

**The two halves of Nav2 disagree about what the rover is, and nothing reconciles
them.** NavFn searches the cost grid as though the rover were a point, so 216 is
drivable and it returned a clean straight path, tortuosity 1.01, and went on
returning it after every replan. DWB checks the real rectangle and would not end a
rollout there. Neither logged anything: as far as each was concerned it was doing
its job.

Two changes, and they are both about the size of the smallest move this chassis
has:

- **The arrival circle went from 15 cm to 22 cm.** One forward sample at 0.40 m/s
  over a 0.8 s rollout is 32 cm, so the shortest move DWB can even weigh is twice
  the old tolerance. A 15 cm circle was a target the controller could not aim at —
  every rollout that closed the gap overshot it and scored worse than standing
  still, which is precisely what it did.
- **The bridge tests a goal against the body before sending it**, and moves it to
  the nearest pose the rover fits in, or refuses when there is none within half a
  metre. That is `goal_fit.py`, which has no ROS in it so the selftest runs the
  same geometry the rover does. Replayed against the recorded costmap, the goal
  that hung is rejected — worst cost under the body 254 — and snaps 5 cm to
  (4.34, -0.93), which is 18 cm from where the rover actually gave up and so
  inside the new tolerance. That run would have arrived.

On the rover: a goal aimed deliberately into a wall came back `arrived`, having
travelled 0.0 m, saying *the spot asked for is too close to something for the rover
to stand in, so the goal was moved 30 cm to the nearest one it fits*.

### The rover will not reverse further than it can see

The lidar is the only thing aboard that sees where the rover is going, and it is
bolted on facing forwards. Everything behind is unmapped and unwatched, and the
collision check that guards a reverse reads the same costmap — so what it is
checking against is whatever was there the last time the rover faced that way.

So DWB has no reverse sample at all now (`min_vel_x: 0.0`, and two samples rather
than three), and an explicit `drive` backwards further than `REVERSE_LIMIT_M` turns
the rover round and drives it forwards instead, which covers the same ground with
the sensor pointed at it. Half a metre is a little over one body length: enough to
back off something the rover has nosed into, short enough that it was looking at
that ground moments ago.

Backing out of a corner still works, because that is the behaviour server's
`backup` recovery and the behaviour tree bounds it to 30 cm. The velocity smoother
therefore keeps its reverse limit even though the controller no longer uses one —
everything the rover drives passes through that node, and a floor of zero there
would silently swallow the recovery.

### Where it got to, and what is still wrong

Goals that timed out at 30 to 39 seconds now arrive in 5 to 11. In open floor DWB
chooses to move on 61 of 62 ticks where it chose to stand still on 282 of 342.

**It is not finished.** Starting close to a wall, NavFn still returns routes about
1.9 times the direct distance, and against one of those DWB goes back to sitting
and rotating — it loses by under one point in forty-five, entirely on the two
critics that measure distance from the path. That is the planner's fault rather
than the controller's: a grid Dijkstra has no curvature in it, so its route is not
one this chassis can follow, and asking a velocity controller to follow it anyway
is asking it to choose between the path and the goal. The lever not yet pulled is a
planner that knows the turning radius, or the configured `smoother_server`, which
is set up and which the stock behaviour tree never calls.

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
`lidar_slam/` still has one, for `libslam2d.so`, and a stale one is the rover
running last week's parser with this week's file next to it on disk.

**Nav2 behind the daemon's `drive`.** `DriveOnHeading` goes straight and stops; it
does not weave. That is a real reduction from what the daemon's own follower did,
and it was chosen rather than papered over: the honest place to want obstacle
avoidance is a call that has a planner behind it, which is `drive_to`. A `drive`
that quietly did its own steering underneath Nav2 would be two controllers with
different ideas of the room.

**A speed for `turn_in_place`.** Nav2's `Spin` takes an angle and a time
allowance, not a rate, so the old tool's "it turns more slowly when something is
within about 25 cm" is now the behaviour server's collision check rather than a
rate this rover chooses. The turn is still refused only if the rotation itself
would sweep through something, which is what mattered: rotating is how a rover
that has got too close to a wall gets away from it.

## There is no going back

There used to be, and this section used to say how: the daemon's own planner was
left in place behind a `--lidar` flag, two crontab edits away, until Nav2 had
driven the rover around the house enough times to have earned the trade. It has,
so that planner, its scan matcher, its route finder and its drive controller have
been deleted — about 5,400 lines — along with the flag that reached them.

What that buys is one arrangement to reason about instead of two that offered the
same 17 tools and the same replies and could not be told apart from a console. The
daemon's startup line no longer has to be checked to know which planner answered a
move; if it says anything but `driving ros2 on 127.0.0.1:8773` then nothing is
driving at all.

Getting the old stack back means `git revert`, not a crontab edit. The commit that
removed it is one commit and it took `lidar_slam/`'s README with it, so the
reasoning is recoverable along with the code.

## Reloading it after a deploy

```bash
ssh bpi-m4zero '~/ugv/ros_nav/restart.sh'                # new nodes or config
ssh bpi-m4zero '~/ugv/ros_nav/restart.sh --supervisor'    # new run_ros_nav.sh
```

**The second form is not optional when `run_ros_nav.sh` itself has changed**, and
the reason is worth knowing because the failure is silent and convincing. bash
parses a function once, when the script starts, and that supervisor runs for weeks.
The sweep that takes the previous launch's nodes down therefore used to be a stale
copy — so adding `nav_bridge` to it changed nothing, the old bridge survived the
reload, the new one could not bind port 8773 and died in the log, and the stack came
back answering every question with the *previous* deploy's code while the reload
reported one of each node running and nothing wrong.

Two things came out of that. The sweep is now [`sweep.sh`](sweep.sh), a separate
file read from disk every time it is called, so anything that adds a node to this
stack adds it there. And `restart.sh` no longer trusts a process count: it also
looks for a death in the log after the launch started, which is what a node losing
a port looks like from outside.
