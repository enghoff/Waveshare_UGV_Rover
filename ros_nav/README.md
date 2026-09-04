# `ros_nav/` — ROS 2 mapping and navigation on the rover

A map that closes its loops, and a navigation stack that plans on it. This
replaced the rover's own SLAM and planner — which used to live in
[`lidar_slam/`](../lidar_slam) and have since been deleted — with `slam_toolbox`
and Nav2, running on the rover's own computer under ROS 2 Jazzy.

## Why this exists

The scan matcher this replaced was good and fast and had one deliberate hole:
**no loop closure**. The pose drifted monotonically and for ever, so driving a
circuit left the two ends of it in different places, and "go back to where you
started" was not something the rover could do. The reason it had no loop closure
was arithmetic — one closure attempt over `slam_toolbox`'s default search window
worked out at about nineteen seconds on the 700 MHz single-core Pi that code was
written for.

The rover is not that board any more. It is a six-core Cortex-A78AE — and was a
quad-core Cortex-A53 when this was written — and `slam_toolbox` does that search
multi-resolution, threaded, in optimised C++. So
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
        v
  Nav2          --> /cmd_vel  (back to base_node, and out to the wheels)
        |
        v
  nav_bridge.py                        loopback TCP 8773
        |                              goals in; the map, the pose and the
        |                              room out
        v
  rover_daemon --ros-nav               drive, drive_to, turn_in_place,
                                       explore, stop_driving,
                                       describe_surroundings,
                                       show_map, drive_to_map_point,
                                       map_png, nav_status, clear_map
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
means "the mapper is still publishing where we are", which is the failure the old
number was really being watched for. `steering_deg` is filled in from the planned
route instead — the bearing to a point a lookahead ahead — which is a fair reading
of which way the rover is trying to go.

## Installing it

Three steps, and the first is the only slow one.

```bash
scp -r ros_nav orin:~/ugv/
ssh orin 'sh ~/ugv/ros_nav/install.sh'          # ~20 min, 6.9 GB
ssh orin 'sh ~/ugv/ros_nav/install-boot.sh --nav'  # mapping and Nav2
```

Note the `sh` in front of both. A checkout that arrived by `scp` is mode 644, so
the shebang is never consulted and running the path directly fails with
"Permission denied" — which reads as a sudo problem and is not one.

`install.sh` needs no root at all, which is the whole reason it is conda and not
apt. That started as a necessity: the Banana Pi ran Debian trixie, ROS 2 Jazzy's
own packages are built for Ubuntu noble, so there was nothing to `apt install` and
building from source on four A53 cores was most of a day.
[RoboStack](https://robostack.github.io/) publishes the same releases as conda
packages with `linux-aarch64` builds.

On the Jetson it is a choice rather than a necessity. This board runs Ubuntu
24.04 and native Jazzy packages do exist; they are deliberately not used, because
one install path that works on both boards is worth more than a second that works
only here. The one time that rule was broken — RTAB-Map, which RoboStack does not
package, pulled in from Ubuntu's ROS repository — is described under
[why RTAB-Map was tried and removed](#why-rtab-map-was-tried-and-removed);
`/opt/ros` no longer exists on the rover.

`install-boot.sh` also checks the daemon's crontab entry, because the two are a
pair: the daemon must have `--board-bridge` and `--ros-nav`. If that is wrong it
prints the `sed` line that fixes it.

## Running it

```bash
ssh orin '~/ugv/ros_nav/restart.sh'        # ~30 s; prints the node list
ssh orin 'tail -f ~/ugv/ros_nav/ros_nav.log'
```

An empty map with the lidar reporting at 10 Hz is usually the driver board,
not the scan. Check `board_ok` in `nav_status` before debugging slam_toolbox;
see [rover_daemon/README.md](../rover_daemon/README.md#when-the-board-does-not-answer).

And by hand, which is what to do when something is wrong:

```bash
ssh orin
. ~/ugv/ros_nav/env.sh                # bash only -- see below
. ~/ugv/ros_nav/dds.sh                # pin discovery to this board
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

**`dds.sh` after `env.sh`, never instead of it.** RoboStack's activate hook sets
discovery to the whole subnet, which is how a laptop rviz works and how a dead
radio's leftover address takes this graph down. The launchers source both. A
laptop that wants rviz on the LAN should source only `env.sh`.

## Saving a map

```bash
ssh orin
. ~/ugv/ros_nav/env.sh
ros2 run nav2_map_server map_saver_cli -f ~/ugv/maps/house
```

which writes `house.pgm` and `house.yaml`. `slam_toolbox` can also serialise its
whole pose graph, which is the thing to keep if the map is ever to be *continued*
rather than just used:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: /home/jetson/ugv/maps/house}"
```

## Clearing it, and where the rover ends up when you do

`clear_map` resets the pose graph, and it is worth knowing what that does to the
rover's coordinates, because two things about it are surprising and both have
cost an afternoon.

**The new map does not start where the rover is standing.** A fresh graph is
anchored on raw odometry, so the map frame becomes the odom frame — whose origin
is where `base_node` started, which is wherever the rover happened to be when the
ROS stack was last launched. Over the 46 clears in the rover's own log the rover
stood between 0 m and 23.7 m from the origin of the map it had just made: 0 m
right after a restart, and metres out for every hour of driving since. That is
why the drawn grid has to follow the map rather than being a fixed square around
the origin — see `_square_holding` in `rover_daemon/ros_navigator.py`, and the
straight edge it used to cut across the room 20 m out.

**And the rover's coordinates jump at the moment it takes effect.** `map -> odom`
is the correction the graph had built up, and clearing discards all of it in one
step: measured on 2026-09-04, the rover's position moved 5.37 m without a wheel
turning, and a few hours later the correction stood at 11.4 m and 35 degrees. The
jump does not land when the reset returns, either. slam_toolbox only folds a scan
into the graph once the rover has travelled `minimum_travel_distance`, so a parked
rover re-anchors nothing — `map -> odom` was bit-for-bit identical over 35 seconds
of standing still — and until the wheels turn, every pose read out of the
transform tree is still in the frame that was thrown away.

Two things allow for that. [`trail.py`](trail.py) holds the drawn track back until
the mapper has published a correction belonging to the new graph, so the track no
longer starts with a 5 m line out of the room; and the renderer breaks the track
at any step longer than `TRACK_BREAK_M`, because a loop closure moves the frame
too and nothing can hold that back.

## Sending it somewhere

The ordinary way is the drive console at `https://<rover>:8771/` — tap the map — or
a conversation with the voice chat. Both reach the same tools, and both need the
daemon started with `--ros-nav`; without it the rover offers 11 tools instead of 18
and the console shows a map it cannot drive on.

Underneath, from the rover itself, either through the daemon — which is what the
consoles do — or straight at Nav2, which skips both:

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

## Letting it map the place on its own

`explore` sends the rover to the edge of what it has mapped, over and over, until
there are no edges left it can reach. From a console it is the **explore** toggle
beside the STOP button, which turns blue while a run is going and stops it when
pressed again; from the daemon it is a tool with an optional `minutes`, which
answers at once and leaves the rover going; from the bridge it is one more move:

```bash
python3 - <<'EOF'
import json, socket
s = socket.create_connection(("127.0.0.1", 8773))
s.sendall(b'{"op": "explore", "budget_s": 120}\n')
for line in s.makefile("r"):
    print(line.strip())
EOF
```

It is a move and not a mode, which is the decision the rest of this follows from.
It takes the same mutex `drive` and `drive_to` take, so a tap on the map while it
is running is refused as busy rather than fighting it for the same action server;
STOP ends it as it ends anything else; and it narrates itself the whole way, so a
console watching one shows what it is doing rather than a stopwatch.

**On the bridge it blocks; above the bridge it does not.** The op holds its
connection for the whole run, because that connection is how it narrates. The
daemon's tool, one layer up, hands that to a thread and answers immediately —
every client of the daemon shares one socket behind one lock, so a tool call that
waited would have blocked `stop_driving` along with everything else. See
[`rover_daemon/README.md`](../rover_daemon/README.md). What asks "is it
exploring" is therefore the rover, not a caller's own call in flight, and
`status` grew an `exploring` flag beside `driving` to answer it.

**None of the driving is new.** Choosing where to go is `frontier.py`, and every
metre of getting there is `NavigateToPose` through `goto` — the same goal check,
the same route-based time allowance, the same recovery ladder, the same escape
behaviours. That was the point of doing it this way rather than the obvious way.

### Why not `explore_lite`

It is the obvious first answer, `docs/jetson-orin-navigation.md` proposed it, and
it is not what is here. Two reasons, and the first is merely awkward while the
second is the real one.

RoboStack has no `ros-jazzy-explore-lite`, so having it means building
`m-explore-ros2` from source into the conda environment — which is possible, since
`behaviors/` already colcon-builds against it, but it is a build step and this
directory has spent some effort not having those.

The second reason is that `explore_lite` publishes straight to
`/navigate_to_pose`. Everything this rover has learned about sending itself a goal
lives between the goal and that action server, and a node that goes around it goes
around all of it: the footprint check that stops a goal being set inside a wall
(`goal_fit.py`, and the thirty seconds of shuffling that paid for it), the move
mutex that keeps two callers from steering at once, the time allowance built from
the route rather than the straight line, and the narration both consoles read. A
second thing publishing goals is also a second thing the STOP button does not know
about. What is here is about three hundred lines and it inherits all of that by
construction, because it asks `goto` rather than Nav2.

### What it costs to decide where to go

`frontier.py` finds the cells the mapper calls free that have unknown ground next
to them, clumps them into frontiers, and ranks them. Two decisions in it are worth
knowing:

**Reachability is decided by walking, not by measuring.** A breadth-first walk out
from the rover over cells the mapper has seen to be free, four-connected. A house
map is full of frontiers two metres away through a wall and eleven metres away
round the corridor, and straight-line ranking cannot tell them apart. The walk
costs one sweep — 18 ms on the 248 x 327 grid the kitchen-loop drive produced —
and drops the unreachable ones instead of ranking them.

**And then the planner is asked anyway, before the rover moves.** The walk goes
through a 5 cm gap between a table leg and a wall; Nav2 plans with the inflated
body and will not. Without the check that frontier is the best-ranked candidate
every round until the goal fails, and each failure costs the full recovery ladder.
`ComputePathToPose` answers the same question the goal would have started by
asking, for about a second, and a frontier it will not route to is written off
having driven nothing.

### The rule that makes it stop

**Every frontier is written off once it has been driven to, whether or not the
rover got there.** For a failure that is obvious; for an arrival it is what makes
the loop terminate. The controller stops within its 22 cm tolerance, and if the
lidar did not happen to see round the corner from there, the frontier is still on
the map and still the nearest one — so a loop that wrote off only failures would
drive the same 30 cm until its budget ran out. What it costs is the occasional
pocket left behind a corner the rover stood next to, and a second `explore` picks
that up, because the blacklist lives exactly as long as one call.

### It said the house was fully explored with 73% of the map unknown

**The rover parked 15.6 cm from a wall and then declared the world finished.**
Recorded on 2026-09-01: one frontier tried, none reached, 0.1 m driven, and an
outcome reading `everything still unmapped is behind something the rover cannot
get through -- 10 more still on the map -- 73% of the map is still unknown`. The
map on the console showed a green reachable floor with obvious unexplored space
all round it, so the sentence was not a near miss; it was wrong on its face.

The cause is one word in the planner's refusal that nothing was reading. The
rover's own log:

```text
planner_server: GridBased plugin failed to plan from (-2.30, 1.46) to
                (0.91, -3.23): "Start occupied"
```

— five times, five different destinations, one pose. The global costmap is the
SLAM map plus a 0.45 m inflation, and the footprint is a 0.20 m circle, so any
cell within 0.20 m of a mapped wall is a cell the planner will not plan *from*.
Measured against the live map, the pose in every one of those lines is **0.156 m
from the nearest occupied cell** — the planner was right — and **30% of the known
floor of this house is inside that band**. This is not a freak position; it is
where a rover ends up whenever it drives to the edge of the map, which is the one
thing exploring does.

So all four candidates came back refused, and the loop read four refusals about
the rover as four verdicts on four frontiers. Replayed at the bench against the
costmap taken off the running planner — `fixtures/start-occupied.json.gz`, which
`plan_bench.py --map` reads directly — with nothing moving:

```text
start (-2.30, 1.46) cost 253 -> goal (1.59, 1.34):  0 of 1 start headings planned
start (-2.40, 1.21) cost 135 -> goal (1.59, 1.34):  1 of 1, 4.60 m, 0.01 s
```

Twenty-seven centimetres. Three of the four frontiers it had just called
unreachable plan from there in a hundredth of a second.

Three things changed. **The planner is asked why, not just whether** —
`ComputePathToPose` fills in an `error_code` even on the goals it aborts, and
`ABOUT_THE_ROVER` in `nav_codes.py` is the two codes that are about the start.
**A refusal about the start moves the rover instead of blaming the map**:
`back_off` asks `goal_fit.fit` for the nearest place the body fits — the same
check on the same costmap the planner just refused with — then turns towards it
and drives forwards, because the lidar looks one way and the ground being driven
onto is worth having a sensor pointed at. Three of those per run, then it says it
is stuck. **And a round that finds nothing writes those candidates off and looks
again**, so the run ends when the map has nothing left on it rather than when a
sample of four was refused. When frontiers really are unreachable the outcome now
says so — `the rover could not get a route to any of the 6 places left on the
map` — which is a different sentence from the house being mapped, and reads
correctly beside the "still unknown" figure that follows it.

### It gives up on a goal that is going nowhere, and only exploring does

**The first unattended run met the repository's oldest open fault, and it cost
fifty seconds.** Recorded on 2026-09-01: the sixth frontier of a run, a goal
2.4 m away across open floor with nothing solid within 40 cm of it, and the
rover moved **six centimetres in fifty seconds**. The log is forty-three
identical lines — `controller_server: Passing new path to controller.`, once a
second — and nothing else. No progress-checker failure, no costmap clear, no
spin. Nav2 never noticed anything was wrong.

That is "The controller is aimed round the corner" below, which is still open:
`MapGridCritic`'s flood runs through walls in the build installed here, so where
the route bends the critics steer at a point behind the wall, every forward
sample is refused, and pivoting is free. What hides it is the progress checker.
This rover runs `PoseProgressChecker` on purpose, so that a legitimate pivot is
not called stuck — and the price is that a rover pivoting for ever is not called
stuck either.

Exploring cannot fix that. What it can do is stop paying for it, because it has
something `drive_to` does not: **sixteen other frontiers and no opinion about
which.** So an exploring goal is given up when the rover has not got half a metre
further on in twenty-five seconds *and Nav2 has attempted no recovery* — the
second half being what separates the aiming trap from a rover that is genuinely
stuck against something, where the recovery ladder is better at this than any
watcher. A commanded `drive_to` is untouched and still gets every recovery Nav2
has, which is what somebody who asked for one particular place is owed.

The numbers come off the recordings rather than out of the air. `frontier.Stall`
is replayed over all four in the selftest, across the window where a goal was
actually being driven:

| recording | commanded | forward commands | net moved | watcher |
|---|---|---|---|---|
| `trap-2026-08-25-spin` | 51 s | 5 of 511 | 0.25 m | gives up at 25 s |
| `corridor-2026-08-25-spin` | 43 s | 20 of 430 | 0.30 m | gives up at 25 s |
| `doorway-2026-08-25` | 56 s | 53 of 315 | 1.76 m | gives up at 25 s |
| `doorway-2026-08-25-after-floor` | 9.6 s | 85 of 99 | 3.52 m | **leaves it alone** |

The first three never reached their goals; the fourth is the rover driving
properly and is the false positive that would matter, because a watcher that
cancels that is a rover that cannot cross a room.

**One trap in reading those recordings is worth passing on.** `nav_record.py`
records a fixed sixty seconds whether or not anything is happening, so every one
of these files ends with the rover sitting idle after the drive finished. Replay
a watcher across the whole file and all four look like stalls — which is how
this test first read, and it is not a fact about the rover. The watcher only ever
runs while a goal is in flight, so the replay is bounded to the commanded window.

### 0.50 m of boundary, and how that number was arrived at

The one setting worth arguing about is how much unmapped edge makes a frontier
worth driving to. `explore_sim.py` runs the shipped policy round the floor plan
the recorded `kitchen-loop` drive produced:

| smallest frontier | goals | driven | floor found |
|---|---|---|---|
| 0.30 m | 23 | 99.8 m | 99.9% |
| 0.50 m | 16 | 82.4 m | 99.6% |
| 0.75 m | 10 | 65.8 m | 98.3% |
| 1.00 m | 6 | 54.8 m | 94.7% |

Four goals get 93% of that house. At 0.30 m, nineteen of the twenty-three goals
and seventy of the hundred metres buy the last 0.3% — which at this rover's speed
is more than the ten minutes an `explore` is given, so the run would end on its
budget having left something real unmapped while it perfected a corner. 0.50 m is
the default and keeps essentially all the coverage for four-fifths of the driving.

The simulation is worth exactly what its room is worth, which is why the room is a
recorded one. What it models below the choosing is crude and admits to it: the
lidar never misses a chair leg, a revealed cell never changes its mind, and the
driving never fails. **So the coverage figure is an optimistic bound and the
termination is the result to trust** — every reason a real run would stop early is
missing from it, so a policy that failed to stop here would certainly fail to stop
in the house.

Both run without a rover, on anything with Python:

```bash
python3 frontier.py fixtures/kitchen-loop.pgm.gz
python3 explore_sim.py fixtures/kitchen-loop.pgm.gz --picture /tmp/run
```

`fixtures/kitchen-loop.pgm.gz` is the grid `map_score.py` wrote from that
replay, kept because it is the only real half-explored house this repository has.

`fixtures/start-occupied.json.gz` is the other one: the global costmap as the
planner held it on 2026-09-01, with the rover standing 15.6 cm from a wall and
refusing to plan anywhere. `plan_bench.py --map` takes it as it is, which is how
the refusal above is replayed without a rover:

```bash
python3 plan_bench.py --map fixtures/start-occupied.json.gz         --start=-2.30,1.46 --goal=1.59,1.34 --step 360     # 0 of 1
python3 plan_bench.py --map fixtures/start-occupied.json.gz         --start=-2.40,1.21 --goal=1.59,1.34 --step 360     # 1 of 1
```

## The calibrations, and why they were not optional

`~/ugv/odometry.json` holds what has been measured about this chassis. The base
node reads all of it from there and invents none of it.

```bash
ssh orin
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

Measured on the Jetson on 2026-08-31 over ten seconds, with the whole stack up —
mapping, Nav2, the daemon, the depth service and the drive console — and the rover
parked:

| process | CPU | RSS |
|---|---|---|
| `nav_bridge.py` | ~25% of one core | 89 MB |
| `lidar_node.py` | ~14% of one core | 65 MB |
| `base_node.py` | ~12% of one core | 70 MB |
| `rover_daemon.py` | ~9% of one core | 98 MB |
| each of Nav2's seven servers | 4–6% of one core | 24–56 MB |
| `async_slam_toolbox_node` | under 1% parked, bursty when mapping | 38 MB |

The whole board:

```
over 10 s, 6 cores:  user 16.4%  system 3.1%  idle 79.1%
BUSY 20.9% of all six cores together -- 4.7 cores spare
```

**The Banana Pi ran the same stack at 41.4% of four cores, with 2.3 spare**, and
its two Python nodes each cost about twice what they cost here. The stack was
never close to the edge on that board and is further from it now; what the extra
headroom actually buys is argued in [hosts.md](../docs/hosts.md), not here.

Note that `nav_bridge.py` is the dearest process on the rover. It was not on the
old board's list at all, which measured mapping alone; it earns its keep by being
the only thing that turns ROS into something the console and the voice model can
use, but it is the first place to look if this ever needs to get cheaper.

**Ignore the load average here.** It sits between 4 and 6, which on a six-core
board reads as most of the machine, while 79% of it is idle. Load counts
threads that are runnable, and a dozen ROS nodes each running several Cyclone DDS
threads produce a lot of those without any of them wanting much CPU. The number
that means something is `/proc/stat`:

```bash
ssh orin 'grep "^cpu " /proc/stat'   # twice, and difference the idle column
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

## An obstacle behind it stopped the rover moving at all

Put something close behind this rover and it would not move: not backwards,
which is right, but also not forwards away from the obstacle and not on the
spot. One line of arithmetic in three Nav2 behaviours caused all of it — each
projects the motion it is about to command and tests the footprint at every
projected pose, and the projection starts at the pose the rover is standing in,
so a rover in contact is refused everything regardless of which way it was
going. With the 0.20 m circular footprint and a chassis reaching 0.16 m behind
`base_link`, anything within about 0.17 m behind the sensor froze it.

`behaviors/` replaces those three with subclasses that defer to Nav2 entirely
except in that one state, where stock Nav2 does nothing at all for ever. Turning
becomes unconditional — a circle rotated about its own centre sweeps no new
ground — and driving is allowed along a heading that leads out of contact.
Driving into something is still refused, which is the part that had to survive.

The whole story, the measurements and the one assumption it rests on are in
[`behaviors/README.md`](behaviors/README.md). It is also the ROS stack's only
build step, and the manifest rebuilds it before every restart for the reason
`lidar_slam/` already learned.

## Recording a drive, and replaying it

```bash
ssh orin '~/ugv/ros_nav/record_drive.sh --seconds 300 --name kitchen-loop'   # then drive
ssh orin '~/ugv/ros_nav/replay_bag.sh recordings/bags/kitchen-loop'
ssh orin '~/ugv/ros_nav/replay_bag.sh recordings/bags/kitchen-loop --out tighter               -- -p minimum_travel_distance:=0.1'
```

`record_drive.sh` starts nothing and stops nothing — it subscribes to `/scan`,
`/tf` and `/odom` while somebody drives the rover normally, and writes a bag under
`recordings/`, which the deploy manifest preserves. `replay_bag.sh` then plays it
into slam_toolbox with whatever settings are named and keeps the map that comes
out, as a picture and as the numbers `map_score.py` prints: how large the grid is,
how many cells are walls, and walls per square metre of floor.

**That last number is the one worth quoting, and it can still lie.** A mapper that
has lost track does not produce an empty map, it produces a *bigger* one with every
wall drawn twice — but one run below spread a room over four times its area and
scored *best*, because the exploded floor is what the walls were divided by. The
picture is the arbiter; the number is the summary.

Two things about the replay are deliberate. It runs on **DDS domain 43** while the
rover is on 42, because a replay publishes `/scan`, `/tf` and `/map`, and on one
domain that is a second lidar feed and a second `map -> odom` reaching a rover that
is driving. And it kills only the processes it started, **by PID and never by
pattern**: every pattern that matches a replay's mapper matches the rover's own.
The reverse also holds — a deploy or `restart.sh` during a replay kills the replay,
because those call `sweep.sh`, which kills by pattern for good reasons of its own.

Record a drive, not a rover standing still. slam_toolbox adds a scan to its graph
only once odometry says the rover has moved, so a parked recording tells you
nothing about anything.

## Why RTAB-Map was tried and removed

For part of 2026-08-31 there were two mappers here. RTAB-Map ran first beside
slam_toolbox as a passenger, then instead of it, and was removed the same day. It
is gone from this repository and from the rover — the packages, the second ROS
installation under `/opt/ros/jazzy` they needed, the launch argument that chose
between mappers, and the boot entry that carried it. **This section is what
remains, because the next person to reach for a second mapper should know how the
last attempt ended.**

The question was worth asking: slam_toolbox is a 2D lidar mapper and RTAB-Map is
the better-known system, so **would RTAB-Map be a better owner of `map -> odom` on
this rover?** The answer, measured on one recorded drive replayed into every
arrangement, was no, and not narrowly:

| | walls per m² of floor |
|---|---|
| **slam_toolbox** | **45.3** |
| RTAB-Map, after three configuration faults were fixed | 64.9 |
| RTAB-Map, keyframes every 10 cm instead of 20 | 55.1 |
| RTAB-Map, keyframes every 5 cm | 51.0 |
| RTAB-Map with lidar odometry in front, no motion guess | 27 × 26 m of blur for a 13 × 17 m room |

**The reason is that this rover gives a mapper no camera.** RTAB-Map's reputation
rests on recognising a place from images, and with a lidar alone that mechanism
cannot fire at all — every closure it makes here comes from proximity detection,
which is the same idea slam_toolbox's loop search already uses. So the comparison
was always RTAB-Map's second-best mechanism against slam_toolbox's only one. What
slam_toolbox does that RTAB-Map as configured did not is match *every* scan against
a rolling buffer of the last ten, by an exhaustive correlative search over ±25 cm
and ±20°; RTAB-Map took the wheels as a starting guess and refined once per
keyframe with ICP, which is a local search from wherever dead reckoning had got to.

Three of its faults here were ours and were fixed before the verdict, which is why
the verdict is worth trusting. ICP was allowed to look 10 cm for a point's partner,
less than the error it existed to correct; a match had to pair up 40% of the scan,
which two views of a room from half a metre apart never do; and closures were
searched for out to three metres and then discarded unless the nearest node was
within one, which is RTAB-Map's default and silently undoes the search. Together
those meant **not one loop closed in fourteen minutes of driving**. Fixing them
took RTAB-Map from a map with doubled walls to a plausible one — and still not to
slam_toolbox's.

What survives the removal is worth more than the experiment. `record_drive.sh` and
`replay_bag.sh` came out of needing to ask the question honestly, and they now
answer any mapping question against a drive that has already happened. Four traps
were paid for and are pinned by the selftest: a background child of a
non-interactive shell inherits SIGINT set to ignore, so a recorder started that way
cannot be stopped; a collector must subscribe before a replay rather than after it;
a replay must have its own DDS domain; and a best-effort publisher with a reliable
subscriber is *incompatible* in DDS rather than merely mismatched, which is a
silence, not an error. That last one caught this rover three times, twice under
names that differed between two nodes of the same package.

**What would change the answer** is the OAK-D's colour and depth on a ROS topic,
which is what RTAB-Map is actually for. That is a real project — one process can
hold that camera, so `oak_depth/depth_server.py` would have to give it up — and if
it ever happens, the question gets reopened with a recorded drive and an afternoon
rather than a week.

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
ssh orin 'bash -c "source ~/ugv/ros_nav/env.sh; python3 - <<EOF
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

**It is not finished.** Starting close to a wall, the planner still returns routes about
1.9 times the direct distance, and against one of those DWB goes back to sitting
and rotating — it loses by under one point in forty-five, entirely on the two
critics that measure distance from the path. The grid Dijkstra that used to
draw those corners has been replaced: `SmacPlannerLattice` searches a 0.5 m
differential control set (this chassis's `max_vel_x / max_vel_theta` to a
centimetre, with in-place rotations) and will not draw a driving corner
tighter than one DWB rollout can follow. Reproduced in `lattice.py` on a 55
degree metre-wide bend before it replaced NavFn; see
[docs/doorway-pivot.md](../docs/doorway-pivot.md).

## "lost -- Nav2 gave up without saying why (code 102)", on the long routes

Code 102 is the controller's `TF_ERROR`, which `nav_codes.py` reads as "lost". It
is not the lidar, not the planner and not the floor. It is `map -> odom` having
gone stale by more than a third of a second, and the whole thing turns on a
subtraction between four numbers in two config files.

**Where the budget comes from.** slam_toolbox does not stamp `map -> odom` with
the time it publishes it. It stamps it with the time of the last scan its laser
callback picked up, plus `transform_timeout` -- and in async mode that callback
*is* where scans are processed, so nothing new is picked up while it works. At the
other end, the controller's pose comes back stamped at the newest
`odom -> base_link`, which is about now, and DWB hands it to
`nav_2d_utils::transformPose`, which refuses a transform older than
`transform_tolerance`. Chain them:

    scan published at T, stamped T - 0.10   (lidar_node stamps the start of the sweep)
    map -> odom therefore stamped           T - 0.10 + 0.20 = T + 0.10
    controller needs, at `now`,             something no older than now - 0.30
    so it fails once                        now > T + 0.40

**0.40 s.** That is how long slam_toolbox may go without picking up a scan before
the next control tick throws. Measured on the rover by stopping the mapper with
`SIGSTOP` and watching `transform_age_s` in `nav_status`, the transform passed
0.40 s exactly 0.44 s after the stall began, and came back within one sample of
`SIGCONT`. `tf_stall_sim.py` is the same arithmetic without a rover, and predicted
0.45 s.

**What spends it, and why it is long routes.** Three things, and all three grow
with the route rather than with the goal:

- `updateMap()` takes `smapper_mutex_`, which `addScan()` also needs, and rebuilds
  the occupancy grid from every scan in the graph. It runs every
  `map_update_interval` -- 2 s here -- and only when something subscribes to
  `/map`, which with Nav2 up is always, because the global costmap's static layer
  does. Its cost grows with the graph, and the graph gains a node every 20 cm.
- A loop closure searches an 8 m square and lands near a second on this board. It
  fires when the rover comes back somewhere it has been, which a goal inside one
  room never does.
- The scan subscription is a `tf2_ros::MessageFilter` on `odom`, so a gap in the
  driver board's telemetry withholds scans just as effectively. That one is
  gentler than it looks -- both transforms freeze together, so the controller
  drives on a stale pose rather than aborting -- but it is the same cliff.

**Nothing recovers from it, and that is deliberate.** The behaviour tree gates
every recovery it has behind `WouldAControllerRecoveryHelp`, whose list is
`UNKNOWN`, `PATIENCE_EXCEEDED`, `FAILED_TO_MAKE_PROGRESS` and `NO_VALID_CONTROL`.
`TF_ERROR` is not on it, because clearing a costmap or spinning on the spot cannot
mend a transform. So the six retries never start: in `ros_nav.log` there are 38
milliseconds between `Aborting handle` and `Goal failed`, with no costmap clear
between them. That is why the console says "gave up without saying why" and not
"gave up after N recovery attempts" -- the count really is zero.

**How to tell it apart from everything else that reads as "lost".** The log names
which of the two TF failures happened, and they mean opposite things:

```bash
ssh orin "grep -a -A2 'Transform data too old' ~/ugv/ros_nav/ros_nav.log | tail -30"
```

`Transform data too old when converting from odom to map`, followed by `Unable to
transform robot pose into global plan's frame`, is this. The pair of timestamps on
the middle line is the measurement: subtract them and that is how far past the
0.30 s tolerance it went. Ten of them over two afternoons here ran 0.30 to 0.35 s,
with one at 1.16 s. `Failed to obtain robot pose` instead is the other fault
entirely -- no `odom -> base_link` at all, which is the driver board, and CLAUDE.md
says how to check that one.

**What would widen the budget.** `restamp_tf: true` in `slam_toolbox.yaml` is the
one-line answer and the parameter is present in the 2.8.5 installed here: it
stamps `map -> odom` with `now + transform_timeout` instead of with the last
scan's time, so the correction never expires and the controller keeps steering on
it through a closure. That is the right trade, because `map -> odom` is a slowly
varying correction rather than a measurement -- `odom -> base_link` is still live
underneath it, so the rover's own motion is never stale. Raising `FollowPath`'s
`transform_tolerance` or slam_toolbox's `transform_timeout` buys the same headroom
less honestly. Halving `loop_search_space_dimension` and lengthening
`map_update_interval` make the stalls shorter and rarer but leave the cliff where
it is.

**The console cannot see this coming.** `TRANSFORM_STALE_S` in `nav_limits.py` is
1.0 s, so `position_trusted` stays true right through a stall that has already
abandoned the goal. The number is there in `nav_status` as `transform_age_s`; it
is the threshold that is too loose.

## The rover wiggled in one spot and then timed out, and it was doing its job

A `drive_to` 2.95 m across a room ended in `timed out` after 53 seconds, with the
rover turning back and forth on the spot for most of them. Nothing was stuck.

**The straight line was through a wall.** Measured against the global costmap
along that line, 12 of 30 steps put a *point* at 253 or worse and 27 of 30 put the
rover's *body* in contact. So NavFn did the right thing and returned a route
round: 346 poses, 8.81 m, out west, north, east, north again, east and then south
to the goal — a detour of 3.0 times the direct distance, setting off 92 degrees
off the nose.

Three things then went wrong, and all three read from outside as a rover that
could not find its way.

**The time allowance was worked out from the wrong distance.** `goto` in
`nav_bridge.py` budgeted `6 x straight-line metres / 0.35`, which for a 2.95 m
goal is 50.6 seconds. Driving that route perfectly — no replan, no recovery —
takes about 44: 22 seconds for the 8.81 m and 22 more for the 598 degrees of
turning the corners ask for. It is worth being exact about what was wrong with
that, because the easy version is a half-truth: 50.6 against 44 is not too short
to drive the route, it is *15% of headroom* on a stack that replans once a second
and spends fifteen seconds on each rung of its recovery ladder. The first thing to
go wrong spends all of it.

**The progress checker cannot tell a pivot from a jam.**
`SimpleProgressChecker` measures one thing, how far the rover has moved, and every
direction change this chassis makes is a turn on the spot — `vx_samples: 2` means
any heading change a 0.5 m arc cannot absorb *is* a pivot. So a rover lining up on
the next leg of its route and a rover jammed against a chair leg are the same
event to it. It fired three times in fifteen-second slices. Each firing aborted
`follow_path`, cleared a costmap that had nothing wrong with it, and threw away the
controller's oscillation and goal state.

**And that is where the wiggle came from.** With the controller restarted and the
planner replanning at 1 Hz, a route whose two ways round the wall are close in cost
comes back the *other* way, so the rover turns back. Turn, get called stuck, turn
the other way, get called stuck. Then the bridge's allowance ran out and the
console said `timed out`, which is the one thing that was true and the least useful
way to say it.

### What changed

- **The allowance follows the route.** `route_cost.py` measures a plan's length
  and the heading change its corners demand, and `goto` rebuilds its deadline from
  that on every replan — pushing outwards only, so a plan that comes back shorter
  cannot pull the deadline back past where the rover has already got to. The same
  route now gets 141.9 seconds. There is a floor of 45 seconds under it that is set
  by the recovery ladder rather than by any distance: two progress-checker windows,
  a spin and a wait is about 40 seconds, and a rover cancelled before that has had
  none of the recoveries it carries. The reply says the route length too, so a
  timeout on 8.8 m of detour no longer reads the same as a timeout going nowhere.
- **`PoseProgressChecker` replaces `SimpleProgressChecker`**, which counts a real
  turn as progress as well as a real move. `required_movement_angle` is 20 degrees:
  above the gyro's own standing bias of about +0.43 deg/s over the 15 second window,
  which is 6.5 degrees of pure invention if `base_node`'s correction ever lapsed,
  and far below any turn the rover actually makes.
- **Turning is counted in the budget, not just distance.** Sampling matters and
  does not fully converge — pose by pose that route reads 3259 degrees, every
  0.25 m it reads 598, every metre 422 — because a 5 cm grid path's heading is
  quantised to eight compass points and a straight run is stored as a staircase.
  0.25 m is the setting, over-counting is the safe direction for a backstop, and
  `route_cost.py` says all of that out loud rather than presenting one number.

### `dwb_bench.py`: reading `/evaluation` without driving the rover

`/evaluation` is DWB publishing its own decision — every candidate twist, what each
critic charged it, and which won — and it is the only place that says why a rover
that could move is standing still. Four of the faults in this stack were found in
it and nowhere else. Reading it used to mean driving the rover into the fault first,
in a room, with somebody watching.

`dwb_bench.py` starts a *second* `controller_server` beside the live one, from this
same `config/nav2.yaml`, subscribed to the same live `/scan` and reading the same
live transform tree — and publishes its `/cmd_vel` into a namespace nothing is
listening to. The real DWB, the real critics, the real room, the real pose, and the
wheels never turn. It refuses to start if anything has subscribed to that topic.

```bash
python3 dwb_bench.py --bearing 0,20,40,60,90     # swept off the nose
python3 dwb_bench.py --goal 2.74,-9.86           # the real planned route there
```

Given `--goal` it asks the live planner for the actual route and scores that, which
is the only way to reproduce a particular run — a straight synthetic line answers a
question about a path Nav2 would never have produced. It also prints the budget
arithmetic, so "would this goal have run out of time" is answerable before anybody
drives.

What it measures is the *margin* between the best turn-on-the-spot and the best
forward arc, because that one comparison, made ten times a second, is what decides
whether the rover goes anywhere. Swept across the heading a straight 3 m path sets
off at, from a rover standing in open floor:

| off the nose | forward wins by | as a share |
|---|---|---|
| 0 deg | 9.00 | 23.1% |
| 20 deg | 10.21 | 21.0% |
| 40 deg | 4.81 | 10.0% |
| 60 deg | 2.41 | 5.5% |
| 90 deg | **−11.79** | −20.8% |

So the crossover is somewhere near 70 degrees, and on the real 8.81 m route — which
leaves 92 degrees off the nose — the pivot wins by 24% on every tick. That is
correct behaviour and it is worth saying so: the rover *should* turn before setting
off on that route. What was wrong was everything downstream of it deciding that a
rover which turns is a rover which has failed.

**Two cautions, both learned the hard way by this tool's own first runs.** It sets
`bond_heartbeat_period: 0.0`, because every Nav2 lifecycle node announces itself on
`/bond` under an id taken from its node *name* and the bench's name is
`controller_server` too — a bench that bonded would be a second heartbeat under the
name the live lifecycle manager is watching, and the bench exiting would read as the
real controller dying. And it kills its whole process group: `ros2 run` is a
launcher, so terminating it leaves an orphan holding the node name, and two nodes of
one name make `ros2 lifecycle` silently answer for whichever it found first.

## The rover turned 3038 degrees and finished where it started, and the model said it could not have

Recorded on 2026-08-25 with `nav_record.py`: a minute of driving, 1.10 m of
path, 3038 degrees of turning, and 28 cm between the start and the end. 506 of
the 511 velocity commands were turns on the spot, 486 of them at exactly the
slowest pivot the speed floor allows, and the direction reversed 93 times. The
recording is `recordings/trap-2026-08-25-spin.json`.

Replayed offline it agreed with nothing: on 329 of 511 ticks the model refused
all twenty-nine candidates, saying the rover was sealed off from its goal, while
the rover itself had commanded a pivot on every one of those ticks and the log
has no "could not find a legal trajectory" anywhere in that minute. **A model
and a rover that disagree that completely are not describing the same fault,
and the model was the one that was wrong.**

**`MapGridCritic`'s flood is not stopped by walls in the library this rover
runs.** Every account of that critic -- upstream's source, this repository's
own notes, four sections of this file -- says the queue refuses 253, 254 and
255 and marks them `obstacle_score_`. In the `libdwb_critics.so` that RoboStack
installed here it is two instructions:

```
dwb_critics::MapGridCritic::MapGridQueue::validCellToQueue:
    mov  w0, #0x1
    ret
```

and nothing in the whole library calls `MapGridCritic::setAsObstacle`. So the
Manhattan distance spreads straight through walls and out the other side. Once
a critic has one seed on its window every cell of that window carries a real
distance, `unreachable_score_` survives only for a critic given no seed at all,
and `PathDist` and `GoalDist` cannot refuse anything except a pose off the grid.

Correcting `corridor_sim.flood` took the model's agreement with that drive from
15% to 84%, and its hit rate on the rover's exact twist from 61 ticks to 386 of
511. That is the first time this fault has had a model that passes its own
gate.

**What it invalidates.** Two tuning arguments rested on the charge the flood was
believed to produce -- `unreachable_score_`, 2881 points once scaled, for a nose
point the flood could not reach. Neither survives: measured over this recording,
0 of 8687 driving candidates and 0 of 6132 pivots are charged it. So the reason
recorded for moving the align look-ahead to 0.325 and for adding `PreferForward`
is withdrawn, and the settings are left exactly where they are, because nothing
measured argues for anything else either. `trap_sim.py --bias` and the comments
in `config/nav2.yaml` say so where somebody about to change them will look.

**What the fault actually is, now that it can be measured.** Two things, and the
first is the larger:

- **On 41% of the ticks not one forward candidate was legal.** The rover was
  standing where every rollout that moved ended on a cell at 253 or worse, so
  `BaseObstacle` refused it and sixteen ways of turning on the spot were the
  only choices left. With `robot_radius: 0.20` the inflation layer paints that
  ring 20 cm deep, and in the recorded spot -- the rover's own cell reads 220 in
  the global costmap -- there is no forward move that leaves it.
- **On the rest, driving lost to turning by a median 4.6 points**, and the bill
  is `PathAlign` +4.0 and `PathDist` +3.2 against `GoalDist` and `GoalAlign`
  pulling back 1.8 each. Both of the path critics are saying the same thing:
  moving would end further from the planned line than standing still does. The
  last twenty-five points of that plan lie on cells the *local* costmap calls
  253 or 254, so the line they are measuring against runs through a wall.

Driven forward from the rover's own costmap and its own plan, the model spins
in place: 0.00 m in twelve seconds, 422 degrees of turning, twenty reversals,
from all twelve starting points and at every align look-ahead from 0.325 to
0.8. It does that with a chassis that obeys perfectly as well as with the
measured one, so the delay and the over-served pivot are not what is doing it.
Holding the costmap fixed is fair on this drive and would not be on another:
the rover never got more than 22 cm from where it had been twelve seconds
earlier, so its 3 m window would have rolled by at most four cells.

### What the trap is made of, and four fixes that are not it

The reproduction is good enough to kill a candidate in a few minutes, so three
were killed before anything was changed on the rover.

**A smaller body does not do it.** `trap_sim.py --rings` re-inflates the
recorded costmap for each shape and drives the model from twelve starts. From a
0.20 m circle down to 0.10 m, and through the measured rectangle, a forward move
becomes *legal* far more often -- 33 ticks of 52 becomes 43 -- and the rover
still never picks one, because the margin against driving only falls from 4.3
points to 3.4. The one escape, at 0.10 m, takes the rover's centre within
0.16 m of a cell the lidar saw something in, and the real body is 0.14 m to its
nearest edge. That is a collision, not an escape, which is why the sweep reports
clearance beside the count: the model knows what the costmap forbids and nothing
about the rover hitting anything, so a smaller body always scores better.

**Nor does letting it turn faster.** `PreferForward` charges `|theta| * 10 *
scale`, so the cheapest turn is always the slowest one and the rover pivots at
15 deg/s when its plan is 140 degrees away -- nine seconds of uninterrupted
turning. Setting that scale to zero makes it choose 39 deg/s instead and turn
604 degrees rather than 422, and it escapes 0 of 12.

**Nor does turning it to face the plan first.** Driving the rotation directly to
within 20 degrees of the plan's heading, which is what a `Spin` recovery would
do, escapes 1 of 12 and ends a median 97 degrees off the plan, because DWB turns
back out of the alignment as soon as it is handed control. It does not want to
be pointed along that plan.

**What it does want is to point at a wall.** `GoalDist` and `GoalAlign` flood
from the last plan point on the window, and this build's flood runs through
walls, so what they reward is the straight-line direction to a goal on the far
side of one. Flooding the same seed both ways at the rover's own poses, the best
nose bearing under the library's flood and under the wall-respecting flood
upstream intends are 75 and 120 degrees apart; on many ticks the seed sits on an
inscribed cell, where upstream's flood would have had no answer at all. Every
forward move in the direction the field likes is refused by the obstacle critic,
and the pivots that are left are separated by 0.4 points of aiming signal
against a 3.4-point turn-rate charge. So the rover turns, and turns.

**Two costmaps that disagree by more than the corridor's margin.** The plan is
drawn on the global costmap, which is slam_toolbox's map; the critics test it
against the local one, built from the live scan. Transformed into the same frame
and compared cell by cell, the local costmap's lethal cells sit a median 0.15 m
from anything the map knows about at the start of this drive, falling to 0.06 m
by the end, with a quarter of them beyond 0.31 m and the worst at 0.75 m. The
inscribed ring is 0.20 m, so that disagreement is enough on its own to put the
planner's route inside the controller's walls -- which is exactly where the last
twenty-five points of it are.

### The controller is aimed round the corner, and that is the trap

The rover is not being asked to do anything it cannot do. It is being aimed at
somewhere it cannot get to, and the reason is one number that was never binding.

DWB does not steer at the goal an operator asked for. It hands the critics the
part of the plan within `min(half the local costmap, forward_prune_distance)`
of the rover and treats the far end of that piece as the goal, which `GoalDist`
and `GoalAlign` then flood outward from. That distance is a *radius*, and with
a 3 m window and the parameter left at its 2.0 default the costmap's own 1.5 m
was always the one that won. Where the route turns a corner the plan doubles
back inside that circle, so the piece kept is far longer than the circle is
wide: on `recordings/trap-2026-08-25-spin.json` it is a median 3.34 m of
driving whose far end is 1.48 m away, and on 34 of 52 ticks that far end is a
cell the rover's centre may not occupy. Because this build's flood runs through
walls, the critics reward the straight-line direction to a point behind one,
the obstacle critic refuses every forward move that way, and pivoting is free.

`trap_sim.py --aim` brings the radius in and drives the model from twelve
starts. The plateau has hard edges: at 2.0 m the rover gets nowhere from all
twelve starts; at 1.1 m it escapes 3; at 1.0 m and below the far end of the
plan stops being a wall entirely and it escapes 9 or 10 of 12, moving 1.15 m.
Below about 0.6 m the seed lands inside the rover's own inflated ring instead
and it goes back to spinning. `forward_prune_distance: 0.9` is the middle of
the part where the seed is never a wall.

Two warnings go with it. The plateau is this room's: what sets the upper edge
is how far the rover was from that corner, so the number transfers only as
"aim nearer than the first bend". And the escapes clear a real lidar return by
0.20 to 0.25 m against a body whose corner is 0.24 m out, so this gets the
rover moving without getting it through the 0.70 m gap cleanly -- approaching
the gap centred is a separate problem, and it is still open. The general fix is
to seed the goal critics from a point the flood can reach at all; this is a
mitigation the reproduction can price today.

The model was checked against the recording after the change: agreement is
still 84%, because `FORWARD_PRUNE_M` defaults to the 2.0 the rover was running
and at 2.0 it changes nothing.

### What is still open

- **The controller aims the rover round the corner at a wall, because the plan
  is cut to a straight-line radius and the distance field it steers by floods
  through walls.** Pulling `FollowPath.forward_prune_distance` in to 0.9 m gets
  the model out from nine or ten of twelve starts (`trap_sim.py --aim`), and is
  deployed. It is a mitigation: the escapes still clip the corner, so getting
  the rover to approach an opening centred is still open. The older reading of
  this bullet follows and still holds for what happens once it is aimed badly.

- **The controller aims the rover at walls, because the distance field it
  steers by does not know they are there.** That is the trap above and it is
  the open fault. It was met again on 2026-09-01, this time unattended, by the
  first `explore` run: fifty seconds, six centimetres, forty-three replans and
  no recovery attempted, because `PoseProgressChecker` counts a pivot as
  progress. Exploring now gives such a goal up after twenty-five seconds and
  drives to a different frontier — a mitigation that stops it costing the
  budget, and no kind of fix. See "It gives up on a goal that is going nowhere". Three candidate fixes have been tried against the
  reproduction and all three are dead: a smaller body, removing
  `PreferForward`'s rate charge, and turning the rover to the plan's heading
  before handing over. What is left is to stop the local goal being a point
  behind a wall -- prune the plan to the last point that is genuinely in free
  space before `GoalDist` seeds from it, or give the critics a flood that
  respects the inflated ring the way upstream's does. The reproduction to test
  either against is `dwb_replay.py recordings/trap-2026-08-25-spin.json
  --drive`, which has to stop reporting STUCK from all twelve starts, and
  `trap_sim.py --rings` is the pattern for how to price a candidate rather than
  just count its escapes.
- **The pivot channel over-serves the smallest rotation the controller can ask
  for, by four times.** DWB's sixteen rotation samples run in 5.96 deg/s steps, and
  `drive_mixer.turn_to_pwm` lifts any pivot request below `MIN_TURN_DPS` up to it.
  Against this chassis's own measured curve: asked 2.98 deg/s the wheels deliver
  11.93, asked 8.94 they deliver 11.93, and everything from 14.9 up is right to
  within 1%. `MIN_TURN_DPS = 12.0` is a constant from `lidar_slam/nav_types.py`,
  measured on the rover as it was, and this chassis's slowest measured pivot is
  9.17 deg/s — so the floor is 31% above what the rover can actually do, and the
  number setting it was measured on a different machine. The *rolling* steering
  channel is fine and was fixed earlier: it lands within 6% from 8.94 deg/s up.
  This is not what caused the stall above — with an 0.8 s rollout executed 0.1 s at
  a time, a plant error of four is a loop gain of a half — but it is a real defect
  and the floor should come from the measured curve rather than from the old rover.
- **Nothing checks that the rover can *leave* where it stands.** `goal_fit.py`
  tests the goal against the body and moves it; the start is unguarded. Measured on
  the rover on 2026-08-24, parked after a manual drive: its own cell read 253, the
  body was in contact at all 24 headings tried, and `ComputePathToPose` returned
  error 208 with zero poses. `drive_to` answers that with "there is no route", which
  sends somebody to look at the map when what is needed is 20 cm of reverse.
- The route being 3.0 times the direct distance is still the planner's, even
  though the corners it draws are no longer tighter than the chassis. The
  lattice will go the long way round when the short way is a cut it cannot
  take while driving, or insert a pivot when an arc will not fit, and that
  is the right trade: a followable detour against a 55-second lock-up. The
  smoother that is configured but never invoked was the obvious next thing to
  try on the old grid paths, and it turned out not to be the fix — see
  [docs/doorway-pivot.md](../docs/doorway-pivot.md).

## "There is no route to there" was the clock, not the room

Long goals started failing outright. On 2026-08-27 one of them was refused
thirteen times in 47 seconds from a standstill and then drove the moment a
recovery spin had turned the rover round, which made the start heading look like
the cause -- a route that has to begin with a turn on the spot is exactly the
sort of thing a lattice planner might be bad at. It was not that.

**The planner was running out of its two seconds, and Nav2 says so in the words
of a different fault.** `SmacPlannerLattice` throws `NoValidPathCouldBeFound`
when the search ends without reaching the goal and `max_iterations` has *not*
been exhausted -- and `max_iterations` is a million here, so the thing that
stops it is always the clock. That arrives as error 208, which
[`nav_codes.py`](nav_codes.py) renders as "there is no route to there that the
rover fits through". A timeout therefore reads as a room with no way through it,
and sends somebody to look at the map.

[`plan_bench.py`](plan_bench.py) settled it without the rover moving. It starts a
second `planner_server` beside the live one, from this same `config/nav2.yaml`,
in its own namespace and its own `benchmap`/`benchbase` frames, and feeds its
global costmap a recorded grid instead of slam_toolbox's. Against
`recordings/trap-2026-08-25-spin.json` the bench costmap came back identical to
the recorded one in all 68,540 cells, which is what makes anything measured on it
a statement about the room the rover was in.

The measurement that mattered is the dullest one available -- the same query, run
again:

| | at `max_planning_time: 2.0` | at 3.0 |
|---|---|---|
| one start heading, ten runs | 4 planned, 6 refused | **10 planned** |
| ...and every success took | 2.01 to 2.09 s | 1.94 to 2.28 s |
| swept over sixteen start headings | 9 planned | **16 planned** |
| ...slowest plan among them | 1.99 s | 2.36 s |

The right-hand column was taken with the bench reading `config/nav2.yaml` off
the rover after the first deploy, when the budget there was 3.0.

So it is a coin toss, and the coin is a wall-clock deadline on a board that is
also running slam_toolbox, DWB and the daemon — four cores when this was measured,
six now, which widens the margin without changing the shape of the problem. Re-running the identical
sixteen-heading sweep moved *which* headings failed. The start heading is real
but small: it shifts the cost of a query that already costs about the budget,
which is why turning the rover round appeared to fix it and why thirteen goals
under 5.5 m in that session produced no aborts at all while nine of 6 m and over
produced twenty-four.

**`max_planning_time` is 4.0, and it was 3.0 first.** Three seconds was the
same mistake made smaller. Asked of the *live* planner on the *live* map, one
plan came back at 3.12 s -- a single sample in roughly seventy, but a ceiling
with the tail of the distribution poking over it is exactly what was wrong at
two seconds, and one spurious "there is no route" every seventy long goals is
not a fix. Across the recorded map and the live one the expensive class runs
1.1 to 2.5 s with that one excursion above it.

Raising it costs nothing on a plan that succeeds, because the planner returns
the moment it has a route rather than spending its budget. The cost falls only
on plans that were going to fail, where it is spent on every rung of the
recovery ladder and has to fit the bridge's allowance in
[`route_cost.py`](route_cost.py). That is why the number is generous rather
than tight.

Two things every measurement here shares, and both understate it. They were
taken with the rover standing still, so DWB was not also running at 10 Hz
beside them. And the deadline is soft: `terminal_checking_interval` is
5000 iterations, so the search notices the clock late and overshoots rather
than stopping on it -- which is how a 3.12 s plan returned a route at all.

If long goals start being refused again, the better answer is to make the
search cheaper -- a smaller map window, or a coarser angular resolution --
rather than to keep raising this.

### A model said `rotation_penalty`, and the rover said no

Worth recording because it is the failure this repository keeps having. The
tempting explanation is structural and reads well: `getTraversalCost` charges an
in-place rotation a flat `rotation_penalty` whatever it achieves, while
`getHeuristicCost` returns `max(obstacle_heuristic, distance_heuristic)` and the
distance heuristic -- the only one of the two that knows which way the rover is
pointing -- is zero outside a window of `lookup_table_size` metres around the
goal. More than five metres out a rotation therefore costs and repays nothing,
so A* should expand everything it can drive to before buying the eight rotations
a half-turn needs.

A Python model of that search agreed enthusiastically: the same route cost it 5.4
times as many expansions from its worst start heading as its best, peaking
exactly at 180 degrees, and dropping `rotation_penalty` from 5.0 to 2.0 took the
spread to 2.0. Priced against the doorway it looked free -- `lattice.py` draws
the same 34.9-degree arc with no pivot at every value from 5.0 down to 0.5.

Run on the real planner it was **worse**: 5 of 16 start headings planned at 2.0
against 9 of 16 at 5.0. Cheap rotations widen the branching in the heading
dimension, so the search has more states to get through, not fewer. The model was
counting expansions to the first solution and had no notion of the frontier it
left behind. `rotation_penalty` stays at 5.0, and the note beside it in
`config/nav2.yaml` says not to lower it again without running that sweep.

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
same tools and the same replies and could not be told apart from a console. The
daemon's startup line no longer has to be checked to know which planner answered a
move; if it says anything but `driving ros2 on 127.0.0.1:8773` then nothing is
driving at all.

Getting the old stack back means `git revert`, not a crontab edit. The commit that
removed it is one commit and it took `lidar_slam/`'s README with it, so the
reasoning is recoverable along with the code.

## The processes are up and the rover will not drive

The console line "Nav2 is not running, so the rover will not drive itself. Only
the mapping half of the stack is up" is this, not a crash. slam_toolbox and Nav2
are listed. The lidar is still logging 9.9 Hz. The map picture is the last grid
the bridge still has. What failed is the ROS graph: CycloneDDS is trying leftover
peers from an address the rover no longer has (`192.168.1.139`, a DHCP lease
that had moved, is the one we have actually seen) and from multicast
`239.255.0.1`. Scans and TF
stop arriving. The bridge cannot see Nav2's action server in two seconds, and
that is the canned sentence.

[`dds.sh`](dds.sh) pins discovery to loopback. The consoles talk TCP 8769 / 8773,
not ROS, so they do not need the graph on the LAN. `sweep.sh` then SIGKILLs
whatever ignored SIGTERM, because a wedged CycloneDDS participant sits inside
`rclpy.spin` past the two-second wait and the next launch starts a second
lidar_node on the same serial port.

## Reloading it after a deploy

```bash
ssh orin '~/ugv/ros_nav/restart.sh'                # new nodes or config
ssh orin '~/ugv/ros_nav/restart.sh --supervisor'    # new run_ros_nav.sh
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
a port looks like from outside. SIGTERM is not enough when CycloneDDS is wedged,
so the sweep SIGKILLs what is left before the next launch.

A change to [`dds.sh`](dds.sh) is picked up by a child restart: `run_ros_nav.sh`
sources it every time around the loop, the same reason the sweep is a file.
