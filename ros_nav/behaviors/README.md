# `ros_nav/behaviors/` — getting off something the rover is touching

Three Nav2 behaviour plugins, replacing `Spin`, `DriveOnHeading` and `BackUp`.
They exist because of one fault, and they change one thing.

## The fault

Put an obstacle close behind this rover and it stops being able to move at all.
Not backwards, which is correct — but also not forwards, away from the thing
blocking it, and not on the spot.

The cause is a single line of arithmetic repeated in all three of Nav2's
behaviours. Each one projects the motion it is about to command forward over
`simulate_ahead_time` and tests the footprint at every projected pose. The
projection starts at cycle zero, where the simulated displacement is zero — so
**the first pose tested is the pose the rover is standing in right now**, and if
that one is in collision the whole behaviour returns `COLLISION_AHEAD` without
moving. Which way the rover was going never enters into it.

The footprint is a 0.20 m circle on `base_link` while the chassis reaches 0.16 m
behind it — [`../config/nav2.yaml`](../config/nav2.yaml) explains at length why a
circle and not the measured rectangle — so anything within about 0.17 m behind
the sensor freezes the rover completely.

Reproduced at a desk in [`../corridor_sim.py`](../corridor_sim.py), which models
these behaviours off the rover's own Nav2 libraries. With a wall 0.12 m behind,
which is inside the chassis and therefore genuinely touching:

| | turn 90° | drive 0.5 m forward | reverse 0.3 m |
|---|---|---|---|
| stock Nav2 | 0.0° | 0.00 m | 0.00 m |
| these plugins | 90.0° | 0.50 m | 0.00 m |

And on the rover, the 180° turn that started this, which used to come back
`blocked -- turning that way would sweep through something`:

```text
turn +180 -> arrived, turned 186.8
turn -180 -> arrived, turned -189.9
turn  +90 -> arrived, turned  97.2
```

The few degrees of overshoot are the chassis's own stiction floor, which the
README in the directory above measures; they are not this.

Nav2 1.3.12, which is what this rover runs, has no parameter that turns the
check off — checked against the strings in `libnav2_spin_behavior.so` and its
neighbours, not assumed. That is why this is compiled code and not a setting.

## What they change

Each class calls Nav2's own implementation first and returns its answer
untouched unless it is specifically `COLLISION_AHEAD`. **Every healthy motion
this rover makes is still Nav2's code, byte for byte.**

Turning and driving are then treated quite differently, because the geometry is
quite different: a rotation about the body's own centre covers the same ground
whatever the heading, while a translation genuinely covers new ground. So the
driving half keeps Nav2's check almost entirely and the turning half does not
need it at all.

`EscapeSpin` goes further than that, and has to. **A rover with a circular
footprint is never refused a turn at all**, whether or not it is in contact.

Rotating a circle about its own centre maps it onto itself, so the ground
covered is identical at every heading: if the rover fits where it stands it fits
at every heading, and if it does not, no heading helps. Nav2's check cannot add
information — it can only agree with the pose the rover is already in, or
disagree with it wrongly. It disagrees because it does not test a circle. A
radius becomes a sixteen-sided polygon and the test walks that outline across a
5 cm grid, so rotating it crosses a slightly different set of cells and one
marginal cell becomes "turning that way would sweep through something" on a
rover standing in open floor. That was watched here on a 180° turn, and it is
what the first version of these plugins failed to fix: escaping only when
already in contact left the common case — standing legally, refused anyway —
untouched.

Reproduced at a desk too: a single lethal cell 0.16 m out leaves the rover legal
where it stands and refused at 108 of 120 headings. A straight wall will not
show it, because a half-plane is symmetric enough that a near-circular outline
clips it at every heading or none — which is why the first version of the desk
test passed while the rover was still stuck.

**If the footprint ever stops being a circle that reasoning stops holding**, so
the plugin measures it rather than trusting the config: it reads the same
footprint topic the collision checker does, takes the shortest vertex over the
longest **about the polygon's own centroid**, and checks that centroid against
`base_link`. Nav2's circle scores 0.98 with an offset of 0.001 m; the rover's
old measured rectangle would score 0.66. A non-circular body keeps Nav2's check
except when already in contact.

The centroid matters and cost a debugging round. `published_footprint` carries
the footprint **already placed at the rover's pose in the costmap's frame**, not
a shape in `base_link` — so measuring vertices from the message origin measures
the rover's distance from `odom`. Parked far out, every footprint scores as a
perfect circle; near the origin the same one scores 0.03. It was watched
swinging between the two within seconds of driving, which made one turn succeed
and every later one fail, and made the success look like the fix working when it
was luck.

`EscapeDriveOnHeadingAction` — and `EscapeBackUpAction`, the same template with
the sign flipped, exactly as Nav2 builds them — allows the motion only when the
far end of the projection is clear, meaning it leads out of contact rather than
deeper in. Driving forward off a rear obstacle passes. Reversing into that same
obstacle does not, and neither does driving forward into a wall while something
is behind: the wedged case, where no is still the honest answer.

Escaping is limited by **lack of progress** rather than by the clock
(`escape_time_limit`, 3 s). Against elapsed time it would be wrong: a 180° turn
at the recovery speed of 0.5 rad/s takes over six seconds, so a stopwatch would
cut off exactly the manoeuvre this exists to allow. A rover that has not moved
for three seconds is grinding, and stops.

## Building it

```bash
~/ugv/ros_nav/behaviors/build.sh
```

This is the ROS stack's **one** build step, and it is a deliberate exception.
[`../install.sh`](../install.sh) explains why there is otherwise no workspace
here: every node is a script the launch files name by absolute path, so there is
nothing to build and therefore nothing to forget to rebuild. A pluginlib class
has no such option — it is a shared object or it is nothing.

So it carries the same protection `lidar_slam/` does: **the manifest runs
`build.sh` before `restart.sh` on every deploy of `ros_nav`**, because a stale
`.so` is a rover running last week's code with this week's source beside it on
disk and nothing anywhere saying so. `../selftest.py` checks that the manifest
still does this, and in that order.

It installs into `behaviors/install` rather than into the conda environment. The
environment is an installed dependency that `install.sh` builds and a
`mamba install` may rewrite; a deploy has no business editing it.
[`../nav.launch.py`](../nav.launch.py) puts that prefix on the behaviour
server's environment and on nobody else's — `AMENT_PREFIX_PATH` so pluginlib
finds the class, `LD_LIBRARY_PATH` so it can then load the library. A missing
one of those shows up as `Failed to create behavior`, which reads like a typo in
the class name rather than a path.

**The build is keyed on a hash of the sources, never on their timestamps, and
without that it never rebuilds at all.** `deploy.py` packs every file with
`mtime = 0` on purpose, so that an unchanged file has an unchanged tar and
rsync's quick check can skip it. The consequence here is that every source on
the rover is dated 1970 and is therefore older than any object file already
built, so an incremental build finds nothing to do — for ever. This was not
theoretical: three deploys in a row rebuilt these plugins and the running
behaviour server kept the library from the first one, while the source beside it
plainly said otherwise. It is the same fault `lidar_slam/build.sh` avoids by
having no build cache at all.

`build.sh` also prints what it produced and runs `ldd -r` over it, because the
build that matters is not the one that compiled but the one the behaviour server
can load: an undefined symbol appears only at plugin-load time, in the launch
log, long after the deploy said it was fine.

## What is proved and what is not

**The turn is proved on the hardware.** The 180° turn that started this now
completes in both directions, repeatably, and the log shows it going through the
escape path with the footprint measured at 0.98 roundness and a 1 mm centroid
offset.

**The drive-away has not been watched on the hardware.** That needs somebody to
put an object behind the rover and ask it to drive off, which is not something a
deploy can do — [`../escape_test.py`](../escape_test.py) is that run as one
command. Until then the honest statement for the driving half is that the fault
reproduces in a model read off this rover's own libraries, and the fix succeeds
in that same model.
