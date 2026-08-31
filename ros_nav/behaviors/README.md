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

Nav2 1.3.12, which is what this rover runs, has no parameter that turns the
check off — checked against the strings in `libnav2_spin_behavior.so` and its
neighbours, not assumed. That is why this is compiled code and not a setting.

## What they change

Each class calls Nav2's own implementation first and returns its answer
untouched unless it is specifically `COLLISION_AHEAD`. **Every healthy motion
this rover makes is still Nav2's code, byte for byte.**

When Nav2 does refuse, the rover's current pose decides what happens:

- **standing somewhere legal** — the obstruction is genuinely in the way, which
  is what the check is for. The refusal stands, untouched. This is what keeps
  the rover from driving into walls, and it is the case that matters most.
- **already in contact** — stock Nav2 will now refuse every motion in every
  direction, for ever. This is the state these plugins exist for.

In that second state, `EscapeSpin` allows the rotation. That is sound because
the footprint is a circle centred on `base_link`: rotating it about its own
centre maps it onto itself, so a turn cannot sweep ground the rover is not
already standing on. It is the same fact that makes Nav2's check useless here —
with a circular body it is either vacuous or an unconditional veto, and
`corridor_sim` shows exactly that, 90.1° or 0.0° and never anything between.

**If the footprint ever stops being a circle that reasoning stops holding.**
`../selftest.py` fails if `nav2.yaml` grows a footprint polygon, which is the
alarm for it.

`EscapeDriveOnHeadingAction` — and `EscapeBackUpAction`, the same template with
the sign flipped, exactly as Nav2 builds them — allows the motion only when the
far end of the projection is clear, meaning it leads out of contact rather than
deeper in. Driving forward off a rear obstacle passes. Reversing into that same
obstacle does not, and neither does driving forward into a wall while something
is behind: the wedged case, where no is still the honest answer.

Escaping is time limited by `escape_time_limit` (3 s). A rover still in contact
after that stops and reports the collision, because at that point it is grinding
rather than escaping.

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

`build.sh` prints what it produced and runs `ldd -r` over it, because the build
that matters is not the one that compiled but the one the behaviour server can
load: an undefined symbol appears only at plugin-load time, in the launch log,
long after the deploy said it was fine.

## What is proved and what is not

The fix is proved in the model and the plugins are proved to load and run on the
rover — `ros_nav.log` names all three classes at startup, and turning the rover
through the replaced `Spin` still works normally.

**The escape itself has not been watched on the hardware.** That needs somebody
to put an object behind the rover and ask it to drive away, which is not
something a deploy can do. Until then the honest statement is that the fault
reproduces in a model that was read off this rover's own libraries, and the fix
succeeds in that same model.
