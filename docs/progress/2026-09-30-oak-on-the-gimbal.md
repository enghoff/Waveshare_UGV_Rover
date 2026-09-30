# The OAK rides the gimbal now, and it exposed two lens errors

**The OAK is on the gimbal's rail, the mount is rigid, and the software treats it
as a second lens on the gimbal rather than a camera bolted to the chassis.** Its
position relative to the gimbal camera was measured in two runs at fifteen gimbal
positions and came out the same at all of them to a tenth of a degree, so the servos no longer
come between the two cameras and the middle of every fisheye picture has depth
behind it wherever the gimbal points. The same measurement found two lens errors
that predate the move: the OAK's lens as the depth service published it was 9.6%
short in focal length, which is fixed, and the fisheye's lens model is about 7%
short of the angles it describes, which is recorded against
[R-WS-10](../requirements/world-state.md#r-ws-10) and not fixed here. Pan to ±90
and tilt from -30 to +60 were exercised with the OAK attached; the ends of the
gimbal's travel were not.

No requirement changes state. [R-WS-10](../requirements/world-state.md#r-ws-10)
stays `failing` with a new, measured term; [R-WS-11](../requirements/world-state.md#r-ws-11)
stays `open` with its OAK offset re-measured on the rail.

## The hardware

The owner moved the OAK-D-Lite from its chassis bracket onto the Picatinny rail
on the gimbal's tilt platform, using the printed mount in [`cad/`](../../cad/README.md),
with its sensors in the plane of the gimbal camera's. The rover was switched off
from 2026-09-08 19:54 until it booted with the new mount at 2026-09-30 14:22, so
nothing it recorded falls between the two mounts.

- **Both cameras work and see the same way.** The OAK's picture is upright and
  centred on what the gimbal camera's is centred on. Nothing of the mount or the
  OAK appears in the gimbal camera's frame, and since the two are rigid, one frame
  settles that for every pan and tilt.
- **The mount is rigid.** See the next section: the rotation between the cameras,
  fitted separately at each of fifteen gimbal positions from pan -45 to +45 and
  tilt 0 to 40, agrees to 0.13 degrees of yaw, with no trend in tilt, which is
  what a clamp sagging under the camera's weight would show first.
- **The servos carry it.** The OAK's own IMU, read while the gimbal stepped, is a
  servo-independent measurement of where the platform went. Tilt reached every
  commanded angle to within 1.3 degrees; pan's 30-degree steps measured 28.6 to
  30.6, the short ones each the first step after a reversal, which is the known
  backlash. The USB link held for the whole sweep.

| commanded tilt, pan 0 | -15 | -30 | +20 | +40 | +60 |
|---|---:|---:|---:|---:|---:|
| gravity turned by, degrees | 15.56 | 31.25 | 19.96 | 40.69 | 60.88 |

Back at level the tilt read 0.75 degrees off after coming up from -30 and 0.17
after coming down from +60, which is tilt backlash of the same order as pan's.
With the tilt at level, panning moved gravity by 0.16 to 0.37 degrees, so the pan
axis is within about a third of a degree of the platform's normal.

**Not exercised: pan beyond ±90 and tilt beyond +60.** The face search sweeps pan
to ±180 and face tracking may tilt to +90, and the OAK's USB cable now travels
with the gimbal. Whether the cable has slack for half a turn each way, and
whether the back of the mount clears the pan base at +90, was left to the owner
to see before the rover is driven there.

The OAK chip read 61 °C after an hour streaming at 15 fps, which is well inside its
limits; there is no earlier figure under load to compare the mount's effect on
cooling with.

## The mount

`world_state/bench_oak.py` matched features between the two cameras' pictures of
the room at pan -30, 0 and +30 and tilt 0, 20 and 40, 848 matches ranged by the OAK
2.4 to 5.5 m out. The bench now takes a tilt as well as a pan and reports the
mount in the gimbal camera's own frame. This first run's points were collected
through the OAK lens as published and are re-expressed through the corrected one
below (the ranges are the service's own, turned back into depth along the axis
and out again). They were fitted with one rigid model for all nine positions:

| model, one fit for all nine positions | yaw | pitch | roll | forward | left | up | fisheye scale | median miss |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| rotation only, OAK lens as published | | | | | | | | 1.14 deg |
| rotation only, OAK lens corrected | +2.63 | +1.27 | -1.15 | | | | | 0.94 deg |
| rotation and offset | +1.59 | +0.38 | -0.77 | +0.234 | -0.021 | +0.071 | | 0.26 deg |
| rotation, offset and fisheye scale | +1.85 | +0.71 | -0.81 | +0.019 | -0.009 | +0.052 | 1.066 | 0.24 deg |
| the same, forward held at 0 | +1.89 | +0.73 | -0.82 | 0 | -0.007 | +0.051 | 1.072 | 0.24 deg |

Offsets are metres in the gimbal camera's frame. **The forward offset is the
owner's** -- the sensors were set in one plane -- because forward is the one
direction the bench cannot separate from the fisheye's angular scale: left free
with the scale fixed it claims 23 cm, which is the scale error spending itself
somewhere. Holding it at nothing moves nothing else by more than 0.04 degrees or
2 mm.

That fit is now `bench_oak.py --joint`, and a second run was taken with it after
the corrected lens was deployed, at different positions -- pan -45, 0 and +45 at
tilt 10 and 30, 585 matches -- and fitted the same way, then both together:

| forward held at 0 | yaw | pitch | roll | left | up | fisheye scale | points | median miss |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| first run | +1.89 | +0.73 | -0.82 | -0.007 | +0.051 | 1.0719 | 836 | 0.24 deg |
| second run | +1.98 | +0.92 | -0.71 | -0.002 | +0.038 | 1.0716 | 581 | 0.23 deg |
| **both, adopted** | **+1.92** | **+0.80** | **-0.80** | **-0.005** | **+0.046** | 1.0718 | 1417 | 0.23 deg |

The fisheye's scale repeats to three parts in ten thousand. The mount repeats to
about a tenth of a degree and, in height, to about a centimetre: each run's own
resampling intervals are half as wide as the difference between the runs, because
height and pitch trade against each other through whatever ranges the room
offered, so the difference between runs is the figure to believe.

With the adopted offset and scale held and only the rotation fitted, position by
position, over both runs' points:

| pan, tilt | points | yaw | pitch | roll | median miss |
|---|---:|---:|---:|---:|---:|
| -45, 10 | 94 | +1.91 | +0.77 | -0.74 | 0.17 |
| -45, 30 | 132 | +1.88 | +0.83 | -0.84 | 0.22 |
| -30, 0 | 22 | +1.91 | +1.01 | -0.71 | 0.21 |
| -30, 20 | 190 | +1.89 | +0.72 | -1.03 | 0.21 |
| -30, 40 | 56 | +2.01 | +0.82 | -0.82 | 0.18 |
| 0, 0 | 125 | +1.90 | +0.81 | -0.73 | 0.21 |
| 0, 10 | 144 | +1.95 | +0.79 | -0.70 | 0.23 |
| 0, 20 | 96 | +1.88 | +0.78 | -0.65 | 0.24 |
| 0, 30 | 50 | +1.98 | +0.73 | -0.71 | 0.27 |
| 0, 40 | 17 | +1.95 | +0.86 | -0.50 | 0.23 |
| +30, 0 | 153 | +1.88 | +0.82 | -1.17 | 0.22 |
| +30, 20 | 147 | +1.94 | +0.77 | -0.92 | 0.29 |
| +30, 40 | 30 | +1.96 | +0.82 | -0.87 | 0.20 |
| +45, 10 | 88 | +1.91 | +0.53 | -1.09 | 0.30 |
| +45, 30 | 72 | +1.95 | +0.76 | -0.69 | 0.22 |

The same bench on the chassis bracket moved by 4.5 degrees of yaw with the
gimbal. Its first run here, with the lens as published and nothing free but the
rotation, seemed to show the same thing -- yaw -0.2 at pan 0 against +4.6 at
both ±30 -- and the overlaid pictures showed why: a scale mismatch, with the chair
slats doubled at the sides and aligned in the middle.

**The mount's yaw is relative to whichever fisheye model defines the camera's
axis.** Against the board-fitted fisheye of 2026-09-07 the first run's points give
a yaw of -0.22 rather than +1.89, because that model's principal point is ten
pixels from the swept model's. The rover draws bearings through the swept model,
so the swept model's figure is the right one for it now; refitting the fisheye
must re-derive the yaw from the same points in the same change, which the points
files kept under `captures/2026-09-30-oak-rail/` allow.

## The OAK's lens was 9.6% short

The depth service asked depthai for the colour lens at 640 x 360 with
`getCameraIntrinsics(socket, 640, 360)`, which scales the stored full-sensor
calibration by the width. The 1080p mode the service runs reads only the middle
3840 x 2160 of the 4208 x 3120 sensor, so the published focal length was 456.5
pixels where the pixels obey 500.3, and the published field of view 70.1 degrees
where the picture takes in 65.2.

Measured with the service stopped, matching the service's own colour mode against
the right mono camera, whose native 640 x 480 leaves no mode to get wrong:

| colour lens | fx | fy | median miss | matches held |
|---|---:|---:|---:|---:|
| as depthai returned it | 456.5 | 456.4 | 0.19 deg | 27 of 88 |
| fitted, stored distortion applied | 499.6 | 497.4 | 0.11 deg | 88 of 88 |
| the 3840 x 2160 window, which is what is now published | 500.3 | 500.2 | | |

The fitted focal length is 1.094 times depthai's; 4208 over 3840 is 1.096. The
device's own depth-to-colour alignment was never affected: depth edges sit on the
chair backs, legs and door frames to the corners of the picture.

**This is what the 2026-09-07 mount measurement could not account for.** Its
forward offset read 87.2 mm at a board 0.555 m out and 99.6 mm at 0.685 m, a
systematic 12.4 mm. A focal length 9.6% short puts the board 9.6% too close to
the OAK; undone, the two distances give 42 and 44 mm, and the tape had said 40.
The chassis mount is kept as `oak.CHASSIS_MOUNT` for reading depth maps recorded
on it, with that corrected 43 mm.

The same error is in the bench capture `oak_camera/capture_rgb.py`, worse at its
smaller sizes because they are cut from the middle of the 1080p picture rather
than scaled from it: 39% short at 1280 x 720. Both now use
`oak_depth/colour_lens.py`.

What the error cost while it stood: a gimbal box mapped onto the OAK's picture
landed up to three degrees off at its edges, and a range there was stretched by
the wrong secant, about 3%.

## The fisheye's lens model is 7% short

The rigid fit only closes once the angles off the fisheye's axis from
`face_tracking/lens.py` are stretched by 7.2%, so that model records a thing 30
degrees from the middle of the picture at 28. Two things say this is the lens and
not the mount or the OAK:

- the first run's points through the board-fitted fisheye of 2026-09-07 -- 1280 x
  960, halved -- need a stretch of 1.1% where the swept model needs 7.2, with the
  same 0.23-degree fit;
- that board model was fitted with no servo in the loop, and it and the swept model
  differ by 5 to 7% over the middle 40 degrees of the picture.

The swept model was fitted in 2026-08 by turning the gimbal and trusting the
angle it was told. The OAK's IMU puts the pan servo's 30-degree steps at 28.6 to
30.6 degrees, so this is not simply the servo over- or under-travelling.

**Not fixed here.** The lens is what face tracking aims through and what every
bearing is drawn through; refitting it changes every bearing recorded afterwards,
leaves the store holding bearings through two models, and moves the mount's yaw.
Until it is refitted, a fisheye box mapped onto the OAK's picture is off by about
7% of its distance from the middle -- nothing at the centre, two degrees at the
OAK's edge -- and a bearing by the same.

## What changed in software

- `world_state/oak.py`: the mount is written in the gimbal camera's own frame and
  turns with the pan and tilt of each look. An OAK look records the gimbal's pan
  and tilt, and `ray_at` takes the whole mount rotation out of each pixel; the
  earlier model, which stored the mount's yaw and pitch as the pan and tilt and
  took out only the roll, would have this mount's 1.92 degrees of yaw come out at
  1.92 at every tilt where it is 2.05 at tilt 20 and 2.54 at tilt 40, because the
  mount turns about the camera's own vertical, which leans back with the tilt.
  `box_for` and `range_from_gimbal` work in the camera's frame, so the servos'
  errors cannot move a box. `pose_at` and `rise_of` turn the offset with the
  look. `mount_at` picks the chassis mount for anything recorded before the move.
- `world_state/inspection_ranges.py`, `inspector.py`, `resolve.py`,
  `negative_evidence.py`, `incremental.py`, and the daemon's `rover_world.py`
  follow; the daemon's OAK capture reads the pan, tilt and approach under the same
  lock as the fisheye's.
- `oak_depth/colour_lens.py` and the depth service's `/health`; `test_colour_lens.py`.
- `face_tracking/aiming.py`: the argument that the OAK's view bounded the rest tilt
  is withdrawn. The rest tilt stays at 20.

world_state 851 passed (831 before, the difference being the new checks),
rover_daemon 978 passed, `oak_depth/test_colour_lens.py` 13 passed, and the
incremental resolver's 9 unit tests pass.

## On the rover

Deployed with `deploy.py --only oak_depth --only world_state --only rover_daemon`
at c4c2b67, the daemon again at 6d19091, and the adopted mount at the commit that
records this entry. On the Orin: world_state 851 passed and rover_daemon 978
passed with the OAK switched on. The depth service's `/health` reports a colour
lens of 500.3 by 500.17 and a field of 65.2 by 39.6 degrees.

**The daemon's suite had depended on a switch.** Its first run on the rover failed
one check, a test world keeping three pictures where it expected two, and passed
with the OAK switched off: the tests' fake world used fake eyes but reached the
real depth service, so on the rover they measured real ranges and kept a depth map
whenever the OAK happened to be on. A fake world now gets a fake depth camera.

What proved the running system, over TCP 8769 and the depth service:

- a grid of fifteen boxes on the fisheye's picture, mapped by the deployed code
  onto the OAK's, landed on the same things -- ceiling lamps, a pendant, a wall
  clock, a small picture, the kitchen doorway -- at pan 0 tilt 20, pan +30 tilt 20,
  pan -30 tilt 0 and pan +60 tilt 30, and all fifteen were ranged at every one.
  On the chassis bracket nothing at pan +60 was in the OAK's picture at all;
- the daemon's own looks, taken as the gimbal moved, were stored with 4 to 9 of
  their 7 to 12 regions ranged at pan -30, +30, +45 and +60.

**No look today has a bearing**, before this change or after, because navigation
has not confirmed where the rover is since it booted, and the capture gate
withholds a direction until it does. So placing a thing through the new geometry
end to end is still to be shown, on the first drive.

## What the rover had already recorded

The rover stored 64 looks between the boot and the moment the new geometry and
lens were both live (15:56:09), 246 of their 569 regions with a range attached
through the chassis geometry and the short lens. None had a bearing, so none
placed anything, and they are left as recorded rather than rewritten: they are
identifiable by time, and a range on a look with no direction is read by nothing.

## Still owed

- **The ends of the gimbal's travel with the OAK on it**: pan beyond ±90 to the
  ±180 the face search reaches, and tilt beyond +60 to +90 -- cable slack and the
  clearance of the back of the mount, which the owner can see and this could not.
- A board measurement of the offset on the rail, which is the check on the one
  centimetre the two runs disagree by.
- Refitting the fisheye's lens model, with the mount's yaw re-derived from the
  points kept under `captures/2026-09-30-oak-rail/` (R-WS-10).
