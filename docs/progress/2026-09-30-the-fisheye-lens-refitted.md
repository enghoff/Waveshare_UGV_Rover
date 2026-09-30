# The fisheye lens was 7% short, and is refitted without trusting a servo

**The gimbal camera's lens model is replaced, and everything the rover had
recorded through the old one is redrawn.** The model it flew since 2026-08-19
recorded a thing 21 degrees from the middle of the picture at 19.8 and one 41
degrees out at 39.9. The new one is fitted to a printed board, to the OAK riding
the same platform and to still pictures of a stepped sweep, none of which asks a
servo where it is, and it passes a check it was not fitted to: tilt steps whose
true size the OAK's accelerometer read off gravity come out at 0.998 of what it
says, where the old lens said 0.962. [R-WS-10](../requirements/world-state.md#r-ws-10)
stays `failing` -- what lifts it is a driven recording -- but the largest term in
it is gone.

## Which camera was wrong

Three measurements said the fisheye was short and one said it was right, so this
had to be settled before anything was changed:

- **the OAK**: one rigid mount fitted fifteen gimbal positions only once the
  fisheye's angles were stretched by 7.2% ([the rail entry](2026-09-30-oak-on-the-gimbal.md));
- **the board calibration of 2026-09-07**, which trusted no servo, differed from
  the flown lens by 5 to 7% across the middle of the picture;
- **gravity**: on tilt steps the OAK's accelerometer measured, the old lens needed
  its angles stretched by 3.9% to match;
- but **in August**, 15 degrees of commanded pan moved the picture 78 pixels,
  which is what the old lens says 15.3 degrees is -- and the servo turns 2 to 4%
  more than it is told.

The board comparison was only admissible once it was known that the 640x480
picture the rover captures is the 1280x960 one the board was photographed in,
halved. Grabbed a moment apart at one pose and matched, the two map onto each
other at a scale of 2.001 about the same centre, twice. So all three
servo-independent measurements describe the camera as it is today, and the August
figure stays unexplained.

## The fit

`face_tracking/lens.py`'s own form -- an angular scale, distortion terms in the
normalised radius, a centre -- fitted to three kinds of evidence at once, each
with its own nuisance parameters and none with a servo angle:

| evidence | what it constrains | points | median residual |
|---|---|---:|---:|
| 161 frames of the printed ChArUco board, 2026-09-07, each its own pose | centre, inner shape | 8061 corners | 0.08 px |
| matches with the OAK through its factory lens, stored distortion and stereo ranges, and the mount | absolute scale | 1433 | 1.07 px (0.22 deg) |
| 27 still-picture pairs of a stepped tilt and pan sweep, distant features only, each pair its own rotation | shape to the edges | 9108 tracks | 0.21 px |

The board alone cannot pin the scale -- a planar target seen nearly face on
trades its focal length against its distance, and one and two distortion terms
disagreed by 1% -- and the sweep alone cannot either; the OAK can, because its
matches carry measured distances. A third distortion term moved no angle by more
than 0.3 degrees at the corners.

| off the axis | 100 px | 150 px | 200 px | 250 px | 300 px | 320 px (side edge) | 400 px (corner) |
|---|---:|---:|---:|---:|---:|---:|---:|
| old lens, degrees | 19.76 | 29.74 | 39.86 | 50.15 | 60.66 | 64.93 | 82.49 |
| new lens, degrees | 20.95 | 31.27 | 41.51 | 51.84 | 62.57 | 67.08 | 87.56 |

The new lens is 12.637 arcmin per pixel, terms -0.0578 and +0.0531, about
(323.3, 225.6): 134.2 by 99.5 degrees of room, its axis three pixels right of the
middle of the frame and fourteen above. Tracks in the far corners fit worst,
0.5 px, which is where the least data is.

**Checked against what it was not fitted to**: eleven 5-degree tilt steps whose
size the OAK's accelerometer read off gravity, a measurement with no scale in it
to be wrong. The new lens needs 0.998 of its angles to match them (interquartile
0.997 to 1.004); the old one needed 1.039. And the step sizes the fit found for
itself, with each pair's rotation free, match what gravity said step for step:
a median of 5.18 degrees for 5 commanded.

What else fell out of the same data:

- **The servos run slightly over.** Pan 2.4% on 5-degree steps, tilt 3.6%, both
  ascending. Neither enters a bearing at pan 0, which is where every stored look
  was taken.
- **The OAK's gyro is not to be trusted for angles without calibrating it.** It
  read 3.8% low about the tilt axis against gravity, and about right about the pan
  axis against the lens -- sensitivity that depends on the axis.
- **The OAK mount moves with the lens**, because its yaw is measured against the
  fisheye's axis, which moved ten pixels. Re-derived from the same points through
  the new lens: yaw +0.37, pitch +0.44, roll -0.75 degrees, 45 mm up and 4 mm to
  the right. The fisheye scale that fit reports is now 1.012, and that last 1% is
  the OAK drawn as a pinhole: its own distortion puts its edges about 1% further
  out.
- **The chassis mount, kept for reading old depth maps, is re-expressed against
  the new lens**: it was measured against the board calibration's own lens, whose
  axis is 0.6 degrees across and 0.5 up from the new one. Yaw +2.10, pitch +5.78,
  roll -1.19 where it was measured as +1.49, +6.26 and -1.20.

## What changed in software

- `face_tracking/lens.py` carries the new lens and two distortion terms, with
  `radius_of` to read it backwards. The half frames it quotes are 67 and 50
  degrees, and a test holds them to the lens. `usb_cameras/calibrate_aim.py`
  reads the lens from there rather than keeping a copy of the old one, and
  `calibrate_fov.py` understands more than one term.
- **The aiming test had one case the deadband explains.** A face on the middle
  row at the edge of the frame gets its pan and not its tilt, because what is
  left vertically is under the two-degree deadband; with the new lens that
  half-degree remainder crossed the test's 0.5-degree bound. The test now checks
  `solve()` with the deadband off, which is what it was about, and lands every
  case within 0.01 degrees.
- `world_state/oak.py`: both mounts as above; `box_for` and `range_from_gimbal`
  take the mount a look was taken on.
- `world_state/relens.py`, the migration below, and its tests.

world_state 863 passed, rover_daemon 983, `calibrate_fov.py --selftest` and
`oak_depth/test_colour_lens.py` pass.

## The stored looks

Every look with a direction -- 2282, all from the drive of 2026-09-08, all at pan
0 -- reproduced exactly through the old lens from the pose, tilt and box stored
with it, so all were redrawn. Every range in a look whose depth map was kept was
replayed from that map through the geometry of the day before being redone.
Recorded when the migration was run on the rover; see below.
