# The gimbal reaches the ends of its travel with the OAK on it

**Nothing needs limiting.** With the OAK riding the gimbal's rail, the gimbal
went to tilt +90 and to pan +180 and -180 at the tilt the face search uses, and
arrived everywhere it was sent with the OAK's USB link unbroken. The cable has the
slack for the whole sweep and the back of the mount clears the pan base looking
straight up. This closes the one hardware item
[the rail entry](2026-09-30-oak-on-the-gimbal.md) left owed; no requirement moves.

## How it was measured

The OAK's own IMU was read throughout while the daemon stepped the gimbal, so
every step was measured by something the servos do not control: tilt by how far
gravity turned from level, pan by the gyro integrated about gravity. Each step was
checked before the next was taken, and a step that came up more than 4 degrees
short -- what a snagged cable or a mount hitting the base would do -- or an IMU
stream that stopped would have walked the gimbal back and ended that leg. None did.

Steps of 15 degrees beyond the ±90 already tested, at the servos' own speed, and
back in steps of 30.

| tilt, pan 0 | 30 | 60 | 70 | 80 | 90 |
|---|---:|---:|---:|---:|---:|
| gravity turned from level, degrees | 30.74 | 61.48 | 71.77 | 81.95 | 91.98 |

| pan, tilt 45 | 105 | 120 | 135 | 150 | 165 | 180 |
|---|---:|---:|---:|---:|---:|---:|
| 15-degree step, measured | 15.34 | 15.53 | 15.23 | 15.45 | 15.26 | 15.09 |

| pan, tilt 45 | -105 | -120 | -135 | -150 | -165 | -180 |
|---|---:|---:|---:|---:|---:|---:|
| 15-degree step, measured | 15.51 | 15.35 | 15.09 | 15.26 | 15.09 | 13.96 |

The last step to -180 came up a degree short, which is the servo at the end of
its travel rather than anything pulling on it: the step back from there measured
28.9 for 30, the same backlash the first step back from +180 showed (29.35).
The tilt servo runs about 2% over what it is told at every angle, the same as
the earlier sweep found.

The kernel logged no USB disconnect while the gimbal moved. The ones either side
of the sweep are the depth service letting go of the camera, which re-enumerates
every time it is switched off.

Taken on the rover on 2026-09-30, 16:41 to 16:44, with the depth service stopped
so the bench could hold the device; the service was restarted afterwards.
