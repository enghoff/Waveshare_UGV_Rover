# Bearings are measured, not held to an accuracy

Status: agreed 2026-10-03 by the owner. Retires
[R-WS-10](../requirements/world-state.md#r-ws-10).

There is no longer a requirement that a recorded bearing be as accurate as the
resolver is told to expect. How accurate the rover's bearings are is measured against
a tape and documented in [world_state/README.md](../../world_state/README.md) under
*What to expect from it*. Each look claims what looks like it have been measured to
be worth, `stated_bearing_sigma_deg`. That claim sits beside the width the resolver
matches with, which stays at the 1.5 degree calibration.

## Why

R-WS-10 asked for bearings within the resolver's 1.5 degrees, or for the measured
error to be represented honestly within a useful envelope without widening the match
tolerance. After a month of work it still failed on the drive of 2026-10-03
([the drive](../progress/2026-10-03-moving-looks-against-the-tape.md)). Still looks were
1.0 to 1.5 degrees off and moving looks 3.2. Moving looks are eight looks in nine.

What remains has three sources: motion, the position of a thing in the fisheye
picture, and the gimbal's 1.5 degrees of backlash. None of them is a software constant
waiting to be found. The heading check on the move took moving looks from 3.8 to 2.8
degrees, and the lens refit, the mount measurement, the elevation bias and the heading
checks have all landed already. The owner judged that this camera and gimbal will not
reach 1.5 degrees on the move, and that a requirement nobody expects to satisfy
measures nothing.

## What was considered

- **Widen the resolver's width to the measured error.** That was rejected on
  2026-10-02. A 2.2 degree bearing sigma took merges of different objects from 8 to 13
  on the labelled drive
  ([the measurement](../progress/2026-10-02-what-to-expect-from-the-hardware.md)).
- **Keep only still looks' bearings.** That would throw away eight looks in nine. On
  2026-10-02, withholding unvouched looks left a drive with 17 of 906 regions directed
  ([the drive](../progress/2026-10-02-heading-check-withheld-the-drive.md)).
- **Keep tuning.** Each remaining term is a fraction of a degree to two degrees, and
  each fix so far has been worth less than the one before.

## What stays required

Honesty, the half of R-WS-10 that is achievable. A look's claim is now separate from
the matching width, as a placement's has been since 2026-10-02. It is set from what
looks of its kind measured: 3.0 degrees from a standstill and 5.0 on the move. A
placement from two or more viewpoints already claims what it delivers, and all six on
2026-10-03 lay within twice their claim. The claim is checked again on every taped
drive.

## What would reopen it

A change to the camera, the gimbal or the way looks are timed that is expected to bring
moving looks near the calibration. Or a decision that needs bearings tighter than the
claim allows, which would then have to say how tight and why.
