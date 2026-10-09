# The gimbal aims across its measured travel

Status: agreed 2026-10-09. The owner left the call to the agent that measured it.
Supports [bearings-are-measured-not-required.md](bearings-are-measured-not-required.md).

A look taken with the camera panned anywhere within ±150 degrees keeps its
direction. Within ±20 it claims what it always did. Between 20 and 150 it claims
0.7 degrees more, in quadrature. Past 150 the direction is still withheld. The
values are `DEMONSTRATED_PAN_DEG`, `BOARD_PAN_DEG` and `WIDE_PAN_SIGMA_DEG` in
`world_state/inspector.py`, and an aimed look pans as far as the first of them
allows.

## Why

**The gimbal goes where it is told.** On 2026-10-09 the OAK's gyro, which rides
the gimbal, measured every ascending arrival from -170 to +180 within 0.6
degrees of the commanded pan. Two sessions in two rooms agreed with each other
to 0.38 degrees, and the gain was 0.9985
([the measurement](../progress/2026-10-09-gimbal-whole-pan-travel.md)). Inside
±20 that is what the board calibration found.

**Turning the rover to face a thing costs more than it buys.** A turn on the
spot is about as quick as a pan of the same size. But it arrives anywhere within
15 degrees of the heading asked for, and leaves the map's heading 9 to 16
degrees out for seconds afterwards. A pan leaves the chassis, the lidar and the
pose where they were. The owner's question was whether the gimbal was accurate
enough to be preferred over the turn. At half a degree against more than ten, it
is.

**±150 because of what the camera sees, not how well it points.** Past 150 the
rover's own antenna comes into the picture, and by 170 it covers a quarter of
it. A look there would mostly record the rover.

**0.7 degrees because that is the doubt the measurement left.** The worst
ascending miss inside ±150 in either session was 0.59 degrees. The gyro
disagreed with the camera's own pictures by up to 0.34. Together that is 0.68.

## What was considered

- **Leave the envelope at ±20 until the sweep's own rule passes.** That rule was
  fixed before the sweep. It asked the gyro and the pictures to agree on the
  backlash to 0.25 degrees, and they agreed to 0.34, so its verdict was
  inconclusive. Holding to it would keep every wide look directionless over a
  tenth of a degree of doubt, against heading errors of seven or more. The rule
  decides what the measurement proves. The envelope adopted carries the doubt
  rather than ignoring it.
- **A third session first.** Its reference would be the same gyro, which drifts
  by about a quarter of a degree over a session. It would most likely end where
  the second did, and cost rover time to say so.
- **Correct each pan by the measured table.** The table is in
  `usb_cameras/pan_candidate_2026-10-09.json`. It would buy a few tenths of a
  degree, and every user of the pan (bearings, face tracking, the depth
  camera's pose) would then need the same correction. Not worth it while a
  look's error is dominated by heading.

## What stays

A pan reached from above still claims the backlash on top (2.3 degrees). The
daemon's every aimed move ends with a step up, and since 583756b a long swing is
waited out before that step, so a wide pan is an ascending arrival.

## What would reopen it

A change to the gimbal, the antenna or the mounting. Or a measurement on the
rover, against a tape or a board, that finds wide-pan looks missing by more than
their claim.
