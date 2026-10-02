# The heading check withheld almost every look on a drive, and looks it cannot vouch for take the navigator's heading again

**The heading check of 2026-10-01 was right whenever it ran, and wrong to withhold when it
could not.** It only runs while no move is in progress, and it withheld the direction of
every look it could not vouch for. So on a driven run almost nothing got a direction, and
almost nothing was placed. Looks it cannot vouch for now take the heading the navigator
believes, as every look did before 2026-10-01. Deployed as `a6a9a66` (world_state and
rover_daemon on the Orin, both suites passing there). On a 17-second drive afterwards, 73
of 84 regions got a direction, and the store went from 5 things to 12.
[R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing` and
[R-WS-16](../requirements/world-state.md#r-ws-16) stays `settled`.

## What the check cost

Regions given a direction, from the recordings in `captures/` and today's run on the rover:

| Run | Heading check | Regions with a direction | Things |
|---|---|---|---|
| 2026-09-08 M0 | none | 1229 of 1328, 92% | 104 |
| 2026-10-01 acceptance | none | 619 of 630, 98% | 71 |
| 2026-10-02 acceptance | withholding | 106 of 653, 16% | 20 |
| 2026-10-02 redo | withholding | 134 of 588, 22% | 27 |
| 2026-10-02 M0a start | withholding | 80 of 458, 17% | 14 |
| 2026-10-02, the owner's run after a map clear | withholding | 17 of 906, 2% | 5 |

The last is a six-minute driven run. Of its 216 looks, 171 were taken on the move with no
fresh check, and were withheld for it. Only two looks were taken standing still between
moves and checked, and those two gave all 17 directions and all five things. The redo
drives did better only because they stopped to face each target.

## The corrections were right; the two bad placements came from depth

The owner reported two of the five placements as significantly off. Each placement was
compared with the lidar map, on the check's corrected heading and on the navigator's:

| Thing | Range | Corrected heading | Navigator's heading |
|---|---|---|---|
| painting in the hallway | 1.48 m | on its wall | 7 cm off the wall, in open floor |
| bedroom wardrobe | 1.81 m | 7 cm from the wall | 18 cm from the wall |
| bed | 1.41 m | on the bed's lidar outline | on it too |
| dark wardrobe with a mirror front | 2.50 m | 0.56 m beyond the wall | 0.42 m beyond it |
| small patch at the far end of the corridor | 4.71 m | 0.76 m beyond a wall | 0.74 m beyond it |

The two that are off are off on either heading. Both errors come from the depth reading:
a mirror, which reads the depth of the reflection, and a patch a few pixels across at
4.7 m. That is not addressed here. A range that puts a thing behind a mapped wall is a
contradiction the occupancy grid could catch, and nothing does yet.

## What changed

A look takes a fresh check's correction whenever there is one: its own, if it was still
and the check ran, or the last check's while that is fresh (under 15 degrees of turning
since, and under half a metre of travel if it corrected). Otherwise it takes the
navigator's heading. That covers a moving look with no fresh check, a still look whose
check was refused mid-move, and a check that never answered. A pose stored without
`checked` is the navigator's own.

Only a search that ran and fitted nowhere withholds, and from then on every look is
withheld until a search fits again. That is the carried rover of
[R-WS-16](../requirements/world-state.md#r-ws-16), unchanged. A search always reports a
score, and a refusal never does. That is how the two are told apart.

The fault was reproduced offline first. A modelled run in which every check is refused
mid-move gave 0 of 20 looks a direction on the deployed code and 20 of 20 after the
change. The carried-rover cases pass on both.

## On the rover

The owner was not asked first, so the drive was kept short: `drive_to` 1.5 m into open
floor and back, 17 s. Nine looks gave 73 of 84
regions a direction. One look gave none, because the rover moved 0.58 m during its
exposure, a rule that predates the check. The still look at the end was checked against
the map and found the navigator's heading right, to 1.0 degree and 4.5 cm.

## What this does not show

- **The accuracy of moving looks on this build.** They have what they had before
  2026-10-01: against the tape that day, looks taken while turning missed by a median
  4.7 degrees, against 2.6 for still ones. Looks taken mid-turn are back with that
  error. Correcting moving looks properly needs the check to run during a move, matching
  each scan to the pose at that scan's own time. That has not been built.
- **A failed check near a wall.** On 2026-10-01 four of six checks beside the cabinet
  scored 88-90% against the 90% a fit needs. Each such failure now withholds every look
  until the next still check fits.
- **Ranges.** Today's run dropped 347 regions' ranges because the rover was turning, under
  `b1843c9`. That rule is untouched here.
- **A full run.** The 17-second drive shows directions are back. It does not show that the
  owner's missing objects are placed where they stand.

## Requirements

None moved. [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`.
[R-WS-16](../requirements/world-state.md#r-ws-16) stays `settled`: the withholding it
rests on is unchanged and passes offline, and no rover has been carried since.
