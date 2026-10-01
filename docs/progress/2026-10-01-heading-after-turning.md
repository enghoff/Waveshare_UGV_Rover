# Turning on the spot puts the heading tens of degrees out, and a refit takes it back to the tape

**The rover's belief about which way it faces is wrong after it turns on the spot, by
up to 43 degrees, and a scan-to-map refit puts it back to within 2 degrees of the
truth.** The truth here is the owner's tape, not the rover's own fit. That makes the
heading error that dominated
[the morning's drive](2026-10-01-the-room-as-it-stands.md) something software can
work around today, whatever its root cause turns out to be.
[R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing` until a drive shows
it, but the largest term in it now has a remedy measured on the rover.

## How it was measured

The rover stood at its second parking spot, confirmed by a refit, and only turned:
`turn_in_place` in sequences of quarter, half and full turns, three seconds' settle,
then a picture and the believed pose. The reference is the morning's two-wall frame.
The tissue box, the toolbox and the pink bucket were found in each picture by colour,
not by the rover's own perception. Each was turned into a direction through the
deployed lens model at the gimbal's tilt, then compared with the direction to its
taped position. Every detection was checked by eye on the marked pictures. A pink
blob seen while facing the living room was a person's shirt and is excluded, as
were early "bucket" hits that turned out to be the toolbox lid. Pictures, poses and
scripts are in `captures/2026-10-01-heading/`.

The first sequence is not used: the rover was riding over its own charging cable.

## What the heading does

From a start 1.4 degrees out, on clear floor (`turn2`):

| after | believed minus true |
|---|---:|
| three quarter turns anticlockwise | +23.2 deg |
| a further full turn anticlockwise and some | +41.2 deg |
| then two quarter turns clockwise | +42.7 deg |
| three more clockwise | +28.9 deg |
| three more clockwise | +5.7 deg |
| four ±30 and ±60 turns | +0.6 to +2.2 deg |

In truth the rover turned 313 degrees during the first three quarter turns. The
wheels and gyro said 276 and the map said 335. **Both are wrong, and in opposite
directions.** The map over-counts each turn by about 7%, in both directions, so the
error builds up over turns one way and unwinds over turns the other way. The rover's
drift check, which happened to run mid-sequence, found the same thing on its own: its
full search put the scan 47.5 degrees from where the rover believed it stood.

## A refit takes it back

The third sequence turned full circles and, after each, measured, refitted
(`refit_pose`, one metre and a full circle of search), and measured again (`turn3`).
Every one of the seven refits found the rover 8 to 24 degrees out and moved it onto
the map, with 96 to 97% of the scan on a wall against 34 to 47% before. Where a target
was in view to check against the tape:

| | before the refit | after |
|---|---:|---:|
| two full turns clockwise | -22.2 deg | +1.7 deg |
| four quarter turns clockwise | -22.7 deg | +0.9 deg |
| two half turns anticlockwise | +24.0 deg | +1.6 deg |

The 1 to 2 degrees left over is the size of what the lens, the pan servo's backlash
and the frame contribute anyway. The stationary start measured 0.2 and 1.4 degrees
on the same targets.

One detail of the refit's own report should not be trusted. The angle it says it
moved the rover by did not always match the change in pose. After the last circle it
reported -7.9 degrees while the pose moved -21.7, and the tape agrees with the pose.

## Why, which is not settled

The map is set to fold in a scan after every 0.2 rad of turning
(`config/slam_toolbox.yaml`), so it is watching these turns and getting them wrong
anyway. One candidate is scans distorted by the rover's own rotation, matched against
the mapper's recent and already-rotated scans rather than the whole map. Another is
the gyro, whose scale was calibrated on 2026-08-23 against the map's heading, with
samples that scatter by about ±10% from turn to turn. Turning at a different speed
would separate the two, because skew grows with turn rate and a scale error does
not. Neither has been tried.

## What it means for P0

A look should only get a direction from a heading checked against the map since the
rover last turned. That is the morning's capture gate
([R-WS-16](../requirements/world-state.md#r-ws-16)) extended from "the position is
confirmed" to "the heading is confirmed". It can be met two ways. The rover could
refit its own heading after each turn, inside a narrow window it cannot jump rooms
from, refusing when the room fits two ways. Or capture could withhold directions until
something has checked. The first keeps the looks and the second discards most of them,
because the rover's looking loop runs straight after every turn. The first also
reverses a choice made in code on 2026-09-07, that only a person refits. That is a
decision for the owner, not a measurement.

## Requirements

None moved. [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`, with
its largest term measured and a remedy shown to work against the tape.
