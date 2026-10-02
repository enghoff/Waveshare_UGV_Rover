# The redo drive, the carry and the stop trials: most of it holds, heights and one painting do not, and M0a never ran

**A carried rover no longer records directions from where it was, the stops halt it within
0.31 s and 0.22 m, and the drive's bearings and ranges are much better than yesterday's.
The drive still fails P0's geometry tolerances, on heights and on the painting behind the
dining chairs, and none of M0a's three supervised runs took place:** the battery ran down,
then the rover dropped off the network. [R-WS-16](../requirements/world-state.md#r-ws-16)
moves to `settled`; [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`,
[R-WS-11](../requirements/world-state.md#r-ws-11) and
[R-AUT-12](../requirements/autonomy.md#r-aut-12) stay `open`.

Everything was frozen before the rover moved, in `captures/m0-2026-10-02/MANIFEST.txt` and
`captures/m3-stop-2026-10-02/MANIFEST.txt`, which also record every deviation. Claude drove
over the daemon's tool protocol with the owner in the room. The six targets and the two-wall
frame are 2026-10-01's, re-fixed by two parked lidar readings today. Those agree with the
rover's own position to 3.6 cm, and the 2026-10-01 frame predicted both readings to within
9 cm.

## The rover did not look once it had stopped

The first drive came back with one look in 75 taken standing still. A look was due after
0.15 m of travel or 25 degrees of turn since the last one, so the last look of every turn
was taken mid-turn. The rover then stood facing its target for six seconds without
looking. A look on the move keeps its bearings only while a recent still look's heading
check vouches for them, so only 106 of 653 regions got a direction. Deployed as `ce55023`
(rover_daemon 1028 on the Orin): a rover that has moved at all since its last look looks
again once it has stood still for 0.6 s. On the rover, a 10-degree pan held still gave one
look 1.6 s later and none after it. The redo drive, on the same plan, took a still look at
21 of its 27 facings.

Turning on the spot carries on 15-30 degrees past the command, so each facing was
corrected from where one scan said the rover faced (`refit_pose`, measuring only). After
that, every target sat within 13 degrees of the middle of the picture.

## The redo drive against the frozen tolerances

| Tolerance | Bar | Result | |
|---|---|---|---|
| each target where the tape puts it | within 0.35 m | 4 of 6: bucket 0.10, cabinet 0.10, painting above it 0.25, toolbox 0.12 | **fail** |
| separations between targets | all within 0.30 m | 6 of 10, every miss involving the landscape painting | **fail** |
| heights | within 0.20 m | 0 of 3: +0.51, -0.36, +0.27 m | **fail** |
| ranges over still looks | 70% within 0.5 m | 5 of 6, 83% | pass, on a small sample |
| bearings over still looks | 90% within 3 deg | 8 of 10, 80%; median 1.2 deg, worst 3.7 | **fail**, narrowly |

On 2026-10-01 the bearings' median was 4.0 degrees; today it was 1.2 over still looks and
2.0 over every look with a bearing. Two targets carry all of the remaining position
failures:

- **The landscape painting** stands behind the dining chairs. Its thing was placed 0.88 m
  off, and its one wrong range read the chair in front (1.22 m against 2.54).
- **The tissue box** on the floor never became a thing of its own. It appears only inside
  two mixed things: one with the rug and the bucket, one with the table and chairs.

Raised targets also come out too high: the painting by half a metre and the toolbox by a
quarter. That is R-WS-11's open elevation question, not a new fault.

## The carry

At the end of the drive the owner lifted the rover and set it down about 2 m away, facing
elsewhere. Navigation went on believing the old position, trusted and settled. The look
taken there kept its picture and gave none of its eleven regions a direction: "the scan
does not fit the map well enough anywhere near here: the best of 3751 poses put 61% of it
on a wall and a fit needs 90%". A `refit_pose` with a wide search found the rover 1.94 m and
71.5 degrees from where it believed it was, and the owner's tape agreed. The next look was
checked to 0.0 degrees and 3 cm, and got its directions back.

One gap remains: the rover's own loop never looked after the carry. It looks when the
believed position moves, and a carried rover's believed position does not move. So its
first look would have come up to five minutes later, and it would have been withheld in
the same way.

## The stop trials (M3, as M0a's entry condition)

The owner opened each run, or told Claude to open it. A stand-in for the executive drove
legs through the daemon's own autonomy path, inside a geofence.

| Trial | What stopped it | Result |
|---|---|---|
| S1 | `stop_driving` from Claude at 0.31 m/s | at rest in 0.18 s and 0.11 m; latched; next action refused; the executive would not restart |
| S2 | the console's stop button, pressed by the owner | latched; but the rover was turning at 0.11 m/s (at rest in 0.09 s, 5 mm). The repeat at speed failed on the battery. The button sends the same `stop_driving` S1 measured |
| S3 | a manual `drive_to` mid-leg, as the console sends it, at 0.31 m/s | latched as a takeover; at rest in 0.31 s and 0.22 m, of which about 0.14 m is a map correction |
| S4 | the stand-in killed with SIGKILL | the run closed 15.3 s after its last renewal, against a 15 s permit; Nav2 stayed up; the old run was refused; the executive would not restart |

Every stop met the frozen limits of 1.0 s and 0.30 m. S4 never had a moving rover to stop:
the leg in flight arrived before the permit ran out, and so did a repeat with a 3 s permit,
by 0.3 s. A run closing on its failure count did stop a moving leg within 0.08 m.

Two hardware limits ended the session:

- **The battery.** At 30% the pack sagged below the 11.2 V that every autonomous run keeps in
  reserve, even on the charger. At that charge the rover also could not finish turning
  round at the end of a leg: it oscillated for 55 s.
- **The network.** During a last attempt at S4 the rover dropped off it.

## What is left for P0

- **M0a's three supervised runs**: at least 20 attempts across 10 places, with three
  absent-target and three occluded cases. None has been run; the path is built and has
  been exercised by hand once
  ([the rover check](2026-10-01-hypothesis-check-on-the-rover.md)).
- **Permission expiry stopping a moving rover**: a leg that is still driving when the
  permit runs out. That needs a short permit and a kill within half a second of the leg
  starting.
- **The geometry prerequisites**: heights (R-WS-11), the painting behind the chairs, and a
  floor object too small to stand out on its own. Under the owner's direction that P0
  works around the hardware, the next step is to declare the envelope these failures
  imply: targets not behind other things, and height not relied on. That is not to
  re-score against looser numbers.

## Requirements

- [R-WS-16](../requirements/world-state.md#r-ws-16) to `settled`: a carried rover's look was
  withheld while navigation still claimed a confirmed position, and given directions again
  once the scan placed it.
- [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`: 80% within 3 degrees
  against 90%, on ten still looks.
- [R-WS-11](../requirements/world-state.md#r-ws-11) stays `open`: heights 0 of 3.
- [R-AUT-12](../requirements/autonomy.md#r-aut-12) stays `open`: no supervised M0a run.
- [R-SAFE-5](../requirements/safety.md#r-safe-5), [R-SAFE-11](../requirements/safety.md#r-safe-11)
  and [R-SAFE-12](../requirements/safety.md#r-safe-12) were exercised on hardware as above;
  their states are unchanged.
