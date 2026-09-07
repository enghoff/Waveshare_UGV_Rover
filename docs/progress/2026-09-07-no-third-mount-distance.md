# The A4 target cannot give the mount a third distance

Four mount captures across the day bracket the problem, and the conclusion is
about the printed sheet rather than the rover. **Both cameras only see the board
well enough between about 0.55 and 0.69 m, and the acceptance rule forbids
everything from 0.455 to 0.786 m.** The band that works is entirely inside the
band that is excluded, so the outstanding third-distance confirmation cannot be
obtained by moving anything. It needs a larger printed target.

The adopted mount is unaffected: it passed its declared gates on the development
set at 0.555 m with the held-out set at 0.686 m as confirmation, and it is
deployed and in use. What is still owed is one further clean confirmation, and
this entry says why it did not arrive.

## Every mount capture

| capture | board, gimbal | gimbal corners | gimbal reproj | board, OAK | OAK corners | OAK reproj | verdict |
|---|---:|---:|---:|---:|---:|---:|---|
| `oak-mount-held-out-03` | 0.371 m | 54 | 0.336 | 0.287 m | **37** | **2.099** | inconclusive |
| `oak-mount-dev` | 0.555 m | 50–54 | 0.177 | 0.463 m | 48 | 0.191 | adopted |
| `oak-mount-held-out-02` | 0.685 m | 50–54 | 0.201 | 0.582 m | 54 | 0.306 | confirms |
| `oak-mount-held-out-04` | 0.909 m | **37–50** | 0.348 | 0.798 m | 54 | **0.569** | inconclusive |

Pixels for reprojection. The gates, declared before any of this: at least 40
corners for the gimbal, 45 for the OAK, worst reprojection no more than 0.5 px,
and a distance at least 0.10 m from every distance already used.

**Too close and the depth camera fails.** At 0.287 m a board turned 44 degrees
off face-on spreads its near and far edges 0.167 m apart in depth, which its lens
cannot hold; it found 37 corners, all of them in board columns 0 to 5, and
reprojected at 2.1 px. Recorded in full in
[the slant entry](2026-09-07-p0-oak-mount.md)'s successor note and in the
runbook.

**Too far and the gimbal camera fails.** At 0.909 m the board subtends little of
a 130-degree fisheye frame: three of five frames found 40 corners or fewer.
Independently, the OAK's own reprojection climbs with distance — 0.191 px at
0.463 m, 0.306 at 0.582, 0.569 at 0.798 — and crosses the 0.5 px gate somewhere
between 0.6 and 0.8 m.

**The excluded band swallows the working band.** 0.555 and 0.686 are used, so a
third distance must be under 0.455 m or over 0.786 m. Under 0.455 the OAK cannot
focus a slanted board; over 0.786 the gimbal cannot count corners and the OAK
cannot reproject. There is no qualifying distance left.

## What the failed capture measured anyway

`oak-mount-held-out-04` was recorded at 0.909 m with the board turned 23 to 24
degrees off face-on — the geometry the runbook asks for, reached after four
setup iterations, with the rover's own headlights at half brightness supplying
the light after dusk. It is kept and marked inconclusive rather than repeated
until it passes.

Its numbers, for the record and not as a result: yaw +3.005, pitch +5.538, roll
−1.322 degrees, forward 0.113, left +0.007, up −0.084 m. Against the adopted
transform that is 1.51 degrees of yaw and 26 mm of forward disagreement, both
outside their gates. **Do not read that as evidence about the mount**: the
gimbal's corner coverage failed and the OAK's reprojection failed, so the fit
those numbers came from is not one the protocol accepts. Its own within-set
repeatability was 0.496 degrees of yaw against the development set's 0.058, which
says the same thing.

The two cameras also disagreed about the board's distance by 0.111 m where their
known separation is 0.087 m — a 24 mm excess that matches the 26 mm forward
disagreement, so both are one fault seen twice rather than two faults.

## The headlights work, and that is worth knowing

The capture happened after dark only because the rover has headlights. Measured
on the depth camera's own picture of the board:

| | brightness | corners found |
|---|---:|---:|
| no lights, after dusk | 32 of 255 | **0** of 54 |
| headlights at half | 141 | **54** of 54 |
| headlights at full | 149 | 54 of 54 |

Half brightness is enough and full adds nothing, so half is what to use — a
white sheet under a bright lamp risks the blown highlights the runbook warns
about, and at half there were no blown pixels at all. This makes the whole
calibration procedure independent of daylight, which it was not before.

## A trap worth writing down

The obliquity of the board — how far it is turned off face-on — has to be
measured as the angle between the board's normal and the line of sight, and
`calibrate_gimbal.pose` returns the **transpose** of the board-to-camera
rotation. So the board's normal in camera coordinates is that matrix's third
*row*; reading its third column instead gives the camera's axis in the board's
frame, which is a different quantity. It reported 8 to 10 degrees while the board
was really at 44, and the owner's eyeball was right where the arithmetic was
wrong. Validated afterwards by re-measuring the two adopted sets, which come out
at 3.7 and 3.5 degrees — matching how they were described.

A second trap alongside it: a pose fit can report a healthy 0.3 px reprojection
while being 19% wrong about distance, if the detected corners are a patchy subset
of a strongly foreshortened board. Both were caught only by checking the distance
a second way — the board's own 120 mm vertical spacing, undistorted through the
fisheye model, which a turn about the vertical does not foreshorten. That check
is worth keeping.

## Requirements

- No requirement changed state. [R-WS-11](../requirements/world-state.md#r-ws-11)
  stays `open` and its OAK-to-gimbal translation is unchanged.
- The adopted mount in `world_state/oak.py` is untouched. Nothing was averaged,
  no rule was relaxed, and no gate was moved after seeing a result.

## Next

1. **Print the target on A3** and re-verify its printed dimensions the way the
   runbook does for A4. At twice the linear scale the gimbal would see at 0.9 m
   roughly what it now sees at 0.6, which puts a qualifying third distance inside
   both cameras' working range. The OAK's share of frame grows too, so its
   reprojection wants watching.
2. The alternative is the gimbal's 2592 x 1944 mode, which the runbook already
   notes would require refitting the lens and repeating held-out validation. That
   is a larger job than printing a sheet.
3. Either way this is a loose end on a working measurement, not a blocker.
