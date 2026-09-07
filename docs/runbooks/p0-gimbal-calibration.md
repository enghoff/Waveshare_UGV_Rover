# P0 gimbal calibration: prepare the printed reference

This prepares the independent printed reference for the bounded P0 measurement.
It does not change a calibration constant or authorize semantic movement. The
experiment and stopping rules are in the
[autonomy plan](../plans/autonomous-curiosity.md#bounded-calibration-protocol).

## Print and verify the target

Use [the A4 ChArUco target](../../output/pdf/p0-gimbal-charuco-a4.pdf), page size A4
in landscape orientation.

1. Print in black and white at **Actual size** or **100%**. Disable Fit, Shrink,
   Scale to printable area and borderless expansion. Use normal or high quality,
   print one side only, and do not photograph the PDF from a screen.
2. Measure the 100 mm line before mounting it. It should be 100.0 mm within the
   practical reading accuracy of the ruler, and no more than 0.5 mm different.
3. Measure the checkerboard's outside width and height: 250.0 x 175.0 mm. Accept
   up to 1.0 mm error on either dimension. Also check one five-square span in each
   direction is 125.0 mm within 0.5 mm. If it is outside those limits, keep the
   sheet and record the measurements, but do not use it as the reference.
4. Check that every black square and marker is sharp and complete. Reprint if ink
   gaps, clipping or banding damage a marker.

These limits validate the printed reference; they are not a demand that the rover
point this accurately. Later analysis carries reference uncertainty separately
from mechanical variation.

## Mount it

- Fix the whole sheet to rigid, flat backing. Keep it free of wrinkles and do not
  put glossy tape over the board.
- Mount it vertically, with the printed text upright, in a well-lit stationary
  part of the room without glare.
- Put the centre roughly level with the gimbal camera and initially 0.6-0.8 m from
  its lens. This distance is setup guidance, not a calibration input.
- Park the rover squarely in front of it on level floor. Leave enough clearance for
  the gimbal to pan at least 30 degrees in both directions without obstruction.
- Do not move the rover, target or backing once capture starts. Keep people and
  moving shadows out of the target area.

## Hand over for capture

Send the four measured print dimensions: the 100 mm bar, outside width, outside
height and both 125 mm five-square spans. Then say the target is mounted and the
rover is ready.

The capture operator will first verify detection without writing calibration. The
first campaign samples commanded pan -20, -10, 0, +10 and +20 degrees from both
approach directions, with paired repeats and held-out angles reserved for later.
Face tracking, voice aiming and other gimbal users must remain off during capture.
The current calibration stays unchanged until development data supports a simple
candidate and a separate held-out session validates it.

Stop immediately if the target or rover moves, the sheet lifts from its backing,
the gimbal touches anything, or another process aims the camera. That run is kept
and marked invalid rather than repeated until it happens to pass.
