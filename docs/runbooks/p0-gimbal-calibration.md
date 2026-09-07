# P0 gimbal calibration: prepare the printed reference

This prepares the independent printed reference for the bounded P0 measurement.
It does not change a calibration constant or authorize semantic movement. The
experiment and stopping rules are in the
[autonomy plan](../plans/autonomous-curiosity.md#bounded-calibration-protocol).

## Print and verify the target

Use [the A4 ChArUco target](../../output/pdf/p0-gimbal-charuco-a4.pdf), page size A4
in landscape orientation.

Canon specifies A4 printable margins of 3.0 mm at the leading edge, 16.7 mm at the
trailing edge and 3.4 mm at each side for the MG2577S. The target keeps every mark
at least 20.5 mm from every PDF edge, so it remains inside the printable area even
if the landscape driver rotates which edge trails. See
[Canon's MG2577S specification](https://in.canon/en/consumer/pixma-mg2577s/main/specification?category=printing&subCategory=).

1. Print in black and white at **Actual size** or **100%**. Disable Fit, Shrink,
   Scale to printable area and borderless expansion. Use normal or high quality,
   print one side only, and do not photograph the PDF from a screen.
2. Measure the vertical 100 mm line before mounting it. It should be 100.0 mm
   within the practical reading accuracy of the ruler, and no more than 0.5 mm
   different.
3. Measure the checkerboard's outside width and height: 240.0 x 168.0 mm. Accept
   up to 1.0 mm error on either dimension. Also check one five-square span in each
   direction is 120.0 mm within 0.5 mm. If it is outside those limits, keep the
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

Send the five measured print dimensions: the 100 mm bar, outside width, outside
height and both 120 mm five-square spans. Then say the target is mounted and the
rover is ready.

The capture operator will first verify detection without writing calibration. The
first campaign samples commanded pan -20, -10, 0, +10 and +20 degrees from both
approach directions, with paired repeats and held-out angles reserved for later.
Face tracking, voice aiming and other gimbal users must remain off during capture.
The current calibration stays unchanged until development data supports a simple
candidate and a separate held-out session validates it.

The operator records the development campaign through the daemon, which remains the
camera and UART owner:

```bash
python usb_cameras/calibrate_gimbal.py captures/p0-gimbal-YYYY-MM-DD/campaign \
  --rover 192.168.1.80:8769 --size 1280x960
```

The tool first takes separate pan/tilt views to fit the fisheye from the printed
geometry without treating servo commands as measurements. It then records three
paired pan sweeps in alternating order, two stationary frames at each stop, raw
corner detections and every command response. It writes progress after every stop,
marks an interrupted run invalid, returns the camera to pan/tilt zero and never
writes a deployed calibration constant. Reanalyse preserved frames with `--fit-only`.
The reference resolves the expected error only when stationary duplicate pose
differences have a median no greater than 0.25 degrees and a 95th percentile no
greater than 0.75 degrees. A run outside that bound is reported as inconclusive,
not as a servo failure or a calibration result.

The first development result selects a single candidate only if the reference gate
passes: leave pan gain unchanged and finish every supported pan placement from the
ascending direction. Validate it in a new folder without changing those decisions:

```bash
python usb_cameras/calibrate_gimbal.py \
  captures/p0-gimbal-YYYY-MM-DD/held-out-01 \
  --rover 192.168.1.80:8769 --size 1280x960 \
  --angles=-20,-15,-5,5,15,20 --candidate consistent-ascending
```

The four intermediate angles are held out; the endpoints check the supported
envelope in the new session. The candidate passes only when the independent
reference passes again, the ascending-only gain error is no greater than 1.0% and
the 95th percentile of its absolute linear-fit residual is no greater than 0.5
degrees. Both directions are still captured so the control condition and remaining
backlash are reported. Do not change this rule after seeing the held-out result.

Stop immediately if the target or rover moves, the sheet lifts from its backing,
the gimbal touches anything, or another process aims the camera. That run is kept
and marked invalid rather than repeated until it happens to pass.
