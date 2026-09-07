# P0 camera geometry calibration

This prepares and uses the independent printed reference for the bounded P0
measurement. It does not authorize semantic movement. The experiment and stopping
rules are in the
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
- Put the centre roughly level with the gimbal camera and **0.45-0.55 m from its
  lens**. This distance decides whether the campaign can certify anything, so it
  is a calibration input and not setup guidance -- which is what it was called
  until 2026-09-07, at 0.6-0.8 m. Measured across seven sessions that day, every
  repeatability figure improved by a factor of three to four as the board came in
  from 0.70 m to 0.47 m, and two sessions failed at 0.70 m purely on measurement
  scatter. The board subtends 28.6 degrees of the frame at 0.47 m and 19.3 at
  0.70, and a target that small determines its own pose poorly. See
  [the tilt-20 result](../progress/2026-09-07-gimbal-tilt20-passes.md).
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

## Current gimbal result

The 1280 x 960 held-out run on 2026-09-07 passed. Within commanded pan -20 to +20
degrees, at tilt zero and at tilt +20, use unchanged gain and finish placement
from the ascending direction. The tilt-zero figures follow; tilt +20 was measured
later the same day and is below. The ascending gain error was -0.34% and absolute residual p95 was 0.297
degrees. Stationary duplicates differed by 0.074 degrees median and 0.225 degrees
p95. Opposite approaches still differ by 1.19-2.23 degrees, so a capture reached
from another direction is outside this demonstrated state. These are measured
limits, not a reason to tune toward unattainable mechanical precision.

Those four figures are from the re-analysis after the pose fit was corrected on
2026-09-07; the same run first read -0.535%, 0.457, 0.117 and 0.423 degrees. The
verdict did not change, only its margin. See
[the mount entry](../progress/2026-09-07-p0-oak-mount.md) for what was wrong.

**The envelope now covers tilt +20 as well as tilt zero**, which matters because
1828 of the rover's 2165 recorded looks were taken at tilt 20 and only 310 at
zero. `tilt20-held-out-02` passed the unchanged rule with an ascending gain error
of -0.92% and a residual p95 of 0.193 degrees. Read that as sitting *on* the rule
rather than inside it: the margin is 0.08 of a point and the development session
at the same tilt read -1.04%. The pan gain genuinely depends on tilt -- about 1.5
percentage points between level and 20 up, against 0.12 points of scatter -- so
**measure at the tilt you will use and never interpolate between tilts**, which
is what `--tilt` is for. The backlash does not depend on tilt: it held between
-1.59 and -1.72 degrees across all seven sessions.

**The capture path enforces this envelope rather than trusting it.** A look
taken at a commanded pan outside ±20 degrees keeps its picture and records no
direction, and a look that reached its angle from the descending side of the
backlash keeps a bearing widened to 2.3 degrees. Both numbers live beside each
other in `world_state/inspector.py` as `DEMONSTRATED_PAN_DEG` and
`UNSEATED_APPROACH_SIGMA_DEG`, and `Rover.centre_gimbal` now undershoots by 30
degrees before settling so that rest is an ascending arrival. **Widening the
envelope is a measurement and not an edit**: run the campaign at the angles
wanted, pass its gates, then move the constant and say where the number came
from. Of the 2162 observations the rover had recorded by 2026-09-07, about 92%
were taken at a pan inside the current envelope and 1952 of them at pan zero,
so the envelope costs the recording under a tenth of its looks. The tilt gap
that stood here on the morning of 2026-09-07 is closed: tilt 20 was measured the
same day and passed.

The camera's advertised maximum is 2592 x 1944 MJPEG at 30 fps, but this campaign
stays at 1280 x 960. A live comparison found the same field of view and more corner
detections at the maximum mode, but 1280 x 960 already passed the reference and
held-out accuracy gates. A resolution change would require refitting the lens and
repeating held-out validation. Use the maximum mode only if later evidence shows
the gimbal reference is the limiting measurement; it does not improve the OAK's
target coverage. Normal face tracking remains at its validated 640 x 480 mode.

## Frame the target for both cameras

The fixed OAK is lower than the gimbal camera and has a narrower view. In the first
mounted preflight it detected 24 of 54 ChArUco corners: all six rows but only four
of nine columns. The upper part of the portrait target was outside its frame. That
fit is retained as inconclusive even though repeated estimates and pixel residuals
looked precise; its pitch changed by 2.44 degrees when the exact OAK distortion was
used, which shows that the partial planar view does not constrain pose adequately.

The target was lowered by **about 120 mm** on 2026-09-07, keeping the sheet vertical,
flat and in the same orientation. This amount follows
from the observed 25-pixel spacing of the 24 mm squares and should put the missing
five columns into view while keeping the full board inside the gimbal image. Exact
centering is unnecessary. The resulting development capture had 48 OAK corners in
every frame and 50-54 gimbal corners after native/enlarged detector selection, so
the coverage gate passes.

## Measure the fixed OAK mount

Once both cameras see at least 45 corners, capture the development measurement:

```bash
python usb_cameras/calibrate_oak_mount.py \
  captures/p0-gimbal-YYYY-MM-DD/oak-mount-dev \
  --gimbal-analysis captures/p0-gimbal-YYYY-MM-DD/held-out-01/analysis.json \
  --rover 192.168.1.80:8769
```

The tool gives the gimbal the validated ascending approach to zero, records five
stationary frames from each camera, reads the OAK's stored intrinsics and distortion,
and returns the gimbal to zero. It never writes `world_state/oak.py`. A development
fit is usable only when both cameras have enough board coverage, reprojection RMS is
at most 0.5 px, each angular estimate spans no more than 0.75 degrees, each offset
component spans no more than 15 mm, and the runtime pinhole approximation differs
from the stored OAK lens by no more than 0.75 degrees across the sampled frame grid.

A seventh gate joins those six: `pose_converged`. It nudges each fitted pose a
tenth of a degree about each axis and fails if that recovers more than 0.5% of the
reprojection residual, which is what an unconverged fit looks like from the
outside. It exists because the bench had one -- see the mount result below.

The development capture passed these gates. Its candidate, after the corrected
fit, is yaw +1.492, pitch +6.256 and roll -1.200 degrees, with offset +0.0872 m
forward, -0.0031 m left and -0.0937 m up. Do not deploy a candidate until the
held-out comparison below passes.

The first held-out move placed the gimbal camera 0.880 m from the target, where the
OAK stream could resolve no markers; that attempt is invalid and retained. For the
replacement held-out validation, use a gimbal-target distance of **0.65-0.70 m**.
From the failed distant position this means moving the rover about **0.20 m straight
forward**, without changing the target. Verify OAK detection before creating a new
acceptance folder, then compare it with the frozen development result:

The normal 640 x 360 OAK frame still cannot resolve the markers at this distance.
Calibration-only preflights with continuous autofocus found 42 corners at 1280 x 720
and all 54 at 1920 x 1080. Use the latter with its size-specific factory intrinsics.
The bench temporarily releases the resident OAK service, captures five settled
frames and restores the service. This does not change its normal 640 x 360 paired
colour/depth stream.

```bash
python usb_cameras/calibrate_oak_mount.py \
  captures/p0-gimbal-YYYY-MM-DD/oak-mount-held-out-02 \
  --gimbal-analysis captures/p0-gimbal-YYYY-MM-DD/held-out-01/analysis.json \
  --rover 192.168.1.80:8769 \
  --oak-size 1920x1080 \
  --compare captures/p0-gimbal-YYYY-MM-DD/oak-mount-dev/mount-analysis.json
```

**A steep slant and a close board cannot be had together, because of the depth
camera.** Turning the board off face-on is what fixes the ill-conditioning of a
nearly fronto-parallel target, and for the gimbal camera it works well: at
0.371 m and 44 degrees off face-on, `oak-mount-held-out-03` found all 54 corners
on all five frames at 0.30 px, the best any mount capture has managed. The OAK
failed the same capture. It sits 87 mm ahead of the gimbal camera, so it was
0.287 m from the board, and at 44 degrees the board's near and far edges are
0.167 m apart in depth -- more than its lens can hold at that range. Its
autofocus settled rather than hunted (all five frames equally soft, and the
capture already discards 45 frames before keeping any), and the corners it lost
were entirely at one edge: six in each of board columns 0 to 5, then 1, 0 and 0.
Image sharpness over the board fell to 1030 from the 1700 of the 0.60 m capture
even though the board was larger in frame, which is what defocus looks like.

So aim for **0.80-0.85 m from the gimbal camera with 25-30 degrees of slant**.
Depth of field grows quickly with distance and a gentler slant roughly halves
the near-to-far spread. Going closer instead makes it worse, which together with
the separation rule below rules the near side out. Measure the obliquity rather
than eyeballing it -- and measure it as the angle between the board's normal and
the line of sight, not as where the board sits in the frame. `gimbal.pose`
returns the *transpose* of the board-to-camera rotation, so the board's normal in
camera coordinates is its third row and not its third column; reading the column
gives the camera's axis in the board's frame, which stayed at 8-10 degrees while
the board was actually turned to 44.

**With the A4 target there is no qualifying third distance, and that is
measured rather than suspected.** Both cameras only see the board well enough
between about 0.55 and 0.69 m: closer, the OAK cannot hold a slanted board in
focus; further, the gimbal cannot find 40 corners in a 130-degree fisheye frame
and the OAK's reprojection crosses 0.5 px somewhere between 0.6 and 0.8 m. Since
0.555 and 0.686 are used, the rule below excludes 0.455 to 0.786 — which is the
whole working band. **Print the target on A3 before attempting a third
distance**, and re-verify its printed dimensions the same way. See
[the four captures](../progress/2026-09-07-no-third-mount-distance.md).
This bites on a *third* distance only. The clean second trial still owed
below needs no new print.

**The rover's own headlights make this procedure independent of daylight.** On
the depth camera's picture of the board after dusk: no lights gave brightness 32
of 255 and **zero** of 54 corners; `set_lights` at 128 gave 141 and all 54. Full
brightness added nothing and risks the glare the mounting section warns about, so
use half. Measured 2026-09-07.

The held-out distance must differ from the 0.555 m development distance by at least
0.10 m. The transform must agree within 0.75 degrees on every angle and 15 mm on
every offset component. Adopt the frozen development transform only after that pass.
If board coverage or agreement fails, report the result as inconclusive and adjust
the measurement geometry; do not tune thresholds or average a biased fit into the
runtime calibration.

## Current OAK mount result

**Adopted on 2026-09-07 and deployed.** `world_state/oak.py` carries yaw +1.492,
pitch +6.256, roll -1.200 degrees and offset +0.0872 m forward, -0.0031 m left and
-0.0937 m up, and the running daemon reports them over TCP 8769.

The held-out set at 0.686 m -- 0.130 m beyond development -- first failed on an
internal yaw range of 0.863 degrees against the 0.75 limit. Attributing its 25 pair
estimates to their source frames put 99.2% of that spread on the five gimbal frames
and 0.8% on the five OAK frames, and the cause turned out to be the pose fit rather
than any frame: it used the analytic planar solution without minimising it. With
both cameras' fits refined, the held-out yaw range is 0.357 degrees and every gate
passes.

**That set was re-analysed, not re-captured, and a clean second trial is still
owed.** It does not need a third distance and so does not need a bigger sheet:
photograph a fresh development set at 0.555 m and a fresh held-out set at
0.686 m, both after the pose-fit correction, and the 0.10 m separation rule is
satisfied by the 0.130 m between them. Adopt the fresh development transform
only if it agrees with the deployed one inside the same 0.75-degree and 15 mm
gates; if it does not, the deployed value is what is in question and not the
gates. A third distance is a separate and larger question -- it would say
whether the 12 mm forward-offset walk keeps growing with distance -- and that
one does need A3.

Either way, **turn the target 20 to 30 degrees off face-on about its vertical
axis**. Very nearly face-on is what makes a planar target determine its own
out-of-plane tilt badly, which is the whole difficulty here; simulation predicts
roughly a three- to fourfold improvement from the turn, which the capture will
test. Keep the sheet flat and the gimbal inside its demonstrated envelope.
