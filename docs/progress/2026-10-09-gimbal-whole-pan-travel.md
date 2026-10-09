# The gimbal pans where it is told across its whole travel, to about half a degree

**Sent anywhere from -170 to +180 degrees of pan and arriving from below, the
gimbal camera lands within 0.6 degrees of the commanded angle with no correction
at all, and within 0.38 degrees of a table measured an hour earlier in another
room.** That is the same order as inside the ±20 degrees the calibration covers
today. The measurement was held to a rule fixed before it ran, and by that rule
it is **inconclusive**: the reference agreed with the camera's own pictures to
0.34 degrees where the rule asked for 0.25. So the envelope in
`world_state/inspector.py` stays at ±20, and widening it is the owner's call.

Two other things came out of it. Returning to rest from a swing of more than
about 135 degrees landed 2.1 degrees off, on the wrong side of the backlash,
while the daemon believed otherwise. That is fixed, deployed and proved on the
rover. And the gimbal turns no faster than the chassis does on the spot. Its
advantage is that it lands where it is told and leaves the rover's pose alone.

## How it was measured

[capture_pan_sweep.py](../../usb_cameras/capture_pan_sweep.py) steps the pan
through its whole travel at the rest tilt of 20 degrees, holding still at every
stop, and [fit_pan_sweep.py](../../usb_cameras/fit_pan_sweep.py) reads where the
camera really went. The pass rule was committed before each session ran
(4043fcc, then 038ff66 for the held-out session).

**Where the camera went is the OAK's gyro, which rides the gimbal and owes the
servo nothing.** Each pan step is the rate about gravity, integrated across the
move, less a bias read at every still hold. Its scale was not taken on trust. A
picture at -180 and one at +180 look the same way, so the travel between them is
360 degrees plus the small angle the two pictures differ by. That gives the
gyro's scale with no lens and no servo in it: 1.0182 and 1.0180 from two pairs
in the first session, and 1.0193 in the second. **The gyro over-reads pan by
about 1.8%.** That is what the full-travel sweep of 2026-09-30 read as the servo
going 2% too far, and the servo does not.

The gimbal camera's picture at every stop is a second, independent account,
where the comparison allows one. Between two stops the lens itself moves,
because it sits a few centimetres ahead of the pan axis, so a ten-degree step
measured from pictures carries about 2.6% of parallax. Between two arrivals at
the same commanded angle from opposite sides it barely moves, and there the
pictures are exact.

There were two sessions, both with the rover parked and the wheels still. The
first was at 12:58 in the bedroom: ascending in tens, descending in tens,
ascending in the fives between, ascending again at tilt 0, and a set of timed
swings. The second was held out, at 13:28 at the charger in the living room:
ascending in fives, descending in tens, and eight returns to rest.

## Where it lands

Actual against commanded pan, ascending arrivals at tilt 20, both sessions
measured from their own stop at zero:

| Commanded | -170 | -150 | -120 | -90 | -60 | -30 | 0 | +30 | +60 | +90 | +120 | +150 | +180 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| First session | -169.18 | -149.66 | -120.07 | -89.98 | -60.04 | -30.01 | 0 | +29.84 | +59.59 | +89.78 | +119.89 | +149.84 | +179.75 |
| Held out | -169.38 | -149.68 | -120.00 | -90.00 | -60.03 | -30.10 | 0 | +29.87 | +59.68 | +89.75 | +119.59 | +149.51 | +179.60 |

A single straight line through the first session gives a gain of 0.9985. That
is no gain error worth correcting, and what is left is a gentle wave of a few
tenths. The left end is the only place it reaches more than half a degree raw:
+0.6 to +0.8 short of -170, and +1.2 at -175 where the servo meets its stop. At
tilt 0 the first session found the same shape to within about 0.1 degree. The
held-out session's 72 stops lay within 0.38 degrees of the first session's
table (p95 0.33) everywhere from -170 to +180.

**The backlash is 1.0 to 2.0 degrees across the whole travel**, about 1.9 within
±80 and falling to about 1.1 beyond +120. The ±20 calibration found 1.6 to 1.7
there. Arrivals from below are what count, because every move the daemon makes
ends with a five-degree step up.

## The reference, and the rule's verdict

| Gate, fixed before the run | First session | Held out |
|---|---|---|
| Closing the circle agrees on the gyro's scale within 0.3% | pass (0.05%) | pass (0.10% from the first) |
| Pictures agree with the gyro on each step | fail: 0.49 against 0.3, which measured the lens's parallax, not the gyro | replaced by the backlash check |
| Backlash by gyro and by pictures agree, p95 within 0.25 | (0.15, checked afterwards) | **fail: 0.34** |
| Pictures of rest before and after agree within 0.2 | fail: 3.4, which was the landing fault below | replaced by the returns check: pass, worst 0.22 |
| Every held-out stop within 0.5 of the candidate | to -165 and +175 | **-170 to +180**, max 0.38 |

The held-out session's backlash check failed between +110 and +160, where the
gyro read the backlash 0.3 degrees larger than the pictures did. Everywhere else
the two agreed to about 0.1. Read against the pictures at every rest in the
session, the gyro drifted by 0.25 degrees over its eight wide swings and by 0.13
over the sweeps. **So the reference is good to about a quarter to a third of a
degree, and the held-out residuals are the same size.** The data cannot tell
whether 0.38 degrees is the servo or the reference. Under the rule that leaves
the envelope unvalidated, and the rule was not moved after the result was seen.

## How fast it turns

From the gyro, a single pan command lands within 0.2 degrees of its target in:

| Swing (deg) | 24 | 34 | 56 | 65 | 86 | 95 | 132 | 141 | 175 | 179 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Seconds | 0.49 | 0.58 | 0.74 | 0.78 | 0.89 | 0.95 | 1.10 | 1.13 | 1.20 | 1.20 |

The peak rate is about 270 degrees a second. An aimed look adds the five-degree
step up and the daemon's one-second settle to that.

The chassis is not slower. Every on-the-spot turn in the autonomy record, 245 of
them and up to 180 degrees, reported arrival at the first two-second poll, and
in-place turns reach 169 degrees a second. **The difference is in where it ends
up.** A turn arrives anywhere within 15 degrees of the heading asked for, and on
this rover it leaves the map's heading 9 to 16 degrees out for seconds afterwards
([2026-10-02](2026-10-02-depth-matched-and-heading-checked-on-the-move.md),
[the arrival-heading entry](2026-10-09-arrival-heading.md)). A pan leaves the
chassis, the lidar and the pose exactly where they were.

## The return to rest from a wide swing

Centring drops five degrees below rest and steps up, so that the servo arrives
from below. It waited a fixed 0.6 seconds between the two. From +180 the swing
takes 1.3 seconds, so the servo was still coming down at about +110 when told to
stop at zero. It arrived from above, 2.1 degrees right of rest, three times in
three. `gimbal_at_rest` still said it had arrived from below, so every look until
the next move would have carried a bearing 2 degrees off and been charged as if
it were seated. From 135 it overshot past the undershoot and came back up, by
luck.

The wait now grows with the swing (583756b): half a second plus one per 200
degrees, never less than the 0.6 it was. Deployed at 038ff66. The running daemon
took 1.43 s to centre from +180 and still 0.61 s from +15. In the held-out
session, returns from 30, ±90, ±135 and ±180 all landed within 0.22 degrees of
each other.

## What it changes

- Nothing yet about which looks keep a direction: `DEMONSTRATED_PAN_DEG` stays at
  20. Widening it needs either the owner's decision on this result or a third
  session with a reference that can settle the last third of a degree.
- The candidate table is frozen in
  [pan_candidate_2026-10-09.json](../../usb_cameras/pan_candidate_2026-10-09.json).
  Within ±20 it agrees with the board calibration of 2026-09-07 to about 0.2
  degrees.
- The looking loop kept recording during both sessions. Its bearing-carrying
  looks are ids 84552-84854 (bedroom, 12:59-13:06) and 85147-85464 (living room,
  13:27-13:33), all at pans within ±20. They were reported to the session running
  M4.

The pictures, IMU streams and stop logs are in
`captures/2026-10-09-pan-sweep/` and `captures/2026-10-09-pan-sweep-held-out/`.
