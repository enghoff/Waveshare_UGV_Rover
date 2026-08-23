# Moving the tracking to new hardware

The face tracking works, and almost none of what makes it work is visible in the
shape of the code. It is in about thirty measured numbers, and in a handful of
mistakes that are easy to make a second time because each of them produces the same
symptom: a camera that will not settle on a face it can plainly see. This is a list
of both, written while the rover still runs on a Raspberry Pi 1, for whoever moves it
onto something faster.

The short version: **the geometry and the architecture carry over unchanged, every
constant does not, and the tuning constants do not merely change value — they change
order.** A gain that is right on this machine is wrong on a faster one, and not by a
little.

## What carries over, what must be re-measured, what must be re-tuned

| | what | where |
|---|---|---|
| **carries over** | the aiming geometry — a pixel is a direction, and the pan/tilt that centres it is a closed-form solve | `aiming.solve` |
| | aiming at an *angle* rather than a pixel, so a correction survives the camera moving under it | `Gimbal.track`, `keep_going` |
| | subtracting motion already in flight, from an exposure stamp rather than a constant | `Gimbal.was_at` |
| | which face is which, and how long a lock outlives a missed detection | `Target` |
| | the detector protocol: JPEG in, boxes out | [`face_detect/`](../face_detect), and `--service local` for a host that can run it |
| **re-measure** | the lens: angular scale, distortion, principal point | `calibrate_fov.py`, both axes |
| | that the aiming can actually use the lens | `calibrate_aim.py` |
| | the servo: counts per degree, speed ceiling, travel limits | the firmware, then check it |
| | command latency, and how old a frame is when the loop gets it | live, per frame |
| | how much the detector's box wanders on a motionless face | live |
| **re-tune** | `GAIN`, `SMOOTHING`, `DEADBAND`, `MAX_DT` | `aiming.py` |
| | `LOST_GRACE_S` and `GRACE_FRAMES`, which are a frame count wearing a stopwatch | `aiming.py` |
| | `SCAN_RATE`, which follows the frame rate | `aiming.py` |

Note the middle block is not optional even when the parts are unchanged. The lens was
re-measured on the same camera in August 2026 and moved by 3% in scale and 13 pixels
in principal point from the figures that had been flying, which was enough to leave
every face two and a half degrees high.

## Measure these, in this order

Each of these answers a question the next one assumes. Doing them out of order gives
confident wrong answers rather than obvious failures, which is how the aiming came to
be wrong for months.

1. **The lens.** `python usb_cameras/calibrate_fov.py sweep/ --axis pan` and again
   with `--axis tilt`. Run both: a sweep pins the coordinate of the principal point
   it moves along and says almost nothing about the other, and a pan-only run here
   put `cy` 97 px out while looking entirely healthy. Cross-check the servo at the
   same time with `--by rover`, which takes the angle from the lidar's scan match
   instead of trusting the pan servo; the two agreeing means the servo is honest as
   well as the lens known.
2. **That the aiming can use it.** `python usb_cameras/calibrate_aim.py aim/`. This
   is the question `calibrate_fov` does not answer — not how wide the lens is but
   whether the degrees the loop computes actually put a face in the middle in one
   move. Do it in a room with something far away in it; see the parallax note below.
3. **The clocks.** How old is a frame when the loop acts on it, and how long after a
   command does the picture start to move? Both are in `tracking_status`. On a
   machine that can keep up these are tens of milliseconds; on this one they were not,
   and that dominated everything else.
4. **The detector's noise.** Track somebody sitting still and take the frame-to-frame
   change in `measured_at` from `tracking_status`, which is the face's angle in the
   room and so is free of the camera's own motion. That number sets `SMOOTHING`.
5. **Only then, the tuning.** `GAIN` and the rest, against the frame age from step 3.

## The five mistakes, all of which look identical from outside

Every one of these presents as *the camera hunts*. They are told apart only by the
intermediate numbers, which is why `Gimbal.last` keeps them and why `tracking_status`
hands them out.

**A pixel is meaningless without the pose it was taken at.** A face box is a position
in one picture from one camera angle. Two things here have made the same mistake with
it. Re-reading a remembered pixel on a frame the detector did not answer applied the
same correction twice and carried the camera 23 degrees past a stationary face;
`keep_going()` exists for that. Averaging the pixel across frames — which is what
`Target` did until August 2026 — averages measurements taken from different poses,
and the loop then reads the average as though it were all measured at the newest one.
That is a lag inside the feedback path, and a lag inside the feedback path rings: the
camera reached a face on the third frame and then swung back out past a fifth of the
frame. **Smooth the angle. The angle is a fact about the room.**

**Never fabricate a timestamp.** When the exposure stamp cannot be matched to its
frame, the honest answer is "unknown", and the loop has a path for that. What it had
instead was a *guess that the frame is fresh*, which is the single worst assumption
available: it switches off the dead-time compensation exactly on the frames where the
compensation is most needed. A wrong timestamp is not detectable afterwards; a
missing one is.

**A picture the loop cannot consume is not free.** This is the one that cost the most
and the one most likely to recur, because it looks like a performance question and is
actually a correctness one. The camera delivers 30 frames a second. This host
reassembles four while the tracking loop has the core. The other 26 do not vanish and
they are not dropped — they queue, strictly first in first out, in v4l2-ctl's ring of
capture buffers and in the 64 kB pipe behind it, about five frames' worth altogether.

**A five-frame queue drained four times a second is 1.25 seconds of delay**, and that
is the whole of it. The rate the camera runs at does not enter the arithmetic except
to guarantee the queue stays full; what matters is the *depth* of the buffering
divided by the *drain rate*. Measured both ways on this rover, optically — swing the
gimbal and watch for the picture to follow, so that no exposure stamp is being
trusted:

| the reader is getting | it reassembles | the picture follows a command after |
|---|---|---|
| the whole core | 30 fps | 300 ms — one frame, plus the servo |
| what the tracking loop leaves it | 4 fps | 1300–1700 ms |

Two things that did *not* fix it are worth recording, because both look obvious.
Asking v4l2-ctl for fewer capture buffers changes nothing while the reader is keeping
up, and while it is not, the buffers are only part of the queue. Draining the pipe
empty on every pass — non-blocking reads until `EAGAIN`, keeping only the newest whole
frame — changes nothing either, because the backlog is *upstream* of the pipe:
v4l2-ctl can only push what it can write, so the ring behind it stays full and it
still dequeues the oldest frame in it. There is no way to skip to the front of that
queue from this end. **The only real fixes are to drain faster or to buffer less, and
on this host neither is available.**

On faster hardware the arithmetic changes but the trap does not: whatever reads the
camera must be able to keep up with it, or the loop ends up steering by the past.
Check it with one number — the age of the frame in hand — and check it under load,
because idle it always looks fine.

Worth knowing how far a code fix got here, because the answer is *not far*, and the
shape of that answer is the point. Reading the pipe in 64 kB bites instead of 4 kB and
looking up an exposure stamp only for the frame actually kept — the others discarded
by length, which costs nothing — measured **1.8 up to 10.4 frames a second** in
isolation on the Pi with a core kept busy. On the live rover the same change bought
4.1 to 4.9, and the frame age went from a median 1430 ms to 1329. The synthetic load
was not the real one, and the real one is a single 700 MHz core already spending
266 ms a frame decoding a JPEG. **This is a hardware limit wearing a software
costume**, and it is the clearest single argument for the new machine. Do not spend a
week on it here — it was worth about a fifth of the delay, and the remaining
four-fifths is arithmetic that only a faster host can change.

What was worth fixing was the honesty rather than the speed. When the stamp cannot be
paired to its frame, the code used to answer `CAMERA_LAG_S` — 41 ms — which on a host
running a second behind is not a small error but a claim of freshness, and it switched
the dead-time compensation off on precisely the frames where the pairing had already
shown something was wrong. It now answers with what frames have lately turned out to
be, kept as a slow average over the ones that did pair. The reported ages went from
two clusters — 20 frames in 80 claiming to be under 400 ms — to one, with none of 83
making that claim. The pictures are no fresher; the loop is no longer lied to about
them.

**Tuning constants are not portable, and their ranking inverts.** Simulated on the
real `Target` and `Gimbal` against a stationary face 39 degrees off centre, worst
swing back past the middle:

| | frame age 200 ms | frame age 1.4 s |
|---|---|---|
| gain 0.5 | none | 42 px |
| gain 0.7 | 3 px | 72 px |
| gain 0.9 | 36 px | 102 px |

On a machine where the picture is fresh, 0.7 is calm and 0.9 overshoots. On this one
every gain rings and the higher ones only ring harder — but the higher gain does get
*closest fastest on the first approach*, which is what a person watching the rover
sees, so it was the right trade here and will be the wrong one later. **`GAIN` is set
to 0.9 for that reason and should go back to 0.7 once the picture is fresh.** The
same applies to `SMOOTHING`, which is worth nothing at all when the box wanders less
than the deadband and is the only thing preventing a twitching servo when it wanders
more.

**Rotation is not the whole story at close range.** The gimbal pivots a few
centimetres behind the lens, so a subject nearer than about a metre moves by parallax
as well as by rotation, and no rotation-only model can take that out. It is worth two
or three degrees at conversational distance and much more at half a metre. It also
ruins any calibration measured against something close: the first run of
`calibrate_aim.py` was taken against a sofa 40 cm away and gave a confident answer in
the wrong direction. Measure across a room.

## Two numbers for one physical thing will drift apart

The lens was described twice — once as a field of view for the map's camera cone, once
as a pixels-per-degree pair for the aiming — and the two were measured separately, by
different methods, months apart. They disagreed by 6%, and nothing could notice
because neither knew the other existed. There is now one lens in `aiming.LENS` and
everything else is derived from it: the half-frame widths that size the sweep and the
deadband, the directions the aiming solves with, the cone on the map. Keep it that
way on the new hardware, and when something needs a number about the optics, make it
ask rather than remember.

The same argument is why `aiming.py` is a separate module rather than the tracking
loop's own code: two scripts drive this gimbal, one on the rover and one on a desk,
and every constant they might disagree about is a way for them to be two different
robots.

## What this Pi managed, as a baseline

Worth having, because "faster" needs something to be faster than, and because a new
machine that comes in *below* one of these is telling you something.

| | measured on the Pi 1 |
|---|---|
| host | BCM2835 armv6, one core at 700 MHz, 474 MB |
| tracking loop | 2.3–2.4 frames a second |
| of which, decoding the JPEG | 275–308 ms wall, 115–135 ms busy |
| of which, the detection on the OAK | 123–127 ms |
| frames reassembled off the camera | 4.9 a second, against 30 delivered |
| age of the frame the loop acts on | median 1.33 s |
| command to the picture moving | 140 ms |
| exposure to a whole frame in hand | 41 ms |
| servo ceiling | about 130 deg/s; 50 degrees in 422 ms |
| gimbal travel | pan ±180, tilt −30 to +90 |
| lens | 130 × 96 degrees, 11.82 arcmin per pixel on the axis |

## What the faster hardware actually fixed

Written before the move and left as written; measured after it, on 2026-08-23, on
a Banana Pi M4 Zero — four Cortex-A53 at 1.416 GHz, 4 GB, aarch64.

| | Pi 1 | Banana Pi M4 Zero |
|---|---|---|
| the detector | an SSD on the OAK's VPU, over loopback HTTP | YuNet in the loop's own process |
| tracking loop | 2.3–2.4 fps | **6.6 fps** |
| decoding the JPEG | 275–308 ms wall | **7 ms** |
| the detection | 123–127 ms on the VPU, 190 ms through the service | **146 ms** on three of four cores |
| age of the frame the loop acts on | median 1.33 s | **~190 ms** |
| frames reassembled off the camera | 4.9 of 30 | 30 of 30, the loop dropping what it cannot use |

Which settles the three predictions below. The frame age, the loop rate and the
decode all came down by about a factor of seven; `GAIN` did **not** need changing,
because 0.9 had already been reverted to 0.7 and 0.7 is the value the tuning table
above gives for a 200 ms frame; the grace window looks after itself, since it is
`max(LOST_GRACE_S, GRACE_FRAMES * dt)` and `dt` shrank; and the OAK did become
unnecessary as an inference stick — it is a depth camera now, see
[`oak_depth/`](../oak_depth).

Two things the move did not fix, as predicted: the parallax at close range and the
servo's ceiling. And one it introduced, which no amount of arithmetic here
anticipates — the new board hard-resets under load, seventeen times in one working
day, with nothing in the journal. A faster host is not automatically a steadier one.

## What faster hardware will and will not fix

It will fix the frame age, the loop rate and the decode, and with them most of the
tuning: expect `GAIN` back down, the grace window back to a time rather than a frame
count, and the sweep faster. It will probably make the OAK unnecessary as a separate
inference stick.

It will not fix the parallax, the gimbal's travel limits, the servo's ceiling, or the
lens's distortion, because none of those are about the computer. Nor will it fix a
model that is the wrong shape — the separable pixels-times-a-gain aiming would still
be leaving a face 20 degrees out in the corner at 60 frames a second; it would simply
walk to it faster and look almost right.

## Prove it on the machine

Not by inference and not because the self-test passed on a workstation. The rule the
repository runs on is in [CLAUDE.md](../CLAUDE.md), and for the tracking specifically
that means: `python3 selftest.py` on the host itself, then call the tool over TCP and
read `tracking_status` while somebody stands in front of it. The fields are there for
this — `error`, `frame_age_ms`, `was_at`, `measured_at`, `face_at`, `sent_to`, read
left to right, are the whole chain from a pixel to a servo command, and a fault in
any link is visible as a disagreement between two of them while each looks reasonable
alone.
