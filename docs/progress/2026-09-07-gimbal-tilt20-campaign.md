# The gimbal's turning at the tilt it actually uses, and why the answer is "not yet"

The pan campaign was repeated at gimbal tilt +20, where 84% of this rover's looks
are taken. It did not validate — and the reason is not the tilt. A control
session at tilt zero, in the same room, at the same board, minutes apart, failed
the same rule by nearly as much. **The gain error is not repeatable enough at the
current board distance to certify at either tilt**, and that is a fact about the
measurement rather than about the servo.

The result that did carry is the backlash, and it is the one the deployed code
depends on.

## The four sessions

Nothing was moved between the last three. The first is the session that passed
this morning, at a board 17 cm closer, and it is here for comparison rather than
as a control.

| session | tilt | board | ascending gain error | residual p95 | backlash | duplicate median | duplicate max | optics RMS | verdict |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| `held-out-01` | 0 | 0.538 m | −0.34% | 0.297 | −1.71 | 0.074 | 0.356 | 0.173 | pass |
| `tilt0-control-01` | 0 | 0.701 m | **−1.07%** | 0.387 | −1.59 | 0.141 | 0.860 | 0.194 | **fail** |
| `tilt20-dev` | +20 | 0.704 m | −0.63% | 0.348 | −1.63 | 0.196 | 0.436 | 0.206 | development |
| `tilt20-held-out-01` | +20 | 0.707 m | **−1.42%** | 0.439 | −1.70 | 0.135 | 0.716 | 0.209 | **fail** |

Degrees except where marked. The rule, declared before any of these were
captured and unchanged by them: the reference gate needs a stationary duplicate
median no worse than 0.25 degrees and a 95th percentile no worse than 0.75, and
the candidate then needs an ascending-only gain error no greater than 1.0% with
an absolute residual 95th percentile no greater than 0.5 degrees.

Every session passed the reference gate. Two failed the candidate on gain alone;
no session failed on residual.

## Tilt is not the cause

The tilt-20 held-out missed the limit by 0.42 points. But the tilt-zero control
at the same board missed it by 0.07, and the gap between the two tilts at that
board is 0.35 to 0.44 points — **smaller than the gap between two sessions at the
same tilt**, which is 0.73 points at tilt zero and 0.79 at tilt 20.

Across all four sessions the ascending gain error spans **1.08 percentage points
against a limit of 1.00**. A quantity that wanders by more than the width of its
own acceptance rule cannot be certified by one measurement, and the honest
reading is that this bench cannot presently pin the gain down at 0.70 m — not
that the servo behaves differently 20 degrees up.

That is consistent with what `world_state/README.md` already said about this
term: "a gain-like walk somewhere between four and eight per cent — the bench
cannot pin it closer, because the room moves while it measures."

**It also weakens this morning's pass.** `held-out-01` returned −0.34% and
cleared the rule comfortably, and nothing here contradicts it; but now that the
same quantity is known to scatter by about the limit, a single clearing session
is thinner evidence than it looked. Whether 0.538 m is genuinely the better
setup or that session was a favourable draw cannot be told from one session at
that distance.

## What did carry, across every session

**The backlash spans 0.12 degrees across all four** — −1.59, −1.63, −1.70,
−1.71 — at two tilts and two board distances. It is the most stable thing this
bench measures, and it is what the newly deployed gate depends on: an angle
reached from the descending side is charged 2.3 degrees of bearing error, from
the 1.19-to-2.23-degree spread the approaches were measured to differ by. That
number is unaffected by anything here, and it now has tilt-20 evidence behind it
where before it had none.

The other thing that carried is that the board is visible at tilt 20 without
moving the sheet: 50 to 54 of 54 corners across pan −20 to +20, against 54 at
tilt zero, with the board's centre at y 620 of 960 rather than 417.

## Everything is noisier at this distance

| | 0.538 m | 0.70 m |
|---|---:|---:|
| optics reprojection RMS | 0.173 px | 0.194–0.209 px |
| per-frame pose reprojection, median | 0.169 px | 0.191–0.196 px |
| stationary duplicate, worst | 0.356 deg | 0.436–0.860 deg |

The board subtends 24.8 degrees of the frame at 0.538 m and 19.0 at 0.707 m, so
a target that was already small in frame is a quarter smaller again. A smaller
target determines its own pose less well, which is the same ill-conditioning that
made the mount measurement difficult earlier today. **This is the most likely
explanation and it is not proved here**: only one of the four sessions was taken
at the closer distance, so the comparison is one session against three rather
than a controlled pair.

## What the bench gained

`usb_cameras/calibrate_gimbal.py` takes `--tilt`, which moves the pan sweeps and
their overshoot together and writes the tilt into the campaign metadata and out
through the analysis, so two campaigns at different tilts cannot be compared as
if they were repeats. Campaigns recorded before the option read as tilt zero,
which is what they were. Re-analysing `held-out-01` reproduces it unchanged.

The optics views already spanned tilt 0 to 20, so the lens fit was never the
tilt-specific part; what the sweeps measure is the servo.

## Requirements

- [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`. This entry
  gives it evidence at a second tilt and a fourth measurement of the backlash,
  and it does not move the verdict.
- No requirement changed state. No constant was changed, no rule was relaxed,
  and no result was averaged with another.

## Next, in order

1. **Move the board back to about 0.55 m, squarely**, and repeat all three
   sessions — tilt-zero control, tilt-20 development, tilt-20 held-out. This is
   the one thing outstanding and it needs a person: driving the rover 15 cm
   toward its own calibration reference is not something to attempt without eyes
   on it. If the gain error comes back inside 1.0% at both tilts, the envelope
   extends to tilt 20 and M0's criterion 10 has an envelope that covers where
   the rover actually looks.
2. If it does not, the rule itself is the thing to reconsider — **before** the
   next capture, not after. A percentage bound on a term that scatters by a
   point is the wrong shape; an absolute bound argued from the 1.5-degree
   bearing budget would be defensible, since −1.42% at pan 20 is 0.28 degrees of
   pointing error. That is a decision to declare in advance and validate on new
   data, never one to reach by looking at a failed result.
3. Meanwhile the deployed envelope stays at pan ±20 with no tilt condition, for
   the reason given in
   [the capture-gate entry](2026-09-07-capture-state-gates.md): gating on tilt
   would withhold the direction from nine looks in ten and satisfy criterion 10
   by abstaining, which that criterion forbids.
