# The envelope reaches the tilt the rover actually uses, by 0.08 of a point

Moving the board from 0.70 m to 0.47 m settled it. The pan campaign now passes
its declared rule at gimbal tilt +20, where 1828 of this rover's 2165 looks are
taken, so the operating envelope covers the state the rover is actually in
instead of one it almost never uses.

Two things qualify that. The margin is **0.08 of a percentage point**, and the
development session at the same tilt and the same board read just outside the
limit — so the true value sits essentially *on* the rule rather than inside it.
And the reason it now passes is that the earlier failures were the measurement
rather than the servo: the board's distance decides whether this bench can
certify anything at all, which the runbook described as "setup guidance, not a
calibration input."

This entry follows [the failed
attempt](2026-09-07-gimbal-tilt20-campaign.md) and confirms the diagnosis that
entry proposed. Both stand.

## Every session, oldest first

Nothing moved within either group of three. The board was moved once, between
`tilt20-held-out-01` and `tilt0-control-02`.

| session | tilt | board | ascending gain | residual p95 | backlash | duplicate median | duplicate worst | optics RMS | verdict |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| `held-out-01` | 0 | 0.538 m | −0.34% | 0.297 | −1.71 | 0.074 | 0.356 | 0.173 | pass |
| `tilt0-control-01` | 0 | 0.701 m | −1.07% | 0.387 | −1.59 | 0.141 | 0.860 | 0.194 | fail |
| `tilt20-dev` | +20 | 0.704 m | −0.63% | 0.348 | −1.63 | 0.196 | 0.436 | 0.206 | development |
| `tilt20-held-out-01` | +20 | 0.707 m | −1.42% | 0.439 | −1.70 | 0.135 | 0.716 | 0.209 | fail |
| `tilt0-control-02` | 0 | 0.470 m | **+0.48%** | 0.226 | −1.72 | 0.044 | 0.179 | 0.184 | **pass** |
| `tilt20-dev-02` | +20 | 0.478 m | −1.04% | 0.223 | −1.68 | 0.042 | 0.165 | 0.188 | development |
| `tilt20-held-out-02` | +20 | 0.479 m | **−0.92%** | 0.193 | −1.70 | 0.047 | 0.184 | 0.185 | **pass** |

Degrees except where marked. The rule was declared before any of these and none
of them changed it: reference gate at stationary duplicate median ≤ 0.25 and p95
≤ 0.75 degrees, then ascending-only gain error ≤ 1.0% with absolute residual p95
≤ 0.5 degrees.

## The board's distance is a calibration input

| | 0.47 m | 0.54 m | 0.70 m |
|---|---:|---:|---:|
| stationary duplicate, median | 0.042–0.047 | 0.074 | 0.135–0.196 |
| stationary duplicate, worst | 0.184 | 0.356 | 0.860 |
| ascending residual p95 | 0.193–0.226 | 0.297 | 0.348–0.439 |

Every measure of repeatability improves monotonically as the board comes closer,
by a factor of three to four across the range. The board subtends 28.6 degrees of
the frame at 0.47 m, 25.1 at 0.538 and 19.3 at 0.707; a target that is already small in frame
determines its own pose poorly, which is the same ill-conditioning that made the
mount measurement difficult earlier in the day.

**So the two failures in the previous entry were noise, not the servo.** At
0.70 m the ascending gain error scattered 0.79 points between two sessions at one
tilt; at 0.47 m the same comparison scatters 0.12. The rule was never the wrong
shape — the setup was too far away to test it.

The runbook said 0.6 to 0.8 m and called the distance setup guidance rather than
a calibration input. On this evidence that is wrong, and it has been corrected to
0.45 to 0.55 m with the reason attached.

## The pan gain really does depend on tilt

With the setup well conditioned and repeatability tight at both tilts, a
difference appears that the noisy sessions could not have shown:

| tilt | ascending gain error | duplicate median |
|---:|---:|---:|
| 0 | +0.48% | 0.044 |
| +20 | −1.04%, −0.92% | 0.042, 0.047 |

About **1.5 percentage points between level and 20 degrees up**, against 0.12
points of session-to-session scatter. That is a real effect, not a draw.

It is not an artefact of how the bench measures the angle. The rotation axis it
extracts in the board frame is the same at both tilts to within 0.09 degrees, so
it is projecting the same rotation both times; a geometric cosine factor would
have to be 6.03% to explain a tilt of 20 degrees and would show up as a moved
axis. No mechanism is offered here, and none is needed for the verdict — the
point is that the gain must be measured at the tilt it will be used at, which is
what `--tilt` now makes possible.

**What it means if the envelope is ever widened.** At pan 20 a 0.92% gain error
is 0.18 degrees of pointing, which is nothing against the 1.5 degrees the
resolver is told to expect. At pan 40 the tilt difference alone would be
0.6 degrees. Anyone extending the pan range has to re-measure at every tilt that
matters, not interpolate.

## The backlash, across all seven sessions

−1.59, −1.63, −1.68, −1.70, −1.70, −1.71, −1.72 degrees. A spread of **0.13
degrees across two tilts, three board distances and seven sessions.** It is by
far the most stable thing this bench measures, and it is what the gate deployed
earlier today rests on: an angle reached from the descending side is charged 2.3
degrees of bearing error, taken from the 1.19-to-2.23-degree spread of the
approaches. That charge now has tilt-20 evidence behind it where this morning it
had none.

## What this does and does not change in the running rover

Nothing was deployed for this. The gate already in service withholds a direction
outside commanded pan ±20 and applies no tilt condition, and that was the right
shape before this measurement and remains so after it: the pan range sampled is
still ±20, and tilt is now validated at the two values that account for 98.8% of
the rover's looks rather than assumed.

No constant was changed. The candidate rule is still "leave the pan gain
unchanged and finish every placement from the ascending direction", and correcting
a 0.92% gain would buy 0.18 degrees at the envelope edge against a 1.5-degree
budget.

## Requirements

- [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`, and its
  record has been corrected where this work made it stale. What it asks for is
  that a *recorded bearing* be as accurate as the resolver is told to expect, and
  no bearing has been measured since any of this landed. The mechanism is now in
  place — a declared envelope at both working tilts, unsupported conditions
  refused, the direction-dependent error charged rather than ignored — but the
  measurement that condemned it was taken on a driven recording, and only another
  driven recording can lift it.
- No requirement changed state.

## Next, in order

1. **The driven acceptance recording.** This was the last calibration blocking
   it. Clear the semantic store, keep the map, put out targets at measured
   distances, and drive.
2. Restart navigation once during that session, for
   [R-WS-16](../requirements/world-state.md#r-ws-16)'s hardware demonstration.
3. The third-distance mount capture with the board turned 20 to 30 degrees off
   face-on, still outstanding from
   [the mount entry](2026-09-07-p0-oak-mount.md). **This position does not
   qualify.** The mount sets were taken at 0.555 m and 0.686 m and the third has
   to be at least 0.10 m from both; the board now reads 0.470 m from the gimbal
   camera, which is 0.085 m from the development set and so 15 mm short. A
   further 20 to 30 mm of clearance would do it, and the turn off face-on
   matters more than the distance does.
