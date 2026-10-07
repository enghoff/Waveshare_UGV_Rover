# The identity trial reached its first viewpoint and stopped on a 12-degree heading

**The prepared visibility trial ran on the rover for the first time, and stopped
itself at its first viewpoint as it is written to.** Navigation believed the
rover faced 47 degrees, while a scan fitted against the map said 59. No look was
taken from either viewpoint. R-WS-13 stays open, and R-WS-17 and R-WS-18 stay
proposed. The trial is the one prepared on 2026-10-06
([the blocker](2026-10-06-direct-trial-blocked.md),
[the run plan](../plans/entity-evidence-drive.md)), unchanged apart from the fix
below.

## Before it

- **A freshly booted rover was refused as "in use".** Navigation reports no
  motor command until its first move after a boot, and the preflight read that
  empty reading as a rover in motion. The board's heartbeat stops a base that
  hears nothing, so the empty reading is as still as a zero. The fix is 083491a,
  with two tests: a rover not driven since boot is at rest, and a motor command
  still held is not.
- **The starting heading agreed.** Navigation read 160.1 degrees and the scan
  fit 159.6, 2 cm apart, score 0.996. The owner confirmed the rover stood where
  the map showed it. The 17.5-degree disagreement of 2026-10-06 is gone, and
  navigation now reads what the scan said that day.

## What happened

Session `visibility-independent-20261007-1`, map `7da19bef3888`.

| | |
|---|---|
| leg | HOME (-17.137, -15.795, 160.1°) to B (-19.05, -14.85), view heading 55°, 2.50 m planned |
| arrived | (-19.096, -14.986), navigation's heading 47.2° |
| scan fit at arrival | 59.1°, 6 cm away; score 0.993 there against 0.487 at navigation's pose |
| outcome | 12.0° against the trial's 10° limit, so it stopped; STOP verified, 13.7 s after the start |

The recording shows where the error came from. Through the final turn, the
gyro-integrated odometry turned 121.3 degrees, and the map heading followed it
(122.8). The scan says the rover ended at 59 degrees, so it actually turned
about 111. The gyro over-counted this turn by about 10 degrees, or 9%.

Standing still, navigation kept the error: it was still 11.5 degrees a minute
later. A refit was refused: "the mapper matched it against its own graph and
kept the rover where it was". After the next 0.7 m of driving it was within 2.0
degrees.

## What it means

- **The rover was in fact facing the view.** It faced 59 degrees against the 55
  asked for, inside the trial's 5-degree view tolerance. The trial stopped
  because navigation and the scan disagreed by more than 10 degrees, not because
  the view was wrong.
- **A look taken there would have had the right direction.** Since 2026-10-01
  a still look corrects its heading by the same scan fit
  ([the heading check](../../world_state/headingcheck.py)). So this 12 degrees
  is a problem for the trial's admission and for navigation. It is not evidence
  that targeted looks are misfiled because of heading. That points the
  [targeted-look misses](2026-10-06-targeted-look-provenance.md) more at
  identity matching than at heading, though that stays unproven until the trial
  gets its views.
- **The turn error is not one stable number.** On 2026-09-04 the map moved
  further than odometry admitted. On 2026-10-01 the mapper over-counted turns by
  about 7%. Today the gyro over-counted by 9%. Not investigated further.

## Next

The trial's arrival admission should accept a trusted scan fit that beats
navigation's own pose by a wide margin, judge the view heading from the fit, and
record the disagreement. The alternative is a short straight move to let the
mapper correct itself before admitting. The earlier warning against loosening the
limit was about not knowing which heading was right. Here the fit is unambiguous,
0.99 against 0.49.

Evidence: `captures/visibility-independent-20261007-1/`, with the trial's own
hashes verified after the copy.
