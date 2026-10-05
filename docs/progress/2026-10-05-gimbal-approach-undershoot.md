# Five degrees takes up the gimbal's slack as well as thirty did

**Every gimbal move now drops 5 degrees below its target and comes back up,
instead of 30, and lands where the 30-degree approach landed.** The pan servo's
backlash measured 1.8 degrees. A landing from 2 degrees below matched the 30-degree
landing to within 0.17 degrees at rest. One degree left the camera 0.85 short,
about half the backlash. At the +20 pan limit, 3 and 4 degrees landed 0.23 short,
and 5 landed within 0.07. Five passes at every position tested, so the pan
calibration of 2026-09-07 still describes where the camera ends up. Deployed at
0d309d5 and checked on the rover: the daemon's own centring lands within 0.09 to
0.15 degrees of a staged 30-degree landing.

## How it was measured

[gimbal_approach.py](../../usb_cameras/gimbal_approach.py), committed with its
pass rule before the first trial (0f049be). The rover was parked at the charger,
on charge, with the camera on the dining-room wall. Each landing started at pan
+30, so the servo began seated on the far side of its play, which is the worst
case. It dropped to the target minus the undershoot, came up to the target,
settled 1.5 s and photographed. Where it landed is the horizontal shift of the
middle of the picture against the first 30-degree landing, by phase correlation.
That was turned into degrees by two 30-degree landings 4 degrees apart (4.87
pixels a degree). The rule: an undershoot passes when every landing is within 0.2
degrees of the mean 30-degree landing at the same position.

| Undershoot (deg) | At rest, worst of 3 | At -20, worst of 2 | At +20, worst of 2 |
|---:|---:|---:|---:|
| 30 | 0.09 (the reference's own spread) | 0.00 | 0.00 |
| 15 | 0.08 | | |
| 8 | 0.11 | | |
| 5 | 0.14 | 0.02 | 0.07 |
| 4 | | 0.06 | 0.24 (3 runs), fails |
| 3 | 0.14 | 0.08 | 0.23, fails |
| 2 | 0.17 | | |
| 1 | 0.86, fails | | |
| 0 (from above) | 1.83, the backlash | | |

The rule's own answer was twice the smallest passing value at rest: 4. That was
fixed before the edges were measured, and the 4 was then tested at both limits
afterwards. It fails at +20, so the deployed value is 5, the smallest passing
everywhere tested. Smaller undershoots land slightly short, by a tenth of a
degree or so, with a small drift of about 0.15 degrees between rounds of the same
sitting.

## What it changes

A move is a 5-degree flick, not a 30-degree swing. Ordinary aimed looks within 25
degrees of straight ahead do not move the gimbal at all
([session 2](2026-10-05-m3-session-2.md)). The calibration bench,
`calibrate_gimbal.py`, still uses 30. It needs no change, because the two land
together. The frames and summary are in `captures/gimbal-approach-2026-10-05/`.
