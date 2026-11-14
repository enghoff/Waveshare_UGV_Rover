# A rover picked up and turned was taken for a faulty gyro, twice in one evening, and could not turn afterwards

On 2026-11-14 the owner reported that turns failed. The navigation log has the
base calling its gyro faulty at 19:34:48 ("turning at -39 deg/s for 3 s while
neither track moved") and, after the full power cycle that message asks for,
again at 19:42:41 (-31 deg/s). Before the second, every turn asked for arrived:
sixteen between 19:40 and 19:42. After each fault, none did. A 90-degree turn at 19:37:26 and
a 105-degree one at 19:43:13 ran their whole eight seconds and ended "timed
out"; turns of 45 and 15 degrees at 19:45 were stopped by hand after 5.4 and
1.7 s.

**The gyro was not faulty. The rover had been picked up and turned.**

## What shows it

- **The gyro at rest.** Read from the board bridge for ten seconds at about 19:48,
  with the fault still standing and the rover parked: 63.2 LSB-seconds in
  10.04 s, 0.39 deg/s, against an offset learned at 0.36 before the fault. The
  four faulty gyros on record read +150, exactly 0, -2,063 and +210.6 deg/s,
  and each did so until the power was cut.
- **The lidar.** The costmap's live layer logs how the scan lies on the map
  whenever that changes. It had said nothing for 16 s when the gyro began to
  turn at 19:42:38, and then changed three times in two seconds (65 returns on
  walls, then 38). At 19:34:45 it went on changing for eleven seconds after
  the fault, between 68 and 177 returns. A parked rover whose gyro lies shows
  none of that: its believed pose is held and its scan does not change.
- **What followed.** Nine seconds after the second fault the rover was refitted
  from the console, and the scan put it 75 cm and 46.5 degrees from where it
  believed it was.

The scan log would have settled it outright and was not running: the map was
held as "position unconfirmed" at both moments, and the log stops under a hold
([R-NAV-17](../requirements/navigation.md#r-nav-17)).

## Why the turns failed

The base holds a turn that only the gyro reports until a track's count moves,
and after three seconds of it stops believing the gyro and holds the heading
([R-SAFE-17](../requirements/safety.md#r-safe-17)). That verdict lasted until
navigation was restarted. A turn ends when the heading says it has turned far
enough, so with the heading held it never ends: the rover spins until the turn's
time runs out. Navigation refused autonomous runs under the fault but
dispatched a turn asked for from the console.

## Reproduced

Through the real `base_node.integrate`, with the deployed code, on the rover: a
rover parked for 30 s, then four seconds of the gyro at -31 deg/s with neither
track's count moving. The fault is the evening's own sentence, word for word.
It still stands after a minute of the gyro at its offset, and a 90-degree turn
commanded next, with the tracks moving, moves the heading 0.0 degrees.

## What was changed

- **The verdict ends when the gyro reads still.** Five seconds within 3 deg/s
  of the offset, nothing commanded and neither track moving
  (`base_node.recover`, `GYRO_REST_S`). The heading then takes everything the
  gyro turned since the turn began, since a refit searches only 45 degrees
  either side of the believed heading. A gyro that stops updating meanwhile is
  a faulty one and stays so.
- **The map is held from the moment of the fault** as off the map, by the same
  latch as a drive by hand that left it
  ([R-NAV-16](../requirements/navigation.md#r-nav-16)): given back when ten
  scans running lie on the map and the wide search agrees, or after a refit.
- **A turn, a drive to a point and an exploration are refused while a fault
  stands**, with the base's reason. A straight drive is not.

In the same replay the fault now ends 5 s after the rover is set down, the
heading has moved -124 degrees, and the 90-degree turn reads 90. The 210.6
deg/s boot of 2026-10-14 is still faulty after a minute, and the frozen gyro of
2026-10-05 still is. The navigation selftest passes on the rover, 941 checks.

## Not shown

No rover has been picked up since the change. The proof on the floor is the
owner lifting it, turning it for four seconds or more and setting it down: the
console's fault line should appear in the air and go five seconds after it is
down, with "off the map" until the scan agrees. An intermittent gyro fault,
which none on record has been, would have its turn taken into the heading each
time it came to rest; the map hold is what stands between that and the map.
