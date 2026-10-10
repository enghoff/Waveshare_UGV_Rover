# A daemon restart turned the parked rover round: the gyro's restarted total read as a turn

**Two daemon deploys on 2026-10-10 left the rover on the charger believing it
faced the wall while it faced the room: 170 degrees out at 18:05 and 158 at
18:18.** The cause was the morning's odometry change, mine. The daemon keeps the
gyro's running total from when it starts and begins it again at zero when it
restarts, so the first reading after a restart differs from the last one before
it by every degree of gyro offset since the daemon last started: thousands of
degrees at once. That always reached `base_node`, which used to throw it away as
the interval of a still rover. Since 5b24d72, which counts a gyro rate 3 degrees
a second above its offset as a turn so that a spin nobody commanded is not lost,
it went into the heading. Reproduced in a test, fixed (6d722c4), and seen fixed
on the rover: a daemon restart afterwards left the heading where it was.
[R-NAV-13](../requirements/navigation.md#r-nav-13) was broken from the morning
until then, for a rover whose daemon restarted, and holds again.

## What the owner saw

The owner found the rover's heading wrong on the charger, turned it to match the
map and rebooted it twice, at about 17:50 and 18:08; both resets were the owner's,
not crashes, though for an hour they were taken for crashes, because the battery
record showed no power cut. The second time, the restore put it right: at 18:14
the scan agreed to 3 cm. Then the daemon was restarted for a deploy at 18:17,
and at 18:19 navigation's drift check had the rover 28 cm and 158 degrees from
where the scan fits. The first time was the same: daemon restarts at 18:05,
the drift check 170 degrees out at 18:07. The flip before the owner's first
reboot was probably the same thing, but its log tail was lost to the power-off.

## The fault, reproduced

`rover_daemon/board_link.py` integrates the gyro into `gz_lsb_s` and counts
`samples` from zero at start. `base_node.integrate` took each interval's turn as
the difference in `gz_lsb_s`, kept its last value across a reconnection to the
board bridge, and never noticed the totals had started again.
`test_odometry.py` drives the real `tick` and `integrate` over 45 s of a parked
rover whose daemon has been running two hours, then restarts the daemon: the
heading moved -101.9 degrees on the deployed code, by either route in, and 0.0
after the fix. A new connection to the bridge now starts the totals again from
the next sample, the bridge client drops its last record with the connection,
and a sample count that goes backwards marks the interval broken. Navigation's
24 odometry checks pass on the rover.

On the rover, after deploying it at 18:24, the daemon was restarted at 18:30:42.
Navigation's full-circle measurement of how far the rover's belief was from the
scan read 28 cm and -157.5 degrees before the restart and the same after. The
console's refit cannot reach a half-turn, since it searches 45 degrees either
side of the belief, so at the owner's yes the same full-circle search was
applied (the daemon's `refit` with `window_deg` 180): the rover moved 26 cm and
-157.3 degrees onto the map, 96% of the scan on a wall against 54% before, and
the next local measurement agreed to 3 cm and 0 degrees.

## The change watch

The change watch deployed that afternoon (38ade1f) counted scans while navigation
called the map settled, and the map stayed settled through the first flip: at
18:06 it logged two furniture-sized changes in the charger room that were the
pose. It now refuses a scan whose hits fall on floor it has already seen clear
and pauses while the drift check places the rover elsewhere (2306f7b); replayed
0.6 m and 20 degrees out, a whole run's scans were refused and nothing reported.
Its files from that hour are kept beside the clean ones on the rover as
`*.misplaced-20261010-1820`.

## Still open

A drift check that finds the rover half a turn out with 98% of the scan fitting
says so and does nothing, by design ([R-NAV-3](../requirements/navigation.md#r-nav-3)),
and the console's refit cannot act on it either. A person pressing refit after
a report like that should get the search that found it.
