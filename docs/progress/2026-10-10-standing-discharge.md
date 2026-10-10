# A standing discharge measured the pack: 1 h 56 min from the charger to switching off, and the reading now comes off it

**On 2026-10-10 the rover stood off the charger with its wheels still from
19:17:55 until it switched itself off at 21:13:47, and the battery reading is
now a table read off that discharge.** The straight line it replaces read up to
14 points high on it, and the table before that up to 42 points low. The pack
simply ran out: the voltage fell steadily to 8.89 V, which the console showed
as its last reading, and dipped to 8.82 V in the final second. This is the
standing discharge [the plan](../plans/battery-charge-curve.md) asked for. It
fits the table to one discharge, so it meets
[R-CTL-11](../requirements/control.md#r-ctl-11) by construction; whether the
table holds for a driven discharge is still to be shown, and the requirement
stays proposed.

## The discharge

From the daemon's record, `boot-02a61b8c.csv` on the rover, one row a second:

- The charger came off at 19:17:55, when the voltage stepped from 12.29 to
  12.12 V with the wheels still. It read 12.00 V a minute later.
- The Orin drew 6.83 to 6.85 W on average in each fifth of the discharge, so
  the load held. Everything the Orin's own monitor does not see (the lidar, the
  board, the servos) is not measured, and nothing in the record says it changed.
- The daemon was restarted once, at 19:19:14, which leaves 17 s without rows.
  The rover's draw went on through it, so time is counted across the gap.
- Over the last six minutes the voltage fell from 9.19 to 8.89 V, and it
  read 8.88 to 8.91 V for the last minute. The last row, at 21:13:47, has a
  mean of 8.85 V and a lowest reading of 8.82 V. There was no earlier dip.

With the load that steady, the share of the time still to go is the share of
what the pack has left. Each point of the table is the median of the mean
voltage over two minutes around that share of the time; one-minute and
four-minute medians agree with it to 0.02 V at every point but the top, where
the pack is still settling off the charger and they differ by 0.04 V.

| Left | Volts | Left | Volts | Left | Volts |
|---|---|---|---|---|---|
| 100% | 12.06 | 65% | 11.32 | 30% | 10.32 |
| 95% | 11.95 | 60% | 11.20 | 25% | 10.20 |
| 90% | 11.90 | 55% | 11.12 | 20% | 10.01 |
| 85% | 11.85 | 50% | 11.00 | 15% | 9.76 |
| 80% | 11.75 | 45% | 10.86 | 10% | 9.47 |
| 75% | 11.59 | 40% | 10.71 | 5% | 9.19 |
| 70% | 11.44 | 35% | 10.54 | 0% | 8.89 |

Single one-second readings put through the table land within 2 points of the
share of time left nine times in ten, and never more than 5 points away.

## How the readings before it compare

| | Furthest from the share left | Where |
|---|---|---|
| The straight line (8.85 to 12.35 V), 19:18 until now | 14 points high | 22% left, read as 36% |
| The table on Waveshare's ends, until 19:18 | 42 points low | 53% left, read as 11% |
| The fitted table | 2 points | 98% left |

The line's "low" (9.55 V) came at 11% left and its "critical" (9.03 V) at 2%.
On the fitted table they are 10.01 V (20%) and 9.19 V (5%), the meanings they
have always been meant to have.

## On the charger

The reading is no use while charging. In the charge recorded earlier the same
day, from 10.95 V at 15:15 to a plateau of 12.30-12.33 V by about 17:15, the
voltage passed the table's 100% a quarter of an hour in. "Full" is therefore
the charger's plateau, 12.30 V, rather than the table's top; off the charger
the pack never reads it.

## What changed

The table, "low", "critical" and "full" in the daemon's board link, its tests,
and the account of them in the [daemon's README](../../rover_daemon/README.md#battery-and-board-telemetry).
The owner's 5% floor is unchanged; on the fitted table it is about six minutes
of standing still before the pack gives out, and setting it again is the
owner's.
