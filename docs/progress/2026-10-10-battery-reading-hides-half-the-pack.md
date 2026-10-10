# The battery reading hides about half the pack below its 10%

**The rover's charge reading is far too pessimistic at the bottom and a little
pessimistic at the top.** In M3 session 16 on 2026-10-10 it read 10% or less from
13:10 and the rover went on driving for at least another 41 minutes, through a
reading of 0%. It switched itself off after 13:51, at about 8.9 V by the owner's
reading of the console. A charge from
full down to that same 10% had lasted about 40 minutes of session driving on
2026-10-08. So something like half of a charge sits below what the reading calls
10%. That is an estimate from two sessions and is not a measurement of the curve;
measuring the curve is [the plan](../plans/battery-charge-curve.md). At the other
end, the highest readings at the start of runs after a charge are 84 to 88%,
which is why the console has seldom shown more than 90%.

Nothing was changed in the table. The reading is now given in whole points rather
than steps of five. The daemon also keeps a record of the pack voltage
([`battery_log.py`](../../rover_daemon/battery_log.py)). A refit needs one, and
none existed before.

## Where the numbers come from

There was no record of the pack voltage over time. The autonomy recorder notes
the voltage at each decision it records, so `~/.ugv/autonomy/episodes.db` held
3,753 readings from 2026-09-08 to 2026-10-10, taken during runs only, and mostly
while driving or just after. They say nothing about charging, and the time from
a full charge to a run's start is unknown.

| What | Voltage | The table's reading |
|---|---|---|
| The highest readings at the start of runs, after a charge | 12.01-12.12 V | 84-88% |
| Session 16 first reads 10% or less (13:10:14) | 10.94 V | 10% |
| Session 16 crosses what used to round to 5% (13:17:49) | 10.66 V | 8% |
| Session 16 first reads 2% (13:44:16) | 9.64 V | 2% |
| The last reading, while driving (13:51:09) | 9.33 V | 1% |
| Switched off (the owner, from the console) | about 8.9 V | 0% |

Session 16 started at 13:06 on a part charge. That was after a brownout, and the
pack read 20% at rest when the rover came back up (the entry for M3 sessions
14 to 17 has the brownout). On 2026-10-08 the
session from 15:10 to 15:50 went from 11.91 V to 10.95 V. That is the comparison
behind "about half".

## Why the table reads this way

The table in `board_link.py` takes its two ends from Waveshare (4.20 V a cell is
full, 3.00 V empty), but nobody ever measured its shape on this pack, and the
shape is the trouble. It puts 10% at 3.68 V a cell and leaves only 10 points
between that and 3.00 V. The 18650 cells in this pack keep much more of their
charge below 3.7 V than that. The empty end is about right: the rover switched
off at about 8.9 V under load, 2.97 V a cell, against the table's 3.00. The table is also applied to a voltage taken under
the rover's own load and never at true rest: the Orin module alone draws 6 to
7.5 W at its input ([the OAK USB drops](2026-10-02-oak-usb-drops.md)), so even a
rover standing still reads lower than the table's resting chart expects. 100% is
12.6 V, which the pack reaches only on the charger.

The voltage thresholds behind the words "low" (11.2 V) and "critical" (10.8 V)
came off the same table, and so did the owner's floor. Session 16 drove for more
than half an hour while the battery tool described the pack as nearly flat and
needing a charge now.

## Brownouts are not explained by a low pack

The rover drove at 9.3-9.6 V under load without browning out. The two brownouts
on record came at much higher voltages: on 2026-10-09 at 18:37, at "25%
recovering", and on 2026-10-10 at 13:02, after a short charge that read 80%. A
pack sagging below what the Orin needs does not fit that, at least not on the
pack voltage alone. The new record keeps the lowest reading in every five seconds
and is synced to disk as it goes, so the next brownout will show what the pack
did in the seconds before it.

## What changed

- **Whole points** (`_battery_percent`). Nothing downstream needed steps of five.
  The floor checks compare with `<=`, and the console and the voice model show or
  say the number. "5%" used to cover everything from 2.5 to 7.5.
- **The voltage record**, written by the daemon on the rover to
  `~/.ugv/battery/boot-<id>.csv`. It is described in the
  [daemon's README](../../rover_daemon/README.md#battery-and-board-telemetry),
  and the [plan](../plans/battery-charge-curve.md) says what to record and how
  to fit it.
- [R-CTL-11](../requirements/control.md#r-ctl-11) is proposed: the percentage is
  the share of a charge left, read standing still. The current table fails it by
  tens of points at the bottom.
