# Fitting the charge reading to this pack

**Status:** the table is fitted and on the rover; what is left is checking it
against driving. A standing discharge on 2026-10-10, from the charger coming off
at 19:17:55 to the rover switching itself off at 21:13:47, gave the table of volts
against charge left that the daemon now reads from
([the measurement](../progress/2026-10-10-standing-discharge.md); how the reading
works now is in the [daemon's README](../../rover_daemon/README.md#battery-and-board-telemetry)).
It replaced a temporary straight line from 8.85 to 12.35 V, which read up to 13
points high on that discharge.

This plan settles [R-CTL-11](../requirements/control.md#r-ctl-11): read standing
still, the battery's percentage is the share of a charge that is left. The table
was fitted from one standing discharge, so it describes that discharge by
construction; whether it describes a driven one is still open.

## What is recorded

The daemon writes one row every second to
`~/.ugv/battery/boot-<id>.csv` on the rover, a new file for each boot. Each row
holds the lowest, mean and highest pack voltage in the interval, how far the
wheels turned, and the Orin's own input power. The columns are described in
[battery_log.py](../../rover_daemon/battery_log.py). Nothing has to be started:
the record runs whenever the daemon does, on the charger too, and keeps 180
days. Copy the files back over ssh (`scp orin:.ugv/battery/*.csv` into a scratch
folder). They are runtime data and stay out of Git
([R-PLAT-11](../requirements/platform.md#r-plat-11)).

## Checking the table against driving

What is still wanted is **sessions driven to the end**, recorded as they
happen; they come for free whenever a supervised session runs until the rover
switches itself off. For every stretch of two minutes or more with no wheel
ticks in such a session, compare the table's reading with the share of the
session's driving time still to come. R-CTL-11 is met when every such reading
is within 10 points over at least two sessions. A miss of more than 10 points
means the standing discharge does not describe a driven one, and the next step
is to find out why rather than to adjust the table by hand.

A second standing discharge would say how far the table repeats, and the pack
will drift as it ages. Neither is urgent; the first is free whenever the rover
is left off the charger with nothing to do.

## What changes with the table

- **The owner's floor needs setting again.** It is 5% at rest, and on the fitted
  table 5% (9.19 V) is about six minutes of standing still before the pack gives
  out, less while driving. On the straight line before it, 5% was 9.03 V, which
  the fitted table calls 2%. The owner sets the floor against the fitted table.
  It is checked in `experiments/m3_trials/` and `experiments/m4/start_trial.py`.

## Later, and not agreed

- **On the charger the reading means nothing.** The voltage passed the table's
  100% a quarter of an hour into a two-hour charge from half. The daemon cannot
  tell that a charger is on; the record can (the voltage steps up with the
  wheels still), and so could the board's own current reading if it has one.
- **A reading taken while driving sags by tens of points.** The console could
  show the last standing reading while the wheels turn, or correct for the
  measured sag. Sessions driven to the end say how big the sag is at each level
  of charge.
- **The brownouts at high voltage.** The standing discharge ended with the pack
  simply running out: it fell steadily to 8.89 V and dropped to 8.82 V in its
  last second. That says nothing about the two brownouts at 25% and 80% on
  2026-10-09 and 10-10; the record keeps the lowest voltage in every second, so
  the next one will show whether the pack dipped before it.
