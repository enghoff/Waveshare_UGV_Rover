# Fitting the charge reading to this pack

**Status:** the voltage record has run on the rover since a435582 was deployed on
2026-10-10, at about 14:06. It wrote a row every five seconds (about 100
readings each) until 18:18, and a row every second (about 20) since. It has one whole
charge (recording 1 below): from 10.95 V at 15:15 to a plateau of 12.30-12.33 V
on the charger by about 17:15, which the owner reads as full at about 12.35 V.
So the daemon's "full" (12.45 V) is never reached. No standing discharge has been
recorded yet. The table is unchanged,
and on 2026-10-10 it was shown to call about half a charge "10% or less"
([the finding](../progress/2026-10-10-battery-reading-hides-half-the-pack.md)).

This plan settles [R-CTL-11](../requirements/control.md#r-ctl-11): read standing
still, the battery's percentage is the share of a charge that is left.

## What is wrong now

The rover's percentage comes from a table of volts a cell against charge left
(`BATTERY_CURVE` in [board_link.py](../../rover_daemon/board_link.py)). Its two
ends are Waveshare's, but its shape was never measured on this pack, and the
reading it is applied to is taken under the rover's own load. Runs that start
after a charge read 84 to 88%. In M3 session 16 the rover drove for at least 41
minutes after the reading reached 10%, kept going through 0%, and switched off
at about 8.9 V under load. So the table's empty end, 9.0 V, is about right, and
it is the shape above it that needs fitting.

## What gets recorded

The daemon writes one row every second to
`~/.ugv/battery/boot-<id>.csv` on the rover, a new file for each boot. Each row
holds the lowest, mean and highest pack voltage in the interval, how far the
wheels turned, and the Orin's own input power. The columns are described in
[battery_log.py](../../rover_daemon/battery_log.py). Nothing has to be started:
the record runs whenever the daemon does, on the charger too, and keeps 180
days.

Three kinds of recording are wanted, and the first and third come for free.

1. **A charge from flat**, standing. The rover is put on charge after a session
   anyway. This gives the voltage a full pack reads under the rover's standing
   load once the charger comes off, and how long a charge takes.
2. **A standing discharge**: from a full charge to the rover switching itself
   off, untethered, with nothing driving. This is the measurement the table is
   fitted from. With a steady load, the share of time still to go is the share
   of charge still left, so the voltage at each moment can be read against it
   directly. Expect two to three hours: the pack is roughly 30 Wh, and the rover
   draws roughly 10 to 12 W standing. Both figures are estimates that this
   recording replaces. The rover does not move, so it needs no handover for
   motion, but the owner has to unplug it, so ask first. Leave the OAK's power
   alone (it switches itself off 30 s after a drive) and let nothing open a run.
3. **Sessions driven to the end**, recorded as they happen. They give the voltage
   drop under drive load at each level of charge, and how much driving a charge
   holds, which is what a floor is really for.

Copy the files back over ssh (`scp orin:.ugv/battery/*.csv` into a scratch
folder). They are runtime data and stay out of Git
([R-PLAT-11](../requirements/platform.md#r-plat-11)).

## How the table is fitted

From recording 2:

- Find the moment the charger came off. The voltage steps down with the wheels
  still, and `host_mw` carries on unchanged. Find the last row before the file
  ends without a `stop`. Everything between is the discharge.
- Check that `host_mw` held roughly steady. If it did, the share left at each row
  is the time still to go over the whole discharge's time. If it wandered, weight
  each interval by its `host_mw` plus a constant for everything the INA3221 does
  not see (the lidar, the board, the servos), and say which was done.
- Smooth `v_mean` with a two-minute median and read off the voltage at every 5%
  from 0 to 100. Write it into `BATTERY_CURVE` in volts a cell, in place of the
  current table, keeping the comment honest about where each point came from.

Then check it against recording 3. For every stretch of two minutes or more with
no wheel ticks in a session driven to switch-off, compare the fitted reading
with the share of the session's driving time still to come. R-CTL-11 is met when
every such reading is within 10 points over at least two sessions. A miss of
more than 10 points means the standing discharge does not describe a driven one,
and the next step is to find out why rather than to adjust the table by hand.

## What changes with the table

- **The owner's floor needs setting again.** It is 5% at rest today, which on the
  current table is still a long way from empty. On a fitted table 5% means about
  5% of a charge, and the drive home has to come out of it. The owner sets the
  new floor against the fitted curve. The floor is checked in
  `experiments/m3_trials/` and `experiments/m4/start_trial.py`.
- **"Full", "low" and "critical"** (`BATTERY_FULL_V`, `BATTERY_LOW_V`,
  `BATTERY_CRITICAL_V`) were read off the same table, and are refitted with it.
- **The numbers people quote**: `experiments/m3_trials/battery_now.py` says
  "11.04 V is 10%". That changes with the table.

## Later, and not agreed

- A reading taken while driving sags by tens of points. The console could show
  the last standing reading while the wheels turn, or correct for the measured
  sag. Recording 3 says how big the sag is at each level of charge.
- The brownouts. The record keeps the lowest voltage in every second, so
  the next brownout will show whether the pack dipped before it. On the evidence
  so far it did not get low: the rover has driven at 9.3 V without one.
