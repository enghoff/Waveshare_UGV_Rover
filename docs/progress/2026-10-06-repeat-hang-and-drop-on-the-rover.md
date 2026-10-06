# A repeated request moves the rover once; a hung or cut-off executive loses it within a third of a metre

**On the rover, a command sent twice moved it once, and a hung executive and a
lost connection each let the rover go no more than 0.31 m after the permission
ran out.** Each was tried once, on a 4.4 m leg in the charger room with the owner
present, at the 0.40-0.45 m/s navigation drives at. That speed is faster than the
0.31 m/s of the 2026-10-02 trials. The slowest stop took 0.31 m, one centimetre
more than the 0.5 m safe-area margin left after the body. So the margin is now
0.6 m, set from these stops as the
[2026-10-02 decision](../decisions/p0-measures-the-hardware.md) asks.
[R-AUT-11](../requirements/autonomy.md#r-aut-11) and
[R-SAFE-12](../requirements/safety.md#r-safe-12) are settled. The console's stop
at speed was tried twice and never pressed, so it is still owed.

## How it was done

A stand-in executive opened its own fenced run. The script, `m3_trials.py`,
and every sample it took are kept in `captures/m3-trials-2026-10-06/`, with
the replays behind the morning's other entries. It took a permit and dispatched one `drive_to` through the daemon's
`autonomy_act`, exactly as the executive does, and renewed every second. A
second process watched the rover eight times a second on a connection of its
own and made the disruption one second after the rover reached speed. The
stand-in reached the daemon through a relay, which could be killed to cut its
connection. Speeds are from the pose, over one-second windows. The daemon's
reported speed swings from 0.31 to 0.74 m/s around the same steady 0.40-0.45.

## The trials

| | Run | Permit | What was done mid-leg | After the permit ran out | Nav2 |
|---|---|---|---|---|---|
| repeat | `run/be1da2f4/2` | 15 s | connection cut; the stand-in reconnected, renewed and sent the same action again; sent it once more after arrival | -- | up |
| lost connection | `run/be1da2f4/3` | 3 s | connection cut, not restored | run ended 0.24 s later; at rest 0.61 s and 0.27 m later | up |
| hung executive | `run/be1da2f4/4` | 3 s | stand-in frozen with SIGSTOP | run ended 0.49 s later; at rest 0.99 s and 0.30 m later (last movement 1.11 s, 0.31 m) | up |

- **The repeat** was answered "trial/dup/...#1 was dispatched 2.0 s ago and is
  not repeated", and after arrival "...11.0 s ago...". The run spent one action
  and 4.38 m, which is the leg once. That is what R-AUT-11 asks: a request
  whose identifier was already dispatched is answered with what happened, and
  not performed again.
- **Both stops ended the run without latching**, as a lost executive should,
  with "the permission ran out 0 s ago and nothing renewed it". After the hang,
  the woken stand-in's renewal was refused because the run had ended. A fresh
  stand-in on the same run was refused the same way. Restarting the program
  that lost authority does not bring it back.
- **The console's stop** was asked for twice, on legs of 3.2 and 4.4 m. Both
  legs arrived without a press, so the button's stopping distance at speed is
  still unmeasured. It sends the same `stop_driving` that stopped the rover in
  0.11 m at 0.31 m/s on 2026-10-02.

## The margin

The watchdog that ends a run for a breached safe area runs on the same
half-second tick that ended these runs. In the worst case it notices one tick
late, 0.22 m at 0.43 m/s. Then the rover stops, which took up to 0.17 m after
the run ended. Add the 0.20 m body and the margin has to be 0.59 m. It was 0.5;
it is now 0.6 (`permission.FENCE_MARGIN_M`, b464c2e). The runbook says to draw a
safe area 0.6 m beyond where a run starts. A faster rover needs this measured
again.

## Requirements

- [R-AUT-11](../requirements/autonomy.md#r-aut-11) to `settled`.
- [R-SAFE-12](../requirements/safety.md#r-safe-12) to `settled`: a hung
  executive and a lost connection each stopped a moving rover on permit expiry,
  with Nav2 up, and a restarted executive got nothing back. The daemon restarted
  for the margin's deploy came back with "autonomy has not been enabled since
  this daemon started": no run, no permit.

## Deployment

b464c2e to rover_daemon, ros_nav and autonomy. Their suites passed on the rover
in the deploy, and navigation came back settled where the rover was parked. The
running daemon, the bridge's guard and autonomy each load 0.6 m. No drive has
been refused on the new margin yet. The daemon's own dispatch check holds a goal
to the safe area itself. The margin is applied by the bridge to the adjusted
goal and route, and by the watchdog to the moving rover, and neither was
exercised after the deploy. A zero-length dispatch sent to check it was
cancelled by my own stop before navigation saw it.
- M3 criterion 9 is met on hardware: a killed executive on 2026-10-02, and a
  hung one, a lost connection and permit expiry today. Criterion 10 is met
  except for the console's button at speed.
