# M3 hardware trials and sessions

The scripts behind M3's supervised trials and sessions on the rover
([the plan](../../docs/plans/autonomous-curiosity.md), Phase 3). Not deployed:
copy them to `/tmp` on the rover for a session. `/tmp` is emptied by a reboot,
so copy again after one. Every one of them moves the rover only through a run
it opens or a person's call, with the owner present and the rover untethered.

| Script | What it does |
|---|---|
| `m3_trials.py dup` | a drive whose connection is cut mid-leg is asked for again with the same identifier; it must be refused (R-AUT-11) |
| `m3_trials.py drop` | the stand-in executive's connection is cut mid-leg and not restored; the permit must stop the rover (R-SAFE-12) |
| `m3_trials.py hang` | the stand-in is frozen mid-leg; the permit must stop the rover, and waking it must not bring the run back (R-SAFE-12) |
| `m3_trials.py console` | the owner presses the console's stop at speed; timed from the daemon's own record of the press |
| `m3_trials.py takeover` | a person's drive is sent mid-leg the way the console sends one; the run must end and the drive be carried out (criterion 12) |
| `start_run.py "purpose" [area]` | opens a session run and starts the executive: fenced to the charger room by default, `flat` for no fence, or `min_x,max_x,min_y,max_y` for another room |
| `watch_run.py [floor] [minutes]` | follows a run every 5 s until it ends, or the battery reads the floor (default 10%, the owner's standing floor) or less three times standing still |
| `session_alerts.py` | a live alert stream for a session (run it under a monitor): a drive 20 s without moving 0.3 m, the run ending, the battery at 10% over 12 s standing, and a summary every two minutes |
| `battery_now.py [seconds]` | the charge now and what kind of reading it is -- under load, recovering from a drive, rising on the charger or at rest -- from the median of a window of readings; run it before stating a charge |
| `trace_moves.py` | matches every move navigation logged during each session to the episode that dispatched it (R-SAFE-9) |

`m3_trials.py` drives one leg between two fixed points 4.4 m apart in the
charger room. It opens its own run with a safe area, and writes every sample to
`/tmp/m3trials/`. Its `voice` mode is kept but
[deferred](../../docs/decisions/m3-defers-the-voice-trial.md).

The fixed points and the safe area are the charger room's on map `7da19bef3888`.
A new map needs them chosen again: a straight leg the body fits along, and a box
0.6 m beyond where a run starts.

M3 wants a session in each of the conditions listed in the plan, not a count of
them ([the decision](../../docs/decisions/trials-are-sized-by-what-they-show.md)).
For another room, take the rover there with a person's `drive_to` before opening
the run, and fence it from where it stands.
