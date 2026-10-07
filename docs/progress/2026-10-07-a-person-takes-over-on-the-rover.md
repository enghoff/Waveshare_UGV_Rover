# On the rover, a person who takes over mid-leg now gets their move

**The fix of 2026-10-06 held on the rover.** A person's `drive_to` sent while an
autonomous leg was driving ended the run and latched autonomy off. The drive was
then carried out, not refused as busy as it was on 2026-10-02. This was the
hardware repeat owed by
[the fix](2026-10-06-a-person-who-takes-over-gets-their-move.md), and it closes
the manual half of M3's criterion 12. The voice half is
[deferred](../decisions/m3-defers-the-voice-trial.md).

## What was done

`experiments/m3_trials/m3_trials.py takeover` was run at 083491a, with the
daemon at 685a541's handover. It opened its own fenced run (`run/58d585a2/1`,
15 s permit) and drove one leg in the charger room, starting from the identity
trial's first viewpoint. 1.5 s into the leg, at 0.35 m/s, it sent
`drive_to {"ahead_m": 0.3}` the way the console sends one, as a person.

| | |
|---|---|
| run ended after the person's drive was sent | 0.13 s, "stopped by a person: somebody drove the rover by hand (drive_to)" |
| autonomy afterwards | disabled and latched |
| the person's drive | answered after 0.38 s: `ok`, `arrived`, 0.112 m travelled by its own count |
| at rest | 0.77 s and 0.26 m after it was sent, 0.26 m along from where it was asked, against 0.3 m asked |
| Nav2 | up throughout |

The stop's coast carried the rover most of the way, so the person's drive had
little left to do. What the trial shows is that it was accepted and finished at
the place asked for. It does not show a long drive after a takeover.

Evidence: `captures/m3-trials-2026-10-07/takeover-1791349779.json`.
