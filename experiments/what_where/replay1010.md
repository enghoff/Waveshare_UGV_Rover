# Step 3 version 2 replayed on 2026-10-10, fixed before running

The owner asked, on 2026-10-14, for earlier recordings to be replayed instead of
a third M5 session. Step 3 was developed on the controls and scene 1 of
2026-10-11, the shakedown and session 1; the recordings of 2026-10-10 were not
used. On that day the owner moved the grey armchair by the charger at about
12:10 and back at about 14:10
([the entry](../../docs/progress/2026-10-10-m5-what-reports-a-change.md),
written before step 3 existed). Its first move has no usable recording before
it; its return does.

- **Return**: sessions 15-16 and 17 as the watch's history, 18-19 as the visit
  before, session 20 after. One staged move: the armchair from (-18.4, -15.4)
  to (-17.18, -16.73), as that entry measured it on the lidar.
- **Controls**: sessions 15-16 before, 17 and 18-19 after, nothing staged; the
  entry found nothing furniture-sized between them. Only false alarms are asked.

Unchanged: step 3 version 2 (`0293ecb`) and `score3.py`, with the watch state
rebuilt from the recordings (`--history`), the map gate from the 2026-10-14
map of the same session, and memory the world store's looks before each
replay's staging. A replay is not an M5 session: its result is reported beside
M5's, not counted in it. Two staged events and a few room-controls cannot give
a rate; this asks whether step 3 behaves on a day it has never seen.

## Result

[replay1010_out/](replay1010_out/). The return: found both ends by both
methods, named 0 of 2 by step 3 (paired with a small departure by the other,
identical armchair). Controls: watch 0 false alarms, step 3 2, both
camera-only new things: a person sitting in an armchair, and a close-up of a
tissue box. Step 3 fails here.
[The entry](../../docs/progress/2026-10-14-what-where-step-3-replayed-on-10-10.md).
