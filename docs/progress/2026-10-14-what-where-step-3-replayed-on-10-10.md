# What/where step 3 replayed on 2026-10-10: no better than the lidar watch on a day it had never seen, and two false alarms the watch did not raise

**At the owner's request, in place of a third M5 session, step 3 version 2 was
replayed on the recordings of 2026-10-10, which it was not developed on.
Measured against the lidar change watch alone:**
- **Finding:** it found the armchair's return, both ends, as the watch did.
- **Naming:** it named neither end right. It paired the armchair's arrival
  with a small departure beside where the other, identical armchair stands.
- **False alarms:** in three runs where nothing was staged, it raised two the
  watch did not: someone sitting in an armchair, and a close-up of a tissue
  box.

Together with M5 sessions 1 and 2, step 3 has failed, passed and now failed
again.

## What was replayed

Fixed before running (branch `what-where`, `replay1010.md`, `32a32a3`). On
2026-10-10 the owner moved the grey armchair by the charger at about 12:10 and
put it back at about 14:10. The lidar measured where on the day, before step 3
existed ([that entry](2026-10-10-m5-what-reports-a-change.md)): away at
(-18.4, -15.4) during sessions 15 to 19, back at (-17.18, -16.73) for session
20. The move away has no usable recording before it; the return has.
- **The return:** sessions 15-16 and 17 as the watch's history, 18-19 as the
  visit before, session 20 after.
- **Controls:** sessions 15-16 before, 17 and 18-19 after, nothing staged.

Step 3 and its scorer are unchanged from version 2 (`0293ecb`). The watch was
rebuilt from the recordings, and the cut-off recording of sessions 18-19 was
reindexed from a copy. A replay is not an M5 session and is not counted in
M5's totals.

## What it found

| | Watch alone | Step 3 |
|---|---|---|
| the return, found | both ends | both ends |
| the return, named right | | 0 of 2 |
| false alarms, three runs with nothing staged | 0 | 2 |

**The return was mis-paired.** Step 3 reported a move to the right place but
from (-19.17, -16.65). That is a small departure beside where the other black
armchair stands, whose remembered pictures matched the arrival slightly better
(cosine 0.81). The real departure, at (-18.45, -15.38), was reported on its
own as gone. Two identical things make a move's origin a guess
([R-WS-2](../requirements/world-state.md#r-ws-2)), and version 2 lets a small
departure take part in a move. That rule helped on session 2; here it misled.

**Both false alarms were "new to the house" from the camera alone**, at the
window end of the living room:
- **Someone sitting in the armchair** behind a glass side table, for minutes.
  A person who stays is reported by design.
- **A green tissue box** on that table, seen close enough to pass the size
  floor and unlike any region the house had seen before.

Neither held up. The lidar watch compares each place with its last visit, so
neither troubled it. Never seen before is not the same as changed.

## What it means

Step 3's evidence now reads:
- **M5 session 1:** failed;
- **M5 session 2:** passed, five of seven against the watch's two;
- **this replay:** failed.

What it adds is real but narrow: it names a move when it can see both ends,
and it finds thin or low things the lidar misses (session 2's ladder). What it
costs is false alarms from novelty that is not change, and mis-paired moves
between identical things. Neither is fixed by tuning on these sessions, which
are all now seen.

The camera-only "new thing" needs the same test the watch uses:
- the place was seen before without the thing;
- the thing is not a person.

That is development for a version 3. Any test of it must be on evidence not
yet recorded. No requirement moved.
