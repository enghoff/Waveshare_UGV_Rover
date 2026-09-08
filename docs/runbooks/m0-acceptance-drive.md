# The M0 acceptance drive

What to set up, what to type, and — the part that matters — **what has to be
written down before the rover moves.** [M0](../plans/autonomous-curiosity.md#milestone-m0-semantic-state-is-safe-enough-to-influence-goal-selection)
asks for tolerances declared in advance and trials the remedies have not seen. A
drive taken without this page cannot certify anything, however good its numbers
turn out to be, and that is why [the drive of
2026-09-08](../progress/2026-09-08-acceptance-drive-two.md) improved on its
predecessor and still moved no criterion.

## Freeze this before driving

Copy this block into the run's manifest file and fill in the blanks with the
owner present. Nothing here may be changed after the recording starts.

### The thresholds under test, as deployed

| what | value | chosen on | owed |
|---|---|---|---|
| two crops are not the same thing below | 0.55 | room recordings before 2026-09-07 | — |
| a match whose score collapses when masked is refused at | 0.20 | the drive of 2026-09-07 | this drive |
| another thing may lead the one being joined by at most | 0.15 | the drive of 2026-09-08 | this drive |
| an exemplar is learnt only from a match at or above | 0.70 | this rover's own measurements | this drive |
| a crossing needs a baseline of at least | 0.4 m | — | — |
| a crossing needs a parallax of at least | 12 degrees | — | — |

**Three of those were chosen after seeing the faults they catch**, which is what
makes this drive worth taking: it is the first recording none of them has seen.
If any value is edited between now and the recording, this drive stops being
held-out for it and the entry must say so.

### The operating envelope

- gimbal pan **-20 to +20 degrees**, tilt **0 or +20**, every placement finishing
  from the ascending direction. Outside it a look keeps its picture and records
  no direction at all.
- the depth camera covers the middle of the gimbal camera's view; a region
  outside it is reported as never-ranged rather than silently unranged.

### The tolerances, declared now

| question | passes if |
|---|---|
| do ranges land on the object they claim? | at least 70% of ranges within 0.5 m of the parallax answer, on things fixed from 3+ standing places |
| is a placement where the object is? | every measured separation between named targets agrees within 0.30 m |
| is any movement-eligible association wrong? | **zero** known-incorrect in at least 50 reviewed decisions |
| does the rover abstain from everything instead? | at least 25 things placed, and at least 15 of them ranged |

The first is set from 69% measured on 2026-09-08, so it asks the rover not to get
worse. The second is set well outside the one separation measured that day
(2.9 m against 2.899 m) and inside what the placements claim. The last exists
because refusing every case is not a pass.

### The targets

At least three objects, put out by the owner, **each separation measured with a
tape and written down before driving**:

| target | what it is | separations |
|---|---|---|
| 1 | | to 2: ___ m, to 3: ___ m |
| 2 | | to 3: ___ m |
| 3 | | |

One of them at floor level, one above the horizontal, and one deliberately placed
so it is seen at the edge of the frame — that last is what exercises the
never-ranged reporting.

## The drive

1. **Clear the semantic store, keep the map.** The sample has to be free of old
   association decisions; the map is the frame everything is measured in, and
   starting localised is what the envelope was declared against.
2. **Check the map came back settled, not merely loaded.** A restore that
   anchored wrong puts a heading error into every bearing in the sample, and it
   is the one failure that wastes the whole run. `map_settled`, not `map_kept` —
   see [R-NAV-2](../requirements/navigation.md#r-nav-2).
3. Drive for at least fifteen minutes, passing each target from **three or more
   standing places at least a metre apart**. Parallax is what makes the ground
   truth; a target seen only from one spot cannot check anything.
4. **Restart navigation once, mid-drive**, and keep looking at things for a
   minute afterwards. `ssh orin '~/ugv/ros_nav/restart.sh --supervisor'`.
5. Drive past something at the edge of the frame on purpose.
6. Stop, and say so, so the recording is archived at a known point.

## Afterwards

Archive before anything restarts — the store is live and a clear would take it:

```bash
ssh orin 'python3 - <<PY
import sqlite3
src = sqlite3.connect("file:/home/jetson/.ugv/world/world.db?mode=ro", uri=True)
dst = sqlite3.connect("/home/jetson/.ugv/archive/world-<date>-acceptance.db")
src.backup(dst); dst.close()
PY'
ssh orin 'mkdir -p ~/.ugv/archive/frames-<date>-acceptance && cp ~/.ugv/world/frames/<date>-*.jpg ~/.ugv/world/frames/<date>-*.depth.gz ~/.ugv/archive/frames-<date>-acceptance/'
```

Copy both down into `captures/`, which is gitignored, and keep a MANIFEST.txt
beside them holding the frozen block above.

Then score it: the parallax check for ranges and placements, and a contact-sheet
review giving **every** thing a written verdict rather than only the ones that
catch the eye. Two mistakes on 2026-09-07 and 2026-09-08 were both made reading
crops too small; five to a row at 200 pixels, and zoom anything doubtful with its
full frame behind it.
