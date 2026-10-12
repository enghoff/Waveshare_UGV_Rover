# M5 shakedown: five of six staged changes found, no false alarm that held up, and a pose that went 1.35 m wrong among the moved armchairs

**The sessions' shakedown, on 2026-10-12 07:44-08:18, ran every step of the
session script and, counted by the proposed rules, found five of the six changes
the owner staged and logged one furniture-sized false alarm in six untouched
room-visits, which a later loop did not repeat.** It did not run cleanly. In the
autonomous run's last minute and a half, among the moved armchairs, navigation's
pose went 1.35 m and 10 degrees wrong while it still called the position
trusted. The second loop was stopped, the rover refitted at the owner's yes and
the loop driven again. Why the pose went wrong is not known: it has to be
replayed into the mapper before anything is changed. The armchairs also ended up
half a metre from the charger's parking spot, and the rover could not park. It
counts as development, as the [plan](../plans/sessions-that-serve-several-milestones.md)
said it would.

## What ran

| Step | Time | What happened |
|---|---|---|
| start | 07:44:38 | 84% at rest off the charger; pose checked to 4.5 cm at 98%; recording `session-shakedown-1012` on |
| loop A | 07:44:43-07:48:05 | all eight stops reached, 3 min 22 s; every stop's position check within 7 cm where it could say; three small changes logged, none furniture-sized |
| wait | 07:48:05-07:52:15 | in the bedroom while the owner staged the living room (below) |
| run | 07:52:15-08:01:32 | `run/72c4e6b6/1`, 9 min 17 s of its 10, 53.5 m, 31 decisions; ended on three failed actions in a row |
| loop B | 08:01:31-08:02:31 | its first two living-room stops blocked; their checks disagreed (below); stopped by hand |
| refit | 08:11 | at the owner's yes ([R-NAV-3](../requirements/navigation.md#r-nav-3)): moved 1.33 m and 9.6 degrees, 90% of the scan on a wall against 56% |
| loop B | 08:13:48-08:17:12 | all eight stops again, 3 min 24 s; every corridor and bedroom stop within 6 cm at 99-100% |
| home | 08:17:15-08:17:57 | blocked short of the charger's spot by an armchair, 0.86 m short by the position check, which put navigation's pose 20 cm and 5.5 degrees out at 97%; put on charge by the owner |

The charge read 53% after ten minutes standing at 08:12, against 84% at the
start.

**What the owner staged**, in their words, while the rover waited: the two black
armchairs moved further from the balcony and rotated, and a red and a black bean
bag brought in from another room. At 08:08 the red bean bag, beside the stuck
rover, was moved again, to near the opposite wall, so that the rover could see.
The places were read off the recording's scans and are kept with the counts
(`experiments/m5_changes/staged/shakedown-1012.json`).

## The counts

By the [proposed rules](../decisions/m5-sessions-are-counted-before-they-start.md),
with `score_session.py`; the first step after the staging that came within
reach of every change was the run.

| Staged | Kind | Outcome |
|---|---|---|
| armchair by the charger, old place | gone | found, 0.60 m, but see below |
| armchair by the charger, new place | new | found, 0.75 m |
| armchair by the balcony, old place | gone | **found small**, 0.50 m |
| armchair by the balcony, new place | new | found, 0.85 m |
| red bean bag | new | found, 0.80 m |
| black bean bag | new | found, 0.65 m |

Five of six found. One old place came out small again, as in
[scene 1](2026-10-11-m5-scene-1-replayed.md). One of the five is not clean.
The charger armchair's old place was logged small, at 0.45 m, at 07:54:55. It
reached furniture size only at 08:01:25, after the pose had gone wrong, and a
pose half a metre out beside that place would see through it. So four of the
six were found with the pose right. Each was logged between 1 and 8 minutes
into the run, three of them within the first 70 seconds.

**Room-controls: six** (corridor and bedroom in loop A, the run and loop B; the
living room was a staged or reset room each time). **One furniture-sized false
alarm:** in the bedroom during the run, the pose right (fits 0.94 to 1.0), a
0.65 m stretch "gone" at (-8.07, -10.82) that was solid on 2026-10-11 at 13:04.
Loop B's report did not have it, so it **did not hold up**: none held up. What
it was is not known.

**The benchmark's two arms**, each replayed on its own scans from what the watch
knew after the wait (`arm_replay.py`). Five changes are scorable by both arms;
the red bean bag's first place had gone before loop B. The run found four, the
same as the live watch logged. Loop B alone found two, the charger armchair's
new place and the black bean bag. The other armchair's new place and old place
came out small, and the charger armchair's old place it missed. Each arm logged
one furniture-sized false alarm in the bedroom, the run's the one above. Loop
B's was 0.65 m gone at (-8.29, -13.05), which the live watch, with its history,
had logged as three small ones. The arms were not given equal time: 9 min 17 s
against 3 min 24 s. By 07:54:55, under 3 minutes in, the run had found three of
the five and the red bean bag. Its fourth was the charger armchair's old place,
furniture-sized only after the pose went wrong. So with the pose right and at
about a loop's length, it is three to the loop's two.

## The executive's run, and what it would have done

All 31 decisions chose `improve_geometry`. Of the 25 that got as far as a look,
one placed its thing better (object:805, from 1.00 to 0.20 m); five ended in
blocked drives, and the last in the run's end. The look at a change, in shadow,
was offered in 22 of the 31 decisions, for each new thing the watch had logged,
and refused each time only by its switch.

Each decision was deliberated again from the situation it recorded. With the
switch off, 30 of 31 chose what they chose on the floor. The last chose nothing,
not explained. **With the switch on, 10 of 31 would have looked**, the first
at 07:55:40, 3 min 25 s into the run. Eight were at the charger armchair's new
place, one at the other armchair's and one at the black bean bag; the red bean
bag never won. These are replayed one by one, each as if no look had been
taken: on the floor, the first look would have made that change worth nothing
to the decisions after it ([criterion 6](2026-10-11-m5-investigate-change.md)).

The proposed minimum improvement for the benchmark is that autonomy looks at
more staged changes than the loop. The loop never looks, so one look decides
it. But with the switch on, the first look came at 3 min 25 s, about when a
loop-length run would end.

## The pose: right until 08:00:58, 1.35 m wrong by 08:02:13

Each recorded scan was matched against the session's map within 0.7 m and 20
degrees of the recorded pose (`pose_truth`). Through the bedroom and corridor
the pose was within 0.15 m at fits of 0.9 and above until 08:00:18, apart from
15 s driving into the living room at 07:58:07-07:58:22 when it read 0.3-0.4 m
and came back. In the living
room the fits were low throughout, 0.4 to 0.7, because the moved armchairs and
bean bags were in view. From 08:00:58 to 08:01:28, beside the charger armchair's
new place, the scans fitted 0.5 to 0.7 m from the recorded pose, at 0.74 to
0.90 against 0.45 to 0.55 where navigation had the rover. From 08:01:38 the
recorded heading was 12 degrees out. By 08:02:13 the error was 1.33 m and 9.6
degrees, outside the search: that is the refit's own answer, found once the red
bean bag had been moved out of the lidar's view. Navigation's `position_trusted`
stayed true throughout.

Odometry alone, carried from 08:00:18, ends 0.9 m and 18 degrees from the
refit's answer, so the wheels are not a clean account either. Which of the
mapper's corrections went wrong, and why, is for a replay of the recording into
the mapper from the graph the session started with, kept as
`~/.ugv/sessions/shakedown-1012-map/`
([R-NAV-11](../requirements/navigation.md#r-nav-11)). Nothing has been changed.

## The check at each stop, among staged furniture

The session script asks navigation's position check
([R-NAV-14](../requirements/navigation.md#r-nav-14)) at every stop. Where staged
furniture stands beside a stop, the check is unreliable in both directions.

- At the real error, the first stop said 0.08 m and 11.5 degrees out, which was
  right, at a fit of 0.69. The second, with the red bean bag a foot away, said
  0.7 m and 48 degrees, at 0.69. That was wrong: the truth was outside its
  search and the bag blocked a quarter of the view.
- With the pose right, after the refit, the first living-room stop said 0.64
  and 0.72 m and 34-39 degrees out, at fits of 0.72-0.75. The stop half a metre
  on agreed to 5 cm at 0.91. The fourth stop said 0.62 m at 0.77; the corridor
  stop after it agreed to 3 cm at 0.99.

So one disagreeing stop is not evidence, and two running is what separated the
real error from these. The session script now stops where it stands at two
disagreeing stops running (over 0.3 m or 10 degrees), and drives nothing after
that, home included. Measured against this session, that rule stops the first
loop B and none of the others.

## What changes before session 1

- **The session script** (`experiments/m5_changes/session.py`). It now has the
  stop rule above. It keeps the run's path, which the counts need: a run has no
  stops, and without its path the run's finds were scored against loop B. And it
  gives the run a loop's length, 3.5 minutes, as the
  [decision](../decisions/m5-sessions-are-counted-before-they-start.md) has it.
  `score_session.py` reads the path, and takes a move made later in the session
  (`also_moved`) as neither found nor a false alarm.
- **Staging keeps clear of the route.** The shakedown's armchairs blocked the
  first living-room stop, crowded two others and took the charger's spot. A
  metre clear round the charger's spot and each stop.
- **The pose among moved furniture** is replayed before session 1. A session
  that loses its place cannot count past that point and cannot get home.
- **The look at a change**, and how long the run is, are the owner's to settle
  with the counts. At a loop's length the switch decides little, because the
  run barely gets to a look.

The bedroom doorway, where the pose went wrong on
[2026-10-11](2026-10-11-bedroom-pose.md), was driven through five times. The
bedroom stops' checks agreed to within 3 cm where they could say, and the run's
scans there to within 7 cm.
