# On the floor, the rover reported two bean bags set down while it watched and went straight to look at both, waiting for its change watch before it ended

The fixes of 2026-10-20 had been shown in replay only
([the watch](2026-10-20-a-move-made-in-the-watchs-sight.md),
[autonomy](2026-10-20-why-autonomy-hardly-went-to-look.md)). On 2026-10-21 one
short session (`experiments/multiday/insight.py`, record in
`experiments/multiday/insight/`) put them on the floor. **The change watch reported
both bean bags set down in its sight as soon as the rover drove off, the
autonomy run went to look at each within ten seconds, waited for the watch
after its last drive, refused a doorway, and went home.** Two of the criteria
fixed before the session fail as written, and both because the test assumed
something about the night before that the watch's own record shows was not so.

## What ran

The owner set out the red bean bag at P1, the black one at P5 and the stool at
P3 before switching the rover on (07:33). The lidar confirmed it: P5 was hit
on every scan, and P2 and P4 were clear. At 07:36-07:37, with the rover parked
on its charger and watching, the owner moved the red bean bag to P2 and the
black one to P4, beside the charger, and left. After three minutes the rover
drove 2 m to living-3 and dwelt a minute. The watch reports nothing parked: a
change needs seeing from places 0.5 m apart. Then came an autonomy run with
the benchmark arm's nine minutes (`run/f284be3a/1`), and home. The run took
2.1 min, 7.8 m and 11 actions. The battery read 83% at the start and 65% at
the end.

## Against the criteria

| Criterion | Result | What happened |
|---|---|---|
| 1. Seen in its sight | fails as written | new at P2, furniture-sized (0.60 m), and new at P4 (0.50 m), both logged at 07:40:23, 12 s after the rover reached living-3; nothing logged while it was parked; nothing gone at P5 |
| 2. Goes to look | passes | its first goal was the red bean bag's arrival (3 s into the run), its second the black one's (9 s); both looked at |
| 3. Waits for the watch | passes | after its last drive it waited 26 s, twice, for the watch's next report, and ended when it came |
| 4. Nothing less safe | passes | no stops; home 0.06 m from the spot it aimed at, its position trusted |
| Control: nothing at P3 | fails as written | new, 0.20 m across, 0.12 m from P3, logged at 07:43:53 |

**P5 had nothing to report.** The watch compares each place with how it was on
its previous visit, and that visit ended at 19:52 on 2026-10-20. Within 0.3 m of
P5 it had 230 hits against 1.5 million passes then: empty. So the black bean bag
was not there overnight. Set out in the morning and moved away again within one
visit, P5 is the same as before. The criterion assumed the things had stood
where they were since day 2. **The same goes for the stool.** P3 read 12 hits
against 385,000 passes on that visit, so the stool was put back in the morning
and was new to the watch. Its change, reported when the run's drives first saw
P3 from the side, is a real one.

Also seen on the floor, from the same fixes: three times at the end of the run,
the change at the corridor's door was refused as "the door opening or shutting
rather than a thing worth a look".

## Found along the way

- **The red bean bag's change is dated 07:34:04, about three minutes before it
  was set down.** Most of its cells date from 07:36:50. One cell had some hits
  before the move, and the arrival took its share over the bar for "solid all
  visit", so it took the visit's start time; the group takes its earliest cell.
  A look taken between the visit's start and the arrival would count as having
  seen the change. Not fixed.
- **At the end, navigation was 0.38 m and 7 degrees out.** The position check
  found it as the rover parked, with the black bean bag filling its view beside
  the charger. Navigation corrected itself within a minute, back to the
  morning's pose within 7 cm. The check at living-3 had agreed to 0.03 m.
- **The scorer first counted every look at a change as a wait**: each ends "the
  change watch still reports it". It now matches the executive's own words for
  the wait, read in full rather than cut at 300 characters.

## What it does not show

- **A departure seen in sight**: that needs a thing standing since the watch's
  previous visit, and nothing was.
- **A run started from the charger.** The session drove the rover off first.
  Parked, the watch reports nothing, and a run with nothing reported ends where
  it stands, as on the benchmark's day 2. A second round whose autonomy arm
  starts from the charger after things were moved in its sight would hit this
  again.

R-AUT-13 held. No requirement changed state.
