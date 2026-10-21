# A scan the map fits better a little way off is no longer counted by the change watch, and the passage that read as new furniture is gone

At 09:14 on 2026-10-21 the change watch logged 0.85 m of new furniture at
(-11.47, -12.59), the left wall of the passage by the corridor. At 09:15 it
logged 0.75 m of the opposite wall gone. Both stayed in its report, and after
[the viewpoint fix](2026-10-21-runs-that-ended-at-once.md) the furniture was
the first goal a run would drive to, 6.35 m across the flat. They were the
rover's pose, not the passage. From about 08:59 the rover stood there 19 cm and
half a degree off. Navigation's drift check said so at 09:03:52, and the watch
paused only from then. The scans counted in those four minutes saw through the
passage's walls, and that visit became the reference the walls were judged
against once a refit had put the pose right
([the navigation side](2026-10-21-the-live-layer-and-a-rover-off-by-its-width.md)).

The watch already refused a scan whose far hits miss the mapper's walls. That
did not catch these, because the far walls ran along the error and still fitted.
The near walls that did not fit are not judged at all.

## Reproduced first

The world model's kept lidar of the morning (07:36 to 09:21, 18 files, merged
and deduplicated) was fed to a watch starting from nothing, as
`ros_nav/change_node.py` feeds it. It used the live node's pauses (the drift
check's 09:03:52 to 09:13:50, the untrusted position at 09:14:56) and was judged
against the map as it is now. As deployed, it logged the passage's wall new at
furniture size from 09:13:50 and the opposite wall gone, as the rover had. It
also logged the cluster round the charger at 09:20, when the rover was 35 cm out
there.

## The change

Each scan's returns out to 3 m are laid on the mapper's walls at every shift up
to 0.40 m and 6 degrees. The scan is refused, and counted as `misplaced`, when
one shift lays 20 more returns within a cell of a wall than the believed pose
does, and a tenth of the scan. That is the live layer's test of the same
morning, with its numbers, so navigation and the watch agree on when the rover
is off. The test refuses scans in exactly the windows where the pose is known to
have been out: 120 to 198 a minute from 09:03 to 09:08 in the passage, none
once the 09:08 refit had put it right, and 75 and 135 at 09:19 and 09:20 by the
charger.

| Replayed | As deployed | Changed |
|---|---|---|
| This morning: changes logged | 15, among them the passage's walls (3 furniture-sized) and 7 at 09:20, most round the charger | 1, the 08:59 gone at (-17.01, -13.39) that run 1 looked at |
| Nine M5 sessions: staged ends found, of 60 | 58 (40 furniture-sized) | 58 (37) |
| ...changes nothing staged explains, furniture-sized | 3, among them 1.2 m "gone" in session 5's corridor | 1, on a marked door |
| ...the same, smaller | 30, 11 of them on doors | 28, 13 of them on doors |
| Benchmark mornings (10-19, 10-20) | the black bean bag, the stool and the red bean bag found; 3 small changes at a door | the same three found; 1 small change at that door |
| Scans refused on the M5 sessions | 3,274 of 36,211 | 4,897 |
| Cost a scan on the Orin | 3.4 ms | 6.7 ms |

Two things are worse. Session 7's armchair, moved 0.2 m, is now found small at
both ends rather than furniture-sized: a big thing moved beside the rover can
look like the rover being off. And this morning's 07:44 bean-bag arrival is
lost in the replay, which has no history before 07:36. The watch refused 14 to
27 scans a minute then, while the rover was parked 0.38 m out
([the session that morning](2026-10-21-the-fixes-on-the-floor.md)). The live
watch had logged that arrival at 07:40 and 07:45.

The 1% of a core is the price. The watch takes every third scan, about three a
second.

## The tallies it already held

What the watch had counted from 08:58:30 to 09:21 was cleared before it
restarted on the new code, as the bedroom's was on 2026-10-11. Cells last seen
in that window lost their visit in progress (21,399 cells, about 53 m²), and
cells whose reference ended in it lost that reference (14,323). Its report went
from the passage's two phantom changes and two one-place sightings by the
charger to nothing. Those places report nothing until each has been seen in two
visits again. The file before is kept as `7da19bef3888.npz.before-misplaced-purge`.

[R-AUT-13](../requirements/autonomy.md#r-aut-13) is held. No requirement
changed state.
