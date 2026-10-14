# A lidar range where the depth camera gave none

M5 criterion 4 failed on 2026-10-14 because record `object:379` took five
pictures of each black armchair
(docs/progress/2026-10-14-m5-session-1-continued.md). The five of the armchair
with the throw were taken from the stop by the balcony door with the depth
camera asleep, so they had a bearing and no range, and the resolver filed them
to 379, which lay 0.95 m along their line of sight beside the other armchair.
The armchair they showed was 2.7 m away.

The candidate fix is a lidar range for each region the depth camera did not
range (branch `lidar-ranges`, world_state/lidar_ranges.py): the lidar's 25th
percentile across the middle half of the region's width, from the scan
navigation's position check matched the look's pose with, with 0.4 m of
uncertainty, for a region whose lower edge reaches the lidar's plane.

## The five pictures against 379 as it stood

The lidar of session 1's own recording reads 2.55 m across each of the five,
and the resolver's range gate (`locate.stands_at_range`), asked against 379
where the notes put it, refuses all five: 0.95 m against 2.55 m with 1.11 m
allowed. The five pictures of the other armchair, which 379 did show, stay:
their lidar ranges sit within 0.4-0.6 m of where 379 lay.

## Which regions the lidar can speak for

The first rule, a box reaching below 57% of the picture and furniture-sized,
missed two of the five: they were taken with the camera level and their boxes
end just above the middle of the picture. A region's lower edge in elevation,
with the tilt's measured bias out, serves both tilts. On sessions 1 and 2 of
2026-10-14 (`lidar_vs_depth`, regions with both a depth and a lidar range):

| Rule | Regions | Within 0.3 m | Within 1 m | Over 0.8 m short | Over 0.8 m long |
|---|---|---|---|---|---|
| every region | 287 | 36% | 77% | 27% | 2% |
| box rule | 145 | 54% | 86% | 17% | 1% |
| lower edge at most 2 degrees up | 173 | 51% | 85% | 15% | 3% |
| lower edge at most 2 degrees up, not by the box rule | 30 | 37% | 77% | 10% | 13% |
| lower edge over 10 degrees up | 107 | 9% | 66% | 46% | 1% |

## Replays

`replay_identity.py --window` replays one session's looks from an empty world.
From empty, session 1 never builds the wrong merge, with or without lidar, so
it cannot judge the fix. `warm_replay.py` starts instead from the rover's own
snapshot of 2026-10-11 23:04 (`~/.ugv/snapshots/20261011-230441-e68e`, 672
records, 39,107 regions) and feeds the 439 looks taken after it, through the
end of session 1, into the live resolver under the records' own ids.

The lidar is read from whichever recording covers a look's shutter, so only
the looks of session 1 and of the earlier M5 recordings get one: 314 regions.
Results are in `warm-baseline-s1.json` and `warm-lidar-s1.json`.

| | Without lidar | With lidar |
|---|---|---|
| records holding both armchairs | 379 (2 and 8), 838 (3 and 1) | none |
| armchair with the throw, largest record | 379 with 8 of 23 | 488 with 13 of 21 |
| plain armchair, largest record | 923 with 11 of 27 | 924 with 11 of 27 |
| records at the end | 685 | 685 |
| regions filed, of 3,047 | 1,193 | 1,181 |

The replay reproduces the live failure: 379 takes eight pictures of the
armchair with the throw at 0.94-0.98 m along their bearing, as the rover did.
With the lidar's ranges none is filed there, and no record holds both. The
plain armchair stays split over several records either way, which is the
other half of criterion 4 and is not what this fixes.
