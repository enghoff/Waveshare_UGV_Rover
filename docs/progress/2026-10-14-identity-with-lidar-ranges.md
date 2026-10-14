# Identity with lidar ranges: the armchair merge goes away on replay, and the rover now ranges what the depth camera cannot

2026-10-14. [R-WS-18](../requirements/world-state.md#r-ws-18), M5 criterion 4.

**What it means:** the wrong merge that failed M5's criterion 4 came from
pictures with no range, and a lidar range for them removes it when the rover's
own looks of that day are replayed from its own saved world. The change is
deployed and a still look on the rover now ranges regions the depth camera did
not. Criterion 4 is not re-measured: the plain armchair is still split over
several records, and no session has been run on the change.

## The cause

In [M5 session 1](2026-10-14-m5-session-1-continued.md) record `object:379`
took pictures of both black armchairs. The five of the armchair with the throw
were taken from the stop by the balcony door with the depth camera asleep, so
each had a bearing and no range, and each was filed to 379, 0.95 m along its
line of sight beside the other armchair. The armchair they showed was 2.7 m
away. The session's own lidar recording reads 2.55 m across each of the five,
and the resolver's range gate refuses 379 for all five with that range (1.6 m
out, 1.1 m allowed). The five pictures of the other armchair, which 379 did
show, pass.

## Which regions the lidar can speak for

A box rule (reaching below 57% of the picture, furniture-sized) missed two of
the five, taken with the camera level. A rule on the region's lower edge, with
the tilt's measured bias out, serves both tilts. Regions of sessions 1 and 2
with both a depth and a lidar range:

| Rule | Regions | Within 0.3 m of depth | Within 1 m | Over 0.8 m short |
|---|---|---|---|---|
| every region | 287 | 36% | 77% | 27% |
| box rule | 145 | 54% | 86% | 17% |
| lower edge at most 2 degrees above the horizontal | 173 | 51% | 85% | 15% |

## The replay

From an empty world session 1 never builds the wrong merge, so that replay
cannot judge the fix. The rover's snapshot of 2026-10-11 23:04 (672 records,
39,107 regions) was copied off and the 439 looks taken after it, through the
end of session 1, fed into the live resolver under the records' own ids, once
as recorded and once with a lidar range for the 314 regions a recording covers:

| | As recorded | With lidar ranges |
|---|---|---|
| records holding both armchairs | 379 (2 and 8 pictures), 838 (3 and 1) | none |
| armchair with the throw, largest record | 379, 8 of 21 | 488, 13 of 21 |
| plain armchair, largest record | 11 of 27 | 11 of 27 |
| records at the end | 685 | 685 |
| regions filed, of 3,047 | 1,193 | 1,181 |

The replay reproduces the live failure. With the lidar ranges no record holds
both armchairs, no extra records are made, and 12 fewer regions are filed.
Scripts and results are in `experiments/identity_lidar/` on the branch
`what-where`.

## On the rover

Deployed at `6a83811` (world state, daemon, navigation). A still look from the
charger with the depth camera off ranged 6 of its 11 regions by the lidar, at
2.7-3.1 m, and left the five high in the picture alone; the same look before the
deploy ranged none. Selftests: world state 1,174 passed.

**The deploy's navigation restart did not get its map back for 13 minutes.**
slam_toolbox took longer than the bridge's 30 seconds to load the saved graph
while the perception service, the daemon and the what/where shadow runner were
starting or running beside it. Each retry every 30 seconds asked for another
load, and over 23 attempts the mapper grew from 2.2 to 4.2 GB until 90 MB was
free. The rover kept its map identity throughout (a restore that has not
loaded is retried, not replaced), so the world state did not move session. A
second navigation restart, with the shadow runner stopped, loaded on the first
attempt and settled where the rover was parked. Not fixed or reproduced:
why the loads accumulate, and whether a retry should wait for one still
running.
