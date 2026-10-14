# What/where streams on the shakedown: results

Scored on 2026-10-14 by the [frozen protocol](protocol.md), after commit
6517144 fixed the code and the development thresholds. Numbers are from
[results.json](results.json).

## By the frozen rule: shows promise, narrowly

| Staged change | Found | How |
|---|---|---|
| red bean bag (arrival) | yes, 12 looks | what, every one; first at 07:52:48, 33 s into the run |
| black bean bag (arrival) | yes, 7 looks | what in 6, where in 1; first at 07:53:41 |
| armchair by the charger, new place | yes, 3 looks | what once (08:00:55, an overexposed partial view), where twice (08:14:44, 08:20:51) |
| armchair by the balcony, new place | no, 1 look | where once (08:20:51) |
| armchair by the charger, old place | no | gone flagged nothing there |
| armchair by the balcony, old place | no | gone flagged nothing there |

Three of the four arrivals were found, against three needed. One persistent
false alarm was counted in the four room-controls, against at most one allowed.
So the frozen rule says it shows promise. Three of the six staged changes were
found in all; the lidar watch found five on the same session, four of them with
the pose right.

**The counted false alarm is the red bean bag.** Regions 100799 and 100803, in
the corridor after the refit, are two of the three Codex labelled as the red
bean bag at 08:17:30-33, seen from the corridor into the living room. The
protocol judged rooms by where the camera stood, so a staged thing seen from a
doorway counts against the control. On the labels, the room-controls hold **no**
persistent false alarm. They hold two flags in one look each: a dark strip of
bedroom floor (100264, where) and a doorway (100376, what).

## What the streams separate

Furniture-sized regions only; the room-controls are the corridor and bedroom
looks' unlabelled regions (60).

| | what, 10th/50th/90th percentile | where, 10th/50th/90th percentile |
|---|---|---|
| red bean bag | 0.273 / 0.322 / 0.372 | 0.225 / 0.338 / 0.447 (10) |
| black bean bag | 0.172 / 0.232 / 0.360 | 0.253 / 0.375 / 0.490 (4) |
| charger armchair | 0.110 / 0.173 / 0.278 | 0.102 / 0.177 / 0.497 (8) |
| balcony armchair | 0.134 / 0.166 / 0.251 | 0.037 / 0.131 / 0.355 (6) |
| room-controls | 0.075 / 0.143 / 0.277 | 0.000 / 0.010 / 0.150 (60) |
| threshold (99th percentile, controls of 10-11) | 0.322 | 0.459 |

As areas under the curve against the room-controls, with the four labelled
bean-bag regions seen from the corridor taken out of them (60 regions left):
what scores bean-bag regions at 0.89 (51 regions) and armchair regions at
0.64; where scores the posed armchair regions at 0.88 (14, from nine looks).
Keeping all 64 camera-assigned control regions, they are 0.85 and 0.84. So
what tells new things from old, and
where tells moved things from unmoved, but both thresholds sit at the top of
the positives' range. The bean bags' what scores are near the threshold, and
most armchair where scores are under it.

## Why where missed most of the armchairs (diagnosis, not tuned)

For each posed armchair region, the best reference region in a consistent
place was traced. The 30 references reached back one to nine days, to
2026-10-03 for the looks at 08:20:51. The best match "here" came from 10-03 to
10-11, never from that morning's first loop; one was from scene 1 of 10-11,
when the armchairs stood elsewhere. In 10 of the 14 that match had no range,
so it agreed with the armchair on bearing alone, across 15 degrees of
tolerance and any distance; an unranged reference also keeps its own bearing
although its camera stood up to a metre away. The ranged tolerances are loose
too: balcony region 100838 matched a reference 14.9 degrees and 0.69 m away,
both allowed, where the staged move was 0.90 m. These are properties of
nearest-memory prediction, not of the encoder: memory deep enough to absorb
the region finder's noise also remembers old arrangements, a bearing is a line,
not a place, and tolerances wide enough for that noise swallow a move of a
metre.

## Gone

The gone score's 99th percentile on the unchanged controls was 0.93: for one
remembered region in ten, no look in an unchanged visit matched it at better
than 0.67 below how well memory agreed on it. The region finder does not find
the same things every time, so a remembered thing not being found is weak
evidence it has gone. Neither old place was flagged. Every remembered region
near an old place that two looks expected was unranged, and in the run none
near the balcony armchair's old place was expected twice.

## Coverage

Of 129 looks in the two scored windows, 68 had a stored pose; all 68 had
references. 104 labelled regions fell in the windows, 101 furniture-sized; the
where stream saw 28 of them, 14 of the 50 armchair regions. The rest had no
pose because the heading check refused the rearranged living room. The drive
recording that holds navigation's pose for them is on the rover, which was
offline. A stored pose is one the heading check accepted, not one shown right:
the looks at 08:20:51, which hold the charger armchair's second where flag and
the balcony armchair's only one, come after the rover stood 20 cm out short of
the charger, past the interval the heading-check replay of 2026-10-12 vouches
for. Without them the charger armchair is still found, by what at 08:00:55 and
where at 08:14:44, but no armchair is found by where alone in two looks.

## Corrections to the protocol and checks after scoring

Codex reproduced every threshold and test number from the frozen code
(`tmp/codex-assistance-replies/ugv-rover-25-ww-codex-1.md`), found no
observation after the cutoff in memory (it ends at 07:47:45), and confirmed the
pan sign and the floor filter. Against the protocol as written:
- The development gone threshold rests on **1,082** remembered regions
  expected twice, not 1,094.
- The frozen code counts a range between 0.2 and 8 m, and its comment wrongly
  says that is the store's rule, which is 0.75 to 12 m. Rerun with the store's
  limits, the finds and the counted flag are the same; three region scores and
  the gone threshold (0.9311) move slightly.
- Not every choice was made on development alone: the furniture-sized floor is
  Codex's labelling floor from this shakedown, and the bar borrowed the lidar
  watch's known result on it.
- The person exclusion and the count of living-room flags away from staged
  places were done by eye, not by `run.py`. No flag in the room-controls
  showed a person. Outside the corridor and bedroom, seven flagged regions are
  unlabelled. Of the four whose pictures are on this desktop, two show the
  charger armchair, one cables by the charger and one a bright patch of floor.
  The other three, all at 08:20:36-56 by the charger, have their pictures only
  on the rover.
- Persistence grouping is greedy and depends on order, and gone is taken per
  remembered region, so one thing seen in different reference looks can split.

# Step 2: with navigation's pose (2026-10-14)

Fixed in [step2.md](step2.md) before running (`f7141e3`); numbers from
[step2_results.json](step2_results.json). Looks the store kept no pose for took
navigation's pose at the shutter from the drive recordings, unless the rover
was turning faster than 20 degrees a second; rooms were judged by where a
flagged thing stands (its depth point, else the lidar's return along its
bearing). The thresholds, derived again on the controls by the same rule,
barely moved: what 0.3216, where 0.4575, gone 0.9316.

**Coverage.** The where stream now sees 23 of the charger armchair's 27
furniture-sized regions (8 before), 21 of the balcony armchair's 23 (6), 23 of
the red bean bag's 23 (10) and 25 of the black's 28 (4).

| Staged change | Found by the rule | Looks flagged | By where | Before 08:17:15 |
|---|---|---|---|---|
| red bean bag | yes | 16 | 5 | yes |
| black bean bag | yes | 17, first 42 s into the run | 11 | yes |
| charger armchair, new place | yes | 4 | 3 | yes |
| balcony armchair, new place | yes | 2 (08:00:57, 08:20:51) | 2 | no |
| charger armchair, old place | yes, but not on the evidence | gone, 2 looks | | |
| balcony armchair, old place | no | | | |

**By the rule: five of six, with one persistent false alarm, so it shows
promise; the lidar watch's count on the same session was also five of six with
one.** On the pictures it is less:
- The gone flag at the charger armchair's old place is a patch of floor in a
  look of 2026-10-10 whose unranged sight line happened to pass within 0.5 m of
  the old place. It says nothing about the armchair. On the evidence, gone
  found neither old place.
- The persistent false alarm is a gone flag: a strip of bedroom floor under the
  chest of drawers, remembered from a look during scene 1 of 2026-10-11, when a
  stool stood at the edge of it. No region flag persisted in the corridor or
  bedroom.
- The balcony armchair's two flags are at 08:00:57, one second before the pose
  began to go wrong, and at 08:20:51, after the rover stood 20 cm out by the
  charger. Without looks after 08:17:15, three arrivals are found.
- Each of the four arrivals is found by where alone in two or more looks.

**Separation.** Against the room-controls' 60 regions: where scores the
armchairs' 44 posed regions at 0.89 (0.89 before 08:17:15), the bean bags' at
0.95; what scores the bean bags at 0.89. Only 5 of the 44 armchair regions pass
the where threshold: their median where is 0.14 against a threshold of 0.46.
Persistence found them, and the stale, wide references of step 1 are still what
keeps most armchair regions under it.

**Flags in the living room not on a staged thing's label.** Eight, all after
the refit. Four show the staged armchairs. Four do not (cables by the charger, a
bright patch of floor, the balcony door, floor shadows), all at 08:17:56-08:20:36
by the charger, when the pose was 20 cm out. Three flags had no place: two of
the black bean bag in the run and one patch of floor.
