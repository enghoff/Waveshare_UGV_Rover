# What would report a change: a look says "gone" half the time; the lidar found the moved armchair and nothing else

**The first of M5's measurements on recordings already held: what an unchanged
revisit would report as changed.** The rover's only "it is not there" signal
today is a look aimed at a thing that files nothing to it. On 2026-10-08 to 10,
when nothing had moved but one armchair, that signal fired on half the looks at
things in the depth camera's view, and a second look repeated the first's answer
11 times in 13. Most of it comes from records that are not one real object, but
even on the 40 that are, a generous rule that the thing is still there cannot be
told from the same rule asked of a different object. The lidar does better. The
scan-against-scan comparison of today's recordings found the grey armchair gone
from its place and standing 1.3 m away in every comparison that spanned its
move, within 0.1 m of where its record puts it, and nothing of its size in the
three comparisons that did not. That is development evidence on six recordings,
with thresholds chosen on them. No requirement moved.

## Looks

The executive marks an aimed look `seen_empty` when the thing's place was inside
the depth camera's view and nothing of the look was filed to it
(`autonomy/executive.py`). Over 2026-10-08 to 10 there were 140 such looks at 79
records: 27 filed something, 70 were seen empty, and 33 had no region with a
bearing at all. After an empty look, the next look at the same thing that day was
empty again 11 times in 13, so looking twice does not separate a missing thing
from a look that cannot find it.

Labelled from photo strips of each record (`experiments/m5_changes`), the 79 are
40 single real objects, 18 records mixing several things, 13 that are not objects
(doorways, windows, glare, a switch plate), 4 people and 4 that cannot be told.
The last three kinds should never be able to raise "missing", and the people are
rightly gone. But the 40 real objects fare little better: 69 looks, 19 filed and
30 seen empty. Taken apart region by region (`why_empty.py`), the region nearest
the thing's bearing mostly pointed at it and was refused on appearance, and
nearly half on range. The photographs show the black cabinet refused at an
appearance of 0.44 against the 0.55 it needs, the green painting behind the
dining chairs at 0.53, and a painting at a grazing angle on a range of 1.46 m
against the 2.45 m its record expects. Those tests decide what may join a record;
they were not built to say whether a thing is still there.

Nor does loosening them work (`presence_rules.py`). Each look at a real object
was asked whether some region pointing at the thing looks like it, and the same
question was asked of a record of another kind of object, as if something else
now stood there:

| Rule | Real object judged present | Another object judged present |
|---|---|---|
| appearance 0.30 or more | 44 of 56 (79%) | 149 of 280 (53%) |
| 0.45 or more, height agrees | 30 of 56 (54%) | 22 of 280 (8%) |
| 0.50 or more, height and range agree | 23 of 56 (41%) | 7 of 280 (2%) |
| filed as today | 19 of 56 (34%) | |

With almost nothing moving outside a test, a rule that calls a present thing
missing on a third to a half of looks is a false-alarm machine.

## The lidar

Each recording (`ros_nav/record_drive.sh`: scans, odometry and the mapper's
correction) was turned into counts per 5 cm cell of the map: how often a beam
ended there and how often one passed through (`change_detect.py`). A cell solid
in one recording and seen through in the other has changed; the comparison
allows 10 cm for the poses, and keeps a cluster of changed cells only when it was
seen across more than 20 s, so that a person walking past does not count.

The recordings are of today's sessions on one map. The armchair was moved at
about 12:10 and put back when the owner refitted the map at about 14:10, so it
was out of place for sessions 15 to 19 and back for 20.

| Earlier | Later | Found, 0.6 m or larger | Smaller |
|---|---|---|---|
| sessions 15-16 (13:09) | 17 (13:30) | nothing | nothing |
| 17 (13:30) | 18-19 (13:46) | nothing | nothing |
| 15-16 (13:09) | 18-19 (13:46) | nothing | two, 0.4 m |
| 15-16 (13:09) | 20 (14:55) | armchair: back in its place 0.95 m, gone from where it stood 0.8 m | one, 0.4 m |
| 17 (13:30) | 20 (14:55) | armchair: back 0.95 m, gone 0.95 m | one, 0.5 m |
| 18-19 (13:46) | 20 (14:55) | armchair: back 0.95 m, gone 0.85 m | three, 0.4 m |

Its place is at (-17.18, -16.73) every time; the armchair's record
(`object:317`) puts it at (-17.15, -16.62), claiming 0.20 m. Where it stood in
between is at (-18.4, -15.4). Each side of it also leaves a 0.4-0.55 m fragment
beside the main cluster, counted with the armchair rather than as smaller
clusters. The camera's records never noticed:
no look was aimed at the armchair while it was away. The smaller clusters lie on
wall lines, mostly the hall's, solid on one side and seen through on the
other, which is how a door opened or shut between two runs looks. Session 18-19's
recording was cut off by the brownout and was reindexed from a copy. This
morning's turning recording cannot be compared, because its poses predate the
turning fix, and the kitchen loop is on another map.

The [entry for sessions 14 to 19](2026-10-10-m3-sessions-14-to-19.md) says the
armchair was moved back at about 13:29. It was not: the lidar has it away until
session 20, and the owner's own word at 14:10 was that it had been moved back
for the refit. Sessions 15 to 19 are therefore unchanged revisits of one another
for M5, and session 20 against them is the real change, the armchair's return.

## What it means for M5

A change worth reporting about the larger things the owner says rarely move
should come from the lidar, compared against what earlier runs saw, with the
world's records attached to it by where they are; a look then says which thing
it was, not whether it is there. The looks' seen-empty signal should not be read
as a change at all. Three unchanged comparisons with no false alarm show only
that the rate is under about two in three, and the thresholds were chosen on
these recordings, so the next step is to make every run produce its comparison,
which turns every run into a control, before a scripted scene. Doors need
recognising as doors. Paintings and small things are above or below the lidar's
plane and stay the looks' problem, which this measurement says is not solved.
