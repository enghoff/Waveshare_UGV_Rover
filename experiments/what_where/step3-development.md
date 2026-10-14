# Step 3, development notes (2026-10-14)

Development only, on the seven controls of 2026-10-11 (unchanged) and scene 1
of the same afternoon (staged). Nothing here is frozen or a test. Code:
[lidar.py](lidar.py), [streams3.py](streams3.py); exploration scripts were not
kept.

## Places from the lidar

Of 437 furniture-sized regions with a depth range on the controls, the lidar
placed 433 when taking the 25th percentile of its returns across the middle
half of the region's width. 57% landed within 0.3 m of the depth camera's own
range, against 48% for the nearest return within 3 degrees. One in ten fell
about 0.8 m short and one in ten about 0.35 m long. Depth itself is noisy, so
this is a floor on agreement, not the lidar's error. The lidar places 90% of
the furniture-sized regions that have a pose and no depth range.

## Where against things at places

Each region is matched against remembered things within 0.6 m of where it
stands. The 99th percentile on the controls is 0.276 for depth-placed regions
and 0.552 for lidar-placed ones, so each source needs its own threshold. On
scene 1, the 14 regions of the armchairs at their second places scored a median
of 0.22, and one or two passed. What explained them were armchair-like regions
remembered from early October within 0.6 m. Those memories predate every
recording here, so the lidar cannot retire them.

Limiting memory to its last 12, 24 or 48 hours, or fading it with age, does not
help. The controls' threshold climbs to 0.55-0.95, because the rover does not
revisit every place often enough, and fewer armchair regions pass, not more.

## Gone from space seen empty

A remembered thing the lidar hit when it was seen is retired once most of its
latest 20 judged scans before the cutoff see through it. It is gone in a visit
when most of the visit's scans do. The 99th percentile of that share on the
controls is 0.58. On scene 1, both armchairs' morning places are flagged, by
dozens of remembered regions each with shares of 0.9-1.0. Unexplained flags,
at shares of 0.6-1.0, are around (-18.4, -13.2), (-17.2, -12.85) and
(-18.1, -10.5) in the living room, and in the bedroom, where the pose was
0.25-0.47 m out that afternoon. Flags still need grouping into events.

## What

Unchanged from step 1. On scene 1 it flagged the bedroom's tipped stool in
eight looks and the pillows in two, every bedroom flag a staged item. The lidar
watch had logged both as small.

## What this says for the design

The latent where stream, matching a region's appearance against remembered
things near its place, does not separate moved armchairs from unchanged ones
on this evidence. Place noise and old memories are too large for a move of a
metre. Geometry does separate them: the lidar seeing through where a thing
stood, and (not yet built here) hitting where it now stands. What is strong
for new things and things below the lidar's notice. The proposed shape is
therefore:
- where and gone from the lidar's latest state, as the change watch keeps it;
- what from the frozen encoder;
- a moved thing joins a gone and an arrival that look alike.
