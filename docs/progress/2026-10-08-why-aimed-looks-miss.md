# Why most aimed looks file nothing: their target is not recognisably in the picture

**Of the 32 aimed looks in M4 session 1 that took a picture and filed nothing,
not one held a region that looked even 0.70 like its target; the best was
0.69.** These looks did not miss the thing: the record they were sent to
improve is not recognisably in front of the camera. Filing by aim is not what
limits M4; the records chosen as targets are. Measured offline on copies of the
rover's store after the session; nothing was changed.

## The gate each look's nearest region failed

For each look, the region whose bearing is nearest the target's
(`experiments/entity_association/aimed_misses.py`, run on the rover against
the session's `captures/2026-10-08-m4-session-1/aimed-looks.json`):

| Gate | Looks | What it means here |
|---|---|---|
| outside the allowance | 20 | mostly a region on the target's bearing whose range puts it well behind or in front of the record: object:412 should be 0.82 m away, and the region on that line is 3.38 m away |
| appearance | 9 | a region at the aim that looks 0.24 to 0.54 like the target -- object:464 is a person who has since left |
| no region with a bearing | 3 | nothing usable in the picture |

The placements read are today's, not each goal's, so a few may have moved since.

## What the pictures show

Eight close-range looks, the rover 0.7 to 1.0 m from the target's position,
show at the aim a bare wall, an empty doorway, floor, or the room behind. Two
or three targets hang high enough (elevation 54-67 degrees) to sit above the
picture at that distance; the rest would fill it if they were there.

## What does not predict it

- **How consistent a record's exemplars are**: median pairwise cosine 0.69 for
  the targets that filed, 0.655 for those that did not, ranges overlapping
  (0.59-0.84 and 0.51-0.81).
- **Viewpoints**: today none of the 6 targets placed from a single viewpoint
  filed, but on the 157 recorded aimed looks of 10-02 to 10-06 none were placed
  from one, and the filing rate was 20% for 2-4 viewpoints and 25% for 5 or
  more.

## What it means for M4

The geometry goal chooses records by how much their position could improve. A
record placed somewhere nothing stands has a large uncertainty and is an
attractive target, and an aimed look at it finds the wall behind. The finding
of [2026-10-08](2026-10-08-depth-placement.md) -- records of one object sit a
median 1.0 m apart, mostly from bearings alone -- is the same thing seen from the
store. An aimed look that finds nothing at its target is evidence about the
record: its range along the aim says how far past the record the camera saw.
Nothing uses that yet; the goal counts as no gain and the record is set aside
for 15 minutes.
