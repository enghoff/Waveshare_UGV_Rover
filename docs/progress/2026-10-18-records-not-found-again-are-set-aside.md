# Records the world model cannot find again are set aside, not called gone

**A record seen on two visits or fewer, not for a week, and not recognised on
three close looks at its place since, is now set aside as not found again.**
The owner agreed to this that morning, after absence from the camera turned
out not to be honest as "gone"
([entry](2026-10-18-absence-from-the-camera-is-not-honest-as-gone.md)).
Nothing is called gone that was not before, and the model files every look as
version 4 does. On the rover 127 of 630 records are set aside.
[R-WS-17](../requirements/world-state.md#r-ws-17) stays proposed.

## The rule, and why this one

The model counts, for each standing record with a depth place, the visits
since it was last filed to in which its place was within 2.5 m and 45 degrees
of the camera in three pictures or more and nothing was filed to it. The
daemon sets the record aside when that count reaches three and the record was
seen on two visits or fewer, not for seven days.

Rules tried on the way, all scored over every picture to the drives of
2026-10-17:

| Set aside when | Set aside | Of things never staged |
|---|---|---|
| nothing filed to it, and nothing placed within 0.5 m, 4 visits in a row (any record) | 226 | 6, the snowy-tree painting's main record still aside at the end |
| the same, but a sight line past it that stopped there or short, or was not ranged, also counts as something there | 0-6 | 0 |
| the same, but only regions filed to a record standing elsewhere are discounted | 30-62 | 0, but 10-31 later found again |
| **seen on two visits or fewer, not for 7 days, nothing filed to it on 3 close visits since** | **81 of 518** | **0, and none of the 81 holds a labelled picture** |

The first misses paintings: hung above what the depth camera ranges from close
up, their pictures give no distance, so nothing seems placed there. The next
two are honest but find almost nothing to set aside: the camera sees 160
degrees, only its middle is ranged, and some unranged sight line passes near
nearly every place. What separates a stray from a thing is how little it was
ever seen. Every labelled object, staged or not, kept its main record standing,
and none of its other records was set aside.

## What was measured

- **The filing is unchanged.** Over every picture to 2026-10-17, the rover's
  model with the count files exactly as version 4, byte for byte. In 12 pieces
  the engine groups and names as one run.
- **On the rover**, after the history was filed again: 630 records, 129 gone,
  127 set aside (104 seen on one visit, 23 on two), 374 in place. `find_thing`
  for the bed, the rug, the blue painting and the front door answered as
  before. The console's header counts 374 records, and the line under its map
  reads "129 gone · 127 not found again".
- **Eight set-aside records, chosen at random, from their pictures**: the bed,
  the painting over the bed and four other paintings, each a second record
  with an established one within 0.4 m (the painting by the charger's, `v856`,
  seen on 37 visits, is 4 cm away), and a black bag under the dining table on
  10-08 and a cloth over a door on 10-05, both passing through.

## Found on the way

- **Lidar stamped before the clock was set.** Filing the history again with the
  job's own lidar gave 74 records gone, not the 129 that came after the fix.
  One chunk logged after a cold boot began in 1970, so it "covered" 1970 to
  2026. Every sighting no drive recording covered was asked of it, found no
  lidar there, and counted against the thing being one the lidar hits. The
  logger, the live reader and the history now leave such stamps out, and an
  engine filed the old way is filed again.
- **First sightings in 1970.** The model's first visits were looks taken at
  cold boots, so records started there said they were first seen then: the
  console showed "first 7:35:24 AM", and `find_thing` would have said "56 years
  ago". A record's first sighting is now its oldest look with a real time; no
  record of the 630 says 1970.
- **A test that passed on the desk and not on the rover**: rewritten in place,
  two records files of one size within one clock tick read as unchanged on the
  rover's filesystem. The tests now write the file as the job does, renamed
  into place.
