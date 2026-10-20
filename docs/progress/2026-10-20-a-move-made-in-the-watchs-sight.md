# The change watch now reports a thing moved in its sight: replayed, the black bean bag the benchmark's day 2 never heard of is reported as the rover drives off

On the multi-day benchmark's day 2 the black bean bag was set down beside the
charger while the rover sat there switched on, and the change watch never
reported it ([why autonomy hardly went to look](2026-10-20-why-autonomy-hardly-went-to-look.md)).
A visit to a place lasts as long as the place stays in sight, so its cells were
seen through for part of the visit and hit for the rest, which is neither
solid nor clear. **Since this change, a cell the whole visit cannot call is
called what its last two minutes say, when they differ from the rest of the
visit by half the beams or more, and is compared with the visit before.**

## Reproduced first

The kept lidar of both benchmark mornings (the world model's 40-minute buffer,
copied at the end of each day), with the world model's scan batches between
them, was fed in time order to a watch starting from nothing, as
`ros_nav/change_node.py` feeds it, judged against the map of day 2.

| Replayed | As deployed | Changed |
|---|---|---|
| Black bean bag, new at P5 | never | 07:42:03, 0.60 m, furniture-sized |
| Stool, new at P3 | 07:43:33, 0.30 m | 07:43:33, 0.30 m |
| Red bean bag, new at P1 | never | 07:42:03, 0.35 m, and 07:49, 0.30 m |
| Round the corridor's doors | 4 small | 4 small |

As deployed, the replay logged the stool and the door the rover logged that
morning; it missed the red bean bag's 0.3 m at 07:50, which the live watch,
with a week of history, had, and logged three small changes round a corridor
door that the live watch did not. Day 1 has no visit before it in the replay,
so it reports nothing either way. Neither version reports anything gone from
P4 or P2, where the black bean bag and the stool stood on day 1: the bean bags
live in a room the rover cannot enter and are brought out for each session, so
those places were probably empty before day 2's staging, which was not checked
with the owner.

## Against the M5 sessions

The nine recorded sessions of 2026-10-14 to 18 were fed in order to a watch
starting from nothing, as deployed and changed, each change logged sorted
against what was staged that session and the one before.

| Logged | As deployed | Changed |
|---|---|---|
| Within a metre of something staged | 78 (29 furniture-sized) | 90 (41) |
| On a marked door | 11 | 11 |
| Elsewhere, furniture-sized | 3 | 3 |
| Elsewhere, smaller | 17 | 19 |
| Staged moves found, of 60 ends | 49 (26 furniture-sized) | 50 (30) |

The two new changes elsewhere are 0.20 m each, in the corridor in the drives of
2026-10-17. Two staged ends found small as deployed are no longer found
(session 6's bean bags' arrivals), and five not found or found small are now
found, four furniture-sized.

**A first version was worse, and the difference is worth keeping.** It also
let a visit that ended mixed hand its last two minutes on as the next visit's
reference, without asking whether the place had flipped, and that logged two
furniture-sized changes in session 4's bedroom, which nobody touched. A visit's
recent windows are now its reference only when the place flipped; otherwise the
whole visit is, as before. A thin thing hit from one place and seen past from
another is mixed in its last two minutes as much as before them, and is called
nothing.

## Cost

Six more arrays the size of the map, saved with the tallies; on this desktop a
scan takes 1.75 ms instead of 1.65 and a report 9 ms instead of 3, once every
30 s. A tallies file from before this loads, and each cell's windows start at
its next scan.

## What it does not show

That the moves autonomy would now be told of make its answers right; that
waits on the world model filing its looks to the things' records, and on stale
records giving way (R-WS-17). And a person standing still for a minute or more
while the rover drives past now reads as something new, where before it did
not, unless the visit began with them there; in the M5 sessions nobody was in
the rooms, so this is not measured.

R-AUT-13 held. No requirement changed state.
