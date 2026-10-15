# A world model kept per visit: unmoved things stay together and nearly every picture is filed, but no moved thing's record follows it, and the test does not pass

**Built offline and run once on M5 sessions 3 and 4, the per-visit model fails
two of the plan's five conditions.** One record of the animals painting took
one picture of the snowy-tree painting beside it. The blue painting sits in
two records where the deployed store has one, although the deployed store
files only a quarter of its pictures. The other three conditions pass. All
five staged moves count as kept, against the deployed store's three. Ten of
eleven unmoved objects sit in two records or fewer. 95% of the labelled
pictures are filed, against 63%. [R-WS-17](../requirements/world-state.md#r-ws-17)
stays proposed.

**Following a moved thing is still not solved, by either system.** All five
"kept" moves are kept only by the rule the test inherited from criterion 4:
the record holding most of the thing afterwards held any one picture of it
before. By the strict reading, where that record is the one that held most of
it, both systems keep none. In the test the model never once moved a record:

- The cabinet's arrival went to an older record of the cabinet, left at the
  place it stood in session 2.
- The red bean bag's arrival looked as much like the black bean bag's record
  (0.74) as like its own (0.73), so the model declined to choose.
- The plain armchair's arrival went to a duplicate record of itself.

## What was built

`experiments/visit_model` on branch `visit-model`, as
[the plan](../plans/a-world-model-kept-per-visit.md) describes:

- **One cluster per object per visit**, by place, size, height and likeness,
  one region per picture.
- **One assignment of each visit against memory.** Objects that stand together
  and are never seen in one picture are merged.
- **Absence**, from the lidar seeing through a place in the drive recordings.
- **A new object stays pending for ten minutes**, so that it can still turn
  out to be a gone one, moved.

Two inputs changed during development, each after measuring them:

- **Raised things are placed where their sight line meets a wall** of the map
  that stayed in every session's grid. From more than 2.5 m this lands 8 cm
  from the object at the median, against 28 cm for depth.
- **A lidar range is a floor, not a place.** It lands 66 cm from its object
  at the median, because it answers with whatever stands nearest in its plane.

Every spread and likeness bar was set from the development labels
(`calibrate.py`). A likeness favours "the same object" by more than 15 to 1
only above 0.75. A quarter of pictures of one object from different sides
score under 0.49.

## The benchmark

- **The data**: every look from 2026-10-03 to session 4, 44,466 regions, in one
  map frame.
- **Labels**: the five staged things across the shakedown and sessions 1 to 4,
  and eleven objects that never moved: five paintings, the front door, the rug,
  the bed, the gold painting over it, a portrait and a wardrobe.
- **Development**: the shakedown and sessions 1 and 2. The model saw no look
  after 2026-10-14 13:00 until the test.
- **A label correction during development**: eight of the village painting's
  regions were a smaller painting beside it, and sixteen more were too near
  the picture's edge to tell. All 24 were left out.
- **Two amendments to the test, both before it ran**: the model must file at
  least the deployed store's share of pictures, so that leaving pictures
  unfiled cannot pass; and records per unmoved object are compared counting at
  least one, since the deployed store holds the front door's test pictures in
  none.

| | Moves kept (strict) | Records holding two objects | Unmoved objects in two records or fewer | Labelled pictures filed |
|---|---|---|---|---|
| deployed, development | 1 of 7 (0) | 3 | 3 of 11 | 69% |
| per visit, development | 4 of 7 (3) | 2 | 9 of 11 | 88% |
| deployed, test | 3 of 5 (0) | 0 | 4 of 11 | 63% |
| per visit, test | 5 of 5 (0) | 1 | 10 of 11 | 95% |

On the test, the records per unmoved object were, per visit against deployed:

| Snowy tree | Animals | Forest | Village | Blue | Door | Rug | Bed | Gold | Portrait | Wardrobe |
|---|---|---|---|---|---|---|---|---|---|---|
| 2 / 3 | 2 / 5 | 1 / 1 | 1 / 4 | 2 / 1 | 1 / 0 | 3 / 6 | 1 / 3 | 1 / 1 | 1 / 3 | 1 / 4 |

## What it means

Keeping a thing that stays put in one record is largely solved, and it was
solved by measuring where places land and averaging over a visit, not by
anything clever. Following a thing that moves is not. The test points at two
causes:

- **An object has several records, one per place it has stood.** No moved
  record was ever folded into another, so an arrival finds an old record of
  itself at its new place.
- **The masked image vectors barely tell look-alikes apart**: two armchairs,
  two bean bags of different colours, two paintings on one wall.

Both are addressed in the next version of the plan. The test data is spent:
the next version needs a session it has not seen.
