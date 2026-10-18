# Absence from the camera would clear the world model's strays, but it calls records of things that were there gone

**Version 4 has no way to drop a stray.** It marks a record gone only when the
lidar sees through its place. On the rover, 284 of its 502 standing records had
been seen on one or two visits
([entry](2026-10-18-aimed-looks-feed-only-established-records.md)), and looks
aimed at such records feed nothing. Taking absence from the camera too was
tried on branch `visit-model`: a close look at a record's place that finds
nothing there counts as an empty visit, and enough in a row mark it gone.
Replayed over every picture to the drives of 2026-10-17, it clears a quarter
to a half of the standing records. **But in every form tried it also calls
records of objects that never moved gone**, so it is not adopted. With the rule
switched off the filing is version 4's byte for byte.
[R-WS-17](../requirements/world-state.md#r-ws-17) stays proposed.

## What was measured

An empty visit is one whose pictures had the record's depth place within
2.5 m and 45 degrees of the camera's axis in three or more of them, and that
filed nothing to it. "Never moved" means the eleven objects that were never
staged (`labels/stayed-put.json`), which were there throughout.

- **How often the model misses a thing that is there**, on version 4's own
  filing: 11% of visits with three pictures within 2.5 m filed nothing to a
  never-moved object's records. The longest run of such visits in a row was
  three, the snowy-tree painting's.

| An empty visit means | In a row | Gone by the camera | Records of never-moved objects called gone | Standing at the end |
|---|---|---|---|---|
| (version 4, the rule off) | | 0 | 0 | 518 |
| nothing filed to it | 4 | 546 | 10, among them the blue painting's, the rug's and the forest painting's main records | 284 |
| nothing filed to it, and no region of the visit placed within 0.5 m | 4 | 254 | 2: a second record of the rug (8 labelled pictures), one with one picture of the animals painting | 363 |
| the same | 8 | 131 | 1: one with one picture of the animals painting | 426 |

"Nothing filed to it" mistakes a look filed to a second record of the same
thing for the place found empty. "Nothing there" leaves every main record
alone. It still calls records gone whose things were there, all of them second
records or one-picture ones of objects held elsewhere. Called gone, such a
record lets the rover say the rug is no longer where it was.

## What it leaves

Whether a record looked for and not found should be set aside without being
called gone. The rover would then say only that it did not find the thing
there again, which is true of a stray and of a second record alike. That
changes what the voice says and needs a staged session to test, so it is the
owner's call.
