# The first autonomous run on the world model's records: it ran end to end, and none of its 14 looks improved the record it was aimed at

**A supervised run planned around version 4's records for the first time**
([the move](2026-10-17-autonomy-plans-around-the-world-models-records.md)),
after the owner's handover on the morning of 2026-10-18. It ran end to end.
It read the records, chose and carried out 14 looks at them, recorded each as
not yet filed, put each aside, and drove back to where it started when nothing
in reach was left. The model filed every look two minutes later. **But none of
the 14 records gained a look**, and none was placed better: the looks went to
other records or to none. The goal it pursued does not improve the model's
records as they are chosen now.
[R-WS-17](../requirements/world-state.md#r-ws-17) stays proposed.

## The run

- `run/c4047614/1`, 07:32:38 to 07:35:28, fenced to the charger room, 900 s
  allowed: 15 turns, 28 actions, 7.1 m driven. It ended because everything in
  reach was put aside, waiting for its look to be filed, and it went back to its
  start, stopping 0.3 m from it.
- Pre-flight from a cold boot: GPU listed, position trusted and the map
  settled, no gyro fault, 605 records. The records were first written while
  the clock read 1970 and answered once rewritten a minute later.
- Every goal was `improve_geometry` at a record, ranked worst-placed first:
  spreads of 0.32 to 0.50 m, eight of the 14 seen on a single visit. Each
  attempt was recorded with `filed: false`.

## What the model did with the looks

- **Filed at 07:37:20**: 194 regions, 605 records to 612; filed up to 07:35:36,
  so every look of the run.
- **No target took a look.** Each of the 14 holds what it held before and its
  spread is unchanged. Of the 194 regions, 111 went to other records and 83 to
  none.
- **Eight looks were the same picture as the last one**: the rover's own looking
  loop had just taken it, so their regions came in under that picture. Of the
  six with pictures of their own, the regions nearest the target:
  - `v430`: one 0.12 m away, filed to `v568`. Probably one object held twice,
    the look going to the better-established record.
  - `v76`: one 0.01 m away, filed to nothing.
  - `v1117`, `v313` and `v1241`: nothing within 0.99, 1.92 and 1.25 m.
  - `v191`: two regions, neither ranged.
- **The worst-placed records are mostly ones the model cannot confirm.** A
  record placed from sight lines alone carries the model's nominal 0.387 m,
  which is not a measured spread, so aiming at its claimed place can miss it.
  One seen once may be a stray or a second record of something held elsewhere.
  Neither is lidar-visible absence, so a look that finds nothing there marks
  nothing gone, and the run would come back every fifteen minutes.

## Fixed on the way

- **Looks stamped before the clock was set.** After the cold boot the looking
  loop recorded eleven regions in 1970. The job filed them as a visit 56 years
  early and reported filed-until as 1970 until its next batch. Live batches now
  pass over them (`4d9fc71`); the history filed from scratch keeps them, as the
  desk's runs did.

## What it leaves open

Which records a look can improve. The candidates are leaving out records whose
spread is nominal (sight lines only, one visit), and giving the model a look
that saw the place and found nothing as evidence it is gone. The second is a
change to version 4 itself. Either is worked out on this run's looks before
the next run.
