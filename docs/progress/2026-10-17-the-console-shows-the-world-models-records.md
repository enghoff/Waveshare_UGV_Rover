# The console's world popup shows the world model's records, with what it has seen gone behind a box that is off by default

**The drive console's world popup now lists and maps version 4's records**
instead of the store's entities, the second consumer moved over
([the plan](../plans/a-world-model-kept-per-visit.md), step 8; the voice tools
went first, [the same day](2026-10-17-the-world-model-answers-find-thing.md)).
At the owner's request, records the model has seen gone are left off unless a
**gone** box beside the evidence bar is ticked, and are then drawn faded with
when they went; the box is clear on every load. The executive still reads the
store's entities. [R-WS-17](../requirements/world-state.md#r-ws-17) stays
proposed.

## What was measured, on the rover

- **The list**: 624 records, 121 of them gone, 1.8 s and 1.7 MB to answer,
  against the store's 741 entities at 9.3 s and 3.3 MB on the same call. The
  first build took 13.6 s: every record's newest look was worked out by reading
  every look again for each record; it is now worked out once per version of
  the records.
- **The page**, opened headless with the devtools protocol (deployed at
  `4bb1fa9`): with the box clear, 69 records on the map, none faded, and "121
  gone" counted under it; ticked, 96, of them 27 faded. The header counts the
  503 records in place as "records", where it had flipped between the model's
  count and the store's every couple of seconds.
- **The search** names each matching look by the record that holds it, so the
  phrase narrows the list of records.

## Choices made

- **A gone record is thinned only against other gone records**, so ticked, a
  faded mark sits beside whatever stands near its old place now; a present
  record within a metre had hidden all but 2 of the 121.
- **A record's row says how many visits it was seen on**, not how many places
  agreed: the model does not count viewpoints.
- **The observation stream stays the store's**, with the store's entity on each
  look; the evidence bar's placement steps still apply, to visits and spread.
