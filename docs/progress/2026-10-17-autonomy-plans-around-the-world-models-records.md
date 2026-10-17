# An autonomous run plans around the world model's records, with no fallback to the store

**Autonomy now reads version 4's records instead of the store's entities, the
third and last consumer moved over** ([the plan](../plans/a-world-model-kept-per-visit.md),
step 8, after [the voice tools](2026-10-17-the-world-model-answers-find-thing.md)
and [the console](2026-10-17-the-console-shows-the-world-models-records.md)).
Unlike those two it has **no fallback to the store's entities**: a run carries
the records' names from one step to the next, and the store names the same
things differently; the choice was put to the owner the same evening. Proved on the rover by reading and
deliberating, without moving it; no run has driven on the records yet.
[R-WS-17](../requirements/world-state.md#r-ws-17) stays proposed;
[R-AUT-2](../requirements/autonomy.md#r-aut-2) holds, since the records'
names are now minted under the model's own generation.

## What was measured, on the rover

- **A reading** (`situation.Situation.read`, every call a read): 0.6 s, 609
  records under generation `2e9fb5c76f8ce1d1`, 114 of them gone and kept in
  the reading, 495 planned around.
- **The first deployment found nothing worth doing.** A deliberation took
  26 s and every one of its 170 candidates was refused: 167 "nowhere to stand"
  geometry goals and 3 frontiers. The store's entities against the same map gave
  29 geometry goals, 10 of them usable, in 2.1 s. The goal arithmetic is the
  store's: a placement has a long axis that a new bearing crosses. A record
  keeps one spread and no axis, so no look could gain anything, and the search
  for a thing worth a trip walked all 495 records.
- **With a record's gain worked out as the model works it** (a look the depth
  camera ranges, combined by inverse variance with 0.25 m, the spread the model
  gives an outlined depth place within 2.5 m): replayed on the same reading,
  0.8 s on the desk, 12 geometry goals, 8 of them usable. Deployed, `decide.py`
  took 3.3 s on the rover, process start included, and would turn to look at
  `v1492` from where the rover stands (placed to 0.50 m, about 0.22 m after).

## Choices made

- **When the records cannot answer, a reading holds no things and says why.**
  A run still explores and looks at changes. Names from two systems in one run
  would make strangers of the goal it just chose and what it put aside.
- **A record the model has seen gone stays in the reading** and is planned
  around by nothing.
- **A look the model has not filed yet says so.** The model files a look when
  its visit closes and a minute after. That is up to about four minutes, so an
  attempt evaluated straight away reads the placement from before it. The
  attempt records `filed: false` instead of "no better", and the thing is put
  aside until the filed look improves it, or for fifteen minutes.
- **Looks no longer name their target to the store**
  ([the decision](../decisions/aimed-looks-file-to-their-target.md), switched
  off). The store's aimed filing does not know the model's records, and the
  model files the look itself. The two store-only rules that hung on it, two
  hours aside for a look that found nothing and setting aside the store's
  same-object suspects, are idle. The model instead marks a missing thing gone.

## Found on the way

- **The world model had stopped for half an hour that evening.** A power cut at
  20:51 left an empty lidar file under a real name. The job's loader did not
  catch its `EOFError`, and every batch failed until the file aged out of the
  job's 30-minute buffer at 21:21. Meanwhile the records went stale, and the
  voice tools and the console answered from the store. The job now puts every
  file on disk before naming it, skips an unreadable lidar file, and keeps an
  unreadable saved engine aside while it files the history afresh.

## Not settled

- **A thing above the lidar's plane that is taken away** is seldom marked gone,
  so a run may look at its old place once every fifteen minutes and find
  nothing; the store's two-hour rule for that is idle.
- **The model is pinned to map session 67.** If navigation starts another map,
  the records stop answering everywhere, and a run plans around no things until
  the model follows the map.
