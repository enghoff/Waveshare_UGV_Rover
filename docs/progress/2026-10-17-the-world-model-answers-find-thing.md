# The world model runs on the rover and answers "where is it": it files the history exactly as the version that passed its test, and says when a thing has been taken away

**Version 4 of [the world model kept per visit](../plans/a-world-model-kept-per-visit.md)
now runs on the rover beside the store, and the voice tools `find_thing` and
`go_to_thing` answer from its records.** The owner chose it over the store's
grouping earlier the same day
([the comparison](2026-10-17-version-4-against-the-deployed-store-on-unseen-pictures.md)).
It files the rover's history exactly as the frozen version that passed M5
session 7 does, it keeps up with new looks a minute at a time, and asked where
the bean bags and the stool are, the rover now says they are no longer where
they were last seen, which is true; the store still placed them there. The
console, the executive and the resolver still read the store's entities.
[R-WS-17](../requirements/world-state.md#r-ws-17) stays proposed.

## What runs

- **A job beside the store** (`world_state/visit_model/`, `cfaaad7` on): it
  reads the looks the store adds, once a minute old, files every visit that has
  closed and writes the records to `~/.ugv/world/visit_model/things.json`. It
  never writes to the store. A lidar logger in navigation's environment writes
  the last 40 minutes of scans and pose to disk for its absence evidence, and
  each filed batch's lidar is kept as a recording, so ordinary drives now leave
  recordings behind. Both start at boot from the crontab and restart after every
  world_state deploy.
- **The voice tools rank its records** by the average of every look each holds,
  as they ranked the store's things (`rover_daemon/rover_recall.py`,
  `world_state/visit_things.py`), with the store answering when the records are
  missing, a quarter of an hour old or on another map.

## What proves it

- **In pieces as whole.** Fed the 2026-10-17 history in 12 or 40 pieces, the
  engine files every region exactly as one run does (same groups, same record
  names), in 14 s.
- **On the rover as on the desk.** The inputs the rover builds from its live
  store are bit-identical whether built in pieces or whole, agree with the
  desk's to a millionth on all 48,606 shared regions, and the rover files the
  history exactly as the frozen desk run (same groups, same names). The first
  attempt differed because the scratch copy lacked the lens module the view
  helpers load; the deploy tree has it.
- **On the rover, over the daemon's port** (after the deploy, `b49afb4`), seven
  phrases, all answered from the world model per the daemon's log:

  | Asked | Answer |
  |---|---|
  | a painting of trees in snow | the snowy-tree painting's record, 820 looks, last seen 2 hours ago |
  | the bed | last seen 4 hours ago, where it stands |
  | a red bean bag | no longer where it was last seen, 5 hours ago |
  | a wooden stool | no longer where it was last seen |
  | a black armchair | a record still in place, last seen 76 hours ago |
  | a front door | last seen 5 hours ago |
  | a giraffe | not seen |

  No distances: navigation had not confirmed its position since it was
  restarted by the deploy (below).
- **Tests**: `rover_daemon/test_recall.py` checks the records answer first,
  `go_to_thing` heads for them, a gone record says so, stale or other-map
  records leave it to the store, and of two alike the present and recent one
  answers; 1,263 daemon checks and 1,199 world_state checks pass on the rover.

## What was found on the way

- **A record's own last-seen time can be days old** while its newest look is
  hours old: the model's merge keeps the survivor's time. The first answers
  said the snowy painting was last seen 191 hours ago; the tools now report a
  record's newest look.
- **A phrase scores the records of one object within about 0.01 of each
  other**, and the first answers took an old record of a black armchair seen
  gone, and a bed record a week old. Of matches that close to the best, the one
  still in place and seen most recently now answers.
- **Navigation's Python is not the system's**: the job crashed on its first
  start for want of PIL in RoboStack's conda environment, which also has its own
  numpy. The model runs on the system Python, where it was verified, and only
  the lidar logger runs in navigation's environment.
- **Changing the deploy manifest redeployed every component**, navigation
  included, and navigation came back without confirming its position (a
  heading of -60 degrees reported for a rover facing about 120). A refit is due
  before the next drive.
- **The step ladder, taken away after session 7, is still a present record**:
  thin things seldom give the lidar the hits that mark a place empty, and things
  above its plane never do.
