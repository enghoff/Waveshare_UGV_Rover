# A run's look no longer waits on the world state's settling

**A run's geometry look now records its picture and leaves deciding identities
to the rover's own settling clock. It may wait up to 9 s for its turn, and
the executive gives that one call 13 s.** In
[M3 session 6](2026-10-06-m3-session-6.md), six of 39 looks failed on the world
state's lock. Three gave up after waiting 5 s for a look that had been running
6-8 s. Three were cut off by the executive's own 10 s timeout while settling
inside the lock. Each counted as a failed goal. Nothing here changes the world
state.

## Why the lock is held

Every look takes the inspector's one lock, and so does the pass that settles
identities. The rover's own clock runs that pass every ten seconds, and a run's
geometry look ran one too (`settle: true`). A pass decides over every bearing
still pending. With the 1,999 pending that session 6 found, against 348 that
morning, it takes most of the ten seconds: 8.4 s at 2,000 when measured on
2026-09-03. With the rover idle on its charger, the daemon still used 37% of a
core. The settling pass's cost belongs to the world-state work. This entry
only stops a run tripping over it.

## The change

- The executive plans a geometry look as `settle: false`
  (`executive.plan`). The rover's own clock settles it within ten seconds.
  What an evaluation straight afterwards loses by that was measured on the
  record at 9 goals in 171 ([the record](2026-10-06-looks-seldom-reach-their-thing.md)).
- The daemon lets a run's look that only records wait 9 s for the lock
  (`rover_autonomy.AUTONOMY_LOOK_WAIT_S`); a look that settles or keeps its
  depth keeps 5 s. 9 s outlasts a pass over 2,000.
- The executive allows that call 13 s (`LOOK_CALL_TIMEOUT_S`) and every other
  call 10 s. Nothing renews the permission during one call, and it is renewed
  just before, so 13 s stays inside the 15 s permit.

## Evidence

- Session 6's six failures are the reproduction: "an inspection has been
  running for 6-8 s" after a 5 s wait, and "autonomy_act: timed out" at
  10.0 s.
- `autonomy/test_executive.py`: a geometry look is dispatched without
  settling, its call gets 13 s while the drive's gets 10, and the client's own
  timeout is restored. `rover_daemon/test_inspection_limits.py`: a run's
  recording-only look waits 9 s, and a settling one 5 s. autonomy 789,
  rover_daemon 1121, drive_web 608 and ros_nav 574 passed.
- On the rover: see the deployment note below. **Not yet seen in a run.**
  The next session's looks are where it shows.
