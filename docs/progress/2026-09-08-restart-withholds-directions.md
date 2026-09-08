# Navigation was restarted mid-drive, and eighteen looks kept their pictures and lost their directions

**The hardware half of [R-WS-16](../requirements/world-state.md#r-ws-16) is
done.** During the drive of 2026-09-08 the owner restarted navigation while the
rover was still looking at things. Every region captured across that gap was
recorded with no direction at all, none entered geometric association, and all of
them kept their picture — and the looks either side of the gap are ordinary.

## What happened, to the second

Navigation was relaunched at **07:40:51**. The rover daemon was not restarted and
had been up since 07:18:35, so the semantic store, the perception path and the
entity set are continuous across the event: the only thing that changed is
whether the rover's place on its map was confirmed.

| time | regions | direction | attached | picture |
|---|---|---|---|---|
| 07:40:44 | 10 | none | none | all kept |
| 07:40:51 | — | *navigation relaunched* | | |
| 07:41:05 | 8 | none | none | all kept |
| 07:41:09 | 10 | all ten | seven of ten | all kept |
| 07:41:13 | 9 | all nine | seven of nine | all kept |

The first silent look precedes the relaunch by seven seconds, which is the old
stack being torn down: the gate starts refusing as soon as the answer stops being
available rather than when the new process appears. Directions come back 18
seconds after the relaunch.

Three of the eighteen silent regions carry a measured distance — 2.08 m, 1.68 m
and 1.63 m — recorded and kept while the bearing that would have made them
placeable was withheld. That is the shape the requirement asks for: the evidence
survives, the geometry does not.

## The withholding is permanent, which is the half that matters

Those eighteen rows still carry no bearing an hour later, with the map long since
settled and the rover placed. Nothing back-fills a pose onto an observation,
because the gate is at capture and not at read time — which is exactly the trap
[R-WS-16](../requirements/world-state.md#r-ws-16) names. A later confirmation
cannot retroactively validate a bearing that was recorded before it, and here it
demonstrably did not.

The deployed mechanism is `rover_world._world_pose`, which refuses a direction
unless navigation reports both a trusted position and `map_settled`
([R-NAV-2](../requirements/navigation.md#r-nav-2)). The offline half — that the
gate withholds and that a later confirmation does not give the bearing back — has
been covered by `rover_daemon/selftest.py` since 2026-09-07 and by
[the refit report](2026-09-07-refit-window.md); what was owed was this.

## Eighty-one other looks were refused for reasons the row does not say

Across the whole drive 99 of 1328 looks got no direction, so the restart accounts
for 18 of them. The rest fall in ten separate stretches, the longest 21 looks
over 61 seconds at 07:31:37, none of them near a restart.

**That is the gate working, not misfiring** — all 99 are unattached with their
pictures kept — but the row does not record *which* of the two answers was
missing. A look refused because no fresh transform existed and a look refused
because the rover's place was not confirmed are different events with different
meanings, and both are currently written as an absent bearing. This is the same
gap the depth refusals had until they were made to
[say which silence they were](2026-09-07-both-remedies-deployed.md), and the same
remedy would fit.

## Requirements

- [R-WS-16](../requirements/world-state.md#r-ws-16) moves to `settled`. The gate
  is deployed, the offline behaviour is covered, and the hardware demonstration
  exists: a navigation restart on the rover withholding directions from eighteen
  regions while keeping their pictures, with no later confirmation putting a
  bearing back.
- M0 criterion 11 is met, on this evidence together with the replay half already
  recorded.
- Nothing was changed or deployed for this entry; it records a demonstration on
  the build that was already running.

## Next

Make the capture path say which answer was missing when it refuses a direction,
so the 81 non-restart refusals can be read. It is a small addition to
`rover_world._world_pose` and it is what turns "no bearing" from a silence into a
fact, exactly as the depth refusals were changed.
