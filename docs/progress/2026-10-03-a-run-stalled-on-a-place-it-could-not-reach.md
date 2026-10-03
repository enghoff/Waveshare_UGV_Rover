# A run stalled driving at a place it could not reach

**The first run after the morning's fixes, run/527d5efb/2, ran for 15 minutes and
22.9 m, then stalled.** Navigation could not reach one patch of unmapped floor,
about 5 m south, and the run tried it four times, each a forty-second stall
while Nav2 ran its recoveries. Three failures in a row then ended the run. Fixed
in autonomy and deployed; no run has shown the fix yet.

## What went wrong

The run's own route check, a walk over the map's free cells, put the frontier
about 5 m away. Navigation, which needs room for the rover's whole body, found
no route the rover fits through. It reported a way round of 37 m for a 5 m goal,
and moved the goal 11 to 27 cm to find room to stand. The walk is a cheap
ranking by design, and the rover's own exploring blacklists a frontier once
navigation fails to reach it. The run had no such list, so the same frontier,
re-detected a few centimetres away each time, was chosen again after every
failure.

Replayed from the record, today's chooser picked that frontier again at each of
the three decisions after the first failure. With the failed place set aside, it
picks a geometry goal nearby each time.

## The fix

A drive that fails or times out writes its goal's place down. For half an hour,
any goal within half a metre of that place is refused, with navigation's reason
attached. A test fails without the fix and passes with it; autonomy 741 passed.

Also in this deploy: a look aimed straight ahead no longer swings the gimbal
when it is already there. Before, every aimed look made two 30-degree swings,
one to aim and one back to rest. rover_daemon 1088 passed.

## What the run did otherwise

59 turns and 113 actions. Two geometry goals improved a placement, by 0.10 m and
0.04 m; the rest put their thing aside. That is better than run 4's one
improvement in nine, but most looks still change nothing. Whether the
predictions hold now that each look faces its thing is not yet measured.

## Requirements

[R-AUT-13](../requirements/autonomy.md#r-aut-13) is still open. This run ended on
three failures, not by going home. [R-SAFE-16](../requirements/safety.md#r-safe-16)
held: three failures in a row ended it, as intended.
