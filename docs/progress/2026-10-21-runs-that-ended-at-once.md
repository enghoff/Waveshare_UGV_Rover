# Runs that ended at once were refusing most of the house as having nowhere to stand, because only the sixty places nearest the rover were tried

On 2026-10-21 four autonomy runs started between 09:02 and 09:07, three on the
charger and one in the doorway by the corridor. Each ended within a second
with "there was nothing left worth doing". The sentence the daemon logged
named two changes refused as a door and a few things under the 0.15 minimum.
It was cut at 400 characters, though. Re-deciding the runs' recorded
decisions from their snapshots, with the deployed code, shows what the cut
hid: **8 things from the charger and 48 from the doorway refused as having
nowhere to stand, every one with all 60 of the places it tried blocked by a
wall.**

## Reproduced first

The episode store was copied on the rover with sqlite's backup, and each
decision since 2026-10-16 was re-decided from its own snapshot (`replay`,
`situation.restore`, `scoring.consider` with `go_further` on, as the executive
sets it). The deployed code reproduced every one of the morning's empty
decisions.

`goals._viewpoints` took the sixty places in the certified band (0.5-2.5 m)
nearest the rover's walk, and then tested each for a wall in the line of sight.
Those sixty are one patch on the rover's own side of the thing. When a wall
stands between that patch and the thing, all sixty are blocked, and the thing
is offered as "nowhere to stand", which `scoring.py` vetoes as unreachable.
Tested over the whole band instead:

| Decision | Rover at | Things with nowhere to stand | ...of which the band had a place that sees it |
|---|---|---|---|
| episode 2043, 09:07 | the doorway (-10.79, -13.30) | 48 | 41, most with hundreds of such places |
| episode 2039, 09:02 | the charger (-18.22, -15.11) | 8 | 7 |

This is not new. It is in every session sampled since 10-16, and the count
follows where the rover stands: 6 to 12 from the charger, 30 to 48 from the
corridor.

## The change

The sixty places weighed are now the sixty nearest the rover's walk **that can
see the thing**. The line test runs over the band until sixty pass, and only
the prediction, the costly part, stays bounded (`autonomy/goals.py`; the
regression test is `test_a_wall_on_the_rovers_side_does_not_hide_the_far_side`,
which fails on the old code). The look at a change already worked this way.

## What it does to the decisions

Over the 83 decisions since 10-16, re-decided with each version on the rover:

- **24 that chose nothing now choose a goal.** Every one of this morning's
  empty decisions now goes to one record, `v125`, from 2.3 m off the charger
  (gain 0.17, just over the minimum). The 48 things refused from the doorway
  mostly turn out to have nothing to gain from a look. The 0.25 to 0.52 they
  carried while refused was the stand-in for a thing nobody could look at, not
  a prediction. So the house is close to done, and these runs will do one or
  two looks and then end honestly.
- **9 choose something slightly worse**, all on 10-16 and 10-17. The
  generator weighs the twelve worst-placed things that have somewhere to stand
  (`ENTITY_LIMIT`). With fewer things written off, the twelve fill sooner, and
  a few looks from where the rover stands, at better-placed things, drop out.
  Utility falls by under 0.05 in eight of them and by 0.12 in one. Raising the
  limit to 24 leaves 2 of the 9, but a decision then takes 6.3 s on average
  against 3.9 s now, so it was not adopted.
- **A decision takes 2.5 s on average and 9.3 s at most, against 3.9 s and
  7.6 s.** The written-off things had been costing the most.

Requirement [R-AUT-13](../requirements/autonomy.md#r-aut-13) is held, not
moved: a run with nothing left worth doing still goes back and ends. What
changed is that "nothing" is now true more often. Not yet seen on the floor.
