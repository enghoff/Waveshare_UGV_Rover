# M4 asks its question of things out of reach; a thing in reach is looked at from where the rover stands

Status: agreed 2026-10-09 by the owner, choosing between the two ways on that
[the development attempts](../progress/2026-10-09-m4-development-attempts.md)
left. Changes M4 in the [plan](../plans/autonomous-curiosity.md) and the
autonomy loop's geometry goals ([autonomy/README.md](../../autonomy/README.md)).
Narrows [m4-measures-where-things-are.md](m4-measures-where-things-are.md),
whose reopening condition the attempts met; its score, its re-look and the rest
of its criteria stand.

Seventeen development attempts on the things taped on 2026-10-03 found a look
from where the rover stood improving its thing as often as driving to the
viewpoint the rover chose: 4 against 3, neither ever worse, a paired difference
of -0.07 (-0.27 to +0.09). The re-looks that helped were 1.7 to 3.3 m from
their thing. Inside a room, a thing is usually within the depth camera's reach
of wherever the rover already is, and there the drive buys nothing a turn does
not. So:

- **A thing in reach is turned to and looked at, not driven to.** In reach means
  within the depth camera's ranging limit of 4 m
  (`world_state.hypothesis_check.DEPTH_FAR_M`), and at least 0.5 m, from where
  the rover stands, in the depth camera's view at a calibrated tilt, with nothing
  on the map in the way. Such a thing gets one candidate, at the rover's own
  spot, carrying the gain the best viewpoint predicts. Every run, trial or not.
- **M4 asks only of things out of reach.** An attempt applies when its thing was
  out of reach of where the rover stood when it chose; a trial refuses a thing in
  reach as `in reach`, since its re-look would be the same look. Criterion 3's
  comparison with the re-look is then the question it was meant to be: whether
  going somewhere chosen adds what looking again cannot.
- **Acceptance runs start away from their things.** Whether a thing is out of
  reach depends on where the rover is, so acceptance attempts come from runs
  opened in another room from their targets, or across a room from them.

## Why

**The measurement beats the model.** The viewpoint planner predicts little gain
from where the rover stands, because a bearing along the same line as earlier
ones pins nothing new. But what improved the things here was depth, and the depth
camera ranges a thing from wherever it can see it. The predicted gains were 0.06
to 0.87 m on all seventeen attempts and fourteen gained nothing.

**Every drive costs battery the rover cannot spare.** A charge gives 20 to 25
minutes of driving. A turn and a look take seconds.

**Out of reach is where the choice can matter.** A thing too far, too high to see
from here, or behind something has nothing a re-look can give it, so whether the
rover chose a good place to go is a real question there, and the comparison is
not settled before it starts.

## What was considered

- **Keeping the comparison and changing its baseline** to the nearest reachable
  viewpoint, as the earlier decision allowed. It asks whether choosing beats
  driving anywhere, after the attempts showed driving at all buys nothing for
  things in reach; well over a hundred paired attempts, two drives each.
- **Dropping the comparison** and judging M4 on mean gain and honesty alone. That
  passes on looking at things, which nobody doubts helps, and says nothing about
  choosing where from: the formality the review of that morning set out to
  remove ([every-gate-says-what-it-is-for.md](every-gate-says-what-it-is-for.md)).
- **A shorter reach, 3.3 m**, the furthest re-look that helped. Four helpful
  re-looks are too few to set a limit by; the depth camera's own limit is the
  one with a reason.

## What would reopen it

- A look from where the rover stands improving its thing clearly less often
  than a chosen viewpoint's for things in reach, in later runs. The record keeps
  both, since a trial still re-looks before every drive.
- Out-of-reach attempts too rare to size a comparison by: if runs seldom find a
  thing out of reach, M4's question is not one this flat asks often enough to
  matter.

## Requirements

Relies on [R-WS-13](../requirements/world-state.md#r-ws-13)'s filing by aim, as
before. Adds and retires no requirement.
