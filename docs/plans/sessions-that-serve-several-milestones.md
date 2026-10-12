# Sessions that serve several milestones at once

Status: proposed 2026-10-11 at the owner's request, to keep the rover's
supervised time for M5 and what follows as small as it can be. The session
script is built and was
[shaken down](../progress/2026-10-12-m5-shakedown.md) on 2026-10-12; *What has
to exist first* is the work before the first session.

The rover's time is the scarce thing: R-SAFE-6 is open and nothing docks it, so
every minute is supervised and every charge plugged in by hand, at 20 to 25
minutes of driving a charge. Until now each run served one purpose -- a control,
a scene, a trial -- and drives between runs were not even recorded. This plan
makes one session per charge serve every milestone that needs the hardware, and
moves everything else onto recordings.

## Rules

1. **Offline first.** Anything that can be shown on recordings is shown there
   before a session is asked for: fixes, thresholds, scorers, goal types. A
   session is only for evidence that has to be fresh -- acceptance counts, which
   cannot come from the recordings the code was tuned on -- and for behaviour
   that has to happen on the floor.
2. **Every minute is recorded.** One recording runs from the session's start to
   its end, drives between runs included, with the world store's looks and the
   episode log kept beside it. A session that is not recorded is a session spent.
3. **A fixed survey loop is the instrument.** The same route through every room
   the rover can reach, stopping at the same viewpoints, driven by a person's
   drives and not the executive. The same viewpoints each time make visits
   comparable, two or more viewpoints a place satisfy the change watch's
   viewpoint rule, it never runs out of things to do as the executive does, and
   it is also the fixed-schedule baseline the multi-day benchmark compares
   autonomy with.
4. **Everything not changed is a control.** Every loop compares every room with
   its last visit, so a loop is a control for each room nobody touched; the
   agreed rule ([two clean controls](../decisions/m5-moves-on-after-two-clean-controls.md))
   lets every later run count.
5. **Scenes are staged out of the rover's sight,** while it waits in another
   room, unless the point is a change it watches. A reset is staged between
   sessions and found by the next session's first loop, so it costs no run.
   Nothing is put within a metre of the charger's spot or a stop: on the
   shakedown, armchairs there blocked a stop, made the position check unreliable
   at two more, and kept the rover off its charger.

## One session

About an hour of the owner's day, twenty minutes of it in person; one charge.

| Step | Rover | Owner | What it gives |
|---|---|---|---|
| 0 | on the charger: the code frozen and deployed, the counts predeclared, the session script dry-run | -- | -- |
| 1 | recording starts; battery and pose checked | unplugs; a refit if asked | -- |
| 2 | **loop A**, 3.5 min | -- | the "before" visit everywhere; the previous session's reset found (criterion 2); a control in each room (criterion 3) |
| 3 | waits in the bedroom, out of range of the rest | stages 3-5 changes elsewhere: furniture-sized things moved at least a metre, one door, and when it is being tested one change in the rover's sight | -- |
| 4 | **an autonomous run**, a loop's length | -- | the executive's choices with changes on the map (criteria 5-6, once it has the goal type); the benchmark's autonomy arm; outcomes for M10; M6's skill executions once there is one |
| 5 | **loop B** | -- | every staged change, found or missed (criterion 2); controls elsewhere (criterion 3); the benchmark's baseline arm |
| 6 | home; recording stops; the session's counts written by script | plugs in; resets the scene now or before the next session | battery under a driven discharge (R-CTL-11); pose quality room by room |

Steps 4 and 5 swap on alternate days so neither arm always sees the scene first.

## How many sessions

| Milestone | What it needs from the hardware | Sessions |
|---|---|---|
| M5 | a detection rate with its interval (criterion 2), false alarms on controls (criterion 3), the executive choosing changed places and losing interest once refreshed (criteria 5-6) | **three**, one a day, after **one shakedown** that counts as development. Four moved things a session give eight changes and four resets the next morning: about forty events over three sessions, enough for an interval of roughly ten points; about twenty room-controls ([counted](../decisions/m5-sessions-are-counted-before-they-start.md)) |
| Multi-day benchmark | three separate days, autonomy against a fixed schedule, restarts and a map reset, scripted changes and controls | **the same three sessions**: loop B is the fixed schedule, step 4 the autonomy; a restart in one session and a map reset in another, at a step where they cost nothing |
| M6 | a few supervised runs of the reference skill stopping through the ordinary path | folded into step 4 of any session after it is built |
| M7, M8 | paired physical trials, sized when there is a skill | folded into step 4 of later sessions; one dedicated session only if the pairing needs more attempts than step 4 holds |
| M9 | none | recordings only |
| M10 | every action's outcome as training data; one hardware confirmation | data from every session; the confirmation folded into a later session |

So M5 and the benchmark take four sessions rather than the twenty-odd runs they
would take one purpose at a time, and M6 to M10 add little or nothing until
they need paired trials.

## What has to exist first

The shakedown drove every step of the session script, and its counts were
scored by script ([2026-10-12](../progress/2026-10-12-m5-shakedown.md)). What is
left:

- **The pose among moved furniture.** In the shakedown's run it went 1.35 m
  wrong beside the moved armchairs while navigation called it trusted. A session
  that loses its place cannot count past that point or get home. It is replayed
  into the mapper from the graph the session started with, which the script
  kept, before anything is changed. The script now stops where it stands when
  two stops running disagree. The bedroom doorway of 2026-10-11
  ([measured](../progress/2026-10-11-bedroom-pose.md)) did not recur in five
  passes.
- **The counts agreed**: what counts as found, a false alarm and its budget
  (set at under one room-control in seven), and the benchmark's questions and
  minimum improvement, all
  [proposed](../decisions/m5-sessions-are-counted-before-they-start.md). The
  scripts that count them have scored the shakedown.
- **The executive's look at a change, turned on** (`investigate_changes`), the
  owner's call. On the shakedown's recorded decisions it would have looked in
  10 of 31, the first 3 min 25 s into the run: at a loop's length, the run
  barely gets to a look, so the switch and the run's length are one question.
- What comes and goes at the corner by the balcony, seven times in the log
  since 2026-10-10: the owner's word, or a look from close to, so that a
  control's count can name it. The shakedown did not get to it.

A move made while the rover watches is not here: scenes are staged out of its
sight (rule 5), and the rule that would see one costs false reports on replay
(open in the [replay's entry](../progress/2026-10-11-m5-scene-1-replayed.md)).
