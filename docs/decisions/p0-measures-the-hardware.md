# P0 measures what the hardware can do; it sets no accuracy bars

Status: agreed 2026-10-02 by the owner. This changes the plan and the acceptance runbook;
no code has changed.

P0 imposes no physical performance limits. How well the rover places things, ranges them,
points at them and answers its checks is **measured** on fresh trials and published as
what to expect from this hardware, condition by condition. It is no longer held against
a bar that fails P0 when the hardware falls short. The aim is to do as well as the
hardware allows and to know what that is.

What stays a condition is what the software can always meet, however good or bad the
hardware turns out to be:

- **Honesty.** The uncertainty the rover states for a direction, a range, a height or a
  placement covers the error measured against the tape. If the hardware is worse than
  the rover claims, the claim widens. It is the claim that is wrong, not the hardware.
- **Abstention.** A check that cannot see answers "can't tell". It never gives a
  confident wrong answer, and nothing acts on an identity it has not checked.
- **Safety boundaries hold.** Permissions, budgets, geofences, latches and the
  watchdog work as specified. Stopping time and distance are measured, and the
  geofence's margin is set from them; a slow stop does not fail a trial.

This revises M0's shared prerequisites 2 and 3, M0a's usefulness floor (criterion 3) and
M3's stopping limits (criterion 10) in the
[plan](../plans/autonomous-curiosity.md#milestone-m0-semantic-state-is-safe-enough-to-influence-goal-selection),
and the pass marks in the [acceptance runbook](../runbooks/m0-acceptance-drive.md). It
supports [R-WS-10](../requirements/world-state.md#r-ws-10), which already asks for
uncertainty to be "represented honestly" rather than for an accuracy target. It does
not touch R-SAFE-3, R-SAFE-4, R-SAFE-12 or R-WS-16.

## Why

Four drives, on 2026-09-07, 09-08, 10-01 and 10-02, failed the geometry tolerances, and
always on the same kinds of target:

- a painting behind chair backs, measured through the chairs: 0.88 m off on 10-02;
- small objects on the floor, below the depth camera's view at its usual tilt;
- the heights of raised things: 0.27 to 0.51 m too high on 10-02.

These are properties of the cameras and their mounting, and more drives in the same room
meet the same targets. Holding P0 to those bars would hold it on the hardware. The owner
had already ruled that out for P0 ("if the hardware is the limiting factor we are going to
have to find way to work around that rather than just fail P0").

## What was considered

- **Keep the bars.** P0 stays shut on hardware limits.
- **Declare a narrower envelope** (targets in view, not behind anything, height not relied
  on) and keep bars inside it. This was proposed on 10-02. The owner turned it down
  because it is still a constraint imposed on the hardware rather than a description of
  it.

## What this asks of the next work

The honesty condition is not met today. On 10-02 the painting behind the chairs was placed
with ±0.05 m claimed and was 0.88 m out, and the painting above the cabinet with ±0.18 m
claimed was 0.25 m out. Bearings measured a median 1.2 degrees, with 80% inside 3 degrees,
against a stated 1.5. So the stated uncertainties need calibrating against the measured
errors, which can be done on the recorded drives without the rover. M0a's runs then report
how often the check answers right, abstains or is wrong, and must show no confident wrong
answer.

## What would reopen this

A task that needs a minimum accuracy to be worth doing at all. Such a bar belongs to that
task when it is proposed, as identity does
([identity is judged action by action](identity-is-judged-action-by-action.md)), not to P0.
