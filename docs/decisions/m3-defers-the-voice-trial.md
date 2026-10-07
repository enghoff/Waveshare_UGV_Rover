# M3 does not wait for a voice trial

Status: agreed 2026-10-07 by the owner; changes M3's criterion 12 in the
[plan](../plans/autonomous-curiosity.md). No code has changed.

M3's criterion 12 asked that "concurrent voice/manual requests follow the
declared priority". For M3 that now means the manual half: a person's move sent
while an autonomous leg is driving ends the run, latches autonomy off, and is
itself carried out. Showing the same with a voice request through the realtime
model is no longer a condition of M3. It is to be picked up again if voice
becomes relevant to autonomy. That would be when voice is given a way to set a
run's purpose or goals, which the
[design](../plans/autonomous-curiosity-design.md) proposes and no phase yet
includes.

## Why

A voice request reaches the daemon as the same tool call a console or an agent
makes. It passes through the same `Rover.call` that ends a run and hands over
the wheels, so the priority it would test is the one the manual trial tests.
What a voice trial adds is the model's part: hearing the words and choosing
the call. That is a test of the voice feature, not of autonomy. It also needs
the owner speaking into the console's microphone at the right moment. One
attempt on 2026-10-06 was lost because the microphone was off.

## What was considered

- **A synthesised phrase fed to the model on the rover**, which needs no person
  at the microphone. It costs free-quota tokens on every attempt, and it would
  still only prove what the manual trial proves about autonomy.
- **Keeping it in M3 as asked.** Declined by the owner.

## What would reopen it

Voice gaining any path into the executive: a goal, a purpose or a policy set by
speaking. A request heard from the model would then do more than take the
rover back, and its priority against a running goal would need its own trial.
