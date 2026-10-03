# The first run from the console looped on one goal, and a fruitless goal now puts its thing aside

**The first run started with the console's button, run/e3efe1d1/2, chose the same look
at object:7 twenty-two times in a row, travelling 2 cm in 66 s and 44 actions, until
it was stopped.** Nothing was unsafe: every action was checked and allowed. But the
run did nothing useful, and nothing would have ended it. A console run has no limit on
actions, and every attempt counted as a success, so the three-failures stop never
applied. Fixed in autonomy `dca1068` and deployed. A run on the rover has not yet
shown the fix.

## Why it looped

- **The viewpoint was where the rover stood.** The goal's viewpoint was 17 cm away, on
  a spot too close to something for the rover to fit, so navigation moved it 20 cm back
  to where the rover already was. The rover "arrived" each time without moving.
- **The look changed nothing, and nothing counted it.** The rover put things aside
  after three looks that left a placement no better. A look from the same spot is the
  same picture, and the world state does not record a repeated picture at all. So
  object:7's look count stayed at four, it was never put aside, and the scorer offered
  the same goal every time.

Replayed from the run's own record, today's scorer chose the same goal from every
recorded situation. Everything else in the room was already put aside or unreachable.

## The fix

A geometry goal that went where it was sent, looked, and left its thing no better now
puts the thing aside at once, for fifteen minutes or until it comes out better.
Applied to the run's first two recorded situations, the second decision moves on to
another thing. In the executive's tests, a goal whose look changes nothing is chosen
again on the next turn without the fix, and not with it. autonomy 707 passed, on the
Orin too.

## What it leaves

The scorer still offers viewpoints the rover cannot stand on. It checks the map's free
floor, and navigation needs room for the whole body. With the fix, each such goal costs
one attempt per thing per fifteen minutes, rather than a loop. Offering only spots the
rover fits is the next fix.

## Requirements

None moved. [R-SAFE-16](../requirements/safety.md#r-safe-16) held: no limit was
crossed. A run with no action limit depends on the chooser not repeating itself, and
this is the case that showed it.
