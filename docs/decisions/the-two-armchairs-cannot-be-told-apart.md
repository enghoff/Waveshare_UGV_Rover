# The two black armchairs are counted as things the rover cannot tell apart

Status: agreed 2026-10-16 by the owner. No code has changed.

The plain black armchair and the one that carries a throw are treated the way
[R-WS-17](../requirements/world-state.md#r-ws-17) already treats the six
identical dining chairs. They are left out of the identity counts and counted
separately. Whether a record holds one or the other, or both, is not a trial
of keeping identity and not a wrong merge.

## Why

No feature the rover has tells them apart:

- **The image vectors.** Masked DINOv2 scores one armchair against the other
  as high as either against itself on another day (0.82 against 0.72-0.80).
- **A likeness learned from the rover's own pictures.** It tells the other
  look-alikes in the flat apart (the bean bags 0.90, the paintings 0.83-0.95)
  but the armchairs only 0.57 of the time, where 0.5 is chance
  ([2026-10-15](../progress/2026-10-15-a-likeness-learned-from-the-rovers-own-pictures.md)).
- **Colour inside each outline.** It helps a little, because of the throw, and
  the throw is not always on or in view.

A person tells them apart by the throw. The rover sees it too rarely to rely on.

## What it changes

- **M5's criterion 4** ([the counting](m5-sessions-are-counted-before-they-start.md#what-counts-as-identity-kept-criterion-4)).
  - The failure recorded on 2026-10-14 was a record holding both armchairs:
    object:379 in session 1 and object:838 in session 2.
  - Recounted with the armchairs set aside, on the stores kept after each
    session, neither session has a record holding two things.
  - The one move left to count, session 2's cabinet, was split. The criterion
    reports that without a bar.
  - The owner signed M5 off on 2026-10-16.
  - **Corrected the same day:** that recount compared pictures within each
    session only. Counted across sessions, `object:914` and `object:930` took
    the black bean bag in the shakedown and the red bean bag in session 1 as
    ordinary matches, so criterion 4 fails whatever is decided here
    ([session 5](../progress/2026-10-16-m5-session-5.md#a-correction-m5s-criterion-4-was-not-met)).
    The owner withdrew the sign-off the same day.
- **Staged sessions** move things that can be told apart. An armchair may still
  be moved, and is reported separately: how many armchair records stand where
  the two armchairs are.
- **What the rover says** about an armchair should be "one of the two black
  armchairs" when it cannot tell which. That is [R-WS-2](../requirements/world-state.md#r-ws-2)
  applied, and is not yet built.
