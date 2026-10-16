# The world model's honesty is within noise: with 1% of unrelated pictures taken away, version 3 makes zero to two wrong merges, and session 5's bean bag swap comes back once in six

**Version 3 of [the world model kept per visit](../plans/a-world-model-kept-per-visit.md)
has no margin on honesty.** Run over the seven labelled sessions with a random
1% of pictures removed (65 pictures none of whose regions are labelled, five
draws), it made 0, 1, 0, 2 and 1 wrong merges; on all the pictures it made 1
([M5 session 6](2026-10-16-m5-session-6.md)). In one draw of six it took the
black bean bag for the red one's record in session 5, 19 pictures, the very
merge it was built to remove. Its zero on the data it was developed on was
one draw from this spread, so neither that zero nor session 6's says what it
will do on the next session.
[R-WS-17](../requirements/world-state.md#r-ws-17) stays proposed.

**Session 6 itself was clean in every draw**, for version 3 and for the build
below, and 3 of its 7 trials were followed strictly in eleven of the twelve
runs.

## What was measured

- **The perturbation.** Five looks files from M5 session 6's, each missing a
  different 1% of the 6,038 pictures that hold no labelled region (432-483 of
  46,330 regions), so the labels and the scoring stay the same. Versions run
  with a fixed hash seed.
- **Version 3** as frozen (`e5b39fd`), and **a build that fixes its likeness
  scale**: the scale from the learned likeness to a score, fitted once on the
  pictures taken before 2026-10-16 and kept beside the learned likeness,
  instead of over every picture given. With it, adding pictures leaves every
  earlier decision as it was (identical decisions before the first added
  picture, checked on session 5's late pictures and on session 6's), which
  version 3 does not.

| Wrong merges, all seven sessions | All pictures | Five draws without 1% |
|---|---|---|
| Version 3 | 1 | 0, 1, 0, 2, 1 |
| Fixed scale | 1 | 0, 1, 0, 0, 2 |

The merges come from three places, every time:

- **The black bean bag in the red one's record**, session 5 (both, one draw).
- **Two far-off pictures of the snowy-tree painting in the blue painting's
  record** (session 5 or 1), dark and small from the charger.
- **One picture of the red bean bag in the rug's record**, session 3.

Moves followed and unmoved objects kept together move by up to two between
draws and do not separate the two builds.

## What it means

Every decision that files a picture or joins two records compares a likeness
with a bar, and some sit within a thousandth of it, so which way they fall is
chance. Tuning the bars again would move the knife edge, not remove it. A
decision that close to its bar has to be held open instead, as the plan
already says for identity that cannot be told (R-WS-2), and honesty judged
over draws like these rather than over one. The fixed scale is kept: a model
whose past changes with every new picture cannot be checked at all.
