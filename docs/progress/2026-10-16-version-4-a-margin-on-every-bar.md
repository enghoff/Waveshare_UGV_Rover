# Version 4 of the world model: a margin on every bar, and no wrong merge in eleven runs, but fewer moves followed; nothing here is a test

**Raising every bar that joins two things by 0.06 removed every wrong merge
from the seven labelled sessions, on all pictures and in ten draws each
without a random 1% of the unlabelled ones.** Version 3 made zero to two a run
([the spread](2026-10-16-the-world-models-honesty-is-within-noise.md)). The
price is moves: version 4 follows 6 of 16 strictly, version 3 10 of 18, the
deployed store 2 of 16. Every session has been seen, so this is development;
its test is M5 session 7.
[R-WS-17](../requirements/world-state.md#r-ws-17) stays proposed.

## The change

Every decision that files a picture to a record, links two pictures, joins two
records or pairs an arrival with a departure compares a likeness with a bar.
In [the world model kept per visit](../plans/a-world-model-kept-per-visit.md)
each of those bars is now 0.06 higher, so a likeness just over a bar starts a
record of its own: a split, which is allowed, instead of a merge, which is not.
The likeness scale is also fitted once, on pictures before 2026-10-16, so that
pictures added later cannot change an earlier decision
([why](2026-10-16-m5-session-6.md)).

| On all pictures, seven sessions | Wrong merges (all; 10 draws) | Moves strict | Unmoved in two records or fewer | Filed |
|---|---|---|---|---|
| Deployed store | 16 | 2 of 16 | 11 of 44 | 69-80% |
| Version 3 | 1; 0-2 a draw | 10 of 18 | 41 of 44 | 86-95% |
| Version 4 | 0; none in any draw | 6 of 16 | 43 of 44 | 78-90% |

Tried and left off:

- **A margin of 0.03**: three wrong merges in six runs, one of them the two
  bean bags in session 6.
- **The margin with the move bars spared**: 10 of 16 moves, but a painting's
  record "moved" onto another painting in one run of eleven, and in one of
  three at 0.05.

On session 6 version 4 follows 1 of 6 moves strictly, where version 3 followed
3 of 7 and the deployed store 2 of 6: the margin costs most where moves are.

## What it means

Version 4 trades moves for honesty, which is the order the rule puts them in.
Whether the trade holds on a session it has not seen is session 7's question,
judged on the run with every picture and on five draws, a wrong merge in any of
them failing it. Frozen on branch `visit-model` at `cabe003`, with its scale in
`scale-1015.npz`.
