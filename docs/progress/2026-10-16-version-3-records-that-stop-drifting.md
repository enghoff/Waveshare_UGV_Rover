# Version 3 of the world model: no record of two things over six sessions, and six of eleven moves followed, but nothing here is a test

**Four changes to [the world model kept per visit](../plans/a-world-model-kept-per-visit.md)
remove every wrong merge from the six labelled sessions and follow more than
half the moves of distinguishable things.** They answer the failure of
[M5 session 5](2026-10-16-m5-session-5.md), where the black bean bag was
taken for the red one's record. Every session has now been seen, so this is
development; the test is a fresh staged session.
[R-WS-17](../requirements/world-state.md#r-ws-17) stays proposed.

## The changes

- **A record unseen for three hours is "here as before" only on the likeness
  a move needs**, 0.75 rather than 0.55. Hours later, a thing standing where
  another stood is no evidence it is the same thing, in a home that gets
  rearranged.
- **An arrival is paired first with a departure seen in the last half hour.**
  It waits for the visit's absences before it is paired with anything older,
  so that its own departure, seen in the same visit, can win. In session 5
  the red bean bag's record was marked gone in the very visit the bean bag
  reappeared, and two older records of it, gone for days, tied with it.
- **Two pictures whose colours match under 0.2 are not linked into one
  cluster.** A cluster of two pictures of the rug and one of the red bean bag
  went to the rug's record in session 3. A single picture's colour separates
  objects only weakly, but this cuts 3% of same-object links against 18% of
  different-object ones, and links chain.
- **Merges run in a fixed order.** They had run in the order of a set of
  names, which Python shuffles from run to run: with two seeds in three, the
  build before this one made that rug merge, and with one it did not. Five
  seeds now give the same result. Versions 1 and 2 vary with the seed only in
  records per unmoved object, by one.

Another was tried and left off: comparing colour with what a record mostly
looks like, not its best sample. It removed session 5's merge and made three
elsewhere, because a vetoed match goes to the next-best record, which is not
always right.

## On the six sessions

Wrong merges counted across sessions, as criterion 4 means them
([corrected the same day](2026-10-16-m5-session-5.md#a-correction-m5s-criterion-4-was-not-met));
session 5's eleven unmoved objects labelled after versions 1 and 2 had been
scored on it, before version 3 was.

| | Wrong merges, all sessions | Distinguishable moves, strict | Unmoved in two records or fewer, sessions 3-5 | Filed, sessions 3-5 |
|---|---|---|---|---|
| deployed store | 8 | 0 of 10 | 4 of 22 | 69-74% |
| version 1 | 4 | 2 of 11 | 20 of 22 | 89-95% |
| version 2 | 1 | 1 of 11 | 18-19 of 22 | 90-95% |
| version 3 | 0 | 6 of 11 | 21 of 22 | 90-93% |

What version 3 still does not follow:

- **The cabinet in session 2**, the first time it moved, and one of the
  moves in sessions 3 and 4.
- **The step ladder**, carried from wall A in session 2 to the balcony window
  in session 5, on two filed pictures.
- **The stool's move in session 5**, where nothing was drawn round it.
- **The black bean bag** comes back to a record of itself, but not the one
  that held most of it.

## What it means

The swap that failed version 2 is the ordinary case the plan exists for,
and version 3 handles it on the evidence it was built from. Whether it
handles a swap it has not seen is session 6's question. Version 3 is frozen
on branch `visit-model` at `e5b39fd`.
