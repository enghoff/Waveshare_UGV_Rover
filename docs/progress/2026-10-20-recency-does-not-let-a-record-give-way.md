# Preferring the newer record at answer time does not let a stale record give way: it breaks the controls before it mends a moved thing

Two answers in the multi-day benchmark's first round were wrong in every state
because a record outlived its thing
([the result](2026-10-20-the-multi-day-benchmark-ends-after-two-days.md)).
`find_thing` already prefers the more recently seen of two records whose
scores against the phrase are within 0.01 of the best. **Widening that margin
was measured on the benchmark's own answers and does not help**: at 0.02 three
of the six control questions go wrong in some state, and no width up to 0.05
mends either moved thing. Answering better needs the world model to know the
new record and the old one are the same thing (R-WS-17), not a looser tie.

## How it was measured

On the rover, each benchmark state -- the morning copy, and it with each arm's
pictures filed on, as `experiments/multiday/answer.py` files them -- was asked
all nine questions, keeping the 25 best-matching records each time. The rules
were then scored here against the owner's truth, right within 1.0 m.

| Rule | Controls right, of 36 | Moved things right: morning, autonomy, loop (of 6 each) |
|---|---|---|
| near-ties within 0.01, newest first (deployed) | 36 | 0, 1, 4 |
| within 0.02 | 27 | 1, 1, 4 |
| within 0.03 | 25 | 1, 1, 4 |
| within 0.05 | 25 | 1, 1, 3 |
| records standing before gone ones, newest first within 0.02 | 27 | 2, 2, 4 |

The deployed rule reproduces the benchmark's answers exactly.

## Why

- **The black bean bag on day 1.** The loop's pictures of it at P4 went into a
  new record there, `v1916`, which scores 0.111 against "a black bean bag";
  `v1470`, last seen on 2026-10-17 at (-16.13, -10.95) and never seen gone,
  scores 0.160. A margin wide enough to bring `v1916` in brings in a stray
  record for every control as well.
- **The stool on day 2.** Its record at day 1's place, `v1937`, was never seen
  gone: a stool's legs give the lidar little to see missing. No record seen in
  either arm's window scores above 0.12 against "a stool".
- **The controls** go wrong because newer records near the bed, the wardrobe
  and the portrait score within 0.02 to 0.03 of each one's main record.

## Next

A record should give way when its thing is seen elsewhere, which needs the
world model to recognise the thing elsewhere -- in the black bean bag's case,
to file the pictures at P4 to the record it already had, which colour alone
separates from the red one (2026-10-15). That is R-WS-17's work, and these two
days are fresh drives to measure it on.

No requirement changed state.
