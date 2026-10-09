# M4's first eleven acceptance attempts: no better than the re-look, and overconfident about big furniture

**The first eleven acceptance attempts, on the eleven things taped on 2026-10-09:
the chosen viewpoint's look improved its thing twice and made it worse three
times, and the re-look improved two and made two worse.** The paired difference
was +0.01, 95% interval -0.67 to +0.53, over six things. At these rates
`score_attempts.py` puts the pairs needed at 66,797, so no number of attempts
this flat can give would tell the two apart. Every attempt that made a thing worse
was a ranged look at a large piece of furniture. The look placed the thing 0.45 to
0.61 m from its taped centre and claimed 0.20 m, so 5 of the 11 chosen looks left
the rover overconfident, against 4 re-looks. Acceptance was paused after this
block. [R-WS-13](../requirements/world-state.md#r-ws-13) stays open.

## The runs

Trial runs on `captures/2026-10-09-m4-targets/truth-acceptance.json`: 51 records
of the 11 things, labelled from their photographs at 14:25 and frozen before any
attempt. The code was the gimbal-aiming change, deployed after the development
attempts: a look from where the rover stands and the M4 re-look pan the gimbal
instead of turning, and an aimed look pans up to ±150 degrees. The runs were
unattended across the whole flat at the owner's word, against
[R-SAFE-6](../requirements/safety.md#r-safe-6)'s standing rule that a person
watches. Readings are in `captures/2026-10-09-m4-acceptance/PLAN.txt`.

| Run | Opened | Attempts | Ended |
|---|---|---|---|
| `run/336e73d6/1` | 14:29, living room, 65% | 6 | 14:32; then four drives "no way past" with the heading 8 degrees out, and it ended in the bedroom |
| `run/336e73d6/2` | 14:33, bedroom | 2 | 14:35 |
| `run/336e73d6/3` | 14:46, bedroom, 25% | 3 | 14:48; nothing free again until about 15:00, at 15% recovering |

The blocked drives in run 1 marked their goals as places it could not reach, for
half an hour. Those were viewpoints for the panel cover, the armchair and the
front door, so they got no attempts.

## The scores

`score_attempts.py` on the rover (`captures/2026-10-09-m4-acceptance/report-1.json`).

| | Improved | Unchanged | Worse | Mean gain, 95% interval |
|---|---|---|---|---|
| chosen viewpoint | 2 | 6 | 3 | -0.17 (-1.19 to +0.46) |
| re-look | 2 | 7 | 2 | -0.19 (-0.98 to +0.32) |

| Attempt | Thing | Arm | Off the tape, before → after | Claimed, before → after |
|---|---|---|---|---|
| 1411 | N7 kitchen cabinet | chosen | 0.95 → 0.61 m | 1.85 → 0.20 m |
| 1414 | N9 footboard | chosen | 0.73 → 0.45 m | 0.49 → 0.21 m |
| 1416 | N8 brown wardrobe | chosen | 0.23 → 0.56 m | 0.70 → 0.20 m |
| 1416 | N8 brown wardrobe | re-look | 0.23 → 0.53 m | 0.70 → 0.20 m |
| 1415 | N10 mirrored wardrobe | re-look | 0.65 → 1.67 m | 1.02 → 0.29 m |
| 1415 | N10 mirrored wardrobe | chosen | 0.65 → 0.35 m | 1.02 → 0.23 m |
| 1422 | N10 mirrored wardrobe | both | 0.46 → 0.21 / 0.20 m | 0.36 → 0.25 m |
| 1429 | N12 portrait | re-look | 0.31 → 0.12 m | 0.50 → 0.31 m |

- **The claim does not grow with the thing.** A ranged look claims about 0.2 m
  whatever it ranged. On a cabinet, a footboard or a wardrobe, the part the region
  covered can be half a metre from the centre the tape names. Twice the placement
  came closer and still scored worse, because the claim narrowed further than the
  placement moved.
- **The re-look caught up.** In the development attempts the re-look improved its
  thing once in thirteen. Here it improved two in eleven, and on the mirrored
  wardrobe it matched the chosen look to the centimetre. Before the gimbal change,
  a re-look that pointed more than 20 degrees from where the turn left the rover
  stopped at the pan's edge; now it pans the whole way. Whether that is why is not
  shown, only consistent.
- **Honesty**: the tape inside the stated figure 5 of 11 after the chosen look and
  6 of 11 after the re-look; overconfident 5 and 4. The rover's own account
  (criterion 8) had the tape's sign 2 times in 5 for chosen looks.

## What it means

At these rates, M4's comparison cannot be settled on this flat, and its honesty
criterion would be judged on a claim that ignores the thing's size. The
[decision](../decisions/m4-measures-where-things-are.md) foresaw the first: re-looks
improving their thing nearly as often as chosen viewpoints is one of its
conditions for reopening. The second is a world-state fault in its own right: the
rover says it knows where a wardrobe is to 20 cm when its centre is half a metre
away. These eleven attempts stand as the first acceptance block, whatever is
decided next.
