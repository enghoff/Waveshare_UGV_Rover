# M4's acceptance: a chosen viewpoint improves its thing more than a look again from where the rover stood, and the claims stay honest

**Over 35 acceptance attempts on the eleven taped things, the look from the
viewpoint the rover chose improved its thing's placement more than a re-look from
where it stood: paired difference +0.33, 95% interval +0.05 to +0.76.** The chosen
look improved its thing 9 times and made one worse; the re-look improved 4 and
made 2 worse. After either look the tape lay inside the rover's stated figure 25
times in 35, about what a one-sigma figure should cover. The chosen looks were
overconfident no more often than the re-looks, 5 against 6. Read against its eight
criteria below, M4 is met; signing it off is the owner's. The attempts stopped at
35 of the 43 declared, by
[the owner's decision](../decisions/m4-acceptance-stops-at-35.md). Had the last
eight come out as nothing, the difference would be +0.27 (+0.04 to +0.66); had
the two open records among them gone as badly as they have before, +0.24 (-0.02
to +0.63). [R-WS-13](../requirements/world-state.md#r-ws-13) stays open.

## The runs

Trial runs on `captures/2026-10-09-m4-targets/truth-acceptance.json`, the 51
records of eleven things labelled and frozen on 2026-10-09 before any attempt,
scored by distance to each thing
([m4-scores-from-the-thing.md](../decisions/m4-scores-from-the-thing.md)). The first
block has [its own entry](2026-10-09-m4-acceptance-first-block.md). Every run was
unattended at the owner's word, against
[R-SAFE-6](../requirements/safety.md#r-safe-6)'s standing rule that a person
watches. Runs were opened alternately from the living room and the bedroom so
that each room's things started out of reach. Every reading is in
`captures/2026-10-09-m4-acceptance/PLAN.txt`.

| Block | When | Runs opened | Attempts | How it ended |
|---|---|---|---|---|
| 1 | 2026-10-09 14:29-14:49 | 3 | 11 | 15% recovering, nothing free until 15:00 |
| 2 | 15:29-15:51 | 4 | 8 | at the 10% floor |
| 3 | 17:24-17:59 | 6 | 7 | at the 10% floor |
| 4 | 18:32 to 2026-10-10 07:13 | 5 | 6 | the owner took the rover back to charge |
| 5 | 2026-10-10 07:34-07:53 | 2 | 3 | the owner's stop |

Most runs ended within minutes, saying there was nothing left worth doing. A
record that a look found empty is set aside for two hours, and a run with only
set-aside records has nothing to choose. On the last morning, re-decided as a
trial from the charger spot, the 51 records stood as follows:

- **No viewpoint to drive to** for the floor lamp's 15 records and the balcony
  painting's 12. From further into the living room, the last run found one for
  the painting.
- **Viewpoints on the dining rug,** or another place the rover may not stop, for
  the front door and the kitchen cabinet.
- **Set aside, with no route from there, or in reach** for the armchair, the panel
  cover and the brown wardrobe.
- **Too little to gain** for three records.
- **Open** for three bedroom records, predicting 0.09 to 0.18 m. From the bedroom
  those three are in reach, which is why a run opened there found nothing.

Things that went wrong on the way and were not M4's:

- **Drives refused near the bedroom and hall doorways.** A short goal is driven
  as one straight leg, and Nav2's collision check on that leg sees the door frame
  on its costmap. Twice a run's drive back left the rover standing inside a
  doorway's padding, where navigation would not plan; a 0.4-0.5 m straight drive
  got it out. After the last run, the rover's heading in the bedroom was 30 to
  36 degrees out and every drive home was refused. A straight drive corrected the
  heading, the spin at the start of the next drive put it 18.5 degrees out again,
  and a second straight drive fixed it for good. All of this belongs with M3's
  [arrival-turn fault](2026-10-09-arrival-heading.md).
- **The GPU did not come up at boot on 2026-10-10.** The perception service's
  own repair reloaded it within a minute, but a run opened in that minute ended
  at once on a failed look. Reopened, the next run worked.
- **The rover went off at 18:37 on 2026-10-09**, seconds after driving back to
  the charger with its battery reading 25% while recovering. There was no
  shutdown in the journal. Whether the battery gave out or it was switched off is
  not known.

## The scores

`score_attempts.py` on the rover, over all attempts so far after each block
(`captures/2026-10-09-m4-acceptance/fp-acc.json` for block 1, then
`report-2.json` to `report-5.json`).

| After block | Pairs | Things | Difference, 95% interval |
|---|---|---|---|
| 1 | 11 | 6 | +0.78 (-0.08 to +1.70) |
| 2 | 19 | 10 | +0.51 (+0.01 to +1.12) |
| 3 | 26 | 10 | +0.38 (+0.01 to +0.92) |
| 4 | 32 | 10 | +0.37 (+0.06 to +0.85) |
| 5 | 35 | 11 | +0.33 (+0.05 to +0.76) |

| | Improved | Unchanged | Worse | Mean gain, 95% interval |
|---|---|---|---|---|
| chosen viewpoint | 9 | 25 | 1 | +0.45 (+0.20 to +0.86) |
| re-look | 4 | 29 | 2 | +0.12 (-0.07 to +0.39) |

- **What improved was depth.** Nine of the eleven chosen looks that ranged their
  thing improved it, and none of the 24 that did not range it changed anything.
  By room, the chosen look improved 6 of 17 attempts in the bedroom, the one in
  the kitchen and 2 of 17 in the living room, where most records had been looked
  at for days and were already well placed.
- **The lead rests on a few large gains.** The autumn-forest painting (N11) went
  from 0.68 m off, claiming 0.66 m, to 0.26 m off in one attempt, while the
  re-look left it where it was. The interval resamples things rather than
  attempts, so it already allows for this.
- **Honesty.** The tape was inside the stated figure 25 times in 35 after the
  chosen look and 25 after the re-look (71% each, against the 68% a one-sigma
  figure should cover). It was more than twice that figure away 5 times after the
  chosen look and 6 after the re-look. One of each is the same record of the
  autumn-forest painting, 1.01 m off and claiming 0.30 m, which neither look
  changed.
- **The rover's own account** (criterion 8): of the 10 chosen looks whose claim
  changed, all 10 had the tape's sign. Of the 6 such re-looks, 5 did (interval
  0.63 to 1.0). The other 25 and 29 changed nothing.

## Against M4's criteria

| | Criterion | Read |
|---|---|---|
| 1 | each inspection names its gap and records its predicted gain and the placement after | met: all 35 acceptance attempts recorded both, and each goal names the record and the gap it would close |
| 2 | each placement traces to its looks, each look to a picture and pose | met for the attempted things: the 51 records rest on 2,424 looks, every one with its pose and its picture on disk (a copy of the store, 2026-10-10). Not checked beyond those records |
| 3 | the chosen look beats the re-look, paired interval above zero | met: +0.33 (+0.05 to +0.76) |
| 4 | predeclared things, score, count and analysis; refusals counted | met, with two departures the owner agreed: the score changed after block 1, and the count went from 43 to 35. All eleven things were attempted. Each deliberation refuses things in reach, but only the reason a run had nothing to do is written down, so those refusals cannot be counted after the fact |
| 5 | the chosen look's mean gain above zero, shares by condition | met: +0.45 (+0.20 to +0.86); shares above |
| 6 | the tape inside the claim about as often as one sigma covers; no more overconfidence after chosen looks | met: 71% after both; overconfident 5 against 6 |
| 7 | a look that finds nothing leaves the gap open and says so | met: 24 chosen looks and 27 re-looks filed nothing, and every one left the placement as it was. The rover set the record aside and said why. Things with no viewpoint were refused as having none |
| 8 | the rover's own account against the tape, reported | reported: 10 of 10 chosen, 5 of 6 re-looks |

## What it means

Choosing where to look from is worth something on this rover, and the reason is
narrow: from the chosen spot, the depth camera can range the thing. Where the
records were already good, as in the living room, there was little left to gain.
The rover's claims stayed honest throughout. Snapshots and store copies stay on
the rover until M4 is signed off.
