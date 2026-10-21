# Placing a low thing by the lidar where the depth camera reads long does not mend the black bean bag's answer: another old record of it answers instead

The multi-day benchmark's day-1 question about the black bean bag was answered
wrong in every state, from `v1470`, a record placed 2 m beyond the bag on one
depth box that read 4.9 m where the lidar said 2.4 to 2.8 m
([2026-10-21](2026-10-21-what-the-world-model-switches-do-to-the-answers.md)).
That entry proposed a narrow rule and left it unmeasured: a low thing whose
depth place runs well past the lidar's return is placed by the lidar. **Tried
on the world model kept per visit, the rule places such pictures better where
the truth is known, but the answer stays wrong, a control goes wrong, and moved
things keep their records less often.** It is not adopted.

## The rule

`LIDAR_LOW_M` in `experiments/visit_model/model.py` (branch `visit-model`,
`02af9cf`): before filing, a picture's region with a depth place (outline or
box), lower than 0.8 m as the camera measures height, whose depth range runs
0.5 m or more past the lidar's return along its bearing at the same moment, is
put at that return plus 0.15 m, the median by which depth runs past the lidar
on things standing on the floor. It is per picture rather than per record, so
it can act before a picture is filed.

## Where it fires, and whether the place is better

Over every picture to 2026-10-20 it moves 269 of 13,068 depth-placed regions;
228 of them are of nothing labelled, 13 of the rug, which the lidar's plane
passes over. Of the 17 moved regions of staged things, judged against where
each was set out (`place_rule.py`):

| | Regions | Depth place off, median | Rule's place off, median | Nearer / farther |
|---|---|---|---|---|
| Black bean bag | 7 | 0.73 m | 0.17 m | 7 / 0 |
| Armchairs, red bean bag, step ladder | 10 | 0.59 m | 0.33 m | 6 / 4 |
| All | 17 | 0.61 m | 0.20 m | 13 / 4 |

Two of the four made worse are 2 m out: session 4's plain armchair, ranged at
3.3 and 4.0 m by the depth camera to within 0.3 m of where it stood, where the
lidar's return along the same bearing was something nearer, at 1.3 and 2.0 m.

## What it does to the answers and the records

Asked as `desk_answers.py` asks the benchmark's nine questions at the end of
each day, both arms' pictures together:

| Filing | Right | Wrong | Unresolved |
|---|---|---|---|
| version 4 | 15 | 2 | 1 |
| with the rule | 15 | 3 | 0 |

- **The black bean bag on day 1 stays wrong.** `v1470` no longer answers, but
  five of the six best-scoring records, within 0.025 of the top, are the
  bag's own, at places by the dining table's end where it stood in earlier
  sessions. One of those, standing, last seen on 2026-10-12, answers instead
  (`diag_rank.py`). No record
  near P4, where the bag was that morning, is in the top six.
- **The red bean bag on day 1 goes from unresolved to right**, as the filing
  changes around it.
- **The wardrobe, a control, goes wrong.** The rule's changes elsewhere leave a
  new two-picture record of nothing labelled, made on day 1 at
  (-13.06, -15.13), inside the 0.01 near-tie, where the newest standing record
  answers first. The wardrobe's own records are where they were.

`score_all.py` over every session: no record holds two labelled things under
either filing, but moved things keep their record in 18 of 39 trials against
23, and 19 are split against 14.

## What it means

The black bean bag's wrong answer is not one misplaced record. The bag has
several records left standing where it used to be, and whichever scores best
answers. That is the same finding as the other three rules tried on 2026-10-20
and 21: moves followed, a gone record giving way, absence judged over the
visit. Each changed which of a thing's records answers, and none reduced how
many it has. The benchmark's wrong answers wait on old records of a moved thing
being found gone, or folded into the one it has now (R-WS-17).

No requirement changed state. R-WS-17 stays proposed.
