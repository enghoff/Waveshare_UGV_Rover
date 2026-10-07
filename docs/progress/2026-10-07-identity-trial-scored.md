# The identity trial, scored: the painting's clear views wait on five records of itself

**The visibility candidate fails on the independent recording, and the reason
is not the one it was built for.** From viewpoint B the camera saw the dining
painting clearly; neither the rover nor the candidate attached those views to
anything. On the rover, the map's reach stopped them: the lidar sees the dining
chairs 1.65 m away and treats the wall behind as hidden, though the camera sees
the painting above the chairs and measured it at 3.9 m. The candidate lifts
that limit, and then the views are left waiting for a different reason. Seven
placed things fit them equally well, five of them records of this one painting,
and appearance cannot choose between copies of the same thing. So the resolver
declines to guess. The candidate also loses the one painting connection the
rover had made, which puts it below the plan's retention bar. It is not
deployed. R-WS-13 stays open; R-WS-17 and R-WS-18 stay proposed.

## Coverage

The [run](2026-10-07-identity-trial-collects-both-views.md) gave clear views of
the painting from B only (regions 76267 and 76277, before and after a centring
turn). From C (76295, 76304), 1.1 m away and about 16° round the painting, a
chair back covers the middle of it. The same is true from the start (76237, 76253).
The plan asked for two clear views at least 0.4 m and 12° apart, plus an
occluded one. It got one clear place and occluded views from two others. The
threshold is not lowered.

## Labels

All 123 regions from the trial's 14 photographs were labelled from the full
frames and the stored outlines, and frozen before any replay. Twelve physical
objects were named, with 58 regions between them, valid parts included. Chairs,
ceiling fans and pendant lamps look alike and were left unresolved, along with
the owner, doorways, the floor and three mixed regions.
They are analyst labels, not owner-confirmed. A listing that showed the
regions' entity identifiers was printed before labelling began, so the review
was not strictly blind; no candidate result existed yet. The label file's
SHA-256 is `fba1c2b4c4eeb2540897e2494a732ee43fd1380d94bea23b6343c661b63c176c`.

## Scores

The recorded calls reproduce exactly: 6 identity passes and 9,416 map queries.
Every query the candidate makes gives the same answer on all the pass maps.

| | rover (control) | candidate |
|---|---|---|
| correct connections between fresh regions | 13 | 13 |
| kept from the rover's | -- | 12 (92%; the bar is 98%) |
| wrong connections | 0 | 0 |
| painting's clear views attached | none | none |
| clear/occluded painting connection | none | one, to an older record (below) |

The candidate lost the two C views' connection to each other and gained the
small painting over the shelf. It also attached the occluded start view
(76237) to `object:301`. That record's older photographs show the painting, so
this is a genuine clear/occluded connection, but to an old record. The fresh-only
score does not count it, and it does not make up for the lost one. The
candidate moved 14 older observations between records as well; those are
unreviewed.

## Why the clear views wait

`explain_ambiguity.py` (new) reruns the calls and prints every decision about
chosen observations, including the ambiguous ones the store does not record.

- **On the rover, nothing fitted.** Against `object:301`, `328` and `375`, which
  are placed on the wall at 1.9-2.0 m and hold 39, 77 and 99 observations, the
  clear views point 3-19 cm from each, agree on height and agree on range. The
  one test they fail is the map's reach.
- **With the candidate, too much fits.** Seven placed things fit:
  `object:375`, `301`, `328`, `246` and `629`, all within 0.3 m of one another
  on the wall 1.9-2.0 m up, plus `337` just below them and `397` in front.
  Appearance gives 0.74 against 0.75-0.76.
  The C views go the same way once the wall records become visible to them,
  which is how their connection is lost.
- **The record the C views had joined is misplaced.** `object:594` sits 1.2 m
  in front of the wall, over the table, at 1.4 m. That is why the rover's
  matching joined the C views to it and why the B views' measured range
  disagrees with it.

## What it means

Reaching the painting needs two things, in order. First, records of one object
must become one record (R-WS-17); while the painting has five, a correct new
view is ambiguous by construction. Second, the reach limit must not hide what
the camera measures above low furniture, which the candidate's rule addresses.
Fixing the visibility rule alone makes this recording worse. The next step is
offline: replay this recording with the painting's duplicate records
consolidated and see whether the clear views attach. That is a counterfactual
diagnostic, not a merge of the real store.

Evidence: the trial's 14 frames and their retained depth, copied from the rover
and hash-verified (`world/recordings/.../frames`, `frames.sha256`); the frozen
labels (`physical-subjects-frozen.json`); replays under
`.cache/visibility-12-*`.
