# M10: what the rover's runs hold to learn outcomes from

Offline work towards [M10](../../docs/plans/autonomous-curiosity.md) (learned
outcome prediction), entered after M4 on its criterion 8.

| Script | What it does |
|---|---|
| `replay_history.py LOOKS.npz OUT.npz RECORDING.npz...` | the rover's world model (`world_state/visit_model/model.py`) replayed over a looks file, noting every record's place, spread and visits each time a visit touches it, and every region's final filing. Over `looks-session8-1018.npz` it files 29,010 regions into 634 records; 99.2% of regions land in a record of the same name as on the rover |
| `outcomes.py GOALS.jsonl HISTORY.npz LOOKS.npz OUT.jsonl` | what each recorded look did to its target on the world model: the target record (by name since 10-17; before, the record holding most of the store thing's earlier pictures, else the one nearest the aim point), its spread just before the look and just after its visit, whether the look's picture fed it, and the gain realised |
| `diagnose.py OUTCOMES.jsonl HISTORY.npz LOOKS.npz OUT.jsonl` | why a look did not reach its target: the regions of its picture pointing at the target's place, and whether they were filed to it, to other records, or left unfiled, or whether none pointed there |
| `features.py OUTCOMES.jsonl HISTORY.npz OUT.jsonl` | each look as the predictor sees it: what was known of the target and its neighbours just before the look, the look as planned, and whether it landed |
| `predict.py FEATURES.jsonl [MODEL.json]` | the landing predictor, a logistic regression, trained before 2026-10-10 and judged after, and again leaving one day out at a time, against a constant and the scorer's rule; calibration in bins |
| `look_trial.py dry|run NAME` | M10's test: the survey loop out and back (session.py's stops and checks), at each stop up to six looks at records in reach drawn at random, each recorded with the frozen predictor's chance before it is taken, in `~/.ugv/sessions/NAME-looks.jsonl`. `dry` lists them from the live records without moving: 47 a loop on 2026-10-18. Run on the rover from `~/m10` |
| `landed.py LOOKS.jsonl [OUT.json]` | scores a trial once the model has filed its pictures: which looks landed, the Brier score against the history's share and the old rule with a bootstrap interval, calibration, and the choice at each stop old against new ([the declared rules](../../docs/decisions/m10-predicts-whether-a-look-lands.md)). On the seven world-model looks of 2026-10-17 and 18 with a picture and a named target: none landed, as `outcomes.py` found |
| `inventory.py [EPISODES.db] [OUT.jsonl]` | one row per goal an autonomous turn chose: when, which world it was chosen from, its type, what it predicted (gain kind and value, gain, utility, time, travel) and what was measured after it (how it closed, placement uncertainty before and after, whether a change was still reported); then a summary by goal type and world. Read-only; run on the rover |

## What the store held on 2026-10-18

805 goals chosen over 15 days, 2026-10-02 to 10-18:

| World | Goal type | Chosen | Measured after | Improved |
|---|---|---|---|---|
| old store | `improve_geometry` | 723 | 630 placements | 53 (8%) |
| old store | `investigate_change` | 16 | 15 | |
| old store | `inspect_hypothesis` | 28 | | |
| old store | `explore_frontier` | 5 | | |
| world model (since 10-17) | `improve_geometry` | 29 | 27 placements | none counted, see below |
| world model | `investigate_change` | 4 | 4 | |

**A look at a world-model record is measured before the model has filed it.**
The model files a look when its visit closes, minutes later, and nothing
measures the placement again afterwards. So all 27 world-model placements read
"no better" because of when they were read, not because of what the look did.
Learning from world-model runs needs the outcome read from the model's records
once the look is filed.

**The 723 older looks were chosen on the old store's records**, under its own
placement and uncertainty. Whether they can teach anything about choices on the
world model's records is the next question: the model has filed all of their
pictures, so each look's effect on the model's record of the same object can be
measured after the fact.

## What the recorded looks did, on the world model

`outcomes.py` over the 2026-10-18 history: 502 geometry looks with a picture in
map session 67 and a target found, 423 of them by the store thing's earlier
pictures.

| | Looks | Fed their target | Improved it | Made it worse | Predicted gain, mean | Realised, mean |
|---|---|---|---|---|---|---|
| store era, target seen on 6+ visits | 322 | 94 | 8 | 5 | 0.77 m | 0.0002 m |
| store era, fewer visits | 92 | 20 | 12 | 0 | 0.91 m | 0.017 m |
| world-model era, 6+ visits | 10 | 1 | 0 | 0 | 0.93 m | 0 |
| world-model era, fewer | 6 | 0 | 0 | 0 | 0.50 m | 0 |

**Most looks that fed their target could not improve it**: 93 of the 115 fed
looks went to records already at the model's floor, 0.10 m
(`PLACE_FLOOR_M2`), while the scorer, reading the old store, put the same
things at 0.6 to 1.1 m (quartiles). **Most looks did not feed their target at
all**: 382 of 502. On the world model's records, which the scorer reads since
2026-10-17, only the second applies, and 1 of 16 looks fed its target.

## Why looks do not land

`diagnose.py`, 430 looks whose target is known by name or by the store thing's
earlier pictures: 102 fed it; 211 sent the regions pointing at it to other
records (153 of them within 0.8 m of the target, 186 records seen on six visits
or more); 140 had no region pointing at it; 41 left them unfiled; 8 had no
posed region. In samples of the pictures, the targets that lost their looks to
other records are second records of a thing held elsewhere -- the dining chairs
and rug, the armchairs, records seen once -- and the missed ones are often aimed
at bare wall, where a stray record stands for nothing.

## Will a look land? (development)

A look lands when a region of its picture is filed to its target. 430 looks,
115 landed. What decides it most is how long ago the target was last seen,
which is the model's own rule (`STALE_S`: a record unseen for three hours takes a
look only on a move's likeness):

| Target last seen | Looks | Landed |
|---|---|---|
| under 3 hours ago | 230 | 41% |
| 3 to 24 hours | 101 | 18% |
| over a day | 99 | 2% |

and then whether a record at least as established stands within 0.8 m (a
rival): 35-40% land without one, 9-13% with one.

`predict.py`, a logistic regression on how established the target is, its
spread, a rival, how crowded its place is, the nearest other record, the hours
since it was seen, the look's range and whether it drives:

| | Brier, held out from 10-10 | Log loss | AUC | Brier, one day out at a time | AUC |
|---|---|---|---|---|---|
| constant (training share, 0.36) | 0.169 | 0.528 | 0.50 | 0.200 | 0.33 |
| the scorer's rule (half if seen on 6+ visits, else none) | 0.234 | 0.829 | 0.47 | 0.240 | 0.54 |
| learned | **0.131** | **0.394** | **0.77** | **0.167** | **0.76** |

Held out, its predictions are calibrated to within 0.05 below 0.3 and
optimistic above (0.64 predicted, 0.31 landed, 29 looks). **This is
development, not a test**: the features were chosen after looking at all 430
looks. The test is looks recorded after the predictor is frozen.

## The test: the look trial of 2026-10-18

`m10trial1-1018`, 92 scored looks at 53 records drawn at random, every one
after the freeze ([the entry](../../docs/progress/2026-10-18-m10-look-trial.md);
the looks with their features and outcomes in
[trials/](trials/m10trial1-1018-landed.json)). 10 landed. **It failed criteria
3 and 4.**

| | Brier |
|---|---|
| learned, frozen `088b4d51a9a9` | **0.100** |
| constant, the history's 115/430 | 0.122 |
| the old rule | 0.188 |
| constant at the trial's own 10/92, in hindsight | 0.097 |

Learned minus constant -0.044 to +0.001, 95% bootstrap; calibrated within 0.09
in every bin of 20 looks or more; AUC 0.68; at each stop the new ranking's
choice landed once, the old one's twice.

**`landed.py` reads only the last filing's merges.** `decisions.jsonl` holds
every filing of the history the job has made, each naming records afresh from
v0; read whole, it folded standing records into others by merges from earlier
filings, and the first scoring of this trial counted 6 landed and every
criterion met.

## What the trial's misses were aimed at

[The entry](../../docs/progress/2026-10-18-what-the-look-trial-was-aimed-at.md).
`pointing.py LANDED.json OUT.json` (on the rover) finds, for each look, the
regions of its picture that point at the target and where the model filed
them; `miss_sheets.py` draws each missed look beside its target's and the
taker's earlier crops; `record_sheets.py` draws records three crops each for
labelling; `record_features.py` writes each record's mean DINOv2 and SigLIP2
vectors and what its regions measured; `objectness.py` reruns the two rounds
of the not-an-object classifier from [labels/](labels/), and fails both, as
on the day. Of 82 misses, 26 were at records of no object, 23 at things gone,
14 at the dining chairs, 9 not boxed, 7 at a second record of a thing standing
there.

## People

`people.py FEATURES.npz labels/records-1018.json labels/people-test-1018.json [PERSON.json]`
fits the person model again from the labels -- development and held-out halves
split by record id before fitting -- and judges it as on the day: held out, 9
flagged, all people, of 13; it reproduces `world_state/visit_model/person.json`
([the entry](../../docs/progress/2026-10-18-records-of-people-are-not-looked-at.md)).
