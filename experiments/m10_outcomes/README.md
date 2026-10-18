# M10: what the rover's runs hold to learn outcomes from

Offline work towards [M10](../../docs/plans/autonomous-curiosity.md) (learned
outcome prediction), entered after M4 on its criterion 8.

| Script | What it does |
|---|---|
| `replay_history.py LOOKS.npz OUT.npz RECORDING.npz...` | the rover's world model (`world_state/visit_model/model.py`) replayed over a looks file, noting every record's place, spread and visits each time a visit touches it, and every region's final filing. Over `looks-session8-1018.npz` it files 29,010 regions into 634 records; 99.2% of regions land in a record of the same name as on the rover |
| `outcomes.py GOALS.jsonl HISTORY.npz LOOKS.npz OUT.jsonl` | what each recorded look did to its target on the world model: the target record (by name since 10-17; before, the record holding most of the store thing's earlier pictures, else the one nearest the aim point), its spread just before the look and just after its visit, whether the look's picture fed it, and the gain realised |
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
