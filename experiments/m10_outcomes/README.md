# M10: what the rover's runs hold to learn outcomes from

Offline work towards [M10](../../docs/plans/autonomous-curiosity.md) (learned
outcome prediction), entered after M4 on its criterion 8.

| Script | What it does |
|---|---|
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
