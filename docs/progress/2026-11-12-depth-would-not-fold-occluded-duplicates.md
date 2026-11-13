# A depth layer would not fold the duplicates that occlusion seems to cause: the outline already keeps the occluder out

**Pictures in which something stands in front of a never-moved object are no
more common in its duplicate records than in its main record, and in the views
where a chair cuts across a painting the region finder's outline already leaves
the chair out of the pixels the likeness, the colour and the range are read
from.** So a depth component that separates a region's near layer from its far
one, such as Depth Anything run on the gimbal picture, has nothing to remove in
almost every case it was meant for. The idea is dropped. Offline, on this
desktop. Nothing on the rover was called or changed.
[R-WS-17](../requirements/world-state.md#r-ws-17) stays proposed.

## The question

From some places a dining chair stands in front of the painting behind the
table, and from others it does not. If the chair's pixels got into the region
of the painting, the picture would look and range unlike the painting's other
pictures and could start a record of its own. A depth layer could then cut the
chair out.

## What was measured

Frozen version 4 (`cabe003`) was run over every picture to 2026-10-20
(`tmp/visit-model/looks-md-1020.npz`, with the usual drive recordings). Its
filing matches `md-v4.json` region for region. The objects were the 11 that never
moved (`labels/stayed-put.json` on branch `visit-model`). For each, the record
holding most of its labelled pictures is its main record. Its duplicates are
every other record holding a labelled picture of it, and every standing record
within 0.8 m and 0.5 m of height of the main one: 70 records in all.

- **Cover, from the region finder's own outlines** (`occlusion.py`, the store
  copy `tmp/ws17/ws17-world.db`): share of a region's box under other regions'
  outlines, and share of its own outline under them (the occluder inside the
  pixels that are read):

  | | Pictures | A fifth or more of the box covered | A tenth or more of the outline covered |
  |---|---|---|---|
  | main records | 6,021 | 7.2% | 1.8% |
  | duplicates | 5,259 | 6.8% | 2.5% |

  Of the labelled pictures, covered ones went to another record of the object
  13% of the time (14 of 110), uncovered ones 16% (250 of 1,560). Covered ones
  went unfiled more often (26% against 12%). Spot checks showed the measure
  misses chairs the finder did not outline, so for the one object where the
  case really occurs it was labelled by eye.
- **The painting behind the dining table** (`forest painting` in the labels),
  48 pictures drawn at random, 24 from its main record and 24 from its
  duplicates. They were labelled blind to the record, by the coding agent
  (`blind_labels.json`, `blind_key.txt`):

  | | Something in front | Across the painting's middle | Edge-on or at the picture's edge |
  |---|---|---|---|
  | main record | 22 of 24 | 3 | 2 |
  | duplicates | 23 of 24 | 8 | 4 |

  Chair backs cross the painting's lower edge in nearly every picture of it,
  so being occluded does not separate the main record from the duplicates.
  Heavier occlusion leans toward the duplicates, 8 against 3, which is not
  significant on 24 each and goes with closer, more oblique viewpoints.
- **Inside the outline.** In 10 of the 11 pictures with a chair across the
  painting's middle, the outline the region was read from excludes the chair:
  the masked likeness, the colour and the outline range already see the
  painting alone. In the eleventh, part of one chair back's top is inside.

## What it means

The pixel-level separation a depth layer would give is already done by the
segmentation outline, and the occluded views sit in the main record as often as
in the duplicates. What occlusion still costs is the part of the painting that
cannot be seen, which no depth estimate restores.

Giving every picture a range, whatever stands in front, is a coverage question
rather than an occlusion one. It was tested the same day and did not fold
duplicates either
([entry](2026-11-12-monocular-ranges-file-more-but-fold-no-duplicates.md)).

## Reproduce

Scripts are in `experiments/depth_for_duplicates/`. Run them with `VISIT_MODEL`
pointing at a checkout of branch `visit-model` (`af20b27`, where the labels
are). Put `model_v4.py` there, written from `git show
cabe003:experiments/visit_model/model.py`. `FRAME_PATHS` is a list of the
pictures on disk:

    python occl_run.py LOOKS.npz RUN.pkl
    python occl_dups.py LOOKS.npz RUN.pkl DUPS.json
    python occlusion.py LOOKS.npz RUN.pkl DUPS.json STORE.db OCC.npz
    python sheets.py LOOKS.npz RUN.pkl STORE.db OUTDIR v93:8 "@name=id,id,..."
