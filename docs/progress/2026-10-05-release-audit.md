# Half the proposed releases are views of the thing itself

**Release-only repair cannot be applied automatically.** Of the 200 observations it
proposes to release from the fresh snapshot, 85 are rightly released: 60 show a
different object from the record they sit in, 18 select pixels from two objects and 7
are glare or a view through a doorway. But 87 are views of the record's own object:
67 the same object, 34 of them clearly and 33 probably, and 20 a dining chair or
armchair like the record's, where no picture says which one. 23 could not be told and
5 are bare floor. The scored result looked safe because labels cover only 30 of the
200. As a list for a person to approve it would be declined about half the time.
R-WS-13 stays open; R-WS-17 and R-WS-18 stay proposed. Nothing changed on the rover.

## What was reviewed

The proposal is `captures/2026-10-04-reader-validation/split-only/release-fresh.json`,
over the snapshot `after.db` (`07ada3fe…`). Every frame it refers to is on disk.
[release_audit.py](../../experiments/entity_association/release_audit.py) `sheets`
draws 41 pages with one block per original record: up to eight of the members the
proposal keeps, largest cluster first and spread over time, then each release as
its full frame, its selected pixels and its plain box. Each release was judged
against what its record mostly shows, by the coding agent, without consulting any
label. The verdicts were hashed before any label was opened (`462b6199…`) and are
committed unchanged as [release_audit.json](../../experiments/entity_association/release_audit.json).
The owner has not reviewed them.

| Verdict | Count | What it means for the release |
|---|---:|---|
| Different object | 60 | right |
| Two objects selected | 18 | right |
| Glare or doorway view | 7 | right |
| The record's own object | 67 | wrong (33 of these with moderate confidence) |
| Same kind of chair | 20 | breaks a same-kind link |
| Bare floor in a floor record | 5 | neutral |
| Could not tell | 23 | unknown |

Eight releases come from records with a single member; all eight are right and none
changes a pair. Four records lose every member. Several records are themselves two
objects — a door with a chair in front, a window behind an armchair, an office chair
with an armchair cluster — and there "the record's object" means the majority.

## What the wrong releases look like

They are the record's object seen badly: far away, cut by the edge of the picture,
washed out by the window, or at night under orange light. That is why the appearance
score isolated them. Nothing simple in the box separates them: 44 of the 87 wrong
releases touch the frame edge against 36 of the 85 right ones, and 20 against 15 are
under 1% of the frame. Of the 67 same-object releases, 16 are rug edges, 13 paintings
and 8 ceiling fans or pendant lamps.

The right releases are mostly the familiar mixtures: chair backs in front of the
paintings, the rug inside a floor record, an armchair inside an office-chair record,
the door leaf beside a doorway, the kitchen seen through a doorway inside a chair
record. Released observation 64638 is the green painting behind chairs, as the
previous entry found; 64625 is the same mixed painting-and-chair region it disputed.

## Comparison with the labels, after freezing

The older drive's labels cover 26 of the releases. 16 agree. 5 are among those left
unclear. On 2 the labels count a painting with chairs in front as the painting where
the review reads mostly chair. Three disagree outright:

- **61656**: labelled the cow painting; the review reads a different, gold-framed
  painting. [label_review.json](../../experiments/entity_association/label_review.json)
  already disputes that label on its full frame, so the review agrees with it.
- **63521**: labelled the grey gold-framed painting, not the blue painting its record
  holds. A closer look shows the gold frame: the label is right, the review's "same
  object" is wrong, and this release was in fact right.
- **62297**: labelled not a dining chair; the review reads the curved back of a dining
  chair close against the frame edge. Unresolved.

The four fresh releases agree with the frozen draft labels, allowing that the review
calls the two painting-behind-chairs regions mixed where the draft calls them the
painting. The frozen verdicts are not edited for any of this.

## Decision and next step

Keep release-only as a source of candidates, not as a repair. Releasing on appearance
alone throws out roughly as many good views as bad ones, and the good ones it throws
out are precisely the degraded views a record most needs to keep. A release needs a
second, independent reason. The obvious one is position: whether the observation's
bearing and range point at the record's placement, which none of these sheets used.
Next, test whether position separates the 85 right releases from the 87 wrong ones
in this frozen review, then confirm on the older recording's labels before any further
scoring. No rover is needed for either. Separately, the 20 same-kind chairs need the
identity policy R-WS-17 owes before any score counts them as errors or as correct.

The review is one agent's reading of low-resolution pictures, a third of its
"same object" verdicts are moderate, and one of three checked disagreements went
against it. Its use is the proportion — that wrong releases are about as common as
right ones — not any single verdict. The sheets, the frozen TSV and the enlarged
disagreement views are in `captures/2026-10-04-reader-validation/release-audit/`.
The [summary](2026-10-05-release-audit.json) records hashes and counts. The 26
experiment tests pass; only experiments and documentation changed.
