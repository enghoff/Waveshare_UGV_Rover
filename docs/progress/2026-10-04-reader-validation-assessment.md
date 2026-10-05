# Fresh analyst assessment supports grouping, but leaves wrong attachments

The owner delegated the doubtful identity judgments to the coding agent rather
than reviewing the six questions. The frozen first-pass labels now support a
development assessment: grouping adds 57 same-object pairs and no different-object
pairs among clearly labelled fresh object observations. The gain comes from two
paintings. An existing painting/table mix remains, four objects still have two
records holding at least two fresh observations each, and 39 clear object regions
remain waiting. R-WS-13 stays open; R-WS-17 and R-WS-18 stay proposed. This is useful
evidence for continued development, not acceptance or reason to enable automatic
merging or expose groups to navigation.

## Analyst decisions and uncertainty

No further owner answers are required to continue development. Working identities
refer to physical objects: a person's head/body are one person, table legs/apron
belong to the table, and painting frame/canvas belong to the painting. Kitchen
regions spanning sink, counter and cabinetry remain mixed. Individual similar
chair identities, clipped/side-on pendant matches, overlapping door regions,
small blurred objects and architectural boundaries remain tentative or excluded;
the pictures do not justify forcing them into confident labels.

The original draft, blank review sheet and review questions are preserved. No
label was changed after seeing the candidate. The 153 clear drafts comprise 146
object regions and seven non-object regions; only the former enter primary pair
scoring. The other 79 drafts are not awaiting owner confirmation as a development
prerequisite. Their uncertainty remains recorded rather than treated as truth.

## Frozen default preview on a real snapshot

The default deployed grouping implementation and weights were applied to a
disposable copy of `captures/2026-10-04-reader-validation/after.db`. The source
snapshot hash, every label's observation/frame/box provenance and all 232 fresh
IDs above 64513 were checked. Source bytes stayed unchanged. No fresh labels or
observations were used to train or tune weights, gates or memberships.

Only pairs whose two observations are fresh are scored, once each. The preview
uses the full snapshot, including historical observations from development and
many of the same physical objects. This measures the deployed candidate on a
later recording, not performance on unseen objects or an isolated new session.
There are no intermediate-checkpoint claims from this end-of-recording pass.

| Clear-object measurement | Original assignments | Grouped reader view |
|---|---:|---:|
| Scored regions / identities | 146 / 16 | 146 / 16 |
| Same-object pairs together | 284 | 341 |
| Same-object pairs apart | 572 | 515 |
| Different-object pairs together | 3 | 3 |
| Different-object pairs apart | 9,726 | 9,726 |
| Waiting regions | 39 | 39 |
| Objects split across two records with at least two fresh regions each | 5 | 4 |

Of the 57 recovered same-object pairs, 55 concern the cattle painting and two the
building-by-water painting. A deliberately broader sensitivity includes all 201
tentative object regions and 23 proposed identities: same pairs rise from 407 to
471, while different pairs stay at 34. Those uncertain identities are not a
23-object acceptance census; they cannot validate the planned minimum coverage.
Pair gains are also not 57 repaired objects or observations.

## A reproduced wrong attachment points to the next task

The three clear different-object pairs all involve painting observation 64638
and table observations 64581, 64592 and 64735, already together in `object:415`.
After scoring, their raw crops and outlines were inspected again: the three
table masks select table legs/apron; the fourth region shows the green painting
behind chair backs. The resolver's stored decision for 64638 reports appearance
0.56 and a compatible bearing to the table record. That records the immediate
attachment decision, not an isolated explanation of why the table's evidence
admitted it. No relabelling or production fix was made from that diagnosis.

The next development step is controlled replay of this attachment with the
record's preceding membership, placement and exemplars restored. Determine
whether a safeguard rejects the painting while retaining genuine table looks,
then test it on other labelled objects. Continue grouping as a revocable
diagnostic; it has a narrow confirmed benefit under analyst labels and cannot
repair existing mixed records. Do not reject the entire direction because it
fails to solve a different fault.

`assess_recording.py` preserves this accounting and writes full pair lists and
per-object memberships. Its waiting/mixed-record accounting tests and the
existing evaluation/review suite pass (12 tests); documentation links resolve.
The [compact result](2026-10-04-reader-validation-assessment.json) records hashes,
decisions and counts. Only workstation experiment code and documents changed;
no deployment, restart or rover movement was required.
