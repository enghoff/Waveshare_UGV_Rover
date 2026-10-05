# What a view shows does not decide a release either

**Asking the rover's own text model what kind of thing each release shows does not
tell a right release from a wrong one.** The predeclared rule keeps a release when
its kind matches its record's: a chair view in a chair record, say. On the
[frozen review](2026-10-05-release-audit.md) it would still release 87% of the right
releases, but it keeps only 25 of the 83 wrong ones against the half it needed. On
both older folds, judged by the older labels, it keeps more of the wrong releases
but lets fewer than 80% of the right ones go. It fails all three. It ranks better
than [position](2026-10-05-release-position.md) did, at 0.68 to 0.72 against 0.51
to 0.63, and it fails for the same reason appearance does. The views wrongly
proposed for release are poor views of their record's object, and the kind assigned
to a poor view slips with it. Appearance, position and kind have now each been
tested against a bar fixed in advance, and none decides a release. R-WS-13 stays
open; R-WS-17 and R-WS-18 stay proposed. The rover was used only to embed 26
phrases; nothing on it changed.

## What was tested

[release_category.py](../../experiments/entity_association/release_category.py)
scores each observation's stored SigLIP2 image vector against 26 household phrases
("a photo of a dining chair", "… a framed painting on a wall", "… a rug",
"… a tiled floor", "… an open doorway", and so on). It takes the best phrase as the
observation's kind. A record's kind is the commonest kind among its largest kept
cluster. A release is let go only where its kind differs from its record's. The
phrases and the pass condition were committed before any phrase was embedded
(13a4d45), on the same terms as the position test. The phrase vectors came from the
perception service already running on the rover. That is the same GPU model that
produced every stored vector, which is what a console search uses. The vectors
are kept in `captures/2026-10-04-reader-validation/release-audit/phrases.json`.

| | Right releases still released | Wrong releases kept | Ranking (AUC) | Passes |
|---|---:|---:|---:|---|
| Frozen review, fresh proposal | 62 of 71 | 25 of 83 | 0.68 | no |
| Older fold 0, older labels | 12 of 17 | 7 of 11 | 0.70 | no |
| Older fold 1, older labels | 11 of 15 | 4 of 7 | 0.72 | no |

The same 18 reviewed releases as before could not be scored, because their record
keeps no cluster. As before, the older folds share most of their observations with
the fresh proposal, so they add a second labeller rather than a second recording.

## Why it fails, read after the result

Of the 58 wrong releases the rule would still let go, the commonest slips are
washed-out paintings read as windows (6), armchairs in glare read as office chairs
(5), rug edges read as floor (4), and dim chairs read as doorways (3). These are
the same poor views that defeat the appearance score. The 9 right releases it keeps
are mostly the same confusions run the other way: a rug edge in a floor record read
as floor, bare floor in a rug record whose own views read as floor, an armchair in
an office-chair record read as an office chair. Two more sit in records that are themselves a window
or a doorway as much as an armchair. Merging the kinds most often confused — rug with floor, door with doorway,
the three chairs — would be tuning on this result and was not done. On this review
it could only be checked against the cases that suggested it.

## Where this leaves cleaning up records

The release step is closed as an automatic repair: three kinds of evidence, each
tested against a bar fixed before it was computed, and each fails. Together with
the earlier attach-time gate, joint repair and masked semantic channel, the pattern
is consistent. A wrongly filed view is usually a clear view of a different object
in the same direction. A view wrongly proposed for release is usually a poor view of
the right object. Neither the stored vectors nor the geometry tell those two apart
one view at a time.

That puts a decision to the owner rather than another experiment. R-WS-18 proposes
that no more than one in twenty filed looks be wrong; the store measured 11% on
2026-10-04, and every repair tested so far removes about as many right looks as
wrong ones. The choice is between holding that tolerance, which means changing what
the rover captures (for example, separating a chair in front of a painting at the
region finder), and accepting the current contamination with readers that tolerate
it. The [summary](2026-10-05-release-category.json) records the predicate, phrases,
scores and hashes. The 30 experiment tests pass; only experiments and documentation
changed.
