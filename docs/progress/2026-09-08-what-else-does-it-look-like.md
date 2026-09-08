# Asking what else the crop looks like, which is a question nothing was asking

Two changes, neither invented here, both deployed to the Orin at `d1aeef9` and
verified on it. They address the two halves of the merge fault
[this morning's drive](2026-09-08-acceptance-drive-two.md) found, after
[the first attempt at it failed](2026-09-08-acceptance-drive-two.md).

## The literature had both of them

The approach was checked against published practice before being built, and the
bespoke idea turned out to be a standard one with a name.

**Lowe's ratio test.** SIFT threw away a correspondence whose best match was not
much better than its second best, and the same idea is what this resolver was
missing. Every gate in it is a threshold a candidate clears on its own — is it
not plainly unrelated, does it survive masking, is it ahead of the other
candidates the geometry accepted. So a crop that resembled a painting at 0.55 was
admitted without anyone asking that it resembled a chair at 0.81. `_by_appearance`
does compare rivals, but only among things the geometry had already accepted, and
the thing a wrongly-attached crop really belongs to is usually somewhere else in
the room entirely.

**Conservative model update.** The contamination that follows a bad attachment is
the model drift visual trackers have been solving for twenty years, by refusing
to learn from an uncertain match. On this rover a crop that joined on a middling
score became an exemplar immediately, which is exactly how `object:8`'s painting
came to have a chair in its own template.

## What they measure

Over the 923 attachments in the drive, scored by how far ahead the best rival
thing was:

| | lead of the best rival over the thing it joined |
|---|---|
| median attachment | −0.002 |
| 95th percentile | +0.146 |
| the chair in the painting | **+0.265** (rank 8) |
| the person in the sofa | **+0.172** (rank 31) |
| the picture in the cabinet | **+0.272** (rank 7) |

All three faults are in the worst 3.4%. Refusing a lead of 0.15 catches all three
and touches 44 attachments in total. Those 44 were built into sheets — the look,
what its thing usually looks like, what the rival looks like — and read: of the
eight worst, seven are plainly right, including a hanging lamp inside a doorway
and a framed picture inside the cabinet. At the threshold itself they become coin
flips, which is what a boundary looks like.

**It also surfaces splits**, which nothing here reports. A blurry gold-framed
picture sits in `object:76` at 0.56 while `object:47`, which is the same picture
three times over, scores 0.90.

## What they do, replayed four ways

| | things | looks attached | chair/painting | person/sofa | picture/cabinet |
|---|---|---|---|---|---|
| as deployed | 155 | 1025 | separate | **merged** | separate |
| conservative exemplars only | 156 | 1035 | separate | **merged** | separate |
| rival veto only | 154 | 1014 | separate | **merged** | separate |
| both | 152 | 1013 | separate | separate | separate |

Neither change alone separates the person from the sofa and together they do, at
a cost of three things and twelve attachments, with nothing measured made worse.
That is the difference from the attempt earlier today, which fixed one fault and
caused another.

**This is a sample of one.** The replay reproduces only the person-and-sofa merge
from the rover's own three; the other two come out separate under the deployed
build as well, so they cannot be scored. The 0.15 was chosen after seeing the
three faults, which the acceptance plan forbids counting as independent evidence.
It is a development candidate frozen for a held-out drive, exactly as
`COLLAPSED_ALONE` was, and it should be read as "this did not make anything
worse and fixed the one thing that could be tested" rather than as a remedy
demonstrated.

## Deployed and verified

`d1aeef9` on the Orin: world_state 806 passed and rover_daemon 856 passed on the
rover itself, perception and the daemon restarted, and the daemon answering on
8769 with 127 things. The count is unchanged because the store's existing
attachments were decided under the old rules — these gates only govern what
happens next.

## Requirements

- [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`, with a second
  remedy deployed and the same debt as the first: a recording it has not seen.
- No M0 criterion moves. Criteria 3 and 8 need an acceptance sample from a drive
  this was not fitted to.

## Next

The held-out drive is now the bottleneck for three separate deployed remedies —
the collapse test, the ratio test and conservative learning — none of which can
be credited until a recording none of them was tuned on says what they do.
Before that drive, the run manifest wants writing: the thresholds as frozen, the
targets and their measured separations, and the pan and tilt envelope.
