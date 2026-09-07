# The acceptance drive: depth is much better, identity is not, and the floor is still a thing

> **Correction, 2026-09-07.** Two things in this entry were wrong about their own
> data and the text below is left as written, per the rule in
> [the log's conventions](README.md).
>
> **The rug and the ceiling fan are objects, not floor and not background.** The
> section headed "The floor is still a thing to go and look at" counted
> `object:11` and `object:6` (a patterned rug) and `object:26` (a ceiling fan) as
> instances of [R-WS-12](../requirements/world-state.md#r-ws-12). The owner's
> call, and the right one: those are real things that could be worth knowing
> about, and R-WS-12 is about *bare* floor and blown-out background with no
> object in it. On this recording that leaves `object:2` and `object:24` — blown-out
> wall and window patches — as the genuine instances, two of the 33 entities
> reviewed rather than five. That the rug is placed twice is a separate fault, a
> split rather than an eligibility question, and it belongs with the splitting
> section.
>
> **"Nothing reported as missing" is too strong.** The section on the depth
> camera's coverage says a named target went unranged for a whole drive with
> nothing reporting it. The rover does report the shortfall, per look and in
> aggregate: 70 of the 89 inferences carry a line like "4 of 9 ranged by the
> depth camera", and five looks whose regions were *all* outside the depth
> picture say so outright — "none of it was in the depth camera's picture", which
> is a message this code has had all along. The shoe's own five looks reported
> "5 of 11", "4 of 9" and "8 of 12" ranged. What is actually missing is narrower
> and still worth fixing: nothing says *which* regions missed out, and nothing
> carries it to the thing, so "this entity has never had its distance measured"
> is recorded nowhere and reportable nowhere.
>
> **Two entity labels were misread the same way, and this is the larger of the
> two errors.** `object:2` and `object:24`, called blown-out wall and window
> patches both in the section below and in the requirements record, are a ceiling
> light panel and a doorway through to the next room. Re-reviewed at a readable
> size, the recording holds no clear instance of
> [R-WS-12](../requirements/world-state.md#r-ws-12) at all: one entity of 43 is
> arguably a patch of nothing and it was seen twice. See
> [the re-review](2026-09-07-no-bare-patches.md).
>
> **Two observation identifiers were misread** off a downscaled contact sheet.
> The look whose crop is a framed painting is `34333` in `object:3` (not 34555,
> which belongs to `object:30`) and `34235` in `object:9` (not 34236). Re-running
> the separation test with the corrected identifiers leaves the conclusion
> unchanged and better supported: `object:1`'s painting look sits 0.21 m from the
> centre of its eight correct ranged looks, whose own spread is 0.40 m, and
> `object:9`'s sits 0.45 m from its six, whose spread is 0.65 m. Both are inside,
> so neither is separable by position. `object:3`'s painting look carries no range
> at all and `object:21` has only one correct ranged look, so neither of those two
> can judge it either way.

A driven recording through the corrected mount and the new capture gates. **M0
does not pass.** Depth attribution improved from 48% to 66% and the mount change
is vindicated; identity is worse than the software suite can see, the floor is
still eligible as something to go and look at, and the drive turned up an
operating limit nobody had written down — **the depth camera only covers the
middle of the gimbal camera's view.**

The recording is `~/.ugv/archive/world-2026-09-07-acceptance-drive.db` on the
rover with its 88 frames beside it: 502 observations from 78 standing places, one
map session, 42 things, 232 ranges. Two objects were put out and named by the
owner, a pair of shoes and a green tissue box.

## How a range was checked without a tape measure

The ground truth is the parallax. Every thing was seen from several standing
places, and crossing those bearings fixes where it is **without using a single
range**; each range is then compared against the distance from its own look's
viewpoint to that crossing. This is the method the
[M0 baseline](2026-09-07-m0-semantic-world-state.md) used, where 48% of ranges
landed within half a metre.

A crossing is only worth comparing against if the thing is really one thing, so
each entity's crossing carries its own residual — how far the fitted point sits
from each ray that claims to see it. That test turned out not to separate cleanly
(coherent entities run to 0.24 m and the rest start at 0.17 m) because it
conflates a merge with a wide object, so it is used here as a filter on the range
figures and not as an identity verdict. The identity verdict was reached by
looking at the crops.

## Depth: 66%, up from 48%

| | ranges | within 0.5 m |
|---|---:|---:|
| all things with a crossing | 177 | **117 (66%)** |
| things whose crossing is compact | 120 | 89 (74%) |

Per thing, the median range error is 0.275 m and 18 of 23 things agree to within
half a metre. It is distance that drives the error, not where the thing sits in
frame:

| true distance | ranges | median error |
|---|---:|---:|
| under 1 m | 16 | **0.098 m** |
| 1 to 2 m | 43 | 0.283 m |
| 2 to 4 m | 54 | 0.348 m |

This is attribution error, not the depth sensor's precision: it includes the
crossing's own error and the fact that the crossing is the object's centre while
the range is to whatever surface the patch sampled.

**The named targets.** The green tissue box is `object:30` — 12 looks, a compact
crossing, and its ranges agree to **0.155 m**. The shoes came out as two
entities, `object:23` and `object:35`, which is correct rather than a split: a
pair is two objects. `object:35` got one range, accurate to 0.035 m. `object:23`
**never got a range at all**, and the reason is the next section.

## The depth camera sees only the middle of the picture

The shoe that got no range was seen five times and every time at the extreme edge
of the frame — its box centred between 0.04 and 0.20, or at 0.87, of the frame
width. The depth camera was awake on all five occasions. Across the whole
recording:

| box centre, across the frame | looks | with a range |
|---|---:|---:|
| 0.0–0.1 | 72 | **0%** |
| 0.1–0.2 | 64 | 22% |
| 0.2–0.3 | 40 | 78% |
| 0.3–0.7 (middle) | 207 | 62–79% |
| 0.7–0.8 | 47 | 66% |
| 0.8–0.9 | 56 | 32% |
| 0.9–1.0 | 39 | **0%** |

The middle 40% of the frame gets a range 71% of the time; the outer 60% gets one
30% of the time. Vertically the same: nothing at all in the top tenth.

**This is geometry, not a fault.** The gimbal camera is a 130-degree fisheye
whose fitted focal length puts about 99 degrees across its frame; the OAK's
colour camera puts about 70 degrees across its own. So the OAK covers roughly the
central 71% of what the gimbal sees, and anything the gimbal catches at the edge
is outside the depth camera entirely.

**What it costs is that "usable ranges where expected" had no definition of
"expected", and now it has one.** A region is rangeable when it sits in the
central two thirds of the frame. Nothing in the capture or goal path knows that
yet, which is why a named target sat in the store for a whole drive with no
distance ever measured for it and nothing reported as missing.

## Identity: 16 wrong attachments in 394, and range would not have caught them

394 attachments across the 33 entities with five or more looks were reviewed by
eye, as strips of their own crops — far more than the fifty decisions the
criterion asks for. Six entities contain at least one attachment that is plainly
a different object, about 16 attachments in all:

| entity | looks | what is mixed in |
|---|---:|---|
| `object:5` | 19 | two different framed paintings, a blank white patch and a dark cabinet |
| `object:29` | 11 | an armchair, a person sitting with a laptop, a bright window and a dark television |
| `object:1` | 15 | fourteen dining chairs and one framed painting |
| `object:3` | 13 | twelve dining chairs and one framed painting |
| `object:9` | 7 | six dining chairs and one framed painting |
| `object:21` | 5 | four dining chairs and one framed painting |

The cause is one thing repeated: **a chair standing in front of a framed picture
on the wall.** That is exactly the failure
[R-WS-13](../requirements/world-state.md#r-ws-13) already names — "every one is a
thing standing behind another thing, which is where an elevation or range gate
earns its place." This recording reproduces it on fresh data.

**But the range gate would not have earned its place here, and that is the new
part.** Taking `object:1`, whose crops are fourteen chairs and one painting: the
fourteen correct looks place the thing within 0.40 m of their own centre, and the
painting look places it **0.21 m** from that centre — comfortably inside. Any
tolerance loose enough to keep the correct looks admits the wrong one. Raw range
separates them no better: the painting reads 1.524 m while the chairs read 0.67
to 2.90 m around a median of 1.40.

There is a reason to expect that, and it is uncomfortable. The box drawn round
the painting had the chair in front of it inside the same box, so the depth patch
may have sampled the chair rather than the painting — in which case **a range
measured through a box containing two objects at different depths makes a wrong
merge look geometrically consistent.** Elevation does not rescue it either: the
affected entities span 12 to 37 degrees of elevation, and so do entities nobody
faulted (a picture at 19.9 degrees, an armchair at 7.5).

## The floor is still a thing to go and look at

[R-WS-12](../requirements/world-state.md#r-ws-12) is unchanged and now has fresh
instances. Of the 33 entities reviewed:

- `object:11` (12 looks) and `object:6` (8 looks) are both the patterned rug on
  the floor, placed, and split into two things;
- `object:2` and `object:24` are blown-out wall and window patches;
- `object:26` (12 looks) is the ceiling fan.

`object:11` also carries ten ranges with a median error of 0.928 m, which is what
a range to a floor plane sampled through a box does.

## Splitting, which nothing currently counts

42 things from a room holding a few real objects. The dining chairs are spread
across at least five entities, the blue armchair across three (`object:27`,
`object:7`, `object:25`), the rug across two. Some of that is correct — six
identical chairs are beyond what this component can distinguish, and
[R-WS-13](../requirements/world-state.md#r-ws-13) says so — but nothing reports a
split the way a merge gets reported, so the count is not visible anywhere.

## What the new gates did during the drive

All 502 looks were taken at commanded pan zero and tilt zero, inside the
demonstrated envelope, so the envelope gate withheld nothing and every look
carries a bearing. Navigation reported the rover's place on its map as confirmed
throughout, so the confirmed-pose gate passed ordinary traffic. Neither gate's
refusing branch was exercised by this recording — for
[R-WS-16](../requirements/world-state.md#r-ws-16) that demonstration still needs
a restart on the rover.

## Requirements

- [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`. Bearings
  were not measured against an independent reference here; what improved is
  depth attribution, which is a different quantity.
- [R-WS-12](../requirements/world-state.md#r-ws-12) stays `open`, with the rug,
  the ceiling fan and two blown-out wall patches as named instances.
- [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`, and its
  remedy needs rethinking: the recording shows the geometry gate it proposed
  would not separate this room's errors.
- [R-WS-16](../requirements/world-state.md#r-ws-16) stays `open`, untouched.
- M0 criteria 2, 3 and 8 fail on this recording. Criterion 1 passes.

## Next, in order

1. **Refuse or flag a region outside the depth camera's coverage** in the
   capture path, and count them, so a target that can never be ranged is
   reported rather than silently unranged. This is the concrete thing M0's
   criterion 10 is asking for and it is now measurable: the central two thirds
   of the frame.
2. **A box containing two objects at different depths is the root of both
   remaining faults.** It merges a chair with the picture behind it and it
   corrupts the range that would otherwise separate them. Splitting a region by
   its own depth histogram before a range is taken from it is the obvious thing
   to try, and it can be tried on this recording without driving again.
3. Floor and background eligibility, which the rug and the ceiling fan make
   concrete.
4. The R-WS-16 restart demonstration, and the third-distance mount capture at
   0.80–0.85 m with 25–30 degrees of slant.
