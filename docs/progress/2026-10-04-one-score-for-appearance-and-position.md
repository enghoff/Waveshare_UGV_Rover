# One score for appearance and position: what it buys for joining, cleaning and merging things

**There is an established way to turn a look's appearance and position into one number,
and on this rover's labelled looks it says that appearance does nearly all the work and
position mostly rules things out.** The number is a log-likelihood ratio. Each kind of
evidence contributes the log of how much likelier it is if the look shows the thing
than if it shows something else, and the contributions add. The weights are fitted by
logistic regression on labelled examples. Fitted that way, a bearing that misses by
three standard deviations costs about as much as 0.1 of resemblance in one of the two
crop measures. Used four ways on today's copy of the rover's store:

- **Taking single wrong looks off a thing** is no better with the combined score than
  with yesterday's masked-crop rule. At 8 of 366 right looks taken off, both remove 15
  or 16 of the 46 wrong ones.
- **Joining looks to things as they arrive**, with the combined score choosing among the
  things the resolver's tolerance admits, keeps far fewer wrong looks (14 of 46 rather
  than all of them) and far fewer duplicates (5 to 8 look-alike pairs rather than 24),
  and joins no two labelled objects. But it parts 104 of the 366 right looks from their
  object: about half are left waiting and half start a second thing.
- **Merging two things** that the store would allow to merge: of 26 candidates the score
  proposed, sampled from 37, 20 are plainly one object and none is plainly two. It finds
  one of the three labelled duplicate pairs, because the other two are stored where their
  own looks do not put them.
- **Rebuilding a few neighbouring things from all their looks at once**, ignoring how they
  are filed now, does best. Leaving the dining chairs aside, it keeps 7 of 37 wrong looks
  with their object, against 10 for single-look removal at the same cost of 34 of 311
  right looks, and it rejoins 4 of the 6 duplicate halves, which removal cannot.

Nothing was deployed. [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`.
The labels are the coding agent's own apart from the 19 the owner decided. Every score
was fitted and judged on the same session, though never with weights fitted on the
thing being judged. Scripts, the copy of the store and the contact sheets are in
`captures/2026-10-04-association-likelihood/`.

## What other systems do

Tracking systems have combined position and appearance this way for decades.

- **The track score of multiple-hypothesis tracking** ([Blackman, 2004](https://scispace.com/pdf/multiple-hypothesis-tracking-for-multiple-target-tracking-3fzibddaoe.pdf))
  is a log-likelihood ratio: the kinematic term and the "signal" or attribute term are
  each the log of how much likelier the measurement is under "this target" than under
  "something else", and they add.
- **Fitting the weights by logistic regression** is how speaker and forensic recognition
  turn several similarity scores into one calibrated likelihood ratio
  ([the BOSARIS toolkit](https://arxiv.org/abs/1304.2865)). It is what was done here.
- **Multi-object trackers** mostly use an uncalibrated weighted sum.
  [DeepSORT](https://arxiv.org/abs/1703.07402) adds a motion distance and an appearance
  distance, gates on both, and in its own setting gives the motion distance no weight at
  all: appearance decides inside a motion gate. [StrongSORT](https://arxiv.org/abs/2202.13514)
  puts 0.98 of the weight on appearance.
- **Object maps from 2D detections**, such as [ConceptGraphs](https://arxiv.org/abs/2309.16650),
  add a geometric overlap score to a normalised feature cosine, attach greedily to the
  best match above a threshold, and merge similar objects periodically.
- **Soft association by expectation-maximisation**
  ([Bowman et al., 2017](https://github.com/seanbow/semantic_slam)) is what
  [yesterday's whole-session test](2026-10-03-whole-session-em.md) tried.
- **Rebuilding identities from scratch** is correlation clustering, also called multicut:
  every pair of detections gets a log-odds of being one identity, from logistic
  regression on appearance and geometry, and the partition that agrees with most of them
  is sought ([Tang et al., 2017](https://is.mpg.de/publications/people-tracking)). The
  usual fast solver, greedy additive edge contraction (Keuper et al., 2015), joins the pair
  of clusters with the most evidence between them until no join is supported
  ([ClusterFuG](https://arxiv.org/abs/2301.12159) describes it).

The resolver today is DeepSORT turned round: position ranks the candidates and
appearance only vetoes or breaks ties.

## The data

The rover's store was copied at 09:59 with sqlite's backup: map session 67, 3,540 looks
and 188 things, with last night's 182 still filed as they were. The labels are those of
[yesterday's test](2026-10-03-cleaning-things-tested-on-recordings.md): 25 things, 366 right
looks and 46 wrong ones. The labels' notes name three duplicate pairs: the green
landscape (`object:246` and `object:351`), the cow painting (`object:249` and
`object:340`) and the painting of a building by the sea (`object:383` and `object:400`).
Dining chairs are labelled by kind, so chair-against-chair is never scored.

Every look was described against a thing's other looks, never its own:

- **Position:** the bearing's miss beyond the thing's width, in units of the claimed
  bearing error (3 degrees still, 5 moving) combined with the placement's own doubt
  across the line of sight. The range miss and the height miss are measured the same way.
- **Appearance:** the median cosine of the plain DINO vector, of the masked DINO
  vector, and of the SigLIP vector against the thing's other looks.

Right looks miss their thing by a median of 0 standard deviations (90th percentile 1.4),
which says the claimed bearing error is honest. Wrong looks miss by 1.1, because most of
them lie on the same line of sight: a chair in front of a painting, or a doorway beside
it. Crop resemblance is 0.71 for right looks, 0.57 for wrong ones, and 0.40 for other
labelled things the resolver's tolerance would have admitted.

## One look against one thing

Fitted on right looks against wrong looks and admitted other things, and scored with
each labelled thing held out:

| Evidence used | Right against wrong in their own thing (AUC) | Wrong looks off, of 46, at 4 / 8 / 19 right looks off |
|---|---:|---|
| position only | 0.68 | 7 / 7 / 9 |
| appearance only (plain and masked DINO) | 0.89 | 6 / 15 / 20 |
| both | 0.89 | 10 / 15 / 22 |
| both, with SigLIP as well | 0.90 | 10 / 16 / 23 |
| yesterday's rule, masked crop below 0.40 / 0.45 / 0.50 | | 11 at 5 / 16 at 8 / 25 at 29 |

The fitted weights are -0.07 per squared standard deviation of bearing miss, -0.16 of
height miss, nothing for range, and +5.5 and +5.9 for the plain and masked cosines.
Including the duplicates as "same" pairs makes position look worthless or worse: they are
two to five standard deviations from each other's stored placement, so a model taught on
them learns that a miss means a match.

## Joining looks as they arrive

The session was replayed look by look through the resolver with one change. Among the
things the deployed tolerance admits, each region's evidence for each thing is the
combined score, against every look the thing holds. A pairing below a threshold is not
allowed, the regions of one picture are shared out to maximise the total evidence, and a
region whose best arrangement is within half a unit of its second best is left waiting.
Founding new things is unchanged. The weights were fitted on half the labelled things and
the replay scored on the other half, then the halves swapped; each row is both runs.

| | Things | Looks attached | Wrong looks kept (of 46) | Right looks parted (of 366) | of them left waiting | Duplicate pairs joined (of 3) | Different labelled objects in one thing | Look-alike pairs within 0.5 m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| deployed | 182 | 2,887 | 46 | 0 | 0 | 0 | 0 | 24 |
| combined, threshold 0 | 124 and 142 | 2,677 and 2,629 | 14 | 104 | 47 | 3 and 1 | 0 and 0 | 5 and 8 |
| combined, threshold -1.5 | 96 and 101 | 2,840 and 2,766 | 20 | 86 | 36 | 2 and 2 | not counted | 4 and 3 |
| combined, threshold -3 | 91 and 96 | 2,851 and 2,851 | 10 | 120 | 27 | 2 and 3 | 1 and 3 | 0 and 2 |

**It changes which mistake the resolver makes rather than removing mistakes.** The
deployed resolver almost never parts an object, and pays for it with wrong looks and
duplicates. The combined score takes most wrong looks and nearly every duplicate out,
and pays with right looks. Lowering the threshold attaches nearly as many looks as the
deployed resolver and halves the number of things, but does not bring the parted right
looks back with their objects, and at -3 it starts joining different objects; why the
right looks stay apart was not traced. A replay took about as long as the deployed resolver's, 2 to 4
minutes on this desktop.

## Merging two things

Two placements are compared by their error ellipses, not by a fixed distance. A thing
placed from a narrow crossing is known to 0.93 m along the line of sight and 0.12 m
across it. Geometry contributes the track-to-track log-likelihood ratio: a Gaussian on
the two ellipses under "same", and under "different" the room's density of things, 0.90
a square metre. Appearance contributes the mean calibrated evidence over pairs of their
looks.

- **Geometry can rule a merge out but cannot make the case for one.** Things here are so
  dense that two placements in exactly the same spot score at most about +2.
- 67 of the 7,729 pairs within 4 m score above zero. 30 of them share a picture, which
  the store refuses to merge: either two boxes drawn on one object or two objects
  photographed together, and which was not checked.
- Of the other 37, 26 were drawn at random and judged on contact sheets by the coding
  agent. 20 are plainly one object: the cow painting, the black cabinet four times, the
  floor lamp, the sea painting, the dark door, the green landscape twice, the armchair
  twice, and others. One pair is dining chairs and one a ceiling fan. 5 are glare, blur or bare
  wall edges that cannot be judged. **None is plainly two different objects.**
- Of the three labelled duplicates it finds only the sea painting (+0.45). The green
  landscape scores -0.15. The cow painting's second record, `object:340`, is stored 2.4 m
  away at 0.4 m high against 1.6 m, and shares a picture with the first.
- A fixed 0.5 m radius, which is how the resolver's look-alike pairs are counted, cannot
  find any of the three: they stand 0.6, 2.4 and 1.4 m apart as stored.

The duplicates are stored where their own looks do not put them. Pooling the right looks
of both records and fitting one position, the sea painting's 17 looks all land within 3
standard deviations and the cow painting's 52 all but 2. Taking the sea painting's one
wrong look out of `object:400` moves its fit 1.8 m along the line of sight.

## Rebuilding a few things from their looks

For each labelled thing, the group is itself and every thing within 0.75 m, or within 3 m
and an ellipse distance under 2: a median of 10 things. Their looks are pooled and
clustered from scratch:

- Each pair of looks gets the calibrated evidence of its plain, masked and SigLIP cosines
  (AUC 0.956 for one object against two, against 0.944 for plain DINO alone).
- Two regions of one picture never join.
- Clusters join while the summed evidence between them, less one per pair, stays positive.
- A join is refused when the pooled position no longer fits the smaller side, a median
  miss over 2.5 standard deviations. This changed very little.
- A cluster of one look is let go.

| | Right looks parted from their object | Wrong looks kept with it | Duplicate halves rejoined | Other labelled objects pulled in |
|---|---:|---:|---:|---:|
| as filed (the deployed resolver) | 0 of 366 | 46 of 46 | 0 of 6 | 0 |
| rebuilt | 56 of 366 | 8 of 46 | 4 of 6 | 1 look |
| rebuilt, dining chairs aside | 34 of 311 | 7 of 37 | 4 of 6 | 1 look |
| single-look removal at the same cost, chairs aside | 34 of 311 | 10 of 37 | 0 | 0 |

Each labelled thing was scored with pairwise weights fitted on the other half of the
labelled things. Fitted on all 25, the rebuild parted 46 right looks (25 with the chairs
aside) and kept the same wrong ones, which is how much in-sample fitting flattered it.

Of the 46 right looks the in-sample rebuild parts, 7 are let go and 39 sit in other
clusters. Those clusters hold few labelled looks, so whether each is a second piece of the same object or a
neighbour's could not be read from the labels. 21 of the 46 are dining chairs, which the
labels cannot judge: a chair filed with another identical chair is not an error by them.
A second pass that joins clusters standing together and looking alike, by the
merge rule above, made 54 joins and barely changed the result: 44 right looks parted and
10 wrong ones kept, in-sample. Wider groups (1.5 m, a median of 24 things) parted 57
right looks, kept 6 wrong ones and pulled in 3 looks of other objects, in-sample. On
this desktop the 25 groups took 32 s, about a second each.

## What is left

The rebuild is the one approach so far that removes most wrong looks without throwing
right ones away, and it merges duplicates as a side effect. The combined score is the
way to build it: the look-against-look evidence it clusters on is that score, and the
merge rule above is its thing-against-thing form. Joining looks by the combined score as
they arrive trades duplicates for parted objects. Removing single looks gains nothing over
the masked crop. Three things stand between the rebuild and the rover:

- **One session and mostly the coding agent's labels.** The 09-08 drive cannot test the
  grouping, because its heading errors put one object's pieces metres apart. One more
  labelled drive, ideally labelled by the owner, would settle whether the gain over
  single-look removal holds.
- **Only groups around the 25 labelled things were rebuilt.** Rebuilding the whole store
  needs the overlapping groups tiled so that each look is decided once, which was not
  tried.
- **A look it lets go would be attached straight back.** The resolver attaches pending
  looks by its own rules, and the live store still has no record that a look is not a
  given thing.

## Addendum, the same day: ranking inside the current checks, and merging afterwards

Two more replays, scored the same way, with every weight fitted without the things being
scored. The first keeps every check and refusal the resolver has and changes only how it
ranks the candidates that pass: by the combined score instead of by how much of the
bearing tolerance they use. The second runs the merge rule above over what a replay
built, joining the best qualifying pair first and refitting the union, until none is left.

| | Things | Wrong looks kept (of 46) | Right looks parted (of 366) | of them left waiting | Different labelled objects in one thing | Look-alike pairs within 0.5 m |
|---|---:|---:|---:|---:|---:|---:|
| deployed | 182 | 46 | 0 | 0 | 0 | 24 |
| deployed, then merged | 155 and 159 | 46 | 0 | 0 | 0 and 0 | 1 and 1 |
| ranked by the combined score | 171 and 168 | 11 | 114 | 8 | 0 and 0 | 18 and 18 |
| ranked by the combined score, then merged | 150 and 152 | 13 | 81 | 8 | 0 and 0 | 4 and 2 |

Ranked with a looser ambiguity margin (0.5 rather than 0.1), it kept 18 wrong looks and
parted 125 right ones. Of the 114 right looks the tighter version parts, 102 sit in a
second record of their own object, 4 went to a thing holding a different labelled object,
and 8 were left waiting.

- **Merging after the deployed resolver is a clean gain on duplicates.** It joined 23 and
  27 pairs, took the look-alike pairs from 24 to 1, rejoined 3 of the 6 labelled duplicate
  halves over the two runs, and put no two labelled objects together. It does nothing for
  wrong looks.
- **Ranking by the combined score swaps wrong looks for split objects**, and merging
  afterwards rejoins only about a third of the split: 81 right looks still sit in a second
  record that the merge rule does not join. Why it does not was not traced.

