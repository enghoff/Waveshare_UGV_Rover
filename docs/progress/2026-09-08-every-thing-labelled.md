# Every thing in the room, given a verdict: two in nine are two objects

**76 things, one written verdict each, and the answer is that 22% of them hold
looks at two different physical objects.** This is the labelling that three
attempted remedies were missing: until now every judgement about identity on this
rover rested on three faults spotted by eye, which is why
[the first attempt today](2026-09-08-acceptance-drive-two.md) could not be told
apart from noise.

The subject is [this morning's drive](2026-09-08-acceptance-drive-two.md) replayed
through the resolver as deployed tonight — the collapse test, the ratio test,
conservative learning and single-viewpoint range placement all in — which
rebuilds to 152 things from 1013 attached looks. Every thing holding four looks
or more got a sheet: five crops at 200 pixels spread across its history, the
identifier burnt in, and anything doubtful zoomed with its full frame behind it.
The verdicts and the split groups are in `captures/m0-2026-09-08/rebuilt/`.

## What the room is made of

| verdict | things | share | looks | share |
|---|---|---|---|---|
| one real object | 52 | 68% | 594 | 71% |
| **two different objects in one thing** | **17** | **22%** | 212 | 25% |
| floor, wall or glare rather than an object | 7 | 9% | 33 | 4% |

M0's criterion 3 asks for **zero** known-incorrect movement-eligible merges in a
sample of fifty decisions. Seventeen of seventy-six is not a marginal failure.

## The faults have shapes, and the shapes repeat

**A person sitting down joins the furniture.** `object:9`, `object:27` and
`object:35` are each the sofa or an armchair with a seated person's looks among
them. Three separate things with the same fault says this is the box containing
two objects at two depths, not bad luck.

**A framed picture joins whatever stands in front of it.** `object:44` is 21
chairs and a painting; `object:26` is a shelf edge and a picture; `object:39` is
a dark door with chairs in it. This is the fault the collapse test was built for
and it survives in a milder form.

**Several similar pictures pool into one thing.** `object:81` and `object:11`
each hold two or three genuinely different framed pictures. A room with a dozen
framed pictures on yellow walls is the hardest case appearance has, and this is
the honest outcome rather than a bug.

## Ten objects are also split across several things

The mirror image of a merge, and nothing here reports it:

| the object | things it is spread across |
|---|---|
| the sofa and armchairs | 6 |
| the dining chairs | 6 |
| a pendant lamp | 4 |
| the rug | 3 |
| a ceiling fan, the green painting, a dark door, the black cabinet, the teal door, the cow painting | 2 each |

Some of that is real — the house has more than one ceiling fan and six identical
dining chairs, and no appearance measure separates those. Some is not: the rug is
one rug and it is three things. **A duplicate count from this rover is an upper
bound and should be reported as unresolved**, which is what the baseline said in
different words and what this now quantifies.

## The useful accident: uncertainty predicts a mixed thing

| verdict | median placement uncertainty |
|---|---|
| one real object | 0.36 m |
| two different objects | **0.60 m** |

A thing that holds two objects is placed between them, so its rays agree less and
it says so in a number the rover already computes and already reports. That is a
signal available at decision time, needing no vectors and no second look, and
nothing currently uses it. It is the most promising lead this labelling turned
up, and unlike the appearance work it can be tested against these 76 verdicts
rather than against three faults.

## Bare floor is back

Seven things are floor, blown-out ceiling, glare or a painted stripe on a wall.
[This morning's re-review](2026-09-07-no-bare-patches.md) found no clear instance
in the rover's own store and withdrew the filter that was about to be written; on
the rebuilt world there are seven, four of them plain floor.
[R-WS-12](../requirements/world-state.md#r-ws-12) has its counter-examples back,
and they are cheap ones — `object:118`, `object:77`, `object:122` and `object:88`
are five crops of tiled floor apiece.

## Requirements

- [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`, and now has a
  measured rate rather than three anecdotes: 17 of 76.
- [R-WS-12](../requirements/world-state.md#r-ws-12) stays `open` with four clear
  instances on the rebuilt world, which is a change from this morning.
- M0 criteria 3 and 8 fail, measured over 76 things rather than sampled.
- Nothing was changed or deployed for this entry.

## Next

The 76 verdicts are the instrument every further attempt should be scored
against, and the first thing to put through it is the uncertainty signal above,
because it is already computed and it separates the two populations by a factor
approaching two. After that, the held-out drive — with the run manifest in
[the acceptance runbook](../runbooks/m0-acceptance-drive.md) filled in first.
