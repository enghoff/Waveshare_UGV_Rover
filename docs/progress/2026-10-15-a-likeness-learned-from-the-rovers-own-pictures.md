# A likeness learned from the rover's own pictures tells look-alikes apart far better, except the two armchairs, and a dozen staged moves cannot show what it does for following them

**A likeness learned with no labels separates pictures of one object from
pictures of two 97% of the time, against 90% for the masked DINOv2 vectors the
store keeps.** It learned from two kinds of pair the rover's own pictures give
for free. Two regions in one picture are two things. Two regions that one
visit's clusters joined are one thing. The method is the classical KISSME
metric, solved in closed form with numpy. The labels were used only to score
it. [R-WS-17](../requirements/world-state.md#r-ws-17) stays proposed.

| Labelled pair | Learned | DINOv2 |
|---|---|---|
| all objects | 0.97 | 0.90 |
| red and black bean bag | 0.90 | 0.64 |
| animals and village painting | 0.90 | 0.71 |
| snowy-tree and animals painting | 0.95 | 0.81 |
| blue and forest painting | 0.83 | 0.77 |
| plain armchair and the one with the throw | 0.57 | 0.54 |

Each figure is the chance that two pictures of one object are found more alike
than one picture of each of the pair.

**Colour inside each region's outline is a veto, not a likeness.** Made on the
rover from the stored pictures and outlines in 82 s, it separates the two bean
bags completely: they score at most 0.17 against each other, against 0.79 to
0.89 within each. It helps the armchairs a little and the paintings not at all,
and mixing it into the likeness adds nothing.

## In the model

Version 2 of [the world model kept per visit](../plans/a-world-model-kept-per-visit.md)
adds three things to version 1:

- the learned likeness, mapped onto the old scale so that every bar lets
  through the same share of two-thing pairs;
- the colour veto;
- joining a record found standing again after half an hour unseen to a
  look-alike seen gone in the same hour, when the two were never seen in one
  visit: one object at two places.

Its development used all five labelled sessions, since sessions 3 and 4 were
spent on version 1's test, so none of this is a test. Run over every look:

| | Moves kept (strict), of 12 | Mixed records | Unmoved in two records or fewer, sessions 3-4 | Filed, sessions 3-4 |
|---|---|---|---|---|
| version 1 | 9 (3) | 3 | 10 of 11 | 95% |
| version 2 | 7 (2) | 2 | 9 of 11 | 94% |

The two versions cannot be told apart this way. On sessions 3 and 4, version 2
keeps the cabinet and the red bean bag by the strict reading, which version 1
never did. On the earlier sessions it keeps two fewer, and three fewer by
the strict reading.

## What it means

- **The armchairs are the benchmark.** Six of the twelve staged moves were
  one of the two armchairs, and no feature the rover has can tell them apart
  better than a coin toss. They decide every comparison of moves.
- **The other look-alikes in the flat can be told apart**, by the learned
  likeness and colour together.
- **A test that can say whether moves are followed** needs several moves of
  distinguishable things in one session, not two armchairs and a bean bag.
