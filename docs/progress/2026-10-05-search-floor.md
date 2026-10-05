# A search for something that is not in the flat now usually says so

**Asked for something that cannot be in the flat, `find_thing` now says it has
not seen one 54 times in 60, against 37 before.** The price is that it misses 8
of 60 real things instead of 1, all of them small or rarely seen. The bar a
match has to clear went from 0.09 to 0.105, and regions the edge of the frame has
cut down to a strip are no longer ranked. What still gets through is genuine
resemblance in the pictures, which no bar on the score can separate from a real
match.

## The fault

The owner asked the voice model about things that are not there, and it answered
with positions: "a purple elephant" came back found, 0.1 m to the right. The bar
was measured against 31 regions. A search by now ranks the newest 2,000 looks,
and the best of 2,000 chance scores is higher than the best of 31.

## How it was measured

[bench_search.py](../../world_state/bench_search.py), against the live store
(8,770 looks, of which a search is handed the newest 2,000), with the phrases
embedded by the rover's own sidecar. Each phrase is judged as `find_thing`
judges it: the first look among the top ten that belongs to a thing, if it clears
the bar. Two frozen sets of phrases. The first, 34 real things from the labelled
drive of 2026-09-08 and 30 that cannot be in the flat, is what the rule was chosen
on. The second, 26 and 30, was written afterwards to check it. Only found or not
found is scored, not whether the right thing was found.

| Rule | Tuning set: absent found | present missed | Fresh set: absent found | present missed |
|---|---:|---:|---:|---:|
| 0.09, slivers ranked (before) | 13 of 30 | 0 of 34 | 10 of 30 | 1 of 26 |
| 0.105, slivers counted out (now) | 3 of 30 | 5 of 34 | 3 of 30 | 3 of 26 |

These are the repository's own `search.rank` replayed over a copy of the store
taken at 20:20, before and after the change.

A sliver is a region touching the edge of the frame and narrower than 40 pixels
of 640 x 480 either way. A few pixels of window frame matched "a traffic light"
and a strip of chair at the edge matched "a violin".

What is still found: "a grand piano" (the black cabinet), "a canoe" and "a
gondola" (the rug), "a harp" (a dining chair seen edge-on), "a horse" (the cow
painting) and "a dinosaur". What is missed: the wardrobe, the tissue box twice
over, the spray bottle, the blue bin, a portrait painting, a box and a sneaker.

## What was tried and not adopted

About thirty other rules were scored on the same looks, each with its threshold
chosen on half the phrases and judged on the other half. None did better than
about one wrong in eight:

- separation from the rest of the field, as a z-score or as best minus median;
- a hubness correction (CSLS) taking away each look's tendency to score high
  against 124 ordinary phrases;
- a margin over background phrases ("a wall", "the floor", "a blurry photo");
- a veto when an ordinary household word describes the same look better;
- several looks of one thing having to agree.

A higher bar alone (0.11, slivers ranked) gave 8 absent found and 10 present
missed across both sets. Counting slivers out is what buys the rest.

## On the rover

Deployed at 0354494, after the rover had been off the network from about 20:28
until a reboot at 21:31. Run on the rover against the live store (8,789 looks)
with the old rule (`--floors 0.09 --keep-slivers`) and the new one, the bench gave
the same counts as the replay: 23 absent found and 1 present missed, then 6 and 8,
with the same phrases on each side. Through the daemon, `find_thing` now answers
not found for "a purple elephant" and "a giraffe", and still finds the desk
(2.7 m), the bed (2.8 m) and a chair (3.4 m).

## What it leaves open

The bar holds because the count is capped: `Store.searchable` hands a search the
newest 2,000 looks, and ranking more would need it measured again.
