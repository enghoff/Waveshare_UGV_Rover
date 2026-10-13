# The voice model's search finds things again, because it ranks things

**Asked for a thing the rover has seen, `find_thing` now names the right one far
more often: by eye on 65 phrases for things in the flat, it named the wrong thing
for 8 of them, where the search it replaces named the wrong one for 17 and found
nothing for 8 more.** It ranks each thing by the average of all its looks instead
of ranking the newest 2,000 looks, and it lets 3 of 60 phrases for things that
cannot be in the flat through. The things it still gets wrong are ones the store
barely holds: a keyboard, a tissue box, a spray bottle, a mirror, a window.
The model also has to ask: spoken "Where are my shoes?", it called no tool at all
two times out of two, and with one sentence added to its instructions telling it
that it remembers the place, it called `find_thing` two times out of two and said
where they were. Separately, when the rover cannot measure a route because it has
not confirmed where it is on a restored map, the model is now told that, instead
of "nothing is publishing the rover's position".

## The fault

The owner reported that the voice model could not get sensible answers out of
the store. The conversation log of 13:20-13:25 IST shows two refusals and a
detour:

- "the bedroom" and "the bed" went to `go_to_thing`, which found the bed and
  then refused: "nothing is publishing the rover's position". That was untrue.
  The rover had come up on its saved map at 13:20:46 with the mapper's anchor 41
  degrees from where it had been parked, so its place on the map was unconfirmed
  ([R-WS-16](../requirements/world-state.md#r-ws-16)); SLAM was publishing a
  position the whole time. The model told the owner it did not know where it was
  and would have to explore.
- Asked about shoes, the model took a picture instead of calling `find_thing`,
  saw a wall, and set off exploring.

Asking `find_thing` directly over the daemon's port afterwards, with no model
involved, showed the search itself was the larger problem:

- "shoes": not seen, scoring 0.101 against a bar of 0.105, though the sneakers
  are in the store as a thing of 16 looks.
- "my shoes": found, but the thing it named was the side of the black sofa,
  three looks, 1.3 m wide.
- "a lamp": not seen, while quoting a best score of 0.128, above the bar. The ten
  best looks all belonged to no thing, and the search only walked ten.

The search ranked the newest 2,000 looks (`Store.searchable`) and walked the top
ten for the first that belonged to a thing. By 10-13 the store held 41,554 looks.
The newest 2,000 spanned 35 hours; 636 of them belonged to a thing, and those
covered 198 of the store's 686 things. 873 had been taken that morning while the
map was unconfirmed, which records the picture and no direction, so none of them
could become part of a thing. [bench_search.py](../../world_state/bench_search.py)
over the same store, judging as `find_thing` did, missed 12 of its 60 real-thing
phrases, against 8 when the bar was set on 10-05.

## How it was measured

Six ways of turning a phrase into a thing were run over a copy of the store, with
phrases embedded by the rover's own sidecar: the 120 frozen phrases of
`bench_search.py` (60 real, 60 that cannot be in the flat) and six from the
conversation. For each real phrase a contact sheet showed what each ranking
settled on, and each was judged by eye as the thing described or not. "The front
door" was left out as undecidable from the pictures, leaving 65.

| Ranking | Wrong thing named | Nothing named | Absent found | Present missed |
|---|---:|---:|---:|---:|
| newest 2,000 looks, first of the top ten that belongs to a thing (before) | 17 | 8 | 2 of 60 at 0.105 | 11 of 66 |
| newest 2,000 looks that belong to a thing | 18 | 0 | 6 of 60 at 0.105 | 2 of 66 |
| each thing by the best of its newest three looks | 16 | 0 | 6 of 60 at 0.111 | 1 of 66 |
| each thing by the best look it has | 14 | 0 | 6 of 60 at 0.125 | 6 of 66 |
| each thing by the mean of its best three looks | 12 | 0 | 6 of 60 at 0.113 | 1 of 66 |
| **each thing by the average of all its looks (now)** | **8** | 0 | 3 of 60 at 0.10 | 4 of 66 |

"Wrong thing named" counts the thing each ranking put first, whether or not it
cleared a bar; the last two columns are found or not found at the bar given. Every
ranking but the first was given a bar that let 6 absent phrases through, the
number the 10-05 bar let through, to compare them fairly; the adopted one got its
own.

Most of the disagreement between rankings is between duplicates. The store holds
several things for one object -- the rug as at least five, the dining chairs as
many more -- and every ranking names one of them. Where they differ in kind,
ranking by the best look is undone by lucky looks: a thing with 400 looks has 400
chances to resemble anything. The average asks what a thing looks like across
everything it was seen as, and a thing that is two objects stitched together (the
bed and the wardrobe as one) averages to neither.

**The bar for the average is 0.10, chosen on the first phrase set and checked on
the second.** On the first set, 2 of 30 absent phrases were found and 1 or 2 of
34 present ones missed anywhere from 0.0975 to 0.105; the upper half was taken,
to err towards not found. On the second it found 1 of 30 absent and missed 2 of
26 present, and both misses (a computer keyboard, a tissue box) were things it
named wrongly when it named anything. What gets through is resemblance in the
pictures: a blurred dark-wood thing as a violin and as a grand piano, a lamp's
glowing bulb as a traffic light.

## What changed

- `world_state.search.ThingIndex` keeps every thing's average look in the daemon:
  normalised looks summed and normalised again, per backend
  ([R-WS-4](../requirements/world-state.md#r-ws-4)), slivers left out. Building
  them all reads every look, 1.3 s on the rover, so it is done once and then only
  for things whose looks changed, which `Store.thing_prints` answers in about
  160 ms. `rank_things` ranks them against `THING_MATCHES = 0.10`.
- `find_thing` and `go_to_thing` use it. A look that belongs to no thing is no
  longer ranked by them at all; it has no position and nothing to drive to. The
  console's own search is unchanged.
- When there is no position to measure from, the answer says why: no position
  published, the map still loading, or the rover's place on a restored map not
  yet confirmed, which a position check from the console settles.
- A "not found" answer no longer quotes scores to the model.
- `bench_search.py` judges the new way by default and the old way with
  `--looks`, and `--sheets` writes the contact sheets to judge by eye.

Deployed at 038838b. On the rover afterwards, through the daemon's port with no
model: "shoes" and "my shoes" found the sneakers, 5.2 m straight ahead; "a lamp"
and "a floor lamp" the floor lamp behind it; "a purple elephant" answered "none of
the 683 things it has seen looks like that". A question takes about 0.3 s. The
rover's heading still agreed with the scan to 0 cm and 0.0 degrees after the
daemon restart.

## The model, asked

With the search fixed and deployed, "Where are my shoes?" was synthesised at
16 kHz and put through the console's own session code on the rover against
plus-realtime, with the live daemon's tool list. Five calls in all, about 29,000
tokens, kept few because each is paid:

| Instructions | Question | What the model did |
|---|---|---|
| as deployed, 21 tools (no camera registered) | "Where are my shoes?" | no tool; "I don't know where your shoes are right now ... first I need to explore the area to build a map" |
| as deployed, 24 tools | "Where are my shoes?" | no tool; "I don't know where your shoes are right now" |
| with the memory sentence, 24 tools | "Where are my shoes?" | `find_thing("shoes")`; "about 5.2 metres straight ahead of me" |
| with the memory sentence, 24 tools | "Where are my shoes?" | `find_thing("shoes")`; "about 5 metres straight ahead of me" |
| with the memory sentence, 24 tools | "What can you see?" | `look` (the control: a picture, not the memory) |

`find_thing`'s description already said to use it for "where is the desk"; what
the instructions lacked was any statement that the model remembers anything. The
sentence, now `MEMORY_PROMPT` in [prompts.py](../../voice_chat/prompts.py), is
sent only when `find_thing` is offered, last, where it was measured. Two samples
a side is little, and the difference is all or nothing in both.

## What is not changed

The size a found thing
reports is the size of whatever region the resolver made a thing of, so the black
sofa came back 0.2 m wide because the thing was its armrest. And the duplicates
are still there ([R-WS-17](../requirements/world-state.md#r-ws-17)); this only
chooses among them better.
