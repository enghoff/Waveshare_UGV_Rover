# M0 baseline: is semantic state safe enough to steer the rover?

Phase 0 of [`task-autonomous-curiosity.md`](task-autonomous-curiosity.md) exists to
stop semantic state choosing where the rover drives while persistent identity is
still unproven. This is what the drive of 2026-09-07 says about that.

**The milestone does not pass.** Five of the six numbered criteria hold, and the
sixth thing the phase asks for -- validating the camera geometry the planned
active perception will lean on -- fails outright, in a way that puts a
qualification on one of the five. Identity itself came out better than expected;
the geometry underneath it did not.

## The recording

Outside Git, because it is frames and a database:

```text
captures/m0-2026-09-07/
  world.db     43 MB, sqlite backup of ~/.ugv/world/world.db taken 08:33
  frames/      257 jpgs, the whole of ~/.ugv/world/frames
  map.json     the occupancy grid from the nav bridge, for the reach bound
  review/      one contact sheet per entity, both replays, and the zooms
```

| | |
|---|---|
| Run | 2026-09-07, 07:20:37 to 08:28:30 local |
| Map | `7660766cbfd4`, world map session 67, and no other session in the file |
| Rover | `jetson-orin`, world\_state deployed at `e9068b8` |
| Repository | `43e7cb2`; no file under `world_state/` differs between the two |
| Observations | 1262, of which 583 carry a measured range |
| The drive itself | 07:20 to 07:40, 1190 observations from about 180 distinct poses |
| After that | parked, 72 observations from one pose |
| Gimbal | pan 0 for every look in the file, without exception |
| Store at the snapshot | 97 entities, 395 unmatched |

Two things happened during the recording that a later reader needs told. The
drive was the rover's owner at the console, not an autonomous run. And at about
07:50 another agent deployed `ros_nav` and restarted the stack, so slam\_toolbox,
Nav2 and the nav bridge went away and came back inside the window -- the map
survived it, `map_id` did not change and the world session stayed at 67, which is
itself part of the evidence for criterion 5 below.

## What the depth camera actually contributed

Every region in the recording was put through the same mapping the daemon uses to
find a gimbal box in the OAK's picture (`oak.box_for` on the corners
`inspection_ranges._corners_of` draws). Of the 1105 regions in looks taken while
the camera was awake, 598 fall inside the OAK's picture and **583 of those, 97 per
cent, came back with a range**. Not one region outside the picture carries a range,
so the gate is doing what it claims.

The ranges themselves are 0.39 m to 6.64 m, median 1.66 m, with a median stated
error of 0.098 m. As numbers they are entirely plausible, and criterion 2 passes on
them -- but read the geometry section before trusting what they are ranges *to*.

## The replay had never been carrying the ranges

`world_state/replay.py` re-inserts a recording's observations into a fresh store
one look at a time and calls the live resolver after each. The list of columns it
copied did not include `range_m`, `range_sigma_m` or `camera`, so **every replay
ever run against this component was a bearing-only replay whatever the rover had
been doing when it recorded the run**. The comparison this phase asks for was not
merely un-run; it was un-runnable.

The columns are carried now and `--no-ranges` drops them on purpose, which is what
makes one recording answerable both ways. The evidence that the flag reproduces the
old behaviour is that bearing-only replay returns exactly what the harness returned
before the change, to every count.

The claim in that file's own docstring -- that replaying an unchanged build
reproduces the entities the rover ended up with -- is worth revisiting in this
light. It is truer now than it was: this recording replays to **97 entities, the
same 97 the rover itself was holding**, and did so only once the ranges were
carried. Bearing-only gives 100.

## Identity, both ways

Same recording, same resolver, same map:

| | bearing-only | range-assisted |
|---|---:|---:|
| entities | 100 | 97 |
| observations attached | 854 | 816 |
| still waiting | 377 | 415 |
| crops of something other than the entity's main thing | 0 | 0 |
| bearings that miss the entity's own position | 54 (6%) | 62 (8%) |
| decisions: new / match / ambiguous | 100 / 533 / 184 | 97 / 501 / 154 |
| matches at appearance 0.70 or better | 235 | 211 |
| matches at appearance 0.55 to 0.70 | 298 | 290 |

Ranges make the rover **less** willing to join things, which is the direction that
costs coverage and buys precision. They also cut the genuinely undecidable cases:
154 ambiguous against 184.

The stray count moving the wrong way -- 6 per cent to 8 -- is real and is not
explained here. A placement pulled to a measured distance can end up further from
the bearings that also voted for it, and given what the geometry section says about
where those ranges were sampled, that is the reading to prefer until it is
measured.

## The review

An association decision is judged by looking at every crop an entity holds and
answering one question: are these one physical object? The sample is the ten
largest entities of the range-assisted replay plus fifteen drawn at random from the
rest, which is **14 entities carrying 311 regions and about 250 match decisions**
once the sheets with nothing to judge are set aside. The plan asks for fifty.

Clean, by eye -- every crop plainly one object:

- a framed animal picture, 41 crops, including five near-edge-on views that read as
  a dark strip until the full frame is put behind them;
- the black armchair, 38 crops;
- a framed picture with pampas grass in front of it, 38 crops;
- a second framed picture, 19; the ceiling fan, 11; a wooden tray, 10.

Wrong, and named:

| entity | what was joined | appearance |
|---|---|---:|
| `object:16` | a framed picture on the wall above the dining table, twice, into an entity of dining chairs | 0.57, 0.58 |
| `object:16` | and the same picture again as the entity's founding observation | -- |
| `object:23` | the glass coffee table into the black armchair | 0.55 |

**No merge at or above the resolver's own `RECOGNISED` threshold of 0.70 was found
to be wrong**, and that is the sense in which criterion 3 passes: zero known
incorrect high-confidence merges in roughly 250 decisions, five times the sample the
plan asks for. Every error found sits in the 0.55-to-0.70 band, where geometry is
deciding and appearance is only failing to veto.

That band is where the danger is, and the mechanism is worth stating plainly: the
reason a picture joins a chair is that **the picture hangs directly behind the
chair from where the rover was standing**. The resolver's first pass offers a
pending observation to the things already placed, and the reason it records is
almost always geometric -- "the bearing points at object:16 3.72 m away, appearance
0.58". Appearance is a veto that did not fire, not a vote that was won.

Left unresolved rather than called either way, because the room defeats the
question: three entities are dining chairs and the room holds six identical ones
standing within a metre of each other, so no amount of looking at crops says
whether one entity is one chair. One entity is the person in the room together
with the chair they are sitting in, which is two things that were genuinely in one
place.

Two entities are made entirely of **bare floor** -- 13 crops and 11 crops of
featureless tile and skirting, one of them placed to within 0.18 m. These are not
merge errors, since the floor is consistently the floor. They are worse for what
comes next: they are confident, drivable goals that are not objects at all, and
nothing in the pipeline currently refuses them.

### Duplicates

51 pairs of entities in the bearing-only replay, and 73 in the range-assisted one,
stand within reach of each other and look alike at 0.70 or better. Both numbers are
upper bounds and probably mostly wrong, for the reason above: six identical chairs
around one table produce exactly this signature whether they have been counted
once each or several times over. Reported as measured, and not as a duplicate
count.

## What the ranges refused

Eleven entities the bearing-only replay invented have no counterpart when the
ranges are carried. Two were checked by eye and are plainly false crossings:

- one pooled ten crops of blown-out floor and a doormat edge into a thing placed
  0.35 m *below* the floor with 1.16 m of uncertainty. The single-link appearance
  score cannot see this error at all -- the crops really do look alike, because
  they are all bright featureless floor -- so this is a case only range or
  elevation could catch, and range caught it.
- one pooled a blue case with a dark red wooden surface at 2.6 m up, near the
  ceiling. One of its five regions had a range; pulling that one to its measured
  distance was enough to stop the rest crossing there.

Against that, the range-assisted run made four merges of its own that bearing-only
did not, which is the honest other half. But bearing-only makes the identical class
of error elsewhere -- its `object:12` is dining chairs with the same wall picture
joined three times -- so the picture-into-chair failure is not something ranges
introduced. They moved which entity it lands in. **Criterion 4 holds:** no increase
in false high-confidence merges, and false crossings demonstrably rejected.

## The geometry does not hold up, and this is the finding that matters

`oak.py` records the OAK's mount as measured on 2026-09-04: yaw -1.53, pitch +3.11,
roll -2.12 degrees, with the offset deliberately left at nothing and a written-down
prediction of what putting the ruler's offset in should do. Three fresh runs of
`bench_oak.py` at gimbal pan 0, in a textured living room with points fitted from
3.0 to 5.2 m, say:

| | yaw | pitch | roll | residual, median |
|---|---:|---:|---:|---:|
| run 1 | +4.59 | +2.17 | -1.70 | 1.19 |
| run 2 | +4.72 | +2.28 | -1.63 | 1.16 |
| run 3 | +4.67 | +2.25 | -1.57 | 1.17 |
| **stored in `oak.MOUNT`** | **-1.53** | **+3.11** | **-2.12** | |

Repeatable to about a tenth of a degree between runs, and **6.2 degrees away from
the constant the rover is using** -- four times what a bearing on this rover is
believed to. The prediction written into `oak.py` also fails: given the ruler's
offset of 40 mm forward and 110 mm below, the pitch was expected to fall by about
1.9 degrees and it rose by 2.09, to +4.26.

The decisive result is the consistency check. The OAK is bolted to the chassis, so
what the bench fits for it must not change when the gimbal moves. Across three
gimbal positions:

| gimbal pan | yaw | pitch | roll |
|---:|---:|---:|---:|
| -20 | +1.14 | +2.06 | -2.99 |
| 0 | +4.71 | +2.19 | -1.67 |
| +20 | +5.59 | +2.02 | +0.43 |
| | spread **4.45** | spread 0.18 | spread **3.42** |

Pitch is stable. Yaw and roll are not: the same bolted-down camera fits four and a
half degrees of different yaw, and three and a half of different roll, depending on
where the *other* camera is pointed. That cannot be a property of the mount. It is
the gimbal camera's own model going wrong away from its axis -- either the fisheye
fit, or the pan servo's commanded-against-actual error, and the roll varying is
what points at the lens, since a roll is exactly how a bearing error bleeds into an
elevation one off-axis.

**`oak.MOUNT` has deliberately not been changed.** Adopting the pan-0 fit would
replace one number with another that three positions disagree about, which is the
mistake that file already warns against in its own words: half of a consistent pair
is worse than neither half. The mount cannot be settled before the gimbal camera's
off-axis model is.

### What that costs the ranges

Every look in the recording was taken at gimbal pan 0, where the stored constant is
out by 6.2 degrees. That is 8.6 per cent of the OAK's picture width, so a region
box narrower than that misses its target entirely, and at 3 m it is 32 cm sideways.

Measured rather than argued: taking the bearing-only placements as an independent
yardstick -- they never saw a range -- and comparing each measured range against the
distance from that look's pose to where the crossings put the thing, the ranges show
**no systematic bias, median -0.08 m, and very wide tails: -1.48 m at the tenth
percentile, +1.24 at the ninetieth, with only 48 per cent agreeing to within half a
metre.** Unbiased and scattered is the signature of a box that lands on a
neighbouring surface rather than one that is offset in a fixed direction.

So criterion 2 passes on the letter -- the recording contains usable ranges where
range is expected -- and should be read with this attached: a good fraction of them
are honest depth readings of the wrong thing.

## Where each criterion stands

1. **the offline suite passes** -- yes. `python world_state/selftest.py`, 748
   passed, 0 failed, 0 skipped, before and after the replay change.
2. **a fresh recording with usable OAK ranges** -- yes, 583 of 598 in-picture
   regions, correctly gated; qualified by the geometry above.
3. **zero known incorrect high-confidence merges in at least 50 decisions** -- yes.
   About 250 decisions reviewed, no error at or above 0.70, four errors below it,
   all named.
4. **range-assisted does not increase false merges, and rejects a false crossing**
   -- yes, with two rejections confirmed by eye.
5. **map-clear and map-session behaviour still keeps old coordinates out** -- yes,
   from the named offline tests and from the live evidence that the 07:50 stack
   restart kept `map_id 7660766cbfd4` and left the world session at 67, so nothing
   in the recording is measured against a map that had gone.
6. **a concise baseline report** -- this document.
7. **validate the remaining camera-to-rover geometry** -- **no.** The OAK mount
   does not reproduce its own 2026-09-04 measurement, and fails its own
   pan-consistency check by 4.45 degrees of yaw.

## What has to happen before Phase 1

The identity work is in better shape than the phase assumed and the geometry is in
worse shape. In order:

1. **Fix the gimbal camera's off-axis model, then re-measure the OAK mount.**
   Nothing else here can be settled first, because every OAK number is expressed
   relative to that camera. `bench_oak.py --pan` is already the test that fails, so
   it is also the test that passes when this is right.
2. **Refuse entities made of bare floor.** Two of fourteen reviewed entities are
   floor, one of them confident to 0.18 m. An executive choosing where to look next
   would spend real distance on them.
3. **Treat the 0.55-to-0.70 appearance band as geometry-only.** Every merge error
   found sits there, and every one is a thing standing behind another thing. This
   is where an elevation or range gate earns its place, not in the band above.
4. Leave the six identical dining chairs alone. Nothing in this component can tell
   them apart and the honest record says so.
