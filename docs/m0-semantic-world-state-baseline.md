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
where the *other* camera is pointed. That cannot be a property of the mount; it is
the gimbal camera's own pointing.

### The gimbal does not come back to the same place

The shape of that spread says which part. Taken as deviations from the pan-0 fit
the three positions are -3.57, 0, +0.88, and no symmetric error does that: a pure
gain error on the pan servo -- the obvious suspect, since this rover's pan is known
to under-travel, told -30 and landing near -27 -- would walk the fit equally and
oppositely either side of zero.

So the bench was run again with the pan positions in both orders. **The first
attempt at that was designed wrongly and its answer was thrown away**, and it is
worth saying how, because the wrong answer was a tidy one. Running `-20 0 +20`
against `+20 0 -20` opposes the approach direction only at pan 0: each end is the
first stop of one order and the last stop of the other, and both of those
approaches move the same way. The ends duly showed no difference and pan 0 showed
1.76 degrees, and that table is exactly what *uniform* backlash also produces. The
"ends are immune" reading, and the mechanism invented to explain it, were artefacts
of the design.

The test that separates them overshoots, so that every sampled position is
interior: sweep -30 to +20 sampling at -20, 0 and +20, then +30 to -20 sampling the
same three. Now each has a genuine approach from either side. Runs are paired
adjacently and in alternating order, so the drift that runs through a sitting
falls on opposite sides of successive pairs and cancels. And the floor is measured
per position rather than assumed, by two runs back to back **in the same
direction** -- because an insensitive fit and an honest zero look identical, and
the two ends do not see the same scene as the middle:

| commanded pan | floor, same direction | backlash, drift-cancelled |
|---:|---:|---:|
| -20 | 0.03 | **+1.58** |
| 0 | 0.03 | **+1.58** |
| +20 | 0.05 | **+1.32** |

The fit is equally sensitive at the ends as in the middle, to a twentieth of a
degree, so a small difference there would have meant something. It is not small.
**The backlash is about one and a half degrees and it is everywhere in the
gimbal's travel**, thirty times the floor, not a property of pan 0. That is the
worse of the two readings: no pan angle is repeatable, rather than one.

### Three faults, two of them identified

All three are measured behaviours. Only the first two have a cause attached; the third is a real effect whose mechanism is still open, and
the heading is worded that way deliberately -- a summary line written when
a finding looked settled is exactly what outlives the finding.

Within a single sweep the fitted yaw also walks steadily with pan, the same way in
both directions -- ascending 2.65, 3.20, 4.35 across -20, 0, +20 and descending
0.95, 1.89, 2.85. That is a **gain-like error**, and unlike the backlash it is
poorly determined. Taken over all ten runs of the sitting the slope ranges from 3.7
to 7.6 per cent, mean 5.6 with a standard deviation of 1.4 -- so a quarter of its
own value, because it is a difference *across* positions in a scene that drifts
about a degree over a sitting, where the backlash is a difference between two runs
minutes apart. It is real and it is in the region of the 10 per cent the known
under-travel implies, but the honest figure is "four to eight" rather than a number.
(An earlier draft of this document said 7 per cent throughout; that was the
descending runs of one block, which happen to sit at the top of the range.)

Backlash is an offset; this is a slope; they are different faults and the sweep
shows both at once. A two-point calibration would have averaged them into one
number that fits neither.

The roll is a third thing and neither of the first two can produce it: it walks
about 0.09 degrees per degree of pan, and a gain error in pan cannot move roll at
all. A pan axis leaning fore-and-aft would -- and the direction is already pinned,
because a sideways lean would move pitch instead and pitch is flat at 2.1
throughout. The tilt that would explain the roll is about 5.6 degrees, and by
`cos(5.6)` that same tilt shortens the yaw by only half a per cent, so it accounts
for about a fourteenth of the 7 and cannot be the cause of it. Three independent
faults, by arithmetic rather than by assumption.

Whether the roll is really the axis is **not settled**, and the reason to doubt it
is that a wrong lens model also puts roll into this fit, since features sweep
across the frame as the camera pans. What would choose between them is linearity: a
tilted axis gives a straight line, a bad distortion model gives a curve. The
measured roll slope falls across the sweep -- 0.153, 0.087, 0.080 degrees per
degree in one run and 0.139, 0.093, 0.096 in the next, reproduced well above the
0.05 floor -- which looks like a curve and would favour the lens.

The obvious escape is that the horizontal axis of that plot is *commanded* pan
while the servo under-travels, so the curve might be an artefact of the axis. **It
cannot be, at least not from a constant gain error.** If actual pan is some fixed
fraction of commanded, then roll stays linear in commanded and only its slope
changes: rescaling an axis does not bend a straight line. Manufacturing this curve
needs the under-travel itself to *worsen with deflection*, which is plausible for a
servo but is a stronger claim than "it under-travels" and is not in evidence. So
either the servo is non-linear or the lens model is wrong, and the same
commanded-against-actual sweep decides which, because it shows whether the gain is
flat or bends. The tilt stays a leading suspicion and no more.

### What it costs the world state

**The store sits almost entirely at pan 0, but nothing makes it so.** Of the 1549
observations the rover holds, 1451 were taken at pan 0, and every one of the 98
that was not falls inside the half hour of bench runs described above -- which is
to say the rover's own looks have all been taken straight ahead. That is luck
rather than design: `rover_world` never aims the camera, it captures wherever the
gimbal happens to be pointing, and `look_at` is a model tool, face tracking drives
the gimbal, and a script can. The instant anything aims it, world-state looks are
taken at that angle.

That distinction decides how bad the error is, because the two faults behave
differently with pan. Backlash is flat: about 1.5 degrees wherever the camera is,
which at pan 0 is already the whole of the 1.5 a bearing here is believed to, and
which way the gimbal last moved is recorded nowhere, so it cannot be corrected
after the fact. The gain error vanishes at pan 0 and grows from there -- across the
measured range of slopes it is 1.1 to 2.3 degrees at pan 30, about 1.7 at the mean.
**So a bearing taken straight ahead can be out by 1.5 degrees against a budget of
1.5, and one taken at wide pan by roughly 3 against the same budget.**

That is worth holding next to the standing measurement that half of every bearing
this rover records already falls outside the accuracy the resolver is told to
expect. At pan 0 backlash alone accounts for it. The consolation is that the term
which grows with pan is the correctable one: a gain is a systematic function of
commanded angle, so measuring commanded against actual removes it in software.
Backlash is harder, and the tilt does not touch bearing at all.

The order of work follows from the arithmetic. **Measure the pan servo's commanded
angle against its actual one first** -- at both signs, several magnitudes, and
approached from both directions, without fitting a single gain to it. That settles
the gain -- which the bench cannot pin down better than "four to eight per cent"
because the scene moves under it -- and it is also the thing that has to be known
before the roll curve can be read at all. The fisheye model and the pan-axis tilt are both still open
behind it.

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
   does not reproduce its own 2026-09-04 measurement, and the gimbal it is
   measured against carries about 1.5 degrees of backlash at every angle plus a
   pan-dependent gain error somewhere between 4 and 8 per cent. Neither is
   corrected, and the
   mount constant cannot honestly be re-measured until the servo is.

## What has to happen before Phase 1

The identity work is in better shape than the phase assumed and the geometry is in
worse shape. In order:

1. **Measure the pan servo's commanded angle against its actual one, from both
   directions, and then re-measure the OAK mount.** Nothing else here can be
   settled first, because every OAK number is expressed relative to the gimbal
   camera. The gimbal carries about 1.5 degrees of backlash at every angle in its
   travel, which is the whole of what a bearing here is believed to, and pan 0 --
   where every world-state look is taken -- is no better than the rest. The cheap
   version of the test is `bench_oak.py --pan -30 -20 0 20` against `--pan 30 20 0
   -20`, paired adjacently in alternating order, with two same-direction runs for
   the floor. Do not use `--pan -20 0 20` against `--pan 20 0 -20`: it opposes the
   approach only at pan 0 and will tell you the ends are clean when they are not.
2. **Refuse entities made of bare floor.** Two of fourteen reviewed entities are
   floor, one of them confident to 0.18 m. An executive choosing where to look next
   would spend real distance on them.
3. **Treat the 0.55-to-0.70 appearance band as geometry-only.** Every merge error
   found sits there, and every one is a thing standing behind another thing. This
   is where an elevation or range gate earns its place, not in the band above.
4. Leave the six identical dining chairs alone. Nothing in this component can tell
   them apart and the honest record says so.
