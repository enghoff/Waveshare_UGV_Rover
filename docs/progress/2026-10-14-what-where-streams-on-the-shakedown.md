# What/where streams on the shakedown: both bean bags and one moved armchair found, nothing gone, and no false alarm that persisted

**Prediction errors in a frozen appearance space and on the map, against the
rover's own memory, found three of the six changes the owner staged in the M5
shakedown: both bean bags and one armchair at its new place. Nothing they
flagged in the corridor or bedroom persisted, except a bean bag seen through
a doorway.** By its frozen rule the
first step of [prediction in what and where streams](../plans/what-where-streams.md)
shows promise, but only just. The one false alarm the rule counted turned out
to be the red bean bag, seen from the corridor. The lidar change watch found
five of the same six. What the streams add is what they say: the bean bags
were flagged as things never seen before, the armchair as a familiar thing out
of place.

Three weaknesses showed:
- **Gone** cannot work from detections. A remembered thing not being found
  again is weak evidence that it left, because the region finder does not find
  the same things every time.
- **Where** remembered too much. Its memory of a place reached back days into
  earlier arrangements, and often matched only on bearing, with no range.
- **Most living-room looks after the staging had no confirmed pose**, so where
  saw 14 of the 50 armchair regions.

## What was done

Offline, on this desktop; nothing on the rover. The evidence was the
read-only backup of the world store taken on 2026-10-12. Each region the region
finder kept there has a DINOv2 vector of its crop with the background blanked,
its map bearing when the look had a confirmed pose, and its range when the
depth camera measured it. Codex had labelled 118 regions of the two armchairs
and two bean bags on 2026-10-12 for the identity audit, before this method
existed. Those labels are the truth here.

Nothing is learned or rendered. Each furniture-sized region gets three scores:
- **new**: how unlike anything in the house's memory it looks;
- **out of place**: how much better it matches something elsewhere than
  anything the 30 latest views from within 1 m and 45 degrees hold in a
  consistent place (bearing within 15 degrees, range within 1 m where both
  were measured);
- **gone**: per remembered region the look should have seen, how well memory
  agreed on it minus how well the look matches it, at its lowest over a visit.

Thresholds are the 99th percentile of each score on the seven unchanged M5
controls of 2026-10-11, with memory frozen at each control's start:

| | new | out of place | gone |
|---|---|---|---|
| threshold | 0.322 | 0.459 | 0.932 |

For the shakedown, memory was frozen at 07:48:05, when the staging began. The
run until the pose went wrong (07:52:15-08:00:58) and everything after the
refit (08:11-08:21) were scored. A change counts as found when flagged in two
or more looks.

The code, protocol and thresholds were committed before the shakedown was
scored, on branch `what-where` (protocol `6517144`). The frozen code counted a
range as measured between 0.2 and 8 m, and its comment wrongly said that was
the store's rule, which is 0.75 to 12 m. Run again afterwards with the store's
limits, the thresholds, the finds and the counted flag came out the same.

## What it found

| Staged change | Found | Looks flagged | Stream |
|---|---|---|---|
| red bean bag | yes | 12, first 33 s into the run | new, every one |
| black bean bag | yes | 7, first 86 s in | new 6, out of place 1 |
| charger armchair, new place | yes | 3 | new 1, out of place 2 |
| balcony armchair, new place | no | 1 | out of place |
| either armchair's old place | no | none | gone flagged nothing |

The shakedown's lidar watch, on its own counts, found five of six, four with
the pose right. It had every scan of the session; the streams had only the
looks with a confirmed pose for where.

**The corridor and bedroom, the room-controls**, after loop A:
- The frozen rule counted one persistent false alarm. It is regions 100799 and
  100803, which Codex labelled as the red bean bag, at its second place, seen
  from the corridor. The rule judged rooms by where the camera stood.
- On the labels there was none.
- Two flags were each in one look only: a dark strip of bedroom floor, and a
  doorway.

How well each score separates, as areas under the curve against the
room-controls' regions, leaving out the four labelled bean-bag regions seen
from the corridor (60 left):
- new, for the bean bags' 51 regions: 0.89 (0.85 keeping all 64);
- out of place, for the armchairs' 14 posed regions from nine looks: 0.88
  (0.84 keeping all 64).

Both thresholds sit near the top of the positives' scores, so most single
regions of a staged thing were not flagged. Persistence across looks is what
found them.

## Why the moved armchairs mostly went unflagged

For each posed armchair region, the best reference in a consistent place was
traced. It came from 2026-10-03 to 10-11, never from that morning's first loop,
and one came from scene 1 of 10-11, when the armchairs stood elsewhere. In 10 of
the 14 it had no range, so it agreed on bearing alone. Thirty references were
chosen on the controls because three left one unchanged region in ten scoring
high: the region finder crops the same thing differently every look. That
depth of memory also remembers old arrangements. The tolerances that absorb the
region finder's noise also absorb a move: one balcony-armchair region was
explained by a reference 14.9 degrees and 0.69 m away, both allowed, where the
staged move was 0.90 m. A memory of things in their latest state, and ranges
from the lidar for things on the floor, are the plan's answer, not yet shown.

Two of the armchair flags, the charger armchair's second "out of place" and
the balcony armchair's only one, come from looks at 08:20:51. That is after the
rover stood 20 cm out short of the charger, beyond what the heading-check
replay of 2026-10-12 vouches for. Without them the charger armchair is still
found, once by each stream, but no armchair is found by "out of place" alone.

Every remembered region near an old place that two looks expected was
unranged, so gone had nothing at those places it could place.

## What it does not show

One session, staged once, labelled by Codex rather than the owner. Four
room-controls cannot give a false-alarm rate: "no persistent false alarm the
labels do not explain" is what was seen, not a rate. Thresholds and memory depth
came from the controls, but the size floor is Codex's labelling floor from this
shakedown, and the bar borrowed the lidar watch's known result on it. The
controls are one morning of one flat.

Codex reproduced every threshold and test number from the frozen code, found
no observation after the cutoff in memory, and confirmed the camera's pan sign;
its corrections are in the branch's `results.md`.
Navigation's pose for the looks the store kept none for is in the drive
recording on the rover, which was offline, so the where stream's coverage in
the rearranged room is unmeasured. No requirement moved.
