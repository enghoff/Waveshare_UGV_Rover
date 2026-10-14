# What/where step 2: with navigation's pose, all four arrivals are found by where alone, and gone still finds nothing real

**Given navigation's own pose for the looks the store had refused one, the
where stream saw the rearranged living room. It found both armchairs at their
new places and both bean bags, each in two or more looks, with no flag that
persisted in the corridor or bedroom.** Step 1 of
[prediction in what and where streams](../plans/what-where-streams.md) had seen
the room through 14 of 50 armchair regions
([step 1](2026-10-14-what-where-streams-on-the-shakedown.md)). Now it sees 44.
Gone still finds nothing real. One of the two armchair finds rests on a look
taken a second before the pose went wrong and another after the rover stood
20 cm out.

## What was done

The rover was switched on and nothing on it was changed. `bag_extract.py` read
navigation's pose and the lidar out of the drive recordings of the shakedown,
the seven controls and scene 1 of 2026-10-11, into `/tmp`, and they were copied
here. Against the store's own poses for the same looks, navigation's pose at
the shutter agrees to 3.5 cm at the median. Heading agrees to 0.5 degrees at the
median but by up to 10 where the rover was turning, because the store and the
shutter disagree by up to 0.2 s.

Two changes to step 1, fixed before running on branch `what-where` (`f7141e3`):
- **Poses.** A look the store kept no pose for takes navigation's pose at its
  shutter, unless the rover was turning faster than 20 degrees a second.
  Bearings are worked out by the store's own lens arithmetic, which reproduces
  all 1,658 stored bearings checked exactly.
- **Rooms.** A flag belongs to the room where the flagged thing stands (its
  depth point, else the lidar's return along its bearing), not the room the
  camera stood in.

The streams, memory, windows and rules are step 1's. The thresholds, derived
again on the controls by the same rule, moved by under 0.002.

Navigation's pose is its belief, not a confirmed one
([R-WS-16](../requirements/world-state.md#r-ws-16)). The shakedown's scan
matching found it right in the scored windows until the rover reached the
charger at 08:17:15.

## What it found

| Staged change | Found | Looks flagged, by where | Before 08:17:15 |
|---|---|---|---|
| red bean bag | yes | 16, 5 by where | yes |
| black bean bag | yes | 17, 11 by where; first 42 s into the run | yes |
| charger armchair, new place | yes | 4, 3 by where | yes |
| balcony armchair, new place | yes | 2, both by where (08:00:57, 08:20:51) | no |
| either armchair's old place | no, on the evidence | | |

The rule also counted the charger armchair's old place as found, by gone. The
picture behind that flag is a patch of floor from 2026-10-10, whose sight line
happened to pass the old place. The rule's one persistent false alarm is
another gone flag: a strip of bedroom floor remembered from scene 1. So by the
rule it is five of six with one false alarm, the lidar watch's count on the
same session. On the pictures it is four of six, all arrivals, with no false
alarm. Gone cannot work from region detections; it needs the space seen
empty.

How well where separates the armchairs' 44 posed regions from the 60 in the
untouched rooms: 0.89 as an area under the curve. But only 5 of the 44 pass its
threshold. Their median where score is 0.14 against a threshold of 0.46, held
down by step 1's stale, wide references. Persistence across looks is what found
them.

Eight flags in the living room are on no labelled thing. Four show the staged
armchairs. The other four are cables, a bright patch of floor, the balcony door
and floor shadows, all taken by the charger after 08:17:56, with the pose 20 cm
out.

## What it does not show

The same one session, now scored twice. Thresholds and memory are unchanged
from step 1, so nothing was tuned on it, but the room rule and the pose source
were chosen after seeing step 1's result. Four room-controls give no rate. The
next test must be a fresh staged session, with step 3 frozen before it. No
requirement moved.
