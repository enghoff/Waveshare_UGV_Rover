# The hypothesis check on the rover: three faults found and fixed, and one honest abstention

**A check look now works end to end on the rover: it holds the camera, wakes the
depth camera, aims at the place from the heading the map measures, and answers.**
Getting there took three fixes that only the rover could show, all deployed the
same afternoon at `627a4e6`. No autonomous run was opened; the attempt below was
run by hand through the same calls the executive makes, because opening a run is
the owner's act. [R-AUT-12](../requirements/autonomy.md#r-aut-12) stays `open`.

## What the rover showed

- **The check look was refused as busy, after it had tilted the gimbal.** The
  rover's own looking loop held the camera, and the tilt had been sent before the
  check asked for it, so a look taken at rest saw the gimbal move under it. The
  tilt now happens only once the camera is held, and a check look waits up to
  five seconds for a look already running.
- **No depth was kept.** The depth camera switches itself off half a minute after
  the wheels stop, and the rover was parked. A check look now wakes it, waits
  until it answers `on`, and holds it on for that look.
- **The check faced 34 degrees away from its place.** The navigator called the
  turn arrived 17 degrees short of the heading it was asked for, and its heading
  was 17.5 degrees out besides, so the place was outside the depth camera's
  view. A check look now measures one scan against the map before the shutter
  and pans the gimbal, within the calibrated 20 degrees and reached from below,
  to put the place in the middle of the picture.

## The attempt

The M0a decision on the rover, run read-only, chose `object:99`: believed 1.1 m
from where the rover stood, to 0.15 m, from five looks. Its looks are of
different framed pictures across the room, and their lines cross in open air
about 0.7 m above the floor. It is a wrong association, and nothing stands there.

After the turn, the look measured the heading 21 degrees out, panned 19 degrees,
and put the place at pixel (158, 89) of the depth camera's 320 by 180 picture,
its middle. Across most of the place depth reached 4.9 m, past the table. But
one corner of the place's 0.36 m allowance held a surface 1.23 m away, against
1.14 m expected, which is the top of the pink bucket. So the check answered
unresolved rather than contradicted. That is the answer the rule is meant to
give: it never calls a place empty while anything stands inside the claim's own
margin, because that something might be the thing.

## What is left

The executive's own attempts, under a run the owner opens: M3's supervised stop
and failure checks first, then M0a's three supervised runs. Absent-target cases
are best made above the floor and clear of other things by more than half a
metre, since anything inside a place's margin turns a contradiction into an
abstention.
