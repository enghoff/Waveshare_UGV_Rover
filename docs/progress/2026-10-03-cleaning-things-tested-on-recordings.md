# Cleaning up the things after the fact, tested on recordings: what helps and what does not

**Three ways to clean up the rover's things after they are made were tested on recordings,
and none is ready to build as it stands.** Merging two things that stand together and look
alike cannot be judged on the only drive labelled in full: there, the pieces of one object
were placed 1.6 to 11 m apart. Removing things the rover keeps failing to see would remove
small real objects with the junk. Taking single looks off the thing they are filed under
works partly: half the wrong looks come off for 2% of the right ones. That needs a way to
stop the resolver attaching them straight back, which the live store does not have.
[R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`.

Scripts, the copy of today's store, the crop sheets and the labels are in
`captures/2026-10-03-merge-review/`. Today's store is map session 67 after the greedy
rebuild of 21:00: 182 things from 3,488 looks.

## Merging two things that stand together

On the drive of [2026-09-08](2026-09-08-every-thing-labelled.md), ten objects were labelled
as split across several things. Placed again from their own looks, only 4 of their 15 pairs
stand within 1.5 m of each other. The ceiling fan's two pieces are 4.2 m apart, and the
pendant lamp's are up to 11 m apart. Working the bearings out again through today's lens
moved none of them closer, so the cause was that drive's heading errors, which the heading
check of [2026-10-01](2026-10-01-photo-heading-check.md) addresses. Several split pairs also
share a picture, because the region finder drew two boxes on one object. The store refuses
to merge two things that share a picture. A merge rule would have rejoined 1 of the 15 pairs
there, so that drive cannot tell how a rule behaves today.

On today's session, 38 candidate pairs stand within 0.75 m with crops alike at 0.55 or more
in the median. That set includes the door the resolver splits again after a rebuild. A
sheet of them was made for review, and then set aside: many things hold more than one
object, so the decision belongs to single looks rather than to pairs of things.

## Removing things the rover keeps failing to see

A thing is expected in a look when it lies within 50 degrees of where the camera pointed and
0.75 to 5 m away. On today's session, it must also be in front of the first wall on the
lidar map. It counts as seen when the look found a region on the line to it that looks like
it.

- On the drive of 2026-09-08, which kept no map, floor, wall and glare were seen where
  expected a median 29% of the time, against 55% for real objects. Any cut-off removing 3 of
  the 5 surfaces also removed 12 of the 51 real objects.
- On today's session, 30 of 147 things were seen in under 20% of the looks expecting them.
  Of the 15 least seen, about 5 are junk: floor, a blurred fan, a blurred doorway. The other
  10 are real: switch plates, the hanging lamp, the ceiling fan, a painting, a corner of the
  rug. Small and thin things are rarely boxed. This reading is the coding agent's own.

## Taking single looks off the wrong thing

25 things with six or more looks were drawn at random from today's session. Every look of
each was labelled by eye as the thing's main object, something else, or unsure, and the
labels were frozen before any rule was scored. The result was 363 main looks, 30 odd and 19
unsure. The labels are the coding agent's, at the owner's request, and are not independent.

| Taken off when | Odd looks taken off (of 30) | Right looks taken off (of 363) |
|---|---:|---:|
| the masked crop scores below 0.40 against the thing's other looks | 10 | 5 |
| the masked crop scores below 0.45 against the thing's other looks | 15 | 8 |
| ...below 0.50, and the bearing misses by over 1 sigma or the height by over half its tolerance | 15 | 13 |
| ...below 0.55, another thing nearby looks more like it, and it misses as above | 10 | 8 |

Appearance with the background masked out is the only measure that separates them.
Geometry adds nothing, because most odd looks lie on the same line of sight as the thing:
a chair in front of the painting, the plant beside it, the kitchen seen past a chair. Of
the 30, about 13 are other real objects the rover holds, such as the cow painting filed
under the green landscape. About 10 are doorways or door edges, and about 7 are blur or
glare.

A look taken off would be attached straight back by the same resolver rules that put it
there. The resolver asks `association_allowed` before attaching, and only the September
prototype ever answered it; the live store does not. So this needs a record that a look
is not a given thing before it can be used.

## Duplicates seen while labelling

Three duplicates turned up while labelling: the green landscape painting is `object:246`
and `object:351`, the cow painting `object:249` and `object:340`, and the blue painting of a
building by the sea `object:383` and `object:400`.

## Addendum, the same evening: the owner decided the unsure looks

The owner went through the 19 unsure looks. Three belong to their thing: the cabinet
photographed at night, and the same chair twice. None of the other 16 do. With that, the
labels hold 46 odd looks and 366 right ones, with none unsure, and the odd ones include
harder cases than before. **Only those 19 carry the owner's verdict.** The other 393
labels are still the coding agent's own and have not been reviewed.

| Taken off when | Odd looks taken off (of 46) | Right looks taken off (of 366) |
|---|---:|---:|
| the masked crop scores below 0.40 | 11 | 5 |
| the masked crop scores below 0.45 | 17 | 8 |
| ...below 0.45, or the measured range misses by over 5 sigma | 19 | 9 |
| ...below 0.45, or the range misses by over 5 sigma, or the height by over 1.5 tolerances | 22 | 13 |

So a single rule takes off about two in five wrong looks for about one right look in forty.

## Refusing them at attach time instead, replayed

The same test was then made where the resolver attaches a look, so that a wrong look never
joins in the first place. A look was refused when its masked crop scored below a threshold
against the thing's masked exemplars, at the point where the resolver already refuses a
look whose resemblance collapses when masked. Today's session was replayed through the
deployed resolver, and scored by look against the same labels.

| Threshold | Things | Odd looks still with their object (of 46) | Right looks parted from their object (of 366) | Look-alike pairs within 0.5 m |
|---|---:|---:|---:|---:|
| none (deployed) | 182 | 46 | 0 | 23 |
| 0.40 | 180 | 37 | 54 | 26 |
| 0.45 | 182 | 23 | 73 | 27 |
| 0.50 | 185 | 15 | 86 | 34 |

The deployed replay reproduces today's rebuild exactly: 182 things, 2,860 looks attached.
Each wrong look kept out costs two to six right looks parted from their object, and the
duplicates rise. The same rule applied afterwards to the finished things cost 8 right looks
for 17 wrong ones. At attach time a thing holds at most five masked exemplars, the newest,
and a right look refused there goes on to start another thing. That is the likely
reason, but it was not separately measured. Nothing was deployed.
