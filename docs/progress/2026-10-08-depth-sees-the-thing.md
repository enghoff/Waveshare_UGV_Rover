# Goals improve their thing when the depth camera can see it

**Of the day's 135 aimed looks with a pose, 14 of the 16 that improved their
thing had it inside the depth camera's vertical view and the other 2 were just
below level; 31 looks at things more than 40 degrees up ranged nothing and
improved nothing.** Geometry
goals now choose viewpoints from which a calibrated tilt puts the thing in that
view, tilt the look level when the thing is below the camera, and set a record
aside for two hours when such a look found nothing to file to it. Replayed over
the day's 173 geometry decisions, the goals chosen with the thing out of view
fall from 29 to none. Built and tested; deployed with this entry. R-WS-13 stays
open. The absence check the rover already has was tried first and said
"nothing there" once in 64 looks.

## The finding

The gimbal rests 20 degrees up and an aimed look does not move it when the
drive already faces the thing, which on the day it always did. The depth camera
(65 by 40 degrees) therefore sees from about level to 40 degrees up. Things were
sorted by their elevation from where each look was taken (sessions 1-3,
`captures/2026-10-08-m4-session-*/aimed-looks.json`, the record's height as now
placed):

| Elevation of the thing | Aimed looks | Filed | Filed with a range | Improved |
|---|---|---|---|---|
| 0-40 degrees, in view | 80 | 16 | 14 | 14 |
| above 40 degrees | 31 | 3 | 0 | 0 |
| below level | 24 | 6 | 2 | 2 |

Paintings seen from a metre or so and things on the floor were most of the 55
looks out of view.

## The absence check, tried first

`hypothesis_check.check` calls a place empty only when depth was measured past
it across its whole uncertainty, and never on a detector's silence. Put through
every aimed look with the place as the goal saw it
(`experiments/entity_association/aimed_absence.py`, 150 looks, 130 with depth):

- at its own limit -- no claim looser than 0.5 m is tested -- it ruled on none;
  loosened to 2 m, most places fell outside the depth view, and it found
  nothing empty;
- asked only about a record's centre, it said "nothing there" once in the 64
  in-view looks that filed nothing; 25 found some surface in the patch it
  tests and 26 had the patch run off the depth image.

It never called a filed look's place empty. As a test of absence it is safe
here and almost silent, so a record is set aside on a weaker but cleaner
signal: an aimed look that could see its place filed nothing to it.

## The changes (autonomy)

- **`goals._tilt_for`**: from a viewpoint the thing's elevation is worked out
  from its placed height, and the calibrated tilt (20 degrees, then 0) whose
  view, 17 degrees either side, contains it. Viewpoints with the thing in view
  are preferred, as the certified distance band is; one with no such tilt is
  kept only when there is nothing else, and `scoring.py` refuses it ("outside
  the depth camera's view"). A thing whose height is too uncertain to say is
  not judged, as before.
- **The look tilts level** (`tilt_deg` 0) only when the thing needs it; at the
  resting tilt the gimbal is not moved.
- **Seen empty**: after a geometry goal whose look had the thing in view and
  filed nothing ("no region of the look points at it"), the record is set aside
  for two hours (`cooling.EMPTY_COOLDOWN_S`) instead of fifteen minutes; an
  improvement to its placement still brings it back at once.

autonomy: 819 tests pass, including the elevations of a painting 1.6 m up from
1 m (out of view) and 2.5 m (in view at rest), a rug 0.4 m below the camera from
1.5 m (in view level), the scorer's refusal, the level tilt on the look and the
two-hour set-aside.

## Replayed over the day's decisions

`experiments/m4_trials/redecide_depth_view.py` restores each geometry decision's
recorded situation and asks the scorer again:

| | Recorded | Made again |
|---|---|---|
| geometry decisions | 173 | 156 geometry, 17 nothing |
| thing in the depth camera's view | 111 | 123 |
| thing out of view | 29 | 0 |
| height too uncertain to judge | 33 | 33 |

The 17 decisions that would choose nothing had only out-of-view viewpoints left.
The replay uses each decision's recorded cooling, so it does not follow on from
one decision to the next as a run would.

## What it does not show

That the change improves a run: the rates above are for looks taken at the
resting tilt, and the level tilt has not yet been used for an aimed look on the
rover. The next supervised session measures it.
