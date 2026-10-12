# The heading check in a changed room: 55 right looks lost their direction, and the far hits could tell them from a carried rover

**Replayed offline on the M5 shakedown, the check that confirms a look's pose
([R-WS-16](../requirements/world-state.md#r-ws-16)) refused 55 looks whose pose
was right. That is nearly half the 121 right poses, almost all in the living room
after its furniture moved.** Scored on the scan's hits beyond 1.5 m only, the
same search would have confirmed 29 of those 55. It confirmed none of the 22
looks whose pose was wrong, and none of 1,703 simulated carried rovers
displaced by 30 cm or 15 degrees or more. It cannot vouch for a heading to a few
degrees, though: it accepted displacements of 15 cm, or 4 to 8 degrees, twenty
times. Nothing was changed on the rover. Codex's audit found the refusals
(`experiments/perception_bench/shakedown_audit.md`), and the cause is measured
here.

## The refusals

A look's heading check asks navigation for its scan search: a window of 0.5 m
and 45 degrees round the believed pose, which needs 90% of the scan on a wall
and a clear margin over any rival. If the search runs and fits nowhere, the look
gets no direction, and nor does any look after it until a search fits again.
That is right for a rover carried to another room, and it is what the rule was
written for. A room whose furniture has moved fails the same test with the rover
exactly where it believes: the armchairs, bean bags and chairs in their new
places are scan points on no wall.

In the shakedown's run, from 07:52:48 to 07:56:22 every look in the living room
was refused, either by its own check (best fits of 70-73%) or by an earlier one
"and none has since". Directions came back at 07:56:23, the moment the rover left
the room. Navigation's own pose was right throughout that stretch, to within
6 cm.

## The replay

Every look of the session with a scan recorded within half a second of its
picture, 155 looks from 07:44 to 08:17, was given to `ros_nav/refit.fit` offline:
the measure's own window, from the pose navigation believed, on the session's
saved map. Truth is the replay of the session that kept its place through the
bad stretch and ended on the refit's answer (the navigation session's correlation
window of 0.3). That replay was itself 0.35 m off beside the armchair at home
after 08:18, so that stretch is left out.

| Pose navigation believed | Looks | Given a direction live | Far hits confirm |
|---|---|---|---|
| right (under 10 cm and 3 degrees) | 121 | 66 | 74 (45 of the 66, 29 of the 55 refused) |
| wrong (over 30 cm or 5 degrees) | 22 | 13 | 0 |
| in between | 12 | 6 | 3 |

"Far hits confirm" means that, scored on the returns beyond 1.5 m, the best pose
in the window puts at least 75% of them on a wall and is within 10 cm and 2
degrees of the believed pose. Far hits are mostly walls, which do not move. It is
the same idea as the change watch's map gate, which judges a scan on its hits
beyond 1.5 m. In the bad stretch, with the pose 0.8-1.3 m wrong, the far hits fit
the believed pose at 34-64% and their best fit always wanted to move the rover
15-78 cm. In the changed room with the pose right, they fit at 69-87%, within
10 cm.

**A rover carried away is never confirmed.** Each of the 131 scans whose pose
was right was asked again from 13 displaced poses: 15 cm to 2 m, 4 to 180
degrees, and combinations. Of 1,703 asks, none displaced by 30 cm or more, or 15
degrees or more, was confirmed. Twenty small ones were: 15 cm along, or 4 to 8
degrees round, which the far hits alone do not pin down. So far hits can say
"this rover has not been carried", which is what R-WS-16 protects against. They
cannot say "this heading is right to the degree", which is what the check's
correction is for.

## The other gap: looks with no check of their own

13 looks whose pose was wrong got a direction live, and nine of them were stored
from a pose 0.39-1.0 m wrong. None of the nine carried a check, seven of them in
the bad stretch from 08:01:34 to 08:02:07. They were looks taken on the move,
which take the heading navigation believes when no check speaks for them. That is what the
check's own design accepts ("a check that never ran has found nothing wrong").
When navigation itself is wrong, as it was then, nothing in the look path can
notice. This is the gap behind Codex's finding that `find_thing` answered from a
bad-pose look and claimed a decimetre place.

## What follows

Proposed, not built:

- **A changed room is not a carried rover.** When the check's search fits
  nowhere but the far hits confirm, the look takes navigation's heading, as an
  unchecked look does, and the check does not go on refusing every later look.
  On this run, 29 of the 55 refused looks would have had directions, and no wrong
  one. It needs the measure's answer to carry the far-hit fit beside its own,
  which is a change to navigation (`ros_nav/nav_map.py`), and the verdict in
  `world_state/headingcheck.py`.
- **Looks with no check of their own while navigation is wrong** need a signal
  from navigation that it may be lost, not a look-by-look test. That is
  navigation's question first.
