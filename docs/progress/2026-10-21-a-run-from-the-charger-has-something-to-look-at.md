# A run started on the charger now has something to look at: what the parked rover saw from one place is a goal, looked at from somewhere else

The change watch reports a change only once it has seen it from places 0.5 m
apart, so a parked rover reports nothing. On 2026-10-21 two bean bags were set
down beside the rover on its charger
([the fixes on the floor](2026-10-21-the-fixes-on-the-floor.md)). The watch had
both for two minutes and reported neither until a fixed drive had moved the
rover. An autonomy run started there would have had nothing to go on: on the
benchmark's day 2 one ended where it stood. **Since this change, the watch also
publishes what it has seen from one place only, and a run looks at each such
sighting from 0.7 m or more away, which gives the watch its second place.**

## Reproduced first

This morning's lidar, as the world model kept it, was fed to the watch from
nothing: last night's final 30 s before the rover was switched off, which was
its previous visit, then the morning from boot. At 07:40:06, after the moves
and before the drive, the watch reported nothing. It held three groups seen
from the charger only:

- new at P4, 0.50 m across;
- new at P2, 0.35 m;
- gone 0.4 m from P5, 0.30 m.

Five seconds after the drive it reported all three. The live watch, with its
whole history, logged P4 and P2 the same way, and not the third. The live
watch had that place clear the evening before, where the replay's 30 s showed
something there: the replay's thin reference is to blame, not the watch.

## The change

- `ros_nav/change_watch.py` keeps the groups it used to drop for want of a
  second place (`changes(unconfirmed=True)`). Every group now says where it
  was seen from (`viewed_from`) and how far apart those places were
  (`spread_m`). `change_node.py` puts the one-place groups in the report
  under `unconfirmed` and never logs them, so the M5 counts, which read the
  log, are unchanged.
- `autonomy/goals.py` offers each one-place group as a look at a change,
  taken 0.7 m or more from where it was seen (`CONFIRM_FROM_M`), even when the
  place is in view from where the rover stands. It is worth three-quarters of
  its size (`UNCONFIRMED_SHARE`). One near a reported change of its kind is
  that change's look. Once the watch reports the change, the look already
  taken counts as the look at it, so the rover does not go back.

**What one-place groups are.** Over the nine M5 sessions replayed in order, 21
groups were seen from one place only. Fifteen were within a metre of something
staged, and 8 of those were later reported from a second place. Of the other
six, a door was reported seconds later. The remaining five, four never
reported, were one stretch of the balcony window seen from one spot near the
charger in sessions 3 and 4. A look at one of those costs a short drive, after
which it is marked as looked at.

## Against the morning, re-decided

The run's first decision this morning was taken from its stored snapshot on
the rover. The rover was put back on the charger, and the report was replaced
by the replayed one from 07:40:06: nothing reported, three groups seen from
one place.

| Code | Chooses |
|---|---|
| Deployed before this | a look at record `v495` across the corridor; nothing at the bean bags |
| This | the black bean bag's arrival at P4, from 0.74 m off the charger; next the red one's at P2, from 0.73 m off |

## Deployed and checked

Deployed at `66290c4`, to `autonomy` and `ros_nav` only. The rover's own
autonomy suite passed, and navigation restarted and settled on the charger at
08:23 with its position trusted. The live report carries `viewed_from` and
`spread_m` on each change and an empty `unconfirmed` list: nothing had been
moved in its sight since the session. A shadow decision on the rover read it
and considered 49 goals.

Not yet seen on the floor: a run started on the charger after a move made in
the watch's sight. That takes a supervised session.

R-AUT-13 held. No requirement changed state.
