# The bedroom is held out of Perception's training

Status: **decided by the owner, 2026-10-28,** before any Perception session;
**amended 2026-10-31**: the lobby and the ensuite are part of it (below).

Perception holds one room of the flat out of all its training and tuning, so
that what it learns can be judged somewhere it has never seen (its target
GEN-1). Which room it is has to be settled with this project before any session
whose data could tune anything, the stationary trials included
(`D:\Perception\docs\rover-drive-spec.md`, "The held-out room"). The owner chose
the bedroom, on this project's proposal in
[the recorder's plan](../plans/recording-for-perception.md).

## Why the bedroom

It has a door and nothing lies beyond it, so the door can stay shut through
every other session without closing off any route. With a shut door the room is
out of sight as well as out of reach, and Perception's visibility rule, which
otherwise excludes every reading whose view could include any part of the room,
is not needed. It is already on the map, and five of the owner's taped things are
in it to check what is seen there against.

What it costs: the bed limits where the rover may go in the room (a hazard the
drive scripts keep it from, not navigation), and the doorway is narrow enough
that drives near it have been refused and, under the old mapper, the pose went a
quarter of a metre wrong passing through it (2026-10-11).

## What was ruled out

- **The living room** with the dining area: the rover parks and charges there,
  so every session starts in it.
- **The kitchen:** it is open to the living room, with no door, so holding it out
  would mean excluding most of the living room's views.
- **The corridor:** every route between the other rooms crosses it.
- **A room behind one of the doors the owner keeps shut:** cleaner, never seen at
  all, but it would have to be mapped first, and it was not the owner's choice.

## What it requires

- **In every Perception session but the bedroom's own,** the bedroom door is shut
  before the session starts and the session records it shut. The bedroom's own
  sessions are evaluation, and the reference captures and validation for its
  truth, whose contents only the evaluator sees.
- **Its outline is on the map** in every session's manifest, so that a pose inside
  it can be found. It is drawn on map `7da19bef3888` by
  `experiments/rooms/bedroom_outline.py` from the room split of 2026-10-19, cut at
  the bedroom door (where the 2026-10-11 drive stood at it) and grown 0.25 m to
  take in the walls and furniture; 13.5 m² of floor, all five of the owner's
  bedroom things inside, the corridor outside. The outline itself is not kept in
  this repository, which is public. A new map needs it redrawn.
- **This project's own recordings never become Perception's training data.**
  Autonomy runs, the multi-day benchmark, the world store and its archives have
  all been in the bedroom, and nothing here changes what they do; but anything
  from them offered to Perception would carry the room in.

## Amended 2026-10-31: the lobby and the ensuite are part of it

The owner corrected the outline (2026-10-30 and 2026-10-31, through Perception's
session). **The bedroom's door is at about x −11.9 on the corridor, not at
x −10.12 where the outline above was cut**, so the corridor's last 1.8 m (the
lobby) is part of the bedroom, not a room or area of its own. Perception found
the door frame on the grid: a jamb from the corridor's north wall at x −12.0 to
−11.8 and a step in its south wall between x −12.0 and −11.4, to about 0.2 m.
The 2026-10-11 drive's "half way down the corridor" point and the M5 sessions'
corridor-2 viewpoint both lie in the lobby, so they were in the bedroom.

**The ensuite bathroom is held out with it.** Its only opening is its door onto
the lobby, inside the bedroom door, so with that door shut it is out of reach
and out of sight as the bedroom is, and holding it out costs nothing. A manifest
names one held-out room, so the held-out outline (`bedroom` in the manifest's
room outlines) is the bedroom with the lobby joined to the ensuite's outline;
the ensuite keeps its own outline beside it.

The outlines are now Perception's, drawn with the owner for every room on map
`7da19bef3888` (`D:\Perception\data\rover\outlines\`, kept out of both
repositories), and installed on the rover as `~/.ugv/recording/rooms.json` with
the rooms the rover may go into and the bed as a keep-out. Nothing in
navigation reads that file: the bed stays a hazard the drive scripts avoid.
The outline drawn by `experiments/rooms/bedroom_outline.py` is superseded, and
the recordings made before this amendment carry it, which leaves out the lobby.
