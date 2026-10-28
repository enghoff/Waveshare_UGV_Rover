# The bedroom is held out of Perception's training

Status: **decided by the owner, 2026-10-28,** before any Perception session.

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
