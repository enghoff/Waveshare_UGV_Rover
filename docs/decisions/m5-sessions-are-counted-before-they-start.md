# What M5's sessions count, set before the first

Status: proposed 2026-10-11, for the owner. Nothing here is in force until
agreed; it is written now because the plan wants every count of the
[sessions](../plans/sessions-that-serve-several-milestones.md) set before the
first of them, and the counts cannot come from the recordings the code was tuned
on.

## What counts as found (criterion 2)

- **The owner says what moved**, a sentence a change: what, from where, to where.
  Where it is unclear, the loop's photographs settle it.
- **A move is two events**, gone where it stood and new where it went. A door
  opened or shut is one, counted on its own line, never in either total.
- **Found** is a furniture-sized change of the right kind logged within 0.6 m
  of the place, in the first loop or run after the staging that passes within
  the lidar's 4.5 m of it. A smaller change there is **found small**, reported
  beside and not counted as found. Anything else is **missed**.
- Reported as found over events with its 95% interval: 32 of 40 would read 64
  to 91%. No bar, as criterion 2 says.

Furniture-sized only, because it is the line false alarms are counted at and the
only size the executive goes to look at. It costs something: in scene 1 one of
the four places an armchair left came out small
([replayed](../progress/2026-10-11-m5-scene-1-replayed.md)), and the rate will
be lower by however often that happens, said beside it.

## What counts as a false alarm, and the budget (criterion 3)

- **A room-control** is a room, in one loop or run, that nobody touched since
  the rover's last visit to it. A session gives about seven: three rooms in each
  loop, less the living room after staging and after the previous reset, and
  what the run covers. Three sessions give about twenty, not the forty the
  sessions plan first said.
- **A false alarm** is a furniture-sized change logged in a room-control, or in
  a staged room more than a metre from anything staged, that is not shown to be
  a door or something else that moved.
- **The budget is on false alarms that hold up**, as the plan proposes: an
  apparent change is a reason to look again, and only one still reported once
  the place has been seen from another viewpoint (the other loop, or the
  executive's look at it) counts.
- **Proposed budget: a held-up false alarm in fewer than one room-control in
  ten.** Twenty clean room-controls show the rate is under 14%, thirty under
  10%, so three sessions can show "under 14%" and a fourth would be needed to
  show "under 10%". Which of the two, or another number, is the owner's call.

## The benchmark (multi-day usefulness)

- **Two arms in each session**: the fixed loop (the baseline, as the plan says)
  and an autonomous run given the same time as a loop takes, measured on the
  shakedown. Their order alternates by session.
- **Each arm judged on its own scans.** The change watch is replayed from the
  recording, on that arm's scans alone, starting from what the watch knew at the
  end of the wait. So the second arm does not inherit what the first saw.
- **"What moved?"** Each arm's answer is the changes it found, scored against
  the owner's list: found, missed, false.
- **"Where was this last seen?"** and **"what remains uncertain?"** are asked
  once a session, of the rover as a whole, and reported: the newest sighting of
  a record within 0.6 m of where each moved thing went; the changes reported but
  not yet looked at.
- **Proposed minimum improvement: the autonomous arm looks at more of the
  staged changes than the loop does, finding no fewer and claiming no more
  false ones, over the three sessions.** A loop sweeps every room, so it is
  unlikely to find fewer; what only autonomy does is go back and take a picture
  of what changed. That needs the executive's look at a change turned on
  (`investigate_changes`, in shadow until then).

## What it changes

The session script's counts become these, by script: per staged change found,
found small or missed; per room-control the furniture-sized changes and whether
each held up; per arm the answers to "what moved?". The sessions plan's "about
forty room-controls" becomes about twenty.
