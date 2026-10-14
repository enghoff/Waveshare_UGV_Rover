# What M5's sessions count, set before the first

Status: proposed 2026-10-11, for the owner; amended 2026-10-14 by the owner, a replay in place of the third session (below); the false-alarm budget set the same
day at under one room-control in seven, the owner having no preference between
it and one in ten. Nothing else here is in force until agreed; it is written now because the plan wants every count of the
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
- **The budget: a held-up false alarm in fewer than one room-control in
  seven** (14%). Twenty clean room-controls show the rate is under 14%, thirty
  under 10%. At three sessions' twenty the choice between the two decides
  nothing: none held up passes one in seven and cannot show one in ten, and one
  held up shows only "under 22%", which fails both. So the budget is what three
  sessions can show, and the sessions pass it only with none held up. One in
  ten was the other candidate and would have needed a fourth session to be
  shown at all.

## What counts as identity kept (criterion 4)

Added 2026-10-12, before the first counted session, after Codex's audit of the
shakedown ([experiments/perception_bench/shakedown_audit.md](../../experiments/perception_bench/shakedown_audit.md))
showed how it can be counted.

- **A trial is a staged move of a thing the world store already held**: at least
  two of its regions assigned to records before the move. A thing brought in from
  another room has no identity to keep; it is reported, not counted.
- **Which regions show the thing is a visual label**, by the coding agent from
  the session's crops, each region given to one thing or left out as small,
  mixed or unclear; the owner settles what pictures cannot. Regions from a pose
  the session shows to be wrong are left out of the count and listed.
- **Kept**: after the move, at least two of its regions are assigned, and the
  record holding most of them held its regions before the move. **Split**: they
  go mostly to a record that did not. **Unresolved**: fewer than two are
  assigned. Reported as kept over trials with its 95% interval, the split and
  unresolved beside it; no bar, as criterion 4 says.
- **An incorrect confident merge fails the criterion outright**: any record
  that, during a session, takes a region of a second physical thing as an
  ordinary match. On the shakedown, record 838 holding both armchairs is one.
- The thing's place history is reported beside: where its records were placed
  before and after, against the staged places.

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
  of what changed. That needs the executive's look at a change, on since
  2026-10-12 and weighted to go first to a change nobody has looked at.

## What it changes

The session script's counts become these, by script: per staged change found,
found small or missed; per room-control the furniture-sized changes and whether
each held up; per arm the answers to "what moved?"; per staged move kept, split
or unresolved, and any confident wrong merge, from labelled crops. The sessions
plan's "about forty room-controls" becomes about twenty.

## Amended 2026-10-14: a replay in place of the third session

After two counted sessions on 2026-10-14, the owner chose to replay earlier
recordings instead of running a third. The replay is of 2026-10-10, when the
owner moved the grey armchair by the charger and put it back. Only the return
can be replayed, because no usable recording precedes the first move:
- **The return:** two events, gone where it stood and new at its place. They
  are counted with the sessions' events, from the places that day's entry
  measured on the lidar before any of this was set.
- **The runs where nothing had moved:** they are counted as room-controls,
  each room once.

The watch replayed is today's code, fed the day's recordings in order, with
its history starting at the first of them. The replay is reported beside the
sessions, not mixed into them. The multi-day benchmark still wants three
separate days of sessions and is not served by it.
