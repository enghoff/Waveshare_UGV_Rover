# The multi-day benchmark compares autonomy with the fixed loop on the same mornings, each judged on its own pictures, by rules set before the first day

Status: agreed with the owner on 2026-10-18 -- the budget and leaving out the
map reset -- and written before the first day was run. It sizes and judges the
plan's
[multi-day usefulness benchmark](../plans/autonomous-curiosity.md#multi-day-usefulness-benchmark)
for its first round. That round ran on 2026-10-19 and 2026-10-20 and was
stopped after two days by the owner, once criterion 3 could no longer be met;
it fails ([the result](../progress/2026-10-20-the-multi-day-benchmark-ends-after-two-days.md)).

## What is compared

**Autonomy (arm A)**: an autonomous run with a nine-minute budget, planning
around the world model's records and the changes the watch reports, as M5
left it. **The fixed schedule (arm B)**: the survey loop out to the bedroom and
back, the same eight stops in the same order with the camera aimed all round
from each (`experiments/m5_changes/session.py`), which takes about nine
minutes. Frontier exploration is not the baseline: the flat is mapped.

One session of each a day, for three days, in the order A then B, B then A,
A then B, then home (`experiments/multiday/day.py`). About 20 minutes of
driving and one charge a day; about 45 minutes of the owner's time a day to set
things out, unplug and plug in, and be within call of STOP.

## The scene

Each morning, before either session, the owner puts three things -- the red
bean bag, the black bean bag and the stool -- on named places in the living
room (`experiments/multiday/benchmark.json`): on day 1 none where the rover
believes it is, on days 2 and 3 two moved and one left where it was. Six
things nobody moves are the controls: the bed, the wardrobe, the floor lamp,
the black cabinet, the bedroom's portrait and the snowy-tree painting.

**Where each thing really is comes from the owner**, who says whether
everything went where asked, and where it went if not. That is written down
(`days/dayN-truth.json`) before the day's answers are read.

## Each arm on its own pictures

Both arms add to one world model, and the arm that drives second would answer
from what the first saw. So the questions are not put to the live memory. The
world model is copied each morning after the scene is set and before either
session drives, and the rover's own engine files onto that copy, separately,
the pictures taken inside each arm's window (`answer.py`). That gives three
states a day: the morning's, the morning's with A's pictures, and the
morning's with B's. On 2026-10-18 both windows of a mock day filed together
onto the morning copy reproduced the live world model exactly, 638 records,
and the replayed records answered every question as `find_thing` did.

## The questions

Put as `find_thing` puts them -- the phrase's SigLIP2 vector, the records ranked
by their average look, the near-ties ordered present and recent first, the
first over the floor answering -- to each state each day:

- **"Where is X?"** for the three things and the six controls: 9 questions a
  state a day, 27 an arm over the three days. An answer from a record still
  standing within 1.0 m of where X is is **right**; one further away is
  **wrong**, an answer that would send a person to the wrong place; no answer,
  or one from a record seen gone or set aside as not found again, is
  **unresolved** -- the rover saying it does not know where the thing is now.
- **"Has X moved since yesterday?"** for the three things on days 2 and 3: an
  arm says yes when its answer is more than 1.0 m from the morning's.
- "What remains uncertain?" is the count of unresolved answers.

## The interruptions

**A restart on day 2**: the daemon and the world model's job restarted with the
rover parked, before the scene is set, every question asked before and after.
**The map reset is left out of this round**: the world model's records belong
to one map and start again from nothing on another, so it would fail by
construction and cost a mapping session. The plan's requirement stands for a
round after the records can cross maps.

## It passes when all of these hold

1. **Retained evidence answers after the restart**: every question gets the
   same answer before and after it.
2. **Stale coordinates drive nothing**: no goal the autonomy arm chose is at a
   record that the morning's records hold as seen gone or set aside.
3. **Autonomy answers better**: over the three days arm A has at least 3 more
   right answers than arm B, and no more wrong ones.
4. **Nothing gets less safe**: no more stops or interventions in arm A than in
   arm B, and both under the same position check, battery floor and STOP as
   every session.

Three of 27 is the smallest difference that would change what is done next; a
smaller one is read as no difference, and is reported as that. Reported
alongside, without a bar: each day's answers, the moved-or-not answers, each
arm's minutes and battery, every stop. A failure leaves M5 standing and the
multi-day usefulness claim unproven, which is itself the answer to whether the
next work is the memory's quality or something else.

## What it does not claim

Three days of one flat with three moved things. The labels of the controls'
places are the main records of 2026-10-18, confirmed from their pictures by
the coding agent; the moved things' places are the owner's.
