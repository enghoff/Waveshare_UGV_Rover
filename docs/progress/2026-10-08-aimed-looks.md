# Aimed looks: filing by aim reaches more targets, but the target's uncertainty barely moves

**Telling the resolver which thing a look was aimed at would put the right region
on its target about three times as often, but in 12 of 16 such cases the target's
stated uncertainty would not change.** That is two blockers for M4, not one. A
region of an aimed look mostly lands on a duplicate. And a thing's placement
uncertainty is the uncertainty of its single best crossing, so a correct new view
seldom moves it. Nothing was changed on the rover; R-WS-13 stays open.

## The rule

An aimed look today is an ordinary look: the inspection call carries no target,
and its regions are filed like any other. The rule, fixed before any look was
scored (`experiments/entity_association/aimed_attachment.py`), is this:

- **Eligible:** a region may go to the target when it points at the target's
  placement as it stood when the goal was chosen, within the resolver's own
  allowance with the map's reach left out. Its height must agree, its range
  where it has one, and its appearance where it can be asked (0.55).
- **Chosen:** the eligible region using the least allowance.
- **Close calls:** a near tie must be broken by appearance, by 0.05, or nothing
  is chosen.

A second, stricter rule searched within 15° for the region that looked most like
the target (0.70, leading by 0.05).

It was run on the 157 aimed looks whose exact picture the
[targeted-look audit](2026-10-06-targeted-look-provenance.md) kept (2026-10-02 to
10-06). The pictures were copied from the rover and hash-checked:
`captures/session67-frames/`, 2,093 photographs.

## What it picked, and whether it was right

| | allowance rule | window rule |
|---|---|---|
| picks | 43 | 6 |
| no region points at the target | 99 | 80 within the window |
| picks the resolver had filed under the target | 5 | 3 |

All 44 picks were reviewed from the photographs beside the target record's
earlier crops, with the rule and the resolver's filing hidden:

| Verdict | Picks |
|---|---|
| shows the target | 16 |
| shows something else | 3 |
| a dining chair, which one cannot be told | 5 |
| target record gone, no reference | 18 |
| target record mixed or not an object | 2 |

For the allowance rule, 16 of its 19 judgeable picks show the target (84%). Of
those 16, the resolver had filed 11 under another record, so the rule adds 11
correct views to their targets, against the resolver's 5. All three wrong picks
had a target whose own record was mixed: an armchair record holding the owner,
and a painting record holding chair backs. The window rule adds one. These are
the analyst's judgments, not owner-confirmed; the labels are in
`.cache/aimed-review/labels.json`.

**Why 99 looks have no region on the target.** For 69 the nearest region misses
the target by a median 6.2° (a quarter by more than 11°). The other 30 point
close enough but do not look like the target's stored crops, many of which are
mixed.

## A correct view seldom moves the placement

Each of the 16 correct picks was added to its target's 24 looks from before the
goal, and the placement refitted as the resolver does it (`locate.best_fix`,
then `locate.refine`):

- 3 targets improve: 0.97 to 0.56 m, 0.94 to 0.17 m, 0.65 to 0.32 m;
- 12 stay exactly where they were;
- 1 gets slightly worse.

The goals predicted 0.10 to 0.26 m. The reason is in `refine`: the uncertainty it
states is the larger of the best crossing's uncertainty and the spread of
agreeing bearings, and the best crossing is chosen by how many bearings agree
with it before how precise it is. So a new view lowers it only when it makes a
better crossing that is also the best supported. That is the
[2026-10-06](2026-10-06-looks-seldom-reach-their-thing.md) record's "right about
one time in ten" seen from the other side. Even a perfectly filed look would
mostly be scored as gaining nothing.

## What it means

M4 needs both fixed. The second comes first, because it decides whether any look
can be shown to help: a placement uncertainty that shrinks honestly as agreeing
evidence accumulates, with bearing errors as heavy-tailed and as shared within a
picture as they were
[measured](2026-10-07-why-one-object-becomes-many-records.md). It must be checked
against positions known independently, not against itself. Filing by aim then
needs the executive to name the target in the inspection call. Because it is an
identity decision, it needs its own case under R-WS-13. The three wrong picks
say where the risk is: targets whose records are already mixed.
