# Checking a guess is built and holds in replay; one look can test a place, not an identity

**The rover can now go and check whether something stands where it believes a
thing is, within limits the daemon enforces, and record every attempt.** This is
M0a's path ([R-AUT-12](../requirements/autonomy.md#r-aut-12)): the goal, the
dispatch limits, the record and the check behind the answer. It has been
exercised against the fake rover holding the real permission rules and replayed
over the real looks of two drives. It has not driven the rover; the supervised
runs M0a asks for are owed, and they are the only thing that can say how often
its answers are right.

## The question is about a place

**One look cannot say which thing it is looking at**, so the check does not try.
On the labelled drive of 2026-09-08, a held-out look of a real object matched the
rest of that object's looks at a cosine of 0.70 or better 43% of the time; a look
at a different object, put where the first was claimed, matched it that well
2.3% of the time over 21,983 pairings. Raising the bar to 0.85 removed the false
matches and kept 5% of the true ones. Requiring the look to resemble the claimed
thing more than any other labelled thing did no better: at a lead of 0.10 it kept
13% of true matches and still passed 2 of 528 wrong ones. That is
[R-WS-13](../requirements/world-state.md#r-ws-13)'s problem, and M0b's.

So the claim an inspection tests is the plainest one a placement makes: something
stands here, to within this much. A look supports it when a region's ranged point
lands inside the claim's uncertainty plus 0.15 m, contradicts it when depth was
measured past the place across the whole of that uncertainty, and otherwise
leaves it unresolved with the reason.

## How strict support has to be

**In a furnished room something stands near almost anywhere.** Replayed over the
looks of 2026-10-01 that kept depth, real things were confirmed by 74% of the
still looks that could see them, and the same claims moved 0.8 m away by 64%;
416 of those 496 moved claims landed within half a metre of another thing the
store holds. Letting a look's own error widen the allowance was what made support
cheap: claims moved into what the store called open floor were confirmed half
the time.

So a look may confirm a place only when its own error, twice its pointing error
at that range and twice its range error combined, is under the 0.15 m allowance.
Then the claims moved into open floor were confirmed once in 44. Most of that
morning's ranges were too coarse to count: on still looks the range error was
0.07 m in the middle under 1.5 m, 0.08 m from 1.5 to 2.5 m and 0.13 m beyond. The
viewpoint is therefore chosen 1.0 to 1.6 m from the place, and real places were
confirmed by 8 of 62 still looks from any distance. Too few of that morning's
still looks fall inside 1.0 to 1.6 m to measure the rate there (4 real claims).

A claim placed looser than half a metre is not tested at all. That keeps 51 of
the 71 things the drive placed, whose median uncertainty was 0.28 m.

## A wrong association, caught

**`object:65` on the drive of 2026-10-01 was two looks at different things whose
bearings crossed in open floor**: a close look at a bare wall corner and a dark
doorway across the room. It was placed 3.89 m from wall B and 2.98 m from wall A,
0.82 m up, to 0.15 m. A later look from 1.48 m found the nearest surface across
the whole patch 2.35 m away, and the check answered contradicted. The picture
agrees: there is open floor there. This is the reproduced incorrect association
M0a's replay asks for, ending in a bounded answer and in nothing that acts on
which thing it was.

The things the 2026-09-08 labels call two objects pooled were mostly supported as
places, from 1 to 22 looks each: one of the pooled objects usually stands where
the looks crossed. That is a correct answer to the question asked, and identity
is not concluded from it.

## What replay covers

Against M0a's second criterion:

- a reproduced incorrect association ending in a bounded answer: `object:65`
  above, replayed by `world_state/bench_inspection.py` over the real recording;
- an absent target: the same case, and claims moved into open floor;
- occlusion and insufficient evidence: real looks answering occluded, too little
  depth, outside the depth camera's view, or a surface with no region on it;
- a stale pose or map: a map replaced between the decision and the drive is
  refused by the daemon, and a look whose direction was withheld answers
  unresolved (`autonomy/test_hypotheses.py`);
- repeated goal generation: a thing renamed between attempts lands on the place
  already spent, and its third attempt is refused, in the ledger and in the
  daemon, and across runs (`rover_daemon/test_inspection_limits.py`);
- budget exhaustion: a drive whose route outruns its attempt's travel is stopped
  by the daemon's watchdog without ending the run, a look after the attempt's
  time is refused, and a case cannot be widened by asking again;
- no identity-dependent follow-on: after the check the executive only reads, and
  an answered place is not inspected again.

The suites: world_state 935, autonomy 697, rover_daemon 1011, all passing.

## What this does not show

How often the answers are right from viewpoints chosen for the claim, at the
check's distances, on still looks with checked headings. The replay's looks were
taken where the drives happened to stand, mostly while turning. That is M0a's
third criterion and needs the supervised runs. Absences at floor level will tend
to go unresolved, because the floor under the place stops depth ever seeming to
pass through it; absent-target cases are better made above the floor, on a table
or a shelf.

## Requirements

- [R-AUT-12](../requirements/autonomy.md#r-aut-12) stays `open`. It is built
  and holds in replay; the hardware trials are owed.
- [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`, with a measured
  limit on what one look can say about identity.
