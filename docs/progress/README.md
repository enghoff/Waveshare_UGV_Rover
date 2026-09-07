# Progress: what was actually measured

This is the log. Each entry is a dated report of something measured on the rover
or replayed from a real recording, and says which
[requirements](../requirements/README.md) it moved and to what. Entries are
written once and not revised afterwards: an entry is a record of what was known
on a particular day, and rewriting it destroys exactly the thing that made it
worth keeping.

## The ledger

Newest first. "Moved" names the requirements whose state this entry changed;
"held" names requirements it confirmed without changing.

| Date | Entry | Moved | Held |
|---|---|---|---|
| 2026-09-07 | [A look only gets a direction from a state that was measured](2026-09-07-capture-state-gates.md) | none; deploys the confirmed-pose gate [R-WS-16](../requirements/world-state.md#r-ws-16) asked for and the gimbal envelope M0's criterion 10 asks for | [R-WS-16](../requirements/world-state.md#r-ws-16) still `open` pending its hardware demonstration; [R-WS-10](../requirements/world-state.md#r-ws-10) still `failing`; 84% of the rover's looks are at a tilt the pan campaign never visited |
| 2026-09-07 | [Where the OAK actually sits, and the arithmetic error that hid it](2026-09-07-p0-oak-mount.md) | none; supplies the OAK-to-gimbal translation [R-WS-11](../requirements/world-state.md#r-ws-11) was half-blocked on, and adopts the mount transform | [R-WS-10](../requirements/world-state.md#r-ws-10) still `failing`, untouched by this measurement |
| 2026-09-07 | [M0 review: bounded calibration and usable operating limits](2026-09-07-m0-review.md) | none; acceptance scope clarified | 748 local software checks passed; no new physical acceptance |
| 2026-09-07 | [Why a refit could not find a rover whose heading was 152 degrees out](2026-09-07-refit-window.md) | none; supplies the measurement [R-WS-16](../requirements/world-state.md#r-ws-16) was owed, and evidence toward [R-NAV-9](../requirements/navigation.md#r-nav-9) | [R-NAV-2](../requirements/navigation.md#r-nav-2), [R-NAV-3](../requirements/navigation.md#r-nav-3), [R-WS-2](../requirements/world-state.md#r-ws-2) |
| 2026-09-07 | [M0 baseline: is semantic state safe enough to steer the rover?](2026-09-07-m0-semantic-world-state.md) | [R-WS-10](../requirements/world-state.md#r-ws-10) to `failing`; [R-WS-12](../requirements/world-state.md#r-ws-12), [R-WS-13](../requirements/world-state.md#r-ws-13) opened with measured detail | [R-WS-1](../requirements/world-state.md#r-ws-1) through [R-WS-9](../requirements/world-state.md#r-ws-9), [R-NAV-4](../requirements/navigation.md#r-nav-4) |

Measurements made before this log existed are in Git history, in the component
READMEs that cite them, and in the bench scripts that produced them. The ledger
starts here rather than being back-filled from memory, because an entry nobody
can reproduce is the one thing this format is meant to prevent.

## Writing an entry

Name the file `YYYY-MM-DD-slug.md`, dated by when the measurement was taken
rather than when it was written up. Then add a row to the ledger above.

An entry should answer, in this order:

1. **What question was being asked, and did it come out yes or no.** Put the
   answer in the first paragraph. An entry whose conclusion is only reachable by
   reading to the end will be read as inconclusive.
2. **What the measurement was.** The recording, the room, the run, the suite —
   enough that somebody could take it again. Name the fixtures and recordings
   that were used, since those are what make it repeatable.
3. **What the numbers were**, including the ones that were unhelpful. Report
   every attempt: successes, unresolved outcomes, refusals and failures counted
   separately, with the full attempt count. Dropping the failures after seeing
   them is how a measurement becomes a claim.
4. **Which requirements moved.** By identifier, with the new state.
5. **What has to happen next**, in order, if anything.

## Rules

**An entry is never edited to agree with later findings.** If a later
measurement contradicts it, write the later entry and let both stand. If an
entry was simply wrong about its own data — a miscount, a mislabelled column —
add a correction note at the top saying what was wrong and pointing at the entry
that supersedes it, and leave the original text alone underneath.

**Say plainly when a milestone does not pass.** A partial pass is a fail with
detail. The 2026-09-07 entry is the model: five of six criteria held, the sixth
failed outright, and the entry's own summary leads with the failure rather than
the five.

**Raw evidence outlives the summary.** Keep enough identifiers, poses, versions
and parameters to rerun the decision offline. Runtime frames and databases stay
out of Git — see [R-PLAT-11](../requirements/platform.md#r-plat-11) — so an entry
references them by identifier and names the recording they came from.

**Separate what was observed from what was predicted.** Replay proves logic.
It does not prove camera geometry, wheel behaviour, USB reliability or stopping
distance. Where a simulation and the rover disagree, the rover is right, and the
entry should say which of the two produced each number.
