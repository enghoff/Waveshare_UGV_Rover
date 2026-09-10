# The M0 acceptance drive

What to set up, what to type, and — the part that matters — **what has to be
written down before the rover moves.** [M0](../plans/autonomous-curiosity.md#milestone-m0-semantic-state-is-safe-enough-to-influence-goal-selection)
now has two gates: **M0a** for bounded hypothesis inspection (R-AUT-12), and
**M0b** for actions relying on persistent identity (R-WS-13). Both remain open.
The [revision](../decisions/m0-hypothesis-inspection.md) changes acceptance, not
runtime permission. Each gate asks for tolerances declared in advance and trials
the remedies have not seen. A
drive taken without this page cannot certify anything, however good its numbers
turn out to be, and that is why [the drive of
2026-09-08](../progress/2026-09-08-acceptance-drive-two.md) improved on its
predecessor and still moved no criterion.

## Freeze this before driving

Copy this block into the run's manifest file and fill in the blanks with the
owner present. Nothing here may be changed after the recording starts.

### Trial authority and questions

Record the gate under test, deployed commit, resolver and all thresholds; whether
this is owner-driven data collection or supervised M0a execution; physical area,
supervisor and stop control; the shared-prerequisite evidence and physical M3
stop/failure evidence required for execution. Owner-driven collection does not
certify autonomous dispatch or budget enforcement. This document is not a command
to enable movement.

For each inspection, freeze:

| field | value to supply before the trial |
|---|---|
| case | independent physical target/region ID; source observation references and store/map generation |
| question | the uncertain identity/location claim, alternatives and what would answer it |
| reference | owner-annotated physical truth, retained images and independent distance/position measurements; unavailable to the policy |
| viewpoint | observation pose and how map/route/physical-area checks validate it independently of the hypothesis |
| effort limits | maximum cumulative travel in metres, duration in seconds and attempts per case, plus run totals; non-empty, finite, enforced limits |
| outcome rule | evidence for supported/contradicted; when visibility or detectability requires unresolved |
| coverage | named useful cases and the task tolerances they must meet |

In M0a, collect at least 20 attempts over at least three fresh supervised runs,
covering at least ten distinct physical target/region cases. Include at least
three false/absent-target cases and three occluded/insufficient-view cases; label
their physical truth independently. Repeated attempts count as effort, never as
new target coverage; re-answering an already resolved physical question is not
another success. Budgets follow the physical route and task, not the entity ID.
Changing an ID or regenerating a goal must not replenish them.

Before these execution trials, demonstrate the incorrect-association reproduction,
refusals, stale-state handling, budget exhaustion and prevention of identity-dependent
follow-on actions in replay. Complete shared geometry/capture checks and supervised
physical stop/failure checks first. Only this frozen supervised protocol may run
before M0a passes; the wider M3 autonomy sessions remain a separate milestone.

### Resolver baseline to verify before the run

These are the last recorded baseline values, not authority over current source or
configuration. Read back the deployed values and freeze what will actually be
tested. The revised gates do not select or deploy an experimental resolver.

| what | value | chosen on | owed |
|---|---|---|---|
| two crops are not the same thing below | 0.55 | room recordings before 2026-09-07 | — |
| a match whose score collapses when masked is refused at | 0.20 | the drive of 2026-09-07 | this drive |
| another thing may lead the one being joined by at most | 0.15 | the drive of 2026-09-08 | this drive |
| an exemplar is learnt only from a match at or above | 0.70 | this rover's own measurements | this drive |
| a crossing needs a baseline of at least | 0.4 m | — | — |
| a crossing needs a parallax of at least | 12 degrees | — | — |

**Three of those were chosen after seeing the faults they catch.** Earlier
recordings are development evidence for them. Collect fresh acceptance recordings
after all policy and threshold choices are frozen; tuning on a recording makes
that recording development data and requires a new acceptance run.

### The operating envelope

- gimbal pan **-20 to +20 degrees**, tilt **0 or +20**, every placement finishing
  from the ascending direction. Outside it a look keeps its picture and records
  no direction at all.
- the depth camera covers the middle of the gimbal camera's view; a region
  outside it is reported as never-ranged rather than silently unranged.
- **object distance 0.5 to 2.5 m.** Declared on 2026-09-08 from what this rover
  actually does: across two drives that day, 61% and 71% of its looks at placed
  things fell inside that band, with a median of about 2 m and a 90th percentile
  of 3 to 4 m. It is the band the small targets can be detected and tape-measured
  in, and it covers roughly two thirds of real work -- so it is a narrowing, not
  an abstention. A look beyond 2.5 m is recorded and flagged as outside the
  certified distance rather than refused, because the depth camera's blind edge
  already refuses half of them and a second refusal on top would hollow the world
  out. Widening it needs tape measurements between objects that detect at range,
  which the tissue box and the bucket do not.

### The tolerances, declared now

| question | passes if |
|---|---|
| do ranges land on the object they claim? | at least 70% of expected in-coverage target ranges are usable, correctly attributed and within 0.5 m of an independent physical reference; missing/ambiguous ranges remain in this denominator |
| is a placement where the object is? | every measured separation between named targets agrees within 0.30 m, and independently referenced target positions relative to the rover meet task-derived position/bearing tolerances frozen in the manifest |
| M0a: is inspection useful? | at least half of all attempts correctly answer the frozen question, in >=20 attempts across >=3 runs and >=10 distinct target/region cases; refusals and unresolved outcomes are not successes |
| M0a: is uncertainty contained? | **zero** unsupported verification conclusions, promotions into identity-dependent actions, or action/budget/safety boundary violations; every attempt terminates and is recorded |
| M0b: is a trusted association wrong? | **zero** known-incorrect high-confidence merges and associations eligible for identity-dependent actions in >=50 distinct reviewed association decisions, covering every eligible confidence band |
| M0b: is trusted identity useful? | each of at least three independently named physical targets supports its predeclared identity-dependent inspection task within the frozen tolerances; record eligible coverage, splits and abstentions for all targets |

The previous 70% range check compared depth with fitted parallax. The numerical
tolerance is retained as the initial task limit, but its denominator and reference
are now explicit and independent; previous percentages are not passes under this
test. Target separations alone cannot reveal a common map offset, so record target
positions relative to independently surveyed rover standing places as well.
The former 25-entity/15-ranged minimum is replaced by physical-target and inspection
outcomes: splitting one object into many records must not improve coverage.
Freeze any task-specific tighter tolerances before collecting data.

### The targets

At least three objects, put out by the owner, **each separation measured with a
tape and written down before driving**:

| target | what it is | separations |
|---|---|---|
| 1 | | to 2: ___ m, to 3: ___ m |
| 2 | | to 3: ___ m |
| 3 | | |

One of them at floor level, one above the horizontal, and one deliberately placed
so it is seen at the edge of the frame — that last is what exercises the
never-ranged reporting.

These are the shared geometry and M0b targets. M0a's ten target/region cases also
include false and unobservable hypotheses. For the edge target, predeclare which
views should have depth and which should abstain; obtain supported views for its
geometry/identity task as well. Keep raw depth, full frames, intrinsics, capture
poses and timestamps so a claimed absence can be checked rather than inferred
from an empty detection list.

## The owner-driven capture procedure

Use this procedure for shared geometry and M0b data collection. M0a additionally
needs the supervised execution protocol above and a complete record of actual
dispatches and outcomes; manually driving these views is not equivalent.

1. **Clear the semantic store, keep the map.** The sample has to be free of old
   association decisions; the map is the frame everything is measured in, and
   starting localised is what the envelope was declared against.
2. **Check the map came back settled, not merely loaded.** A restore that
   anchored wrong puts a heading error into every bearing in the sample, and it
   is the one failure that wastes the whole run. `map_settled`, not `map_kept` —
   see [R-NAV-2](../requirements/navigation.md#r-nav-2).
3. Drive for at least fifteen minutes, passing each target from **three or more
   standing places at least a metre apart**. Parallax is what makes the ground
   truth; a target seen only from one spot cannot check anything.
4. **Restart navigation once, mid-drive**, and keep looking at things for a
   minute afterwards. `ssh orin '~/ugv/ros_nav/restart.sh --supervisor'`.
5. Drive past something at the edge of the frame on purpose.
6. Stop, and say so, so the recording is archived at a known point.

## Afterwards

Archive before anything restarts — the store is live and a clear would take it:

```bash
ssh orin 'python3 - <<PY
import sqlite3
src = sqlite3.connect("file:/home/jetson/.ugv/world/world.db?mode=ro", uri=True)
dst = sqlite3.connect("/home/jetson/.ugv/archive/world-<date>-acceptance.db")
src.backup(dst); dst.close()
PY'
ssh orin 'mkdir -p ~/.ugv/archive/frames-<date>-acceptance && cp ~/.ugv/world/frames/<date>-*.jpg ~/.ugv/world/frames/<date>-*.depth.gz ~/.ugv/archive/frames-<date>-acceptance/'
```

Copy both down into `captures/`, which is gitignored, and keep a MANIFEST.txt
beside them holding the frozen block above.

Then score it against the independent range/position references, and a contact-sheet
review giving **every** thing a written verdict rather than only the ones that
catch the eye. Two mistakes on 2026-09-07 and 2026-09-08 were both made reading
crops too small; five to a row at 200 pixels, and zoom anything doubtful with its
full frame behind it.

Parallax agreement remains a diagnostic; acceptance uses the independent references
above. For M0a, review every attempt as supported, contradicted or unresolved and
check its conclusion against the physical truth. A missed detection under occlusion
must remain unresolved. Report correct answers, false conclusions, distinct target
coverage, refusals, unresolved attempts, duplicates, false merges, and total and
unsuccessful travel/time, with the frozen budgets beside the actual effort.
Review the actual action class and follow-on requests, not just the goal's label.

Publish separate M0a and M0b results against every shared and gate-specific criterion
in the plan, including the M0b range-assisted/bearing-only replay comparison. A
failed, incomplete or unmeasured criterion stays open. Passing M0a does not settle
R-WS-13 or pass M3, and neither gate can be passed by the September development
recordings or the documentation revision.
