# Development plan: curiosity-driven autonomy

Status: Phase 0 (P0) closed on 2026-10-02 ([the closure](../progress/2026-10-02-p0-closed.md)).
Phases 1 and 2 (P1, P2) passed their milestones on 2026-09-08. Phase 3 (P3) is under way: the executive and the
daemon-enforced permission it works under are deployed, and on 2026-10-02 they moved
the rover in supervised stop trials and in M0a's inspection runs. Nineteen M3
sessions have run; [the third](../progress/2026-10-06-m3-session-3.md)
showed R-AUT-13's ending, and on 2026-10-06 a repeated request, a hung executive
and a lost connection were [tried on the rover](../progress/2026-10-06-repeat-hang-and-drop-on-the-rover.md).
M3's criteria 6, 7, 9, 10 and 12 are met. Every condition criterion 4 lists has
now been driven, which replaced a count of twenty on 2026-10-07
([the decision](../decisions/trials-are-sized-by-what-they-show.md)), the last
of them and the re-drives on 2026-10-10 ([sessions 14 to 19](../progress/2026-10-10-m3-sessions-14-to-19.md)).
What remains is one run on the fix for the fault session 17 found. M4 was [revised on 2026-10-09](../decisions/m4-measures-where-things-are.md) to what the hardware and vision models measure, where things are against tape, judged against a re-look; it needs a new taped set and two small pieces of code before its attempts. The same day every remaining gate was tied to what it is for ([agreed](../decisions/every-gate-says-what-it-is-for.md)): M5 starts from false alarms on unchanged scenes, M6 and M7 wait until there is a skill to learn, M10 can follow M4. Seventeen development attempts then found a look from where the rover stood doing as well as the chosen viewpoint for things in reach, so M4 now asks only of things out of reach and the rest are turned to and looked at ([agreed](../decisions/m4-asks-of-things-out-of-reach.md)). Thirteen development attempts out of reach then found the chosen viewpoint improving its thing five times against the re-look's once ([2026-10-09](../progress/2026-10-09-m4-out-of-reach.md)). Later phases remain proposed. This is the
implementation and acceptance plan for
[the architecture it implements](autonomous-curiosity-design.md).

The plan is gated by capability dependencies, not a requirement to finish every
phase before starting the next. M1 recording and M2 shadow decisions proceeded
alongside the existing P0 work, with no action authority, and both have now
passed. The offline control-boundary tests M3 asks for are written and passing,
which is what allowed P3 to be built alongside P0. **M3 semantic movement required
M0a for hypothesis inspection, which passed on 2026-10-02. An action relying on
identity needs its own case decided first** ([2026-10-01](../decisions/identity-is-judged-action-by-action.md)). Supervised stop/failure trials precede bounded inspection acceptance
trials and autonomy sessions; geometric-only goals remain subject to the same
control and physical safety gates. Later capabilities require
recorded evidence for the primitives, semantics and execution substrate they use;
code existing is not acceptance. Physical criteria must be observed on the rover.

P0's record runs from the
[`M0 baseline`](../progress/2026-09-07-m0-semantic-world-state.md) to
[the closure](../progress/2026-10-02-p0-closed.md), which lists what it leaves for later. The
[September 10 decision](../decisions/m0-hypothesis-inspection.md) relaxed the identity
prerequisite only for bounded verification of uncertain hypotheses, and that is still
the only semantic movement M0 permits.

The existing repository rule still applies: reproduce failures before fixing them,
validate on recordings before changing the running rover, and treat hardware as
authoritative where simulation disagrees.

## Definition of done

The programme is successful when the rover can, in a physically pre-cleared flat
environment:

- autonomously choose between geometric exploration, semantic inspection, revisits
  and bounded search based on explicit expected utility;
- gather observations that measurably improve its world knowledge rather than
  merely accumulating images;
- retain temporal changes and uncertainty with provenance;
- record every autonomous decision and outcome in replayable episodic memory;
- acquire versioned high-level skills by abstracting repeated successful behaviour;
- promote a new skill only after deterministic validation and measured trials;
- demonstrate that learned knowledge/skills improve later performance on held-out
  tasks or environments;
- remain interruptible and unable to bypass the existing daemon/Nav2 safety
  boundary.

Useful autonomy through M5 must also pass the multi-day benchmark below. Full
procedural learning additionally requires M6 and M7. A practice curriculum (M8),
reflection and predictive models are extensions whose value must be measured, not
prerequisites for recording or the first useful inspection loop.

The target is not online self-modification of arbitrary code or continual neural
fine-tuning on the physical rover.

## Measurement rules

Every milestone uses the following rules unless it explicitly says otherwise.

### Establish the baseline first

A new metric is measured on the current implementation before a change claims to
improve it. If no baseline exists, create a fixed replay or annotated scenario set
and check it into the repository where practical; runtime frames/databases remain
outside Git.

### Preserve raw evidence

Record enough input/output detail to rerun the decision offline. A score or model
summary without the observation IDs, pose, skill version and input parameters that
produced it is not a useful test artefact.

### Prefer precision over forced completion

For persistent identity, changed-object detection and semantic claims, an unresolved
answer is preferable to a confident false merge or invented fact. Acceptance tests
must therefore report false-positive and unresolved counts separately rather than
only an aggregate success score.

### Fixed test sets stay fixed during a comparison

Do not tune a prompt, threshold or scoring weight against a test case and then count
that same case as independent evidence. Split captured scenarios into development
and acceptance sets when model/prompt tuning is involved.

### Hardware gates are explicit

Simulation/replay proves logic. It does not prove camera geometry, wheel behaviour,
USB reliability, collision sensing or stopping distance. Physical acceptance items
must be observed on the rover.

### Count every attempt and separate termination from success

Predeclare applicable cases, success predicates, minimum improvement and trial
counts before acceptance. Report successes, unresolved outcomes, precondition
refusals, failures and aborts separately, with the full attempt count. Do not drop
failed inspections after seeing their outcomes or count exhausted viewpoints as a
resolved question. Report uncertainty in estimated rates and group correlated
observations by object/run.

### Size a trial set by what it has to show

A count is chosen from the question the trials answer, and the milestone says
which question that is ([2026-10-07](../decisions/trials-are-sized-by-what-they-show.md)).
A rate needs enough trials for its interval to separate the threshold from what
it rules out: 18 successes in 20 show, at 95% confidence, only that the true rate
is above about 70%. A search for faults needs conditions that differ, because a
trial that repeats a condition already passed rarely finds what the first missed.
A trial set stops once its outcome is settled either way. A fix is seen working
in the next trial that reaches it. A round number is not a reason for a count.

### Replay has a coverage boundary

Stored outcomes may be reused only for matching action parameters and relevant
context. An unvisited viewpoint or untried action has an unknown outcome, not the
outcome of the recorded alternative. Decision replay can show a changed choice;
performance comparisons need matching recorded coverage, a simulator validated
against hardware, or new supervised trials. Report unsupported alternatives and
simulation predictions separately from observed physical results.

## Metrics to collect from the beginning

The exact dashboard can evolve, but these quantities should be recorded from the
first autonomy episode so later learning can be evaluated retrospectively.

### Safety and control

- autonomous distance travelled;
- autonomous time moving;
- manual-stop requests, acknowledgement latency, and measured time/distance to
  physical standstill;
- permission expiry, executive failure and manual-takeover stops;
- navigation refusals/failures by reason;
- safety-vetoed candidate goals;
- unexpected physical contacts;
- autonomy restarts after stop;
- battery at start/end and low-battery aborts.

### World knowledge

- observations and resolved entities;
- pending/unresolved observations;
- independently reviewed false merges and duplicate entities;
- number of claims by confidence band;
- claim accuracy on annotated acceptance cases;
- stale claims revisited;
- scene changes detected/missed/false-alarmed;
- spatial/semantic coverage by area.

### Curiosity

- candidate goals generated by type;
- selected goal and all score terms;
- predicted information gain;
- realised information gain;
- information gain per metre, minute and Wh when available;
- goals that produced no useful evidence;
- repeated attraction to already-resolved novelty.

### Skills

- skill/version used;
- precondition failures;
- success/failure reason;
- duration and travel;
- recovery attempts;
- success rate by context;
- candidate-to-verified promotion rate;
- verified-skill regressions.

### Model usage

- provider/model/version;
- purpose of call;
- latency;
- input/output usage and cost when the API reports it;
- schema/parse failures;
- invalid entity/episode references;
- acceptance result for model-selected actions.

## Phase 0 -- close the semantic-world-state prerequisite

### Purpose

Do not let semantic state choose motion while persistent identity/range behaviour is
still unvalidated. This phase is mostly existing work from
`docs/plans/semantic-world-state.md`, but it is an explicit dependency of autonomy.

### Where it ended

**The owner closed P0 on 2026-10-02**, against criteria that measure the hardware
and hold the rover's claims to what was measured, rather than holding it to
accuracy bars ([the decision](../decisions/p0-measures-the-hardware.md)).
[The closure](../progress/2026-10-02-p0-closed.md) says what met each condition and
what P0 leaves for later: the inspection check answers rarely, the depth camera
drops off USB, small objects are never found, and bearing claims on moving looks
are still smaller than their measured error. R-AUT-12, R-WS-11, R-WS-16 and
R-SAFE-10 are settled. [R-WS-10](../requirements/world-state.md#r-ws-10) was retired on
2026-10-03 in favour of measured performance and honest per-look claims.

The six taped targets of 2026-10-01 and their wall frame are in
`captures/m0-2026-10-01/MANIFEST.txt`. On 2026-10-03 the owner reported the
temporary ones gone: the black cabinet, the painting above it and the landscape
painting over the dining table are still where they were taped. Three more fixed
paintings were chosen from the rover's own looks that day and taped by the owner;
the six are in `captures/2026-10-03-targets/TARGETS.txt`, and the drive past them is
[2026-10-03's entry](../progress/2026-10-03-moving-looks-against-the-tape.md). The board method in the [P0 camera geometry runbook](../runbooks/p0-gimbal-calibration.md)
checks the OAK's offset on the rail. A further calibration, such as the third tilt
the closure suggests, follows the protocol below.

### Bounded calibration protocol

1. **Declare the task and limits before fitting.** Start with inspecting large,
   well-separated static objects. Record the tested pan/tilt range, distances,
   minimum angular object size and clearance from neighbouring surfaces, approach
   directions, settling rule and camera mode. Choose numerical alignment and
   task-success tolerances before acceptance, from whether the sampled patch stays
   on the intended object and whether identity remains correct. Do not select a
   universal degree target merely because it is already configured.
2. **Establish an independent reference and its uncertainty.** The current lens
   was fitted using commanded gimbal angles, and the OAK bench reads commanded
   angles too. Use a measured calibration target with independently calibrated
   optics/pose estimation, or a validated external angle reference. Do not use the
   same unvalidated lens/servo fit as ground truth for itself. First confirm that
   the reference can resolve the task-relevant error; otherwise report the test as
   inconclusive rather than blaming the hardware. Require enough reference coverage
   to constrain a planar pose, compare the camera's stored lens model with the model
   used at runtime, and repeat cross-camera geometry at a different target placement.
3. **Measure repeatability before correction.** With the chassis stationary and
   the scene fixed, sample pan -20, -10, 0, +10 and +20 degrees at tilt 0, if these
   positions are mechanically available. Approach each from both directions using
   overshoot beyond the sampled endpoints. Use three paired repetitions per
   position, alternating order, and record command history, capture/settle times,
   measured orientation and reference uncertainty. Include repeated same-direction
   approaches to distinguish measurement variation from direction-dependent error.
   Wider pan or nonzero tilt remain outside this initial envelope until tested.
4. **Try only simple, justified remedies.** Evaluate a versioned bias/angle mapping
   and, if practical, a consistent final approach direction with a settling rule.
   A consistent approach is a candidate policy, not an assumed cure. Permit at
   most two correction candidates in this initial campaign, using development
   data; do not fit a single gain unless the measurements support one.
5. **Validate independently.** Freeze the selected candidate and test at held-out
   intermediate angles such as -15, -5, +5 and +15 degrees, repeating both approach
   directions, plus endpoint checks in a separate session. Report bias, residual
   spread/tails, reference uncertainty, abstentions and task failures by condition.
   Propagate residual uncertainty to bearings and depth-region selection. A wider
   uncertainty must result in unresolved/rejected ambiguous matches, not a wider
   permission to attach them to the nearest entity.
6. **Validate the complete path.** Within the supported envelope, validate the
   gimbal/OAK transform and offsets, then test depth-to-object alignment using known
   foreground/background targets at the intended working distances. Check the
   deployed capture path uses the validated approach and settling state; a bench
   result is insufficient if face tracking, voice or scripts can leave it elsewhere.
   Outside that state, reject semantic movement eligibility and unsafe depth
   attribution. Preserve the existing recordings as evidence; they cannot recover
   unrecorded approach history or depth patches never captured.

The initial budget is one baseline campaign, at most two simple correction
candidates, and one independent acceptance campaign. Freeze trial counts, numeric
task tolerances and the minimum worthwhile improvement in the run manifest before
collecting acceptance data. If a comparison's improvement does not exceed the
reference/repeatability uncertainty or does not change task eligibility, stop that
line of tuning. A failed acceptance set is not reused as held-out evidence after
further fitting. Further experiments require a named unresolved question and a new
bounded protocol, rather than repeating sweeps until a favourable fit appears.

### Exit decisions

- **Characterise and represent honestly:** held-out geometry and range attribution
  are measured against independent references and published by condition, the
  rover's stated uncertainty covers the measured error, and runtime checks abstain
  where the rover cannot see. There is no accuracy bar
  ([2026-10-02](../decisions/p0-measures-the-hardware.md)).
- **Narrow and retest:** adequate performance is limited to fewer angles, larger
  objects or better-separated surfaces. Declare that narrower scope before fresh
  acceptance; report excluded and unresolved cases. Rejecting every useful task
  cannot count as a pass.
- **Stop at a physical or measurement limit:** the bounded campaign cannot support
  the useful task. State the remaining error, its evidence and what specific
  reference/hardware change would enable a meaningful next test. M0 remains open;
  read-only M1/M2 continue. No hardware purchase follows automatically.

### Verification

Run the existing `world_state/selftest.py` suite and relevant replay/bench tools.
The acceptance recording must be preserved outside Git with enough metadata to be
replayed by path/manifest.

### Milestone M0: semantic state is safe enough to influence goal selection

**Status: passed on 2026-10-02, and P0 is closed** ([the closure](../progress/2026-10-02-p0-closed.md)).
M0a's 27 supervised attempts gave three answers, all right, and none wrong, inside
every limit, against criteria that measure the hardware rather than hold it to bars
([the decision](../decisions/p0-measures-the-hardware.md)). M0b was retired on
2026-10-01, when identity became
[a case decided for each action](../decisions/identity-is-judged-action-by-action.md), so M0
meant M0a and the shared prerequisites. What P0 leaves for later is listed in the
closure: the check answers rarely, the depth camera drops off USB, and small objects
are not found. The
[agreed revision](../decisions/m0-hypothesis-inspection.md) permits investigation
before persistent identity is trusted. Earlier reports used the original,
stricter movement-eligibility contract and are not retrospectively rescored as passes.

#### Shared prerequisites

M0a requires these, and an identity-dependent action's case may rely on them:

1. the normal world-state offline suite passes, together with the affected
   autonomy and daemon checks;
2. camera-to-rover geometry and residual error are measured on fresh hardware
   trials against independent physical references, published by condition (distance,
   height, in front of or behind other things, still or turning), and the
   uncertainty the rover states for directions, ranges, heights and placements covers
   that measured error. There is no accuracy bar: worse hardware means wider stated
   uncertainty, not a failed prerequisite ([2026-10-02](../decisions/p0-measures-the-hardware.md)). Agreement
   between depth and fitted parallax alone does not establish accuracy;
3. where usable OAK ranges are present, and whether they belong to the intended
   objects, is measured against annotated targets and published with the conditions
   that defeat it. Unsupported or ambiguous patches are refused and counted
   separately;
4. map clear/map-session checks keep old coordinates from being treated as current
   placement. Unconfirmed poses cannot give observations usable directions, nor
   can later confirmation restore them retrospectively. Replay and hardware
   restart/refit checks retain the images and withhold affected bearings (R-WS-16);
5. no longer a condition: bare floor and background patches
   ([R-WS-12](../requirements/world-state.md#r-ws-12)) were
   [taken out of P0](../decisions/bare-patches-are-not-a-p0-gate.md) on
   2026-10-01, because no available rule keeps them all out and an attempt on one
   is already bounded by M0a. Deliberate geometric coverage remains a distinct
   goal type;
6. the deployed capture, goal and dispatch paths enforce the accepted envelope.
   A hypothesis never certifies free space, route safety or movement authority.
   Existing daemon/Nav2, stop, permission, battery and physical-area requirements
   remain in force (R-SAFE-3, R-SAFE-4 and R-SAFE-12);
7. a report records false merges, duplicates, unresolved observations, physical
   target coverage and every attempted inspection, including refusals and failures.

#### M0a: bounded inspection of uncertain hypotheses

This gate permits only an explicitly recorded attempt to test a hypothesis, as
required by R-AUT-12. Its identity or location may be incorrect. Pass when:

1. each request records source observations, the uncertain claim and alternatives,
   a verification question, a separately validated observation viewpoint, and
   finite travel/time/attempt limits. The destination is not assumed to be free
   because an entity was placed there;
2. replay shows a reproduced incorrect association leading to bounded verification
   or refusal, with no identity-dependent follow-on action. It also covers an
   absent target, occlusion/insufficient evidence, stale pose/map, repeated goal
   generation and budget exhaustion. Dispatch enforces the action class and limits;
   changing an entity ID cannot restart the same inspection budget;
3. on at least 20 inspection attempts across at least three fresh supervised runs,
   with at least ten distinct physical target/region cases represented, independent
   review reports how often attempts answered their predeclared question correctly,
   abstained and were wrong. There is no usefulness floor
   ([2026-10-02](../decisions/p0-measures-the-hardware.md)); a confident wrong answer is criterion 5's. Include
   at least three false/absent-target cases and three occluded/insufficient-view
   cases. Independent review checks answers against retained images and physical
   references. Refusals, unresolved cases and repeated photos are not successful
   verification; a justified contradiction can be. A missing detection alone does
   not establish absence. Re-answering an already resolved physical question is
   not another success. These are entry minima, not a reliability guarantee;
4. every attempt ends within its frozen effort limits and records supported,
   contradicted or unresolved with evidence. Report successful target coverage,
   false conclusions, unresolved/refused attempts, and total and unsuccessful
   travel/time; repeated attempts remain in the denominator. No entity-count
   threshold or retained-pair score can substitute for useful physical outcomes;
5. there are zero observed promotions of an unvalidated identity into an
   identity-dependent action, zero unsupported claims of verification, and zero
   violations of the action, budget or safety boundaries in those trials.

Before M0a hardware acceptance, shared prerequisites, replay and M3 supervised
physical stop/failure checks must pass. Only the frozen supervised acceptance
protocol may then exercise this path; passing M0a does not pass M3's autonomous
sessions or waive their remaining criteria. Read-only development remains allowed.

#### M0b: retired on 2026-10-01

M0b was one bar for every action whose correctness depends on knowing which
object is which. The owner [replaced it](../decisions/identity-is-judged-action-by-action.md) with an evaluation of
each such action when it is proposed, weighed against what a wrong identity would
cost that action; [R-WS-13](../requirements/world-state.md#r-ws-13) says what an
evaluation owes. No autonomous run may take an identity-dependent action whose
case is undecided. Today that holds because an autonomous run can only drive to a
point, look and stop, and an inspection's outcome leads to no follow-on action.

Passing M0a permits the M3 inspection path only under its own control gates; it
does not unlock any identity-dependent action.

## Phase 1 -- episodic memory and read-only autonomy telemetry

### Purpose

Create the evidence trail before creating an executive that can act.

### Milestone M1 passed on 2026-09-08

Every criterion is met, across three entries of that date: the record and its
names, [the clear that emptied the world under it](../progress/2026-09-08-the-clear-that-proved-it.md),
and [the driven run](../progress/2026-09-08-the-driven-run.md) that supplied the
navigation half of criterion 4. What the milestone does *not* give is listed
under "what is still ahead" below. Since M3, the executive records its own
decisions, so the first gap the list once named, that nothing decided anything,
is closed.

### The component, which exists

`autonomy/` records episodes and has no authority over anything. What it holds and
how it works is [its own README](../../autonomy/README.md), and the reference
scheme underneath it is settled in
[a decision record](../decisions/episode-references-survive-the-world-state.md).
Its runtime data lives outside the deploy tree at `~/.ugv/autonomy/`. It is
started on 2026-09-08 and read-only in every sense: nothing on the rover writes
to it yet, because the executive that will is Phase 2 work.

The durable evidence contract this phase asked for is decided and implemented. A
world reference carries the generation of the store that minted it, so a name
from a cleared world fails closed instead of resolving to a stranger; a decision
keeps a copy of the world it was made from; evidence is copied out of the world
state and named by its own bytes. `world_state` mints and reports that generation
as of the same date. What remains open is written into
[the autonomy requirements](../requirements/autonomy.md).

### What is still ahead

- **The recorder is not a service.** It is run by hand for as long as somebody
  wants a recording. Nothing starts it at boot, so a rover left alone records
  nothing, and retention is not run on a schedule either.
- **Nothing tells the record about a merge or a split.** A retained reference
  cannot be broken or redirected by one, because replay never resolves against
  the live store; but "what became of that thing" stays unanswered until
  something calls `store.alias`.
- **A look that found nothing leaves no episode.** The recorder keys an episode
  to an inspection's observations, and an inspection that found no region writes
  no observation — so "the rover looked and saw nothing", which is itself worth
  knowing, is invisible to it. Closing that needs the daemon to expose its
  inspection log, which it does not today.
- **Only thirty-two of the driving loop's sentences are retained**, so a
  recorder away for longer than that loses some. It counts and reports what it
  lost, which is the honest floor rather than a fix.

### Milestone M1: every future autonomous action can be reconstructed

Pass when all are true:

1. schema/store/replay self-tests pass from an empty database and through at least
   one migration;
2. a mock-rover scenario produces an episode whose selected action, parameters and
   result can be reconstructed solely from stored records;
3. replaying a recorded episode does **not** issue hardware calls;
4. a 30-minute rover shadow run records normal navigation/world-state events while
   the autonomy component has no movement-capable API path;
5. deleting/restarting the autonomy process does not alter ROS map or world-state
   data;
6. database growth and retention policy are documented;
7. map clear, entity merge, process restart and local-ID reuse cannot break or
   redirect retained episode references; replay uses the original snapshot/evidence;
8. explicit deletion and retention expiry are reported honestly as missing evidence,
   never silently replaced by newer records with the same local ID.

## Phase 2 -- candidate goals and curiosity scoring in shadow mode

### Purpose

Make "curiosity" explicit and measurable before allowing it to move the rover.

### Milestone M2 passed on 2026-09-08

Every criterion is met. What was built is described where it runs -- in
[the component's own README](../../autonomy/README.md) -- and what it is required
to do is in [the autonomy requirements](../requirements/autonomy.md),
[R-AUT-8](../requirements/autonomy.md#r-aut-8) to
[R-AUT-10](../requirements/autonomy.md#r-aut-10). The measurement is
[the shadow decisions entry](../progress/2026-09-08-shadow-decisions.md).

**Two of the six goal types exist**, which is what this milestone asked for:
`explore_frontier` and `improve_geometry`. The scoring is arithmetic over a
snapshot with every term recorded, refusals are gates and vetoes that run before
the score rather than terms inside it, and the acceptance set is 49 curated
rooms with the expected outcome written beside each.

### What is still ahead

- **Four goal types are missing**, and they are the semantic ones:
  `inspect_uncertain_entity` (gaps about what a thing is, which wait for
  attribute claims: *Attribute claims, later*, under Phase 4),
  `revisit_stale_entity`, `investigate_change` and `search_for_missing_entity`
  (the last three need M5 temporal and visibility semantics). Each needs shadow
  acceptance of its own, and the fixed acceptance set must be extended before
  any of them is enabled for movement. Skill-practice goals wait until a skill
  metric exists.
- **A thing seen once and never placed is never proposed**, because one bearing
  gives no position to plan a viewpoint around. The pool of unplaced sightings
  is invisible to the choosing, and closing that needs the observation history
  rather than the entity listing.
- **The purpose term is 1.0 everywhere**, because the owner has not declared
  one. It is configuration and not code; until it is set, the rover weighs
  exploring and inspecting equally and says so.
- **Whether a chosen goal paid off is recorded only by the rover's own
  account.** Since 2026-10-08 a geometry goal records the gain it predicted and
  its target's stated uncertainty before and after. Whether it came closer to
  where the thing really is needs the tape, and is M4's scoring script
  (*What is ahead before the acceptance attempts*, under Phase 4). It is also
  what any learned ranking later rests on.

### Milestone M2: the rover can explain what it would investigate next

**Criterion 6 asked for an hour of shadow running until 2026-09-08, when the
owner removed the clock from it.** The reason it is worth recording rather than
quietly editing: an hour was a stand-in for variety, and what the criterion is
actually about is that the choosing works against the real world state and the
real map rather than against drawn rooms. A run that shows that in twenty
minutes shows it; one that idles for an hour in front of an unchanging scene
does not show it better. The run reports its own length, so a reader can judge
what it covered.

Pass when all are true:

1. candidate generation and scoring are deterministic under replay;
2. every selected candidate records a full score decomposition;
3. hard-veto tests show that increasing information-gain weights cannot override a
   veto;
4. at least 40 curated scenarios cover the enabled goal types, ties, no-action cases
   despite reachable candidates, cooldowns, switching, low battery and unavailable
   hardware; extend the fixed acceptance set before enabling later goal types;
5. expected ordering is correct in at least 95% of the curated acceptance cases;
   disagreements are reviewed and the expected set is changed only with a written
   reason, not merely to make the metric pass;
6. a rover shadow run emits candidate decisions against the real world state and
   the real map while making **zero autonomy movement calls**, and the entry
   reporting it says how long it ran and what it saw;
7. the console or log can answer in one concise record: what it wanted to do, why,
   estimated cost, and why higher-scoring-but-vetoed options were refused.

## Phase 3 -- bounded autonomous execution using only existing operations

### Purpose

Close the first real loop without yet learning new skills or semantic viewpoint
strategies.

### What has landed, and where it is described

The executive and the permission it works under are built, deployed and checked
offline, and on 2026-10-02 both moved the rover under supervision:

- **The stop trials** ([the trials](../progress/2026-10-02-drive-carry-and-stops.md)).
  A stop request, the console's stop button, a manual takeover mid-leg, and a killed
  executive each ended the run and latched autonomy off, and the executive could not
  restart it. At 0.31 m/s every moving stop came to rest within the frozen 1.0 s and
  0.30 m, which fits inside the fence's 0.5 m stopping margin; the margin itself has
  not been re-derived from them.
- **Permission expiry** ([the first M0a runs](../progress/2026-10-02-first-m0a-runs.md)).
  With a 2 s permit and its program killed mid-leg, the daemon stopped the rover on
  its own: at rest 0.54 s and 0.23 m after expiry, with Nav2 still up.
- **M0a's runs** ([27 attempts](../progress/2026-10-02-m0a-runs-reach-twenty.md)). The
  executive itself chose and drove 27 bounded inspections over eight runs, each
  recorded in an episode, with every limit held.

What each of them is belongs to the
component that holds it rather than to this plan: the permission the daemon
issues, spends and takes back is in
[rover_daemon/README.md](../../rover_daemon/README.md) under *Moving by itself*,
and the loop that uses it — its states, its planning, and why the rover's own
`explore` is not one of its operations — is in
[autonomy/README.md](../../autonomy/README.md) under *Going and doing it*.

Three operations are admitted and no others: `drive_to`, `world_inspect` and
`stop`. `run_script` and `start_script` are not autonomy primitives and are
refused by name.

### What is still ahead

What is left of M3 is one run:

- **One run on the fix session 17 found** (criterion 4 and the list under
  it), with the owner present. Every condition has been driven. Session 17's
  first drive was refused because navigation's drift check held the lock every
  move takes (fixed in 4e3a447), so a run on that fix, starting in the bedroom
  and free to use the whole flat, that finds nothing new is still owed. The
  sessions so far, the third to eighth in the charger room
  ([1](../progress/2026-10-05-m3-session-1.md), 8.7 minutes;
  [2](../progress/2026-10-05-m3-session-2.md), 11 minutes, stopped at 5% battery;
  [3](../progress/2026-10-06-m3-session-3.md), 3.6 minutes, ended by going home;
  [4 and 5](../progress/2026-10-06-m3-sessions-4-and-5.md), 8.3 and 1.5 minutes,
  both stopped on the battery, 44 drives and none failed;
  [6](../progress/2026-10-06-m3-session-6.md), 10.4 minutes, its way home refused
  by the new margin;
  [7](../progress/2026-10-07-m3-session-7.md), 8.6 minutes, no look failed on
  the lock, stopped by a person after a near goal swung on the spot for 72 s;
  [8](../progress/2026-10-07-m3-session-8.md), about 9 minutes, no short goal
  over 4.6 s with [the swing fixed](../progress/2026-10-07-why-the-rover-turns-back-and-forth-on-the-spot.md),
  ended by going home past the margin;
  [9](../progress/2026-10-07-m3-session-9.md), about 3 minutes with the depth
  camera off, stopped when the rover could no longer pivot at 30%;
  [10](../progress/2026-10-07-m3-session-10.md), about 5 minutes, the owner in
  view only once, the rover snagged once on shoes below the lidar's plane;
  [11](../progress/2026-10-07-m3-session-11.md), about 5 minutes, the owner
  blocking it twice, when it swung on the spot; a near goal now waits for a
  blocked way to clear;
  [12](../progress/2026-10-07-m3-session-12.md), about 13 minutes, stuck on the
  rug under the dining table;
  [13](../progress/2026-10-07-m3-session-13.md), about 5 minutes, a near goal
  blind to a person until it had set off, fixed;
  [14 to 19](../progress/2026-10-10-m3-sessions-14-to-19.md) on 2026-10-10,
  furniture moved, the bedroom without a fence, and the charger room, the
  bedroom and the whole flat again on the turning and restore fixes);
  M0a's runs were its own protocol and do not count towards them. Open each
  with no action limit. A fenced condition gets a safe area around the cleared
  room, drawn 0.6 m beyond where the run starts. A fenced run ends by going home
  ([R-AUT-13](../requirements/autonomy.md#r-aut-13), settled by session 3); an
  unfenced one has the whole flat and ends on the battery, since runs carry no
  battery floor. Start each on a full charge and judge the battery at rest: a
  reading while driving sags by up to 30 points. A supervised session may run
  down to 5% at rest, the owner's standing floor since 2026-10-10 (10% from
  2026-10-07), and is then stopped and driven back to the charger. Session 9
  found the rover could not pivot at 30% (11.3 V at rest), and the percent
  reading is erratic; a floor in volts is a proposal for the owner, and until
  they agree the floor is 5%.
- **Contacts the lidar cannot see.** It scans one plane about 20 cm up, so
  shoes and rugs are invisible to it. In session 10 the rover snagged on a pair
  of shoes, and the owner reports it often snags on a rug. Criterion 5 counts
  both. Shoes are a matter of clearing the floor; a rug in the room stays a
  hazard until it is marked on the map as somewhere not to drive, or taken up
  for sessions.
- **Looks that improve nothing.** Of 171 geometry goals on record, 10 improved
  their thing when measured and 9 more within a minute
  ([the measurement](../progress/2026-10-06-looks-seldom-reach-their-thing.md)).
  The depth camera works; the look's evidence mostly does not reach the thing
  it was aimed at, which waits on the identity work (R-WS-13). Until then a
  run's geometry goals mostly spend battery, and a session's count of looks is
  not a count of work done.
- **Criteria 9 and 10 are met on the rover** (2026-10-06:
  [the trials](../progress/2026-10-06-repeat-hang-and-drop-on-the-rover.md) and
  [the console's stop](../progress/2026-10-06-the-console-stop-at-speed.md), at
  rest within 0.17 m at 0.41 m/s), and the safe area's margin is set from them
  (0.6 m). No drive has yet been refused on the new margin. A rover whose gyro
  has gone wrong is now refused a run ([R-SAFE-17](../requirements/safety.md#r-safe-17),
  proposed); reload the console after any restart.
- **Criteria 6 and 7 are met on the record**: every move made under a run in
  sessions 1-5 pairs with a recorded episode's drive, and every failure was
  followed by a recorded decision ([the trace](../progress/2026-10-06-every-move-traced-to-its-episode.md)).
- **Criterion 12 is met on the rover.** A person's manual move sent while a
  leg is driving ends the run and is carried out
  ([2026-10-07](../progress/2026-10-07-a-person-takes-over-on-the-rover.md)); on
  2026-10-02 it had been refused as busy. Voice is
  [deferred](../decisions/m3-defers-the-voice-trial.md). A map change and a pose
  jump ending a run stand on the offline tests, because neither can be produced
  on the rover without throwing its map away.
- **The hardware limits P0 found.** The depth camera drops off USB, which cost five
  checks in M0a's runs, and a charge gives about 20 to 25 minutes of this driving.
  Both bound how long a session can usefully run.
- **Identity.** Hypothesis inspection is M0a's and has passed; it leads to no
  follow-on action. An action relying on persistent identity needs its own case
  decided under R-WS-13 before any run may take it.

### Physical test environment

Because current sensing does not detect drops/steps reliably, unattended execution
is limited to a pre-cleared flat test area with physical hazards removed. Early
hardware trials remain supervised even inside that area.

### Milestone M3: safe closed-loop curiosity with existing behaviours

Pass when all are true:

1. mock/replay tests cover success, daemon refusal, timeout, service loss,
   low-battery abort, manual stop and no-candidate idle;
2. a model/VLM outage cannot start a new physical action unless that action was
   already fully validated and model-independent;
3. manual stop during each executive state results in an abort and the stop latch
   prevents automatic restart;
4. a supervised hardware session has run in each condition listed below, and
   no fault a session found is left unfixed. A session that finds a fault is
   followed by its fix and by that condition again. Then the charger room,
   another room and the whole flat are each driven once more on the code M3
   accepts, because the planner's live layer and route watch changed every
   route there after those conditions had passed, and those drives find
   nothing new. M4's development attempts can be those drives. Each session
   reports its autonomy time, and there is no total
   ([2026-10-09](../decisions/every-gate-says-what-it-is-for.md));
5. no session produces a physical contact with something the scan could see, or
   movement outside its configured boundary. A contact with something below the
   lidar's plane, such as shoes or a rug, is a failure of the clearing
   [R-SAFE-6](../requirements/safety.md#r-safe-6) makes a precondition; it is
   recorded and its cause removed;
6. every movement is attributable to one episode/goal ID;
7. after any action failure, the next action is either an explicitly recorded
   recovery candidate or idle -- never an unlogged implicit retry loop;
8. normal daemon/Nav2 verification remains healthy after the sessions;
9. executive kill/hang, connection loss and permission expiry stop physical motion
   while Nav2 remains running; daemon/executive restart cannot restore authority;
10. stop and takeover trials measure stopping time and distance at the permitted
    speeds, and the geofence's stopping margin is set from those measurements; an
    acknowledged request alone is not a measurement;
11. run budgets hold across background and nested actions, duplicate requests cannot
    repeat a move, and stale map/permission requests are refused at dispatch;
12. concurrent manual requests follow the declared priority, and map changes
    or loss of valid pose during execution revoke the affected movement.
    (voice requests deferred by [the decision of 2026-10-07](../decisions/m3-defers-the-voice-trial.md))
    The map and pose half stands on the offline tests. What it tests is the
    daemon's response to navigation's own signal, and producing either on the
    rover means throwing its map away. A natural occurrence in a session is
    recorded.

The sessions are a search for timing, interrupt and recovery faults. Those have
come from conditions a session had not met before, not from repeats
([2026-10-07](../decisions/trials-are-sized-by-what-they-show.md)). The
conditions are:

- **the charger room, fenced**: met by sessions 3 to 8, the last with every
  fix the others found;
- **another room, fenced, starting away from the charger**: new geometry for
  choosing goals, fitting the body and finding the way home. Partly met by
  [M4 session 2](../progress/2026-10-08-m4-session-2.md): a one-minute run in
  the bedroom, driven home by hand. Met by M4's runs from the bedroom
  ([2026-10-09](../progress/2026-10-09-m4-out-of-reach.md)), which drove out and
  came back, and by [session 17](../progress/2026-10-10-m3-sessions-14-to-19.md),
  which chose its first goals there with no fence; fenced, the bedroom has
  nothing a run can reach (session 14);
- **the whole flat, unfenced, under the current code**: doorways, long routes,
  and a run ending on the battery. Met by
  [M4 session 1](../progress/2026-10-08-m4-session-1.md): 9.5 minutes and about
  51 m, ended on the battery and driven home;
- **straight after a full power-up**: the cold start in which the gyro fault
  appeared, with the map's restore, the gyro's bias and the depth camera all
  fresh. Met by [M4 session 1](../progress/2026-10-08-m4-session-1.md), four
  minutes after the owner's power cycle;
- **a person moving through the area**, crossing the rover's path and standing
  where it wants to go, several times in a session. Session 10 had the owner in
  view once, which does not count; session 11 found the rover swinging when
  blocked, session 12 had nobody block it, and session 13 found a near goal
  blind to a person until it had set off, now fixed. In
  [M4 session 2](../progress/2026-10-08-m4-session-2.md) the owner stepped into
  a near goal's way once (confirmed by the owner): it waited 3 s and went round
  in three straight legs, 11 s in all. Standing in its way for more than 5 s is
  still owed. In [M4 session 4](../progress/2026-10-08-m4-session-4.md) the
  owner blocked a 1.8 m goal, beyond the near goals' 1.5 m, and it turned on the
  spot for 27 s; longer goals now watch their route the same way. In the
  [blocking trials](../progress/2026-10-08-blocking-trials.md) the owner stood
  still in front of a 1.2 m goal and it waited and went round them: met. On a
  4.4 m goal it stopped and waited but the way round failed twice, its last leg
  drifting towards the owner until Nav2 stopped it. With the planner's
  [live layer](../progress/2026-10-08-live-layer.md) the same 4.4 m leg was
  driven round the owner standing still, without a stop: met for a person with
  room to pass. [Standing in a doorway](../progress/2026-10-09-no-way-round.md)
  with no way round, Nav2 spun the rover in front of the owner; it now holds
  still and waits 3 s instead, seen on the rover, though given 45 cm beside
  them it squeezes past at the wall margin, which the owner chose to keep;
- **furniture moved since the map was made**: something standing in a mapped
  gap, or a door open that the map has shut. Met by
  [session 15](../progress/2026-10-10-m3-sessions-14-to-19.md), with the armchair
  by the charger moved;
- **the depth camera unavailable**, its service stopped for the run, which is
  how a USB drop leaves a run: looks that cannot range. Met by session 9: the
  looks neither failed nor stopped the run.

## Phase 4 -- semantic claims, knowledge gaps and active perception

### Purpose

Make the rover move **to learn something specific**, not only to expand the map.
Since [2026-10-09](../decisions/m4-measures-where-things-are.md) the knowledge M4
measures is where a thing is and how sure the rover may be of that, which is what
its depth camera, region finder and appearance vectors can measure. What a thing
is, its colour and whether it moves wait for a model that describes things
(*Attribute claims, later*, below).

### Where it stands

The loop M4 tests is built and has run in four supervised sessions on
2026-10-08. An aimed look files the region at its aim to the record it was sent
to improve ([R-WS-13](../requirements/world-state.md#r-ws-13), the case agreed
for supervised runs), and that region's depth sets the record's position and
claim. Goals choose viewpoints from which the depth camera can see the thing,
and tilt level for low things. A record that a look in depth view found empty
is set aside for two hours, with its group-mates. The executive records the gain
it predicts before it travels and the placement it measured after. How each of
these works is in [autonomy/README.md](../../autonomy/README.md) and
[world_state/README.md](../../world_state/README.md).

On 2026-10-08 about one aimed look in six with its thing in the depth camera's
view improved that thing (14 of 80), and none of 31 at things more than 40
degrees up did ([2026-10-08](../progress/2026-10-08-depth-sees-the-thing.md)).
Most looks that filed nothing were aimed at records not where their thing is
([2026-10-08](../progress/2026-10-08-why-aimed-looks-miss.md)): one object split
across several records, records placed where nothing stands. That rate belongs to
the world state ([R-WS-13](../requirements/world-state.md#r-ws-13),
[R-WS-17](../requirements/world-state.md#r-ws-17),
[R-WS-18](../requirements/world-state.md#r-ws-18),
[its plan](semantic-world-state.md#one-thing-per-object-and-only-that-objects-looks)).
M4 reports it and no longer waits for it.

### What is ahead

The acceptance attempts are done: 35 pairs over the eleven things taped on
2026-10-09, stopped short of the 43 declared by
[the owner's decision](../decisions/m4-acceptance-stops-at-35.md). The chosen
viewpoint beat the re-look, +0.33 (95% interval +0.05 to +0.76), and the claims
stayed honest. Read against the eight criteria below, M4 is met
([the result](../progress/2026-10-10-m4-acceptance.md)), and the owner signed it
off on 2026-10-10. Left open, for a later phase that leans on criterion 3's
margin: two records, the footboard's and the portrait's, to be attempted from
the living room. Gone badly, they would bring the interval to zero.

Also ahead, though M4 does not need it: proposing things seen once and never
placed, which are a third of the observations and one of the questions the
[design](autonomous-curiosity-design.md) asks the rover to answer.

### Knowledge gaps

An inspection names one of the gaps the rover measures:

- placed more loosely than a viewpoint from elsewhere would fix, which is the
  gain the goals predict today;
- seen from one side only, or never ranged;
- ranged looks that disagree;
- another record suspected of being the same thing;
- seen empty: a look with the place in the depth camera's view found nothing
  to file.

Gaps about what a thing is or how it looks (conflicting attributes, changed
appearance, relations to other things) wait for attribute claims.

### Viewpoint choice

The viewpoint planner exists and is deterministic: candidates sampled round a
placed thing on the map, filtered by reachability, the safe area and the depth
camera's view, scored on predicted placement gain against travel
([autonomy/README.md](../../autonomy/README.md)). Learning its bounded
parameters can begin once acceptance attempts exist, compared against the frozen
planner on held-out runs within the same safety bounds. That needs no autonomous
skill discovery and no new neural policy.

### Attribute claims, later

Typed claims over things, without changing the observations under them: what a
thing is, its colour, a size class, whether it moves, simple relations. Each
would carry its evidence IDs, method and model version, time and confidence, and
expire to stale without losing its history.

None of this is in M4, because nothing on the rover makes such claims. The region
finder names nothing, the vector models compare appearance, and text only searches
stored pictures. The last model that described things was removed
([cosmos-reason2.md](../decisions/cosmos-reason2.md)). Attribute claims come
back once a describing model has been benchmarked offline on labelled pictures
from this rover, abstains when unsure, and is right about as often as it claims.
The comparison M4 used to allow, attribute correctness from several viewpoints
against one, then becomes its own criterion.

### Milestone M4: chosen viewpoints measurably improve where things are known to be

Pass when all are true:

1. every autonomous inspection names the gap it is for and records the gain it
   predicts before it travels and the placement it measured after;
2. every placement the rover states can be traced to the looks it rests on, and
   each look to a stored picture and pose;
3. on the acceptance set, a look from the viewpoint the rover chose improves its
   target's placement more than a re-look from where it stood when it chose.
   Each attempt is paired with its own re-look, both are scored against the same
   copy of the store, and the paired difference's 95% interval is above zero;
4. the acceptance attempts are predeclared: the things, the score, the count and
   the analysis, with the count set from the development rates so that criterion
   3's comparison can be told apart. Whether an attempt applies is decided from
   what the rover knew before it: the thing out of reach of where the rover stood
   (beyond the depth camera's 4 m, outside its view or behind something on the
   map), and a reachable viewpoint with the thing inside the depth camera's view
   ([2026-10-09](../decisions/m4-asks-of-things-out-of-reach.md)). A refused thing
   is counted as a refusal, not dropped;
5. across the full attempt set the mean realised gain of chosen-viewpoint looks is
   positive, its 95% interval above zero. A look that finds nothing counts as
   zero; one that moves a placement away from the tape, or files to a record of
   another thing, counts against; attempts whose predicted gain was wrong stay in
   the report. The shares of attempts that improved, did nothing and did harm are
   reported by condition (in view, ranged, the thing's size, room), not held to a
   bar;
6. claims stay honest: for the attempted things, how often the tape lies inside
   the stated uncertainty is reported against what a one-sigma figure should
   cover, and overconfident claims, the tape more than twice the stated figure
   away, are no more common after chosen-viewpoint looks than after re-looks;
7. failing to find a useful safe viewpoint, or finding nothing there, leaves the
   gap open and says so; no placement is invented;
8. each attempt reports whether the chosen viewpoint's look, on its own, changed
   the target's claim the way the tape's score says it changed the placement:
   the claim change worked out as the executive works it out in an ordinary run,
   on the same copy of the store as the tape score. Over the looks whose claim
   changed, the share where the tape's score has the same sign is reported with
   its interval. Looks that changed neither are counted apart, since agreeing on
   nothing says nothing. Re-looks are reported the same way, separately. It is
   not a bar here. After M4 there is no tape: M7 would judge a
   skill's success, and M10 predict a goal's gain, from the rover's own account,
   so M10's entry uses this figure
   ([2026-10-09](../decisions/every-gate-says-what-it-is-for.md)).

Honesty (criterion 6) is pass/fail for the same reason: every later phase scores
itself by the rover's claims, and a claim that narrows past the truth would teach
it to.

Gain is never the rover becoming more confident alone. The score rewards a
placement for coming closer to the tape and for an uncertainty that narrows
honestly, and penalises one that narrows past the tape. The proposal is the
log-likelihood of the tape's position under the stated position and uncertainty,
floored at that of a 2 m claim so that one wild claim cannot decide the mean. It
is fixed in the scoring script before the acceptance attempts.

## Phase 5 -- temporal memory, staleness and change-driven curiosity

### Purpose

Learn that the environment is not static.

### Work

- Classify entities/claims by expected change rate: fixture, usually static,
  movable, transient, unknown.
- Add last-confirmed time and configurable staleness functions.
- Compare new observations with prior evidence before creating a new entity.
- Represent disappearance/movement/change as hypotheses with evidence.
- Add revisit candidates whose expected value combines staleness, past change rate,
  user purpose and travel cost.
- Preserve the full history when a moved object is re-identified.

### Controlled scene-change test

Use scripted changes that can be independently recorded, for example:

- move known object within the same room;
- remove it;
- return it elsewhere;
- add a novel object;
- change contents on a surface;
- leave an unchanged control area.

### Before any scripted scene

A change the rover reports is worth having only if it is right more often than
wrong, and that depends on how often things here really change. If things in the
flat move in one visit in ten, it needs false alarms on fewer than about one
unchanged revisit in eleven at 80% detection. The store as it stands would fail
long before that: 20 of the 32 aimed looks that filed nothing on 2026-10-08 were
aimed at records not where their thing is
([the measurement](../progress/2026-10-08-why-aimed-looks-miss.md)), and an
unchanged revisit would read each of those as missing. So M5 begins with two
measurements ([2026-10-09](../decisions/every-gate-says-what-it-is-for.md)):

- **how often things in the flat change between visits**, from the owner and the
  archived looks. The false-alarm budget is set from it, so that a reported change
  is right more often than wrong;
- **how many change hypotheses replayed revisits of unchanged scenes raise**, on
  the recordings already held. Scripted scenes wait until replay meets the budget.

M3's remaining sessions are recorded for these, at the owner's request
(2026-10-10). Its re-drives of the charger room, another room and the whole flat
are revisits of unchanged rooms, and its furniture-moved session is a real
change, noted by the owner as it is made. Nothing raises a change today, so these
recordings are replayed once something does.

The same budget is what [R-WS-17](../requirements/world-state.md#r-ws-17) and
[R-WS-18](../requirements/world-state.md#r-ws-18)'s proposed tolerances are to be
checked against.

### Milestone M5: the rover notices useful changes without filling the world with duplicates

Pass when all are true:

1. scripted changed scenes and unchanged control revisits are recorded as an
   acceptance set, their counts set so that criterion 3's budget can be told
   apart from what it rules out. Ten controls with at most one alarm show only
   that the rate is under 39%;
2. how often changed scenes generate the intended change, missing or moved
   hypothesis is reported with its interval, not held to a bar;
3. confident change alarms on unchanged controls stay within the budget set
   before any scripted scene;
4. on predeclared applicable moved-entity trials, how often identity and history
   are kept is reported with its interval; unresolved outcomes are reported but
   do not count as kept, and any incorrect confident merge fails this criterion;
5. the revisit scorer demonstrably ranks a recently changed/stale area above an
   equally distant recently-confirmed static area under the configured policy;
6. repeated revisits reduce utility after the knowledge has been refreshed, so the
   rover does not become trapped by one formerly novel object.

Criterion 4 and the milestone's duplicates are the world state's to prevent. What it
must deliver is [R-WS-17](../requirements/world-state.md#r-ws-17) (each object is one
thing) and [R-WS-18](../requirements/world-state.md#r-ws-18) (each thing holds one
object's looks). The work towards them is
[one thing per object](semantic-world-state.md#one-thing-per-object-and-only-that-objects-looks)
in the world-state plan.

### Multi-day usefulness benchmark

Before claiming useful lifelong memory, compare autonomy through M5 with
fixed-schedule revisits over at least three separate days. Frontier-only
exploration is not a baseline: once the flat is mapped it has nowhere to go, so
beating it would show nothing
([2026-10-07](../decisions/trials-are-sized-by-what-they-show.md)). Use matched initial knowledge, scripted scene changes and unchanged controls,
equal time/travel budgets, and independent annotations. Counterbalance comparison
order or reset matched scenes so one policy does not inherit another's observations.
Keep acceptance objects/runs separate from tuning data.

State its budget in charges and owner hours before planning it. R-SAFE-6 is open
and nothing docks the rover, so every minute of it is supervised and every charge
is plugged in by hand, at 20 to 25 minutes of driving a charge. Its questions and
minimum improvement are sized by the comparison
([2026-10-09](../decisions/every-gate-says-what-it-is-for.md)).

Include a process/rover restart and a controlled map reset with retained historical
evidence. Predeclare question sets such as "what moved?", "where was this last
seen?" and "what remains uncertain?", plus a useful minimum improvement before the
run. Report answer correctness, false claims, unresolved answers, change detection,
inspection cost and interventions across all attempts.

Pass when retained evidence remains resolvable after those interruptions, stale
coordinates cannot drive movement, and autonomy meets the predeclared improvement
over the baseline without increasing false confident claims or weakening safety
limits. Failure leaves the individual M5 results intact but the programme's
multi-day usefulness claim unproven. Repeat the relevant comparison with M7
learning enabled against frozen skills/parameters to show the benefit of learning.

## Phase 6 -- restricted procedural skill library

Phases 6 and 7 are entered once the episode record holds a repeated multi-step
behaviour that the executive does not already perform as one goal, and that
succeeds in some contexts and not others
([2026-10-09](../decisions/every-gate-says-what-it-is-for.md)). An autonomous run
has three operations, drive, look and stop, and one way of using them, which is
the reference skill below. A learner over that record could only rediscover it or
propose something trivial, and either would pass M7 without the rover knowing
anything new. Until then the learning asked for first is the viewpoint planner's
bounded parameters (*Viewpoint choice*, under Phase 4) and M10's outcome model.

### Purpose

Give the rover a safe representation in which new high-level behaviours can be
stored, tested and reused.

### Do not use arbitrary Python as the learned representation

`docs/runbooks/scripting.md` correctly notes that current scripts are not filesystem-sandboxed.
Model-written Python therefore must not become the lifelong-learning mechanism.

Implement a small typed DSL, behaviour tree or graph with only admitted nodes.
Candidate node types might include:

```text
Sequence
Fallback
If
Repeat(max_count)
Deadline
CallSkill
DaemonCall(admitted_name, typed_args)
ObserveWorld
EvaluatePredicate
```

Every node has deterministic validation. No imports, file access, shell commands,
network access, dynamic evaluation or raw motor loops exist in the language.

### Skill metadata

Store:

- stable skill ID and version;
- parameter schema;
- preconditions;
- expected effects;
- bounds: time, travel, retries and battery;
- success predicate;
- known failure/recovery outcomes;
- dependencies on other skills/primitives;
- provenance: episodes/proposal that created it;
- trial statistics by context;
- lifecycle state: proposed, validated, supervised, verified, quarantined.

### First reference skill

Hand-author `multi_view_inspect(entity)` in the new representation. This proves the
interpreter and lifecycle without conflating language-model skill invention with
runtime correctness.

### Milestone M6: a learned-skill substrate exists and cannot escape its authority

Pass when all are true:

1. parser/schema/interpreter tests reject unknown nodes, unbounded loops, invalid
   parameter types and movement bounds outside policy;
2. a deliberately malicious model-like payload attempting code/file/shell access is
   rejected as data, not executed;
3. replayed against recorded daemon outcomes, the hand-authored reference skill
   makes the same calls within the same limits as the executive's own multi-view
   inspection from M4, and in a few supervised runs on the rover it executes and
   stops through the ordinary path. How often it answers its question is M4's
   measurement, not this milestone's
   ([2026-10-07](../decisions/trials-are-sized-by-what-they-show.md));
4. failure cases correctly classify precondition failure, navigation failure,
   perception failure and timeout rather than reporting generic success;
5. skill execution remains interruptible through the ordinary stop path;
6. changing a skill version does not rewrite prior episode records; replay names the
   exact version used;
7. a quarantined skill cannot be selected by the executive.

## Phase 7 -- skill discovery and promotion from experience

### Purpose

Move from a manually populated skill library to genuine autonomous procedural
learning.

### Experience mining

Look for repeated subsequences and recurrent recovery patterns in successful
episodes. Use deterministic clustering/features first; a reasoning model can then
propose an abstraction such as:

```text
episodes 81, 97, 103
  -> move to useful side viewpoint
  -> inspect
  -> if unresolved, try opposite side

proposal:
  multi_view_inspect(entity)
```

The proposal must generalise concrete IDs into typed parameters and supply
preconditions, effects, limits and an externally observable success predicate.

### Proposal validation

Use three gates:

1. **static** -- schema and policy validation;
2. **replay/simulation** -- test against stored/mock scenarios including known
   failures;
3. **supervised hardware** -- only after the first two pass.

Promotion thresholds are configuration and recorded with the promotion event.

### Milestone M7: the rover can create and verify a useful new skill

Pass when all are true:

1. the learner proposes at least one non-trivial parameterised skill from multiple
   episodes rather than copying one recorded action sequence verbatim;
2. the proposed skill references only admitted primitives/verified skills;
3. static validation catches seeded invalid proposals;
4. offline evaluation shows the candidate succeeds at least as often as the
   unabstracted baseline on held-out applicable cases whose action outcomes are
   covered by recordings or a hardware-validated simulator; uncovered alternatives
   remain unknown and require supervised evidence before performance is claimed;
5. in supervised physical trials the candidate does at least as well as the
   executive without it on the same applicable cases, paired, with the count
   sized for that comparison and every attempt counted. A promoted skill replaces
   what the executive would have done, so that is the bar it has to clear
   ([2026-10-09](../decisions/every-gate-says-what-it-is-for.md));
6. promotion is performed by evaluator policy, not by the model that proposed the
   skill;
7. after promotion, the executive retrieves and uses the skill on at least five new
   applicable tasks without hard-coded task/entity IDs;
8. a deliberately regressed new version is detected by acceptance replay and not
   promoted.

The milestone should ideally use a skill other than the hand-authored reference
skill so it demonstrates new procedural knowledge rather than rediscovery of the
example.

## Phase 8 -- self-generated curriculum and competence progress

This phase is optional since 2026-10-07
([the decision](../decisions/trials-are-sized-by-what-they-show.md)). Practising
for competence is not among the things the
[design](autonomous-curiosity-design.md) asks the rover to become good at, and
there is nothing to practise until a learned skill exists. It is entered only once
M7 has produced a skill whose success varies with context, neither saturated nor
consistently impossible.

### Purpose

Allow the rover to choose some goals because practising them is expected to improve
its competence, not just because the environment is novel.

### Competence model

For each skill/context family, maintain a learning curve from actual trials. A
practice goal receives utility when:

- the skill is useful under current human policy;
- its success is neither already saturated nor consistently impossible;
- recent trials show measurable improvement or uncertainty about competence;
- a safe, low-cost practice instance is available.

This prevents endless rehearsal of an already mastered light switch or repeated
attempts at a task impossible with current hardware.

### Curriculum comparison

Use the simulator/mock room and then bounded hardware sessions to compare:

- random eligible practice;
- novelty-only choice;
- competence-progress choice.

Use fixed task pools and count attempts to reach a target success threshold.

### Milestone M8: practice choices improve competence efficiently

Pass when all are true:

1. skill success is tracked by context rather than one global number;
2. saturated and consistently impossible skills lose practice utility;
3. on a fixed mock/simulation task pool, competence-progress selection reaches the
   declared mastery criterion in fewer attempts than random eligible selection;
4. at least one physical skill shows a pre-registered improvement from its first
   block of trials to a later block without changing the underlying low-level
   controller;
5. no learning-progress signal can override movement/safety vetoes;
6. the system can explain whether a goal was chosen to learn about the environment
   or to improve a skill.

## Phase 9 -- periodic reflection with a provider-independent reasoning model

### Purpose

Use foundation-model reasoning where it adds value without giving it direct
physical authority.

### Reflection input

Send compact structured summaries, not the entire raw database. Include explicit:

- episode IDs;
- entities/claims referenced;
- surprises/failures;
- recent skill statistics;
- unresolved knowledge gaps;
- policy constraints.

### Allowed outputs

A reflection model may propose:

- a new hypothesis;
- a candidate knowledge gap;
- a bounded experiment;
- a failure explanation;
- a skill abstraction/refactor;
- a priority/purpose suggestion.

It may not directly execute a daemon call, edit a verified skill in place, promote a
skill, change safety policy or mutate observations.

### Provider benchmark

Build a fixed reflection/tool-planning evaluation set from real episodes. Run the
same set against candidate providers/models and record:

- valid structured output rate;
- grounded-reference rate;
- useful proposal rate under human review;
- unsafe/invalid proposal rate;
- latency and cost.

This is where Qwen/Gemini/OpenAI or future local models can be compared on actual
rover cognition rather than generic benchmarks.

### Milestone M9: reflection produces grounded useful proposals but is not required for safety

Pass when all are true:

1. all model references to stored objects resolve by ID before a proposal is
   accepted into the candidate queue;
2. hallucinated IDs and unsupported operations are rejected in automated tests;
3. fixed reflection cases are drawn from real episodes, with a held-out subset for
   prompt/model selection, their count sized for criterion 4's comparison;
4. reflection finds what the scorer would not. On recordings, a held-out proposal
   counts when it names a gap the scorer's own choices missed and a later look
   resolved it; if that shows any, supervised runs compare reflected experiments
   with the scorer's choices at the same budget. Nothing later depends on
   reflection, so this is the only thing it is for; whether proposals sound
   sensible to a reviewer is not evidence of it
   ([2026-10-09](../decisions/every-gate-says-what-it-is-for.md));
5. disabling network/model access leaves mapping, safety, current verified skills
   and model-independent curiosity operation functional;
6. a reflected experiment must still pass ordinary goal scoring, safety veto and
   budget checks before execution.

## Phase 10 -- empirical predictive world model

### Purpose

Learn the consequences/costs of skills from accumulated episodes before attempting
large learned dynamics models.

### Initial prediction targets

For context + skill + parameters, predict:

- success probability;
- failure category;
- duration;
- travel cost;
- realised information gain;
- optionally battery/energy cost when measurement is reliable.

Use interpretable baselines first: empirical tables, calibrated logistic models,
gradient-boosted trees or similarly inspectable methods. A neural model is justified
only if it beats these on held-out data and the dataset is large enough to support
it.

### Planner integration

Predictions can influence **ranking** among already safe candidate goals/skills.
They do not remove hard constraints.

### Entered after M4, for the goal types that exist

The scorer's predicted gain was right for about one geometry goal in ten
([2026-10-06](../progress/2026-10-06-looks-seldom-reach-their-thing.md)), so a run
spends most of its charge on looks it expected to pay. Every later phase that
chooses trades a predicted gain against battery, and that trade means little while
the prediction is off by that much. An outcome model for the goal types that exist
needs only M4's attempts, so this phase is entered for them once M4 has run,
without waiting for Phases 6 to 9, provided M4's criterion 8 shows the rover's own
gain agreeing in sign with the tape more often than chance. Below that the model
would learn the rover's own noise
([2026-10-09](../decisions/every-gate-says-what-it-is-for.md)). Calibration
(criterion 5) is the threshold the later phases need.

### Milestone M10: learned outcome prediction improves planning on held-out experience

Pass when all are true:

1. training and held-out episodes are separated by time/run so near-duplicate
   trajectories do not leak across the split;
2. a simple prior/baseline predictor is recorded;
3. the learned predictor beats the baseline on declared held-out metrics (for
   example Brier score for success and absolute error for duration/information
   gain);
4. a fixed comparison with covered action outcomes or a hardware-validated simulator
   shows lower-cost successful plans than the pre-learning ranking; supervised
   hardware comparison confirms improvement before claiming physical benefit.
   Re-scoring an untried plan with the same predictor does not prove it succeeds;
5. calibration is checked -- predicted 80% success cases should be approximately
   80% successful over a sufficiently populated bin rather than merely ranked
   correctly;
6. removing/corrupting the predictor falls back to conservative baseline estimates,
   not unrestricted action.

Only after M10 is there a data-backed reason to investigate a learned latent/video
world model or policy optimisation.

## Phase 11 -- optional neural policy/world-model research

This phase is deliberately not required for the main project goal.

Possible experiments include:

- learned viewpoint-value model;
- model-based prediction of local semantic observations;
- policy optimisation for high-level goal ordering;
- simulation-trained navigation/inspection sub-policies;
- offline reinforcement learning from accepted episodes;
- action-conditioned visual world models for manipulation if the rover later gains
  an arm/probe.

Any neural policy that can affect movement is first trained/evaluated offline and
runs behind the same daemon/Nav2 hard boundaries. It must beat the explicit skill
system on a held-out benchmark before receiving hardware trials.

### Milestone M11: optional learned policy earns a bounded hardware trial

Pass only when:

1. a fixed offline benchmark exists before model selection;
2. the learned policy materially beats the explicit baseline on that benchmark;
3. failure modes are characterised, including distribution shift;
4. output is constrained to the same admitted high-level actions as the existing
   executive;
5. a shadow-mode rover run shows no unsafe/uninterpretable requests;
6. the first hardware trial has a strict geofence, low speed, short deadline and
   human supervision.

## Cross-phase test infrastructure

These pieces are worth building early because several milestones depend on them.

### Deterministic autonomy scenario format

A checked-in text/JSON fixture should be able to describe:

- map/frontier summary;
- rover pose/battery/health;
- entities, observations and claims;
- skill availability/statistics;
- candidate model outputs where needed;
- expected vetoes/candidate ordering/action.

It should not contain runtime credentials or large images.

### Episode replay

Replay should support at least:

- recomputing curiosity scores under a new configuration;
- re-running candidate generators;
- testing a skill interpreter against recorded daemon outcomes;
- substituting recorded model responses;
- comparing planner decisions before/after a learned predictor;
- generating a concise difference report.

Each comparison records its action/context coverage and labels outcomes as recorded,
simulated or unknown. The harness must reject attempts to attach the recorded result
of one viewpoint/action to a different, unsupported choice. Fixed model responses
reproduce decisions; they do not establish that newly proposed observations exist.

### Physical acceptance manifest

For each important hardware run record outside Git:

```text
run ID/date
Git commit
map identity
world/autonomy DB paths
recording paths
hardware present
calibration version
model/provider versions
human annotations
purpose and milestone
```

A small checked-in manifest may reference those artefacts without committing the
artefacts themselves.

### Fault injection

The mock rover should eventually support deliberate:

- Nav2 refusal;
- blocked route;
- lost world-state/perception service;
- stale map identity;
- model timeout/malformed output;
- low battery;
- impossible knowledge gap;
- skill timeout;
- manual stop during action;
- executive crash/hang, lost connection and expired movement permission;
- restart, manual contention, duplicate requests and map changes during action;
- map clear, entity merge, evidence deletion and local-ID reuse during later replay.

A system that only learns from successes will create optimistic skills and brittle
plans.

## Proposed implementation boundaries

These are starting points, not mandatory file names.

```text
autonomy/
  README.md
  schema.py
  store.py
  episode.py
  goals.py
  curiosity.py
  constraints.py
  executive.py
  replay.py
  reflection.py
  claims.py
  change.py
  viewpoint.py
  skills/
    schema.py
    validate.py
    execute.py
    library.py
    learn.py
  models/
    provider.py
    recorded.py
    ... provider adapters
  tests / selftest
```

The component should depend on `rover_daemon`'s public protocol rather than import
hardware implementation details. This makes mock/replay testing possible and keeps
the hardware owner unchanged.

`world_state/` remains responsible for visual observations, persistent entity
resolution and source evidence. The autonomy component may add claims/episodes in
its own store initially rather than expanding the world-state database before the
new semantics are proven.

If a claim type later becomes a stable part of what an entity **is**, it can migrate
into `world_state` with an additive schema change.

## Deployment progression

Movement authority should be exposed gradually:

| Stage | Hardware authority |
|---|---|
| M1 | none; event recording only |
| M2 | none; shadow decisions only |
| M3 | after M1/M2 and control tests; semantic hypothesis inspection additionally needs M0a, an identity-dependent action its own decided case; bounded operations in supervised safe area |
| M4-M5 | bounded semantic inspection/revisit in same safe area |
| M6-M7 | validated interpreter; candidates only in supervised trials after offline gates, verified skills eligible for autonomous reuse |
| M8-M9 | executive chooses practice/reflection goals; same physical boundary |
| M10+ | learned ranking/prediction only; hard constraints unchanged |

A change that adds a deployed autonomy component will eventually need a
`deploy/manifest.json` entry and host verification. Documentation-only commits do
not require deployment.

## Stop conditions and rollback

Pause expansion of autonomy and investigate if any of these occurs:

- an unvalidated identity is used as fact by an identity-dependent action;
- a hypothesis inspection bypasses viewpoint validation, escapes its effort limits
  or claims verification without sufficient evidence (R-AUT-12);
- an autonomous action cannot be attributed to an episode/goal;
- manual stop is ignored or autonomy restarts without explicit re-enable;
- skill execution reaches an operation not present in its validated graph;
- a model-proposed reference is accepted without resolving to stored evidence;
- duplicate/change behaviour regresses materially after adding temporal semantics;
- a new learned component improves its target metric only by weakening ambiguity or
  safety thresholds;
- physical behaviour disagrees with replay in a safety-relevant way.

Rollback means disable the new decision layer and retain the evidence. Do not
rewrite or delete the failed episodes; they are exactly the data needed to reproduce
the problem.

## Milestone summary

| Milestone | Capability | Proof |
|---|---|---|
| M0a (passed 2026-10-02) | bounded verification of uncertain hypotheses | shared geometry/capture gates + replay refusals + useful outcomes in >=20 attempts across >=3 fresh supervised runs |
| M0b (retired 2026-10-01) | actions relying on persistent identity | no single gate: each such action's case is decided under R-WS-13 before an autonomous run may take it |
| M1 | episodic memory | durable reconstruction across resets/merges, no authority |
| M2 | curiosity shadow mode | fixed scenarios + a no-action rover shadow run |
| M3 | bounded autonomous loop | a supervised session in each listed condition, then the current code over the conditions its changes touched; measured stops, permission expiry and failure tests |
| M4 (passed 2026-10-10) | active perception | placement against tape: chosen viewpoints beat a paired re-look, mean gain positive, claims honest, over a predeclared attempt set sized for it; the rover's own gain checked against the tape |
| M5 | temporal curiosity | false alarms on replayed unchanged revisits within a budget set from the flat's change rate, then scripted changed/unchanged scenes |
| M1-M5 usefulness | memory improves useful answers over days | >=3 days, an equal-budget fixed-schedule baseline, restarts/reset, all attempts counted; budget stated in charges and owner hours |
| M6 (on evidence) | restricted skill substrate | malicious/invalid rejection + reference skill equal to the executive in replay and stopping on the rover |
| M7 (on evidence) | autonomous skill acquisition | proposed -> replayed -> at least as good as the executive on the rover, paired -> reused skill |
| M8 (optional) | self-generated curriculum | competence-progress beats random baseline |
| M9 | model reflection | finds gaps the scorer missed; outage-safe architecture |
| M10 (after M4) | empirical world model | held-out prediction and planning improvement, calibrated |
| M11 | optional neural policy | offline win + shadow gate before bounded hardware trial |

## What comes next

M1 and M2 were the first slice and passed on 2026-09-08. In order from here:

1. **Finish M3 on the rover.** Every condition has been driven, and the
   charger room, another room and the whole flat again on the current code
   (criterion 4). One run is left: on the fix for the drive session 17 had
   refused (4e3a447), from the bedroom over the whole flat. Then the owner's
   sign-off.
2. **Run M4's attempts.** The trial run, its re-look and the scoring exist;
   what is left is labelling the six old targets' records, development attempts
   on them to size the set, a new taped set, and then the acceptance attempts
   (*What is ahead before the acceptance attempts*, under Phase 4).
3. **Let looks reach their things.** This is the world state's work (R-WS-13,
   R-WS-17, R-WS-18). M4 no longer waits on it, but it sets how many attempts
   improve anything, about one in six with the thing in view on 2026-10-08, and
   M5 does wait on it: its first measurement is how many false alarms the store
   raises on unchanged scenes.
4. **M10's outcome model for the goal types that exist**, once M4 has run.
5. **The owner's choice on running unattended.** No milestone reaches the
   design's goal of a rover left to occupy itself: R-SAFE-6 is open and nothing
   docks the rover ([2026-10-09](../decisions/every-gate-says-what-it-is-for.md)).

The question the programme now turns on is: **given what the rover actually
knows, does choosing where to look make it know more?** M3 shows that it can
choose and act safely. M4 now asks it of where things are, on average and
against looking again from where the rover stands; how often a single attempt
helps stays the world state's to raise.
