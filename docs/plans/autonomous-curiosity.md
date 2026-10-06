# Development plan: curiosity-driven autonomy

Status: Phase 0 (P0) closed on 2026-10-02 ([the closure](../progress/2026-10-02-p0-closed.md)).
Phases 1 and 2 (P1, P2) passed their milestones on 2026-09-08. Phase 3 (P3) is under way: the executive and the
daemon-enforced permission it works under are deployed, and on 2026-10-02 they moved
the rover in supervised stop trials and in M0a's inspection runs. Five of M3's twenty
supervised sessions have run, about 33 minutes; [the third](../progress/2026-10-06-m3-session-3.md)
showed R-AUT-13's ending, and on 2026-10-06 a repeated request, a hung executive
and a lost connection were [tried on the rover](../progress/2026-10-06-repeat-hang-and-drop-on-the-rover.md).
Later phases remain proposed. This is the
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
procedural learning additionally requires M6-M8; reflection and predictive models
are extensions whose value must be measured, not prerequisites for recording or
the first useful inspection loop.

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
observations by object/run; 20 trials or 50 decisions are minimum checks, not a
general reliability guarantee.

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
under "what is still ahead" below, and the first item is the one that matters:
nothing decides anything yet.

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

- **Nothing decides anything.** The recorder watches and writes down; the
  executive that would choose a goal is Phase 2, and until it exists every
  episode is an occasion of the rover acting rather than of it deciding. The
  record has been proved against real events -- 59 moves and 426 looks across
  three fillings of the world state -- and never against a decision, because
  there are none to record.
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

- **Nothing acts on any of it.** The executive, the daemon-enforced movement
  permission and the stop latch are M3, and none of them exists. Every
  deliberation closes `abandoned` for the same reason: there is no authority.
- **Four goal types are missing**, and they are the semantic ones:
  `inspect_uncertain_entity` (explicit gaps and claims from M4),
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
- **Nothing measures whether a chosen goal would have paid off.** Predicted gain
  against realised gain is what M4's evaluator needs and what any learned
  ranking later rests on, and it cannot be measured until something acts.

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

Most of M3 is still physical:

- **The twenty supervised sessions**, totalling at least two hours of autonomy
  time in a pre-cleared area, with the owner present. Five have happened
  ([1](../progress/2026-10-05-m3-session-1.md), 8.7 minutes;
  [2](../progress/2026-10-05-m3-session-2.md), 11 minutes, stopped at 5% battery;
  [3](../progress/2026-10-06-m3-session-3.md), 3.6 minutes, ended by going home;
  [4 and 5](../progress/2026-10-06-m3-sessions-4-and-5.md), 8.3 and 1.5 minutes,
  both stopped on the battery, 44 drives and none failed);
  M0a's runs were its own protocol and do not count towards them. Nothing below
  criterion 3 can be answered without them. Open each with no action limit and
  a safe area fenced to the cleared room, drawn 0.6 m beyond where the run
  starts. A fenced run ends by going home
  ([R-AUT-13](../requirements/autonomy.md#r-aut-13), settled by session 3); an
  unfenced one has the whole flat and ends on the battery, since runs carry no
  battery floor. Start each on a full charge and judge the battery at rest: a
  reading while driving sags by up to 30 points.
- **Looks that improve nothing.** Of 171 geometry goals on record, 10 improved
  their thing when measured and 9 more within a minute
  ([the measurement](../progress/2026-10-06-looks-seldom-reach-their-thing.md)).
  The depth camera works; the look's evidence mostly does not reach the thing
  it was aimed at, which waits on the identity work (R-WS-13). Until then a
  run's geometry goals mostly spend battery, and a session's count of looks is
  not a count of work done.
- **The console's stop at speed.** Criterion 9 is met on the rover, and so is
  R-AUT-11 ([2026-10-06](../progress/2026-10-06-repeat-hang-and-drop-on-the-rover.md)),
  and the safe area's margin is now set from measured stops (0.6 m, criterion
  10). The console's button is still unmeasured at speed: it was exercised on a
  slow turn on 2026-10-02 and asked for twice on 2026-10-06 without a press. It
  sends the same `stop_driving` that stopped the rover in 0.11 m. It wants a
  trial where the owner is told the moment to press, from beside the console.
  No drive has yet been refused on the new margin.
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
4. 20 supervised hardware autonomy sessions complete in the pre-cleared area,
   totalling at least 120 minutes of autonomy time;
5. no session produces an unexpected physical contact or movement outside its
   configured boundary;
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
12. concurrent voice/manual requests follow the declared priority, and map changes
    or loss of valid pose during execution revoke the affected movement.

The 20-session count is deliberately about repeated opportunities for timing,
interrupt and recovery faults rather than distance travelled.

## Phase 4 -- semantic claims, knowledge gaps and active perception

### Purpose

Make the rover move **to learn something specific**, not only to expand the map.

### Semantic claim layer

Add typed claims/hypotheses over entities without changing immutable observations.
Initial predicates should be small and testable, for example:

- coarse category/description;
- colour;
- approximate size class where geometry supports it;
- movable/static hypothesis;
- simple spatial relationships.

Every accepted claim stores evidence IDs, method/model version, timestamp and
confidence. A claim can expire to stale without deleting its history.

### Knowledge gaps

Generate explicit questions such as:

- insufficient independent viewpoints;
- conflicting attribute estimates;
- appearance changed since last observation;
- uncertain relation to another placed entity;
- entity was expected but not visible in a sufficiently complete revisit.

### Viewpoint planner v1

Start deterministic. Candidate viewpoints can be sampled from the current map around
a placed entity and filtered through reachability/safety. Score them using predicted:

- parallax improvement;
- image scale/distance;
- occlusion proxy where available;
- travel cost;
- pose uncertainty;
- expected ability to test the specific hypothesis.

The planner records the predicted gain before travel.

Versioned learning of bounded viewpoint parameters can begin here once enough
episodes exist. Compare learned parameter choices with a frozen baseline on held-out
runs, retaining the same safety bounds. This does not require autonomous skill
discovery or a new neural policy.

### Acceptance dataset

Build a physically annotated set of at least 30 entities/questions across multiple
viewpoints. Keep a development subset separate from an acceptance subset if prompts
or thresholds are tuned.

### Milestone M4: additional viewpoints measurably improve knowledge

Pass when all are true:

1. every autonomous inspection names the knowledge gap it is trying to resolve;
2. claims cannot exist without resolvable evidence IDs;
3. active viewpoint selection beats a defined baseline (same-pose re-look or nearest
   reachable viewpoint) on the held-out acceptance set for the chosen metric --
   attribute correctness, association resolution or calibrated uncertainty;
4. a predeclared set of at least 20 applicable hardware active-inspection attempts
   is completed, reporting useful viewpoints, unresolved outcomes and failures;
5. median realised information gain across that full attempt set is positive;
   no-evidence attempts count as zero, incorrect changes are penalised, and cases
   where predicted gain was wrong remain in the report;
6. false high-confidence semantic claims are reported separately and do not exceed
   the baseline single-view system;
7. failure to find a useful safe viewpoint leaves the gap unresolved rather than
   inventing a result.

Do not define "information gain" as the model becoming more confident alone. The
acceptance metric must include correctness against annotation or resolution of a
geometry/consistency constraint.

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

### Milestone M5: the rover notices useful changes without filling the world with duplicates

Pass when all are true:

1. at least 10 scripted changed scenes and 10 unchanged control revisits are
   recorded as an acceptance set;
2. at least 8/10 changed scenes generate the intended change/missing/moved
   hypothesis;
3. no more than 1/10 unchanged controls generates a high-confidence change alarm;
4. on at least 10 predeclared applicable moved-entity trials, at least 80% correctly
   retain identity/history; unresolved outcomes are reported but do not count as
   successes, and any incorrect confident merge fails this criterion;
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

Before claiming useful lifelong memory, compare autonomy through M5 with both
frontier-only exploration and fixed-schedule revisits over at least three separate
days. Use matched initial knowledge, scripted scene changes and unchanged controls,
equal time/travel budgets, and independent annotations. Counterbalance comparison
order or reset matched scenes so one policy does not inherit another's observations.
Keep acceptance objects/runs separate from tuning data.

Include a process/rover restart and a controlled map reset with retained historical
evidence. Predeclare question sets such as "what moved?", "where was this last
seen?" and "what remains uncertain?", plus a useful minimum improvement before the
run. Report answer correctness, false claims, unresolved answers, change detection,
inspection cost and interventions across all attempts.

Pass when retained evidence remains resolvable after those interruptions, stale
coordinates cannot drive movement, and autonomy meets the predeclared improvement
over both baselines without increasing false confident claims or weakening safety
limits. Failure leaves the individual M5 results intact but the programme's
multi-day usefulness claim unproven. Repeat the relevant comparison with M7/M8
learning enabled against frozen skills/parameters to show the benefit of learning.

## Phase 6 -- restricted procedural skill library

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
3. the hand-authored reference skill satisfies its independent success predicate
   in at least 18/20 predeclared applicable supervised trials; stopping with an
   unresolved question is reported separately, not counted as success;
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
5. supervised physical trials meet a pre-declared promotion threshold, initially
   suggested as at least 18/20 successes when preconditions hold;
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
3. at least 50 fixed reflection cases are available, with a held-out subset for
   prompt/model selection;
4. at least one model produces useful grounded proposals on a clear majority of the
   held-out cases under documented human review criteria;
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
- restart, manual/voice contention, duplicate requests and map changes during action;
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
| M2 | curiosity shadow mode | fixed scenarios + one-hour no-action rover shadow |
| M3 | bounded autonomous loop | 20 supervised sessions / >=120 min, measured stops, permission expiry and failure tests |
| M4 | active perception | held-out multi-view gain + >=20 hardware inspections |
| M5 | temporal curiosity | scripted changed/unchanged scene benchmark |
| M1-M5 usefulness | memory improves useful answers over days | >=3 days, two equal-budget baselines, restarts/reset, all attempts counted |
| M6 | restricted skill substrate | malicious/invalid rejection + 18/20 reference-skill trials |
| M7 | autonomous skill acquisition | proposed -> replayed -> physically verified -> reused skill |
| M8 | self-generated curriculum | competence-progress beats random baseline |
| M9 | model reflection | grounded fixed benchmark; outage-safe architecture |
| M10 | empirical world model | held-out prediction and planning improvement |
| M11 | optional neural policy | offline win + shadow gate before bounded hardware trial |

## First implementation slice

P0 is already underway. Continue it on its existing track; the next independent
implementation slice is M1 -> M2, without movement authority:

1. use the current P0 baseline and follow-up recordings as versioned inputs, clearly
   marked with their unresolved calibration/association limitations;
2. define durable evidence references and retention, then add append-only episodic
   storage and deterministic replay; coordinate shared world-state changes with P0;
3. define the candidate-goal record and hard-veto interface;
4. implement geometric-frontier and geometric-uncertainty generators in shadow mode;
   semantic and temporal generators follow their M4/M5 evidence dependencies;
5. add score decomposition and a fixed scenario harness;
6. run the shadow executive on real rover state without action authority;
7. inspect the resulting choices; M3 needs M1/M2 and daemon-enforced control tests,
   followed by supervised physical stop/failure trials. Implement and replay
   R-AUT-12 before the narrowly scoped M0a supervised acceptance trials; semantic
   inspection beyond those trials needs M0a, and an identity-dependent action
   needs its own case decided. Starting read-only work waives neither.

At the end of that slice the project will already answer a useful empirical
question: **given what the rover actually knows today, does an explicit curiosity
objective choose sensible things to investigate?** If the answer is no, it can be
fixed in replay without having moved the robot under an unproven executive.
