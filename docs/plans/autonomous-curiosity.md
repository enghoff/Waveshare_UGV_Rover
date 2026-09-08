# Development plan: curiosity-driven autonomy

Status: Phase 0 (P0) is in progress and Phase 1 (P1) is under way alongside
it, with no action authority; later phases remain proposed. This is the
implementation and acceptance plan for
[the architecture it implements](autonomous-curiosity-design.md).

The plan is gated by capability dependencies, not a requirement to finish every
phase before starting the next. M1 recording and M2 shadow decisions may proceed
alongside the existing P0 work, with no action authority. M3 movement requires M0,
M1 and M2 acceptance plus offline control-boundary tests; supervised stop/failure
trials precede its autonomy sessions. Later capabilities require
recorded evidence for the primitives, semantics and execution substrate they use;
code existing is not acceptance. Physical criteria must be observed on the rover.

P0 is tracked through the existing
[`M0 baseline`](../progress/2026-09-07-m0-semantic-world-state.md) and its follow-up work. The
September 7 recording already exists; do not restart that investigation or count
this document revision as a pass. The additions below clarify the acceptance
contract for the ongoing work, rather than prescribe a separate calibration fix.

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

### Current approach and progress

P0 remains in progress and M0 has not passed. The
[latest review](../progress/2026-09-07-m0-review.md) separates the recorded evidence
from the calibration work still to do. The gimbal measurement has passed its
held-out gate at tilt zero and pan -20 to +20 degrees when every placement finishes
from the ascending direction, and the envelope now extends to tilt +20. The
fixed-OAK mount has passed its development fit and its second-distance held-out
check and is deployed; what it still owes is a *cleanly captured* confirmation,
because the confirming set was re-analysed rather than re-photographed after a
correction to the pose fit. That is a loose end on a working measurement and not
an M0 criterion, and it needs no larger printed target. The baseline's historical
pass counts and "before Phase 1" heading are not the current gate: read-only
M1/M2 may proceed.

The agreed target is useful, demonstrated accuracy within a declared operating
envelope. The current 1.5-degree bearing uncertainty is not an accuracy demand on
the hardware. Finite backlash, flex and settling variation may remain; correcting
the bias and representing those limits honestly is a valid outcome.

This phase addresses R-WS-10 (bearing uncertainty), R-WS-11 (geometry), R-WS-12
(background eligibility), R-WS-13 (movement-eligible identity) and R-WS-16
(confirmed capture pose). None is marked settled by this plan revision.

The new [refit report](../progress/2026-09-07-refit-window.md) records observations
stamped from an unconfirmed pose. Before driven acceptance, reproduce and close
that capture gate and prevent the affected evidence from entering association as
valid geometry, preserving its images and provenance. Stationary calibration
against an independent reference can proceed without trusting that map pose.

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

- **Accept a useful envelope:** held-out geometry, range attribution and identity
  checks pass, and runtime eligibility checks enforce the documented limits. M0
  can pass within that scope once every criterion below is met.
- **Narrow and retest:** adequate performance is limited to fewer angles, larger
  objects or better-separated surfaces. Declare that narrower scope before fresh
  acceptance; report excluded and unresolved cases. Rejecting every useful task
  cannot count as a pass.
- **Stop at a physical or measurement limit:** the bounded campaign cannot support
  the useful task. State the remaining error, its evidence and what specific
  reference/hardware change would enable a meaningful next test. M0 remains open;
  read-only M1/M2 continue. No hardware purchase follows automatically.

### Owner preparation and next handoff

The owner printed, measured, mounted and lowered the reference as requested. Both
cameras pass their board-coverage gates.
The mount was then measured at 0.555 m and confirmed at 0.686 m, using a
calibration-only 1920 x 1080 OAK preflight that sees all 54 corners without
changing the normal 640 x 360 colour/depth stream, and it is deployed. The exact
sequence is in the
[P0 camera geometry runbook](../runbooks/p0-gimbal-calibration.md).

**What is asked of the owner now is one board setup, not a purchase.** Both
cameras only see the A4 target well between about 0.55 and 0.69 m, so no third
distance qualifies and the owed clean confirmation is instead a fresh pair at
those same two distances, turned 20 to 30 degrees off face-on. The rover's own
headlights at half brightness make that independent of daylight. No calibration
jig, new sensor, larger print or attempt to remove the measured backlash is
requested. A later driven acceptance run still needs the owner present in the
pre-cleared test area.

### Work

What has landed is recorded where it was measured, not here: the mount
([the mount measurement](../progress/2026-09-07-p0-oak-mount.md)), the
demonstrated pointing envelope and its ascending approach
([tilt +20](../progress/2026-09-07-gimbal-tilt20-passes.md)), the two capture
gates in the deployed path
([the capture gates](../progress/2026-09-07-capture-state-gates.md)), and the
driven evidence for depth attribution and identity
([the acceptance drive](../progress/2026-09-07-m0-acceptance-drive.md)). What is
still ahead:

- **Ask the appearance question twice and refuse a look whose score collapses.**
  A box holding a chair in front of a picture makes the picture resemble an
  entity of chairs, and removing everything but the object's own pixels is what
  exposes that. Measured on the recording: refusing a drop of 0.20 or more
  catches four of four wrong attachments for 25 of 360 correct ones, at 84 ms on
  a 550 ms look. Freeze the threshold in the run manifest before collecting
  acceptance data, because it was chosen after seeing those four. Addresses
  R-WS-13 and criteria 3 and 8.
- **Keep the depth evidence, then test the range remedy.** Sampling the range on
  the object's own pixels rather than its whole box cannot be tested offline at
  all today: the recording never saved a depth map. Save each region's depth
  patch, or the frame's depth map beside its JPEG, before the next drive. The
  range remedy itself also needs a foreground/background pair inside the depth
  camera's 43-degree vertical field, which a framed picture on a wall is not.
- **Refuse or flag a region outside the depth camera's coverage.** It sees the
  central two thirds of the gimbal's frame; count the refusals and carry "never
  ranged" to the entity, so a target that can never be ranged is reported rather
  than silently absent. Addresses criteria 2 and 10.
- **Make a bare patch ineligible as an inspection goal** without excluding real
  floor-level and ceiling-level objects. Addresses R-WS-12 and criterion 9.
- **Demonstrate the confirmed-pose gate on hardware** through one navigation
  restart, with the images retained. Addresses R-WS-16 and criterion 11.
- **Take the clean second mount confirmation**, a fresh pair at 0.555 m and
  0.686 m. Not an M0 criterion; a loose end on a measurement that passed.
- **Measure where the gimbal camera sits relative to the pose SLAM reports**,
  which is the half of R-WS-11 still missing and what keeps absolute height above
  the floor unavailable.
- **Then a second driven acceptance run**, with the same named targets plus
  something at the frame edges to exercise the coverage reporting.

Two rules hold throughout: preserve the recordings as before-change evidence, and
keep ambiguous evidence unresolved rather than lowering a threshold to raise the
placed count.

### Verification

Run the existing `world_state/selftest.py` suite and relevant replay/bench tools.
The acceptance recording must be preserved outside Git with enough metadata to be
replayed by path/manifest.

### Milestone M0: semantic state is safe enough to influence goal selection

Pass when all are true:

1. the normal world-state offline suite passes;
2. within the predeclared operating envelope, a fresh hardware recording contains
   usable OAK ranges where expected, and annotated targets confirm that ranges
   belong to the intended objects; unsupported/ambiguous patches are refused and
   counted separately;
3. independent review finds **zero known incorrect high-confidence entity merges**
   in an acceptance sample of at least 50 association decisions; ambiguous cases
   may remain unresolved;
4. range-assisted replay does not increase false high-confidence merges relative to
   bearing-only replay and demonstrably rejects at least one previously plausible
   false crossing or equivalent constructed/replayed case;
5. map clear/map-session behaviour still keeps old coordinates from being treated
   as current placement;
6. a concise baseline report records duplicate, unresolved and false-merge counts.

The baseline predates these clarified gates. M0 also requires:

7. camera-to-rover geometry and its residual uncertainty meet predeclared
   task-derived tolerances within the accepted envelope on held-out trials;
   mechanical repeatability need not improve beyond what those tasks require;
8. review covers every association confidence band eligible to influence movement,
   with zero known incorrect movement-eligible associations in the acceptance
   sample; appearance similarity alone is not calibrated identity confidence;
9. floor/background patches are not eligible as object-inspection goals. Deliberate
   geometric coverage of a floor region remains a distinct goal type;
10. the tested envelope supports the predeclared useful inspection cases, and the
    deployed capture/goal path refuses unsupported conditions. Record coverage and
    unresolved counts so abstaining from everything cannot satisfy M0;
11. unconfirmed map poses cannot give observations usable directions; a later pose
    confirmation does not retroactively validate bearings recorded before it.
    Replay and hardware restart/refit checks demonstrate this with images retained
    and affected observations withheld from geometric association (R-WS-16).

These gates remain part of the existing P0 work. Until they pass, semantic motion
stays disabled; read-only M1/M2 development can continue.

If a 50-decision acceptance sample cannot be obtained from one room/run, accumulate
it across multiple runs without reusing the same physical association as multiple
independent decisions.

## Phase 1 -- episodic memory and read-only autonomy telemetry

### Purpose

Create the evidence trail before creating an executive that can act.

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
  record has been proved against real events; it has not been proved against a
  decision, because there are none to record.
- **The recorder is not a service.** It is run by hand for as long as somebody
  wants a recording. Nothing starts it at boot, so a rover left alone records
  nothing, and retention is not run on a schedule either.
- **Nothing tells the record about a merge or a split.** The alias table exists
  and is tested; the world state does not call it, so identity changes are
  recorded only when something puts them there.
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

### Candidate goal types

For M2, implement:

- `explore_frontier` -- reachable unknown map boundary;
- `improve_geometry` -- another bearing/range likely to improve placement;

Add the following when their evidence and outcome tests exist, with shadow
acceptance before enabling each type for movement:

- `inspect_uncertain_entity` -- explicit semantic gaps and claims from M4;
- `revisit_stale_entity` -- knowledge old relative to expected mobility;
- `investigate_change` -- current evidence conflicts with a prior stable belief;
- `search_for_missing_entity` -- bounded search for something expected but absent.

The last three depend on M5 temporal/visibility semantics. M2 may test their schema
with fixtures, but does not require pretending that the live rover has those
semantics. Skill-practice goals wait until a skill metric exists.

Each candidate contains:

```text
id
type
target/evidence IDs
expected observation/action
estimated travel/time/energy
risk class
expected information gain or uncertainty reduction
purpose relevance
hard constraints
score terms
```

### Curiosity scorer

Implement the score from the architecture as deterministic code. Log every term
and the configuration version used to compute it.

Start with purpose-weighted useful knowledge gain minus time, travel, energy and
switching costs. Specify units/scales and conservative handling of missing estimates.
Novelty and uncertainty inform the gain estimate rather than earning duplicate
rewards. Idle has zero utility; a safe candidate must still clear the configured
minimum worthwhile gain. Test cooldowns, bounded retries and switching costs so an
unresolvable gap or small score fluctuation cannot keep the rover busy indefinitely.

Hard vetoes run before scoring. Initial vetoes should include at least:

- outside configured safe/geofenced region if one is active;
- no known reachable pose for a movement-requiring goal;
- battery below autonomy threshold;
- autonomy disabled or manually stopped;
- navigation/world-state service unhealthy;
- candidate requiring unavailable hardware;
- candidate requiring unsupported drop/edge assumptions;
- semantic candidate outside the validated calibration envelope or with unknown
  approach/settling state required by that envelope.

### Scenario harness

Create a small deterministic scenario format that supplies a synthetic map/world
summary and expected candidate properties. It should test score ordering without
needing ROS or a cloud model.

Include ambiguous cases where **no** candidate is acceptable.

### Milestone M2: the rover can explain what it would investigate next

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
6. a one-hour rover shadow run emits candidate decisions but makes **zero autonomy
   movement calls**;
7. the console or log can answer in one concise record: what it wanted to do, why,
   estimated cost, and why higher-scoring-but-vetoed options were refused.

## Phase 3 -- bounded autonomous execution using only existing operations

### Purpose

Close the first real loop without yet learning new skills or semantic viewpoint
strategies.

### Initial action set

Only wrap operations already bounded and understood by the rover, for example:

- existing frontier `explore`;
- stop;
- move through Nav2 to an already validated safe goal;
- `world_inspect` or an equivalent existing inspection path;
- camera/gimbal actions needed by that inspection.

Do not expose arbitrary `run_script`/`start_script` as an autonomy primitive.

### Executive state machine

A minimal state machine is enough:

```text
IDLE -> SELECT -> PLAN -> EXECUTE -> EVALUATE -> IDLE
                         |          |
                         +-> ABORT <-+
```

A run has hard budgets:

- maximum wall-clock duration;
- maximum distance/travel goal count;
- minimum battery reserve;
- maximum consecutive failures;
- human stop latch.

A stop latch remains set until an explicit human re-enable; the executive must not
interpret "idle" after a stop as permission to choose another goal.

### Daemon-enforced movement permission

Before any M3 movement, implement a short-lived permission owned by the daemon and
renewed by the executive. The daemon enforces expiry and cumulative run budgets
independently of executive health, including during background exploration and
nested actions. Granting or renewing permission never clears a human stop latch.
Restart defaults to autonomy disabled and invalidates earlier permissions.

Human stop has highest priority. Manual takeover cancels autonomy and requires
explicit re-enable before autonomy can resume. Voice submits goals through the same
arbitration; it cannot silently unstop the rover. Revalidate permission, map identity,
pose validity, boundary and hardware requirements at dispatch and monitor relevant
conditions during execution. Reject stale or duplicate action requests using stable
episode/action IDs. A healthy Nav2 process must not keep an orphaned autonomy run
moving after the executive fails.

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
10. stop and takeover trials meet predeclared physical stopping time/distance limits
    at the permitted speeds; an acknowledged request alone is not a pass;
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
| M3 | after M0/M1/M2 and control tests; daemon-enforced bounded operations in supervised safe area |
| M4-M5 | bounded semantic inspection/revisit in same safe area |
| M6-M7 | validated interpreter; candidates only in supervised trials after offline gates, verified skills eligible for autonomous reuse |
| M8-M9 | executive chooses practice/reflection goals; same physical boundary |
| M10+ | learned ranking/prediction only; hard constraints unchanged |

A change that adds a deployed autonomy component will eventually need a
`deploy/manifest.json` entry and host verification. Documentation-only commits do
not require deployment.

## Stop conditions and rollback

Pause expansion of autonomy and investigate if any of these occurs:

- an incorrect movement-eligible world-state association causes a movement decision;
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
| M0 (P0 in progress) | useful semantic goals within measured operating limits | bounded calibration + held-out alignment/identity + envelope refusals + confirmed-pose capture |
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
7. inspect the resulting choices; M3 movement remains blocked until M0, M1, M2 and
   offline daemon-enforced control tests pass, then begin with supervised physical
   stop/failure trials. Starting read-only work does not waive P0.

At the end of that slice the project will already answer a useful empirical
question: **given what the rover actually knows today, does an explicit curiosity
objective choose sensible things to investigate?** If the answer is no, it can be
fixed in replay without having moved the robot under an unproven executive.
