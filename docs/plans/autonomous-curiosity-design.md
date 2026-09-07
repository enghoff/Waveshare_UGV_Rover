# Curiosity-driven autonomy and lifelong learning

Status: design proposal; Phase 0 (P0) validation is already in progress. Nothing in
this document gives the autonomy layer movement authority yet. The staged acceptance
plan is in
[the implementation plan](autonomous-curiosity.md).

## Goal

Build a rover that can be left in a known-safe environment and productively occupy
itself: explore unmapped space, notice what it does not know, choose observations
that reduce uncertainty, remember changes, revisit stale knowledge, and gradually
acquire reusable high-level skills from experience.

"Curiosity" here is an operational description, not a claim about subjective
experience. The rover is curious when it chooses an action because the expected
knowledge or competence gain is worth the travel, time, energy and risk.

A successful system should become measurably better acquainted with one particular
environment over days and weeks. It should be able to answer not only "what is
here?" but also questions such as:

- what has changed since yesterday;
- which things have only been seen once or from one direction;
- which areas are still poorly observed even though the floor map is complete;
- where a previously known object was last seen;
- which inspection strategy usually resolves an uncertain object;
- which route or approach tends to fail and under what conditions;
- what useful procedure it has learned from repeated successful behaviour.

The target is deliberately narrower than general autonomous robot learning. The
rover should not learn arbitrary motor control from scratch, rewrite its deployed
software, or decide that a model-generated program is safe because a language
model said so.

## What already exists

This proposal builds on the current boundaries rather than replacing them.

- ROS 2, `slam_toolbox` and Nav2 own mapping, pose, route planning and movement.
- `rover_daemon` owns the physical hardware and remains the only path to motors,
  gimbal, lights and navigation.
- `ros_nav/frontier.py` already provides geometric frontier exploration.
- `world_state/` stores immutable observations separately from the rover's current
  entity hypotheses. An observation retains its source frame, capture time,
  camera, pose, bearing, uncertainty, optional range and appearance vectors.
- Current perception uses YOLOE regions, DINOv2 appearance vectors and SigLIP2
  text-search vectors. Geometry is primary for identity; language is not allowed
  to assign persistent identity merely by naming something.
- `rover_daemon` already exposes bounded navigation, exploration, inspection and
  semantic recall operations.
- `docs/runbooks/scripting.md` provides a useful composition mechanism, but scripts are
  ordinary Python under the `jetson` account. That is process isolation, not a
  sandbox, and therefore is not the representation for autonomously learned
  skills.
- The realtime voice model is currently a conversational/tool-using client of the
  rover. It need not become the always-on executive.

P0 already has a fresh driven recording and a
[`baseline report`](../progress/2026-09-07-m0-semantic-world-state.md). That report does not pass
M0: camera geometry and range-to-object alignment remain unproven, and the review
also found lower-confidence association errors and floor patches treated as objects.
Continue that work under P0; this proposal does not restart it or prescribe a
calibration fix. M0 remains a prerequisite for semantic movement. Read-only episodic
recording and shadow decisions may be developed while P0 is in progress.
The [current review](../progress/2026-09-07-m0-review.md) records the bounded
calibration scope and the additional requirement to withhold usable bearings when
the rover's map pose is unconfirmed. Calibration accuracy cannot compensate for a
wrong observer pose.

## Design principles

### Safety authority stays below cognition

The autonomy layer may request a goal. It does not command wheel PWM, bypass Nav2,
weaken collision checks, or create a new hardware-control path. A learned skill is
only a composition of capabilities already admitted by the daemon.

Before autonomous execution, the daemon must enforce a short-lived permission to
move that the executive renews, together with run-wide time, travel and battery
limits. An executive crash, hang or lost connection must revoke its motion even if
Nav2 is still healthy. These are proposed requirements, not claims about current
daemon behaviour. Human stop and manual takeover pre-empt autonomy; voice may submit
goals but cannot silently restore revoked authority. Explicit human re-enable is
required after a stop or takeover. Map identity, pose validity and applicable safety
conditions are checked again at dispatch and monitored during execution.

The present lidar cannot see steps, drops, table edges or obstacles outside its
scan plane. Until the rover gains independently validated drop/edge sensing,
unsupervised motion is restricted to a physically pre-cleared flat area. No amount
of language-model reasoning turns that sensor limitation into a safe capability.

### Evidence and belief remain different things

Observations are evidence and stay immutable. Entities, attributes, relationships
and hypotheses are the rover's current interpretation of that evidence and may
change.

A statement such as "the sofa is grey" should eventually be represented as a
claim with confidence, provenance and time, not as prose written into the database:

```text
claim
  subject: entity-42
  predicate: colour
  value: grey
  confidence: 0.94
  evidence: [observation-124, observation-131]
  method: visual-attribute-model@version
  last_confirmed: ...
```

A model can propose a claim. It cannot make the claim true merely by producing it.

### Uncertainty is first-class state

Unknown, ambiguous and stale are useful states. They must not be collapsed into a
best guess simply so the database looks complete.

The rover needs to know, for example, that an entity is well placed geometrically
but poorly characterised semantically, or that a previously confident fact is now
stale because the object is movable and has not been seen recently.

### Calibration has a physical limit

M0 means accuracy and uncertainty are demonstrated adequate within declared
operating limits, with unsupported cases refused. It does not require perfect
pointing, identifying every object, or removing every mechanical error.

Separate repeatable bias, physical variation from backlash/flex/settling, and the
uncertainty of the measurement itself. The current 1.5-degree bearing setting is an
assumption to validate, not a hardware specification. The measured difference
between approach directions is not automatically the residual error after a
correction, nor is it a standard deviation.

Use a bounded experiment to establish repeatability, test a simple correction and
measure held-out residuals. The initial useful envelope may restrict pan/tilt,
approach direction, settling, range, object angular size and separation from other
surfaces. Those are measured limits, not promises that the entire gimbal travel
will be usable. Larger residuals may justify honest uncertainty or fewer eligible
goals; they must not make incompatible observations easier to merge.

Reference quality includes observability as well as print scale and repeatability.
A clipped planar target can return a small pixel residual and closely repeated pose
while leaving one rotation weakly constrained. Require enough target coverage, test
the lens model stored on the camera against any simpler runtime model, and repeat
the transform at a different target placement. Treat a model-sensitive fit as
inconclusive even when its reprojection score looks good.

Stop calibration when the agreed experiment budget is spent or improvements cannot
be distinguished from measurement uncertainty. Then accept a useful demonstrated
envelope, narrow and retest it, or name the specific hardware limitation preventing
the task. Expanding the envelope requires fresh validation. The bounded P0 protocol
and its exit decisions are in the development plan.

### Curiosity is scored, not prompted

The executive should not receive a vague instruction to "wander around and be
curious". It should generate candidate goals from explicit knowledge gaps and rank
them with a utility function.

A starting form, evaluated only for candidates that pass hard constraints, is:

```text
utility(goal) =
    purpose_relevance * expected_useful_knowledge_gain
  - w_energy      * energy_cost
  - w_travel      * travel_cost
  - w_time        * time_cost
  - switching_cost
```

Novelty, staleness and uncertainty are inputs to the gain estimate, not separate
rewards for the same knowledge gap. Define units, scales and missing-cost handling,
and log every term and its configuration version. Add a competence-progress reward
only once a measured skill metric exists; retain extra terms only when comparisons
show that they improve choices.

Idle has zero utility. A candidate must exceed a configured minimum worthwhile gain;
being safe and reachable is not enough. Charge for switching unfinished goals and
apply bounded retries and cooldowns after unproductive inspections. Resume a cooled
gap when new evidence or changed conditions justify it. Repeated views of the same
unresolvable ambiguity must not manufacture progress.

Weights are configuration, not hidden model behaviour. Hard safety constraints
veto candidates before scoring; a high curiosity score never buys permission to
cross a safety boundary. Candidates outside the validated calibration envelope
are ineligible for semantic movement even when navigation itself can reach them.

### Active perception beats passive description

A conventional visual assistant answers "what is in this image?". An autonomous
robot also asks "what observation should I make next?".

If an object could be a bag or a box from one bearing, the useful action may be to
move to a second reachable viewpoint and look again. If two entities may be the
same physical object, a crossing from a viewpoint with better parallax may be
worth more than another image from the current pose.

The current multi-view world-state geometry is a natural basis for this. The new
layer should choose observations because of the uncertainty they are expected to
resolve, not because movement itself is considered progress.

### Learn procedures before learning weights

For one rover, explicit procedural learning is initially much more valuable and
verifiable than continually fine-tuning a neural policy on a small correlated
stream of physical experience.

Early learning therefore means:

- remembering facts and their reliability;
- learning useful parameter choices;
- learning which observation strategies work;
- discovering repeated successful action patterns;
- proposing reusable composite skills;
- measuring each skill's success, cost and failure modes;
- retrieving and composing verified skills for future goals.

Neural policy or predictive-world-model learning remains possible later, after the
system has accumulated a sufficiently large, well-labelled experience record and
has replay/simulation gates around it.

### Model providers are components, not architecture

A cloud reasoning/VLM model can be useful for semantic interpretation, hypothesis
generation, planning and reflection, but the architecture must not depend on one
provider's conversational state or tool-call idiosyncrasies.

Model calls should therefore have typed inputs and outputs, recorded model/version,
repeatable evaluation cases and a deterministic validation boundary. Qwen, Gemini,
OpenAI or a future local model can then be compared on the same rover tasks.

## Proposed architecture

```text
                            human goals / policies
                                    |
                                    v
+-----------------------------------------------------------------------+
|                         AUTONOMY EXECUTIVE                            |
| candidate goals -> hard constraints -> curiosity score -> selection   |
+------------------------------+----------------------------------------+
                               |
                               v
+-----------------------------------------------------------------------+
|                       PLANNER + SKILL LIBRARY                         |
| verified skills, preconditions, effects, recovery, expected cost      |
+------------------------------+----------------------------------------+
                               |
                     admitted daemon operations
                               v
+-----------------------------------------------------------------------+
|                    ROVER CONTROL / SAFETY BOUNDARY                    |
| rover_daemon -> Nav2 / SLAM / camera / gimbal / hardware             |
+------------------------------+----------------------------------------+
                               |
                               v
+-----------------------------------------------------------------------+
|                        PERCEPTION + MEMORY                            |
| map | observations | entities | claims | episodes | skill outcomes   |
+------------------------------+----------------------------------------+
                               |
                               +-----------------------------+
                               |                             |
                               v                             v
                    active-perception planner        reflection / learning
```

The loops run at different timescales. Motor control remains fast and local.
Navigation runs on its existing timescale. Perception happens on observations.
Goal selection might happen after a goal completes or the world materially changes.
Reflection can happen only after a useful batch of episodes exists. There is no
reason to put a large language model in a continuous control loop.

## Memory model

The rover should distinguish three kinds of memory.

### Semantic memory: what appears to be true

This extends `world_state` rather than replacing it.

Candidate additions include:

- typed attributes with confidence and evidence;
- entity-to-entity relationships such as `near`, `on`, `inside` or `part_of`;
- mobility/change class: static fixture, usually static, movable, transient;
- last-confirmed time and staleness policy;
- explicit knowledge gaps;
- hypotheses that have not met the evidence threshold for becoming claims;
- uncertainty/calibration information rather than only a fused confidence number.

The existing distinction between observation and entity should remain. New
semantics must not weaken the geometry and ambiguity rules already established in
`world_state`.

### Evidence lifetime and map changes

Current map-clear behaviour deletes world-state observations, entities and frames,
and resets entity counters. An episode that keeps only those local IDs would lose
its evidence or later refer to a different entity. Before accepting M1, define
globally unique references (or a durable store-generation namespace), immutable
decision snapshots and an archive of referenced evidence outside the deploy tree.
Entity merges need recorded aliases/history, not dangling references.

The proposed lifelong-memory behaviour separates invalidating current placement
from deleting historical evidence. A map reset must not make old coordinates
actionable, but prior episodes must still resolve their evidence. Explicit user
deletion remains available; record that evidence was deleted and that dependent
episodes can no longer be fully replayed. Document retention and storage budgets
rather than promising unlimited raw-frame retention.

### Episodic memory: what happened

Autonomy needs an append-only record of attempts, including failures:

```text
episode
  trigger
  candidate_goals
  selected_goal
  score_terms
  evidence_used
  plan
  skill_versions
  actions_requested
  action_results
  observations_created
  beliefs_changed
  surprise
  success_criteria
  outcome
  energy/time/travel cost
  model calls and versions
```

This is the material used for replay, evaluation, reflection and later learning.
It should be possible to explain an autonomous decision from stored data without
asking the model that made it to remember why.

Replay proves what follows from recorded inputs. It does not reveal the image from
an unvisited viewpoint or the outcome of an untried action. Alternative procedures
need matching recorded coverage or a simulator validated against hardware; missing
outcomes remain unknown. New supervised trials establish physical performance.

### Procedural memory: what the rover knows how to do

A skill is a versioned, constrained procedure with explicit semantics:

```text
skill: multi_view_inspect
version: 3
parameters:
  entity_id: entity
preconditions:
  - entity_has_approximate_location
  - alternate_viewpoint_reachable
effects:
  - adds_observations(entity_id)
  - may_reduce_semantic_uncertainty(entity_id)
limits:
  max_travel_m: 4
  max_duration_s: 120
success:
  - independently_verified_information_gain >= threshold
other_outcomes:
  - unresolved_no_useful_viewpoint
  - unresolved_budget_exhausted
  - navigation_or_perception_failure
```

The stored representation should be a small behaviour-tree/DSL or similarly
restricted graph, not arbitrary Python. Its instruction set maps to known daemon
operations and bounded control structures. Loops are bounded, deadlines are
mandatory, and unsupported operations fail validation before execution.

The existing scripting API remains useful as an implementation reference and
possibly as an execution backend for hand-authored code, but autonomous skill
creation should not hand model-generated Python to the rover account.

Stopping correctly is distinct from resolving the question. Report all attempts,
including safe but unresolved termination, when evaluating a skill.

## Goal generation

The executive can have several independent candidate generators. A goal enters the
common scorer only after its generator can provide evidence, expected outcome and
constraints.

### Geometric frontier

Use the existing exploration stack when reachable unknown floor boundary remains.
This is the baseline form of information gathering.

### Semantic uncertainty

Examples:

- entity has only one useful observation;
- entity identity is unresolved;
- attribute hypotheses disagree;
- geometry would benefit from greater parallax;
- a relationship is plausible but not observed well enough.

The proposed action is an observation plan, not merely "go look at something".

### Novelty

A newly observed region or appearance that is unlike stored evidence may deserve
inspection. Novelty should decay once the rover has gathered enough evidence; a
bright unusual object must not attract the rover forever.

### Temporal change and staleness

Movable objects become stale sooner than walls. Areas that historically change can
be revisited more often than static ones. Unexpected appearance differences can
create an `investigate_change` candidate rather than immediately creating a new
persistent entity.

### Missing expected entities

If a stable or frequently observed object is absent where expected, the rover may
choose a bounded search. The absence itself remains a hypothesis until enough of
the expected view has actually been observed.

### Competence progress

A candidate can exist because practising a not-yet-reliable skill is expected to
improve it. This is different from repeatedly exercising a skill that is already
mastered or hopeless under current hardware constraints.

The useful signal is learning progress: how much success or prediction error is
improving with experience.

### Human purpose

Curiosity can be purpose-directed. A user policy such as "learn where household
objects tend to be" or "understand every doorway and route" adds utility without
turning the goal into a fixed script. This is preferable to novelty for novelty's
sake when the rover is expected to become useful rather than merely busy.

## Active perception

An observation planner should answer three questions:

1. Which uncertainty or hypothesis is being tested?
2. Which reachable observation would best discriminate the alternatives?
3. Is the expected gain worth the cost and risk?

Initially this can be rule-based and geometrical. Examples include choosing a
second bearing with more parallax, revisiting an entity from the opposite side, or
moving closer only when the expected image scale is currently too small.

The planner should predict its intended gain before moving. The evaluator compares
that prediction with what actually changed. That produces learning data for better
viewpoint selection later.

Small, versioned updates to viewpoint parameters may be evaluated at this stage,
before autonomous skill discovery exists. Compare them on held-out runs with fixed
safety bounds and a frozen baseline; a better parameter choice is already useful
learning without a new skill language.

## Skill lifecycle

A skill has a lifecycle rather than becoming trusted at creation time:

```text
proposed
   |
   v
static validation
   |
   v
replay / simulation
   |
   v
supervised physical trials
   |
   v
verified
   |
   +---- performance regression ----> quarantined
```

A proposal should include:

- the repeated episodes or failure pattern that motivated it;
- a generalised parameterisation rather than copied entity IDs;
- preconditions and effects;
- bounds on movement, duration and resource use;
- a success predicate observable without trusting the proposer;
- known failure/recovery cases;
- the skills/primitives it composes.

Promotion is based on measured trials. A model may propose a skill abstraction but
cannot promote its own proposal.

The first useful learned skills are likely to be perceptual/navigation procedures,
for example:

- `multi_view_inspect(entity)`;
- `search_region_for(description)`;
- `recheck_changed_entity(entity)`;
- `recover_view_after_occlusion(entity)`;
- `inspect_frontier_before_entering(frontier)`;
- `revisit_stale_area(area)`.

## Reflection

A stronger reasoning model can periodically read compacted episodes and propose:

- surprising outcomes worth investigating;
- recurrent failures and candidate explanations;
- repeated action sequences worth abstracting into skills;
- stale assumptions;
- candidate knowledge gaps;
- experiments that could distinguish competing hypotheses.

Reflection output is advisory structured data. Every referenced entity, episode,
skill and observation must resolve to existing stored IDs. Invalid references are
rejected. Any proposed physical experiment re-enters the ordinary candidate-goal
and safety pipeline.

This keeps hallucination out of the authority path while retaining the useful part
of language-model reasoning.

## Predictive world model: later, and initially small

A useful world model need not start as a generative video model. The first learned
predictor can simply estimate outcomes such as:

```text
P(skill outcome | context, skill, parameters)
expected duration
expected travel
expected information gain
expected failure reason
```

Examples:

- a particular doorway is unreliable when approached at a large angle;
- a second viewpoint one metre sideways usually resolves small-object ambiguity;
- a search pattern finds objects more efficiently than random viewpoints;
- a route has more dynamic obstacles at certain times.

Once the episodic store contains enough diverse data, these predictors can be
trained and evaluated on held-out episodes. Only after that is there a reason to
consider learned latent dynamics, video world models, reinforcement learning or
policy fine-tuning.

## Model roles

The design separates tasks that require a foundation model from tasks that do not.

| Function | Preferred initial implementation |
|---|---|
| motor control and collision avoidance | existing deterministic local stack |
| mapping and route planning | existing ROS/Nav2 stack |
| region proposal and visual embeddings | existing local perception |
| entity placement/association | existing geometry-first resolver |
| curiosity scoring | deterministic code |
| hard safety constraints | deterministic code |
| goal execution | verified skill interpreter + daemon |
| semantic attribute extraction | VLM on demand |
| ambiguous scene reasoning | VLM/reasoning model on demand |
| reflection and skill proposals | stronger reasoning model, periodic |
| skill promotion | measured evaluator, never the proposing model |

This also reduces cloud cost. Most autonomous activity should not require an open
realtime voice session. Voice remains a human interface that can insert goals,
policies and questions into the same executive.

## Safety invariants

These are architectural requirements, not tuning preferences.

1. `rover_daemon` remains the physical authority.
2. Nav2 and existing movement checks cannot be bypassed by a learned skill.
3. No autonomous procedure emits raw wheel or gimbal control loops.
4. Every autonomous run has a time, travel and battery budget.
5. A human stop request pre-empts autonomy and prevents immediate self-restart.
6. Model outage or malformed output must fail closed for actions that depend on it;
   fully validated model-independent operations remain available within policy.
7. Unsupported/unknown skill operations are rejected before execution.
8. New skills cannot promote themselves.
9. Semantic uncertainty may cause the rover to gather evidence; it cannot grant
   movement authority to an unvalidated world-state hypothesis.
10. Runtime learning data, model weights, frames and databases stay outside Git.
11. Unsupervised motion remains confined to a pre-cleared flat environment until
    drop/edge sensing is independently validated.
12. Every autonomous decision and physical action is attributable to an episode.
13. Daemon-enforced permission expiry and budgets stop autonomy independently of
    executive health; restart never restores revoked authority automatically.
14. Map changes invalidate placement without silently destroying the evidence used
    by retained episodes; explicit deletion is recorded.

## What success would look like

A mature version should be able to enter an unfamiliar but safe flat environment
and, with no sequence of human waypoints:

1. map reachable floor using the existing geometric explorer;
2. create and resolve semantic observations while retaining ambiguity where the
   evidence is weak;
3. identify which entities or regions deserve another observation;
4. deliberately choose new viewpoints that improve knowledge;
5. stop geometric exploration when the map is complete but continue useful
   semantic investigation;
6. notice and record changes on later passes;
7. choose revisits based on staleness and expected information gain;
8. accumulate episodes of what worked and failed;
9. propose and verify reusable procedural skills from repeated behaviour;
10. demonstrate improved task success or lower cost from learned experience.

Demonstrate these benefits over several days, including restarts, changed objects
and unchanged controls. Compare against frontier-only exploration and scheduled
revisits under equal travel/time budgets. Measure answers to useful questions,
change detection, retained knowledge and inspection cost across all attempts.
More observations or higher confidence alone do not establish lifelong learning.

## Research influences

These are design references, not runtime dependencies:

- Voyager, *An Open-Ended Embodied Agent with Large Language Models* -- automatic
  curriculum, executable skill library and self-verification:
  https://arxiv.org/abs/2305.16291
- Lifelong Robot Library Learning -- experience memory, self-guided exploration and
  growth of a composable robot skill library without gradient-based continual
  training: https://arxiv.org/abs/2406.18746
- H-GRAIL, *A Motivational Architecture for Open-Ended Learning Challenges in
  Robots* -- intrinsic motivation, autonomous goal discovery and skill sequencing
  demonstrated in a physical-robot setting: https://arxiv.org/abs/2506.18454
- *World Model for Robot Learning: A Comprehensive Survey* -- predictive world
  models for planning, policy learning, simulation and evaluation:
  https://arxiv.org/abs/2605.00080

The implementation should borrow mechanisms that survive measurement on this rover,
not reproduce any one research architecture wholesale.
