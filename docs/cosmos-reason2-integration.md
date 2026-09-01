# Cosmos physical reasoning on the rover

_Status: design recommendation, 2026-09-01._

## Decision

Add a **local physical-reasoning sidecar** on the Jetson Orin Nano, initially
using **NVIDIA Cosmos Reason 2 2B**, but do not make it another navigator and do
not replace the current Alibaba Qwen Omni voice agent with it.

The recommended split is:

```text
human
  |
  | speech / conversation
  v
Alibaba Qwen Omni
  |                         current role stays here
  | high-level rover tools
  v
rover agent / daemon boundary
  |
  +-----------------------+--------------------------+
  |                       |                          |
  | scene question        | semantic target          | ordinary move
  v                       v                          v
Cosmos Reason 2 2B    world state / targets      existing nav bridge
local on Orin             registry                    |
  |                       |                           v
  | observation +         |                     SLAM Toolbox + Nav2
  | recommended action    |                           |
  +-----------+-----------+                           v
              |                                    wheels
              v
        action validator
              |
              +---- only validated semantic actions reach navigation
```

In one sentence:

> **Qwen talks to the user, Cosmos reasons about the physical scene, the world
> state gives both models stable names for things, and Nav2 remains the only
> component that decides how the rover physically gets somewhere.**

This is an incremental extension of the system that exists today, not a second
robot stack. In particular:

- keep `slam_toolbox` as the authoritative mapper;
- keep Nav2 as the planner/controller/recovery stack;
- keep the measured rover-specific navigation checks in `ros_nav/`;
- keep the current `explore` implementation as the default autonomous explorer;
- keep `rover_daemon` as the single model-facing hardware/navigation tool
  boundary;
- keep Qwen Omni for microphone, conversation, speech and general tool use;
- add Cosmos only where physical visual reasoning is useful;
- add a small world-state/target registry rather than an LLM scratchpad or a
  heavyweight robotics knowledge framework.

## Why this shape fits the repository

The rover already has most of the components a generic proposal would suggest
adding.

Current navigation is:

```text
driver-board odometry + IMU       D500 lidar
             |                        |
             +-----------+------------+
                         v
                    SLAM Toolbox
                         |
                        /map
                         |
                         v
                       Nav2
                         |
                         v
                  ros_nav/nav_bridge.py
                         |
                         v
                    rover_daemon
                         |
                         v
               browser / Qwen Omni
```

The current exploration implementation is also already rover-specific and
measured. `ros_nav/frontier.py` deliberately separates frontier detection and
ranking from body-fit and Nav2 path validation; `nav_bridge.py` verifies a chosen
goal before it moves. This is preferable here to replacing it with
`explore_lite`, which would publish directly to Nav2 and bypass checks the rover
has acquired through real failures.

RTAB-Map was also already evaluated and removed as the mapper. Nothing about a
Cosmos integration requires reopening that decision. If RGB/depth becomes a
proper ROS input later, visual localization can be evaluated separately; Cosmos
does not depend on RTAB-Map.

## What Cosmos should do

Cosmos Reason 2 is useful here as a **deliberative physical-world reasoner**. It
can be asked questions such as:

- what objects and traversable-looking regions are visible;
- which doorway or frontier is most relevant to a task;
- whether the current view appears to contain the requested object;
- which of several *already generated and validated candidate targets* is the
  most sensible next place to inspect;
- what additional camera view would reduce ambiguity;
- whether a short sequence of observations appears consistent with entering a
  room, approaching an object, or failing to find it.

NVIDIA describes Reason 2 as a physical-AI VLM with spatial/temporal reasoning,
2D/3D point localization and bounding-box output, and embodied reasoning about
what action might come next:

- <https://docs.nvidia.com/cosmos/latest/reason2/index.html>
- <https://docs.nvidia.com/cosmos/latest/reason2/reference.html>

That makes it a good fit for **semantic selection and interpretation**, not for a
100 Hz or even 10 Hz wheel-control loop.

### Cosmos should not drive the rover directly

Do not expose any of these to Cosmos:

```text
/cmd_vel
left/right motor power
raw UART/CAN/GPIO
arbitrary ROS topic publication
shell execution
raw map-frame x/y as a free-form model choice
```

The model is not an embodiment-specific rover policy. A visually plausible
answer such as "move 0.6 m forward and turn 23 degrees" is not a substitute for
costmaps, the real footprint, collision checking, the planner, or the rover's
measured actuator envelope.

The safe control chain remains:

```text
Cosmos recommendation
        |
        v
validate target handle and current generation
        |
        v
existing body-fit / planner checks
        |
        v
Nav2 action
        |
        v
DWB today / MPPI only if separately benchmarked
        |
        v
motors
```

A later Cosmos 3 action/policy experiment can use the same boundary; it should
not be allowed to collapse it.

## Keep Qwen Omni

Cosmos should initially **complement**, not replace, the current hosted Qwen Omni
session.

Qwen currently supplies:

- realtime microphone input;
- conversational state;
- general rover tool choice;
- streaming speech output;
- the user-facing personality and turn-taking path.

Cosmos does not provide a reason to rebuild that working path. Instead, give the
Qwen-side agent one high-level capability such as:

```text
reason_about_scene(question, objective?)
```

or, if narrower tools prove easier for the model to choose reliably:

```text
inspect_scene(question)
locate_visible_object(description)
choose_frontier(objective, frontier_ids)
```

These are **Qwen tools**, but they do not execute movement. They capture or reuse
a current image, query local Cosmos, and return a concise structured result.
Qwen may then call the normal rover tools.

This also leaves a clean future path to an entirely local voice stack without
coupling that decision to navigation or semantic memory.

## Run Cosmos as a sidecar, not inside the daemon

The first implementation should be a separate local inference process on the
Orin, listening only on loopback.

```text
Cosmos model files
       |
       v
llama.cpp server, 127.0.0.1:<local port>
       ^
       |
cosmos_client.py
       ^
       |
agent / rover daemon integration
```

Reasons to keep it separate:

1. `rover_daemon` owns safety-critical hardware and already has a good reason to
   stay small and responsive.
2. A CUDA/model failure must not take the UART owner or STOP path down with it.
3. The model can be restarted, upgraded or replaced independently.
4. Memory pressure and inference latency are then observable as a separate
   process.
5. Cosmos 3 or another VLM can later replace Reason 2 behind the same client
   interface.

### Orin Nano 8 GB runtime

Do **not** use the canonical full Transformers/vLLM Reason 2 recipe as the first
Orin Nano implementation. NVIDIA's main Reason 2 documentation lists much larger
memory requirements for that path. NVIDIA has separately demonstrated the 2B
model on an **Orin Nano 8 GB** by serving a 4-bit GGUF build with `llama.cpp`;
in that example the VLM footprint dropped from about 6.6 GB to about 2.2 GB, and
the complete VLM + speech + TTS + robot stack used about 4.5 GB of roughly
7.6 GB usable RAM:

- <https://developer.nvidia.com/blog/maximizing-memory-efficiency-to-run-bigger-models-on-nvidia-jetson/>

The starting deployment should therefore be:

- headless Orin where practical;
- Cosmos Reason 2 2B;
- Q4 GGUF;
- NVIDIA's Jetson `llama.cpp` container/runtime;
- one request at a time;
- bounded image resolution and context;
- no model inference in a navigation control callback;
- explicit monitoring of total RAM and Nav2/SLAM scheduling latency.

The model should be invoked **event driven**, for example on:

```text
user asks a visual question
goal reached
new room / materially changed view
object-search decision
frontier-selection decision
navigation failure requiring semantic reconsideration
explicit inspect/look request
```

not once per camera frame.

## Treat Reason 2 as replaceable from day one

NVIDIA now marks the Cosmos Reason 2 repository as maintenance-only and directs
new development toward Cosmos 3:

- <https://github.com/nvidia-cosmos/cosmos-reason2>
- <https://docs.nvidia.com/cosmos/latest/cosmos3/quickstart_guide.html>

Reason 2 2B is still a useful first rover model because NVIDIA has demonstrated a
workable Orin Nano 8 GB deployment. The code should nevertheless depend on an
interface such as:

```python
class PhysicalReasoner:
    def inspect(self, image, prompt, context): ...
```

not on Reason-2-specific response classes throughout the application.

That makes the model choice a benchmark result rather than an architectural
commitment.

## Do not use an LLM scratchpad as robot state

The rover needs memory, but the authoritative memory should be a **small typed
world-state component**, not hidden model reasoning and not a growing prompt.

Use three kinds of state:

### 1. Navigation state: existing ROS/Nav2 ownership

Examples:

```text
current pose
occupancy grid
costmaps
active goal
planner/controller state
recovery state
```

These remain authoritative in SLAM Toolbox/Nav2. Do not duplicate them into a
semantic database except for references or snapshots.

### 2. Ephemeral semantic/navigation handles

Examples:

```text
frontier:g184:0
frontier:g184:1
visible:frame921:2
```

These are valid only against the map/frame generation that created them.

Keep them in RAM. An old frontier must fail closed rather than silently resolve
to the nearest new frontier.

### 3. Persistent semantic/task memory

Examples:

```text
place:kitchen
place:dock
object:17 = black backpack, last seen near place:office
task:42 = find the red toolbox; kitchen already searched
```

Persist these in SQLite once they exist. This is enough for the likely scale of
one rover in a house and keeps state inspectable with ordinary tools.

Do not introduce KnowRob, ROSPlan, a vector database, LangGraph persistence or a
separate knowledge graph at this stage. They solve larger problems than the
current rover has.

## Recommended new component: `world_state`

Author a thin component rather than importing a framework. It should be a registry
and resolver, **not a planner**.

A reasonable repository shape is:

```text
ros_nav/
  frontier.py                 existing, stays stateless
  frontier_registry.py        NEW: current-generation handles
  target_registry.py          NEW: place/object target handles
  world_store.py              NEW: small SQLite persistence layer
  nav_bridge.py               existing, gains internal ops

rover_daemon/
  cosmos_client.py            NEW: physical-reasoner client/adaptor
  rover_nav.py                existing, semantic actions call nav bridge
  tool_schemas.py             existing, only carefully chosen tools exposed
```

The exact filenames can change during implementation; the important boundary is
that **frontier geometry stays beside the current map/Nav2 stack**, while
model-facing semantics stay above it.

### Why frontier state belongs on the navigation side

`frontier.py` already receives the occupancy grid and produces candidates. The
thing that assigns an ID should therefore sit next to it, where it can also know
which map revision produced the candidate.

Do not copy frontier poses into a long-lived model database and later trust them.
The map changes as SLAM closes loops and discovers space.

A candidate record can look like:

```json
{
  "id": "frontier:g184:2",
  "generation": 184,
  "map_pose": [7.8, 5.9, 1.57],
  "route_distance_m": 8.4,
  "frontier_size_m": 2.1,
  "score": 4.2,
  "status": "active"
}
```

The **model-facing** representation should normally omit `map_pose`:

```json
{
  "id": "frontier:g184:2",
  "distance_m": 8.4,
  "size_m": 2.1,
  "description": "open boundary beyond the doorway ahead"
}
```

The description can initially be geometric and later be enriched by Cosmos.

## Frontier IDs and exploration

Keep the existing zero-semantics autonomous explorer:

```text
explore(minutes)
```

It is already measured, bounded, stoppable and useful when the desired task is
simply "finish mapping this place".

Add semantic frontier selection **beside it**, not instead of it.

### Proposed internal operations

```text
list_frontiers()
explore_frontier(frontier_id)
```

`list_frontiers()`:

1. reads the current occupancy grid;
2. runs the existing frontier algorithm;
3. applies the same reachability/ranking logic used today;
4. assigns IDs tied to the current generation;
5. returns the best few candidates rather than every boundary cell.

`explore_frontier(id)`:

1. rejects a missing or expired generation;
2. rechecks the goal against the **current** map/costmap;
3. applies the existing `goal_fit` body-fit check;
4. asks `ComputePathToPose` as the current explorer does;
5. runs the same Nav2 goal path and existing recovery/stall logic;
6. invalidates the old frontier generation after the map materially changes;
7. returns a concrete outcome.

This means Cosmos can answer:

```json
{
  "recommended_action": {
    "kind": "explore_frontier",
    "target": "frontier:g184:2"
  },
  "reason": "This frontier is through the only doorway that appears to lead to an unsearched room."
}
```

without ever owning a map coordinate.

### Do not make IDs stable across maps

A frontier is not a landmark. It is a boundary of ignorance, and successful
exploration destroys it.

A model calling an old handle should get an explicit result such as:

```json
{
  "ok": false,
  "reason": "frontier_expired",
  "refresh": true
}
```

and be made to choose from the current list.

## `navigate_to(target)`

A semantic `navigate_to()` is useful, but it should be a **resolver above the
existing navigation path**, not another movement implementation.

Examples:

```text
navigate_to("place:dock")
navigate_to("place:kitchen")
navigate_to("object:17")
```

The resolution chain is:

```text
target handle
     |
     v
world_state.resolve()
     |
     +-- place -> stored observation/entry pose
     |
     +-- object -> current/last trusted map pose + approach policy
     |
     v
safe approach pose
     |
     v
existing Nav2 NavigateToPose path
```

The world-state component never sends wheel commands.

### Keep the existing tools too

Do not immediately remove `drive_to` or `drive_to_map_point`.

`drive_to_map_point` in particular has a good safety property: the model points at
**the map image it is actually looking at**, and the daemon resolves that picture
coordinate through the pose captured when the map was rendered. That is much
safer than asking a model to invent map-frame metres.

`navigate_to(target)` solves a different problem: returning to a **named,
registered semantic entity** later.

## Object IDs should come later than frontier IDs

Frontier IDs can be implemented immediately because the authoritative geometry
already exists.

Persistent object navigation should wait until the rover can establish a
trustworthy object position in the map frame.

A Cosmos bounding box is useful semantic grounding, but it is not a metric
navigation goal. The desired future pipeline is:

```text
camera image
    |
    v
Cosmos: object label + image bbox / point
    |
    v
trusted depth measurement
    |
    v
camera intrinsics -> 3D camera-frame point
    |
    v
TF -> map frame
    |
    v
object registry
    |
    v
calculate accessible stand-off / approach pose
    |
    v
Nav2
```

Until a depth camera and its transforms are part of the running stack, Cosmos can
say **"the requested backpack is visible on the right"** but should not create an
`object:*` map position from monocular visual guessing.

This is the boundary that prevents semantic perception from becoming invented
geometry.

## Cosmos response contract

Do not make downstream code parse prose if it can avoid it. Ask Cosmos for a
small JSON application schema and validate it before use.

For example:

```json
{
  "summary": "Hallway with an open doorway on the right; no red toolbox visible.",
  "observations": [
    {
      "label": "doorway",
      "bbox_norm": [0.62, 0.18, 0.94, 0.89]
    }
  ],
  "recommended_action": {
    "kind": "explore_frontier",
    "target": "frontier:g184:2"
  },
  "reason": "The right doorway is the most likely route to an unsearched room."
}
```

The allowed action enum should be narrow, for example:

```text
none
inspect_again
look_at
explore_frontier
navigate_to
```

Treat malformed JSON, an unknown action, an unknown target, or an expired target
as **no action**.

Do not make a model-supplied numeric `confidence` a safety gate. If stored at all,
it is useful for diagnostics/ranking only; it is not a calibrated collision or
localization probability.

## Action validator

All Cosmos recommendations pass through deterministic checks before movement.

For `explore_frontier`:

```text
known handle?
current generation?
still a frontier?
body fits?
Nav2 can plan?
no other move owns the mutex?
not stopped/cancelled?
```

For `navigate_to`:

```text
known semantic target?
position still trusted / not too old for target type?
valid stand-off pose?
body fits?
Nav2 can plan?
no other move owns the mutex?
```

The validator should call the same functions the current explorer and
`drive_to` use rather than reimplementing them.

A refusal is a normal tool result, not an exception that the model is encouraged
to explain away.

## Task memory

A task record is useful once Cosmos starts choosing where to inspect.

Minimal example:

```json
{
  "id": "task:42",
  "objective": "find the red toolbox",
  "state": "searching",
  "visited": ["place:kitchen", "place:hallway"],
  "rejected_objects": ["object:9"],
  "last_action": "explore_frontier",
  "last_result": "reached"
}
```

This should be application data, not a saved chain of thought. On each Cosmos
call, construct a small fresh context from authoritative state:

```text
objective
current robot/place state
places already searched
current camera image
current candidate frontiers/targets
last relevant action result
```

That avoids stale prompt history becoming a second world model.

## Proposed tool surface

Keep the current rover tools as the foundation. Add only tools that provide a
clear semantic capability.

### Model-facing additions worth considering

```text
reason_about_scene(question, objective?)
get_known_targets(query?)
navigate_to(target_id)
```

Potentially expose semantic frontier selection as either:

```text
get_frontiers()
explore_frontier(frontier_id)
```

or hide both behind a single agent operation if tests show Qwen handles the extra
choice poorly.

### Keep internal

These should be daemon/nav implementation operations, not necessarily Qwen tools:

```text
resolve_target(id)
register_frontiers(...)
expire_frontier_generation(...)
register_object(...)
update_object_pose(...)
validate_target(...)
compute_approach_pose(...)
```

### Never model-facing

```text
raw /cmd_vel
motor commands
raw map-frame coordinate navigation
SQLite writes
frontier ID creation
TF manipulation
map revision manipulation
arbitrary ROS calls
```

## Off-the-shelf versus authored code

The preferred split is:

| Need | Use | Author? |
|---|---|---:|
| 2D mapping / loop closure | current SLAM Toolbox | no |
| planning/control/recovery | current Nav2 | no |
| rover tool boundary | current `rover_daemon` | extend only |
| frontier detection/ranking | current `ros_nav/frontier.py` | extend only |
| frontier route/body validation | current nav bridge + `goal_fit` + Nav2 | no new planner |
| voice/conversation | current Qwen Omni path | no |
| physical scene reasoning | Cosmos local sidecar | thin adaptor |
| frontier handle lifecycle | small registry | **yes** |
| persistent place/object/task state | SQLite-backed registry | **yes** |
| semantic target -> existing Nav2 goal | thin resolver | **yes** |
| ontology/knowledge graph | none initially | no |

The custom portion is deliberately small because it is the part that is specific
to this rover's agent semantics. Everything safety- or geometry-critical remains
in the existing measured stack.

## Recommended implementation phases

### Phase 1 — Cosmos sidecar, advisory only

Goal: establish whether Reason 2 adds enough value to justify running it.

1. Add the quantized `llama.cpp` sidecar on the Orin.
2. Add `cosmos_client.py` behind a generic `PhysicalReasoner` interface.
3. Feed it still images from the existing gimbal camera path.
4. Ask for structured descriptions, grounding and next-action recommendations.
5. Do **not** let a recommendation execute movement.
6. Record latency, peak RAM, temperature/power if convenient, and whether SLAM
   or Nav2 timing is affected while inference runs.
7. Compare the same saved scenes against Qwen's own vision results.

Acceptance criterion: Cosmos is measurably better at the physical/semantic tasks
we care about and does not destabilize the running navigation stack.

### Phase 2 — semantic frontier selection

Goal: make Cosmos useful for task-directed exploration without giving it motion
control.

1. Refactor the current frontier calculation so the best candidates can be
   listed as well as automatically chosen.
2. Add transient generation-bound frontier IDs.
3. Add `list_frontiers` and `explore_frontier` internal nav operations.
4. Keep existing `explore` unchanged as the non-semantic baseline.
5. Let Cosmos choose **only among those candidate IDs**.
6. Validate and execute the selected ID through the current checks.
7. Compare task completion against the current frontier score alone.

Example benchmark tasks:

```text
find a doorway into another room
find a desk area
find a black backpack placed in one of N visible/searchable rooms
inspect likely human-occupied areas first
```

Useful measurements:

```text
task success
metres driven
time to target
frontiers visited
unnecessary room visits
Cosmos calls
total model latency
navigation failures
```

### Phase 3 — persistent places and task memory

Goal: let the rover return to named places and avoid repeatedly searching the
same area.

1. Add SQLite `world_store`.
2. Add `place:*` identifiers first; these can be manually registered from known
   map points or map-image selections.
3. Add task state: objective, searched places, last outcomes.
4. Add `navigate_to(place:...)` resolver.
5. Keep geometry in map/Nav2 and semantic metadata in the store.

This phase does not require object depth yet.

### Phase 4 — metric semantic objects

Goal: support `navigate_to(object:...)`.

Only start this when the deployed camera stack provides trustworthy depth plus
camera-to-map transforms.

1. Cosmos identifies/grounds the object in the image.
2. Depth supplies metric range.
3. Intrinsics + TF produce the map-frame position.
4. Association logic decides whether this is a new or existing object.
5. The object registry stores pose, age/source and semantic labels.
6. Navigation generates a safe stand-off pose rather than driving into the
   object's literal coordinate.

### Phase 5 — decide whether Qwen still earns its place

Do this only after the local physical-reasoning path is proven. Replacing Qwen
is a voice/conversation architecture decision, not a prerequisite for Cosmos.

Possible later local pipeline:

```text
STT -> small general agent -> PhysicalReasoner -> rover tools -> local TTS
```

NVIDIA's Orin Nano memory-efficiency example demonstrates that a local VLM + STT
+ TTS combination can fit in 8 GB with careful runtimes, but that should be
benchmarked against the current Qwen experience before replacing it.

## Tests to require before model-driven movement

The first semantic movement path should have deterministic tests for:

- stale frontier ID rejected;
- made-up frontier ID rejected;
- frontier remapped while Cosmos is thinking;
- body no longer fits at selected goal;
- planner refuses selected goal;
- STOP while Cosmos inference is in progress;
- STOP between recommendation and execution;
- another move owns the mutex;
- Cosmos server unavailable/OOM/restarting;
- malformed/non-JSON response;
- valid JSON containing an unrecognised action;
- valid action containing an unrecognised target;
- no camera frame / frame too old;
- task resumes after a model failure without inventing completed actions.

The safe failure for every one is **no new movement**.

The current `frontier.py` style is worth retaining: keep the frontier algorithm,
registry and target-resolution arithmetic usable in offline self-tests without a
ROS installation, and test them against recorded map fixtures where possible.

## What not to add yet

Do not add these as part of the first Cosmos integration:

- RTAB-Map merely because Cosmos is visual;
- a new ROS navigation framework;
- `explore_lite` merely to obtain frontier IDs;
- KnowRob or a general ontology server;
- a vector database for dozens of entities;
- a second tool-execution agent competing with Qwen;
- continuous VLM inference on the camera feed;
- VLM-generated metric navigation coordinates;
- direct VLM motor actions;
- persistent storage of hidden chain-of-thought reasoning.

Each can be revisited if a measured requirement appears.

## Target system structure

After phases 1–3 the preferred structure is:

```text
                              USER
                               |
                         speech / text
                               |
                               v
                    Alibaba Qwen Omni
                    conversation + tools
                               |
                               v
                  drive_web / voice bridge
                               |
                               v
                         rover_daemon
                 single model-facing boundary
                     /         |          \
                    /          |           \
                   v           v            v
           PhysicalReasoner  world state   existing direct tools
             adaptor          facade       lights/gimbal/battery
                   |           |
                   v           |
          Cosmos Reason 2B     |
          llama.cpp, local     |
                               |
                               v
                         ros_nav bridge
                    /          |           \
                   /           |            \
                  v            v             v
          frontier registry  target       existing explore
          + frontier.py      resolver       baseline
                  \            |             /
                   +-----------+------------+
                               |
                         action validator
                               |
                               v
                              Nav2
                               |
                         DWB / recoveries
                               |
                               v
                             wheels

                  slam_toolbox remains authoritative
                  for map -> odom and occupancy map
```

The important properties are:

1. **one movement authority** — Nav2;
2. **one model-facing rover boundary** — the daemon;
3. **one authoritative source for geometry** — SLAM/Nav2 state;
4. **one small semantic registry** — typed handles and task facts;
5. **Cosmos is advisory and replaceable**;
6. **Qwen remains conversational, not the geometric source of truth**;
7. **a model can choose names, never invent the coordinates behind them**.

## Top recommendation

Implement **Phase 1 and Phase 2 first**.

That gives the project the most interesting capability — task-directed semantic
exploration — with very little new authoritative state:

```text
current image + task + current frontier candidates
                         |
                         v
                     Cosmos
                         |
                 choose frontier ID
                         |
                         v
               deterministic validator
                         |
                         v
                   current Nav2 path
```

Only add persistent `place:*`, `object:*` and task memory once a real use case
needs them. In particular, **do not build object navigation before there is a
trusted metric depth/TF path**.

This keeps the first experiment small enough to answer the important question:

> Does a physical-reasoning VLM choose meaningfully better places for this rover
> to inspect than the existing geometric frontier score and Qwen vision alone?

If the answer is no, the rover has gained a benchmark and a clean inference
sidecar that can be removed. If the answer is yes, the same interfaces naturally
extend into persistent semantic navigation without giving a language model
control of the wheels.
