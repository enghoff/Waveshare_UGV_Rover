# Cosmos physical reasoning and semantic world state

_Status: design recommendation, revised 2026-09-01._

## Decision

Add a **local physical-reasoning sidecar** on the Jetson Orin Nano, initially
using **NVIDIA Cosmos Reason 2 2B**, and use it to build and maintain a small
semantic world model while the rover operates.

Do **not** make Cosmos another navigator and do **not** replace the current
Alibaba Qwen Omni voice agent with it initially.

The intended split is:

```text
human
  |
  | speech / conversation
  v
Alibaba Qwen Omni
  |                         user-facing agent
  | high-level rover tools
  v
rover agent / daemon boundary
  |
  +---------------------+--------------------------+
  |                     |                          |
  | semantic query      | semantic target          | ordinary move
  v                     v                          v
Cosmos Reason 2B   semantic world state       existing nav bridge
local on Orin       RAM + SQLite                    |
  ^                     |                           v
  |                     |                     SLAM Toolbox + Nav2
  |                     |                           |
  +--- event-driven ----+                           v
       scene observer                             wheels
```

The central rule is:

> **Qwen talks to the user, Cosmos reasons about the physical scene, the world
> state stores stable semantic facts and map-bound observations, and Nav2 remains
> the only component that decides how the rover physically gets somewhere.**

This is an incremental extension of the system that exists today:

- keep `slam_toolbox` as the authoritative mapper;
- keep Nav2 as the planner/controller/recovery stack;
- keep the measured rover-specific navigation checks in `ros_nav/`;
- keep the current `explore` implementation as the default geometric explorer;
- keep `rover_daemon` as the model-facing hardware/navigation boundary;
- keep Qwen Omni for microphone, conversation, speech and general tool use;
- add Cosmos for physical visual reasoning and semantic observation;
- add a small typed world-state layer rather than an LLM scratchpad or a
  heavyweight robotics knowledge framework.

## Why this shape fits the repository

The rover already contains most of the infrastructure a generic semantic-robot
architecture would otherwise add.

Current navigation is approximately:

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

The existing frontier explorer is also already rover-specific and measured.
`ros_nav/frontier.py` deliberately separates frontier detection/ranking from
body-fit and Nav2 path validation; the bridge verifies a chosen goal before the
rover moves. Keep this rather than introducing `explore_lite` merely to obtain
frontier IDs.

RTAB-Map was already evaluated and removed as the mapper. Cosmos does not require
reopening that decision. If RGB/depth later justifies a visual-localization
experiment, that remains a separate question from semantic reasoning.

## Cosmos should be engaged automatically, but event driven

Cosmos is **not intended to run only when a human explicitly asks a visual
question**.

The rover should use it automatically at meaningful semantic checkpoints while
it moves through an environment and use those observations to populate/update
world state.

It should also **not** run continuously on every camera frame.

The normal autonomous loop should be:

```text
Nav2 drives
   |
   | no VLM needed during ordinary path following
   v
goal/frontier/observation point reached
   |
   v
rover/camera settles
   |
   +-- capture current image
   +-- current trusted pose
   +-- map session/revision
   +-- current task, if any
   +-- candidate frontiers/targets, if relevant
   |
   v
Cosmos inspection
   |
   +-- semantic observations
   +-- useful object/place candidates
   +-- optional next-action recommendation
   |
   v
world-state association / validation
   |
   +-- transient observation
   +-- update existing entity
   +-- promote salient new entity
   |
   v
next high-level decision
```

Typical automatic triggers:

```text
navigation goal reached
frontier reached
new room or materially changed view
camera deliberately pointed somewhere new
object-search decision
frontier-selection decision
navigation failure requiring semantic reconsideration
explicit user visual question
explicit inspect/look request
```

Do not invoke Cosmos in a Nav2 control callback or at camera frame rate.

Benefits of event-driven use:

- lower RAM/GPU pressure;
- less inference on blurred/transitional views;
- deterministic navigation timing remains independent of VLM latency;
- the semantic database grows even before a user asks about a specific object;
- the rover can use prior observations to answer/search more efficiently later.

## Background semantic mapping versus task-directed perception

There are two related modes.

### Background semantic mapping

When Cosmos is invoked automatically, ask it for **generally useful semantic
information about the current scene**.

For example:

```text
office
- desk
- chair
- monitor
- black backpack near desk
- doorway to hallway
```

A later question such as:

```text
"Where is the black backpack?"
```

can then query existing world state before beginning a fresh search.

The database therefore does **not** need a previous prompt specifically asking
for a backpack before it can know that a backpack was seen.

### Task-directed perception

A user objective increases attention and retention for relevant things.

For example:

```text
objective = "find the blue screwdriver"
```

causes otherwise low-salience screwdriver-like candidates, negative room
searches, and relevant frontiers to be retained for that task.

The task-directed pass can be more detailed than normal background mapping.

## Do not inventory everything

Automatic semantic mapping should have a **salience policy** rather than turning
every visible item into a permanent database row.

Good default persistent candidates include:

- rooms and meaningful areas;
- doors/passages and navigation-relevant landmarks;
- furniture-scale landmarks;
- charging dock;
- distinctive movable objects;
- useful household/work objects such as backpack, toolbox, laptop, parcel,
  bottle, keys, etc.;
- anything explicitly relevant to a current task;
- anything the user explicitly asks the rover to remember.

Low-value/background details can remain transient unless later promoted.

Conceptually:

```text
Cosmos observation
       |
       v
transient observation
       |
       v
salient or task relevant?
      / \
    no   yes
    |     |
 expire   associate with existing entity
              |
              v
        geometry trustworthy?
          /          \
        no            yes
        |              |
 semantic-only     map-bound observation
 memory possible       |
                        v
                   persistent DB
```

## Cosmos should reason; it should not drive

Cosmos is useful as a **deliberative physical-world reasoner** for questions such
as:

- what useful objects and areas are visible;
- whether a requested object appears to be present;
- which doorway/frontier is most relevant to an objective;
- which of several already-generated candidate targets is worth inspecting;
- what camera view would reduce ambiguity;
- whether observations are consistent with entering a room or failing to find
  something there.

Do not expose:

```text
/cmd_vel
left/right motor power
raw UART/CAN/GPIO
arbitrary ROS publication
shell execution
raw map-frame x/y as a free-form model choice
```

The safe movement chain remains:

```text
Cosmos recommendation
        |
        v
validate action + target handle
        |
        v
resolve authoritative map geometry
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

Cosmos chooses **what/where semantically**. Nav2 chooses **how to move there**.

## Keep Qwen Omni initially

Cosmos should complement the current hosted Qwen Omni session.

Qwen currently supplies:

- realtime microphone input;
- conversational state;
- general rover tool choice;
- streaming speech output;
- user-facing turn taking.

Cosmos should not force a rewrite of that working path.

Qwen can access semantic capabilities such as:

```text
reason_about_scene(question, objective?)
get_known_targets(query?)
navigate_to(target_id)
```

However, **automatic semantic observation should not require Qwen to ask for it**.
A small event-driven semantic observer/orchestrator can invoke the local
`PhysicalReasoner` directly after navigation/observation events and update world
state independently of the voice session.

That distinction is important:

```text
Qwen tool call -> Cosmos
```

is one path, but not the only path.

The rover itself can also do:

```text
navigation event -> semantic observer -> Cosmos -> world state
```

## Run Cosmos as a local sidecar

Run the model as a separate process on the Orin, listening only on loopback.

```text
Cosmos model files
       |
       v
llama.cpp server, 127.0.0.1:<local port>
       ^
       |
PhysicalReasoner / cosmos_client.py
       ^
       |
semantic observer + Qwen-facing tools
```

Reasons:

1. `rover_daemon` owns safety-critical hardware and should remain responsive.
2. A CUDA/model failure must not take STOP or the UART owner down with it.
3. The model can be restarted/replaced independently.
4. Memory pressure and latency are observable separately.
5. Cosmos 3 or another VLM can replace Reason 2 behind the same interface.

A generic interface is preferable to Reason-2-specific code throughout the
application:

```python
class PhysicalReasoner:
    def inspect(self, image, prompt, context): ...
```

### Orin Nano 8 GB runtime

Start with:

- Cosmos Reason 2 2B;
- Q4 GGUF;
- NVIDIA's Jetson `llama.cpp` route;
- one inference at a time;
- bounded image resolution/context;
- explicit RAM and Nav2/SLAM latency monitoring.

NVIDIA has demonstrated a quantized Reason 2 2B configuration on Orin Nano 8 GB:

- <https://developer.nvidia.com/blog/maximizing-memory-efficiency-to-run-bigger-models-on-nvidia-jetson/>

NVIDIA now marks Reason 2 maintenance-only and points new development toward
Cosmos 3, so model replacement should be considered normal rather than a rewrite:

- <https://github.com/nvidia-cosmos/cosmos-reason2>
- <https://docs.nvidia.com/cosmos/latest/cosmos3/quickstart_guide.html>

## World state is not an LLM scratchpad

The authoritative robot memory should be a **typed world-state layer**, not model
conversation history or hidden reasoning.

Use four categories of state.

### 1. Navigation state — existing ROS/Nav2 authority

Examples:

```text
current pose
occupancy grid
costmaps
active goal
planner/controller state
recovery state
```

Keep these authoritative in SLAM Toolbox/Nav2.

### 2. Ephemeral semantic/navigation state — RAM

Examples:

```text
frontier:map42:r381:0
frontier:map42:r381:1
visible:frame921:2
current candidate objects
current semantic inspection
```

These may expire in seconds/minutes and should not be treated as permanent facts.

### 3. Persistent semantic entities — SQLite

Examples:

```text
place:kitchen
place:office
place:dock
object:17 = black backpack
object:23 = red toolbox
task:42 = find red toolbox
```

Semantic identity was meant to outlive any single SLAM map. It does not:
everything the store holds is measured in the map's frame, so the console's
map clear takes the world state with it -- see `world_state/README.md`.

### 4. Map-bound observations — SQLite

Metric geometry belongs to a specific SLAM mapping session and observation
revision.

Examples:

```text
object:17 was observed at x/y/z in map_session:42
place:kitchen had an entry/observation pose in map_session:42
```

This separation is fundamental:

> **Semantic entities belong to an environment. Metric locations belong to a
> specific SLAM map/session.**

## Tie geometry explicitly to the SLAM map

Every stored coordinate that can later influence navigation must identify which
SLAM map produced it.

Use at least:

```text
environment_id
map_session_id
map_revision
```

### `environment_id`

Identifies the physical environment, for example:

```text
environment:home
environment:workshop
```

A kitchen in one building must not silently become the kitchen in another.

### `map_session_id`

A new session is created when the SLAM map/pose graph is cleared and rebuilt.

Example:

```text
map_session:41  archived
map_session:42  active
```

Coordinates from session 41 must never be used as session-42 navigation goals
without explicit relocalization/reassociation.

### `map_revision`

Tracks meaningful changes within one active mapping session.

This is especially relevant because SLAM loop closure/pose-graph optimization can
change the relationship between earlier observations and the current optimized
map. Frontier handles should be tightly revision-bound. Persistent object/place
observations can be less aggressively invalidated, but their source revision must
remain known and their geometry must be revalidated or updated if optimization
materially changes it.

Do not assume that a database point written once is forever correct merely
because `clear_map` was not called.

## Recommended SQLite model

A minimal schema can stay simple.

```text
environments
------------
id
name
created_at

map_sessions
------------
id
environment_id
started_at
ended_at
status             # active / archived

entities
--------
id
environment_id
type               # object / place
label
attributes_json
created_at
last_seen_at

observations
------------
id
entity_id
map_session_id
map_revision
source_frame_id
observed_at
semantic_json
x                  # nullable
y                  # nullable
z                  # nullable
geometry_status    # none / trusted / stale

places / relations can initially live in semantic_json or a small relation table
as requirements become clear.

tasks
-----
id
environment_id
objective
state
context_json
created_at
updated_at
```

This is intentionally ordinary SQLite rather than a knowledge graph/vector DB.

## Clearing the SLAM map must not blindly clear semantic memory

`clear_map()` should reset the SLAM pose graph/map as it does today, and **the
world-state layer must react to that reset**.

Recommended behavior:

```text
active map_session:41
        |
        | clear_map()
        v
archive map_session:41
create map_session:42
```

Immediately:

```text
frontier:*                DELETE / expire
visible:*                 DELETE / expire
current nav targets       cancel / expire
map-bound candidate state expire
```

For persistent entities:

```text
semantic identity         KEEP
semantic attributes       KEEP
observation history       KEEP
old metric locations      KEEP AS HISTORY, mark stale
navigation using them     REFUSE
```

Example before reset:

```text
object:17
label = black backpack
observation:
  map_session = 41
  pose = (4.2, 7.1)
  geometry_status = trusted
```

After `clear_map()`:

```text
object:17
label = black backpack              # retained
last known environment = home       # retained
old observation map_session = 41    # retained as history
old pose = (4.2, 7.1)               # retained as historical data
geometry_status = stale             # no longer navigable
```

Therefore:

```text
navigate_to("object:17")
```

must fail with a result such as:

```json
{
  "ok": false,
  "reason": "target_geometry_stale",
  "map_session": 42
}
```

until a trusted observation in the active map re-establishes its location.

This gives useful persistent memory without ever navigating on coordinates from
an obsolete map.

## `clear_map()` and `new_environment()` are different operations

Treat these differently.

### `clear_map()`

Means roughly:

> Remap/relocalize this same physical environment.

Result:

- archive the old map session;
- create a new map session under the same environment;
- retain semantic entities/history;
- invalidate old navigable geometry.

### `new_environment()`

Means:

> The rover is now mapping a different physical environment.

Result:

```text
environment:home
  map_session:1
  map_session:2

environment:workshop
  map_session:1
```

Semantic target lookup defaults to the active environment, preventing entities
from unrelated buildings being mixed together.

## Reacquiring entities after a map reset

After a new map session begins, Cosmos can help re-associate persistent semantic
entities.

For example:

```text
old semantic entity:
place:kitchen
geometry_status = stale

new map_session:42
       |
rover enters room
       |
Cosmos: "this appears to be the kitchen"
       |
current trusted rover/camera pose
       |
association/revalidation
       |
new observation for place:kitchen
map_session = 42
geometry_status = trusted
```

The same principle applies to movable objects, but association should be more
conservative because a backpack can actually move.

Historical observations remain useful for statements such as:

```text
"the backpack was last seen in the office in the previous map session"
```

without pretending the old coordinate is a current navigation goal.

## Observation promotion and object geometry

Cosmos output should enter the system first as an **observation**, not as an
unquestioned permanent entity.

Example:

```text
Cosmos:
"black backpack visible near the desk"
```

Without trustworthy depth, the rover may persist useful semantic information:

```text
object:17
label = backpack
attributes = black
relation = seen_in place:office
geometry_status = none
```

but it should not invent a map coordinate from monocular visual reasoning.

Once a trusted depth + TF pipeline is available:

```text
camera image
    |
    v
Cosmos: object bbox / point
    |
    v
trusted depth
    |
    v
camera intrinsics -> 3D camera point
    |
    v
TF -> active SLAM map
    |
    v
association with entity
    |
    v
map-bound observation in SQLite
```

Then `object:*` can become navigable.

## Frontier IDs

Frontier IDs should come from the navigation side, not from Cosmos.

Use IDs tied to the map session and revision, for example:

```text
frontier:map42:r381:0
frontier:map42:r381:1
```

A frontier is not a persistent landmark. Successful exploration usually destroys
it.

`list_frontiers()` should:

1. read the current occupancy grid;
2. use the existing `frontier.py` algorithm;
3. apply current reachability/ranking logic;
4. return only useful candidate clumps;
5. assign session/revision-bound handles.

`explore_frontier(id)` should:

1. reject unknown/expired/session-mismatched handles;
2. re-evaluate against the current map;
3. apply the existing body-fit check;
4. ask Nav2 whether a route exists;
5. execute through the same movement/recovery path as today;
6. return a concrete outcome.

A stale call should fail explicitly:

```json
{
  "ok": false,
  "reason": "frontier_expired",
  "refresh": true
}
```

Cosmos can choose an ID, but it never owns the coordinates behind it.

## Keep ordinary `explore()` as the baseline

The existing:

```text
explore(minutes)
```

should remain the reliable zero-semantics baseline for:

> map whatever reachable unknown space remains.

Semantic exploration is an additional mode:

```text
current frontier candidates
       +
current visual scene
       +
objective / prior semantic state
       |
       v
Cosmos
       |
       v
choose frontier ID
       |
       v
deterministic validator
       |
       v
existing Nav2 path
```

This gives a measurable comparison between geometric frontier ranking and
semantic task-directed exploration.

## `navigate_to(target)`

A semantic navigation API should resolve handles above the existing Nav2 path.

Examples:

```text
navigate_to("place:dock")
navigate_to("place:kitchen")
navigate_to("object:17")
```

Resolution:

```text
target handle
     |
     v
world_state.resolve(active environment, active map session)
     |
     +-- semantic entity exists?
     +-- trusted active-map geometry exists?
     +-- observation sufficiently recent for entity type?
     |
     v
compute safe approach/observation pose
     |
     v
existing Nav2 NavigateToPose path
```

The semantic registry never sends wheel commands.

Keep `drive_to` and `drive_to_map_point` as separate existing capabilities.
`drive_to_map_point` has a useful safety property: the model points at the map
image it is actually viewing, and the daemon resolves that image coordinate using
the pose captured for that rendered map. `navigate_to(target)` solves persistent
semantic return-to-entity navigation instead.

## Cosmos response contract

Prefer a small validated JSON application schema rather than downstream prose
parsing.

Example:

```json
{
  "summary": "Office with a desk, chair and black backpack; open doorway left.",
  "observations": [
    {
      "label": "black backpack",
      "kind": "object",
      "bbox_norm": [0.55, 0.42, 0.74, 0.78],
      "salience": "useful"
    }
  ],
  "recommended_action": {
    "kind": "explore_frontier",
    "target": "frontier:map42:r381:2"
  },
  "reason": "The doorway leads toward an unsearched area."
}
```

Allowed recommendations should be narrow, for example:

```text
none
inspect_again
look_at
explore_frontier
navigate_to
```

Malformed JSON, an unknown action, unknown target, stale map session, or expired
handle means **no action**.

A model-supplied numeric confidence is diagnostic/ranking information, not a
safety probability.

## Action validator

All model-driven movement recommendations pass deterministic checks.

For `explore_frontier`:

```text
known handle?
active map session?
current-enough revision?
still a frontier?
body fits?
Nav2 can plan?
no other move owns the mutex?
not stopped/cancelled?
```

For `navigate_to`:

```text
known entity?
correct environment?
trusted observation in active map session?
geometry still current enough for entity type?
valid stand-off pose?
body fits?
Nav2 can plan?
no other move owns the mutex?
```

Reuse the same rover-specific validation functions used by current exploration
and `drive_to` rather than implementing a second opinion.

A refusal is a normal result, not an exception for a model to reason around.

## Task memory

Task state should be ordinary application data, not saved chain-of-thought.

Example:

```json
{
  "id": "task:42",
  "environment": "home",
  "objective": "find the red toolbox",
  "state": "searching",
  "visited": ["place:kitchen", "place:hallway"],
  "rejected_objects": ["object:9"],
  "last_action": "explore_frontier",
  "last_result": "reached"
}
```

Build each Cosmos context freshly from authoritative state:

```text
objective
active environment/map session
current robot/place state
places already searched
current camera image
current candidate frontiers/targets
last relevant action result
```

This prevents stale conversation history becoming a second world model.

## Recommended implementation components

Author only the thin pieces specific to semantic robotics.

A reasonable shape is:

```text
ros_nav/
  frontier.py                 existing, stateless frontier algorithm
  frontier_registry.py        NEW: active map/revision handles
  target_registry.py          NEW: semantic target resolver
  world_store.py              NEW: SQLite persistence
  nav_bridge.py               existing, gains internal semantic ops

rover_daemon/
  physical_reasoner.py        NEW: replaceable reasoner interface
  cosmos_client.py            NEW: Reason 2 adaptor
  semantic_observer.py        NEW: event-driven background inspection
  rover_nav.py                existing movement/tool orchestration
  tool_schemas.py             existing model-facing schemas
```

Exact filenames are flexible. The important boundaries are:

- frontier geometry stays beside map/Nav2;
- semantic identity/history lives in world state;
- model inference is outside the safety-critical daemon core where practical;
- all motion still converges on the existing Nav2 path.

## Off-the-shelf versus authored code

| Need | Use | Author? |
|---|---|---:|
| 2D mapping / loop closure | current SLAM Toolbox | no |
| planning/control/recovery | current Nav2 | no |
| rover tool boundary | current `rover_daemon` | extend only |
| frontier detection/ranking | current `ros_nav/frontier.py` | extend only |
| route/body validation | current bridge + `goal_fit` + Nav2 | no new planner |
| voice/conversation | current Qwen Omni | no initially |
| physical scene reasoning | local Cosmos sidecar | thin adaptor |
| event-driven semantic observation | small orchestrator | **yes** |
| map/revision-bound frontier handles | small registry | **yes** |
| persistent semantic memory | SQLite world store | **yes** |
| semantic target -> Nav2 goal | thin resolver | **yes** |
| ontology/knowledge graph | none initially | no |

Do not introduce KnowRob, ROSPlan, a vector DB, LangGraph persistence, or another
navigation framework without a measured requirement.

## Recommended implementation phases

### Phase 1 — Cosmos sidecar, advisory + background observation

Goal: determine whether Reason 2 adds useful physical/semantic understanding on
the actual rover.

1. Add quantized `llama.cpp` Reason 2 2B on the Orin.
2. Add the generic `PhysicalReasoner` interface + Cosmos adaptor.
3. Feed still images from the existing gimbal-camera path.
4. Add event-driven inspection after selected navigation/observation events.
5. Store observations transiently; initially persist only simple semantic logs if
   useful for evaluation.
6. Let Cosmos recommend actions but do **not** execute them automatically yet.
7. Record latency, peak RAM and any effect on Nav2/SLAM scheduling.
8. Compare saved scenes against Qwen's own vision.

Acceptance criterion: Cosmos materially improves the physical/semantic tasks we
care about without destabilizing navigation.

**Done on 2026-09-01, and the criterion is not met.** Reason 2 2B runs locally
on the Orin's CPU and its per-frame perception is accurate and hallucination-free,
but it will not re-identify anything it has already named -- not from another
angle and not from the identical frame, with the list of its own entities in
front of it. The world state fills with duplicates instead. The account is in
[`../world_state/README.md`](../world_state/README.md) and the slice that
produced it is [`task-semantic-world-state.md`](task-semantic-world-state.md),
which folded that task and its follow-up into one document on 2026-09-01.

### Phase 2 — semantic frontier selection

**Blocked on the result of Phase 1.** Letting a model choose where to drive on
the strength of a world state that grows a new identity for the same sofa every
time it looks would be building on sand. What has to come first is an identity
step that works -- appearance matching, or the bearing geometry the store already
records, or a model that can do it -- and only then the steps below.

1. Refactor current frontier calculation so candidates can be listed.
2. Add session/revision-bound frontier IDs.
3. Add `list_frontiers` and `explore_frontier` internal nav operations.
4. Keep existing `explore` unchanged as baseline.
5. Give Cosmos only candidate IDs to choose from.
6. Validate and execute through the current checks.
7. Benchmark semantic versus geometric exploration.

Useful tests:

```text
find a doorway into another room
find a desk area
find a black backpack placed in one of N rooms
inspect likely useful/human-occupied areas first
```

Measure:

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

### Phase 3 — persistent map-aware world state

1. Add SQLite `world_store`.
2. Add `environment_id`, `map_session_id`, and `map_revision` semantics from the
   beginning.
3. Wire `clear_map()` to archive the current map session and invalidate old
   geometry rather than deleting semantic entities.
4. Add `place:*` identifiers and task state.
5. Add `navigate_to(place:...)` resolver.
6. Let event-driven Cosmos observation populate/update salient semantic entities.

This phase does not require object depth.

### Phase 4 — metric semantic objects

Only start once the deployed camera stack provides trustworthy depth plus
camera-to-map transforms.

1. Cosmos identifies/grounds objects.
2. Depth supplies range.
3. Intrinsics + TF produce active-map geometry.
4. Association determines new versus existing entity.
5. Store a map-session/revision-bound observation.
6. Navigation computes a stand-off pose rather than driving to the object's
   literal coordinate.

### Phase 5 — decide whether Qwen still earns its place

Replacing Qwen is a voice/conversation decision, not a prerequisite for Cosmos.

A future local path could be:

```text
STT -> small general agent -> PhysicalReasoner -> rover tools -> local TTS
```

Benchmark that against the existing realtime Qwen experience before replacing
it.

## Tests required before model-driven movement

Require deterministic coverage for at least:

- stale frontier ID rejected;
- made-up frontier ID rejected;
- frontier remapped while Cosmos is thinking;
- wrong map session rejected;
- old object/place geometry rejected after `clear_map`;
- semantic entity survives `clear_map`;
- `new_environment()` prevents cross-environment target resolution;
- body no longer fits at selected goal;
- planner refuses selected goal;
- STOP while Cosmos inference is running;
- STOP between recommendation and execution;
- another move owns the mutex;
- Cosmos server unavailable/OOM/restarting;
- malformed/non-JSON response;
- unknown action/target rejected;
- no camera frame / frame too old;
- map revision changes during inference;
- task resumes after model failure without inventing completed actions.

The safe failure for every case is **no new movement**.

Keep the current `frontier.py` style: make registry/target arithmetic testable
offline against recorded fixtures without requiring ROS where practical.

## What not to add yet

Do not add as part of the first implementation:

- RTAB-Map merely because Cosmos is visual;
- another navigation framework;
- `explore_lite` merely to obtain frontier IDs;
- a general ontology server;
- a vector database for dozens/hundreds of entities;
- continuous VLM inference on the camera feed;
- VLM-generated metric navigation coordinates;
- direct VLM motor actions;
- persistent hidden chain-of-thought;
- automatic permanent storage of every object Cosmos mentions.

## Target system structure

After phases 1–3:

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
           semantic tools   world-state   existing direct tools
                   |          facade       lights/gimbal/battery
                   |            |
                   v            v
             semantic observer + resolver
                   |            |
                   v            v
          PhysicalReasoner   SQLite store
             adaptor        environments
                   |         map sessions
                   v         entities
          Cosmos Reason 2B  observations
          llama.cpp, local  tasks
                   |
                   +-------------+
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

                 SLAM Toolbox remains authoritative
                 for map -> odom and occupancy map
```

Important properties:

1. **one movement authority** — Nav2;
2. **one model-facing rover boundary** — the daemon;
3. **one authoritative source for geometry** — current SLAM/Nav2 state;
4. **semantic identity survives remapping, geometry does not**;
5. **all navigable DB geometry is bound to map session/revision**;
6. **Cosmos observes automatically at semantic checkpoints, not every frame**;
7. **Cosmos is advisory and replaceable**;
8. **Qwen remains conversational rather than geometric source of truth**;
9. **models choose semantic handles, never invent the coordinates behind them**.

## Top recommendation

Implement the smallest loop that demonstrates persistent semantic value without
weakening navigation safety:

```text
Nav2 reaches meaningful observation point
             |
             v
      capture settled image
             |
             v
           Cosmos
             |
      semantic observation
             |
             v
  transient association/salience
             |
             v
 SQLite entity/observation if useful
             |
             v
 optional semantic frontier choice
             |
             v
 deterministic validator
             |
             v
 existing Nav2 movement path
```

The world store should be **map-aware from its first schema migration**, even if
Phase 1 initially stores only observations. Retrofitting map identity after
persistent navigation is already implemented is exactly the kind of ambiguity
that can turn harmless stale memory into a physical navigation error.

The practical rule to carry into implementation is:

> **The rover may remember that the black backpack exists and was seen in the
> office across map resets. It may only navigate to the backpack when it has a
> trusted location tied to the currently active SLAM map session.**
