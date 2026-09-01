# Task: Cosmos world-state proof of concept

## Goal

Build the first practical Cosmos integration on the rover as an **observational
world-state experiment**, not a navigation/control feature.

The rover should be able to take a current camera image, ask a locally running
physical-reasoning VLM (initially NVIDIA Cosmos Reason 2 2B) what is meaningfully
present, store the resulting semantic observations in SQLite, associate repeated
observations with persistent entities where reasonable, and expose the resulting
state in a **read-only World State popup in the existing drive console**.

The point of this task is to answer one question with real rover data:

> **Does Cosmos build and maintain a semantic description of the environment that
> remains coherent and useful as the rover sees the same place and objects from
> different views and at different times?**

Do **not** let Cosmos influence movement in this task. No frontier selection,
semantic `navigate_to`, direct drive calls, or object-coordinate navigation are
part of this POC.

Read [`docs/cosmos-reason2-integration.md`](cosmos-reason2-integration.md) first;
this task is the first implementation slice of that design. Also follow
[`CLAUDE.md`](../CLAUDE.md): a change is not complete until the component is
deployed to the Orin and the running system is verified there.

---

## Existing system to preserve

Do not disturb the existing authority boundaries:

```text
Qwen Omni                user-facing voice/conversation/tool agent
rover_daemon             single model-facing rover/hardware boundary
SLAM Toolbox + Nav2      geometry, planning, control, recovery
current explore          autonomous geometric frontier exploration
D500 lidar               navigation obstacle/map source
OAK-D-Lite               existing fixed depth service, not required for this POC
```

The current gimbal camera is sufficient for this first experiment. Cosmos may
reason from its RGB images, but must not invent metric map coordinates for visual
entities.

The existing `look` path and camera ownership rules matter. Do not create a second
process that independently opens the gimbal camera while the daemon owns it.
Reuse or extend the current capture/handoff path.

---

## Target POC

Conceptually:

```text
                   gimbal camera
                        |
                        v
                 current JPEG/frame
                        |
                        v
              PhysicalReasoner client
                        |
                        v
              Cosmos Reason 2 2B
                  local on Orin
                        |
               structured JSON only
                        |
                        v
                 world_state
             /         |          \
            /          |           \
      entities    observations    relations
            \          |           /
             +---------+----------+
                        |
                     SQLite
                        |
                        v
                 read-only API
                        |
                        v
              drive console popup
```

The model **proposes observations**. Application code owns IDs, persistence,
validation, association and the current canonical state.

Cosmos must never receive arbitrary SQL access and must never directly write or
update database rows.

---

## 1. Physical-reasoner boundary

Do not spread Reason-2-specific code through the application. Introduce a small
replaceable interface, e.g. conceptually:

```python
class PhysicalReasoner:
    def inspect(self, image, context):
        """Return one validated structured observation result."""
```

The first implementation may use Cosmos Reason 2 2B, but a later Cosmos 3 or
other VLM should be able to replace it without changing the world-state schema or
console.

Run the model as a **separate local sidecar process**, not inside
`rover_daemon`. A model crash/OOM/restart must not take down the process that owns
STOP, the driver-board UART or the gimbal camera.

For the Orin Nano 8 GB, prefer the quantized `llama.cpp` path described in
[`docs/cosmos-reason2-integration.md`](cosmos-reason2-integration.md) rather than
loading a full-precision Transformers stack into the daemon process.

If getting Cosmos itself running safely on the Orin becomes a separate substantial
piece of work, keep the reasoner interface testable with a deterministic fake so
the database and console can be completed and exercised independently. Do not
silently substitute a cloud model and call the local-Cosmos requirement complete.

---

## 2. Cosmos output contract

Ask Cosmos for a small application-owned JSON schema. Do not parse free-form prose
to update the world model.

A reasonable first shape is:

```json
{
  "scene": "A living room with a sofa, coffee table and open doorway.",
  "observations": [
    {
      "existing_entity": "object:12",
      "kind": "object",
      "label": "sofa",
      "description": "grey three-seat sofa",
      "location_hint": "ahead-left",
      "bbox_norm": [0.08, 0.31, 0.48, 0.84],
      "relations": []
    },
    {
      "existing_entity": null,
      "kind": "opening",
      "label": "doorway",
      "description": "open doorway leading to another area",
      "location_hint": "right",
      "bbox_norm": [0.70, 0.15, 0.96, 0.94],
      "relations": []
    }
  ]
}
```

The exact schema may be adjusted during implementation, but keep these rules:

- model output is validated before storage;
- `existing_entity` may only reference IDs that were supplied in the request;
- the model never creates IDs;
- unknown IDs are rejected, not auto-created;
- malformed JSON is stored/logged as a failed inference and causes no entity
  mutation;
- bounding boxes are normalized image coordinates only;
- no model-provided metric `x/y/z`, map pose or navigation target in this task;
- retain the original/raw validated model result for later inspection.

Do not use a model-supplied numeric confidence as an authority or safety gate. It
may be stored for diagnostics if the model naturally supplies one, but this POC
is about observable consistency rather than pretending the number is calibrated.

---

## 3. World-state storage

Create a small component, preferably under a clear new directory such as
`world_state/`. Keep it ordinary Python + stdlib SQLite unless the existing code
strongly suggests another placement.

The important design is to separate **observations** from **entities**.

Do not implement this as:

```text
Cosmos -> UPDATE objects SET description = ...
```

Implement it as:

```text
Cosmos result
     |
     v
immutable-ish observation record
     |
     v
association / canonical entity update
     |
     v
entity current state
```

### Suggested tables

Use migrations/schema creation that can safely open an empty DB. A minimal useful
shape is:

```text
entities
--------
id
kind
label
canonical_description
state
created_at
last_seen_at
observation_count

observations
------------
id
entity_id                 nullable if association failed/new candidate rejected
observed_at
source                     e.g. cosmos_visual
frame_id
scene_summary
label
description
location_hint
bbox_json
raw_json

relations                  optional for the first cut if it delays the POC
---------
id
subject_entity_id
predicate
object_entity_id           nullable if relation target is not an entity
object_text                nullable textual target
observed_at
source
```

Add fields only when they answer a real POC question. Avoid building a generic
ontology, vector database or knowledge graph.

The DB file is runtime state and must not be committed to Git.

### Provenance

Every stored semantic fact/observation must preserve enough provenance to answer:

```text
what did Cosmos actually say?
which frame did it come from?
when was it seen?
which current entity did we associate it with?
what is the current canonical value versus the observation history?
```

This is essential: the popup must let us distinguish a bad model observation from
bad association/storage logic.

---

## 4. Entity identity and association

Application code owns entity IDs, for example:

```text
object:1
object:2
opening:1
person:1
place_hint:1
```

Do not let Cosmos allocate, rename or recycle them.

For each new inspection, pass a **bounded relevant current-state summary** into
Cosmos so it can indicate whether a visible thing appears to be an entity already
known. For example:

```text
Known entities relevant to the current view/task:
- object:12 — grey sofa
- object:13 — wooden coffee table
- opening:3 — open doorway
```

Then ask it to classify observations as either:

```text
existing_entity: "object:12"
```

or:

```text
existing_entity: null
```

The store allocates the ID for a new entity.

### Keep association deliberately conservative

The failure we most want to see is whether the system creates duplicates or
incorrectly merges distinct things. Do not hide that behind aggressive fuzzy
matching.

For the first cut:

1. accept a model reference only if the ID was in the supplied known-entity list;
2. otherwise create a new entity for a sufficiently concrete observation;
3. preserve all observations even if canonical entity wording changes;
4. never silently merge two existing IDs;
5. make duplicate entities visible in the inspector so the POC can expose this
   failure mode rather than conceal it.

It is acceptable for the first POC to over-create entities. The inspector exists
partly to measure that behavior.

---

## 5. What to observe

Keep the semantic vocabulary loose and practical. We need a useful environment
memory, not a formal ontology.

A reasonable starting set is:

```text
object
furniture
opening
person
room_hint
text
hazard
unknown
```

Do not force Cosmos to classify everything visible. Prefer a smaller set of
salient entities that a human would plausibly care about later.

Useful examples:

```text
grey sofa
wooden coffee table
television
red toolbox
black backpack
open doorway
closed door
stair / drop hazard
sign or readable label
```

Avoid storing transient image trivia as persistent entities unless it matters.

---

## 6. No invented geometry

This task uses the gimbal camera only for semantics.

It is valid to store:

```text
location_hint = "ahead-left"
bbox_norm = [...]
frame_id = "..."
```

It is not valid to store a Cosmos guess as:

```text
map_x = 4.72
map_y = 2.18
distance_m = 2.4
```

Persistent metric object location belongs to a later OAK-D + intrinsics + TF
integration. Leave metric position nullable/absent now so the schema can be
extended later without pretending monocular estimates are facts.

---

## 7. Inspection trigger

For the first POC, make inspections explicit and controllable rather than
continuously calling the VLM.

Add a console action such as **Inspect world** that:

1. takes a fresh gimbal image through the existing ownership path;
2. builds the bounded current-state context;
3. calls the local physical reasoner;
4. validates the structured result;
5. records observations/entities;
6. refreshes the World State popup if it is open;
7. reports failure plainly without changing world state.

If the existing console structure makes a separate button awkward, an equivalent
explicit control is fine. Do not start automatic inspection on every video frame.

After the manual POC is proven, a later task can trigger inspection on events such
as goal completion, significant motion or heading change.

---

## 8. Console World State popup

Add a **World State** control to the existing drive console and show the state in
a popup/modal consistent with the current console UI.

This POC viewer should be **read-only** except for an explicit clear/reset action.

At minimum show:

```text
summary
- entity count
- observation count
- last successful inspection time
- last inference status/error

entities
- ID
- kind
- label / canonical description
- first seen
- last seen
- observation count

selected entity detail
- full canonical fields
- all observation history, newest first
- source
- frame ID
- location hint / bbox if present
- raw Cosmos JSON for each observation
```

Useful tabs/sections if they fit naturally:

```text
Entities
Observations
Relations             only if relations are implemented
Raw / diagnostics
```

The display should make these problems obvious rather than smoothing them over:

- duplicate entities;
- contradictory descriptions;
- entities that have not been seen for a long time;
- a current canonical description that does not match its observation history;
- invalid/failed Cosmos responses;
- repeated creation of the same object under new IDs.

### Clear/reset

A **Clear world state** action is useful for repeatable POC runs, but make it
explicit/destructive in the UI and keep it separate from navigation/map reset.
Clearing semantic memory must not clear SLAM Toolbox or Nav2 state.

---

## 9. APIs and authority

Expose only what the console/integration needs. Exact transport should match the
existing `drive_web` / daemon patterns rather than introducing another LAN service
without reason.

Conceptually useful operations are:

```text
world_state_summary
world_state_entities
world_state_entity(id)
world_state_observations(entity_id?)
world_state_clear
world_inspect
```

These do **not** need to become Qwen model-facing tools in this task. The purpose
is to evaluate Cosmos and the world model first, not to add more agent authority.

Keep database writes behind the world-state component. The console should never
issue SQL and Cosmos should never receive a generic database mutation API.

---

## 10. Diagnostics

Make inference/store behavior inspectable without reading hidden model reasoning.
Log or persist normal operational facts such as:

```text
inspection start/end
duration
model/backend used
frame ID / age
number of supplied known entities
number of returned observations
number matched to existing entities
number of new entities created
validation failure reason
model/server unavailable/OOM/timeout
```

Do not store chain-of-thought. Store the application response and concise reason
fields only.

It would be useful for the popup to show the last few inference outcomes so a
missing update can be identified as model failure versus "nothing salient found".

---

## 11. Failure behavior

Every failure in this POC should degrade to **no world-state mutation** (apart
from recording an inference error/diagnostic where appropriate).

Cover at least:

```text
Cosmos sidecar unavailable
Cosmos timeout
malformed JSON
schema-invalid JSON
unknown existing_entity ID
frame capture failure
frame too old / wrong frame token
SQLite open/write failure
console request while an inspection is already in progress
clear during/around inspection
model returns zero observations
```

No failure here should stop or interfere with rover driving, STOP, Nav2, face
tracking, the camera's existing safety/ownership rules, or the Qwen voice session.

---

## 12. Tests

Add deterministic tests before deploying.

At minimum exercise:

### Store/schema

- empty DB creation;
- entity allocation;
- observation history retained after canonical update;
- persistence across process restart;
- clear/reset affects only semantic state;
- malformed stored JSON cannot take the viewer/API down.

### Association

- supplied known entity can be referenced;
- unknown model-supplied ID rejected;
- new entity gets an application-generated ID;
- two new observations do not silently merge existing IDs;
- same known ID accumulates observation history.

### Model response validation

- valid structured response accepted;
- malformed JSON rejected;
- wrong types/out-of-range bbox rejected or safely normalized according to the
  documented contract;
- model-supplied metric/map coordinates ignored/rejected if present.

### Console/API

- empty world renders;
- populated world renders;
- raw observation detail is available;
- inference error state renders;
- clear action works and does not touch navigation state.

Where possible use a deterministic fake `PhysicalReasoner` so ordinary self-tests
do not require a GPU/model download.

---

## 13. Real rover validation

Per `CLAUDE.md`, local tests and a commit are not completion.

Deploy the affected registered components to `orin`, restart only what is needed,
and prove the running system works there.

Perform a small repeatable real-world experiment. For example:

1. clear semantic world state;
2. park the rover in a room containing several obvious persistent objects;
3. run **Inspect world**;
4. inspect the popup and record the entity/observation counts;
5. move/turn the rover or gimbal to view the same objects from a materially
   different angle;
6. inspect again;
7. repeat for at least three views;
8. verify whether the same objects retain IDs or duplicate;
9. inspect raw observations for any hallucinated or contradictory entities;
10. restart the relevant service and confirm the SQLite state persists;
11. clear semantic state and confirm the navigation map remains untouched.

If safe/practical, include one deliberately new object introduced between two
inspections to show whether it is added without rewriting unrelated state.

Report the actual outcome plainly:

```text
works / does not work
how many entities accumulated
obvious duplicates
obvious hallucinations
same-object continuity across viewpoints
whether persistence survived restart
model latency and approximate RAM impact
any observed effect on the existing rover services
```

Do not tune away a poor result just to make the demonstration look good. The
purpose of the POC is to measure whether this approach is worth continuing.

---

## 14. Scope exclusions

Do **not** add any of the following in this task:

```text
Cosmos-selected frontiers
semantic explore
navigate_to(object:* / place:*)
OAK-D metric object localization
RTAB-Map
new SLAM/navigation stack
vector database
KnowRob / ontology framework
LangGraph or generic agent-memory framework
model-generated map coordinates
raw motor/cmd_vel access
continuous video-rate Cosmos inference
automatic periodic inspection
Qwen replacement
user-edit/merge tools for entities beyond clear/reset
```

If implementation reveals that one of these is genuinely required to make the
POC function, stop and document the dependency rather than silently expanding the
architecture.

---

## Acceptance criteria

This task is complete when all of the following are true:

- [ ] A replaceable `PhysicalReasoner` boundary exists.
- [ ] Cosmos Reason 2 2B can be called locally on the Orin, or any blocker to the
      local runtime is explicitly demonstrated while the remaining POC is proven
      against a deterministic fake.
- [ ] A fresh gimbal frame can be inspected without violating existing camera
      ownership.
- [ ] Only schema-valid structured observations mutate semantic world state.
- [ ] Application code, never Cosmos, owns entity IDs.
- [ ] SQLite persists entities and complete observation history across restart.
- [ ] Raw validated Cosmos observations and provenance are retained.
- [ ] No metric/map coordinates are accepted from the VLM as world facts.
- [ ] The drive console has a read-only World State popup showing entities,
      observation history and raw inference detail.
- [ ] Semantic state can be explicitly cleared without touching the SLAM/Nav2 map.
- [ ] Deterministic offline tests cover storage, association, validation and UI/API
      behavior.
- [ ] The changed components are deployed and restarted on the Orin.
- [ ] A real multi-view inspection experiment is performed on the rover.
- [ ] The final report says plainly whether object/entity continuity is good enough
      to justify proceeding to semantic frontier selection.

## Expected follow-up

If this POC produces a coherent world state, the next task should be **semantic
frontier selection**: expose generation-bound frontier IDs from the existing
`ros_nav/frontier.py`, let Cosmos choose among those IDs using the observed world
state and task objective, and send the selected target through the current
deterministic body-fit/planner/action path.

If this POC produces unstable entity identity, frequent hallucinations or an
unusable memory representation, fix or replace the perception/world-state layer
before giving it any influence over rover movement.
