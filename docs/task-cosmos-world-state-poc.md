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

## Outcome, 2026-09-01

**Built, deployed and run. The question it was set to answer came back no.**

Cosmos Reason 2 2B runs locally on the Orin's own CPU and its per-frame perception
is good: across six inspections of a living room it named the sofa, the glass
table, the coiled cable on the floor, the spray bottle, the framed pictures and the
doorway, every label a real thing in the frame, with boxes that land on the objects
once they are read on the scale the model actually uses.

**It will not re-identify anything it has already named.** Shown four entities it
had itself created, it matched none and created four more; shown fifteen, on the
identical camera view it had already inspected, it matched none and created six
more. A bench probe put the known list first, told it explicitly to prefer a name
from that list, and handed it the very frame those entities came from a minute
earlier: `existing_entity` was null for all six things it had just named. So the
failure is the model rather than the prompt, and the store fills with duplicates --
two sofas, two tables, two doorways.

Everything else the task asked for works and is deployed. The store, the console
popup, the failure behaviour and the two clears each do what section by section
they were asked to. An inspection costs 56-89 s, the sidecar holds about 4 GB
resident, and nothing else on the rover noticed: zero dropped lidar scans
throughout.

The full account -- the numbers, the three defects that only running it exposed,
and what the run did not test -- is in
[`world_state/README.md`](../world_state/README.md). The work is on the
`cosmos-world-state-poc` branch.

### What this means for the next task

**Do not proceed to semantic frontier selection.** Letting a model choose where to
drive on the strength of a world state that grows a new identity for the same sofa
every time it looks would be building on sand. The perception is worth keeping; the
identity layer has to be fixed first, and the two obvious routes are to stop asking
the model to do it -- match on appearance embeddings, or on the bearing geometry
the store already records -- or to try Cosmos 3, which the `PhysicalReasoner`
boundary exists to make a swap rather than a rewrite.

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
                    /     \
                   /       \
            entities     observations
                   \       /
                    +-----+
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

Keep the reasoner interface testable with a deterministic fake so the database and
console can be built and exercised without a GPU or a model download. The fake is
for development and for the offline tests; it does not finish the task. If Cosmos
cannot be made to run locally on the Orin, this POC is **blocked, not complete** --
bank the store, the console and the tests, write up the blocker, and leave the
question the POC exists to answer open. Do not silently substitute a cloud model
and call the local-Cosmos requirement complete either.

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
      "bbox_norm": [0.08, 0.31, 0.48, 0.84]
    },
    {
      "existing_entity": null,
      "kind": "opening",
      "label": "doorway",
      "description": "open doorway leading to another area",
      "location_hint": "right",
      "bbox_norm": [0.70, 0.15, 0.96, 0.94]
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
- retain the original/raw validated model result for later inspection;
- retain the frame the model was shown and the measured pose it was taken from,
  beside the result -- see [section 3](#3-world-state-storage).

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
frame_path                 the stored JPEG this observation was read from
scene_summary
label
description
location_hint
bbox_json
observer_pan_deg           gimbal angles reported by the capture call
observer_tilt_deg
observer_pose_json         rover map pose at capture, null if SLAM had none
map_session                which SLAM map was live, null if unknown
model_id                   model build and quantization that answered
prompt_version             the prompt this observation was produced under
raw_json
```

There is no relations table. Relations are out of this slice entirely -- see
[section 14](#14-scope-exclusions) -- because nothing consumes them, and every
hour spent on predicates is an hour not spent on the identity question the POC
exists to answer.

Add fields only when they answer a real POC question. Avoid building a generic
ontology, vector database or knowledge graph.

The DB file is runtime state and must not be committed to Git.

### Keep the frame, and where it was taken from

Store the JPEG each inspection ran on -- as a file beside the database, named by
`frame_id` -- and record that path on every observation the frame produced.
Without the picture there is no way to separate a hallucinated entity from a real
one that the person reading the popup had forgotten was in the room, and telling
those two apart is most of what this POC is for. Frames are runtime state like the
database: never in Git, and removed by clear/reset along with the rows that
reference them.

Record the **observer** pose with each observation as well:

```text
observer_pan_deg / observer_tilt_deg   gimbal angles from the capture call
observer_pose_json                     x_m, y_m, heading from SLAM if available
```

This is not the invented geometry [section 6](#6-no-invented-geometry) forbids. It
is where the camera actually was, measured by the rover, rather than a distance the
model guessed from one picture. It is also what makes the POC's central question
answerable at all: without it, "the sofa acquired a second ID" and "the rover was
looking somewhere else" leave the same record behind.

The one-shot capture call already reports the gimbal angles and whether the frame
came off the live tracking loop or a fresh grab, and the rover's map pose is
available from the navigator. Store whatever is available and leave the rest null
rather than failing the inspection over a missing pose.

### Stamp observations with the map session

Add the `map_session` column now, even though nothing in this POC reads it. The
staleness model in the design document rests entirely on it: entities and their
history are meant to survive a SLAM map clear, but anything positional recorded
against the old map has to remain recognisable as belonging to a map that no
longer exists. Adding the column later means migrating a database that already
holds the experiment's results.

The store owns the number. Start at one and increment it when the SLAM map is
cleared -- the console already owns that button, so it can tell the store rather
than the store polling for it -- and record null while the current session is
unknown. Nothing here refuses anything on the strength of it; the popup shows it,
so that an entity last seen under an older map is visible as such.

Both directions matter, and only one of them is obvious: clearing semantic state
must not touch the map, and clearing the map must not delete semantic state.

### Provenance

Every stored semantic fact/observation must preserve enough provenance to answer:

```text
what did Cosmos actually say?
which frame did it come from?
when was it seen?
which model build and prompt version produced it?
which SLAM map session was live at the time?
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
unknown
```

`room_hint`, `text` and `hazard` are deliberately left out of this slice. Nothing
consumes them, they pull the experiment away from the identity question, and a
`hazard` label that reads as a safety signal while driving nothing at all is worse
than no label. A stair is still worth observing -- as an ordinary entity whose
description happens to say it is a stair.

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
the stair down to the hall
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
observer_pan_deg = 35
observer_tilt_deg = -10
observer_pose_json = {"x_m": 2.1, "y_m": 0.4, "heading_deg": 88}
```

It is not valid to store a Cosmos guess as:

```text
map_x = 4.72
map_y = 2.18
distance_m = 2.4
```

The line is drawn by who measured the number, not by whether there is a number.
Where the camera was pointing and where the rover was standing are readings the
rover already takes; how far away the sofa is would be an inference the model made
from a single monocular image. Store the first as provenance of the observation,
never the second as a property of the object.

Persistent metric object location belongs to a later OAK-D + intrinsics + TF
integration. Leave metric object position nullable/absent now so the schema can be
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

Add a **World State** button to the main drive console that opens a separate popup
over the page. The console today is cards in three stacks and has no modal of any
kind, so this is new furniture rather than an existing pattern to copy: build the
overlay, and keep the world state inside it rather than growing another card in the
page behind it.

How the state is drawn is open, and the most useful shape is probably the semantic
state over a copy of the SLAM map the console already serves at `/map.png`.
Entities have no metric position and must not be given one, but every observation
now records where the rover stood and where the gimbal pointed, so an entity can
honestly be drawn as a **bearing from a measured pose** -- a ray or a narrow cone
from the observation point, along the camera's direction, with the bounding box's
horizontal position refining the angle. Several observations of one entity then
appear as several rays from different places, and whether they converge on one
corner of the room is precisely the question this POC is asking; a duplicate shows
up as two ID labels on rays pointing at the same thing.

That is a suggestion rather than a requirement -- a plain list that makes the
failures visible beats a map picture that hides them. Fall back to the list
whenever there is no map, no pose, or nothing observed yet.

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
- frame ID, and the stored frame itself with its bbox drawn on it
- gimbal pan/tilt and rover pose at capture
- location hint / bbox if present
- raw Cosmos JSON for each observation
```

Useful tabs/sections if they fit naturally:

```text
Entities
Observations
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

Give the inspection its own connection and its own patience. The console holds
several separate connections to the daemon, each with the timeout its own work
needs, and the wifi scan already has one of its own precisely because it outlasts
everything else by a factor of three. An inference of seconds to tens of seconds
on the shared status connection would stall the polling behind it -- lights,
tracking, wifi -- and must never be able to sit in front of a STOP.

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
SQLite busy/locked while the console is reading
console request while an inspection is already in progress
clear during/around inspection
model returns zero observations
sidecar generates without stopping
```

The runaway generation is the likeliest way this hangs. Cap the sidecar's output
tokens and put a hard wall-clock limit on the call, so a model that will not stop
talking fails one inspection rather than holding a connection open indefinitely.

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
- the frame file is written and referenced by every observation it produced;
- gimbal angles and rover pose are retained, and a missing pose stores nulls
  rather than failing the inspection;
- clear/reset affects only semantic state, and removes stored frames with it;
- a map clear starts a new session and leaves entities and history intact;
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
- an observation whose stored frame is missing still renders;
- the map overlay falls back to the list view with no map or no pose;
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

### Getting it onto the rover

A new directory deploys nothing by itself. Register the world-state component in
[`deploy/manifest.json`](../deploy/README.md) with its own sources, restart script
and verification command, the way every component already on the Orin is
registered; until that entry exists the deployer will not copy a single file of
it.

Three things have to stay out of the deployer's way:

```text
model weights     a quantized GGUF is gigabytes, and belongs neither in Git nor in
                  a deploy payload. Fetch it on the Orin from an install script,
                  the way the depth camera's vendored tree is installed, and have
                  the component's verification check the file is present rather
                  than ship it.
database          ~/.ugv/, beside the rover's own secrets and deploy state, where
                  no deploy and no prune can reach it.
stored frames     the same place, in a directory of their own.
```

Open SQLite in WAL mode. The console reads world state while the inspection that
is writing it is still running, and the default journal turns that ordinary
overlap into a locked database rather than a slightly stale read.

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
9. check each observation against its stored frame for hallucinated or
    contradictory entities;
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
relations / predicates between entities
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

All met on 2026-09-01, with the two caveats noted underneath.

- [x] A replaceable `PhysicalReasoner` boundary exists.
- [x] Cosmos Reason 2 2B runs locally on the Orin and answers a real inspection of
      a real frame. The deterministic fake covers development and offline tests
      only; if the local runtime cannot be made to work, the task is blocked rather
      than complete.
- [x] A fresh gimbal frame can be inspected without violating existing camera
      ownership.
- [x] Only schema-valid structured observations mutate semantic world state.
- [x] Application code, never Cosmos, owns entity IDs.
- [x] SQLite persists entities and complete observation history across restart.
- [x] Raw validated Cosmos observations and provenance are retained.
- [x] The frame behind each observation is stored and viewable, together with the
      gimbal angles and rover pose it was taken from.
- [x] No metric/map coordinates are accepted from the VLM as world facts.
- [x] A button in the main drive console opens a read-only World State popup
      showing entities, observation history, stored frames and raw inference
      detail.
- [x] Semantic state can be explicitly cleared without touching the SLAM/Nav2 map.
- [x] Deterministic offline tests cover storage, association, validation and UI/API
      behavior.
- [x] The world-state component is registered in `deploy/manifest.json` with its
      own verification, the model weights are installed on the Orin rather than
      committed or deployed, and the database and stored frames live under
      `~/.ugv/` where no deploy can overwrite them.
- [x] The changed components are deployed and restarted on the Orin.
- [x] A real multi-view inspection experiment is performed on the rover.
- [x] The final report says plainly whether object/entity continuity is good enough
      to justify proceeding to semantic frontier selection. It is not.

### The two caveats

**The popup has not been looked at in a browser.** It was driven end to end through
the console's own action path -- open, refresh, select an entity, close -- and every
payload and URL behind it was read and checked, including a stored frame served as
a 45 kB JPEG. What has not happened is a person seeing it drawn, because this
repository has no browser in its test loop. The markup, the styles and the
JavaScript are therefore unproven in the one way that matters for a viewer.

**The rover was never driven during the experiment.** There was somebody sitting in
the room and nobody watching the wheels, so the five views are gimbal pans from one
parked pose, and no new object was carried in between inspections. Every observation
therefore shares an origin, and the part of the map view that asks whether rays
*from different places* converge on one corner has been exercised only in the
offline tests. Neither gap weakens the negative finding -- a model that will not
re-identify an object in the identical frame is not going to do better from a new
angle -- but both would have to be closed before a *positive* result could be
believed.

## Expected follow-up

The POC produced unstable entity identity, so this is the second of the two forks
the task set out: **fix or replace the perception/world-state layer before giving it
any influence over rover movement.** Semantic frontier selection is not the next
task.

What is worth keeping is everything below the model: the store, the provenance, the
popup and the sidecar are all sound, and the identity step is one replaceable piece
inside them. The candidates, cheapest first:

1. **Stop asking the model to do it.** Match a new observation against existing
   entities on appearance -- an embedding of the bounding box's contents -- or on
   the bearing geometry the store already records, and leave the model to say what
   things are rather than which thing this is. This is the fork the association
   rules were deliberately written to leave room for.
2. **Try Cosmos 3, or an 8B build.** The `PhysicalReasoner` boundary exists exactly
   so this is a swap rather than a rewrite. Against it: an 8B model at this
   quantization would not leave room on an 8 GB board beside SLAM, and 2B is already
   a minute a look.
3. **Give the model fewer, sharper choices.** Asking "is this one of these three
   things, which are the ones you could plausibly be looking at from here" is a
   different and much easier question than the twenty-one-entity list it was handed
   here. The bearing geometry is what would narrow the list.

Whichever is tried, the measurement to repeat is the one in this document: clear,
inspect from several views, and count how many identifiers survive.
