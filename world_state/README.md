# Semantic world state

What the rover has been told is in the room, kept apart from where things are.

SLAM Toolbox and Nav2 own geometry: the occupancy grid, the pose, the routes. This
component owns the other kind of memory — that there is a grey sofa, that it was
seen three times from three places, and that the model was shown a picture of it
each time and said so. Nothing in here drives, plans or refuses a move, and nothing
in here is offered to a voice model. It exists to answer one question with real
rover data:

> Does Cosmos build and maintain a description of the environment that stays
> coherent as the rover sees the same place from different views?

The design it implements is [`docs/cosmos-reason2-integration.md`](../docs/cosmos-reason2-integration.md);
the slice it implements is [`docs/task-cosmos-world-state-poc.md`](../docs/task-cosmos-world-state-poc.md).

```text
                gimbal camera
                      |            rover_daemon owns it, and takes the picture
                      v            through the same path camera_jpeg uses
              PhysicalReasoner
                      |
                      v            loopback 8775, its own process
            llama.cpp + Cosmos Reason 2 2B (Q4)
                      |
              structured JSON only
                      |
                      v
        validate -> observation -> entity
                      |
                      v
        SQLite + JPEGs under ~/.ugv/world
                      |
                      v
      control calls on TCP 8769 -> drive console popup
```

## The one rule

**The model proposes; the application disposes.** Cosmos never allocates an
identifier, never names an entity it was not shown, never writes a row and never
states a distance. What it returns is a proposal that [`contract.py`](contract.py)
validates and [`store.py`](store.py) records, and every path through
[`inspector.py`](inspector.py) that fails leaves the world exactly as it was, with
one line in the diagnostics log saying which failure it was.

That matters more here than it usually would. The question this component exists to
answer is whether *the model* keeps track of a sofa, and any fuzzy matching on our
side — merging two entities because their labels look alike, creating an entity for
an identifier the model invented — would answer a question about our code instead.
So association is deliberately blunt:

1. a reference to an identifier that was in the list the model was shown is accepted;
2. anything else becomes a new entity, if its label names something in particular;
3. two existing entities are never merged;
4. no observation is ever rewritten.

Over-creating is the expected failure and is left visible: two entities with the
same label sit next to each other in the popup and are marked as sharing it.

## Observations and entities are different things

An **observation** is what the model said about one picture at one moment. It is
never rewritten. An **entity** is the application's current opinion about a lasting
thing in the room, derived from observations.

Collapsing the two — letting an answer update a row in place — would destroy the
evidence, because "the sofa's description changed" and "the model saw a different
sofa" would leave the same record behind. The canonical description follows the
newest observation, and every earlier one is still there, so an entity whose wording
has drifted from its own history says so in the popup rather than quietly becoming
the new wording.

There is no `state` column on entities, although the task's suggested schema lists
one: nothing in this slice would write anything but `present` into it and nothing
would read it. Whether an entity has gone quiet is `last_seen_at`; whether it
belongs to a map that no longer exists is `map_session`. Both are answered from
columns something actually writes.

## Provenance, and the line it draws

Every observation carries where the camera was:

```text
observer_pan_deg / observer_tilt_deg    the gimbal, from the capture call
observer_pose_json                      x, y and heading from SLAM, or null
frame_id / frame_path                   the JPEG the model was shown
map_session                             which SLAM map was live
model_id / prompt_version               which build, under which wording
```

**The line is drawn by who measured the number, not by whether there is a number.**
Where the gimbal was pointing and where the rover was standing are readings the
rover already takes. How far away the sofa is would be an inference the model made
from a single photograph, and none of that is stored: `contract.py` strips
`distance_m`, `map_x` and their relatives out of the answer before anything sees it,
and reports that it did.

That provenance is also what lets the popup draw an entity on the map without giving
it a position. [`view.py`](view.py) turns one observation into a **bearing from a
measured pose** — a cone from where the rover stood, along where the camera pointed,
narrowed by where in the picture the thing sat. Several observations of one entity
are several cones from different places, and whether they converge is the question.
No pose or no gimbal angle means no cone, rather than a cone from the origin.

## Clearing, in both directions

Only one direction is obvious.

- **Clearing the semantic world does not touch the map.** It cannot: this process
  does not own the map. `world_state_clear` deletes entities, observations, the
  diagnostics log and the stored frames, and leaves SLAM and Nav2 alone.
- **Clearing the SLAM map does not delete semantic memory.** Entities outlive the
  maps they were seen under. What happens instead is that the store starts a new
  map session, so every observation recorded against the old map stays recognisable
  as belonging to a map that no longer exists. The console owns the map's clear
  button, so it tells the store; nothing polls for it.

## Where things are

```text
~/ugv/world_state/            this component, deployed
~/ugv/world_state/vendor/     the model weights and llama.cpp, fetched by install.sh
~/.ugv/world/world.db         entities, observations, inferences
~/.ugv/world/frames/          one JPEG per inspection
```

The database and the frames are under `~/.ugv/` for the same reason the TLS keys
are: a source deploy replaces `~/ugv`, and an experiment's results are not source.
The weights are in `vendor/`, which the deploy manifest preserves, for the reason
the depth camera's DepthAI tree is — two gigabytes is neither describable by a
commit nor sensible to send over the rover's wi-fi on every change.

## Installing it

A deploy copies the source and will then **fail its own verification** on a host
that has no model yet, saying so. That is deliberate: the alternative is a component
that deploys clean and cannot answer.

```bash
python deploy/deploy.py --only world_state       # copies; fails if no model
ssh orin '~/ugv/world_state/install.sh'          # ~2 GB, several minutes
python deploy/deploy.py --only world_state       # now passes
```

`install.sh` fetches a pinned Q4 GGUF of Cosmos Reason 2 2B and its vision
projector from Hugging Face, unpacks a pinned aarch64 `llama.cpp` release, checks
both against their expected sizes, and adds the sidecar's `@reboot` crontab entry.
It is idempotent and resumes a part-fetched file, which is most of why it is worth
re-running rather than starting again.

**The runtime is the CPU.** This Jetson has no CUDA toolkit installed and nothing
else deployed on the rover uses its GPU, so the released CPU build is what actually
runs — four threads, which is this board's whole processor.

## Running it

```bash
ssh orin '~/ugv/world_state/restart.sh'              # reload the sidecar
ssh orin '~/ugv/world_state/restart.sh --supervisor' # after changing run_cosmos.sh
ssh orin 'tail ~/ugv/world_state/cosmos.log'
```

Use `restart.sh` rather than relaunching `run_cosmos.sh` by hand: the supervisor is
where the flags live, and the `pkill` patterns live in a file where an ssh command
cannot match itself.

## The calls

All of these are control calls on the daemon's TCP 8769, and none of them is in
`list_tools`. This slice exists to find out whether the world state is worth
trusting; giving a model the authority to write to it, or to throw it away, before
that has an answer would be the wrong order.

| Call | What it does |
|---|---|
| `world_state_summary` | counts, the last inference, the last few outcomes |
| `world_state_entities` | every entity, its rays, the observation stream, the unmatched |
| `world_state_entity(id)` | one entity and its whole recent history |
| `world_state_observations(entity_id?)` | the history on its own |
| `world_state_frame(frame_id)` | the stored JPEG, base64, for the console |
| `world_inspect` | take a picture, ask the model, record what it said |
| `world_state_clear` | empty the semantic world; the map is untouched |
| `world_map_session` | the map was cleared, so start a new session |

`world_inspect` is about a minute on this board. It runs on the calling thread, so
the daemon goes on answering STOP, status and the map throughout, and the console
gives it a connection of its own with a patience of its own — the same arrangement,
for the same reason, as the wi-fi scan.

## Tests

```bash
python world_state/selftest.py        # the store, the rules, one inspection
python rover_daemon/selftest.py       # the daemon's control calls
python drive_web/selftest.py          # the console's payload and its two URLs
```

Everything there runs against `FakeReasoner` and a temporary directory. **That
proves the store and the rules and nothing whatever about Cosmos** — which is why
the fake is development scaffolding rather than a result, and why the task this
implements treats a local runtime that cannot be made to work as a blocker rather
than as a pass.

Two things are deliberately not covered. The popup's rendering is JavaScript in a
browser and this repository has no browser in its test loop; what is checked instead
is the payload it draws from and the two URLs it fetches. And the quality of what
Cosmos says can only be measured on the rover, against real rooms, which is what
`docs/task-cosmos-world-state-poc.md` calls the real validation.

## What was measured on the rover

Recorded here because it is the sort of thing a later reader will otherwise
rediscover:

- **One inspection is about 60 s** at 640×480, Q4, four CPU threads: roughly 480
  prompt tokens and 450 generated. The sidecar's own wall clock is 180 s and the
  console's patience is 200 s, in that order, so the sidecar always gives up first.
- **The model answers on a 1000-unit grid, not in fractions.** Cosmos Reason 2 is a
  Qwen3-VL fine-tune and places things the way that family was trained to, whatever
  the prompt asks for. The prompt now asks for the grid and `contract.py` reads
  both, because a picture is one unit across and a box in the hundreds is therefore
  not a fraction.
- **Given an example box of real numbers, it copies them.** The prompt's example
  used `[0.1, 0.2, 0.4, 0.8]` and that exact box came back on every observation. The
  example now names the four corners in words instead.
