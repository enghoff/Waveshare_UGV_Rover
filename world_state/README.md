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
the plan it belongs to is [`docs/task-semantic-world-state.md`](../docs/task-semantic-world-state.md).

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
`docs/task-semantic-world-state.md` calls the real validation.

## What was measured on the rover

Measured on 2026-09-01, in a living room, with the rover parked and the gimbal
panned between inspections. All of it is here because it is the sort of thing a
later reader would otherwise have to rediscover.

### It runs, and what it sees is right

Cosmos Reason 2 2B runs locally on the Orin's CPU and describes the room
accurately. Across six inspections it named the sofa, the glass table, the coiled
cable on the floor, the spray bottle, the framed pictures and the doorway —
**every label was a real thing in the frame, with no hallucinations** — and once
the boxes are read on the right scale they land on the objects: the sofa came back
as `[0.40, 0.18, 0.86, 0.65]` in a frame where it occupies roughly `[0.35, 0.13,
0.90, 0.65]`.

### It cannot recognise what it named a minute ago

**This is the finding the experiment exists for, and it is negative.** In every
inspection where the model was shown entities it had itself created, it matched
none of them:

| view | entities shown | matched | created |
|---|---:|---:|---:|
| centred | 0 | 0 | 4 |
| panned 30° left | 4 | 0 | 5 |
| panned 30° right | 9 | 0 | 6 |
| centred again — the identical view | 15 | 0 | 6 |

Not a prompt-layout problem. A bench probe put the known list first, instructed the
model explicitly to prefer a name from it, and showed it **the very frame those
entities had been created from** a minute earlier; it answered `existing_entity:
null` for all six things it had just named.

So the store fills with duplicates: two sofas, two tables, two doorways, two spray
bottles. That is the behaviour the popup is built to make visible rather than to
hide, and it is what a later slice has to fix — by giving the association step
something better than the model's own word, or by using a model that can do this.

**What this run did not test.** The rover was never driven: there was somebody
sitting in the room and nobody watching the wheels, so the views are gimbal pans
from one parked pose. Every observation therefore shares an origin, and the part of
the map view that asks whether rays *from different places* converge on one corner
has been exercised only in the offline tests. Nothing was carried into the room
between inspections either. Neither gap weakens the finding above -- a model that
will not re-identify an object in the identical frame is not going to do better
from a new angle -- but both would have to be closed before a positive result could
be believed.

### Numbers

- **One inspection is 56–89 s** at 640×480, Q4, four CPU threads — about 480 prompt
  tokens and 450 generated, roughly seven tokens a second. The sidecar's wall clock
  is 180 s and the console's patience 200 s, in that order, so the sidecar always
  gives up first.
- **The sidecar holds about 4 GB resident**, of which 2.1 GB is the two mapped GGUF
  files, leaving about 2.8 GB available on this 8 GB board.
- **Nothing else on the rover noticed.** Through the whole run the lidar reported
  zero dropped scans, a scan age of 0.01 s and a trusted position, and the driver
  board stayed healthy.
- The database and its frames came to 2.6 MB after six inspections.

### Three things that had to be fixed, all found by running it

- **The model answers on a 1000-unit grid, not in fractions.** Cosmos Reason 2 is a
  Qwen3-VL fine-tune and places things the way that family was trained to, whatever
  the prompt asks. The prompt now asks for the grid and `contract.py` reads both,
  because a picture is one unit across and a box in the hundreds is therefore not a
  fraction.
- **Given an example box of real numbers, it copies them.** The prompt's example
  used `[0.1, 0.2, 0.4, 0.8]` and that exact box came back on every observation. The
  example now names the four corners in words.
- **A grammar constrains the shape and not the size.** Asked about a living room,
  the model wrote three and a half thousand characters of essay into the scene
  sentence and ran out of tokens before closing the object, so a good look at the
  room was thrown away as truncated. `maxLength` in the schema fixes it, and the
  observation cap and the token cap now agree with each other.

### And one thing about the camera

A grab that closely follows another one comes back empty — v4l2-ctl exits at once,
says nothing on stderr, and hands back no whole picture — while a standalone
inspection worked every time. Reproduced three times out of three by taking a
picture and then inspecting. The daemon now asks twice before losing an inspection
over it, which is worth half a second when a minute of model is about to follow.

## Would a bigger model fix it? Cosmos 3, measured on the same frames

Measured on 2026-09-01, after the POC above. The short answer is **no, and it
would make the world state worse rather than merely emptier.**

### Cosmos 3 will not run on this rover, and not for want of tuning

NVIDIA ships Cosmos 3 in three sizes — Super at 64B, Nano at 16B and Edge at 4B.
Edge is the one aimed at Jetsons, but at the AGX Orin rather than this 8 GB Orin
Nano, and it is a genuinely new architecture: `cosmos3_edge`, with ReLU²
activations and a 131k vocabulary, so it is **not** the Qwen3-VL derivative that
made Reason 2 a drop-in. No GGUF of it exists and llama.cpp cannot load it.
Teaching llama.cpp the architecture is a project, not the afternoon that the
`PhysicalReasoner` boundary was supposed to reduce a model swap to.

What can be run is a community extraction of Nano's *understanding path* — the
reasoning and vision half, which really is a Qwen3-VL-8B-class model, re-keyed
and converted with mainline tooling. That is a fair stand-in for "a bigger, newer
Cosmos", and it is what was tested.

It needs **6.19 GB of weights** (5.03 GB of model, 1.16 GB of vision projector).
The headroom was measured rather than guessed: the running 2B sidecar holds
2.97 GB of *anonymous* memory on top of its mapped weights, so stopping it would
leave about 5.85 GB free on a board with **no swap**. The weights alone do not
fit, before the 8B's own KV cache. Nothing was deployed; the evaluation was run
off the rover, against the rover's own recorded frames.

### The experiment

The rover was parked, and the gimbal frames and poses of three inspections were
kept: a forward view of the armchair and the glass table, **the same view again
without anything moving**, and — after turning 180° in two 90° steps, reversed
afterwards to unwind the tether — **a second armchair of the same design** across
the room, with a sheepskin throw over its back.

`world_state`'s own prompt, JSON schema, validation and association rules were
imported and pointed at a different server, so the only thing that changed
between runs was the model. The harness reproduces the rover's live result
exactly, which is what makes it trustworthy.

### Cosmos 3 sees the room better

No argument on perception. Cold, with nothing known, it described the twin-chair
frame as "a black leather armchair with a sheepskin blanket ... a dining area to
the left, and a floor lamp to the right", and picked out the sheepskin, the
doorway, the dining table, the floor lamp and a wall-mounted shelf — every one
real, every box roughly right. Reason 2 on the same frame offers "sofa, coffee
table, doorway", one of which is not there.

### But it cannot say "this is new", and that is worse than never matching

| | Cosmos Reason 2 2B | Cosmos 3 (Nano understanding path, 8B) |
|---|---|---|
| the identical frame, replayed | 0 of 4 matched — four fresh duplicates | 6 of 6 matched, correctly |
| decoys planted in the known list | — | ignored them, matched the six real ones |
| a scene with **none** of the known entities in it | creates new entities, correctly | **matches them anyway, and writes them into the picture** |
| told nothing has been named yet | leaves the field out | invents `object:1` … `object:6` |

The third row is the finding. Shown a known list containing only a **grand piano**
and a **fish tank**, neither of them in the room, Cosmos 3 matched the armchair to
the piano, matched the glass table to the fish tank, and then **hallucinated a
fish tank into the scene** — box, "a glass aquarium with gravel and green plants,
lit from above" copied verbatim out of the list it had been handed, and a mention
in the scene sentence. Byte-identical on a repeat run at temperature zero. The
same thing happened unprompted in the twin-chair frame, where four observations
came back sharing one meaningless box and descriptions copied from the previous
view.

So the known list does not merely fail to settle identity: **it contaminates
perception.** The model reads it as a list of things it ought to be seeing.

That also disposes of the good news in row one. Matching 6 of 6 on the identical
frame is not evidence of recognition, because the same model emits the same list
against a scene that shares nothing with it. It is agreement by repetition.

Two things were ruled out before concluding any of this. The JSON schema permits
`null` and does not require the field at all, and llama.cpp's grammar really does
admit a bare `null` — checked directly against the running server — so refusing to
say "new" is the model's choice and not the harness's cage. And the model card's
warning that Qwen-VL needs at least 1024 image tokens for grounding was taken
seriously: at that setting the fish tank stops being drawn into the scene, but
both the armchair and the glass table are still forced onto the grand piano, the
twin-chair frame still comes back as five carried-forward matches, and a single
look costs **164 s on a fast desktop CPU** — a configuration this rover could not
afford even if the weights fitted.

### What it means

Reason 2's failure is conservative: it never matches, so it over-creates, and the
store stays truthful about what was actually seen. Cosmos 3's failure is
corrupting: it over-matches, so the store fills with wrong identities and with
objects that were never in the room. **For a rover, whose whole job is to keep
meeting things it has not seen before, "cannot say new" is the more dangerous of
the two.**

This closes the "try Cosmos 3" fork. The remaining route is the one in
[docs/task-semantic-world-state.md](../docs/task-semantic-world-state.md):
take identity away from the model and give it to measured placement and
appearance. One amendment to that document falls out of this: it allows Cosmos to
keep receiving the known entities "for semantic context", with its identity answer
treated as advisory. The evidence here says go further and **withhold the known
list from the perception call altogether**, because what it corrupts is the
detections themselves, not just the identity field.

### A trap left behind for the next model

Cosmos 3 writes the *string* `"null"` where Reason 2 leaves the field out, which
the validator refused as an invented identifier — quietly throwing away every
observation in three perfectly good answers. `contract.py` now reads the null-words
as no entity, while still refusing anything that merely looks like an identifier.
Any model swapped in behind `PhysicalReasoner` is worth checking for the same
class of thing before its answers are believed.
