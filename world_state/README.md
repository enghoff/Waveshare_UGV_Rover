# Semantic world state

What the rover has been told is in the room, kept apart from where things are.

SLAM Toolbox and Nav2 own geometry: the occupancy grid, the pose, the routes. This
component owns the other kind of memory — that there is a grey sofa, that it was
seen three times from three places, and where the rover was standing each time.
Nothing in here drives, plans or refuses a move, and nothing in here is offered to
a voice model.

The plan it belongs to is [`docs/task-semantic-world-state.md`](../docs/task-semantic-world-state.md);
the earlier design it was first built to, [`docs/cosmos-reason2-integration.md`](../docs/cosmos-reason2-integration.md),
is now history rather than instruction, for the reason below.

## Where this stands

The question this component was built to answer has an answer, and it is no.

> Does the model build and maintain a description of the environment that stays
> coherent as the rover sees the same place from different views?

It does not, and no better model fixes it, because **the information is not in the
picture**: two identical chairs at opposite ends of a room are identical in the
image, and what separates them is where they are. The measurements behind that
conclusion are further down; the short version is that Cosmos Reason 2 never
recognises anything it has named, and Cosmos 3 recognises things that are not in
the room and writes them into the scene.

So the model has lost the job. **It is now shown one picture and told nothing
about what the rover has already seen**, and what it returns is a kind, a name and
a box — no description, no location hint, and no claim about which lasting thing
it is looking at. Identity will come from a triangulated map position, and until
the resolver that does that arrives, an inspection records observations and no
entity is created at all. `entities` is an empty table waiting for a better
answer, not a dead one.

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
              validate -> observation
                      |            with the gimbal angles and the rover pose
                      v            that will one day place it
        SQLite + JPEGs under ~/.ugv/world
                      |
                      v
      control calls on TCP 8769 -> drive console popup
```

## The one rule

**The model proposes; the application disposes.** It never allocates an
identifier, never claims which lasting thing it is looking at, never writes a row
and never states a distance. What it returns is a proposal that
[`contract.py`](contract.py) validates and [`store.py`](store.py) records, and
every path through [`inspector.py`](inspector.py) that fails leaves the world
exactly as it was, with one line in the diagnostics log saying which failure it
was.

That rule has grown teeth. It used to mean the model could only refer to an
identifier it had been shown; it now means there is no field it could use, no
grammar that would let it produce one, and a validator that strips an identity
claim out of the answer and reports having done so — the same treatment metres
have always had. What is left is deliberately blunt:

1. every observation is stored exactly as the model said it, with the frame, the
   gimbal angles and the rover pose behind it;
2. no observation is ever rewritten;
3. nothing is matched to anything, because nothing yet knows where anything is;
4. a label that names nothing in particular — "a thing" — is kept and marked as
   unmatchable, because it is what the model said.

There is deliberately no cheap stand-in in the meantime. Matching on the label
would be the obvious one, and the rover has already measured what it would be
worth: the same chair came back as a "black leather recliner" and then, on the
byte-identical frame, a "blue leather recliner", with its twin becoming a "black
leather couch". An empty entity table beats confident wrong answers.

## Observations and entities are different things

An **observation** is what the model said about one picture at one moment. It is
never rewritten. An **entity** is the application's current opinion about a lasting
thing in the room, derived from observations.

Collapsing the two — letting an answer update a row in place — would destroy the
evidence, because "the sofa's description changed" and "the model saw a different
sofa" would leave the same record behind.

Keeping them apart is what makes the current state survivable. Nothing creates an
entity at the moment, so every observation is an orphan; but each carries the
frame, the gimbal angles and the rover pose behind it, which is exactly what the
resolver will need to place it later. **Nothing has to be re-photographed when
identity arrives.**

Three columns are written by nothing and kept anyway: `description` and
`location_hint` on observations, and `known_count` on inferences. All three
belonged to asking a model which lasting thing it was looking at. The rows holding
them are the evidence for that having failed, so columns are added and never
dropped, and `store._add_columns` adds new ones to a database an older build
created.

There is no `state` column on entities, although the task's suggested schema lists
one: nothing would write anything but `present` into it and nothing would read it.
Whether an entity has gone quiet is `last_seen_at`; whether it belongs to a map
that no longer exists is `map_session`. Both are answered from columns something
actually writes.

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

That provenance is now the whole basis of identity rather than a decoration on it.
[`view.py`](view.py) turns one observation into a **bearing from a measured pose** —
a cone from where the rover stood, along where the camera pointed, narrowed by
where in the picture the thing sat — and [`locate.py`](locate.py) turns two such
bearings, taken from places far enough apart, into a point on the map with an
uncertainty attached. One bearing is a direction and not a position, so a thing
gets located only once the rover has driven between two looks. No pose or no
gimbal angle means no cone, rather than a cone from the origin.

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
~/ugv/world_state/vendor/     weights, llama.cpp, the ONNX models and the two
                              unpacked wheels, fetched by the two install scripts
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
python deploy/deploy.py --only world_state          # copies; fails if no models
ssh orin '~/ugv/world_state/install.sh'             # ~2 GB, the language model
ssh orin '~/ugv/world_state/install_perception.sh'  # ~0.5 GB, the three encoders
python deploy/deploy.py --only world_state          # now passes
```

Two installers because there are two model sets and they have to break
independently. `install.sh` fetches a pinned Q4 GGUF of Cosmos Reason 2 2B and its
vision projector, and unpacks a pinned aarch64 `llama.cpp`.
`install_perception.sh` fetches FastSAM, DINOv2 and SigLIP2 as ONNX graphs and
unpacks ONNX Runtime and the SigLIP tokenizer as wheels. Both check what they
fetch against its expected size, add their sidecar's `@reboot` crontab entry, and
resume a part-fetched file — which is most of why they are worth re-running rather
than starting again.

**Both halves now use the GPU, and they get there by different roads.** The
language model goes through Vulkan, which needs nothing but a 27 MB llama.cpp
build. Perception goes through TensorRT, which comes from JetPack. What it does
*not* go through is ONNX Runtime, and that is worth stating because it is the
thing most likely to be tried again: CUDA and cuDNN are perfectly available for
this board from NVIDIA, but **no build of ONNX Runtime exists for JetPack 7**.
The community Jetson wheel index stops at JetPack 6, and the official aarch64
wheel on PyPI carries compiled kernels for every architecture except this Orin's
own sm_87 — so it finds the GPU, opens a session on it, and dies at the first
kernel launch with "no kernel image is available for execution on the device".
Installing more CUDA does not help; the gap is inside the wheel.

## Running it

```bash
ssh orin '~/ugv/world_state/restart.sh'                  # reload the language model
ssh orin '~/ugv/world_state/restart_perception.sh'       # reload the encoders
ssh orin '~/ugv/world_state/restart.sh --supervisor'     # after changing run_*.sh
ssh orin 'tail ~/ugv/world_state/cosmos.log'
ssh orin 'tail ~/ugv/world_state/perception.log'
```

Use the restart scripts rather than relaunching the `run_*.sh` by hand: the
supervisors are where the flags live, and the `pkill` patterns live in files where
an ssh command cannot match itself. That last point is not theoretical — writing
`pkill -f llama-vulkan/llama-server` into an ssh command while writing this killed
the session mid-sentence, for the fourth time in this repository's history.

## The perception half

Three ONNX models in a sidecar of their own on loopback 8776, and between them
they know no categories at all.

```text
                                                    GPU        CPU
FastSAM-s      what regions are in this frame         5 ms     418 ms
DINOv2-small   is this the same instance as that     70 ms   1 137 ms   12 crops
SigLIP2        what is it called, and text search    42 ms     854 ms   12 crops
```

**Which backend runs is decided by whether the board has engines built for it**,
and every look and every health check says which one answered. The GPU is the one
to want. It is about sixteen times faster, and it is also *more accurate*: against
a full-precision reference on the rover's own frame the engines agree to 1.000
where the int8 graphs the CPU path runs agree to 0.86, and the CPU's names are
visibly worse — three regions called "a bookcase" where the reference says a
window, a television and a chair. Dynamic quantisation is what costs that, since
it recomputes its scales from each activation rather than from a calibration set.

The CPU path is kept because an engine is not a model file: it is compiled for one
GPU and one TensorRT version, so a fresh install or a JetPack upgrade leaves a
rover that must still be able to see. **Vectors from the two backends must never
be compared with each other**, which is why the backend that produced one travels
with it.

Building the engines is the slow part of installing — about ten minutes, once, and
it needs the board largely to itself. The first attempt with the language model
still resident ended with the kernel's out-of-memory killer taking the language
model and the build together, so `install_perception.sh` now stops that sidecar
for the duration and starts it again afterwards.

The vocabulary is [`vocabulary.txt`](vocabulary.txt) and nothing in the models
knows it. A region's name is the nearest phrase to its stored SigLIP2 vector, so
editing that file re-labels every object the rover has ever seen without
reprocessing a frame.

```bash
ssh orin 'cd ~/ugv/world_state && python3 bench_perceive.py'   # what a look costs
curl -s 127.0.0.1:8776/health
```

### Three things that were measured rather than assumed

**fp16 does not merely blunt SigLIP2, it destroys it.** Built as an fp16 engine,
the text tower collapsed: all fifty-seven vocabulary vectors came back within 0.92
of one another, so a single phrase won every region in the frame and every label
was wrong. The image tower went with it, agreeing with full precision to only
0.71. This is invisible until the work reaches a real GPU, because ONNX Runtime
has no fp16 kernels on the CPU and quietly computes such graphs in fp32 — so a
model that "works in fp16" on a desk can be worthless on a board. The engines are
therefore built at full precision, except the region finder, whose boxes in fp16
match the CPU's to a mean overlap of 0.998.

**Turning off onnxruntime's spin-waiting is worth 3.3x.** This applies to the CPU
fallback only, and it is still true there. Three sessions run one
after another on every look, and by default each one's thread pool keeps spinning
after its own work is done — so FastSAM's threads burn cores through DINOv2's turn,
and DINOv2's through SigLIP2's. A look is 2.64 s with spinning on and 0.80 s with
it off, on the same models. Each model alone is exactly as fast either way, which
is precisely why it is easy to miss.

**SigLIP2 patch32-256 beats patch16-224 at both jobs.** The patch16 model is the
obvious choice and the worse one here: 66 ms a crop against 24, and on the rover's
own living-room frame it called the spray bottle a cardboard box and the armchair a
sofa, where patch32 named the spray bottle, the armchair and the framed picture
correctly. Sixty-four patch tokens rather than a hundred and ninety-six, and the
crops are small.

**A vocabulary of mixed phrase lengths has one entry that always wins.** An early
list had "a power cable on the floor" among two-word phrases and it beat everything
on every region in every frame, including an armchair and a framed picture. A
longer, more circumstantial caption matches a whole scene better than a short one
does. Keep them two or three words, an article, no clauses.

### Appearance is weaker than the plan assumed, and that matters

The plan records DINOv2 scoring the same chair at 0.995 and the twin chair across
the room at 0.653, and treats appearance as a usable tiebreaker on that basis. On
this rover's own frames, at the model's native input size:

| | DINOv2 |
|---|---:|
| the same chair, two crops of the same view | 0.981 |
| **the same chair, across a real change of viewpoint** | **0.696** |
| **the twin chair across the room, seen from a similar angle** | **0.735** |
| the chair against the spray bottle | 0.122 |

**The twin scores higher than the object itself.** The plan's 0.995 is the first
row — same object, same viewpoint — which is a measure of the crop, not of the
object. Once the viewpoint genuinely changes, appearance answers "does this look
like that picture" rather than "is this the same thing", and the two come apart
exactly where the rover needs them not to.

This does not change the design; it strengthens the reason for it. Geometry was
always going to be the arbiter, and this says appearance cannot even be trusted as
a strong tiebreaker: the resolver must never let a high appearance score overrule
an incompatible placement, which is the redundant-furniture test the whole thing
exists to pass. Appearance is kept because it is nearly free beside a bearing, it
separates a chair from a bottle without effort, and it is safe once placement has
already narrowed the field to one candidate.

### The language sidecar was eating the board, and had been all along

`llama-server` defaults to **8192 MiB of prompt cache**. This board has 7485 MiB
and no swap. So the sidecar grew by about a hundred megabytes an inspection and
never gave any back: sixteen inspections took the rover from 1.5 GB free to 48 MB,
which is where the out-of-memory killer starts choosing between the language model
and the process that owns STOP.

`--cache-ram 0` in `run_cosmos.sh` settles it at 3.2 GB after six inspections and
holds there, with no inspection any slower — every request here is a new picture
with a new prompt, so there was never anything in that cache worth reusing.

**This was not new and was not caused by the switch to Vulkan**: the CPU build
leaks at the same rate, measured directly. The note further down that "the sidecar
holds about 4 GB resident" was this leak caught halfway up, and it went unnoticed
because nobody had run more than six inspections in a row.

### What the GPU is worth

Switching llama.cpp to its Vulkan build took an inspection from **38 s to 9.5 s**,
and to 10–12 s with the perception sidecar loaded alongside. Twenty-seven megabytes,
no toolchain, one flag. The same binary carries the CPU backends, so
`--n-gpu-layers 0` is the way back if the driver ever misbehaves.

Moving perception to TensorRT took a look's model time from about 2.4 s to about
120 ms, on the same frame with the same twelve regions. That one cost 9.4 GB of
JetPack and an installer that takes ten minutes longer, and it bought accuracy as
well as speed — see the perception section above for what the int8 CPU path was
getting wrong.

Ten alternating rounds of a look and an inspection, with everything loaded: llama
flat at 3228 MB, perception flat at 1216 MB, about 1.65 GB free throughout, and
**zero dropped lidar scans**.

## The calls

All of these are control calls on the daemon's TCP 8769, and none of them is in
`list_tools`. This slice exists to find out whether the world state is worth
trusting; giving a model the authority to write to it, or to throw it away, before
that has an answer would be the wrong order.

| Call | What it does |
|---|---|
| `world_state_summary` | counts, the last inference, the last few outcomes |
| `world_state_entities` | every entity, its rays, the observation stream, the unmatched — no entities exist yet, so this is the observation stream |
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
python world_state/selftest.py        # the store, the rules, the geometry
python rover_daemon/selftest.py       # the daemon's control calls
python drive_web/selftest.py          # the console's payload and its two URLs
```

Everything there runs against `FakeReasoner` and a temporary directory. **That
proves the store, the rules and the arithmetic, and nothing whatever about any real
model** — which is why the fake is development scaffolding rather than a result.

Three checks are worth knowing about by name. One asserts that an identity the
model volunteers is thrown away rather than obeyed, because a stale build or a
model with an opinion of its own is exactly how a guess would creep back into
being believed. One asserts that the prompt never mentions what the rover has seen
before — not even to forbid it, since naming the subject in order to forbid it is
still naming it. And one opens a database built to the older schema and checks that
the missing column is added rather than the insert failing on the one machine that
matters.

Two things are deliberately not covered. The popup's rendering is JavaScript in a
browser and this repository has no browser in its test loop; what is checked instead
is the payload it draws from, the two URLs it fetches, and -- since a tab whose pane
is never unhidden is a tab that does nothing -- that every element the page's script
reaches for by name exists in its markup. **Nobody has yet opened the page and looked
at it**, which is the gap that matters: until the validation drive there was nothing
placed for it to draw.

And whether placement really separates two identical chairs can only be measured on
the rover, driving. That drive happened on 2026-09-02 and is written up in
`docs/task-semantic-world-state.md`: twenty-three things placed from three positions,
including a person ten centimetres from the armchair he was sitting in. It does not
settle the identical-chairs question, though, because nine duplicate entities came
with it -- and a rover that cannot merge keeps two chairs apart whether or not it can
tell them apart.

## What was measured on the rover, 2026-09-02

A second day of measuring, after the perception models moved to the GPU. Three
things came out of it and each changed a constant.

**A bearing is good to 1.5 degrees, not 5.** Measured in three parts, because
knowing which part dominates says whether it is worth improving: the box
contributes 0.13 degrees of scatter over eight inspections of an unchanging
scene, the rover's own heading 0.2 over two minutes standing still, and the
gimbal the rest -- within 0.7 degrees out to plus or minus fifteen, and about
three at minus thirty. That last is the pan servo not arriving where it was told,
and it is the same size and sign for two objects on opposite sides of the frame,
which rules out the lens; the lens checks out to within 0.9 degrees of its own
calibration. There is nothing to correct it with, because the driver board's
telemetry carries the inertial sensors and the wheel encoders but no gimbal
feedback. **A rover that inspected only within fifteen degrees of centre would do
better than 1.5 says.**

**A search is decided by the raw score, not by how far it stands clear of the
field.** The module shipped believing the opposite. Forty queries against the
rover's own stored regions -- twenty-four for things it had seen, in the
vocabulary's words and in other people's, and sixteen for things that are not in
the building -- separate almost perfectly by raw score and not at all by
separation. The best cut any threshold on the separation could make still gets
fourteen of the forty wrong; a floor at 0.09 on the score gets four, and three of
those are the safe way round.

**Better geometry cost the resolver answers until it was told not to.** The rival
test refuses a placement when a competing crossing sits further away than their
combined uncertainty allows. Shrinking the bearing error shrank those
uncertainties with it, so a pair of crossings thirty centimetres apart -- which
had comfortably overlapped -- became a standoff and the resolver stopped placing
chairs it had been placing. Two placements within half a metre now name the same
thing whichever is right.

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

That amendment has since been made and the code follows it: the known list, the
identity field and the whole question are gone from the perception call, and the
prompt does not mention any of it even to forbid it.

### A trap left behind for the next model

Cosmos 3 writes the *string* `"null"` where Reason 2 leaves the field out, which
the validator refused as an invented identifier — quietly throwing away every
observation in three perfectly good answers. That particular trap has been
disarmed by removing the field it was set in; the class of thing it belongs to has
not. Any model swapped in behind `PhysicalReasoner` is worth reading raw output
from before its answers are believed, because what makes this kind of bug
expensive is that it looks like the model saying nothing.
