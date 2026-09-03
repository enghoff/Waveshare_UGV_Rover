# Semantic world state

What the rover has seen in the room, kept apart from where things are.

SLAM Toolbox and Nav2 own geometry: the occupancy grid, the pose, the routes. This
component owns the other kind of memory — that there is *something* at the far end
of the room, that it was seen three times from three places, and where the rover
was standing each time. Not what it is called: nothing measures that, and the two
attempts to are written up below. What a person gets instead is the picture each
look was read from, with the box on it, and a search box that takes a description
and compares it against what the rover actually saw. Nothing in here drives, plans
or refuses a move, and nothing in here is offered to a voice model.

The plan it belongs to is [`docs/task-semantic-world-state.md`](../docs/task-semantic-world-state.md);
the earlier design it was first built to, [`docs/cosmos-reason2-integration.md`](../docs/cosmos-reason2-integration.md),
is closed history rather than instruction, for the reason below — the local
language model it describes is no longer on the rover.

## Where this stands

**A second drive on the afternoon of 2026-09-03 placed nothing false and almost
nothing at all: 866 regions over 214 looks, and one entity.** The cause is that
**163 of those 214 looks stored no direction for anything they saw**, because the
limit on how far the rover may travel while the shutter is open was derived from a
0.29 s capture at 0.35 m/s and the rover now takes 0.36 s and drives at 0.47 —
so an ordinary look taken while driving straight was refused by a hair. That left
94 bearings from eight standing places out of a thirteen-minute drive. Travel now
buys a wider answer instead of no answer, turning still costs the look its
bearing, a standoff between two crossings no longer ends the whole pairing pass,
and the appearance floor is 0.55 rather than 0.5 — measured over 3,741 pairs of
regions taken from one frame, which are different things by construction, and
where 0.5 was admitting a fifth of them. On that recording the three together take
the run from one entity to two, both of which are things a person can point at in
the crops, where the standoff fix on its own placed five of which three were two
objects each. **The size of the recovery is a prediction and wants the next
drive**: the looks that lost their bearing recorded no pose either, so it cannot be
replayed. The write-up is *One entity out of a thirteen-minute drive* below.

**The run of that morning placed four things and three of them are where nothing
is.** One entity is right: `object:3` is three crops of one framed picture. The
other three are not, and between them they say what is left to fix. `object:1`
holds a dark cabinet, a lit doorway and a blue sofa; `object:2` a doorway seen
from across the room and a door frame seen up close; `object:4` six crops of one
picture and the pole of a floor lamp. **Every one of the four sits in open floor
between 0.6 and 2.8 m from the camera that saw it, while every crop is of
something on a wall several metres away.** The write-up is *Three things placed
where nothing is* below. One of the three causes is fixed here; the other two are
measured and are not.

**A fifth fault was found on 2026-09-03 and it is upstream of all of them: the
rover works out its bearings with the model face tracking was measured off and
moved away from a fortnight earlier.** `view.py` turns a box into an angle with one
multiplication, which on a 130-degree fisheye on a tilting gimbal is only right
along the two centre lines — and the gimbal's own tilt, recorded on every
observation, is dropped rather than used. On the 71 boxes of that morning's
recording the median error is 1.34 degrees and the worst is 14.5, against the 1.5
that `BEARING_SIGMA_DEG` promises the geometry: **half of every bearing the rover
has ever stored is outside the accuracy the resolver is told to expect.** It is
measured and **not fixed** — the write-up, the shape of the fix and why it wants
its own drive are under *The bearing is worked out with the model aiming was
measured off* below. It is also the best current explanation for the open problem
two paragraphs down.

**A fourth fault was found on 2026-09-02 and it is the one that mattered most:
the rover was placing things through walls, and its own map said so.** An entity
held observations of two different objects in two different rooms, and what joined
them was not appearance at all — it was that two bearings pointed at two
different things a couple of metres away and crossed ten metres off, outside the
edge of the map. Nothing asked whether the rover could have seen that far in that
direction. It can now, out of the occupancy grid; the write-up is *You cannot see
a thing through a wall* below.

**Three earlier faults were found the same way, and two of them were upstream of
everything the design argues about.** The write-up is
under *What replaying a real run found* below; the short version is that the pose
an observation was recorded against was not where the rover was, that a sixth of
the regions it stored were pictures of nothing, and that a placed thing would
move out from under the looks that placed it. All three are fixed. What is not
fixed, and is now the open problem, is that **an entity is still often a mixture
of several objects** — see the same section for why the resolver's own rules
turn out not to be where that comes from.

[`replay.py`](replay.py) is the harness that found them: it feeds a database the
rover wrote back through the live resolver, one look at a time, and scores what
comes out. Replaying an unchanged build reproduces the rover's entities exactly,
which is what makes a change measurable before it flies.

The question this component was built to answer has an answer, and it is no.

> Does the model build and maintain a description of the environment that stays
> coherent as the rover sees the same place from different views?

It does not, and no better model fixes it, because **the information is not in the
picture**: two identical chairs at opposite ends of a room are identical in the
image, and what separates them is where they are. The measurements behind that
conclusion are further down; the short version is that Cosmos Reason 2 never
recognises anything it has named, and Cosmos 3 recognises things that are not in
the room and writes them into the scene.

So the language model lost the job, and then — on 2026-09-02 — it left the rover
altogether. **An inspection measures rather than asks.** A frame goes to the
perception sidecar, which draws regions with YOLOE and describes each one with a
DINOv2 appearance vector and a SigLIP2 semantic vector; the store keeps those
beside the gimbal angles and the rover pose, and identity is settled afterwards by
bearings that cross. Nothing in this component sends a picture to a language model
any more, and there is no local one on the rover to send it to: the sidecar, the
2.1 GB of weights and the `PhysicalReasoner` boundary in front of them are all
gone. A person who wants prose about what the camera can see asks the daemon's
`look`, which puts the frame in front of the conversation's own model.

```text
                gimbal camera
                      |            rover_daemon owns it, and takes the picture
                      v            through the same path camera_jpeg uses
              perception_client
                      |
                      v            loopback 8776, its own process
        YOLOE -> DINOv2 + SigLIP2    (TensorRT on the GPU, ONNX on the CPU)
                      |
        a box and two vectors per region
                      |
                      v
                 observation
                      |            with the gimbal angles and the rover pose
                      v            that turn a box into a bearing
        SQLite + JPEGs under ~/.ugv/world
                      |
                      v            resolve.py: two bearings that cross
      control calls on TCP 8769 -> drive console popup
```

## The one rule

**Perception proposes; the application disposes.** The sidecar never allocates an
identifier, never claims which lasting thing it is looking at, never writes a row
and never states a distance. What it returns is a measurement that
[`store.py`](store.py) records, and every path through
[`inspector.py`](inspector.py) that fails leaves the world exactly as it was, with
one line in the diagnostics log saying which failure it was.

That rule has grown teeth. It used to mean a language model could only refer to an
identifier it had been shown; then that there was no field it could use and a
validator that stripped an identity claim out of the answer; now there is nothing
in the pipeline capable of forming an opinion about identity at all. What is left
is deliberately blunt:

1. every observation is stored exactly as it was measured, with the frame, the
   gimbal angles and the rover pose behind it;
2. no observation is ever rewritten;
3. what a thing is *called* is not recorded at all, because nothing here can
   measure it — see below.

There is deliberately no cheap stand-in. Matching on a name would be the obvious
one, and the rover has measured what it would be worth twice over. Asked of a
language model, the same chair came back as a "black leather recliner" and then,
on the byte-identical frame, a "blue leather recliner", with its twin becoming a
"black leather couch". Asked of a fixed word list and a SigLIP2 vector, the score
sat between 0.08 and 0.12 whatever the crop held, so only the ranking meant
anything — and the ranking put "a computer monitor" on a sofa. An empty entity
table beats confident wrong answers.

## Nothing is named, and that is a result rather than an omission

**A region has a box, two vectors and no word.** It used to carry the nearest
phrase in `vocabulary.txt` to its SigLIP2 vector, and that name reached the
console, the entity list, the search results and — worse — the resolver's first
gate, which threw out any candidate whose name was not a synonym of this one.
All of it is gone: the file, the start-up embedding of it, the cached vectors,
the score beside the name, the synonym families, the list of things that move,
and the `furniture:`/`opening:` identifier prefixes that were guessed from the
word.

The reason is that the number underneath was never a measurement. Those scores
sit in a band of 0.08 to 0.12 whichever crop they are taken of, so there is no
threshold at which the answer is "none of these" and no sense in which one name
is more confident than another. What is left when you strip the calibration away
is an argmax over fifty-seven phrases, and it is wrong often enough to see: on
the rover's own frame it named a sofa a computer monitor.

**The same vector answers the same question honestly when the question comes
from a person.** Forty queries typed against the rover's own stored regions
separate present from absent almost perfectly at a floor of 0.09 — four wrong
out of forty, three of them the safe way round. So the vector is stored and
[`search.py`](search.py) asks it what somebody actually wants to find. The
picture was never the problem; naming it without being asked was.

Two things had to replace what the name was doing.

- **The resolver's semantic gate** is now `DIFFERENT_THING` in
  [`resolve.py`](resolve.py): two crops whose DINOv2 vectors are less alike than
  0.5 are not two looks at one object, so the candidate is removed. It removes
  only and never confirms, exactly as the synonym gate did, and it sits between
  two numbers this rover measured — a chair against a spray bottle at 0.122, and
  the same chair across a real change of viewpoint at 0.696.
- **The console** shows a thing by its identifier and its pictures. Choosing an
  entity fills the pane beside the list with every look that was decided to be
  it, each drawn as the stored frame with the measured box on it, scrolling on
  its own. Whether four observations really are one object is a question the
  crops answer and a word never did.

One thing is not replaced and is worth stating plainly: the resolver used to
know which things move, from a list of names, and refuse to identify a bottle by
position alone. It cannot know that any more. What survives is the half that
needed no name — a look whose crop does not resemble what the entity has shown
is not that entity however well the bearing points at it.

## Observations and entities are different things

An **observation** is one region of one picture, as measured at one moment. It is
never rewritten. An **entity** is the application's current opinion about a lasting
thing in the room, derived from observations.

Collapsing the two — letting a later look update a row in place — would destroy
the evidence, because "this thing looks different now" and "the rover was looking
at something else" would leave the same record behind.

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
frame_id / frame_path                   the JPEG the encoders were given
map_session                             which SLAM map was live
model_id / vectors_from                 which backend measured it
```

**The line is drawn by who measured the number, not by whether there is a number.**
Where the gimbal was pointing and where the rover was standing are readings the
rover already takes. How far away the sofa is would be a guess from a single
photograph, and nothing here is in a position to make one: the sidecar returns a
box in fractions of the frame and two vectors, and metres enter this component
only from SLAM. `prompt_version` is a column the language model used to fill and
nothing writes now.

That provenance is now the whole basis of identity rather than a decoration on it.
[`view.py`](view.py) turns one observation into a **bearing from a measured pose** —
a cone from where the rover stood, along where the camera pointed, narrowed by
where in the picture the thing sat — and [`locate.py`](locate.py) turns two such
bearings, taken from places far enough apart, into a point on the map with an
uncertainty attached. One bearing is a direction and not a position, so a thing
gets located only once the rover has driven between two looks. No pose or no
gimbal angle means no cone, rather than a cone from the origin.

Once a thing has been placed, `view.relate` answers the other half of it: how one
look stands to the one position the application settled on — how far away it is
from there, how many degrees off the bearing was, how far that misses by across
the line of sight, and whether that is inside what the resolver allows. It calls
`locate.agrees` against `locate.match_tolerance`, which is the same pair
`resolve._against_known` uses to attach a look in the first place, so **a look the
console draws as off the thing is a look the rover would not attach today.** That
is what the drive console's map is built from; the drawing is described in
[`drive_web/README.md`](../drive_web/README.md).

## Clearing, in both directions

Only one direction is obvious.

- **Clearing the semantic world does not touch the map.** It cannot: this process
  does not own the map. `world_state_clear` deletes entities, observations, the
  diagnostics log and the stored frames, and leaves SLAM and Nav2 alone.
- **Clearing the SLAM map clears the semantic world with it.** It did not always:
  entities were meant to outlive the maps they were seen under, and a map clear
  only started a new map session so that observations recorded against the old map
  stayed recognisable as belonging to one that no longer exists. That is
  defensible and it is not what a person wants. Everything the store holds is a
  position or a bearing measured in the map's own frame, so what survived a clear
  was a list of things with nowhere to be, and in practice the two were always
  cleared together. There is one button now and it is the map's; it still starts a
  new session afterwards, so a clear that half fails cannot leave old coordinates
  comparable with new ones. The console owns that button, so it tells the store;
  nothing polls for it.

## Where things are

```text
~/ugv/world_state/            this component, deployed
~/ugv/world_state/vendor/     the ONNX models, their TensorRT engines and the two
                              unpacked wheels, fetched by the install script
~/.ugv/world/world.db         entities, observations, inferences
~/.ugv/world/frames/          one JPEG per inspection
```

The database and the frames are under `~/.ugv/` for the same reason the TLS keys
are: a source deploy replaces `~/ugv`, and an experiment's results are not source.
The models are in `vendor/`, which the deploy manifest preserves, for the reason
the depth camera's DepthAI tree is — half a gigabyte is neither describable by a
commit nor sensible to send over the rover's wi-fi on every change.

## Installing it

A deploy copies the source and will then **fail its own verification** on a host
that has no models yet, saying so. That is deliberate: the alternative is a
component that deploys clean and cannot answer.

```bash
pip install ultralytics onnx onnxslim                # on this machine, not the rover
python world_state/export_regions.py                 # makes the region finder
scp yoloe-11s-seg-objectness.onnx orin:~/ugv/world_state/vendor/
python deploy/deploy.py --only world_state           # copies; fails if no models
ssh orin '~/ugv/world_state/install_perception.sh'   # ~0.5 GB, the three encoders
python deploy/deploy.py --only world_state           # now passes
```

**The first three lines are the price of running YOLOE**, and they are new. Every
other model here is downloaded; this one is not published in a form the rover can
use, and making it wants ultralytics and torch, which the rover has neither the
room nor the need for. So it is exported on a workstation, copied into `vendor/`
by hand, and `install_perception.sh` checks for it and says this if it is
missing. What the export does beyond the stock one is fold YOLOE's 4,585 class
scores down to their maximum inside the graph — see
[`export_regions.py`](export_regions.py), which is not deployed and never runs on
the rover.

One installer, where there were two: the second fetched two gigabytes of language
model and a llama.cpp server, and both left the rover on 2026-09-02 along with the
code that called them. `install_perception.sh` fetches DINOv2 and SigLIP2 as ONNX
graphs, unpacks ONNX Runtime and the SigLIP tokenizer as wheels, and builds the
TensorRT engines. It checks what it fetches against its expected size, adds the
sidecar's `@reboot` crontab entry, and resumes a part-fetched file — which is
most of why it is worth re-running rather than starting again.

**Perception reaches the GPU through TensorRT, which comes from JetPack.** What it
does *not* go through is ONNX Runtime, and that is worth stating because it is the
thing most likely to be tried again: CUDA and cuDNN are perfectly available for
this board from NVIDIA, but **no build of ONNX Runtime exists for JetPack 7**.
The community Jetson wheel index stops at JetPack 6, and the official aarch64
wheel on PyPI carries compiled kernels for every architecture except this Orin's
own sm_87 — so it finds the GPU, opens a session on it, and dies at the first
kernel launch with "no kernel image is available for execution on the device".
Installing more CUDA does not help; the gap is inside the wheel.

## Running it

```bash
ssh orin '~/ugv/world_state/restart_perception.sh'              # reload the encoders
ssh orin '~/ugv/world_state/restart_perception.sh --supervisor' # after changing run_perception.sh
ssh orin 'tail ~/ugv/world_state/perception.log'
```

Use the restart script rather than relaunching `run_perception.sh` by hand: the
supervisor is where the flags live, and the `pkill` patterns live in files where
an ssh command cannot match itself. That last point is not theoretical — writing
`pkill -f llama-vulkan/llama-server` into an ssh command while writing this killed
the session mid-sentence, for the fourth time in this repository's history.

## The perception half

Three ONNX models in a sidecar of their own on loopback 8776, and between them
they know no categories at all.

```text
                                                    GPU        CPU
YOLOE-11s      what regions are in this frame        16 ms     520 ms
DINOv2-small   is this the same instance as that     70 ms   1 137 ms   12 crops
SigLIP2        what a typed phrase would match       42 ms     854 ms   12 crops
```

**Which backend runs is decided by whether the board has engines built for it**,
and every look and every health check says which one answered. The GPU is the one
to want. It is about sixteen times faster, and it is also *more accurate*: against
a full-precision reference on the rover's own frame the engines agree to 1.000
where the int8 graphs the CPU path runs agree to 0.86 — wide enough to move a
search's ranking. Dynamic quantisation is what costs that, since it recomputes
its scales from each activation rather than from a calibration set.

The CPU path is kept because an engine is not a model file: it is compiled for one
GPU and one TensorRT version, so a fresh install or a JetPack upgrade leaves a
rover that must still be able to see. **Vectors from the two backends must never
be compared with each other**, which is why the backend that produced one travels
with it.

Building the engines is the slow part of installing — about ten minutes, once, and
it needs the board largely to itself. The first attempt ended with the kernel's
out-of-memory killer taking the build, with the language model holding 3.1 GB of
the board's 7.5 at the time; the installer stopped that sidecar for the duration
and started it again afterwards. Neither is needed now that the model is gone, so
the installer stops nothing.

The text tower is never held. It is the largest of the four engines at over half
a gigabyte, nothing on the per-look path wants it, and a search loads it for the
call and gives it back — so an ordinary start-up does not open it at all. That is
new: it used to be loaded at every start to embed the word list.

```bash
ssh orin 'cd ~/ugv/world_state && python3 bench_perceive.py'   # what a look costs
curl -s 127.0.0.1:8776/health
```

### The region finder was swapped, and what that changed

**YOLOE-11s replaced FastSAM-s on 2026-09-02**, and this is what the rover gets
for it. Both models were run over the 38 frames the world state had already
stored — the rover's own rooms, not a benchmark — and put through the same size
filter, the same nesting-aware suppression and the same refusal of crops with no
picture in them, so the only difference between the two columns is the graph that
drew the boxes.

| over 38 stored frames | FastSAM-s at 0.4 | YOLOE-11s at 0.15 |
|---|---:|---:|
| regions kept | 258 | 237 |
| per lit frame | 7.6 | 7.1 |
| lit frames it finds nothing in | 0 | 2 |
| regions the other model also has | 170 | 170 |
| regions only this model has | 88 | 67 |
| regions that survive to the next look of the same scene | 64% | 72% |

**The reason for the swap is the armchair.** On six consecutive looks at a chair
filling the near half of the frame, FastSAM proposed no box on the chair at all —
it drew the television, a doorway, the air purifier and a scatter of small boxes
on the furniture behind — while YOLOE puts one box around the whole chair. Thirty
of the 88 regions FastSAM has and YOLOE lacks are of that kind: a part sitting
inside a whole that YOLOE proposed and FastSAM never did.

**The cost is that this model can only propose things it has a word for.** YOLOE
finds regions by matching them against a built-in vocabulary of 4,585 tags, and
where FastSAM would draw a box around anything at all, YOLOE draws nothing where
nothing in that list fits: 58 of those 88 are places it proposed nothing
whatsoever, and two lit frames come back empty that did not before. That is why
its threshold is 0.15 and not the 0.4 FastSAM ran at — at 0.4 it keeps only 146
regions, and the walk down to 0.15 buys back the chairs round the dining table, a
flipchart and a second doorway before the extra boxes stop being things.

**It costs 10 ms a look**, which is 16 ms on the GPU against FastSAM's 5.7, on a
look that runs to about a fifth of a second; on the CPU fallback it is 520 ms
against 342. Two thirds of that difference is the maximum over the 4,585 class
scores. Folded into the graph it is 16 ms; left outside it, where the stock
export leaves it, the same work is a 99 MB tensor copied off the GPU and 32 ms of
numpy, and the region finder costs 51 ms instead.

The names themselves never leave the workstation. `export_regions.py` folds the
4,585 scores to their maximum inside the graph, so what the rover loads answers
"is something here" and cannot answer "what". It is worth knowing what those
names were, though, because they say what the model is: it called the doorways
"entrance hall", the sideboard a "chiffonier" and the dark sofa a "ticket booth".

### Three things that were measured rather than assumed

**fp16 does not merely blunt SigLIP2, it destroys it.** Built as an fp16 engine,
the text tower collapsed: fifty-seven phrases came back within 0.92 of one
another, so every phrase matched everything. The image tower went with it,
agreeing with full precision to only
0.71. This is invisible until the work reaches a real GPU, because ONNX Runtime
has no fp16 kernels on the CPU and quietly computes such graphs in fp32 — so a
model that "works in fp16" on a desk can be worthless on a board. The engines are
therefore built at full precision, except the region finder, whose boxes in fp16
match the CPU graph's to a mean overlap of 0.998 — measured again after the swap
to YOLOE, over 418 boxes from the rover's own frames, worst case 0.894.

**Turning off onnxruntime's spin-waiting is worth 3.3x.** This applies to the CPU
fallback only, and it is still true there. Three sessions run one
after another on every look, and by default each one's thread pool keeps spinning
after its own work is done — so the region finder's threads burn cores through DINOv2's turn,
and DINOv2's through SigLIP2's. A look is 2.64 s with spinning on and 0.80 s with
it off, on the same models. Each model alone is exactly as fast either way, which
is precisely why it is easy to miss.

**SigLIP2 patch32-256 beats patch16-224 at both jobs.** The patch16 model is the
obvious choice and the worse one here: 66 ms a crop against 24, and on the rover's
own living-room frame it matched the spray bottle to "a cardboard box" and the
armchair to "a sofa", where patch32 got the spray bottle, the armchair and the
framed picture right. Sixty-four patch tokens rather than a hundred and
ninety-six, and the crops are small.

**Phrase length decides a match more than content does.** An early word list had
"a power cable on the floor" among two-word phrases and it beat everything on
every region in every frame, including an armchair and a framed picture: a
longer, more circumstantial caption matches a whole scene better than a short one
does. That was the measurement that killed the word list, and it is still worth
knowing at the search box — a long, circumstantial description will score better
against a busy region than a short accurate one does.

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

_History. The sidecar was removed from the rover on 2026-09-02, which is where
the three gigabytes below went._

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
no toolchain, one flag. That measurement is history — the language model is off the
rover — and it is kept because it is the honest floor for anything like it: even
four times faster, a language model in the per-look path cost ten seconds where
the encoders cost a fifth of one.

Moving perception to TensorRT took a look's model time from about 2.4 s to about
120 ms, on the same frame with the same twelve regions. That one cost 9.4 GB of
JetPack and an installer that takes ten minutes longer, and it bought accuracy as
well as speed — see the perception section above for what the int8 CPU path was
getting wrong.

Ten alternating rounds of a look and an inspection, with everything loaded: llama
flat at 3228 MB, perception flat at 1216 MB, about 1.65 GB free throughout, and
**zero dropped lidar scans**. Those 3228 MB are now free, because that process no
longer exists on the rover.

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
| `world_inspect(settle?)` | take a picture, measure the regions in it, record them; `settle: false` records without deciding identity |
| `world_state_clear` | empty the semantic world; the map is untouched |
| `world_map_session` | the map was cleared, so start a new session |

`world_inspect` is 0.45 s on this board — 0.29 s of camera and 0.16 s of encoders
— where it was a minute when a language model answered it. What is slow now is
deciding identity from the pool afterwards, which grows as the square of what is
waiting: 1.4 s at 500 pending bearings and 8 s at 2000. So the rover's own looking
loop asks for the cheap half alone and settles on its own clock, and the console's
button does both, because somebody who has just pressed it is watching for what
was placed. It still runs on the calling thread, so the daemon goes on answering
STOP, status and the map throughout, and the console still gives it a connection
of its own with a patience of its own — the same arrangement, for the same reason,
as the wi-fi scan.

## Tests

```bash
python world_state/selftest.py        # the store, the rules, the geometry
python rover_daemon/selftest.py       # the daemon's control calls
python drive_web/selftest.py          # the console's payload and its two URLs
```

Everything there runs against `FakeEyes` and a temporary directory. **That proves
the store, the rules and the arithmetic, and nothing whatever about what the real
encoders see** — which is why the fake is development scaffolding rather than a
result.

**A fourth thing runs against what the rover actually saw**, and it is the one to
reach for before changing a rule about identity:

```bash
scp orin:'~/.ugv/world/world.db' /tmp/run.db
scp orin:'~/.ugv/world/frames/*.jpg' /tmp/frames/
python world_state/replay.py /tmp/run.db --frames /tmp/frames --detail
```

To replay with the wall check the rover now applies, save the grid too and pass
`--map`:

```bash
ssh orin 'python3 - <<"EOF"
import json, socket
s = socket.create_connection(("127.0.0.1", 8773), 20)
f = s.makefile("rwb"); f.write(b"{\"op\": \"map\"}\n"); f.flush()
open("/tmp/map.json", "w").write(f.readline().decode())
EOF'
scp orin:/tmp/map.json /tmp/map.json
python world_state/replay.py /tmp/run.db --frames /tmp/frames --map /tmp/map.json
```

A fifth runs at a desk against the same database and needs nothing else. It asks
what the bearing model costs, by comparing the one multiplication `view.py` does
with the swept lens in `face_tracking/lens.py`:

```bash
python world_state/bench_bearing.py /tmp/run.db
```

**`replay.py` cannot judge a change to that model as it stands**, which is worth
knowing before reaching for it: `resolve.ray_of` reads the bearing off the row
rather than recomputing it from the box, deliberately — the bearing is a
measurement taken at the moment of the look — so a recording replays with the
bearings it was recorded with whatever `view.py` now says. Judging a new bearing
model on a recording means teaching the replay to recompute them.

[`replay.py`](replay.py) feeds a recorded database back through the live resolver
one look at a time and scores what comes out — how much of each entity is
something other than the thing it is mostly of, and how many of its own bearings
miss its own position. Replaying an unchanged build reproduces the rover's own
entities exactly, which is what makes the comparison mean anything. Three of the
resolver's rules were swept against a real run this way and none of them turned
out to matter; see *What replaying a real run found* above.

Three checks are worth knowing about by name. One asserts that two looks at the
same room create no entity and claim no match, because a store that matched on
anything cheap is exactly how a confident wrong answer would creep in. One asserts
that a look with no pose stores no bearing rather than a guessed one. And one
opens a database built to the older schema and checks that the missing column is
added rather than the insert failing on the one machine that matters.

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
including a person ten centimetres from the armchair they were sitting in. It does
not settle the identical-chairs question, though, because nine duplicate entities
came with it -- and a rover that cannot merge keeps two chairs apart whether or not
it can tell them apart.

## What replaying a real run found, 2026-09-02

Ninety-six minutes of the rover building its world state by itself: 34 looks, 338
regions, 23 things placed. [`replay.py`](replay.py) puts that recording back
through the resolver at a desk, and reproduces it exactly, so a change can be
measured against what the rover actually saw rather than against a fake.

Two scores come out of it, and neither asks the resolver to mark its own
homework. **Mixed** counts the crops in an entity that belong to something other
than the biggest thing in it, clustered from the stored appearance vectors at a
threshold between what two regions of one frame score (0.32) and what one object
scores across a change of viewpoint (0.70). **Stray** counts an entity's own
bearings that miss its own stated position by more than bearing error and its own
uncertainty allow.

| | entities | mixed | stray |
|---|---:|---:|---:|
| what the rover did | 23 | 19% | **45%** |
| the placement fixed | 21 | 26% | 26% |
| and the two bad inputs removed | 22 | 21% | 18% |
| and the thing's own width, not the candidate's | 25 | 21% | **15%** |

### The pose was not where the rover was

`rover_world._world_pose` read `nav.slam.pose`, and that is not the rover's
position: `nav.slam` is the occupancy grid the map renderer was last handed, and
the pose on it belongs to whoever last asked for a map picture. With a console
open it tracks, because a console polls the map three times a second. With no
console open it stands still while the rover drives. And on a daemon that has
just started it is the placeholder's `(0, 0, 0)`.

The rover recorded all three. The daemon restarted twice during this run, and the
two inspections that followed — 22 regions — went into the database on bearings
drawn from the map origin. Those crossed real bearings 4.8 m away at a healthy
parallax off a healthy baseline, so nothing downstream could tell, and six of the
run's biggest entities are built partly on them.

It asks the navigator now, and **a position the navigator does not trust is no
position at all**: `position_trusted` is slam_toolbox still publishing where the
rover is, and without it the observation is stored with no bearing, which is a
state this store already handles honestly.

### A sixth of the regions were pictures of nothing

58 of the 338, and every one was a window the camera had burnt to white or a bare
patch of wall. The box filter cannot see them — they are the right size and the
right shape — and they do real harm rather than merely wasting a slot, because
**two pictures of nothing resemble each other**. `object:14` was built almost
entirely out of blown-out windows and wandered four metres across the map.

`perceive._blank` refuses them: below 12 of contrast, or more than 60% of the
crop at full white. Every one of the 58 it removes from this recording is a
non-thing, checked by eye.

### A placed thing moved out from under its own evidence

An entity is re-placed from everything attached to it whenever a look joins, and
`locate.best_fix` took the pair of bearings with the smallest uncertainty. That is
a statement about two rays and about nothing else, so one lucky pair could move a
thing with a dozen looks behind it clean out from under all of them: 13 of this
run's 151 re-placements moved more than half a metre and one moved 2.6 m in a
single step. `object:14`'s stored position was agreed by 5 of its own 18
bearings.

`best_fix` counts agreement first now and uses uncertainty only to break the tie,
which takes stray from 45% to 26%. Agreement is counted in rays rather than in
viewpoints, which is the opposite of how `_place_one` counts support, and the two
really are different questions — there a phantom near the camera collects
agreement from half the room, here every ray already belongs to this one thing.
Counting viewpoints instead leaves 37%.

### You cannot see a thing through a wall

**This is the fault that put two rooms inside one entity, and the constraint that
fixes it was in the rover all along.** A bearing carries no range. Two bearings
therefore cross *somewhere*, whatever they are pointed at — and two cameras
aimed at two different things a couple of metres away in two different rooms
produce rays that meet ten metres off, at a healthy angle and off a healthy
baseline. Every guard in `locate` accepted that, because none of them asked the
question that settles it.

The rover placed three things in the run of 2026-09-02 after the first three fixes
were deployed. **Two of them sat outside the edge of its own map**, one 4.7 m past
it; and 19 of the 22 bearings attached to those three claimed a thing further away
than the first obstacle along their own bearing. One claimed something 3.8 m away
through a wall 55 cm in front of the rover.

The appearance vectors never came into it. `DIFFERENT_THING` only ever removes,
at 0.5, and a blown-out window against a picture frame on a yellow wall clears
that easily. Geometry joined them, and geometry was working with rays of unbounded
length.

So the occupancy grid now bounds every sighting. `rover_world._world_reach` walks
the bearing out from the pose and stops at the first occupied cell or at the edge
of what has been mapped, and `locate.beyond_reach` refuses a crossing — and
refuses a later bearing joining one — that sits further out than that. On the
same recording, all three placements go and all 68 observations stay pending,
which is the honest answer: standing in a few places looking at different things
from each, this rover had not earned a single placement.

```text
       the rover at (-0.72, 0.08)         a wall 0.55 m ahead
                    o------------------X- - - - - - - - - - - -> claimed 3.84 m
                                          the crossing lived here
```

Two things about the number. The margin is a metre — `SEE_PAST_M` — and it
is generous on purpose: the map is drawn by a 2D scanner at chassis height while
the camera sits on a gimbal above it, so a sofa two metres away is a wall to the
lidar and something the camera looks straight over. It costs nothing here, because
every placement this refuses is claiming 3.3 to 9.6 m past its own first obstacle
and the whole band from 0 to 3 m refuses the same three. And **nothing revisits a
placement already made**: the guard stops a wrong entity being created and does
not withdraw one that exists, so a store written before this arrived keeps its
mistakes until it is cleared.

### The driven run, and why it placed nothing

The map was cleared and the rover was driven round a flat: seven looks in four
minutes from seven places, 37 regions, **nothing placed at all.** That is the
right answer, and none of it is the wall check, which refused exactly one pair of
bearings out of six hundred. Three things, and the first is the one that matters.

**Triangulation needs two looks that share an object *and* stand apart, and this
rover has never once had both at the same time.** Of the 21 pairs of looks in the
run, three had a usable baseline (0.4 to 3 m) and the pair that shared by far the
most — 33 pairs of crops that could be one object — was taken from two places
**eight centimetres apart**: the rover looked at the same hallway wall at the
start and the end of its loop. Everything else is 4 to 6 m apart, through
doorways, with nothing in common.

The reason is the cadence rather than the code. A look was taken when the rover
had stopped, had moved 0.4 m, and 15 s had passed — and 0.4 m is `MOVED_ENOUGH_M`,
which is the *minimum* baseline the geometry can use. Driven in hops at a third of
a metre a second, the fifteen seconds were what decided, and by the time they were
up the rover was five metres on and in another room. The parked run has the same
problem from the other end: 115 of its pairs shared something and almost all of
them were taken from the same spot. **The cadence was rebuilt on 2026-09-03 and
what it cost to measure is the next section.**

**And the gimbal has never been panned. Not once, in any run.** Every observation
the rover has ever stored has `observer_pan_deg` of 0. So a look is the hundred
degrees in front of the chassis, and two stops a metre apart only share anything
if the chassis happened to be pointing the same way. Panning is what would turn a
stop into a survey; `_world_worth_looking` already counts a gimbal turn as a new
direction, so the machinery is waiting for something to do it.

**Two of the seven frames were 95% and 97% black.** The rover drove out of a lit
hallway into an unlit room and inspected before the camera's automatic exposure
caught up, and each of those looks yielded a single junk region. That is the
white-out of the earlier drive from the other end, and the same trap: "1 of 6
regions kept" is also what a working rover says about a bare room. A frame that
dark is refused whole now and says which it was — see `DARK_FRACTION`, and note
that there is deliberately no matching test at the bright end, because nothing in
the stored frames is washed out enough to set one from.

The rest of the run is the design working. Four crossings survived every gate,
and they were refused because four rays from two viewpoints make a two-by-two
grid of crossings, two of which conflict — the phantom, and from two viewpoints
it is genuinely unknowable. A third look from a third place settles it, and the
rover never took one.

### One entity ate fifty-three degrees of hallway

Left parked in that hallway afterwards, the rover placed one thing a metre and a
half in front of it and then attached thirteen more bearings to it, spanning
**fifty-three degrees**: a ceiling corner, a dark doorway, a small framed picture
and a wall panel, all one entity. The pictures are what say so; the geometry says
why.

The slack a bearing was allowed came from *whichever crop was asking to join*,
capped at `MAX_EXTENT_M`, and the cap saturated. Over one run's 54,607 tolerance
decisions the median total was 1.00 m of which 0.75 was that cap — a cone
eleven degrees wide in the median, against a bearing measured to one and a half.
So a region spanning most of the frame could claim any small thing roughly in its
direction, and observation 2632 of that run did exactly that: 79 degrees of
hallway wall, pointed 25 degrees away from the picture, granted 0.75 m of room
against a miss of 0.67.

**The width is the thing's own now.** `locate.extent_of` measures it when the
crossing is made, from the angular width of the two crops at the range they put
it, taking the *smaller* of the two views — the region finder segments parts as
readily as wholes, so one view of a picture can come back as the picture and the
other as the wall panel around it, and the tighter claim is the better one. It
travels with the placement; the candidate's own span is only a fallback for a
placement written before this existed.

Replayed over the 96-minute run, the entity's own bearings that miss its own
position fall from 26% to **15%**, and the crops belonging to something other than
the thing an entity is mostly of from 26% to 21%. It also finds 25 things where it
found 21, which is the right direction: fewer entities swallowing their
neighbours.

### What is still wrong, and where it is not

**Entities are still mixtures**, which is the fault a person notices first: a
lampshade entity that also holds a chair back, a framed picture and a sheepskin;
one that starts as a sheepskin, becomes a chair and ends as eleven looks at a
picture on the wall. Nineteen of the 23 hold at least two visually distinct
things.

Every rule in the resolver was swept against the recording and **none of them
moves that number.** Pinning the founding appearance vectors so a later look
cannot rewrite what an entity looks like: 19% to 17% on one measure and worse on
the other. Narrowing the match cone: no better at any cap. Raising the appearance
floor from 0.5 to 0.75: 23%, having thrown away half the attachments. The reason
is upstream of all of them and is two things.

**Seventy-one per cent of every attachment came from a viewpoint that entity had
already been seen from.** The rover had 16 distinct positions in 96 minutes and
never once panned the gimbal. From one place a bearing cannot separate two things
along the same line, so a repeat look confirms nothing — and it is still allowed
to add an exemplar and to move the placement. That is exactly how a lampshade
becomes a picture on the wall, one 0.97-scoring step at a time.

**The appearance gate is not a gate.** It took the best of five exemplars, and
the five are a sliding window of whatever last attached. Measured on this
recording: an unrelated crop clears the 0.5 floor against one exemplar 12% of the
time and against five **45%** of the time. And 46% of later attachments would
fail that floor against the look that founded the entity, where only 8% fail
against the rolling window — the gate is measuring against a set the drift itself
wrote. Fixed on 2026-09-03: `resolve.appearance` scores against the middle of the
exemplars rather than the best, which is measured further down along with why no
value of `DIFFERENT_THING` rescues the gate on the vectors the GPU produces.

### The merge veto does not work, and Phase 5 is built on it

The plan makes co-occurrence the safety rule for merging: two entities ever seen
in one frame are different things. Tested against the five near-duplicate pairs in
this recording, **it vetoes all five**, including `object:8` and `object:10`,
which sit 3 cm apart and are two halves of one sofa: 17 frames hold observations
of both. The premise is false wherever the region finder splits one object into
parts, which is the case merge exists to fix. Phase 5 needs a different veto.

## The cadence was the fault, 2026-09-03

A drive of about three minutes came back with **four looks**, which is the fault
the previous section predicted, arriving. Twenty regions were measured across
those four looks and not one thing was placed. Four pairs of bearings did cross
cleanly — 5.3 m of baseline at 34 to 40 degrees of parallax — and all four were
refused by the wall check, because they landed 8 to 9.4 m out where the map put
the first obstacle at 1.3 m and 3.1 m. That part was the design working. The four
looks were not.

Three separate things held the rate down, and the first is most of it.

**The loop refused to look while the wheels were turning, at all.** The argument
was sound: a look taken mid-drive carries the pose the rover had reached rather
than the pose the shutter opened at, and a bearing is only as good as the pose
behind it. The remedy was far too blunt — it meant a rover crossing a building
learned nothing on the way, and a look happened only in whatever gap fell between
one `drive` call and the next. What replaced it is a measurement rather than a
refusal: `Inspector` reads the pose on **both sides** of the capture, uses the
midpoint, and drops the bearing — keeping the picture, the regions and the
vectors — on the looks where the rover covered more ground than that midpoint can
account for. See `MOVED_WHILE_LOOKING_M` and `TURNED_WHILE_LOOKING_DEG` — and
note that they no longer say the same thing, because the drive of that afternoon
found that treating them alike threw away three quarters of a run. Travel widens
the answer and turning still refuses the look: *One entity out of a thirteen-
minute drive* below has the measurement.

**`LOOK_EVERY_S` was 15 s, and what set it was the resolver.** Identity was
settled inside every inspection, and one pass compares every pair in the pending
pool, so a rover that recorded faster than it could place things got slower and
slower at placing them. Two changes take that away:

  - **The resolver got 145 times cheaper**, and the fix was one line of ordering.
    `_place_one` asked what two crops looked like *before* asking whether their
    bearings crossed — a dot product over 384 floats in front of a dozen
    multiplications, in a module whose own first paragraph says the gates run
    cheapest first. On the recording, 97 of the 123 pairs that reach there have no
    usable crossing, so almost all of that arithmetic was spent on pairs geometry
    was about to throw out. Profiled at a 500-observation pool, it was **96% of
    the resolver's entire running time.** With the geometry first, and with each
    vector unpacked once per pass instead of once per comparison, one pass over
    500 bearings went from **37 s to 1.45 s on the rover**.
  - **Looking and settling are now separately paced.** A look is a flat 0.45 s;
    a pass is 1.4 s at 500 pending bearings and 8 s at 2000, because it grows as
    the square of what is waiting. So `inspect(settle=False)` records and decides
    nothing, and the building loop settles on `SETTLE_EVERY_S`. Settling takes
    priority when it is due, because a driving rover has a look due every second
    and the other order starves it completely.

**And the pending pool was jamming rather than filling.** `unplaced` took the
*oldest* `limit` rows, so once that many bearings had accumulated with nowhere to
go — and this recording left 60 of 71 with nowhere to go — the resolver was handed
the same unplaceable rows for ever and nothing recorded afterwards was ever looked
at again. Every look past that point was wasted, silently, and the faster the
rover looks the sooner it happens. The window is the newest `limit` now. What that
costs is a bearing older than the window pairing with one taken now, which is a
rover that saw something, drove away and came back much later; what it buys is
that a look is never wasted.

### What the rate is bounded by now, measured on this host

A bounded capture is **0.29 s** on the Orin and a look through the encoders
**0.16 s**, so a look a second is a 45% duty cycle. (Timed again through the
daemon's own socket on the afternoon of 2026-09-03 the grab is **0.36 s**, which
is the number the shutter-motion limit is now derived from; the difference is the
call rather than the camera, and it is the honest one for that purpose because the
pose readings bracket the call.) The question that decides
whether that is affordable is not the GPU but the lidar, because the scan matcher
is the only odometer this rover has and `uvc_camera.snapshot` records a camera
held open costing it 22% of its revolutions. Re-measured here against the nav
bridge's own counters, twenty seconds per condition, rover parked:

| | revolutions/s | dropped scans |
|---|---:|---:|
| no captures | 9.90 | 0 |
| a capture every 2.0 s | 9.90 | 0 |
| a capture every 1.0 s | 9.95 | 0 |
| no captures again | 9.95 | 0 |

**A capture a second costs the scan matcher nothing on the Orin.** That is a
statement about six cores and not about the design: the 22% was the Pi's four,
and the same experiment is owed to any host this moves to next.

So the rate is set by `LOOK_EVERY_S = 1.0` while driving and by `MOVED_ENOUGH_M`
as the rover slows. That constant went from 0.4 m to 0.15 m, deliberately *below*
the minimum baseline: two looks that close cannot be triangulated against each
other, but they can be against the look three back, and in the meantime each is a
picture of the room from a place the rover has not photographed. At 0.35 m/s that
is a look every 0.4 s, so `LOOK_EVERY_S` governs while driving — a picture a
second, which is 40 to 80 times what the run of that morning managed.

### Two consequences worth knowing about

**The resolver's horizon is about 85 seconds of driving.** `resolve` reads at most
500 pending bearings and the rover now records six or so a second, so two bearings
more than that apart can never be paired. Inside the window there is plenty of
baseline — 29 m of travel at 0.35 m/s — and anything already *placed* is matched
against for ever, because `_against_known` reads every placed entity with no limit
at all. What ages out is a bearing that never crossed anything, which is the right
thing to lose. Raising the cap is a straight trade against the settle: 500 is 1.4 s
and 1000 is 3.2 s, on a board that is also running SLAM.

**A pose the navigator will not vouch for no longer stops the looking.** It used
to, and that meant a rover whose scan matcher had lost confidence fell back to one
look every five minutes — while it was driving through the part of the building
that had just confused it. It looks every `LOOK_BLIND_S` now and stores frames
with no bearing, which the store already handles honestly: the picture is kept and
only the direction is missing.

### The bearing is worked out with the model aiming was measured off, and it is the biggest error in the component

**Not fixed. Measured, and it is worse than everything else here put together.**

[`view.py`](view.py) turns a box into an angle with one multiplication:

```python
offset_deg = (0.5 - centre) * fov_deg
```

That is the separable pixels-times-a-gain model, and this repository already
found out what it is worth. `face_tracking/aiming.py` did the same thing until
2026-08-19, and a probe on the rover measured it 2 degrees out at 20 degrees off
centre, 5 to 9 at 35 to 45, and 13 to 20 at 50 to 65 — on a 130-degree fisheye
hung on a gimbal that pans about the vertical and then tilts about its own
horizontal, it is only right along the two centre lines. It was replaced by a
swept and fitted lens in [`face_tracking/lens.py`](../face_tracking/lens.py) and
verified live. **The world state was never moved with it**, and its bearings are
the foundation everything in `locate.py` and `resolve.py` stands on.

What the difference comes to on the 71 boxes of the recording of 2026-09-03,
against `lens.ray_at` at the frames' own 640x480 with the lens as fitted —
[`bench_bearing.py`](bench_bearing.py) is the measurement, and it runs at a desk
against any `world.db` the rover wrote:

| | median | 90th | worst | over 1.5 deg |
|---|---:|---:|---:|---:|
| the fisheye's own non-separability | 1.03 deg | 7.09 | 7.99 | 27 of 71 |
| and the 10 degrees the gimbal was tilted | 1.34 deg | 12.74 | **14.51** | **35 of 71** |

**Half the bearings the rover has ever recorded are outside the accuracy the
geometry is told to expect, and the worst is ten times it.** `BEARING_SIGMA_DEG`
is 1.5, and every uncertainty `locate.fix` reports and every support `agrees`
accepts or rejects is computed from it. The error is not uniform either, which is
what makes it poisonous rather than merely large: it is under a degree across the
middle band of the frame and blows up at the top and bottom, so two looks at the
same object from the same place score differently depending on where in the frame
it happened to sit.

Two separate faults, and the second is the cheaper one to be sure of:

  - **The lens is not separable.** Angle off the lens axis goes with *radius*, so
    a box high in the frame and off to one side has an azimuth that the horizontal
    coordinate alone cannot give. `lens.ray_at` already does this.
  - **`observer_tilt_deg` is recorded on every observation and never used.**
    `view.ray` is `heading - pan + offset`; the tilt is written down and dropped.
    A camera tilted 10 degrees up and looking at something off to the side is not
    pointing where a level camera would, and the gimbal has been at 10 degrees for
    every look in this recording.

The fix has an obvious shape and it is not a copy of the lens. The daemon already
hands `Inspector` its camera, its pose and its reach as callables, "supplied
rather than reached for, because the things they read belong to the daemon" — and
the lens belongs to the daemon too, for exactly the same reason. So a `bearing`
callable alongside them, with `store.record` and `view.ray` taking the tilt they
are already given. One copy of the calibration, in the file whose whole purpose is
to be the one place it is re-measured.

**What has to happen with it, and why it was not done in the same pass as the
cadence.** Changing this changes every bearing the rover records from then on, so
the database becomes half one model and half the other; `BEARING_SIGMA_DEG` was
measured under the old model and has to be re-measured under the new one; and
`replay.py` cannot judge it as it stands, because `ray_of` reads the bearing off
the row rather than recomputing it from the box — so replaying a change to the
bearing model needs the replay harness extended first. None of that is hard and
all of it is more than an afternoon, and the honest order is to do it as its own
change with a drive behind it rather than to bolt it onto a rate fix.

It is also the best current explanation for the open problem this component has
been carrying: **an entity is often a mixture of several objects.** Rays that are
several degrees wrong in a direction that depends on where in the frame the thing
sat will cross where nothing is, and will agree with things they are not pointing
at. Nothing in the resolver's own rules has to be wrong for that to happen.

**What is not yet measured is whether it places more things.** Everything above is
a rate and a cost. The recording of 2026-09-03 replays to exactly the entity the
rover produced from it, which is what makes the change safe rather than useful,
and only another drive says whether a picture a second is what triangulation was
missing. The gimbal has still never been panned in any run.

### One entity, twenty-six crops, six objects

The run of 2026-09-03 finished with **416 observations and one entity**, and that
entity held twenty-six crops of a dark cabinet, two different framed pictures, a
doorway, a table with a bottle on it and the head of the person standing beside
the rover. Fifty-eight per cent of its own bearings missed its own stated
position. Three causes, and they compound; all three are fixed, and `replay.py`
turns that recording into **three entities of twenty-two crops, no known-wrong
attachments, and 5% stray bearings**, which by eye are the cabinet and the two
pictures.

**More than half of it was wrong without anybody having to identify anything.**
Fourteen of the twenty-six crops were a second-or-later region of a frame that
had already given one, and two regions of one frame are two different things by
the region finder's own overlap suppression -- their boxes here barely touch, the
worst pair overlapping by 0.05. The resolver has always had that rule. It kept it
in a dictionary rebuilt at the top of every pass, and the pending pool outlives
the pass by design, so a frame donated one more region every time round: traced
on this recording, frame 429 joined on three consecutive passes and frame 446 on
four. `WorldStore.entities_in_frame` asks the store instead, so the rule now
holds across passes and across restarts. **A look a second makes this much
worse rather than better**, which is why it surfaced now: the donation rate is
one region per pass, and passes used to be fifteen seconds apart.

**A badly placed thing was given a wide net, and the net is what made it worse.**
`object:1` was founded on a crossing at 13.9 degrees of parallax, which bought it
0.464 m of uncertainty, and that went whole into how far a later bearing could be
off and still count: 0.82 m at two metres, a cone 46 degrees wide, of which the
bearing -- the only term about the new ray at all -- was 6%. But a shallow
crossing is not uncertain in every direction. It is uncertain a long way *down*
its own line of sight and precise across it, and charging a lengthways error to a
sideways question is the mistake. `locate.fix` now records the shape of the error
as well as its size and `locate.cross_track` reads it: on the test case a fix
2.1 m uncertain along the sight line is 0.19 m uncertain across it, an eleven-fold
difference. **This is not a narrowed cone and the distinction matters** -- capping
the match angle was swept on the previous recording and helped at no cap, and with
this in place the result is the same whether the angle is capped at 10 degrees or
not capped at all.

**And the appearance gate got looser every time it was wrong.** A crop that joins
an entity becomes one of its exemplars, and the score was the best of them, so an
entity that had swallowed something unrelated accepted the next unrelated thing
more readily for it. Measured on this recording: an entity holding one exemplar
admitted 10% of the pending pool at `DIFFERENT_THING`, holding twenty-one it
admitted 64%, monotonically the whole way. `resolve.appearance` takes the middle
of the exemplars instead of the best. Pinning the founding vectors instead, which
was the previous attempt, is measured again here and still buys nothing.

**What is not fixed, and cannot be fixed by a threshold.** DINOv2 on this rover's
own frames does not separate identity from coincidence. Over the 157 crops of this
run: two regions of one frame -- different objects by construction -- score a
median cosine of 0.354 and a 95th percentile of **0.740**, while the one genuine
cross-viewpoint match in the whole run, the pair of looks that founded
`object:1`, scored **0.674**. The real match sits below the noise. Raising
`DIFFERENT_THING` to 0.70 makes the run place nothing at all, and the sweep
between is not even monotone. (**The 0.740 is a 157-crop sample and it was
pessimistic.** Measured again over both drives of that day, 3,741 same-frame
pairs, the 95th percentile is 0.715 in the morning and 0.677 in the afternoon and
the band is stable between them — which is what later made 0.55 a better place for
the floor than 0.5. It is still not a separation: see *One entity out of a
thirteen-minute drive*.) The numbers in `resolve.py` that put 0.5 between a
chair-versus-spray-bottle at 0.122 and a chair across a viewpoint change at 0.696
were measured on the CPU int8 graphs; the rover has been running the TensorRT
engines, whose vectors are not the same and were never re-measured against them.
So appearance is a tiebreak here and the geometry has to carry identity, which is
what the three fixes above assume. Mean-centring the vectors was tried and does
not rescue DINOv2 (10.9% of unrelated pairs beat the founding pair, 10.7% after
centring); it does help SigLIP2 markedly, from 6.1% to 1.9%, which is worth a
proper measurement on a run with more than one true cross-viewpoint pair in it.

## Three things placed where nothing is, 2026-09-03

The run of that morning is 466 regions over 99 looks, and it placed four things.
One of them is right and three are not, and the three fail in two different ways
that had been sitting behind each other.

Replaying the recording with the map the rover was using reproduces its four
entities exactly, which is what makes any of the below measurable.

### What the four actually are

Read off the stored crops rather than off a score:

| | what it holds |
|---|---|
| `object:3` | three crops of one framed picture. Right. |
| `object:1` | a dark cabinet, a lit doorway on a yellow wall, and a blue sofa |
| `object:2` | a doorway seen from across the room, and a door frame seen up close |
| `object:4` | six crops of one framed picture, and the pole of a floor lamp |

**The replay's own mixing score says 12% and the pictures say three entities out
of four**, so the score is worth stating carefully: it clusters an entity's crops
by DINOv2 at 0.55 and counts what is not in the biggest cluster, and on the
TensorRT vectors this rover runs, a dark cabinet and a doorway score 0.57. Single
linkage then chains them into one cluster and reports a clean entity. The number
is still useful for comparing two builds on one recording; it is not evidence that
an entity is one thing, and the crops in the console are.

### Nowhere near enough of the run could be placed at all

**314 of the 466 regions carry no rover pose**, because `slam_toolbox` was not
publishing a position it trusted, and an observation with no pose gets no bearing
by design — the picture is kept and the geometry is not invented. That is the
right behaviour and it is not free: two thirds of a run recorded at a look a
second is evidence that can never be used.

Of the 152 that do have a pose there are **six distinct standing places in the
whole run**, and 127 of the 152 share one of them. Rays from one place share an
origin exactly and cross nowhere, so the run's entire triangulating power is five
viewpoints, all within about 3 m of each other.

### Every fix landed a metre or two from the camera, and the geometry says it had to

All four placements sit in open floor 0.6 to 2.8 m from the cameras that saw them.
Every crop behind them is of something on a wall four to six metres away.

The reason is the parallax floor, working exactly backwards from the way it reads.
`MIN_PARALLAX_DEG` is 12 degrees and it is there to refuse shallow crossings that
run away down their own line of sight. But parallax is not free: two looks a
baseline `b` apart at a thing `r` away subtend about `2·atan(b / 2r)`, so
**demanding 12 degrees is the same as capping the range of any accepted fix at
about 4.8 times the baseline.** On this run:

| | baseline | recorded parallax | the furthest 12° allows | where it was placed |
|---|---:|---:|---:|---:|
| `object:3` | 0.70 m | 25.5° | 3.3 m | 0.75–1.30 m |
| `object:2` | 2.34 m | 57.5° | 11.1 m | 1.55–2.78 m |
| `object:4` | 2.60 m | 65.4° | 12.4 m | 1.14–1.91 m |
| `object:1` | 3.12 m | 29.9° | 14.8 m | 0.58–1.77 m |

`object:3`'s two looks are 0.70 m apart, so nothing beyond about 3.3 m was ever
placeable from them — and the picture is on the far wall. The 25.5 degrees it
recorded is not geometry, because the geometry cannot produce it: a thing 4.5 m
away, seen from those two poses, subtends **5.7 degrees**, which the resolver
would have refused. The extra 20 came from the bearings being wrong, and once
they are wrong in that direction the crossing lands wherever they put it and every
remaining guard is satisfied. Simulated on those two poses, with a thing genuinely
4.5 m out and each bearing 3 degrees wrong the closing way, the crossing lands at
3.05 m; 6 degrees wrong puts it at 2.34 m.

**So the arithmetic in `locate.py` is not what is broken. Its inputs are four to
ten times worse than it is told they are, and its baselines are a metre when they
need to be several.** What it does with that is not refuse: it reports
`object:3` to within 0.138 m, because the uncertainty is worked out by nudging
each bearing by `BEARING_SIGMA_DEG`, and that constant still says 1.5 degrees.

Two independent measurements of how wrong a bearing is, on this recording:

- `bench_bearing.py` over its 466 boxes, against the swept lens in
  `face_tracking/lens.py`: **median 1.06 degrees, 90th percentile 7.10, worst
  17.62, and 180 of 466 outside the 1.5 the resolver is promised.** The worst is
  a box at the top right corner of the frame, where the one multiplication says
  −60.8 degrees and the fitted lens says −78.4.
- Six looks at one framed picture from one standstill, with the gimbal at one
  commanded angle, spread their computed bearings over **10.4 degrees**
  (`obs4194` to `obs4261`, from the pose at 6.36, −3.76). Nothing moved and the
  answer moved ten degrees.

Rewriting every bearing in the recording with the fitted lens and replaying it
still gives four entities with the same mixing, so the lens is real and is not
the whole of it. The other terms are the gimbal arriving where it was not told —
this run commanded a pan of 150 degrees, far outside the ±30 the 1.5 was measured
over — and SLAM's heading, which the same standing place reports as 158.1 and then
169.1 degrees over the run.

### Two rays at two different things, and nothing that can tell

`object:1` and `object:2` are the phantom this component was built to refuse, and
they got through the whole gauntlet. `object:1` is a ray at a dark cabinet from
one pose and a ray at a blue sofa from another, 3.12 m apart, crossing at 29.9
degrees in the middle of the floor. Baseline, parallax, range, both wall checks
and the rival test all pass, because each of the two bearings is individually fine
and the two things really are in those directions.

The only gate that could refuse it is appearance, and it cannot: the cabinet and
the sofa score **0.57** against a `DIFFERENT_THING` floor of 0.5, and `object:2`'s
doorway and door frame score 0.64. Those sit inside the band already measured on
this rover's TensorRT vectors, where two regions of one frame — different things by
construction — reach a 95th percentile of 0.740 while a genuine cross-viewpoint
match scored 0.674. **There is no threshold that separates them**, which is the
same conclusion the section above this one reached, arrived at from the other end.

The floor is 0.55 now rather than 0.5, which does not change that: it refuses the
0.542 and 0.538 pairs the afternoon drive would otherwise have built things out of
and it would not have refused either of these two. What it does is stop the
*plainly* unrelated founding a thing, and the pair that founds this run's own
framed picture clears it by seven thousandths.

### One gate was missing, and it is fixed

Every way into an entity applies `DIFFERENT_THING` except one, and that one was
`_place_one`'s last loop: a thing has just been placed from two looks, and
everything else in the same group whose bearing lands inside its silhouette joins
it — on geometry alone, with nothing asked about what it looks like. The tolerance
that loop uses is deliberately the wide one, because it asks whether a bearing
lands within a thing's own width; a wide gate on where it is wants the same gate
on what it looks like as everywhere else.

It is what put the lit doorway into `object:1`, at 0.28 and 0.26 against that
entity's two exemplars, and the pole of the floor lamp into `object:4` at 0.09 —
both far below what the founding pair itself had to clear. The loop now asks
`appearance` against the entity's exemplars, which is the same call
`_against_known` makes.

Replaying the recording: the lamp pole and the doorway both go, `object:4` becomes
seven crops of framed pictures with nothing else in it, and the replay's mixing
score falls from 5 crops of 18 to 0 of 15. **It does not fix `object:1` or
`object:2`**, whose fault is the crossing itself and not the joining, and it does
not move where anything was placed.

### What is left, in the order it matters

1. **The bearing.** Everything above is downstream of it. The fitted lens is the
   part that can be fixed at a desk; the gimbal and the heading want the rover.
   See *The bearing is worked out with the model aiming was measured off* above.
2. **`BEARING_SIGMA_DEG` is a promise the rover does not keep**, and it is what
   the uncertainty, the tolerance and the parallax floor are all sized from. Until
   the bearing is fixed it is understating by a factor of four in the median and
   ten at the edges of the frame, which is what makes a phantom report two
   centimetres of doubt.
3. **A short baseline should refuse a distant thing rather than manufacture a
   near one.** The parallax floor caps range at 4.8 baselines whether anybody
   meant it to or not, so a fix taken near that cap is a fix the geometry cannot
   support and is worth refusing outright.
4. **Appearance cannot separate what the rover sees**, on the vectors it runs.
   Nothing above changes that, and no threshold rescues it.

## One entity out of a thirteen-minute drive, 2026-09-03 (afternoon)

A second drive the same day, after the appearance gate above was closed: 866
regions over 214 looks, and **one entity**. Nothing false was placed, which is
what the previous fix was for; almost nothing was placed at all.

Replaying the recording reproduces that one entity exactly, so everything below
is measured rather than argued.

### Three quarters of the run recorded no bearing, and it was a constant

**163 of the 214 looks stored no direction for anything they saw.** Not because
SLAM had lost the rover -- the pose was there -- but because `Inspector._where`
refused to draw a bearing from it. It reads the pose either side of the shutter
and drops the bearing when the rover covered too much ground between the two, and
the limit was 0.12 m.

That number was derived from a 0.29 s capture at the 0.35 m/s the rover explores
at, which is 0.10 m of travel with a little room over. **Neither half of the
derivation still held.** A bounded grab timed through the daemon is 0.36 s, and
the travel recorded on the looks that lost their bearing puts the explore speed at
0.47 m/s. So an ordinary look taken while driving in a straight line covers 0.17 m
-- median, over 63 such looks -- and was refused by a hair.

| | looks | regions |
|---|---:|---:|
| the run | 214 | 866 |
| lost the bearing | 163 | 772 |
| ...of those, turned less than 3 deg | 63 | 342 |
| had a bearing to work with | 51 | 94 |

What was left is **94 bearings from eight standing places**, 60 of them from one
of those places. Rays from one place share an origin exactly and cross nowhere, so
the run's whole triangulating power was seven usable viewpoints in thirteen
minutes of driving.

### Travelling and turning are not the same mistake

The fix is not to loosen the limit and hope. The two kinds of movement do
different things to a ray and only one of them can be paid for:

- **Travelling shifts where the ray starts.** The midpoint of two pose readings
  leaves the origin out by half of what was covered -- 0.085 m on an ordinary
  look -- and the crossing moves by about as much, in a direction nothing can
  predict. That is a number, and a number can be carried: it is written on the
  observation as `origin_sigma_m` and `locate.fix` adds both rays' worth to the
  answer's uncertainty, widening it in both axes.
- **Turning swings where the ray points.** There is nothing to charge that to but
  a crossing somewhere there is nothing at all, which is how a phantom is made.

So `MOVED_WHILE_LOOKING_M` is 0.30 -- twice an ordinary look's travel, refusing
the genuine outliers (the worst on this run was 0.65 m) -- and
`TURNED_WHILE_LOOKING_DEG` stays at 3.0, which is what cost the other 100 looks
their bearings. Those are the ones worth losing.

The pose is also read the moment the picture is in hand rather than after it has
been written to disk, so the bracket measures the shutter and nothing else. That
is correctness rather than a saving: saving a frame is 1.4 ms, measured on the
Orin.

**This cannot be replayed.** The recording has no pose at all on the 163 looks
that lost one, because dropping the bearing is what happened at the time -- so the
size of the recovery is a prediction from the counts above and not a measurement.
It wants the next drive.

### A standoff in one corner stopped the other corner

The second fault is structural and it is measurable here. `_place_one` ranks the
crossings it can make and refuses the best one when another crossing built from
one of the same rays disagrees -- correctly: a ray points at one thing, and
nothing in a pool of bearings can say which of two answers on it is right.

But it answered `None` for that, and `None` is also how it says "there is nothing
left to place here", which is what `_pair_up` stops on. **So one standoff threw
away every other crossing in the pool.** Measured on this run: 65 of the 181
pairing passes ended that way, with a median of four crossings still on the table.

The rival test is now asked of each candidate in turn (`_contested`), and the
first crossing nothing contradicts is placed. A candidate sharing a ray with a
standoff is refused along with it, because that ray is spoken for either way.

**What it recovers is real.** Deployed to the rover, standing still, on the pool
it already had, it placed an office chair -- seen from 15.34,1.15 and from
13.13,-0.42, a 2.71 m baseline at 79 degrees of parallax, to within 0.072 m --
which the old code had refused because a third ray crossed one of its two
somewhere 0.64 m away. That is a genuine thing the rover saw twice and could not
keep.

**And it exposed what the early exit had been hiding.** It placed five things, not
two: the chair, the framed picture it already had, and three that are plainly two
objects each. The early exit was never refusing those -- it simply stopped before
reaching them.

### The appearance floor was letting a fifth of the noise through

Read off the crops of the five, the three wrong ones are a door with a blown-out
ceiling panel, a framed picture with a bright doorway, and a landscape with a
doorway. None of them is a bad crop: only one is blown out at all. What joined
each pair is DINOv2 scoring it above `DIFFERENT_THING`, which was 0.5.

Measured the same way on both drives of that day -- every pair of regions taken
from **one frame**, which are different objects by construction, so 3,741 labelled
negatives rather than the 157-crop sample the constant was last argued from:

| | pairs | median | p90 | p95 | over 0.50 | over 0.55 |
|---|---:|---:|---:|---:|---:|---:|
| morning | 1,138 | 0.349 | 0.623 | 0.715 | 20.2% | 15.5% |
| afternoon | 2,603 | 0.336 | 0.594 | 0.677 | 19.1% | 13.4% |

**The band is stable between runs and tighter than 0.740**, which is what makes
0.5 identifiably the wrong place for the floor rather than merely a guess. The
three wrong pairs score 0.542, 0.525 and 0.538; the chair scores 0.648.

So the floor is **0.55**. On the afternoon recording that leaves exactly the two
entities a person can point at in the crops and none of the three that are two
things, and the morning recording replays unchanged at four entities with nothing
mixed. Against what the rover actually did -- one entity -- that is one more valid
thing and no false ones.

**It is not a separation and the margin is thin.** 13-15% of known-different pairs
still clear 0.55, and the pair of looks that founds the afternoon run's framed
picture scores **0.557** -- seven thousandths above the line. A floor of 0.56 would
lose it. Appearance still cannot tell this rover what it is looking at; what has
changed is only that the plainly unrelated can no longer found a thing.

### Recovering the bearings walks into a cost cliff, so it is bounded

Both fixes above make the pending pool much bigger, and the resolver's cost is
worse than linear in it: a pass compares every pair of bearings and then asks
every ray whether it agrees with each crossing that survived. Measured on the Orin
over a synthetic pool of 500 rays dense enough to place things from -- which is
roughly what a driven run will now produce, where the recorded run had 94:

| pool | one pass, before | one pass, with the standoff fix |
|---:|---:|---:|
| 200 | 3.6 s | 4.9 s |
| 350 | 16.4 s | 21.0 s |
| 500 | 47.0 s | 54.7 s |

**So the standoff fix costs 17% and the pool size costs everything.** 47 seconds
was already the price at a full pool; the old build never paid it because three
quarters of every look recorded no bearing, and stopping at the first standoff cut
the work short besides.

Two things bound it. `MAX_NEW_PER_PASS` stops a pairing pass after two new
placements and leaves the rest of the pool for the next one -- the cost is linear
in that (2.7 s per placement at a 500-ray pool, so 5.3 s for two against a
`SETTLE_EVERY_S` of 10), and two every ten seconds is twelve a minute against the
four things the rover's best drive has placed in thirteen minutes. **Only new
things are rationed**; joining an observation to something already placed is the
cheap path and still runs over the whole pool every pass.

And the loop's settle clock is now read *after* the pass rather than before it. A
pass longer than `SETTLE_EVERY_S` was otherwise due again the instant it returned,
so a rover with a large enough pool would have settled for ever and never looked
again -- a trap the small pool was hiding.

### What a sparse pool does to everything else

Worth stating plainly, because it explains why nothing here is settled by tuning:
with 94 bearings and seven viewpoints, **which entities appear is decided by which
two rays happen to be in the pool.** Two things were tried and left alone for it.
Tightening the blown-out-crop filter from 0.6 to 0.5 -- every one of the run's 24
most blown-out crops is a window or a light panel, so the cut is defensible on its
own -- removes one false entity and removes the office chair with it, by changing
which crossing wins a ranking three passes later. And requiring a third viewpoint
before a thing may be invented, which is the design's own answer to a standoff,
loses the chair outright: the rover only ever saw it from two places, and one of
the phantoms had three rays agreeing with it because agreement is cheap when the
thing is close and its tolerance covers its own width.

The design's own answer to a standoff is "a third look from somewhere else settles
it". The whole of this run had seven places to look from.

### What is left, in the order it matters

1. **The next drive is the measurement.** Everything in the first section is a
   prediction until the rover drives again: the counts say the pool should go from
   94 bearings to something over 400, from seven viewpoints to more than sixty.
2. **The bearing itself**, which the section above this one measures and which
   nothing here has touched.
3. **The gimbal never moved.** Pan was 0.0 on all 866 regions of this run, because
   nothing in the world-building loop turns it and the camera therefore stares
   down the rover's nose for a whole drive. Whatever is beside the route is seen
   once, in passing, at the edge of the frame. A sweep while driving is the
   obvious next multiplier and it is a behaviour change, not a constant.
4. **Appearance still cannot separate what the rover sees.** 0.55 is a better
   floor than 0.5 and it is not a threshold that separates: 13% of pairs known to
   be different things clear it, and this run's own framed picture clears it by
   seven thousandths. SigLIP2's vectors are stored on every observation and have
   never been tried for this -- measured on the same pairs they order them the
   same way and no better (the real pairs at 0.851 and 0.906 against a same-frame
   95th percentile of 0.854), so there is nothing free there either.

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
rover's own stored regions -- twenty-four for things it had seen and sixteen for
things that are not in the building -- separate almost perfectly by raw score and
not at all by separation. The best cut any threshold on the separation could make still gets
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

**Everything from here to the end of this file is about the local language model,
which was removed from the rover on 2026-09-02.** None of it describes what runs
now; it is the evidence for why nothing like it does. Anyone proposing a local VLM
for this rover again should start here rather than repeat it.

### It runs, and what it sees is right

Cosmos Reason 2 2B runs locally on the Orin's CPU and describes the room
accurately. Across six inspections it named the sofa, the glass table, the coiled
cable on the floor, the spray bottle, the framed pictures and the doorway —
**every name was a real thing in the frame, with no hallucinations** — and once
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
  the prompt asks. The prompt was changed to ask for the grid, and the validator
  read both, because a picture is one unit across and a box in the hundreds is
  therefore not a fraction. (Both went with the model; FastSAM answers in
  fractions and always did.)
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

That amendment was made, and then overtaken: there is no perception call to a
language model any more, no prompt, and no answer to validate. The encoders
replaced the whole path on 2026-09-02.

### A trap left behind for the next model

Cosmos 3 writes the *string* `"null"` where Reason 2 leaves the field out, which
the validator refused as an invented identifier — quietly throwing away every
observation in three perfectly good answers. That particular trap went with the
field it was set in, and then with the validator itself; the class of thing it
belongs to did not. Any model ever put behind a call like this is worth reading
raw output from before its answers are believed, because what makes this kind of
bug expensive is that it looks like the model saying nothing.
