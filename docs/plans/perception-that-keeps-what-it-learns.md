# Perception that keeps what it sees, and what it learns about it

Status: proposed 2026-10-12 at the owner's request. Nothing in it is built. It
answers the owner's question: how could the rover lose less of what its sensors
give between the room and a language model, and keep more of the understanding
itself, rather than have a language model add it afterwards and forget it? The
first steps are offline measurements against the deployed pipeline, on a
benchmark Codex has been asked to assemble in `experiments/perception_bench/`.

## The short answer

**Most of what the camera sees is thrown away before anything tries to
understand it, and what is understood is never written down.** The camera
records 6% of the pixels it can deliver. A detector then keeps the regions it
scores as objects, at most twelve a picture, and on the 6,310 pictures the rover
holds today those regions cover a median of a quarter of each picture. Each
region becomes one vector and one point on the map. Meaning, what a thing is,
what it is for, whether it moves, is stored nowhere, by rule. So every judgement
about meaning is made again, at the moment of asking, by a language model, the
voice model in conversation or the coding agent over ssh, and is lost when that
model's context ends.

The change proposed is architectural, and comes in five parts, smallest first:

1. **Keep the evidence the sensors give**: full resolution, undistorted, with
   depth for every look.
2. **Index all of every picture, not twelve regions of it**: dense features,
   fused into three dimensions by depth and pose. An object becomes a way of
   reading that map, chosen when a question needs it, rather than a decision
   made once at the shutter.
3. **Store meaning as claims with their evidence**, written by a describing
   model during the hours the rover spends on its charger. A claim can say what
   a thing is, what it is for, whether it moves and what state it is in, and it
   never decides identity.
4. **One memory with two readers that can also write**: the voice model and the
   coding agent read the evidence, not summaries of it, and what they understand
   goes back into the rover as claims, marked with who said it.
5. **Time as a model, not a log**: how long each thing tends to stay put,
   learned from the rover's own history and from what kind of thing it is.

Each part is measured offline against what is deployed before it touches the
rover, under this repository's rule of offline first.

## Where information is lost today

Measured on the rover's store on 2026-10-12 unless a link says otherwise.

| Step | What is kept | What is lost | What that costs |
|---|---|---|---|
| The picture | the gimbal camera at 640x480 through a 130-degree fisheye, about 5 pixels a degree; the OAK-D-Lite at 640x360 colour and 320x180 depth | the camera does 2592x1944: 94% of its pixels are never recorded. The store holds 3,181 depth maps beside 6,303 pictures | at 3 m a 10 cm thing is about ten pixels across; a small object on the floor "often never becomes a thing of its own" ([world_state](../../world_state/README.md#what-to-expect-from-it)); labels and text cannot be read |
| The region | what YOLOE, without a vocabulary, scores as an object above 0.15: at most 12 a picture, each between 0.4% and a third of the frame, and none too dark, blown out or flat | walls, floors, doors, shelves and their contents, surfaces, anything under about 35 pixels or over a third of the frame | the kept regions' boxes cover a median of 25% of each picture (quartiles 15% and 39%); 19% of each picture ends up in a box filed to a thing; 10% of pictures keep no region at all, and 7% hit the limit of twelve |
| The vector | one DINOv2-small and one SigLIP2 vector per region, from a 224- or 256-pixel crop of the fisheye picture as distorted | parts, state (open, shut, on, off), text, relations between things, how many there are, and the shape of the thing | a held-out look of a real object matches that object's other looks at 0.70 or better only 43% of the time ([world_state](../../world_state/README.md#checking-a-hypothesis-with-one-look)); asked what kind of thing a poor view shows, the rover's own text vectors slip as its appearance does ([2026-10-05](../progress/2026-10-05-release-category.md)) |
| The place | one point per region, from crossed bearings and the depth under its outline | the thing's extent and shape, what stands behind it, and any measurement that a place is empty | one object becomes several records: 3 of 19 labelled objects are two things each and 46 of 412 looks are filed under the wrong thing ([the plan](semantic-world-state.md#one-thing-per-object-and-only-that-objects-looks)); a thing behind other things is placed 0.88 m out |
| The meaning | nothing: no name or kind is stored ([R-WS-7](../requirements/world-state.md#r-ws-7)) | what a thing is, what it is for, whether it moves | every answer about meaning is invented again when asked, and forgotten |
| Time | each look's time; the lidar's log of furniture-sized changes | how often each thing moves, and what is probably still where it was | the only camera signal that something has gone fired on half the looks at things in view when nothing had moved ([2026-10-10](../progress/2026-10-10-m5-what-reports-a-change.md)) |
| The language models | the voice model gets sentences from `find_thing` (how far, which way, how often seen) or a fresh photograph from `look`; the coding agent reads the database and pictures over ssh | the evidence behind an answer, and any way to write back what was understood ([R-CTL-4](../requirements/control.md#r-ctl-4)) | the coding agent's labels of the rover's pictures live in files under `experiments/` that nothing on the rover reads; what the voice model understood ends with the conversation |

None of these was a mistake when it was made. A local vision-language model was
tried as the describer and removed: an inspection took a minute, its names
drifted on byte-identical frames and it saw objects that were not there
([cosmos-reason2.md](../decisions/cosmos-reason2.md)). Regions and vectors were
the stable, fast replacement, and the rules against names and against the voice
model writing protect a store whose identities are not yet trustworthy. The
cost of that design is that the understanding has nowhere to live, which is
the owner's observation.

## What the field has learned

**Dense features have replaced the region as the unit of memory.** VLMaps puts
per-pixel language features into a top-down grid, the same shape as a lidar map
(Huang et al., 2023). OpenScene fuses pixel features from many views onto 3D
points (Peng et al., 2023). Nothing is lost for not being cut out, and a
half-hidden thing contributes whatever patches each view can see; what a field
gives up is instances and counts, until a question asks for them. The encoders
that make this cheap now exist. NVIDIA's RADIO models distil language-aligned,
dense and boundary-sharp features into one backbone (Heinrich et al., 2025), and
RADSeg's 106M model beat combinations of models eight to thirteen times its size
at open-vocabulary segmentation (Alama et al., 2026). RayFronts stores RADIO
features in voxels within depth range and as semantic rays beyond it, runs in
real time on a Jetson AGX Orin by its authors' account, and is MIT-licensed with
a Jetson container (Alama et al., 2025). It is the nearest published thing to
layer 2 of this proposal.

**How finely to divide the world into objects can be left to the question.**
Clio clusters a scene into objects at the level of detail a list of tasks needs
(Maggio et al., 2024), and its successor models a stove's knob or the whole stove
as asked (Maggio et al., 2026). BBQ associates masks across views by DINOv2
features and geometry, aimed at exactly one object becoming several records
(Linok et al., 2025). MaskClustering joins two masks by how many other views
contain both (Yan et al., 2024). DAAAM hangs localised captions on a scene graph
over space and time, and lets a tool-calling language model query it, the way
the rover's voice model already calls tools (Gorlo et al., 2026).

**Occlusion is handled by many views, or by a guess marked as one.** SAM 2 keeps
a memory of an object across frames and finds it again after it has been hidden
(Ravi et al., 2025). SAM 3 returns every instance of a concept named in words
(Carion et al., 2026). SAM 3D Objects reconstructs a whole object, hidden parts
included, from one picture and a mask (SAM 3D Team, 2026). Only the first is
small enough to try on the rover. What the others infer about a hidden part is
a claim, not evidence.

**Language models answer better from pictures than from records about
pictures.** On OpenEQA's questions about a recorded walk through a home, GPT-4
scored 33.5 with no memory at all, 36.5 from a scene graph written as text,
43.6 from captions of the frames, and 55.3 from the frames themselves, against
86.8 for people (Majumdar et al., 2024). 3D-Mem's memory of pictures beat a
memory of object lists, 52.6 against 47.2 on its main benchmark (Yang et al.,
2025). In GraphEQA a scene graph given as text alone fell to 13.6%, and the same
graph with two relevant pictures scored 63.5% (Saxena et al., 2025). A benchmark
published this month reaches the same order: storing frames beats storing
descriptions of them, costs less to write, and frames together with a scene
graph do best for every model tested (Wang et al., 2026). The rover's voice model
is given the record and never the picture.

**Absence is a measurement, not a missed detection.** Khronos marks an object
gone only when stored camera rays show its place was in view and empty, and
re-checks that history after every loop closure (Schmid et al., 2024). DynaMem
keeps small cubes holding a feature and a last-seen time, and deletes one when
the camera sees straight through it. It found moved objects 70% of the time
against 30% for a memory that does not (Liu et al., 2024). The rover's "seen
empty" is the opposite construction, a region that was not filed, and it fired
on half the looks at things that had not moved.

**Things that move have rates, and the rates can be learned.** Rosen's
persistence filter turns a prior on how long a feature survives, plus each time
it was seen or missed, into an exact probability that it is still there (Rosen
et al., 2016). Perpetua adds the probability that something gone comes back, and
fits each feature's own history (Saavedra-Ruiz et al., 2025). PreSIST has a
vision-language model estimate how long each object will stay, uses that as the
prior, and teaches the estimate to a small DINOv2-based model that answers in
0.04 s instead of 7-8 s (Adkins et al., 2026). A scene graph whose links carry
such filters answered 72% of queries correctly over three weeks of real changes,
against 56% for DualMap and 36% for ConceptGraphs (Saavedra-Ruiz et al., 2026).
FreMEn models the daily and weekly cycles in which places change (Krajník et
al., 2017).

**A robot can learn its own house.** BYE trains a small encoder on one
exploration of a single home, from the robot's own tracked objects, and uses it
to find them again after they move. With CLIP it associated moved objects
correctly 95.6% of the time against CLIP's 88.9% in simulation, and got all 62
association tasks right on a real robot (Huang et al., 2024). Frey et al. fuse
single-frame predictions into a 3D map, render the map back as labels, and
fine-tune the robot's own network on them, replaying old data so that it does
not forget (Frey et al., 2022). PreSIST's small model is the same move: a large
model teaches, a small one keeps the lesson.

**Consolidation pays.** Generative agents retrieve memories by recency,
importance and relevance, and periodically write higher-level conclusions back
into memory (Park et al., 2023). Letta's sleep-time compute works out likely
answers before questions arrive, and needs about five times less computation at
question time for the same accuracy (Lin et al., 2025). ReMEmbR captions what
the robot sees every few seconds with the time and place, on a Jetson Orin, and
lets a language model search by words, place or time (Anwar et al., 2025).
WorldLines keeps what the robot saw apart from what it was told, and labels each
belief fresh, stale, uncertain or contradicted (Zhang et al., 2026). That is the
honesty this repository already asks of the rover's claims, written into the
data.

**Understanding has to be written down to be kept, and robots that keep it
write it as structured state.** ConceptGraphs, the published system closest to
this rover's design (regions embedded, fused into objects), adds the layer the
rover lacks: a vision-language model captions each object, and a language
model reasons over the captions. On a real robot that was enough to sort
obstacles to push through, such as a curtain, from ones to avoid, such as
bricks (Gu et al., 2024). Statler keeps the world state as text that a language
model rewrites after each action, and beats Code as Policies, which keeps no
such state (Yoneda et al., 2024). GraphPad lets a vision model edit a scene
graph through API calls, and reached 55.3% on OpenEQA's questions while looking
at five times fewer frames (Ali et al., 2025). That is the shape of claims
written back into memory.

**Physical common sense is weak in every vision-language model tested, and a
robot can learn its own physics from its own experience.** PhysBench's 10,002
items found all 75 models it tested poor at physics (Chow et al., 2025). Meta's
V-JEPA 2 world model plans arm movements from 62 hours of robot video, but takes
16 s an action and needs a picture of the goal (Assran et al., 2025). Navigation
world models that imagine the view along a path drift in unfamiliar places (Bar
et al., 2025). BADGR, by contrast, labelled its own experience automatically,
collisions from the lidar, being stuck from the IMU, bumpiness and position from
odometry, and learned to predict those events from the camera and the planned
action, on a Jetson TX2 (Kahn et al., 2021). The rover's recordings already hold
those signals. For the properties of things, NeRF2Physics has a language model
propose materials and reads mass and friction off language-embedded points with
no training on those properties (Zhai et al., 2024). For movement, DynaMem's
memory, which drops and re-adds objects as they are seen to move, found moved
objects twice as often as static memories (Liu et al., 2024).

**The slow model hands the fast system goals, never motor commands.** Figure's
Helix runs a 7B model at 7-9 Hz steering an 80M policy at 200 Hz; NVIDIA's GR00T
and Physical Intelligence's π0.5 split the same way (Figure, 2025; NVIDIA, 2025;
Physical Intelligence, 2025). On this rover Nav2 already is the fast system,
and the daemon's permission is the boundary
([the daemon](../../rover_daemon/README.md#moving-by-itself-a-permission-the-daemon-can-take-back)). What to borrow is the
interface: a model says where and what, and existing machinery decides how.

**A conversation can have its eyes open.** Alibaba's realtime Qwen Omni, which
the rover talks through, accepts camera frames during a session: JPEG at 480p
or 720p, about one a second. The current flash model keeps the last 240 s of
frames, and can call tools including remote MCP servers (Alibaba Cloud Model
Studio, 2026). At listed prices, 480p at one frame a second is about a million
tokens an hour, roughly $0.25 an hour on the flash model and $2.27 on the plus
model the rover uses now. Gemini Live takes video at up to one frame a second;
OpenAI's realtime API takes still images only.

## The proposal

Five layers, each feeding the next, and one interface over all of them. Geometry
stays first and navigation keeps its authority: nothing here moves the rover,
and what it believes grants no movement
([R-SAFE-4](../requirements/safety.md#r-safe-4)).

```text
 room ──► 1 EVIDENCE        full-resolution pictures, undistorted views, depth for
           (kept as is)     every look, pose and gimbal; never rewritten
                │
                ▼
          2 FIELD           dense features for the whole picture, fused by depth and
           (3-D, dense)     pose into voxels over the flat; what was seen there and
                │           when, and what was seen through (empty)
                ▼
          3 THINGS          segments of the field, read at the granularity a question
           (on demand)      needs; today's records become anchors on them
                │
                ▼
          4 CLAIMS          what a thing is, what it is for, whether it moves, its
           (with evidence)  state, what it is on or in; who said so, from which looks,
                │           how sure; never identity
                ▼
          5 TIME            how long each thing tends to stay put; what changed today

   one memory interface ◄──► voice model (in conversation)
   search, evidence,    ◄──► coding agent (offline, over MCP)
   cards, propose claim ◄──► describing model (on the charger, at night)
```

### 1. Evidence: keep what the sensors give

Record still and aimed looks at the camera's full resolution. Run today's models
on a 640x480 reduction, so nothing downstream changes, and keep the full picture
as the evidence. A 1280x960 still already exists as a diagnostic call and took
0.39-0.49 s on the rover today, the same as a 640x480 one, at 100 kB instead of
34 kB; the full 2592x1944 is not yet allowed by that call and has not been
timed. Moving looks come off the camera's running 640x480 stream, which face
tracking shares, so whether they can be taken larger is measured rather than
assumed. Keep the depth map with every look, at the depth camera's own
resolution. Show the models undistorted perspective views cut from the fisheye
rather than the fisheye itself.

The store already keeps its pictures and the frames stay out of Git
([R-WS-1](../requirements/world-state.md#r-ws-1),
[R-PLAT-11](../requirements/platform.md#r-plat-11)). At about a thousand looks a
day, full-resolution pictures add 1-2 GB a day against 800 GB free.

### 2. The field: index all of every picture

Run a dense encoder over each whole view, one language-aligned feature per patch
of about 16 pixels, instead of one vector per region. Where the patch has depth,
put its feature into a voxel of a sparse 3D grid in the map frame (5-10 cm),
keeping a running average, how often and when it was seen, and how often it was
seen *through*: the depth ray passed the voxel and hit something behind it. The
last is the measurement the rover lacks today. It turns "nothing was filed to
that thing" into "that place was seen empty at 10:41 from two viewpoints". A
patch without depth contributes a ray, as bearings do now, with less weight.

A word query then scores the whole field, not twelve regions a picture: walls,
doors, rugs, the shelf and what is on it, the handle and not only the cupboard.
The answer can be shown as a heat map over the picture or the map, which is
also how a person checks it.

RayFronts is the starting point, with a RADIO-B or RADSeg encoder. Only looks
whose pose the scan has confirmed are fused, as the store already withholds a
direction from the rest
([R-WS-16](../requirements/world-state.md#r-ws-16)). A heading 7 degrees out
smears a feature 37 cm at 3 m. On a revisit, comparing each new look's patches
with the stored look taken nearest that pose shows where the room looks
different, above and below the lidar's one slice, at almost no extra cost.

### 3. Things: a reading of the field, not a decision at the shutter

A thing is a segment of the field: voxels that hang together in space and look
alike, read at the granularity a question asks for (the dining set, a chair, a
chair's cushion). Identity is then mostly the same voxels over time. That
attacks R-WS-17 and R-WS-18 at their root. Today two looks are joined when
their bearings cross and their vectors agree, and the plan for one thing per
object records how often that fails. In a field, two views of one object fall in
the same place, whatever angle they were taken from. The six identical dining
chairs become six places that persist, rather than six appearances nothing can
tell apart.

Today's records survive as anchors on the field, so the executive, the voice
tools and the episode references keep working while the field is proved
([episode references](../decisions/episode-references-survive-the-world-state.md)).

### 4. Claims: meaning written down with its evidence

The design already describes a claim
([autonomous-curiosity-design.md](autonomous-curiosity-design.md#evidence-and-belief-remain-different-things)):
a subject, a predicate, a value, the evidence, the method and model version, a
confidence and a time. What is new is who writes them and when:

- **a describing model, at night, on the charger**, for things that are new or
  changed today. It is shown each thing's best views at full resolution, with
  their surroundings, and fills a typed form: what it is (free words, plus a
  short name from a vocabulary the house grows), what it is for, how it moves
  (fixed, rarely, often, carried about), the state it is in, what it stands on
  or in or beside, and any writing on it. Every field may say "unsure";
- **the owner, in conversation**: "that red bean bag lives in the bedroom" is a
  claim, and it outranks a model's;
- **the coding agent**: its labels of the rover's photographs become claims
  rather than files under `experiments/`;
- **the voice model**, at the lowest standing, for what it saw in a photograph
  during a conversation;
- **the rover's own sensors**: the change watch's "moved at 07:54", the field's
  "seen empty".

A claim never decides identity: names drifted and re-identification failed in
unsafe directions when a model's words were used that way
([R-WS-7](../requirements/world-state.md#r-ws-7)). Claims hang on things that
geometry has already formed. Disagreement between claims from different views
or sources is not resolved by vote. It becomes a knowledge gap, which the
executive's curiosity can go and look at.

A describing model earns this role the way the curiosity plan already demands
of attribute claims ([Phase 4](autonomous-curiosity.md#attribute-claims-later)).
It is benchmarked offline on labelled pictures from this rover. It abstains when
unsure. Its confident claims are right about as often as it says. And it gives
the same answer twice on the same picture, which is where the last local model
failed.

### 5. Time: how long things stay put

Each thing carries a belief that it is still where it was last seen, which
decays with time at a rate that depends on what kind of thing it is. A sofa's
rate is near zero, a dining chair's is daily, and a bean bag's falls in
between. The rate starts from the describing model's "how it moves" and is then
learned from the rover's own history: every change the watch logs, and every
revisit that found the thing still there. "Is my bag in the living room?" is
then answered with how sure, how stale, and where else it has been.

Each night the rover also writes a short diary of what changed, from the change
log and the claims: "07:52, the two armchairs moved away from the balcony and
turned; a red and a black bean bag appeared from another room". A question about
the past is answered from the diary first and the evidence second.

### One memory interface, for every model that reads or writes

The daemon's world-state calls become a memory interface that both language
models use, the voice model through its tools and the coding agent over MCP:

- **search** by words, place and time, answered with evidence: the best pictures
  with the matching part marked, not only a distance and a bearing;
- **card**: a thing's claims, history and how sure the rover is it is still there;
- **diary**: what changed, over a period;
- **propose a claim**, with its source, validated before it is stored: every
  reference must resolve and every value must be of its declared type;
- **ask for a look**: queued for the executive, which decides under its own
  budgets and the daemon's permission; never a movement of its own.

The voice model's `find_thing` then hands its best picture into the
conversation through the frame handoff `look` already uses
([voice_chat](../../voice_chat/README.md#seeing-through-the-gimbal-camera)), so
the model confirms "that is the bed" by seeing it rather than by trusting a
cosine. Proposing a claim is a write. Changing
[R-CTL-4](../requirements/control.md#r-ctl-4) to allow it, claims only and never
observations or identity, is the owner's decision.

### The night shift

The rover spends most of the day on its charger. That time can be the
consolidation it has no time for while driving: in a low-priority process, with
the perception sidecar stopped to free the GPU's memory, after the last run of
the day and only while no conversation is open.

1. re-perceive the day's looks at full resolution with heavier models than run
   live;
2. fuse them into the field, and re-read the things, proposing joins and
   splits through the reviewed merge step that exists;
3. describe what is new or changed, within a daily budget;
4. update each thing's rate of moving;
5. write the diary, and a list of what the rover does not understand, which
   becomes curiosity goals and, rationed, questions for the owner;
6. teach the on-board models: small heads trained on the night's claims (this
   house's kinds of thing) and on the field's own correspondences (which views
   are the same surface), kept only if they beat the current ones on held-out
   days.

The last step is how understanding moves into the rover rather than staying
with a cloud model. What a large model said at night becomes, after enough
nights, something the rover's own small models recognise by day.

## Steps, each measured before the next

Every step is scored against the deployed pipeline on the same evidence, the
benchmark first, and nothing reaches the rover until it beats or matches it
there. The predicates are fixed here, before any result.

**0. The benchmark** (asked of Codex on 2026-10-12, read-only, in
`experiments/perception_bench/`): phrases with known answers, tape-measured
places from M4, objects with all the records that are theirs, and M5's staged
changes, each with the raw evidence it has. The deployed pipeline's scores on it
are the baseline for everything below.

**1. Full-resolution evidence.** A sweep of the gimbal from the charger across
its measured travel, without driving, recorded at 640x480 and at full
resolution with depth, then the next supervised session's looks the same way.
Today's models are run on the 640x480 pictures as now and on undistorted views
cut from the full one. *It passes if* the full-resolution views find at least
twice as many of the benchmark's things under 20 cm by their words, accept
nonsense phrases no more often, and match views of one object to each other at
least as often as now, with no time added to any move.

**2. A describing model, chosen offline.** Two or three hosted models and one
small model that fits on the rover, given the benchmark's things as the night
shift would show them. *A model qualifies if* its confident answers to what a
thing is are right at least 90% of the time against the labels, it answers
"unsure" rather than wrongly on the records labelled mixed or unclear at least
as often as it answers wrongly, and it gives the same answer to the same picture
at least 49 times in 50. How it judges movement and function is reported but is
not a gate until the owner has labelled those for fifty things.

**3. The field, offline.** Built from the looks that carry depth and their
recorded poses, and scored against the deployed records. *It passes if* the
plan for one thing per object's predicates hold: same-object pairs together at
least as often as now, different-object pairs together no more often
([semantic-world-state.md](semantic-world-state.md#what-is-still-to-find-out)).
It must also find the benchmark's phrases at least as often at the same rate of
accepted nonsense, and place the taped things no worse. For M5, it is reported
whether "seen empty" from the field finds the moved armchairs' old places at
furniture size, which the lidar found small
([2026-10-11](../progress/2026-10-11-m5-scene-1-replayed.md)), and whether it
stays silent on the controls.

**4. Claims and the memory interface.** The claims table, added without
rewriting anything ([R-WS-9](../requirements/world-state.md#r-ws-9)); the
interface calls; `find_thing` answering with a picture; the owner's words and
the coding agent's labels stored as claims. Proposing claims from the voice model
waits on the owner's decision about
[R-CTL-4](../requirements/control.md#r-ctl-4).

**5. The night shift on the rover**, one job at a time: re-perception and the
field first, then the diary, then describing within a budget, then the small
models it teaches. Each job is measured for time, memory and charge while
docked. The diary is checked against the change log and the owner's word for
each M5 session.

What it would move: [R-WS-17](../requirements/world-state.md#r-ws-17) and
[R-WS-18](../requirements/world-state.md#r-ws-18) through the field;
[R-WS-11](../requirements/world-state.md#r-ws-11), a thing's height, which the
field measures directly; [R-WS-12](../requirements/world-state.md#r-ws-12),
because in a field the floor is the floor rather than a candidate thing; and
[R-WS-13](../requirements/world-state.md#r-ws-13) for the actions that rely on
identity. It would propose three requirements, numbered when agreed: a
look's evidence is kept at the resolution the sensor gave; a claim carries its
evidence, method and confidence, abstains when unsure, is right about as often
as it says, and never decides identity; and an answer about whether something
is still there states how stale it is.

## Where it runs, and what it costs

Measured figures from NVIDIA's benchmarks and published reports; none of the
newer models has been timed on this rover. The Orin runs JetPack 7.2.

| Where | What fits | What it costs |
|---|---|---|
| On the rover while it drives (2.5-3.5 GB free) | today's models; one more dense encoder at about a picture a second (DINOv2-B ran at 126 frames a second on an Orin Nano in Super mode); prompted masks (NanoSAM, 27 ms); depth fused into voxels (NVIDIA's nvblox took 2-13 ms a step on an Orin Nano, memory not published, and its current release targets a newer ROS than the rover's) | nothing but the memory headroom; the vision-language models worth having need 4.3-4.7 GB and do not fit |
| On the charger, with the perception sidecar stopped (about 5 GB) | a 2B vision-language model through TensorRT Edge-LLM: Qwen3-VL-2B took 4.5 GB, read a picture in 167 ms and wrote 36 tokens a second, so about 4-5 s a description and 2.5-3 hours for 2,000; bigger encoders over the day's pictures; the field rebuilt | electricity, and heat: the Orin runs without its fan, and no multi-hour GPU job has been measured on it |
| In the cloud | descriptions of the quality the coding agent gives; a whole-flat Gaussian splat, which needs a desktop-class GPU (no splatting has been measured on an Orin Nano) | $0.35-9 a day to describe 2,000 pictures and $0.05-1.25 a day for 50 questions over retrieved pictures, depending on the model; batch pricing halves it |

**Paid use is the owner's decision.** The rover's conversations run on the free
quota of a dated model snapshot, a million tokens that end on 2026-11-15
([voice_chat/session.py](../../voice_chat/session.py)). An hour of conversation
with the camera streaming at 480p, one frame a second, is about a million tokens
by itself. Describing at night, or seeing during conversation, needs
pay-as-you-go turned on with a provider the owner chooses. A slower path, the
2B model on the charger, keeps every picture in the flat.

## Moon-shots

These are further out, and each would be its own experiment. They are here
because the owner asked for them, and because the five layers above are what
they would stand on.

**The flat as a model the rover can render.** Each night, rebuild a
photographic 3D model of the flat from the day's pictures and depth, as a
Gaussian splat with language features attached, on a rented or desktop GPU.
Then any change is a difference between what the model predicts from a pose and
what the camera sees from it. 3DGS-CD does this for rearranged objects: it
renders the earlier splat from the new camera's pose and segments the
differences, in about 18 s per new picture, given at least 150 earlier ones (Lu
et al., 2025). That catches a cup moved on a table, a picture
taken off the wall, a door half open: every height and size, where the lidar
sees one slice at 0.6 m. It also lets anyone look at any place from any angle
without driving there, and "what did the corner by the balcony look like on
Tuesday" becomes a picture rather than a search.

**Learning the house's physics from its own life.** Every stall, slip and bump
the rover has had is in its recordings, along with the wheel odometry, the IMU
and the lidar that would have told it so. BADGR's recipe labels those events
from the robot's own sensors and learns to predict them from the picture and the
planned move. The rug that stopped the rover on 2026-10-11 becomes a place with
a claim, "stalls the wheels when crossed at a shallow angle", rather than a box
drawn by hand. Further out, a small model that predicts the next view from the
current one and the move, trained on the rover's own driving, would give
curiosity a measure of surprise: go where your predictions fail. None of this
reaches the safety layer, which still treats every obstacle the same in motion.

**The rover asks.** When something new and furniture-sized has appeared and the
describing model is unsure what it is, the rover can raise it once in
conversation, the way it already announces an arrival: "something red appeared
by the window yesterday; what is it?" The answer becomes a claim with the
owner's name on it. TidyBot turned a person's few examples of where things go
into rules that placed 85% of new objects correctly on a real robot (Wu et al.,
2023). The owner telling the rover where things belong is the same signal, and
the strongest the rover can get.

**Its own eyes in the conversation.** Stream the camera into the realtime
session while someone is talking to the rover, and give the session the memory
interface as tools. "This one" and "over there" can then be understood, and what
the model sees can be offered back as claims for the owner to confirm. Costs
are in the table above.

**Its own vocabulary.** Cluster the field's segments across the whole flat into
the kinds of thing this house contains, and let the describing model name each
kind. The six identical dining chairs are then one kind with six places,
followed by where they are and where they went, which is how a person tells
them apart too.

**Answers ready before the question.** From what the owner asks about, the night
shift can keep answers ready, where the bag is, what changed in the living
room, so that the conversation answers at once and the evidence is already
chosen.

## What it changes in the rules

- [R-WS-7](../requirements/world-state.md#r-ws-7) stands as written. Claims do
  not name a thing for identity, and no claim is taken from the nearest text; a
  new requirement says what a claim must carry instead.
- [R-CTL-4](../requirements/control.md#r-ctl-4), read-only access for the
  conversational model, would allow proposing claims, never observations or
  identity. The owner's decision.
- [R-WS-4](../requirements/world-state.md#r-ws-4) holds: the field's features
  are a backend of their own and are never compared with today's vectors. Keeping
  the evidence is what lets the field be rebuilt whenever its encoder changes.
- [R-WS-5](../requirements/world-state.md#r-ws-5) and
  [R-WS-6](../requirements/world-state.md#r-ws-6) hold: the field belongs to a
  map session, and a new map is built again from the evidence and its poses.
- [R-SAFE-4](../requirements/safety.md#r-safe-4) holds: nothing believed grants
  movement, and no claim enters the safety layer.
- Pictures go to a hosted model only once the owner has chosen to allow it.
  Today they leave the flat only when the voice model is shown a `look`.

## What could go wrong

- **A describing model's confident nonsense**, as the last local one produced.
  It is contained by the benchmark gate, by "unsure" being an answer, by
  agreement across views and sources, by every claim carrying its source, and by
  claims never deciding identity.
- **Cost that creeps**: a daily budget for the night shift, batch prices, and a
  report of what each night spent.
- **Heat and memory on the Orin overnight**: one job at a time, its temperature
  and memory recorded, and stopped at a declared limit.
- **Two representations at once**: the field is built beside today's records
  and replaces none of them until it beats them on the benchmark.
- **The conversation's free quota ends on 2026-11-15** whatever this plan
  does.

## Sources

Opened on 2026-10-12 by the research behind this plan. Preprints from 2026 are
not yet peer-reviewed, and most are scored on their authors' own benchmarks.

Representation:
Huang et al., 2023, VLMaps, https://github.com/vlmaps/vlmaps;
Peng et al., 2023, OpenScene, https://arxiv.org/abs/2211.15654;
Heinrich et al., 2025, RADIOv2.5, https://arxiv.org/abs/2412.07679;
Alama et al., 2026, RADSeg, https://arxiv.org/abs/2511.19704;
Alama et al., 2025, RayFronts, https://github.com/RayFronts/RayFronts;
Maggio et al., 2024, Clio, https://github.com/MIT-SPARK/Clio;
Maggio et al., 2026, FOUND-IT, https://arxiv.org/abs/2605.25371;
Linok et al., 2025, Beyond Bare Queries, https://linukc.github.io/BeyondBareQueries/;
Yan et al., 2024, MaskClustering, https://pku-epic.github.io/MaskClustering/;
Gorlo et al., 2026, DAAAM, https://github.com/MIT-SPARK/DAAAM;
Ravi et al., 2025, SAM 2, https://github.com/facebookresearch/sam2;
Carion et al., 2026, SAM 3, https://github.com/facebookresearch/sam3;
SAM 3D Team, 2026, SAM 3D Objects, https://huggingface.co/facebook/sam-3d-objects;
Lu et al., 2025, 3DGS-CD, https://github.com/520xyxyzq/3DGS-CD.

Evidence and memory:
Majumdar et al., 2024, OpenEQA, https://open-eqa.github.io/;
Yang et al., 2025, 3D-Mem, https://arxiv.org/abs/2411.17735;
Saxena et al., 2025, GraphEQA, https://arxiv.org/abs/2412.14480;
Wang et al., 2026, What Should Embodied Agents Remember?, https://arxiv.org/abs/2610.12561;
Schmid et al., 2024, Khronos, https://arxiv.org/abs/2402.13817;
Liu et al., 2024, DynaMem, https://arxiv.org/abs/2411.04999;
Rosen et al., 2016, the persistence filter, https://dspace.mit.edu/handle/1721.1/107620;
Saavedra-Ruiz et al., 2025, Perpetua, https://arxiv.org/abs/2507.18808;
Adkins et al., 2026, PreSIST, https://arxiv.org/abs/2607.04057;
Saavedra-Ruiz et al., 2026, Predictive Spatio-Temporal Scene Graphs, https://arxiv.org/abs/2605.00121;
Krajník et al., 2017, FreMEn, https://github.com/gestom/fremen;
Huang et al., 2024, BYE, https://arxiv.org/abs/2412.02449;
Frey et al., 2022, continual adaptation from 2D-3D maps, https://arxiv.org/abs/2111.02156;
Park et al., 2023, Generative Agents, https://arxiv.org/abs/2304.03442;
Lin et al., 2025, Sleep-time Compute, https://arxiv.org/abs/2504.13171;
Anwar et al., 2025, ReMEmbR, https://arxiv.org/abs/2409.13682;
Zhang et al., 2026, WorldLines, https://arxiv.org/abs/2606.18847;
Wu et al., 2023, TidyBot, https://arxiv.org/abs/2305.05658.

Understanding and integration:
Gu et al., 2024, ConceptGraphs, https://concept-graphs.github.io/;
Yoneda et al., 2024, Statler, https://arxiv.org/abs/2306.17840;
Ali et al., 2025, GraphPad, https://arxiv.org/abs/2506.01174;
Chow et al., 2025, PhysBench, https://arxiv.org/abs/2501.16411;
Assran et al., 2025, V-JEPA 2, https://arxiv.org/abs/2506.09985;
Bar et al., 2025, Navigation World Models, https://arxiv.org/abs/2412.03572;
Kahn et al., 2021, BADGR, https://arxiv.org/abs/2002.05700;
Zhai et al., 2024, NeRF2Physics, https://arxiv.org/abs/2404.04242;
Figure, 2025, Helix, https://www.figure.ai/news/helix;
NVIDIA, 2025, GR00T N1, https://arxiv.org/abs/2503.14734;
Physical Intelligence, 2025, π0.5, https://arxiv.org/abs/2504.16054;
Alibaba Cloud Model Studio, realtime models and prices,
https://www.alibabacloud.com/help/en/model-studio/realtime and
https://www.alibabacloud.com/help/en/model-studio/model-pricing;
Google, Gemini Live API, https://ai.google.dev/gemini-api/docs/live-api/capabilities;
OpenAI, realtime conversations, https://developers.openai.com/api/docs/guides/realtime-conversations.

Hardware and prices:
NVIDIA TensorRT Edge-LLM benchmarks (JetPack 7.2),
https://nvidia.github.io/TensorRT-Edge-LLM/latest/user_guide/performance/performance-benchmarks.html;
NVIDIA, Jetson Orin Nano Super,
https://developer.nvidia.com/blog/nvidia-jetson-orin-nano-developer-kit-gets-a-super-boost/;
NanoSAM, https://github.com/NVIDIA-AI-IOT/nanosam;
nvblox, https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html;
price pages: https://ai.google.dev/gemini-api/docs/pricing,
https://platform.claude.com/docs/en/about-claude/pricing,
https://developers.openai.com/api/docs/pricing.
