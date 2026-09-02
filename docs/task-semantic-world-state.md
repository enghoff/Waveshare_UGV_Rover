# Task: a spatially grounded semantic world state

_Supersedes `task-cosmos-world-state-poc.md` and
`task-cosmos-world-state-reidentification.md`, both folded into this document on
2026-09-01. What the proof of concept measured is kept in
[`world_state/README.md`](../world_state/README.md), which is the record; this is
the plan._

## Where this stands

**All five phases are built, deployed and running on the rover, and the
validation drive has now happened.** Driven round three places in a real room and
inspected at six headings from each, the rover placed twenty-three things from
two hundred and six observations: one television from seven looks, a sofa, a
cabinet, a doorway, and *two* armchairs, which is how many the room has. The
person in the room came out at (0.04, 0.63) and the armchair he was sitting in at
(0.10, 0.55) -- ten centimetres apart, which is the whole claim of this programme
working on hardware.

It also found three faults that no selftest could have, written up under *The
validation drive* below. Nine duplicate entities remain, and closing them needs
one mechanism this design does not yet have at all -- see *What is left*.

The store was cleared after the drive, so those twenty-three things are a record
of what happened rather than what the rover is holding now. Nothing populates the
world state on its own: it records when an inspection asks it to, and a thing
gets a position only once the rover has driven between two looks.

An inspection now measures rather than asks. It costs about a fifth of a second
on the GPU, stores twelve regions with a bearing worked out from the rover's own
pose and gimbal angle and two vectors apiece, and then decides which lasting
thing each one belongs to from where it is. On the rover, standing still, that
decision is correctly *nothing*: the observations sit in the pending pool with a
direction and no home, and the popup says so, because a thing seen once from one
place is a thing the rover cannot honestly claim to have located.

Describing a thing in words finds it. Forty queries against the rover's own
stored regions, twenty-four for things it had seen and sixteen for things that
are not in the building, come out right thirty-six times -- and the four it gets
wrong are three misses and one near-hit ("a laptop computer" finding the
television), which is the safe direction to err in. That measurement replaced the
rule the search shipped with; see *Saying "nothing matches"* below.

**What has not been demonstrated is the case where it can place something**,
which needs the rover to drive between two looks. Everything up to that point is
proved in the selftests, including the two-identical-chairs case, and none of it
is proved on the hardware.

Built, running on the rover and worth keeping: a SQLite store that separates
immutable observations from canonical entities, application-owned identifiers,
stored frames with the gimbal angles and rover pose behind each one, a read-only
console popup, a clear that does not touch the SLAM map, a local model in a
sidecar process that cannot take the daemon down with it, and a deploy component
with its own verification.

Disproved, twice, on the rover's own frames: **that a vision-language model can
be asked which persistent thing it is looking at.** Cosmos Reason 2 2B never
matches — shown four entities it had itself created, on a byte-identical frame,
it matched none and made four more. Cosmos 3, an 8B model and much the better
eye, fails the opposite way: shown a known list containing a grand piano and a
fish tank, neither of them in the room, it matched the armchair to the piano and
drew the fish tank into the scene, description copied verbatim from the list. It
will not say "new".

Neither failure is a prompt problem, and no better model fixes it, because the
information is not in the picture. **Two identical chairs at opposite ends of a
room are identical in the image.** What separates them is where they are.

So identity moves out of the model and onto measured geometry, and the model
loses the per-look job entirely.

---

## The approach

```text
                       gimbal camera
                             |
                             v
                        one JPEG frame
                             |
              +--------------+--------------+
              |                             |
              v                             v
      FastSAM: regions                SLAM pose + gimbal angle
      (no categories)                        |
              |                              |
              v                              |
      crop each region                       |
        /           \                        |
       v             v                       v
   SigLIP2        DINOv2              bearing per region
  embedding      embedding             (view.ray, exists)
       |             |                       |
       |             |                       v
       |             |              two bearings from two
       |             |              places -> a map point
       |             |              (locate.fix, exists)
       |             |                       |
       +------+------+-----------------------+
              |
              v
         entity resolver
   spatial gate, then appearance
              |
      MATCH / NEW / AMBIGUOUS
              |
              v
          SQLite world state
        observations + entities
        + embeddings + placement
              |
        +-----+-----+
        |           |
        v           v
   console      text search
    popup     "find the spray bottle"
```

Four jobs, four mechanisms, and the important part is that they are separate:

| Question | Answered by |
|---|---|
| What regions are in this frame? | FastSAM, which knows no categories |
| Is this the same object as that one? | DINOv2 similarity, gated by geometry |
| **Which persistent thing is it?** | **Triangulated map position** |
| Find me the thing I am describing | SigLIP2, text against stored embeddings |
| What is it called? | Derived from the SigLIP2 vector, not detected |
| What is that, in words? | The VLM, on demand, when a person asks |

---

## Why this shape, measured

Every choice below came from the rover's own frames rather than from a benchmark.
The numbers are worth keeping because they are what a later reader would
otherwise have to rediscover.

### Boxes are trustworthy; labels are not

On two byte-identical frames, the detectors redrew the same box to within 0.001
to 0.003 of the frame width — **0.13° to 0.26° of bearing**. Cosmos Reason 2 on
the same pair of frames moved the sofa's bearing by **4.8°** and the coffee
table's by 2.6°, purely from redrawing the box.

Labels went the other way. Florence-2 called one chair a "black leather
recliner" and then, on the identical frame, a "blue leather recliner"; the twin
chair became a "black leather couch". Three names for two chairs. Cosmos Reason 2
drifts between sofa, couch and armchair the same way.

**So the box is the measurement and the label is a hint.** Anything that keys
identity off the label inherits the drift.

### Bearing accuracy buys driving distance

Two bearings from two places intersect at a point. How well depends on how far
apart the two places were and how good the bearings are. For an object four
metres away, position uncertainty:

| bearing error | 1.0 m | 1.5 m | 2.0 m | 3.0 m | 4.0 m | 6.0 m apart |
|---|---|---|---|---|---|---|
| 5° (Cosmos Reason 2) | — | 4.50 m | 2.24 m | 1.05 m | 0.72 m | 0.57 m |
| 3° | — | 1.84 m | 1.09 m | 0.57 m | 0.41 m | 0.33 m |
| **1.5° (measured, 2026-09-02)** | — | **0.75 m** | **0.47 m** | **0.26 m** | **0.19 m** | **0.15 m** |
| 1° (a detector at its best) | — | 0.46 m | 0.30 m | 0.17 m | 0.13 m | 0.11 m |

Read across the top row: with the VLM's boxes the rover had to drive three to
four metres between looks to place a thing within a metre. **It now needs about a
metre and a half**, and this row is measured rather than projected -- see below.

The dash at 1.0 m is not a rounding error. Below about 12° of parallax the
intersection runs away down the line of sight and `locate.fix` refuses to answer,
which is the correct response and not a failure.

#### What a bearing is actually made of, measured 2026-09-02

The 5° figure was taken when a language model was drawing the boxes. Re-measured
now that FastSAM draws them, in three parts, because knowing which part dominates
is what says whether it is worth trying to improve:

| term | how it was measured | result |
|---|---|---|
| the box | eight inspections of an unchanging scene, regions matched between them by their DINO vectors | **0.13°** of scatter, worst 0.16 |
| the rover's heading | the same inspections, over the two minutes of a gimbal sweep, standing still | **0.2°** |
| the gimbal | the same objects at pan −30, −15, 0, +15, +30 | **0.7°** out to ±15, **3°** at −30 |

`BEARING_SIGMA_DEG` is 1.5, the root-mean-square across that whole sweep.

Two conclusions. **The box is no longer the problem** -- it fell by a factor of
forty and is now the smallest of the three terms, so nothing is gained by a
better region finder. **The gimbal is**, and it is not noise but the pan servo
not arriving where it was told: the error at −30° is the same size and sign for
two different objects on opposite sides of the frame, which rules out the lens,
and the lens itself checks out to within 0.9° of its own calibration. There is
nothing to correct it with, because the driver board's telemetry carries the
inertial sensors and the wheel encoders but no gimbal feedback. **A rover that
inspected only within ±15° of pan would do better than 1.5° says.**

What none of this covers is *driving*. Every measurement above was taken standing
still, so the heading term is two minutes of drift and not the error SLAM
accumulates over a few metres -- which is exactly the case a fix is taken in.
That still wants measuring, and it needs the rover to drive.

#### Saying "nothing matches"

The search shipped with a relative rule: SigLIP's raw cosines were held to be
uncalibrated, so a real match was one that stood clear of the field rather than
one that scored above a line. **Measured on the rover, that is wrong.** Forty
queries against its own stored regions:

| statistic | present queries | absent queries | best cut it can make |
|---|---|---|---|
| raw score | 0.065 – 0.140 | 0.040 – 0.098 | **4 of 40 wrong at 0.09** |
| stands clear of the field | 1.58 – 4.45 | 1.56 – 3.07 | 14 of 40 wrong |

The separation carries no signal at all -- its two distributions are the same
shape. So `MATCHES = 0.09` decides and the separation is only reported, and the
old rule is kept as a test so it cannot come back by accident.

### Decomposition wants no vocabulary at all

Region counts on the same three frames, and every one of these is a *class-
agnostic or open-vocabulary* method:

| | regions per frame | per frame | notes |
|---|---:|---:|---|
| YOLOv8n (COCO) | 1 | 40 ms | found only the chair; no doorway class exists |
| Florence-2 `<OD>` | 3 | 1.9 s | missed table, doorway, cable, purifier |
| Florence-2 `<REGION_PROPOSAL>` | 1 | 1.8 s | worse than its own detection mode |
| **FastSAM** | **35, ~18 after filtering** | **0.12 s** | found the bottle, purifier, pictures, doorway |

FastSAM wins by a wide margin on the thing that matters, which is recall, and it
wins because it is not trying to name anything. The filter that takes 35 down to
18 is three lines: drop regions larger than a third of the frame (floor, wall),
smaller than 0.4% (a highlight on a tile), or more than six times longer than
they are wide.

### The name can be derived rather than detected

Ranking a region's SigLIP2 embedding against a word list named the spray bottle,
the framed picture, the armchair, the air purifier and a doorway in the forward
frame. In the twin-chair frame the top-scoring region at 0.162 was **"a sheepskin
blanket"** — the most distinctive thing in that picture, which no detector
labelled at all.

This matters beyond convenience. The vocabulary lives in a config file rather
than in a model, so **changing the word list re-labels every stored object
without reprocessing a single frame.**

Two limits to carry forward. A word list still exists; it has only moved
somewhere editable. And SigLIP's raw cosines are not calibrated — scores sit
around 0.10 to 0.16 and only the ranking is meaningful, so a "nothing here"
threshold has to be calibrated against real crops rather than guessed.

### Appearance is evidence, geometry is the key

DINOv2 scored the same chair across two frames at **0.995** and the twin chair at
**0.653**. SigLIP2 on the same pairs gave 0.990 and 0.87 — a much narrower
margin, because SigLIP is trained for semantics and sees chair-ness where DINOv2
sees an instance.

That endorses using both, for different jobs. It does **not** promote DINOv2 to
the primary key: some of that 0.653 is viewpoint rather than identity, since the
twin was nearer, at a different angle, and wearing a blanket. Two identical
chairs seen from similar distances would score high on both. Geometry stays the
arbiter and appearance stays the tiebreaker.

### The VLM has no place in the per-look path

A single Cosmos inspection costs 59 to 70 s on the Orin's CPU. Moving it to the
GPU is real and cheap — llama.cpp's prebuilt `ubuntu-vulkan-arm64` binary is
27 MB, needs no toolchain, and took the same frame to 25.7 s at 33 tokens a
second against 7 — but the floor is still ten to fifteen seconds, because most of
the time is the model *writing*, not looking. The pipeline above runs in well
under a second.

Keep the VLM for the conversational `look`, where a person is waiting for prose
and a minute is acceptable.

---

## Existing decisions that stay closed

Do not reopen these while doing this work.

- **SLAM Toolbox and the current Nav2 stack remain authoritative** for map
  geometry, robot pose, path planning and control.
- **MPPI** was validated separately and rejected for this rover. Do not
  reintroduce it.
- **RTAB-Map** was validated separately and rejected. Projection from a bearing
  and a SLAM pose is sufficient here; it is not needed as an intermediary just
  because images are involved.
- The geometric `explore` remains the autonomous exploration baseline.
- **Cosmos 3 is closed.** It cannot run here: the 4B Edge variant is an
  architecture llama.cpp cannot load, and the only runnable build wants 6.19 GB
  of weights against 5.85 GB of measured headroom on a board with no swap.
- **Cloud models are on hold** at the owner's direction, not ruled out. If they
  return, they change the perception ceiling and nothing about identity.
- No model gets movement authority in this task.
- No vector database. Dozens of objects and a 768-float vector is a SQLite BLOB
  and a numpy dot product.

## Authority boundaries to preserve

```text
Qwen Omni                user-facing voice/conversation/tool agent
rover_daemon             single model-facing rover/hardware boundary
SLAM Toolbox + Nav2      geometry, planning, control, recovery
current explore          autonomous geometric frontier exploration
D500 lidar               navigation obstacle/map source
```

The daemon owns the camera. Nothing here may open it a second time; reuse the
existing capture path, which already retries once because a grab that closely
follows another comes back empty. Perception runs in the sidecar process, not in
`rover_daemon`, so a model fault cannot take STOP or the UART owner down with it.
The world-state calls stay control calls rather than model-facing tools.

---

## Implementation plan

Five phases. Each is deployable on its own and each ends with something
measurable, so the sequence can be stopped at any point without leaving the rover
half-converted.

### Phase 0 — stop asking the model for identity — **done, deployed 2026-09-01**

The smallest change, and it makes what is already deployed honest rather than
wrong.

Running on the rover at `ed53402`. An inspection now returns a kind, a name and a
box per thing and stores each with the frame, the gimbal angles and the rover
pose behind it, creating no entity: `stored 2, created 0, matched 0`, with
`entity_id` null on both rows and no description or location hint written. Two
measured side effects, neither predicted: **an inspection fell from 56–89 s to
38 s**, because most of the cost was the model writing the two fields that are
gone, and the model's own detections are cleaner — a dimly lit room came back as
a dining table and a doorway with no invented coffee table. Lidar reported a
0.01 s scan age and zero dropped scans throughout.

Two things had to be decided along the way and are worth not re-deciding.
The prompt does not mention identity **even to forbid it**, because naming the
subject in order to forbid it is still naming it and what was measured is that
raising previously-seen objects at all changes what the model reports. And an
identity the model volunteers anyway is stripped and reported, the same treatment
metres get, so that a stale build or a model with an opinion of its own cannot
put a guess back into the store by the back door.

The rover's database still holds the fourteen duplicate entities the POC created.
They are inert -- nothing matches to them and nothing updates them -- and a
`world_state_clear` before the validation drive is the moment to be rid of them.

- Remove `existing_entity` from the prompt and from `RESPONSE_SCHEMA`.
- Remove the known-entity list from the prompt entirely. Measured: with the list
  gone the 2B's own detections improve, from four observations to six on one
  frame, and it stops reporting a coffee table that is not in the room. The list
  contaminates perception, not merely identity.
- Drop `description` and `location_hint` from the schema. They are roughly 60% of
  the generated tokens and nothing downstream will key off them once SigLIP2
  supplies the naming.
- Every inspection becomes an honest record of what was seen from a measured pose
  at a known time. Entities stop being created at all until Phase 3.

**Done when** an inspection stores observations with no identity claim, the
world-state selftest passes, and the change is deployed and verified on the Orin.
All three met.

### Phase 1 — the perception sidecar — **done, deployed 2026-09-01**

Running on the rover at `f1a00d`. The three models are installed and answer on
loopback 8776 in a process of their own; `install_perception.sh` fetches them and
`bench_perceive.py` measures them. Four things came out of it that change what
the later phases can assume.

**The sub-second target was missed and cannot be met here. One look is 2.0 to
2.3 seconds** for twelve regions. There is no GPU lever: this JetPack has the
driver but no CUDA toolkit and no cuDNN, and NVIDIA's Jetson wheel index stops at
JetPack 6, so an ONNX Runtime GPU provider would mean building it from source
against CUDA 13 — far outside the "few hundred megabytes" this phase allowed.
The CPU is saturated above four threads, FastSAM is already one size above where
it starts losing objects, and the only configuration that reaches 1.14 s does it
by embedding eight regions instead of twelve. **What the target was protecting is
intact**: zero dropped lidar scans throughout, and a look is thirty times cheaper
than the language model it replaces. Take the miss or drop to eight regions; that
is a call for the owner rather than for this document.

**The GPU was worth 4x to the language model.** llama.cpp's Vulkan build took an
inspection from 38 s to 9.5 s for a 27 MB download and no toolchain, which is the
top of the range this phase predicted.

**`llama-server` had been leaking the whole board, since before this work.** Its
prompt cache defaults to 8192 MiB on a machine with 7485 MiB and no swap, so it
grew about a hundred megabytes an inspection: sixteen inspections took the rover
from 1.5 GB free to 48 MB. `--cache-ram 0` fixes it at no cost. The CPU build
leaks identically, so this is not Vulkan's doing and the POC's "about 4 GB
resident" was this caught halfway up.

**Appearance is weaker than this document assumed, and phase 3 must be told.**
The 0.995-against-0.653 figure quoted below compares the same chair from the
*same viewpoint* against a different chair from a different one. Measured across
a real change of viewpoint, the same chair scores 0.696 and the twin chair across
the room scores **0.735** — higher. So DINOv2 answers "does this look like that
picture", not "is this the same object". The design already makes geometry the
arbiter; what changes is that appearance cannot be a strong tiebreaker either,
and the resolver must never let it overrule a placement.

The original plan for this phase follows.

### Phase 1 as planned

- Put ONNX Runtime with a GPU provider on the Orin. Prefer it over PyTorch: a few
  hundred megabytes against two to three gigabytes. The rover's packages are
  vendored under `~/ugv/vendor`, alongside the existing OpenCV.
- Add FastSAM, SigLIP2 and DINOv2 behind one interface, in the existing sidecar
  process rather than in `rover_daemon`.
- Benchmark all three on the Orin GPU against the three recorded frames. The
  desktop figures — 0.12 s, and 129 ms a crop for DINOv2 on a CPU — are a
  starting point, not a prediction.
- Keep `PhysicalReasoner` and the llama.cpp sidecar for the conversational
  `look`. While that is still in the per-look path, switch it to the Vulkan
  binary: 27 MB, no toolchain, 2.4x to 4x.

**Done when** one frame yields regions, embeddings and derived labels on the
rover in under a second, with the lidar still reporting no dropped scans. The
lidar half passed; the second did not, and why is above.

### Phase 2 — placement — **done, deployed 2026-09-01**

Every observation now carries the bearing worked out at the moment of the look,
the box's angular width, what drew the box, both vectors as raw float32, and
**which backend produced those vectors** — that last one because the GPU engines
and the CPU graphs agree with full precision to 1.000 and 0.86, which is far too
wide a gap to compare across. An inspection goes through the encoders rather
than the language model.

Three things the first real inspection on the rover showed, none of them
predicted:

- A bearing is three numbers added together and it ran past half a turn: the
  rover stored −205.9°, which points exactly where +154.1° does and compares
  with nothing. Every bearing is wrapped now.
- The two vector columns were about to reach the console as raw bytes, which
  JSON cannot serialise. The entity list would have crashed the first time it
  met an observation carrying one.
- The camera grab, not the model, is now the expensive part of an inspection:
  6.3 s of which the look was 0.35 s, and 0.6 s on the second inspection when
  the camera was already open.

**Not demonstrated:** the recorded multi-position drive. That needs the rover to
move between two looks, and it has not been driven.

### Phase 2 as planned

`world_state/locate.py` already exists, with 196 passing tests covering
triangulation, the two-identical-chairs case, and the turning-on-the-spot
degeneracy. What is missing is the plumbing.

- Store a bearing per observation, as `view.ray` already computes.
- Hold observations that cannot yet be placed in a pending pool. One bearing is a
  direction, not a position, and an object gets a location only when a second
  compatible look arrives from somewhere far enough away.
- Add to the schema, extending rather than replacing:

```text
observations   bearing_deg, span_deg, region_source, siglip_blob, dino_blob
entities       placement_json, placement_uncertainty_m, placement_map_session,
               placement_updated_at, exemplars
```

- Keep every observation-level measurement when an entity's estimate is updated.
  Never destroy the evidence that produced a placement.
- A `map_session` change invalidates comparison. Never compare coordinates across
  sessions.

**Done when** a recorded multi-position drive produces map points with
uncertainties, and the popup can show which two looks placed each thing.

### Phase 3 — the resolver — **done, deployed 2026-09-01**

Built as planned, with the gates in the planned order, and one thing the plan
did not anticipate: **the phantom**.

Two identical chairs seen from two places give four rays and *four* valid
crossings — the two chairs, and two ghosts where a ray to one chair crosses a
ray to the other. All four are sound geometry. Appearance cannot break the tie,
for the reason recorded further up: the twin chair scores 0.735 against the same
chair's 0.696 across a change of viewpoint. From two viewpoints the answer is
genuinely not knowable.

Two things settle it, and both are in the code with the measurement that forced
them. Support for a crossing is counted in **viewpoints rather than in rays**,
because two regions of one frame cannot be the same object — without that a
ghost 0.7 m from the rover collected three rays from two frames and beat the
real chair at 4.2 m, since close to the camera every bearing agrees with
everything. And a crossing that ties with a conflicting one built from a shared
ray is refused outright, which is how the two-chair case comes out AMBIGUOUS
until a third look arrives from somewhere else. With that third look it resolves
into two things in two places.

**Re-measured on 2026-09-02 and now 1.5°**, and the working out is under
*Bearing accuracy buys driving distance* above. Tightening it broke the
two-identical-chairs test, and why is worth keeping: the resolver refuses a
placement when a rival crossing sits further away than their combined uncertainty
allows, so shrinking the uncertainty made it refuse cases it had been accepting.
Better measurement must not cost the rover answers. Two placements within
`SAME_PLACE_M` of each other now name the same thing whichever is right.

### Phase 3 as planned

Three outcomes, and the third is not optional:

```text
MATCH       one candidate is spatially and visually consistent
NEW         no candidate is spatially consistent
AMBIGUOUS   two candidates are, or the evidence is too thin to choose
```

Order, cheapest gate first:

1. **Semantic gate** — compatible labels only, with a small synonym set. A hint,
   not a key.
2. **Map-session check** — compare positions only within one session.
3. **Spatial gate** — reject candidates outside the placement uncertainty. For
   static furniture this is a hard gate. A very high appearance score must never
   override clearly incompatible placement; that is the redundant-furniture test
   and it is the one the whole design exists to pass.
4. **Appearance** — DINOv2 against stored exemplars, several per entity rather
   than one averaged vector.
5. **History** — recent visibility and placement as supporting evidence.

Keep the pieces inspectable rather than fusing them into a fake probability:

```text
candidate object:7   appearance 0.98   4.2 m away  -> spatial reject
candidate object:9   appearance 0.97   0.16 m away -> MATCH
```

Movable things — bottles, bags, cables — cannot have location as an eternal
identity rule. Keep the first cut conservative: placement is strong for furniture
and openings; for movable objects keep the placement history and allow relocation
only on strong appearance evidence; and prefer `AMBIGUOUS` to a confident wrong
answer. A full object-motion model is not wanted in this slice.

**Done when** the acceptance criteria below pass on the rover.

### Phase 4 — search and the console — **done, deployed 2026-09-02**

Describe a thing in your own words and the rover finds it. The phrase goes
through the same text tower that named every region, so it lands in the same
space as the stored vectors and the comparison is a dot product over a few
hundred of them -- there is no vector database in this design and should not be
one. Measured end to end through the console: "a wooden chair" comes back
confident with the chair on top, "a slice of pizza" comes back saying nothing
here matches, and both answers arrive in about four seconds.

The four seconds are the model, not the search. On the GPU the text tower is
loaded for the call and given back afterwards, and **getting that to fit was most
of the work in this phase.** Three faults, all found by running it on the rover
and none of them visible offline:

* The query path reached for the numeric library the loader installs *before*
  calling the loader, so the first search after a reboot died with an
  `AttributeError`. Every search on the rover did, because the sidecar loads its
  models on the first look and a search is not one.
* Loading the text tower asked the board for 1.1 GB while Python still held the
  same 1.1 GB as the bytes it had just read. On 7.4 GB shared with the GPU and
  three gigabytes of language model that is where it ran out, and the
  out-of-memory killer took llama-server and then the sidecar. The engine file is
  mapped now rather than read.
* Even then the text tower and the three engines a look needs do not both fit.
  TensorRT answers `None` instead of raising when it cannot make an execution
  context, so the failure arrived several frames later as an attribute error on
  nothing. A search now puts a look's engines down first and the next look picks
  them up again, which costs a few seconds afterwards -- the right way round for
  something a person types.

A fourth was in the deployer rather than the rover: `rover_daemon` holds the
world-state modules in memory, so deploying `world_state` alone put a new search
rule on the rover and left the old one answering. The component restarts the
daemon now.

The console draws the rest of it. A placed thing is a disc on the map, as big as
the fix that placed it is uncertain, so a thing the rover is sure about and a
thing it has merely pointed at twice look different at a glance. Its position is
on the row and in the detail; an entity with no position says it has been seen
from one place and needs two. And the resolver's own sentence about why an
observation belongs to a thing is kept on the observation rather than returned
once and lost, because the question a person asks of an identity is not what it
decided but why it thought that was the same chair.

**Unproven on hardware:** the disc. Nothing has been placed on the rover, so
nothing has ever drawn one. It needs the drive.

### Phase 4 as planned

- Text query embeds with SigLIP2 and ranks stored vectors by cosine. Brute force
  over a few hundred vectors is a numpy dot product.
- Calibrate a "nothing matches" threshold against real crops.
- Extend the popup: placement and its uncertainty, the two looks that produced
  it, the association reason, and the search box.
- The label is derived from the stored vector at display time, so the word list
  can change without reprocessing.

**Done when** "find the spray bottle" returns the spray bottle from the console,
and a bad match can be explained from the popup without reading the database.

---

## Acceptance criteria before semantic frontier selection

On the real rover, all of them:

- [ ] an identical-frame replay reuses existing identifiers rather than making a
      second copy — *not attempted since identity stopped coming from the model;
      the old result was about Cosmos and says nothing about this*;
- [ ] the same static object seen from materially different positions keeps one
      identifier — *half proven: one television from seven looks across three
      places, but nine duplicates remain, so it holds for some things and fails
      for others*;
- [ ] **two visually near-identical chairs in different places keep different
      identifiers** — *the drive kept two armchairs as two, which is right, but
      while duplicates exist this cannot be told apart from the duplicate bug:
      keeping them separate is what a rover that cannot merge does anyway*;
- [ ] metric positions for a repeatedly observed static object agree within a
      documented tolerance — *no tolerance documented and none measured; wants
      the same object placed on two separate drives*;
- [ ] a high appearance score cannot override clearly incompatible placement —
      *holds in the selftests; never exercised on the rover, because no case
      arose where appearance and placement disagreed*;
- [ ] uncertain cases can stay `AMBIGUOUS` without creating or merging wrongly —
      *the drive produced a handful of them and none was wrong, which is
      encouraging rather than evidence*;
- [ ] observations, embeddings, placement evidence and association reasons are
      visible enough in the popup to explain a bad match — *the daemon supplies
      all of it and the page draws it, but **the page has never been rendered in
      a browser**: there is none in this repository's test loop, and until the
      drive there was nothing placed to draw*;
- [x] world state survives a restart without losing identity or placement
      evidence — *23 entities came through a daemon restart with every position,
      uncertainty and all five exemplars each intact*;
- [x] a semantic clear still does not touch SLAM or Nav2 state — *cleared, then
      drove three legs on the same map with the pose still trusted*;
- [x] a map-session change does not silently compare stale coordinates — *proven
      twice and in both directions: observations from session 8 were excluded
      from session 9's resolving, and when the map was cleared after the drive
      its 23 placements were orphaned rather than compared against the new one*;
- [ ] none of it destabilises lidar, SLAM, Nav2, STOP, camera ownership or the
      Qwen voice path — *the drive exercised lidar, SLAM, Nav2 and driving for
      twenty minutes with no trouble; STOP and the voice path were not touched*.

### The validation drive, 2026-09-02

Three places about 1.3, 0.9 and 2.1 m apart, six headings surveyed from each with
the gimbal centred, 206 observations. **23 things placed, and the geometry works.**
What it found:

| | |
|---|---|
| the camera was blind | Auto-exposure had wound up for a dark room and never came back down, so every frame was white and the first attempt at the drive recorded regions that were not there — a spray bottle, a rug, a fan, in a room holding two armchairs. In "Aperture Priority Mode" this camera saturates regardless of the exposure time it reports; forcing manual at the shortest setting fixed it, 0 regions to 30, and it has not recurred. Judged a one-off and not being pursued. **The lesson is the one worth keeping: a blank frame reads exactly like an empty room.** "0 of 0 regions kept" and "nothing to place" are both things the rover says when it is working perfectly, so a whole drive was recorded off white frames before anyone looked at one. |
| fixes landed on the lens | All six placements of the first attempt sat 0.13–0.59 m from the camera that saw them. Two rays pointing inward from two nearby places cross in the gap between them at a healthy parallax off a healthy baseline, so neither guard caught it — and nudging a bearing by 1.5° barely moves a point that close, so the phantom reported two centimetres of uncertainty and outranked every real thing in the room. `MIN_RANGE_M` is now 0.75. |
| things were placed twice | Four televisions, two of them 8 cm apart, and three people where there was one. An entity is stored as a point but a television is a metre wide, so two looks from different sides of it centre on different parts of it and the match was refused; those looks then paired with each other into a second television, because the list of known things is read once before the pairing pass runs. The match tolerance now carries the thing's own width, and a thing created while pairing is offered to everything still waiting. On the same recording: 37 entities to 23, 22 duplicates to 9. |

**The open problem is the nine duplicates that remain**, and they are not all the
same thing. Two armchairs and three wooden chairs may well be two armchairs and
three wooden chairs. Two windows 0.4 m apart are one patio door that the region
finder split. Three people, metres apart, are one person who moved — and a person
is in `MOVABLE`, so placement should not be identity for them at all. One of the
three is a fix at 14° of parallax reporting ±1.42 m, which is barely a fix.

### What is left

**Nothing merges.** Every fix so far prevents duplicates at the moment of
creation -- a minimum range, a width-aware match tolerance, offering each new
thing to whatever is still waiting. Once two entities exist for one television,
nothing in the system ever reconciles them, and that is a missing mechanism
rather than a threshold to tune. Two windows 40 cm apart are one patio door the
region finder cut in half; a rule that merges same-label entities closer together
than `SAME_PLACE_M` would take them, and would take whatever the next drive
throws up too.

**A person should never have been placed.** `MOVABLE` currently only asks
appearance to agree before creating a thing; it does not stop position from
becoming that thing's identity. Three people metres apart are one person who
walked between looks, and for something that moves, a crossing of two bearings
taken minutes apart means nothing. Changing what movable means during pairing
removes a third of the duplicates on its own.

**The console has never been looked at.** The placement disc, the uncertainty
circle and the association reasons are written and deployed and no one has seen
them render. A viewer that silently fails is worth nothing, and it is the only
way anyone will ever explain a bad match.

Two smaller things fall out of the same recording: `'a floor'` and `'a wall'` are
being tracked as entities when they are surfaces rather than things, and dropping
them from the vocabulary removes two more duplicates and a good deal of noise.

None of it is a reason to keep the rover parked, and all of it except the drive
for the metric-agreement criterion can be settled without moving it.

---

## What not to do

```text
model-selected frontiers, or any model influence over movement
navigate_to(object:*) before the criteria above pass
continuous video-rate inference
model-generated metric coordinates
aggressive automatic merging
a vector database, a knowledge graph, an ontology framework
a new SLAM or navigation stack
MPPI or RTAB-Map, in any role
```

The task stays narrow:

> **Turn regions in a frame into stable, map-anchored, searchable entities,
> including telling apart two objects that look the same because they are in
> different places.**

If it works, the world state becomes a credible foundation for semantic
exploration. If it does not, the failure stays inside the perception and identity
layer and the navigation stack is untouched.
