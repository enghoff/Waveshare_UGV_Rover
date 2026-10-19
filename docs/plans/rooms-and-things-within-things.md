# Rooms, and things on, in and part of other things

Status: proposed 2026-10-19 at the owner's question; nothing built. The research
behind it is below. A first offline try split the map of 2026-10-15 into rooms
in under a second, and did not reproduce the owner's three rooms at any width
([2026-10-19](../progress/2026-10-19-rooms-from-the-map.md)). It proposes
[R-WS-20](../requirements/world-state.md#r-ws-20) and
[R-WS-21](../requirements/world-state.md#r-ws-21), and rests on
[R-WS-17](../requirements/world-state.md#r-ws-17) and
[R-WS-18](../requirements/world-state.md#r-ws-18), which are not settled. The
first owner decision is naming the rooms on a draft; the second, whether
pictures go to a hosted model at night, is already open in
[perception that keeps what it learns](perception-that-keeps-what-it-learns.md).

## The short answer

**Yes, in three levels of very different difficulty, and in that order.** The
rover keeps one flat list of things, each a place on the map with a spread and
a height. It has no rooms, no idea that the throw lies on the armchair or the
books stand on the shelf, and nothing about what a painting shows.

1. **Rooms are routine.** Every robot vacuum with a lidar splits its map into
   rooms from the map's geometry, and every one of them has the owner name the
   rooms and correct the split. The rover's own map gives such a draft in under
   a second. A thing's room is then a matter of where it stands. The work is in
   keeping the rooms attached to the map as navigation replaces it.
2. **What stands on or in what is feasible as a claim, not as a fact.** Geometry
   proposes the pair: the throw's height and place fall within the armchair's,
   or its outline sits inside the armchair's in one picture at the same depth. A
   describing model then confirms the pair from the full-resolution picture. Nobody
   has published how well this works with sparse depth on a real robot, and both
   halves are weak alone, so it needs its own benchmark on the rover's pictures
   before anything reads it.
3. **What a thing shows or holds is a description, not structure.** A painting's
   subject, or what is on the shelf, is written as text from the best few views,
   by a model, with its evidence. Keeping the shelf's books as records of their
   own, each tracked, is beyond what this rover's pictures support, and beyond
   what the published systems do reliably even with dense 3D.

**Nothing published can be adopted whole.** The systems that build a full
hierarchy (Hydra, Clio, ConceptGraphs, HOV-SG and their successors) start from a
dense 3D reconstruction from depth over the whole view, and most need a desktop
GPU and a hosted language model. The rover has a 2D lidar map, depth over the
middle quarter of a camera that turns, and 2.5-3.5 GB of shared memory to spare
while it drives. Pieces of them can be borrowed: the rooms step, how rooms are
named from pictures, how views are chosen for describing, and the rule that a
relation is recomputed when either end moves.

## What stands in the way today

- **The region finder throws away a region that sits mostly inside another.**
  Two boxes overlapping by more than four fifths of the smaller one keep only
  the higher-scoring box (`world_state/perceive.py`, `_suppress`). That rule is
  deliberate and right for what it was for: before it, 57% of regions were a
  part of another, a cushion beside its sofa, and the rover recorded two things
  where there was one. It also means the throw on the armchair is thrown away
  when the picture is taken. The region model itself seldom proposes nested
  regions (one in 237 over 38 frames), so keeping them takes more than undoing
  the rule.
- **Small things seldom become regions.** A picture keeps at most twelve
  regions, largest first, none under 0.4% of the frame, at 640x480 through a
  fisheye; at 3 m a 10 cm thing is about ten pixels across. The books on a shelf
  are below all three.
- **A record is a point with a spread and a height, not a shape.** That is
  enough to say which room a thing is in. It is too coarse to tell "on the
  table" from "beside it" for anything small: a thing behind other things has
  been placed 0.88 m out.
- **Identity is not settled.** A relation between two records is only as good as
  both records, and one record has held both armchairs
  ([R-WS-17](../requirements/world-state.md#r-ws-17),
  [R-WS-18](../requirements/world-state.md#r-ws-18);
  [the armchairs](../decisions/the-two-armchairs-cannot-be-told-apart.md)).
- **No meaning is stored at all**, by
  [R-WS-7](../requirements/world-state.md#r-ws-7), and the voice model cannot
  write to the memory ([R-CTL-4](../requirements/control.md#r-ctl-4)). The shape
  of a claim that respects both is already designed
  ([the design](autonomous-curiosity-design.md#evidence-and-belief-remain-different-things),
  with `on`, `inside` and `part_of` among its relationships), and so is who
  writes claims and when
  ([step 4 of the perception plan](perception-that-keeps-what-it-learns.md#4-claims-meaning-written-down-with-its-evidence)).
  Nothing writes one yet.

## What the field has learned

A survey on 2026-10-19 of academic work, commercial products and open-source
projects, done under the rule in [AGENTS.md](../../AGENTS.md#survey-the-field-before-choosing-a-route).
Most numbers come from simulated or curated scans (Replica, HM3D, Matterport,
3RScan); the real-robot ones are said to be.

### Rooms

**Splitting a 2D map into rooms is decades old and works where there are
doorways.** On 20 furnished office maps, Bormann et al. measured Voronoi
segmentation at 86.6% recall and 94.5% precision in about 12 s, and
morphological segmentation at 84.6% and 90.5% in about 1 s; Voronoi
over-segments corridors. Their `ipa_room_segmentation` is ROS 1 and licensed for
non-commercial use only. Hydra shrinks the free space by 0.45-1.2 m, picks the
room count that holds over the most shrink widths and floods back; its authors
say it would not work in an open floor plan. OccuSG (2026) splits a 2D free-space
layer in 2.5 ms an update and keeps a room's identity across updates by
overlap: a new region inherits the old room it overlaps by more than 0.2 of
their union, if its centre moved under 1.5 m and its area changed less than
fourfold. On twelve Matterport homes it found 38% of rooms at 52% precision,
against Hydra's 15% at 70%, so even the best map-only method misses most rooms
in real homes. Open-plan rooms are the failure every source names: HOV-SG,
Hydra and Matterport's own support pages.

**Every home robot that names rooms has a person do it.** iRobot splits the map
automatically and the owner renames, merges and splits; its patent carries the
labels onto each new map and asks the owner about conflicts. Roborock, Ecovacs
and Dreame split and the owner names. Amazon's Astro learns rooms on a guided
tour, with boundaries and names edited in the app. Apple's RoomPlan and
Matterport type rooms automatically, and both keep the label editable;
Matterport says open-plan spaces segment poorly. Roborock's claim that the S7
MaxV recognises bedrooms and living rooms from their furniture comes with no
figures.

**Naming rooms from what is in them is mediocre; from pictures, better.** Language
models given the true list of objects in a room named it 27-53% of the time
over 23 room types, and 65-69% fine-tuned; Hydra's own object-based naming
scored 27-48%. HOV-SG instead votes ten CLIP views of each room against room
words: 73.9% (84.1% leniently) in simulation, and on a real Spot it found the
right room 55.6% of the time. OrionNav, the nearest published system to this
rover (SLAM Toolbox on a 2D lidar, an Orin Nano for navigation, an AGX Orin for
perception), groups objects into rooms and has an on-board language model name
each group; 85 of its 96 real-robot trials succeeded, and it gives no accuracy
for the names. For one flat with four or five rooms and an owner who lives in
it, a person naming each room once beats every one of these.

### What stands on, in or is part of what

**Two products ship nesting, both computed once per scan from geometry.** Apple's
RoomPlan gives each object a parent: a sink's is the cabinet it sits in, a
chair's can be the table. Meta's Quest makes a box stacked on another its child
and a door the child of its wall. Neither updates the relation afterwards, and
nothing commercial keeps small objects on furniture as things.

**Inferring support from pictures is hard, and segmentation is most of why.** From
one RGB-D picture, Silberman et al. found what supports what 73-76% of the time
given hand-drawn regions and about 55% with automatic ones. On 3D
reconstructions with instance labels (3DSSG, 40 relations including "standing
on" and "lying on"), supervised models reach a recall at three of 0.89-0.98 and
an open-vocabulary one 0.63; FROSS does it at 144 frames a second under TensorRT
but recalls 22-27% of relations.

**Vision-language models judge geometry poorly cold, and well when shown the
regions and depth.** The best of 18 models scored about 56% on What'sUp's
spatial relations, against 99% for people; GPT-4V got 63% of above and below
right. SpatialRGPT, a 7B model prompted with the two regions and depth, got 99%
on its own benchmark. ConceptGraphs proposes pairs from 3D box overlap and has
a language model label each; people judged 88% of its edges right on Replica,
and nobody measured how many it missed. Its prompt asks whether one thing is
"commonly placed on top of" another, which is a prior, not an observation: it
would put any throw near an armchair on it. OpenIN proposes carried-on-carrier
pairs from the carrier's size, the closest distance and overlap in each axis,
which is closer to what the rover can measure. BBQ found that giving a language
model the distances between objects helped it pick the right one, and giving it
left, right and above made it worse.

### What a thing shows

**Describing a thing from one view is unstable, and describing it from several
needs reconciling.** ConceptGraphs found LLaVA's captions of the same object from
different views incoherent and had GPT-4 merge them. A 2026 captioner with a
memory of the object made its captions only 7.4% more alike; RAG-3DSG treats
disagreement between views as uncertainty. BBQ picks views to describe by
grouping the viewpoints into five and taking the view with the largest mask in
each. NVIDIA's Describe Anything describes a marked region in detail. Parts are
harder still: OpenFunGraph, on laser-scanned rooms, found 44% of parts such as
handles and knobs at three tries. Nothing commercial describes what a painting
shows.

### Things that move, and how the hierarchy is stored

**Commercial products keep only the latest state.** Amazon's patent sorts
obstacles into static, variable and dynamic and keeps the last observed state;
iRobot carries labels onto new maps with the owner confirming. The richest
design found, X Development's patent, gives each object place probabilities that
decay with time, room priors, links to other objects ("the mug is often on that
desk") and how often it moves. It was never a product.

**In research, a move recomputes only what it touches.** DovSG, after a change,
recomputes only the moved object's relations and its parents' and children's,
and on a real robot kept 84-89% of its graph right against 46-55% for GPT-4o.
PBD-AG (2026) keeps support relations with a reliability weight and refuses to
delete what was merely hidden, in simulation. Khronos calls a thing absent only
when 60% of the rays over five seconds pass through its place.

**The data structure is the easy part.** Hydra's Spark-DSG is a layered graph whose
nodes carry features and times, saved as JSON; it is worth adopting only with
Hydra's tools. At this flat's scale, hundreds of records, a table of rooms and a
table of claims beside the store does the same job, and language models answer
better from pictures with a graph than from the graph written out as text
([the perception plan's survey](perception-that-keeps-what-it-learns.md#what-the-field-has-learned)).

**Why the hierarchical systems cannot run here whole.** ConceptGraphs was measured
at 0.16 frames a second and 8 GB of GPU memory, HOV-SG at 0.03 and 24 GB; BBQ
needs over 10 GB to map and 16 GB for its model; Clio ran on a laptop RTX 4090.
Hydra runs on a Xavier NX but builds its own dense distance field from depth over
the whole view, and its ROS 2 build is tested only on Ubuntu's own Jazzy, where
the rover's comes from RoboStack. USS-Nav (2026) builds regions and objects from
an occupancy grid with YOLOE under TensorRT at 15 Hz, on an Orin NX with 16 GB,
and its code is not out.

## The proposal

Three additions beside the store, none replacing what is there, each built and
tested offline before anything on the rover reads it. Navigation keeps its
authority, nothing here moves the rover, and no room, relation or description
ever decides which record a look belongs to.

### 1. Rooms, named by the owner

- **A draft from the map.** `experiments/rooms/split_rooms.py` splits the map at
  doorway width. It gives regions, not the owner's rooms
  ([2026-10-19](../progress/2026-10-19-rooms-from-the-map.md)): the kitchen
  always joins the living room, and the bedroom holds together only at one
  width. Before any rover code, try the Voronoi split and Hydra's
  most-stable-count rule on the same map, with the width chosen before the
  answer is seen.
- **The owner corrects and names it once**, shown the draft as a picture with
  the regions numbered: which regions are one room, where an open-plan room is
  to be divided, and each room's name. That is what every product does, and the
  only naming that cannot drift. A suggestion from the rover's own pictures
  (SigLIP2 vectors of each region's looks voted against room words, HOV-SG's
  method) is not worth building for four or five rooms, and under
  [R-WS-7](../requirements/world-state.md#r-ws-7) it could only ever be a
  suggestion.
- **Rooms are kept beside the store, on the map they were drawn on**: an outline,
  a name, who named it and when. A thing's room is where its place falls, with
  0.5 m to spare. A thing on a wall between two rooms takes the room it was seen
  from, which the camera's pose says.
- **A replaced map gets its rooms back without the owner** where it can: the
  rooms are redrawn on the new map and matched to the old ones after the
  transform `world_state/reanchor.py` finds, by OccuSG's overlap rule. A room
  that matches nothing is shown to the owner rather than guessed.
- **What reads them**: `find_thing` says "in the bedroom", the console's world
  popup groups by room, an autonomous run can be sent to look in one room, and
  the M5 and multi-day scoring stop carrying a hand-drawn room rule
  (`experiments/m5_changes/score_session.py`, `room_of`).

**Test, declared now:** on the map in use, every thing with an owner-given room
(the eleven taped targets and the staged things of the M5 manifests) is put in
that room by its record's place, at least 19 times in 20, with a wall-mounted
thing judged by the side it was seen from. Then the same again after the next
time navigation replaces the map, with no help from the owner. The bars are the
owner's to agree.

### 2. What stands on or in what, as claims

- **Keep what the region finder already proposes inside another region.** The
  rule that drops it stays for filing, because it is what keeps a cushion from
  becoming a second sofa. The dropped box and outline are kept on the look as
  lying inside the region that won, cost nothing to keep, are never filed as
  things, and are embedded at night if at all, so no look is slowed. First
  measure, on
  stored pictures, how many there are and what they are: the region model
  seldom proposes them, and if it proposes almost none, this step needs a
  region model asked for parts, such as a prompted segmenter, and is costed
  again.
- **Propose a pair from geometry.** From records: one thing's height sits at
  another's top and its place falls inside the other's spread. From one look:
  its outline lies inside the other's and the depth under its lower edge agrees
  with the other's within a few centimetres, which needs both inside the depth
  camera's middle 65 by 40 degrees. OpenIN's carrier rules are the starting
  point.
- **Confirm the pair from the picture.** A describing model is shown the
  full-resolution still with both outlines drawn and asked one question, "is
  the marked thing resting on, inside or part of the other?", with "unsure" an
  answer. Never asked whether such things are commonly found together.
- **Store it as a claim** in the shape the design already gives: the two
  records, the relation, the looks and pictures it rests on, the method and
  model, how sure, and when it was first and last seen to hold
  ([R-WS-21](../requirements/world-state.md#r-ws-21)). When either thing is seen
  to move, the claim is closed at the move, not carried over; a new look
  proposes it afresh, as DovSG recomputes only what a move touched.

**Test, declared now:** on the rover's own full-resolution stills since
2026-10-12, at least 40 pairs where one thing rests on or in another and 40
nearby pairs where it does not, labelled from the photographs by the coding
agent and by the owner for what photographs cannot settle, and frozen before
anything is scored. Of the relations it claims, at least 9 in 10 are right;
how many it claims is reported, and an "unsure" is not a claim. Asked twice of
the same picture, it gives the same answer at least 9 times in 10, which is
where the last local model failed
([cosmos-reason2.md](../decisions/cosmos-reason2.md)).

### 3. What a thing shows or holds, as descriptions

- **A description per thing from its best views**: viewpoints grouped and the
  largest view taken from each (BBQ's choice), at full resolution, described
  by the same model at night, with each view's answer kept and one summary
  written from them. Views that disagree make the summary "unsure", not a
  vote. This is the attribute claim the curiosity plan already defers to a
  benchmarked describing model
  ([Phase 4](autonomous-curiosity.md#attribute-claims-later)).
- **What is on a shelf is part of the shelf's description**, "three books and a
  plant", not records of their own. Small things as tracked records wait for
  the dense field of
  [the perception plan](perception-that-keeps-what-it-learns.md#2-the-field-index-all-of-every-picture),
  which reads things at the granularity a question asks for.
- **The rover can already find the cow painting by words**, because text search
  compares a phrase with the stored pictures. What this adds is being able to
  say what the painting shows, and to say it the same way twice.

**Test, declared now:** on 20 things with something to describe, chosen and
frozen before describing (paintings, shelves, the table, the bed), the summary
is judged right from the photographs at least 17 times in 20, and two
descriptions made from different views of one thing agree at least as often.

### What is not proposed

Adopting Hydra, ConceptGraphs, HOV-SG or their successors whole, for the reasons
above; a room vocabulary or class head that names rooms; relations or
descriptions used to join, split or place a record; and the voice model writing
any of it, which stays the owner's decision under
[R-CTL-4](../requirements/control.md#r-ctl-4). When a route is chosen, what was
ruled out goes into a decision record.

## References

Opened on 2026-10-19 by the survey behind this plan. Preprints from 2026 are not
yet peer-reviewed.

Rooms:
Bormann et al., 2016, room segmentation survey, https://arxiv.org/abs/1709.09899;
ipa_room_segmentation, https://github.com/ipa320/ipa_coverage_planning;
Hughes et al., 2022 and 2024, Hydra, https://arxiv.org/abs/2201.13360, https://arxiv.org/abs/2305.07154, https://github.com/MIT-SPARK/Hydra;
Cueto Zumaya et al., 2026, OccuSG, https://arxiv.org/abs/2606.13727, https://github.com/crcz25/OccuSG;
Werby et al., 2024, HOV-SG, https://arxiv.org/abs/2403.17846;
Chen et al., 2022, room classification from object lists, https://arxiv.org/abs/2209.05629;
Devarakonda et al., 2024, OrionNav, https://arxiv.org/abs/2410.06239;
iRobot, US11571813, https://patents.google.com/patent/US11571813;
Amazon Science, 2021, Astro's Intelligent Motion, https://amazon.science/blog/astros-intelligent-motion-brings-state-of-the-art-navigation-to-the-home;
Apple, RoomPlan sections and parents, https://developer.apple.com/documentation/roomplan/capturedroom/section, https://developer.apple.com/documentation/roomplan/capturedroom/object/parentidentifier;
Matterport, 2024, automated room labelling, https://matterport.com/news/matterport-announces-the-next-generation-of-ai-powered-real-estate-insights;
Roborock, S7 MaxV, https://us.roborock.com/pages/roborock-s7-maxv.

Relations:
Silberman et al., 2012, support inference from RGB-D, https://cs.nyu.edu/~fergus/datasets/indoor_seg_support.pdf;
Wald et al., 2020, 3DSSG, https://arxiv.org/abs/2004.03967;
Koch et al., 2024, Open3DSG, https://arxiv.org/abs/2402.12259;
Hou et al., 2025, FROSS, https://arxiv.org/abs/2507.19993;
Kamath et al., 2023, What'sUp, https://arxiv.org/abs/2310.19785;
Cheng et al., 2024, SpatialRGPT, https://arxiv.org/abs/2406.01584;
Gu et al., 2024, ConceptGraphs, https://arxiv.org/abs/2309.16650, https://github.com/concept-graphs/concept-graphs;
Tang et al., 2025, OpenIN, https://arxiv.org/abs/2501.04279;
Linok et al., 2025, Beyond Bare Queries, https://arxiv.org/abs/2406.07113;
Meta, MR Utility Kit, https://developers.meta.com/horizon/documentation/unreal/unreal-mr-utility-kit-features/.

Descriptions and parts:
Lian et al., 2025, Describe Anything, https://arxiv.org/abs/2504.16072;
captions with object memory, 2026, https://arxiv.org/abs/2603.24257;
RAG-3DSG, 2026, https://arxiv.org/abs/2601.10168;
Zhang et al., 2025, OpenFunGraph, https://arxiv.org/abs/2503.19199.

Change and storage:
Yan et al., 2024, DovSG, https://arxiv.org/abs/2410.11989;
Bao et al., 2026, PBD-AG, https://arxiv.org/abs/2608.10449;
Schmid et al., 2024, Khronos, https://arxiv.org/abs/2402.13817;
X Development, US10754343, https://patents.google.com/patent/US10754343B2/en;
Amazon, US11409296, https://patents.google.com/patent/US11409296;
MIT-SPARK, Spark-DSG, https://github.com/MIT-SPARK/Spark-DSG;
compute of ConceptGraphs and HOV-SG, https://arxiv.org/abs/2412.01539;
USS-Nav, 2026, https://arxiv.org/abs/2602.00708.
