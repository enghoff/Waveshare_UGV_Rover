# What brains and video models suggest for how the rover sees, remembers and finds its way

Status: a design, proposed 2026-10-12 at the owner's request, who asked to go
back to first principles and set hardware aside. Nothing in it is built. It is
the longer view behind [the perception plan](perception-that-keeps-what-it-learns.md),
which is what can be built on today's hardware. That plan's layers are the first
steps of this design, and its research sections already cover dense 3D feature
maps, persistence filters and robot world models, which this document links
rather than repeats. Five of the ideas below are cheap enough to test on the
recordings the rover already holds; they come at the end, with their tests
fixed in advance.

## The short answer

**An animal does not describe what it sees; it predicts it, and attends to,
learns from and remembers mostly what the prediction got wrong.** It carries a
model of its world and of its own body, samples the world with a small sharp
fovea where that model is unsure, anchors its sense of place on what does not
move, and treats an object as something that persists along a path through
space and time. Sleep replays the day's experience into slower, general
knowledge, and planning is that same model run forwards in imagination.

The rover does nearly the opposite. It describes every picture from scratch,
treats each region as a fresh appearance to be matched, decides identity look by
look from where bearings cross and how alike regions look, with no notion of a
path between sightings, anchors its pose on whatever the lidar sees, furniture
included, and has no expectation of what it should see next. So nothing about a
picture is surprising to it, and a moved armchair is not an event but a puzzle
of mismatched records.

**Video models arrived at physics by the same route the brain uses, prediction,
and stopped where the brain does not.** Trained only to predict what comes
next, they learned depth, rough 3D, occlusion and object permanence. When the
prediction is made in a learned representation rather than in pixels, the model
is surprised by impossible events much as an infant is, from as little as a week
of video (Garrido et al., 2025). But they copy the nearest case they have seen
rather than learn laws, they break conservation of mass and momentum about 40%
of the time, and they forget a place within about a minute unless their memory
is pinned to positions in 3D. For a robot that lives in one house, the first
weakness hardly matters: its own house is the one world it will ever need to
interpolate, and what falls outside it is exactly the surprise it should notice.

Six principles follow, each a change in how the rover is built rather than a
better model:

1. **Seeing is predicting.** Keep a model of the house that predicts each view
   and scan from the rover's pose, and let the error drive everything else:
   what is new, what changed, where to look, what to learn, what to store.
2. **Look, rather than store everything.** The eye is sharp over about one
   degree and moves two or three times a second; the brain keeps little detail
   and looks again instead. The gimbal camera is an eye, and memory's job is to
   hold expectations and where to look, as well as the evidence.
3. **Anchor on what does not move, and learn which cues those are.** Animals
   stop using landmarks they learn are unstable. The rover's pose went 1.35 m
   wrong in the room where the armchairs had moved, its scan matcher fitting the
   furniture as if it were walls.
4. **An object is a path before it is a look.** Infants keep objects apart by
   where they are and how they moved before they use what they look like. An
   object does not vanish and reappear somewhere else; it is moved, usually by
   somebody.
5. **Two memories, joined in sleep.** A fast store of particular episodes, kept
   apart by their place and time, and a slow store of general knowledge, taught
   by replaying the episodes at rest.
6. **Plan by imagining.** The same model that predicts the next view predicts
   the view from somewhere else, so where to look and where to go are chosen by
   what the rover expects to learn there.

And meaning comes from what things do before it comes from words: a thing that
blocks the way, moves every day or stalls the wheels is understood by the rover
in terms it can use, and a name, from the owner or a language model, attaches
to that.

## How an animal does it, from the eye inwards

### The eye samples, and the world is its own memory

Only the foveola, about one degree across and less than a hundredth of a percent
of the visual field, sees sharply (Intoy, Mostofi & Rucci, 2021). The eye moves
to point it two or three times a second (Poletti, 2023), and the brain makes of
those glimpses a world that feels complete and detailed but is neither. Change
blindness shows how little is kept: with a brief blank between two versions of
a scene, people miss large changes, and the authors concluded that observers
"never form a complete, detailed representation" (Rensink, O'Regan & Clark,
1997). Only half the pedestrians giving directions noticed that the person they
were talking to had been swapped for someone else (Simons & Levin, 1998).
O'Regan argued that the world serves as its own external memory: there is no
need to store what can be looked at again (O'Regan, 1992; O'Regan & Noë, 2001).
Visual working memory holds about four items (Luck & Vogel, 1997). People
copying a pattern of blocks look back at the model just before each piece is
needed rather than memorise it, using fixations as pointers into the world
(Ballard, Hayhoe & Pelz, 1995; Ballard et al., 1997). What is held is the gist,
taken from the whole image without dividing it into objects (Oliva & Torralba,
2006): a scene shown for a tenth of a second is understood, though it takes
longer to be remembered (Potter, 1976).

*The rover instead* records whatever the camera points at and tries to keep a
complete description. Its gimbal is a fovea it rarely aims on purpose: most
looks are taken wherever the camera happens to point. *Copy*: a periphery for
gist and salience (the fisheye, low resolution), a fovea for detail (a
full-resolution crop or the depth camera, aimed), and a policy that points the
fovea where the model is least sure or most surprised, with inhibition of
return so that it does not keep looking at the same thing. Memory holds the
expectation and the pointer, "the red bag should be by the window; look there
from the doorway", as well as the evidence.

### Two streams: what a thing is, and where and how to act

Visual cortex divides into a ventral stream that recognises what things are and
a dorsal stream that locates them and guides action (Mishkin, Ungerleider &
Macko, 1983; Goodale & Milner, 1992). A patient who could not say the shape,
size or orientation of an object still reached for it and gripped it with
"strikingly accurate" movements (Goodale et al., 1991): action does not wait on
recognition. Scene-selective areas code the kind of place and its layout. One
responds as strongly to an empty room as to a furnished one, and more than
twice as strongly as to furniture without a room (Epstein & Kanwisher, 1998).
Another, the occipital place area, codes where in a scene one can walk, even
when the task is something else entirely (Bonner & Epstein, 2017), and
disrupting it impairs memory for places defined by walls but not by landmarks
(Julian et al., 2016). A third, the retrosplenial cortex, responds only to the
most permanent landmarks, and people who navigate poorly are worse at telling
which landmarks are permanent (Auger, Mullally & Maguire, 2012). The brain sees
rooms, walls and paths before it sees objects. Gibson called the possibilities
for action that a scene offers affordances, and much of what an animal needs,
such as the time before it hits a surface, can be read straight off how fast
the image expands, without knowing the distance (Gibson, 1979; Lee, 1976). The
two streams talk all the time: where something is decides where to look, and
what it is decides what can be done with it.

*The rover* has a good dorsal stream, the lidar, the map and Nav2, and a weak
ventral one, the world state. Its rule that every obstacle in motion is handled
the same, whatever it is, is the dorsal stream's way, and the patient's: right
for safety, and nothing here changes it. But the two barely talk: the world
state withholds a look's direction when navigation is unsure, and navigation
knows nothing of what the world state has seen. *Copy*: let each inform the other. Navigation's
map tells the camera where the stable walls and the changing places are, and the
camera tells navigation which obstacles are furniture that moves. Perceive
affordances for the rover directly: floor it can cross, gaps it fits through,
surfaces that stall its wheels.

### Seeing is predicting

In predictive coding, feedback carries predictions down the visual system and
the feedforward signal carries what they failed to predict (Rao & Ballard,
1999). In the primary visual cortex of a running mouse, many cells respond to
the mismatch between the visual motion it sees and the motion its own running
should produce (Keller, Bonhoeffer & Hübener, 2012). Friston generalised the
idea into active inference, in which action too serves to reduce surprise,
including by going to look (Friston, 2010). Before an eye movement, cells in
parietal cortex already respond to what the movement will bring into view,
driven by a copy of the motor command, the efference copy (Duhamel, Colby &
Goldberg, 1992). The cerebellum learns forward models that predict the
consequences of commands (Wolpert, Miall & Kawato, 1998); one's own touch is
less ticklish because its sensation is predicted and cancelled (Blakemore,
Wolpert & Frith, 1998). And prediction error is the teacher: dopamine neurons
signal it for reward (Schultz, Dayan & Montague, 1997), and the hippocampus
flags information "not already stored", which loops through the dopamine system
to strengthen what gets kept (Lisman & Grace, 2005).

*The rover* predicts nothing about what it will see. It does not predict the
consequence of its own movements either: told to turn 30 degrees it can turn 50,
and a picture's heading can be several degrees out
([R-NAV-9](../requirements/navigation.md#r-nav-9),
[world_state](../../world_state/README.md#what-to-expect-from-it)), and both
are found by bench measurement rather than noticed in use. *Copy*: a model of
the house that predicts each view and scan, so that change, novelty and
curiosity are all the same signal, the error; and forward models of the
rover's own body, wheels and gimbal, that predict how the scan and the picture
should move for each command, recalibrate from the difference all day, and flag
the day the difference is a fault.

### Objects are paths that persist

Adults track a handful of moving things as "object files", bound to a place and
a path, not to a description: seeing something first primes it later only when
both glimpses belong to the same object, even after it has moved (Kahneman,
Treisman & Gibbs, 1992). Infants come with core knowledge about objects: they
move as connected wholes, separately from each other, keep their size and
shape, and act on each other only on contact (Spelke, 1990; Spelke & Kinzler,
2007). Infants look longer at events that break these rules, which is how the rules
are found. Ten-month-olds use where things are and how they moved to tell one
object from two, before they use colour or kind; kind comes in at about twelve
months, with the first nouns (Xu & Carey, 1996).

*The rover* decides identity look by look: geometry admits the candidates, and
appearance chooses among them. Nothing carries a path from one sighting to the
next, and no rule says that a thing seen gone here and a similar thing seen new
there is one thing moved. On the M5
shakedown the red bean bag became two records, 917 and 920, across its move;
the charger armchair's history is spread over at least seven records; and one
record, 838, holds both armchairs
([the audit](../../experiments/perception_bench/shakedown_audit.md)).
*Copy*: object files, whose identity is carried by a continuous path where the
rover saw one and by an explanation where it did not. A change is explained as
the fewest events that account for it, a move, an arrival, a departure, under
the rules that things do not vanish, do not pass through walls, and move when
somebody is there to move them. Appearance is evidence for the explanation, not
the identity itself.

### Physics is a simulation run in the head

People judge whether a tower of blocks will fall as a noisy physics simulation
would, and errors that look like biases are what a probabilistic simulation
with uncertain inputs produces (Battaglia, Hamrick & Tenenbaum, 2013). Ullman
and colleagues argue the mind runs something like a game engine: objects with
mass, support, containment and collisions, simulated approximately and quickly
(Ullman et al., 2017). The brain areas engaged by predicting physical outcomes
overlap those that plan actions (Fischer et al., 2016).

*The rover* has no physics at all beyond its own driving. *Copy*: a small,
explicit physics of the house, objects rest on something, things inside a
cupboard are still there when the door shuts, a person can carry a bean bag but
not a wardrobe, together with what the rover learns from its own body: the rug
stalls the wheels at a shallow angle, the threshold needs speed. This is the
part learned models get most wrong, so it stays explicit.

### The map: walls, places, and where things are relative to them

The hippocampal formation holds a map. Place cells fire at particular places
(O'Keefe & Dostrovsky, 1971); grid cells fire on a hexagonal lattice that spans
any room and gives a metric scaffold (Hafting et al., 2005); head-direction
cells hold a compass, turned by landmarks (Taube, Muller & Ranck, 1990);
boundary vector cells fire at a distance and direction from walls (Lever et
al., 2009); and object-vector cells fire at a distance and direction from objects,
whatever the object (Høydal et al., 2019). Some hippocampal cells fire where an
object used to be, after it has gone (Deshmukh & Knierim, 2013). Path
integration keeps the estimate going between sightings, and landmarks correct
it (McNaughton et al., 2006). Crucially, the landmarks used are the stable
ones. Rats disoriented before each session, so that a cue card seemed to move
unpredictably, came to give it much weaker control over their place and
head-direction cells (Knierim, Kudrimoti & McNaughton, 1995). Rats use
landmarks to find a goal only if the landmarks are stable (Biegler & Morris,
1993); once a rat sees a cue move, its place fields stop following it, though
trust built over days lingers (Jeffery, 1998). In a fly, the mapping from the
visual scene to its internal compass rewires within minutes (Fisher et al.,
2019). The Tolman-Eichenbaum Machine models the hippocampus as keeping the
structure of a space apart from its contents, so that structure learned in one
place is reused in the next; grid, place and object-vector cells emerge from it
(Whittington et al., 2020).

*The rover* anchors its pose on every lidar return, the armchairs included,
and on the shakedown that took it 1.35 m from the truth while it still called
the position trusted ([2026-10-12](../progress/2026-10-12-m5-shakedown.md)). Its
things are stored in coordinates of one map, which expire when the map changes.
Two things it already does point the same way: the change watch counts, cell by
cell, where the room was solid and where it was seen through, and today's
heading check trusts the far returns, mostly walls, over the near ones in a
room whose furniture moved
([2026-10-12](../progress/2026-10-12-heading-check-in-a-changed-room.md)).
*Copy*: a stability for every cell of the map, learned from how often it has
changed over weeks; localisation that leans on stable cells and treats furniture
as objects to be tracked rather than as walls; and things recorded relative to
the stable structure near them, "half a metre from the balcony wall", so that
they survive a map's rebuilding. Robots that localise this way do measurably
better among moved furniture ([below](#what-happened-when-robots-copied-the-brain)).
One caution from the rats: trust built over days lingers after a landmark is
seen to move, so stability must be able to fall quickly as well as rise
slowly.

### Two memories, joined in sleep

The brain learns fast and slow in separate systems: the hippocampus stores
particular episodes at once, and the neocortex learns general structure slowly,
taught by the hippocampus replaying episodes during rest and sleep, interleaved
so that new learning does not overwrite old (McClelland, McNaughton & O'Reilly,
1995; Kumaran, Hassabis & McClelland, 2016). Cells that fired together in the
day fire together again in sleep (Wilson & McNaughton, 1994), and suppressing
those replay events impairs memory (Girardeau et al., 2009); what gets replayed
at night is tagged while awake (Yang et al., 2024). Within the hippocampus, the
dentate gyrus keeps even slightly different experiences apart, and CA3 restores
a whole stored memory from a degraded cue (pattern separation and completion;
Leutgeb et al., 2007; Neunuebel & Knierim, 2014). A food-caching bird gives
each cache its own sparse "barcode", uncorrelated even for neighbouring caches
with similar place codes, which comes back when that cache is retrieved
(Chettih et al., 2024). That is the same trade-off the rover's identity work is
stuck in: join too readily and two armchairs become one record, separate too
readily and one armchair becomes seven. The brain separates by default, gives
each episode an identity of its own apart from its place, and completes when a
cue matches strongly. New knowledge that fits an existing schema is
consolidated after one trial rather than weeks (Tse et al., 2007). And a memory
recalled becomes changeable again before it is stored anew (Nader, Schafe &
LeDoux, 2000), which is how it is kept up to date.

*The rover* has one memory, the store, filled as looks arrive and changed
afterwards mainly by merges a person reviews. *Copy*: a fast episodic store of what was
surprising, kept apart by place and time; a slow store of general knowledge,
kinds of thing, how often they move, where they tend to be, what they afford;
and a nightly replay that teaches the second from the first. The perception
plan's night shift is this, and its small models trained on the night's claims
are the neocortex. Identity should separate by context and complete on
distinctive cues: the charger armchair's throw and boxes are the cue that would
have kept its history together.

### Planning is imagining

Before a rat runs to a remembered goal, its place cells fire in sequences that
start where it stands and sketch the path it is about to take, even between a
start and a goal never paired before (Pfeiffer & Foster, 2013). When a new
barrier appears, those sequences reroute round it quickly while the place cells
themselves stay put (Widloski & Foster, 2022). Grid and place cells also sweep
alternately left and right ahead of the animal several times a second, into
places it has never been, like an attention that samples the map (Vollan et
al., 2025). The hippocampus can be read as a predictive map that codes each
place by the places likely to follow it, the successor representation
(Stachenfeld, Botvinick & Gershman, 2017), and which memories get replayed can
be explained by how much each would improve the next choice (Mattar & Daw,
2018), though replay does not always predict the choice that follows (Gillespie
et al., 2021). Imagining the future and remembering the past use the same
machinery: people with hippocampal amnesia imagine scenes that fall apart
(Hassabis et al., 2007; Schacter & Addis, 2007).

*The rover* plans routes with Nav2 and chooses viewpoints by a deterministic
score, which is right for safety. *Copy*: let the model of the house imagine
the view from each candidate viewpoint before going, and choose looks by how
much the rover expects to learn there: where its predictions are poorest, where
things tend to change, where something has gone missing.

### Insects: a lot from very little

A desert ant integrates its path by sky compass and step counting, and returns
home along remembered views by moving so that what it sees feels familiar
(Cartwright & Collett, 1983; Baddeley et al., 2012); a fly holds its heading in
a ring of neurons (Seelig & Jayaraman, 2015). Robots built this way home to
within centimetres with a few kilobytes of memory, though they only retrace
routes they were taught (Dupeyroux et al., 2019; Gattaux et al., 2025).

*The rover* has one source of where it is on its map, the lidar matched against
it. *Copy*: a second,
independent opinion from the camera, whether the view looks like the view from
where the rover believes it is, cheap enough to run on every look. On the
shakedown, the living room seen from 1.35 m away would not have looked like
the living room seen from the believed pose.

### What happened when robots copied the brain

**Copy the computation, not the neurons.** The literal copies are impressive
and have not displaced engineering. RatSLAM, an attractor network of "pose
cells" with visual templates, mapped a 66 km suburb from one webcam and closed
loops carrying 1.2 km of odometry error, but its map is coherent rather than
measured (Milford & Wyeth, 2008). Grid cells emerged in a network trained to
path-integrate in simulated mazes (Banino et al., 2018), but across more than
11,000 trained networks they appeared only under particular hand-chosen designs
(Schaeffer, Khona & Fiete, 2022). Nothing bio-inspired that the research found
has beaten conventional lidar SLAM on a real indoor robot.

The principles, carried into ordinary engineering, are what work. Episodic
non-Markov localisation sorts each lidar return into permanent structure,
unmapped static things such as chairs, and moving things, and matches only the
first against the map; it ran for hundreds of kilometres on office robots
(Biswas & Veloso, 2017). Localising on a floor plan's walls plus recognised
objects took global localisation from 27% to 100% with furniture moved
(Zimmerman et al., 2023). A vehicle that saved a new "experience" of a place
whenever localisation failed was never lost after its sixteenth lap of a route
driven over three months (Churchill & Newman, 2013). Robots with explicit
object memories found things that had gone out of sight 19 times in 20, where
earlier systems without them found none (Huang et al., 2024). Frozen DINOv3
features, kept as several appearance prototypes per object, re-identified
indoor objects after the robot left and came back at 90% (Diaz-Pereda et al.,
2026). Khronos, which tracks a scene's changes over time, lists the rover's own
failure as its limitation: a moved chair comes out as one object gone and
another appeared (Schmid et al., 2024).

## What video models learned, and what they did not

**Prediction alone taught them geometry and permanence.** Probes read depth
straight out of an image diffusion model's internals, and editing those
internals changes the picture, so the depth is used, not incidental (Chen,
Viégas & Wattenberg, 2023). Video generators keep scenes roughly consistent in
3D as the camera moves and mostly keep objects that pass out of view. A model
trained only to predict missing parts of video in its own learned
representation, Meta's V-JEPA, tells possible events from impossible ones on
the IntPhys benchmark 98% of the time, object permanence and continuity
included, where a model predicting pixels and two large multimodal language
models score near chance (Garrido et al., 2025).

**They copy cases rather than learn laws.** Trained on simulated balls, a video
model's error inside the range of speeds it saw was close to the measurement
floor, and about ten times larger outside it, and more data did not close the
gap; it matched new cases to old ones by colour before shape (Kang et al.,
2025). Scored on real physical experiments against how well two real takes of
the same experiment agree, the best model reached 29.5%, and how realistic a
model's videos looked was unrelated to how physically correct they were
(Motamed et al., 2025); frontier models given one starting frame still score
26-32% on the benchmark's self-reported leaderboard in October 2026. They
break conservation of mass and of momentum about 40% of the time (Bansal et al.,
2025), and vision-language models score about 50% on physics questions against
96% for people (Chow et al., 2025).

**Their memory of a place lasts about a minute, unless it is pinned to 3D.**
Genie 3 stays consistent for a few minutes with a visual memory reaching back
about one minute (Google DeepMind, 2025); Minecraft world models rearrange the
landscape when the player turns round. Video world models that index past views
by the 3D surfaces they saw recover consistency when the camera returns: VMem's
surface-indexed memory beat choosing past views by recency, by camera distance
or by field of view (Li et al., 2025). Field-of-view selection fails, its
authors note, between interconnected indoor rooms, which is what a flat is (Yu
et al., 2025).

**Conditioning on actions turns a video model into a model of consequences,
and little labelled data is needed.** V-JEPA 2's action-conditioned version
learned from 62 hours of unlabelled robot video to plan pick-and-place, 80% and
65% successful, about 16 s an action (Assran et al., 2025). Dreamer 4 learned
from mostly unlabelled Minecraft video to obtain diamonds from offline data
alone (Hafner, Yan & Lillicrap, 2025). The rover's recordings already pair every
picture and scan with the wheel odometry, IMU and commands that moved it.

What this means for the rover:

- **Predict in a learned representation, not pixels**: that is where the
  physical intuitions appear, and it ignores the detail nothing can predict.
- **Condition on the rover's own actions**, which its recordings already carry.
- **Anchor the model's memory in a 3D map, and keep what changes apart from what
  does not**: the perception plan's field is that memory.
- **Train on its own house.** A model that only interpolates is at its best in a
  narrow world seen over and over. The rover's house is exactly that, and the
  rare events, a moved chair, a door, a bump, are what to keep and learn from.
- **Keep explicit rules and a checker** where learned models are weakest:
  things persist, walls are solid, the rover's own motion follows its
  kinematics. The model proposes; the map and the rules decide.

## A rover built on these principles

```text
  ┌──► act: drive, turn, point the gimbal
  │      │  efference copy: what this move should do to the scan and the picture
  │      ▼
  │    sense: the fovea where the model is unsure, the periphery for gist
  │      │
  │      ▼
  │    compare with what the model of the house predicted ──► error
  │                                                             │
  │       ┌─────────────────────────────────────────────────────┤
  │       ▼                          ▼                          ▼
  │    recalibrate the self-model   explain the change: a move,  keep the surprise
  │    (wheels, gimbal)             an arrival, a departure      as an episode
  │                                  │                           │
  │                                  ▼                           │ nightly replay
  │                        object files; a map of what           ▼
  │                        does not move                  slow knowledge: kinds,
  │                                  │                    rates, habits, affordances
  │                                  └──────────┬────────────────┘
  │                                             ▼
  │                         the model of the house, which imagines
  │                         the view from places not yet visited
  │                                             │
  └──────── choose where to look and go ◄───────┘
                         ▲
                         │  a language model names, explains and asks, from the
                         │  model's state and its evidence; it is not where seeing happens
```

Nine parts, each a principle above made concrete. None of them needs authority
over movement; the daemon and Nav2 keep that
([R-SAFE-4](../requirements/safety.md#r-safe-4)).

1. **An eye with a fovea.** The fisheye as periphery for gist and salience; a
   full-resolution crop or the depth camera, aimed by the gimbal, as fovea. A
   looking policy that points the fovea where expected information is highest,
   and does not keep returning to the same place.
2. **A model of its own body.** Forward models of the wheels and the gimbal,
   predicting how the scan and the picture should move for each command,
   recalibrated continuously from the difference, which is the efference copy.
   Today's bench-measured faults, the over-turn, the backlash, the heading of a
   picture, become things the rover tracks itself.
3. **A model of the house that predicts.** Learned from the rover's own
   recordings, predicting in a learned representation from pose and action,
   with its memory anchored in the 3D field. Its error is the one signal for
   change, novelty and curiosity.
4. **A map that knows what does not move.** A stability for every cell, learned
   from weeks of change; localisation that leans on stable structure; things
   recorded relative to it.
5. **Object files.** Identity carried by continuous paths and distinctive
   features; changes explained as the fewest events, under the core rules that
   things persist, move continuously and are moved by somebody.
6. **Two memories.** Surprises kept as episodes, separated by place and time;
   general knowledge of kinds, rates, habits and affordances; nightly replay
   from the first into the second.
7. **Imagination for choosing.** Predicted views from candidate places, so that
   looks and visits are chosen by what the rover expects to learn.
8. **Meaning from doing.** Kinds formed from what things do: blocks the way,
   moves daily, stalls the wheels, can be looked behind. Names attach to those
   kinds, from the owner and from a language model.
9. **Language at the top.** A language model that reads the model's state and
   its evidence, names, explains and asks, and writes back claims, as the
   perception plan describes. It is not where seeing happens.

## Today's failures, read this way

| Measured failure | What a brain does | What the design does |
|---|---|---|
| Pose 1.35 m wrong among moved armchairs, still called trusted ([shakedown](../progress/2026-10-12-m5-shakedown.md)) | stops using landmarks it has learned move; boundary cells anchor on walls; a second sense disagrees | localise on stable cells; track furniture as objects; a camera second opinion on where the rover is |
| Red bean bag split across its move; one armchair's history over seven records ([audit](../../experiments/perception_bench/shakedown_audit.md)) | object files follow paths; infants individuate by path before looks; things are moved, not replaced | explain changes as moves, arrivals and departures; identity by path and distinctive features |
| One record holds both armchairs (same audit) | pattern separation by context; completion only on a strong distinctive cue | separate by place and time by default; join on distinctive cues |
| "Seen empty" fired on half the looks at unmoved things ([2026-10-10](../progress/2026-10-10-m5-what-reports-a-change.md)) | absence is an expectation that failed, from a place that should have been visible | absence as prediction error with depth seen through the place |
| A turn of 30 degrees goes 50; a picture's heading several degrees out ([R-NAV-9](../requirements/navigation.md#r-nav-9)) | efference copy and forward models, recalibrated continuously | a self-model of wheels and gimbal, recalibrated from every move |
| Kept regions cover a quarter of each picture; walls, floors, doors ignored ([the perception plan](perception-that-keeps-what-it-learns.md#where-information-is-lost-today)) | gist and layout from the whole view; fovea on what matters | periphery for gist and layout; fovea aimed by expected information |
| No idea what anything is or whether it moves | concepts from interaction, names later | kinds from what things do, then names |

## What could be tried on the recordings now

Hardware is set aside above, but these five need only the recordings and stores
the rover already holds. Each is a measurement first; anything that passes goes
through the ordinary replay, deploy and verify cycle of the component it
touches. Navigation's files belong to whichever session holds `ros_nav` at the
time, so the first is a proposal to it.

1. **Localising on stable cells.** Weight each lidar return by how stable its
   cell has been, learned only from the recordings made before the shakedown,
   so that the test does not already know the armchairs moved, and replay the
   shakedown from the checkpoint before 08:00
   ([R-NAV-11](../requirements/navigation.md#r-nav-11)). *It passes if* the pose
   stays within 0.3 m of the refit's answer through 08:02:40, where the settings
   the shakedown ran with ended 1.33 m out, stays within 5 cm of the live pose
   elsewhere in the session, and makes no single correction over 0.15 m that
   those settings did not. It is compared with the setting navigation adopted after
   the shakedown, which tackles the same failure another way: the scan matcher
   charging for distance from where odometry puts the rover, which in replay
   ends 2 cm from the refit's answer
   ([2026-10-12](../progress/2026-10-12-the-pose-among-moved-furniture.md)). The
   two are not exclusive, and the comparison is whether stability adds anything
   in rooms where odometry itself is poor. Navigation's session runs replays on
   the rover one at a time, so this one is scheduled with it.
2. **Explaining changes.** Pair each furniture-sized "gone" with a similar "new"
   logged in the same window, as a move, under continuity rules (no move through
   a wall, one object in one place). *It passes if*, on the shakedown and scene
   1, it pairs the red bean bag's two places and each armchair's old and new
   places, and on the seven controls proposes no move at all. This is what M5's
   criterion 4 needs ([autonomous-curiosity.md](autonomous-curiosity.md#milestone-m5-the-rover-notices-useful-changes-without-filling-the-world-with-duplicates)).
3. **Surprise.** For each look, compare its dense features with those of the
   stored look taken nearest the same pose, and map the difference. *It passes
   if* the places with the largest differences on the shakedown and scene 1
   include every staged change, and on the controls none rises above the level
   the staged changes reach.
4. **A forward model of turning.** From every recorded turn, predict the scan's
   rotation from the command and the gyro, and measure the rest. *It passes if*,
   on turns held out by day, the predicted rotation is within 2 degrees of the
   scan's in nine turns out of ten. It would also give
   [R-NAV-9](../requirements/navigation.md#r-nav-9) the measurement it lacks.
5. **A second opinion on place.** Describe each look's whole picture with a
   frozen place-recognition descriptor (DINOv2 features, as AnyLoc does; it
   found the right indoor place first time 75% of the time against 53% for
   NetVLAD; Keetha et al., 2023), and ask whether it matches the stored looks from the
   believed pose better than those from elsewhere. *It passes if* it flags the
   shakedown's 08:01-08:11 stretch, when the pose was wrong, and flags no more
   than one look in a hundred elsewhere in the recordings, when it was right.

Further out, the house's own world model: an action-conditioned predictor in a
learned representation, trained on all the rover's recordings, whose surprise
map would be compared with the change watch's log and the owner's staged
changes. It is the one part of this design that needs a GPU the rover does not
have.

## What does not change

- Navigation and the daemon keep authority over movement; nothing believed
  grants it ([R-SAFE-4](../requirements/safety.md#r-safe-4)), and every blocker
  in motion is still handled the same way.
- A learned model proposes, and the map, the rules and the evidence decide. That
  is the lesson of the video models' failures as much as of this repository's
  history with a local vision model
  ([cosmos-reason2.md](../decisions/cosmos-reason2.md)).
- Nothing is changed on the rover by this document.

## Sources

Opened on 2026-10-12 by the research behind this design. Most findings are
taken from abstracts. Where a publisher refused the fetch, the abstract was read
through Europe PMC and the entry gives its PubMed number. Papers from 2026 are
preprints, not yet peer-reviewed. Sources the perception plan already lists
(dense feature maps, persistence filters, V-JEPA 2, navigation world models)
are in [its list](perception-that-keeps-what-it-learns.md#sources).

Seeing as sampling:
Intoy, Mostofi & Rucci, 2021, PNAS, https://aplab.bcs.rochester.edu/assets/download/PDFs/articles/IntoyEtAl21.pdf;
Poletti, 2023, Vision Research 211:108277, https://aplab.bcs.rochester.edu/assets/download/PDFs/articles/PolettiVR23.pdf;
Rensink, O'Regan & Clark, 1997, Psychological Science, https://cim.mcgill.ca/~clark/vmrl/web-content/papers/jjclark_psychscience_08_1997.html;
Simons & Levin, 1998, Psychonomic Bulletin & Review, https://experts.illinois.edu/en/publications/failure-to-detect-changes-to-people-during-a-real-world-interacti/;
O'Regan, 1992, Canadian Journal of Psychology, https://pubmed.ncbi.nlm.nih.gov/1486554/;
O'Regan & Noë, 2001, Behavioral and Brain Sciences, https://pubmed.ncbi.nlm.nih.gov/12239892/;
Luck & Vogel, 1997, Nature, https://pubmed.ncbi.nlm.nih.gov/9384378/;
Ballard, Hayhoe & Pelz, 1995, Journal of Cognitive Neuroscience, https://pubmed.ncbi.nlm.nih.gov/23961754/;
Ballard, Hayhoe, Pook & Rao, 1997, Behavioral and Brain Sciences, https://pubmed.ncbi.nlm.nih.gov/10097009/;
Oliva & Torralba, 2006, Progress in Brain Research, https://pubmed.ncbi.nlm.nih.gov/17027377/;
Potter, 1976, Journal of Experimental Psychology: Human Learning and Memory, https://pubmed.ncbi.nlm.nih.gov/1003124/.

Pathways and affordances:
Mishkin, Ungerleider & Macko, 1983, Trends in Neurosciences, https://irp.nih.gov/accomplishments/anatomy-of-perception-identifying-two-visual-streams-in-the-brain;
Goodale & Milner, 1992, Trends in Neurosciences, https://pubmed.ncbi.nlm.nih.gov/1374953/;
Goodale, Milner, Jakobson & Carey, 1991, Nature, https://pubmed.ncbi.nlm.nih.gov/1986306/;
Epstein & Kanwisher, 1998, Nature, https://pubmed.ncbi.nlm.nih.gov/9560155/;
Bonner & Epstein, 2017, PNAS, https://pmc.ncbi.nlm.nih.gov/articles/PMC5422815/;
Julian et al., 2016, Current Biology, https://pubmed.ncbi.nlm.nih.gov/27020742/;
Auger, Mullally & Maguire, 2012, PLoS One, https://pubmed.ncbi.nlm.nih.gov/22912894/;
Gibson, 1979, The Ecological Approach to Visual Perception, https://doi.org/10.4324/9781315740218;
Lee, 1976, Perception, https://pubmed.ncbi.nlm.nih.gov/1005020/.

Prediction:
Rao & Ballard, 1999, Nature Neuroscience, https://pubmed.ncbi.nlm.nih.gov/10195184/;
Keller, Bonhoeffer & Hübener, 2012, Neuron, https://pubmed.ncbi.nlm.nih.gov/22681686/;
Friston, 2010, Nature Reviews Neuroscience, https://pubmed.ncbi.nlm.nih.gov/20068583/;
Duhamel, Colby & Goldberg, 1992, Science, https://pubmed.ncbi.nlm.nih.gov/1553535/;
Wolpert, Miall & Kawato, 1998, Trends in Cognitive Sciences, https://pubmed.ncbi.nlm.nih.gov/21227230/;
Blakemore, Wolpert & Frith, 1998, Nature Neuroscience, https://pubmed.ncbi.nlm.nih.gov/10196573/;
Schultz, Dayan & Montague, 1997, Science, https://pubmed.ncbi.nlm.nih.gov/9054347/;
Lisman & Grace, 2005, Neuron, https://pubmed.ncbi.nlm.nih.gov/15924857/.

Objects and physics:
Kahneman, Treisman & Gibbs, 1992, Cognitive Psychology, https://pubmed.ncbi.nlm.nih.gov/1582172/;
Spelke, 1990, Cognitive Science, https://doi.org/10.1207/s15516709cog1401_3;
Spelke & Kinzler, 2007, Developmental Science, https://pubmed.ncbi.nlm.nih.gov/17181705/;
Xu & Carey, 1996, Cognitive Psychology, https://www.harvardlds.org/wp-content/uploads/2018/05/Xu-Infants’-metaphysics-The-case-of-numerical-identity..pdf;
Battaglia, Hamrick & Tenenbaum, 2013, PNAS, https://pubmed.ncbi.nlm.nih.gov/24145417/;
Ullman, Spelke, Battaglia & Tenenbaum, 2017, Trends in Cognitive Sciences, https://pubmed.ncbi.nlm.nih.gov/28655498/;
Fischer, Mikhael, Tenenbaum & Kanwisher, 2016, PNAS, https://pubmed.ncbi.nlm.nih.gov/27503892/.

The map:
O'Keefe & Dostrovsky, 1971, Brain Research, https://pubmed.ncbi.nlm.nih.gov/5124915/;
Hafting et al., 2005, Nature, https://pubmed.ncbi.nlm.nih.gov/15965463/;
Taube, Muller & Ranck, 1990, Journal of Neuroscience, https://pubmed.ncbi.nlm.nih.gov/2303851/;
Lever et al., 2009, Journal of Neuroscience, https://pubmed.ncbi.nlm.nih.gov/19657030/;
Høydal et al., 2019, Nature, https://pubmed.ncbi.nlm.nih.gov/30944479/;
Deshmukh & Knierim, 2013, Hippocampus, https://pubmed.ncbi.nlm.nih.gov/23447419/;
McNaughton et al., 2006, Nature Reviews Neuroscience, https://pubmed.ncbi.nlm.nih.gov/16858394/;
Knierim, Kudrimoti & McNaughton, 1995, Journal of Neuroscience, https://pubmed.ncbi.nlm.nih.gov/7891125/;
Biegler & Morris, 1993, Nature, https://pubmed.ncbi.nlm.nih.gov/8437622/;
Jeffery, 1998, Neuropharmacology, https://pubmed.ncbi.nlm.nih.gov/9705005/;
Fisher et al., 2019, Nature, https://pubmed.ncbi.nlm.nih.gov/31748749/;
Whittington et al., 2020, Cell, https://pubmed.ncbi.nlm.nih.gov/33181068/.

Memory:
McClelland, McNaughton & O'Reilly, 1995, Psychological Review, https://pubmed.ncbi.nlm.nih.gov/7624455/;
Kumaran, Hassabis & McClelland, 2016, Trends in Cognitive Sciences, https://pubmed.ncbi.nlm.nih.gov/27315762/;
Wilson & McNaughton, 1994, Science, https://pubmed.ncbi.nlm.nih.gov/8036517/;
Girardeau et al., 2009, Nature Neuroscience, https://pubmed.ncbi.nlm.nih.gov/19749750/;
Yang et al., 2024, Science, https://pubmed.ncbi.nlm.nih.gov/38547293/;
Leutgeb et al., 2007, Science, https://pubmed.ncbi.nlm.nih.gov/17303747/;
Neunuebel & Knierim, 2014, Neuron, https://pubmed.ncbi.nlm.nih.gov/24462102/;
Chettih et al., 2024, Cell, https://pubmed.ncbi.nlm.nih.gov/38554707/;
Tse et al., 2007, Science, https://pubmed.ncbi.nlm.nih.gov/17412951/;
Nader, Schafe & LeDoux, 2000, Nature, https://pubmed.ncbi.nlm.nih.gov/10963596/.

Planning and imagining:
Pfeiffer & Foster, 2013, Nature, https://pubmed.ncbi.nlm.nih.gov/23594744/;
Widloski & Foster, 2022, Neuron, https://pubmed.ncbi.nlm.nih.gov/35180390/;
Vollan et al., 2025, Nature, https://pubmed.ncbi.nlm.nih.gov/39900625/;
Stachenfeld, Botvinick & Gershman, 2017, Nature Neuroscience, https://pubmed.ncbi.nlm.nih.gov/28967910/;
Mattar & Daw, 2018, Nature Neuroscience, https://pubmed.ncbi.nlm.nih.gov/30349103/;
Gillespie et al., 2021, Neuron, https://pubmed.ncbi.nlm.nih.gov/34450026/;
Hassabis, Kumaran, Vann & Maguire, 2007, PNAS, https://pubmed.ncbi.nlm.nih.gov/17229836/;
Schacter & Addis, 2007, Philosophical Transactions of the Royal Society B, https://pubmed.ncbi.nlm.nih.gov/17395575/.

Insects:
Cartwright & Collett, 1983, Journal of Comparative Physiology A 151:521 (bibliographic details only);
Baddeley, Graham, Husbands & Philippides, 2012, PLoS Computational Biology, https://pubmed.ncbi.nlm.nih.gov/22241975/;
Seelig & Jayaraman, 2015, Nature, https://pubmed.ncbi.nlm.nih.gov/25971509/;
Dupeyroux, Serres & Viollet, 2019, Science Robotics, https://www.cnrs.fr/en/press/first-walking-robot-moves-without-gps;
Gattaux, Wystrach, Serres & Ruffier, 2025, Nature Communications, https://ideas.repec.org/a/nat/natcom/v16y2025i1d10.1038_s41467-025-62327-3.html.

Robots that copied the brain:
Milford & Wyeth, 2008, IEEE Transactions on Robotics, https://faculty.washington.edu/minster/bio_inspired_robotics_2018/papers/Milford_wyeth_mapping_a_suburb_bio_inspired_SLAM_TRO2008.pdf;
Banino et al., 2018, Nature, https://research.google/pubs/vector-based-navigation-using-grid-like-representations-in-artificial-agents/;
Schaeffer, Khona & Fiete, 2022, NeurIPS, https://papers.neurips.cc/paper_files/paper/2022/file/66808849a9f5d8e2d00dbdc844de6333-Paper-Conference.pdf;
Biswas & Veloso, 2017, Robotics and Autonomous Systems, https://publications.ri.cmu.edu/episodic-non-markov-localization;
Zimmerman et al., 2023, IEEE Robotics and Automation Letters, https://arxiv.org/abs/2210.01456;
Churchill & Newman, 2013, International Journal of Robotics Research, https://ori.ox.ac.uk/media/5616/2013ijrr_churchill.pdf;
Huang et al., 2024, Out of Sight, Still in Mind, https://arxiv.org/abs/2309.15278;
Diaz-Pereda et al., 2026, REMIND, https://arxiv.org/abs/2607.09267;
Schmid et al., 2024, Khronos, https://arxiv.org/abs/2402.13817;
Keetha et al., 2023, AnyLoc, https://arxiv.org/abs/2308.00688.

Video and world models:
Garrido et al., 2025, Intuitive physics understanding emerges from self-supervised pretraining on natural videos, https://arxiv.org/abs/2502.11831;
Chen, Viégas & Wattenberg, 2023, depth and saliency inside a diffusion model, https://arxiv.org/abs/2306.05720;
Kang et al., 2025, How Far is Video Generation from World Model, ICML, https://arxiv.org/abs/2411.02385;
Motamed et al., 2025, Physics-IQ, https://arxiv.org/abs/2501.09038, and its leaderboard, https://github.com/google-deepmind/physics-IQ-benchmark;
Bansal et al., 2025, VideoPhy-2, https://arxiv.org/abs/2503.06800;
Chow et al., 2025, PhysBench, https://arxiv.org/abs/2501.16411;
Google DeepMind, 2025, Genie 3, https://deepmind.google/discover/blog/genie-3-a-new-frontier-for-world-models/;
Li, Torr, Vedaldi & Jakab, 2025, VMem, https://arxiv.org/abs/2506.18903;
Yu et al., 2025, Context as Memory, https://arxiv.org/abs/2506.03141;
Assran et al., 2025, V-JEPA 2, https://arxiv.org/abs/2506.09985;
Hafner, Yan & Lillicrap, 2025, Dreamer 4, https://arxiv.org/abs/2509.24527.
