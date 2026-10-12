# What brains and video models suggest for how the rover sees, remembers and finds its way

Status: a design, proposed 2026-10-12 at the owner's request, who asked to go
back to first principles and set hardware aside. Nothing in it is built. It is
the longer view behind [the perception plan](perception-that-keeps-what-it-learns.md),
which is what can be built on today's hardware. That plan's layers are the first
steps of this design, and its research sections already cover dense 3D feature
maps, persistence filters and robot world models, which this document links
rather than repeats. The same day, the owner's questions turned the principles
into a specific architecture, how it would be built and whether it is feasible
on this rover, and those sections were added; Codex's second reading of the
whole was taken in the same afternoon ([what changed](#second-reading)). Five of the ideas are cheap enough to test on the
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
weakness matters less than it sounds, since most of what it sees repeats; but
the unusual events it most needs to notice are where such a model is weakest,
so those are left to explicit memory and rules.

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

**Built on this rover** ([below](#how-it-would-be-built)), the lidar map and
SLAM stay as the scaffold, with a filter in front that stops the mapper trusting
places known to move. The camera joins the same map instead of keeping a
separate store. Every look is predicted before it is taken, by rendering an
explicit 3D memory from the coming pose, and only the error is processed. Two
learners divide the work: the memory learns in one shot by being written, and a
network that generalises learns slowly, at night. [The verdict](#is-it-feasible-on-this-rover)
is three tiers: prototypes of the parts that answer today's measured failures
in weeks, though proving them needs truth that does not yet exist; the 3D memory
and prediction by rendering in months, once the gimbal's pivots and pointing are
modelled; and the learned world model a research gamble for want of video and
commands to learn from.

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
the identity itself. And an explanation stays a hypothesis: two
indistinguishable chairs swapped while nobody looked cannot be told apart by the
fewest events, so both readings are kept, with a look to settle them, rather
than one joined to make the story tidy
([R-WS-2](../requirements/world-state.md#r-ws-2),
[R-WS-8](../requirements/world-state.md#r-ws-8)).

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
distinctive cues: the charger armchair's throw and boxes are a cue that could
have kept its history together, though a removable one, never an identity key
on its own.

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
earlier systems without them found none (Out of Sight, Still in Mind; Huang et al., 2024). Frozen DINOv3
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
alone (Hafner, Yan & Lillicrap, 2025). The rover's recordings are not that kind
of data yet: its 21 complete recordings, 4 h 35 min, hold the lidar, odometry
and IMU, with no camera and no commands. Commands exist only for one day's turn
trials, in separate files.

What this means for the rover:

- **Predict in a learned representation, not pixels**: that is where the
  physical intuitions appear, and it ignores the detail nothing can predict.
- **Condition on the rover's own actions**, which means recording the commands
  beside the camera, as it does not today.
- **Anchor the model's memory in a 3D map, and keep what changes apart from what
  does not**: the perception plan's field is that memory.
- **Train on its own house.** A model that only interpolates is at its best in a
  narrow world seen over and over, and most of what the rover sees repeats. But
  the rare events, a moved chair, a door, a bump, are both what it most needs to
  notice and where such a model is weakest, so they are kept and learned from
  deliberately, and checked by memory and rules rather than left to the model.
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

## How it would be built

The principles made specific, as worked out with the owner after this document
was first written. Nothing here is built; each choice is a proposal with its
reason, and the questions still open are listed at the end of the section.

### Keep the lidar map, and put what moves in front of it

slam_toolbox, Nav2 and the saved map stay as the scaffold. Where the room matches
the map, the mapper's per-update corrections are mostly under 5 cm and the
position check agrees to about 3 cm. Its measured failures are local: the
shakedown, where the scan matcher fitted moved furniture as if it were walls
([2026-10-12](../progress/2026-10-12-the-pose-among-moved-furniture.md)), and the
bedroom on [2026-10-11](../progress/2026-10-11-bedroom-pose.md). Nav2, the refit
and the change watch are built on that map, and no brain-style mapper has beaten
a conventional one on a real robot.

What changes is a filter that keeps the mapper from matching against map cells
known to change, learned from how often each cell has been seen solid and seen
through across visits. Its contract matters more than its idea:

- **The mapper alone gets the filtered scan**, on a topic of its own. The raw
  `/scan` stays untouched for everything else that reads it today: the local
  controller's obstacle layer, the global planner's live-obstacle layer, the
  refit and position checks, the change watch and the recordings. Learned
  stability never removes evidence from avoidance, from the checks, or from the
  truth that later tests are scored against.
- **A dropped return must not become free space.** slam_toolbox cannot weight
  individual returns, so a dropped one is marked invalid, and how the installed
  mapper treats an invalid sample is verified first: one read as maximum range
  would draw a ray of free space through a real obstacle.
- **The mask is chosen from a pose that may already be wrong**, so it can throw
  away the very wall returns that would correct it and reinforce its own
  mistake. It keeps a minimum coverage of returns by angle and by distance, falls
  back to the raw scan whenever the pose is uncertain, and logs every return it
  removes.
- **Trust falls quickly and is earned back slowly**: a cell seen to change loses
  its stability at once, as the rats' trust in a moved cue does, and a cell where
  something new stays put earns it back, so the map is not frozen.

It complements the setting navigation adopted after the shakedown, the scan
matcher charging for distance from odometry (1333b68). That setting helps when
odometry is right and resists a correction whether the correction is right or
wrong: in replay it wobbled 0.6-0.9 m among the moved furniture before settling
on the truth, and on the drive home it ended 0.47 m out where the live mapper
ended 0.21 m out (the same entry).

### Recognising which place it is in

Today a new place needs the map cleared by a person, after saving a snapshot if
the old place is to be kept. A snapshot holds the map, the visual memory, the
change watch's tallies and the rover's pose, and a known place is returned to by
restoring it, then refitting with a person's yes if the rover is not where the
snapshot left it ([R-NAV-3](../requirements/navigation.md#r-nav-3)). Untold, the
rover goes on believing its old map, and its position check says the scan fits
nowhere near where it thinks it is.

A brain recognises which map it is in: a rat taken to a different environment
switches to an independent map, and the old one returns when it is brought back
(Leutgeb et al., 2005; Fyhn et al., 2007). The same can sit on top of the
existing SLAM. At boot, or when the scan fits nowhere, search every snapshot's
map for where the current scan and the camera's place descriptor fit. If one
fits clearly, propose it; if none does, propose a new map and keep the old one.
Each map carries its own record of which cells move. A person confirms with one
tap until the proposals' confident answers have been measured right often enough
to relax R-NAV-3, which would be the owner's decision. For a rover that lives in
one flat the common case is being carried to another room of the same flat,
which the existing refit handles.

### One map, two senses

The lidar stays the backbone for geometry: accurate to centimetres, cheap, and
working in the dark. The camera cannot replace it: it rides a gimbal with a
degree and a half of backlash and pivots that are not modelled, and visual SLAM
on an Orin Nano was measured dropping about 30% of frames at 424x240
([the perception plan's sources](perception-that-keeps-what-it-learns.md#sources)).
What ends is the separation. Today pictures borrow the lidar's position after
the fact and the lidar knows nothing the camera has seen. Joined, the two inform
each other:

- **vision into the map**: a 3D layer over the lidar's 2D map from the depth
  camera, holding what the lidar's one slice misses, and a record of which
  places move, built from both senses;
- **vision into localisation**: the camera's sense of place as a second opinion
  that flags and never moves the rover, and stable visual landmarks, a picture
  on a wall, a door frame, where the lidar is ambiguous;
- **the map into vision**: where the walls are, which places have changed and
  where to aim, and every picture's position.

Avoidance while driving stays the lidar's alone. Nothing the camera sees is
trusted for safety in motion, and every obstacle is still handled the same
([R-NAV-8](../requirements/navigation.md#r-nav-8)).

**The join is a chain of transforms**, each picture's position in the map frame
built from: the map to the rover's frame, which is defined at the lidar rather
than the chassis centre, from SLAM and checked by a scan match at the shutter as
today ([R-WS-16](../requirements/world-state.md#r-ws-16)); the lidar to the
camera, taped by the owner on 2026-10-01 at 23.5 cm above the floor and about
2 cm ahead of the lidar's spin centre, on the centre line
([R-WS-11](../requirements/world-state.md#r-ws-11), settled); the pan and tilt,
as commanded and corrected by the self-model; and the camera to the depth
camera, measured to about a centimetre and a tenth of a degree. What is missing
is a full model of the two pivots: they sit behind and below the lens, so the
camera swings about a centimetre within the gimbal's ±20 degrees, and nothing
models that or carries its uncertainty. Every link carries its timestamp and its
uncertainty. Until the pointing faults, backlash and gain, are corrected,
anything the camera adds to the map is placed a few degrees out, a quarter of a
metre at 3 m.

### What is kept, and where

Five layers, all in the map's frame and keyed by its identity:

- **Evidence**: each look's full picture, depth map, camera pose with its
  uncertainty, and time. Never rewritten; everything below is rebuilt from it.
- **Voxels**: a sparse 5 cm grid from the floor to about 2 m. Each holds how
  often it was seen solid and seen through, and when, kept apart from any
  judgement of what is there. Its appearance is not one average, which would
  blend different occupants, days and lighting into a ghost, but a few
  prototypes by period and viewpoint, each with the observations it came from.
- **Summary layers on the lidar grid's own 5 cm cells**, so that navigation can
  read them directly: something in the cell the lidar's slice misses, how often
  the cell has changed, ground that has stalled the wheels, and what kind of
  thing stands there.
- **Object slots**, kept apart from the voxels: each holds the voxels it
  occupies now, several stored views, its path as intervals (last seen in the
  old place, first seen in the new, the window in which it moved, and when that
  was inferred), and what is known about it. Occupied voxels are not lasting
  objects: a bean bag that moves 3.7 m leaves its old voxels and fills new ones,
  and only an explicit association carries the slot across. That association is
  the "explaining changes" test below, gated on the two frozen regression cases
  (`experiments/perception_bench/`): the red bean bag keeps one ordered history
  across its move, and record 838's two armchairs are refused as one.
- **Rooms and doorways** as a graph cut from the free space, each with view
  descriptors for recognising where the rover is.

Evidence, slots and what is known about them live in the existing SQLite store.
Voxels and summary layers are compressed arrays in a directory per map, the way
the change watch keeps its tallies; they go into snapshots, and a new map
rebuilds them from the evidence rather than reusing them
([R-WS-5](../requirements/world-state.md#r-ws-5),
[R-WS-6](../requirements/world-state.md#r-ws-6)). For this flat it is an
estimated tens of megabytes.

### What navigation uses

From the safest outward:

1. **Localisation**: the stability filter in front of the mapper, and the
   camera's second opinion, which flags and proposes a refit and never moves the
   rover itself.
2. **Route planning**: the global planner's cost map already stacks the SLAM map,
   a live-obstacle layer this repository built (`ugv_behaviors::LiveObstacleLayer`)
   and the padding round obstacles, so a memory layer is a familiar kind of
   plug-in. Its rules: it may only add cost, and fading never clears what the
   live layer or the saved map holds; it fades with staleness, so a moved
   chair's old place frees itself; and it never reaches the local controller,
   whose avoidance stays on the raw lidar.
3. **Choosing where to look and where to stand**: by 3D visibility, so the rover
   can tell whether the camera actually sees the thing past the table from a
   given spot, where today's viewpoint planner reasons in 2D.

### Prediction: the order of work for every look

1. **Predict.** Before the shutter, render from the voxels what the camera
   should see from the pose it is about to take: the depth, the features, and
   which known objects should be in view and where.
2. **Compare.** What was observed minus what was predicted is all that is
   processed further.
3. **Act on the shape of the error**, which says more than its size. Each shape
   is a hypothesis to be weighed against its alternatives, not a verdict:
   - *small everywhere*: nothing new. What was seen is filed as confirmation of
     what was predicted, once its visibility is checked, rather than resolved
     afresh. Today each picture's regions are kept as observations (40,352 from
     6,312 pictures by 11:44 on 2026-10-12), and the resolver attaches each to a
     thing it already holds or founds a new one, look by look; the founding is
     where duplicates come from. The evidence is still kept either way;
   - *local, in a patch*: probably a change, something arrived, left or moved,
     and it goes to the explaining step. A nearer surface is often an occluder
     rather than something new, and missing depth is unknown, not empty;
   - *coherent across the whole view*, everything shifted alike: probably the
     rover's pose, though calibration, a timing error at the shutter or a large
     thing moving in front can do the same. On the shakedown the 1.35 m error
     would have shown as the whole living room offset, the moved armchairs as
     patches;
   - *where the prediction was uncertain*, unseen, stale or often-changing
     places: worth learning, and where to look next, as long as the uncertainty
     is one a look can reduce, not glare or noise that no look will.
4. **Self-motion is predicted too**: a pan or a turn predicts how the image and
   the scan should shift, and the difference calibrates the gimbal and wheels.

What is processed then grows with surprise rather than with time, while the
evidence is kept as before. The rover already has
two predictive pieces, both on the lidar side: the change watch compares each
scan with what earlier visits saw, and the position check asks whether the scan
fits the map where the rover believes it is. The camera side has none, which is
why it can only describe.

### How the prediction is made

**For the present, by rendering.** From the expected camera pose, the fisheye
lens model the rover already has gives every pixel's direction. Each ray, per
pixel or per 16-pixel patch, steps through the voxels until one is solid: the
distance is the predicted depth, and that voxel's feature and colour the
predicted appearance. A ray that meets unexplored space first predicts nothing,
which is itself information. Projecting each object slot's voxels gives where
it should appear. A 320x180 depth view is a few million voxel lookups,
milliseconds on a GPU by estimate.

**The prediction is soft.** Each voxel's "should be here" carries a confidence:
high for walls, and for a thing decaying with time since it was last seen, at
the rate its kind tends to move. A sofa unseen for a week is predicted firmly, a
bean bag unseen since yesterday weakly, so a missing bean bag is mildly
surprising and a missing sofa very. Surprise is a predictive likelihood, each
residual normalised by the spread the prediction expected, so that it can be
checked for calibration, rather than confidence multiplied by an arbitrary
feature distance. The rover's pose uncertainty widens the expected spread at
edges, where a degree of heading makes depth jump. The failure runs both ways:
a change of pose or exposure can make a stable scene surprising, and stale
confidence can make a real move unsurprising.

**Depth first, appearance second.** Measured farther than predicted is evidence
that something has gone only when the ray confidently passes through a volume
that was occupied, with valid depth, a trusted pose and the place in view;
otherwise it is unknown. Nearer is often an occluder rather than something new.
Appearance is compared on learned features that tolerate lighting, since
daylight against lamplight would otherwise look like change everywhere, and this
camera's exposure is known to be unreliable.

**A start before any voxels exist: prediction by recall.** Fetch the stored look
taken nearest the coming pose and warp it, using its own depth map, to the new
viewpoint. It works for small changes of viewpoint and needs only what the store
holds; it is test 3 below.

**Consequences** are the other kind of prediction: "crossing the rug at this
angle stalls the wheels", or "the bag is usually in the bedroom". They start as
tables learned from the rover's own stalls and from counts of where things were
found, and grow into the learned model below.

### The models, and how each learns

Two learners, because one-shot learning and reliable generalisation pull against
each other inside one network: fast learning goes into an explicit memory that is
written, not trained, and slow learning into network weights, trained at night.
Prediction uses both. It is the brain's own division, the hippocampus fast and
particular, the neocortex slow and general, with replay between them.

1. **One shared representation.** A pretrained, language-aligned foundation
   encoder, RADIO-B or DINOv3-B with a text head, turns each 16-pixel patch of a
   view into a feature compressed to about 128 numbers, and the depth camera
   gives each patch a distance. The memory's encoder stays fixed, so memory
   written on Monday is comparable on Friday. If it is ever adapted, the adapted
   one is a new backend: versioned together with its text head, compression,
   crop, lens and calibration, and the memory is re-embedded before anything is
   compared across the change. Its features are never compared with today's
   vectors ([R-WS-4](../requirements/world-state.md#r-ws-4)).
2. **The fast learner: the explicit memory above, written on every observation.**
   One ray-cast write into the voxels is the learning; nothing is retrained.
   RayFronts does this with RADIO features. An object slot opens tentatively on
   its first good sighting and is confirmed by a second from another viewpoint,
   because one picture never fixes an identity
   ([R-WS-2](../requirements/world-state.md#r-ws-2)). Place keyframes are added
   when a view is novel.
3. **The dependable predictor of the present: rendering plus small Bayesian
   filters, no network.** Each object slot runs a two-state filter, there or
   gone, with leave and return rates; things with little history borrow the
   rates of their kind, so the dining chairs share theirs. This is a
   simplification of Perpetua, which also models false and missed detections
   and warns that sparse histories leave the rates unidentifiable
   (Saavedra-Ruiz et al., 2025). A home produces few moves: one staged day
   cannot calibrate a house's rates or its daily rhythms, so the priors stay
   wide, staged moves are kept apart from natural ones, and no filter can repair
   a wrong association underneath it. Where a missing thing went is predicted
   from counts of where it has been found, seeded by the owner or a language
   model (O-STaR's tables; Menon et al., 2026). The rates are fitted to
   intervals, not to the moment a record was made, which is when the resolver
   noticed, not when the thing moved. The self-model is a running regression of
   commanded against measured motion. All of it updates in one step and predicts
   only what has been seen.
4. **The generalising predictor: an action-conditioned transformer working on
   retrieved memory and predicting features, not pixels.** Its inputs are the
   current view's patch features, the action (the change of pose, the pan and
   tilt), and memory tokens, the features rendered from the voxels at the target
   pose with their confidences. Its output is the next view's patch features,
   each with a predicted variance. It is trained by predicting the fixed
   encoder's features of the view that actually came, first on public indoor
   video and then on the rover's own sequences, at night, with older days
   replayed. Four things make that training honest:
   - *causal*: the memory tokens are rendered only from what was known before
     the target view, with the map and pose as they stood then; rendering after
     the target was absorbed leaks the answer, and so does splitting nearly
     identical neighbouring frames between training and test, so splits are by
     day;
   - *the action is the command*: training on the pose change that happened
     cannot teach what a command will do, and the recordings hold no commands
     today;
   - *a proper scoring rule*: the loss is the Gaussian negative log-likelihood,
     each residual squared over its variance plus the log of the variance, so
     that predicting a huge variance is not a way out, with calibration checked
     on held-out days; a single mean and variance may not fit an unseen place
     with several likely looks;
   - *baselines it must beat*: copying the rendered memory alone, no memory,
     and shuffled actions, scored separately on changed and newly uncovered
     regions, so that well-predicted walls cannot hide failure on the rare
     events the system is for.
   About 100 million parameters, trained on a rented GPU and run on the Orin on
   the charger or at a low rate; the size, the value of public video and the
   running time on this Orin are hypotheses, not estimates anyone has measured.
   The memory
   tokens are why it can stay small: the network never has to memorise the
   house, because the memory does that in one shot, and it learns what memory
   cannot supply, how the unseen parts of rooms usually look, occlusion,
   lighting, what people move, and what a turn does. That split, general
   structure in the weights and particular content in the memory, is the
   Tolman-Eichenbaum Machine's, and it is how the video world models with a 3D
   memory stay consistent (Xiao et al., 2025; Li et al., 2025).
5. **When each learns.** On every observation: voxel writes, slot updates,
   counts, filter updates, the self-model's regression. Each night: refit each
   kind's rates and daily rhythms; train small heads on top of the fixed encoder,
   for kinds of thing and for re-identification, the kind of house-specific
   learning that took re-identification of moved objects from 88.9% to 95.6%
   (BYE; Huang et al., 2024,
   [the perception plan's sources](perception-that-keeps-what-it-learns.md#sources));
   and train the predictor on the day's sequences. The pairs that geometry
   proposes, two views of one voxel as one surface, are candidates to be checked,
   not free truth: across a move, an occlusion, a bad pose or a loop closure they
   are wrong, and record 838 is the standing counterexample. New weights are kept
   only if they predict a held-out recent day better than the current ones and
   still pass the identity regression cases and a retrieval test on old days.
6. **New and known places.** In a known place memory dominates: predictions are
   sharp and surprise means change. In a new place the memory is empty, the
   predictor runs on general priors with high uncertainty, and that uncertainty
   is what sends the rover exploring; after one pass, memory dominates there too.
   A new place gets a new memory, and the weights carry over.

Geometry is predicted from measured memory, never invented by the network; every
network output carries its own variance and is checked against the explicit
rules. A claim may describe a thing but never repairs identity or geometry, and
feature similarity never licenses a geometric match or a deletion of free space
([R-WS-7](../requirements/world-state.md#r-ws-7),
[R-WS-8](../requirements/world-state.md#r-ws-8)).

### Smaller alternatives to compare first

Before committing to the whole of this, each part should beat a simpler version
of itself on the same evidence, so that a gain can be traced to the
representation, the memory or the learned predictor:

- **A bank of keyframes**: stored pictures with their depth and pose, indexed by
  where they were taken, plus cautious object slots and visibility tests. It is
  the cheapest form of prediction by recall, and the baseline everything else is
  measured against.
- **A surface or distance-field memory** (TSDF or surfels) as the geometric and
  rendering baseline beside the voxels.
- **Hydra**, which builds hierarchical 3D scene graphs of places, rooms and
  objects incrementally, and whose current code targets the rover's Ubuntu 24.04
  and ROS 2 Jazzy; its memory and speed on this Orin are unmeasured.
- **Khronos**, for how it keeps an active window, associates fragments across
  time and reconciles over the long term after loop closures, and for how it
  handles empty space; it does not by itself cure re-identification across a
  distant move.
- **Gaussian splats**, as a photographic renderer rather than a trustworthy
  record of occupancy, free space or lasting identity.

None of them is an oracle for which physical object is which.

### Questions still open

- Whether pictures can be anchored to the mapper's graph nodes, so they move
  when a loop closure moves the map. The installed slam_toolbox (2.8.5) shows its
  live graph only as visualisation markers (`/slam_toolbox/graph_visualization`)
  and as a saved file, with no structured per-node interface; whether the
  markers carry the timestamps and node bindings anchoring needs is unproven.
  Rebuilding from the same old map-frame poses would repeat the old error, so
  the evidence keeps the raw odometry and timestamps, and each rebuild uses the
  corrected trajectory and records the map's revision, not only its identity.
- A full model of the gimbal's two pivots, and its corrected pointing, which
  every layer inherits.
- Whether a memory layer in the route planner earns its place or only blocks
  routes with stale furniture; replays can say before it is ever enabled.
- Whether feature residuals survive the flat's lighting, and this camera's
  exposure, well enough to be surprise rather than noise.
- Whether leave and return rates can be calibrated from the few moves a home
  produces.
- How far the stability filter can thin returns in furniture-heavy rooms before
  scan matching suffers.
- The data for the generalising predictor: it needs camera video paired with the
  commands that moved the rover, which is not recorded today.

## Is it feasible on this rover

**Mostly, in three tiers, and building a prototype is a different claim from
proving its benefit.**

- **Prototypes in weeks**: tentative object slots with move hypotheses, wide
  priors shared by kind, counts of where things are found, the camera's
  diagnostic sense of place, a stability filter that leaves the raw scan alone,
  and a turning model on the logged trials. They answer the shakedown's measured
  failures and are cheap to compute. What is not in weeks is proving them:
  identity kept across an unseen move, continuous self-calibration and calibrated
  rates of moving all need truth that does not exist yet, such as which scene-1
  armchair went where, poses checked independently of the mapper, and more than
  one day of moves.
- **Feasible with effort, over months, with medium confidence**: the voxel layer
  and prediction by rendering or recall, which give most of "seeing is
  predicting" without a learned model. The limit is geometry rather than
  compute: the gimbal's pivots and pointing have to be modelled first. Its size
  and speed are estimates until measured with the full running stack beside it:
  feature width and precision, several prototypes per voxel, indexes,
  provenance and free-space coverage all count.
- **A research gamble**: the generalising predictor and imagined views. The
  systems that do this trained on tens to thousands of hours of video. The rover
  holds 21 complete recordings totalling 4 h 35 min of lidar, odometry and IMU,
  with no camera video and no commands, and about 6,300 pictures. Most drives
  are supervised, three controls ran unattended at the owner's word, and
  nothing docks it, so data builds slowly. Worth trying once the first two tiers
  stand, not as the foundation; a model of its size is neither shown impossible
  on the Orin nor shown useful there.

What would move the third tier is autonomy more than compute: a dock, so that the
rover roams and records every day ([R-SAFE-6](../requirements/safety.md#r-safe-6)),
and recording camera video at a few frames a second with pose and commands, the
cheapest step that buys the most later. Start with the first tier; its first
test, explaining changes as moves, needs only stores already held, and it is
what M5's identity criterion needs.

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

Codex's second reading found that some of the truth these tests need does not
exist yet ([its review](#second-reading)): individual armchair correspondence in
scene 1, poses checked independently of the mapper at every moment, turn trials
from a second day, and change masks. So each test says what has to be frozen
before it is run, and a test whose truth is missing reports what it found
without passing or failing.

1. **Localising on stable cells.** Weight the mapper's own copy of each scan by
   how stable each cell has been, learned only from recordings made before the
   shakedown, so that the test does not already know the armchairs moved, and
   replay the shakedown from the checkpoint before 08:00 with the raw scan left
   to everything else ([R-NAV-11](../requirements/navigation.md#r-nav-11)).
   *Frozen first*: per-scan reference poses from the scan matched against the
   map independently of the mapper, with the stretches where that match is
   ambiguous marked unknown rather than used. *It passes if* it ends within
   0.3 m and 3 degrees of the refit's answer over the stationary stretch
   08:02:30-08:11, where the rover did not move and the refit is a fair
   reference; it stays within 0.15 m of the reference poses at least as often as
   the deployed setting does wherever they are known; it ends no worse than the
   deployed setting's 0.47 m on the drive home; and it makes no correction over
   0.15 m that neither the live mapper nor the deployed setting made. It reports
   translation and heading, scan fit and coverage, every removed return and every
   processing failure. The deployed setting, the scan matcher charging for
   distance from odometry ([2026-10-12](../progress/2026-10-12-the-pose-among-moved-furniture.md)),
   is the baseline; the two are not exclusive, and the question is whether
   stability adds anything where odometry itself is poor. Navigation's session
   runs replays on the rover one at a time, so this one is scheduled with it.
2. **Explaining changes.** Pair each "gone" with a similar "new" in the same
   window, as a move hypothesis, under continuity rules (no move through a wall,
   one object in one place). *Frozen first*: the candidate change events with
   their visual evidence, keeping the old places the watch logged only small (the
   shakedown armchair's at 0.50 m, scene 1's at 0.35 and 0.40 m) rather than
   requiring furniture-sized pairs; and either labels of which scene-1 armchair
   went where, or "unresolved" accepted as an answer there. *It passes if* the
   red bean bag keeps one ordered history across its move and record 838's two
   armchairs are refused as one, the two frozen regression cases, judged
   separately from any pairing; no move is paired wrongly where labels exist; and
   on the seven controls it proposes no furniture move, with the doors closed
   before control 1 and any person or lighting change labelled rather than
   counted. A plausible pairing alone does not show that the histories stayed
   pure. This is what M5's criterion 4 needs
   ([autonomous-curiosity.md](autonomous-curiosity.md#milestone-m5-the-rover-notices-useful-changes-without-filling-the-world-with-duplicates)).
3. **Surprise.** For each look, warp the stored look taken nearest the same pose
   to it by its depth, and map the difference in depth and in dense features.
   *Frozen first*: which staged changes were in view and scorable, how the
   reference look is chosen, a flagging threshold tuned on scene 1 and then
   fixed, a 0.6 m matching radius, and how looks that cannot be scored are
   counted. *It passes if*, with that threshold, on the shakedown it flags every
   staged change that was in view within 0.6 m, and on the controls it flags no
   more than one look in twenty outside labelled doors, people and lighting
   changes, with the share of looks it abstains on reported beside it.
4. **A turning model.** On the 24 logged turn trials of 2026-10-10
   (`captures/2026-10-10-turns/`), the only recordings with commands, predict
   each whole turn's rotation from the command and the gyro, with the clocks of
   command, gyro and scan aligned, and take the scan's rotation as the measure.
   The gyro is input and the scan the target, never the gyro-integrated odometry
   on both sides. *It passes if*, leaving one trial out at a time, the predicted
   whole-turn angle is within 2 degrees of the scan's in nine turns out of ten,
   and beats the calibrated gyro alone. The claim that it holds on a new day waits
   for a second day's trials, frozen before they are scored. It would also give
   [R-NAV-9](../requirements/navigation.md#r-nav-9) part of the measurement it
   lacks.
5. **A second opinion on place.** Describe each look's whole picture with a
   frozen place-recognition descriptor (DINOv2 features, as AnyLoc does; it
   found the right indoor place first time 75% of the time against 53% for
   NetVLAD; Keetha et al., 2023), and ask whether it matches stored looks from
   the believed pose, taken with the same camera, gimbal angle and field of view,
   better than those from elsewhere. References come from earlier days than the
   looks scored. *Frozen first*: per-picture pose labels, right, wrong or
   unknown, where the error grew from about 08:00:58 and the stationary stretch
   08:02:30-08:11 is the clearly wrong one; and looks beside moved furniture
   labelled apart from looks with a wrong pose. *It passes if* it flags the
   stationary wrong stretch within its first five looks, and flags no more than
   one correct-pose look in a hundred elsewhere, with false alarms per episode
   reported beside it. It only ever flags; it never chooses or refits a map
   ([R-NAV-3](../requirements/navigation.md#r-nav-3)).

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

## Second reading

Codex read the whole document at d2efc51 on 2026-10-12, read-only, against the
code, the copied stores, its catalogue and the recordings' metadata; its review
is kept outside the repository, in `tmp/codex-assistance-replies/`. It found the
points from its review of the sister plan taken in correctly, and approved
offline prototypes but not high confidence in weeks for dependable explanation
of moves. Taken in:

- **Facts corrected**: the camera's position relative to the lidar was taped on
  2026-10-01 and R-WS-11 is settled; what is missing is a model of the gimbal's
  pivots. The 40,352 rows are observations, which the resolver attaches to
  things it holds or founds anew, not 40,000 new things. The mapper's live graph
  is visible as markers. The recordings hold no camera and no commands. Controls
  2-4 ran unattended, so not every drive was supervised.
- **The stability filter's contract**: the mapper alone gets a filtered copy of
  the scan; dropped returns must not turn into free space; the mask can
  reinforce a wrong pose, so it keeps coverage, falls back to the raw scan when
  the pose is uncertain and logs every removal.
- **The encoder stays fixed**, or a changed one is a new backend and the memory
  is rebuilt before comparison; geometry's pairs are candidates, not free truth.
- **The predictor's training**: causal memory, commands rather than realised
  motion, a proper likelihood, and baselines scored on changed regions.
- **Surprise as a calibrated likelihood with a visibility model**, its shapes as
  hypotheses; explanations of moves kept as hypotheses where two candidates are
  indistinguishable.
- **Every test given the truth it needs**, and told to report rather than pass
  or fail where that truth does not exist.
- **Feasibility split** into building prototypes and proving their benefit, and
  **smaller alternatives** to compare first.

It also asked that the scientific sections be read as inspiration from competing
accounts of how brains and models work, not as proof that one mechanism or one
error signal guarantees this architecture. That is how they are meant.

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
Whittington et al., 2020, Cell, https://pubmed.ncbi.nlm.nih.gov/33181068/;
Leutgeb et al., 2005, Science, https://pubmed.ncbi.nlm.nih.gov/16040709/;
Fyhn et al., 2007, Nature, https://pubmed.ncbi.nlm.nih.gov/17322902/.

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
Huang et al., 2024, Out of Sight, Still in Mind, https://arxiv.org/abs/2309.15278 (the BYE paper of the same authors' surname is in the perception plan's list);
Diaz-Pereda et al., 2026, REMIND, https://arxiv.org/abs/2607.09267;
Schmid et al., 2024, Khronos, https://arxiv.org/abs/2402.13817;
Keetha et al., 2023, AnyLoc, https://arxiv.org/abs/2308.00688.

Memory and prediction in the architecture:
Saavedra-Ruiz et al., 2025, Perpetua, https://arxiv.org/abs/2507.18808;
Menon et al., 2026, O-STaR, https://www.hrl.uni-bonn.de/publications/2026/menon26grc;
Xiao et al., 2025, WorldMem, https://arxiv.org/abs/2504.12369;
Hydra, MIT-SPARK, https://github.com/MIT-SPARK/Hydra;
slam_toolbox interfaces upstream, https://github.com/SteveMacenski/slam_toolbox#api (the installed 2.8.5 lacks the newer node and graph topics).

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
