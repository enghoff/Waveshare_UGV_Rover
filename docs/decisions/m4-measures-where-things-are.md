# M4 measures where things are, against a re-look, on average

Status: agreed 2026-10-09 by the owner, who asked that M4 be made achievable
with the rover's present hardware, vision models and approach. Changes Phase 4
and M4 in the [plan](../plans/autonomous-curiosity.md), and supersedes the M4
item of [trials-are-sized-by-what-they-show.md](trials-are-sized-by-what-they-show.md).
No code has changed. A reported figure, whether the rover's own gain agrees with
the tape, is added by
[every-gate-says-what-it-is-for.md](every-gate-says-what-it-is-for.md).

M4 asked that additional viewpoints measurably improve knowledge, and judged it
by a bar no change of viewpoint can meet on this rover: more than half of a set
of hardware attempts had to improve their thing (the median gain positive), with
"knowledge" open to include what a thing is, its colour and whether it moves.
It now asks the same question of the one kind of knowledge the rover measures
well, where a thing is and how sure it may be of that, and answers it with a
comparison rather than a majority:

- **The knowledge is placement, scored against tape.** A thing's position and
  stated uncertainty, before and after an attempt, scored against the owner's
  tape with a rule that rewards coming closer and narrowing honestly and
  penalises narrowing past the tape. Attribute claims leave M4 until a model that
  describes things has been shown to do it on this rover's pictures.
- **Each attempt is paired with a re-look.** Before driving to the viewpoint it
  chose, the rover aims at the thing from where it stands and takes one look. Both
  looks are scored offline against a copy of the store as it stood before the
  attempt, so each starts from the same knowledge. Chosen viewpoints must improve
  placement more than re-looks do, by a margin the attempts can tell apart from
  none.
- **The mean replaces the median.** Across the full attempt set the average gain
  must be positive. A look that finds nothing counts as zero and a look that makes
  a placement worse, or files to another thing, counts against. The share of
  attempts that improved anything is reported by condition, not held to a bar.
- **A new taped set.** The six things taped on 2026-10-03 fixed the depth
  placement rule, so they are development evidence. Acceptance needs new ones.
- **Honesty stays pass/fail.** Overconfident claims, the tape more than twice the
  stated uncertainty away, are no more common after chosen-viewpoint looks than
  after re-looks, and how often the tape lies inside the stated figure is reported
  against what a one-sigma figure should cover.

## Why

**Placement is what the hardware measures well.** Where a look reaches the right
thing, the depth camera puts it a median 0.11 m from the tape on five of the six
taped things, and the stated uncertainty covers the tape 68% of the time, about
what a one-sigma figure should
([2026-10-08](../progress/2026-10-08-depth-placement.md)). Bearings alone stop at
about 0.36 m however many views are added. A second viewpoint in depth view is
therefore something that can genuinely improve knowledge, and it is measurable
against tape.

**Attributes need a model the rover does not run.** The region finder (YOLOE)
names nothing, the two vector models (DINOv2, SigLIP2) compare appearance, and
SigLIP2's text side only searches stored pictures. The last model that described
things, tested locally, gave different names for byte-identical frames and took
about a minute a look ([cosmos-reason2.md](cosmos-reason2.md)). A milestone that may be judged on
attribute correctness cannot pass on a model that does not exist yet.

**The median measured the store, not the viewpoints.** On 2026-10-08 about one
aimed look in six with its thing in the depth camera's view improved that thing,
14 of 80 ([2026-10-08](../progress/2026-10-08-depth-sees-the-thing.md)). Of 32
looks that filed nothing, 20 were aimed at records not where their thing is, 9
found nothing that looked like it, and 3 found no usable region at all
([2026-10-08](../progress/2026-10-08-why-aimed-looks-miss.md)). Choosing targets
better raises the rate to about one in three at best
([2026-10-08](../progress/2026-10-08-what-predicts-a-filing.md)). What holds it down
is one object split across several records and records placed where nothing
stands (R-WS-13, R-WS-17, R-WS-18). That is world-state work, which has had no
owner since 2026-10-07. A milestone gated on it would be a measurement of the
world state, which is what the decision of 2026-10-07 said M4's trials must not
become.

**A comparison still answers M4's question.** Whether driving somewhere chosen
adds knowledge that looking again does not is the question a viewpoint planner
exists to answer. A mean above zero says the attempts do more good than harm,
and the paired difference says the choice of place is why.

## What is given up

M4 no longer says that a typical attempt helps. At today's rate most attempts
still spend battery and learn nothing; that is reported, by condition, and not
hidden. M4 also no longer says anything about what things are.

## What was considered

- **Keeping the median.** Unreachable until the store holds one record per
  object, placed where the object is, and nobody is working on that.
- **Counting only attempts at records known to be well placed.** That chooses
  attempts by their answer, which the plan's rule on counting every attempt
  forbids. Applicability is decided instead from what the rover knew before each
  attempt: a reachable viewpoint with the thing inside the depth camera's view.
- **The nearest reachable viewpoint as the baseline**, which the plan also allowed.
  It tests the choice of place more sharply, but doubles the driving on a battery
  that gives 20 to 25 minutes. Unless it improves its target far less often than
  chosen viewpoints, it needs well over a hundred pairs to tell them apart, against
  about 40 with a re-look that seldom improves anything. The re-look costs seconds.
- **Attribute claims from SigLIP2's text matching.** Its scores are not calibrated
  as claims: at the old search floor, 23 of 60 nonsense phrases found something
  (2026-10-05), and the raised floor quarters that rather than ending it.
- **Attribute claims from the hosted realtime model.** Never benchmarked on this
  rover's pictures, and every look would spend network and quota.

## What would reopen it

- The store reaching one record per object, placed where its object is
  (R-WS-13, R-WS-17, R-WS-18). A typical attempt helping then becomes a fair bar,
  and the median can come back.
- A describing model passing an offline benchmark on labelled pictures from this
  rover, abstaining when unsure and right about as often as it claims. Attribute
  claims then return, as their own criterion comparing several viewpoints with one.
- Re-looks improving their target nearly as often as chosen viewpoints. The
  comparison would then be with the nearest reachable viewpoint.

## Requirements

Relies on [R-WS-13](../requirements/world-state.md#r-ws-13)'s filing by aim, the
case agreed for supervised runs. No longer waits on
[R-WS-17](../requirements/world-state.md#r-ws-17) or
[R-WS-18](../requirements/world-state.md#r-ws-18), which stay proposed. Retires
no requirement.
