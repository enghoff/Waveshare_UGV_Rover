# A trial set is sized by what it has to show

Status: agreed 2026-10-07 by the owner for M3's sessions. The plan's other counts,
and the scope of M8, were revised in a review of the
[plan](../plans/autonomous-curiosity.md) that the owner asked for the same day.
No code has changed.
Its M4 item, that M4 waits on looks reaching their thing, is superseded by
[m4-measures-where-things-are.md](m4-measures-where-things-are.md) (2026-10-09).

M3 no longer counts its sessions. Its criterion 4 asked for twenty supervised
sessions totalling at least 120 minutes. It now asks for one session in each of
seven conditions, listed under the milestone. The criterion passes when every
condition has had a session, no fault a session found is left unfixed, and the
last three sessions turned up nothing new. A session that finds a fault is
followed by its fix and by that condition again. Sessions 3 to 6 are the charger
room's. Each session still reports how long it ran, and criteria 5 and 8 still
apply to every one.

The same rule now governs every count in the plan: a trial set is sized by what
it has to show, and stops once that is shown either way. The same review applied
it as follows:

- **The measurement rules** say so, in place of "20 trials or 50 decisions are
  minimum checks".
- **M3 criterion 12's map and pose half** stands on the offline tests.
- **M4 waits on looks reaching their thing**
  ([R-WS-13](../requirements/world-state.md#r-ws-13),
  [R-WS-17](../requirements/world-state.md#r-ws-17),
  [R-WS-18](../requirements/world-state.md#r-ws-18)) before it spends rover time.
  Its acceptance set starts from the targets already taped, and both it and the
  hardware attempts are sized for the comparison they decide, not set at 30 and 20.
- **M5's multi-day benchmark** compares against fixed-schedule revisits only.
- **M6's reference skill** is shown equal to the executive's own inspection in
  replay and run a few times on the rover, instead of 18 successes in 20 trials.
- **M8 is optional.** It is entered only once M7 has produced a skill whose
  success varies with context.

## Why

The plan said the twenty sessions were "deliberately about repeated opportunities
for timing, interrupt and recovery faults rather than distance travelled". Six
have run, and four of them were in the same room from the same spot. Every fault
they found came from something new in that session: the first run over the
whole flat, the first fenced to one room, the first after the body-fit change,
the first after a reboot. None came from repeating a session already run. The
fourteen still owed would mostly have repeated session 6, at the cost of a charge
and the owner's attendance each.

The same holds for fixed counts elsewhere. Twenty trials in one condition are a
weak test of a rate: 18 successes in 20 show, at 95% confidence, only that the
true rate is above about 70%. They are also a poor search for a fault nobody has
seen yet. A count chosen from what the trials must tell apart says what it
proves. A round number does not.

The individual changes:

- **M5's frontier-only baseline.** Once the flat is mapped, a rover that only
  explores frontiers has nowhere to go. Beating it would show nothing, and
  running it was a third of the benchmark's rover time.
- **M6's twenty trials.** They would measure whether looking from more than one
  side answers a question, which is M4's measurement. What M6 adds is the
  interpreter. That is shown by the skill making the same calls as the executive
  in replay, and by a few runs in which it executes and stops on the rover.
- **M8's practice curriculum.** Practising for competence is not among the things
  the [design](../plans/autonomous-curiosity-design.md) asks the rover to become
  good at. It earns a place only when a learned skill exists that practice could
  improve.
- **M4's waiting.** Of 171 geometry goals on record, 19 improved the thing they
  were aimed at ([the measurement](../progress/2026-10-06-looks-seldom-reach-their-thing.md)).
  Until evidence reaches its thing, a well-chosen viewpoint cannot improve
  knowledge, so trials of the viewpoint planner would measure the world state
  instead.

## What was considered

- **Keeping twenty sessions and 120 minutes.** Declined by the owner, because most
  would repeat a session already run.
- **A smaller fixed count, such as ten.** That still says nothing about what the
  sessions cover.
- **Poor light as a condition.** It was proposed first and dropped. Light changes
  what the cameras see, not how the rover drives or stops. A look that cannot see
  is covered by the condition with the depth camera unavailable.
- **Hardware trials of a map change and of a pose lost mid-run.** Either can only
  be produced on purpose by throwing the map away. What they would test is the
  daemon's response to navigation's own signal, and the offline tests exercise
  that. A natural occurrence in any session is still recorded.

## What would reopen it

A fault that appears in a condition already passed would mean the list missed
what causes it, and the list would grow. A requirement that needs a rate rather
than a search would get a count, set from that rate.

No requirement is added or retired.
