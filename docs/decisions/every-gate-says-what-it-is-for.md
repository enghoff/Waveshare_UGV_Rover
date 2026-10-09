# Every milestone gate says what it is for

Status: agreed 2026-10-09 by the owner, who asked that each remaining milestone
be tied either to a performance a later feature needs or to something about the
rover worth knowing, and that no gate be a formality. Changes M3, M5, M7, M9 and
M10, adds a reported figure to M4, and defers M6 and M7 in the
[plan](../plans/autonomous-curiosity.md). No code has changed.

Each remaining gate was asked what goes wrong later if it is not met, or what is
learnt by meeting it. Most survive. The ones changed here either could not show
what they claimed to, waited on nothing, or could be passed without the rover
becoming any better:

- **M3 stops once the current code has driven the conditions its changes
  touched.** "The last three sessions turned up nothing new" would mostly be met
  by repeats, which [2026-10-07](trials-are-sized-by-what-they-show.md) found
  rarely turn anything up. What is worth knowing is whether the planner's live
  layer and route watch, which changed every route after the charger room and the
  whole flat had passed, broke anything there. So once each condition has had its
  session, the charger room, another room and the whole flat are driven once more
  on the code M3 accepts, and those drives find nothing new. M4's development
  attempts drive exactly those places, so they can be those drives. And a contact
  with something below the lidar's plane, like the shoes and the rug in sessions
  10, 12 and 13, is a failure of the clearing that
  [R-SAFE-6](../requirements/safety.md#r-safe-6) makes a precondition: it is
  recorded and its cause removed. Criterion 5 is failed by contact with something
  the scan could see. As written it was failed already and would have been waved
  through.
- **M4 also reports whether the rover's own account of gain can be trusted.**
  After M4 there is no tape. M7 would judge a skill's success, and M10 predict a
  goal's realised gain, from the gain the rover records for itself. So each
  acceptance attempt reports whether that recorded gain has the sign of the tape's
  score, and the agreement rate is reported with its interval. It is not a bar in
  M4; M10's entry uses it. Nothing else in M4 changes (see *What was considered*).
- **M5 starts by measuring false alarms against how often things here really
  change.** A change the rover reports is worth having only if it is right more
  often than wrong. If things in the flat move in one visit in ten, that needs
  false alarms on fewer than about one unchanged revisit in eleven at the 80%
  detection M5 asks for; ten controls with at most one alarm show only that the
  rate is under 39%, and eight detections in ten only that detection is above 49%.
  The store as it stands would fail well before that: 20 of the 32 aimed looks
  that filed nothing on 2026-10-08 were aimed at records not where their thing is
  ([the measurement](../progress/2026-10-08-why-aimed-looks-miss.md)), and an
  unchanged revisit would read each of those as missing. So M5 begins with two
  measurements and no scripted scenes: how often things in the flat change between
  visits, from the owner and the archived looks, and how many change hypotheses
  replayed revisits of unchanged scenes raise. The false-alarm budget follows from
  the first, and scripted scenes wait until replay meets it. That budget is also
  what the proposed tolerances of [R-WS-17](../requirements/world-state.md#r-ws-17)
  and [R-WS-18](../requirements/world-state.md#r-ws-18) should be checked against.
  Detection (8 in 10) and identity kept across a move (80%) become measurements
  with intervals, as P0's accuracy bars did. Confident wrong merges and confident
  false alarms stay pass/fail, and every count is set by what it must decide.
- **The multi-day benchmark states its cost.** It is the one test of whether the
  programme is useful, and it stays. But R-SAFE-6 is open and nothing docks the
  rover, so every minute of it is supervised and every charge plugged in by hand,
  at 20 to 25 minutes of driving a charge. Its questions and minimum improvement
  are sized by the comparison, and its budget is stated in charges and owner hours
  before it is planned.
- **M6 and M7 wait until there is a skill to learn.** An autonomous run has three
  operations, drive, look and stop, and one way of using them, which M6
  hand-authors as its reference skill and M7 would rather not see rediscovered. A
  learner over that record can only rediscover it or propose something trivial,
  and either would pass M7's gates without the rover knowing anything new. So,
  like M8, they are entered once the episode record holds a repeated multi-step
  behaviour the executive does not already perform as one goal, and that succeeds
  in some contexts and not others. Until then the learning the
  [design](../plans/autonomous-curiosity-design.md) asks for first is the
  viewpoint planner's bounded parameters and M10's outcome model. M7's "at least
  18 of 20" goes, as M6's did on 2026-10-07: a promoted skill replaces what the
  executive would have done, so it must do at least as well as the executive on
  the same cases, paired, with the count sized for that comparison.
- **M9 is judged by what reflection adds.** Nothing later depends on reflection,
  and the plan says the rover must not need it, so it earns its place only by
  finding what the scorer would not. "Useful on a clear majority of 50 cases under
  human review" measures whether proposals sound sensible, which a model manages
  without adding anything. Proposals are judged instead against the scorer's own
  choices at the same budget: first on recordings, by whether a proposal named a
  gap the scorer missed that a later look resolved, then in supervised runs if
  that shows any. The containment criteria stay.
- **M10 can start after M4, not after M9.** The scorer's predicted gain was right
  for about one geometry goal in ten
  ([2026-10-06](../progress/2026-10-06-looks-seldom-reach-their-thing.md)), so a
  run spends most of its charge on looks it expected to pay. Every later phase
  that chooses (revisits, skills, practice, reflection's experiments) trades a
  predicted gain against battery, and that trade means little while the
  prediction is off by that much. An outcome model for the goal types that exist
  needs only M4's attempts, so M10 is entered for them once M4 has run, provided
  M4's sign agreement is better than chance; below that the model would learn the
  rover's own noise. Its criteria are unchanged, and its calibration check is the
  threshold the later phases need.

## What stays, and why

- **M3's other criteria** are safety properties or already met on the rover, and
  its moved-furniture condition is the live layer's own case.
- **M4 as agreed this morning**, with the reported figure above.
- **M5's revisit criteria** (5 and 6) are offline checks of behaviour M4's
  sessions showed going wrong: the rug was looked at twice by way of two records.
- **M6's containment criteria** are M7's safety case for whenever it is entered.
- **M8 and M11** are already entered only on evidence.

## Not decided here: running unattended

The design's goal is a rover that "can be left in a known-safe environment and
productively occupy itself". No milestone gets it there. R-SAFE-6 is open with no
plan to validate drop or off-plane sensing, nothing docks the rover, and in M3
session 9 the battery's percentage moved between 5 and 55 within minutes. Either a
milestone for running unattended is added, with off-plane sensing validated,
docking and a battery reading that can be trusted, or the definition of done says
the rover is supervised. That is the owner's choice, and until it is made every
physical milestone is owner hours.

## What was considered

- **Taking the re-look out of M4**, as the first review of M3 and M4 that day
  suggested. The acceptance attempts are needed for the mean gain anyway, a
  re-look costs seconds, and it is the only thing that will show if looking again
  from where the rover stands does as well as driving. If re-looks seldom help,
  the comparison is redundant but almost free; if they help, it is the question.
- **Dropping M6 and M7.** The definition of done asks for learned skills; an entry
  condition defers them on evidence instead of removing them.
- **Setting M5's false-alarm budget now, at one in ten.** Nobody has measured how
  often things in the flat change. A budget set before that is the round number
  this record replaces.
- **A fixed number of M5 controls, such as thirty.** Sized instead by what they
  must show, which waits on the change rate.

## What would reopen it

- The flat's change rate turning out high enough that one false alarm in ten
  already leaves a change report right more often than wrong.
- Autonomy being given more operations, which makes compositions worth learning
  and meets M6 and M7's entry sooner.
- The owner deciding on unattended running, which adds a milestone or changes the
  definition of done.

## Requirements

Names [R-SAFE-6](../requirements/safety.md#r-safe-6) as the open requirement
between the programme and its stated goal. The tolerances of
[R-WS-17](../requirements/world-state.md#r-ws-17) and
[R-WS-18](../requirements/world-state.md#r-ws-18) stay proposed, to be checked
against M5's false-alarm budget. Adds and retires no requirement.
