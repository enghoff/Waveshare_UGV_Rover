# A cleaner side view reproduces the background visibility rejection

R-WS-13 remains open; R-WS-17 and R-WS-18 remain proposed. The proposed 5%
contamination limit is unchanged. No production resolver change was deployed.

## The console's successful exit supersedes the parking inference

The owner tapped the console map and the rover moved away from the wall. Its
completed `drive_to` status reported a 25 cm backoff followed by arrival. The
earlier planning-only `START_OCCUPIED` query omitted that production recovery
step. The separate saved costmaps did not reproduce the console's execution and
cannot establish that its successful exit was impossible or unsafe. The
[earlier report](2026-10-06-return-heading-and-owner-audit.md) remains a record
of those preparation checks, not a reproduced navigation fault. No navigation
fix, map reset or forced fit was made.

## What the run retained

With the owner's control handover, the unchanged rover reached A and B, turned
for the painting, completed a stationary fresh depth inspection, and returned
through A to the saved start. All movement calls completed serially; no command
watchdog fired. A fresh stationary localization check resolved an intermediate
stale drift warning before the last return leg.

The final fresh pose is approximately 13 cm from the start by map estimate,
with heading about 132 degrees different. STOP was acknowledged and idle status
showed zero speed, turn and motor PWM. Reported charge fell from 85% to 75%.
This is not a measured docking position. The owner was told movement was over,
then that power could be switched off once evidence had been copied.

The 27 copied files match the remote SHA-256 manifest. They contain complete
world-call before/after databases, events, pass maps, eleven photographs and
passive board feedback. Exact control replay reproduces 17 resolver passes,
57 checkpoints and 21,106 ordered reach answers; recomputing the answers from
the saved maps also agrees. All 66 available stored sampler answers reproduce,
including 24 ranges, with no minority-band abstention flags. One frame's raw
depth is missing, so complete sampler verification is false. Its nine regions
have no stored range, but their sampler answers remain unverified.

## The useful distinction in the images

Review used full photographs and the actual stored outlines before this run's
candidate output. Some live resolver summaries had already been seen; these
are development labels, not independent acceptance truth.

The earlier painting region 69807 includes foreground chair pixels inside its
outline. It is mixed and excluded from clean same-subject scoring. Its live
owner is chair record 375. Regions 69831 and 69841 are valid painting parts with
the foreground chair excluded, and have ranges 3.832 and 3.842 m. They are
almost identical stationary views, not independent viewpoints. Their live
owners are both unassigned despite plain/masked similarities of 0.974/0.958.
The actual mapped reach for them is only 1.525/1.550 m at the foreground
furniture. Thus the clean background-region visibility failure is reproduced
again, separately from a mixed-region attachment.

The first-to-side bearing separation is 10.7 degrees, below the frozen 12-degree
minimum; the two clean stationary views differ by only 0.5 degrees. No two
clean whole views were obtained. Coverage failed and the run cannot grant
deployment acceptance. The unchanged a61f1c4 matching-only depth extension is
tested only as a conditional development diagnostic; original discovery,
refitting and other gates remain intact, with every candidate map query required
to agree across all grids in its recorded pass.

## Candidate outcome

The fixed candidate completes its map-invariant replay. It changes seven owners.
On the frozen six clean development regions, control and candidate both retain
one useful chair connection and no cross-subject connection; neither connects
the clean painting pair. The painting gain criterion therefore fails. The
candidate assigns 69841 to painting record 328 and 69831 to record 351. Selected
archived examples in both records depict this same painting behind chairs:
opening the visibility gate does not resolve duplicate historical ownership.
This is an exploratory destination review, not complete record-purity proof.

All seven changed regions were reviewed after scoring against selected available
historical photographs and outlines. The other painting-part and window changes
are consistent with their sampled destinations. However, 69900 shows the side of
the armchair and its throw, while destination 488's sampled references are a white
standing lamp. The full photograph puts that lamp outside the selected outline.
Its accepted crop appearance is 0.567 and bearing allowance used is 0.851. This
is an additional wrong attachment exposed by the candidate; it was outside the
frozen six-region score and must not be concealed by that score's zero crosses.

The conclusion is firm for this candidate: do not deploy it alone. Map visibility
is one reproduced blocker, but mixed/partial region discrimination and duplicate
records are separate blockers. Preserve this harmful attachment as a regression
case before attempting a narrower admission rule. Do not relax the coverage,
retention or contamination criteria, or infer that every possible repair is a
dead end from this particular failure. The next software experiment can use the
saved recording without consuming another battery run.

The existing global rival veto is present in single-region matching but absent
from frame cost assignment. Applying it to every frame candidate, at its existing
0.15 threshold, rejects the lamp error and assigns both clean painting parts to
328. However, it loses the frozen correct chair pair 69806/69830. One painting
connection gained and one chair connection lost is a retention failure, not a
pass. This wider change also remains undeployed. A narrower diagnostic requires
the extra veto only for candidates forbidden by the original unextended ray;
its thresholds and frozen criteria are unchanged.

That narrower diagnostic retains the frozen chair connection, adds the clean
painting pair and refuses 69900's lamp destination. It changes five owners;
all five are covered by the destination review above, with no new reviewed wrong
destination. Its exact control also matches the separately reproduced original
control's final state. On the older frozen proxies it retains 638/643 connections
(99.2%) on October 1 and all existing connections on both October 2 recordings.
The original mixed-crop cross-target pair 48456/48769 still fails the frozen
October 1 identity score. Its selected-region review was already discussed in
the [subject diagnosis](2026-10-05-subject-fragmentation.md); neither those labels
nor this failure are rewritten. This is promising development evidence, not
deployment or independent acceptance. Only one reviewed existing connection was
available in the new six-region score, and viewpoint coverage failed.

`replay_visibility_rival_veto.py --new-visibility-only` preserves the narrower
candidate. Without that option it reproduces the wider retention failure.
`--older-source` verifies each older original control before its candidate arm.
The criterion was narrowed after seeing the failures, so this is explicitly
development rather than a newly independent test. No production source was
modified. Keep the narrower candidate for a genuinely independent recording;
the evidence supports neither deploying the extension alone nor abandoning
all visibility repair as impossible.

## Operational failures remain failures

Return began 78.5 seconds after first motion, missing the 60-second deadline.
The inspection finished around 49 seconds; subsequent preparation consumed the
remaining margin. Admission checks refused late outbound actions but did not
cause a timely return. Agent reminders plus that guard are insufficient.

The navigation support recorder was interrupted before its normal completion.
It writes its accumulated episode only after its timed loop, so the interruption
lost its plans, controller ticks and pose history. They were not copied or
reconstructed. World-call replay and individual costmap snapshots do not replace
that missing continuous navigation evidence. Passive board feedback survived.

Before another handover, prepare and verify a serial executor that proceeds to
checked return without waiting for conversational decisions, and a recording
completion/flush procedure that demonstrably preserves STOP. Do not request a
powered rover while those preparations or offline analysis are unfinished.

Evidence is under `captures/visibility-morning-prepared-3/`; exact control and
subject trace are `.cache/visibility-morning3-control/` and
`.cache/visibility-morning3-trace/`. The [measurement](2026-10-06-side-painting-recording.json)
retains hashes, scores and limitations. This is offline experiment/documentation
work; it requires no service restart or deployment.
