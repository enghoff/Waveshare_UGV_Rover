# Progress: what was actually measured

This is the log. Each entry is a dated report of something measured on the rover
or replayed from a real recording, and says which
[requirements](../requirements/README.md) it moved and to what. Entries are
written once and not revised afterwards: an entry is a record of what was known
on a particular day, and rewriting it destroys exactly the thing that made it
worth keeping.

## The ledger

Newest first. "Moved" names the requirements whose state this entry changed;
"held" names requirements it confirmed without changing.

| Date | Entry | Moved | Held |
|---|---|---|---|
| 2026-09-09 | [Five autonomy review defects fixed and deployed](2026-09-09-autonomy-review-fixes.md) | none; R-SAFE-9, R-SAFE-10, R-SAFE-11, R-SAFE-12 and R-AUT-11 remain open | deployed at 9d1a5c2; on-host suites and stationary protocol checks pass; moving acceptance remains |
| 2026-09-08 | [The rover can be given permission to move itself, and three ways to take it back](2026-09-08-permission-to-move.md) | [R-SAFE-9](../requirements/safety.md#r-safe-9) to [R-SAFE-12](../requirements/safety.md#r-safe-12) from `proposed` to `open`; [R-AUT-11](../requirements/autonomy.md#r-aut-11) new and `open`; M3 criteria 1 and 2 met | nothing has driven under it: the rover's pose is identical before and after every check. Ten checks over 8769 including the watchdog ending a run nobody renewed; the executive weighed 23 real goals and refused every one against a 1 cm safe area. Two faults the offline suite could not find, one of them a parked rover ending its own run within a minute. A failing suite on the rover could not fail a deploy, and the first thing that fix caught was `autonomy` failing there. autonomy 641, rover_daemon 963 on the Orin |
| 2026-09-08 | [Placement uncertainty knows something about a merge, and nowhere near enough to refuse one](2026-09-08-uncertainty-cannot-gate.md) | none; [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open` with its most promising remedy measured and refused | nothing deployed to the resolver: a merge outranks a real object 0.680 of the time (95% 0.517-0.827), and the tightest gate admitting no merge keeps 1 of 52 real objects; the 76 verdicts are checked in and `world_state/bench_identity.py` scores against them; world_state 830 passed |
| 2026-09-08 | [The rover can say what it would go and look at, and why it may not: M2 passes](2026-09-08-shadow-decisions.md) | **Milestone M2 passes**; [R-AUT-8](../requirements/autonomy.md#r-aut-8) to [R-AUT-10](../requirements/autonomy.md#r-aut-10) added and `settled` | 25 deliberations in 25 minutes on the rover with 0 calls to it, 625 candidates recorded with their scores, 0.63 s each; 49 of 49 curated scenarios, two first-run disagreements resolved against the code; 5 of the 12 worst-placed things have no reachable viewpoint; a snapshot cost 97 kB until the listing and map were stored once |
| 2026-09-08 | [The rover was driven round the property and the record kept up: M1 passes](2026-09-08-the-driven-run.md) | **Milestone M1 passes**; criterion 4 met by a 34-minute run holding both navigation and world-state events | [R-AUT-1](../requirements/autonomy.md#r-aut-1) to [R-AUT-7](../requirements/autonomy.md#r-aut-7) held, R-AUT-5 across a driven run; 59 moves and 215 looks, the loop's sentences 608 to 1748 with no hole and no look recorded twice; 24 kB a picture and about 9 MB an hour while driving, against a 104 MB estimate that assumed a look a second |
| 2026-09-08 | [The acceptance run: geometry passes inside a declared band, identity does not](2026-09-08-the-acceptance-run.md) | none; M0's placement tolerance passes on held-out trials within a newly declared 0.5-2.5 m band, criteria 2, 3 and 8 fail | 1208 looks, 117 things, 917 decisions; separations within 14 cm of the tape and height within 2.5 cm; two confirmed merges among the 24 most-seen things |
| 2026-09-08 | [Four measurements against a tape, and the geometry passes its declared tolerance](2026-09-08-the-tape-measure-run.md) | none; first held-out geometry result against tolerances declared in advance, and first check of the vertical | all four separations within 12 cm of the tape against a 0.30 m mark; 4 of 29 things still hold two objects, so criterion 3 fails with all three remedies deployed |
| 2026-09-08 | [The world state was emptied for real, and 210 episodes went quiet about what they named](2026-09-08-the-clear-that-proved-it.md) | none; adds the build each episode's evidence was produced by, which the durable evidence contract had asked for and I had left out | [R-AUT-2](../requirements/autonomy.md#r-aut-2) held on hardware rather than on a suite -- 0 of 9 stored names resolved against the store that replaced theirs, while the episode itself still read; the morning's 210 episodes stay silent about the rules change at ~09:43 |
| 2026-09-08 | [Every thing in the room, given a verdict: two in nine are two objects](2026-09-08-every-thing-labelled.md) | none; [R-WS-13](../requirements/world-state.md#r-ws-13) gains a measured rate and [R-WS-12](../requirements/world-state.md#r-ws-12) regains its counter-examples | nothing deployed; 17 of 76 things hold two different objects, 7 are floor or glare, 10 objects are split across several things, and a mixed thing's placement uncertainty runs 0.60 m against 0.36 |
| 2026-09-08 | [The M0 acceptance drive runbook](../runbooks/m0-acceptance-drive.md) is written and the thresholds are frozen in it | none; declares in advance what M0 criterion 7 requires to be declared in advance | not a measurement: the run manifest, tolerances and envelope for the next drive |
| 2026-09-08 | [Thirty minutes of watching the rover, and the daemon restarted in the middle of it](2026-09-08-shadow-run.md) | [R-AUT-4](../requirements/autonomy.md#r-aut-4), [R-AUT-5](../requirements/autonomy.md#r-aut-5), [R-AUT-6](../requirements/autonomy.md#r-aut-6) to `settled`; M1 criteria 5 and 6 met, criterion 4 half met | [R-AUT-1](../requirements/autonomy.md#r-aut-1) to [R-AUT-3](../requirements/autonomy.md#r-aut-3) held against a real recording; 210 looks in 1800 s across a daemon restart, nothing lost or repeated; 30.3 kB a picture; retention freed 2.18 MB oldest-first and spared 10 pinned episodes |
| 2026-09-08 | [Asking what else the crop looks like, which is a question nothing was asking](2026-09-08-what-else-does-it-look-like.md) | none; [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open` with a second deployed remedy owed a held-out drive | deployed at d1aeef9 and proved live: the three merges sit in the worst 3.4% of 923 attachments by rival lead, and the two changes together separate the one fault the replay reproduces at a cost of three things |
| 2026-09-08 | [A measured distance can now stand a thing up on its own](2026-09-08-one-look-can-place-a-thing.md) | none; [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open` | deployed at 012cdf9 and proved live: the daemon holds 127 things, 18 of them asserted by one ranged look, and the spray can that no crossing could place is placed |
| 2026-09-08 | [Phase 1 starts: a record of what the rover decides, with names that survive a clear](2026-09-08-episode-record-starts.md) | [R-AUT-1](../requirements/autonomy.md#r-aut-1), [R-AUT-2](../requirements/autonomy.md#r-aut-2), [R-AUT-3](../requirements/autonomy.md#r-aut-3) new and `settled`; [R-AUT-4](../requirements/autonomy.md#r-aut-4), [R-AUT-5](../requirements/autonomy.md#r-aut-5), [R-AUT-6](../requirements/autonomy.md#r-aut-6) new and `open`; M1 criteria 2, 3, 7 and 8 met | no world-state requirement moved; `autonomy` 168 passed and `world_state` 806 passed on the Orin at 2d492a5, which reports world_generation f49e9206997fbe42 |
| 2026-09-08 | [Navigation was restarted mid-drive, and eighteen looks kept their pictures and lost their directions](2026-09-08-restart-withholds-directions.md) | [R-WS-16](../requirements/world-state.md#r-ws-16) to `settled`; M0 criterion 11 met | nothing deployed; 18 regions across a restart at 07:40:51 recorded with no direction and none placed, and 81 further refusals whose reason the row does not record |
| 2026-09-08 | [The second drive: the geometry is right and a thing can still be two objects](2026-09-08-acceptance-drive-two.md) | none; M0 criteria 3 and 8 fail on a second driven recording, and the candidate remedy is measured and rejected | [R-WS-13](../requirements/world-state.md#r-ws-13), [R-WS-16](../requirements/world-state.md#r-ws-16) `open`; [R-WS-10](../requirements/world-state.md#r-ws-10) `failing`; two targets 2.9 m apart placed 2.899 m apart, depth attribution 69% against 66% |
| 2026-09-08 | [Cutting a rim of frontier into pieces makes the rover pirouette](2026-09-08-rim-frontiers-pirouette.md) | [R-NAV-6](../requirements/navigation.md#r-nav-6) to `failing`; the 2026-09-07 cutting fix reverted | [R-NAV-5](../requirements/navigation.md#r-nav-5) holds -- the cut rim cost 869 degrees of turning for 44 cm, and the cap is inert on a well-mapped room in both directions |
| 2026-09-07 | [The bare-patch fault does not reproduce, and two of my labels were wrong](2026-09-07-no-bare-patches.md) | none; corrects the instances [R-WS-12](../requirements/world-state.md#r-ws-12) was cited on and withdraws the filter that was about to be written for them | one of 43 things on the acceptance recording is arguably a patch of nothing, and it was seen twice; the two named as blown-out wall and window are a ceiling light and a doorway |
| 2026-09-07 | [Both remedies are on the rover, and the depth camera's blind edge is now visible](2026-09-07-both-remedies-deployed.md) | none; deploys the collapse test [R-WS-13](../requirements/world-state.md#r-ws-13) asked for and the per-look coverage counts M0's criterion 2 asks for | proved live at 16621ea: masks cover 40-64% of a crop at 24 ms a look, and a region at 0.95 of the frame width is reported outside the depth camera's view |
| 2026-09-07 | [Masking the crop prevents all four picture-into-chairs merges](2026-09-07-masking-the-crop.md) | none; supplies [R-WS-13](../requirements/world-state.md#r-ws-13) with a remedy that works on all four measured faults, and refutes the frame-coverage gate | nothing deployed; 4 of 4 wrong attachments caught for 7% of the correct ones, at 84 ms on a 550 ms look, with the 0.20 threshold owed a held-out run |
| 2026-09-07 | [The A4 target cannot give the mount a third distance](2026-09-07-no-third-mount-distance.md) | none; the outstanding third-distance mount confirmation is shown to be unobtainable with the A4 sheet, and the headlights make the procedure independent of daylight | the adopted mount is untouched; [R-WS-11](../requirements/world-state.md#r-ws-11) stays `open` |
| 2026-09-07 | [The masks the rover already computes and throws away](2026-09-07-masks-are-already-there.md) | none; redirects [R-WS-13](../requirements/world-state.md#r-ws-13)'s remedy from a depth histogram to the segmentation masks the engine already returns | nothing deployed; the mask separates the objects on three cases at 6 ms a look, but that it fixes the merge is untested |
| 2026-09-07 | [The acceptance drive: depth is much better, identity is not, and the floor is still a thing](2026-09-07-m0-acceptance-drive.md) | none; M0 criteria 2, 3 and 8 fail on a fresh driven recording, and the depth camera's frame coverage is measured for the first time | [R-WS-10](../requirements/world-state.md#r-ws-10) `failing`; [R-WS-12](../requirements/world-state.md#r-ws-12), [R-WS-13](../requirements/world-state.md#r-ws-13), [R-WS-16](../requirements/world-state.md#r-ws-16) `open`; depth attribution 66% against 48% at the baseline |
| 2026-09-07 | [The envelope reaches the tilt the rover actually uses, by 0.08 of a point](2026-09-07-gimbal-tilt20-passes.md) | none; extends the declared gimbal envelope to tilt +20 and corrects the board-distance guidance | [R-WS-10](../requirements/world-state.md#r-ws-10) still `failing` -- the mechanism is in place but no bearing has been measured since; the backlash holds within 0.13 deg across seven sessions |
| 2026-09-07 | [The gimbal's turning at the tilt it actually uses, and why the answer is "not yet"](2026-09-07-gimbal-tilt20-campaign.md) | none; adds a second tilt and a fourth backlash measurement toward [R-WS-10](../requirements/world-state.md#r-ws-10) | the pan gain error scatters 1.08 points across four sessions against a 1.00-point limit, so it is uncertified at either tilt; the backlash spans 0.12 deg and carries |
| 2026-09-07 | [A look only gets a direction from a state that was measured](2026-09-07-capture-state-gates.md) | none; deploys the confirmed-pose gate [R-WS-16](../requirements/world-state.md#r-ws-16) asked for and the gimbal envelope M0's criterion 10 asks for | [R-WS-16](../requirements/world-state.md#r-ws-16) still `open` pending its hardware demonstration; [R-WS-10](../requirements/world-state.md#r-ws-10) still `failing`; 84% of the rover's looks are at a tilt the pan campaign never visited |
| 2026-09-07 | [Where the OAK actually sits, and the arithmetic error that hid it](2026-09-07-p0-oak-mount.md) | none; supplies the OAK-to-gimbal translation [R-WS-11](../requirements/world-state.md#r-ws-11) was half-blocked on, and adopts the mount transform | [R-WS-10](../requirements/world-state.md#r-ws-10) still `failing`, untouched by this measurement |
| 2026-09-07 | [M0 review: bounded calibration and usable operating limits](2026-09-07-m0-review.md) | none; acceptance scope clarified | 748 local software checks passed; no new physical acceptance |
| 2026-09-07 | [Why a refit could not find a rover whose heading was 152 degrees out](2026-09-07-refit-window.md) | none; supplies the measurement [R-WS-16](../requirements/world-state.md#r-ws-16) was owed, and evidence toward [R-NAV-9](../requirements/navigation.md#r-nav-9) | [R-NAV-2](../requirements/navigation.md#r-nav-2), [R-NAV-3](../requirements/navigation.md#r-nav-3), [R-WS-2](../requirements/world-state.md#r-ws-2) |
| 2026-09-07 | [M0 baseline: is semantic state safe enough to steer the rover?](2026-09-07-m0-semantic-world-state.md) | [R-WS-10](../requirements/world-state.md#r-ws-10) to `failing`; [R-WS-12](../requirements/world-state.md#r-ws-12), [R-WS-13](../requirements/world-state.md#r-ws-13) opened with measured detail | [R-WS-1](../requirements/world-state.md#r-ws-1) through [R-WS-9](../requirements/world-state.md#r-ws-9), [R-NAV-4](../requirements/navigation.md#r-nav-4) |

Measurements made before this log existed are in Git history, in the component
READMEs that cite them, and in the bench scripts that produced them. The ledger
starts here rather than being back-filled from memory, because an entry nobody
can reproduce is the one thing this format is meant to prevent.

## Writing an entry

Name the file `YYYY-MM-DD-slug.md`, dated by when the measurement was taken
rather than when it was written up. Then add a row to the ledger above.

An entry should answer, in this order:

1. **What question was being asked, and did it come out yes or no.** Put the
   answer in the first paragraph. An entry whose conclusion is only reachable by
   reading to the end will be read as inconclusive.
2. **What the measurement was.** The recording, the room, the run, the suite —
   enough that somebody could take it again. Name the fixtures and recordings
   that were used, since those are what make it repeatable.
3. **What the numbers were**, including the ones that were unhelpful. Report
   every attempt: successes, unresolved outcomes, refusals and failures counted
   separately, with the full attempt count. Dropping the failures after seeing
   them is how a measurement becomes a claim.
4. **Which requirements moved.** By identifier, with the new state.
5. **What has to happen next**, in order, if anything.

## Rules

**An entry is never edited to agree with later findings.** If a later
measurement contradicts it, write the later entry and let both stand. If an
entry was simply wrong about its own data — a miscount, a mislabelled column —
add a correction note at the top saying what was wrong and pointing at the entry
that supersedes it, and leave the original text alone underneath.

**Say plainly when a milestone does not pass.** A partial pass is a fail with
detail. The 2026-09-07 entry is the model: five of six criteria held, the sixth
failed outright, and the entry's own summary leads with the failure rather than
the five.

**Raw evidence outlives the summary.** Keep enough identifiers, poses, versions
and parameters to rerun the decision offline. Runtime frames and databases stay
out of Git — see [R-PLAT-11](../requirements/platform.md#r-plat-11) — so an entry
references them by identifier and names the recording they came from.

**Separate what was observed from what was predicted.** Replay proves logic.
It does not prove camera geometry, wheel behaviour, USB reliability or stopping
distance. Where a simulation and the rover disagree, the rover is right, and the
entry should say which of the two produced each number.
