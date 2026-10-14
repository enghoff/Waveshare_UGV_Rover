# Step 3: events that join the lidar's geometry to the camera's appearance

The owner chose on 2026-10-14 to take where and gone from the lidar rather
than push a latent where stream, after development showed the latent version
could not separate moved armchairs from unchanged ones
([step3-development.md](step3-development.md)). This file says what step 3 is,
what it did on the evidence it was developed on, and the test it faces next.

## What it is ([events.py](events.py), [step3.py](step3.py))

- **Geometry** is the change watch the rover runs (`ros_nav/change_watch.py`),
  replayed from the drive recordings as `change_node` feeds it: every third
  scan received, none taken turning faster than 40 degrees a second, a report
  every 30 s and one when the scans end, only news logged. GONE is a place solid
  before and seen through now; NEW the other way round. Changes of one kind
  within 0.5 m whose evidence falls within one visit (the watch's 300 s gap)
  are one change. Navigation's own trust in the pose is not in a recording, so
  the replay does not apply it.
- **Appearance** is the what stream of step 1 (frozen DINOv2, threshold 0.3216
  from the controls). A NEW takes the furniture-sized regions within 0.5 m plus
  half its extent of it, placed by depth or the lidar, from a minute before the
  watch first saw it until a GONE is first seen at the same place or the visit
  ends. A GONE takes the 20 newest remembered regions there, and any seen there
  earlier in the visit.
- **Persistence** is the watch's own 20 s: camera evidence counts only from
  looks at least 20 s apart.
- **Events:**
  - *moved*: a GONE and a NEW 0.5-4 m apart, one of them furniture-sized, whose
    regions look alike (cosine 0.7 or more) in looks 20 s apart; most alike
    first, and each region, seen or remembered, supports one move only;
  - *new thing*: a NEW whose regions include a group alike to one another
    (0.5) and new to the house, in looks 20 s apart; or, away from every NEW,
    such a group within 0.6 m, which the lidar called small or did not see;
  - *arrived* and *gone*: the watch's other furniture-sized NEW and GONE.
  Small watch changes are reported only when the camera confirms a new thing
  or a move at them. Groups are connected components, whatever order the
  regions come in.

Looks without a stored pose take navigation's pose from the recording, as in
[step 2](step2.md).

## Development (2026-10-14)

All on evidence already seen, so none of it is a test. For development the
watch was replayed over the seven controls and scene 1 of 2026-10-11 and the
shakedown, in order, with the shakedown's map gating every scan and without the
shakedown's bad-pose window (both chosen with hindsight). Its history starts at
control 1, not where the live watch's did. Each event's regions were looked at.
Codex reviewed the first version (`ugv-rover-25-ww-codex-2.md` in
`tmp/codex-assistance-replies/`); the rules above are the second, after its
review: one region one move, attachment bounded in time, persistence 20 s
apart, groups as components, the replay's scan count and last report.

| Window | Events | On the pictures |
|---|---|---|
| seven controls | 1 | an arrival in the bedroom (control 5) with no regions to show, false by the controls' definition |
| scene 1 | 6 | an armchair moved from its first new place to a second; another moved; both armchairs' first new places as arrivals; the morning place of one gone; the tipped stool as a new thing, which the watch had logged small |
| shakedown run | 6 | an armchair moved from the charger armchair's old place to the balcony armchair's new place (the two are identical, and which went where is not told); the charger armchair's new place as an arrival; both bean bags as new things; a GONE beside the charger armchair's old place; the bedroom's lidar false alarm |
| shakedown after refit | 3 | the red bean bag's second place as a new thing and its first place gone; an arrival near the balcony armchair's old place, not in the staging notes |

Scene 1's pillows (two regions in one look) and pink bucket were not reported.
The first version's camera-only false alarm in control 1 (a bright patch of
floor, two pictures a second apart) and its false move by the corridor door
are gone under the 20 s persistence and one-region-one-move rules.

## The test: the next staged M5 session

Fixed here, and executable as [score3.py](score3.py), before that session is
recorded. The session, the staging list, the counting and the room-controls
are M5's ([the decision](../../docs/decisions/m5-sessions-are-counted-before-they-start.md),
`experiments/m5_changes/score_session.py`); both methods are counted by its
`score`, from the same scans.

- **Inputs:** the session's record, its staging list, its recording through
  `bag_extract.py`, a SQLite copy of the world store taken after the session
  and before anything is cleared, its `nav_grid` reply and the watch state
  `session.py` saved after the wait.
- **Replay:** each loop or run after the staging is replayed on its own from
  that saved state, as `arm_replay.py` does, with a last report at its end.
  The watch alone is its furniture-sized changes; step 3 is its events from
  the same changes. No scan is left out by hindsight: a session that lost its
  place stops counting where `session.py` stopped it, as M5 does.
- **Appearance:** memory is every region stored before the staging began (the
  wait's start); a step's regions are those stored within it; looks without a
  stored pose take navigation's pose from the recording unless turning faster
  than 20 degrees a second.
- **Found**, both methods: M5's rule, a change of the right kind within 0.6 m
  of the staged place in the first loop or run after the staging that came
  within the lidar's reach of it. For step 3 every event counts as
  furniture-sized, a move as a GONE where it left and a NEW where it arrived.
- **Named**, step 3: of its events within 0.6 m of a staged place in that
  place's counting step, a move is right when it left from within 0.6 m of a
  staged move's old place and arrived within 0.6 m of a staged move's new
  place (which of two identical things is not asked); a new thing when within
  0.6 m of an addition; arrived and gone are never right. No such events means
  naming fails.
- **False alarms**, both methods: M5's rule, furniture-sized changes (for step
  3, events, both ends of a move) in a room-control, not at a door.

**Step 3 passes** if it finds every staged change the watch finds; it finds at
least one more, or names at least four in five of its counted events right;
and it raises at most one more false alarm in the room-controls than the
watch. Otherwise it showed no benefit on that session.

### Dry run on the shakedown

`score3.py` run on the shakedown with the watch state rebuilt from the 11
October recordings (`--history`; the saved state is on the rover), as a check
that the scoring runs, not as a test
([score3_dry_run_shakedown.json](score3_dry_run_shakedown.json)). The watch
alone found five of six, the balcony armchair's old place missed, with two
false alarms in the bedroom, one per arm: the arm replays of the shakedown's
entry, reproduced. Step 3 found the same five with the same two false alarms,
and named three of its four counted events right: the charger armchair's new
place came out as an arrival, its departure already paired with the other
armchair's arrival. **So on the shakedown step 3 would not pass**: it names,
but finds nothing the watch misses.

## The test, scored: M5 session 1 (2026-10-14)

[score3_session1-1014.json](score3_session1-1014.json). The watch alone found
six of seven staged events, step 3 the same six; step 3 named one of its four
counted events right; false alarms five and six. **Step 3 fails.** Two of the
scorer's rules inflated both methods' counts alike: the bean bag's arrival was
counted for the dining chair beside it too (M5's own live count missed the
chair, 0.64 m away), and the run, replayed on its own, re-found the staged
changes in a room M5 counts as a control by then. The reasons it named so
little, and what changes before a next test, are in
[the session's entry](../../docs/progress/2026-10-14-m5-session-1.md).

## Version 2, after session 1 (2026-10-14), for session 2

Developed on session 1, the shakedown, the controls and scene 1, all now seen;
the test is M5 session 2. What changed and why:

- **Names are asked over the whole session.** Finding stays M5's (the first
  step after the staging); naming counts, of the staged changes step 3 reported
  anything at, how many it named right at least once. On a loop the camera
  looks where the rover drives; the run is what turns it to a change. In
  session 1 the run went to look at all five reported changes in its first
  90 s and named both armchair moves; loop B had seen the arrivals once or not
  at all.
- **"New" is new to this place, for a furniture-sized arrival**: its regions
  are unlike what the newest five remembered looks that viewed the place had
  within 1 m of it. Session 1's bean bag had been seen two days before, so it
  was familiar to the house but new here. A small arrival must still be new to
  the whole house: on the controls, new-to-the-place turned three small lidar
  blips into new things.
- **Moves are paired before novelty is asked**, except for an arrival new to
  the whole house, which cannot have moved here (on the shakedown the black
  bean bag otherwise paired with an unrelated departure).
- **An arrival's pictures end at a departure from the same place** (0.5 m),
  not one anywhere within its radius: session 1's armchair moved 0.57 m, and
  its own old place cut its pictures off.
- **Step 3 names what the watch currently reports** during an arm, not only
  what is news in it, and counts as news only events not reported in an
  earlier arm or logged by the watch before the arm.
- **The scorer** replays everything after the staging in one piece, as the
  live watch ran, and matches each staged event to one change only. Session 1
  under version 1's scorer found 6 of 7 for both methods with 5 and 6 false
  alarms; the live log, and version 2's scorer, give 5 of 7 and none.

| Development check | Watch found | Step 3 found | Named right | False alarms, watch / step 3 |
|---|---|---|---|---|
| session 1 | 5 of 7 | 5 of 7 | 6 of 6 | 0 / 0 |
| shakedown (dry, history rebuilt) | 5 of 6 | 5 of 6 | 6 of 7 | 1 / 2 |
| seven controls (`step3.py`) | | | | 1 event in all |

Both sessions pass on version 2, which says only that it was fixed against
them. The pass rule for session 2 is unchanged: every staged change the watch
finds, and one more or four in five named right, and at most one more false
alarm.

## The test, scored: M5 session 2 (2026-10-14)

Version 2 (`0293ecb`, committed 12:22) on session 2 (12:39-12:55),
[score3_session2-1014.json](score3_session2-1014.json). The watch alone found
two of seven staged events, step 3 five: both moves, from small departures the
camera matched with what arrived, and the step ladder by the camera alone. It
named all it reported right, and its one counted false alarm is the watch's, a
door the household opened. **Step 3 passes.** The write-up, with what it does
not show, is [the session's entry](../../docs/progress/2026-10-14-m5-session-2.md).
