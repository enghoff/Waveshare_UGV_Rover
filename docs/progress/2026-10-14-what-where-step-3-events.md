# What/where step 3: the change watch's geometry joined to the camera's appearance names what changed, but on the shakedown finds nothing the watch misses

**Taking where and gone from the lidar's change watch, and joining them to the
camera's frozen appearance vectors, turns the watch's "something changed here"
into "a new thing", "something moved from here to there" or "this left".** On
the shakedown it found the same five of six staged changes as the watch, with
the same two false alarms. It named three of its four events there right, so
by its own test it would not pass on that session. On scene 1 of 2026-10-11 it
found the tipped stool, which the watch logged as small. All of this was seen
while building it. The test is the next staged M5 session, scored by
`score3.py`, which is written and runs. A latent where, tried first, could not
tell moved armchairs from unmoved ones and was dropped at the owner's choice.

## Why where comes from the lidar

Step 3 of [prediction in what and where streams](../plans/what-where-streams.md)
first tried a latent where. Each region got a place: the depth camera's range,
or else the lidar's returns across the region's width, 57% of which land within
0.3 m of the depth range on the controls. Remembered things were retired once
the lidar saw through their places. A region then scored by how much better it
matched something elsewhere than anything remembered within 0.6 m of it.

On scene 1, 14 regions of the armchairs at their second places scored a median
of 0.22, against an unchanged-room threshold of 0.28-0.55, and one or two
passed. What explained them were armchair-like regions remembered from early
October within 0.6 m, older than any recording and so beyond the lidar's reach
to retire. Limiting memory to its last 12-48 hours made the unchanged rooms'
threshold climb to 0.55-0.95, and fewer armchair regions passed, not more. The
lidar seeing through where each armchair had stood flagged both places
strongly. The owner chose to take where and gone from the lidar's geometry and
keep the latent for appearance.

## What step 3 is

- **Geometry:** the change watch (`ros_nav/change_watch.py`), replayed from the
  drive recordings as `change_node` feeds it, supplies NEW and GONE places.
- **Appearance:** the camera's regions at a NEW, from a minute before the lidar
  first saw it until something leaves there, and the regions remembered at a
  GONE.
- **Moved:** a GONE and a NEW 0.5-4 m apart that look alike in looks 20 s
  apart, with each region supporting one move only.
- **New thing:** a NEW, or a place the watch logged nothing furniture-sized at,
  whose regions are new to the house and alike, in looks 20 s apart.
- **Arrived** and **gone:** the watch's other furniture-sized changes.

Codex reviewed the first version and found moves sharing evidence, attachment
mixing scene 1's successive rearrangements, and pictures a second apart
counted as persistence. All three were fixed. Branch `what-where`: `events.py`,
`score3.py` and `step3.md`.

## What it reported, on evidence already seen

The watch replayed over the 11 October recordings and the shakedown in order.
Each event's pictures were looked at.

| Window | Events | What they were |
|---|---|---|
| seven controls | 1 | an arrival in the bedroom with no picture to name it |
| scene 1 | 6 | armchairs moved and arrived, all armchairs; one morning place gone; the stool as a new thing |
| shakedown run | 6 | an armchair moved from the charger armchair's old place to the balcony armchair's new place (identical armchairs, so which went where is not told); the charger armchair's new place as an arrival; both bean bags as new things; a GONE beside the charger armchair's old place; the bedroom's lidar false alarm |
| shakedown after refit | 3 | the red bean bag's second place as a new thing and its first place gone; an arrival near the balcony armchair's old place, not in the staging notes |

Scene 1's pillows (two regions in one look) and pink bucket were not reported.

**The dry run of the test on the shakedown.** The watch's state at the staging
was rebuilt from the 11 October recordings, because the saved one is on the
rover. Each arm was replayed from it and both methods were counted by M5's own
scorer.
- **Watch alone:** five of six found; the balcony armchair's old place missed;
  two bedroom false alarms. That reproduces the shakedown's arm replays.
- **Step 3:** the same five and the same two false alarms. It named three of
  its four counted events right: the charger armchair's new place came out as
  an arrival, because its departure was already paired with the other
  armchair's arrival.
- **Verdict:** below the bar of four in five named right and finding nothing
  more, so step 3 would not pass on this session.

## What it does not show

Everything above was seen while building it. The development replay used the
shakedown's map and left out its bad-pose window, both chosen with hindsight;
the test does neither. The test, fixed in the branch's `step3.md`, asks of the
next staged session that step 3:
- finds every staged change the watch finds;
- finds one more, or names four in five of its events right;
- raises at most one more false alarm in the untouched rooms.

That session needs only its recording extracted and its world store copied
before anything is cleared. No requirement moved.
