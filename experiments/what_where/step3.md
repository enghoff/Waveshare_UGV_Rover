# Step 3: events that join the lidar's geometry to the camera's appearance

The owner chose on 2026-10-14 to take where and gone from the lidar rather
than push a latent where stream, after development showed the latent version
could not separate moved armchairs from unchanged ones
([step3-development.md](step3-development.md)). This file says what step 3 is,
what it did on the evidence it was developed on, and the test it faces next.

## What it is ([events.py](events.py), [step3.py](step3.py))

- **Geometry** is the change watch the rover runs (`ros_nav/change_watch.py`),
  replayed from the drive recordings as `change_node` feeds it, with the map
  gate set from the shakedown's map. GONE is a place solid before and seen
  through now; NEW the other way round. Changes of one kind within 0.5 m in a
  visit are one change.
- **Appearance** is the what stream of step 1 (frozen DINOv2, threshold 0.3216
  from the controls). A NEW takes the furniture-sized regions standing within
  0.5 m plus half its extent of it in the visit, placed by depth or the lidar;
  a GONE takes the 20 newest remembered regions that stood there.
- **Events:**
  - *moved*: a GONE and a NEW 0.5-4 m apart whose regions look alike (best
    cosine 0.7 or more), most alike first, each change used once, the NEW not
    new to the house;
  - *new thing*: a NEW whose regions are new to the house in two looks; or,
    away from any NEW, regions new to the house in two looks within 0.6 m of
    each other and alike (0.5), which the lidar called small or did not see;
  - *arrived* and *gone*: the watch's other furniture-sized NEW and GONE.
  Small watch changes are reported only when the camera confirms a new thing
  at them.

Looks without a stored pose take navigation's pose from the recording, as in
[step 2](step2.md).

## Development (2026-10-14)

All on evidence already seen, so none of it is a test. The watch was replayed
over the seven controls and scene 1 of 2026-10-11 and the shakedown, in order,
without the shakedown's bad-pose window; its history therefore starts at
control 1, not where the live watch's did. Each event's regions were looked at.

| Window | Events | On the pictures |
|---|---|---|
| seven controls | 2 | a bright patch of floor as a camera-only new thing (control 1); an arrival in the bedroom with no regions to show (control 5). Both false by the controls' definition |
| scene 1 | 6 | armchairs moved, to the first new place and to a second place, and arrived, all armchairs; the tipped stool as a new thing the lidar had called small; one "move" from a patch of floor by the corridor door, false |
| shakedown run | 6 | both armchairs' new places as moves from where an armchair stood (which armchair is not told apart: they are identical); both bean bags as new things; the charger armchair's old place gone; the bedroom's lidar false alarm, as the live watch logged it |
| shakedown after refit | 4 | the charger armchair moved; the red bean bag at its second place as a new thing and its first place gone; an armchair near the balcony armchair's old place, not in the staging notes |

On the shakedown's staged changes the run reports five of six: both new places,
both bean bags and the charger armchair's old place. The balcony armchair's old
place is missed, as in step 2. Scene 1's pillows (two regions in one look) and
pink bucket were not reported.

What step 3 adds to the watch alone, on this evidence: it says what each change
is (new to the house, or a known thing moved), and it found the stool, which
the watch logged small. What it costs: camera-only new things bring their own
false alarms (one in seven controls), and moves between identical things cannot
say which went where
([R-WS-2](../../docs/requirements/world-state.md#r-ws-2)).

## The test: the next staged M5 session

Fixed here before that session is recorded. The session, the staging list, the
counting rules and the room-controls are M5's
([the decision](../../docs/decisions/m5-sessions-are-counted-before-they-start.md));
the watch's own count on the same session is the baseline, replayed the same
way as step 3's geometry so that both see the same scans.

- **Found**: a staged arrival when a moved, new thing or arrived event lies
  within 1 m of it; a staged departure when a moved (its from) or gone event
  does; both counted in the first step after the staging that came within reach.
- **Named**: an event is named right when a staged move is reported as moved
  and a staged addition as a new thing. Which of two identical things moved is
  not asked.
- **False alarm**: an event more than 1 m from every staged place in a
  room-control, judged by where the event stands, or in the staged room away
  from every staged place, counted separately. Persistence is the watch's and
  the two-look rule's.

**Step 3 passes** if it finds every staged change the watch finds; it finds at
least one the watch does not, or names at least four in five of its events
right; and it raises at most one false alarm in the room-controls more than the
watch does. Otherwise it showed no benefit on that session.

The session's recording must be extracted with `bag_extract.py`, and the
world store copied through SQLite before anything is cleared.
