# Step 2: the same streams with navigation's pose, fixed before running

Step 1 ([protocol.md](protocol.md), [results.md](results.md)) saw the
rearranged living room through the few looks whose pose the heading check
confirmed. Step 2 asks how much of its result was that blindness. It changes
two things and nothing else, and this file was committed before it ran.

**1. Poses.** A look the store kept no pose for takes navigation's pose at its
shutter (the frame's `taken_at`) from the drive recording that covers it,
unless the rover was turning faster than 20 degrees a second, where the
store's pose and the shutter disagree by several degrees. Its regions'
bearings are worked out by the store's own arithmetic (`world_state/view.py`),
which reproduces every stored bearing of 1,658 checked exactly. The recordings
are those of the seven controls and scene 1 of 2026-10-11 and the shakedown,
extracted on the rover with `bag_extract.py` on 2026-10-14. Against the
store's own poses for the same looks, navigation's pose at the shutter agrees
to 3.5 cm at the median; heading agrees to 0.5 degrees at the median but by up
to 10 where the rover was turning.

This is navigation's belief, not a confirmed pose
([R-WS-16](../../docs/requirements/world-state.md#r-ws-16)): it was right in
the scored windows by the shakedown's scan matching, except after 08:17:15,
when the rover stood 20 cm out short of the charger. Finds without the looks
after 08:17:15 are reported beside the rest.

**2. Rooms.** A flagged region belongs to the room where the flagged thing
stands, not where the camera stood: its depth-measured point, or else the
nearest lidar return within 3 degrees of its bearing in the scan nearest the
shutter, within 6 m. A flag with neither is unplaced, reported and not counted
either way.

**Unchanged:** the streams, the memory cutoff, the windows, the persistence
and found rules, the bar. The thresholds are derived again on the controls by
the same rule (99th percentile), because the controls' looks now carry
recorded poses too.
