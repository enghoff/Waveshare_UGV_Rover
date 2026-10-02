# Depth read from the picture's own moment, and the heading checked on the move

**Both are on the rover. A look now reads the depth frame taken within a few hundredths
of a second of its picture. A moving look now gets a heading check of its own, and those
checks follow the heading drift that turning leaves behind.** On a short drive, 14 of 38
regions took their direction from a check made on the move, and only 2 ranges were
dropped for turning. Today's earlier six-minute run had dropped 347. Neither change has
been measured against the tape yet. [R-WS-10](../requirements/world-state.md#r-ws-10)
stays `failing`.

Deployed as `86222ef` (oak_depth, world_state, rover_daemon) and `2d6d912` (ros_nav,
world_state, rover_daemon). On the Orin, world_state 1006, rover_daemon 1050 and the depth
history's own check pass, and ros_nav passes 555 at the desk. Before navigation was
restarted, the rover was turned in place so that the map graph saved: it had been 86
minutes stale. Asked for 25 degrees, the rover turned about 60. After the restart the map
came back as the same map (`f6e304df483b`) and the position was confirmed. The world
state stayed on map session 67. Drives, samples and scripts are in
`captures/2026-10-02-drive-matching/`.

## Depth from the picture's moment

The depth service keeps three seconds of frames and answers `/depth.raw?at=` with the frame
taken nearest that moment. On the rover, asked for one second earlier, it returned a frame
taken 4 ms from it. Asked for now, it returned its newest frame, which was 136 ms old:
the camera's own delay. That confirms the device's stamps are on the host's clock. On the
proving drive, both depth maps kept were 23 and 31 ms from their pictures. 7 regions were
ranged, all from their outlines, 2 were dropped for turning and 19 were outside the depth
camera's view. A range is now dropped for turning only above 33 degrees a second. Below
that, the region is turned by the rover's turn between the picture and the frame before
the depth under it is read.

## The heading on the move

The navigator was sampled four times a second through a second drive out and back. There
were 71 samples. 20 were trusted checks on the move. 34 were refused because the sweep
turned faster than 30 degrees a second: the in-place turns of `drive_to` reach 169. Each
check took a median 0.24 s, and a stop mid-move answered at once.

The corrections are not a timestamp error, which would follow the turn rate. They are
-12 and -13 degrees at under 4 degrees a second. **They follow the drift a turn leaves**:
-9 to -13 degrees after the outbound turn, falling to -2.5 as the rover drives, then +13
to +15 after the turn home, falling to +3. Parked afterwards, the navigator agreed with
the map to 0.5 degrees. That is the over-count on turning measured on
[2026-10-01](2026-10-01-heading-after-turning.md), and the navigator correcting itself once
it drives. Consecutive checks step by a median 4 degrees. Part of that is the navigator's
own steps as it corrects, and the rest is the check's scatter.

## What this does not show

- **Whether a moving check is right, against the tape.** Its scatter suggests a single
  check is worth 3 to 4 degrees. That beats the 10 to 15 degrees the navigator is out
  after a turn. It is worse than leaving the navigator alone once the navigator has
  caught up on a long straight drive, and nothing here tells the two cases apart yet.
  A drive past the taped targets is the test.
- **Ranges on a drive with the depth camera awake throughout.** The camera had been
  asleep and was still waking for the first looks, so the proving drive ranged only 7
  regions.

## Requirements

None moved. [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`.
