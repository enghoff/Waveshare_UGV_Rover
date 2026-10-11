# Snapshots on the rover: a save is ten seconds and 607 MB, and a restore and its undo leave the rover where it was

**On 2026-10-11 evening the snapshot calls (4d9828a) were deployed and driven on the
rover, parked by the charger with its place on the map confirmed.** A snapshot held
the whole state -- 672 things, 39,107 looks, map `7da19bef3888` and the live pose
to the millimetre -- and restoring it, then restoring the snapshot the restore had
kept, each left the map, the pose and the world counts as they were. A restart of
the navigation stack afterwards came back to the restored map. This moves
[R-PLAT-12](../requirements/platform.md#r-plat-12) and
[R-PLAT-13](../requirements/platform.md#r-plat-13) from open to settled.

## Before

The rover had just been rebooted by the owner. The map came back settled, the
lidar agreeing to 3 cm and 0.0 degrees, and the rover had not moved since the
boot, so the deploy's restart of the navigation stack could lose no mapping. It
came back settled on the same map. The navigation suite ran on the rover with
ROS: 789 passed, 0 failed.

## What was measured

| Step | Took | Result |
|---|---|---|
| save "deploy check" | 9.6 s | 672 things, 39,107 looks, 9,169 pictures linked and none copied; the map's identity and pose equal to `nav_status`'s |
| of which the world store | 2.4 s | `world.db` 343 MB |
| of which the graph | about 7 s | `map.posegraph` 163 MB and `map.data` 101 MB |
| restore it, same map | 25.5 s | rover kept where it was (moved 1 mm, 0.0 deg), settled; 672 and 39,107 back; "before restoring deploy check" kept |
| restore that undo snapshot | 24.9 s | same map, settled, 672 and 39,107 |
| navigation restarted after | -- | same map, settled, lidar agreeing to 3 cm and 0.0 deg; world on session 67 and still recording |

A picture in the snapshot was the same file as the store's (one inode), so the
snapshot costs its database and its graph: 607 MB, with 801 GB free. A restore is
about ten seconds of keeping the state it replaces and the rest the graph being
read back. The saved map on disk, 7.7 hours old before the restore, was 23 s old
after it, because a restore installs the snapshot's graph as the saved map.

The console, served on the rover, listed the snapshot with its map, pose, counts
and size, opened from the map card's new button in a headless browser and closed
again.

## Not shown on the rover

A restore onto a *different* map -- the rover put where the snapshot stood, the
saved map replaced under another identity -- was not driven: it needs a second map,
and making one means clearing the owner's. It is covered offline
(`ros_nav/test_snapshot.py`, `rover_daemon/test_snapshots.py`). The two undo
snapshots were deleted; "deploy check" is left on the rover.
