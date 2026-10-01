# The heading check is on the rover: within 2 degrees of the tape, at a price

**After it turns, the rover now checks its heading against the map, and against
the tape it stays within 2 degrees.** The same 24-turn sequence that left the
heading 23 to 43 degrees out without the check
([the measurement](2026-10-01-heading-after-turning.md)) measured +0.2, +1.7, -1.1,
+1.4 and +1.5 degrees at the five steps where a target was in view. Deployed as
`85d5909` (ros_nav and rover_daemon, both suites passing on the Orin: ros_nav 571,
rover_daemon 986). It was described in `ros_nav/README.md` and `ros_nav/posecheck.py`
at that commit; both went with the
[rollback the same day](2026-10-01-heading-check-rolled-back.md). Pictures and poses are in
`captures/2026-10-01-heading/turn4/`.

Two things it costs, both measured, neither yet dealt with:

- **A check that moves the rover takes seconds.** It writes the pose graph and
  loads it again, which is the only way slam_toolbox in mapping mode can be put
  somewhere. The 24-turn sequence took 494 s with the check against 110 s without:
  about 15 s per corrected turn, added to the move's reply. A check that finds
  nothing to correct, or is refused, costs a fraction of a second.
- **Close to a wall, the check is often refused.** Facing the wall beside the
  cabinet, four of six checks found the best fit with 88 to 90% of the scan on a
  wall, against the 90% `refit.fit` requires, while the morning's refits at the
  same spot scored 95 to 97%. Refused, the pose stays unchecked and the world state
  takes no bearings, which is the safe direction. The next check that fits clears
  it: one found 13.4 degrees accumulated across two refusals and corrected it to
  1.0. Why the scores fell is not established. The candidates are people near the
  rover, or the map having taken in scans at the wrong heading during the two
  uncorrected turning runs earlier the same afternoon.

The carried-rover half (R-WS-16) is deployed but not demonstrated: no rover was
carried after it went on.

## Requirements

None moved. [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing` until
a drive shows its bearings within tolerance, and
[R-WS-16](../requirements/world-state.md#r-ws-16) stays `failing` until a carried
rover is shown to lose its bearings.

## Next

- Find why slam_toolbox's own continuous matching gets a turn 7% wrong. If that
  is fixed, the check rarely has anything to correct, and the 15 s goes with it.
  The gyro scale is the cheapest place to start: it reads a turn 12% short, and
  the matcher starts from it.
- Find why fits near a wall score just under 90%: a stationary refit with nobody
  in the room tells people apart from a damaged map.
