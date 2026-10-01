# The heading check is rolled back: it held the wheels for 15 seconds

**The in-move heading check deployed as `85d5909` is reverted (`ffc1585`) and the
rover runs the code it ran before it.** The owner found the regression from the
console. A target clicked while a move was running was dropped with "the move it
interrupted did not let go of the wheels". The check ran inside the move after the
stop, and a check that corrected the heading held the move mutex for about 15 s
while it wrote and reloaded the pose graph. The console waits 6 s for an interrupted
move, and `go_to_thing` waits 3. A step that adds 15 s to a move is not viable
anywhere, so this was not tuned but reverted.

Proved after the revert: a half turn stopped 0.5 s in replied 0.07 s after the stop.
Both suites pass at their old counts on the Orin (ros_nav 544, rover_daemon 983),
and the map came back settled on the same `map_id`.

What stands from the day's measurements is unchanged: turning on the spot leaves the
heading about 7% of each turn out, and a scan-to-map match finds the truth to within
2 degrees ([the measurement](2026-10-01-heading-after-turning.md)). Navigation does not
need the correction, because driving corrects the heading and Nav2 replans. Only the
direction stamped on a look needs it. So whatever replaces this lives in the world
state's path. It measures only, in a fraction of a second, and never moves the rover,
holds the wheels or blocks a command.

## Requirements

None moved. [R-WS-10](../requirements/world-state.md#r-ws-10) and
[R-WS-16](../requirements/world-state.md#r-ws-16) stay `failing`.
