# Rotation feedback is frozen while navigation keeps requesting turns

The owner's report of prolonged spinning has a concrete sensor-feedback fault
behind it. At rest the driver board supplies fresh telemetry and incrementing
sample counts, but all nine IMU fields are identical across the recorded full
readings. Raw `gz` and accumulated `gz_lsb_s` are exactly zero. Navigation still
marks the position trusted while its own independent lidar check places the
heading 106.5 degrees away. STOP was sent; no movement command was issued during
this diagnosis. R-WS-16 remains settled for world observations, whose separate
scan check refuses bearings, but that does not establish safe navigation.
R-WS-13 remains open; R-WS-17 and R-WS-18 remain proposed.

This is a more specific diagnosis than the preceding
[localization-blocked evidence drive](2026-10-05-evidence-drive-blocked.md).
That drive's zero reported rotation and changed camera scene were an earlier
clue to missing turn feedback. They were initially reported only as localization
failure. The later logs contain explicit turns and recovery turns canceled by
STOP; with no gyro rotation, the odometry used by Spin cannot measure completion.
The action still has an existing time allowance; the owner's apparent indefinite
spinning does not establish that its timeout is absent or broken.

The three-second board capture contains 148 streamed messages, six full telemetry
messages and 59 new underlying sensor samples. Every full IMU tuple is
`(10094,-25918,-32479,18176,4320,0,5366,15810,32655)` in the order
`ax,ay,az,gx,gy,gz,mx,my,mz`. This reproduces frozen feedback without turning
the rover. Evidence is archived under `captures/2026-10-05-spin-diagnostic/`:
`board-still.json`, `gyro-bias-history.json`, and `navigation-log-tail.txt`.
The precise cause inside the sensor, its bus or the driver-board firmware is
unproven. No shared I2C bus was probed or modified.

Runtime motor/control file hashes match the current committed source for
`board_link.py`, `board_bridge.py`, `base_node.py`, `drive_mixer.py` and
`nav_moves.py`. The recorder deployments changed instrumentation and restarted
the daemon; they did not change these control files. Today's last preceding
nonzero bias log is at epoch 1791192810 (0.429 degrees/second). The current boot's
first zero bias log is at 1791194780, before the recorder deployment and first
stationary recording. This points to a fault beginning at that boot; historical
bias logs alone do not prove which internal IMU operation failed.

A full power cycle was requested, with a stationary feedback check prepared for
when the owner confirms boot. Recovery has not been observed and a further drive
has not been authorized in this diagnostic. After feedback recovers, localization
also needs confirmation before movement. A software guard against frozen feedback
remains to be designed and reproduced against both failed and healthy recordings;
no such guard or speculative navigation fix was deployed here.
