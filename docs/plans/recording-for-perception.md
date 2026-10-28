# A recorder for Perception's sessions

Status: **proposed 2026-10-28; nothing built.** Perception asked for it and for
answers to eight questions; the answers are below, from the code and a first
reading on the rover the same day
([progress](../progress/2026-10-28-what-a-recording-would-cost.md)). The owner
chose the bedroom as the held-out room the same day
([decision](../decisions/the-bedroom-is-held-out-for-perception.md)). Waiting on
the schema's agreement; nothing on the rover is needed before the recorder is
built.
([R-CTL-13](../requirements/control.md#r-ctl-13) proposed;
[R-CTL-6](../requirements/control.md#r-ctl-6) needs a session exception.)

Perception (`D:\Perception`, the owner's perception research) asks for stationary
trials with the gimbal moving and the chassis still, later driven sessions, and a
recorder that writes every stream on one clock to a contract of its own:
`D:\Perception\docs\rover-request.md` (what it asks overall),
`D:\Perception\docs\rover-drive-spec.md` (the recorder's contract) and
`D:\Perception\docs\rover-schema-v1.json` (the schema, drafted 2026-10-28, revised
at `48efd89` in that repository). The recorder is this project's code, built and
deployed under this project's rules; Perception reads what it writes and never
connects to the rover.

## The eight questions, answered

**1. One camera feed for every caller, its stamps and exposure.** Not today, and
buildable. One continuous feed exists, but only face tracking opens it and it
hands each frame to a single reader; every other picture (the console every 0.5-2
s, world-state looks, the model's tools) is a one-shot grab that opens the device
and cannot run while a feed holds it. The recorder needs the daemon to keep the
feed open for the whole session and serve every caller from it, the recorder
taking every frame and the others the newest. Holding the feed costs about a
tenth of a core and leaves navigation alone (parked, measured 2026-10-28). Each
frame carries the driver's buffer stamp on the monotonic clock, which the driver
labels start of exposure; for this USB camera that label is not evidence of when
the exposure began, so the camera's lag is measured by the spec's gimbal steps.
Exposure is not available per frame: the camera has no gain control at all, and
in automatic mode nothing reports the exposure in force. In manual mode the
recorder can state the value it set. The camera runs at 30 frames a second only;
10 a second is every third frame.

**2. One OAK pipeline for depth and the IMU.** Yes, with a change: the depth
service already owns the OAK and builds no IMU node; the IMU (a BMI270, raw
accelerometer and gyro at 200 a second, which the 2026-10-09 sweeps recorded at
201.2 a second with no gap in 450 s) is added to the same pipeline with its own
reader. They have never run together. Two things stand in the way of a
continuous IMU: the daemon switches the OAK off 30 s after the rover stops
(R-CTL-6), so a session must hold it on; and each wake costs 4-6 s and restarts
the device's clock. Depth is 320x180 millimetres aligned to the OAK's colour
camera, at 15 a second fixed when the pipeline is built; 5 a second is every
third frame.

**3. Raw lidar sweeps and raw board values.** Neither is kept today, and both can
be. The published scan is 360 directions with the nearest return in each, no
intensities, stamped when the next revolution begins, and deskewed in place. Only
the lidar node can read the sensor (it holds the serial port), so the raw
47-byte packets -- about 417 a second, 12 returns each with an intensity, a start
and end angle, and the sensor's millisecond clock -- are recorded there. Each
return's angle is interpolated between its packet's two angles, which is all the
protocol gives. A sweep runs clockwise from the rover's left, about 500 returns
at 10 a second. The board's feedback is about 20 lines a second (111,484 in 5,613
s on 2026-10-15; comments saying 17 are stale) of raw integers -- per-track
encoder counts, accelerometer, gyro, magnetometer, pack voltage -- with no time of
the board's own. The daemon reads them in drains every 20 ms (the board bridge
pumps at 50 a second) and today keeps only the newest; the recorder takes every
line at the drain, so a line's arrival stamp is its drain's, up to 20 ms late,
and the odd two lines drained together share one.

**4. Cost on the Orin.** First reading, parked, with navigation and the localizer
running: the board 42.6% busy of six cores, 3.5 GB of memory available of 7.5,
788 GB free on the NVMe. The feed adds about 0.13 of a core and no change in
navigation's rates. The recording itself is estimated at about 1 MB/s at the
base configuration (camera 10 a second at about 30 kB, depth 5 a second at 115 kB,
lidar as JSON, the rest small), about 1.5 GB for 25 minutes, and about 2.6 MB/s
at camera 30 and depth 15; the disk takes either without effort and holds
hundreds of sessions. What the recorder's own processes cost, and whether
navigation holds its rates while it writes and while driving, is commissioning's
measurement.

**5. What stays on.** On: the daemon (the board link and its stops, the run
watchdog, power holds, the battery log); the base node and its board bridge
(odometry, and the motor stop when commands cease); the lidar node, localizer,
mapper, Nav2 and the navigation bridge, which every drive and every stop goes
through; the depth service, which becomes the OAK pipeline; the console (the
owner's STOP and the "keep it on" card); networking; the boot-time GPU repair.
Off: the voice model, world-state perception and its looking loop (it takes
pictures and writes looks into the world store), the visit model's job, face
tracking, autonomy (no run opened) and the shadow runner. Never during a session:
the gamepad or the board's own web page, which write to the board around the
daemon and so around its command log.

**6. ROS's clock.** Yes, the system's wall clock, with no simulated time; seen
live on 2026-10-28. Every stamp is the moment of arrival moved by a fixed
allowance -- odometry 48 ms back, a scan 0.1 s back, the localizer's correction
0.1 s ahead -- not a sensor's time. The wall clock reads 1970 for about 75 s
after a cold boot, so a session starts after it is set, and the recorder writes
the monotonic, wall and ROS clocks together once a second rather than only at the
start and end.

**7. OAK drops since the reseat.** One: 2026-10-13 at 13:26, while the rover was
driving, the device crashed and depth stopped for 5.9 s before the service
reopened it; that is in about 13½ hours of streaming since 2026-10-03, and none
parked. Before the reseat it dropped dozens of times. One event is not a rate.

**8. Manual exposure for a session, and the dimmest room.** Yes, once the daemon
holds the feed for the session: the code that hands exposure back to automatic
runs when a feed opens and in a one-shot grab's recovery from a white or dark
picture, and in a session neither need happen -- the feed opens once, manual is
set after it, and nobody takes one-shot grabs. A session mode keeps manual and
records every change. Manual exposure runs 5 ms to 1 s in 1 ms steps; above 33
ms it slows the frame rate. No exposure for any room is recorded here, only
picture brightness, so the settings for the dimmest room are commissioning's to
measure: manual exposures from 5 to 33 ms in that room, gimbal still and at the
schedules' speeds, reading brightness and sharpness.

## The schema's pending items, answered

**The daemon's command log.** JSON lines, written by the daemon at the one place
every write to the board's serial port passes (the board link's `send`, under its
write lock, so the log's order is the order the board received them), flushed
per line. A header like the streams', `"stream": "daemon_commands"`; then one line
a write: `daemon_seq` (continuous for the daemon's life; a restart writes a
restart line and begins a new epoch), `t_arr` (monotonic nanoseconds when the
write returned), `ok` (the write succeeded), `client` (`daemon`, or `bridge` for
what the base node and the calibration scripts send through the loopback bridge)
and `line` (the exact text written, without its newline). It covers every write:
gimbal (T:133, T:134), motors (T:11), the firmware heartbeat (T:136), lights,
probes and the rest. A `gimbal_cmd` is a line with T:133 or T:134, a
`chassis_cmd` one with T:11, each carrying that line's `daemon_seq`; the other
lines have no stream. The session's copy is the slice between its markers. Two
limits: the log and the streams come from the same write, so matching them checks
the recorder's queues rather than the board, which acknowledges nothing; and
writes over the board's own Wi-Fi page are not in it.

**The calibration, entry by entry.** Each cites its file, commit and date of
measurement:

- *fisheye* (`face_tracking/lens.py`, `05e046b`, 2026-09-30): equidistant with an
  even polynomial, angle from the axis = r · s · (1 + b1·u² + b2·u⁴), with r pixels
  from the centre and u = r/(width/2); at 640x480 s = 12.637 arcmin a pixel, b1
  -0.0578, b2 +0.0531, centre (323.3, 225.6); rays x right, y down, z forward;
  about 134 by 100 degrees. Checked against tilt steps read off gravity, 0.998 of
  their size. The elevation bias at the rest tilt of 20: 4.9 degrees subtracted,
  2.2 degrees one sigma (`world_state/locate.py`, `59c7e43`, 2026-10-02).
- *gimbal* (`rover_daemon/rover.py`, `face_tracking/aiming.py`): commanded in whole
  degrees, one commanded degree a degree of servo (4095 counts a turn); pan
  positive right, tilt positive up; limits pan ±180, tilt -30 to +90; rest pan 0,
  tilt 20; every move to rest arrives from 5 degrees below, settling max(0.6, 0.5
  + swing/200) s (`c8cb0fb`, 2026-10-05; `583756b`, 2026-10-09); backlash 1.6-1.8
  degrees. No gain correction is applied. Measured within ±20 degrees: gain error
  -0.34% and +0.48% at tilt 0, -0.92% and -1.04% at tilt 20 (2026-09-07); across
  the travel 0.9985, within 0.59 degrees inside ±150, inconclusive by its rule
  (2026-10-09).
- *oak_mount* (`world_state/oak.py`, `05e046b`, 2026-09-30), in the gimbal camera's
  frame: yaw +0.37 degrees (positive right), pitch +0.44 (positive up), roll
  -0.75; forward 0 m (held, not fitted), left -0.004 m, up +0.045 m; good to about
  a tenth of a degree and a centimetre. The OAK colour camera at 640x360 as a
  pinhole: fx 500.30, fy 500.17, cx 321.23, cy 190.69 pixels
  (`oak_depth/colour_lens.py`, `c4c2b67`); depth at 320x180 is half that.
- *oak_imu_axes*: **not available.** Nothing here encodes the IMU's axes against
  the camera; the sweeps projected the gyro onto measured gravity and needed none.
  What is known is the scale: the gyro over-reads pan by 1.8% (1.01824,
  `usb_cameras/pan_candidate_2026-10-09.json`, `038ff66`) and reads tilt 3.8% low
  (2026-09-30, in the report only). The axes come from the OAK's own calibration
  or the first stationary trial.
- *chassis_transforms*: `base_link` is the lidar's centre (base_link to laser is
  the identity, `ros_nav/base_node.py`, `9e3fe73`); the lidar's zero points left
  and it sweeps clockwise (`lidar_slam/slam2d.c`). The camera's lens is about 2 cm
  ahead of the lidar's centre on the centre line and 23.5 cm above the floor at
  tilt 0, the lidar's plane about 20 cm (taped 2026-10-01, in documents only). The
  board IMU's orientation is not recorded; a positive `gz` is a turn to the left.
  There is no URDF.
- *odometry* (`~/ugv/odometry.json` on the rover, runtime state outside git, so
  cited by its hash): 16.14613 gyro units per degree a second (re-measured
  2026-10-10, `cfcbe6f`), 107.206 encoder counts a metre on the mean of the two
  tracks (2026-08-31), stamped 48 ms before arrival (`35230e6`, 2026-10-10); no
  track width (heading is the gyro's alone) and no slip factor.
- *board_scales*: `odl`/`odr` raw per-track counts, zeroed when the board reboots;
  `gz` 16.14613 units per degree a second as measured, `gx`/`gy` unmeasured
  (nominally the same); `ax`/`ay`/`az` about 8,192 units a g, read off `az` at
  rest and not measured (the base node's comment calling them milli-g is wrong;
  nothing reads that topic); `mx`/`my`/`mz` with no known scale; `v` hundredths of
  a volt; `L`/`R` not used here and their meaning not established.

**The gimbal's speed and acceleration** are the board's own units. Today the
daemon always sends T:133 with SPD 0 and ACC 0, which means as fast as the servos
go, in whole degrees. Slower moves can be asked of the board two ways -- T:133's
SPD and ACC, never used or measured here, or T:134's per-axis speeds in servo
counts a second (11.375 a degree) -- and the speed the servos actually reach has
to be measured from frames at commissioning, so the board's integers are recorded
with 0 meaning unlimited, and their measured meaning goes in `inputs.json`.

**The localizer's status reduced to a fit.** The localizer matches every scan
(5 a second); parked, it searches one step in five, and above 60 degrees a second
of turning it does not search at all. The fit, frozen before any data is read:
*lost* when the localizer has no map, the map's restore has not been confirmed
(`map_settled` false), the gyro fault is latched, or no step has been searched in
the last 2 s; *poor* when the last searched step scored under the localizer's own
0.55 and kept odometry, or the latest drift check disagrees with the map;
*good* otherwise; *unknown* when the status is missing or unreadable. This needs
the localizer to publish each step -- today it publishes a status once a second
showing the last of five -- so map_pose's periodic record is every step, stamped
with the scan it used, `correction` marking an accepted step, `restore` and
`refit` a pose handed to it.

**depth.frame:** `oak_rgb_optical` is right; depth is aligned to the OAK's colour
camera.

**The two proposals:** yes to `map_pose.fit`, by the rule above; yes to
`stills/index.jsonl`.

## What the schema asks that the rover cannot give as written

- `camera.gain` is always null (no such control); `exposure_s` is null in
  automatic mode and the value set in manual, never a reading.
- `board_feedback` has no device time; lines drained together share `t_arr`.
- `lidar.beam_t` comes from the sensor's millisecond clock, which wraps every 30
  s; `deskewed_ranges` per return does not exist, because the rover deskews into
  its 360 directions -- recorded beside the sweeps as its own derived stream, or
  null.
- `odometry.t_stamp` and every other ROS stamp are wall time minus a fixed
  allowance; the recorder converts them to the monotonic clock with its
  once-a-second clock record and keeps the original.
- `chassis_cmd`: the board is driven by per-track PWM, not velocities;
  `linear`/`angular` exist only as the request the base node turned into PWM,
  which it will pass along with each command. Its keepalive repeats the command
  every 0.167 s, each a command. Which ROS node asked for the velocity is not
  known at the base node.
- `stills`: the rover has no fixed camera; its stills are the gimbal camera at a
  fixed pose.
- `room_outlines` do not exist yet; the map's own split does not give the owner's
  rooms ([progress](../progress/2026-10-19-rooms-from-the-map.md)), so they are
  drawn with the owner.
- The annotation tool does not exist; it is built with the recorder.

## Where the recorder taps

Every writer stamps arrival on the monotonic clock, which every process on the
Orin shares, and writes into the session's directory through a bounded queue that
drops and counts (R-CTL-13):

- **the daemon:** the camera feed, every board line at its drain, every command
  and the command log at the board link's `send`, the gimbal schedule's runs, the
  annotations, health and the clock record;
- **the depth service:** depth and the IMU, written by the service itself so no
  frame crosses a socket;
- **the lidar node:** the raw packets with their arrival stamps, turned into
  `lidar/scans.jsonl` when the session closes;
- **a ROS subscriber:** odometry, the localizer's steps and events, the
  correction, and the 360-direction scan;
- **a closing step:** the manifest, hashes, clock mapping and final counts.

## What counts as done

The recorder is commissioned when, in a parked session and then a gentle drive,
every stream holds its configured rate within the spec's starting limits and
navigation holds its own while it records: at least 9.8 scans and 19.5 odometry
updates a second and 5 localizer steps a second, each over the whole session,
against the 10, 20 and 5 measured on 2026-10-28 with nothing recording.

## The held-out room

The bedroom, chosen by the owner on 2026-10-28
([decision](../decisions/the-bedroom-is-held-out-for-perception.md)): its door
is shut and recorded so in every other session, and its outline goes into every
manifest, drawn by `experiments/rooms/bedroom_outline.py`.

## Order of work

1. The schema is agreed with these answers.
2. Build the taps above, the session mode (feed held, manual exposure kept, the
   OAK held on, the looking loop off) and the annotation tool; replay-test what
   can be replayed and run the suites.
3. Deploy, then commissioning as in the spec, against the bar above.
