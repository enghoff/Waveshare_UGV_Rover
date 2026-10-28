# The recorder on the rover: parked, nothing dropped and navigation at its rates, after five faults found on the rover

**A two-minute parked session at full rates (the gimbal camera at 30 frames a
second, depth at 15, the OAK's IMU at 200) dropped nothing and left navigation
where it was without one: 9.91 scans, 19.82 odometry updates and 4.99 map poses
a second (one short of the localizer's 5 a second over 125.6 s, which is the
window's edges), against at least 9.8, 19.5 and 5 that
[R-CTL-13](../requirements/control.md#r-ctl-13) asks.** Getting there took
five fixes, each found by running a session on the rover or by Perception's own
checker on one, and none of them visible in the offline tests that had passed
the day before. The recorder was deployed on 2026-10-28 at `b9672c5`, then
`fd0ac3c`, `6ed9ee1`, `b9fc218` and `753cb6b`
([plan](../plans/recording-for-perception.md)).

## How it was run

Parked on the charger in the living room, from this desktop over TCP 8769: a
session of kind `commissioning`, rooms `living`, configuration `full`, a door
mark for the bedroom (closed, not looked at: the owner's standing rule), a
13-step pan sweep of ±30 degrees at 1.5 s a step, a still, and the session's
end. No wheel turned. Each session's directory was copied to the desktop with
`tar` and checked there: against Perception's `rover-schema-v1.json`, and by
Perception's `scripts/rover_check.py` (its commit `704dd8e`), which reports
seven items as pass, fault or not assessable.

## What was wrong, and what proved each fix

**The manifest named commit `0000000`.** The deployer records each component's
commit as a plain string; the recorder read it as an object. Found reading the
deploy state before the first session; the session test now writes the state in
the deployer's own form.

**A session that switched the OAK on recorded no IMU at all.** The first
two-minute session (11:15) wrote 1,850 depth frames and an IMU file holding
only its header. The camera's pipeline is built when it is switched on, the
session was seen by the depth service half a second later, while the camera was
still uploading its firmware, and only an already open device was reopened for
the IMU. Reproduced with a 25-second session started with the OAK off (116 depth
frames, no IMU, no reopening in the log). Fixed in two steps: a device that opens
without what a session wants is reopened before it gives the session anything
(3,580 IMU records in 25 s, but the camera booted twice); then the pipeline reads
the session's announcement when it is built, since the daemon announces before
it wakes the camera (4,300 records in 25 s, one boot, no reopening). Started with
the OAK already on, a session reopens it once, as designed (3,785 in 25 s).

**Every stream from the other processes began 2 to 4 s after the session's
start.** The checker counts rates between the `session_start` and `session_end`
annotations, and the script had marked the start at once: depth began 3.9 s
later, the IMU 3.7 s, odometry 2.1 s, the scan 2.2 s, the map pose 2.3 s and the
ROS clock 3.1 s, because the OAK boots and the ROS recorder starts after the
session opens. Inside the session no stream had a gap longer than three of its
periods. The start is now refused until every steady stream has given a record.

**One map pose was written twice, with the same sequence number.** A change of
fit seen in the localizer's status went through the path that writes a record
and then a second one for the change; and the second, copied from a record
already written, kept its number. The ROS test now drives three changes of fit
through the recorder and fails on the old code with five records numbered 0, 1,
1, 2, 2.

**Health named the ROS streams only once they had reported**, and lost about
one record a minute to its own work between seconds. It names every stream from
its first record and keeps a fixed beat.

The checker also faulted every depth frame for holding values beyond 20,000 mm:
0.56% of a frame's pixels, at discrete values from 20,947 to 65,535, which are
disparities of one to three sub-pixel steps and what the OAK itself outputs.
Perception decided to keep the raw values and count them as no depth.

## The full-rate session

13:18 IST, at `753cb6b`, the OAK already on (a depth-service restart wakes it),
so the session reopened it once for the IMU. An early `session_start` was
refused; every stream had begun 6.9 s after the session opened, and the start
was marked then. 125.6 s between start and end, nothing dropped in any stream:

| Stream | A second, between start and end | Nominal |
|---|---|---|
| Odometry | 19.82 | 20 |
| Published scan | 9.91 | 10 |
| Map pose | 4.99 | 5 |
| Board feedback | 19.93 | about 20 |
| Gimbal camera | 29.90 | 30 |
| OAK depth | 15.00 | 15 |
| OAK IMU | 201.13 | 200 |

Lidar packets 55,355 and the 13 scheduled gimbal commands, each in the command
log under its own number. The checker (its commit `255ac63`, with the depth
rule changed at `74ebf96`) passed order, boundary and schema; timing and the
held-out ledger were not assessable in a session with no timing manoeuvre and
no ledger, as expected. What it still reports, and why neither is the
recorder's:

- **One odometry gap of 0.159 s** against a limit of three periods (0.15 s), at
  52.6 s. Odometry's own stamps step 0.16 s there too, and the board's feedback
  lines stop for 0.14 s at the same moment: the gap is upstream, in the board's
  telemetry or its drain. The 11:43 session had one of 0.16 s as well.
- **Nine depth frames with no valid depth**: the first 0.65 s after the OAK
  reopened, six before the start and three after. The stereo gives empty frames
  for that long after every opening, which a mid-session restart would repeat;
  whether those count is the checker's rule.

The windows fault is the script's own still, taken for a unit that never
started to see the warning the recorder now gives.

## What it does not show

Nothing drove. Commissioning, with the owner, runs the timing manoeuvres and a
session that drives; until then the rates above are parked rates. The desktop's
own wifi was slow through the afternoon (50 to 80 ms to the router), which
slowed the copies and the deploys but touched nothing on the rover.
