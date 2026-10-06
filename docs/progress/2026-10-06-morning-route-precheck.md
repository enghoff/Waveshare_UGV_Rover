# Morning route check stops the trial before movement

The rover was healthy enough to start, but the proposed return route failed the
predeclared short-route limit. No movement or entity trial was performed. The
rover stayed at its charging start, acknowledged STOP and reported zero commanded
speed. R-WS-13 remains open; R-WS-17 and R-WS-18 remain proposed.

The owner handed over the rover on October 6. Initial battery was 90% at 12.18 V.
Map ID was `7da19bef3888`, pose approximately (-17.439, -15.691, -177.4 degrees),
with trusted position and live lidar/Nav2. Fresh read-only localization scored
98.9%, agreeing within 4.5 cm and zero degrees. Passive board samples showed
changing IMU values and nonzero rotation accumulation while stationary; this
rules out the earlier completely frozen tuple at preflight, not future failures.
The starting camera showed daylight and the charging-area armchair/cabinet scene.

The prepared helper ran on Orin from `/tmp` and requested only
`ComputePathToPose`. It sent no navigation, velocity, refit or reset commands.
The HOME-to-A request used the current pose; the other requests used explicit
hypothetical starts. All four planning actions succeeded with error code zero:

| Leg | Returned length | Direct distance | Trial limit | Result |
|---|---:|---:|---:|---|
| HOME to A | 1.074 m | 1.038 m | 1.538 m | Within limit |
| A to B | 0.851 m | 0.831 m | 1.331 m | Within limit |
| B to A | 3.260 m | 0.831 m | 1.331 m | Failed |
| A to HOME | 1.074 m | 1.038 m | 1.538 m | Within limit |

The B-to-A hypothetical start faced 167.486 degrees and its requested arrival
heading was -12.514 degrees. The returned path loops away from the destination;
summed path heading changes are approximately 340 degrees. This is a live
planner answer for specified hypothetical conditions, not an observed drive or
proof that the path is wrong. A separately checked stationary turn before the
return is a possible next preparation step. It has not been live-verified, and
the existing source must not be changed on this evidence alone.

As the plan required, the failed leg ended preparation. STOP was acknowledged;
final navigation reported no driving/exploring, zero speed and no active move.
The navigation response's PWM field was null, so no final PWM measurement is
claimed. Final battery was 85% at 12.11 V. No world-call marker or support drive
recorder was enabled; no new controlled painting evidence or candidate score was
obtained. The owner was told the charger could be reconnected.

Local evidence is `captures/visibility-morning-prepared-1/`: RPC history, starting
photograph, four complete planner paths and `preflight-summary.json`. The exact
planner transcript SHA-256 is
`dfad84ad042ae2fe1291cb6eddd1f0165cf870cb2c79064d673d470d45f0bade`.
No production file changed or service restarted. The preparation helper's live
planning function is now verified; route execution and side-view coverage remain
unproven. Next preparation must account for heading changes on both outbound
and return legs before asking for another drive.
