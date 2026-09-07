<!-- requirement-area: NAV -->

# Mapping, localization and movement

What the rover must do to know where it is and get somewhere. The conventions for
these records are in [README.md](README.md); what actually runs is
[ros_nav/README.md](../../ros_nav/README.md).

<a id="r-nav-1"></a>
### R-NAV-1 — The rover builds a 2D map of reachable floor and keeps it between sessions

- **State:** settled
- **Evidence:** [ros_nav/README.md](../../ros_nav/README.md) — `slam_toolbox`
  serializes its pose graph and last trusted pose under `~/.ugv/map/`

The map survives a reboot and a deploy. It lives outside the deploy tree
precisely so that shipping new code cannot take the room with it — see
[R-PLAT-1](platform.md#r-plat-1).

<a id="r-nav-2"></a>
### R-NAV-2 — A restart may not overwrite the saved pose until something has confirmed where the rover stands

- **State:** settled
- **Evidence:** [ros_nav/README.md](../../ros_nav/README.md) — `nav_status`
  reports `map_settled` beside `map_kept`; `python ros_nav/selftest.py`

On restart the mapper anchors the graph it has just read with its own scan
matcher. That anchor is the mapper's opinion, not the rover's belief: landing on
the saved pose confirms it, and landing somewhere else does not disprove it,
because a room whose two ends look alike will happily match the wrong one.

Either way the map is kept. What the confirmation gates is the right to write
over the saved pose, and the reason it is gated is that a session which cannot
place itself must not destroy the record that would let the next session try
again. Watching `map_kept` alone will therefore say a restart went well when it
did not.

<a id="r-nav-3"></a>
### R-NAV-3 — Fitting the rover onto its map is a person's act, never automatic

- **State:** settled
- **Evidence:** [ros_nav/README.md](../../ros_nav/README.md) — the console's
  "refit to map" button and the daemon's `refit_pose` control call

A fit moves the rover on the strength of one scan matched against a stored graph.
The case it exists for is somebody having carried the rover while it was off,
which is exactly the case where that scan agrees with the map least, so a boot
that fitted itself would be most confident when it was most likely to be wrong.
It is also deliberately kept off the model's tool list — see
[R-CTL-3](control.md#r-ctl-3).

<a id="r-nav-4"></a>
### R-NAV-4 — Clearing the map clears what was measured against it, in one act

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md) —
  `clear_map` empties the world store and re-points it before answering, and
  records the 2026-09-06 failure that made it necessary

Every row in the visual memory is a position in the map's frame or a bearing from
a pose in it, so anything surviving a map clear is a list of things with nowhere
to be. The rover does both halves itself rather than relying on whatever pressed
the button: on 2026-09-06 a map cleared from a console whose world panel was shut
left 423 things behind, measured against a map that had gone, and said nothing.

<a id="r-nav-5"></a>
### R-NAV-5 — Exploration chooses reachable frontiers and gives up on ones going nowhere

- **State:** settled
- **Evidence:** [ros_nav/README.md](../../ros_nav/README.md);
  `python ros_nav/selftest.py`

See [R-SAFE-7](safety.md#r-safe-7) for the bounds it runs under.

<a id="r-nav-6"></a>
### R-NAV-6 — No frontier is longer than one arrival can account for

- **State:** settled
- **Evidence:** `ros_nav/fixtures/ringed-2026-09-07.json.gz` replayed through
  `python ros_nav/selftest.py`

A frontier is written off once the rover has driven to it, so an over-long one
retires ground the rover never saw. A rover ringed by unknown floor treats the
whole rim as a single frontier, is sent to its centre — which is where it already
is — and retires the lot on arriving without having moved. Boundaries past the
configured maximum are cut into pieces with a goal each; the fixture is a map
that failed this way.

<a id="r-nav-7"></a>
### R-NAV-7 — The chassis refuses to drive without its measured calibration

- **State:** settled
- **Evidence:** [ros_nav/README.md](../../ros_nav/README.md) — `base_node.py`
  refuses to run without `~/ugv/odometry.json`

Gyro scale, encoder distance and motor curves are specific to this chassis and
are runtime state rather than deployed source, so a fresh host has no honest
odometry until they are copied or remeasured. Refusing is better than driving on
a default, because wrong odometry corrupts the map rather than merely producing a
wrong move.

<a id="r-nav-8"></a>
### R-NAV-8 — The rover is not to be relied on to detect drops or obstacles off the scan plane

- **State:** settled
- **Evidence:** [ros_nav/README.md](../../ros_nav/README.md);
  [reference/d500-lidar.md](../reference/d500-lidar.md)

This is a requirement in the negative: the lidar sees one horizontal plane, and
no movement operation — manual, routed or exploratory — can detect a stair, a
threshold, a table top or anything wholly above or below that plane. Depth from
the OAK is used for ranging visual regions and has never been validated as
obstacle authority.

It is recorded here so that anything built on top inherits it explicitly rather
than by omission. It is what keeps [R-SAFE-6](safety.md#r-safe-6) open.

<a id="r-nav-9"></a>
### R-NAV-9 — A commanded rotation turns the rover by the angle commanded, within a stated tolerance

- **State:** open
- **Blocked by:** no tolerance has been declared and no measurement of commanded
  against achieved rotation is recorded in this repository

Turn accuracy is load-bearing for the active-perception work, which plans to put
the rover at a chosen viewpoint and expects the bearing it then measures to mean
something. The nearest recorded facts are that the minimum pivot response is
coarser than the smallest angular velocity the controller can ask for, and that
the gyro scale is a single measured constant; neither says how far a commanded
thirty-degree turn actually goes.

Settling this needs a declared tolerance and a measurement from both directions,
in the manner of the gimbal work in
[R-WS-10](world-state.md#r-ws-10) — which is the same class of fault and was
found only because somebody approached the same angle from two sides.

<a id="r-nav-10"></a>
### R-NAV-10 — The local controller does not steer into inflated obstacle cost

- **State:** open
- **Blocked by:** a controller fix; shorter plan pruning reduces it and
  [decisions/doorway-pivot.md](../decisions/doorway-pivot.md) has why the
  obvious fixes were wrong

The controller can aim around a corner into an inflated wall. A rover already
touching inflated cost may have no valid route out at all and need a short manual
reverse before it can replan. Both are reproducible from recordings under
`ros_nav/`, which is where a candidate fix has to be shown working first.

<a id="r-nav-11"></a>
### R-NAV-11 — Navigation faults are reproducible without the rover

- **State:** settled
- **Evidence:** [ros_nav/README.md](../../ros_nav/README.md) — `nav_record.py`
  records pose, plans, costmaps and controller evaluation; `dwb_replay.py`,
  `smac_replay.py`, `trap_sim.py` and `steering_sim.py` replay them

This is a requirement on the system rather than on the people using it: the
running stack has to emit enough to reconstruct a failure offline. The working
rule that says to do so before fixing anything is in
[../../AGENTS.md](../../AGENTS.md).

<a id="r-nav-12"></a>
### R-NAV-12 — The refit search reaches as far as the rover's localization actually goes wrong

- **State:** proposed
- **Proposed in:** [2026-09-07 refit window](../progress/2026-09-07-refit-window.md)

The manual fit of [R-NAV-3](#r-nav-3) searches a metre of position and
forty-five degrees of heading around where the rover believes it is. Set against
the localization errors this rover has actually recorded, that is generous in
position and too narrow in heading: 81 degrees with 41 cm on 2026-09-06, 37
degrees with 10 cm at a restore on 2026-09-07, and 152.5 degrees with 15 cm later
the same day. Every one of them was mostly a heading error with the rover roughly
in the right place, and the widest was more than three times what the search can
see.

The consequence is that the fit is unavailable in the cases it exists for. On
2026-09-07 the scan scored 0.99 against a 0.90 bar at the rover's true pose, so
nothing was wrong with the threshold; the window simply did not reach. Within
forty-five degrees the best candidate scored 0.807, was refused, and had it been
accepted would have moved the rover 1.1 m and 29.5 degrees further from the
truth. Opening the window to the full circle recovered the rover on the first
attempt at the existing thresholds, and `window_deg` is already a parameter of
the call rather than a code change.

This is recorded as proposed rather than agreed because the counter-argument is
real and is already documented in
[ros_nav/README.md](../../ros_nav/README.md): a wide search can place the rover
confidently in the wrong one of two rooms that look alike, which is why measuring
first with a score floor above 1 is the advised habit. The wider window has so
far been demonstrated on one scan in one room.

What made this particular error findable is that it was almost purely a heading
error: the accepted fit moved the rover 152.5 degrees and 0.149 m. That is not a
general property of the search. The score varies strongly with position — a scan
lying half a metre off the walls scores near zero, which is what makes the search
work at all — so a fit is not cheap in position merely because this one was.
`refit.py`'s narrower claim about a metre of translation is that it barely
changes *which points land on mapped ground*, which is the argument for leaving
unmapped returns out of the average, not a claim that position is weakly
determined.

Settling it means deciding what the window should be, on more than one room, and
saying what protects a symmetric room once the search can reach round it. Until
then the widened fit stays a thing a person asks for by hand.

**Both halves of that case have since moved, and against the proposal.** Two
measurements on 2026-09-07, after this record was written:

The evidence *for* widening is largely gone, because the errors it was sized
against were manufactured. A parked rover was integrating the residue of its own
gyro zero-offset at 0.6 degrees a minute — half a turn across a working day —
and [R-NAV-13](#r-nav-13) has stopped it. The large heading errors quoted above
were mostly that bug accumulating, not the localization this rover does when it
is working properly, so sizing a search window against them would be sizing it
against a fault that no longer exists.

The evidence *against* is now concrete and measured on this rover rather than
argued from a README. In its actual parking spot the room is symmetric enough
that a full-circle search finds 98% of the scan on a wall half a turn away
against 94% where the rover really stands. A wide search there does not merely
risk preferring the wrong of two similar rooms; it does prefer it, and the fit
correctly refuses to choose because nothing in a planar scan can separate them.

What survives is narrower and worth keeping: a 180-degree fit asked for by hand
is what recovered the rover from 174 degrees out, so the wide window earns its
place as a recovery tool a person reaches for knowingly. Making it the default is
now the harder argument, not the easier one. Anyone settling this should also
weigh [R-NAV-14](#r-nav-14), which closes the half of the problem that actually
kept the rover wrong for a working day — not that the window could not reach the
error, but that nothing was looking for it.

<a id="r-nav-13"></a>
### R-NAV-13 — A rover that is standing still does not accumulate heading

- **State:** settled
- **Evidence:** `ad76f0c`; `python ros_nav/selftest.py` — the offline odometry
  model asks for none of the invented rotation; deployed to the rover at
  `0f53e12`

The gyro's zero-offset is estimated and subtracted while the wheels are still,
and what that leaves behind is a rate, so it integrates. Measured hours after
boot with the estimate converged, the gyro read 0.443 degrees per second against
a still rate near 0.433: the hundredth left over came to 0.6 degrees a minute, 36
an hour, half a turn across a working day of standing in one place.

Nothing was going to catch it either. The mapper is asked for a scan only after
0.2 m or 0.2 rad of apparent motion, so a rover that never appears to move is
never corrected, and the manual fit searches 45 degrees, which is no use against
174.

Past the settle grace that already keeps a coast out of the average, a still
interval now contributes nothing to the pose at all — the wheels say the rover is
not turning, so it has not turned. The bias is still learned there, because that
is what corrects the intervals the rover is moving through.

What this gives up is a rover rotated while its wheels do not turn: carried, or
lifted and set down facing another way. Dead reckoning never measured that
rover's position either, so it removes half of a measurement that was already
missing rather than a working one, and being moved while switched on is what the
fit in [R-NAV-3](#r-nav-3) is for.

The offline check this got past used to ask that *most* of the invented rotation
be removed, and 75% of 0.46 degrees per second is still 7 degrees an hour. A
threshold that permits a rate permits an unbounded error given time.

<a id="r-nav-14"></a>
### R-NAV-14 — A parked rover checks its belief against the lidar and says when the two disagree

- **State:** settled
- **Evidence:** `8d8b6b2`, `0f53e12`; `map_drift` in `nav_status` and the
  console's `vs lidar` row; `python ros_nav/selftest.py` at 546 checks; deployed
  at `0f53e12`

"The rover drifts" and "the rover cannot tell that it is wrong" are different
faults, and only the second explains how a heading stayed 174 degrees out for a
working day with somebody in the house. [R-NAV-13](#r-nav-13) fixed the first.

Nothing used to consult the lidar while the rover was parked: the mapper corrects
`map -> odom` only when it folds a scan into the graph and will not fold one
until the rover has apparently moved, so a stationary rover had the walls in
plain sight and asked nobody. Meanwhile the console read `position: trusted`
throughout, because that is the mapper's confidence in a match it made hours ago
rather than a statement about now — the same gap between *fresh* and *right* that
[R-WS-16](world-state.md#r-ws-16) is about.

So every five minutes, when no move is running, the scan is matched against the
whole map and the disagreement is reported: `agrees`, `cannot say`, or `OFF BY
43 cm, 174 deg`. It is logged once when it starts disagreeing and once when it
stops, because a line every five minutes is how a log stops being read.

**It measures and reports; it does not act.** No pose is written, no graph
touched, `map_settled` is not moved and the rover is not driven. That is what
keeps [R-NAV-3](#r-nav-3) true — deciding the rover has been moved is still a
person's call, and this only makes sure the person is told.

A refusal now carries the score where the rover believes it is, because "the
lidar cannot say" on its own does not distinguish a rover standing where the map
explains 94% of what it sees from one standing where it explains 30%, and only
the second is worth getting up for.
