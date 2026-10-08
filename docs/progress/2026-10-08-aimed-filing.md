# Filing by aim is built: the right region reaches the target, its depth sets the claim

**A geometry goal's look can now name its target, and the world state gives the
target the region at the aim, places it from that look's depth, and remembers
which other records the region fitted.** Put through the recorded aimed looks, it
filed 24 regions where the resolver had filed 5, 19 of them taken from another
record, and 16 targets' claims fell. Against the tape one painting went from
0.47 m off to 0.06 m. The painting behind the dining chairs is the exception: 3
of its 6 filings claimed less than they were off. Deployed at 46df458 with
target naming off: nothing files by aim until
[the case](../decisions/aimed-looks-file-to-their-target.md) is agreed under
R-WS-13. What is live now is reading a placement by its claim, and cooling a
target's suspected same-object records with it.

## What was built

- **World state** (`world_state/aimed.py`, new): the rule measured on 157 recorded
  aimed looks ([2026-10-08](2026-10-08-aimed-looks.md)) chooses the region. It is
  attached to the target, but not as an appearance example. A new table,
  `aimed_looks`, keeps each filing and the other records the region fitted.
  `resolve._replace_placement` takes the target's position and claim
  (`stated_uncertainty_m`) from aimed looks that ranged it, never claiming below
  0.20 m, and leaves the tolerance it matches with to the bearings. The inspector
  files inside its own lock, so no settling pass takes the region first.
- **Daemon**: `world_inspect` passes a `target` through to the inspector, and
  `world_state_entities` lists each thing's same-object suspects.
- **Autonomy**: the geometry look names its target. Goals predict and measure
  gain on the claim (`situation.claimed_m`) rather than the matching tolerance. A
  geometry attempt puts the target's suspected same-object records aside for 15
  minutes, whether or not it helped.

## Tests

world_state 1,075, rover_daemon 1,124, autonomy 804; all pass. New checks:

- **Filing:** with a look-alike second record beside the target, the region goes
  to the target and the second record is named on both. A look pointing
  elsewhere files nothing, and two alike regions at the aim are refused.
- **Claim:** an aimed range moves the position to the painting and lowers the
  claim, never below the floor, while the tolerance stays the bearings'.
- **Plumbing:** an inspection reports its filing; the daemon passes the target
  and lists the suspects.
- **Autonomy:** a look that helped puts the target's other record aside, and the
  claim is preferred to the tolerance.

## On the recorded looks

`experiments/entity_association/replay_aimed.py` put the 157 aimed looks of
2026-10-02 to 10-06 through the production code, on a copy of that day's store:

| | Looks |
|---|---|
| target no longer in the store | 58 |
| no region with a bearing | 2 |
| no region points at the target | 73 |
| filed | 24 |
| ...taken from another record | 19 |
| ...with a same-object suspect | 20 |
| ...ranged | 18 |
| ...claim fell by more than 2 cm | 16 |

Seven filings were for targets the owner taped on 2026-10-03:

- **Window-side painting:** 0.47 m off before, 0.06 m after, claiming 0.20.
- **Green landscape painting, behind the dining chairs:** three filings ended
  0.21 to 0.38 m off within their claims, and three ended 0.32 to 0.55 m off
  while claiming 0.20 to 0.25. Those three ranges came from the outline with
  stated sigmas of 0.05 to 0.29 m; nothing in them marks the chair the camera
  probably measured instead.

Without the 0.20 m floor, one claim fell to 0.04 m. The floor is the median miss
of single ranges on the taped paintings.

## On the rover

Deployed with `deploy.py` at 46df458, world_state, rover_daemon and autonomy
together; each self-test passed on the Orin and the daemon came back. Checked
there:

- all 421 things listed carry `same_object_suspects`;
- the store has its `aimed_looks` table;
- `executive.NAME_THE_TARGET` is off.

On a throwaway copy of the rover's store, the rule filed a region from each of
the five newest looks. One ranged filing took its thing's claim from 0.27 to
0.20 m, and four named a suspected same-object record.

## Next

The owner decides the case. If it is agreed, `NAME_THE_TARGET` is turned on and
deployed, and the first M4 look is taken on the rover with every aimed filing
reported beside its photograph.
