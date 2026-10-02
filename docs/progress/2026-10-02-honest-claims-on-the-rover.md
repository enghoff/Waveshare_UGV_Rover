# Honest placement claims and the elevation correction are on the rover

**The rover now corrects its 4.9-degree elevation bias, and every placement carries
what it has been measured to be worth beside the tolerance it matches with.** Both were
deployed at `4342c37` (world_state, rover_daemon and autonomy). The on-rover suites
passed: world_state 954, autonomy 698, rover_daemon 1039. The running code was then asked
directly:

- a 30-degree elevation at the rest tilt reads 25.1;
- a one-look placement matching to 0.05 m claims 1.0 m.

The rover was on its charger and did not move. A refit confirmed its position, to within
2 cm and 0.5 degrees, and the map is settled.

The 36 things already in the store were placed before the deploy and carry no
`stated_uncertainty_m` until they are placed again. The hypothesis generator falls back to
their matching figure, so the store should be rebuilt by a drive before M0a's runs. Each
run's drive does that.

What the claims were measured against, and why the bearing and range claims were left
as recorded, is in
[what to expect from the hardware](2026-10-02-what-to-expect-from-the-hardware.md) and
[world_state/README.md](../../world_state/README.md#what-to-expect-from-it).
Replayed on the three taped drives:

- every target placement lies within twice what it claims, 14 of 14;
- taped heights lie inside their claims, 7 of 7;
- on the labelled drive of 2026-09-08, redrawn through today's lens, the rover merged
  different objects 8 times, against 10 with the code as deployed before.

## Requirements

None moved. [R-WS-11](../requirements/world-state.md#r-ws-11) waits for heights measured
on the rover with the correction in place, which M0a's runs will give.
[R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing` on the per-look bearing
claim.
