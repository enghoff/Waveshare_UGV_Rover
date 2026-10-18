# The world model starts again when the store is cleared or put on another map, and a restart no longer leaves it without lidar

**The world model's records now answer only on the map, and from the filling of
the store, they were filed from, and the job starts again when either
changes.** Before, a map cleared from the console left the old map's records
standing on the new one at the old coordinates. Separately, every restart of
the job had been leaving its lidar logger alive but deaf, so the model filed
without lidar until the next boot; that is fixed too.
[R-WS-5](../requirements/world-state.md#r-ws-5) and
[R-WS-6](../requirements/world-state.md#r-ws-6) hold;
[R-WS-17](../requirements/world-state.md#r-ws-17) stays proposed.

## Following the store

How a map changes now. Since the restore fix of 2026-09-06 navigation names a
new map only when somebody clears it or no map is saved, and the rover has been
on map session 67 since then. The two cases reach the world state differently
(`store.follow_map`):

- **A map cleared from the console** empties the store first, and an empty
  store takes the new map without a new session. Session 67 stays.
- **A new map with the store's rows kept** moves the session.

The records were checked by session alone. Reproduced on the desk
(`rover_daemon/test_recall.py`, run before the fix): after a map clear the old
map's record answered `world_state_entities` as if nothing had happened, and so
did a record after a clear of the world state with the map kept, whose looks
were gone from the store. Only the new map with rows kept was refused.

The job now writes beside its records which map and which filling of the
store they are of (map identity, session, store generation), and the daemon
refuses records of any other. When the store changes, the job sets its engine,
records, decisions, and the lidar, recordings and walls of that map aside, and
files what the store holds then, from nothing, under a new name for its
records. Until it has filed something, voice and the console answer from the
store and an autonomous run plans around no things, as they did before.

**On a new map, the walls come from that map's own grid.** Version 4 places
paintings where their sight line meets a wall, and its walls file was drawn on
this map. Over every picture to 2026-10-17:

| Walls | Records | Regions filed | Paintings' labelled pictures filed | Trials kept | Wrong merges |
|---|---|---|---|---|---|
| frozen, four grids intersected | 624 | 28,115 | 206 | 5 of 7 | 0 |
| one grid as navigation sends it | 630 | 28,120 | 206 | 5 of 7 | 0 |
| none | 523 | 26,614 | 188 (snowy-tree 31 of 41) | 5 of 6 | 0 |

So the job asks the daemon for the grid and refetches it every ten minutes
while the rover maps a new room.

**On the rover**, deployed at `beff65a`: the job kept its engine (records still
named `ee1d222ead6b0ff6`, 629 of them) and now writes map `7da19bef3888`,
session 67 and the store's generation beside them; the daemon answers from
them, and "the bed" is 10.0 m to the right, seen 84 times, as before. The
follow and the refusal ran there only as tests (the world state's and the
daemon's suites, which the deploy runs). A real clear was not tried, since it
would empty the owner's world state.

## The lidar logger went deaf on every restart

The logger runs in navigation's ROS environment, and rclpy's own handler took a
SIGTERM as "shut ROS down" and kept the process running. Reproduced on the
rover: a listener heard 30 scans in 3 s, none in the 3 s after the signal, and
stayed alive. `restart_job.sh` -- every world_state deploy -- therefore left a
logger that heard nothing, which the supervisor never replaced and the restart
script counted as running.

- Today the job filed without lidar from 07:46, its restart then, to 09:43;
  the lidar was publishing at 10 Hz throughout. Since the job started on
  2026-10-17, 15 of its 46 batches (145 of 908 regions) were filed without
  lidar, so nothing in those visits could be seen gone.
- Fixed at `779c329`: rclpy's handlers are off, a SIGTERM writes what is held
  and exits within half a second (measured on the rover, twice), and ROS
  stopping for any other reason exits too. The deaf logger from boot needed
  killing by hand once; after that a `restart_job.sh` replaced the logger in
  30 s, its last 10 s written, and the job's next batch, at 09:52, had lidar.
