# A supervised autonomy session

What to type to let the rover choose where it goes, how to stop it, and what to
check before and after. **A person stays in the room for the whole of it** —
the rover cannot see a step, a threshold or a table edge
([R-SAFE-6](../requirements/safety.md#r-safe-6)), so the area is cleared by hand
and watched.

Autonomy is off every time the daemon starts. Nothing below survives a restart,
which is deliberate: authority that was taken away cannot come back by
restarting something.

## Before the rover moves

1. **Clear the area by hand.** Anything the lidar cannot see at 20 cm off the
   floor is the hazard: steps, thresholds, cables, pet bowls, an open stair.
2. **Check the rover agrees about where it is.** Over the tool protocol on the
   rover itself:

   ```bash
   ssh orin 'python3 - <<PY
   import json, socket
   s = socket.create_connection(("127.0.0.1", 8769), 5)
   f = s.makefile("rwb")
   for call in ("nav_status", "autonomy_status"):
       f.write(json.dumps({"call": call}).encode() + b"\n"); f.flush()
       r = json.loads(f.readline())
       print(call, {k: r.get(k) for k in
                    ("position_trusted", "map_settled", "map_id", "enabled",
                     "latched", "why")})
   PY'
   ```

   `position_trusted` and `map_settled` must both be true. If they are not, the
   rover has not confirmed where it stands and every drive will be refused at
   dispatch — see [rover-unresponsive.md](rover-unresponsive.md) and
   `refit_pose`.
3. **Have the console open** on the drive page, because its stop button is the
   fastest thing in the room. Closing the last console tab also stops the rover
   and ends the run.

## Opening a run

Enabling is a person's act and no program on the rover can do it. Name yourself
and say what the session is for; both go into the record.

```bash
ssh orin 'python3 - <<PY
import json, socket
s = socket.create_connection(("127.0.0.1", 8769), 5)
f = s.makefile("rwb")
f.write(json.dumps({"call": "autonomy_enable", "arguments": {
    "by": "the owner",
    "why": "M3 supervised session 1",
    "budget": {"seconds": 600, "travel_m": 30,
               "geofence": {"x_m": 0.0, "y_m": 0.0, "radius_m": 4.0}}}}).encode() + b"\n")
f.flush()
print(json.dumps(json.loads(f.readline()), indent=2))
PY'
```

Every budget may be made smaller than the standing limit and never larger; leave
one out and it takes the standing value. The `geofence` is the area you cleared —
a circle as above, or a box of `min_x_m`/`max_x_m`/`min_y_m`/`max_y_m` — in the
map's own coordinates, which the console shows when you hover the map. Leave it
out and the map is the only boundary, which is honest but wider than a cleared
room.

## Running it

```bash
ssh orin 'cd ~/ugv/autonomy && python3 executive.py'
```

It attaches to the run you opened, says so, and then narrates one line per goal.
It cannot open a run of its own: started with none open it says what is missing
and exits.

`--turns 1` carries out a single goal and stops, which is the right way to take
the first session of the day.

## Stopping it

Any of these, and the first is the one to reach for:

- **the console's stop button**, or closing the last console tab;
- **saying so out loud** to the voice model, which calls the same `stop_driving`;
- **driving it by hand** from the console, which is a takeover and ends the run;
- `Ctrl-C` on the executive, which stops the rover and hands the run back.

All but the last latch autonomy off until somebody enables it again. That is the
point of the latch: a stop that had to be pressed twice because the rover chose
another goal in between would not be a stop.

Killing the executive outright is also safe, and is worth doing once per session
of trials: the daemon notices within fifteen seconds that nothing is renewing the
permission, stops the wheels and closes the run, with Nav2 still perfectly
healthy underneath.

## After the session

Read back what it did. Each turn is one episode, with the candidates it weighed,
the goal it chose, every call it made and what changed:

```bash
ssh orin 'cd ~/ugv/autonomy && python3 -c "
import store, summary
s = store.EpisodeStore()
print(summary.recent(s, limit=10))"'
```

And the run's own account, including why it ended and what it spent:

```bash
ssh orin 'python3 -c "
import json, socket
s = socket.create_connection((\"127.0.0.1\", 8769), 5)
f = s.makefile(\"rwb\")
f.write(b\"{\\\"call\\\":\\\"autonomy_status\\\"}\\n\"); f.flush()
print(json.dumps(json.loads(f.readline()), indent=2))"'
```

For an [M3](../plans/autonomous-curiosity.md) session, write down before you
start: the area, the budget, and the stopping distance you expect. An
acknowledged stop request is not a pass — what the milestone asks for is the
distance the rover actually travelled after it was told.

## When it will not start

| It says | What it means |
|---|---|
| `autonomy is not enabled` | no run is open; the enable above is what opens one |
| a name and a reason, under `latched` | somebody stopped the rover; enabling again clears it |
| `the run ended: ...` | a budget, the battery, or the watchdog closed it — open a new one |
| `a run is already open` | one is running; stop it before opening another |
| the executive chooses nothing, every turn | read the refusals it prints: an unsettled map, an untrusted pose or a flat battery gate every goal at once |

## Boundary allowance

Navigation and the watchdog reserve 0.5 m inside a declared safe area for the body
and stopping. A route or adjusted goal reaching that inset is refused or cancelled.
Start and goal must fit inside it. Physical braking and latency still require the
supervised acceptance measurements; this number is not a certified stopping distance.
