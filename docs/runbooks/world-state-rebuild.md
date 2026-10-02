# Rebuilding the world state after a change to how looks are measured

What to type to read the rover's kept looks again the way a live look is read now,
throw the things away and let the resolver build them again from those looks, oldest
first. It is for after a change to ranging or to the resolver, when the things the rover
holds were built the old way. What it does and why is in
[world_state/rebuild.py](../../world_state/rebuild.py).

The rover's own looks are held off while it runs, for a few minutes. Every thing gets a
new number, so close the console's world panel first and do not run it during an
autonomy session (it would end the run).

## Ask what it would change

```bash
ssh orin 'python3 - <<PY
import json, socket
s = socket.create_connection(("127.0.0.1", 8769), 600)
f = s.makefile("rwb")
f.write(json.dumps({"call": "world_state_rebuild", "arguments": {}}).encode() + b"\n"); f.flush()
print(json.dumps(json.loads(f.readline()), indent=1))
PY'
```

`rerange` counts the looks with a kept depth map, the regions read again, how many
ranges would move by more than 2 cm, and which reading each would come from. Nothing is
written.

## Do it

The same call with `{"apply": true}` as its arguments. It answers with the backup it
made, the re-ranging counts and `rebuild`: things before and after, the looks let back
in, and how long it took.

## How to tell it worked

```bash
ssh orin 'python3 - <<PY
import json, socket
s = socket.create_connection(("127.0.0.1", 8769), 20)
f = s.makefile("rwb")
f.write(json.dumps({"call": "world_state_summary"}).encode() + b"\n"); f.flush()
print(json.loads(f.readline())["summary"])
PY'
```

`entities` matches `rebuild.things_after`, and `observations` is what it was before:
the rebuild deletes things and never looks. A later look's diagnostics line reads
"N of M ranged by the depth camera", with "(K from the box)" when some regions fell
back to the box.

## When it fails

- **"an inspection has been running for longer than 20 s"**: a look is stuck. Nothing
  was changed; try again in a minute, and look at the world loop if it repeats.
- **It stopped part way** (the daemon restarted, the connection dropped): run it again.
  A rebuild picks up looks an interrupted one left held back. To put the store back as
  it was instead, stop the daemon, copy the backup it named over
  `~/.ugv/world/world.db` and start the daemon again
  ([deploy.md](deploy.md) has the restart).
