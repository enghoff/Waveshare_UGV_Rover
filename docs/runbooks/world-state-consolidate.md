# Consolidating the world state by EM, reviewing it, and rolling it back

What to type to re-solve every thing in the current map session at once by EM, look at
the result in the console, and put the store back if it is worse. What it does and why is
in [world_state/consolidate.py](../../world_state/consolidate.py); what it did to the
session it was measured on is in
[the replay](../progress/2026-10-03-whole-session-em.md).

The rover's own looks and its resolver are held off while it works, for seconds rather
than minutes. Things keep their numbers, but some disappear into others and some go, so
do not run it during an autonomy session: like a rebuild, any of these calls ends the run.

## Ask what it would do

```bash
ssh orin 'python3 - <<PY
import json, socket
s = socket.create_connection(("127.0.0.1", 8769), 600)
f = s.makefile("rwb")
f.write(json.dumps({"call": "world_state_consolidate", "arguments": {}}).encode() + b"\n"); f.flush()
print(json.dumps(json.loads(f.readline()), indent=1))
PY'
```

Nothing is written. `things_before` and `things_after` are the counts, `merges` lists
which things would be taken into which, `dropped` lists the things that would go with
how many looks each held, and `looks_moved`, `looks_let_go` and `looks_taken_up` say how
many looks change thing, leave every thing, or join one from waiting.

## Do it

The same call with `{"apply": true}` as its arguments. It answers with the same summary
and `run`, the number of this consolidation.

## Review it

Open the console's world popup. The list is the consolidated world, each thing's looks
are the pictures it now holds, and a look that was moved says so under it, naming the
run and the thing it came from. Things worth opening first are the `merges` and the
largest things that changed, because a merge of two different objects and a thing whose
looks were re-split with a neighbour standing in the same pictures are what this does
wrong.

## Put it back

The same call with `{"rollback": true}`. It answers `rolled_back` with the run's number,
how many things and looks went back, and `founded_since_and_emptied`: things the
resolver founded after the consolidation out of looks it had let go, which the rollback
takes back and removes. `{"runs": true}` lists every consolidation and whether it was
rolled back.

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

After applying, `entities` is `things_after` plus whatever is placed in other map
sessions. After a rollback it is the count before the consolidation, plus any things
the resolver founded since from looks taken since. `observations` never changes: nothing
here deletes a look.

## When it fails

- **"an inspection has been running for longer than 20 s"**: a look is stuck. Nothing
  was changed; try again in a minute.
- **"the things changed since this was worked out"** or **"changed map or was cleared"**:
  the store moved while it worked. Nothing was changed; ask again.
- **"no consolidation is applied"**: there is nothing to roll back, or it has already
  been rolled back. Only the last one applied can be, and not after the world state has
  been cleared or the map has changed.
- **The connection dropped part way through**: applying and rolling back are each one
  database transaction, so either all of it happened or none of it did. Ask with
  `{"runs": true}` to see which.
