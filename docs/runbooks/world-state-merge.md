# Joining things the resolver split, reviewing them first, and putting them back

What to type to ask which things look like one object, draw them for review, join the
ones a person accepts, and undo it. What it does and why is in
[world_state/merging.py](../../world_state/merging.py); what it was measured to do is in
[the progress entry](../progress/2026-10-04-one-score-for-appearance-and-position.md).

Every call ends an autonomy run, asking included, as the rebuild does: do not use it
during one. Joining holds the rover's own looks off only while it writes.

## Ask what it would join

```bash
ssh orin 'python3 - <<PY
import json, socket
s = socket.create_connection(("127.0.0.1", 8769), 60)
f = s.makefile("rwb")
f.write(json.dumps({"call": "world_state_merge", "arguments": {}}).encode() + b"\n"); f.flush()
print(json.dumps(json.loads(f.readline()), indent=1))
PY'
```

Nothing is written. Each proposal has a number `n`, the thing that would keep its name
(`keep`) and the one that would be joined to it (`gone`), the `score` with its two parts,
`appearance` and `geometry`, how far apart the two stand, and `shown`: some looks of each.
No thing is in two proposals; ask again after joining for the next round.

## Look at them

```bash
ssh orin 'python3 ~/ugv/world_state/merge_sheet.py'
```

It asks the same question and draws one row per proposal into `~/.ugv/world/review/`,
printing the paths: the number and score, crops of `keep`, a red bar, crops of `gone`.
Copy them off with `scp` to look at them. Refuse a row when the two sides are different
objects, or when either side is mostly something else; glare, blur and bare wall are not
worth joining either way. Low scores are where the mistakes are.

## Join the accepted ones

The same call with the accepted pairs, `keep` first:

```bash
ssh orin 'python3 - <<PY
import json, socket
s = socket.create_connection(("127.0.0.1", 8769), 60)
f = s.makefile("rwb")
pairs = [["object:318", "object:381"], ["object:241", "object:329"]]
f.write(json.dumps({"call": "world_state_merge", "arguments": {"apply": pairs}}).encode() + b"\n"); f.flush()
print(json.dumps(json.loads(f.readline()), indent=1))
PY'
```

It answers `run`, the number of this joining, `joined` with how many looks moved for
each pair, `refused` with the reason for any pair it would not join, and `things_now`.
Each moved look's note says which thing it came from and which run moved it, which is
what the console's popup shows. The kept thing's position is worked out again from its
looks, as the resolver does after a look.

## Put it back

The same call with `{"rollback": true}` puts the last run back: both things of every
pair as they were, and every moved look on its old thing with its old note. A look
recorded since stays where the resolver put it. Rolling back again puts back the run
before. `{"runs": true}` lists every run and whether it was rolled back.

## When it fails

- **"an inspection has been running for longer than 20 s"**: a look is stuck. Nothing
  was changed; try again in a minute.
- **A pair in `refused`**: one of the two is gone, they now share a picture, or a thing
  was named in two pairs. The rest were joined.
- **"no merge run is applied"**: there is nothing to roll back. Runs cannot be rolled
  back across a clear of the world state or a change of map.
- **The connection dropped part way through**: joining and rolling back are each one
  database transaction, so all of it happened or none of it did. Ask with
  `{"runs": true}` to see which.
